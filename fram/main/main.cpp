#include "fram.h"
#include "fram_store.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cstring>
#include <inttypes.h>  

static const char *TAG = "MAIN";

// ===== Pin map =====
#define FRAM_PIN_CS     GPIO_NUM_13
#define FRAM_PIN_SCLK   GPIO_NUM_14
#define FRAM_PIN_MOSI   GPIO_NUM_15
#define FRAM_PIN_MISO   GPIO_NUM_32

// ===== SPI / FRAM parameters =====
#define FRAM_SPI_HOST     VSPI_HOST
#define FRAM_SPI_FREQ_HZ  (1 * 1000 * 1000)

// example struct to store
struct MyConfig {
    uint32_t uptime_sec;
    uint32_t counter;
    uint8_t flags;
};
static_assert(std::is_trivially_copyable<MyConfig>::value, "POD required");

extern "C" void app_main(void)
{
    FRAM fram(FRAM_SPI_HOST, FRAM_PIN_CS, FRAM_PIN_SCLK, FRAM_PIN_MOSI, FRAM_PIN_MISO, FRAM_SPI_FREQ_HZ);
    ESP_ERROR_CHECK(fram.init());

    // choose base address in FRAM (must not overlap other data)
    constexpr FRAM::addr_t BASE_ADDR = 0x0200;
    // use 4 rotating slots -> simple wear-leveling
    fram_store::Persistent<MyConfig> store(fram, BASE_ADDR, /*slots=*/4, /*version=*/1);

    // mutex to protect store if multiple tasks use it
    SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

    // load existing config (if any)
    MyConfig cfg = {0, 0, 0}; // <--- fully initialize to avoid warnings
    if (store.load(cfg) == ESP_OK) {
        ESP_LOGI(TAG, "Loaded cfg: uptime=%" PRIu32 " cnt=%" PRIu32 " flags=%u",
                 cfg.uptime_sec, cfg.counter, cfg.flags); // <--- use PRIu32
    } else {
        ESP_LOGI(TAG, "No valid stored cfg, initializing");
        cfg = {0, 0, 0};
        // initial immediate save
        xSemaphoreTake(mutex, portMAX_DELAY);
        store.store_immediate(cfg);
        xSemaphoreGive(mutex);
    }

    // periodic task: update and persist once per minute
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60000)); // 60s

        // update in RAM
        cfg.uptime_sec += 60;
        cfg.counter++;

        // deferred store: update RAM cache, flush immediately for atomic commit
        xSemaphoreTake(mutex, portMAX_DELAY);
        store.store_deferred(cfg);
        // flush commits to FRAM (atomic: payload then header)
        esp_err_t err = store.flush();
        xSemaphoreGive(mutex);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Saved cfg seq OK (cnt=%" PRIu32 ")", cfg.counter); // <--- use PRIu32
        } else {
            ESP_LOGW(TAG, "Save failed: %d", err);
        }
    }
}