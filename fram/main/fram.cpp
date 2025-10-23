/**
 * @file fram.cpp
 * @author Petr Vanek (petr@fotoventus.cz)
 * @date 2025-10-23
 * 
 * @copyright Copyright (c) 2025 Petr Vanek
 *  
 */


#include "fram.h"
#include <vector>
#include <cstring>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/spi_master.h"

static const char *TAG = "FRAM_C++";

// MB85RS64 opcodes
static constexpr uint8_t FRAM_CMD_WREN = 0x06;
static constexpr uint8_t FRAM_CMD_WRDI = 0x04;
static constexpr uint8_t FRAM_CMD_RDSR = 0x05;
static constexpr uint8_t FRAM_CMD_WRSR = 0x01;
static constexpr uint8_t FRAM_CMD_READ = 0x03;
static constexpr uint8_t FRAM_CMD_WRITE = 0x02;
static constexpr uint8_t FRAM_CMD_RDID = 0x9F;

FRAM::FRAM(spi_host_device_t host, gpio_num_t cs, gpio_num_t sclk, gpio_num_t mosi, gpio_num_t miso, int freq_hz)
    : host_(host), cs_(cs), sclk_(sclk), mosi_(mosi), miso_(miso), freq_hz_(freq_hz)
{}

FRAM::~FRAM()
{
    if (dev_) {
        spi_bus_remove_device(dev_);
        dev_ = nullptr;
    }
    // try to free bus (ignore errors in dtor)
    spi_bus_free(host_);
}

esp_err_t FRAM::init()
{
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num     = mosi_;
    buscfg.miso_io_num     = miso_;
    buscfg.sclk_io_num     = sclk_;
    buscfg.quadwp_io_num   = -1;
    buscfg.quadhd_io_num   = -1;
    buscfg.max_transfer_sz = 4096;
    buscfg.flags           = SPICOMMON_BUSFLAG_MASTER;
    ESP_RETURN_ON_ERROR(spi_bus_initialize(host_, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi_bus_initialize");

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = freq_hz_;
    devcfg.mode           = 0;
    devcfg.spics_io_num   = cs_;
    devcfg.queue_size     = 3;
    devcfg.flags          = 0;
    ESP_RETURN_ON_ERROR(spi_bus_add_device(host_, &devcfg, &dev_), TAG, "spi_bus_add_device");

    // sanity: read RDID
    uint8_t id[4] = {0};
    if (rdid(id, sizeof id) == ESP_OK) {
        ESP_LOGI(TAG, "RDID: %02X %02X %02X %02X", id[0], id[1], id[2], id[3]);
    } else {
        ESP_LOGW(TAG, "RDID failed");
    }

    // status reg read (sanity)
    uint8_t tx[2] = { FRAM_CMD_RDSR, 0x00 }, rx[2] = {0};
    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    ESP_ERROR_CHECK(spi_device_transmit(dev_, &t));
    ESP_LOGI(TAG, "SR=0x%02X", rx[1]);

    return ESP_OK;
}

esp_err_t FRAM::cmd8(uint8_t cmd)
{
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &cmd;
    return spi_device_transmit(dev_, &t);
}

esp_err_t FRAM::wren(bool en)
{
    return cmd8(en ? FRAM_CMD_WREN : FRAM_CMD_WRDI);
}

esp_err_t FRAM::rdid(uint8_t *out, size_t n)
{
    ESP_RETURN_ON_FALSE(out && n > 0, ESP_ERR_INVALID_ARG, TAG, "bad args");
    size_t txlen = 1 + n;
    std::vector<uint8_t> tx(txlen, 0), rx(txlen, 0);
    tx[0] = FRAM_CMD_RDID;

    spi_transaction_t t = {};
    t.length = 8 * txlen;
    t.tx_buffer = tx.data();
    t.rx_buffer = rx.data();

    esp_err_t err = spi_device_transmit(dev_, &t);
    if (err == ESP_OK) {
        memcpy(out, rx.data() + 1, n);
    }
    return err;
}

esp_err_t FRAM::read(addr_t addr, void *buf, size_t len)
{
    ESP_RETURN_ON_FALSE(buf && len, ESP_ERR_INVALID_ARG, TAG, "bad args");
    if ((uint32_t)addr + len > FRAM_SIZE_BYTES) return ESP_ERR_INVALID_ARG;

    size_t txlen = 3 + len;
    std::vector<uint8_t> tx(txlen, 0), rx(txlen, 0);
    tx[0] = FRAM_CMD_READ;
    tx[1] = static_cast<uint8_t>(addr >> 8);
    tx[2] = static_cast<uint8_t>(addr & 0xFF);

    spi_transaction_t t = {};
    t.length = 8 * txlen;
    t.tx_buffer = tx.data();
    t.rx_buffer = rx.data();

    esp_err_t err = spi_device_transmit(dev_, &t);
    if (err == ESP_OK) memcpy(buf, rx.data() + 3, len);
    return err;
}

esp_err_t FRAM::write(addr_t addr, const void *buf, size_t len)
{
    ESP_RETURN_ON_FALSE(buf && len, ESP_ERR_INVALID_ARG, TAG, "bad args");
    if ((uint32_t)addr + len > FRAM_SIZE_BYTES) return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_ERROR(wren(true), TAG, "WREN");

    size_t txlen = 3 + len;
    std::vector<uint8_t> tx(txlen);
    tx[0] = FRAM_CMD_WRITE;
    tx[1] = static_cast<uint8_t>(addr >> 8);
    tx[2] = static_cast<uint8_t>(addr & 0xFF);
    memcpy(tx.data() + 3, buf, len);

    spi_transaction_t t = {};
    t.length = 8 * txlen;
    t.tx_buffer = tx.data();

    esp_err_t err = spi_device_transmit(dev_, &t);
    if (err == ESP_OK) err = wren(false);
    return err;
}