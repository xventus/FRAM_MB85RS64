/**
 * @file fram.h
 * @author Petr Vanek (petr@fotoventus.cz)
 * @brief FRAM SPI driver C++ wrapper for MB85RSxx devices.
 * @date 2025-10-23
 * 
 * @copyright Copyright (c) 2025 Petr Vanek
 *  All functions return esp_err_t values (ESP_OK on success).
 */

#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
#include <span>
#include <string_view>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

class FRAM {
public:
    /// 16-bit device address type
    using addr_t = uint16_t;

    /// Total device size in bytes (used for bounds checking)
    static constexpr size_t FRAM_SIZE_BYTES = 8 * 1024;

    /**
     * @brief Construct a FRAM driver instance.
     * @param host SPI host (e.g. HSPI_HOST / VSPI_HOST)
     * @param cs   GPIO pin used for chip-select
     * @param sclk GPIO pin used for SCLK
     * @param mosi GPIO pin used for MOSI
     * @param miso GPIO pin used for MISO
     * @param freq_hz SPI clock frequency in Hz (default 1MHz)
     *
     * @note Constructor only stores configuration. Call init() to initialize
     *       the SPI bus and attach the device.
     */
    FRAM(spi_host_device_t host,
         gpio_num_t cs,
         gpio_num_t sclk,
         gpio_num_t mosi,
         gpio_num_t miso,
         int freq_hz = 1 * 1000 * 1000);

    /**
     * @brief Destructor.
     * @details Detaches the SPI device and attempts to free the SPI bus.
     *          Destructor ignores errors from bus free operations.
     */
    ~FRAM();

    /**
     * @brief Initialize SPI bus and attach FRAM device.
     * @return ESP_OK on success, otherwise an esp_err_t error code.
     * @note This must be called before any read/write/rdid calls.
     */
    esp_err_t init();

    /* ---------------------------------------------------------------------
     * Low-level C-style operations
     * ------------------------------------------------------------------*/

    /**
     * @brief Read the JEDEC/device ID (RDID command).
     * @param[out] out Pointer to buffer receiving ID bytes.
     * @param[in]  n   Number of ID bytes to read.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG if args are invalid,
     *         or other esp_err_t on SPI/driver error.
     *
     * @note The buffer must be valid and at least n bytes long.
     */
    esp_err_t rdid(uint8_t *out, size_t n);

    /**
     * @brief Read a block of data from FRAM.
     * @param[in]  addr Address to start reading from.
     * @param[out] buf  Destination buffer to receive data.
     * @param[in]  len  Number of bytes to read.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for bad args or out-of-range,
     *         or other esp_err_t on SPI/driver error.
     *
     * @note This call blocks until the SPI transfer completes.
     */
    esp_err_t read(addr_t addr, void *buf, size_t len);

    /**
     * @brief Write a block of data to FRAM.
     * @param[in] addr Address to start writing to.
     * @param[in] buf  Source buffer holding data to write.
     * @param[in] len  Number of bytes to write.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for bad args or out-of-range,
     *         or other esp_err_t on SPI/driver error.
     *
     * @note The implementation issues a WREN before writing and clears it after.
     *       The buffer is not modified by this call.
     */
    esp_err_t write(addr_t addr, const void *buf, size_t len);

    /* ---------------------------------------------------------------------
     * C++-friendly wrapper overloads (convenience)
     * ------------------------------------------------------------------*/

    /**
     * @brief Read JEDEC/device ID into a std::span.
     * @param[out] out Span to receive ID bytes.
     * @return esp_err_t same semantics as rdid(uint8_t*, size_t)
     */
    inline esp_err_t rdid(std::span<uint8_t> out) {
        return rdid(out.data(), out.size());
    }

    /**
     * @brief Read bytes into a span of std::byte.
     * @param[in] addr Address to read from.
     * @param[out] out Destination span.
     * @return esp_err_t same semantics as read(addr, void*, size_t)
     */
    inline esp_err_t read(addr_t addr, std::span<std::byte> out) {
        return read(addr, out.data(), out.size());
    }

    /**
     * @brief Read bytes into a span of uint8_t.
     * @param[in] addr Address to read from.
     * @param[out] out Destination span.
     * @return esp_err_t same semantics as read(addr, void*, size_t)
     */
    inline esp_err_t read(addr_t addr, std::span<uint8_t> out) {
        return read(addr, out.data(), out.size());
    }

    /**
     * @brief Write bytes from a span of std::byte.
     * @param[in] addr Address to write to.
     * @param[in] data Source span.
     * @return esp_err_t same semantics as write(addr, const void*, size_t)
     */
    inline esp_err_t write(addr_t addr, std::span<const std::byte> data) {
        return write(addr, data.data(), data.size());
    }

    /**
     * @brief Write bytes from a span of uint8_t.
     * @param[in] addr Address to write to.
     * @param[in] data Source span.
     * @return esp_err_t same semantics as write(addr, const void*, size_t)
     */
    inline esp_err_t write(addr_t addr, std::span<const uint8_t> data) {
        return write(addr, data.data(), data.size());
    }

    /**
     * @brief Write data from a std::string_view (useful for text).
     * @param[in] addr Address to write to.
     * @param[in] s    String view containing bytes to write (not NUL-terminated requirement).
     * @return esp_err_t same semantics as write(addr, const void*, size_t)
     */
    inline esp_err_t write(addr_t addr, std::string_view s) {
        return write(addr, reinterpret_cast<const void*>(s.data()), s.size());
    }

    // non-copyable
    FRAM(const FRAM&) = delete;
    FRAM& operator=(const FRAM&) = delete;

private:
    /**
     * @brief Send an 8-bit command (single byte) over SPI.
     * @param[in] cmd Command opcode to send.
     * @return ESP_OK on success, otherwise an esp_err_t.
     *
     * @note Low-level helper, used to implement WREN/WRDI and similar commands.
     */
    esp_err_t cmd8(uint8_t cmd);

    /**
     * @brief Enable or disable write operations on the FRAM (WREN / WRDI).
     * @param[in] en true to enable (WREN), false to disable (WRDI).
     * @return ESP_OK on success, otherwise an esp_err_t.
     */
    esp_err_t wren(bool en);

    spi_host_device_t host_;
    gpio_num_t cs_, sclk_, mosi_, miso_;
    int freq_hz_;
    spi_device_handle_t dev_{nullptr};
};