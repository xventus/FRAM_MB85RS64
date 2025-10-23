/**
 * @file fram_store.h
 * @author Petr Vanek (petr@fotoventus.cz)
 * @brief FRAM storage wrapper.
 * @date 2025-10-23
 * 
 * @copyright Copyright (c) 2025 Petr Vanek
 *  All functions return esp_err_t values (ESP_OK on success).
 */


#pragma once
#include <cstdint>
#include <cstring>
#include <vector>
#include <type_traits>
#include "fram.h"
#include "esp_err.h"

namespace fram_store {

#pragma pack(push,1)
struct Header {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t seq;
    uint32_t len;
    uint32_t crc;
};
#pragma pack(pop)

static constexpr uint32_t STORE_MAGIC = 0x4652414D; // 'FRAM'

static uint32_t crc32(const void* data, size_t len)
{
    static uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int j = 0; j < 8; ++j)
                c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            table[i] = c;
        }
        init = true;
    }
    uint32_t c = 0xFFFFFFFFu;
    const uint8_t* p = reinterpret_cast<const uint8_t*>(data);
    for (size_t i = 0; i < len; ++i)
        c = table[(c ^ p[i]) & 0xFFu] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

/*
  Persistent<T>
  - supports N circular slots starting at base_addr
  - slot layout: [Header][payload]
  - atomic commit: write payload then header
  - methods: load(), store_immediate(), store_deferred(), flush()
*/
template<typename T>
class Persistent {
    static_assert(std::is_trivially_copyable<T>::value, "T must be trivially_copyable");
public:
    Persistent(FRAM &fram,
               FRAM::addr_t base_addr,
               size_t slots = 2,
               uint16_t version = 1)
        : fram_(fram), base_(base_addr), slots_(slots), version_(version),
          slot_size_(sizeof(Header) + sizeof(T)), dirty_(false)
    {}

    // load latest valid copy into dst
    esp_err_t load(T &dst) {
        Header best_hdr{0};
        FRAM::addr_t best_addr = 0;
        bool found = false;

        for (size_t i = 0; i < slots_; ++i) {
            FRAM::addr_t a = base_ + static_cast<FRAM::addr_t>(i * slot_size_);
            Header h;
            if (fram_.read(a, &h, sizeof(h)) != ESP_OK) continue;
            if (h.magic != STORE_MAGIC || h.version != version_ || h.len != sizeof(T)) continue;
            std::vector<uint8_t> buf(h.len);
            if (fram_.read(a + sizeof(Header), buf.data(), buf.size()) != ESP_OK) continue;
            if (crc32(buf.data(), buf.size()) != h.crc) continue;
            if (!found || h.seq > best_hdr.seq) {
                best_hdr = h;
                best_addr = a;
                found = true;
            }
        }

        if (!found) return ESP_ERR_NOT_FOUND;
        // read payload
        return fram_.read(best_addr + sizeof(Header), &dst, sizeof(T));
    }

    // immediate store: writes to next slot (rotates), returns when committed
    esp_err_t store_immediate(const T &src) {
        Header cur_best{0};
        FRAM::addr_t best_addr = base_;
        bool found = false;

        for (size_t i = 0; i < slots_; ++i) {
            FRAM::addr_t a = base_ + static_cast<FRAM::addr_t>(i * slot_size_);
            Header h;
            (void)fram_.read(a, &h, sizeof(h));
            if (h.magic == STORE_MAGIC && h.version == version_) {
                if (!found || h.seq > cur_best.seq) {
                    cur_best = h;
                    best_addr = a;
                    found = true;
                }
            }
        }
        uint32_t next_seq = found ? (cur_best.seq + 1) : 1;
        // pick next slot (circular) after best_addr
        FRAM::addr_t next = found
            ? static_cast<FRAM::addr_t>( ((best_addr - base_) / slot_size_ + 1) % slots_ ) * slot_size_ + base_
            : base_;

        Header h;
        h.magic = STORE_MAGIC;
        h.version = version_;
        h.reserved = 0;
        h.seq = next_seq;
        h.len = static_cast<uint32_t>(sizeof(T));
        h.crc = crc32(&src, sizeof(T));

        // write payload then header (atomicity)
        esp_err_t err = fram_.write(next + sizeof(Header), &src, sizeof(T));
        if (err != ESP_OK) return err;
        err = fram_.write(next, &h, sizeof(h));
        if (err != ESP_OK) return err;

        // update cache
        cache_ = src;
        dirty_ = false;
        last_seq_ = next_seq;
        return ESP_OK;
    }

    // deferred store: update RAM cache only, call flush() to commit
    void store_deferred(const T &src) {
        cache_ = src;
        dirty_ = true;
    }

    // flush deferred cache to FRAM (commits immediately)
    esp_err_t flush() {
        if (!dirty_) return ESP_OK;
        return store_immediate(cache_);
    }

    bool dirty() const { return dirty_; }

private:
    FRAM &fram_;
    FRAM::addr_t base_;
    size_t slots_;
    uint16_t version_;
    size_t slot_size_;
    T cache_;
    bool dirty_;
    uint32_t last_seq_{0};
};

} // namespace fram_store