# FRAM storage library for MB85RS64 for ESP32 - IDF

Small FRAM SPI driver and a tiny persistent store for storing POD structs with integrity and simple wear‑leveling.

## FRAM
- Non‑volatile, byte‑addressable, fast, very high write endurance.
- Use for frequent writes: counters, config, small logs.

## Usage
- SPI controlled (CS, SCLK, MOSI, MISO). See main/main.cpp.
- API: FRAM::init(), FRAM::read(), FRAM::write(), FRAM::rdid().

## fram_store
- Persist POD types with header {magic, version, seq, crc}.
- Supports N rotating slots (wear‑leveling), atomic commit (payload then header), deferred or immediate writes.
- API: store.load(), store.store_deferred(), store.flush(), store.store_immediate().

## Notes
- Stored type must be trivially copyable.
- Ensure slots do not overlap: slot_size = sizeof(Header) + sizeof(T).
- For 1 write/minute, 2–4 slots are sufficient; FRAM endurance is high.

## Files
- main/fram.h + .cpp — FRAM driver
- main/fram_store.h — fram_store::Persistent
- main/main.cpp — example
