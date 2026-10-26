#pragma once
#include <stddef.h>
#include <stdint.h>

// Firmware info reply payload: ASCII, one "key=value\n" line per setting.
// Arduino-free and host tested; the keys are listed in docs/protocol.md.

namespace info
{
    constexpr uint8_t schema_version = 1;
    constexpr size_t payload_max_bytes = 2048;
}

// Sensor state when the payload is built.
struct InfoStatus
{
    bool imu_ok;
    bool power_ok;
    const char *last_reset; // "watchdog", "software", "reset", "unknown"
    uint32_t reset_flags;   // SRC_SRSR at boot
    bool watchdog_armed;
};

// Appends "key=value\n" lines to a fixed buffer. Values are formatted without
// printf, so host and Teensy output match byte for byte.
class InfoWriter
{
public:
    InfoWriter(char *buf, size_t cap) : buf_(buf), cap_(cap) {}

    void text(const char *key, const char *value);
    void integer(const char *key, int32_t value);
    // "major.minor.patch"
    void version(const char *key, uint8_t major, uint8_t minor, uint8_t patch);
    // Rounded to `decimals` places (at most 6), trailing zeros kept.
    void fixed(const char *key, float value, uint8_t decimals);

    // Payload length, or 0 once anything failed to fit.
    size_t length() const { return ok_ ? len_ : 0; }

private:
    void put(const char *s);
    void putChar(char c);
    void putUint(uint64_t v, uint8_t min_digits = 1);

    char *buf_;
    size_t cap_;
    size_t len_ = 0;
    bool ok_ = true;
};

// Writes the payload for the compiled config and `status` into buf; returns
// its length, or 0 when it does not fit in cap.
size_t writeFirmwareInfo(const InfoStatus &status, char *buf, size_t cap);
