#pragma once
#include <stddef.h>
#include <stdint.h>

#include "dip_record.h"

// Firmware info reply payload: ASCII, one "key=value\n" line per setting.
// Arduino-free and host tested; the keys are listed in docs/protocol.md.

namespace info
{
    constexpr uint8_t schema_version = 1;
    constexpr size_t payload_max_bytes = 3072;
}

// LSM9DS1 configuration registers, read back once the IMU starts.
struct Lsm9ds1Registers
{
    bool read;        // false until the IMU answered
    uint8_t ctrl1_g;  // 0x10 on the accel/gyro: ODR_G, FS_G, BW_G
    uint8_t ctrl6_xl; // 0x20 on the accel/gyro: ODR_XL, FS_XL
    uint8_t ctrl1_m;  // 0x20 on the magnetometer: OM, DO, FAST_ODR
    uint8_t ctrl2_m;  // 0x21 on the magnetometer: FS
    uint8_t ctrl3_m;  // 0x22 on the magnetometer: MD
};

// Rates and range decoded from the registers. With the gyro on, the
// accelerometer runs at the gyro rate whatever ODR_XL says.
uint16_t lsm9ds1GyroOdrHz(const Lsm9ds1Registers &r);
uint16_t lsm9ds1AccelOdrHz(const Lsm9ds1Registers &r);
float lsm9ds1MagOdrHz(const Lsm9ds1Registers &r);
uint8_t lsm9ds1MagRangeGauss(const Lsm9ds1Registers &r);

// Sensor state when the payload is built.
struct InfoStatus
{
    bool imu_ok;
    bool power_ok;
    const char *last_reset; // "watchdog", "software", "reset", "unknown"
    uint32_t reset_flags;   // SRC_SRSR at boot
    bool watchdog_armed;
    Lsm9ds1Registers imu_regs;
    DipSummary dips;
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
