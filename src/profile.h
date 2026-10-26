#pragma once
#include <stdint.h>

// Profiling hooks, active only in the teensy41_profile build (-DRCN_PROFILE).
// Without it every macro compiles to nothing. Report lines go to USB serial
// once per second; format in docs/architecture.md, capture with
// scripts/profile_capture.py.

namespace prof
{
    enum Period : uint8_t
    {
        drive_loop, // DriveControl loop starts
        tx_slot,    // telemetry frames sent
        rx_loop,    // UARTRx loop starts
        imu_loop,   // ReadIMUStates loop starts
        power_loop, // ReadCurrAndVolt loop starts
        rc_loop,    // RxReceive loop starts
        dot_loop,   // DotMatrix loop starts
        led_loop,   // loop() calls of updateLedStrip
        period_count
    };

    enum Section : uint8_t
    {
        drive_body,      // DriveControl from channel copy to PWM write
        imu_read,        // LSM9DS1 getEvent, bus held
        imu_bus_wait,    // ReadIMUStates waiting for I2CMutex
        power_read,      // INA226 current and voltage, bus held
        power_bus_wait,  // ReadCurrAndVolt waiting for I2CMutex
        led_show,        // FastLED.show
        tx_send,         // telemetry build and Serial1.write
        rx_frame,        // command frame read and CRC
        cmd_to_pwm,      // command frame accepted to the PWM write that used it
        irq_late,        // lateness of a 10 kHz top-priority timer interrupt
        section_count
    };

    enum Counter : uint8_t
    {
        tx_frames,
        info_chunks,
        rx_frames,
        rx_crc_fail,
        rx_short,
        imu_distinct, // LSM9DS1 samples that differ from the previous one
        power_reads,
        counter_count
    };

    // Starts the lateness timer; call once from setup().
    void begin();
    // Emits the report when a second has passed; call from loop().
    void report();

    void period(Period id);
    uint32_t now();                       // cycle counter
    void section(Section id, uint32_t start); // records now() - start
    void count(Counter id);
    void commandAccepted();               // UARTRx, after a command is applied
    void commandUsed();                   // DriveControl, after the PWM write of a Pi command
}

#ifdef RCN_PROFILE
#define PROF_BEGIN() prof::begin()
#define PROF_REPORT() prof::report()
#define PROF_PERIOD(id) prof::period(prof::id)
#define PROF_START(var) const uint32_t var = prof::now()
#define PROF_SECTION(id, start) prof::section(prof::id, start)
#define PROF_COUNT(id) prof::count(prof::id)
#define PROF_CMD_ACCEPTED() prof::commandAccepted()
#define PROF_CMD_USED() prof::commandUsed()
#else
#define PROF_BEGIN() ((void)0)
#define PROF_REPORT() ((void)0)
#define PROF_PERIOD(id) ((void)0)
#define PROF_START(var) ((void)0)
#define PROF_SECTION(id, start) ((void)0)
#define PROF_COUNT(id) ((void)0)
#define PROF_CMD_ACCEPTED() ((void)0)
#define PROF_CMD_USED() ((void)0)
#endif
