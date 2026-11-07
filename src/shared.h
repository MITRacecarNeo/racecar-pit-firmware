#pragma once
#include <Arduino.h>
#include <TeensyThreads.h>

#include "config.h"
#include "info.h"
#include "seqlock.h"

// State shared between threads and the drive interrupt. Each block has one
// writer. What the drive interrupt reads goes through a Seqlock (an interrupt
// cannot wait on a mutex); other blocks wider than one word take a mutex;
// single words are volatile.

// Pi drive command, written by UARTRx (link.cpp).
struct DriveCommand
{
  float steer;    // normalized [-1,1], -1 = full left
  float throttle; // normalized [-1,1], -1 = full reverse
};
extern Seqlock<DriveCommand> drive_cmd;

// Pi display command, written by UARTRx under CmdMutex.
extern Threads::Mutex CmdMutex;
extern bool dotmatrix_cmd[wire::dot_matrix_bytes];
extern uint8_t led_cmd[cfg::RGB_LED_WS2812B::num_leds * 3];
extern uint8_t system_state;          // see cfg::SYS_STATE
extern volatile uint32_t lastRxTime;  // micros() of the last CRC-valid command, 0 before any

// True when a Pi command arrived within timeout_ms.
bool piCommandFresh(uint32_t timeout_ms);

// RC receiver, written by RxReceive (sensors.cpp).
struct RcChannels
{
  uint16_t us[cfg::FLYSKY_IA6B_RX::channel_count]; // raw pulse widths, 0 when absent
};
extern Seqlock<RcChannels> rc_channels;

// Thread-side read: retries, yielding, while a write is in progress.
template <typename T>
T readShared(const Seqlock<T> &lock)
{
  T value;
  while (!lock.tryRead(value))
    threads.yield();
  return value;
}

// INA226, written by ReadCurrAndVolt (sensors.cpp).
struct PowerSample
{
  float amps;
  float volts;
};
extern Seqlock<PowerSample> power_sample;
extern Seqlock<DipSummary> dip_summary; // also ReadCurrAndVolt

// Stall guard output, written by the drive tick (drive.cpp): packStallWord
// (src/stall_guard.h), and millis() at the last trip.
extern volatile uint32_t stall_word;
extern volatile uint32_t stall_trip_ms;

// LSM9DS1, written by ReadIMUStates (sensors.cpp).
struct ImuSample
{
  float v[9]; // accel xyz m/s^2, gyro xyz rad/s, mag xyz uT
};
extern Seqlock<ImuSample> imu_sample;

// Both sensors share the bus.
extern Threads::Mutex I2CMutex;

// LSM9DS1 registers, written once by ReadIMUStates before imu_ok is set.
extern Lsm9ds1Registers imu_regs;

// Sensor presence, written by the sensor threads once begin() succeeds.
extern volatile bool imu_ok;
extern volatile bool power_ok;

// Drive state, written by the drive interrupt (drive.cpp).
extern volatile bool manual_override;      // swD high (lever down): Pi locked out of drive
extern volatile bool rc_driving;           // sticks out of their neutral band
extern volatile float measured_speed_mps;  // signed ground speed from the hall encoder
