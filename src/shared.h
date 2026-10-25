#pragma once
#include <Arduino.h>
#include <TeensyThreads.h>

#include "config.h"

// State shared between threads. Each block has one writer; a mutex guards
// what is wider than one word, single words are volatile.

// Pi command, written by UARTRx (link.cpp) under CmdMutex.
extern Threads::Mutex CmdMutex;
extern float cmd_setpoint[2]; // {steer, throttle} normalized [-1,1]
extern bool dotmatrix_cmd[wire::dot_matrix_bytes];
extern uint8_t led_cmd[cfg::RGB_LED_WS2812B::num_leds * 3];
extern uint8_t system_state;          // see cfg::SYS_STATE
extern volatile uint32_t lastRxTime;  // micros() of the last CRC-valid command, 0 before any

// True when a Pi command arrived within timeout_ms.
bool piCommandFresh(uint32_t timeout_ms);

// RC receiver, written by RxReceive (sensors.cpp).
extern Threads::Mutex RxMutex;
extern uint16_t channelVals[cfg::FLYSKY_IA6B_RX::channel_count]; // raw us, 0 when absent

// INA226, written by ReadCurrAndVolt (sensors.cpp).
extern Threads::Mutex PowerMutex;
extern float power_amps;
extern float power_volts;

// LSM9DS1, written by ReadIMUStates (sensors.cpp).
extern Threads::Mutex ImuMutex;
extern float imuVals[9]; // accel xyz m/s^2, gyro xyz rad/s, mag xyz uT

// Both sensors share the bus.
extern Threads::Mutex I2CMutex;

// Sensor presence, written by the sensor threads once begin() succeeds.
extern volatile bool imu_ok;
extern volatile bool power_ok;

// Drive state, written by DriveControl (drive.cpp).
extern volatile bool manual_override;      // swD up: Pi locked out of drive
extern volatile bool rc_driving;           // sticks out of their neutral band
extern volatile float measured_speed_mps;  // signed ground speed from the hall encoder
