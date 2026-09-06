#pragma once
#include <Arduino.h>
#include <Servo.h>
#include <FastLED.h>
#include <INA226.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <MD_Parola.h>

#include "battery.h"
#include "config.h"
#include "crc16.h"
#include "drive_map.h"
#include "packets.h"

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW

// ----------------------- Error state ------------------------------ //
// 0 none, 1 INA226 absent, 2 LSM9DS1 absent.
extern uint8_t errorFlag;

// ------------------- UART Teensy <-> Pi --------------------------- //
extern TXPacket txPacketPing, txPacketPong;
extern RxPacket rxPacketPing, rxPacketPong;
extern volatile uint32_t lastRxTime; // micros() of the last CRC-valid Pi command

// --------------------------- RGB LED ------------------------------ //
extern CRGB leds[cfg::RGB_LED_WS2812B::num_leds];

// ------------------- Display command from Pi ---------------------- //
extern bool dotmatrix_cmd[192];             // 8x24 bitmap, row-major, 1 = on
extern uint8_t led_cmd[cfg::RGB_LED_WS2812B::num_leds * 3]; // RGB triplets
extern uint8_t system_state;                // see cfg::SYS_STATE

// -------------------- Servo and motor ----------------------------- //
extern float cmd_setpoint[2]; // {steer, throttle} normalized [-1,1] from the Pi
extern float cmd_pwm[2];      // {steer, throttle} resolved actuation, us

// RC-manual endpoints, derived from percent_throt/percent_rev at boot.
extern uint16_t max_pwm_throt;
extern uint16_t max_pwm_rev;

// Switch state resolved in testPWM, read by DotMatrix.
extern volatile bool manual_override; // swD up: Pi locked out of drive
extern volatile bool rc_driving;      // sticks out of their neutral band

// ----------------------- FlySky IA6B Rx --------------------------- //
extern uint16_t channelVals[cfg::FLYSKY_IA6B_RX::channel_count];

// ------------- INA226 voltage and current sensor ------------------ //
extern float power[3]; // current (A), bus voltage (V), power (W)

// -------------------- LSM9DS1 9-DOF IMU --------------------------- //
extern float imuVals[9]; // accel xyz, gyro xyz, mag xyz

// ------------------- Hall-effect encoder -------------------------- //
extern volatile uint32_t lastTransitionTime;
extern volatile uint32_t deltaTime;
extern volatile uint8_t currentState;
extern volatile uint8_t lastState;
extern volatile int8_t direction;
extern volatile float lastRPM;

// --------------------------- Threads ------------------------------ //
// Each runs forever under TeensyThreads; started from setup().
void UARTTx();
void fillTxBuff();
void UARTRx();
void ReadCurrAndVolt();
void ReadIMUStates();
void RxReceive();
void testPWM();
void DotMatrix();

// Called from loop(), not its own thread.
void ledStuffs();

// --------------------------- Helpers ------------------------------ //
void fillRxData();
void onHallChange(); // hall pin ISR
void getRPM();
float getCurrentSpeed(); // m/s at the wheels, signed
