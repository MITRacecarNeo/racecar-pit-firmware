#pragma once
#include <Arduino.h>

#include <Arduino.h>
#include <Servo.h>
#include <FastLED.h>
#include <INA226.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <MD_Parola.h>
#include "packets.h"
#include "config.h"
// ----------------------- Variables -------------------------------- //
extern uint8_t errorFlag, errType;

// ------------- UART-TEENSY-PI --------------- //

extern TXPacket txPacketPing, txPacketPong;
extern RxPacket rxPacketPing, rxPacketPong;
extern uint32_t magicNumRx;
extern uint8_t window[4];
extern volatile uint32_t lastRxTime;   // micros() of the last CRC-valid Pi command

// --------------------------- RGB LED ---------------------------- //
extern CRGB leds[cfg::RGB_LED_WS2812B::num_leds];
extern uint8_t colorIdx;
extern uint8_t numleds;

// -------------- Servo and Motor ------------ //
extern float cmd_setpoint[2];
extern float cmd_pwm[2];

// ------------- ESC_10BL60_G2 --------------- //
extern uint16_t  max_pwm_throt;
extern uint16_t  max_pwm_rev;

// -------------- SERVO_D135F ---------------- //    
extern uint16_t  max_pwm_left;
extern uint16_t  max_pwm_right;

// ----------------------------- FlySky IA6B Rx --------------------- //

extern uint16_t channelVals[cfg::FLYSKY_IA6B_RX::channel_count];
extern uint8_t   drive_mode_sw;

// --------------------------- INA226 Voltage and Current Sensor ------------------------ //
extern float power[3];  // Current (mA), Voltage (V), Power (mW)

// ---------------------------- LSM9DS1 9-DOF IMU ----------------------------------------//
extern sensors_event_t a, m, g, temp;
extern float imuVals[9];

//------------------------------ MAX 7219 Dot Matrix -------------------------------------//
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW

// --------------------------------------- Hall - Effect Encoder ---------------------------//
extern volatile uint32_t lastTransitionTime;
extern volatile uint32_t deltaTime;
extern volatile uint8_t currentState;
extern volatile uint8_t lastState;
extern volatile int8_t direction;
extern volatile float lastRPM;
extern volatile int8_t lastDir;

// -------------------- functions ------------------------ //
float getCurrentSpeed();
void InnerControlLoop();

// ------ UART -------- //
uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);

void UARTTx();
void fillTxBuff();

void UARTRx();
void fillRxData();

// ----------- Hall Effect -------- //
int8_t getDirection(uint8_t, uint8_t);
void onHallChange();
void getRPM();

// ---------- Current and Voltage Sense --------- //
void ReadCurrAndVolt();

// ------------ IMU States ------------- //
void ReadIMUStates();

// ------------- FlySky Receiver ---------- //
void RxReceive();

// ---------------- ESC and Servo ---------- //
void testPWM();

// --------------- Dot Matrix ------------- //
void DotMatrix();

// ----------------- RGB LED -------------- //
void ledStuffs();