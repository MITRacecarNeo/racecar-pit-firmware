#pragma once
#include <Arduino.h>
#include <TeensyThreads.h>

extern Threads::Mutex CmdMutex;      // cmd_setpoint[], dotmatrix_cmd[], led_cmd[], system_state
extern Threads::Mutex RxMutex;       // channelVals[]
extern Threads::Mutex I2CMutex;      // I2C bus
extern Threads::Mutex SPIMutex;      // SPI bus
extern Threads::Mutex PowerMutex;    // power[]
extern Threads::Mutex ErrMutex;      // errorFlag
extern Threads::Mutex ImuMutex;      // imuVals[]
extern Threads::Mutex TransmitMutex; // txPacketPing
extern Threads::Mutex ReceiveMutex;  // rxPacketPing
extern Threads::Mutex RecPongMutex;  // rxPacketPong
