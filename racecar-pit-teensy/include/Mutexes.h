#pragma once
#include <Arduino.h>
#include <TeensyThreads.h>

extern Threads::Mutex CmdMutex;                // for use with cmd_packet}{}
extern Threads::Mutex RxMutex;                 // for use with channelVals[]
extern Threads::Mutex I2CMutex;                // for use with I2C bus
extern Threads::Mutex SPIMutex;                // for use with SPI bus
extern Threads::Mutex PowerMutex;              // for use with power[]
extern Threads::Mutex ErrMutex;                // for use with errorFlag
extern Threads::Mutex ImuMutex;                // for use with imuVals[]
extern Threads::Mutex HallMutex;               // for use with currentState
extern Threads::Mutex TransmitMutex;           // for use with txPacketPing
extern Threads::Mutex RecieveMutex;            // for use with rxPacketPing
extern Threads::Mutex TransPongMutex;          // for use with txPacketPong
extern Threads::Mutex RecPongMutex;            // for use with rxPacketPong
extern Threads::Mutex SerMutex;                // for use with Serial1