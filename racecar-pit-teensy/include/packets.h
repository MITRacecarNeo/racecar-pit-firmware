#pragma once
#include <Arduino.h>

#include "config.h"

// Teensy --> Pi
struct HeaderPacket {
  uint32_t MagicNum;
  uint8_t VersionNum;
  uint8_t IDNumber;
  uint16_t PacketBytes;
  uint32_t FrameBytes;
  uint16_t PacketIndex;
  uint16_t PacketsPerFrame;
}__attribute__((packed));

// Teensy <-- Pi
struct HeaderPacketNoMagic {
  uint8_t VersionNum;
  uint8_t IDNumber;
  uint16_t PacketBytes;
  uint32_t FrameBytes;
  uint16_t PacketIndex;
  uint16_t PacketsPerFrame;
}__attribute__((packed));

// Teensy --> Pi
struct TXPacket {
  HeaderPacket header;
  int32_t Timestamp;
  uint16_t VoltAndCurr[2];
  uint16_t RxReceiver[cfg::FLYSKY_IA6B_RX::channel_count];
  float IMU[9];
  float Encoder;
  float EKF[3];
  uint16_t Checksum;
} __attribute__((packed));

// Teensy <-- Pi
struct RxPacket {
  HeaderPacketNoMagic header;
  float ServoAng;
  float MotorSpeed;
  bool DotMatrix[192];
  uint8_t LED [255];
  uint8_t SystemState;
  uint16_t Checksum;
}__attribute__((packed));

inline void initTxHeader(HeaderPacket &h) {
  h.MagicNum          = cfg::UART::tx_magic_num ;
  h.VersionNum        = 1;
  h.IDNumber          = 0;
  h.PacketBytes       = sizeof(TXPacket);
  h.PacketIndex       = 0;
  h.PacketsPerFrame   = 1;
}

inline void initRxHeader(HeaderPacketNoMagic &h) {
  h.VersionNum        = 1;
  h.IDNumber          = 0;
  h.IDNumber          = 0;
  h.PacketBytes       = sizeof(RxPacket);
  h.PacketIndex       = 0;
  h.PacketsPerFrame   = 1;
}