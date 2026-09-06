#pragma once
#include <Arduino.h>

#include "config.h"

// Wire format, authoritative for both ends; the Pi driver mirrors it in
// pit_protocol.py. Packed little-endian, with a trailing CRC-16/CCITT-FALSE
// over every preceding byte of the frame as sent.

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

// Teensy <-- Pi. UARTRx matches the magic word with a byte-sliding window, so
// it is not part of the struct.
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
  int32_t Timestamp;                                        // micros()
  uint16_t VoltAndCurr[2];                                  // bus mV, current mA
  uint16_t RxReceiver[cfg::FLYSKY_IA6B_RX::channel_count];  // raw pulse widths, us
  float IMU[9];                                             // accel xyz, gyro xyz, mag xyz
  float Encoder;                                            // ground speed, m/s
  float EKF[3];                                             // reserved: dx, dy, dtheta
  uint16_t Checksum;
} __attribute__((packed));

// Teensy <-- Pi
struct RxPacket {
  HeaderPacketNoMagic header;
  float ServoAng;      // normalized [-1,1], -1 = full left
  float MotorSpeed;    // normalized [-1,1], -1 = full reverse
  bool DotMatrix[192]; // 8x24 bitmap, row-major, 1 = on
  uint8_t LED[255];    // RGB triplets, one per LED
  uint8_t SystemState; // see cfg::SYS_STATE
  uint16_t Checksum;
}__attribute__((packed));

// A change here is a protocol break and must land in pit_protocol.py at the
// same time, under a matching minor version in both repos.
static_assert(sizeof(HeaderPacket) == 16, "TX header size changed");
static_assert(sizeof(HeaderPacketNoMagic) == 12, "RX header size changed");
static_assert(sizeof(TXPacket) == 94, "telemetry packet size changed");
static_assert(sizeof(RxPacket) == 470, "command packet size changed");

inline void initTxHeader(HeaderPacket &h) {
  h.MagicNum          = cfg::UART::tx_magic_num;
  h.VersionNum        = 1;
  h.IDNumber          = 0;
  h.PacketBytes       = sizeof(TXPacket);
  h.FrameBytes        = 0;
  h.PacketIndex       = 0;
  h.PacketsPerFrame   = 1;
}

inline void initRxHeader(HeaderPacketNoMagic &h) {
  h.VersionNum        = 1;
  h.IDNumber          = 0;
  h.PacketBytes       = sizeof(RxPacket);
  h.FrameBytes        = 0;
  h.PacketIndex       = 0;
  h.PacketsPerFrame   = 1;
}
