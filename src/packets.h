#pragma once
#include <stddef.h>
#include <stdint.h>

// Wire format, authoritative for both ends; the Pi driver mirrors it in
// pit_protocol.py. Packed little-endian, with a trailing CRC-16/CCITT-FALSE
// over every preceding byte of the frame as sent. Arduino-free, so the frame
// builders are host tested. Description: docs/protocol.md.

namespace wire
{
    constexpr uint8_t protocol_version = 1;

    constexpr uint32_t telemetry_magic = 0xDEADBEEF; // Teensy -> Pi, every telemetry slot
    constexpr uint32_t command_magic = 0xBEEFDEAD;   // Pi -> Teensy
    constexpr uint32_t info_magic = 0xFEEDC0DE;      // Teensy -> Pi, only on request

    constexpr uint8_t rc_channels = 8;
    constexpr uint16_t dot_matrix_bytes = 192; // 8x24 bitmap, row-major
    constexpr uint16_t led_bytes = 255;        // RGB triplets

    // Info chunks carry at most this many payload bytes, one chunk per
    // telemetry slot, so telemetry keeps its rate.
    constexpr uint16_t info_chunk_bytes = 128;
}

struct HeaderPacket {
  uint32_t MagicNum;
  uint8_t VersionNum;       // wire::protocol_version
  uint8_t IDNumber;         // telemetry 0; info: payload revision
  uint16_t PacketBytes;     // bytes in this packet, magic and checksum included
  uint32_t FrameBytes;      // telemetry 0; info: payload bytes across all chunks
  uint16_t PacketIndex;     // telemetry 0; info: chunk index
  uint16_t PacketsPerFrame; // telemetry 1; info: chunk count
} __attribute__((packed));

// Teensy <-- Pi. UARTRx matches the magic word with a byte-sliding window, so
// it is not part of the struct.
struct HeaderPacketNoMagic {
  uint8_t VersionNum;
  uint8_t IDNumber;
  uint16_t PacketBytes;
  uint32_t FrameBytes;
  uint16_t PacketIndex;
  uint16_t PacketsPerFrame;
} __attribute__((packed));

// Teensy --> Pi
struct TXPacket {
  HeaderPacket header;
  int32_t Timestamp;                      // micros()
  uint16_t VoltAndCurr[2];                // bus mV, current mA
  uint16_t RxReceiver[wire::rc_channels]; // raw pulse widths, us
  float IMU[9];                           // accel xyz m/s^2, gyro xyz rad/s, mag xyz uT
  float Encoder;                          // ground speed, m/s
  float EKF[3];                           // reserved, always 0
  uint16_t Checksum;
} __attribute__((packed));

// Teensy <-- Pi
struct RxPacket {
  HeaderPacketNoMagic header;
  float ServoAng;   // normalized [-1,1], -1 = full left
  float MotorSpeed; // normalized [-1,1], -1 = full reverse
  bool DotMatrix[wire::dot_matrix_bytes];
  uint8_t LED[wire::led_bytes];
  uint8_t SystemState; // see cfg::SYS_STATE
  uint16_t Checksum;
} __attribute__((packed));

// A change here is a protocol break and must land in pit_protocol.py at the
// same time, under a matching minor version in both repos.
static_assert(sizeof(HeaderPacket) == 16, "TX header size changed");
static_assert(sizeof(HeaderPacketNoMagic) == 12, "RX header size changed");
static_assert(sizeof(TXPacket) == 94, "telemetry packet size changed");
static_assert(sizeof(RxPacket) == 470, "command packet size changed");

constexpr size_t info_packet_max_bytes = sizeof(HeaderPacket) + wire::info_chunk_bytes + 2;

inline void initTxHeader(HeaderPacket &h) {
  h.MagicNum          = wire::telemetry_magic;
  h.VersionNum        = wire::protocol_version;
  h.IDNumber          = 0;
  h.PacketBytes       = sizeof(TXPacket);
  h.FrameBytes        = 0;
  h.PacketIndex       = 0;
  h.PacketsPerFrame   = 1;
}

// Chunk count for a payload of `payload_len` bytes; an empty payload still
// sends one empty chunk so the Pi sees a reply.
uint16_t infoChunkCount(size_t payload_len);

// Writes info chunk `index` of `payload` into `out` (header, chunk bytes,
// CRC-16) and returns the packet length, or 0 when `index` is out of range or
// `out_cap` is too small.
size_t buildInfoPacket(const char *payload, size_t payload_len, uint8_t revision,
                       uint16_t index, uint8_t *out, size_t out_cap);
