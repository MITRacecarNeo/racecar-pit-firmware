#include "command_framer.h"

#include <string.h>

#include "crc16.h"

bool CommandFramer::push(uint8_t byte)
{
  if (!synced_)
  {
    memmove(window_, window_ + 1, sizeof(window_) - 1);
    window_[3] = byte;
    uint32_t seen;
    memcpy(&seen, window_, sizeof(seen));
    if (seen == wire::command_magic)
    {
      synced_ = true;
      collected_ = 0;
      memset(window_, 0, sizeof(window_));
    }
    return false;
  }

  reinterpret_cast<uint8_t *>(&frame_)[collected_++] = byte;
  if (collected_ < sizeof(RxPacket))
    return false;
  synced_ = false;
  collected_ = 0;
  return true;
}

void CommandFramer::reset()
{
  synced_ = false;
  collected_ = 0;
  memset(window_, 0, sizeof(window_));
}

bool CommandFramer::crcOk() const
{
  // magic is little-endian here as on the Teensy, so its bytes match the wire.
  const uint32_t magic = wire::command_magic;
  const uint16_t crc = crc16_ccitt(reinterpret_cast<const uint8_t *>(&magic), sizeof(magic));
  return crc16_ccitt(reinterpret_cast<const uint8_t *>(&frame_), sizeof(RxPacket) - 2, crc) == frame_.Checksum;
}
