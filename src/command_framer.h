#pragma once
#include <stddef.h>
#include <stdint.h>

#include "packets.h"

// Assembles Pi command frames from the UART byte stream one byte at a time,
// so the receive thread never waits for a frame to finish arriving.
// Arduino-free and host tested.
//
// Syncs on the command magic with a 4-byte sliding window, then collects
// sizeof(RxPacket) body bytes. The caller abandons a frame that stalls
// (reset()) and checks its CRC (crcOk()).
class CommandFramer
{
public:
    // Feeds one byte; true when it completes a frame, which then stays in
    // frame() until the next byte is pushed.
    bool push(uint8_t byte);

    // Back to syncing; drops a partial frame.
    void reset();

    bool inFrame() const { return synced_; }
    const RxPacket &frame() const { return frame_; }
    // CRC-16 over the magic and the body, against the frame's checksum.
    bool crcOk() const;

private:
    RxPacket frame_ = {};
    uint8_t window_[4] = {0, 0, 0, 0};
    size_t collected_ = 0;
    bool synced_ = false;
};
