#pragma once
#include <stddef.h>
#include <stdint.h>

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final xor).
// Check value over "123456789" is 0x29B1. Matches the Pi driver's
// pit_protocol.crc16_ccitt so both ends agree on the checksum. Pass the running
// value back in as `crc` to checksum a frame in more than one chunk.
uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF);
