#include "packets.h"

#include <string.h>

#include "crc16.h"

uint16_t infoChunkCount(size_t payload_len)
{
  if (payload_len == 0)
    return 1;
  return (uint16_t)((payload_len + wire::info_chunk_bytes - 1) / wire::info_chunk_bytes);
}

size_t buildInfoPacket(const char *payload, size_t payload_len, uint8_t revision,
                       uint16_t index, uint8_t *out, size_t out_cap)
{
  const uint16_t count = infoChunkCount(payload_len);
  if (index >= count)
    return 0;

  const size_t offset = (size_t)index * wire::info_chunk_bytes;
  const size_t remaining = payload_len - (payload_len == 0 ? 0 : offset);
  const size_t chunk = remaining < wire::info_chunk_bytes ? remaining : wire::info_chunk_bytes;
  const size_t total = sizeof(HeaderPacket) + chunk + 2;
  if (total > out_cap)
    return 0;

  HeaderPacket h;
  h.MagicNum = wire::info_magic;
  h.VersionNum = wire::protocol_version;
  h.IDNumber = revision;
  h.PacketBytes = (uint16_t)total;
  h.FrameBytes = (uint32_t)payload_len;
  h.PacketIndex = index;
  h.PacketsPerFrame = count;

  memcpy(out, &h, sizeof(h));
  if (chunk > 0)
    memcpy(out + sizeof(h), payload + offset, chunk);
  const uint16_t crc = crc16_ccitt(out, total - 2);
  memcpy(out + total - 2, &crc, sizeof(crc));
  return total;
}
