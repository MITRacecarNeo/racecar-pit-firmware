#include "car_constants.h"

#include <string.h>

#include "crc16.h"

namespace car_constants
{
  const Field fields[count] = {
      {"pid.ff_fwd_us", &SpeedConstants::ff_fwd_us, 0.0f, 300.0f},
      {"pid.kv_ff_fwd", &SpeedConstants::kv_ff_fwd, 0.0f, 200.0f},
      {"pid.ff_rev_us", &SpeedConstants::ff_rev_us, 0.0f, 300.0f},
      {"pid.kv_ff_rev", &SpeedConstants::kv_ff_rev, 0.0f, 200.0f},
      {"pid.kp", &SpeedConstants::kp, 0.0f, 200.0f},
      {"pid.kp_boost", &SpeedConstants::kp_boost, 0.0f, 300.0f},
      {"pid.boost_error_mps", &SpeedConstants::boost_error_mps, 0.0f, 2.0f},
      {"pid.ki", &SpeedConstants::ki, 0.0f, 500.0f},
      {"pid.ki_rev", &SpeedConstants::ki_rev, 0.0f, 500.0f},
      {"pid.max_integral", &SpeedConstants::max_integral, 0.0f, 1000.0f},
      {"pid.start_rev_from_us", &SpeedConstants::start_rev_from_us, 0.0f, 500.0f},
      {"pid.start_rev_max_us", &SpeedConstants::start_rev_max_us, 0.0f, 500.0f},
  };
}

namespace
{
  using namespace car_constants;

  void put16(uint8_t *p, uint16_t v)
  {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
  }
  void put32(uint8_t *p, uint32_t v)
  {
    for (int i = 0; i < 4; i++)
      p[i] = (uint8_t)(v >> (8 * i));
  }
  uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
  uint32_t get32(const uint8_t *p)
  {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  }

  bool inRange(float v, const Field &f)
  {
    return v == v && v >= f.lo && v <= f.hi; // v == v rejects NaN
  }
}

const char *blockStatusName(BlockStatus s)
{
  switch (s)
  {
  case BlockStatus::ok:
    return "ok";
  case BlockStatus::empty:
    return "empty";
  case BlockStatus::bad_magic:
    return "bad_magic";
  case BlockStatus::bad_format:
    return "bad_format";
  case BlockStatus::bad_crc:
    return "bad_crc";
  case BlockStatus::out_of_range:
    return "out_of_range";
  }
  return "unknown";
}

void encodeBlock(const SpeedConstants &c, uint32_t written_ymd, uint8_t *out)
{
  put32(out, magic);
  put16(out + 4, format);
  put16(out + 6, (uint16_t)sizeof(SpeedConstants));
  for (size_t i = 0; i < count; i++)
  {
    uint32_t bits;
    const float v = c.*fields[i].member;
    memcpy(&bits, &v, 4);
    put32(out + 8 + 4 * i, bits);
  }
  put32(out + 8 + sizeof(SpeedConstants), written_ymd);
  put16(out + block_bytes - 2, crc16_ccitt(out, block_bytes - 2));
}

BlockStatus decodeBlock(const uint8_t *in, SpeedConstants &c, uint32_t &written_ymd, uint16_t &crc)
{
  bool erased = true;
  for (size_t i = 0; i < block_bytes; i++)
    erased = erased && in[i] == 0xFF;
  if (erased)
    return BlockStatus::empty;
  if (get32(in) != magic)
    return BlockStatus::bad_magic;
  if (get16(in + 4) != format || get16(in + 6) != sizeof(SpeedConstants))
    return BlockStatus::bad_format;
  crc = get16(in + block_bytes - 2);
  if (crc16_ccitt(in, block_bytes - 2) != crc)
    return BlockStatus::bad_crc;

  SpeedConstants read = {};
  for (size_t i = 0; i < count; i++)
  {
    const uint32_t bits = get32(in + 8 + 4 * i);
    float v;
    memcpy(&v, &bits, 4);
    read.*fields[i].member = v;
  }
  if (firstOutOfRange(read) >= 0)
    return BlockStatus::out_of_range;
  c = read;
  written_ymd = get32(in + 8 + sizeof(SpeedConstants));
  return BlockStatus::ok;
}

int firstOutOfRange(const SpeedConstants &c)
{
  for (size_t i = 0; i < count; i++)
    if (!inRange(c.*fields[i].member, fields[i]))
      return (int)i;
  if (c.start_rev_max_us < c.start_rev_from_us)
    return (int)count - 1;
  return -1;
}

bool setByKey(SpeedConstants &c, const char *key, float value)
{
  for (size_t i = 0; i < count; i++)
  {
    if (strcmp(key, fields[i].key) != 0)
      continue;
    if (!inRange(value, fields[i]))
      return false;
    SpeedConstants next = c;
    next.*fields[i].member = value;
    if (next.start_rev_max_us < next.start_rev_from_us)
      return false;
    c = next;
    return true;
  }
  return false;
}

bool sameConstants(const SpeedConstants &a, const SpeedConstants &b)
{
  return memcmp(&a, &b, sizeof(SpeedConstants)) == 0;
}

#include <stdlib.h>

#include "info.h"

ConstantsSession openSession(const uint8_t *block, const SpeedConstants &defaults)
{
  ConstantsSession s = {defaults, defaults, BlockStatus::empty, 0, 0};
  s.eeprom = decodeBlock(block, s.saved, s.written_ymd, s.crc);
  if (s.eeprom == BlockStatus::ok)
    s.live = s.saved;
  else
  {
    s.saved = defaults;
    s.crc = 0;
    s.written_ymd = 0;
  }
  return s;
}

const char *constantsSource(const ConstantsSession &s, const SpeedConstants &defaults)
{
  if (s.eeprom == BlockStatus::ok)
    return sameConstants(s.live, s.saved) ? "eeprom" : "eeprom_modified";
  return sameConstants(s.live, defaults) ? "builtin" : "builtin_modified";
}

namespace
{
  // Appends "a[ b]\n" at out[n]; returns the new length, or 0 when it does
  // not fit.
  size_t appendLine(char *out, size_t cap, size_t n, const char *a, const char *b = "")
  {
    const size_t la = strlen(a), lb = strlen(b);
    const size_t need = la + (lb ? lb + 1 : 0) + 1;
    if (n + need + 1 > cap)
      return 0;
    memcpy(out + n, a, la);
    n += la;
    if (lb)
    {
      out[n++] = ' ';
      memcpy(out + n, b, lb);
      n += lb;
    }
    out[n++] = '\n';
    out[n] = '\0';
    return n;
  }

  size_t reply(char *out, size_t cap, const char *a, const char *b = "")
  {
    return appendLine(out, cap, 0, a, b);
  }

  bool parseYmd(const char *s, uint32_t &ymd)
  {
    char *end = nullptr;
    const unsigned long v = strtoul(s, &end, 10);
    if (end == s || (*end != '\0' && *end != ' ') || v < 20200101UL || v > 20991231UL)
      return false;
    const unsigned long m = (v / 100) % 100, d = v % 100;
    if (m < 1 || m > 12 || d < 1 || d > 31)
      return false;
    ymd = (uint32_t)v;
    return true;
  }

  void writeDate(InfoWriter &w, const char *key, uint32_t ymd)
  {
    if (ymd == 0)
    {
      w.text(key, "none");
      return;
    }
    char d[11] = "0000-00-00";
    const uint32_t y = ymd / 10000, m = (ymd / 100) % 100, day = ymd % 100;
    d[0] = (char)('0' + (y / 1000) % 10);
    d[1] = (char)('0' + (y / 100) % 10);
    d[2] = (char)('0' + (y / 10) % 10);
    d[3] = (char)('0' + y % 10);
    d[5] = (char)('0' + m / 10);
    d[6] = (char)('0' + m % 10);
    d[8] = (char)('0' + day / 10);
    d[9] = (char)('0' + day % 10);
    w.text(key, d);
  }

  void writeCrc(InfoWriter &w, const char *key, uint16_t crc, bool valid)
  {
    if (!valid)
    {
      w.text(key, "none");
      return;
    }
    static const char hex[] = "0123456789abcdef";
    char c[5] = {hex[(crc >> 12) & 15], hex[(crc >> 8) & 15], hex[(crc >> 4) & 15], hex[crc & 15], '\0'};
    w.text(key, c);
  }
}

void writeConstantsKeys(InfoWriter &w, const SpeedConstants &c)
{
  for (size_t i = 0; i < car_constants::count; i++)
    w.fixed(car_constants::fields[i].key, c.*car_constants::fields[i].member, 3);
}

void writeCalKeys(InfoWriter &w, const ConstantsInfo &ci)
{
  w.text("cal.source", ci.source);
  w.integer("cal.format", car_constants::format);
  w.text("cal.eeprom", blockStatusName(ci.eeprom));
  writeCrc(w, "cal.crc", ci.crc, ci.eeprom == BlockStatus::ok);
  writeDate(w, "cal.written", ci.written_ymd);
}

ConstantsInfo constantsInfo(const ConstantsSession &s, const SpeedConstants &defaults)
{
  return {true, s.live, constantsSource(s, defaults), s.crc, s.written_ymd, s.eeprom};
}

ConstantsAction constantsCommand(ConstantsSession &s, const SpeedConstants &defaults, const char *line,
                                 bool stopped, uint32_t &save_ymd, char *out, size_t cap)
{
  out[0] = '\0';
  while (*line == ' ')
    line++;
  if (strncmp(line, "cal ", 4) != 0)
  {
    reply(out, cap, "err not a cal command");
    return ConstantsAction::none;
  }
  const char *verb = line + 4;
  while (*verb == ' ')
    verb++;

  if (strcmp(verb, "get") == 0)
  {
    InfoWriter w(out, cap);
    writeCalKeys(w, constantsInfo(s, defaults));
    writeConstantsKeys(w, s.live);
    if (w.length() == 0 || appendLine(out, cap, w.length(), "ok get") == 0)
      reply(out, cap, "err reply too long");
    return ConstantsAction::none;
  }
  if (strncmp(verb, "set ", 4) == 0)
  {
    if (!stopped)
    {
      reply(out, cap, "err car moving");
      return ConstantsAction::none;
    }
    SpeedConstants next = s.live;
    char buf[160];
    strncpy(buf, verb + 4, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    int pairs = 0;
    for (char *tok = strtok(buf, " "); tok; tok = strtok(nullptr, " "))
    {
      char *eq = strchr(tok, '=');
      char *end = nullptr;
      float v = 0.0f;
      if (eq)
      {
        *eq = '\0';
        v = strtof(eq + 1, &end);
      }
      if (!eq || end == eq + 1 || *end != '\0' || !setByKey(next, tok, v))
      {
        reply(out, cap, "err bad value", tok);
        return ConstantsAction::none;
      }
      pairs++;
    }
    if (pairs == 0)
    {
      reply(out, cap, "err nothing to set");
      return ConstantsAction::none;
    }
    s.live = next;
    reply(out, cap, "ok set");
    return ConstantsAction::apply;
  }
  if (strncmp(verb, "save", 4) == 0 && (verb[4] == ' ' || verb[4] == '\0'))
  {
    if (!stopped)
    {
      reply(out, cap, "err car moving");
      return ConstantsAction::none;
    }
    if (verb[4] != ' ' || !parseYmd(verb + 5, save_ymd))
    {
      reply(out, cap, "err save needs YYYYMMDD");
      return ConstantsAction::none;
    }
    return ConstantsAction::save;
  }
  if (strcmp(verb, "revert") == 0)
  {
    s.live = s.eeprom == BlockStatus::ok ? s.saved : defaults;
    reply(out, cap, "ok revert");
    return ConstantsAction::apply;
  }
  if (strcmp(verb, "defaults") == 0)
  {
    if (!stopped)
    {
      reply(out, cap, "err car moving");
      return ConstantsAction::none;
    }
    s.live = defaults;
    reply(out, cap, "ok defaults");
    return ConstantsAction::apply;
  }
  if (strcmp(verb, "erase") == 0)
  {
    if (!stopped)
    {
      reply(out, cap, "err car moving");
      return ConstantsAction::none;
    }
    return ConstantsAction::erase;
  }
  reply(out, cap, "err unknown command");
  return ConstantsAction::none;
}

size_t finishSave(ConstantsSession &s, const uint8_t *block, char *out, size_t cap)
{
  SpeedConstants read = {};
  uint32_t ymd = 0;
  uint16_t crc = 0;
  const BlockStatus st = decodeBlock(block, read, ymd, crc);
  if (st != BlockStatus::ok || !sameConstants(read, s.live))
  {
    s.eeprom = st;
    return reply(out, cap, "err readback", blockStatusName(st));
  }
  s.saved = read;
  s.eeprom = st;
  s.crc = crc;
  s.written_ymd = ymd;
  InfoWriter w(out, cap);
  writeCrc(w, "cal.crc", crc, true);
  return w.length() ? appendLine(out, cap, w.length(), "ok save") : 0;
}

size_t finishErase(ConstantsSession &s, const uint8_t *block, char *out, size_t cap)
{
  SpeedConstants read = {};
  uint32_t ymd = 0;
  uint16_t crc = 0;
  s.eeprom = decodeBlock(block, read, ymd, crc);
  s.crc = 0;
  s.written_ymd = 0;
  if (s.eeprom != BlockStatus::empty)
    return reply(out, cap, "err readback", blockStatusName(s.eeprom));
  return reply(out, cap, "ok erase");
}
