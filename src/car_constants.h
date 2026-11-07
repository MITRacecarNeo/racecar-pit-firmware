#pragma once
#include <stddef.h>
#include <stdint.h>

// Per-car speed-loop constants and their EEPROM block. Arduino-free, unit
// tested under tests/.
//
// Block layout, little-endian: magic u32, format u16, payload length u16,
// payload (SpeedConstants as floats in field order), written date u32
// (YYYYMMDD, 0 for unknown), CRC-16/CCITT-FALSE over everything before it.

struct SpeedConstants
{
  float ff_fwd_us;        // pid.ff_fwd_us: forward feedforward offset
  float kv_ff_fwd;        // pid.kv_ff_fwd: forward feedforward, us per m/s
  float ff_rev_us;        // pid.ff_rev_us
  float kv_ff_rev;        // pid.kv_ff_rev
  float kp;               // pid.kp
  float kp_boost;         // pid.kp_boost
  float boost_error_mps;  // pid.boost_error_mps
  float ki;               // pid.ki
  float ki_rev;           // pid.ki_rev
  float max_integral;     // pid.max_integral
  float start_rev_from_us; // pid.start_rev_from_us
  float start_rev_max_us;  // pid.start_rev_max_us
};

namespace car_constants
{
  constexpr uint32_t magic = 0x4B4E4352; // "RCNK"
  constexpr uint16_t format = 1;
  constexpr size_t count = sizeof(SpeedConstants) / sizeof(float);
  constexpr size_t block_bytes = 4 + 2 + 2 + sizeof(SpeedConstants) + 4 + 2;

  struct Field
  {
    const char *key; // info key
    float SpeedConstants::*member;
    float lo, hi; // accepted range
  };
  extern const Field fields[count];
}

static_assert(sizeof(SpeedConstants) == car_constants::count * sizeof(float), "SpeedConstants must be floats only");

enum class BlockStatus : uint8_t
{
  ok,
  empty,        // erased EEPROM (all 0xFF)
  bad_magic,
  bad_format,
  bad_crc,
  out_of_range, // CRC fine, a value outside its range
};

const char *blockStatusName(BlockStatus s);

// Writes block_bytes into out.
void encodeBlock(const SpeedConstants &c, uint32_t written_ymd, uint8_t *out);

// Fills c and written_ymd only when the result is ok.
BlockStatus decodeBlock(const uint8_t *in, SpeedConstants &c, uint32_t &written_ymd, uint16_t &crc);

// Index of the first value outside its range, or -1.
int firstOutOfRange(const SpeedConstants &c);

// Sets the field named by its info key; false for an unknown key, a value
// outside the range, or start_rev_max_us below start_rev_from_us.
bool setByKey(SpeedConstants &c, const char *key, float value);

bool sameConstants(const SpeedConstants &a, const SpeedConstants &b);

// State of the constants on the car: what drives now, what the EEPROM holds.
struct ConstantsSession
{
  SpeedConstants live;    // used by the speed loop
  SpeedConstants saved;   // EEPROM copy, valid when eeprom == ok
  BlockStatus eeprom;
  uint16_t crc;           // of the EEPROM block, when ok
  uint32_t written_ymd;   // when ok
};

// Starts from the EEPROM block when it decodes, else from the defaults.
ConstantsSession openSession(const uint8_t *block, const SpeedConstants &defaults);

// "eeprom", "eeprom_modified", "builtin" or "builtin_modified".
const char *constantsSource(const ConstantsSession &s, const SpeedConstants &defaults);

enum class ConstantsAction : uint8_t
{
  none,
  apply, // live changed; publish it to the speed loop
  save,  // write live to the EEPROM with the date in `save_ymd`
  erase, // erase the EEPROM block
};

// One USB command line, "cal <verb> ...":
//   cal get                      every key=value, then "ok"
//   cal set KEY=VALUE [...]      all or nothing; applies at once, not saved
//   cal save YYYYMMDD            write live to the EEPROM (car stopped)
//   cal revert                   live back to the EEPROM copy, or defaults
//   cal defaults                 live to the built-in fleet values
//   cal erase                    erase the EEPROM block (car stopped)
// The reply goes to `out` ("err ..." on failure). For save and erase the
// caller does the write, then calls finishSave / finishErase for the reply.
ConstantsAction constantsCommand(ConstantsSession &s, const SpeedConstants &defaults, const char *line,
                                 bool stopped, uint32_t &save_ymd, char *out, size_t cap);

// After the caller wrote the block and read it back into `block`.
size_t finishSave(ConstantsSession &s, const uint8_t *block, char *out, size_t cap);
size_t finishErase(ConstantsSession &s, const uint8_t *block, char *out, size_t cap);

// For the info reply.
struct ConstantsInfo
{
  bool valid; // false: report the defaults
  SpeedConstants live;
  const char *source;
  uint16_t crc;
  uint32_t written_ymd; // 0 for none
  BlockStatus eeprom;
};
ConstantsInfo constantsInfo(const ConstantsSession &s, const SpeedConstants &defaults);

class InfoWriter;
// The per-car pid.* keys and the cal.* keys, as in the info reply.
void writeConstantsKeys(InfoWriter &w, const SpeedConstants &c);
void writeCalKeys(InfoWriter &w, const ConstantsInfo &ci);
