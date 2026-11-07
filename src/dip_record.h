#pragma once
#include <stdint.h>

// Battery voltage dip record. Arduino-free, unit tested under tests/.
//
// A dip is the bus voltage below dip_v for at least min_ms. The next dip
// counts only after the voltage recovers to rearm_v. Readings below valid_v
// (no sensor reading yet) are ignored.

struct DipParams
{
  float dip_v;
  float rearm_v;
  uint32_t min_ms;
  float valid_v;
};

struct DipSummary
{
  bool any_reading;     // min_v is valid
  float min_v;          // lowest reading since boot
  uint16_t count;       // dips since boot
  uint32_t last_dip_ms; // uptime at the start of the last dip; valid when count > 0
};

struct DipRecord
{
  DipSummary summary = {false, 0.0f, 0, 0};
  bool below = false;    // under dip_v, not yet counted
  bool counted = false;  // current dip counted; waiting for rearm_v
  uint32_t below_since_ms = 0;
};

void dipTick(DipRecord &r, float volts, uint32_t now_ms, const DipParams &p);
