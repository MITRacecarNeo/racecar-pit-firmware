#include "dip_record.h"

void dipTick(DipRecord &r, float volts, uint32_t now_ms, const DipParams &p)
{
  if (volts < p.valid_v)
    return;

  DipSummary &s = r.summary;
  if (!s.any_reading || volts < s.min_v)
    s.min_v = volts;
  s.any_reading = true;

  if (r.counted)
  {
    if (volts >= p.rearm_v)
      r.counted = false;
    return;
  }

  if (volts >= p.dip_v)
  {
    r.below = false;
    return;
  }
  if (!r.below)
  {
    r.below = true;
    r.below_since_ms = now_ms;
  }
  if ((uint32_t)(now_ms - r.below_since_ms) >= p.min_ms)
  {
    s.count++;
    s.last_dip_ms = r.below_since_ms;
    r.below = false;
    r.counted = true;
  }
}
