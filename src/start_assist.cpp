#include "start_assist.h"

bool motionConfirmed(const MotionSinceStop &m, int8_t dir, float measured_mps, uint32_t now_us,
                     uint16_t confirm_intervals, uint32_t confirm_us)
{
  if (m.stopped || dir == 0 || m.intervals < confirm_intervals)
    return false;
  if ((uint32_t)(now_us - m.first_edge_us) < confirm_us)
    return false;
  return dir > 0 ? measured_mps > 0.0f : measured_mps < 0.0f;
}

bool handoverSpeedReached(float target_mps, float measured_mps, const StartAssistParams &p)
{
  const float target = target_mps < 0.0f ? -target_mps : target_mps;
  const float measured = measured_mps < 0.0f ? -measured_mps : measured_mps;
  float handover = p.handover_frac * target;
  if (p.handover_mps < handover)
    handover = p.handover_mps;
  return measured >= handover;
}

StartAssistOutput startAssistTick(StartAssist &s, int8_t dir, bool motion_confirmed, bool speed_reached,
                                  uint32_t now_us, const StartAssistParams &p)
{
  StartAssistOutput out = {false, false, 0.0f};
  if (dir == 0)
  {
    s = StartAssist{};
    return out;
  }
  if (dir != s.dir)
  {
    s = StartAssist{};
    s.dir = dir;
    s.try_start_us = now_us;
  }

  const bool enabled = p.max_us > 0.0f;
  const uint32_t elapsed_try = now_us - s.try_start_us;

  // Hand over once motion is confirmed and the speed is up (or the try has
  // run its time); after that only a standstill restarts the assist.
  if (motion_confirmed && (s.confirmed || !enabled || speed_reached || elapsed_try >= p.timeout_us))
  {
    s.confirmed = true;
    s.resting = false;
    out.confirmed = true;
    return out;
  }
  if (s.confirmed)
  {
    s.confirmed = false;
    s.try_start_us = now_us;
  }

  if (!enabled)
    return out;

  if (s.resting)
  {
    if ((uint32_t)(now_us - s.rest_start_us) < p.rest_us)
    {
      out.rest = true;
      return out;
    }
    s.resting = false;
    s.try_start_us = now_us;
  }

  const uint32_t elapsed = now_us - s.try_start_us;
  if (!motion_confirmed && elapsed >= p.timeout_us)
  {
    s.resting = true;
    s.rest_start_us = now_us;
    out.rest = true;
    return out;
  }

  float offset = p.from_us + p.ramp_us_per_s * (float)elapsed * 1e-6f;
  if (offset > p.max_us)
    offset = p.max_us;
  out.offset_us = offset;
  return out;
}
