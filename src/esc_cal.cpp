#include "esc_cal.h"

#include <stdio.h>

bool escCalApply(EscCal &cal, char c, uint32_t now_ms)
{
  cal.last_rx_ms = now_ms;
  switch (c)
  {
  case '?':
    return true;
  case 'n':
    cal.step = cal.step == EscCalStep::FullReverse ? EscCalStep::Done : EscCalStep::Neutral;
    cal.aborted = false;
    return true;
  case 'f':
    if (cal.step != EscCalStep::Neutral && cal.step != EscCalStep::Done)
      return false;
    cal.step = EscCalStep::FullForward;
    cal.aborted = false;
    return true;
  case 'r':
    if (cal.step != EscCalStep::FullForward)
      return false;
    cal.step = EscCalStep::FullReverse;
    return true;
  default:
    return false;
  }
}

void escCalTick(EscCal &cal, uint32_t now_ms, uint32_t timeout_ms)
{
  const bool held = cal.step == EscCalStep::FullForward || cal.step == EscCalStep::FullReverse;
  if (held && (uint32_t)(now_ms - cal.last_rx_ms) > timeout_ms)
  {
    cal.step = EscCalStep::Neutral;
    cal.aborted = true;
  }
}

uint16_t escCalPwm(const EscCal &cal, uint16_t pwm_min, uint16_t pwm_neutral, uint16_t pwm_max)
{
  switch (cal.step)
  {
  case EscCalStep::FullForward:
    return pwm_max;
  case EscCalStep::FullReverse:
    return pwm_min;
  default:
    return pwm_neutral;
  }
}

const char *escCalStepName(EscCalStep step)
{
  switch (step)
  {
  case EscCalStep::FullForward:
    return "full_forward";
  case EscCalStep::FullReverse:
    return "full_reverse";
  case EscCalStep::Done:
    return "done";
  default:
    return "neutral";
  }
}

size_t escCalStatus(const EscCal &cal, uint16_t pwm, char *buf, size_t cap)
{
  const int n = snprintf(buf, cap, "esc_cal step=%s pwm=%u aborted=%d\n", escCalStepName(cal.step), (unsigned)pwm,
                         cal.aborted ? 1 : 0);
  return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
}

int escCalLedHalf(uint16_t pwm, uint16_t pwm_min, uint16_t pwm_max, int half)
{
  if (half < 1 || pwm_max <= pwm_min)
    return half < 1 ? 0 : 1;
  if (pwm <= pwm_min)
    return 1;
  if (pwm >= pwm_max)
    return half;
  return 1 + (int)((uint32_t)(pwm - pwm_min) * (uint32_t)(half - 1) / (uint32_t)(pwm_max - pwm_min));
}
