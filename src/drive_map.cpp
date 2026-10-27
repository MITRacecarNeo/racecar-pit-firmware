#include "drive_map.h"

uint16_t mapNormToPwm(float n, uint16_t lo, uint16_t neu, uint16_t hi)
{
  if (n > 1.0f)
    n = 1.0f;
  if (n < -1.0f)
    n = -1.0f;

  float pwm = (n >= 0.0f) ? (neu + n * ((float)hi - (float)neu))
                          : (neu + n * ((float)neu - (float)lo));

  if (pwm < (float)lo)
    pwm = (float)lo;
  if (pwm > (float)hi)
    pwm = (float)hi;
  return (uint16_t)pwm;
}

int8_t hallDirection(uint8_t prev, uint8_t curr)
{
  static const uint8_t forwardSeq[6] = {1, 3, 2, 6, 4, 5};

  for (uint8_t i = 0; i < 6; i++)
  {
    if (forwardSeq[i] == prev)
    {
      if (curr == forwardSeq[(i + 1) % 6])
        return -1;
      if (curr == forwardSeq[(i + 5) % 6])
        return 1;
      return 0;
    }
  }
  return 0;
}

uint32_t medianInterval(const uint32_t *v, uint8_t n)
{
  if (n == 0)
    return 0;
  uint32_t s[8];
  if (n > 8)
    n = 8;
  for (uint8_t i = 0; i < n; i++)
  {
    uint32_t x = v[i];
    uint8_t j = i;
    for (; j > 0 && s[j - 1] > x; j--)
      s[j] = s[j - 1];
    s[j] = x;
  }
  return (n % 2) ? s[n / 2] : (s[n / 2 - 1] + s[n / 2]) / 2;
}

int8_t filterHallDirection(HallDirFilter &f, int8_t d, uint8_t confirm)
{
  if (d == 0)
    return f.committed;
  if (f.committed == 0 || d == f.committed)
  {
    f.committed = d;
    f.count = 0;
    return f.committed;
  }
  if (d == f.pending)
    f.count++;
  else
  {
    f.pending = d;
    f.count = 1;
  }
  if (f.count >= confirm)
  {
    f.committed = d;
    f.count = 0;
  }
  return f.committed;
}

bool rcPulseValid(uint16_t pulse, uint16_t lo, uint16_t hi)
{
  return pulse >= lo && pulse <= hi;
}

bool rcSwitchUp(uint16_t pulse, uint16_t lo, uint16_t hi, uint16_t threshold)
{
  return rcPulseValid(pulse, lo, hi) && pulse >= threshold;
}

bool rcSwitchDown(uint16_t pulse, uint16_t lo, uint16_t hi, uint16_t threshold)
{
  return rcPulseValid(pulse, lo, hi) && pulse < threshold;
}

float throttleToTargetMps(float motor, float deadband, float min_mps, float max_mps)
{
  if (motor > 1.0f)
    motor = 1.0f;
  if (motor < -1.0f)
    motor = -1.0f;

  if (motor > 0.0f)
  {
    float scale = (motor - deadband) / (1.0f - deadband);
    return min_mps + scale * (max_mps - min_mps);
  }
  float scale = (motor + deadband) / (-1.0f + deadband);
  return -min_mps - scale * (max_mps - min_mps);
}

int32_t mapRange(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  const int32_t in_range = in_max - in_min;
  const int32_t out_range = out_max - out_min;
  if (in_range == 0)
    return out_min + out_range / 2;
  int32_t num = (x - in_min) * out_range;
  if (out_range >= 0)
    num += in_range / 2;
  else
    num -= in_range / 2;
  const int32_t result = num / in_range + out_min;
  if (out_range >= 0)
  {
    if ((int64_t)in_range * num < 0)
      return result - 1;
  }
  else if ((int64_t)in_range * num >= 0)
  {
    return result + 1;
  }
  return result;
}

uint16_t rcThrottleToPwm(uint16_t pulse, const RcThrottleMap &m)
{
  int32_t pwm = m.pwm_neutral;
  if (pulse > m.fwd_break)
    pwm = mapRange(pulse, m.pulse_neutral, m.pulse_max, m.fwd_floor, m.pwm_max);
  else if (pulse < m.rev_break)
    pwm = mapRange(pulse, m.pulse_min, m.pulse_neutral, m.pwm_min, m.rev_break);
  if (pwm < m.pwm_min)
    pwm = m.pwm_min;
  if (pwm > m.pwm_max)
    pwm = m.pwm_max;
  return (uint16_t)pwm;
}

float rcThrottleToNorm(uint16_t pulse, const RcThrottleMap &m)
{
  float n = 0.0f;
  if (pulse > m.fwd_break)
    n = (float)(pulse - m.pulse_neutral) / (float)(m.pulse_max - m.pulse_neutral);
  else if (pulse < m.rev_break)
    n = ((float)pulse - (float)m.pulse_neutral) / (float)(m.pulse_neutral - m.pulse_min);
  if (n > 1.0f)
    n = 1.0f;
  if (n < -1.0f)
    n = -1.0f;
  return n;
}
