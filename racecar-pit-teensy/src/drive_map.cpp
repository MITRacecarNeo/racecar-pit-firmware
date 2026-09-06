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
