#include "led_patterns.h"

void fill(Rgb *out, uint16_t n, Rgb c)
{
  for (uint16_t i = 0; i < n; i++)
    out[i] = c;
}

void batteryMeter(Rgb *out, uint16_t n, uint8_t soc_percent, Rgb color)
{
  fill(out, n, {0, 0, 0});
  const uint8_t soc = soc_percent > 100 ? 100 : soc_percent;
  const uint16_t lit = (uint16_t)((uint32_t)soc * (n / 2) / 100);
  for (uint16_t i = 0; i < lit; i++)
  {
    out[i] = color;
    out[n - 1 - i] = color;
  }
}

void loadingBar(Rgb *out, uint16_t n, uint32_t elapsed_ms, uint32_t duration_ms, Rgb color)
{
  fill(out, n, {0, 0, 0});
  const uint16_t lit = (duration_ms == 0 || elapsed_ms >= duration_ms)
                           ? n
                           : (uint16_t)((uint64_t)elapsed_ms * n / duration_ms);
  for (uint16_t i = 0; i < lit; i++)
    out[i] = color;
}
