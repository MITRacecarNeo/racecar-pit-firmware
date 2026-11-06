#include "led_patterns.h"

namespace
{
  uint8_t lerp8(uint8_t a, uint8_t b, uint32_t num, uint32_t den)
  {
    return (uint8_t)(((uint32_t)a * (den - num) + (uint32_t)b * num + den / 2) / den);
  }

  // Index of the LED at distance d from the back middle, on either side.
  uint16_t leftOf(uint16_t n, uint16_t d) { return n / 2 - 1 - d; }
  uint16_t rightOf(uint16_t n, uint16_t d) { return n / 2 + d; }
}

Rgb gradientAt(uint16_t d, uint16_t half, const LedGradient &g)
{
  if (half <= 1)
    return g.inner;
  if (d >= half)
    d = half - 1;
  const uint32_t den = half - 1;
  return {lerp8(g.inner.r, g.outer.r, d, den), lerp8(g.inner.g, g.outer.g, d, den),
          lerp8(g.inner.b, g.outer.b, d, den)};
}

void fill(Rgb *out, uint16_t n, Rgb c)
{
  for (uint16_t i = 0; i < n; i++)
    out[i] = c;
}

void batteryMeter(Rgb *out, uint16_t n, uint8_t soc_percent, const LedGradient &g)
{
  fill(out, n, {0, 0, 0});
  const uint16_t half = n / 2;
  const uint8_t soc = soc_percent > 100 ? 100 : soc_percent;
  const uint16_t lit = (uint16_t)(((uint32_t)soc * half + 50) / 100);
  for (uint16_t d = 0; d < lit; d++)
  {
    const Rgb c = gradientAt(d, half, g);
    out[leftOf(n, d)] = c;
    out[rightOf(n, d)] = c;
  }
}

void loadingScroll(Rgb *out, uint16_t n, uint32_t ms, uint32_t step_ms, const LedGradient &g)
{
  // One wave spans the half: orange rising to crimson and back, so the
  // pattern repeats without a seam.
  const uint16_t half = n / 2;
  if (half == 0 || step_ms == 0)
    return;
  const uint32_t period = 2u * half;
  const uint32_t shift = (ms / step_ms) % period;
  for (uint16_t d = 0; d < half; d++)
  {
    const uint32_t phase = (d + period - shift) % period;
    const uint16_t pos = (uint16_t)(phase < half ? phase : period - 1 - phase);
    const Rgb c = gradientAt(pos, half, g);
    out[leftOf(n, d)] = c;
    out[rightOf(n, d)] = c;
  }
}
