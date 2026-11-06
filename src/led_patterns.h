#pragma once
#include <stdint.h>

// LED strip patterns. Arduino-free, unit tested under tests/.
//
// The band runs from index 0 at the front-left around the rear to n-1 at the
// front-right, so the back middle sits between n/2-1 and n/2. Patterns are
// symmetric about it and use the brand gradient: orange at the back middle,
// crimson toward the front tips.

struct Rgb
{
  uint8_t r, g, b;
};

struct LedGradient
{
  Rgb inner; // back middle
  Rgb outer; // front tips
};

// Color at distance d (0 = next to the back middle) out of half LEDs.
Rgb gradientAt(uint16_t d, uint16_t half, const LedGradient &g);

// Charge meter: soc_percent of each half lit from the back middle outward.
void batteryMeter(Rgb *out, uint16_t n, uint8_t soc_percent, const LedGradient &g);

// Gradient waves moving outward from the back middle, one LED per step_ms.
void loadingScroll(Rgb *out, uint16_t n, uint32_t ms, uint32_t step_ms, const LedGradient &g);

void fill(Rgb *out, uint16_t n, Rgb c);
