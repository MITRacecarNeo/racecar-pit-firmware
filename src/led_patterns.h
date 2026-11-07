#pragma once
#include <stdint.h>

// LED strip patterns. Arduino-free, unit tested under tests/.
// The band runs from index 0 at the front-left around the rear to n-1 at the
// front-right.

struct Rgb
{
  uint8_t r, g, b;
};

// Charge meter: soc_percent of each half lit, filling from both front ends
// toward the back middle.
void batteryMeter(Rgb *out, uint16_t n, uint8_t soc_percent, Rgb color);

// Loading bar: fills from both front ends toward the back middle over
// duration_ms, then stays full.
void loadingBar(Rgb *out, uint16_t n, uint32_t elapsed_ms, uint32_t duration_ms, Rgb color);

void fill(Rgb *out, uint16_t n, Rgb c);
