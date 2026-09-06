#pragma once
#include <stdint.h>

// Arduino-free actuation math, unit tested under test/.

// Map a normalized [-1,1] command to a PWM microsecond value, piecewise about
// neutral: -1 -> lo, 0 -> neu, +1 -> hi. Result clamped to [lo, hi].
uint16_t mapNormToPwm(float n, uint16_t lo, uint16_t neu, uint16_t hi);

// Rotation direction from a pair of consecutive 3-bit hall states (A<<2|B<<1|C).
// Returns +1, -1, or 0 for a state pair that is not adjacent in the commutation
// sequence (a glitch or a missed edge).
int8_t hallDirection(uint8_t prev, uint8_t curr);
