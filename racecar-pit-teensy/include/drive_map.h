#pragma once
#include <stdint.h>

// Arduino-free actuation math, unit tested under test/.

// Normalized [-1,1] to PWM microseconds, piecewise about neutral:
// -1 -> lo, 0 -> neu, +1 -> hi, clamped to [lo, hi].
uint16_t mapNormToPwm(float n, uint16_t lo, uint16_t neu, uint16_t hi);

// Rotation direction from consecutive 3-bit hall states (A<<2|B<<1|C).
// Returns +1, -1, or 0 for a pair that is not adjacent in the commutation
// sequence (a glitch or a missed edge).
int8_t hallDirection(uint8_t prev, uint8_t curr);

// True when an RC pulse is inside the plausible band. PPMReader reports 0 for a
// channel that is absent, out of band, or past its failsafe.
bool rcPulseValid(uint16_t pulse, uint16_t lo, uint16_t hi);

// Two-position switch decode. An implausible pulse reads as down, so a lost
// receiver falls back to the permissive position rather than latching a switch.
bool rcSwitchUp(uint16_t pulse, uint16_t lo, uint16_t hi, uint16_t threshold);

// Normalized [-1,1] throttle to a signed speed setpoint. Magnitudes past the
// deadband span min_mps to max_mps, so the first usable stick position already
// clears the ESC deadband and full stick lands on max_mps exactly. Passing a
// reduced max_mps rescales the whole travel rather than clipping its top, which
// is what keeps stick resolution usable in slow mode.
float throttleToTargetMps(float motor, float deadband, float min_mps, float max_mps);
