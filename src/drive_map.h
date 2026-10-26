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

// Direction debounce: a reversal is accepted only after `confirm` consecutive
// transitions agree, so one bad edge cannot flip the speed sign. The first
// direction from a standstill (committed 0) is accepted at once.
struct HallDirFilter
{
  int8_t committed = 0;
  int8_t pending = 0;
  uint8_t count = 0;
};

// Median of n intervals (n <= 8); 0 for n == 0. A contact bounce adds a pair
// of near-zero intervals, which shifts a mean but not the median.
uint32_t medianInterval(const uint32_t *v, uint8_t n);

// Feeds one hallDirection() result; returns the committed direction.
int8_t filterHallDirection(HallDirFilter &f, int8_t d, uint8_t confirm);

// True when an RC pulse is inside the plausible band. PPMReader reports 0 for a
// channel that is absent, out of band, or past its failsafe.
bool rcPulseValid(uint16_t pulse, uint16_t lo, uint16_t hi);

// Two-position switch decode. An implausible pulse reads as down, so a lost
// receiver falls back to the permissive position rather than latching a switch.
bool rcSwitchUp(uint16_t pulse, uint16_t lo, uint16_t hi, uint16_t threshold);

// The other position. An implausible pulse reads as neither up nor down.
bool rcSwitchDown(uint16_t pulse, uint16_t lo, uint16_t hi, uint16_t threshold);

// Normalized [-1,1] throttle to a signed speed setpoint. Magnitudes past the
// deadband span min_mps to max_mps, so the first usable stick position already
// clears the ESC deadband and full stick lands on max_mps exactly. Passing a
// reduced max_mps rescales the whole travel rather than clipping its top, which
// is what keeps stick resolution usable in slow mode.
float throttleToTargetMps(float motor, float deadband, float min_mps, float max_mps);

// Integer range map, rounded toward zero; the same arithmetic as Teensyduino's
// map() for integer inputs.
int32_t mapRange(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

// RC throttle stick setting, from one pulse width.
struct RcThrottleMap
{
  uint16_t pulse_min, pulse_neutral, pulse_max; // stick travel, us
  uint16_t rev_break, fwd_break;                // stick deadzone, us
  uint16_t fwd_floor;                           // first PWM past the ESC forward deadband
  uint16_t pwm_min, pwm_neutral, pwm_max;       // RC-manual ESC endpoints
};

// Direct-path ESC PWM for a stick pulse: neutral inside the deadzone, then
// fwd_floor..pwm_max forward and pwm_min..rev_break in reverse.
uint16_t rcThrottleToPwm(uint16_t pulse, const RcThrottleMap &m);

// The same stick as a normalized [-1,1] command, for the closed loop; 0 inside
// the deadzone.
float rcThrottleToNorm(uint16_t pulse, const RcThrottleMap &m);
