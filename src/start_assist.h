#pragma once
#include <stdint.h>

// Start assist for the speed loop. Arduino-free, unit tested under tests/.
//
// From a standstill a few hall edges arrive close together even when the rotor
// only twitches, and their short intervals read as real speed. Motion counts as
// confirmed only after `confirm_intervals` intervals since the standstill,
// spanning at least `confirm_us`, in the target direction.
//
// Until then, in a direction with an assist configured, the output offset from
// neutral is at least a ramp from `from_us` up to `max_us` at `ramp_us_per_s`.
// The assist keeps the ramp after confirmed motion until the speed reaches the
// handover speed (the caller's `speed_reached`), or until `timeout_us` with the
// motion confirmed. With no confirmed motion after `timeout_us`, the output
// rests at neutral for `rest_us` and the ramp starts over.

struct StartAssistParams
{
  float from_us;       // offset magnitude at the start of the ramp; 0 with max_us 0 = no assist
  float max_us;        // offset magnitude the ramp holds at
  float ramp_us_per_s; // ramp rate
  uint32_t timeout_us; // assist time before a rest
  uint32_t rest_us;    // neutral time before the next try
  float handover_mps;  // hand over at this speed or handover_frac of the target, whichever is lower
  float handover_frac;
};

struct MotionSinceStop
{
  uint16_t intervals;     // hall intervals since the last standstill
  uint32_t first_edge_us; // time of the first edge after the standstill
  bool stopped;           // no edge within the standstill timeout
};

struct StartAssist
{
  int8_t dir = 0;        // direction being started, 0 when idle
  bool confirmed = false;
  uint32_t try_start_us = 0;
  bool resting = false;
  uint32_t rest_start_us = 0;
};

struct StartAssistOutput
{
  bool confirmed;  // motion confirmed in the target direction
  bool rest;       // write neutral this tick
  float offset_us; // minimum offset magnitude from the assist, 0 for none
};

bool motionConfirmed(const MotionSinceStop &m, int8_t dir, float measured_mps, uint32_t now_us,
                     uint16_t confirm_intervals, uint32_t confirm_us);

// Whether |measured| has reached the handover speed for this target.
bool handoverSpeedReached(float target_mps, float measured_mps, const StartAssistParams &p);

// One drive tick. dir is the sign of the target (0 resets the assist).
StartAssistOutput startAssistTick(StartAssist &s, int8_t dir, bool motion_confirmed, bool speed_reached,
                                  uint32_t now_us, const StartAssistParams &p);
