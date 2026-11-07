#pragma once
#include <stdint.h>

// Stall protection. Arduino-free, unit tested under tests/.
//
// A stall is throttle commanded, the wheels not moving, and battery current
// above `trip_a`. Time in that condition adds to a timer and time out of it
// takes away at the same rate, so a stall broken up by start-assist rests
// still adds up. The guard trips when the timer reaches `trip_us`.
//
// Tripped: output neutral until the command returns to neutral, then a
// cooldown of `cooldown_us`. The `lockout_trips`-th trip within
// `lockout_window_us` gives a lockout of `lockout_us` in place of the
// cooldown. Driving resumes only from normal.

enum class StallState : uint8_t
{
  normal = 0,
  tripped = 1,  // neutral until the command is neutral
  cooldown = 2, // neutral for cooldown_us
  lockout = 3,  // neutral for lockout_us
};

struct StallParams
{
  float trip_a;                // battery current above this counts toward a trip
  float moving_mps;            // |speed| at or above this is moving
  uint32_t trip_us;            // stall time that trips
  uint32_t cooldown_us;        // after a trip, once the command is neutral
  uint32_t lockout_us;         // in place of the cooldown after repeated trips
  uint8_t lockout_trips;       // trips within the window that give a lockout, at most 8
  uint32_t lockout_window_us;
};

struct StallInputs
{
  float current_a;     // battery current
  float speed_mps;     // measured wheel speed
  bool command_active; // throttle commanded away from neutral (Pi or RC)
};

struct StallGuard
{
  static constexpr uint8_t max_history = 8;
  StallState state = StallState::normal;
  uint32_t stall_us = 0;        // accumulated stall time
  uint32_t last_us = 0;         // previous tick
  bool started = false;
  bool lockout_pending = false;  // the current trip ends in a lockout
  uint32_t countdown_start_us = 0;
  uint32_t trip_times_us[max_history] = {};
  uint8_t history = 0;          // valid entries in trip_times_us, newest last
  uint16_t trips = 0;           // since boot
};

struct StallOutput
{
  StallState state;
  bool force_neutral;  // write neutral this tick
  bool tripped_now;    // a trip happened on this tick
  float seconds_left;  // cooldown or lockout time left, else 0
  uint16_t trips;
};

StallOutput stallGuardTick(StallGuard &g, const StallInputs &in, uint32_t now_us, const StallParams &p);

// The guard's output in one 32-bit word, so the drive interrupt can publish it
// with a single store: bits 0-3 state, 4-15 hundredths of a second left
// (clamped at 40.95 s), 16-31 trips since boot.
uint32_t packStallWord(const StallOutput &out);

struct StallWord
{
  StallState state;
  float seconds_left;
  uint16_t trips;
};
StallWord unpackStallWord(uint32_t w);
