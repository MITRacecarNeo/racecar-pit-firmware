#include "stall_guard.h"

namespace
{
  bool stalling(const StallInputs &in, const StallParams &p)
  {
    const float speed = in.speed_mps < 0.0f ? -in.speed_mps : in.speed_mps;
    return in.command_active && speed < p.moving_mps && in.current_a > p.trip_a;
  }

  // Records a trip; true when it completes lockout_trips within the window.
  bool recordTrip(StallGuard &g, uint32_t now_us, const StallParams &p)
  {
    g.trips++;
    if (g.history == StallGuard::max_history)
    {
      for (uint8_t i = 1; i < StallGuard::max_history; i++)
        g.trip_times_us[i - 1] = g.trip_times_us[i];
      g.history--;
    }
    g.trip_times_us[g.history++] = now_us;

    const uint8_t need = p.lockout_trips < StallGuard::max_history ? p.lockout_trips : StallGuard::max_history;
    if (need == 0 || g.history < need)
      return false;
    const uint32_t oldest = g.trip_times_us[g.history - need];
    return (uint32_t)(now_us - oldest) <= p.lockout_window_us;
  }

  float secondsLeft(uint32_t elapsed_us, uint32_t total_us)
  {
    return elapsed_us >= total_us ? 0.0f : (float)(total_us - elapsed_us) * 1e-6f;
  }
}

StallOutput stallGuardTick(StallGuard &g, const StallInputs &in, uint32_t now_us, const StallParams &p)
{
  const uint32_t dt = g.started ? (uint32_t)(now_us - g.last_us) : 0;
  g.started = true;
  g.last_us = now_us;

  StallOutput out = {g.state, true, false, 0.0f, g.trips};

  switch (g.state)
  {
  case StallState::normal:
    if (stalling(in, p))
      g.stall_us += dt;
    else
      g.stall_us = g.stall_us > dt ? g.stall_us - dt : 0;

    if (g.stall_us >= p.trip_us)
    {
      g.stall_us = 0;
      g.state = StallState::tripped;
      out.tripped_now = true;
      // Counted at the trip; the countdown starts once the command is neutral.
      g.lockout_pending = recordTrip(g, now_us, p);
      if (g.lockout_pending)
        g.history = 0;
    }
    break;

  case StallState::tripped:
    if (!in.command_active)
    {
      g.state = g.lockout_pending ? StallState::lockout : StallState::cooldown;
      g.lockout_pending = false;
      g.countdown_start_us = now_us;
    }
    break;

  case StallState::cooldown:
  case StallState::lockout:
  {
    const uint32_t total = g.state == StallState::lockout ? p.lockout_us : p.cooldown_us;
    const uint32_t elapsed = (uint32_t)(now_us - g.countdown_start_us);
    if (elapsed >= total)
    {
      g.state = StallState::normal;
      g.stall_us = 0;
    }
    break;
  }
  }

  if (g.state == StallState::cooldown || g.state == StallState::lockout)
    out.seconds_left = secondsLeft((uint32_t)(now_us - g.countdown_start_us),
                                   g.state == StallState::lockout ? p.lockout_us : p.cooldown_us);

  out.state = g.state;
  out.force_neutral = g.state != StallState::normal;
  out.trips = g.trips;
  return out;
}
