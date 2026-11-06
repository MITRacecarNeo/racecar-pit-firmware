#include <unity.h>

#include "config.h"
#include "stall_guard.h"

namespace
{
  const StallParams P = {10.0f, 0.1f, 500000, 2000000, 30000000, 3, 60000000};
  const StallInputs STALL = {12.0f, 0.0f, true};
  const StallInputs CLEAR = {2.0f, 0.0f, true};
  const StallInputs NEUTRAL = {2.0f, 0.0f, false};
  constexpr uint32_t MS = 1000;

  StallGuard g;
  uint32_t now;

  StallOutput run(const StallInputs &in, uint32_t ms, uint32_t step_ms = 1)
  {
    StallOutput out = {};
    for (uint32_t t = 0; t < ms; t += step_ms)
    {
      now += step_ms * MS;
      out = stallGuardTick(g, in, now, P);
    }
    return out;
  }

  // Stall until the trip, then release the command.
  StallOutput tripAndRelease()
  {
    run(STALL, 500);
    return run(NEUTRAL, 1);
  }
}

void setUp()
{
  g = StallGuard{};
  now = 0;
  stallGuardTick(g, NEUTRAL, now, P);
}

void tearDown() {}

void test_trips_after_trip_time_of_stall()
{
  StallOutput out = run(STALL, 499);
  TEST_ASSERT_EQUAL(StallState::normal, out.state);
  TEST_ASSERT_FALSE(out.force_neutral);
  out = run(STALL, 1);
  TEST_ASSERT_EQUAL(StallState::tripped, out.state);
  TEST_ASSERT_TRUE(out.tripped_now);
  TEST_ASSERT_TRUE(out.force_neutral);
  TEST_ASSERT_EQUAL_UINT16(1, out.trips);
}

void test_each_condition_alone_does_not_trip()
{
  TEST_ASSERT_EQUAL(StallState::normal, run({12.0f, 0.5f, true}, 2000).state);   // moving
  TEST_ASSERT_EQUAL(StallState::normal, run({12.0f, -0.5f, true}, 2000).state);  // moving in reverse
  TEST_ASSERT_EQUAL(StallState::normal, run({12.0f, 0.0f, false}, 2000).state);  // no command
  TEST_ASSERT_EQUAL(StallState::normal, run({9.9f, 0.0f, true}, 2000).state);    // below the current limit
}

void test_short_start_peaks_do_not_add_up()
{
  // 0.19 s peaks, then normal current for as long.
  for (int i = 0; i < 20; i++)
  {
    run(STALL, 190);
    run(CLEAR, 190);
  }
  TEST_ASSERT_EQUAL(StallState::normal, run(CLEAR, 1).state);
}

void test_stall_broken_by_assist_rests_still_trips()
{
  // Pushes broken by 0.3 s rests at neutral; the rest only takes back 0.3 s.
  run(STALL, 400);
  run(NEUTRAL, 300);
  TEST_ASSERT_EQUAL(StallState::normal, run(STALL, 1).state);
  TEST_ASSERT_EQUAL(StallState::tripped, run(STALL, 399).state);
}

void test_tripped_holds_until_the_command_is_neutral()
{
  run(STALL, 500);
  TEST_ASSERT_EQUAL(StallState::tripped, run(CLEAR, 10000).state);
  StallOutput out = run(NEUTRAL, 1);
  TEST_ASSERT_EQUAL(StallState::cooldown, out.state);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, out.seconds_left);
}

void test_cooldown_counts_down_then_resumes()
{
  tripAndRelease();
  StallOutput out = run(CLEAR, 1000);
  TEST_ASSERT_EQUAL(StallState::cooldown, out.state);
  TEST_ASSERT_TRUE(out.force_neutral);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 1.0f, out.seconds_left);
  out = run(CLEAR, 1000);
  TEST_ASSERT_EQUAL(StallState::normal, out.state);
  TEST_ASSERT_FALSE(out.force_neutral);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, out.seconds_left);
}

void test_third_trip_in_the_window_locks_out()
{
  tripAndRelease();
  run(NEUTRAL, 2000);
  tripAndRelease();
  run(NEUTRAL, 2000);
  StallOutput out = tripAndRelease();
  TEST_ASSERT_EQUAL(StallState::lockout, out.state);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, out.seconds_left);
  TEST_ASSERT_EQUAL_UINT16(3, out.trips);
  out = run(NEUTRAL, 29999, 1);
  TEST_ASSERT_EQUAL(StallState::lockout, out.state);
  out = run(NEUTRAL, 1);
  TEST_ASSERT_EQUAL(StallState::normal, out.state);
}

void test_lockout_clears_the_trip_history()
{
  for (int i = 0; i < 3; i++)
  {
    tripAndRelease();
    run(NEUTRAL, 30000, 10);
  }
  TEST_ASSERT_EQUAL(StallState::normal, run(NEUTRAL, 1).state);
  // The next trip starts a new count: cooldown, not lockout.
  TEST_ASSERT_EQUAL(StallState::cooldown, tripAndRelease().state);
  TEST_ASSERT_EQUAL_UINT16(4, g.trips);
}

void test_trips_spread_past_the_window_cool_down()
{
  for (int i = 0; i < 4; i++)
  {
    StallOutput out = tripAndRelease();
    TEST_ASSERT_EQUAL(StallState::cooldown, out.state);
    run(NEUTRAL, 31000, 10); // trips 31.5 s apart: never 3 within 60 s
  }
}

void test_microsecond_clock_wrap()
{
  g = StallGuard{};
  now = 0xFFFFFFFFu - 200 * MS;
  stallGuardTick(g, NEUTRAL, now, P);
  TEST_ASSERT_EQUAL(StallState::tripped, run(STALL, 500).state);
  TEST_ASSERT_EQUAL(StallState::cooldown, run(NEUTRAL, 1).state);
  TEST_ASSERT_EQUAL(StallState::normal, run(NEUTRAL, 2000).state);
}

void test_config_params_match_the_approved_states()
{
  const StallParams &c = cfg::STALL_GUARD::params;
  TEST_ASSERT_EQUAL_UINT32(2000000, c.cooldown_us);
  TEST_ASSERT_EQUAL_UINT32(30000000, c.lockout_us);
  TEST_ASSERT_EQUAL_UINT8(3, c.lockout_trips);
  TEST_ASSERT_EQUAL_UINT32(60000000, c.lockout_window_us);
  TEST_ASSERT_TRUE(c.lockout_trips <= StallGuard::max_history);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_trips_after_trip_time_of_stall);
  RUN_TEST(test_each_condition_alone_does_not_trip);
  RUN_TEST(test_short_start_peaks_do_not_add_up);
  RUN_TEST(test_stall_broken_by_assist_rests_still_trips);
  RUN_TEST(test_tripped_holds_until_the_command_is_neutral);
  RUN_TEST(test_cooldown_counts_down_then_resumes);
  RUN_TEST(test_third_trip_in_the_window_locks_out);
  RUN_TEST(test_lockout_clears_the_trip_history);
  RUN_TEST(test_trips_spread_past_the_window_cool_down);
  RUN_TEST(test_microsecond_clock_wrap);
  RUN_TEST(test_config_params_match_the_approved_states);
  return UNITY_END();
}
