#include <string.h>
#include <unity.h>

#include "config.h"
#include "display_state.h"

namespace
{
  const DisplayParams P = {1000, 2000, 250};
  constexpr uint32_t BOOT_TIMEOUT = 180000;

  DisplayInputs live()
  {
    return {LinkState::live, false, false, false, StallState::normal, 0, false};
  }

  DisplayInputs tripped(StallState s, uint32_t ms_since_trip, LinkState link = LinkState::live)
  {
    DisplayInputs in = live();
    in.link = link;
    in.stall = s;
    in.ms_since_trip = ms_since_trip;
    in.tripped_once = true;
    return in;
  }

  void assertText(const char *expected, const DisplayChoice &c)
  {
    TEST_ASSERT_EQUAL(MatrixShow::text, c.matrix);
    TEST_ASSERT_EQUAL_STRING(expected, c.text);
  }
}

void setUp() {}
void tearDown() {}

void test_link_waits_for_boot_then_goes_live()
{
  LinkTracker t;
  TEST_ASSERT_EQUAL(LinkState::boot_wait, linkTick(t, false, 5000, BOOT_TIMEOUT));
  TEST_ASSERT_EQUAL(LinkState::boot_wait, linkTick(t, false, 5000 + 90000, BOOT_TIMEOUT));
  TEST_ASSERT_EQUAL(LinkState::live, linkTick(t, true, 5000 + 91000, BOOT_TIMEOUT));
}

void test_link_lost_when_commands_stop_and_back_when_they_resume()
{
  LinkTracker t;
  linkTick(t, true, 0, BOOT_TIMEOUT);
  TEST_ASSERT_EQUAL(LinkState::lost, linkTick(t, false, 10, BOOT_TIMEOUT));
  TEST_ASSERT_EQUAL(LinkState::lost, linkTick(t, false, 500000, BOOT_TIMEOUT));
  TEST_ASSERT_EQUAL(LinkState::live, linkTick(t, true, 500010, BOOT_TIMEOUT));
}

void test_boot_wait_times_out_to_lost()
{
  LinkTracker t;
  linkTick(t, false, 1000, BOOT_TIMEOUT);
  TEST_ASSERT_EQUAL(LinkState::boot_wait, linkTick(t, false, 1000 + BOOT_TIMEOUT - 1, BOOT_TIMEOUT));
  TEST_ASSERT_EQUAL(LinkState::lost, linkTick(t, false, 1000 + BOOT_TIMEOUT, BOOT_TIMEOUT));
  TEST_ASSERT_EQUAL(LinkState::live, linkTick(t, true, 1000 + BOOT_TIMEOUT + 5000, BOOT_TIMEOUT));
}

void test_boot_wait_shows_the_logo_and_the_loading_bar()
{
  DisplayInputs in = live();
  in.link = LinkState::boot_wait;
  const DisplayChoice c = chooseDisplay(in, P);
  TEST_ASSERT_EQUAL(MatrixShow::logo, c.matrix);
  TEST_ASSERT_EQUAL(StripShow::loading, c.strip);
}

void test_lost_shows_off_with_the_strip_dark()
{
  DisplayInputs in = live();
  in.link = LinkState::lost;
  in.matrix_flag = in.strip_flag = true; // stale flags from before the loss
  const DisplayChoice c = chooseDisplay(in, P);
  assertText("OFF", c);
  TEST_ASSERT_EQUAL(StripShow::off, c.strip);
}

void test_live_defaults_are_the_logo_and_battery_meter()
{
  const DisplayChoice c = chooseDisplay(live(), P);
  TEST_ASSERT_EQUAL(MatrixShow::logo, c.matrix);
  TEST_ASSERT_EQUAL(StripShow::battery, c.strip);
}

void test_live_shows_pi_content_when_flagged()
{
  DisplayInputs in = live();
  in.matrix_flag = in.strip_flag = true;
  DisplayChoice c = chooseDisplay(in, P);
  TEST_ASSERT_EQUAL(MatrixShow::pi_bitmap, c.matrix);
  TEST_ASSERT_EQUAL(StripShow::pi_pixels, c.strip);
  in.driver_starting = true;
  TEST_ASSERT_EQUAL(StripShow::driver_start, chooseDisplay(in, P).strip);
}

void test_stall_words_one_second_each_from_the_trip()
{
  assertText("WARN", chooseDisplay(tripped(StallState::tripped, 0), P));
  assertText("WARN", chooseDisplay(tripped(StallState::tripped, 999), P));
  assertText("HIGH", chooseDisplay(tripped(StallState::cooldown, 1000), P));
  assertText("CURR", chooseDisplay(tripped(StallState::cooldown, 2500), P));
  assertText("WARN", chooseDisplay(tripped(StallState::tripped, 3000), P));
}

void test_lockout_shows_lock_and_flashes_until_it_ends()
{
  for (uint32_t ms = 0; ms < 32000; ms += 500)
  {
    const DisplayChoice c = chooseDisplay(tripped(StallState::lockout, ms), P);
    assertText("LOCK", c);
    TEST_ASSERT_EQUAL(StripShow::stall_flash, c.strip); // 500 ms steps land on the lit half
  }
  TEST_ASSERT_EQUAL(StripShow::stall_dark, chooseDisplay(tripped(StallState::lockout, 25250), P).strip);
  assertText("LOCK", chooseDisplay(tripped(StallState::lockout, 25250, LinkState::lost), P));
  TEST_ASSERT_EQUAL(StripShow::battery, chooseDisplay(tripped(StallState::normal, 32000), P).strip);
}

void test_stall_words_override_pi_content_and_off()
{
  DisplayInputs in = tripped(StallState::cooldown, 1000);
  in.matrix_flag = true;
  assertText("HIGH", chooseDisplay(in, P));
  assertText("HIGH", chooseDisplay(tripped(StallState::cooldown, 1000, LinkState::lost), P));
}

void test_strip_flashes_orange_for_two_seconds_after_a_trip()
{
  TEST_ASSERT_EQUAL(StripShow::stall_flash, chooseDisplay(tripped(StallState::tripped, 0), P).strip);
  TEST_ASSERT_EQUAL(StripShow::stall_dark, chooseDisplay(tripped(StallState::tripped, 250), P).strip);
  TEST_ASSERT_EQUAL(StripShow::stall_flash, chooseDisplay(tripped(StallState::cooldown, 1500), P).strip);
  TEST_ASSERT_EQUAL(StripShow::stall_dark, chooseDisplay(tripped(StallState::cooldown, 1999), P).strip);
  // After the flash the strip goes back to what the link state shows.
  TEST_ASSERT_EQUAL(StripShow::battery, chooseDisplay(tripped(StallState::cooldown, 2000), P).strip);
  TEST_ASSERT_EQUAL(StripShow::off,
                    chooseDisplay(tripped(StallState::cooldown, 2000, LinkState::lost), P).strip);
}

void test_normal_after_the_flash_is_the_plain_display()
{
  const DisplayChoice c = chooseDisplay(tripped(StallState::normal, 2000), P);
  TEST_ASSERT_EQUAL(MatrixShow::logo, c.matrix);
  TEST_ASSERT_EQUAL(StripShow::battery, c.strip);
}

void test_rx_time_expires_before_the_clock_wraps()
{
  const uint32_t max_age = 60000000;
  TEST_ASSERT_EQUAL_UINT32(0, expireRxTime(0, 123, max_age));
  TEST_ASSERT_EQUAL_UINT32(1000, expireRxTime(1000, 1000 + max_age, max_age));
  TEST_ASSERT_EQUAL_UINT32(0, expireRxTime(1000, 1001 + max_age, max_age));
  // Across the 32-bit wrap.
  TEST_ASSERT_EQUAL_UINT32(0xFFFFFF00u, expireRxTime(0xFFFFFF00u, 0x100u, max_age));
  TEST_ASSERT_EQUAL_UINT32(0, expireRxTime(0xFFFFFF00u, max_age + 0x101u, max_age));
}

void test_config_values()
{
  TEST_ASSERT_EQUAL_UINT32(1000, cfg::DISP::stall_display.word_ms);
  TEST_ASSERT_EQUAL_UINT32(2000, cfg::DISP::stall_display.flash_ms);
  TEST_ASSERT_TRUE(cfg::DISP::boot_timeout_ms > 90000);
  TEST_ASSERT_TRUE(cfg::DISP::rx_expire_us < 4294967295u - cfg::DISP::cmd_timeout_ms * 1000u);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_link_waits_for_boot_then_goes_live);
  RUN_TEST(test_link_lost_when_commands_stop_and_back_when_they_resume);
  RUN_TEST(test_boot_wait_times_out_to_lost);
  RUN_TEST(test_boot_wait_shows_the_logo_and_the_loading_bar);
  RUN_TEST(test_lost_shows_off_with_the_strip_dark);
  RUN_TEST(test_live_defaults_are_the_logo_and_battery_meter);
  RUN_TEST(test_live_shows_pi_content_when_flagged);
  RUN_TEST(test_stall_words_one_second_each_from_the_trip);
  RUN_TEST(test_stall_words_override_pi_content_and_off);
  RUN_TEST(test_lockout_shows_lock_and_flashes_until_it_ends);
  RUN_TEST(test_strip_flashes_orange_for_two_seconds_after_a_trip);
  RUN_TEST(test_normal_after_the_flash_is_the_plain_display);
  RUN_TEST(test_rx_time_expires_before_the_clock_wraps);
  RUN_TEST(test_config_values);
  return UNITY_END();
}
