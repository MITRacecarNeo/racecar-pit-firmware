#include <unity.h>

#include "config.h"
#include "dip_record.h"

namespace
{
  const DipParams P = {7.2f, 7.3f, 100, 1.0f};
  DipRecord r;
  uint32_t now;

  void feed(float volts, uint32_t ms, uint32_t step_ms = 10)
  {
    for (uint32_t t = 0; t < ms; t += step_ms)
    {
      dipTick(r, volts, now, P);
      now += step_ms;
    }
  }
}

void setUp()
{
  r = DipRecord{};
  now = 5000;
}

void tearDown() {}

void test_no_reading_until_a_valid_voltage()
{
  feed(0.0f, 100);
  TEST_ASSERT_FALSE(r.summary.any_reading);
  feed(8.1f, 10);
  TEST_ASSERT_TRUE(r.summary.any_reading);
  TEST_ASSERT_EQUAL_FLOAT(8.1f, r.summary.min_v);
}

void test_min_tracks_the_lowest_reading()
{
  feed(8.2f, 50);
  feed(7.6f, 10);
  feed(8.1f, 50);
  TEST_ASSERT_EQUAL_FLOAT(7.6f, r.summary.min_v);
  TEST_ASSERT_EQUAL_UINT16(0, r.summary.count);
}

void test_short_drop_is_not_a_dip()
{
  feed(8.0f, 50);
  feed(7.1f, 90);
  feed(8.0f, 50);
  TEST_ASSERT_EQUAL_UINT16(0, r.summary.count);
  TEST_ASSERT_EQUAL_FLOAT(7.1f, r.summary.min_v);
}

void test_dip_counts_after_the_minimum_time_and_records_its_start()
{
  feed(8.0f, 50);
  const uint32_t start = now;
  feed(7.1f, 110);
  TEST_ASSERT_EQUAL_UINT16(1, r.summary.count);
  TEST_ASSERT_EQUAL_UINT32(start, r.summary.last_dip_ms);
}

void test_one_long_sag_is_one_dip()
{
  feed(7.1f, 100000, 100);
  TEST_ASSERT_EQUAL_UINT16(1, r.summary.count);
}

void test_next_dip_needs_recovery_to_the_rearm_voltage()
{
  feed(7.1f, 200);
  feed(7.25f, 200); // above the dip level, below rearm
  feed(7.1f, 200);
  TEST_ASSERT_EQUAL_UINT16(1, r.summary.count);
  feed(7.35f, 20);
  feed(7.1f, 200);
  TEST_ASSERT_EQUAL_UINT16(2, r.summary.count);
}

void test_config_matches_the_info_keys()
{
  TEST_ASSERT_EQUAL_FLOAT(7.2f, cfg::INA226_SENSOR::dip.dip_v);
  TEST_ASSERT_EQUAL_UINT32(100, cfg::INA226_SENSOR::dip.min_ms);
  TEST_ASSERT_TRUE(cfg::INA226_SENSOR::dip.rearm_v > cfg::INA226_SENSOR::dip.dip_v);
  // 40 A across the shunt stays inside the INA226's 81.9 mV input range.
  TEST_ASSERT_TRUE(cfg::INA226_SENSOR::max_curr_expected * cfg::INA226_SENSOR::shunt_resistance <= 0.0819f);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_no_reading_until_a_valid_voltage);
  RUN_TEST(test_min_tracks_the_lowest_reading);
  RUN_TEST(test_short_drop_is_not_a_dip);
  RUN_TEST(test_dip_counts_after_the_minimum_time_and_records_its_start);
  RUN_TEST(test_one_long_sag_is_one_dip);
  RUN_TEST(test_next_dip_needs_recovery_to_the_rearm_voltage);
  RUN_TEST(test_config_matches_the_info_keys);
  return UNITY_END();
}
