#include <unity.h>

#include "config.h"
#include "led_patterns.h"

namespace
{
  const Rgb RED = {255, 0, 0};
  constexpr uint16_t N = 84;
  Rgb px[N];

  bool lit(const Rgb &c) { return c.r || c.g || c.b; }
  uint16_t litCount()
  {
    uint16_t k = 0;
    for (uint16_t i = 0; i < N; i++)
      k += lit(px[i]);
    return k;
  }
}

void setUp() {}
void tearDown() {}

void test_meter_fills_from_both_front_ends()
{
  batteryMeter(px, N, 10, RED); // 10% of 42 per side = 4
  TEST_ASSERT_EQUAL_UINT16(8, litCount());
  for (uint16_t i = 0; i < 4; i++)
  {
    TEST_ASSERT_TRUE(lit(px[i]));
    TEST_ASSERT_TRUE(lit(px[N - 1 - i]));
  }
  TEST_ASSERT_FALSE(lit(px[4]));
  TEST_ASSERT_FALSE(lit(px[N - 5]));
  TEST_ASSERT_FALSE(lit(px[41]));
  TEST_ASSERT_FALSE(lit(px[42]));
}

void test_meter_full_meets_at_the_back_middle()
{
  batteryMeter(px, N, 100, RED);
  TEST_ASSERT_EQUAL_UINT16(N, litCount());
  batteryMeter(px, N, 250, RED);
  TEST_ASSERT_EQUAL_UINT16(N, litCount());
  batteryMeter(px, N, 0, RED);
  TEST_ASSERT_EQUAL_UINT16(0, litCount());
}

void test_meter_matches_the_previous_rounding()
{
  // Previous firmware: half = soc * (N / 2) / 100, truncated.
  for (uint8_t soc = 0; soc <= 100; soc++)
  {
    batteryMeter(px, N, soc, RED);
    TEST_ASSERT_EQUAL_UINT16(2 * (soc * (N / 2) / 100), litCount());
  }
}

void test_loading_bar_fills_from_both_front_ends_over_the_duration()
{
  loadingBar(px, N, 0, 75000, RED);
  TEST_ASSERT_EQUAL_UINT16(0, litCount());
  loadingBar(px, N, 37500, 75000, RED); // half of 42 per side
  TEST_ASSERT_EQUAL_UINT16(42, litCount());
  for (uint16_t i = 0; i < 21; i++)
  {
    TEST_ASSERT_TRUE(lit(px[i]));
    TEST_ASSERT_TRUE(lit(px[N - 1 - i]));
  }
  TEST_ASSERT_FALSE(lit(px[21]));
  TEST_ASSERT_FALSE(lit(px[N - 22]));
  TEST_ASSERT_FALSE(lit(px[41]));
  TEST_ASSERT_FALSE(lit(px[42]));
  loadingBar(px, N, 74999, 75000, RED);
  TEST_ASSERT_EQUAL_UINT16(82, litCount()); // back middle pair last
}

void test_loading_bar_stays_full_after_the_duration()
{
  loadingBar(px, N, 75000, 75000, RED);
  TEST_ASSERT_EQUAL_UINT16(N, litCount());
  loadingBar(px, N, 170000, 75000, RED);
  TEST_ASSERT_EQUAL_UINT16(N, litCount());
}

void test_config_colors_and_timing()
{
  const Rgb &r = cfg::RGB_LED_WS2812B::bar_red;
  TEST_ASSERT_TRUE(r.r == 255 && r.g == 0 && r.b == 0);
  TEST_ASSERT_EQUAL_UINT32(60000, cfg::RGB_LED_WS2812B::loading_bar_ms);
  const Rgb &o = cfg::RGB_LED_WS2812B::loading_orange;
  TEST_ASSERT_TRUE(o.r == 255 && o.g == 40 && o.b == 0);
  // Stall and lockout flash match the driver's safety-override color.
  const Rgb &s = cfg::RGB_LED_WS2812B::stall_orange;
  TEST_ASSERT_TRUE(s.r == 255 && s.g == 110 && s.b == 0);
  TEST_ASSERT_TRUE(cfg::DISP::boot_timeout_ms > cfg::RGB_LED_WS2812B::loading_bar_ms);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_meter_fills_from_both_front_ends);
  RUN_TEST(test_meter_full_meets_at_the_back_middle);
  RUN_TEST(test_meter_matches_the_previous_rounding);
  RUN_TEST(test_loading_bar_fills_from_both_front_ends_over_the_duration);
  RUN_TEST(test_loading_bar_stays_full_after_the_duration);
  RUN_TEST(test_config_colors_and_timing);
  return UNITY_END();
}
