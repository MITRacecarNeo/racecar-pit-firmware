#include <unity.h>

#include "config.h"
#include "led_patterns.h"

namespace
{
  const LedGradient G = {{255, 93, 19}, {255, 6, 22}};
  constexpr uint16_t N = 84;
  Rgb px[N];

  bool lit(const Rgb &c) { return c.r || c.g || c.b; }
  bool same(const Rgb &a, const Rgb &b) { return a.r == b.r && a.g == b.g && a.b == b.b; }
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

void test_gradient_runs_inner_to_outer()
{
  TEST_ASSERT_TRUE(same(G.inner, gradientAt(0, 42, G)));
  TEST_ASSERT_TRUE(same(G.outer, gradientAt(41, 42, G)));
  TEST_ASSERT_TRUE(same(G.outer, gradientAt(99, 42, G)));
  const Rgb mid = gradientAt(20, 42, G);
  TEST_ASSERT_TRUE(mid.g < G.inner.g && mid.g > G.outer.g);
}

void test_meter_grows_from_the_back_middle()
{
  batteryMeter(px, N, 10, G); // 4 of 42 per side, rounded
  TEST_ASSERT_EQUAL_UINT16(8, litCount());
  for (uint16_t i = 38; i <= 45; i++)
    TEST_ASSERT_TRUE(lit(px[i]));
  TEST_ASSERT_FALSE(lit(px[37]));
  TEST_ASSERT_FALSE(lit(px[46]));
  TEST_ASSERT_TRUE(same(G.inner, px[41]));
  TEST_ASSERT_TRUE(same(G.inner, px[42]));
}

void test_meter_full_and_empty()
{
  batteryMeter(px, N, 100, G);
  TEST_ASSERT_EQUAL_UINT16(N, litCount());
  TEST_ASSERT_TRUE(same(G.outer, px[0]));
  TEST_ASSERT_TRUE(same(G.outer, px[N - 1]));
  batteryMeter(px, N, 0, G);
  TEST_ASSERT_EQUAL_UINT16(0, litCount());
  batteryMeter(px, N, 250, G);
  TEST_ASSERT_EQUAL_UINT16(N, litCount());
}

void test_meter_is_symmetric()
{
  batteryMeter(px, N, 63, G);
  for (uint16_t i = 0; i < N / 2; i++)
    TEST_ASSERT_TRUE(same(px[i], px[N - 1 - i]));
}

void test_loading_lights_the_whole_band_symmetrically()
{
  loadingScroll(px, N, 12345, 30, G);
  TEST_ASSERT_EQUAL_UINT16(N, litCount());
  for (uint16_t i = 0; i < N / 2; i++)
    TEST_ASSERT_TRUE(same(px[i], px[N - 1 - i]));
}

void test_loading_moves_outward_one_led_per_step()
{
  Rgb before[N];
  loadingScroll(before, N, 0, 30, G);
  loadingScroll(px, N, 30, 30, G);
  // The color next to the back middle moves one LED outward each step.
  for (uint16_t d = 0; d + 1 < N / 2; d++)
    TEST_ASSERT_TRUE(same(before[N / 2 + d], px[N / 2 + d + 1]));
}

void test_loading_repeats_after_a_full_period()
{
  Rgb a[N];
  loadingScroll(a, N, 0, 30, G);
  loadingScroll(px, N, 30 * N, 30, G); // period is 2 * half = N steps
  for (uint16_t i = 0; i < N; i++)
    TEST_ASSERT_TRUE(same(a[i], px[i]));
}

void test_config_gradient_is_orange_inside_crimson_outside()
{
  const LedGradient &g = cfg::RGB_LED_WS2812B::gradient;
  TEST_ASSERT_TRUE(g.inner.g > g.outer.g);
  TEST_ASSERT_EQUAL_UINT16(84, cfg::RGB_LED_WS2812B::num_leds);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_gradient_runs_inner_to_outer);
  RUN_TEST(test_meter_grows_from_the_back_middle);
  RUN_TEST(test_meter_full_and_empty);
  RUN_TEST(test_meter_is_symmetric);
  RUN_TEST(test_loading_lights_the_whole_band_symmetrically);
  RUN_TEST(test_loading_moves_outward_one_led_per_step);
  RUN_TEST(test_loading_repeats_after_a_full_period);
  RUN_TEST(test_config_gradient_is_orange_inside_crimson_outside);
  return UNITY_END();
}
