#include <string.h>
#include <unity.h>

#include "prof_stats.h"

void setUp(void) {}
void tearDown(void) {}

static PeriodStat period;

static void test_period_counts_sum_and_max(void)
{
    period.clear();
    period.add(1000);
    period.add(1500);
    period.add(900);
    TEST_ASSERT_EQUAL_UINT32(3, period.n);
    TEST_ASSERT_EQUAL_UINT32(3400, period.sum_us);
    TEST_ASSERT_EQUAL_UINT32(1500, period.max_us);
}

// Quantiles report the upper edge of the bin, so they never understate.
static void test_period_quantiles_use_upper_bin_edges(void)
{
    period.clear();
    for (int i = 0; i < 99; i++)
        period.add(1010); // bin 20: 1000-1049 us
    period.add(6020);     // bin 120: 6000-6049 us
    TEST_ASSERT_EQUAL_UINT32(1050, period.quantile(0.5f));
    TEST_ASSERT_EQUAL_UINT32(1050, period.quantile(0.99f));
    TEST_ASSERT_EQUAL_UINT32(6050, period.quantile(1.0f));
}

static void test_period_quantile_of_empty_is_zero(void)
{
    period.clear();
    TEST_ASSERT_EQUAL_UINT32(0, period.quantile(0.5f));
}

// A stall longer than the histogram lands in the last bin; max keeps the
// real value.
static void test_period_overflow_goes_to_the_last_bin(void)
{
    period.clear();
    period.add(50000);
    TEST_ASSERT_EQUAL_UINT16(1, period.hist[prof::bins - 1]);
    TEST_ASSERT_EQUAL_UINT32(50000, period.max_us);
    TEST_ASSERT_EQUAL_UINT32(prof::bins * prof::bin_us, period.quantile(0.5f));
}

static void test_section_stat(void)
{
    SectionStat s;
    s.clear();
    s.add(30);
    s.add(10);
    TEST_ASSERT_EQUAL_UINT32(2, s.n);
    TEST_ASSERT_EQUAL_UINT32(40, s.sum_us);
    TEST_ASSERT_EQUAL_UINT32(30, s.max_us);
}

static void test_line_formats(void)
{
    char buf[96];
    period.clear();
    period.add(1000);
    period.add(1000);
    TEST_ASSERT_EQUAL_UINT32(strlen("P,5000,period,drive_loop,2,2000,1000,1050,1050"),
                             formatPeriod(buf, sizeof(buf), 5000, "drive_loop", period));
    TEST_ASSERT_EQUAL_STRING("P,5000,period,drive_loop,2,2000,1000,1050,1050", buf);

    SectionStat s;
    s.clear();
    s.add(7);
    formatSection(buf, sizeof(buf), 5000, "imu_read", s);
    TEST_ASSERT_EQUAL_STRING("P,5000,section,imu_read,1,7,7", buf);

    formatCounter(buf, sizeof(buf), 5000, "tx_frames", 152);
    TEST_ASSERT_EQUAL_STRING("P,5000,counter,tx_frames,152", buf);
}

static void test_line_too_long_returns_zero(void)
{
    char buf[8];
    TEST_ASSERT_EQUAL_UINT32(0, formatCounter(buf, sizeof(buf), 5000, "tx_frames", 152));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_period_counts_sum_and_max);
    RUN_TEST(test_period_quantiles_use_upper_bin_edges);
    RUN_TEST(test_period_quantile_of_empty_is_zero);
    RUN_TEST(test_period_overflow_goes_to_the_last_bin);
    RUN_TEST(test_section_stat);
    RUN_TEST(test_line_formats);
    RUN_TEST(test_line_too_long_returns_zero);
    return UNITY_END();
}
