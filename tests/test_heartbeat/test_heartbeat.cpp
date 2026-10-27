#include <unity.h>

#include "heartbeat.h"

void setUp(void) {}
void tearDown(void) {}

static const uint32_t A = 1u << 0, B = 1u << 1, C = 1u << 2;

static void test_no_refresh_until_every_context_beats(void)
{
    Heartbeat h(A | B | C);
    TEST_ASSERT_FALSE(h.collect());
    h.beat(A);
    h.beat(B);
    TEST_ASSERT_FALSE(h.allSeen());
    TEST_ASSERT_EQUAL_HEX32(C, h.missing());
    TEST_ASSERT_FALSE(h.collect());
    h.beat(C);
    TEST_ASSERT_TRUE(h.collect());
}

// A collect starts a new round: the same contexts must beat again.
static void test_collect_starts_a_new_round(void)
{
    Heartbeat h(A | B);
    h.beat(A);
    h.beat(B);
    TEST_ASSERT_TRUE(h.collect());
    TEST_ASSERT_FALSE(h.collect());
    TEST_ASSERT_EQUAL_HEX32(A | B, h.missing());
    h.beat(A);
    h.beat(B);
    TEST_ASSERT_TRUE(h.collect());
}

// The failure the watchdog exists for: one context stops, the others keep
// running, and no round ever completes.
static void test_a_stopped_context_blocks_every_later_round(void)
{
    Heartbeat h(A | B | C);
    h.beat(A);
    h.beat(B);
    h.beat(C);
    TEST_ASSERT_TRUE(h.collect());
    for (int i = 0; i < 100; i++)
    {
        h.beat(A);
        h.beat(C);
        TEST_ASSERT_FALSE(h.collect());
    }
    TEST_ASSERT_EQUAL_HEX32(B, h.missing());
}

static void test_repeated_beats_count_once(void)
{
    Heartbeat h(A | B);
    for (int i = 0; i < 10; i++)
        h.beat(A);
    TEST_ASSERT_FALSE(h.collect());
    h.beat(B);
    TEST_ASSERT_TRUE(h.collect());
}

static void test_bits_outside_the_required_set_are_ignored(void)
{
    Heartbeat h(A);
    h.beat(C);
    TEST_ASSERT_FALSE(h.collect());
    h.beat(A);
    TEST_ASSERT_TRUE(h.collect());
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_no_refresh_until_every_context_beats);
    RUN_TEST(test_collect_starts_a_new_round);
    RUN_TEST(test_a_stopped_context_blocks_every_later_round);
    RUN_TEST(test_repeated_beats_count_once);
    RUN_TEST(test_bits_outside_the_required_set_are_ignored);
    return UNITY_END();
}
