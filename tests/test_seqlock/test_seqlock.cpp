#include <unity.h>

#include "seqlock.h"

void setUp(void) {}
void tearDown(void) {}

struct Pair
{
    float a;
    float b;
};

static void test_read_returns_the_written_value(void)
{
    Seqlock<Pair> lock;
    lock.write({0.25f, -0.5f});
    Pair out = {9.0f, 9.0f};
    TEST_ASSERT_TRUE(lock.tryRead(out));
    TEST_ASSERT_EQUAL_FLOAT(0.25f, out.a);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, out.b);
}

static void test_initial_value_is_zero(void)
{
    Seqlock<Pair> lock;
    Pair out = {9.0f, 9.0f};
    TEST_ASSERT_TRUE(lock.tryRead(out));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.a);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.b);
}

// What the drive interrupt sees when it fires in the middle of a write: the
// read fails and the caller keeps its previous copy untouched.
static void test_read_during_a_write_fails_and_keeps_out(void)
{
    Seqlock<Pair> lock;
    lock.write({1.0f, 2.0f});
    lock.beginWrite();
    lock.data().a = 3.0f; // half written
    Pair out = {1.0f, 2.0f};
    TEST_ASSERT_FALSE(lock.tryRead(out));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out.a);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, out.b);
    lock.data().b = 4.0f;
    lock.endWrite();
    TEST_ASSERT_TRUE(lock.tryRead(out));
    TEST_ASSERT_EQUAL_FLOAT(3.0f, out.a);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, out.b);
}

static void test_sequence_is_odd_only_while_writing(void)
{
    Seqlock<Pair> lock;
    TEST_ASSERT_EQUAL_UINT32(0, lock.sequence() & 1u);
    lock.beginWrite();
    TEST_ASSERT_EQUAL_UINT32(1, lock.sequence() & 1u);
    lock.endWrite();
    TEST_ASSERT_EQUAL_UINT32(0, lock.sequence() & 1u);
    lock.write({0.0f, 0.0f});
    TEST_ASSERT_EQUAL_UINT32(4, lock.sequence());
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_read_returns_the_written_value);
    RUN_TEST(test_initial_value_is_zero);
    RUN_TEST(test_read_during_a_write_fails_and_keeps_out);
    RUN_TEST(test_sequence_is_odd_only_while_writing);
    return UNITY_END();
}
