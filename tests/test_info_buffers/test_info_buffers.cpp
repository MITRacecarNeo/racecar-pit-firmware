#include <string.h>
#include <unity.h>

#include "info_buffers.h"

void setUp(void) {}
void tearDown(void) {}

using Buffers = InfoBuffers<32>;

static void fill(Buffers::Payload *p, const char *text, uint8_t revision)
{
    strcpy(p->data, text);
    p->len = strlen(text);
    p->revision = revision;
}

static void test_nothing_to_send_before_the_first_publish(void)
{
    Buffers b;
    TEST_ASSERT_NULL(b.take());
}

static void test_sender_gets_the_published_payload(void)
{
    Buffers b;
    Buffers::Payload *p = b.beginBuild();
    TEST_ASSERT_NOT_NULL(p);
    fill(p, "info=1\n", 0);
    b.publish();
    const Buffers::Payload *s = b.take();
    TEST_ASSERT_EQUAL_PTR(p, s);
    TEST_ASSERT_EQUAL_STRING("info=1\n", s->data);
}

// The builder may not overwrite a published buffer the sender has not taken.
static void test_builder_waits_until_the_sender_takes(void)
{
    Buffers b;
    fill(b.beginBuild(), "a", 0);
    b.publish();
    TEST_ASSERT_NULL(b.beginBuild());
    b.take();
    TEST_ASSERT_NOT_NULL(b.beginBuild());
}

// While the sender sends from one buffer, the builder writes the other.
static void test_builder_never_writes_the_buffer_being_sent(void)
{
    Buffers b;
    fill(b.beginBuild(), "first", 0);
    b.publish();
    const Buffers::Payload *sending = b.take();
    for (int round = 0; round < 5; round++)
    {
        Buffers::Payload *next = b.beginBuild();
        TEST_ASSERT_NOT_NULL(next);
        TEST_ASSERT_TRUE(next != sending);
        fill(next, round % 2 ? "odd" : "even", (uint8_t)round);
        b.publish();
        TEST_ASSERT_NULL(b.beginBuild()); // not taken yet
        TEST_ASSERT_EQUAL_STRING(round == 0 ? "first" : (round % 2 ? "even" : "odd"), sending->data);
        sending = b.take();
        TEST_ASSERT_EQUAL_STRING(round % 2 ? "odd" : "even", sending->data);
    }
}

// A sender that starts several cycles without a new publish keeps the same
// payload, and the builder is free to write the other buffer.
static void test_repeated_takes_return_the_same_payload(void)
{
    Buffers b;
    fill(b.beginBuild(), "same", 4);
    b.publish();
    const Buffers::Payload *a = b.take();
    const Buffers::Payload *c = b.take();
    TEST_ASSERT_EQUAL_PTR(a, c);
    TEST_ASSERT_EQUAL_UINT8(4, c->revision);
    TEST_ASSERT_TRUE(b.beginBuild() != a);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_nothing_to_send_before_the_first_publish);
    RUN_TEST(test_sender_gets_the_published_payload);
    RUN_TEST(test_builder_waits_until_the_sender_takes);
    RUN_TEST(test_builder_never_writes_the_buffer_being_sent);
    RUN_TEST(test_repeated_takes_return_the_same_payload);
    return UNITY_END();
}
