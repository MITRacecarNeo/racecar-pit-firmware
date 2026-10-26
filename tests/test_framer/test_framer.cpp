#include <string.h>
#include <unity.h>

#include "command_framer.h"
#include "crc16.h"
#include "packets.h"

void setUp(void) {}
void tearDown(void) {}

static const size_t WIRE = 4 + sizeof(RxPacket);

// One command frame as the Pi sends it: magic, body, CRC over both.
static void buildFrame(uint8_t *out, float servo, float motor, uint8_t state)
{
    const uint32_t magic = wire::command_magic;
    RxPacket body;
    memset(&body, 0, sizeof(body));
    body.header.VersionNum = wire::protocol_version;
    body.header.PacketBytes = sizeof(RxPacket);
    body.header.PacketsPerFrame = 1;
    body.ServoAng = servo;
    body.MotorSpeed = motor;
    body.SystemState = state;
    memcpy(out, &magic, 4);
    memcpy(out + 4, &body, sizeof(body));
    body.Checksum = crc16_ccitt(out, WIRE - 2);
    memcpy(out + 4, &body, sizeof(body));
}

// Pushes bytes; returns how many frames completed with a good CRC.
static int pushAll(CommandFramer &f, const uint8_t *data, size_t len)
{
    int good = 0;
    for (size_t i = 0; i < len; i++)
        if (f.push(data[i]) && f.crcOk())
            good++;
    return good;
}

static void test_one_frame(void)
{
    uint8_t frame[WIRE];
    buildFrame(frame, 0.25f, -0.5f, 0x21);
    CommandFramer f;
    TEST_ASSERT_EQUAL_INT(1, pushAll(f, frame, sizeof(frame)));
    TEST_ASSERT_EQUAL_FLOAT(0.25f, f.frame().ServoAng);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, f.frame().MotorSpeed);
    TEST_ASSERT_EQUAL_HEX8(0x21, f.frame().SystemState);
    TEST_ASSERT_FALSE(f.inFrame());
}

// The receive thread sees a frame in arbitrary pieces across its turns.
static void test_frame_split_across_turns(void)
{
    uint8_t frame[WIRE];
    buildFrame(frame, 1.0f, 0.1f, 0);
    CommandFramer f;
    const size_t cuts[] = {1, 3, 2, 60, 200, 100, 108};
    size_t at = 0;
    int good = 0;
    for (size_t c : cuts)
    {
        good += pushAll(f, frame + at, c);
        at += c;
        if (at < sizeof(frame))
            TEST_ASSERT_TRUE(f.inFrame() || at < 4);
    }
    TEST_ASSERT_EQUAL_UINT32(sizeof(frame), at);
    TEST_ASSERT_EQUAL_INT(1, good);
}

static void test_garbage_and_partial_magic_before_a_frame(void)
{
    uint8_t stream[8 + WIRE];
    const uint8_t junk[8] = {0x00, 0xAD, 0xDE, 0xEF, 0x11, 0xAD, 0xDE, 0x22};
    memcpy(stream, junk, sizeof(junk));
    buildFrame(stream + sizeof(junk), -1.0f, 0.0f, 0);
    CommandFramer f;
    TEST_ASSERT_EQUAL_INT(1, pushAll(f, stream, sizeof(stream)));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, f.frame().ServoAng);
}

static void test_back_to_back_frames(void)
{
    uint8_t stream[2 * WIRE];
    buildFrame(stream, 0.1f, 0.2f, 0);
    buildFrame(stream + WIRE, 0.3f, 0.4f, 0);
    CommandFramer f;
    TEST_ASSERT_EQUAL_INT(2, pushAll(f, stream, sizeof(stream)));
    TEST_ASSERT_EQUAL_FLOAT(0.3f, f.frame().ServoAng);
}

static void test_corrupt_frame_fails_crc_and_the_next_one_passes(void)
{
    uint8_t stream[2 * WIRE];
    buildFrame(stream, 0.1f, 0.2f, 0);
    buildFrame(stream + WIRE, 0.3f, 0.4f, 0);
    stream[100] ^= 0x04;
    CommandFramer f;
    int complete = 0, good = 0;
    for (size_t i = 0; i < sizeof(stream); i++)
        if (f.push(stream[i]))
        {
            complete++;
            good += f.crcOk() ? 1 : 0;
        }
    TEST_ASSERT_EQUAL_INT(2, complete);
    TEST_ASSERT_EQUAL_INT(1, good);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, f.frame().ServoAng);
}

// Body bytes are never searched for the magic, as with the old blocking read.
static void test_magic_inside_a_body_does_not_resync(void)
{
    uint8_t frame[WIRE];
    buildFrame(frame, 0.0f, 0.0f, 0);
    const uint32_t magic = wire::command_magic;
    memcpy(frame + 40, &magic, 4); // inside the dot-matrix bytes
    RxPacket body;
    memcpy(&body, frame + 4, sizeof(body));
    body.Checksum = 0;
    memcpy(frame + 4, &body, sizeof(body));
    body.Checksum = crc16_ccitt(frame, WIRE - 2);
    memcpy(frame + 4, &body, sizeof(body));
    CommandFramer f;
    TEST_ASSERT_EQUAL_INT(1, pushAll(f, frame, sizeof(frame)));
}

// A stalled frame is dropped by the caller; the next frame still lands.
static void test_reset_drops_a_partial_frame(void)
{
    uint8_t stream[2 * WIRE];
    buildFrame(stream, 0.1f, 0.2f, 0);
    buildFrame(stream + WIRE, 0.7f, 0.8f, 0);
    CommandFramer f;
    TEST_ASSERT_EQUAL_INT(0, pushAll(f, stream, 200));
    TEST_ASSERT_TRUE(f.inFrame());
    f.reset();
    TEST_ASSERT_FALSE(f.inFrame());
    TEST_ASSERT_EQUAL_INT(1, pushAll(f, stream + WIRE, WIRE));
    TEST_ASSERT_EQUAL_FLOAT(0.7f, f.frame().ServoAng);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_one_frame);
    RUN_TEST(test_frame_split_across_turns);
    RUN_TEST(test_garbage_and_partial_magic_before_a_frame);
    RUN_TEST(test_back_to_back_frames);
    RUN_TEST(test_corrupt_frame_fails_crc_and_the_next_one_passes);
    RUN_TEST(test_magic_inside_a_body_does_not_resync);
    RUN_TEST(test_reset_drops_a_partial_frame);
    return UNITY_END();
}
