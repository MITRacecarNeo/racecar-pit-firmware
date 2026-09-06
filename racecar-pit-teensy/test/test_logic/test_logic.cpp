#include <unity.h>

#include "crc16.h"
#include "drive_map.h"

// Bounds mirrored from cfg::SERVO_D135F and cfg::ESC_10BL60_G2. They are
// restated here so the host test stays free of the Arduino headers config.h
// pulls in; config.h static_asserts its own ordering invariants.
static const uint16_t STEER_LO = 1025, STEER_NEU = 1500, STEER_HI = 1975;
static const uint16_t THROT_LO = 800, THROT_NEU = 1500, THROT_HI = 2200;

void setUp(void) {}
void tearDown(void) {}

// ------------------------------ crc16 ----------------------------- //

static void test_crc_check_value(void)
{
    const uint8_t msg[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16_ccitt(msg, sizeof(msg)));
}

static void test_crc_empty_input_returns_init(void)
{
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, crc16_ccitt(nullptr, 0));
}

static void test_crc_chunked_matches_single_pass(void)
{
    const uint8_t msg[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03};
    uint16_t whole = crc16_ccitt(msg, sizeof(msg));
    uint16_t chunked = crc16_ccitt(msg, 4);
    chunked = crc16_ccitt(msg + 4, sizeof(msg) - 4, chunked);
    TEST_ASSERT_EQUAL_HEX16(whole, chunked);
}

// A single flipped bit has to change the checksum; this is the property the
// receive path relies on to drop corrupt frames.
static void test_crc_detects_single_bit_flip(void)
{
    uint8_t msg[16] = {0};
    uint16_t clean = crc16_ccitt(msg, sizeof(msg));
    msg[7] ^= 0x08;
    TEST_ASSERT_NOT_EQUAL(clean, crc16_ccitt(msg, sizeof(msg)));
}

// --------------------------- mapNormToPwm ------------------------- //

static void test_map_endpoints_and_center(void)
{
    TEST_ASSERT_EQUAL_UINT16(STEER_LO, mapNormToPwm(-1.0f, STEER_LO, STEER_NEU, STEER_HI));
    TEST_ASSERT_EQUAL_UINT16(STEER_NEU, mapNormToPwm(0.0f, STEER_LO, STEER_NEU, STEER_HI));
    TEST_ASSERT_EQUAL_UINT16(STEER_HI, mapNormToPwm(1.0f, STEER_LO, STEER_NEU, STEER_HI));
}

// The throttle range is asymmetric about neutral (800/1500/2200), so each half
// has to scale against its own span rather than a shared one.
static void test_map_asymmetric_halves_scale_independently(void)
{
    TEST_ASSERT_EQUAL_UINT16(1850, mapNormToPwm(0.5f, THROT_LO, THROT_NEU, THROT_HI));
    TEST_ASSERT_EQUAL_UINT16(1150, mapNormToPwm(-0.5f, THROT_LO, THROT_NEU, THROT_HI));
}

static void test_map_clamps_out_of_range_input(void)
{
    TEST_ASSERT_EQUAL_UINT16(THROT_HI, mapNormToPwm(9.0f, THROT_LO, THROT_NEU, THROT_HI));
    TEST_ASSERT_EQUAL_UINT16(THROT_LO, mapNormToPwm(-9.0f, THROT_LO, THROT_NEU, THROT_HI));
}

static void test_map_stays_inside_bounds_across_sweep(void)
{
    for (int i = -120; i <= 120; i++)
    {
        uint16_t pwm = mapNormToPwm(i / 100.0f, THROT_LO, THROT_NEU, THROT_HI);
        TEST_ASSERT_GREATER_OR_EQUAL_UINT16(THROT_LO, pwm);
        TEST_ASSERT_LESS_OR_EQUAL_UINT16(THROT_HI, pwm);
    }
}

// --------------------------- hallDirection ------------------------ //

static void test_hall_sequence_directions(void)
{
    const uint8_t seq[6] = {1, 3, 2, 6, 4, 5};
    for (int i = 0; i < 6; i++)
    {
        TEST_ASSERT_EQUAL_INT8(-1, hallDirection(seq[i], seq[(i + 1) % 6]));
        TEST_ASSERT_EQUAL_INT8(1, hallDirection(seq[i], seq[(i + 5) % 6]));
    }
}

// 0 and 7 are all-low / all-high, which the sensors never produce; a skipped
// state means a missed edge. Both must report unknown rather than a direction.
static void test_hall_invalid_transitions_report_zero(void)
{
    TEST_ASSERT_EQUAL_INT8(0, hallDirection(0, 1));
    TEST_ASSERT_EQUAL_INT8(0, hallDirection(1, 7));
    TEST_ASSERT_EQUAL_INT8(0, hallDirection(1, 2));
    TEST_ASSERT_EQUAL_INT8(0, hallDirection(3, 3));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_crc_check_value);
    RUN_TEST(test_crc_empty_input_returns_init);
    RUN_TEST(test_crc_chunked_matches_single_pass);
    RUN_TEST(test_crc_detects_single_bit_flip);
    RUN_TEST(test_map_endpoints_and_center);
    RUN_TEST(test_map_asymmetric_halves_scale_independently);
    RUN_TEST(test_map_clamps_out_of_range_input);
    RUN_TEST(test_map_stays_inside_bounds_across_sweep);
    RUN_TEST(test_hall_sequence_directions);
    RUN_TEST(test_hall_invalid_transitions_report_zero);
    return UNITY_END();
}
