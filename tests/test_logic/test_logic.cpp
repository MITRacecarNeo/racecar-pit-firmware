#include <unity.h>

#include "battery.h"
#include "config.h"
#include "crc16.h"
#include "drive_map.h"

static const uint16_t STEER_LO = cfg::SERVO_D135F::angle_pwm_min;
static const uint16_t STEER_NEU = cfg::SERVO_D135F::angle_pwm_neutral;
static const uint16_t STEER_HI = cfg::SERVO_D135F::angle_pwm_max;
static const uint16_t THROT_LO = cfg::ESC_10BL60_G2::speed_pwm_min;
static const uint16_t THROT_NEU = cfg::ESC_10BL60_G2::speed_pwm_neutral;
static const uint16_t THROT_HI = cfg::ESC_10BL60_G2::speed_pwm_max;
static const uint16_t RC_LO = cfg::AUTO_DRIVE::rc_valid_min;
static const uint16_t RC_HI = cfg::AUTO_DRIVE::rc_valid_max;
static const uint16_t SW_THRESH = cfg::FLYSKY_IA6B_RX::sw_threshold;
static const float DEADBAND = cfg::ESC_10BL60_G2::cmd_deadband;
static const float MIN_MPS = cfg::PID_CTRL::min_speed_mps;
static const float MAX_MPS = cfg::PID_CTRL::max_speed_mps;
static const float SLOW_MPS = cfg::PID_CTRL::slow_speed_mps;

// The RC map DriveControl uses, fixed to the shipped 0.25 trim so the expected
// PWM values below stay hand-checkable.
static const RcThrottleMap RC_MAP = {1000, 1500, 2000, 1475, 1525, 1575, 1325, 1500, 1675};

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

// The property the receive path relies on to drop corrupt frames.
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

// The throttle range is asymmetric about neutral, so each half scales against
// its own span.
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
// state means a missed edge. Both report unknown, not a direction.
static void test_hall_invalid_transitions_report_zero(void)
{
    TEST_ASSERT_EQUAL_INT8(0, hallDirection(0, 1));
    TEST_ASSERT_EQUAL_INT8(0, hallDirection(1, 7));
    TEST_ASSERT_EQUAL_INT8(0, hallDirection(1, 2));
    TEST_ASSERT_EQUAL_INT8(0, hallDirection(3, 3));
}

// --------------------------- RC switches -------------------------- //

static void test_rc_pulse_validity_band(void)
{
    TEST_ASSERT_TRUE(rcPulseValid(1500, RC_LO, RC_HI));
    TEST_ASSERT_TRUE(rcPulseValid(RC_LO, RC_LO, RC_HI));
    TEST_ASSERT_TRUE(rcPulseValid(RC_HI, RC_LO, RC_HI));
    TEST_ASSERT_FALSE(rcPulseValid(0, RC_LO, RC_HI));
    TEST_ASSERT_FALSE(rcPulseValid(2500, RC_LO, RC_HI));
}

static void test_rc_switch_positions(void)
{
    TEST_ASSERT_TRUE(rcSwitchUp(2000, RC_LO, RC_HI, SW_THRESH));
    TEST_ASSERT_TRUE(rcSwitchUp(SW_THRESH, RC_LO, RC_HI, SW_THRESH));
    TEST_ASSERT_FALSE(rcSwitchUp(1499, RC_LO, RC_HI, SW_THRESH));
    TEST_ASSERT_FALSE(rcSwitchUp(1000, RC_LO, RC_HI, SW_THRESH));
}

// The safety property behind both switches: an absent channel reads 0, which
// must resolve to down, so a lost receiver leaves fast mode and autonomy on
// rather than latching the car slow or locking the Pi out.
static void test_rc_switch_absent_channel_reads_down(void)
{
    TEST_ASSERT_FALSE(rcSwitchUp(0, RC_LO, RC_HI, SW_THRESH));
    TEST_ASSERT_FALSE(rcSwitchUp(2500, RC_LO, RC_HI, SW_THRESH));
}

// ----------------------- throttleToTargetMps ---------------------- //

// The property slow mode rests on: full stick lands on the cap exactly, in
// either mode, so "capped at 1.0 m/s" is a setpoint and not an approximation.
static void test_setpoint_full_stick_hits_the_cap_exactly(void)
{
    TEST_ASSERT_EQUAL_FLOAT(MAX_MPS, throttleToTargetMps(1.0f, DEADBAND, MIN_MPS, MAX_MPS));
    TEST_ASSERT_EQUAL_FLOAT(-MAX_MPS, throttleToTargetMps(-1.0f, DEADBAND, MIN_MPS, MAX_MPS));
    TEST_ASSERT_EQUAL_FLOAT(SLOW_MPS, throttleToTargetMps(1.0f, DEADBAND, MIN_MPS, SLOW_MPS));
    TEST_ASSERT_EQUAL_FLOAT(-SLOW_MPS, throttleToTargetMps(-1.0f, DEADBAND, MIN_MPS, SLOW_MPS));
}

// Slow mode rescales rather than clips: half stick has to land strictly between
// the endpoints, not pinned at the cap the way a clamp would leave it.
static void test_setpoint_slow_mode_rescales_rather_than_clips(void)
{
    float half = throttleToTargetMps(0.5f, DEADBAND, MIN_MPS, SLOW_MPS);
    TEST_ASSERT_TRUE(half > MIN_MPS);
    TEST_ASSERT_TRUE(half < SLOW_MPS);

    // A wider band than the shipped one, so the rescale shows even when the
    // build's ceiling equals the slow-mode cap.
    float wide_half = throttleToTargetMps(0.5f, DEADBAND, MIN_MPS, 4.0f);
    TEST_ASSERT_TRUE(wide_half > half);
}

static void test_setpoint_just_past_deadband_starts_at_min(void)
{
    TEST_ASSERT_EQUAL_FLOAT(MIN_MPS, throttleToTargetMps(DEADBAND, DEADBAND, MIN_MPS, MAX_MPS));
    TEST_ASSERT_EQUAL_FLOAT(-MIN_MPS, throttleToTargetMps(-DEADBAND, DEADBAND, MIN_MPS, MAX_MPS));
}

static void test_setpoint_clamps_and_stays_in_band(void)
{
    TEST_ASSERT_EQUAL_FLOAT(SLOW_MPS, throttleToTargetMps(9.0f, DEADBAND, MIN_MPS, SLOW_MPS));
    TEST_ASSERT_EQUAL_FLOAT(-SLOW_MPS, throttleToTargetMps(-9.0f, DEADBAND, MIN_MPS, SLOW_MPS));

    for (int i = -100; i <= 100; i++)
    {
        float mps = throttleToTargetMps(i / 100.0f, DEADBAND, MIN_MPS, SLOW_MPS);
        TEST_ASSERT_TRUE(mps <= SLOW_MPS + 1e-4f);
        TEST_ASSERT_TRUE(mps >= -SLOW_MPS - 1e-4f);
    }
}

// --------------------------- RC throttle -------------------------- //

static void test_map_range_matches_teensyduino_rounding(void)
{
    TEST_ASSERT_EQUAL_INT32(1675, mapRange(2000, 1500, 2000, 1575, 1675));
    TEST_ASSERT_EQUAL_INT32(1580, mapRange(1526, 1500, 2000, 1575, 1675));
    TEST_ASSERT_EQUAL_INT32(1325, mapRange(1000, 1000, 1500, 1325, 1475));
    TEST_ASSERT_EQUAL_INT32(1467, mapRange(1474, 1000, 1500, 1325, 1475));
    TEST_ASSERT_EQUAL_INT32(50, mapRange(7, 7, 7, 0, 100));
}

static void test_rc_map_matches_config(void)
{
    TEST_ASSERT_EQUAL_UINT16(cfg::ESC_10BL60_G2::rc_pwm_min, RC_MAP.pwm_min);
    TEST_ASSERT_EQUAL_UINT16(cfg::ESC_10BL60_G2::rc_pwm_max, RC_MAP.pwm_max);
    TEST_ASSERT_EQUAL_UINT16(cfg::ESC_10BL60_G2::rc_throt_fwd_floor, RC_MAP.fwd_floor);
    TEST_ASSERT_EQUAL_UINT16(cfg::ESC_10BL60_G2::rc_throt_fwd_break, RC_MAP.fwd_break);
    TEST_ASSERT_EQUAL_UINT16(cfg::ESC_10BL60_G2::rc_throt_rev_break, RC_MAP.rev_break);
}

// Inside the deadzone the ESC sees neutral; just past it the output jumps to
// the first PWM that moves the car, so the stick has no dead travel.
static void test_rc_throttle_deadzone_and_floor(void)
{
    TEST_ASSERT_EQUAL_UINT16(1500, rcThrottleToPwm(1500, RC_MAP));
    TEST_ASSERT_EQUAL_UINT16(1500, rcThrottleToPwm(1525, RC_MAP));
    TEST_ASSERT_EQUAL_UINT16(1500, rcThrottleToPwm(1475, RC_MAP));
    TEST_ASSERT_EQUAL_UINT16(1580, rcThrottleToPwm(1526, RC_MAP));
    TEST_ASSERT_EQUAL_UINT16(1467, rcThrottleToPwm(1474, RC_MAP));
}

static void test_rc_throttle_endpoints_and_clamp(void)
{
    TEST_ASSERT_EQUAL_UINT16(1675, rcThrottleToPwm(2000, RC_MAP));
    TEST_ASSERT_EQUAL_UINT16(1325, rcThrottleToPwm(1000, RC_MAP));
    TEST_ASSERT_EQUAL_UINT16(1675, rcThrottleToPwm(2100, RC_MAP));
    TEST_ASSERT_EQUAL_UINT16(1325, rcThrottleToPwm(900, RC_MAP));
}

static void test_rc_throttle_norm(void)
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rcThrottleToNorm(1500, RC_MAP));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rcThrottleToNorm(1520, RC_MAP));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, rcThrottleToNorm(2000, RC_MAP));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, rcThrottleToNorm(1000, RC_MAP));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, rcThrottleToNorm(1750, RC_MAP));
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, rcThrottleToNorm(1250, RC_MAP));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, rcThrottleToNorm(2100, RC_MAP));
}

// --------------------------- battery SoC -------------------------- //

static void test_soc_endpoints_and_clamping(void)
{
    TEST_ASSERT_EQUAL_UINT8(100, batterySocPercent(batt_full_mv));
    TEST_ASSERT_EQUAL_UINT8(100, batterySocPercent(9000));
    TEST_ASSERT_EQUAL_UINT8(0, batterySocPercent(batt_empty_mv));
    TEST_ASSERT_EQUAL_UINT8(0, batterySocPercent(6000));
    TEST_ASSERT_EQUAL_UINT8(0, batterySocPercent(0));
}

static void test_soc_is_monotonic_across_the_band(void)
{
    uint8_t prev = 0;
    for (uint16_t mv = batt_empty_mv; mv <= batt_full_mv; mv += 10)
    {
        uint8_t pct = batterySocPercent(mv);
        TEST_ASSERT_LESS_OR_EQUAL_UINT8(100, pct);
        TEST_ASSERT_GREATER_OR_EQUAL_UINT8(prev, pct);
        prev = pct;
    }
}

// Every measured point returns its own charge exactly.
static void test_soc_hits_each_measured_point(void)
{
    const uint16_t mv[] = {8200, 8110, 8050, 7860, 7740, 7620, 7545, 7485, 7450, 7395, 7305, 7280, 7195};
    const uint8_t pct[] = {100, 95, 90, 80, 70, 60, 50, 40, 30, 20, 10, 5, 2};
    for (size_t i = 0; i < sizeof(mv) / sizeof(mv[0]); i++)
        TEST_ASSERT_EQUAL_UINT8(pct[i], batterySocPercent(mv[i]));
}

// A full battery reads about 8.19 V under load and must show full; the last
// 10% sits between 7.3 and 7.0 V. A linear map would read 85% and 22% there.
static void test_soc_tracks_the_curve_not_the_voltage(void)
{
    TEST_ASSERT_EQUAL_UINT8(100, batterySocPercent(8300));
    TEST_ASSERT_EQUAL_UINT8(99, batterySocPercent(8190));
    TEST_ASSERT_EQUAL_UINT8(10, batterySocPercent(7305));
    TEST_ASSERT_EQUAL_UINT8(66, batterySocPercent(7700));
}

static void test_soc_interpolates_between_breakpoints(void)
{
    TEST_ASSERT_EQUAL_UINT8(57, batterySocPercent(7600));
    TEST_ASSERT_EQUAL_UINT8(42, batterySocPercent(7500));
    TEST_ASSERT_EQUAL_UINT8(20, batterySocPercent(7400));
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
    RUN_TEST(test_rc_pulse_validity_band);
    RUN_TEST(test_rc_switch_positions);
    RUN_TEST(test_rc_switch_absent_channel_reads_down);
    RUN_TEST(test_setpoint_full_stick_hits_the_cap_exactly);
    RUN_TEST(test_setpoint_slow_mode_rescales_rather_than_clips);
    RUN_TEST(test_setpoint_just_past_deadband_starts_at_min);
    RUN_TEST(test_setpoint_clamps_and_stays_in_band);
    RUN_TEST(test_map_range_matches_teensyduino_rounding);
    RUN_TEST(test_rc_map_matches_config);
    RUN_TEST(test_rc_throttle_deadzone_and_floor);
    RUN_TEST(test_rc_throttle_endpoints_and_clamp);
    RUN_TEST(test_rc_throttle_norm);
    RUN_TEST(test_soc_endpoints_and_clamping);
    RUN_TEST(test_soc_is_monotonic_across_the_band);
    RUN_TEST(test_soc_hits_each_measured_point);
    RUN_TEST(test_soc_tracks_the_curve_not_the_voltage);
    RUN_TEST(test_soc_interpolates_between_breakpoints);
    return UNITY_END();
}
