#pragma once
#include <stddef.h>
#include <stdint.h>

#include "packets.h"

// All tunables live here. Nothing in src/ should hardcode a value that belongs
// in this file. Arduino-free, so host tests read the same values; the Pi reads
// them back through the info reply (docs/protocol.md).
namespace cfg
{

    namespace UART
    {
        constexpr uint32_t baud_rate = 921600;
        constexpr uint16_t tx_freq = 200;    // Hz, telemetry rate
        constexpr uint32_t rx_timeout = 50;  // ms, command staleness -> neutral
        constexpr bool verify_rx_crc = true; // drop command frames failing CRC-16
        // The default 64-byte Serial1 FIFO overflows mid-frame: a 474-byte
        // command at 921600 baud fills it in ~0.7 ms, less than UARTRx can be
        // preempted for.
        constexpr size_t rx_buffer_bytes = 2048;
        // Holds one telemetry frame plus one info chunk, so a slot never
        // blocks the transmit thread.
        constexpr size_t tx_buffer_bytes = 512;
        // A command body takes 5.1 ms on the wire; a frame still incomplete
        // this long after its magic is dropped.
        constexpr uint32_t rx_read_timeout_ms = 20;
    }

    namespace DEBUG
    {
        constexpr uint32_t usb_baud = 115200;
        // Teleplot trace on USB serial; runs inside the 1 kHz actuation loop.
        constexpr bool serial_plot = false;
    }

    namespace I2C
    {
        // INA226 and LSM9DS1 share the bus. Wire blocks the calling thread for
        // each transfer: one LSM9DS1 read is about 1.1 ms at 400 kHz and 3.9 ms
        // at 100 kHz. Applied after the sensors start (sensors.cpp).
        constexpr uint32_t clock_hz = 400000;
    }

    namespace RGB_LED_WS2812B
    {
        constexpr uint16_t num_leds = 84;
        constexpr uint8_t led_pin = 9;
        constexpr uint8_t brightness = 25;
    }

    namespace ESC_10BL60_G2
    {
        constexpr uint8_t motor_pin = 3;
        // Full RC throw instead of the percent_* trim.
        constexpr bool speed_demon = false;

        // RC-manual throw as a fraction of the range, when speed_demon is off.
        constexpr float percent_throt = 0.25;
        constexpr float percent_rev = 0.25;

        // PWM safety bounds. The Pi does not scale the range, so the Teensy owns
        // the physical limits.
        constexpr uint16_t speed_pwm_min = 800;      // us, full-reverse bound; deeper than 1000 to clear the ESC reverse deadband
        constexpr uint16_t speed_pwm_neutral = 1500; // us, ESC arm / neutral
        constexpr uint16_t speed_pwm_max = 2200;     // us, full-forward bound

        // RC-manual PWM endpoints on the direct (fast-mode) path.
        constexpr uint16_t rc_pwm_max = speed_demon
                                            ? speed_pwm_max
                                            : (uint16_t)(speed_pwm_neutral + (speed_pwm_max - speed_pwm_neutral) * percent_throt);
        constexpr uint16_t rc_pwm_min = speed_demon
                                            ? speed_pwm_min
                                            : (uint16_t)(speed_pwm_neutral - (speed_pwm_neutral - speed_pwm_min) * percent_rev);

        // RC throttle stick deadzone, us.
        constexpr uint16_t rc_throt_fwd_break = 1525;
        constexpr uint16_t rc_throt_rev_break = 1475;
        constexpr uint16_t rc_throt_fwd_floor = 1575; // first PWM past the ESC forward deadband

        constexpr float cmd_deadband = 0.01; // ignore joystick noise around zero
    }

    namespace SERVO_D135F
    {
        constexpr uint8_t servo_pin = 2;

        // Steering PWM safety bounds. -1 = full left, +1 = full right.
        constexpr uint16_t angle_pwm_min = 1025;     // us, full-left bound
        constexpr uint16_t angle_pwm_neutral = 1500; // us, centered
        constexpr uint16_t angle_pwm_max = 1975;     // us, full-right bound
    }

    namespace FLYSKY_IA6B_RX
    {
        constexpr uint8_t ppm_pin = 5;
        // PPM frame width. The iA6B has six servo rails, but its PPM stream
        // carries the whole frame, so all eight slots populate.
        constexpr uint8_t channel_count = wire::rc_channels;

        // Zero-based indices into channelVals[]. AETR order, so PPM channel N
        // is index N-1: 0 aileron (right stick X), 1 elevator (right stick Y),
        // 2 throttle (left stick Y), 3 rudder (left stick X), 4-7 the aux
        // channels, which the FS-i6 assigns swA through swD.
        //
        // Drive reads the two self-centering axes. The transmitter's own
        // throttle axis (CH3) is ratcheted and rests at its low endpoint, which
        // the RC override would read as a held reverse command.
        constexpr uint8_t ch_steer = 3; // CH4 rudder, left stick X
        constexpr uint8_t ch_throt = 1; // CH2 elevator, right stick Y

        // Two-position switches, shared with the driver: it reads the same
        // channels and threshold out of the telemetry RC array.
        //   ch_speed_mode      down = fast (config cap), up = slow
        //   ch_manual_override down = Pi autonomy allowed, up = RC only
        // Both default to the permissive position when the channel is absent,
        // so losing the receiver never leaves the car slow or locked out.
        constexpr uint8_t ch_speed_mode = 4;      // CH5, swA
        constexpr uint8_t ch_manual_override = 7; // CH8, swD
        constexpr uint16_t sw_threshold = 1500;   // us, at or above = switch up

        // PPMReader clamps valid pulses into this band and reports 0 for a
        // channel that is absent or stale past its 500 ms failsafe.
        constexpr uint16_t pulse_min = 1000; // us
        constexpr uint16_t pulse_max = 2000; // us
    }

    namespace INA226_SENSOR
    {
        constexpr uint8_t i2c_addy = 0x40;
        constexpr uint8_t avg_set = 2; // INA226 library enum index: 2 = 16 samples
        // 16 samples of 1.1 ms bus plus 1.1 ms shunt conversion give a new
        // value every 35.2 ms. A slower poll leaves the bus to the IMU thread,
        // which then holds the CPU longer under round-robin scheduling.
        constexpr uint16_t poll_ms = 1;
        constexpr float max_curr_expected = 8.0;  // Amps
        constexpr float shunt_resistance = 0.002; // Ohms
    }

    namespace LSM9DS1_IMU
    {
        // Physical settings; sensors.cpp maps them onto the Adafruit enums.
        constexpr uint8_t accel_range_g = 8;      // 2, 4, 8, 16
        constexpr uint16_t accel_rate_hz = 119;   // 10, 50, 119, 238, 476, 952
        constexpr uint8_t mag_range_gauss = 8;    // 4, 8, 12, 16
        constexpr uint16_t gyro_range_dps = 500;  // 245, 500, 2000
    }

    namespace MAX_7219_DOT_MATRIX
    {
        constexpr uint8_t cs_pin = 10;
        constexpr uint8_t num_matrix = 3;
        constexpr uint8_t intensity = 8;
        constexpr uint8_t rows = 8;
        constexpr uint16_t cols = 8 * num_matrix;
    }

    namespace ES2845S_4900KV_MOTOR
    {
        constexpr uint8_t hall_a_pin = 30;
        constexpr uint8_t hall_b_pin = 31;
        constexpr uint8_t hall_c_pin = 32;
        constexpr uint8_t mot_temp_pin = 41;        // A17; wired, not sampled
        constexpr uint32_t stop_timeout_us = 30000; // no hall edge in this window -> RPM 0
        constexpr uint8_t pole_pairs = 2;
        constexpr uint8_t hall_states_per_erev = 6;
        constexpr uint8_t rpm_window = 5;       // hall intervals averaged
        constexpr float rpm_filter_alpha = 0.8; // weight on the new sample
    }

    namespace AUTO_DRIVE
    {
        // The RC transmitter takes priority whenever a stick output leaves its
        // neutral band; otherwise the Pi command stream drives.
        constexpr uint16_t throt_override_band = 40; // us around speed_pwm_neutral
        constexpr uint16_t steer_override_band = 40; // us around angle_pwm_neutral
        // Outside this band the receiver is treated as absent. PPMReader only
        // ever reports 0 or 1000-2000, so in practice this rejects the 0.
        constexpr uint16_t rc_valid_min = 900;  // us
        constexpr uint16_t rc_valid_max = 2100; // us
    }

    namespace DISP
    {
        constexpr uint32_t cmd_timeout_ms = 1000; // Pi command staleness -> idle defaults
        constexpr uint32_t led_load_ms = 10000;   // rainbow sweep duration on Pi connect
        // The idle LED bar reads charge, not volts; endpoints and the 2S curve
        // live in battery.h (batt_empty_mv / batt_full_mv).
    }

    // Bit layout of RxPacket.SystemState (Pi -> Teensy).
    namespace SYS_STATE
    {
        constexpr uint8_t mode_mask = 0x03; // 0 idle, 1 manual, 2 auto
        constexpr uint8_t mode_idle = 0;
        constexpr uint8_t mode_manual = 1;
        constexpr uint8_t mode_auto = 2;
        constexpr uint8_t dotmatrix_active = 0x04; // Pi is commanding the dot matrix
        constexpr uint8_t led_active = 0x08;       // Pi is commanding the LED strip
        constexpr uint8_t driver_starting = 0x10;  // Pi driver just launched: play the LED loading sweep
        constexpr uint8_t info_request = 0x20;     // send info chunks until the Pi clears it
    }

    namespace TEENSY_THREADS
    {
        constexpr uint16_t startup_delay = 2000; // ms
        constexpr uint16_t tick_ms = 1;          // worker thread yield
        constexpr uint16_t loop_tick_ms = 25;    // Arduino loop() yield
        constexpr uint16_t disp_tick_ms = 50;    // dot matrix refresh
        constexpr uint16_t splash_tick_ms = 20;  // boot splash animation step
    }

    namespace CAR_SPECS
    {
        constexpr float pi = 3.14159265f;
        constexpr float wheel_diameter = 0.072;                    // m
        constexpr float wheel_circumference = wheel_diameter * pi; // m
        constexpr float ratio_motor_to_transaxle = 18.0 / 61.0;
        constexpr float ratio_transaxle_to_diff = 15.0 / 37.0;
    }

    namespace PID_CTRL
    {
        // The normalized [-1,1] Pi throttle maps onto this band; a PI controller
        // on the hall encoder holds the setpoint. Indoor build: 1.0 m/s ceiling.
        // The field build raises max_speed_mps to 4.0.
        constexpr float max_speed_mps = 1.0f; // top speed at full joystick
        constexpr float min_speed_mps = 0.5f; // crawl speed at minimum joystick
        // Slow mode (swA up) rescales full stick onto this, on both the Pi and
        // the RC path; RC closes the loop in slow mode so the cap is real.
        constexpr float slow_speed_mps = 1.0f;

        constexpr float kp = 20.0f;
        constexpr float ki = 30.0f;
        constexpr float max_integral = 150.0f; // anti-windup limit
        constexpr float kd = 0.0f;
        constexpr float kd_filter_alpha = 0.05f;

        // Feedforward and gain scheduling.
        constexpr float deadband_fwd_us = 75.0f;
        constexpr float deadband_rev_us = 70.0f;

        constexpr float kv_ff_fwd = 10.0f;
        constexpr float kv_ff_rev = 40.0f;

        constexpr float kp_boost = 60.0f;
        constexpr float err_threshold = 0.25f;

        // Breaks static friction from a standstill; the reverse deadband is too
        // wide for the PI term alone.
        constexpr float stiction_kick_rev_us = 100.0f;
        constexpr float stiction_speed_mps = 0.05f; // below this the wheels count as stopped
    }

    // Invariants the actuation math assumes. A bad edit here is a runaway servo
    // or ESC, so catch it at compile time.
    static_assert(ESC_10BL60_G2::speed_pwm_min < ESC_10BL60_G2::speed_pwm_neutral &&
                      ESC_10BL60_G2::speed_pwm_neutral < ESC_10BL60_G2::speed_pwm_max,
                  "ESC PWM bounds must be ordered min < neutral < max");
    static_assert(ESC_10BL60_G2::speed_pwm_min <= ESC_10BL60_G2::rc_pwm_min &&
                      ESC_10BL60_G2::rc_pwm_max <= ESC_10BL60_G2::speed_pwm_max,
                  "RC endpoints must sit inside the ESC bounds");
    static_assert(SERVO_D135F::angle_pwm_min < SERVO_D135F::angle_pwm_neutral &&
                      SERVO_D135F::angle_pwm_neutral < SERVO_D135F::angle_pwm_max,
                  "servo PWM bounds must be ordered min < neutral < max");
    static_assert(ESC_10BL60_G2::rc_throt_rev_break < ESC_10BL60_G2::speed_pwm_neutral &&
                      ESC_10BL60_G2::speed_pwm_neutral < ESC_10BL60_G2::rc_throt_fwd_break,
                  "RC throttle deadzone must straddle neutral");
    static_assert(AUTO_DRIVE::rc_valid_min < FLYSKY_IA6B_RX::pulse_min &&
                      FLYSKY_IA6B_RX::pulse_max < AUTO_DRIVE::rc_valid_max,
                  "RC validity band must contain the nominal pulse range");
    static_assert(PID_CTRL::min_speed_mps < PID_CTRL::max_speed_mps,
                  "speed band must be ordered min < max");
    static_assert(FLYSKY_IA6B_RX::ch_steer < FLYSKY_IA6B_RX::channel_count &&
                      FLYSKY_IA6B_RX::ch_throt < FLYSKY_IA6B_RX::channel_count &&
                      FLYSKY_IA6B_RX::ch_speed_mode < FLYSKY_IA6B_RX::channel_count &&
                      FLYSKY_IA6B_RX::ch_manual_override < FLYSKY_IA6B_RX::channel_count,
                  "RC channel indices must fit channelVals[]");
    static_assert(FLYSKY_IA6B_RX::ch_steer != FLYSKY_IA6B_RX::ch_throt &&
                      FLYSKY_IA6B_RX::ch_speed_mode != FLYSKY_IA6B_RX::ch_manual_override &&
                      FLYSKY_IA6B_RX::ch_speed_mode != FLYSKY_IA6B_RX::ch_throt &&
                      FLYSKY_IA6B_RX::ch_speed_mode != FLYSKY_IA6B_RX::ch_steer &&
                      FLYSKY_IA6B_RX::ch_manual_override != FLYSKY_IA6B_RX::ch_throt &&
                      FLYSKY_IA6B_RX::ch_manual_override != FLYSKY_IA6B_RX::ch_steer,
                  "each RC function must read its own channel");
    static_assert(FLYSKY_IA6B_RX::pulse_min < FLYSKY_IA6B_RX::sw_threshold &&
                      FLYSKY_IA6B_RX::sw_threshold < FLYSKY_IA6B_RX::pulse_max,
                  "switch threshold must sit inside the pulse range");
    static_assert(PID_CTRL::slow_speed_mps >= PID_CTRL::min_speed_mps &&
                      PID_CTRL::slow_speed_mps <= PID_CTRL::max_speed_mps,
                  "slow-mode cap must sit inside the speed band");
    static_assert(ES2845S_4900KV_MOTOR::rpm_filter_alpha > 0.0f &&
                      ES2845S_4900KV_MOTOR::rpm_filter_alpha <= 1.0f,
                  "RPM filter alpha must be in (0, 1]");
    static_assert(RGB_LED_WS2812B::num_leds * 3 <= wire::led_bytes,
                  "LED strip must fit the command LED field");
    static_assert(MAX_7219_DOT_MATRIX::rows * MAX_7219_DOT_MATRIX::cols == wire::dot_matrix_bytes,
                  "dot matrix must match the command bitmap");
    static_assert(UART::tx_buffer_bytes >= sizeof(TXPacket) + info_packet_max_bytes,
                  "transmit buffer must hold one telemetry frame and one info chunk");
    static_assert(LSM9DS1_IMU::accel_range_g == 2 || LSM9DS1_IMU::accel_range_g == 4 ||
                      LSM9DS1_IMU::accel_range_g == 8 || LSM9DS1_IMU::accel_range_g == 16,
                  "unsupported LSM9DS1 accel range");
    static_assert(LSM9DS1_IMU::accel_rate_hz == 10 || LSM9DS1_IMU::accel_rate_hz == 50 ||
                      LSM9DS1_IMU::accel_rate_hz == 119 || LSM9DS1_IMU::accel_rate_hz == 238 ||
                      LSM9DS1_IMU::accel_rate_hz == 476 || LSM9DS1_IMU::accel_rate_hz == 952,
                  "unsupported LSM9DS1 accel rate");
    static_assert(LSM9DS1_IMU::mag_range_gauss == 4 || LSM9DS1_IMU::mag_range_gauss == 8 ||
                      LSM9DS1_IMU::mag_range_gauss == 12 || LSM9DS1_IMU::mag_range_gauss == 16,
                  "unsupported LSM9DS1 mag range");
    static_assert(LSM9DS1_IMU::gyro_range_dps == 245 || LSM9DS1_IMU::gyro_range_dps == 500 ||
                      LSM9DS1_IMU::gyro_range_dps == 2000,
                  "unsupported LSM9DS1 gyro range");

}
