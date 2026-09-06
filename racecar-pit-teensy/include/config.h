#pragma once
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>

// All tunables live here. Nothing in src/ should hardcode a value that belongs
// in this file.
namespace cfg
{

    namespace UART
    {
        constexpr uint32_t baud_rate = 921600;
        constexpr uint32_t tx_magic_num = 0xDEADBEEF;
        constexpr uint32_t rx_magic_num = 0xBEEFDEAD;
        constexpr uint16_t tx_freq = 200;    // Hz, telemetry rate
        constexpr uint32_t rx_timeout = 50;  // ms, command staleness -> neutral
        constexpr bool verify_rx_crc = true; // drop command frames failing CRC-16
        // Serial1 RX FIFO. The default 64 bytes overflows mid-frame: a 474-byte
        // command at 921600 baud fills 64 bytes in ~0.7 ms, and UARTRx can be
        // preempted longer than that under the thread scheduler.
        constexpr size_t rx_buffer_bytes = 2048;
    }

    namespace DEBUG
    {
        constexpr uint32_t usb_baud = 115200;
        // Teleplot-style control-loop trace on the USB serial port. Off by
        // default; the trace runs inside the 1 kHz actuation loop.
        constexpr bool serial_plot = false;
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
        constexpr bool speed_demon = false;

        // RC-manual throw as a fraction of the configured range, applied around
        // neutral when speed_demon is off.
        constexpr float percent_throt = 0.25;
        constexpr float percent_rev = 0.25;

        // PWM safety bounds. Tighten min/max to protect the drivetrain; the Pi
        // does not scale the range (its throttle scaling is 1), so the Teensy
        // owns the physical limits.
        constexpr uint16_t speed_pwm_min = 800;      // us, full-reverse bound; deeper than a symmetric 1000 to clear the ESC reverse deadband
        constexpr uint16_t speed_pwm_neutral = 1500; // us, ESC arm / neutral
        constexpr uint16_t speed_pwm_max = 2200;     // us, full-forward bound

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
        constexpr uint8_t channel_count = 8;
        // Channel indices into channelVals[], zero-based.
        constexpr uint8_t ch_steer = 0;      // aileron
        constexpr uint8_t ch_throt = 2;      // throttle
        constexpr uint8_t ch_drive_mode = 4; // SWA, selects reverse under speed_demon
        constexpr uint16_t pulse_min = 1000; // us
        constexpr uint16_t pulse_max = 2000; // us
    }

    namespace INA226_SENSOR
    {
        constexpr uint8_t i2c_addy = 0x40;
        constexpr uint8_t avg_set = 2;
        constexpr float max_curr_expected = 8.0;  // Amps
        constexpr float shunt_resistance = 0.002; // Ohms
    }

    namespace LSM9DS1_IMU
    {
        constexpr Adafruit_LSM9DS1::lsm9ds1AccelRange_t accel_range = Adafruit_LSM9DS1::lsm9ds1AccelRange_t::LSM9DS1_ACCELRANGE_8G;
        constexpr Adafruit_LSM9DS1::lsm9ds1AccelDataRate_t poll_rate = Adafruit_LSM9DS1::lsm9ds1AccelDataRate_t::LSM9DS1_ACCELDATARATE_119HZ;
        constexpr Adafruit_LSM9DS1::lsm9ds1MagGain_t mag_gain = Adafruit_LSM9DS1::lsm9ds1MagGain_t::LSM9DS1_MAGGAIN_8GAUSS;
        constexpr Adafruit_LSM9DS1::lsm9ds1GyroScale_t gyro_scale = Adafruit_LSM9DS1::lsm9ds1GyroScale_t::LSM9DS1_GYROSCALE_500DPS;
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
        constexpr uint8_t mot_temp_pin = A17;       // wired, not sampled yet
        constexpr uint32_t stop_timeout_us = 30000; // no hall edge in this window -> RPM 0
        constexpr uint8_t pole_pairs = 2;
        constexpr uint8_t hall_states_per_erev = 6;
        constexpr uint8_t rpm_window = 5;    // hall intervals averaged
        constexpr float rpm_filter_alpha = 0.8; // weight on the new sample
    }

    namespace AUTO_DRIVE
    {
        // Drive priority: the RC transmitter overrides the Pi whenever a stick
        // leaves its neutral band. With the sticks centered (or no receiver),
        // the Pi command stream drives as the fallback.
        // RC-active detection: |RC-commanded PWM - neutral| beyond this -> human driving.
        constexpr uint16_t throt_override_band = 40; // us around speed_pwm_neutral
        constexpr uint16_t steer_override_band = 40; // us around angle_pwm_neutral
        // Plausible RC pulse band; outside it the receiver is treated as absent.
        constexpr uint16_t rc_valid_min = 900;  // us
        constexpr uint16_t rc_valid_max = 2100; // us
    }

    namespace DISP
    {
        // Dot matrix + LED behavior. When the Pi stops sending fresh command
        // frames (driver down), the Teensy falls back to idle defaults.
        constexpr uint32_t cmd_timeout_ms = 1000; // Pi command staleness -> idle defaults
        constexpr uint32_t led_load_ms = 10000;   // rainbow "loading" sweep duration on Pi connect
        // Idle LED voltage bar endpoints, centivolts.
        constexpr int volt_bar_min_cv = 750;
        constexpr int volt_bar_max_cv = 840;
    }

    // Bit layout of RxPacket.SystemState (Pi -> Teensy): the low two bits carry
    // the drive mode for the dot-matrix glyph; two flag bits say whether the Pi
    // is actively driving each display so the Teensy knows when to show defaults.
    namespace SYS_STATE
    {
        constexpr uint8_t mode_mask = 0x03; // 0 idle, 1 manual, 2 auto
        constexpr uint8_t mode_idle = 0;
        constexpr uint8_t mode_manual = 1;
        constexpr uint8_t mode_auto = 2;
        constexpr uint8_t dotmatrix_active = 0x04; // Pi is commanding the dot matrix
        constexpr uint8_t led_active = 0x08;       // Pi is commanding the LED strip
        constexpr uint8_t driver_starting = 0x10;  // Pi driver just launched: play the LED loading sweep
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
        constexpr float wheel_diameter = 0.072;                    // m
        constexpr float wheel_circumference = wheel_diameter * PI; // m
        constexpr float ratio_motor_to_transaxle = 18.0 / 61.0;
        constexpr float ratio_transaxle_to_diff = 15.0 / 37.0;
    }

    namespace PID_CTRL
    {
        // The Pi sends a normalized [-1,1] throttle; it maps onto this speed
        // band and a PI controller on the hall encoder holds it.
        constexpr float max_speed_mps = 5.0f;  // top speed at full joystick
        constexpr float min_speed_mps = 0.75f; // crawl speed at minimum joystick

        constexpr float kp = 20.0f;
        constexpr float ki = 30.0f;
        constexpr float max_integral = 150.0f; // anti-windup limit, heavy to bridge the deadband gap
        constexpr float kd = 0.0f;
        constexpr float kd_filter_alpha = 0.05f;

        // Feedforward and gain scheduling.
        constexpr float deadband_fwd_us = 75.0f;
        constexpr float deadband_rev_us = 70.0f;

        constexpr float kv_ff_fwd = 10.0f;
        constexpr float kv_ff_rev = 40.0f;

        constexpr float kp_boost = 60.0f;
        constexpr float err_threshold = 0.25f;

        // Extra reverse PWM applied while the wheels are still, to break static
        // friction; the reverse deadband is too wide for the PI term alone.
        constexpr float stiction_kick_rev_us = 100.0f;
        constexpr float stiction_speed_mps = 0.05f; // below this the wheels count as stopped
    }

    // Invariants the actuation math assumes. A bad edit here is a runaway
    // servo or ESC, so catch it at compile time.
    static_assert(ESC_10BL60_G2::speed_pwm_min < ESC_10BL60_G2::speed_pwm_neutral &&
                      ESC_10BL60_G2::speed_pwm_neutral < ESC_10BL60_G2::speed_pwm_max,
                  "ESC PWM bounds must be ordered min < neutral < max");
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
                      FLYSKY_IA6B_RX::ch_drive_mode < FLYSKY_IA6B_RX::channel_count,
                  "RC channel indices must fit channelVals[]");
    static_assert(ES2845S_4900KV_MOTOR::rpm_filter_alpha > 0.0f &&
                      ES2845S_4900KV_MOTOR::rpm_filter_alpha <= 1.0f,
                  "RPM filter alpha must be in (0, 1]");

}
