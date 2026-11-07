#pragma once
#include <stddef.h>
#include <stdint.h>

#include "car_constants.h"
#include "dip_record.h"
#include "display_state.h"
#include "led_patterns.h"
#include "packets.h"
#include "stall_guard.h"
#include "start_assist.h"

// All tunables live here. Nothing in src/ should hardcode a value that belongs
// in this file. Arduino-free, so host tests read the same values; the Pi reads
// them back through the info reply (docs/protocol.md).
namespace cfg
{

    namespace UART
    {
        constexpr uint32_t baud_rate = 921600;
        constexpr uint16_t tx_freq = 200;    // Hz, telemetry rate, IntervalTimer
        // Below the drive tick (144) and the Servo, PPM, hall and serial
        // interrupts (128); above every thread.
        constexpr uint8_t tx_irq_priority = 160;
        constexpr uint32_t info_rebuild_ms = 100; // info payload refresh while requested
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
        // Teleplot trace of the speed loop on USB serial, printed from loop()
        // at its 25 ms tick; lines are skipped while nothing reads the port.
        // Build with -DRCN_SERIAL_PLOT to turn it on.
#ifdef RCN_SERIAL_PLOT
        constexpr bool serial_plot = true;
#else
        constexpr bool serial_plot = false;
#endif
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
        constexpr Rgb bar_red = {255, 0, 0};      // battery meter and loading bar
        constexpr Rgb stall_orange = {255, 40, 0}; // stall flash
        // Loading bar from LED 0 to the last over this long after power-on,
        // about the time the Pi takes to start the driver.
        constexpr uint32_t loading_bar_ms = 60000;
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
        // Equal to the ESC throttle calibration endpoints (env teensy41_esc_cal);
        // past them the ESC gives no more throttle.
        constexpr uint16_t speed_pwm_min = 1000;     // us, full-reverse bound
        constexpr uint16_t speed_pwm_neutral = 1500; // us, ESC arm / neutral
        constexpr uint16_t speed_pwm_max = 2000;     // us, full-forward bound

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

        // Two-position switches, decoded by pulse width: high at or above
        // sw_threshold, low below it. On the FS-i6 as set up, lever up (away
        // from the operator) sends low, and the transmitter starts only with
        // every lever up:
        //   ch_speed_mode      low (up) = slow, high (down) = fast (config cap)
        //   ch_manual_override low (up) = Pi allowed, high (down) = RC only
        // A channel that is absent or out of band reads as neither: fast mode
        // and the Pi allowed, so a receiver without a transmitter falls back to
        // the Pi.
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
        // value every 35.2 ms; reading faster only repeats it.
        constexpr uint16_t poll_ms = 10;
        // 40 A is the shunt's range (81.9 mV across 2 mOhm); a locked rotor
        // draws about 16 A. Used without the library's 1/2/5 LSB rounding,
        // which would coarsen 1.22 mA to 2 mA.
        constexpr float max_curr_expected = 40.0; // Amps
        constexpr float shunt_resistance = 0.002; // Ohms
        // Voltage dip record (src/dip_record.h): {dip_v, rearm_v, min_ms,
        // valid_v}. Below 7.2 V for 100 ms counts; the next counts after 7.3 V.
        constexpr DipParams dip = {7.2f, 7.3f, 100, 1.0f};
    }

    namespace LSM9DS1_IMU
    {
        // Physical settings; sensors.cpp maps them onto the Adafruit enums.
        constexpr uint8_t accel_range_g = 8;      // 2, 4, 8, 16
        // Written to ODR_XL. With the gyro on the chip runs the accelerometer
        // at the gyro rate instead; the info reply reports the rates read back.
        constexpr uint16_t accel_rate_hz = 119;   // 10, 50, 119, 238, 476, 952
        constexpr uint8_t mag_range_gauss = 8;    // 4, 8, 12, 16
        constexpr uint16_t gyro_range_dps = 500;  // 245, 500, 2000
        // Read period. Each read (accel, gyro, mag) blocks the bus; 250 Hz
        // keeps a fresh sample in every 200 Hz telemetry frame.
        constexpr uint32_t read_period_us = 4000;
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
        constexpr uint8_t rpm_window = 5;       // hall intervals, median taken
        constexpr float rpm_filter_alpha = 0.8; // weight on the new sample
        constexpr uint8_t dir_confirm_transitions = 2; // agreeing edges before a reversal counts
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
        // No Pi command this long after power-on: show OFF instead of LOAD. The
        // Pi takes about 90 s to boot.
        constexpr uint32_t boot_timeout_ms = 180000;
        // {word_ms, flash_ms, blink_ms}: each stall word 1 s; the strip
        // flashes orange for 2 s after a trip.
        constexpr DisplayParams stall_display = {1000, 2000, 250};
        // A Pi command timestamp older than this is cleared, well inside the
        // 71.6 min wrap of the microsecond clock.
        constexpr uint32_t rx_expire_us = 60000000;
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

    namespace WATCHDOG
    {
        constexpr bool enabled = true;
        // Reset after this long without a refresh. Every context must beat
        // within it; the slowest, DotMatrix, loops every 50 ms.
#ifdef RCN_WDT_TIMEOUT_MS
        constexpr uint32_t timeout_ms = RCN_WDT_TIMEOUT_MS;
#else
        constexpr uint32_t timeout_ms = 100;
#endif
        // While an EEPROM save runs. A full flash sector is erased with
        // interrupts off (typical 45 ms, up to 400 ms) and then rewritten.
        constexpr uint32_t flash_timeout_ms = 1500;
    }

    namespace DRIVE_LOOP
    {
        constexpr uint32_t period_us = 1000; // drive tick, IntervalTimer
        // Below the Servo, PPM, hall and serial interrupts (default 128), so
        // pulse edges and RC timing are never delayed; above every thread.
        constexpr uint8_t irq_priority = 144;
    }

    namespace TEENSY_THREADS
    {
        constexpr uint16_t startup_delay = 2000; // ms
        constexpr uint16_t tick_ms = 1;          // worker thread yield
        constexpr uint16_t loop_tick_ms = 25;    // Arduino loop() yield
        constexpr uint16_t disp_tick_ms = 50;    // dot matrix refresh
        constexpr uint16_t info_tick_ms = 10;    // info builder poll
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
        constexpr float ki = 60.0f;
        // Reverse sits on a steep part of the ESC curve, where the feedforward
        // alone lands tenths of a m/s off; the integrator does the tracking.
        constexpr float ki_rev = 60.0f;
        constexpr float max_integral = 150.0f; // anti-windup limit
        constexpr float kd = 0.0f;
        constexpr float kd_filter_alpha = 0.05f;

        // Feedforward and gain scheduling.
        // On the 1000/1500/2000 ESC calibration (docs/test-log.md): forward fitted
        // open loop; reverse from the PWM the closed loop settles at, since
        // open-loop reverse points depend on how hard the launch was.
        constexpr float deadband_fwd_us = 69.0f;
        constexpr float deadband_rev_us = 94.0f;

        constexpr float kv_ff_fwd = 12.5f;
        constexpr float kv_ff_rev = 55.0f;

        constexpr float kp_boost = 60.0f;
        constexpr float err_threshold = 0.25f;

        // Start assist (src/start_assist.h). Motion counts once this many hall
        // intervals since the standstill span start_confirm_us; a twitch is about
        // four edges. Until then the output is at least the per-direction ramp.
        constexpr uint16_t start_confirm_intervals = 5;
        constexpr uint32_t start_confirm_us = 50000;
        // {from_us, max_us, ramp_us_per_s, timeout_us, rest_us, handover_mps,
        // handover_frac}; max_us 0 is off. Reverse holds the ramp until 0.5 m/s
        // or 80% of the target: handed over at the first edges, the rotor is too
        // slow for the running feedforward to carry it.
        constexpr StartAssistParams start_assist_fwd = {0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f};
        constexpr StartAssistParams start_assist_rev = {160.0f, 320.0f, 2000.0f, 600000, 300000, 0.5f, 0.8f};

        // Fleet values, used when the EEPROM holds no valid per-car set
        // (src/car_constants.h). Field order follows SpeedConstants.
        constexpr SpeedConstants speed_defaults = {
            deadband_fwd_us, kv_ff_fwd, deadband_rev_us, kv_ff_rev, kp, kp_boost, err_threshold,
            ki, ki_rev, max_integral, start_assist_rev.from_us, start_assist_rev.max_us};
    }

    namespace STALL_GUARD
    {
        constexpr bool enabled = true;
        // Battery current includes the Pi, about 2 A at idle. Normal reverse
        // starts peak near 16 A for up to 0.19 s; a locked rotor holds 12-16 A.
        // {trip_a, moving_mps, trip_us, cooldown_us, lockout_us, lockout_trips,
        // lockout_window_us}
        constexpr StallParams params = {10.0f, 0.1f, 500000, 2000000, 30000000, 3, 60000000};
    }

    // Raw-PWM test build (-DRCN_RAW_PWM, env teensy41_raw_pwm): a Pi command
    // drives the ESC open loop at 1500 + command * us_per_unit us, for
    // measuring the PWM-to-speed curve. RC driving is unchanged. Never ship it.
    namespace RAW_PWM_TEST
    {
        constexpr float us_per_unit = 1000.0f; // command 0.001 = 1 us
        constexpr float max_offset_us = 300.0f;
    }

    // ESC calibration build (-DRCN_ESC_CAL, env teensy41_esc_cal): holds the
    // ESC at one calibration step at a time, chosen over USB serial by
    // scripts/esc_cal.py; Pi and RC driving are off. The LED strip shows the
    // held PWM in purple. Never ship it.
    namespace ESC_CAL
    {
        // A step away from neutral falls back to neutral when USB serial goes
        // quiet this long (the script polls every 0.5 s).
        constexpr uint32_t link_timeout_ms = 2000;
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
