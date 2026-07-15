#pragma once
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>

namespace cfg {
    
    namespace UART {
        constexpr uint32_t  baud_rate               = 921600;
        constexpr uint32_t  tx_magic_num            = 0xDEADBEEF;
        constexpr uint32_t  rx_magic_num            = 0xBEEFDEAD;
        constexpr uint16_t  tx_freq                 = 200;              // Hz
        constexpr uint16_t  rx_freq                 = 60;               // Hz
        constexpr uint32_t  rx_timeout              = 50;               // ms
        constexpr bool      verify_rx_crc           = true;             // drop command frames failing CRC-16
    }

    namespace RGB_LED_WS2812B {
        constexpr uint16_t      num_leds                = 84;
        constexpr uint8_t       led_pin                 = 9;
        constexpr uint8_t       brightness              = 25;
    }

    namespace ESC_10BL60_G2 {
        constexpr uint8_t   motor_pin               = 3;
        constexpr bool      speed_demon             = false;

        constexpr float     percentThrot            = 0.25;
        constexpr float     percentRev              = 0.25;

        // Configurable PWM safety bounds. A normalized [-1,1] Pi command maps
        // piecewise: -1 -> min, 0 -> neutral, +1 -> max. Tighten min/max to
        // protect the drivetrain; the Pi no longer scales the range (its
        // throttle scaling is 1, so the Teensy owns the physical limits).
        constexpr uint16_t  speed_pwm_min           = 800;              // us, full-reverse bound; deeper than a symmetric 1000 to clear the ESC reverse deadband
        constexpr uint16_t  speed_pwm_neutral       = 1500;             // us, ESC arm / neutral
        constexpr uint16_t  speed_pwm_max           = 2200;             // us, full-forward bound

        constexpr float     cmd_deadband            = 0.01;             // Ignore tiny joystick noise
    }

    namespace SERVO_D135F {
        constexpr uint8_t   servo_pin               = 2;

        // Steering PWM safety bounds; same [-1,1] -> min/neutral/max mapping as
        // the ESC. -1 = full left, +1 = full right.
        constexpr uint16_t  angle_pwm_min           = 1025;             // us, full-left bound
        constexpr uint16_t  angle_pwm_neutral       = 1500;             // us, centered
        constexpr uint16_t  angle_pwm_max           = 1975;             // us, full-right bound
    }

    namespace FLYSKY_IA6B_RX {
        constexpr uint8_t   ppm_pin                 = 5;
        constexpr uint8_t   channel_count           = 8;
    }

    namespace INA226_SENSOR {
        constexpr uint8_t   i2c_addy                    = 0x40;
        constexpr uint8_t   avg_set                     = 2;
        constexpr float     max_curr_expected           = 8.0;               // Amps
        constexpr float     shunt_resistance            = 0.002;             // Ohms   
    }

    namespace LSM9DS1_IMU {
        constexpr Adafruit_LSM9DS1::lsm9ds1AccelRange_t     accel_range         = Adafruit_LSM9DS1::lsm9ds1AccelRange_t::LSM9DS1_ACCELRANGE_8G;
        constexpr Adafruit_LSM9DS1::lsm9ds1AccelDataRate_t  poll_rate           = Adafruit_LSM9DS1::lsm9ds1AccelDataRate_t::LSM9DS1_ACCELDATARATE_119HZ;
        constexpr Adafruit_LSM9DS1::lsm9ds1MagGain_t        mag_gain            = Adafruit_LSM9DS1::lsm9ds1MagGain_t::LSM9DS1_MAGGAIN_8GAUSS;
        constexpr Adafruit_LSM9DS1::lsm9ds1GyroScale_t      gyro_scale          = Adafruit_LSM9DS1::lsm9ds1GyroScale_t::LSM9DS1_GYROSCALE_500DPS;
    }

    namespace MAX_7219_DOT_MATRIX {
        constexpr uint8_t   cs_pin                      = 10;
        constexpr uint8_t   num_matrix                  = 3;
        constexpr uint8_t   intensity                   = 8;
    }

    namespace ES2845S_4900KV_MOTOR {
        constexpr uint8_t   hall_a_pin                  = 30;
        constexpr uint8_t   hall_b_pin                  = 31;
        constexpr uint8_t   hall_c_pin                  = 32;
        constexpr uint8_t   mot_temp_pin                = A17;
        constexpr uint32_t  stop_timeout_us             = 30000;           // no hall edge in this window -> RPM 0
    }

    namespace AUTO_DRIVE {
        // Drive priority: the RC transmitter overrides the Pi whenever a stick
        // leaves its neutral band. With the sticks centered (or no receiver),
        // the Pi command stream drives as the fallback. The Pi command spans
        // the full [-1,1] range now; the speed_pwm_*/angle_pwm_* bounds cap it.
        // RC-active detection: |RC-commanded PWM - neutral| beyond this -> human driving.
        constexpr uint16_t  throt_override_band         = 40;               // us around speed_pwm_neutral
        constexpr uint16_t  steer_override_band         = 40;               // us around angle_pwm_neutral
        // Plausible RC pulse band; outside it the receiver is treated as absent.
        constexpr uint16_t  rc_valid_min                = 900;              // us
        constexpr uint16_t  rc_valid_max                = 2100;             // us
    }

    namespace DISP {
        // Dot matrix + LED behavior. When the Pi stops sending fresh command
        // frames (driver down), the Teensy falls back to idle defaults.
        constexpr uint32_t  cmd_timeout_ms             = 1000;             // Pi command staleness -> idle defaults
        constexpr uint32_t  led_load_ms                = 10000;            // rainbow "loading" sweep duration on Pi connect
    }

    // Bit layout of RxPacket.SystemState (Pi -> Teensy): the low two bits carry
    // the drive mode for the dot-matrix glyph; two flag bits say whether the Pi
    // is actively driving each display so the Teensy knows when to show defaults.
    namespace SYS_STATE {
        constexpr uint8_t   mode_mask                   = 0x03;             // 0 idle, 1 manual, 2 auto
        constexpr uint8_t   mode_idle                   = 0;
        constexpr uint8_t   mode_manual                 = 1;
        constexpr uint8_t   mode_auto                   = 2;
        constexpr uint8_t   dotmatrix_active            = 0x04;             // Pi is commanding the dot matrix
        constexpr uint8_t   led_active                  = 0x08;             // Pi is commanding the LED strip
        constexpr uint8_t   driver_starting             = 0x10;             // Pi driver just launched: play the LED loading sweep
    }

    namespace TEENSY_THREADS {
        constexpr uint16_t  startup_delay               = 2000;             // ms
    }

    namespace CAR_SPECS {
        constexpr float     wheel_diameter              = 0.072;                            // m
        constexpr float     wheel_circumference         = wheel_diameter * PI;          // m
        constexpr float     ratio_motor_to_transaxle    = 18.0 / 61.0;
        constexpr float     ratio_transaxle_to_diff     = 15.0 / 37.0;
    }

    namespace PID_CTRL {
        // Target physical speeds in meters per second (m/s)
        constexpr float max_speed_mps = 3.0f; // Top speed at full joystick
        constexpr float min_speed_mps = 0.5f; // Crawl speed at minimum joystick
        
        // PID Tuning Constants
        constexpr float kp = 30.0f;
        constexpr float ki = 30.0f;
        constexpr float max_integral = 150.0f; // Anti-windup limit, heavy to bridge deadband gap
        constexpr float kd = 0.0f;
        constexpr float kd_filter_alpha = 0.05f;

        // Feedfoward and Gain Scheduling Constants
        constexpr float deadband_us = 50.0f;
        constexpr float kv_ff = 20.0f;

        constexpr float kp_boost = 60.0f;
        constexpr float err_threshold = 0.5f;
    }

}
