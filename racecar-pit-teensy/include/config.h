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
        constexpr bool      speed_demon             = true;
        
        constexpr float     percentThrot            = 0.25;
        constexpr float     percentRev              = 0.25;

        constexpr uint16_t  throt_center            = 1500;             // us
        constexpr uint16_t  throt_mag               = 750;              // us
        constexpr float     throt_slew_us           = 1.0;              // max Pi-throttle PWM change per control iter; ESC needs a ramp, not a step
    }

    namespace SERVO_D135F {
        constexpr uint8_t   servo_pin               = 2;
        constexpr bool      max_steer               = false;
        
        constexpr float     percent_left            = 1.0;
        constexpr float     percent_right           = 1.0;

        constexpr uint16_t  steer_center            = 1500;             // us
        constexpr uint16_t  steer_mag               = 475;              // us
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
        constexpr uint32_t  stop_timeout_us             = 100000;           // no hall edge in this window -> RPM 0
    }

    namespace AUTO_DRIVE {
        // Drive priority: the RC transmitter overrides the Pi whenever a stick
        // leaves its neutral band. With the sticks centered (or no receiver),
        // the Pi command stream drives as the fallback.
        constexpr float     throt_limit                 = 0.25;             // Pi authority: fraction of throt_mag
        constexpr float     steer_limit                 = 1.0;              // Pi authority: fraction of steer_mag
        // RC-active detection: |RC-commanded PWM - center| beyond this -> human driving.
        constexpr uint16_t  throt_override_band         = 40;               // us around throt_center
        constexpr uint16_t  steer_override_band         = 40;               // us around steer_center
        // Plausible RC pulse band; outside it the receiver is treated as absent.
        constexpr uint16_t  rc_valid_min                = 900;              // us
        constexpr uint16_t  rc_valid_max                = 2100;             // us
    }

    namespace TEENSY_THREADS {
        constexpr uint16_t  startup_delay               = 2000;             // ms
    }

    namespace CAR_SPECS {
        constexpr float     wheel_diameter              = 0.072;                            // m
        constexpr float     wheel_circumference         = wheel_diameter * 3.1415;          // m
        constexpr float     ratio_motor_to_transaxle    = 18.0 / 61.0;
        constexpr float     ratio_transaxle_to_diff     = 15.0 / 37.0;
    }

}