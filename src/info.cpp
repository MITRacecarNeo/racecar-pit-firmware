#include "info.h"

#include "battery.h"
#include "config.h"
#include "version.h"

void InfoWriter::putChar(char c)
{
  // One byte stays free for the terminating NUL.
  if (!ok_ || len_ + 1 >= cap_)
  {
    ok_ = false;
    return;
  }
  buf_[len_++] = c;
  buf_[len_] = '\0';
}

void InfoWriter::put(const char *s)
{
  while (*s)
    putChar(*s++);
}

void InfoWriter::putUint(uint64_t v, uint8_t min_digits)
{
  char digits[21];
  uint8_t n = 0;
  do
  {
    digits[n++] = (char)('0' + v % 10);
    v /= 10;
  } while (v > 0 || n < min_digits);
  while (n > 0)
    putChar(digits[--n]);
}

void InfoWriter::text(const char *key, const char *value)
{
  put(key);
  putChar('=');
  put(value);
  putChar('\n');
}

void InfoWriter::integer(const char *key, int32_t value)
{
  put(key);
  putChar('=');
  if (value < 0)
  {
    putChar('-');
    putUint((uint64_t)(-(int64_t)value));
  }
  else
  {
    putUint((uint64_t)value);
  }
  putChar('\n');
}

void InfoWriter::version(const char *key, uint8_t major, uint8_t minor, uint8_t patch)
{
  put(key);
  putChar('=');
  putUint(major);
  putChar('.');
  putUint(minor);
  putChar('.');
  putUint(patch);
  putChar('\n');
}

void InfoWriter::fixed(const char *key, float value, uint8_t decimals)
{
  if (decimals > 6)
    decimals = 6;
  uint64_t scale = 1;
  for (uint8_t i = 0; i < decimals; i++)
    scale *= 10;

  const double v = (double)value;
  const bool negative = v < 0.0;
  const uint64_t scaled = (uint64_t)((negative ? -v : v) * (double)scale + 0.5);

  put(key);
  putChar('=');
  if (negative && scaled != 0)
    putChar('-');
  putUint(scaled / scale);
  if (decimals > 0)
  {
    putChar('.');
    putUint(scaled % scale, decimals);
  }
  putChar('\n');
}

uint16_t lsm9ds1GyroOdrHz(const Lsm9ds1Registers &r)
{
  static const uint16_t odr[8] = {0, 15, 60, 119, 238, 476, 952, 0};
  return odr[(r.ctrl1_g >> 5) & 0x07];
}

uint16_t lsm9ds1AccelOdrHz(const Lsm9ds1Registers &r)
{
  const uint16_t gyro = lsm9ds1GyroOdrHz(r);
  if (gyro != 0)
    return gyro;
  static const uint16_t odr[8] = {0, 10, 50, 119, 238, 476, 952, 0};
  return odr[(r.ctrl6_xl >> 5) & 0x07];
}

float lsm9ds1MagOdrHz(const Lsm9ds1Registers &r)
{
  if ((r.ctrl3_m & 0x03) != 0)
    return 0.0f; // not in continuous mode
  if (r.ctrl1_m & 0x02)
  {
    // FAST_ODR: the rate follows the X/Y operating mode.
    static const float fast[4] = {1000.0f, 560.0f, 300.0f, 155.0f};
    return fast[(r.ctrl1_m >> 5) & 0x03];
  }
  static const float odr[8] = {0.625f, 1.25f, 2.5f, 5.0f, 10.0f, 20.0f, 40.0f, 80.0f};
  return odr[(r.ctrl1_m >> 2) & 0x07];
}

uint8_t lsm9ds1MagRangeGauss(const Lsm9ds1Registers &r)
{
  static const uint8_t fs[4] = {4, 8, 12, 16};
  return fs[(r.ctrl2_m >> 5) & 0x03];
}

namespace
{
  // INA226 library averaging enum index to sample count.
  constexpr uint16_t ina226_avg_samples[] = {1, 4, 16, 64, 128, 256, 512, 1024};
  static_assert(cfg::INA226_SENSOR::avg_set < sizeof(ina226_avg_samples) / sizeof(ina226_avg_samples[0]),
                "INA226 averaging index out of range");
}

size_t writeFirmwareInfo(const InfoStatus &status, char *buf, size_t cap)
{
  using namespace cfg;
  InfoWriter w(buf, cap);

  w.integer("info", info::schema_version);

  w.version("fw.version", fw::version_major, fw::version_minor, fw::version_patch);
  w.text("fw.label", FW_STR(FW_LABEL));
  w.text("fw.git", FW_STR(FW_GIT_HASH));
  w.integer("fw.git_dirty", FW_GIT_DIRTY ? 1 : 0);
  w.text("fw.board", "teensy41");
#ifdef RCN_PROFILE
  w.integer("fw.profile", 1);
#else
  w.integer("fw.profile", 0);
#endif
  w.integer("fw.protocol", wire::protocol_version);
#if defined(RCN_ESC_CAL)
  w.text("fw.speed_control", "esc_cal");
#elif defined(RCN_RAW_PWM)
  w.text("fw.speed_control", "raw_pwm");
#else
  w.text("fw.speed_control", "closed_loop");
#endif

  w.integer("link.baud", (int32_t)UART::baud_rate);
  w.integer("link.telemetry_hz", UART::tx_freq);
  w.integer("link.cmd_timeout_ms", (int32_t)UART::rx_timeout);
  w.integer("link.cmd_crc", UART::verify_rx_crc ? 1 : 0);

  w.integer("steer.pwm_min_us", SERVO_D135F::angle_pwm_min);
  w.integer("steer.pwm_center_us", SERVO_D135F::angle_pwm_neutral);
  w.integer("steer.pwm_max_us", SERVO_D135F::angle_pwm_max);

  w.integer("esc.pwm_min_us", ESC_10BL60_G2::speed_pwm_min);
  w.integer("esc.pwm_neutral_us", ESC_10BL60_G2::speed_pwm_neutral);
  w.integer("esc.pwm_max_us", ESC_10BL60_G2::speed_pwm_max);
  w.integer("esc.rc_pwm_min_us", ESC_10BL60_G2::rc_pwm_min);
  w.integer("esc.rc_pwm_max_us", ESC_10BL60_G2::rc_pwm_max);

  w.fixed("speed.min_mps", PID_CTRL::min_speed_mps, 3);
  w.fixed("speed.max_mps", PID_CTRL::max_speed_mps, 3);
  w.fixed("speed.slow_mps", PID_CTRL::slow_speed_mps, 3);
  w.fixed("speed.deadband", ESC_10BL60_G2::cmd_deadband, 3);

  w.fixed("pid.kp", PID_CTRL::kp, 3);
  w.fixed("pid.ki", PID_CTRL::ki, 3);
  w.fixed("pid.kd", PID_CTRL::kd, 3);
  w.fixed("pid.max_integral", PID_CTRL::max_integral, 3);
  w.fixed("pid.kp_boost", PID_CTRL::kp_boost, 3);
  w.fixed("pid.boost_error_mps", PID_CTRL::err_threshold, 3);
  w.fixed("pid.ff_fwd_us", PID_CTRL::deadband_fwd_us, 1);
  w.fixed("pid.ff_rev_us", PID_CTRL::deadband_rev_us, 1);
  w.fixed("pid.kv_ff_fwd", PID_CTRL::kv_ff_fwd, 3);
  w.fixed("pid.kv_ff_rev", PID_CTRL::kv_ff_rev, 3);
  w.fixed("pid.stiction_kick_rev_us", PID_CTRL::stiction_kick_rev_us, 1);
  w.fixed("pid.stiction_speed_mps", PID_CTRL::stiction_speed_mps, 3);

  constexpr float gear_ratio = CAR_SPECS::ratio_motor_to_transaxle * CAR_SPECS::ratio_transaxle_to_diff;
  constexpr uint16_t edges_per_rev = ES2845S_4900KV_MOTOR::pole_pairs * ES2845S_4900KV_MOTOR::hall_states_per_erev;
  w.fixed("drivetrain.wheel_diameter_m", CAR_SPECS::wheel_diameter, 4);
  w.fixed("drivetrain.gear_ratio", gear_ratio, 6);
  w.integer("encoder.edges_per_motor_rev", edges_per_rev);
  w.fixed("encoder.m_per_edge", CAR_SPECS::wheel_circumference * gear_ratio / edges_per_rev, 6);
  w.integer("encoder.rpm_window", ES2845S_4900KV_MOTOR::rpm_window);
  w.fixed("encoder.filter_alpha", ES2845S_4900KV_MOTOR::rpm_filter_alpha, 3);
  w.integer("encoder.stop_timeout_us", (int32_t)ES2845S_4900KV_MOTOR::stop_timeout_us);

  w.integer("rc.ch_steer", FLYSKY_IA6B_RX::ch_steer);
  w.integer("rc.ch_throttle", FLYSKY_IA6B_RX::ch_throt);
  w.integer("rc.ch_speed_mode", FLYSKY_IA6B_RX::ch_speed_mode);
  w.integer("rc.ch_manual_override", FLYSKY_IA6B_RX::ch_manual_override);
  w.integer("rc.switch_threshold_us", FLYSKY_IA6B_RX::sw_threshold);
  w.integer("rc.valid_min_us", AUTO_DRIVE::rc_valid_min);
  w.integer("rc.valid_max_us", AUTO_DRIVE::rc_valid_max);
  w.integer("rc.override_throttle_us", AUTO_DRIVE::throt_override_band);
  w.integer("rc.override_steer_us", AUTO_DRIVE::steer_override_band);

  w.text("imu.model", "LSM9DS1");
  w.integer("imu.accel_range_g", LSM9DS1_IMU::accel_range_g);
  w.integer("imu.gyro_range_dps", LSM9DS1_IMU::gyro_range_dps);
  w.integer("imu.mag_range_gauss", LSM9DS1_IMU::mag_range_gauss);
  // Read back from the chip; 0 until the IMU answers.
  const Lsm9ds1Registers &r = status.imu_regs;
  w.integer("imu.gyro_odr_hz", r.read ? lsm9ds1GyroOdrHz(r) : 0);
  w.integer("imu.accel_odr_hz", r.read ? lsm9ds1AccelOdrHz(r) : 0);
  w.fixed("imu.mag_odr_hz", r.read ? lsm9ds1MagOdrHz(r) : 0.0f, 3);
  w.integer("imu.mag_range_read_gauss", r.read ? lsm9ds1MagRangeGauss(r) : 0);
  w.integer("imu.reg_ctrl1_g", r.ctrl1_g);
  w.integer("imu.reg_ctrl6_xl", r.ctrl6_xl);
  w.integer("imu.reg_ctrl1_m", r.ctrl1_m);
  w.integer("imu.reg_ctrl2_m", r.ctrl2_m);
  w.integer("imu.reg_ctrl3_m", r.ctrl3_m);
  w.integer("imu.read_hz", (int32_t)(1000000UL / LSM9DS1_IMU::read_period_us));
  w.text("imu.accel_unit", "m/s2");
  w.text("imu.gyro_unit", "rad/s");
  w.text("imu.mag_unit", "uT");

  w.text("power.sensor", "INA226");
  w.fixed("power.shunt_ohm", INA226_SENSOR::shunt_resistance, 4);
  w.fixed("power.max_current_a", INA226_SENSOR::max_curr_expected, 1);
  w.integer("power.avg_samples", ina226_avg_samples[INA226_SENSOR::avg_set]);
  w.integer("power.poll_ms", INA226_SENSOR::poll_ms);
  w.integer("battery.cells", 2);
  w.integer("battery.empty_mv", batt_empty_mv);
  w.integer("battery.full_mv", batt_full_mv);
  w.integer("i2c.clock_hz", (int32_t)I2C::clock_hz);

  w.integer("display.leds", RGB_LED_WS2812B::num_leds);
  w.integer("display.dot_rows", MAX_7219_DOT_MATRIX::rows);
  w.integer("display.dot_cols", MAX_7219_DOT_MATRIX::cols);

  w.integer("watchdog.timeout_ms", (int32_t)WATCHDOG::timeout_ms);
  w.text("status.imu", status.imu_ok ? "ok" : "missing");
  w.text("status.power", status.power_ok ? "ok" : "missing");
  w.text("status.watchdog", !WATCHDOG::enabled ? "off" : status.watchdog_armed ? "armed" : "starting");
  w.text("status.last_reset", status.last_reset ? status.last_reset : "unknown");
  w.integer("status.reset_flags", (int32_t)status.reset_flags);

  return w.length();
}
