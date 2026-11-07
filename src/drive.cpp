#include "drive.h"

#include <Arduino.h>
#include <IntervalTimer.h>
#include <Servo.h>

#include "config.h"
#include "drive_map.h"
#include "profile.h"
#include "start_assist.h"
#include "watchdog.h"
#include "shared.h"

// ------------------------- Hall encoder --------------------------- //

namespace
{
  // ISR state, read under noInterrupts().
  volatile uint32_t lastTransitionTime = 0;
  volatile uint32_t intervals[cfg::ES2845S_4900KV_MOTOR::rpm_window] = {0};
  volatile uint8_t intervalCount = 0; // valid entries, up to rpm_window
  volatile uint8_t intervalIdx = 0;
  volatile uint8_t lastState = 0;
  volatile int8_t direction = 0;
  volatile uint16_t intervalsSinceStop = 0;
  volatile uint32_t firstEdgeTime = 0;
  HallDirFilter dirFilter; // ISR only

  void onHallChange()
  {
    using namespace cfg::ES2845S_4900KV_MOTOR;
    const uint32_t now = micros();
    const uint8_t state = (digitalRead(hall_a_pin) << 2) | (digitalRead(hall_b_pin) << 1) | digitalRead(hall_c_pin);
    if (state == lastState)
      return;

    const uint32_t dt = now - lastTransitionTime;
    lastTransitionTime = now;
    // From a standstill, take the first direction at once and drop the stale intervals.
    if (dt > stop_timeout_us)
    {
      dirFilter = HallDirFilter{};
      intervalCount = 0;
      intervalsSinceStop = 0;
      firstEdgeTime = now;
    }
    else
    {
      intervals[intervalIdx] = dt;
      intervalIdx = (intervalIdx + 1) % rpm_window;
      if (intervalCount < rpm_window)
        intervalCount++;
      if (intervalsSinceStop < UINT16_MAX)
        intervalsSinceStop++;
    }

    direction = filterHallDirection(dirFilter, hallDirection(lastState, state), dir_confirm_transitions);
    lastState = state;
  }

  float filteredRpm = 0.0f;

  // Signed ground speed, m/s, from the median of the last rpm_window hall
  // intervals, low-pass filtered. An open interval longer than the median
  // stands in for it, so the estimate falls while the motor slows between
  // edges; 0 once no edge has arrived within stop_timeout_us.
  float updateSpeedEstimate()
  {
    using namespace cfg::ES2845S_4900KV_MOTOR;

    uint32_t buf[rpm_window];
    noInterrupts();
    const uint32_t lastT = lastTransitionTime;
    const uint8_t n = intervalCount;
    const int8_t dir = direction;
    for (uint8_t i = 0; i < rpm_window; i++)
      buf[i] = intervals[i];
    interrupts();

    const uint32_t open = micros() - lastT;
    if (n == 0 || open > stop_timeout_us)
    {
      filteredRpm = 0.0f;
      return 0.0f;
    }

    uint32_t dt = medianInterval(buf, n);
    if (open > dt)
      dt = open;

    const float transitionsPerSec = 1000000.0f / dt;
    const float electricalRpm = (transitionsPerSec * 60.0f) / hall_states_per_erev;
    const float mechanicalRpm = electricalRpm / pole_pairs;

    filteredRpm = (filteredRpm * (1.0f - rpm_filter_alpha)) + (mechanicalRpm * dir * rpm_filter_alpha);

    const float wheelRpm = filteredRpm * cfg::CAR_SPECS::ratio_motor_to_transaxle * cfg::CAR_SPECS::ratio_transaxle_to_diff;
    return wheelRpm * cfg::CAR_SPECS::wheel_circumference / 60.0f;
  }
}

static MotionSinceStop readMotionSinceStop()
{
  noInterrupts();
  const uint32_t lastT = lastTransitionTime;
  MotionSinceStop m = {intervalsSinceStop, firstEdgeTime, false};
  interrupts();
  m.stopped = (uint32_t)(micros() - lastT) > cfg::ES2845S_4900KV_MOTOR::stop_timeout_us;
  return m;
}

void initHallEncoder()
{
  using namespace cfg::ES2845S_4900KV_MOTOR;
  for (uint8_t pin : {hall_a_pin, hall_b_pin, hall_c_pin})
  {
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), onHallChange, CHANGE);
  }
}

// -------------------------- Speed loop ---------------------------- //

namespace
{
  float speedIntegral = 0.0f;
  float speedLastErr = 0.0f;
  float speedFilteredDeriv = 0.0f;
  StartAssist startAssist;

  void resetSpeedLoop()
  {
    speedIntegral = 0.0f;
    speedLastErr = 0.0f;
    startAssist = StartAssist{};
  }

  // PI plus feedforward on wheel speed. The feedforward term carries the output
  // across the ESC deadband; the gain schedule adds authority on large error.
  // The output stays on the target's side of neutral: a forward pulse while
  // reversing (or the reverse) makes the ESC brake and jerk the drivetrain.
  uint16_t speedPID(float target_mps, float measured_mps, float dt, const MotionSinceStop &motion, uint32_t now_us)
  {
    using namespace cfg::PID_CTRL;
    using cfg::ESC_10BL60_G2::speed_pwm_max;
    using cfg::ESC_10BL60_G2::speed_pwm_min;
    using cfg::ESC_10BL60_G2::speed_pwm_neutral;

    const float err = target_mps - measured_mps;
    float ff_term = 0.0f;

    if (target_mps > cfg::ESC_10BL60_G2::cmd_deadband)
    {
      ff_term = deadband_fwd_us + kv_ff_fwd * target_mps;
    }
    else if (target_mps < -cfg::ESC_10BL60_G2::cmd_deadband)
    {
      ff_term = -deadband_rev_us + kv_ff_rev * target_mps;
    }

    const int8_t dir = target_mps > 0.0f ? 1 : (target_mps < 0.0f ? -1 : 0);
    const bool confirmed =
        motionConfirmed(motion, dir, measured_mps, now_us, start_confirm_intervals, start_confirm_us);
    const StartAssistParams &assist_params = dir > 0 ? start_assist_fwd : start_assist_rev;
    const StartAssistOutput assist = startAssistTick(startAssist, dir, confirmed,
                                                     handoverSpeedReached(target_mps, measured_mps, assist_params),
                                                     now_us, assist_params);
    if (assist.rest)
    {
      speedIntegral = 0.0f;
      speedLastErr = err;
      return speed_pwm_neutral;
    }

    // The boost adds acceleration on a large error in both directions. The
    // side-of-neutral clamp below keeps a noisy sample from swinging the
    // output into a brake pulse, and the start assist keeps twitch readings
    // out of the error.
    const float active_kp = fabsf(err) > err_threshold ? kp_boost : kp;
    const float p_term = active_kp * err;

    const float raw_deriv = (dt > 1e-4f) ? (err - speedLastErr) / dt : 0.0f;
    speedFilteredDeriv = (kd_filter_alpha * raw_deriv) + ((1.0f - kd_filter_alpha) * speedFilteredDeriv);
    const float d_term = kd * speedFilteredDeriv;
    speedLastErr = err;

    // Integrate only off the limits, or when the error pushes back into range.
    const float pwm_lo = target_mps > 0.0f ? (float)speed_pwm_neutral : (float)speed_pwm_min;
    const float pwm_hi = target_mps < 0.0f ? (float)speed_pwm_neutral : (float)speed_pwm_max;
    const float active_ki = dir < 0 ? ki_rev : ki;
    const float predicted_pwm = speed_pwm_neutral + ff_term + p_term + (active_ki * speedIntegral) + d_term;
    const bool hit_upper_limit = (predicted_pwm >= pwm_hi) && (err > 0);
    const bool hit_lower_limit = (predicted_pwm <= pwm_lo) && (err < 0);
    // Frozen while the start assist holds the output, so the hand-off at
    // confirmed motion does not carry a wound-up integral.
    const bool assisting = assist.offset_us > 0.0f;
    if (!hit_upper_limit && !hit_lower_limit && !assisting)
      speedIntegral += err * dt;
    speedIntegral = constrain(speedIntegral, -max_integral, max_integral);

    float final_pwm = speed_pwm_neutral + ff_term + p_term + (active_ki * speedIntegral) + d_term;
    if (assisting)
    {
      const float assist_pwm = speed_pwm_neutral + dir * assist.offset_us;
      final_pwm = dir > 0 ? fmaxf(final_pwm, assist_pwm) : fminf(final_pwm, assist_pwm);
    }
    return (uint16_t)constrain(final_pwm, pwm_lo, pwm_hi);
  }

  constexpr RcThrottleMap rc_throttle_map = {
      cfg::FLYSKY_IA6B_RX::pulse_min,
      cfg::ESC_10BL60_G2::speed_pwm_neutral,
      cfg::FLYSKY_IA6B_RX::pulse_max,
      cfg::ESC_10BL60_G2::rc_throt_rev_break,
      cfg::ESC_10BL60_G2::rc_throt_fwd_break,
      cfg::ESC_10BL60_G2::rc_throt_fwd_floor,
      cfg::ESC_10BL60_G2::rc_pwm_min,
      cfg::ESC_10BL60_G2::speed_pwm_neutral,
      cfg::ESC_10BL60_G2::rc_pwm_max,
  };

  bool pulseValid(uint16_t pulse)
  {
    return rcPulseValid(pulse, cfg::AUTO_DRIVE::rc_valid_min, cfg::AUTO_DRIVE::rc_valid_max);
  }

  // An absent or out-of-band channel reads as neither position.
  bool switchUp(uint16_t pulse)
  {
    return rcSwitchUp(pulse, cfg::AUTO_DRIVE::rc_valid_min, cfg::AUTO_DRIVE::rc_valid_max,
                      cfg::FLYSKY_IA6B_RX::sw_threshold);
  }

  bool switchDown(uint16_t pulse)
  {
    return rcSwitchDown(pulse, cfg::AUTO_DRIVE::rc_valid_min, cfg::AUTO_DRIVE::rc_valid_max,
                        cfg::FLYSKY_IA6B_RX::sw_threshold);
  }
}

// --------------------------- Actuation ---------------------------- //

namespace
{
  Servo steerServo, throtMot;
  IntervalTimer drive_timer;
  uint32_t last_tick_us = 0;

  // Last values that read cleanly, used when a write is in progress.
  RcChannels rc_last = {};
  DriveCommand cmd_last = {0.0f, 0.0f};
  PowerSample power_last = {0.0f, 0.0f};
  StallGuard stall_guard;

  volatile uint16_t esc_cal_pwm_us = cfg::ESC_10BL60_G2::speed_pwm_neutral;

  // For the serial trace, printed from loop().
  volatile float trace_target_mps = 0.0f;
  volatile float trace_measured_mps = 0.0f;
  volatile uint16_t trace_pwm = cfg::ESC_10BL60_G2::speed_pwm_neutral;

  void driveTick()
  {
#ifdef RCN_WDT_TEST
    if (wdt::test_hang == 'd')
      return;
#endif
    wdt::beat(wdt::drive_tick);
    wdt::tick();
    PROF_PERIOD(drive_loop);
    PROF_START(t_body);
    const uint32_t now = micros();
    const float dt = (float)(now - last_tick_us) * 1e-6f;
    last_tick_us = now;

    rc_channels.tryRead(rc_last);
    const uint16_t *ch = rc_last.us;
    const uint16_t steer_ch = ch[cfg::FLYSKY_IA6B_RX::ch_steer];
    const uint16_t throt_ch = ch[cfg::FLYSKY_IA6B_RX::ch_throt];

    // An unplugged or out-of-range receiver reads outside the pulse band; that
    // is no RC, not a command.
    const bool rc_signal = pulseValid(steer_ch) && pulseValid(throt_ch);
    // Low pulse = lever up on the FS-i6, the position it starts in. swA low is
    // slow; swD high is manual override. No receiver signal reads as neither:
    // fast, and the Pi in control.
    const bool slow_mode = switchDown(ch[cfg::FLYSKY_IA6B_RX::ch_speed_mode]);
    const bool override_on = switchUp(ch[cfg::FLYSKY_IA6B_RX::ch_manual_override]);

    uint16_t rc_steer = cfg::SERVO_D135F::angle_pwm_neutral;
    uint16_t rc_throt = cfg::ESC_10BL60_G2::speed_pwm_neutral;
    float rc_norm = 0.0f;
    bool rc_active = false;

    if (rc_signal)
    {
      rc_steer = steer_ch;
      rc_throt = rcThrottleToPwm(throt_ch, rc_throttle_map);
      rc_norm = rcThrottleToNorm(throt_ch, rc_throttle_map);
      rc_active =
          (abs((int)rc_throt - (int)cfg::ESC_10BL60_G2::speed_pwm_neutral) > cfg::AUTO_DRIVE::throt_override_band) ||
          (abs((int)rc_steer - (int)cfg::SERVO_D135F::angle_pwm_neutral) > cfg::AUTO_DRIVE::steer_override_band);
    }

    manual_override = override_on;
    rc_driving = rc_active;

    // Sticks centered or no receiver: a fresh Pi command drives, a stale one
    // falls to neutral. Override locks the Pi out entirely, so centered sticks
    // stop the car rather than handing it back to autonomy.
    const bool pi_drive = !rc_active && !override_on && piCommandFresh(cfg::UART::rx_timeout);

    uint16_t steer_out = cfg::SERVO_D135F::angle_pwm_neutral;
    float pi_motor = 0.0f;
    if (rc_active)
    {
      steer_out = rc_steer;
    }
    else if (pi_drive)
    {
      drive_cmd.tryRead(cmd_last);
      pi_motor = cmd_last.throttle;
      steer_out = mapNormToPwm(cmd_last.steer, cfg::SERVO_D135F::angle_pwm_min,
                               cfg::SERVO_D135F::angle_pwm_neutral, cfg::SERVO_D135F::angle_pwm_max);
    }

    const float measured_mps = updateSpeedEstimate();
    measured_speed_mps = measured_mps;

    // Slow mode rescales the whole stick onto the reduced band, so full travel
    // lands on slow_speed_mps exactly instead of clipping the top. It closes
    // the loop on the RC stick too, since the cap is a speed. Fast-mode RC and
    // the stale-command failsafe write PWM straight through, so an override
    // and a stop are both instant.
    const float speed_cap = slow_mode ? cfg::PID_CTRL::slow_speed_mps : cfg::PID_CTRL::max_speed_mps;
    const float db = cfg::ESC_10BL60_G2::cmd_deadband;

    bool closed_loop = false;
    float norm_cmd = 0.0f;
    if (rc_active)
    {
      closed_loop = slow_mode && fabsf(rc_norm) > db;
      norm_cmd = rc_norm;
    }
    else if (pi_drive && fabsf(pi_motor) > db)
    {
      closed_loop = true;
      norm_cmd = pi_motor;
    }

    float target_mps = 0.0f;
    uint16_t throt_out;
#ifdef RCN_RAW_PWM
    if (closed_loop && !rc_active)
    {
      throt_out = rawCmdToPwm(norm_cmd, db, cfg::RAW_PWM_TEST::us_per_unit, cfg::RAW_PWM_TEST::max_offset_us,
                              cfg::ESC_10BL60_G2::speed_pwm_min, cfg::ESC_10BL60_G2::speed_pwm_neutral,
                              cfg::ESC_10BL60_G2::speed_pwm_max);
      resetSpeedLoop();
    }
    else
#endif
    if (closed_loop)
    {
      target_mps = throttleToTargetMps(norm_cmd, db, cfg::PID_CTRL::min_speed_mps, speed_cap);
      throt_out = speedPID(target_mps, measured_mps, dt, readMotionSinceStop(), now);
    }
    else
    {
      throt_out = rc_active ? rc_throt : cfg::ESC_10BL60_G2::speed_pwm_neutral;
      resetSpeedLoop();
    }

#if !defined(RCN_ESC_CAL) && !defined(RCN_RAW_PWM)
    if (cfg::STALL_GUARD::enabled)
    {
      // Throttle only: a steering-only stick move is not a command to the motor.
      const bool rc_throttle =
          rc_active && abs((int)rc_throt - (int)cfg::ESC_10BL60_G2::speed_pwm_neutral) >
                           cfg::AUTO_DRIVE::throt_override_band;
      const bool command_active = rc_throttle || (pi_drive && fabsf(pi_motor) > db);
      power_sample.tryRead(power_last);
      const StallOutput stall =
          stallGuardTick(stall_guard, {power_last.amps, measured_mps, command_active}, now, cfg::STALL_GUARD::params);
      if (stall.tripped_now)
        stall_trip_ms = millis();
      stall_word = packStallWord(stall);
      if (stall.force_neutral)
      {
        throt_out = cfg::ESC_10BL60_G2::speed_pwm_neutral;
        target_mps = 0.0f;
        resetSpeedLoop();
      }
    }
#endif

#ifdef RCN_ESC_CAL
    // Calibration build: no driving; the ESC sees only the held step.
    steer_out = cfg::SERVO_D135F::angle_pwm_neutral;
    throt_out = esc_cal_pwm_us;
    target_mps = 0.0f;
    resetSpeedLoop();
#endif
    steerServo.writeMicroseconds(steer_out);
    throtMot.writeMicroseconds(throt_out);
    PROF_SECTION(drive_body, t_body);
    if (pi_drive)
      PROF_CMD_USED();

    trace_target_mps = target_mps;
    trace_measured_mps = measured_mps;
    trace_pwm = throt_out;
  }
}

void setEscCalPwm(uint16_t pwm_us)
{
  esc_cal_pwm_us = pwm_us;
}

uint16_t escCalPwmUs()
{
  return esc_cal_pwm_us;
}

void startDriveControl()
{
  steerServo.attach(cfg::SERVO_D135F::servo_pin);
  throtMot.attach(cfg::ESC_10BL60_G2::motor_pin);
  steerServo.writeMicroseconds(cfg::SERVO_D135F::angle_pwm_neutral);
  throtMot.writeMicroseconds(cfg::ESC_10BL60_G2::speed_pwm_neutral);

  last_tick_us = micros();
  drive_timer.priority(cfg::DRIVE_LOOP::irq_priority);
  drive_timer.begin(driveTick, cfg::DRIVE_LOOP::period_us);
}

namespace
{
  // Hundredths, sign kept, without float printf.
  int centi(float v)
  {
    return (int)(v * 100.0f + (v < 0.0f ? -0.5f : 0.5f));
  }
}

void printDriveTrace()
{
  if (!cfg::DEBUG::serial_plot)
    return;
  const int target = centi(trace_target_mps);
  const int measured = centi(trace_measured_mps);
  char line[96];
  const int n = snprintf(line, sizeof(line),
                         ">Target_MPS:%s%d.%02d\r\n>Measured_MPS:%s%d.%02d\r\n>Sent_PWM:%u\r\n",
                         target < 0 ? "-" : "", abs(target) / 100, abs(target) % 100,
                         measured < 0 ? "-" : "", abs(measured) / 100, abs(measured) % 100,
                         (unsigned)trace_pwm);
  // Skip the line when the USB buffer is full: with nothing reading the port
  // a blocking print stalls loop() and the watchdog resets the board.
  if (n > 0 && n < (int)sizeof(line) && Serial.availableForWrite() >= n)
    Serial.write(line, n);
}
