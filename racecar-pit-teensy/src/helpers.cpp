#include <Arduino.h>

#include <Servo.h>
#include <FastLED.h>
#include <TeensyThreads.h>
#include <PPMReader.h>
#include <SPI.h>
#include <Wire.h>
#include <INA226.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <MD_Parola.h>

#include "battery.h"
#include "config.h"
#include "crc16.h"
#include "drive_map.h"
#include "helpers.h"
#include "Mutexes.h"
#include "packets.h"

// ----------------------- Error state ------------------------------ //
uint8_t errorFlag = 0;

// ------------------- UART Teensy <-> Pi --------------------------- //
TXPacket txPacketPing, txPacketPong;
RxPacket rxPacketPing, rxPacketPong;
volatile uint32_t lastRxTime = 0;

// --------------------------- RGB LED ------------------------------ //
CRGB leds[cfg::RGB_LED_WS2812B::num_leds];

// ------------------- Display command from Pi ---------------------- //
bool dotmatrix_cmd[192] = {false};
uint8_t led_cmd[cfg::RGB_LED_WS2812B::num_leds * 3] = {0};
uint8_t system_state = 0;

// -------------------- Servo and motor ----------------------------- //
static Servo steerServo, throtMot;
float cmd_setpoint[2];
float cmd_pwm[2];
uint16_t max_pwm_throt;
uint16_t max_pwm_rev;

// ----------------------- FlySky IA6B Rx --------------------------- //
// AETR order, 1000-2000 us, 0 when a channel is absent or past failsafe:
// {aileron, elevator, throttle, rudder, aux1, aux2, unused, unused}.
uint16_t channelVals[cfg::FLYSKY_IA6B_RX::channel_count] = {0, 0, 0, 0, 0, 0, 0, 0};

// ------------- INA226 voltage and current sensor ------------------ //
static INA226 INA(cfg::INA226_SENSOR::i2c_addy);
float power[3] = {0, 0, 0};

// -------------------- LSM9DS1 9-DOF IMU --------------------------- //
static Adafruit_LSM9DS1 lsmimu = Adafruit_LSM9DS1();
static sensors_event_t a, m, g, temp;
float imuVals[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

// ---------------------- MAX7219 dot matrix ------------------------ //
static MD_Parola mx = MD_Parola(HARDWARE_TYPE, cfg::MAX_7219_DOT_MATRIX::cs_pin, cfg::MAX_7219_DOT_MATRIX::num_matrix);

// ------------------- Hall-effect encoder -------------------------- //
volatile uint32_t lastTransitionTime = 0;
volatile uint32_t deltaTime = 0;
volatile uint8_t currentState = 0;
volatile uint8_t lastState = 0;
volatile int8_t direction = 0;
volatile float lastRPM = 0;

// ------------------------ Thread mutexes -------------------------- //
Threads::Mutex CmdMutex;
Threads::Mutex RxMutex;
Threads::Mutex I2CMutex;
Threads::Mutex SPIMutex;
Threads::Mutex PowerMutex;
Threads::Mutex ErrMutex;
Threads::Mutex ImuMutex;
Threads::Mutex TransmitMutex;
Threads::Mutex ReceiveMutex;
Threads::Mutex RecPongMutex;

// -------------------------- Telemetry ----------------------------- //

void fillTxBuff()
{
  while (true)
  {
    // Snapshot each source under its own lock, then commit under TransmitMutex
    // so UARTTx never copies a half-written frame.
    uint16_t rc_local[cfg::FLYSKY_IA6B_RX::channel_count];
    float imu_local[9];
    float curr, volt;

    RxMutex.lock();
    memcpy(rc_local, channelVals, sizeof(rc_local));
    RxMutex.unlock();

    ImuMutex.lock();
    memcpy(imu_local, imuVals, sizeof(imu_local));
    ImuMutex.unlock();

    PowerMutex.lock();
    curr = power[0];
    volt = power[1];
    PowerMutex.unlock();

    TransmitMutex.lock();
    memcpy(txPacketPing.RxReceiver, rc_local, sizeof(rc_local));
    memcpy(txPacketPing.IMU, imu_local, sizeof(imu_local));
    txPacketPing.VoltAndCurr[0] = (uint16_t)(volt * 1000.0f);
    // Amps packed as mA so sub-amp resolution survives the uint16.
    txPacketPing.VoltAndCurr[1] = (uint16_t)(curr < 0.0f ? 0.0f : curr * 1000.0f);
    txPacketPing.Encoder = getCurrentSpeed();
    txPacketPing.Timestamp = (int32_t)micros();
    TransmitMutex.unlock();

    threads.delay(cfg::TEENSY_THREADS::tick_ms);
  }
}

void UARTTx()
{
  uint32_t lastTime = 0;
  const uint32_t interval = 1000000UL / cfg::UART::tx_freq;

  while (true)
  {
    if (micros() - lastTime >= interval)
    {
      TransmitMutex.lock();
      memcpy(&txPacketPong, &txPacketPing, sizeof(txPacketPing));
      TransmitMutex.unlock();
      // Checksum the exact bytes going on the wire, magic included.
      txPacketPong.Checksum = crc16_ccitt((const uint8_t *)&txPacketPong, sizeof(TXPacket) - 2);
      Serial1.write((byte *)&txPacketPong, sizeof(txPacketPong));
      lastTime = micros();
    }
    threads.delay(cfg::TEENSY_THREADS::tick_ms);
  }
}

// --------------------------- Commands ----------------------------- //

void fillRxData()
{
  RecPongMutex.lock();
  float servo = rxPacketPong.ServoAng;
  float motor = rxPacketPong.MotorSpeed;
  uint8_t state = rxPacketPong.SystemState;

  CmdMutex.lock();
  cmd_setpoint[0] = servo;
  cmd_setpoint[1] = motor;
  memcpy(dotmatrix_cmd, rxPacketPong.DotMatrix, sizeof(dotmatrix_cmd));
  memcpy(led_cmd, rxPacketPong.LED, sizeof(led_cmd));
  system_state = state;
  CmdMutex.unlock();

  RecPongMutex.unlock();
}

void UARTRx()
{
  uint8_t window[4] = {0, 0, 0, 0};
  uint32_t magicNumRx = 0;

  while (true)
  {
    ReceiveMutex.lock();
    while (Serial1.available())
    {
      // Slide a 4-byte window over the stream until the magic word lands.
      for (size_t i = 0; i < sizeof(window) - 1; i++)
      {
        window[i] = window[i + 1];
      }
      window[3] = Serial1.read();

      memcpy(&magicNumRx, window, sizeof(window));
      if (magicNumRx != cfg::UART::rx_magic_num)
        continue;

      Serial1.readBytes((byte *)&rxPacketPing, sizeof(rxPacketPing));

      // magicLE is little-endian on the Teensy, so its bytes match the wire.
      uint32_t magicLE = cfg::UART::rx_magic_num;
      uint16_t crc = crc16_ccitt((const uint8_t *)&magicLE, sizeof(magicLE));
      crc = crc16_ccitt((const uint8_t *)&rxPacketPing, sizeof(RxPacket) - 2, crc);

      if (cfg::UART::verify_rx_crc && crc != rxPacketPing.Checksum)
        continue;

      RecPongMutex.lock();
      memcpy(&rxPacketPong, &rxPacketPing, sizeof(rxPacketPong));
      RecPongMutex.unlock();
      fillRxData();
      lastRxTime = micros();
    }
    ReceiveMutex.unlock();
    threads.delay(cfg::TEENSY_THREADS::tick_ms);
  }
}

// ------------------------- Hall encoder --------------------------- //

void onHallChange()
{
  uint32_t cTime = micros();

  uint8_t a_pin = digitalRead(cfg::ES2845S_4900KV_MOTOR::hall_a_pin);
  uint8_t b_pin = digitalRead(cfg::ES2845S_4900KV_MOTOR::hall_b_pin);
  uint8_t c_pin = digitalRead(cfg::ES2845S_4900KV_MOTOR::hall_c_pin);

  currentState = (a_pin << 2) | (b_pin << 1) | c_pin;

  if (currentState == lastState)
    return;

  deltaTime = cTime - lastTransitionTime;
  lastTransitionTime = cTime;

  int8_t d = hallDirection(lastState, currentState);
  if (d != 0)
  {
    direction = d;
  }
  lastState = currentState;
}

void getRPM()
{
  noInterrupts();
  uint32_t dt = deltaTime;
  uint32_t lastT = lastTransitionTime;
  int8_t current_dir = direction;
  interrupts();

  // Without the timeout, deltaTime holds its last value forever and the encoder
  // reports a ghost speed from a stale interval.
  if (dt == 0 || (micros() - lastT) > cfg::ES2845S_4900KV_MOTOR::stop_timeout_us)
  {
    lastRPM = 0;
    return;
  }

  constexpr uint8_t N = cfg::ES2845S_4900KV_MOTOR::rpm_window;
  static uint32_t dt_buffer[N] = {0};
  static uint8_t dt_idx = 0;

  dt_buffer[dt_idx] = dt;
  dt_idx = (dt_idx + 1) % N;

  uint32_t sum_dt = 0;
  for (uint8_t i = 0; i < N; i++)
  {
    sum_dt += dt_buffer[i];
  }
  uint32_t avg_dt = sum_dt / N;

  float transitionsPerSec = 1000000.0f / avg_dt;
  float electricalRPM = (transitionsPerSec * 60.0f) / cfg::ES2845S_4900KV_MOTOR::hall_states_per_erev;
  float mechanicalRPM = electricalRPM / cfg::ES2845S_4900KV_MOTOR::pole_pairs;

  float instantaneousRPM = mechanicalRPM * current_dir;
  constexpr float alpha = cfg::ES2845S_4900KV_MOTOR::rpm_filter_alpha;
  lastRPM = (lastRPM * (1.0f - alpha)) + (instantaneousRPM * alpha);
}

float getCurrentSpeed()
{
  getRPM();
  float out_rpm = lastRPM * cfg::CAR_SPECS::ratio_motor_to_transaxle * cfg::CAR_SPECS::ratio_transaxle_to_diff;
  return out_rpm * cfg::CAR_SPECS::wheel_circumference / 60.0f;
}

// ----------------------- Sensor threads --------------------------- //

void ReadCurrAndVolt()
{
  I2CMutex.lock();
  while (!INA.begin())
  {
    ErrMutex.lock();
    errorFlag = 1;
    ErrMutex.unlock();
  }
  I2CMutex.unlock();

  ErrMutex.lock();
  errorFlag = 0;
  ErrMutex.unlock();

  I2CMutex.lock();
  INA.setAverage(cfg::INA226_SENSOR::avg_set);
  INA.setMaxCurrentShunt(cfg::INA226_SENSOR::max_curr_expected, cfg::INA226_SENSOR::shunt_resistance);
  I2CMutex.unlock();

  while (true)
  {
    I2CMutex.lock();
    PowerMutex.lock();
    power[0] = INA.getCurrent();
    power[1] = INA.getBusVoltage();
    power[2] = INA.getPower();
    PowerMutex.unlock();
    I2CMutex.unlock();

    threads.delay(cfg::TEENSY_THREADS::tick_ms);
  }
}

void ReadIMUStates()
{
  I2CMutex.lock();
  while (!lsmimu.begin())
  {
    ErrMutex.lock();
    errorFlag = 2;
    ErrMutex.unlock();
  }
  I2CMutex.unlock();

  ErrMutex.lock();
  errorFlag = 0;
  ErrMutex.unlock();

  I2CMutex.lock();
  lsmimu.setupAccel(cfg::LSM9DS1_IMU::accel_range, cfg::LSM9DS1_IMU::poll_rate);
  lsmimu.setupMag(cfg::LSM9DS1_IMU::mag_gain);
  lsmimu.setupGyro(cfg::LSM9DS1_IMU::gyro_scale);
  I2CMutex.unlock();

  while (true)
  {
    I2CMutex.lock();
    lsmimu.getEvent(&a, &m, &g, &temp);
    I2CMutex.unlock();

    ImuMutex.lock();
    imuVals[0] = a.acceleration.x;
    imuVals[1] = a.acceleration.y;
    imuVals[2] = a.acceleration.z;
    imuVals[3] = g.gyro.x;
    imuVals[4] = g.gyro.y;
    imuVals[5] = g.gyro.z;
    imuVals[6] = m.magnetic.x;
    imuVals[7] = m.magnetic.y;
    imuVals[8] = m.magnetic.z;
    ImuMutex.unlock();

    threads.delay(cfg::TEENSY_THREADS::tick_ms);
  }
}

void RxReceive()
{
  PPMReader ppm(cfg::FLYSKY_IA6B_RX::ppm_pin, cfg::FLYSKY_IA6B_RX::channel_count);

  while (true)
  {
    for (uint8_t i = 1; i <= cfg::FLYSKY_IA6B_RX::channel_count; i++)
    {
      uint16_t value = ppm.latestValidChannelValue(i, 0);
      RxMutex.lock();
      channelVals[i - 1] = value;
      RxMutex.unlock();
    }
    threads.delay(cfg::TEENSY_THREADS::tick_ms);
  }
}

// -------------------------- Actuation ----------------------------- //

static float speedIntegral = 0.0f;
static float speedLastErr = 0.0f;

// PI plus feedforward on wheel speed. The feedforward term carries the output
// across the ESC deadband; the gain schedule adds authority on large error.
static uint16_t speedPID(float target_mps, float measured_mps, float dt)
{
  using namespace cfg::PID_CTRL;

  float err = target_mps - measured_mps;
  float ff_term = 0.0f;

  if (target_mps > cfg::ESC_10BL60_G2::cmd_deadband)
  {
    ff_term = deadband_fwd_us + kv_ff_fwd * target_mps;
  }
  else if (target_mps < -cfg::ESC_10BL60_G2::cmd_deadband)
  {
    ff_term = -deadband_rev_us + kv_ff_rev * target_mps;
    if (measured_mps > -stiction_speed_mps)
    {
      ff_term -= stiction_kick_rev_us;
    }
  }

  float active_kp = (fabsf(err) > err_threshold) ? kp_boost : kp;
  float p_term = active_kp * err;

  float raw_deriv = (dt > 1e-4f) ? (err - speedLastErr) / dt : 0.0f;
  static float filtered_deriv = 0.0f;
  filtered_deriv = (kd_filter_alpha * raw_deriv) + ((1.0f - kd_filter_alpha) * filtered_deriv);
  float d_term = kd * filtered_deriv;
  speedLastErr = err;

  // Integrate only off the limits, or when the error pushes back into range.
  float predicted_pwm = cfg::ESC_10BL60_G2::speed_pwm_neutral + ff_term + p_term + (ki * speedIntegral) + d_term;
  bool hit_upper_limit = (predicted_pwm >= cfg::ESC_10BL60_G2::speed_pwm_max) && (err > 0);
  bool hit_lower_limit = (predicted_pwm <= cfg::ESC_10BL60_G2::speed_pwm_min) && (err < 0);

  if (!hit_upper_limit && !hit_lower_limit)
  {
    speedIntegral += err * dt;
  }
  speedIntegral = constrain(speedIntegral, -max_integral, max_integral);

  float final_pwm = cfg::ESC_10BL60_G2::speed_pwm_neutral + ff_term + p_term + (ki * speedIntegral) + d_term;
  return (uint16_t)constrain(final_pwm, (float)cfg::ESC_10BL60_G2::speed_pwm_min, (float)cfg::ESC_10BL60_G2::speed_pwm_max);
}

// Normalized [-1,1] throttle to a speed setpoint spanning min_speed_mps to
// max_speed_mps, so the first usable stick position clears the ESC deadband.
static float throttleToTargetMps(float motor)
{
  using namespace cfg::PID_CTRL;
  const float db = cfg::ESC_10BL60_G2::cmd_deadband;

  if (motor > 0.0f)
  {
    float scale = (motor - db) / (1.0f - db);
    return min_speed_mps + scale * (max_speed_mps - min_speed_mps);
  }
  float scale = (motor + db) / (-1.0f + db);
  return -min_speed_mps - scale * (max_speed_mps - min_speed_mps);
}

void testPWM()
{
  steerServo.attach(cfg::SERVO_D135F::servo_pin);
  throtMot.attach(cfg::ESC_10BL60_G2::motor_pin);

  steerServo.writeMicroseconds(cfg::SERVO_D135F::angle_pwm_neutral);
  throtMot.writeMicroseconds(cfg::ESC_10BL60_G2::speed_pwm_neutral);

  uint32_t lastUs = micros();
  float throt_out = cfg::ESC_10BL60_G2::speed_pwm_neutral;

  while (true)
  {
    uint32_t now = micros();
    float dt = (float)(now - lastUs) * 1e-6f;
    lastUs = now;

    uint16_t ch[cfg::FLYSKY_IA6B_RX::channel_count];
    RxMutex.lock();
    memcpy(ch, channelVals, sizeof(ch));
    RxMutex.unlock();

    const uint16_t steer_ch = ch[cfg::FLYSKY_IA6B_RX::ch_steer];
    const uint16_t throt_ch = ch[cfg::FLYSKY_IA6B_RX::ch_throt];

    // An unplugged or out-of-range receiver reads outside the pulse band; that
    // is no RC, not a command.
    bool rc_signal = (steer_ch >= cfg::AUTO_DRIVE::rc_valid_min && steer_ch <= cfg::AUTO_DRIVE::rc_valid_max) &&
                     (throt_ch >= cfg::AUTO_DRIVE::rc_valid_min && throt_ch <= cfg::AUTO_DRIVE::rc_valid_max);

    float rc_steer = cfg::SERVO_D135F::angle_pwm_neutral;
    float rc_throt = cfg::ESC_10BL60_G2::speed_pwm_neutral;
    bool rc_active = false;
    bool pi_drive = false;

    if (rc_signal)
    {
      rc_steer = steer_ch;

      if (cfg::ESC_10BL60_G2::speed_demon)
      {
        bool reverse = ch[cfg::FLYSKY_IA6B_RX::ch_drive_mode] < 1500;
        rc_throt = reverse
                       ? map(throt_ch, cfg::FLYSKY_IA6B_RX::pulse_min, cfg::FLYSKY_IA6B_RX::pulse_max,
                             cfg::ESC_10BL60_G2::speed_pwm_neutral, cfg::ESC_10BL60_G2::speed_pwm_min)
                       : map(throt_ch, cfg::FLYSKY_IA6B_RX::pulse_min, cfg::FLYSKY_IA6B_RX::pulse_max,
                             cfg::ESC_10BL60_G2::speed_pwm_neutral, cfg::ESC_10BL60_G2::speed_pwm_max);
      }
      else if (throt_ch > cfg::ESC_10BL60_G2::rc_throt_fwd_break)
      {
        rc_throt = map(throt_ch, cfg::ESC_10BL60_G2::speed_pwm_neutral, cfg::FLYSKY_IA6B_RX::pulse_max,
                       cfg::ESC_10BL60_G2::rc_throt_fwd_floor, max_pwm_throt);
      }
      else if (throt_ch < cfg::ESC_10BL60_G2::rc_throt_rev_break)
      {
        rc_throt = map(throt_ch, cfg::FLYSKY_IA6B_RX::pulse_min, cfg::ESC_10BL60_G2::speed_pwm_neutral,
                       max_pwm_rev, cfg::ESC_10BL60_G2::rc_throt_rev_break);
      }
      rc_throt = constrain(rc_throt, (float)max_pwm_rev, (float)max_pwm_throt);

      rc_active =
          (fabsf(rc_throt - cfg::ESC_10BL60_G2::speed_pwm_neutral) > cfg::AUTO_DRIVE::throt_override_band) ||
          (fabsf(rc_steer - cfg::SERVO_D135F::angle_pwm_neutral) > cfg::AUTO_DRIVE::steer_override_band);
    }

    if (rc_active)
    {
      cmd_pwm[0] = rc_steer;
      cmd_pwm[1] = rc_throt;
    }
    else
    {
      // Sticks centered or no receiver: a fresh Pi command drives, a stale one
      // falls to neutral.
      bool fresh = (lastRxTime != 0) &&
                   ((micros() - lastRxTime) < (cfg::UART::rx_timeout * 1000UL));

      if (fresh)
      {
        CmdMutex.lock();
        float servo = cmd_setpoint[0];
        float motor = cmd_setpoint[1];
        CmdMutex.unlock();

        cmd_pwm[0] = mapNormToPwm(servo, cfg::SERVO_D135F::angle_pwm_min,
                                  cfg::SERVO_D135F::angle_pwm_neutral, cfg::SERVO_D135F::angle_pwm_max);
        cmd_pwm[1] = motor;
        pi_drive = true;
      }
      else
      {
        cmd_pwm[0] = cfg::SERVO_D135F::angle_pwm_neutral;
        cmd_pwm[1] = cfg::ESC_10BL60_G2::speed_pwm_neutral;
      }
    }

    // Only the Pi throttle runs closed loop. RC and the stale-command failsafe
    // write PWM straight through, so an override and a stop are both instant.
    float measured_mps = getCurrentSpeed();
    float target_mps = 0.0f;

    if (pi_drive && fabsf(cmd_pwm[1]) > cfg::ESC_10BL60_G2::cmd_deadband)
    {
      target_mps = throttleToTargetMps(cmd_pwm[1]);
      throt_out = speedPID(target_mps, measured_mps, dt);
    }
    else
    {
      throt_out = pi_drive ? cfg::ESC_10BL60_G2::speed_pwm_neutral : cmd_pwm[1];
      speedIntegral = 0.0f;
      speedLastErr = 0.0f;
    }

    steerServo.writeMicroseconds(cmd_pwm[0]);
    throtMot.writeMicroseconds(throt_out);

    if (cfg::DEBUG::serial_plot)
    {
      Serial.print(">Target_MPS:");
      Serial.println(target_mps);
      Serial.print(">Measured_MPS:");
      Serial.println(measured_mps);
      Serial.print(">Sent_PWM:");
      Serial.println(throt_out);
    }

    threads.delay(cfg::TEENSY_THREADS::tick_ms);
  }
}

// --------------------------- Displays ----------------------------- //

void DotMatrix()
{
  SPIMutex.lock();
  mx.begin();
  mx.setIntensity(cfg::MAX_7219_DOT_MATRIX::intensity);
  mx.displayClear();
  mx.displayText("RACECAR NEO", PA_CENTER, 50, 1500, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
  SPIMutex.unlock();

  MD_MAX72XX *gfx = mx.getGraphicObject();
  bool splashDone = false;
  bool wasPi = false;
  int16_t lastMode = -1;

  while (true)
  {
    bool fresh = (lastRxTime != 0) &&
                 ((micros() - lastRxTime) < (cfg::DISP::cmd_timeout_ms * 1000UL));

    static bool bmp[192];
    uint8_t state;
    CmdMutex.lock();
    memcpy(bmp, dotmatrix_cmd, sizeof(bmp));
    state = system_state;
    CmdMutex.unlock();

    if (fresh && (state & cfg::SYS_STATE::dotmatrix_active))
    {
      // The raw setPoint column axis runs opposite to the MD_Parola text
      // engine, so reverse it or the Pi's text renders mirrored.
      constexpr uint16_t W = cfg::MAX_7219_DOT_MATRIX::cols;
      SPIMutex.lock();
      gfx->control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
      for (uint8_t r = 0; r < cfg::MAX_7219_DOT_MATRIX::rows; r++)
        for (uint16_t c = 0; c < W; c++)
          gfx->setPoint(r, (W - 1) - c, bmp[r * W + c]);
      gfx->control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
      SPIMutex.unlock();
      wasPi = true;
      lastMode = -1; // force the glyph to redraw when the Pi releases the display
      threads.delay(cfg::TEENSY_THREADS::disp_tick_ms);
    }
    else if (!splashDone)
    {
      SPIMutex.lock();
      bool done = mx.displayAnimate();
      SPIMutex.unlock();
      if (done)
        splashDone = true;
      threads.delay(cfg::TEENSY_THREADS::splash_tick_ms);
    }
    else
    {
      // Idle default: the drive-mode glyph.
      uint8_t mode = state & cfg::SYS_STATE::mode_mask;
      if (wasPi || (int16_t)mode != lastMode)
      {
        const char *glyph = (mode == cfg::SYS_STATE::mode_manual) ? "M"
                            : (mode == cfg::SYS_STATE::mode_auto) ? "A"
                                                                  : "I";
        SPIMutex.lock();
        mx.displayClear();
        mx.displayText(glyph, PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
        mx.displayReset();
        SPIMutex.unlock();
        lastMode = mode;
        wasPi = false;
      }
      SPIMutex.lock();
      mx.displayAnimate();
      SPIMutex.unlock();
      threads.delay(cfg::TEENSY_THREADS::disp_tick_ms);
    }
  }
}

void ledStuffs()
{
  const int N = cfg::RGB_LED_WS2812B::num_leds;

  uint8_t state;
  static uint8_t local_led[cfg::RGB_LED_WS2812B::num_leds * 3];
  CmdMutex.lock();
  memcpy(local_led, led_cmd, sizeof(local_led));
  state = system_state;
  CmdMutex.unlock();

  bool fresh = (lastRxTime != 0) &&
               ((micros() - lastRxTime) < (cfg::DISP::cmd_timeout_ms * 1000UL));

  // Gated on driver_starting, not comms freshness, so a Teensy booting into an
  // already-running Pi goes straight to the idle bar.
  static uint32_t load_start = 0;
  static bool was_starting = false;
  bool starting = fresh && (state & cfg::SYS_STATE::driver_starting);
  if (starting && !was_starting)
    load_start = millis();
  was_starting = starting;

  for (int i = 0; i < N; i++)
    leds[i] = CRGB::Black;

  if (starting)
  {
    uint32_t elapsed = millis() - load_start;
    int lit = map(elapsed, 0, cfg::DISP::led_load_ms, 0, N);
    for (int i = 0; i < lit && i < N; i++)
      leds[i] = CHSV((uint8_t)(i * 255 / N + elapsed / 20), 255, 255);
  }
  else if (fresh && (state & cfg::SYS_STATE::led_active))
  {
    for (int i = 0; i < N; i++)
      leds[i] = CRGB(local_led[i * 3], local_led[i * 3 + 1], local_led[i * 3 + 2]);
  }
  else
  {
    // Idle default: red charge bar filling from both ends.
    PowerMutex.lock();
    float volt = power[1];
    PowerMutex.unlock();
    float mv = constrain(volt * 1000.0f, 0.0f, 65535.0f);
    int half = (batterySocPercent((uint16_t)mv) * (N / 2)) / 100;
    for (int i = 0; i < half; i++)
    {
      leds[i] = CRGB::Red;
      leds[N - 1 - i] = CRGB::Red;
    }
  }

  FastLED.show();
}
