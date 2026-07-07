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

#include "helpers.h"
#include "packets.h"
#include "Mutexes.h"
#include "config.h"

// ----------------------- Variables -------------------------------- //
uint8_t errorFlag = 0, errType;

// ------------- UART-TEENSY-PI --------------- //
TXPacket txPacketPing, txPacketPong;
RxPacket rxPacketPing, rxPacketPong;

uint32_t magicNumRx;
uint8_t window[4] = {0, 0, 0, 0};
volatile uint32_t lastRxTime = 0;

// --------------------------- RGB LED ---------------------------- //
CRGB leds[cfg::RGB_LED_WS2812B::num_leds];
CRGBPalette16 currrentPalette = RainbowStripeColors_p;
uint8_t colorIdx = 1;
uint8_t numleds;

// ------------------- Display command from Pi -------------------- //
bool    dotmatrix_cmd[192] = {false};
uint8_t led_cmd[cfg::RGB_LED_WS2812B::num_leds * 3] = {0};
uint8_t system_state = 0;

// -------------- Servo and Motor ------------ //
Servo steerServo, throtMot;
float cmd_setpoint[2];
float cmd_pwm[2];
// ------------- ESC_10BL60_G2 --------------- //
uint16_t  max_pwm_throt;
uint16_t  max_pwm_rev;

// -------------- SERVO_D135F ---------------- //    
uint16_t  max_pwm_left;
uint16_t  max_pwm_right;

// ----------------------------- FlySky IA6B Rx --------------------- //
// Range between 1000 - 2000 us {Aileron, Elevator, Throttle, Rudder, SWA, SWB, SWC, SWD, Ch9, Ch10} //
uint16_t channelVals[cfg::FLYSKY_IA6B_RX::channel_count] =       {0,       0,        0,        0,      0,   0,   0,   0};
uint8_t   drive_mode_sw;

// --------------------------- INA226 Voltage and Current Sensor ------------------------ //
INA226 INA(cfg::INA226_SENSOR::i2c_addy);
float power[3] = {0, 0, 0};  // Current (mA), Voltage (V), Power (mW)

// ---------------------------- LSM9DS1 9-DOF IMU ----------------------------------------//
Adafruit_LSM9DS1 lsmimu = Adafruit_LSM9DS1();
sensors_event_t a, m, g, temp;
float imuVals[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

//------------------------------ MAX 7219 Dot Matrix -------------------------------------//
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
MD_Parola mx = MD_Parola(HARDWARE_TYPE, cfg::MAX_7219_DOT_MATRIX::cs_pin, cfg::MAX_7219_DOT_MATRIX::num_matrix);

// --------------------------------------- Hall - Effect Encoder ---------------------------//
volatile uint32_t lastTransitionTime = 0;
volatile uint32_t deltaTime = 0;
volatile uint8_t currentState = 0;
volatile uint8_t lastState = 0;
volatile int8_t direction = 0;
volatile float lastRPM;
volatile int8_t lastDir;

// ----------------- Thread Mutexes ----------------------- //
Threads::Mutex CmdMutex;                // for use with cmd_packet}{}
Threads::Mutex RxMutex;                 // for use with channelVals[]
Threads::Mutex I2CMutex;                // for use with I2C bus
Threads::Mutex SPIMutex;                // for use with SPI bus
Threads::Mutex PowerMutex;              // for use with power[]
Threads::Mutex ErrMutex;                // for use with errorFlag
Threads::Mutex ImuMutex;                // for use with imuVals[]
Threads::Mutex HallMutex;               // for use with currentState
Threads::Mutex TransmitMutex;           // for use with txPacketPing
Threads::Mutex RecieveMutex;            // for use with rxPacketPing
Threads::Mutex TransPongMutex;          // for use with txPacketPong
Threads::Mutex RecPongMutex;            // for use with rxPacketPong
Threads::Mutex SerMutex;                // for use with Serial1


// -------------------- functions ------------------------ //

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection). Matches the
// Pi driver's pit_protocol.crc16_ccitt so both ends agree on the checksum.
uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc)
{
  for(size_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for(uint8_t b = 0; b < 8; b++)
    {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

void fillTxBuff()
{
  while(1)
  {
    // Snapshot each source under its own lock, then commit to the packet under
    // TransmitMutex so UARTTx never copies a half-written frame.
    uint16_t rc_local[cfg::FLYSKY_IA6B_RX::channel_count];
    float    imu_local[9];
    float    curr, volt;

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
    txPacketPing.VoltAndCurr[0] = (uint16_t)(volt * 1000.0f);           // bus voltage, mV
    // INA226 getCurrent() is amps; pack as mA so sub-amp resolution survives the
    // uint16 (0..65535 mA covers the 8 A shunt range). Clamp negatives to 0.
    txPacketPing.VoltAndCurr[1] = (uint16_t)(curr < 0.0f ? 0.0f : curr * 1000.0f);
    txPacketPing.Encoder        = getCurrentSpeed();                    // vehicle speed, m/s
    txPacketPing.Timestamp      = (int32_t)micros();
    TransmitMutex.unlock();

    threads.delay(1);
  }
}

void UARTTx()
{
  uint32_t lastTime = 0;
  const uint32_t interval = 1000000/200;  //1 second / 200 Hz
  while(true){
    if(micros() - lastTime >= interval)
    {
      TransmitMutex.lock();
      //(where it needs to go, what needs to go there, size of what needs to go)
      memcpy(&txPacketPong, &txPacketPing, sizeof(txPacketPing));
      TransmitMutex.unlock();
      // Checksum the exact bytes going on the wire: whole packet up to the
      // trailing 2-byte Checksum field (magic included).
      txPacketPong.Checksum = crc16_ccitt((const uint8_t *)&txPacketPong, sizeof(TXPacket) - 2);
      Serial1.write((byte *)&txPacketPong, sizeof(txPacketPong));
      lastTime = micros();
    }
    threads.delay(1);
  }


}

// Map a normalized [-1,1] command to PWM microseconds, piecewise about neutral.
uint16_t mapNormToPwm(float n, uint16_t lo, uint16_t neu, uint16_t hi)
{
  n = constrain(n, -1.0f, 1.0f);
  float pwm = (n >= 0.0f) ? (neu + n * ((float)hi - (float)neu))
                          : (neu + n * ((float)neu - (float)lo));
  return (uint16_t)constrain(pwm, (float)lo, (float)hi);
}

void fillRxData()
{
  RecPongMutex.lock();
  float   servo = rxPacketPong.ServoAng;
  float   motor = rxPacketPong.MotorSpeed;
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
  while(true)
  {
    RecieveMutex.lock();
    while(Serial1.available())
    {
      // Serial.println("I am Here!");
      for(uint i = 0; i < sizeof(window)/sizeof(window[0]) - 1; i++)
      {
        window[i] = window[i+1];
      }
      window[3] = Serial1.read();
      
      memcpy(&magicNumRx, &window, sizeof(window));
      if(magicNumRx == cfg::UART::rx_magic_num)
      {
        // Serial.println("BEEF DEAD");
        Serial1.readBytes((byte *)&rxPacketPing, sizeof(rxPacketPing));

        // CRC over the wire bytes: 4 magic bytes + body up to the trailing
        // 2-byte Checksum field. magicLE is little-endian on the Teensy, so
        // its bytes match the on-wire magic order.
        uint32_t magicLE = cfg::UART::rx_magic_num;
        uint16_t crc = crc16_ccitt((const uint8_t *)&magicLE, sizeof(magicLE));
        crc = crc16_ccitt((const uint8_t *)&rxPacketPing, sizeof(RxPacket) - 2, crc);

        if(!cfg::UART::verify_rx_crc || crc == rxPacketPing.Checksum)
        {
          RecPongMutex.lock();
          memcpy(&rxPacketPong, &rxPacketPing, sizeof(rxPacketPong));
          RecPongMutex.unlock();
          fillRxData();
          lastRxTime = micros();
        }
      }
    }
    RecieveMutex.unlock();
    threads.delay(1);
  }
}

int8_t getDirection(uint8_t prev, uint8_t curr) {
    static const uint8_t forwardSeq[6] = {1, 3, 2, 6, 4, 5};
    
    for (int i = 0; i < 6; i++) {
        if (forwardSeq[i] == prev) {
          int next = forwardSeq[(i + 1) % 6];
          int prevInSeq = forwardSeq[(i + 5) % 6];
          if (curr == next) return -1;
          if (curr == prevInSeq) return 1;
        }
    }
    return 0; // invalid/glitch transition
}

void onHallChange()
{
  uint32_t cTime = micros();

  uint8_t a = digitalRead(cfg::ES2845S_4900KV_MOTOR::hall_a_pin);
  uint8_t b = digitalRead(cfg::ES2845S_4900KV_MOTOR::hall_b_pin);
  uint8_t c = digitalRead(cfg::ES2845S_4900KV_MOTOR::hall_c_pin);

  currentState = (a << 2) | (b << 1) | c;

  if(currentState != lastState)
  {
    deltaTime = cTime - lastTransitionTime;
    lastTransitionTime = cTime;
    direction = getDirection(lastState, currentState);
    lastState = currentState;
  }
  else
  {
    direction = 0;
  }
}

void getRPM() {
    noInterrupts();
    uint32_t dt = deltaTime;
    uint32_t lastT = lastTransitionTime;
    interrupts();

    // No hall edge within the timeout -> wheel stopped. Without this, deltaTime
    // holds its last value forever and the encoder reports a ghost speed.
    if(dt == 0 || (micros() - lastT) > cfg::ES2845S_4900KV_MOTOR::stop_timeout_us)
    {
      lastRPM = 0;
      return;
    }

    float transitionsPerSec = 1000000.0f / dt;
    float electricalRPM = (transitionsPerSec * 60.0f) / 6.0f;
    float mechanicalRPM = electricalRPM / 2.0f;
    lastRPM = mechanicalRPM;
}


float getCurrentSpeed()
{
  getRPM();
  float inp_rpm = lastRPM;
  // Serial.println(inp_rpm);
  float out_rpm = inp_rpm * cfg::CAR_SPECS::ratio_motor_to_transaxle * cfg::CAR_SPECS::ratio_transaxle_to_diff;
  // Serial.println(out_rpm);
  float speed = out_rpm * cfg::CAR_SPECS::wheel_circumference / 60;
  return speed;
}

void InnerControlLoop()
{
  while(true)
  {
    Serial.print("Dir: " + String(direction) + "\t");
    Serial.println(getCurrentSpeed());
    threads.delay(20);
  }
}



void ReadCurrAndVolt()
{
  I2CMutex.lock();
  while(!INA.begin())
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
  INA.setAverage(2);
  INA.setMaxCurrentShunt(8, 0.002);       // to be calibrated
  I2CMutex.unlock();  
  
  while(1)
  {
    I2CMutex.lock();
    PowerMutex.lock();
    power[0] = INA.getCurrent();
    power[1] = INA.getBusVoltage();
    power[2] = INA.getPower();
    PowerMutex.unlock();
    I2CMutex.unlock();

    threads.delay(1);
  }
}

void ReadIMUStates()
{
  
  I2CMutex.lock();
  while(!lsmimu.begin())
  {
    ErrMutex.lock();
    errorFlag = 2;
    Serial.println("Cannot Find LSM9DS1 IMU!! Retrying..");
    ErrMutex.unlock();
  }
  I2CMutex.unlock();
  
  ErrMutex.lock();
  errorFlag = 0;
  ErrMutex.unlock();
  
  I2CMutex.lock();
  lsmimu.setupAccel(lsmimu.LSM9DS1_ACCELRANGE_8G, lsmimu.LSM9DS1_ACCELDATARATE_119HZ);
  lsmimu.setupMag(lsmimu.LSM9DS1_MAGGAIN_8GAUSS);
  lsmimu.setupGyro(lsmimu.LSM9DS1_GYROSCALE_500DPS);
  I2CMutex.unlock();
  
  while(1)
  {
    I2CMutex.lock();
    lsmimu.getEvent(&a, &m, &g, &temp);   // (accel, mag, gyro, temp)
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

    threads.delay(1);
  }

}

void RxReceive()
{
  PPMReader ppm(cfg::FLYSKY_IA6B_RX::ppm_pin, cfg::FLYSKY_IA6B_RX::channel_count);
  
  while(1)
  {
    for(uint8_t i = 1; i <= cfg::FLYSKY_IA6B_RX::channel_count; i++)
    {
      uint16_t value = ppm.latestValidChannelValue(i, 0);
      RxMutex.lock();
      channelVals[i-1] = value;
      RxMutex.unlock();
    }
    threads.delay(1);
  }
}

void testPWM()
{  
  steerServo.attach(cfg::SERVO_D135F::servo_pin);
  throtMot.attach(cfg::ESC_10BL60_G2::motor_pin);

  steerServo.writeMicroseconds(cfg::SERVO_D135F::angle_pwm_neutral);
  throtMot.writeMicroseconds(cfg::ESC_10BL60_G2::speed_pwm_neutral);

  while(1)
  {
    uint16_t ch[cfg::FLYSKY_IA6B_RX::channel_count];
    RxMutex.lock();
    memcpy(ch, channelVals, sizeof(ch));
    RxMutex.unlock();

    // Is a live RC signal present on the sticks we read?
    bool rc_signal = (ch[0] >= cfg::AUTO_DRIVE::rc_valid_min && ch[0] <= cfg::AUTO_DRIVE::rc_valid_max) &&
                     (ch[2] >= cfg::AUTO_DRIVE::rc_valid_min && ch[2] <= cfg::AUTO_DRIVE::rc_valid_max);

    float rc_steer = cfg::SERVO_D135F::angle_pwm_neutral;
    float rc_throt = cfg::ESC_10BL60_G2::speed_pwm_neutral;
    bool  rc_active = false;
    bool  pi_drive = false;   // true only when the Pi command drives the throttle

    if(rc_signal)
    {
      // What the RC transmitter would command (the manual mapping).
      rc_steer = ch[0];

      if(cfg::ESC_10BL60_G2::speed_demon)
      {
        drive_mode_sw = (ch[4] < 1500) ? true : false;
        rc_throt = drive_mode_sw ? map(ch[2], 1000, 2000, cfg::ESC_10BL60_G2::speed_pwm_neutral, cfg::ESC_10BL60_G2::speed_pwm_min)
                                 : map(ch[2], 1000, 2000, cfg::ESC_10BL60_G2::speed_pwm_neutral, cfg::ESC_10BL60_G2::speed_pwm_max);
      }
      else
      {
        //Throttle deadzone 1140 < 1500 < 1575
        if(ch[2] > 1525)      rc_throt = map(ch[2], 1500, 2000, 1575, max_pwm_throt);
        else if(ch[2] < 1475) rc_throt = map(ch[2], 1000, 1500, max_pwm_rev, 1475);
        else                  rc_throt = cfg::ESC_10BL60_G2::speed_pwm_neutral;
        rc_throt = (rc_throt > max_pwm_throt) ? max_pwm_throt : rc_throt;
        rc_throt = (rc_throt < max_pwm_rev)   ? max_pwm_rev   : rc_throt;
      }

      // Human is driving if either stick output leaves its neutral band.
      rc_active =
        (fabsf(rc_throt - cfg::ESC_10BL60_G2::speed_pwm_neutral) > cfg::AUTO_DRIVE::throt_override_band) ||
        (fabsf(rc_steer - cfg::SERVO_D135F::angle_pwm_neutral)   > cfg::AUTO_DRIVE::steer_override_band);
    }

    if(rc_active)
    {
      // RC override: the transmitter has priority over the Pi.
      cmd_pwm[0] = rc_steer;
      cmd_pwm[1] = rc_throt;
    }
    else
    {
      // Fallback to the Pi: sticks centered, or no receiver. Actuate a fresh
      // command (normalized [-1,1] passthrough); neutral if the Pi is silent.
      // The Pi sends the full [-1,1] range; the speed_pwm_*/angle_pwm_* bounds
      // cap it. +1 -> max, 0 -> neutral, -1 -> min (verify polarity on hardware;
      // flip pit.yaml steering_sign/speed_sign if a stick drives the wrong way).
      bool fresh = (lastRxTime != 0) &&
                   ((micros() - lastRxTime) < (cfg::UART::rx_timeout * 1000UL));

      if(fresh)
      {
        CmdMutex.lock();
        float servo = cmd_setpoint[0];
        float motor = cmd_setpoint[1];
        CmdMutex.unlock();

        cmd_pwm[0] = mapNormToPwm(servo, cfg::SERVO_D135F::angle_pwm_min,
                                  cfg::SERVO_D135F::angle_pwm_neutral, cfg::SERVO_D135F::angle_pwm_max);
        cmd_pwm[1] = mapNormToPwm(motor, cfg::ESC_10BL60_G2::speed_pwm_min,
                                  cfg::ESC_10BL60_G2::speed_pwm_neutral, cfg::ESC_10BL60_G2::speed_pwm_max);
        pi_drive = true;
      }
      else
      {
        cmd_pwm[0] = cfg::SERVO_D135F::angle_pwm_neutral;
        cmd_pwm[1] = cfg::ESC_10BL60_G2::speed_pwm_neutral;
      }
    }

    // Steering actuates immediately. The Pi's throttle ramps toward the target
    // (the ESC ignores a step from neutral). RC and the neutral failsafe apply
    // directly, so a human override and the stale-command stop are both instant.
    static float throt_out = cfg::ESC_10BL60_G2::speed_pwm_neutral;
    bool rev_cmd = cmd_pwm[1] < cfg::ESC_10BL60_G2::speed_pwm_neutral;
    if(pi_drive && !(cfg::ESC_10BL60_G2::rev_snap && rev_cmd))
    {
      float d = cmd_pwm[1] - throt_out;
      float step = cfg::ESC_10BL60_G2::throt_slew_us;
      throt_out += (d > step) ? step : (d < -step ? -step : d);
    }
    else
    {
      throt_out = cmd_pwm[1];
    }

    steerServo.writeMicroseconds(cmd_pwm[0]);
    throtMot.writeMicroseconds(throt_out);

    threads.delay(1);
  }

}

void DotMatrix()
{
  SPIMutex.lock();
  mx.begin();
  mx.setIntensity(cfg::MAX_7219_DOT_MATRIX::intensity);
  mx.displayClear();
  // Intro splash on boot; scrolls once, then the loop switches to the glyph.
  mx.displayText("RACECAR NEO", PA_CENTER, 50, 1500, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
  SPIMutex.unlock();

  MD_MAX72XX* gfx = mx.getGraphicObject();
  bool    splashDone = false;
  bool    wasPi = false;
  int16_t lastMode = -1;

  while(1)
  {
    bool fresh = (lastRxTime != 0) &&
                 ((micros() - lastRxTime) < (cfg::DISP::cmd_timeout_ms * 1000UL));

    static bool     bmp[192];
    uint8_t         state;
    CmdMutex.lock();
    memcpy(bmp, dotmatrix_cmd, sizeof(bmp));
    state = system_state;
    CmdMutex.unlock();

    bool pi_dm = fresh && (state & cfg::SYS_STATE::dotmatrix_active);

    if(pi_dm)
    {
      // Render the Pi's 8x24 bitmap directly (row-major, 1 = LED on). The raw
      // setPoint column axis runs opposite to the MD_Parola text engine, so
      // reverse it to match the splash/glyph orientation (else text mirrors).
      const uint16_t W = 8 * cfg::MAX_7219_DOT_MATRIX::num_matrix;
      SPIMutex.lock();
      gfx->control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
      for(uint8_t r = 0; r < 8; r++)
        for(uint16_t c = 0; c < W; c++)
          gfx->setPoint(r, (W - 1) - c, bmp[r * W + c]);
      gfx->control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
      SPIMutex.unlock();
      wasPi = true;
      lastMode = -1;   // force the glyph to redraw when the Pi releases the display
      threads.delay(50);
    }
    else if(!splashDone)
    {
      // Play the boot splash scroll to completion.
      SPIMutex.lock();
      bool done = mx.displayAnimate();
      SPIMutex.unlock();
      if(done) splashDone = true;
      threads.delay(20);
    }
    else
    {
      // Idle default: the drive-mode glyph (I / M / A from SystemState).
      uint8_t mode = state & cfg::SYS_STATE::mode_mask;
      if(wasPi || (int16_t)mode != lastMode)
      {
        const char* glyph = (mode == cfg::SYS_STATE::mode_manual) ? "M"
                          : (mode == cfg::SYS_STATE::mode_auto)   ? "A" : "I";
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
      threads.delay(50);
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

  // The rainbow "loading" sweep plays only while the Pi asserts driver_starting
  // (held for ~led_load_ms right after the driver launches). Booting the Teensy
  // into an already-running Pi leaves the flag clear -> straight to red idle.
  static uint32_t load_start = 0;
  static bool     was_starting = false;
  bool starting = fresh && (state & cfg::SYS_STATE::driver_starting);
  if(starting && !was_starting) load_start = millis();
  was_starting = starting;
  bool loading = starting;

  for(int i = 0; i < N; i++) leds[i] = CRGB::Black;

  if(loading)
  {
    // Rainbow wipe that fills the strip over led_load_ms.
    uint32_t elapsed = millis() - load_start;
    int lit = map(elapsed, 0, cfg::DISP::led_load_ms, 0, N);
    for(int i = 0; i < lit && i < N; i++)
      leds[i] = CHSV((uint8_t)(i * 255 / N + elapsed / 20), 255, 255);
  }
  else if(fresh && (state & cfg::SYS_STATE::led_active))
  {
    // Pi is driving the strip: one RGB triplet per LED.
    for(int i = 0; i < N; i++)
      leds[i] = CRGB(local_led[i * 3], local_led[i * 3 + 1], local_led[i * 3 + 2]);
  }
  else
  {
    // Idle default: single-color red voltage bar, filling from both ends.
    PowerMutex.lock();
    float volt = power[1];
    PowerMutex.unlock();
    int half = map((int)(volt * 100), 700, 840, 0, N / 2);
    half = constrain(half, 0, N / 2);
    for(int i = 0; i < half; i++) { leds[i] = CRGB::Red; leds[N - 1 - i] = CRGB::Red; }
  }

  FastLED.show();
}