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
    txPacketPing.VoltAndCurr[1] = (uint16_t)(curr < 0.0f ? 0.0f : curr); // current, raw INA226 counts
    txPacketPing.Encoder        = lastRPM;
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

void fillRxData()
{
  RecPongMutex.lock();
  float servo = rxPacketPong.ServoAng;
  float motor = rxPacketPong.MotorSpeed;
  RecPongMutex.unlock();

  CmdMutex.lock();
  cmd_setpoint[0] = servo;
  cmd_setpoint[1] = motor;
  CmdMutex.unlock();
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
    lsmimu.getEvent(&a, &g, &m, &temp);
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
  steerServo.attach(2);
  throtMot.attach(3);

  steerServo.writeMicroseconds(cfg::SERVO_D135F::steer_center);
  throtMot.writeMicroseconds(cfg::ESC_10BL60_G2::throt_center);

  while(1)
  {
    uint16_t ch[cfg::FLYSKY_IA6B_RX::channel_count];
    RxMutex.lock();
    memcpy(ch, channelVals, sizeof(ch));
    RxMutex.unlock();

    // Is a live RC signal present on the sticks we read?
    bool rc_signal = (ch[0] >= cfg::AUTO_DRIVE::rc_valid_min && ch[0] <= cfg::AUTO_DRIVE::rc_valid_max) &&
                     (ch[2] >= cfg::AUTO_DRIVE::rc_valid_min && ch[2] <= cfg::AUTO_DRIVE::rc_valid_max);

    float rc_steer = cfg::SERVO_D135F::steer_center;
    float rc_throt = cfg::ESC_10BL60_G2::throt_center;
    bool  rc_active = false;

    if(rc_signal)
    {
      // What the RC transmitter would command (the manual mapping).
      rc_steer = ch[0];

      if(cfg::ESC_10BL60_G2::speed_demon)
      {
        drive_mode_sw = (ch[4] < 1500) ? true : false;
        rc_throt = drive_mode_sw ? map(ch[2], 1000, 2000, 1500, 750)
                                 : map(ch[2], 1000, 2000, 1500, 2250);
      }
      else
      {
        //Throttle deadzone 1140 < 1500 < 1575
        if(ch[2] > 1525)      rc_throt = map(ch[2], 1500, 2000, 1575, max_pwm_throt);
        else if(ch[2] < 1475) rc_throt = map(ch[2], 1000, 1500, max_pwm_rev, 1475);
        else                  rc_throt = 1500;
        rc_throt = (rc_throt > max_pwm_throt) ? max_pwm_throt : rc_throt;
        rc_throt = (rc_throt < max_pwm_rev)   ? max_pwm_rev   : rc_throt;
      }

      // Human is driving if either stick output leaves its neutral band.
      rc_active =
        (fabsf(rc_throt - cfg::ESC_10BL60_G2::throt_center) > cfg::AUTO_DRIVE::throt_override_band) ||
        (fabsf(rc_steer - cfg::SERVO_D135F::steer_center)   > cfg::AUTO_DRIVE::steer_override_band);
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
      bool fresh = (lastRxTime != 0) &&
                   ((micros() - lastRxTime) < (cfg::UART::rx_timeout * 1000UL));

      if(fresh)
      {
        CmdMutex.lock();
        float servo = cmd_setpoint[0];
        float motor = cmd_setpoint[1];
        CmdMutex.unlock();

        servo = constrain(servo, -1.0f, 1.0f);
        motor = constrain(motor, -1.0f, 1.0f);

        cmd_pwm[0] = cfg::SERVO_D135F::steer_center + servo * cfg::SERVO_D135F::steer_mag * cfg::AUTO_DRIVE::steer_limit;
        cmd_pwm[1] = cfg::ESC_10BL60_G2::throt_center - motor * cfg::ESC_10BL60_G2::throt_mag * cfg::AUTO_DRIVE::throt_limit;
      }
      else
      {
        cmd_pwm[0] = cfg::SERVO_D135F::steer_center;
        cmd_pwm[1] = cfg::ESC_10BL60_G2::throt_center;
      }
    }

    steerServo.writeMicroseconds(cmd_pwm[0]);
    throtMot.writeMicroseconds(cmd_pwm[1]);

    threads.delay(1);
  }

}

void DotMatrix()
{
  SPIMutex.lock();
  mx.begin();
  mx.setIntensity(8);
  mx.displayClear();
  SPIMutex.unlock();

  while(1)
  {
    char msgbuf[16];

    SPIMutex.lock();
    mx.displayClear();
    mx.flush();
    PowerMutex.lock();
    dtostrf(power[0], 5, 2, msgbuf);
    PowerMutex.unlock();
    mx.displayText(msgbuf, PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
    mx.displayAnimate();
    SPIMutex.unlock();
    threads.delay(100);
  }

}

void ledStuffs()
{
  PowerMutex.lock();
  numleds = map(power[1]*100, 700, 840, 0, 42);
  PowerMutex.unlock();
  for(int i = 0; i < numleds; i++)
  {
    leds[i] = ColorFromPalette(currrentPalette, colorIdx, cfg::RGB_LED_WS2812B::brightness, LINEARBLEND);
    leds[83-i] = ColorFromPalette(currrentPalette, colorIdx, cfg::RGB_LED_WS2812B::brightness, LINEARBLEND);
    colorIdx++;
  }
  threads.delay(5);
  FastLED.show();
  for(int i = 0; i < cfg::RGB_LED_WS2812B::num_leds; i++)
  {
    leds[i] = CRGB::Black;
  }
}