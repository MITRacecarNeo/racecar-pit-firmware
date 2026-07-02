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

void fillTxBuff()
{
  while(1)
  {
    RxMutex.lock();
    memcpy(&txPacketPing.RxReceiver, &channelVals, sizeof(channelVals));
    RxMutex.unlock();

    TransmitMutex.lock();
    txPacketPing.Encoder = lastRPM;
    TransmitMutex.unlock();
    
    // Serial.print(String(sizeof(txPacketPing)) + "\t");
    // for (uint8_t i=0; i<sizeof(txPacketPing.RxReceiver)/sizeof(txPacketPing.RxReceiver[0]); i++)
    // {
    //   Serial.print(String(txPacketPing.RxReceiver[i]) + "|");  
    // }
    // Serial.println();
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
      Serial1.write((byte *)&txPacketPong, sizeof(txPacketPong));
      lastTime = micros();
    }
    threads.delay(1);
  }


}

void fillRxData()
{
  RecPongMutex.lock();
  cmd_setpoint[0] = rxPacketPong.ServoAng;
  cmd_setpoint[1] = rxPacketPong.MotorSpeed;
  RecPongMutex.unlock();
  // Serial.println(String(cmd_setpoint[0]) + "\t" + String(cmd_setpoint[1]));
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
        RecPongMutex.lock();
        memcpy(&rxPacketPong, &rxPacketPing, sizeof(rxPacketPong));
        RecPongMutex.unlock();
        fillRxData();
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
    if (deltaTime == 0) lastRPM = 0;
    
    noInterrupts();
    uint32_t dt = deltaTime;
    interrupts();

    float transitionsPerSec = 1000000.0 / dt;
    float electricalRPM = (transitionsPerSec * 60.0) / 6.0;
    float mechanicalRPM = electricalRPM / 2;
    // Serial.println(mechanicalRPM);

    if(mechanicalRPM == lastRPM)
    {
      lastRPM = 0;
    }
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
    RxMutex.lock();
    cmd_pwm[0] = channelVals[0]; //Steering
    
    if(cfg::ESC_10BL60_G2::speed_demon)
    {
      drive_mode_sw = (channelVals[4] < 1500) ? true : false;
      if(drive_mode_sw)
      {
        cmd_pwm[1] = map(channelVals[2], 1000, 2000, 1500, 750);
      } else {
        cmd_pwm[1] = map(channelVals[2], 1000, 2000, 1500, 2250);
      }
    } else 
    {
      //Throttle deadzone 1140 < 1500 < 1575
      if(channelVals[2] > 1525)
      {
        cmd_pwm[1] = map(channelVals[2], 1500, 2000, 1575, max_pwm_throt);
      } else if (channelVals[2] < 1475)
      {
        cmd_pwm[1] = map(channelVals[2], 1000, 1500, max_pwm_rev, 1475);
      } else {
        cmd_pwm[1] = 1500;
      }
      cmd_pwm[1] = (cmd_pwm[1] > max_pwm_throt) ? max_pwm_throt : cmd_pwm[1];
      cmd_pwm[1] = (cmd_pwm[1] < max_pwm_rev) ? max_pwm_rev : cmd_pwm[1];
    }

    RxMutex.unlock();

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