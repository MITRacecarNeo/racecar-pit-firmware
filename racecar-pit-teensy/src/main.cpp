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

// ------------------------- UART Pi & Teensy ----------------------- //
struct HeaderPacket {
  const uint32_t MagicNum = 0xDEADBEEF;
  const uint8_t VersionNum = 1;
  const uint8_t IDNumber = 0;
  uint16_t PacketBytes;
  uint32_t FrameBytes;
  uint16_t PacketIndex;
  uint16_t PacketsPerFrame;
}__attribute__((packed));

struct TXPacket {
  HeaderPacket header;
  int32_t Timestamp;
  uint16_t VoltAndCurr[2];
  uint16_t RxReceiver[8]; //8
  float IMU[9];  //9
  int16_t Encoder; //1
  float EKF[3]; //3
  uint16_t Checksum; //1
} __attribute__((packed)) txPacketPing, txPacketPong;

struct RxPacket {
  HeaderPacket header;
  float ServoAng;
  float MotorAng;
  bool DotMatrix[192]; //192
  uint8_t LED [255]; //255
  uint8_t SystemState;
  uint16_t Checksum;
}__attribute__((packed)) rxPacketPing, rxPacketPong;

// --------------------------- RGB LED ---------------------------- //
#define NUM_LEDS 84
#define DATA_PIN 9
#define BRIGHTNESS 25

CRGB leds[NUM_LEDS];
CRGBPalette16 currrentPalette = RainbowStripeColors_p;
uint8_t colorIdx = 1;
uint8_t numleds;
// -------------------------- Servo and ESC ------------------------ //
Servo steerServo, throtMot;
const bool speedDemon = true;
const uint16_t maxPWMThrot = 1580;
const uint16_t maxPWMRev = 1400;
//          array format: {center PWM, magnitude (max when center + magnitude)}
const int steer_yaml_consts[2] = {6000, 1900}; // quarter-micro seconds
const int throt_yaml_consts[2] = {6000, 3000}; // quarter-micro seconds

const int steer_consts[2] = {steer_yaml_consts[0]/4, steer_yaml_consts[1]/4}; // 1025 : 1500 : 1975 micro seconds
const int throt_consts[2] = {throt_yaml_consts[0]/4, throt_yaml_consts[1]/4}; // 750 : 1500 : 2250 micro seconds

//           {steer, throt}
uint16_t cmds[2] = {1500, 1500};
bool swA = 0;



// ----------------------------- Teensy Threading ------------------- //
Threads::Mutex CmdMutex;      // for use with cmd_packet}{}
Threads::Mutex RxMutex;       // for use with channelVals[]
Threads::Mutex I2CMutex;      // for use with I2C bus
Threads::Mutex SPIMutex;      // for use with SPI bus
Threads::Mutex powerMutex;    // for use with power[]
Threads::Mutex ErrMutex;      // for use with errorFlag
Threads::Mutex ImuMutex;      // for use with imuVals[]
Threads::Mutex HallMutex;     // for use with currentState;
Threads::Mutex transmitMutex; // for use with txPacketPing
Threads::Mutex recieveMutex;  // for use with rxPacketPing
uint8_t errorFlag = 0, errType;

// ----------------------------- FlySky IA6B Rx --------------------- //
const uint8_t PPM_PIN = 5;
const uint8_t ChannelCount = 8;
// Range between 1000 - 2000 us {Aileron, Elevator, Throttle, Rudder, SWA, SWB, SWC, SWD, Ch9, Ch10} //
uint16_t channelVals[8] =       {0,       0,        0,        0,      0,   0,   0,   0};

// --------------------------- INA226 Voltage and Current Sensor ------------------------ //
INA226 INA(0x40);
float power[3] = {0, 0, 0};  // Current (mA), Voltage (V), Power (mW)

// ---------------------------- LSM9DS1 9-DOF IMU ----------------------------------------//
Adafruit_LSM9DS1 lsmimu = Adafruit_LSM9DS1();
sensors_event_t a, m, g, temp;
float imuVals[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

//------------------------------ MAX 7219 Dot Matrix -------------------------------------//
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define NUM_DOT_MATRIX 3
#define CS_PIN 10

MD_Parola mx = MD_Parola(HARDWARE_TYPE, CS_PIN, NUM_DOT_MATRIX);

// --------------------------------------- Hall - Effect Encoder ---------------------------//

const uint8_t HALL_A_PIN =  30;
const uint8_t HALL_B_PIN = 31;
const uint8_t HALL_C_PIN = 32;
const uint8_t MOT_TEMP_PIN = A17;

volatile uint32_t lastTransitionTime = 0;
volatile uint32_t deltaTime = 0;
volatile uint8_t currentState = 0;
volatile uint8_t lastState = 0;
volatile int8_t direction = 0;

volatile float lastRPM;
volatile int8_t lastDir;


// --------------------------------- Function Defintions -----------------------------------//

void fillTxBuff()
{
  while(1)
  {
    RxMutex.lock();
    memcpy(&txPacketPing.RxReceiver, &channelVals, sizeof(channelVals));
    RxMutex.unlock();
    Serial.print(String(sizeof(txPacketPing)) + "\t");
    for (uint8_t i=0; i<sizeof(txPacketPing.RxReceiver)/sizeof(txPacketPing.RxReceiver[0]); i++)
    {
      Serial.print(String(txPacketPing.RxReceiver[i]) + "|");  
    }
    Serial.println();
    threads.delay(5);
  }
}

void updateFromRxBuff()
{

}

void UARTTx()
{
  while(true){
    transmitMutex.lock();
    //(where it needs to go, what needs to go there, size of what needs to go)
    memcpy(&txPacketPong, &txPacketPing, sizeof(txPacketPing));
    transmitMutex.unlock();
    Serial1.write((byte *)&txPacketPong, sizeof(txPacketPong));
    threads.delay(5);
  }


}

void UARTRx()
{

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

  uint8_t a = digitalRead(HALL_A_PIN);
  uint8_t b = digitalRead(HALL_B_PIN);
  uint8_t c = digitalRead(HALL_C_PIN);

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

float getRPM() {
    if (deltaTime == 0) return 0;
    
    noInterrupts();
    uint32_t dt = deltaTime;
    interrupts();

    float transitionsPerSec = 1000000.0 / dt;
    float electricalRPM = (transitionsPerSec * 60.0) / 6.0;
    float mechanicalRPM = electricalRPM / 2;

    if(mechanicalRPM == lastRPM)
    {
      return 0;
    }
    lastRPM = mechanicalRPM;
    return mechanicalRPM;
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
    powerMutex.lock();
    power[0] = INA.getCurrent();
    power[1] = INA.getBusVoltage();
    power[2] = INA.getPower();
    powerMutex.unlock();
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
  PPMReader ppm(PPM_PIN, ChannelCount);
  
  while(1)
  {
    for(uint8_t i = 1; i <= ChannelCount; i++)
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

  steerServo.writeMicroseconds(steer_consts[0]);
  throtMot.writeMicroseconds(throt_consts[0]);

  while(1)
  {    
    RxMutex.lock();
    cmds[0] = channelVals[0]; //Steering
    
    if(speedDemon)
    {
      swA = (channelVals[4] > 1500) ? true : false;
      if(swA)
      {
        cmds[1] = map(channelVals[2], 1000, 2000, 1500, 750);
      } else {
        cmds[1] = map(channelVals[2], 1000, 2000, 1500, 2250);
      }
    } else 
    {
      //Throttle deadzone 1140 < 1500 < 1575
      if(channelVals[2] > 1525)
      {
        cmds[1] = map(channelVals[2], 1500, 2000, 1575, maxPWMThrot);
      } else if (channelVals[2] < 1475)
      {
        cmds[1] = map(channelVals[2], 1000, 1500, maxPWMRev, 1475);
      } else {
        cmds[1] = 1500;
      }
      cmds[1] = (cmds[1] > maxPWMThrot) ? maxPWMThrot : cmds[1];
      cmds[1] = (cmds[1] < maxPWMRev) ? maxPWMRev : cmds[1];
    }

    RxMutex.unlock();

    steerServo.writeMicroseconds(cmds[0]);
    throtMot.writeMicroseconds(cmds[1]);

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
    // char msgbuff[32];
    
    // SPIMutex.lock();
    // mx.displayClear();
    // mx.flush();
    // powerMutex.lock();
    // dtostrf(power[1], 5, 2, msgbuf);
    // powerMutex.unlock();
    // mx.displayText(msgbuf, PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
    // mx.displayAnimate();
    // SPIMutex.unlock();
    // threads.delay(750);

    SPIMutex.lock();
    mx.displayClear();
    mx.flush();
    powerMutex.lock();
    dtostrf(power[0], 5, 2, msgbuf);
    powerMutex.unlock();
    mx.displayText(msgbuf, PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
    mx.displayAnimate();
    SPIMutex.unlock();
    threads.delay(100);
  }

}

void DebugUsingSerial()
{
  while(1)
  {

    // CmdMutex.lock();
    // Serial.println("Servo CMD: " + String(cmd_pkt.deltaSteer));
    // Serial.println("Throttle CMD: " + String(cmd_pkt.deltaThrot));
    // CmdMutex.unlock();

    // for(int i = 0; i < uint8_t(sizeof(channelVals) / sizeof(channelVals[0])); i++)
    // {
    //   RxMutex.lock();
    //   Serial.print("Channel: " + String(i+1) + ": " + String(channelVals[i]) + "\t");
    //   RxMutex.unlock();
    // }
    // Serial.println();
    // threads.delay(2);

    // powerMutex.lock();
    // Serial.println("Current: " + String(power[0]) + " A | Voltage: " + String(power[1]) + " V | Power: " + String(power[2]) + " W");
    // powerMutex.unlock();

    // for(uint8_t i = 0; i < (uint8_t)(sizeof(imuVals) / sizeof(imuVals[0])); i++)
    // {
    //   ImuMutex.lock();
    //   Serial.print(String(imuVals[i]) + " | ");
    //   ImuMutex.unlock();
    // }
    // Serial.println();
    
    // Serial.println("Direction:" + String(direction) + "\t" + "RPM: " + String(getRPM()));

    threads.delay(50);
  }
}

void ledStuffs()
{
  powerMutex.lock();
  numleds = map(power[1]*100, 700, 840, 0, 42);
  powerMutex.unlock();
  for(int i = 0; i < numleds; i++)
  {
    leds[i] = ColorFromPalette(currrentPalette, colorIdx, BRIGHTNESS, LINEARBLEND);
    leds[83-i] = ColorFromPalette(currrentPalette, colorIdx, BRIGHTNESS, LINEARBLEND);
    colorIdx++;
  }
  threads.delay(5);
  FastLED.show();
  for(int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black;
  }
}

void setup()
{
  
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  
  Wire.begin();
  Serial.begin(115200);
  Serial1.begin(230400);
  SPI.begin();

  pinMode(HALL_A_PIN, INPUT);
  pinMode(HALL_B_PIN, INPUT);
  pinMode(HALL_C_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(HALL_A_PIN), onHallChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_B_PIN), onHallChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_C_PIN), onHallChange, CHANGE);

  threads.addThread(testPWM);
  threads.addThread(RxReceive);
  //threads.addThread(DebugUsingSerial);
  threads.addThread(ReadCurrAndVolt);
  threads.addThread(ReadIMUStates);
  threads.addThread(DotMatrix);
  threads.addThread(fillTxBuff);
  threads.addThread(UARTTx);
  delay(2000);

}

void loop()
{
  ledStuffs();
  threads.delay(25);
}



