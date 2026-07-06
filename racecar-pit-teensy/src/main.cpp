#include <Arduino.h>

#include "config.h"
#include "packets.h"
#include "Mutexes.h"
#include "helpers.h"

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

void setup()
{
  // -------------------------- initialize headers ------------------------ //
  initTxHeader(txPacketPing.header);
  initTxHeader(txPacketPong.header);
  initRxHeader(rxPacketPing.header);
  initRxHeader(rxPacketPong.header);

  // -------------------------- set PWM bounds ----------------------------- //
  if(cfg::ESC_10BL60_G2::speed_demon)
  {
    max_pwm_throt = cfg::ESC_10BL60_G2::throt_center + cfg::ESC_10BL60_G2::throt_mag;
    max_pwm_rev  = cfg::ESC_10BL60_G2::throt_center - cfg::ESC_10BL60_G2::throt_mag;
  } else {
    max_pwm_throt = cfg::ESC_10BL60_G2::throt_center + cfg::ESC_10BL60_G2::throt_mag * cfg::ESC_10BL60_G2::percentThrot;
    max_pwm_rev   = cfg::ESC_10BL60_G2::throt_center - cfg::ESC_10BL60_G2::throt_mag * cfg::ESC_10BL60_G2::percentRev;
  }
  
  // ------------------------- setup WS2182B RGB LED Strip ----------------- //
  FastLED.addLeds<WS2812B, cfg::RGB_LED_WS2812B::led_pin, GRB>(leds, cfg::RGB_LED_WS2812B::num_leds);
  FastLED.setBrightness(cfg::RGB_LED_WS2812B::brightness);
  
  // --------------------------- setup UART, SPI, I2C interfaces ----------- //
  Wire.begin();
  Serial.begin(115200);
  Serial1.begin(cfg::UART::baud_rate);
  // The default 64-byte RX FIFO overflows mid-frame: a 474-byte command at
  // 921600 baud fills 64 bytes in ~0.7 ms, but UARTRx can be preempted longer
  // than that under the thread scheduler. Hold several full frames instead.
  static uint8_t serial1_rx_buf[2048];
  Serial1.addMemoryForRead(serial1_rx_buf, sizeof(serial1_rx_buf));
  SPI.begin();

  // ----------------- setup hall sensing pins as input interrupts --------- //
  pinMode(cfg::ES2845S_4900KV_MOTOR::hall_a_pin, INPUT);
  pinMode(cfg::ES2845S_4900KV_MOTOR::hall_b_pin, INPUT);
  pinMode(cfg::ES2845S_4900KV_MOTOR::hall_c_pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(cfg::ES2845S_4900KV_MOTOR::hall_a_pin), onHallChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(cfg::ES2845S_4900KV_MOTOR::hall_b_pin), onHallChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(cfg::ES2845S_4900KV_MOTOR::hall_c_pin), onHallChange, CHANGE);


  // ------------------------- add thread functions ----------------------- //
  threads.addThread(testPWM);
  threads.addThread(RxReceive);
  threads.addThread(ReadCurrAndVolt);
  threads.addThread(ReadIMUStates);
  threads.addThread(DotMatrix);
  threads.addThread(fillTxBuff);
  threads.addThread(UARTTx);
  threads.addThread(UARTRx);
  threads.addThread(InnerControlLoop);
  delay(2000);

}

void loop()
{
  ledStuffs();
  threads.delay(25);
}



