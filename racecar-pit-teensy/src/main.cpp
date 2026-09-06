#include <Arduino.h>

#include <FastLED.h>
#include <SPI.h>
#include <TeensyThreads.h>
#include <Wire.h>

#include "config.h"
#include "helpers.h"
#include "Mutexes.h"
#include "packets.h"

void setup()
{
  initTxHeader(txPacketPing.header);
  initTxHeader(txPacketPong.header);
  initRxHeader(rxPacketPing.header);
  initRxHeader(rxPacketPong.header);

  // RC-manual endpoints. speed_demon uses the full configured range; otherwise
  // the percent_* scalars trim the throw around neutral.
  if (cfg::ESC_10BL60_G2::speed_demon)
  {
    max_pwm_throt = cfg::ESC_10BL60_G2::speed_pwm_max;
    max_pwm_rev = cfg::ESC_10BL60_G2::speed_pwm_min;
  }
  else
  {
    max_pwm_throt = cfg::ESC_10BL60_G2::speed_pwm_neutral +
                    (cfg::ESC_10BL60_G2::speed_pwm_max - cfg::ESC_10BL60_G2::speed_pwm_neutral) * cfg::ESC_10BL60_G2::percent_throt;
    max_pwm_rev = cfg::ESC_10BL60_G2::speed_pwm_neutral -
                  (cfg::ESC_10BL60_G2::speed_pwm_neutral - cfg::ESC_10BL60_G2::speed_pwm_min) * cfg::ESC_10BL60_G2::percent_rev;
  }

  FastLED.addLeds<WS2812B, cfg::RGB_LED_WS2812B::led_pin, GRB>(leds, cfg::RGB_LED_WS2812B::num_leds);
  FastLED.setBrightness(cfg::RGB_LED_WS2812B::brightness);

  Wire.begin();
  Serial.begin(cfg::DEBUG::usb_baud);
  Serial1.begin(cfg::UART::baud_rate);
  static uint8_t serial1_rx_buf[cfg::UART::rx_buffer_bytes];
  Serial1.addMemoryForRead(serial1_rx_buf, sizeof(serial1_rx_buf));
  SPI.begin();

  pinMode(cfg::ES2845S_4900KV_MOTOR::hall_a_pin, INPUT);
  pinMode(cfg::ES2845S_4900KV_MOTOR::hall_b_pin, INPUT);
  pinMode(cfg::ES2845S_4900KV_MOTOR::hall_c_pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(cfg::ES2845S_4900KV_MOTOR::hall_a_pin), onHallChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(cfg::ES2845S_4900KV_MOTOR::hall_b_pin), onHallChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(cfg::ES2845S_4900KV_MOTOR::hall_c_pin), onHallChange, CHANGE);

  threads.addThread(testPWM);
  threads.addThread(RxReceive);
  threads.addThread(ReadCurrAndVolt);
  threads.addThread(ReadIMUStates);
  threads.addThread(DotMatrix);
  threads.addThread(fillTxBuff);
  threads.addThread(UARTTx);
  threads.addThread(UARTRx);

  delay(cfg::TEENSY_THREADS::startup_delay);
}

void loop()
{
  ledStuffs();
  threads.delay(cfg::TEENSY_THREADS::loop_tick_ms);
}
