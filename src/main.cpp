#include <Arduino.h>

#include <SPI.h>
#include <TeensyThreads.h>
#include <Wire.h>

#include "config.h"
#include "displays.h"
#include "drive.h"
#include "link.h"
#include "sensors.h"

void setup()
{
  initLedStrip();

  Wire.begin();
  Wire.setClock(cfg::I2C::clock_hz);
  Serial.begin(cfg::DEBUG::usb_baud);
  Serial1.begin(cfg::UART::baud_rate);
  static uint8_t serial1_rx_buf[cfg::UART::rx_buffer_bytes];
  Serial1.addMemoryForRead(serial1_rx_buf, sizeof(serial1_rx_buf));
  static uint8_t serial1_tx_buf[cfg::UART::tx_buffer_bytes];
  Serial1.addMemoryForWrite(serial1_tx_buf, sizeof(serial1_tx_buf));
  Serial1.setTimeout(cfg::UART::rx_read_timeout_ms);
  SPI.begin();

  initHallEncoder();

  threads.addThread(DriveControl);
  threads.addThread(RxReceive);
  threads.addThread(ReadCurrAndVolt);
  threads.addThread(ReadIMUStates);
  threads.addThread(DotMatrix);
  threads.addThread(UARTTx);
  threads.addThread(UARTRx);

  delay(cfg::TEENSY_THREADS::startup_delay);
}

void loop()
{
  updateLedStrip();
  threads.delay(cfg::TEENSY_THREADS::loop_tick_ms);
}
