#include <Arduino.h>

#include <SPI.h>
#include <TeensyThreads.h>
#include <Wire.h>

#include "config.h"
#include "displays.h"
#include "drive.h"
#include "esc_cal.h"
#include "link.h"
#include "profile.h"
#include "watchdog.h"
#include "sensors.h"

void setup()
{
  wdt::captureResetCause();
  initLedStrip();

  Wire.begin();
  Wire.setClock(cfg::I2C::clock_hz);
  Serial.begin(cfg::DEBUG::usb_baud);
  Serial1.begin(cfg::UART::baud_rate);
  static uint8_t serial1_rx_buf[cfg::UART::rx_buffer_bytes];
  Serial1.addMemoryForRead(serial1_rx_buf, sizeof(serial1_rx_buf));
  static uint8_t serial1_tx_buf[cfg::UART::tx_buffer_bytes];
  Serial1.addMemoryForWrite(serial1_tx_buf, sizeof(serial1_tx_buf));
  SPI.begin();

  initHallEncoder();
  startDriveControl();
  startTelemetry();

  threads.addThread(RxReceive);
  threads.addThread(ReadCurrAndVolt);
  threads.addThread(ReadIMUStates);
  threads.addThread(DotMatrix);
  threads.addThread(InfoBuilder);
  threads.addThread(UARTRx);
  PROF_BEGIN();

  delay(cfg::TEENSY_THREADS::startup_delay);
}

#ifdef RCN_ESC_CAL
namespace
{
  EscCal esc_cal;

  // Calibration build: one command character per byte on USB serial; every
  // byte gets a status line back.
  void serviceEscCal()
  {
    using cfg::ESC_10BL60_G2::speed_pwm_max;
    using cfg::ESC_10BL60_G2::speed_pwm_min;
    using cfg::ESC_10BL60_G2::speed_pwm_neutral;
    const uint32_t now = millis();
    while (Serial.available() > 0)
    {
      const char c = (char)Serial.read();
      if (c == '\n' || c == '\r')
        continue;
      escCalApply(esc_cal, c, now);
      const uint16_t pwm = escCalPwm(esc_cal, speed_pwm_min, speed_pwm_neutral, speed_pwm_max);
      setEscCalPwm(pwm);
      char line[64];
      const size_t n = escCalStatus(esc_cal, pwm, line, sizeof(line));
      if (n > 0 && Serial.availableForWrite() >= (int)n)
        Serial.write(line, n);
    }
    escCalTick(esc_cal, now, cfg::ESC_CAL::link_timeout_ms);
    setEscCalPwm(escCalPwm(esc_cal, speed_pwm_min, speed_pwm_neutral, speed_pwm_max));
  }
}
#endif

void loop()
{
#ifdef RCN_ESC_CAL
  serviceEscCal();
#endif
#ifdef RCN_WDT_TEST
  // Test build: a character on USB serial stops one context (t telemetry
  // tick, b info builder, r, p, i, c, m, d for the drive tick, l for loop).
  if (Serial.available() > 0)
    wdt::test_hang = (char)Serial.read();
  while (wdt::test_hang == 'l')
  {
  }
#endif
  wdt::beat(wdt::main_loop);
  wdt::poll();
  updateLedStrip();
  printDriveTrace();
  PROF_REPORT();
  threads.delay(cfg::TEENSY_THREADS::loop_tick_ms);
}
