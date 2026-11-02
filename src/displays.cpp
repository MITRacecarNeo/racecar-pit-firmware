#include "displays.h"

#include <Arduino.h>
#include <FastLED.h>
#include <MD_Parola.h>
#include <TeensyThreads.h>

#include "battery.h"
#include "config.h"
#include "drive.h"
#include "esc_cal.h"
#include "profile.h"
#include "watchdog.h"
#include "shared.h"

namespace
{
  CRGB leds[cfg::RGB_LED_WS2812B::num_leds];
}

void initLedStrip()
{
  FastLED.addLeds<WS2812B, cfg::RGB_LED_WS2812B::led_pin, GRB>(leds, cfg::RGB_LED_WS2812B::num_leds);
  FastLED.setBrightness(cfg::RGB_LED_WS2812B::brightness);
}

void DotMatrix()
{
  static MD_Parola mx(MD_MAX72XX::FC16_HW, cfg::MAX_7219_DOT_MATRIX::cs_pin, cfg::MAX_7219_DOT_MATRIX::num_matrix);
  mx.begin();
  mx.setIntensity(cfg::MAX_7219_DOT_MATRIX::intensity);
  mx.displayClear();
  mx.displayText("RACECAR NEO", PA_CENTER, 50, 1500, PA_SCROLL_LEFT, PA_SCROLL_LEFT);

  MD_MAX72XX *gfx = mx.getGraphicObject();
  bool splashDone = false;
  bool wasPi = false;
  int16_t lastMode = -1;

  while (true)
  {
    PROF_PERIOD(dot_loop);
    wdt::beat(wdt::dot_matrix);
    WDT_TEST_HANG('m');
    static bool bmp[wire::dot_matrix_bytes];
    CmdMutex.lock();
    memcpy(bmp, dotmatrix_cmd, sizeof(bmp));
    const uint8_t state = system_state;
    CmdMutex.unlock();

    // Override: the Teensy owns the display too. M while the sticks are live,
    // I otherwise; the Pi's auto glyph has no meaning while it is locked out.
    const bool override_on = manual_override;
    const uint8_t mode = override_on
                             ? (rc_driving ? cfg::SYS_STATE::mode_manual : cfg::SYS_STATE::mode_idle)
                             : (uint8_t)(state & cfg::SYS_STATE::mode_mask);

    if (!override_on && piCommandFresh(cfg::DISP::cmd_timeout_ms) && (state & cfg::SYS_STATE::dotmatrix_active))
    {
      // The raw setPoint column axis runs opposite to the MD_Parola text
      // engine, so reverse it or the Pi's text renders mirrored.
      constexpr uint16_t W = cfg::MAX_7219_DOT_MATRIX::cols;
      gfx->control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
      for (uint8_t r = 0; r < cfg::MAX_7219_DOT_MATRIX::rows; r++)
        for (uint16_t c = 0; c < W; c++)
          gfx->setPoint(r, (W - 1) - c, bmp[r * W + c]);
      gfx->control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
      wasPi = true;
      lastMode = -1; // force the glyph to redraw when the Pi releases the display
      threads.delay(cfg::TEENSY_THREADS::disp_tick_ms);
    }
    else if (!splashDone)
    {
      if (mx.displayAnimate())
        splashDone = true;
      threads.delay(cfg::TEENSY_THREADS::splash_tick_ms);
    }
    else
    {
      if (wasPi || (int16_t)mode != lastMode)
      {
        const char *glyph = (mode == cfg::SYS_STATE::mode_manual) ? "M"
                            : (mode == cfg::SYS_STATE::mode_auto) ? "A"
                                                                  : "I";
        mx.displayClear();
        mx.displayText(glyph, PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
        mx.displayReset();
        lastMode = mode;
        wasPi = false;
      }
      mx.displayAnimate();
      threads.delay(cfg::TEENSY_THREADS::disp_tick_ms);
    }
  }
}

void updateLedStrip()
{
  PROF_PERIOD(led_loop);
  constexpr int N = cfg::RGB_LED_WS2812B::num_leds;

  static uint8_t local_led[N * 3];
  CmdMutex.lock();
  memcpy(local_led, led_cmd, sizeof(local_led));
  const uint8_t state = system_state;
  CmdMutex.unlock();

  const bool fresh = piCommandFresh(cfg::DISP::cmd_timeout_ms);

  // Gated on driver_starting, not comms freshness, so a Teensy booting into an
  // already-running Pi goes straight to the idle bar.
  static uint32_t load_start = 0;
  static bool was_starting = false;
  const bool starting = fresh && (state & cfg::SYS_STATE::driver_starting);
  if (starting && !was_starting)
    load_start = millis();
  was_starting = starting;

  for (int i = 0; i < N; i++)
    leds[i] = CRGB::Black;

#ifdef RCN_ESC_CAL
  // Calibration build: a purple bar from both ends shows the held PWM, from
  // one LED per end at full reverse to the whole strip at full forward.
  const int cal_half = escCalLedHalf(escCalPwmUs(), cfg::ESC_10BL60_G2::speed_pwm_min,
                                     cfg::ESC_10BL60_G2::speed_pwm_max, N / 2);
  for (int i = 0; i < cal_half; i++)
  {
    leds[i] = CRGB::Purple;
    leds[N - 1 - i] = CRGB::Purple;
  }
  (void)starting;
#else
  if (starting)
  {
    const uint32_t elapsed = millis() - load_start;
    const int lit = map(elapsed, 0, cfg::DISP::led_load_ms, 0, N);
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
    const float volts = readShared(power_sample).volts;
    const float mv = constrain(volts * 1000.0f, 0.0f, 65535.0f);
    const int half = (batterySocPercent((uint16_t)mv) * (N / 2)) / 100;
    for (int i = 0; i < half; i++)
    {
      leds[i] = CRGB::Red;
      leds[N - 1 - i] = CRGB::Red;
    }
  }
#endif

  PROF_START(t_show);
  FastLED.show();
  PROF_SECTION(led_show, t_show);
}
