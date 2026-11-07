#include "displays.h"

#include <Arduino.h>
#include <FastLED.h>
#include <MD_Parola.h>
#include <TeensyThreads.h>

#include "battery.h"
#include "config.h"
#include "display_state.h"
#include "drive.h"
#include "esc_cal.h"
#include "led_patterns.h"
#include "profile.h"
#include "shared.h"
#include "small_font.h"
#include "watchdog.h"

namespace
{
  constexpr uint16_t N = cfg::RGB_LED_WS2812B::num_leds;
  CRGB leds[N];

  // One link tracker for both displays; loop() and the DotMatrix thread both
  // read it.
  Threads::Mutex display_mutex;
  LinkTracker link_tracker;

  struct Snapshot
  {
    DisplayChoice choice;
    uint8_t state; // SystemState bits
  };

  Snapshot currentDisplay()
  {
    CmdMutex.lock();
    const uint8_t state = system_state;
    CmdMutex.unlock();

    const bool fresh = piCommandFresh(cfg::DISP::cmd_timeout_ms);
    display_mutex.lock();
    const LinkState link = linkTick(link_tracker, fresh, millis(), cfg::DISP::boot_timeout_ms);
    display_mutex.unlock();

    DisplayInputs in = {};
    in.link = link;
    in.matrix_flag = state & cfg::SYS_STATE::dotmatrix_active;
    in.strip_flag = state & cfg::SYS_STATE::led_active;
    in.driver_starting = state & cfg::SYS_STATE::driver_starting;
    const StallWord stall = unpackStallWord(stall_word);
    in.stall = stall.state;
    in.tripped_once = stall.trips > 0;
    in.ms_since_trip = millis() - stall_trip_ms;
    return {chooseDisplay(in, cfg::DISP::stall_display), state};
  }

  void show(const Rgb *px)
  {
    for (uint16_t i = 0; i < N; i++)
      leds[i] = CRGB(px[i].r, px[i].g, px[i].b);
  }
}

void initLedStrip()
{
  FastLED.addLeds<WS2812B, cfg::RGB_LED_WS2812B::led_pin, GRB>(leds, N);
  FastLED.setBrightness(cfg::RGB_LED_WS2812B::brightness);
}

void DotMatrix()
{
  static MD_Parola mx(MD_MAX72XX::FC16_HW, cfg::MAX_7219_DOT_MATRIX::cs_pin, cfg::MAX_7219_DOT_MATRIX::num_matrix);
  mx.begin();
  mx.setIntensity(cfg::MAX_7219_DOT_MATRIX::intensity);
  mx.setFont(small_font);
  mx.displayClear();

  MD_MAX72XX *gfx = mx.getGraphicObject();
  MatrixShow shown = MatrixShow::blank;
  const char *shown_text = nullptr;
  bool first = true;

  while (true)
  {
    PROF_PERIOD(dot_loop);
    wdt::beat(wdt::dot_matrix);
    WDT_TEST_HANG('m');

    const DisplayChoice c = currentDisplay().choice;
    if (c.matrix == MatrixShow::pi_bitmap)
    {
      static bool bmp[wire::dot_matrix_bytes];
      CmdMutex.lock();
      memcpy(bmp, dotmatrix_cmd, sizeof(bmp));
      CmdMutex.unlock();
      // The raw setPoint column axis runs opposite to the MD_Parola text
      // engine, so reverse it or the Pi's text renders mirrored.
      constexpr uint16_t W = cfg::MAX_7219_DOT_MATRIX::cols;
      gfx->control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
      for (uint8_t r = 0; r < cfg::MAX_7219_DOT_MATRIX::rows; r++)
        for (uint16_t col = 0; col < W; col++)
          gfx->setPoint(r, (W - 1) - col, bmp[r * W + col]);
      gfx->control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
    }
    else if (first || c.matrix != shown || c.text != shown_text)
    {
      mx.displayClear();
      if (c.matrix == MatrixShow::text)
      {
        mx.displayText(c.text, PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
        mx.displayReset();
      }
    }
    if (c.matrix == MatrixShow::text)
      mx.displayAnimate();
    shown = c.matrix;
    shown_text = c.text;
    first = false;
    threads.delay(cfg::TEENSY_THREADS::disp_tick_ms);
  }
}

void updateLedStrip()
{
  PROF_PERIOD(led_loop);
  static Rgb px[N];
  fill(px, N, {0, 0, 0});

#ifdef RCN_ESC_CAL
  // Calibration build: a purple bar from both ends shows the held PWM, from
  // one LED per end at full reverse to the whole strip at full forward.
  const int cal_half = escCalLedHalf(escCalPwmUs(), cfg::ESC_10BL60_G2::speed_pwm_min,
                                     cfg::ESC_10BL60_G2::speed_pwm_max, N / 2);
  for (int i = 0; i < cal_half; i++)
  {
    px[i] = {128, 0, 128};
    px[N - 1 - i] = {128, 0, 128};
  }
#else
  const Snapshot s = currentDisplay();
  const uint32_t now = millis();

  // Driver-start rainbow: grows over led_load_ms from when the bit appears.
  static uint32_t load_start = 0;
  static bool was_starting = false;
  const bool starting = s.choice.strip == StripShow::driver_start;
  if (starting && !was_starting)
    load_start = now;
  was_starting = starting;

  switch (s.choice.strip)
  {
  case StripShow::off:
  case StripShow::stall_dark:
    break;
  case StripShow::loading:
    loadingBar(px, N, now, cfg::RGB_LED_WS2812B::loading_bar_ms, cfg::RGB_LED_WS2812B::bar_red);
    break;
  case StripShow::battery:
  {
    const float volts = readShared(power_sample).volts;
    const float mv = constrain(volts * 1000.0f, 0.0f, 65535.0f);
    batteryMeter(px, N, batterySocPercent((uint16_t)mv), cfg::RGB_LED_WS2812B::bar_red);
    break;
  }
  case StripShow::pi_pixels:
    CmdMutex.lock();
    for (uint16_t i = 0; i < N; i++)
      px[i] = {led_cmd[i * 3], led_cmd[i * 3 + 1], led_cmd[i * 3 + 2]};
    CmdMutex.unlock();
    break;
  case StripShow::driver_start:
  {
    const uint32_t elapsed = now - load_start;
    const int lit = map(elapsed, 0, cfg::DISP::led_load_ms, 0, N);
    for (int i = 0; i < lit && i < N; i++)
    {
      const CRGB c = CHSV((uint8_t)(i * 255 / N + elapsed / 20), 255, 255);
      px[i] = {c.r, c.g, c.b};
    }
    break;
  }
  case StripShow::stall_flash:
    fill(px, N, cfg::RGB_LED_WS2812B::stall_orange);
    break;
  }
#endif

  show(px);
  PROF_START(t_show);
  FastLED.show();
  PROF_SECTION(led_show, t_show);
}
