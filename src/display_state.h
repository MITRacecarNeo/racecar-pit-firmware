#pragma once
#include <stdint.h>

#include "stall_guard.h"

// What the LED strip and dot matrix show. Arduino-free, unit tested under
// tests/. The Teensy runs on its own supply, so it sees the Pi only through
// the command link:
//
//   boot_wait  power-on until the first Pi command; LOAD, gradient scroll
//   live       Pi commands fresh; Pi content, else blank matrix and battery meter
//   lost       commands stopped after live (shutdown, brownout, driver
//              restart), or boot_wait timed out; OFF, strip dark
//
// A stall overrides both: WARN, HIGH, STALL on the matrix while the guard
// holds neutral; the strip flashes orange for flash_ms after a trip, then
// shows its usual content.

enum class LinkState : uint8_t
{
  boot_wait,
  live,
  lost,
};

struct LinkTracker
{
  LinkState state = LinkState::boot_wait;
  bool started = false;
  uint32_t boot_ms = 0;
};

LinkState linkTick(LinkTracker &t, bool pi_fresh, uint32_t now_ms, uint32_t boot_timeout_ms);

enum class MatrixShow : uint8_t
{
  blank,
  pi_bitmap,
  text,
};

enum class StripShow : uint8_t
{
  off,
  loading,     // gradient scroll while the Pi boots
  battery,     // charge meter from the back middle outward
  pi_pixels,
  driver_start, // rainbow sweep when the driver launches
  stall_flash, // orange, on/off
  stall_dark,  // the off half of the flash
};

struct DisplayInputs
{
  LinkState link;
  bool matrix_flag;       // SystemState dotmatrix_active
  bool strip_flag;        // SystemState led_active
  bool driver_starting;   // inside the driver-start sweep window
  StallState stall;
  uint32_t ms_since_trip; // since the last trip; ignored before the first
  bool tripped_once;
};

struct DisplayParams
{
  uint32_t word_ms;   // each stall word
  uint32_t flash_ms;  // strip flash after a trip
  uint32_t blink_ms;  // flash on and off half-period
};

struct DisplayChoice
{
  MatrixShow matrix;
  const char *text; // when matrix == text
  StripShow strip;
};

DisplayChoice chooseDisplay(const DisplayInputs &in, const DisplayParams &p);

// Clears a Pi command timestamp once it is older than max_age_us, so a 32-bit
// microsecond clock wrapping past it cannot make it look fresh again.
uint32_t expireRxTime(uint32_t last_rx_us, uint32_t now_us, uint32_t max_age_us);
