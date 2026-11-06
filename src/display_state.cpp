#include "display_state.h"

LinkState linkTick(LinkTracker &t, bool pi_fresh, uint32_t now_ms, uint32_t boot_timeout_ms)
{
  if (!t.started)
  {
    t.started = true;
    t.boot_ms = now_ms;
  }
  if (pi_fresh)
    t.state = LinkState::live;
  else if (t.state == LinkState::live)
    t.state = LinkState::lost;
  else if (t.state == LinkState::boot_wait && (uint32_t)(now_ms - t.boot_ms) >= boot_timeout_ms)
    t.state = LinkState::lost;
  return t.state;
}

DisplayChoice chooseDisplay(const DisplayInputs &in, const DisplayParams &p)
{
  static const char *const stall_words[] = {"WARN", "HIGH", "STALL"};

  DisplayChoice c = {MatrixShow::blank, nullptr, StripShow::battery};
  switch (in.link)
  {
  case LinkState::boot_wait:
    c = {MatrixShow::text, "LOAD", StripShow::loading};
    break;
  case LinkState::lost:
    c = {MatrixShow::text, "OFF", StripShow::off};
    break;
  case LinkState::live:
    if (in.matrix_flag)
      c.matrix = MatrixShow::pi_bitmap;
    if (in.driver_starting)
      c.strip = StripShow::driver_start;
    else if (in.strip_flag)
      c.strip = StripShow::pi_pixels;
    break;
  }

  if (in.stall != StallState::normal)
  {
    c.matrix = MatrixShow::text;
    c.text = stall_words[(in.ms_since_trip / p.word_ms) % 3];
  }
  if (in.tripped_once && in.ms_since_trip < p.flash_ms)
    c.strip = (in.ms_since_trip / p.blink_ms) % 2 == 0 ? StripShow::stall_flash : StripShow::stall_dark;
  return c;
}

uint32_t expireRxTime(uint32_t last_rx_us, uint32_t now_us, uint32_t max_age_us)
{
  if (last_rx_us != 0 && (uint32_t)(now_us - last_rx_us) > max_age_us)
    return 0;
  return last_rx_us;
}
