#include "watchdog.h"

#include <Arduino.h>

#include "config.h"
#include "heartbeat.h"
#include "profile.h"

namespace wdt
{
  namespace
  {
    Heartbeat heartbeat(all_contexts);
    volatile bool is_armed = false;
    uint32_t reset_flags = 0;
    const char *reset_cause = "unknown";

    // RTWDOG keys and clock: the 32 kHz low-power clock, no prescaler.
    constexpr uint32_t unlock_key = 0xD928C520;
    constexpr uint32_t refresh_key = 0xB480A602;
    constexpr uint32_t ticks_per_ms = 32;
    constexpr uint32_t timeout_ticks = cfg::WATCHDOG::timeout_ms * ticks_per_ms;
    static_assert(timeout_ticks >= 32 && timeout_ticks <= 0xFFFF,
                  "watchdog timeout must be 1 to 2047 ms without the prescaler");

    void start()
    {
      CCM_CCGR5 |= CCM_CCGR5_WDOG3(3);
      __disable_irq();
      // Unlock, then reconfigure within the 128 bus clocks the unlock allows.
      if (WDOG3_CS & WDOG_CS_CMD32EN)
      {
        WDOG3_CNT = unlock_key;
      }
      else
      {
        volatile uint16_t *const cnt16 = (volatile uint16_t *)(IMXRT_WDOG3_ADDRESS + 4);
        *cnt16 = (uint16_t)(unlock_key & 0xFFFF);
        *cnt16 = (uint16_t)(unlock_key >> 16);
      }
      WDOG3_WIN = 0;
      WDOG3_TOVAL = timeout_ticks;
      WDOG3_CS = WDOG_CS_FLG | WDOG_CS_UPDATE | WDOG_CS_CMD32EN | WDOG_CS_CLK(1) | WDOG_CS_EN;
      __enable_irq();
    }

    void refresh()
    {
      WDOG3_CNT = refresh_key;
    }
  }

  void captureResetCause()
  {
    reset_flags = SRC_SRSR;
    SRC_SRSR = reset_flags; // write-1-to-clear, so the next boot reads its own cause
    if (reset_flags & SRC_SRSR_WDOG3_RST_B)
      reset_cause = "watchdog";
    else if (reset_flags & SRC_SRSR_LOCKUP_SYSRESETREQ)
      reset_cause = "software";
    else if (reset_flags & SRC_SRSR_IPP_RESET_B)
      reset_cause = "reset";
  }

  const char *lastResetCause() { return reset_cause; }
  uint32_t resetFlags() { return reset_flags; }
  bool armed() { return is_armed; }

  void beat(Context c)
  {
    heartbeat.beat(c);
  }

  void poll()
  {
    if (!cfg::WATCHDOG::enabled || is_armed || !heartbeat.allSeen())
      return;
    heartbeat.collect();
    start();
    is_armed = true;
  }

  void tick()
  {
    if (is_armed && heartbeat.collect())
    {
      refresh();
      PROF_PERIOD(wdt_refresh);
    }
  }

#ifdef RCN_WDT_TEST
  volatile char test_hang = 0;
#endif
}
