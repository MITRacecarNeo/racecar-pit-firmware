#pragma once
#include <stdint.h>

// RTWDOG (watchdog 3) supervision. Every context below beats once per loop;
// the drive tick refreshes the watchdog only while all of them run. It is
// armed from loop() once each context has beaten, so boot and sensor start
// never reset the board. A missed refresh resets the Teensy after
// cfg::WATCHDOG::timeout_ms; it boots with servo and ESC at neutral.
namespace wdt
{
    enum Context : uint32_t
    {
        drive_tick = 1u << 0,
        telemetry = 1u << 1,
        command_rx = 1u << 2,
        rc_rx = 1u << 3,
        imu = 1u << 4,
        power = 1u << 5,
        dot_matrix = 1u << 6,
        main_loop = 1u << 7,
        info_builder = 1u << 8,
    };
    constexpr uint32_t all_contexts = (1u << 9) - 1;

    // Reads and clears the reset cause; call first thing in setup().
    void captureResetCause();
    // "watchdog", "software", "reset", or "unknown", from the last boot.
    const char *lastResetCause();
    // SRC_SRSR as read at boot.
    uint32_t resetFlags();

    void beat(Context c);
    // Arms the watchdog once every context has beaten; call from loop().
    void poll();
    // Refreshes when every context beat this round; call from the drive tick.
    void tick();
    bool armed();

#ifdef RCN_WDT_TEST
    // Test build only: the context whose code matches stops (see main.cpp).
    extern volatile char test_hang;
#endif
}

#ifdef RCN_WDT_TEST
#define WDT_TEST_HANG(code)              \
    do                                   \
    {                                    \
        while (wdt::test_hang == (code)) \
            threads.yield();             \
    } while (0)
#else
#define WDT_TEST_HANG(code) ((void)0)
#endif
