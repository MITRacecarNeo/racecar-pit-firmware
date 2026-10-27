#pragma once
#include <stdint.h>

// Which contexts have run since the watchdog was last refreshed. Each context
// sets its bit every loop; the drive tick refreshes the watchdog only once
// every required bit is set, then starts a new round. A context that stops
// running stops the refreshes. Arduino-free and host tested; beat() is safe
// from threads and interrupts.
class Heartbeat
{
public:
    explicit constexpr Heartbeat(uint32_t required) : required_(required) {}

    void beat(uint32_t bit) { __atomic_fetch_or(&seen_, bit, __ATOMIC_RELAXED); }

    bool allSeen() const { return (__atomic_load_n(&seen_, __ATOMIC_RELAXED) & required_) == required_; }

    // True when every required context beat since the last true; clears them
    // for the next round. A beat landing between the check and the clear is
    // lost, which only delays that context to the next round.
    bool collect()
    {
        if (!allSeen())
            return false;
        __atomic_fetch_and(&seen_, ~required_, __ATOMIC_RELAXED);
        return true;
    }

    // Required contexts that have not beaten this round.
    uint32_t missing() const { return required_ & ~__atomic_load_n(&seen_, __ATOMIC_RELAXED); }

private:
    uint32_t seen_ = 0;
    const uint32_t required_;
};
