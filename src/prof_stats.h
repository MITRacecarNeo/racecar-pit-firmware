#pragma once
#include <stddef.h>
#include <stdint.h>

// Statistics for the profiling build (RCN_PROFILE). Arduino-free and host
// tested; timing comes from profile.cpp. Line format: docs/architecture.md.

namespace prof
{
    constexpr uint32_t bin_us = 50;     // histogram bin width
    constexpr uint16_t bins = 400;      // covers 0 to 20 ms; longer lands in the last bin
}

// Intervals between successive events of one context (loop starts, sends).
struct PeriodStat
{
    uint16_t hist[prof::bins];
    uint32_t n;
    uint32_t sum_us;
    uint32_t max_us;

    void clear();
    void add(uint32_t us);
    // Upper edge of the bin holding quantile q (0..1), in us; 0 when empty.
    uint32_t quantile(float q) const;
};

// Durations of one code section, or of one latency.
struct SectionStat
{
    uint32_t n;
    uint32_t sum_us;
    uint32_t max_us;

    void clear();
    void add(uint32_t us);
};

// One report line each, without a trailing newline; return the length, or 0
// when `cap` is too small:
//   P,<t_ms>,period,<name>,<n>,<sum_us>,<max_us>,<p50_us>,<p99_us>
//   P,<t_ms>,section,<name>,<n>,<sum_us>,<max_us>
//   P,<t_ms>,counter,<name>,<value>
size_t formatPeriod(char *buf, size_t cap, uint32_t t_ms, const char *name, const PeriodStat &s);
size_t formatSection(char *buf, size_t cap, uint32_t t_ms, const char *name, const SectionStat &s);
size_t formatCounter(char *buf, size_t cap, uint32_t t_ms, const char *name, uint32_t value);
