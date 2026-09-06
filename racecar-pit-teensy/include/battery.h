#pragma once
#include <stdint.h>

// 2S LiPo state of charge, Arduino-free and unit tested under test/.

constexpr uint16_t batt_empty_mv = 7000; // 3.50 V/cell
constexpr uint16_t batt_full_mv = 8400;  // 4.20 V/cell

// Pack voltage to remaining charge, 0-100%. The 2S discharge curve is flat
// between roughly 7.6 and 8.0 V and collapses below 7.4 V, so a bar drawn
// linearly on voltage sits near full for most of a run and then empties in
// seconds; this tracks charge instead.
uint8_t batterySocPercent(uint16_t pack_mv);
