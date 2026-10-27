#pragma once
#include <stdint.h>

// 2S LiPo state of charge, Arduino-free and unit tested under test/.

constexpr uint16_t batt_empty_mv = 7000; // 3.50 V/cell
constexpr uint16_t batt_full_mv = 8400;  // 4.20 V/cell

// Battery voltage under load to remaining charge, 0-100%, from a measured
// discharge curve. Half the charge is used by 7.55 V and the last 10% goes
// between 7.3 and 7.0 V, so a bar drawn linearly on voltage reads far from
// the charge left; this tracks charge instead.
uint8_t batterySocPercent(uint16_t pack_mv);
