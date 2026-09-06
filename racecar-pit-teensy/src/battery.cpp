#include "battery.h"

namespace
{
    struct SocPoint
    {
        uint16_t mv;
        uint8_t pct;
    };

    // 2S LiPo rest-voltage discharge curve, sampled at the knees.
    constexpr SocPoint curve[] = {
        {8400, 100},
        {8200, 88},
        {8000, 78},
        {7800, 64},
        {7700, 55},
        {7600, 40},
        {7500, 25},
        {7400, 13},
        {7200, 4},
        {7000, 0},
    };
    constexpr uint8_t curve_n = sizeof(curve) / sizeof(curve[0]);

    static_assert(curve[0].mv == batt_full_mv, "curve must start at batt_full_mv");
    static_assert(curve[curve_n - 1].mv == batt_empty_mv, "curve must end at batt_empty_mv");
}

uint8_t batterySocPercent(uint16_t pack_mv)
{
    if (pack_mv >= curve[0].mv)
        return curve[0].pct;

    for (uint8_t i = 1; i < curve_n; i++)
    {
        if (pack_mv >= curve[i].mv)
        {
            const uint16_t span_mv = curve[i - 1].mv - curve[i].mv;
            const uint8_t span_pct = curve[i - 1].pct - curve[i].pct;
            const uint32_t rise = (uint32_t)(pack_mv - curve[i].mv) * span_pct;
            return curve[i].pct + (uint8_t)(rise / span_mv);
        }
    }
    return 0;
}
