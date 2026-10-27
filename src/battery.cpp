#include "battery.h"

namespace
{
    struct SocPoint
    {
        uint16_t mv;
        uint8_t pct;
    };

    // Measured 2S discharge (5000 mAh) at the INA226, under the car's idle
    // load of about 2.4 A, so no rest-voltage correction. A full battery
    // reads about 8.19 V under that load, hence 100% down to 8200 mV.
    constexpr SocPoint curve[] = {
        {8400, 100},
        {8200, 100},
        {8110, 95},
        {8050, 90},
        {7860, 80},
        {7740, 70},
        {7620, 60},
        {7545, 50},
        {7485, 40},
        {7450, 30},
        {7395, 20},
        {7305, 10},
        {7280, 5},
        {7195, 2},
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
