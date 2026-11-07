#pragma once
#include <stdint.h>

// "NEO V2" for the 8 x 24 dot matrix: the narrow font (src/small_font.h) with
// a 2-column gap between the words and a 1-dot margin on every side. One byte
// per column, left to right; bit 0 is the top row. Checked against the font
// by tests/scripts/test_dotmatrix_preview.py.
constexpr uint8_t neo_v2_logo[24] = {
    0, 126, 12, 48, 126, 0, 126, 74, 66, 0, 60, 66, 66, 60, 0, 0, 62, 64, 62, 0, 98, 82, 76, 0};
