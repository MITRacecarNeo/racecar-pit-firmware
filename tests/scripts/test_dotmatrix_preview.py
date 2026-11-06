import dotmatrix_preview as dp
import pytest

FONT_V2 = """
MD_MAX72XX::fontType_t PROGMEM _sysfont[] =
{
#if USE_NEW_FONT
  'F', 2, 0, 65, 0, 67, 8,
  5, 126, 9, 9, 9, 126,  // 65 - 'A'
  1, 127,  // 66 - 'B'
  0,  // 67 - 'C'
#else
  'F', 1, 0, 255, 8,
  5, 0x7c, 0x12, 0x11, 0x12, 0x7c,
#endif
};
"""


@pytest.fixture
def font():
    return dp.parse_font(FONT_V2)


def test_parse_reads_only_the_first_table(font):
    assert font == {65: [126, 9, 9, 9, 126], 66: [127], 67: []}


def test_characters_are_one_blank_column_apart(font):
    assert dp.text_columns("AB", font) == [126, 9, 9, 9, 126, 0, 127]


def test_column_bit_zero_is_the_top_row():
    frame = dp.columns_to_frame([0b10000001])
    assert [row[0] for row in frame] == [True] + [False] * 6 + [True]


def test_short_text_is_one_centered_frame(font):
    frames = dp.render("A", font)
    assert len(frames) == 1
    lit_cols = [c for c in range(dp.COLS) if any(frames[0][r][c] for r in range(dp.ROWS))]
    assert lit_cols == list(range(9, 14))  # 5 columns, (24 - 5) // 2 = 9 blank on the left


def test_long_text_scrolls_in_and_out(font):
    frames = dp.render("A" * 5, font)  # 29 columns
    assert len(frames) == 29 + dp.COLS + 1
    assert not any(any(row) for row in frames[0])
    assert not any(any(row) for row in frames[-1])


def test_text_of_exactly_display_width_stays_static():
    font = {ord("X"): [1] * dp.COLS}
    assert len(dp.render("X", font)) == 1


def test_version_1_header_starts_at_its_first_code():
    src = "x[] = {\n  'F', 1, 65, 66, 8,\n  1, 2,  // 65 'A'\n  0,  // 66\n};\n"
    assert dp.parse_font(src) == {65: [2], 66: []}


@pytest.fixture
def firmware_font():
    return dp.parse_font(dp.DEFAULT_FONT.read_text())


@pytest.mark.parametrize("text", ["OFF", "LOAD", "WARN", "HIGH", "STALL"])
def test_static_display_words_have_equal_margins(firmware_font, text):
    (frame,) = dp.render(text, firmware_font)
    top, bottom, left, right = dp.margins(frame)
    assert (top, bottom) == (1, 1)
    assert left == right


def test_firmware_glyphs_stay_on_rows_one_to_six(firmware_font):
    assert all(col & 0b10000001 == 0 for cols in firmware_font.values() for col in cols)


def test_blank_frame_has_no_margins():
    assert dp.margins([[False] * dp.COLS for _ in range(dp.ROWS)]) is None
