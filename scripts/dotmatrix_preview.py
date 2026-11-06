"""Render dot matrix text as images, without the display connected.

Reads the firmware's dot matrix font (src/small_font.h, or any MD_MAX72XX font
table with --font) and lays text out like MD_Parola: 1 blank column between
characters, centered when it fits the 8 x 24 display, scrolled left when it
does not. Writes a PNG per static text and an animated GIF per scrolled text,
and prints the margins of static text.

    python3 scripts/dotmatrix_preview.py OFF LOADING --out previews/
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path
import re

ROWS = 8
COLS = 24
CHAR_SPACING = 1
DEFAULT_FONT = Path(__file__).resolve().parent.parent / "src" / "small_font.h"

Font = dict[int, list[int]]
Frame = list[list[bool]]


def parse_font(source: str) -> Font:
    """Parse the first MD_MAX72XX font table: header, then `width, col...` per code.

    The first table is the one selected by USE_NEW_FONT, the library default.
    """
    start = source.index("'F'")
    end = min(i for i in (source.find(m, start) for m in ("#else", "#endif", "};")) if i > 0)
    body = source[start:end]
    body = re.sub(r"//[^\n]*", "", body)
    tokens = [t.strip() for t in body.split(",") if t.strip()]
    version = int(tokens[1], 0)
    nums = [int(t, 0) for t in tokens[2:]]
    if version == 2:  # first, last as 16-bit big-endian, then height
        first, last, header = nums[0] << 8 | nums[1], nums[2] << 8 | nums[3], 5
    else:  # first, last, height
        first, last, header = nums[0], nums[1], 3
    values = nums[header:]
    font: Font = {}
    i = 0
    for code in range(first, last + 1):
        if i >= len(values):
            break
        width = values[i]
        font[code] = values[i + 1 : i + 1 + width]
        i += 1 + width
    return font


def text_columns(text: str, font: Font) -> list[int]:
    cols: list[int] = []
    for n, ch in enumerate(text):
        if n:
            cols.extend([0] * CHAR_SPACING)
        cols.extend(font.get(ord(ch), []))
    return cols


def columns_to_frame(cols: Sequence[int]) -> Frame:
    """Bit 0 of a column byte is the top row."""
    return [[bool(cols[c] >> r & 1) for c in range(len(cols))] for r in range(ROWS)]


def centered(cols: Sequence[int]) -> Frame:
    pad = (COLS - len(cols)) // 2
    full = [0] * pad + list(cols) + [0] * (COLS - pad - len(cols))
    return columns_to_frame(full)


def scroll_frames(cols: Sequence[int]) -> list[Frame]:
    """Enter from the right edge, leave past the left edge, one column per frame."""
    strip = [0] * COLS + list(cols) + [0] * COLS
    return [columns_to_frame(strip[s : s + COLS]) for s in range(len(cols) + COLS + 1)]


def render(text: str, font: Font) -> list[Frame]:
    cols = text_columns(text, font)
    return [centered(cols)] if len(cols) <= COLS else scroll_frames(cols)


def margins(frame: Frame) -> tuple[int, int, int, int] | None:
    """Blank (top, bottom, left, right) around the lit dots, or None when dark."""
    lit_rows = [r for r in range(ROWS) if any(frame[r])]
    lit_cols = [c for c in range(COLS) if any(frame[r][c] for r in range(ROWS))]
    if not lit_rows:
        return None
    return lit_rows[0], ROWS - 1 - lit_rows[-1], lit_cols[0], COLS - 1 - lit_cols[-1]


def to_image(frame: Frame, dot: int = 16):  # type: ignore[no-untyped-def]
    from PIL import Image, ImageDraw

    gap = dot // 4
    img = Image.new("RGB", (COLS * dot + gap, ROWS * dot + gap), (20, 20, 20))
    draw = ImageDraw.Draw(img)
    for r, row in enumerate(frame):
        for c, lit in enumerate(row):
            x, y = c * dot + gap, r * dot + gap
            color = (255, 40, 30) if lit else (55, 25, 25)
            draw.ellipse((x, y, x + dot - gap, y + dot - gap), fill=color)
    # Module seams every 8 columns.
    for m in range(1, COLS // 8):
        x = m * 8 * dot + gap // 2
        draw.line((x, 0, x, img.height), fill=(90, 90, 90))
    return img


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("texts", nargs="+")
    ap.add_argument("--font", type=Path, default=DEFAULT_FONT)
    ap.add_argument("--out", type=Path, default=Path("."))
    ap.add_argument("--ms", type=int, default=50, help="scroll time per column")
    args = ap.parse_args(argv)

    font = parse_font(args.font.read_text())
    args.out.mkdir(parents=True, exist_ok=True)
    for text in args.texts:
        frames = render(text, font)
        name = re.sub(r"[^A-Za-z0-9]+", "_", text).strip("_").lower() or "blank"
        if len(frames) == 1:
            path = args.out / f"{name}.png"
            to_image(frames[0]).save(path)
        else:
            path = args.out / f"{name}.gif"
            imgs = [to_image(f) for f in frames]
            imgs[0].save(path, save_all=True, append_images=imgs[1:], duration=args.ms, loop=0)
        width = len(text_columns(text, font))
        if len(frames) == 1:
            m = margins(frames[0])
            where = f"margins top/bottom/left/right {m}" if m else "blank"
        else:
            where = "scrolls"
        print(f"{text!r}: {width} columns, {where} -> {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
