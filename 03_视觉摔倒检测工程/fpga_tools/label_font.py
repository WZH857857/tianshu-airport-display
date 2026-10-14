#!/usr/bin/env python3
"""The picture-box label font from ``rtl/hdmi/box_overlay.v``, in Python.

Two tag geometries live in ``box_overlay.v``, both in *vision* pixels (the
640x480 picture stretches one vision column into two picture columns and one
vision row into 8/3 picture rows):

* the person readout ``87%`` -- a 3x5 digit font and a 5x5 percent sign, one
  font pixel per vision pixel, packed into three 8-wide tiles: 24 x 5 vision
  pixels, i.e. 48 x 13 picture pixels, the size of a detector tag;
* the fall alarm ``摔倒`` -- two 24x18 bitmap cells rendered by
  ``gen_glyphs.py``: 48 x 18 vision pixels, i.e. 96 x 48 picture pixels.

``box_overlay.v`` keeps a Chinese cell's leftmost column in bit 23 (``cell_row``
reproduces that bit for bit) and places a digit in tile columns 2..4 and the
percent sign in 1..5 (``person_mask`` does the same).  So a test bench dump of
the RTL can be compared against this model (``label_font_check.py``) and the
demo renderer (``machine_view.py``) draws the tag the board actually shows.
"""

from __future__ import annotations

import numpy as np

import gen_glyphs

# 3x5 font, five rows of three bits, row 0 first.
DIGIT_PATTERN = [
    0b111_101_101_101_111,
    0b010_110_010_010_111,
    0b111_001_111_100_111,
    0b111_001_111_001_111,
    0b101_101_111_001_001,
    0b111_100_111_001_111,
    0b111_100_111_101_111,
    0b111_001_001_001_001,
    0b111_101_111_101_111,
    0b111_101_111_001_111,
]

# 5x5 percent sign: two dots, a diagonal, then two more dots.
PERCENT_PATTERN = 0b11001_11010_00100_01011_10011

GLYPH_CHARS = ["摔", "倒"]
GLYPHS = [gen_glyphs.rows_of(ch) for ch in GLYPH_CHARS]

CELL_W, CELL_H = 24, 18

# Person tags: three tiles of eight vision columns, five font rows tall.
TILE_W = 8
TAG_CELLS = 3
TAG_W, TAG_H = TILE_W * TAG_CELLS, 5
# Where a glyph sits inside its tile.
DIGIT_COL, PERCENT_COL = 2, 1

# ch codes shared with the RTL.
CH_SHOUAI, CH_DAO, CH_DIGIT0, CH_DIGIT9, CH_PERCENT = 0, 1, 2, 11, 12


def digit_group(d: int, row: int) -> int:
    """One font row of a digit: 3 pixels, leftmost in bit 2."""
    if row >= TAG_H:
        return 0
    return (DIGIT_PATTERN[d] >> (12 - 3 * row)) & 0b111


def percent_group(row: int) -> int:
    """One font row of the percent sign: 5 pixels, leftmost in bit 4."""
    if row >= TAG_H:
        return 0
    return (PERCENT_PATTERN >> (20 - 5 * row)) & 0b11111


def cell_row(ch: int, row: int) -> int:
    """One 24-bit row of a Chinese alarm cell, bit 23 = leftmost column."""
    return GLYPHS[ch][row]


def cell_mask(ch: int) -> np.ndarray:
    """(18, 24) uint8 mask of one Chinese alarm cell."""
    out = np.zeros((CELL_H, CELL_W), np.uint8)
    for r in range(CELL_H):
        bits = cell_row(ch, r)
        for c in range(CELL_W):
            out[r, c] = (bits >> (CELL_W - 1 - c)) & 1
    return out


def score_chars(score: int) -> list[int]:
    """`87` -> the two RTL glyph codes for a 0..99 confidence."""
    score = max(0, min(99, int(score)))
    return [CH_DIGIT0 + score // 10, CH_DIGIT0 + score % 10]


def tag_mask(cells: list[int]) -> np.ndarray:
    """(18, 24*len(cells)) mask of a chain of Chinese alarm cells."""
    out = np.zeros((CELL_H, CELL_W * len(cells)), np.uint8)
    for k, ch in enumerate(cells):
        out[:, k * CELL_W:(k + 1) * CELL_W] = cell_mask(ch)
    return out


def person_tag(score: int) -> np.ndarray:
    """The (5, 24) `87%` tag a tracked person carries."""
    out = np.zeros((TAG_H, TAG_W), np.uint8)
    cells = score_chars(score) + [CH_PERCENT]
    for tile, ch in enumerate(cells):
        base = tile * TILE_W
        if ch == CH_PERCENT:
            group, col = percent_group, PERCENT_COL
            for r in range(TAG_H):
                g = group(r)
                for k in range(5):
                    out[r, base + col + k] = (g >> (4 - k)) & 1
        else:
            d = ch - CH_DIGIT0
            for r in range(TAG_H):
                g = digit_group(d, r)
                for k in range(3):
                    out[r, base + DIGIT_COL + k] = (g >> (2 - k)) & 1
    return out


def alarm_tag() -> np.ndarray:
    """The (18, 48) `摔倒` tag a confirmed fall carries."""
    return tag_mask([CH_SHOUAI, CH_DAO])


def ascii(mask: np.ndarray, marks: str = "#") -> str:
    """Debug view: one character per font pixel."""
    return "\n".join("".join(marks if v else "." for v in row)
                     for row in mask)
