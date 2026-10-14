#!/usr/bin/env python3
"""Render the box-label glyphs from a Windows font into the bit tables
that box_overlay.v embeds.

The overlay draws its labels in vision space (320x180 stretched to 640x480),
so a glyph cell of 24x18 vision pixels lands on the monitor as a 48x48
square.  ``--verilog`` prints the flat case table pasted into
box_overlay.v, ``--python`` prints the same rows for machine_view.py.

    python gen_glyphs.py --verilog
"""

from __future__ import annotations

import argparse

import numpy as np
from PIL import Image, ImageDraw, ImageFont

CHARS = ["摔", "倒"]
CW, CH, PX = 24, 18, 20
FONT = "C:/Windows/Fonts/simhei.ttf"
THRESH = 96
OFFSET = (0, -1)


def rows_of(ch: str) -> list[int]:
    font = ImageFont.truetype(FONT, PX)
    img = Image.new("L", (CW, CH), 0)
    ImageDraw.Draw(img).text(OFFSET, ch, fill=255, font=font)
    mask = np.asarray(img) > THRESH
    return [int("".join("1" if b else "0" for b in row), 2) for row in mask]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--verilog", action="store_true")
    ap.add_argument("--python", action="store_true")
    ap.add_argument("--ascii", action="store_true")
    args = ap.parse_args()
    tables = [rows_of(c) for c in CHARS]
    if args.verilog:
        for ci, rows in enumerate(tables):
            for ri, v in enumerate(rows):
                print(f"            6'd{ci * 32 + ri}: bits = 24'h{v:06X};")
    if args.python:
        print("GLYPHS = [")
        for rows in tables:
            body = ", ".join(f"0x{v:06X}" for v in rows)
            print(f"    [{body}],")
        print("]")
    if args.ascii:
        for ch, rows in zip(CHARS, tables):
            print(f"--- {ch} ---")
            for v in rows:
                print("".join("#" if v & (1 << (CW - 1 - b)) else "."
                              for b in range(CW)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
