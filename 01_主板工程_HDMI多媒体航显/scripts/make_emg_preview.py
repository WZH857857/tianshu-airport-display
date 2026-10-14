# -*- coding: utf-8 -*-
"""Render the fire emergency overlay dump into a PNG contact sheet.

sim/tb_emg_dump.v runs the real ui_overlay RTL with every ordinary layer
disabled and writes one raw 640x480 RGB frame per animation phase.  This
script turns the two phases into a single picture so the alarm screen can be
checked without a monitor.

Usage:
    python scripts/make_emg_preview.py [--dir .] [--out build/emg_preview.png]
"""

from __future__ import annotations

import argparse
import os

from PIL import Image, ImageDraw


WIDTH = 640
HEIGHT = 480
FRAME_BYTES = WIDTH * HEIGHT * 3


def load_raw(path: str) -> Image.Image:
    with open(path, "rb") as handle:
        data = handle.read()
    if len(data) != FRAME_BYTES:
        raise SystemExit(
            "%s is %d bytes, expected %d" % (path, len(data), FRAME_BYTES)
        )
    return Image.frombytes("RGB", (WIDTH, HEIGHT), data)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default=".")
    parser.add_argument("--out", default=os.path.join("build", "emg_preview.png"))
    args = parser.parse_args()

    dark = load_raw(os.path.join(args.dir, "emg_dark.raw"))
    flash = load_raw(os.path.join(args.dir, "emg_flash.raw"))

    sheet = Image.new("RGB", (WIDTH * 2 + 24, HEIGHT + 32), (24, 26, 30))
    sheet.paste(dark, (8, 24))
    sheet.paste(flash, (WIDTH + 16, 24))
    draw = ImageDraw.Draw(sheet)
    draw.text((8, 6), "emergency overlay - dark phase", fill=(255, 255, 255))
    draw.text((WIDTH + 16, 6), "emergency overlay - flash phase",
              fill=(255, 255, 255))

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    sheet.save(args.out)
    print("wrote", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
