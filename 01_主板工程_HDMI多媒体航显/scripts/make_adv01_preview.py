"""Compose the ADV01 overlay dump on top of a real airport BMP.

sim/tb_ui_overlay_dump.v renders 16 frames of rtl/ui_overlay.v with a white
background.  This script classifies the known overlay colors, re-creates the
same fixed-point alpha blend on the airport picture, and writes both the
expected monitor image and a short animation that shows the two ticker speeds:

    0x00E0FF  ANLU TIS logo text
    0xFFD060  airport TIME MM:SS text
    0x7FE7FF  slow airport-operation ticker text
    0xFFFFFF  flight-information subtitle text
    0x808080  fixed 50% dark panels
    row-specific grey  subtitle alpha-gradient background
"""

import glob
import os

from PIL import Image

WIDTH = 640
HEIGHT = 480
ZOOM = 2

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
DUMP_GLOB = os.path.join(ROOT, "sim", "build_adv01",
                         "adv01_overlay_frame_*.raw")
PHOTO = os.path.join(ROOT, "sdcard", "IMG03.BMP")
OUT = os.path.join(ROOT, "doc", "adv01_overlay_preview.png")
GIF_OUT = os.path.join(ROOT, "doc", "adv01_parallax_preview.gif")

LOGO_TEXT = b"\x00\xE0\xFF"
TIME_TEXT = b"\xFF\xD0\x60"
TICK_TEXT = b"\x7F\xE7\xFF"
SUB_TEXT = b"\xFF\xFF\xFF"
WHITE = b"\xFF\xFF\xFF"

TOP_X0, TOP_Y0, TOP_W, TOP_H = 12, 12, 224, 32
TIME_X0, TIME_Y0, TIME_W, TIME_H = 456, 12, 168, 32
TICK_X0, TICK_Y0, TICK_W, TICK_H = 0, 432, 640, 8
SUB_X0, SUB_Y0, SUB_W, SUB_H = 0, 448, 640, 32
SUB_TEXT_Y0, SUB_TEXT_Y1 = 456, 472

PANEL_ALPHA = 32
TICK_ALPHA = 20


def blend_black(rgb, alpha):
    """Match rtl/ui_overlay.v blend_black()."""
    inv = 64 - alpha
    return bytes(((channel * inv + 32) >> 6) for channel in rgb)


def subtitle_alpha(row):
    """Match rtl/ui_overlay.v sub_bg_alpha()."""
    ramp = row if row < 16 else 31 - row
    return 12 + 2 * ramp


def load_overlay(path):
    with open(path, "rb") as handle:
        overlay = handle.read()
    expected = WIDTH * HEIGHT * 3
    if len(overlay) != expected:
        raise SystemExit("%s is %d bytes, expected %d" %
                         (path, len(overlay), expected))
    return overlay


def compose(overlay, src):
    expected = WIDTH * HEIGHT * 3
    out = bytearray(expected)
    stats = {
        "logo_pixels": 0,
        "time_pixels": 0,
        "ticker_pixels": 0,
        "subtitle_pixels": 0,
        "alpha_pixels": 0,
        "gradient_levels": set(),
    }

    for index in range(WIDTH * HEIGHT):
        offset = index * 3
        y, x = divmod(index, WIDTH)
        pixel = overlay[offset:offset + 3]
        base = src[offset:offset + 3]

        in_top = (TOP_X0 <= x < TOP_X0 + TOP_W) and (TOP_Y0 <= y < TOP_Y0 + TOP_H)
        in_time = (TIME_X0 <= x < TIME_X0 + TIME_W) and (TIME_Y0 <= y < TIME_Y0 + TIME_H)
        in_tick = (TICK_X0 <= x < TICK_X0 + TICK_W) and (TICK_Y0 <= y < TICK_Y0 + TICK_H)
        in_sub = (SUB_X0 <= x < SUB_X0 + SUB_W) and (SUB_Y0 <= y < SUB_Y0 + SUB_H)

        if in_top and pixel == LOGO_TEXT:
            out[offset:offset + 3] = LOGO_TEXT
            stats["logo_pixels"] += 1
        elif in_time and pixel == TIME_TEXT:
            out[offset:offset + 3] = TIME_TEXT
            stats["time_pixels"] += 1
        elif in_tick and pixel == TICK_TEXT:
            out[offset:offset + 3] = TICK_TEXT
            stats["ticker_pixels"] += 1
        elif in_sub and (SUB_TEXT_Y0 <= y < SUB_TEXT_Y1) and pixel == SUB_TEXT:
            out[offset:offset + 3] = SUB_TEXT
            stats["subtitle_pixels"] += 1
        elif in_top or in_time:
            alpha = PANEL_ALPHA
            expected_bg = blend_black(WHITE, alpha)
            if pixel == expected_bg:
                out[offset:offset + 3] = blend_black(base, alpha)
                stats["alpha_pixels"] += 1
            else:
                out[offset:offset + 3] = base
        elif in_tick:
            expected_bg = blend_black(WHITE, TICK_ALPHA)
            if pixel == expected_bg:
                out[offset:offset + 3] = blend_black(base, TICK_ALPHA)
                stats["alpha_pixels"] += 1
            else:
                out[offset:offset + 3] = base
        elif in_sub:
            alpha = subtitle_alpha(y - SUB_Y0)
            expected_bg = blend_black(WHITE, alpha)
            stats["gradient_levels"].add(alpha)
            if pixel == expected_bg:
                out[offset:offset + 3] = blend_black(base, alpha)
                stats["alpha_pixels"] += 1
            else:
                out[offset:offset + 3] = base
        else:
            out[offset:offset + 3] = base

    return bytes(out), stats


def main():
    frame_paths = sorted(glob.glob(DUMP_GLOB))
    if not frame_paths:
        raise SystemExit("no overlay frames found at %s" % DUMP_GLOB)

    photo = Image.open(PHOTO).convert("RGB")
    if photo.size != (WIDTH, HEIGHT):
        raise SystemExit("photo is %s, expected %dx%d" %
                         (photo.size, WIDTH, HEIGHT))
    src = photo.tobytes()

    static_bytes, stats = compose(load_overlay(frame_paths[0]), src)
    image = Image.frombytes("RGB", (WIDTH, HEIGHT), static_bytes)
    image = image.resize((WIDTH * ZOOM, HEIGHT * ZOOM), Image.NEAREST)
    image.save(OUT)

    gif_frames = []
    for path in frame_paths:
        frame_bytes, unused = compose(load_overlay(path), src)
        gif_frames.append(Image.frombytes("RGB", (WIDTH, HEIGHT),
                                          frame_bytes))
    gif_frames[0].save(GIF_OUT, save_all=True, append_images=gif_frames[1:],
                       duration=80, loop=0, optimize=True)

    print("logo pixels       : %d" % stats["logo_pixels"])
    print("time pixels       : %d" % stats["time_pixels"])
    print("parallax pixels   : %d" % stats["ticker_pixels"])
    print("subtitle pixels   : %d" % stats["subtitle_pixels"])
    print("alpha pixels      : %d" % stats["alpha_pixels"])
    print("alpha levels      : %d" % len(stats["gradient_levels"]))
    print("written           : %s" % OUT)
    print("animation         : %s (%d frames)" % (GIF_OUT, len(gif_frames)))


if __name__ == "__main__":
    main()
