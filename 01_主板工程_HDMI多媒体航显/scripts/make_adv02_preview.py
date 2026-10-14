"""Render the ADV02 fade transition from the real RTL behaviour.

The picture gain is the 5-bit fade level that rtl/fade_ctrl.v drives with
STEP = 2, so a transition is 16 video frames down and 16 frames back up.
This script applies exactly that gain to two airport photos and then runs the
same overlay composition as scripts/make_adv01_preview.py, which keeps the
overlay text at full brightness while the photo behind it fades - the effect
the board actually shows.

Outputs:
    doc/adv02_fade_preview.gif   the transition at roughly real speed
    doc/adv02_fade_steps.png     the key gain steps as a contact sheet
"""

import glob
import os

from PIL import Image, ImageDraw, ImageFont

import make_adv01_preview as adv01

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))

PHOTO_A = os.path.join(ROOT, "sdcard", "IMG03.BMP")
PHOTO_B = os.path.join(ROOT, "sdcard", "IMG04.BMP")
GIF_OUT = os.path.join(ROOT, "doc", "adv02_fade_preview.gif")
SHEET_OUT = os.path.join(ROOT, "doc", "adv02_fade_steps.png")

STEP = 2
LEVEL_MAX = 31
FRAME_MS = 20         # 16 video frames at 60 Hz is about 267 ms
LEAD_FRAMES = 4       # steady picture before and after the transition


def fade_out_levels():
    levels = []
    level = LEVEL_MAX
    while level > STEP:
        levels.append(level)
        level -= STEP
    levels.append(0)              # the frame store swaps at black
    return levels


def fade_in_levels():
    levels = [0]
    level = 0
    while level < LEVEL_MAX - STEP:
        level += STEP
        levels.append(level)
    levels.append(LEVEL_MAX)
    return levels


def scale(photo_bytes, level):
    if level >= LEVEL_MAX:
        return photo_bytes
    return bytes((channel * level + LEVEL_MAX // 2) // LEVEL_MAX
                 for channel in photo_bytes)


def render(photo_bytes, overlay_bytes, level):
    frame, unused = adv01.compose(overlay_bytes, scale(photo_bytes, level))
    return Image.frombytes("RGB", (adv01.WIDTH, adv01.HEIGHT), frame)


def label(tile, text):
    canvas = Image.new("RGB", (tile.width, tile.height + 18), (16, 16, 16))
    canvas.paste(tile, (0, 18))
    draw = ImageDraw.Draw(canvas)
    draw.text((6, 4), text, fill=(235, 235, 235),
              font=ImageFont.load_default())
    return canvas


def main():
    overlay_paths = sorted(glob.glob(adv01.DUMP_GLOB))
    if not overlay_paths:
        raise SystemExit("no overlay frames; run tb_ui_overlay_dump first")

    photo_a = Image.open(PHOTO_A).convert("RGB").tobytes()
    photo_b = Image.open(PHOTO_B).convert("RGB").tobytes()
    overlays = [adv01.load_overlay(path) for path in overlay_paths]

    def overlay_for(index):
        return overlays[index % len(overlays)]

    frames = []

    for i in range(LEAD_FRAMES):
        frames.append(render(photo_a, overlay_for(i), LEVEL_MAX))

    for i, level in enumerate(fade_out_levels()):
        frames.append(render(photo_a, overlay_for(LEAD_FRAMES + i), level))

    for i, level in enumerate(fade_in_levels()):
        frames.append(render(photo_b, overlay_for(LEAD_FRAMES + 16 + i), level))

    for i in range(LEAD_FRAMES):
        frames.append(render(photo_b, overlay_for(LEAD_FRAMES + 33 + i),
                             LEVEL_MAX))

    frames[0].save(GIF_OUT, save_all=True, append_images=frames[1:],
                   duration=FRAME_MS, loop=0, optimize=True)

    picks = [
        ("FADE OUT level 31", render(photo_a, overlays[4], 31)),
        ("FADE OUT level 21", render(photo_a, overlays[4], 21)),
        ("FADE OUT level 11", render(photo_a, overlays[4], 11)),
        ("BLACK     level 0", render(photo_a, overlays[6], 0)),
        ("FADE IN  level 11", render(photo_b, overlays[8], 11)),
        ("FADE IN  level 21", render(photo_b, overlays[8], 21)),
        ("FADE IN  level 31", render(photo_b, overlays[8], 31)),
    ]

    tile_w = adv01.WIDTH
    tile_h = adv01.HEIGHT
    sheet = Image.new("RGB", (tile_w * len(picks), tile_h + 18),
                      (16, 16, 16))
    for index, (text, tile) in enumerate(picks):
        sheet.paste(label(tile, text), (index * tile_w, 0))
    sheet.save(SHEET_OUT)

    print("fade-out frames    : %d" % len(fade_out_levels()))
    print("fade-in frames     : %d" % len(fade_in_levels()))
    print("animation frames   : %d (%d ms each)" % (len(frames), FRAME_MS))
    print("written            : %s" % GIF_OUT)
    print("written            : %s" % SHEET_OUT)


if __name__ == "__main__":
    main()
