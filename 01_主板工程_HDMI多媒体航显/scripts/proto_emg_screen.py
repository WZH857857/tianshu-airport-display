# -*- coding: utf-8 -*-
"""Prototype for the ADV22 emergency screen (software mock of the RTL).

Renders the same geometry that ui_overlay.v will use, so the fire symbol and
the running-man exit pictogram can be checked before a TD build is spent.
"""

from PIL import Image

W, H = 640, 480
img = Image.new("RGB", (W, H))
px = img.load()


def in_slant(x, y, y0, y1, x0, x1, half):
    if y < y0 or y > y1 or (y1 <= y0):
        return False
    dy = y1 - y0
    num = (x1 - x0) * (y - y0)
    shift = num // dy if num >= 0 else -((-num) // dy)
    return abs(x - (x0 + shift)) <= half


def seg_dist(x, y, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    l2 = dx * dx + dy * dy
    if l2 == 0:
        return ((x - x0) ** 2 + (y - y0) ** 2) ** 0.5
    t = ((x - x0) * dx + (y - y0) * dy) / l2
    t = max(0.0, min(1.0, t))
    px_, py_ = x0 + t * dx, y0 + t * dy
    return ((x - px_) ** 2 + (y - py_) ** 2) ** 0.5


def fire_icon(x, y):
    """Fire warning pictogram, 0 = outside, 1 = white flame on red."""
    # Teardrop flame built from a five segment width profile.  Every slope is
    # a shift so the hardware version needs no divider.
    vy = y + 160                       # panel local, 0..319
    if 60 <= vy <= 238:
        if vy < 96:
            w = 1 + (vy - 60) // 8
        elif vy < 140:
            w = 5 + (vy - 96) // 4
        elif vy < 178:
            w = 16 + ((vy - 140) * 3) // 8
        elif vy < 200:
            w = 30 + (vy - 178) // 16
        elif vy < 222:
            w = 32 - ((vy - 200) * 3) // 8
        else:
            w = 24 - ((vy - 222) * 5) // 4
        if abs(x) <= w:
            return 1
    return 0


def exit_icon(x, y):
    """Running man + door, 0 = outside, 1 = white on green."""
    # head
    if (x - 2) ** 2 + (y + 62) ** 2 <= 12 * 12:
        return 1
    # body
    if in_slant(x, y, -48, 22, -6, 6, 11):
        return 1
    # arms
    if in_slant(x, y, -40, -20, -22, -34, 5):
        return 1
    if in_slant(x, y, -44, -12, 12, 26, 5):
        return 1
    # legs
    if in_slant(x, y, 20, 66, -4, -30, 4):
        return 1
    if in_slant(x, y, 20, 46, 4, 30, 4):
        return 1
    if in_slant(x, y, 46, 66, 30, 44, 4):
        return 1
    # door frame
    if -78 <= y <= 2 and 22 <= x <= 30:
        return 1
    if -78 <= y <= -70 and 22 <= x <= 60:
        return 1
    return 0


def rounded_rect(x, y, half_w, half_h, r):
    """Rounded rectangle in centre-origin coordinates."""
    dx = max(abs(x) - (half_w - r), 0)
    dy = max(abs(y) - (half_h - r), 0)
    return dx * dx + dy * dy <= r * r


for y in range(H):
    for x in range(W):
        rgb = (0x60, 0x08, 0x08)          # dark red field

        if y < 16 or y >= 464:            # hazard rails
            d = x + y
            rgb = (0xFF, 0xC6, 0x1A) if ((d >> 4) ^ (d >> 3)) & 1 else (0x18, 0x18, 0x18)

        # fire warning panel
        fx, fy = x - 140, y - 258
        if rounded_rect(fx, fy, 120, 160, 18):
            rgb = (0xC8, 0x10, 0x10)
            if fire_icon(fx, fy):
                rgb = (0xFF, 0xFF, 0xFF)

        # exit sign panel
        ex, ey = x - 452, y - 258
        if rounded_rect(ex, ey, 100, 160, 16):
            rgb = (0x00, 0x78, 0x3C)
            if exit_icon(ex, ey):
                rgb = (0xFF, 0xFF, 0xFF)

        # texts
        if 456 <= x < 520 and 92 <= y < 108:
            d = x + y
            rgb = (0xFF, 0xC6, 0x1A) if ((d >> 4) ^ (d >> 3)) & 1 else (0x18, 0x18, 0x18)
        if 396 <= x < 444 and 92 <= y < 108:
            rgb = (0x18, 0x18, 0x18)

        px[x, y] = rgb

img.save(r"build/adv22_emg_proto.png")
print("wrote build/adv22_emg_proto.png")
