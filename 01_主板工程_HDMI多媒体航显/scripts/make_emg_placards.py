# -*- coding: utf-8 -*-
"""Build the two fire emergency placards that the FPGA reads from the TF card.

Output (640x480, 24 bit BI_RGB bottom-up BMP, the format bmp_stream_parser.v
expects):

    sdcard/EMGFIRE.BMP   fire warning poster: the standard "当心火灾" sign
                         (black bordered yellow triangle with the black flame
                         and hearth bar) on a red hazard field
    sdcard/EMGEXIT.BMP   the standard green 安全出口 sign: text block on the
                         left, door with the running man in the middle and an
                         arrow pointing out on the right

The pictograms are the real standard artwork rather than a redraw: the warning
triangle is the ISO 7010 W021 sign, the door / runner / arrow are cut straight
out of the 安全出口 sign by scripts/extract_exit_sign_art.py.  Both live in
assets/.

Both files must sit in the root directory of the card.  They are matched by
name during the directory scan, kept out of the picture playlist and shown by
player_ctrl while the flame alarm is latched.
"""

import os

from PIL import Image, ImageDraw, ImageFont

W, H = 640, 480
HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "sdcard")
ASSET_DIR = os.path.join(HERE, "..", "assets")
PREVIEW_DIR = os.path.join(HERE, "..", "build")

FONT_BOLD = "C:/Windows/Fonts/msyhbd.ttc"
FONT_REG = "C:/Windows/Fonts/msyh.ttc"

RED_FIELD = (0xA5, 0x0F, 0x0E)
GREEN_FIELD = (29, 137, 44)      # the green the 安全出口 signs actually use
HAZARD_YELLOW = (0xFF, 0xC6, 0x1A)
HAZARD_BLACK = (0x18, 0x18, 0x18)

FIRE_SIGN = "fire_warning_triangle.png"
EXIT_MAN = "exit_man.png"
EXIT_ARROW = "exit_arrow.png"


def hazard_rail(draw, y0, y1):
    for y in range(y0, y1):
        for x in range(W):
            d = x + y
            colour = HAZARD_YELLOW if ((d >> 4) ^ (d >> 3)) & 1 else HAZARD_BLACK
            draw.point((x, y), fill=colour)


def load_art(name):
    """Load one of the standard sign pictograms and trim its empty margin."""
    img = Image.open(os.path.join(ASSET_DIR, name))
    if "A" in img.getbands():
        art = img.convert("RGBA")
    else:
        # the extracted cut outs are plain greyscale masks: the grey level is
        # the coverage, so it has to become the alpha channel
        grey = img.convert("L")
        art = Image.merge("RGBA", (Image.new("L", grey.size, 255),) * 3 +
                          (grey,))
    box = art.getchannel("A").point(lambda v: 255 if v > 8 else 0).getbbox()
    return art.crop(box) if box else art


def paste_art(base, art, x, y, colour=None):
    """Paste artwork through its alpha channel, optionally recolouring it -
    the exit sign carries white pictograms on green, so the cut outs are
    flattened to white here."""
    layer = Image.new("RGB", art.size, colour) if colour else art
    base.paste(layer, (x, y), art)


def centred_text(draw, text, font, y, width=W, fill=(255, 255, 255), outline=2):
    box = draw.textbbox((0, 0), text, font=font)
    x = (width - (box[2] - box[0])) // 2 - box[0]
    for dx in range(-outline, outline + 1):
        for dy in range(-outline, outline + 1):
            if dx or dy:
                draw.text((x + dx, y + dy), text, font=font, fill=(0, 0, 0))
    draw.text((x, y), text, font=font, fill=fill)


def fit_font(font_path, text, target_w, hi=64, lo=28):
    probe = ImageDraw.Draw(Image.new("L", (1, 1)))
    for size in range(hi, lo - 1, -1):
        font = ImageFont.truetype(font_path, size)
        box = probe.textbbox((0, 0), text, font=font)
        if box[2] - box[0] <= target_w:
            return font
    return ImageFont.truetype(font_path, lo)


def text_size(draw, text, font):
    box = draw.textbbox((0, 0), text, font=font)
    return box[2] - box[0], box[3] - box[1]


def spaced_text(draw, text, font, x, y, gap, fill=(255, 255, 255)):
    for ch in text:
        draw.text((x, y), ch, font=font, fill=fill)
        box = draw.textbbox((0, 0), ch, font=font)
        x += (box[2] - box[0]) + gap
    return x


def build_fire():
    """The standard fire warning sign, as it hangs over fire doors and
    extinguisher stations, on the red alarm field."""
    img = Image.new("RGB", (W, H), RED_FIELD)
    draw = ImageDraw.Draw(img)
    hazard_rail(draw, 0, 24)
    hazard_rail(draw, H - 24, H)

    sign = load_art(FIRE_SIGN)
    width = 302
    sign = sign.resize((width, int(round(sign.height * width / sign.width))),
                       Image.LANCZOS)
    paste_art(img, sign, (W - sign.width) // 2, 26)

    f_title = ImageFont.truetype(FONT_BOLD, 50)
    f_sub = ImageFont.truetype(FONT_BOLD, 28)
    f_small = ImageFont.truetype(FONT_REG, 22)
    centred_text(draw, "火灾警报", f_title, 294)
    centred_text(draw, "FIRE ALARM", f_sub, 366, fill=(255, 220, 90))
    centred_text(draw, "请立即沿安全出口疏散", f_small, 410)
    return img


def build_exit():
    """The standard green evacuation sign: text block on the left, the door
    with the running man in the middle, arrow pointing out on the right."""
    img = Image.new("RGB", (W, H), GREEN_FIELD)
    draw = ImageDraw.Draw(img)

    man = load_art(EXIT_MAN)
    man_h = 240
    man = man.resize((int(round(man.width * man_h / man.height)), man_h),
                     Image.LANCZOS)
    arrow = load_art(EXIT_ARROW)
    arrow = arrow.resize((140, int(round(arrow.height * 140 / arrow.width))),
                         Image.LANCZOS)

    x_arrow = W - 12 - arrow.width
    x_man = x_arrow - 16 - man.width
    paste_art(img, man, x_man, (H - man.height) // 2, (255, 255, 255))
    paste_art(img, arrow, x_arrow, (H - arrow.height) // 2, (255, 255, 255))

    f_title = fit_font(FONT_BOLD, "安全出口", x_man - 38, hi=54)
    f_sub = ImageFont.truetype(FONT_BOLD, 34)
    _, title_h = text_size(draw, "安全出口", f_title)
    _, sub_h = text_size(draw, "EXIT", f_sub)
    gap = 16
    y_title = (H - (title_h + gap + sub_h)) // 2
    draw.text((14, y_title), "安全出口", font=f_title, fill=(255, 255, 255))
    spaced_text(draw, "EXIT", f_sub, 18, y_title + title_h + gap, 10)
    return img


def bmp_info(path):
    with open(path, "rb") as fh:
        head = fh.read(54)
    return {
        "size": os.path.getsize(path),
        "bpp": int.from_bytes(head[28:30], "little"),
        "width": int.from_bytes(head[18:22], "little", signed=True),
        "height": int.from_bytes(head[22:26], "little", signed=True),
        "compression": int.from_bytes(head[30:34], "little"),
        "offset": int.from_bytes(head[10:14], "little"),
    }


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(PREVIEW_DIR, exist_ok=True)
    fire, exit_s = build_fire(), build_exit()
    for name, im in (("EMGFIRE.BMP", fire), ("EMGEXIT.BMP", exit_s)):
        path = os.path.join(OUT_DIR, name)
        im.save(path, "BMP")
        print(name, bmp_info(path))
    preview = Image.new("RGB", (W, H * 2 + 8), (0, 0, 0))
    preview.paste(fire, (0, 0))
    preview.paste(exit_s, (0, H + 8))
    preview.save(os.path.join(PREVIEW_DIR, "adv22_placards.png"))
    print("wrote build/adv22_placards.png")


if __name__ == "__main__":
    main()
