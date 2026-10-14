"""Build the SD card test pictures this player accepts.

The picture path is strict on purpose: the parser only takes 640 x 480, 24 bit,
BI_RGB (uncompressed) BMP files with a positive height, i.e. the usual bottom up
row order.  Everything else is rejected with an error code, which on the board
shows up as LED3 staying on.  These files are generated rather than
downloaded so the card contents can be reproduced at any time.

Only the standard library plus Pillow is used, so the bundled python that
ships with the desktop app can run it directly.
"""

import os
import struct

from PIL import Image, ImageDraw

WIDTH = 640
HEIGHT = 480
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sdcard")


def index_marker(draw, index):
    """White squares in the top left corner: the picture number when counting."""
    for i in range(index):
        x = 8 + i * 26
        draw.rectangle([x, 8, x + 18, 26], fill=(255, 255, 255))
        draw.rectangle([x, 8, x + 18, 26], outline=(0, 0, 0))


def frame(draw):
    """Single pixel border plus diagonals: shows whether the whole area is up."""
    draw.rectangle([0, 0, WIDTH - 1, HEIGHT - 1], outline=(255, 255, 255))
    draw.line([(0, 0), (WIDTH - 1, HEIGHT - 1)], fill=(255, 255, 255))
    draw.line([(WIDTH - 1, 0), (0, HEIGHT - 1)], fill=(255, 255, 255))


def color_bars():
    img = Image.new("RGB", (WIDTH, HEIGHT), (0, 0, 0))
    draw = ImageDraw.Draw(img)
    bars = [
        (255, 255, 255),
        (255, 255, 0),
        (0, 255, 255),
        (0, 255, 0),
        (255, 0, 255),
        (255, 0, 0),
        (0, 0, 255),
        (0, 0, 0),
    ]
    bar_w = WIDTH // len(bars)
    for i, colour in enumerate(bars):
        draw.rectangle([i * bar_w, 0, (i + 1) * bar_w - 1, HEIGHT - 1], fill=colour)
    for i in range(8):
        value = i * 36
        draw.rectangle(
            [i * bar_w, HEIGHT // 2, (i + 1) * bar_w - 1, HEIGHT - 1],
            fill=(value, value, value),
        )
    return img


def rgb_ramp():
    img = Image.new("RGB", (WIDTH, HEIGHT))
    px = img.load()
    for y in range(HEIGHT):
        for x in range(WIDTH):
            if y < HEIGHT // 2:
                px[x, y] = (x * 255 // (WIDTH - 1),) * 3
            else:
                px[x, y] = (
                    x * 255 // (WIDTH - 1),
                    (y - HEIGHT // 2) * 255 // (HEIGHT // 2 - 1),
                    255 - x * 255 // (WIDTH - 1),
                )
    return img


def checkerboard():
    img = Image.new("RGB", (WIDTH, HEIGHT), (0, 0, 0))
    px = img.load()
    for y in range(HEIGHT):
        for x in range(WIDTH):
            if (x + y) % 2 == 0:
                px[x, y] = (255, 255, 255)
    return img


def grid():
    img = Image.new("RGB", (WIDTH, HEIGHT), (18, 18, 18))
    draw = ImageDraw.Draw(img)
    for x in range(0, WIDTH, 40):
        draw.line([(x, 0), (x, HEIGHT - 1)], fill=(0, 128, 255))
    for y in range(0, HEIGHT, 40):
        draw.line([(0, y), (WIDTH - 1, y)], fill=(0, 128, 255))
    draw.rectangle(
        [WIDTH // 2 - 20, HEIGHT // 2 - 20, WIDTH // 2 + 20, HEIGHT // 2 + 20],
        outline=(255, 64, 0),
        width=2,
    )
    return img


def grey_steps():
    img = Image.new("RGB", (WIDTH, HEIGHT), (0, 0, 0))
    draw = ImageDraw.Draw(img)
    steps = 16
    step_h = HEIGHT // steps
    for i in range(steps):
        value = i * 255 // (steps - 1)
        draw.rectangle(
            [0, i * step_h, WIDTH - 1, (i + 1) * step_h - 1], fill=(value,) * 3
        )
    for x in range(WIDTH):
        value = x * 255 // (WIDTH - 1)
        draw.line([(x, HEIGHT - 40), (x, HEIGHT - 21)], fill=(value,) * 3)
    return img


def check(path):
    """Fail loudly if Pillow ever writes something the parser would reject."""
    with open(path, "rb") as handle:
        header = handle.read(54)
    magic = header[0:2]
    data_offset = struct.unpack_from("<I", header, 10)[0]
    dib_size = struct.unpack_from("<I", header, 14)[0]
    width, height = struct.unpack_from("<ii", header, 18)
    planes, bpp = struct.unpack_from("<HH", header, 26)
    compression = struct.unpack_from("<I", header, 30)[0]
    assert magic == b"BM", (path, magic)
    assert dib_size >= 40, (path, dib_size)
    assert width == WIDTH and height == HEIGHT, (path, width, height)
    assert planes == 1 and bpp == 24, (path, planes, bpp)
    assert compression == 0, (path, compression)
    assert data_offset >= 54, (path, data_offset)


def build(pattern, index, name):
    img = pattern()
    draw = ImageDraw.Draw(img)
    frame(draw)
    index_marker(draw, index)
    path = os.path.join(OUT_DIR, name)
    img.save(path, format="BMP")
    check(path)
    return path


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for entry in os.listdir(OUT_DIR):
        full = os.path.join(OUT_DIR, entry)
        if os.path.isfile(full) and os.path.splitext(entry)[1].lower() == ".bmp":
            os.remove(full)

    build(color_bars, 1, "IMG01.BMP")
    build(rgb_ramp, 2, "IMG02.BMP")
    build(checkerboard, 3, "IMG03.BMP")
    build(grid, 4, "IMG04.BMP")
    build(grey_steps, 5, "IMG05.BMP")

    print("written to", os.path.abspath(OUT_DIR))
    for entry in sorted(os.listdir(OUT_DIR)):
        full = os.path.join(OUT_DIR, entry)
        print("  {:12s} {:8d} bytes".format(entry, os.path.getsize(full)))


if __name__ == "__main__":
    main()
