"""Contact sheet of the generated SD card test pictures.

Used as evidence that the five BMP files are the expected 640 x 480 24 bit
pictures, and as a reference the board output can be compared against.
The sheet is written as PNG because it is meant for viewing, not for the card.
"""

import os

from PIL import Image, ImageDraw

WIDTH = 640
HEIGHT = 480
SCALE = 2  # 320 x 240 cells

HERE = os.path.dirname(os.path.abspath(__file__))
CARD_DIR = os.path.normpath(os.path.join(HERE, "..", "sdcard"))
OUT = os.path.normpath(os.path.join(HERE, "..", "doc", "sdcard_test_patterns.png"))

NAMES = ["IMG01.BMP", "IMG02.BMP", "IMG03.BMP", "IMG04.BMP", "IMG05.BMP"]
LABELS = [
    "IMG01 color bars",
    "IMG02 ramp",
    "IMG03 checker",
    "IMG04 grid",
    "IMG05 grey steps",
]


def main():
    cell_w = WIDTH // SCALE
    cell_h = HEIGHT // SCALE
    label_h = 26
    cols, rows = 2, 3
    sheet = Image.new("RGB", (cell_w * cols, (cell_h + label_h) * rows), (24, 24, 27))
    draw = ImageDraw.Draw(sheet)

    for i, name in enumerate(NAMES):
        src = Image.open(os.path.join(CARD_DIR, name)).convert("RGB")
        thumb = src.resize((cell_w, cell_h), Image.BOX)
        col = i % cols
        row = i // cols
        x = col * cell_w
        y = row * (cell_h + label_h)
        sheet.paste(thumb, (x, y))
        draw.rectangle([x, y, x + cell_w - 1, y + cell_h - 1], outline=(70, 70, 78))
        draw.text((x + 8, y + cell_h + 6), LABELS[i], fill=(225, 225, 230))

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    sheet.save(OUT, format="PNG", optimize=True)
    print("wrote", OUT, os.path.getsize(OUT), "bytes")


if __name__ == "__main__":
    main()
