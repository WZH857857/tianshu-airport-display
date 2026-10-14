# -*- coding: utf-8 -*-
"""Pull the "door + running man" and the arrow out of the standard 安全出口
sign that the emergency placard is modelled on, and store them next to the
other assets as alpha masks.

The picture we cut from has the artwork as white shapes on the flat sign green,
so the alpha is just the distance of the darkest channel from the green level:
that keeps the antialiased edges instead of posterising them.

Usage:  python scripts/extract_exit_sign_art.py [reference.png]
"""

import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ASSET_DIR = os.path.join(HERE, "..", "assets")

DEFAULT_REF = (r"C:\Users\ASUS\AppData\Local\Temp"
               r"\codex-clipboard-e5848dab-3128-4d89-aed4-e6401d886afe.png")

# generously padded boxes around the two shapes on the 1080x517 sign
REGIONS = {
    "exit_man": (406, 80, 800, 480),
    "exit_arrow": (795, 180, 1080, 400),
}


def alpha_mask(image, box):
    crop = np.asarray(image.convert("RGB").crop(box)).astype(np.float32)
    dark = crop.min(axis=2)
    # sign green sits near 30, the artwork is 255; 60..230 keeps soft edges
    alpha = np.clip((dark - 60.0) / 170.0, 0.0, 1.0)
    return Image.fromarray((alpha * 255.0).astype(np.uint8), "L")


def tight_crop(mask, threshold=12):
    box = mask.point(lambda v: 255 if v > threshold else 0).getbbox()
    return mask.crop(box) if box else mask


def main():
    ref = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_REF
    image = Image.open(ref)
    os.makedirs(ASSET_DIR, exist_ok=True)
    for name, box in REGIONS.items():
        mask = tight_crop(alpha_mask(image, box))
        path = os.path.join(ASSET_DIR, name + ".png")
        mask.save(path)
        print(name, mask.size, "->", os.path.relpath(path, HERE))


if __name__ == "__main__":
    main()
