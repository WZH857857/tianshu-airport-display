"""Render the crowd path's four-slot boxes from the RTL run's own dumps.

Nothing here is a Python re-run of the algorithm.  ``multi_top_check.py``
streams the clip through ``fall_core_multi_top.v`` and leaves behind

    fpga/sim/gen/multitop_<tag>.565        RGB565 little-endian source frames
    fpga/sim/gen/multitop_<tag>.morph.bin  motion_morph mask, 320x180, MSB first
    fpga/sim/gen/multitop_<tag>.out        four "found x y w h area" lines/frame

This script only decodes those files and draws them: the camera frame with
the four boxes, the morph mask with the same boxes and the exclusion
rectangles each pass worked against, and a per-frame slot table.

Usage::

    python make_crowd_strip.py [--tag crowd01_4f] [--frames 3]
                               [--out out_frames/visual_report/crowd_multi.png]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
GEN = ROOT / "fpga" / "sim" / "gen"

W_FRAME, H_FRAME = 640, 480
W_WORK, H_WORK = 320, 180
ROW_BYTES = W_WORK // 8
NUM_BLOBS = 4
MARGIN = 8
SCALE = 2

BG = (17, 19, 24)
PANEL = (28, 32, 39)
FG = (236, 239, 244)
MUTED = (154, 163, 176)
GRID = (58, 65, 78)
ACCENT = (90, 170, 255)
SLOT_COLORS = [
    (255, 92, 92),     # slot 0
    (255, 170, 60),    # slot 1
    (58, 214, 130),    # slot 2
    (90, 170, 255),    # slot 3
]
MASK = (0, 230, 140)

CELL_W, CELL_H = W_WORK * SCALE, H_WORK * SCALE
GAP = 20
PAD = 40
TEXT_W = 330


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    names = ([r"C:\Windows\Fonts\msyhbd.ttc", r"C:\Windows\Fonts\msyh.ttc"]
             if bold else
             [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\msyhbd.ttc"])
    for name in names:
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def load565(path: Path) -> np.ndarray:
    raw = np.frombuffer(path.read_bytes(), dtype="<u2")
    return raw.reshape(-1, H_FRAME, W_FRAME)


def to_rgb(pix: np.ndarray) -> np.ndarray:
    r5 = ((pix >> 11) & 0x1F).astype(np.uint16)
    g6 = ((pix >> 5) & 0x3F).astype(np.uint16)
    b5 = (pix & 0x1F).astype(np.uint16)
    r8 = (r5 << 3) | (r5 >> 2)
    g8 = (g6 << 2) | (g6 >> 4)
    b8 = (b5 << 3) | (b5 >> 2)
    return np.stack([r8, g8, b8], axis=-1).astype(np.uint8)


def resize_nn(img: np.ndarray, w: int, h: int) -> np.ndarray:
    ys = np.minimum(np.arange(h) * img.shape[0] // h, img.shape[0] - 1)
    xs = np.minimum(np.arange(w) * img.shape[1] // w, img.shape[1] - 1)
    return img[ys][:, xs]


def unpack_masks(path: Path) -> np.ndarray:
    raw = np.frombuffer(path.read_bytes(), dtype=np.uint8)
    if len(raw) % ROW_BYTES:
        raise SystemExit(f"{path.name}: not a whole number of rows")
    bits = np.unpackbits(raw.reshape(-1, ROW_BYTES), axis=1, bitorder="big")
    return bits.astype(bool).reshape(-1, H_WORK, W_WORK)


def load_boxes(path: Path) -> list[list[tuple[int, ...]]]:
    rows = [tuple(int(v) for v in line.split())
            for line in path.read_text().splitlines() if line.strip()]
    if len(rows) % NUM_BLOBS:
        raise SystemExit(f"{path.name}: {len(rows)} lines is not a multiple of "
                         f"{NUM_BLOBS}")
    return [rows[i:i + NUM_BLOBS] for i in range(0, len(rows), NUM_BLOBS)]


def exclusion_state(boxes: list[list[tuple[int, ...]]]
                    ) -> list[list[tuple[int, int, int, int]]]:
    """Replay the module's exclusion table from the boxes it published.

    Left/top edge stays on the box, right/bottom edge is pushed out by
    MARGIN, and a slot that comes up empty keeps the previous frame's
    rectangle -- the same rules multi_bbox.v applies.
    """
    state: list[tuple[int, int, int, int] | None] = [None] * NUM_BLOBS
    per_frame = []
    for frame_boxes in boxes:
        for k, (found, x, y, w, h, _area) in enumerate(frame_boxes):
            if found:
                state[k] = (x, y, x + w + MARGIN, y + h + MARGIN)
            else:
                state[k] = None
        per_frame.append([b for b in state if b is not None])
    return per_frame


def mask_panel(mask: np.ndarray,
               ex: list[tuple[int, int, int, int]]) -> Image.Image:
    rgb = np.where(mask[..., None], np.array(MASK, np.uint8),
                   np.array(PANEL, np.uint8))
    img = Image.fromarray(rgb).resize((CELL_W, CELL_H), Image.NEAREST)
    tint = Image.new("RGBA", img.size, (0, 0, 0, 0))
    td = ImageDraw.Draw(tint)
    for x0, y0, x1, y1 in ex:
        td.rectangle((x0 * SCALE, y0 * SCALE, x1 * SCALE - 1, y1 * SCALE - 1),
                     fill=(255, 255, 255, 30))
    return Image.alpha_composite(img.convert("RGBA"), tint).convert("RGB")


def draw_boxes(img: Image.Image, frame_boxes, ex, show_ex: bool) -> None:
    d = ImageDraw.Draw(img)
    if show_ex:
        for x0, y0, x1, y1 in ex:
            d.rectangle((x0 * SCALE, y0 * SCALE, x1 * SCALE - 1,
                         y1 * SCALE - 1), outline=(120, 128, 142), width=2)
    for k, (found, x, y, w, h, _area) in enumerate(frame_boxes):
        if not found:
            continue
        col = SLOT_COLORS[k]
        d.rectangle((x * SCALE, y * SCALE, (x + w) * SCALE - 1,
                     (y + h) * SCALE - 1), outline=col, width=4)
        label = f"slot {k}"
        lx, ly = x * SCALE, max(0, y * SCALE - 26)
        d.rectangle((lx, ly, lx + 10 + 10 * len(label), ly + 24),
                    fill=(0, 0, 0))
        d.text((lx + 5, ly + 2), label, font=font(18, True), fill=col)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tag", default="crowd01_4f")
    ap.add_argument("--frames", type=int, default=3)
    ap.add_argument("--skip", type=int, default=1,
                    help="first frame index to draw (frame 0 has no motion)")
    ap.add_argument("--out", type=Path,
                    default=ROOT / "out_frames" / "visual_report"
                    / "crowd_multi.png")
    args = ap.parse_args()

    pix = load565(GEN / f"multitop_{args.tag}.565")
    masks = unpack_masks(GEN / f"multitop_{args.tag}.morph.bin")
    boxes = load_boxes(GEN / f"multitop_{args.tag}.out")
    ex_all = exclusion_state(boxes)

    n = min(args.frames, len(boxes) - args.skip)
    if n <= 0:
        raise SystemExit("no frames to draw")

    width = PAD * 2 + CELL_W * 2 + GAP + TEXT_W
    height = PAD + 96 + n * (CELL_H + 46) + PAD
    img = Image.new("RGB", (width, height), BG)
    d = ImageDraw.Draw(img)

    d.text((PAD, 26), "人群路径：单帧 4 槽排除式检测（RTL 导出）",
           font=font(34, True), fill=FG)
    d.text((PAD, 70),
           f"crowd01  ·  fall_core_multi_top.v  ·  320x180 工作分辨率  ·  "
           f"MARGIN={MARGIN}  ·  灰框 = 该 pass 的排除矩形",
           font=font(20), fill=MUTED)

    f_lab = font(19, True)
    f_body = font(18)
    f_small = font(16)

    for r in range(n):
        f_idx = args.skip + r
        y = PAD + 96 + r * (CELL_H + 46)
        d.text((PAD, y - 30), f"frame {f_idx}", font=f_lab, fill=ACCENT)

        cam = Image.fromarray(resize_nn(to_rgb(pix[f_idx]), W_WORK, H_WORK))
        cam = cam.resize((CELL_W, CELL_H), Image.NEAREST)
        draw_boxes(cam, boxes[f_idx], ex_all[f_idx], show_ex=False)
        img.paste(cam, (PAD, y))
        d.rectangle((PAD - 1, y - 1, PAD + CELL_W, y + CELL_H), outline=GRID)

        mx = PAD + CELL_W + GAP
        mp = mask_panel(masks[f_idx], ex_all[f_idx])
        draw_boxes(mp, boxes[f_idx], ex_all[f_idx], show_ex=True)
        img.paste(mp, (mx, y))
        d.rectangle((mx - 1, y - 1, mx + CELL_W, y + CELL_H), outline=GRID)

        tx = mx + CELL_W + GAP
        ry = y + 6
        for k, (found, bx, by, bw, bh, area) in enumerate(boxes[f_idx]):
            col = SLOT_COLORS[k] if found else MUTED
            d.text((tx, ry), f"slot {k}", font=f_lab, fill=col)
            if found:
                d.text((tx + 78, ry + 2),
                       f"({bx},{by}) {bw}x{bh}  面积 {area}",
                       font=f_body, fill=FG)
            else:
                d.text((tx + 78, ry + 2), "空（本帧无残留运动）",
                       font=f_body, fill=MUTED)
            ry += 34
        d.text((tx, ry + 6),
               f"排除矩形 {len(ex_all[f_idx])} 个（跨帧保留）",
               font=f_small, fill=MUTED)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    img.save(args.out)
    print(f"wrote {args.out} ({img.width}x{img.height})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
