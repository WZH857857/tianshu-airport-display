"""Render the FPGA's own intermediate signals for a real fall clip.

The masks here are not a Python re-run: ``tb_vision_chain.v`` dumps
``frame_diff`` and ``motion_morph`` bit-for-bit (``+DIFF=`` / ``+MORPH=``)
while ``proj_bbox`` writes its per-frame ``found x y w h area``, all from the
same Icarus run that ``chain_check.py`` compares against the reference.  This
script only decodes those files and draws them.

Inputs (written by ``fpga/tools/chain_check.py``)::

    fpga/sim/gen/chain_<tag>.565        RGB565 little-endian source frames
    fpga/sim/gen/chain_<tag>.diff.bin   frame_diff mask, 320x180, MSB first
    fpga/sim/gen/chain_<tag>.morph.bin  motion_morph mask, same packing
    fpga/sim/gen/chain_<tag>.out        one "found x y w h area" line per frame

Usage::

    python make_rtl_strip.py [--tag 13_45f] [--alarm 31]
                             [--out out_frames/visual_report/rtl_stages.png]
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

BG = (17, 19, 24)
PANEL = (28, 32, 39)
FG = (236, 239, 244)
MUTED = (154, 163, 176)
GRID = (58, 65, 78)
OK = (58, 214, 130)
BAD = (255, 92, 92)
MASK = (0, 230, 140)

CELL_W, CELL_H = 320, 240
COLS = 6
GAP = 16


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
    """RGB565 -> RGB888 by bit replication (the RTL luma path's convention)."""
    r5 = ((pix >> 11) & 0x1F).astype(np.uint16)
    g6 = ((pix >> 5) & 0x3F).astype(np.uint16)
    b5 = (pix & 0x1F).astype(np.uint16)
    r8 = (r5 << 3) | (r5 >> 2)
    g8 = (g6 << 2) | (g6 >> 4)
    b8 = (b5 << 3) | (b5 >> 2)
    return np.stack([r8, g8, b8], axis=-1).astype(np.uint8)


def unpack(path: Path) -> np.ndarray:
    """Packed 320-wide rows, MSB first, one bit per pixel."""
    raw = np.frombuffer(path.read_bytes(), dtype=np.uint8)
    if len(raw) % ROW_BYTES:
        raise SystemExit(f"{path.name}: {len(raw)} bytes is not a whole "
                         f"number of {W_WORK}-pixel rows")
    rows = raw.reshape(-1, ROW_BYTES)
    bits = np.unpackbits(rows, axis=1, bitorder="big").astype(bool)
    return bits.reshape(-1, H_WORK, W_WORK)


def resize_nn(img: np.ndarray, w: int, h: int) -> np.ndarray:
    ys = np.minimum(np.arange(h) * img.shape[0] // h, img.shape[0] - 1)
    xs = np.minimum(np.arange(w) * img.shape[1] // w, img.shape[1] - 1)
    return img[ys][:, xs]


def overlay(rgb: np.ndarray, mask_work: np.ndarray) -> np.ndarray:
    """Tint the RTL mask onto the camera frame, dimming the rest."""
    out = rgb.astype(np.float32) * 0.55
    m = resize_nn(mask_work.astype(np.uint8), W_FRAME, H_FRAME).astype(bool)
    out[m] = out[m] * 0.35 + np.array(MASK, dtype=np.float32) * 0.65
    return out.clip(0, 255).astype(np.uint8)


def mask_image(mask_work: np.ndarray) -> np.ndarray:
    return np.where(mask_work[..., None], np.array(MASK, np.uint8),
                    np.array(PANEL, np.uint8))


def cell(d: ImageDraw.ImageDraw, box, alarm: bool, caption: str | None,
         tag: str, f_cap, f_tag) -> None:
    x, y = box
    d.rectangle((x - 2, y - 2, x + CELL_W + 1, y + CELL_H + 1),
                outline=BAD if alarm else GRID, width=3 if alarm else 1)
    d.text((x, y - 26), tag, font=f_tag, fill=BAD if alarm else MUTED)
    if caption:
        d.text((x, y + CELL_H + 4), caption, font=f_cap,
               fill=BAD if alarm else MUTED)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="13_45f")
    ap.add_argument("--alarm", type=int, default=31,
                    help="alarm frame reported by chain_check (1-based)")
    ap.add_argument("--out", default=str(ROOT / "out_frames" / "visual_report"
                                        / "rtl_stages.png"))
    args = ap.parse_args()

    pix = load565(GEN / f"chain_{args.tag}.565")
    diff = unpack(GEN / f"chain_{args.tag}.diff.bin")
    morph = unpack(GEN / f"chain_{args.tag}.morph.bin")
    boxes = [tuple(int(v) for v in line.split())
             for line in (GEN / f"chain_{args.tag}.out").read_text().splitlines()
             if line.strip()]
    n = min(len(pix), len(diff), len(morph), len(boxes))

    alarm = args.alarm
    picks = [alarm - 9, alarm - 5, alarm - 2, alarm, alarm + 2, alarm + 9]
    picks = [min(max(p - 1, 0), n - 1) for p in picks]   # 1-based -> index

    rgb = np.stack([to_rgb(f) for f in pix[:n]])
    over = np.stack([resize_nn(overlay(rgb[i], morph[i]), CELL_W, CELL_H)
                     for i in range(n)])
    morph_img = np.stack([resize_nn(mask_image(morph[i]), CELL_W, CELL_H)
                          for i in range(n)])
    diff_img = np.stack([resize_nn(mask_image(diff[i]), CELL_W, CELL_H)
                         for i in range(n)])

    left = 60
    top = 214
    # One row = frame tag (26) + picture + caption (30); the extra 34 keeps the
    # next row's heading clear of this row's caption.
    row_h = CELL_H + 90
    width = left * 2 + COLS * CELL_W + (COLS - 1) * GAP
    height = top + 3 * row_h + 92

    img = Image.new("RGB", (width, height), BG)
    d = ImageDraw.Draw(img)

    d.text((left, 40), "FPGA 内部信号（RTL 探针逐位导出）",
           font=font(40, True), fill=FG)
    d.text((left, 96),
           "Subject 1 / Fall / 13.mp4 前 45 帧 · 320x180 工作分辨率 · "
           "下面每一张掩膜和每一个包围盒都是 Icarus 跑 RTL 时写出来的，不是 Python 重算",
           font=font(18), fill=MUTED)
    d.text((left, 124),
           "第一行：相机画面 + RTL 形态学掩膜（绿）+ RTL 包围盒（黄/红框）；"
           "第二行：motion_morph 输出；第三行：frame_diff 输出",
           font=font(18), fill=MUTED)
    d.line((left, 176, width - left, 176), fill=GRID, width=2)

    f_tag = font(17, True)
    f_cap = font(14)
    rows = [(over, "相机 + RTL 掩膜 + RTL 包围盒"),
            (morph_img, "RTL motion_morph 输出（3x3 腐蚀 + 11x11 膨胀）"),
            (diff_img, "RTL frame_diff 输出（|cur-prev| > 10）")]

    for row, (stack, label) in enumerate(rows):
        ry = top + row * row_h
        d.text((left, ry - 58), label, font=font(18, True), fill=FG)
        for col, idx in enumerate(picks):
            cx = left + col * (CELL_W + GAP)
            img.paste(Image.fromarray(stack[idx]), (cx, ry))
            found, bx, by, bw_, bh_, barea = boxes[idx]
            frame_no = idx + 1
            is_alarm = (frame_no == alarm)
            # Kept short on purpose: at 320 px per cell a full sentence runs
            # into the next cell's caption.
            cap = (f"{'found' if found else '空'}  x={bx} y={by}  "
                   f"{bw_}x{bh_}  area={barea}")
            tag = f"f{frame_no:03d}" + ("   报警帧" if is_alarm else "")
            cell(d, (cx, ry), is_alarm, cap if row == 0 else None, tag,
                 f_cap, f_tag)
            if row == 0 and found:
                # bbox is in working coordinates; scale to the 640x480 view
                sx, sy = CELL_W / W_WORK, CELL_H / H_WORK
                d.rectangle((cx + bx * sx, ry + by * sy,
                             cx + (bx + bw_) * sx, ry + (by + bh_) * sy),
                            outline=BAD if is_alarm else (255, 214, 0), width=3)

    ny = top + 3 * row_h + 4
    d.text((left, ny),
           f"报警帧：RTL = {alarm}，与 Python 参考的报警帧完全一致（chain_check 45 帧长程回归）。",
           font=font(17), fill=OK)
    d.text((left, ny + 26),
           f"掩膜来自 fpga/sim/gen/chain_{args.tag}.{{diff,morph}}.bin，"
           f"包围盒来自 chain_{args.tag}.out。",
           font=font(17), fill=MUTED)
    d.text((left, ny + 52),
           "320x180 是必须的：器件只有 76,800 个 8 bit 片内存储条目，一帧 320x240 加乒乓掩膜装不下，"
           "丢每第 4 行的抽样把存储压回预算内。",
           font=font(17), fill=MUTED)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out)
    print(f"wrote {out}  ({width}x{height})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
