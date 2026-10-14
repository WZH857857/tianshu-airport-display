"""Contact sheet for the per-person coloured boxes.

Runs the offline detector with the shipped parameters (frame-difference
threshold 10 and the radius-3 dilation the RTL now uses) over the two demo
clips and draws the frames that show the effect:

  * top row   -- crowded scene: several people each carry a different colour
  * bottom row-- fall clip: the standing person is green, the confirmed fall
                 is red

Usage::

    python make_colour_strip.py [--out OUT.png]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import imageio.v3 as iio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fall_proto import FallDetector, Params, annotate  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
CROWD = ROOT / "out" / "test_videos" / "crowd_test.mp4"
FALL = ROOT / "out" / "test_videos" / "fall_test.mp4"

BG = (17, 19, 24)
PANEL = (28, 32, 39)
FG = (236, 239, 244)
MUTED = (154, 163, 176)
RED = (255, 92, 92)
CELL_W = 380
COLS = 4


def load_font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    names = ([r"C:\Windows\Fonts\msyhbd.ttc", r"C:\Windows\Fonts\msyh.ttc"]
             if bold else
             [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf"])
    for name in names:
        if Path(name).exists():
            return ImageFont.truetype(name, size)
    return ImageFont.load_default()


def scan(clip: Path, params: Params) -> list[tuple[int, int, bool]]:
    """One pass: (frame, boxes drawn, any alarm) per frame."""
    det = FallDetector(params)
    rows = []
    for index, frame in enumerate(iio.imiter(clip, plugin="FFMPEG")):
        dets = det.update(frame)
        boxes = sum(1 for d in dets if d.bbox is not None)
        rows.append((index, boxes, any(d.flagged for d in dets)))
    return rows


def pick(rows: list[tuple[int, int, bool]], want: int, min_gap: int,
         alarm_only: bool = False) -> list[int]:
    if alarm_only:
        hits = [f for f, _, alarm in rows if alarm]
        if hits:
            return hits[:want]
    order = sorted(rows, key=lambda r: (-r[1], r[0]))
    out: list[int] = []
    for frame, boxes, _ in order:
        if boxes == 0:
            break
        if all(abs(frame - f) >= min_gap for f in out):
            out.append(frame)
        if len(out) == want:
            break
    return sorted(out)


def render(clip: Path, params: Params, wanted: list[int]) -> dict[int, np.ndarray]:
    det = FallDetector(params)
    shots: dict[int, np.ndarray] = {}
    for index, frame in enumerate(iio.imiter(clip, plugin="FFMPEG")):
        dets = det.update(frame)
        if index in wanted:
            shots[index] = annotate(frame, det.mask_cache, dets, det.scale)
    return shots


def cell(image: np.ndarray, title: str, note: str, alarm: bool) -> Image.Image:
    img = Image.fromarray(image)
    scale = CELL_W / img.width
    img = img.resize((CELL_W, int(round(img.height * scale))), Image.BILINEAR)
    bar = 46
    panel = Image.new("RGB", (CELL_W, img.height + bar), PANEL)
    panel.paste(img, (0, 0))
    draw = ImageDraw.Draw(panel)
    draw.text((10, img.height + 6), title, font=load_font(15, True), fill=FG)
    draw.text((10, img.height + 25), note, font=load_font(14),
              fill=RED if alarm else MUTED)
    return panel


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=ROOT / "out" / "colour_strip.png")
    args = ap.parse_args()

    params = Params()
    crowd_rows = scan(CROWD, params)
    fall_rows = scan(FALL, params)
    crowd_frames = pick(crowd_rows, COLS, min_gap=25)
    fall_frames = pick(fall_rows, COLS, min_gap=25, alarm_only=True)
    if len(fall_frames) < COLS:
        fall_frames = sorted(set(fall_frames)
                             | set(pick(fall_rows, COLS, min_gap=25)))
    crowd_shots = render(CROWD, params, set(crowd_frames))
    fall_shots = render(FALL, params, set(fall_frames))

    by_frame = {frame: boxes for frame, boxes, _ in crowd_rows}
    by_alarm = {f: alarm for f, _, alarm in fall_rows}
    cells = []
    for frame in crowd_frames:
        boxes = by_frame.get(frame, 0)
        cells.append(cell(crowd_shots[frame], f"人群 · 第 {frame} 帧",
                          f"{boxes} 个人 · 一人一色", False))
    for frame in fall_frames:
        alarm = by_alarm.get(frame, False)
        cells.append(cell(fall_shots[frame], f"摔倒素材 · 第 {frame} 帧",
                          "红色 = 摔倒确认" if alarm else "绿色 = 正常跟踪",
                          alarm))

    cell_h = max(c.height for c in cells)
    rows = 2
    pad = 12
    sheet = Image.new("RGB", (pad + COLS * (CELL_W + pad),
                              pad + rows * (cell_h + pad) + 46), BG)
    draw = ImageDraw.Draw(sheet)
    draw.text((pad, 12), "彩色人框效果（阈值 10 / 膨胀半径 3，与 RTL 同参数）",
              font=load_font(20, True), fill=FG)
    for i, c in enumerate(cells):
        col = i % COLS
        row = i // COLS
        x = pad + col * (CELL_W + pad)
        y = 46 + pad + row * (cell_h + pad)
        sheet.paste(c, (x, y))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(args.out)
    print(f"wrote {args.out} ({sheet.width}x{sheet.height})")
    print(f"crowd frames {crowd_frames}  fall frames {fall_frames}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
