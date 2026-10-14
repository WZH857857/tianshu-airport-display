"""How big are the boxes multi_bbox actually publishes?

Replays the RTL front end (gray_front -> frame_diff -> motion_morph) over a
clip and runs the ``multi_model`` box finder at the same 320x180 raster the
FPGA uses, so a threshold change can be judged on box tightness before it is
written into the RTL and sent through a 4-minute PNR.

``--tighten 0`` reproduces the old band-height box (x, band_start, w,
band_len); ``--tighten 1`` is what the RTL publishes now, with the vertical
trim from ST_VSPAN (longest row run inside the chosen column run).  Running
both side by side is the before/after evidence for the "box too big" fix.

Usage::

    python box_size_stats.py CLIP.mp4 [--frames 240] [--tighten 0,1]
"""

from __future__ import annotations

import argparse
import statistics as stats
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(ROOT / "tools"))

import multi_model  # noqa: E402
from morph_check import DILATE_R, ERODE_R, KEEP_ROWS, morph_ref  # noqa: E402
from pixel_check import down2_box, luma_from_565, quantise565  # noqa: E402

DIFF_THRESH = 10


def masks_of(clip: Path, nframes: int, dilate: int = 2,
             erode: int = 2) -> np.ndarray:
    import imageio.v3 as iio

    frames = []
    for i, frame in enumerate(iio.imiter(clip, plugin="FFMPEG")):
        if i >= nframes:
            break
        frame = np.asarray(frame)
        # Same normalisation as morph_check.masks_from_clip: the camera path
        # is 640x480 -> 2x2 box -> 320x180.  Skipping it silently changes the
        # raster for any other source size, and then the median box sizes no
        # longer describe the FPGA.
        h, w = frame.shape[:2]
        if (w, h) != (640, 480):
            ys = np.minimum(np.arange(480) * h // 480, h - 1)
            xs = np.minimum(np.arange(640) * w // 640, w - 1)
            frame = frame[ys][:, xs]
        frames.append(frame)
    frames = np.stack(frames)
    luma = np.stack([down2_box(luma_from_565(quantise565(f)))[KEEP_ROWS]
                     for f in frames])
    masks = np.zeros_like(luma, dtype=bool)
    for i in range(1, len(luma)):
        diff = np.abs(luma[i].astype(np.int16) - luma[i - 1].astype(np.int16))
        masks[i] = diff > DIFF_THRESH
    if dilate == DILATE_R and erode == ERODE_R:
        return np.stack([morph_ref(m) for m in masks])
    from scipy import ndimage
    from fall_proto import disk

    out = []
    for m in masks:
        if not m.any():
            out.append(np.zeros_like(m))
            continue
        e = ndimage.binary_erosion(m, structure=disk(erode), border_value=0)
        out.append(ndimage.binary_dilation(e, structure=disk(dilate),
                                           border_value=0))
    return np.stack(out)


def summarise(name: str, rows: list[tuple[int, int, int, int, int]]) -> None:
    if not rows:
        print(f"  {name:<28} no boxes")
        return
    w = [r[2] for r in rows]
    h = [r[3] for r in rows]
    fill = [r[4] / (r[2] * r[3]) for r in rows if r[2] * r[3]]
    print(f"  {name:<28} boxes={len(rows):<5} "
          f"w med={int(stats.median(w)):<4} max={max(w):<4} "
          f"h med={int(stats.median(h)):<4} max={max(h):<4} "
          f"fill med={stats.median(fill):.2f}")


def boxes_of(mask: np.ndarray, tight: int) -> list[tuple[int, ...]]:
    """One frame, four exclusion passes.

    tight=0 is the pre-ST_VSPAN RTL: the box keeps the whole vertical band.
    tight=1 is the current RTL: ``multi_model.project_band`` trims it.
    """
    m = multi_model
    band = m.band_of(mask)
    if band is None:
        return []
    y0, y1 = band
    boxes: list[tuple[int, int, int, int]] = []
    out: list[tuple[int, ...]] = []
    for _ in range(m.NUM_BLOBS):
        if tight:
            res = m.project_band(mask, y0, y1, boxes)
        else:
            work = mask[y0:y1].copy()
            for bx0, by0, bx1, by1 in boxes:
                lo = max(by0, y0) - y0
                hi = min(by1, y1) - y0
                if hi > lo:
                    work[lo:hi, bx0:bx1] = False
            bh = y1 - y0
            if not work.any():
                res = (0, 0, y0, 0, bh, 0)
            else:
                x, w, area = m.scan(work.sum(axis=0))
                frame_h, frame_w = mask.shape
                spans = ((w >= frame_w - m.SPAN_MARGIN) or
                         (bh >= frame_h - m.SPAN_MARGIN))
                found = 1 if (w != 0 and area >= m.MIN_AREA
                              and not spans) else 0
                res = (found, x, y0, w, bh, area)
        out.append(res)
        if res[0]:
            boxes.append((res[1], res[2],
                          min(mask.shape[1], res[1] + res[3] + m.MARGIN),
                          min(mask.shape[0], res[2] + res[4] + m.MARGIN)))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("clip", type=Path)
    ap.add_argument("--frames", type=int, default=240)
    ap.add_argument("--col-pix", default="3,4", help="MIN_PIX values to sweep")
    ap.add_argument("--min-area", default="160,200")
    ap.add_argument("--tighten", default="0,1")
    ap.add_argument("--dilate", type=int, default=2)
    ap.add_argument("--erode", type=int, default=2)
    args = ap.parse_args()

    masks = masks_of(args.clip, args.frames, args.dilate, args.erode)
    h, w = masks.shape[1:]
    print(f"{args.clip.name}: {len(masks)} frames, raster {w}x{h}, "
          f"erode={args.erode} dilate={args.dilate}")
    print(f"  motion pixels/frame: median {int(np.median(masks.sum(axis=(1, 2))))}")

    for tight in (int(v) for v in args.tighten.split(",")):
        for col in (int(v) for v in args.col_pix.split(",")):
            multi_model.MIN_PIX = col
            multi_model.MIN_ROW_PIX = col
            for area in (int(v) for v in args.min_area.split(",")):
                multi_model.MIN_AREA = area
                boxes = [boxes_of(m, tight) for m in masks]
                flat = [(b[1], b[2], b[3], b[4], b[5])
                        for frame in boxes for b in frame if b[0]]
                summarise(f"col>={col} area>={area} tight={tight}", flat)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
