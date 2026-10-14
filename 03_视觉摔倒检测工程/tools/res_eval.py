"""Compare the 320x240 working resolution with 320x180.

The device will not hold a 320x240 byte frame plus the packed motion mask in
on-chip ERAM (76,800 + 9,600 entries against a 76,800-entry budget), so the
candidate fix is one 3:2 vertical decimation instead of the current 2:1:
keep 2 of every 3 rows, i.e. 180 rows of the 240-row downsampled frame.

This script runs the same integer pipeline as tools/fall_proto.py on both
grids and reports the per-frame bounding-box disagreement and the alarms
raised by each.  A drop from 240 to 180 rows is only acceptable if the
alarms and the box geometry stay put.

Usage::

    python tools/res_eval.py --frames 80
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import imageio.v3 as iio
import numpy as np
from scipy import ndimage

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))

from fall_proto import (  # noqa: E402
    Obs, Params, PostureMachine, disk, largest_span, to_gray,
)


def down2(gray: np.ndarray) -> np.ndarray:
    h, w = gray.shape
    h2, w2 = h - h % 2, w - w % 2
    g = gray[:h2, :w2].astype(np.uint16)
    return (g[0::2, 0::2] + g[0::2, 1::2] + g[1::2, 0::2] + g[1::2, 1::2]) >> 2


def to_240(gray: np.ndarray) -> np.ndarray:
    """640x480 -> 320x240, the current RTL path."""
    return down2(gray)


def to_180(gray: np.ndarray) -> np.ndarray:
    """640x480 -> 320x240 -> keep rows 0,2 of every 3 -> 320x160.

    Name kept for the command line; the pattern is the hardware one, a mod-3
    counter with a keep when (y % 3) != 2 on top of the existing mod-2 pair
    logic.  It yields 160 rows, not 180: 240 * 2/3 = 160.
    """
    full = down2(gray)
    idx = np.array([y for y in range(full.shape[0]) if y % 3 != 2])
    return full[idx]


def to_180_true(gray: np.ndarray) -> np.ndarray:
    """640x480 -> 320x240 -> drop every 4th boxed row -> 320x180.

    The hardware gate is a mod-4 phase counter that skips the emit when
    row_phase == 3, so rows 3,7,11,... are dropped: 180 of 240, one line
    every 4, which keeps the vertical geometry far closer to 240 than the
    mod-3 gate's 320x160 does.
    """
    full = down2(gray)
    idx = np.array([y for y in range(full.shape[0]) if y % 4 != 3])
    return full[idx]


def bbox_of(mask: np.ndarray, p: Params):
    if not mask.any():
        return None, 0
    span_y = largest_span(mask.sum(axis=1), p.min_pixels)
    if span_y is None:
        return None, 0
    y0, y1 = span_y
    band = mask[y0:y1]
    span_x = largest_span(band.sum(axis=0), p.min_pixels)
    if span_x is None:
        return None, 0
    x0, x1 = span_x
    area = int(band[:, x0:x1].sum())
    if area < p.min_area:
        return None, area
    return (x0, y0, x1 - x0, y1 - y0), area


def replay(stack: np.ndarray, p: Params):
    prev = None
    machine = PostureMachine(p)
    machine.frame_w, machine.frame_h = 320, stack.shape[1]
    recs = []
    for idx, gray in enumerate(stack):
        if prev is None:
            mask = np.zeros_like(gray, dtype=bool)
        else:
            diff = np.abs(gray.astype(np.int16) - prev.astype(np.int16))
            mask = diff > p.motion_threshold
            if mask.any():
                mask = ndimage.binary_erosion(mask, structure=disk(p.erode_radius))
                mask = ndimage.binary_dilation(mask, structure=disk(p.dilate_radius))
        prev = gray
        bbox, area = bbox_of(mask, p)
        obs = None
        if bbox is not None:
            x, y, w, h = bbox
            obs = Obs(idx, bbox, area, h / float(w) if w else 0.0,
                      x + (w - 1) / 2.0, y + (h - 1) / 2.0)
        alarm, state, _ = machine.feed(obs, idx)
        recs.append((bbox, area, alarm, state))
    return recs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=80)
    ap.add_argument("--clips", nargs="*", default=None)
    args = ap.parse_args()

    if args.clips:
        clips = [Path(c) for c in args.clips]
    else:
        clips = sorted((ROOT / "datasets" / "gmdcsa24").glob("Subject */*/*.mp4"))
        clips += sorted((ROOT / "datasets" / "crowd").glob("crowd0[1-6].mp4"))
        clips += sorted((ROOT / "datasets" / "cctv_fall").glob("*.mp4"))

    print(f"{'clip':30s} {'N':>4s} {'box240':>7s} {'box160':>7s} {'box180':>7s} "
          f"{'al240':>5s} {'al160':>5s} {'al180':>5s}")
    totals = [0, 0, 0, 0, 0, 0]
    for clip in clips:
        if not clip.exists():
            continue
        try:
            frames = [f for f in iio.imiter(clip)][: args.frames]
        except Exception as exc:  # noqa: BLE001
            print(f"{clip.name:34s}  load failed: {exc}")
            continue
        p = Params()
        s240 = np.stack([to_240(to_gray(f)) for f in frames])
        s160 = np.stack([to_180(to_gray(f)) for f in frames])
        s180 = np.stack([to_180_true(to_gray(f)) for f in frames])
        r240 = replay(s240, p)
        r160 = replay(s160, p)
        r180 = replay(s180, p)
        al240 = sum(1 for r in r240 if r[2])
        al160 = sum(1 for r in r160 if r[2])
        al180 = sum(1 for r in r180 if r[2])
        print(f"{clip.name:30s} {len(frames):4d} {sum(1 for r in r240 if r[0]):7d} "
              f"{sum(1 for r in r160 if r[0]):7d} {sum(1 for r in r180 if r[0]):7d} "
              f"{al240:5d} {al160:5d} {al180:5d}")
        totals[0] += sum(1 for r in r240 if r[0])
        totals[1] += sum(1 for r in r160 if r[0])
        totals[2] += sum(1 for r in r180 if r[0])
        totals[3] += al240
        totals[4] += al160
        totals[5] += al180
    print(f"{'TOTAL':30s} {'':4s} {totals[0]:7d} {totals[1]:7d} {totals[2]:7d} "
          f"{totals[3]:5d} {totals[4]:5d} {totals[5]:5d}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
