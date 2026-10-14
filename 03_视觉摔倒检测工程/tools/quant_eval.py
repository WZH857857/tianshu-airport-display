"""Measure how much luma quantisation the frame-difference front end can take.

The on-chip ERAM budget is the binding constraint on this device: TD spends
about 1.56 bits of ERAM per 8-bit memory entry, so a 320x240 byte frame plus
the packed motion mask does not fit.  Storing the previous frame at fewer
bits per pixel is the cheapest way back under the limit.

This script replays the same integer pipeline the RTL implements
(quantise565 -> 77/150/29 luma -> 2x2 box -> |diff| > thresh
 -> erode -> dilate -> projection bbox -> PostureMachine) but rounds the
 luma down to N bits before the frame store, which is exactly where the RTL
 would be lossy.  It prints, per clip and bit width, how many frames produce
 a different bounding box and how many alarms survive.

Usage::

    python tools/quant_eval.py --bits 8 6 5 4 --frames 60
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


def quantise(gray: np.ndarray, bits: int) -> np.ndarray:
    """Round-to-nearest quantisation to ``bits`` levels per pixel."""
    if bits >= 8:
        return gray.copy()
    step = 1 << (8 - bits)
    half = step >> 1
    return np.minimum((gray.astype(np.uint16) + half) >> (8 - bits),
                      (1 << bits) - 1).astype(np.uint8)


def down2_box(gray: np.ndarray) -> np.ndarray:
    h, w = gray.shape
    h2, w2 = h - h % 2, w - w % 2
    g = gray[:h2, :w2].astype(np.uint16)
    return (g[0::2, 0::2] + g[0::2, 1::2] + g[1::2, 0::2] + g[1::2, 1::2]) >> 2


def load_clip(path: Path, target_w: int, target_h: int) -> np.ndarray:
    """Return the clip as a 320x240 uint8 luma stack."""
    frames = list(iio.imiter(path))
    out = []
    for frame in frames:
        gray = to_gray(frame)
        h, w = gray.shape
        # Same box-average the RTL does for the 2:1 camera path.  Wider
        # sources get an extra integer decimation first so that every clip
        # lands on one comparable 320x240 grid.
        fy = max(1, int(round(h / float(target_h))))
        fx = max(1, int(round(w / float(target_w))))
        if fx > 2 or fy > 2:
            h2, w2 = h - h % fy, w - w % fx
            gray = (gray[:h2, :w2].astype(np.uint16)
                    .reshape(h2 // fy, fy, w2 // fx, fx).mean(axis=(1, 3))
                    .astype(np.uint8))
        out.append(down2_box(gray))
    return np.stack(out)


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


def replay(stack: np.ndarray, bits: int, p: Params, variant: str = "scale"):
    """One pass of the single-target chain.  Returns per-frame records.

    ``variant`` selects how the previous-frame store is compressed:

    ``scale``  the RTL's natural shape.  Only the previous frame is stored
               quantised; the current pixel keeps its 8-bit luma.  Quantising
               to N bits multiplies the difference by about 1 (with rounding),
               so the motion threshold is rescaled to keep the same physical
               contrast.
    ``both``   both frames are quantised before the subtraction, which is
               what the first version of this script did and what a naive
               "store 4-bit pixels" design would do.
    ``raw``    ``both`` but the 8-bit threshold is left alone, i.e. the
               mistake the first run exposed.
    """
    prev = None
    machine = PostureMachine(p)
    records = []
    for idx, gray in enumerate(stack):
        q = quantise(gray, bits)
        if prev is None:
            mask = np.zeros_like(q, dtype=bool)
        else:
            if variant == "both":
                cur = q.astype(np.int16)
                thr = p.motion_threshold if bits >= 8 else max(
                    1, int(round(p.motion_threshold / float(1 << (8 - bits)))))
            elif variant == "raw":
                cur = q.astype(np.int16)
                thr = p.motion_threshold
            else:  # scale
                cur = gray.astype(np.int16)
                thr = p.motion_threshold if bits >= 8 else max(
                    1, int(round(p.motion_threshold * (1 << (8 - bits)) / 256.0)))
            diff = np.abs(cur - prev.astype(np.int16))
            mask = diff > thr
            if mask.any():
                mask = ndimage.binary_erosion(mask, structure=disk(p.erode_radius))
                mask = ndimage.binary_dilation(mask, structure=disk(p.dilate_radius))
        prev = q
        bbox, area = bbox_of(mask, p)
        obs = None
        if bbox is not None:
            x, y, w, h = bbox
            obs = Obs(idx, bbox, area,
                      h / float(w) if w else 0.0,
                      x + (w - 1) / 2.0,
                      y + (h - 1) / 2.0)
        alarm, state, _ = machine.feed(obs, idx)
        records.append((bbox, area, alarm, state))
    return records


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bits", type=int, nargs="+", default=[8, 6, 5, 4])
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--clips", nargs="*", default=None)
    ap.add_argument("--variant", choices=["scale", "both", "raw"], default="scale")
    args = ap.parse_args()

    if args.clips:
        clips = [Path(c) for c in args.clips]
    else:
        clips = sorted((ROOT / "datasets" / "gmdcsa24").glob("Subject */*/*.mp4"))[:8]
        clips += sorted((ROOT / "datasets" / "crowd").glob("crowd0[1-4].mp4"))

    base_bits = args.bits[0]
    print(f"{'clip':38s} {'bits':>4s} {'frames':>6s} {'bboxdiff':>8s} "
          f"{'alarms':>6s} {'base_alarms':>11s}")
    for clip in clips:
        if not clip.exists():
            print(f"{clip.name:38s}  MISSING")
            continue
        stack = load_clip(clip, 320, 240)[: args.frames]
        p = Params()
        base = replay(stack, base_bits, p, args.variant)
        base_bbox = [r[0] for r in base]
        base_alarms = sum(1 for r in base if r[2])
        for bits in args.bits:
            rec = replay(stack, bits, p, args.variant)
            diff = sum(1 for a, b in zip(base_bbox, [r[0] for r in rec]) if a != b)
            alarms = sum(1 for r in rec if r[2])
            print(f"{clip.name:38s} {bits:4d} {len(rec):6d} {diff:8d} "
                  f"{alarms:6d} {base_alarms:11d}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
