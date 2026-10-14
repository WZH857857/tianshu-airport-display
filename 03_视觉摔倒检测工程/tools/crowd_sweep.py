"""Crowd mask sweep -- find a threshold/dilation pair that does not glue a
crowd into one screen-sized blob.

The FPGA front end is  frame_diff(threshold) -> erode -> dilate  and the two
knobs that decide whether a dense scene stays separable are
``motion_threshold`` and ``dilate_radius``.  This script runs the same mask
pipeline as tools/fall_proto.py over the first ``--frames`` frames of a clip
and reports, per parameter pair,

  comps>=200   mean number of connected components that clear min_area
  span%        frames where the biggest component touches both raster edges
  big%         frames where the biggest component covers >20% of the raster
  none%        frames with no component at all

So a good pair keeps ``comps`` up, ``span``/``big`` down and ``none`` low.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import imageio.v3 as iio
import numpy as np
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fall_proto import FallDetector, Params  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("clip", type=Path)
    ap.add_argument("--frames", type=int, default=600)
    ap.add_argument("--thresholds", type=int, nargs="*",
                    default=[10, 16, 22, 28])
    ap.add_argument("--dilates", type=int, nargs="*", default=[5, 3, 2])
    args = ap.parse_args()

    frames = []
    for i, frame in enumerate(iio.imiter(args.clip, plugin="FFMPEG")):
        if i >= args.frames:
            break
        frames.append(frame)

    print(f"{args.clip.name}: {len(frames)} frames, "
          f"{frames[0].shape[1]}x{frames[0].shape[0]}")
    print(f"{'thr':>4} {'dil':>4} {'comps>=200':>10} {'span%':>7} "
          f"{'big%':>6} {'none%':>6}")

    for thr in args.thresholds:
        for dil in args.dilates:
            p = Params(motion_threshold=thr, dilate_radius=dil)
            det = FallDetector(p)
            counts = []
            span = big = none = 0
            for frame in frames:
                gray = det.prepare(frame)
                mask = det.motion_mask(gray)
                labels, n = ndimage.label(mask, structure=np.ones((3, 3), bool))
                comps = 0
                biggest = 0
                span_hit = False
                for index, slc in enumerate(ndimage.find_objects(labels),
                                            start=1):
                    if slc is None:
                        continue
                    local = mask[slc] & (labels[slc] == index)
                    area = int(local.sum())
                    if area < p.min_area:
                        continue
                    comps += 1
                    ys, xs = np.nonzero(local)
                    h = int(ys.max() - ys.min()) + 1
                    w = int(xs.max() - xs.min()) + 1
                    if area > biggest:
                        biggest = area
                    if w >= p.target_w - 8 or h >= p.target_h - 8:
                        span_hit = True
                counts.append(comps)
                if comps == 0:
                    none += 1
                if span_hit:
                    span += 1
                if biggest > 0.20 * p.target_w * p.target_h:
                    big += 1
            total = max(1, len(counts))
            print(f"{thr:>4} {dil:>4} {np.mean(counts):>10.2f} "
                  f"{100.0 * span / total:>6.1f}% {100.0 * big / total:>5.1f}% "
                  f"{100.0 * none / total:>5.1f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
