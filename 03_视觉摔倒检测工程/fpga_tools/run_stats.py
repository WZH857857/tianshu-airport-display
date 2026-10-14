"""Measure row-run and connected-component density of real motion masks.

The multi-target front end has to hold every horizontal run of a 320x180 mask
on-chip, so this script answers two sizing questions before any RTL is built:

  * how many horizontal runs (>= MIN_PIX long) a real frame contains
  * how many connected components survive the area/height gates

Run it on the crowd footage to choose the run-array depth and to see how many
people a projection-based labeller can realistically separate.

Usage::

    python run_stats.py [--clips a.mp4 ...] [--frames 40]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(ROOT / "tools"))

from fall_proto import disk  # noqa: E402
from morph_check import GEN, W, H, masks_from_clip, morph_ref  # noqa: E402

MIN_PIX = 3
MIN_AREA = 200
MIN_HEIGHT = 14


def runs_per_frame(mask: np.ndarray) -> int:
    """Horizontal runs of >= MIN_PIX ones, counted exactly as the RTL does."""
    total = 0
    prev = np.zeros(mask.shape[1], dtype=bool)
    for row in mask:
        diff = np.diff(np.concatenate(([False], row, [False])).astype(np.int8))
        starts = np.nonzero(diff == 1)[0]
        ends = np.nonzero(diff == -1)[0]
        lens = ends - starts
        total += int((lens >= MIN_PIX).sum())
    del prev
    return total


def components(mask: np.ndarray) -> list[tuple[int, int, int, int, int]]:
    labels, _ = ndimage.label(mask, structure=np.ones((3, 3), bool))
    out = []
    for label, slc in enumerate(ndimage.find_objects(labels), start=1):
        if slc is None:
            continue
        local = mask[slc] & (labels[slc] == label)
        area = int(local.sum())
        if area < MIN_AREA:
            continue
        ys, xs = np.nonzero(local)
        if ys.size == 0:
            continue
        h = int(ys.max()) - int(ys.min()) + 1
        if h < MIN_HEIGHT:
            continue
        w = int(xs.max()) - int(xs.min()) + 1
        out.append((area, int(slc[1].start), int(slc[0].start), w, h))
    out.sort(reverse=True)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--clips", type=Path, nargs="*", default=[])
    ap.add_argument("--frames", type=int, default=40)
    args = ap.parse_args()
    if not args.clips:
        print("no clips given")
        return 1

    print(f"W={W} H={H} MIN_PIX={MIN_PIX} MIN_AREA={MIN_AREA} "
          f"MIN_HEIGHT={MIN_HEIGHT}")
    for clip in args.clips:
        if not clip.exists():
            print(f"SKIP missing {clip}")
            continue
        masks = masks_from_clip(clip, args.frames)
        if masks is None:
            print(f"SKIP {clip.name}: not enough frames")
            continue
        run_counts, comp_counts = [], []
        max_runs = 0
        for m in masks:
            mm = morph_ref(m)
            r = runs_per_frame(mm)
            c = len(components(mm))
            run_counts.append(r)
            comp_counts.append(c)
            max_runs = max(max_runs, r)
        arr = np.asarray(run_counts)
        car = np.asarray(comp_counts)
        print(f"{clip.name:28s} frames={len(masks):3d} "
              f"runs/frame mean={arr.mean():7.1f} p95={np.percentile(arr, 95):7.1f} "
              f"max={max_runs:4d} | comps mean={car.mean():4.2f} max={car.max()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
