"""Summarise a fall_proto.py --dump CSV.

Reports how many frames carried a box, how many of those boxes spanned the
raster, and how many alarms fired.  Alarm frames are also printed, so a clip
that repeats the same event can be read as hits-per-loop instead of raw
counts.
"""

from __future__ import annotations

import argparse
import collections
import csv
from pathlib import Path

import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csvs", type=Path, nargs="+")
    parser.add_argument("--frame-w", type=int, default=320)
    parser.add_argument("--frame-h", type=int, default=240)
    args = parser.parse_args()

    span_w = args.frame_w - 8
    span_h = args.frame_h - 8
    for path in args.csvs:
        rows = list(csv.DictReader(path.open(encoding="utf-8")))
        boxed = [r for r in rows if r["w"] != ""]
        span = [r for r in boxed
                if int(r["w"]) >= span_w or int(r["h"]) >= span_h]
        flagged = sorted({int(r["frame"]) for r in rows if r["flagged"] == "1"})
        tracks = collections.Counter(r["track"] for r in boxed)
        widths = np.array([int(r["w"]) for r in boxed]) if boxed else np.zeros(0)
        heights = np.array([int(r["h"]) for r in boxed]) if boxed else np.zeros(0)
        print(f"== {path.name}")
        print(f"   frames={len(rows)} boxed={len(boxed)} "
              f"spanning={len(span)} flagged={len(flagged)} "
              f"distinct_tracks={len(tracks)}")
        if boxed:
            print(f"   box w p50={np.percentile(widths, 50):.0f} "
                  f"p90={np.percentile(widths, 90):.0f} max={widths.max()}")
            print(f"   box h p50={np.percentile(heights, 50):.0f} "
                  f"p90={np.percentile(heights, 90):.0f} max={heights.max()}")
        print(f"   alarm frames={flagged}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
