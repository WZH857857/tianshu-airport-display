"""How long does a published box survive, and how big does it get?

The complaint this measures is "the box should stay glued to one person":
before the grace window a track died the first time the frame difference
missed a limb, so the monitor showed a box for one frame and then nothing.

For every clip this prints

  * raw    -- boxes published by ``multi_bbox`` (the pre-tracker HUD);
  * PNR20  -- no grace window, publish the detection box unchanged;
  * grace  -- grace window only;
  * lock   -- elastic lock box only;
  * PNR21  -- the current policy: grace window + elastic lock box.

A *track* is a run of consecutive frames where one slot stays occupied, so
the lifetime numbers are directly what the eye sees on the HDMI monitor.

Usage::

    python track_stats.py CLIP.mp4 [CLIP.mp4 ...] [--frames 500]
"""

from __future__ import annotations

import argparse
import statistics as stats
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import box_size_stats as B  # noqa: E402
import track_model as T  # noqa: E402


def box_area(slot) -> int:
    return int(slot[3]) * int(slot[4])


def runs(frames: list[list[tuple[int, ...]]]) -> list[dict]:
    """One entry per contiguous slot occupancy: the lifetime of a box."""
    live: list[dict | None] = [None] * T.NUM_BLOBS
    out: list[dict] = []
    for f, frame in enumerate(frames):
        for s in range(T.NUM_BLOBS):
            slot = frame[s] if s < len(frame) else (0, 0, 0, 0, 0, 0)
            if slot[0]:
                if live[s] is None:
                    live[s] = {"start": f, "frames": 0, "areas": [],
                               "centres": []}
                live[s]["frames"] += 1
                live[s]["areas"].append(box_area(slot))
                live[s]["centres"].append((int(slot[1]) + int(slot[3]) // 2,
                                           int(slot[2]) + int(slot[4]) // 2))
            elif live[s] is not None:
                out.append(live[s])
                live[s] = None
    for s in range(T.NUM_BLOBS):
        if live[s] is not None:
            out.append(live[s])
    return out


def summarise(tag: str, tracks: list[dict], frames: int, boxes: int) -> None:
    if not tracks:
        print(f"  {tag:<24} tracks=0")
        return
    life = sorted(t["frames"] for t in tracks)
    areas = [a for t in tracks for a in t["areas"]]
    steps = []
    for t in tracks:
        for a, b in zip(t["centres"], t["centres"][1:]):
            steps.append(max(abs(a[0] - b[0]), abs(a[1] - b[1])))
    short = sum(1 for v in life if v <= 3)
    long_ = sum(1 for v in life if v >= 30)
    jit_med = int(stats.median(steps)) if steps else 0
    jit90 = int(np.percentile(steps, 90)) if steps else 0
    area_med = int(stats.median(areas)) if areas else 0
    area_p90 = int(np.percentile(areas, 90)) if areas else 0
    print(f"  {tag:<24} tracks={len(tracks):<4} "
          f"life med={int(stats.median(life)):<4} max={life[-1]:<4} "
          f"<=3fr={100.0 * short / len(life):4.1f}%  "
          f">=30fr={long_:<3} "
          f"boxes={boxes:<5} ({boxes / max(1, frames):.2f}/fr) "
          f"area med={area_med:<5} p90={area_p90:<6} max={max(areas) if areas else 0:<6} "
          f"jitter med={jit_med:<3} p90={jit90}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("clips", type=Path, nargs="+")
    ap.add_argument("--frames", type=int, default=500)
    ap.add_argument("--detail", action="store_true",
                    help="print the longest tracks of the current policy")
    args = ap.parse_args()

    for clip in args.clips:
        if not clip.exists():
            print(f"SKIP missing {clip}")
            continue
        masks = B.masks_of(clip, args.frames, 2, 2)
        raw = [[tuple(int(v) for v in b) for b in B.boxes_of(m, 1)]
               for m in masks]
        n = len(raw)
        raw_boxes = sum(1 for f in raw for s in f if s[0])

        keep_lock = T.lock_update
        raw_lock = lambda lock, det, shrink=None: (det[1], det[2],   # noqa
                                                   det[3], det[4])
        variants = []
        for tag, use_lock, grace_frames in (
                ("PNR20 (no grace, no lock)", False, 0),
                ("grace window only", False, 6),
                ("lock box only", True, 0),
                ("lock + grace (PNR21)", True, 6)):
            T.lock_update = keep_lock if use_lock else raw_lock
            T.GRACE_FRAMES = grace_frames
            variants.append((tag, T.track_slots(raw)))
        T.lock_update = keep_lock

        print(f"{clip.name}: {n} frames, motion px/frame median "
              f"{int(np.median(masks.sum(axis=(1, 2))))}")
        summarise("raw multi_bbox", runs([[tuple(b) for b in f]
                                          for f in raw]), n, raw_boxes)
        for tag, seq in variants:
            summarise(tag, runs(seq), n,
                      sum(1 for f in seq for s in f if s[0]))
        if args.detail:
            long_tracks = sorted(runs(variants[-1][1]),
                                 key=lambda t: -t["frames"])[:8]
            for t in long_tracks:
                first = t["centres"][0]
                last = t["centres"][-1]
                print(f"    f{t['start']:>3} life={t['frames']:>3} "
                      f"area {t['areas'][0]:>5} -> {t['areas'][-1]:<5} "
                      f"centre {first} -> {last}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
