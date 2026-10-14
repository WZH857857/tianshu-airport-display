"""Slot-order statistics for the crowd path.

``multi_bbox`` publishes its four boxes ordered by area *within the frame*,
not by identity: slot 0 is always the strongest blob of this frame.  The
board path adds ``track_assoc`` to renumber those boxes by IoU.  This script
measures both the raw order and the tracked order on the same clips.

For every consecutive frame pair it IoU-matches each found slot against the
previous frame's four boxes and reports

  * slots found per frame (how much of the crowd the detector sees),
  * same-slot match rate (box k still overlaps box k of the previous frame),
  * cross-slot rate (another slot is the better match: an order swap),
  * mean IoU of the same-slot matches (how much the box itself moves).

Run with the RTL-equivalent model (``multi_model.frames_model``), i.e. the
boxes whose equality with the RTL is pinned down by ``multi_bbox_check.py``.

Usage::

    python multi_order_stats.py [--clips a.mp4 ...] [--frames 60]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(ROOT / "tools"))

from morph_check import masks_from_clip, morph_ref  # noqa: E402
from multi_model import frames_model  # noqa: E402
from track_model import track_slots  # noqa: E402

NUM_BLOBS = 4
MATCH_IOU = 0.25        # keep in step with the RTL: 4*inter >= union

DEFAULT_CLIPS = [ROOT / "datasets" / "crowd" / f"crowd{i:02d}.mp4"
                 for i in (1, 3, 5)]


def iou(a: tuple[int, ...], b: tuple[int, ...]) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x0, y0 = max(ax, bx), max(ay, by)
    x1, y1 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    if x1 <= x0 or y1 <= y0:
        return 0.0
    inter = (x1 - x0) * (y1 - y0)
    union = aw * ah + bw * bh - inter
    return inter / union if union else 0.0


def stats_for_frames(frames) -> dict:
    found_per_frame = []
    same_slot = 0
    cross_slot = 0
    new_box = 0
    same_ious = []

    for f in range(1, len(frames)):
        prev = frames[f - 1]
        cur = frames[f]
        prev_boxes = [(k, t[1:5]) for k, t in enumerate(prev) if t[0]]
        found_per_frame.append(sum(1 for t in cur if t[0]))
        for k, t in enumerate(cur):
            if not t[0]:
                continue
            box = t[1:5]
            best_k, best_iou = None, 0.0
            for pk, pbox in prev_boxes:
                v = iou(box, pbox)
                if v > best_iou:
                    best_k, best_iou = pk, v
            if best_iou < MATCH_IOU:
                new_box += 1
            elif best_k == k:
                same_slot += 1
                same_ious.append(best_iou)
            else:
                cross_slot += 1

    total = same_slot + cross_slot + new_box
    return {
        "mean_found": float(np.mean(found_per_frame)) if found_per_frame else 0.0,
        "total": total,
        "same": same_slot,
        "cross": cross_slot,
        "new": new_box,
        "mean_iou": float(np.mean(same_ious)) if same_ious else 0.0,
    }


def stats_for(clip: Path, nframes: int) -> tuple[dict, dict] | None:
    masks = masks_from_clip(clip, nframes)
    if masks is None:
        return None
    morphed = np.stack([morph_ref(m) for m in masks])
    raw = frames_model(morphed)
    tracked = track_slots(raw)
    return stats_for_frames(raw), stats_for_frames(tracked)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--clips", type=Path, nargs="*", default=DEFAULT_CLIPS)
    ap.add_argument("--frames", type=int, default=60)
    args = ap.parse_args()

    for clip in args.clips:
        if not clip.exists():
            print(f"SKIP missing {clip}")
            continue
        res = stats_for(clip, args.frames)
        if res is None:
            print(f"SKIP {clip.name}: not enough frames")
            continue
        raw, tracked = res
        print(f"{clip.name}: {args.frames} frames")
        for label, s in (("raw", raw), ("tracked", tracked)):
            pct = 100.0 * s["same"] / s["total"] if s["total"] else 0.0
            cont = s["same"] + s["cross"]
            cont_pct = 100.0 * s["same"] / cont if cont else 0.0
            print(f"  {label:7s} slots/frame {s['mean_found']:.2f}, "
                  f"same-slot {s['same']}/{s['total']} = {pct:.1f}%, "
                  f"continuations {s['same']}/{cont} = {cont_pct:.1f}%, "
                  f"cross-slot {s['cross']}, new {s['new']}, "
                  f"mean IoU {s['mean_iou']:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
