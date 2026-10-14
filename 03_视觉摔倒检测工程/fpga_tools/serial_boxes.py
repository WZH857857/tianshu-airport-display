"""Decode a captured serial log into "is the box locked on the person?" numbers.

``serial_capture.py`` only counts lines; this one reconstructs the per-slot
box track that the HDMI monitor actually drew, so two bitstreams can be
compared on the same scene:

  * how many frames show at least one box (a box that dies at once leaves
    most frames empty);
  * how long one slot stays occupied -- the *lifetime* of what the eye sees;
  * how big the published boxes are.

Usage::

    python serial_boxes.py out/serial_pnr21_live.txt
    python serial_boxes.py --compare out/serial_pnr20_ab.txt out/serial_pnr21_live.txt
"""

from __future__ import annotations

import argparse
import re
import statistics as stats
from pathlib import Path

import numpy as np

LINE = re.compile(
    r"^S([0-3]) F([0-9A-F]{4}) ([01]) ([0-9A-F]{3}) ([0-9A-F]{3}) "
    r"([0-9A-F]{3}) ([0-9A-F]{3}) ([0-9A-F]{5}) ([0-7])")


def parse(path: Path):
    """-> {frame: {slot: (w, h)}} plus the raw line count."""
    slots: dict[int, dict[int, tuple[int, int]]] = {}
    n_lines = 0
    for line in path.read_text(errors="replace").splitlines():
        m = LINE.match(line.strip())
        if not m:
            continue
        n_lines += 1
        frame = int(m.group(2), 16)
        found = m.group(3) == "1"
        if found:
            w = int(m.group(6), 16)
            h = int(m.group(7), 16)
            slots.setdefault(frame, {})[int(m.group(1))] = (w, h)
        else:
            slots.setdefault(frame, {})
    return slots, n_lines


def runs(slots: dict[int, dict[int, tuple[int, int]]]):
    """Consecutive-frame occupancy of one slot = the lifetime of one box."""
    order = sorted(slots)
    per_slot: dict[int, list[int]] = {s: [] for s in range(4)}
    for f in order:
        for s in range(4):
            per_slot[s].append(1 if s in slots[f] else 0)
    out = []
    for s, bits in per_slot.items():
        run = 0
        for b in bits:
            if b:
                run += 1
            elif run:
                out.append(run)
                run = 0
        if run:
            out.append(run)
    return out


def report(tag: str, path: Path) -> dict:
    slots, n_lines = parse(path)
    frames = sorted(slots)
    span = frames[-1] - frames[0] + 1 if frames else 0
    boxes = [wh for f in slots.values() for wh in f.values()]
    busy = sum(1 for f in slots.values() if f)
    lives = sorted(runs(slots))
    row = {
        "tag": tag,
        "frames": span,
        "lines": n_lines,
        "busy": busy,
        "boxes": len(boxes),
        "life_med": int(stats.median(lives)) if lives else 0,
        "life_p90": (sorted(lives)[int(0.9 * (len(lives) - 1))]
                     if lives else 0),
        "life_max": lives[-1] if lives else 0,
        "short": (100.0 * sum(1 for v in lives if v <= 3) / len(lives)
                  if lives else 0.0),
        "w": int(stats.median([b[0] for b in boxes])) if boxes else 0,
        "h": int(stats.median([b[1] for b in boxes])) if boxes else 0,
        "w90": int(np.percentile([b[0] for b in boxes], 90)) if boxes else 0,
        "h90": int(np.percentile([b[1] for b in boxes], 90)) if boxes else 0,
        "wmax": max((b[0] for b in boxes), default=0),
        "hmax": max((b[1] for b in boxes), default=0),
        "runs": len(lives),
    }
    print(f"{tag}")
    print(f"  {row['frames']} frames ({row['lines']} S lines, "
          f"{row['lines'] / max(1, row['frames']):.2f}/frame)")
    print(f"  frames with >=1 box : {row['busy']}/{row['frames']} "
          f"({100.0 * row['busy'] / max(1, row['frames']):.1f}%)")
    print(f"  published boxes     : {row['boxes']} "
          f"({row['boxes'] / max(1, row['frames']):.2f}/frame)")
    print(f"  box lifetimes       : tracks={row['runs']} "
          f"med={row['life_med']} p90={row['life_p90']} "
          f"max={row['life_max']} <=3fr={row['short']:.1f}%")
    print(f"  box size (w x h)    : {row['w']} x {row['h']} (median)")
    print(f"                        {row['w90']} x {row['h90']} (p90), "
          f"{row['wmax']} x {row['hmax']} (max)")
    return row


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logs", type=Path, nargs="+")
    ap.add_argument("--compare", action="store_true")
    args = ap.parse_args()

    rows = [report(p.stem, p) for p in args.logs]
    if args.compare and len(rows) == 2:
        a, b = rows
        print(f"\n{a['tag']} -> {b['tag']}")
        print(f"  frames with a box : {100.0 * a['busy'] / max(1, a['frames']):.1f}%"
              f" -> {100.0 * b['busy'] / max(1, b['frames']):.1f}%")
        print(f"  box lifetime med  : {a['life_med']} -> {b['life_med']} frames")
        print(f"  tracks <=3 frames : {a['short']:.1f}% -> {b['short']:.1f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
