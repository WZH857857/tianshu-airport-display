"""Evaluate the multi-target fall detector on crowd negatives and CCTV falls.

The GMDCSA24 batch is a single-person benchmark.  This script covers the two
cases that motivated the multi-target upgrade:

* ``datasets/crowd``  -- dense street/traffic footage with no fall.  Every
  alarm is a false positive.  The report also records how many movers were
  tracked at once, which is the evidence that one dominant box is not enough.
* ``datasets/cctv_fall`` -- confirmed fall events from the Tencent CCTV set.
  These clips are 1 fps, so the time windows are rescaled by
  ``Params.for_fps`` before the detector runs.

Usage::

    python crowd_eval.py [--crowd-dir DIR] [--fall-dir DIR] [--json REPORT]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import imageio.v3 as iio

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fall_proto import FallDetector, Params  # noqa: E402


def evaluate(path: Path, scale_fps: bool = True) -> dict:
    meta = iio.immeta(path, plugin="FFMPEG")
    fps = float(meta.get("fps") or 25.0)
    params = Params().for_fps(fps) if scale_fps else Params()
    detector = FallDetector(params)
    alarms: list[dict] = []
    active_tracks: set[int] = set()
    max_simultaneous = 0
    frames = 0
    for index, frame in enumerate(iio.imiter(path, plugin="FFMPEG")):
        frames = index + 1
        dets = detector.update(frame)
        max_simultaneous = max(max_simultaneous, len(dets))
        for det in dets:
            active_tracks.add(det.track_id)
            if det.flagged:
                alarms.append({
                    "frame": index,
                    "time": round(index / fps, 2),
                    "track": det.track_id,
                    "detail": det.detail,
                })
    return {
        "clip": path.name,
        "path": str(path),
        "fps": round(fps, 3),
        "frames": frames,
        "duration": round(frames / fps, 2),
        "alarms": alarms,
        "max_simultaneous_tracks": max_simultaneous,
        "unique_tracks": len(active_tracks),
        "params": {
            "max_targets": params.max_targets,
            "confirm_frames": params.confirm_frames,
            "collapse_frames": params.collapse_frames,
            "max_assoc_gap": params.max_assoc_gap,
            "min_fall_area_frac": params.min_fall_area_frac,
        },
    }


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--crowd-dir", type=Path,
                        default=root / "datasets" / "crowd")
    parser.add_argument("--fall-dir", type=Path,
                        default=root / "datasets" / "cctv_fall")
    parser.add_argument("--json", type=Path,
                        default=root / "out_frames" / "crowd_report.json")
    parser.add_argument("--fixed-params", action="store_true",
                        help="do not rescale the time windows by fps")
    args = parser.parse_args()

    crowd_rows = []
    if args.crowd_dir.is_dir():
        for clip in sorted(args.crowd_dir.glob("*.mp4")):
            try:
                row = evaluate(clip, not args.fixed_params)
            except Exception as exc:  # incomplete download or codec issue
                print(f"SKIP crowd/{clip.name:<24} {exc}")
                continue
            crowd_rows.append(row)
            verdict = "FP  " if row["alarms"] else "OK  "
            print(f"{verdict} crowd/{clip.name:<24} "
                  f"{row['frames']:4d} frames  max_tracks="
                  f"{row['max_simultaneous_tracks']:<2d} "
                  f"alarms={[a['time'] for a in row['alarms']] or '-'}")

    fall_rows = []
    if args.fall_dir.is_dir():
        for clip in sorted(args.fall_dir.glob("*.mp4")):
            try:
                row = evaluate(clip, not args.fixed_params)
            except Exception as exc:
                print(f"SKIP cctv/{clip.name:<28} {exc}")
                continue
            fall_rows.append(row)
            verdict = "HIT " if row["alarms"] else "MISS"
            print(f"{verdict} cctv/{clip.name:<28} "
                  f"{row['frames']:4d} frames  max_tracks="
                  f"{row['max_simultaneous_tracks']:<2d} "
                  f"alarms={[a['time'] for a in row['alarms']] or '-'}")

    fp = sum(1 for row in crowd_rows if row["alarms"])
    hits = sum(1 for row in fall_rows if row["alarms"])
    print()
    print(f"crowd clips : {fp}/{len(crowd_rows)} false alarms")
    print(f"cctv falls  : {hits}/{len(fall_rows)} detected")

    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps({
        "fps_scaled": not args.fixed_params,
        "summary": {
            "crowd_total": len(crowd_rows),
            "crowd_false_alarms": fp,
            "cctv_fall_total": len(fall_rows),
            "cctv_fall_hits": hits,
            "max_simultaneous_tracks": max(
                [row["max_simultaneous_tracks"] for row in crowd_rows] or [0]),
        },
        "crowd": crowd_rows,
        "cctv_fall": fall_rows,
    }, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"report written: {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
