"""Batch evaluation of the fall detector against the GMDCSA24 annotations.

For every clip the ground-truth "Falling [a to b]" interval is parsed out of
the subject CSV, the detector is replayed over the whole clip, and an alarm is
scored as a hit when it lands inside that interval.

ADL clips carry no Falling interval, so any alarm there is a false positive.

Usage::

    python batch_eval.py [--root DATASETS] [--blocks N] [--json REPORT.json]
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path

import imageio.v3 as iio

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fall_proto import FallDetector, Params  # noqa: E402

# The CSVs are hand written: some rows say "Falling (BW)[1.2 to 5]", others
# "Fall (FW)[0 to 3]", so accept both spellings.
FALL_RE = re.compile(r"Fall(?:ing)?[^\[\]]*\[\s*([0-9.]+)\s*to\s*([0-9.]+)\s*\]")


def read_annotations(csv_path: Path) -> dict[str, tuple[float, float] | None]:
    """Map clip filename -> (fall_start, fall_end) seconds, or None for ADL."""
    table: dict[str, tuple[float, float] | None] = {}
    with csv_path.open(newline="", encoding="utf-8-sig") as fh:
        for raw in csv.DictReader(fh):
            row = {k.strip(): (v or "") for k, v in raw.items() if k}
            name = row.get("File Name", "").strip()
            if not name:
                continue
            match = FALL_RE.search(row.get("Classes", ""))
            table[name] = ((float(match.group(1)), float(match.group(2)))
                           if match else None)
    return table


def evaluate(path: Path, gt: tuple[float, float] | None,
             params: Params, scale_fps: bool = True) -> dict:
    meta = iio.immeta(path, plugin="FFMPEG")
    fps = float(meta.get("fps") or 25.0)
    run_params = params.for_fps(fps) if scale_fps else params
    detector = FallDetector(run_params)
    alarms: list[tuple[float, str]] = []
    frames = 0
    for index, frame in enumerate(iio.imiter(path, plugin="FFMPEG")):
        frames = index + 1
        for det in detector.update(frame):
            if det.flagged:
                alarms.append((index / fps, det.detail))
    hit = None
    if gt is not None:
        hit = any(gt[0] - 0.5 <= t <= gt[1] for t, _ in alarms)
    return {
        "clip": str(path),
        "fps": round(fps, 3),
        "frames": frames,
        "duration": round(frames / fps, 2),
        "gt": gt,
        "alarms": [round(t, 2) for t, _ in alarms],
        "details": [d for _, d in alarms],
        "hit": hit,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path,
                        default=Path(__file__).resolve().parents[1] / "datasets" / "gmdcsa24")
    parser.add_argument("--json", type=Path, default=None)
    parser.add_argument("--only-fall", action="store_true",
                        help="skip ADL clips (faster smoke runs)")
    parser.add_argument("--fixed-params", action="store_true",
                        help="do not rescale the time windows by fps")
    parser.add_argument("--erode", type=int, default=None,
                        help="erosion radius (default 1; RTL now uses 2)")
    parser.add_argument("--dilate", type=int, default=None,
                        help="dilation radius (default 3; RTL now uses 2)")
    args = parser.parse_args()

    params = Params()
    if args.erode is not None:
        params.erode_radius = args.erode
    if args.dilate is not None:
        params.dilate_radius = args.dilate
    rows = []
    for subject_dir in sorted(args.root.glob("Subject *")):
        for kind, csv_name in (("Fall", "Fall.csv"), ("ADL", "ADL.csv")):
            csv_path = subject_dir / csv_name
            if not csv_path.exists():
                continue
            if args.only_fall and kind == "ADL":
                continue
            table = read_annotations(csv_path)
            clip_dir = subject_dir / kind
            if not clip_dir.is_dir():
                continue
            for clip in sorted(clip_dir.glob("*.mp4")):
                gt = table.get(clip.name, None)
                result = evaluate(clip, gt, params, not args.fixed_params)
                result["subject"] = subject_dir.name
                result["kind"] = kind
                rows.append(result)
                if result["hit"] is None:
                    verdict = "FP  " if result["alarms"] else "OK  "
                else:
                    verdict = "HIT " if result["hit"] else "MISS"
                alarms = result["alarms"]
                print(f"{verdict} {subject_dir.name}/{kind}/{clip.name:<8} "
                      f"gt={gt} alarm={alarms if alarms else '-'}")

    falls = [r for r in rows if r["gt"] is not None]
    adls = [r for r in rows if r["gt"] is None]
    hits = sum(1 for r in falls if r["hit"])
    fp = sum(1 for r in adls if r["alarms"])
    print()
    print(f"fall clips : {hits}/{len(falls)} detected")
    print(f"adl clips  : {fp}/{len(adls)} false alarms")

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps({
            "params": params.__dict__,
            "summary": {
                "fall_total": len(falls),
                "fall_hits": hits,
                "adl_total": len(adls),
                "adl_false_alarms": fp,
            },
            "rows": rows,
        }, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"report written: {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
