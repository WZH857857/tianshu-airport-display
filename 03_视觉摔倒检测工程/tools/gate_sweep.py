"""Sweep the standing-reference gates at the board resolution.

``fall_stage_probe.py`` showed the seven missed GMDCSA24 fall clips never
adopt a standing reference at all: the walk-in blob is short and wide
relative to the person-likeness gate (h >= w).  This sweep measures what
relaxing that gate costs in false alarms, over every fall clip and every ADL
clip in the dataset plus the crowd negatives.

No fps rescaling: the RTL uses fixed frame counts, so the offline model has
to as well or the comparison is meaningless.

Usage::

    python tools/gate_sweep.py [--root datasets/gmdcsa24]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import imageio.v3 as iio

sys.path.insert(0, str(Path(__file__).resolve().parent))
from batch_eval import read_annotations  # noqa: E402
from fall_proto import FallDetector, Params  # noqa: E402


def board_params() -> Params:
    p = Params()
    p.target_w, p.target_h = 320, 180
    p.motion_threshold = 8
    p.erode_radius = 2
    p.dilate_radius = 2
    p.min_pixels = 4
    p.min_area = 160
    return p


def run(path: Path, p: Params) -> tuple[bool, int]:
    det = FallDetector(p)
    alarms = 0
    for frame in iio.imiter(path, plugin="FFMPEG"):
        for det_item in det.update(frame):
            if det_item.flagged:
                alarms += 1
    return alarms > 0, alarms


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", type=Path,
                    default=Path(__file__).resolve().parents[1]
                    / "datasets" / "gmdcsa24")
    ap.add_argument("--crowd", type=Path,
                    default=Path(__file__).resolve().parents[1]
                    / "datasets" / "crowd")
    args = ap.parse_args()

    falls: list[tuple[Path, tuple[float, float]]] = []
    adls: list[Path] = []
    for subject in sorted(args.root.glob("Subject *")):
        for kind, csv_name in (("Fall", "Fall.csv"), ("ADL", "ADL.csv")):
            csv_path = subject / csv_name
            if not csv_path.exists():
                continue
            table = read_annotations(csv_path)
            for clip in sorted((subject / kind).glob("*.mp4")):
                if kind == "Fall":
                    gt = table.get(clip.name)
                    if gt is not None:
                        falls.append((clip, gt))
                else:
                    adls.append(clip)
    crowds = sorted(args.crowd.glob("*.mp4"))
    print(f"fall clips {len(falls)}, adl clips {len(adls)}, "
          f"crowd clips {len(crowds)}")

    variants = [
        ("baseline h>=w", 1.00, 160, 14),
        ("h>=.90w", 0.90, 160, 14),
        ("h>=.80w", 0.80, 160, 14),
        ("h>=.70w", 0.70, 160, 14),
        ("h>=.60w", 0.60, 160, 14),
        ("h>=.80w area120", 0.80, 120, 14),
        ("h>=.70w area120", 0.70, 120, 14),
        ("h>=.70w area100 h12", 0.70, 100, 12),
    ]

    for label, ratio, min_area, min_h in variants:
        p = board_params()
        p.min_area = min_area
        p.min_height = min_h
        # person_like / PostureMachine use these three; keep them in step.
        p.min_ref_area = min_area
        # fall_proto bounds person_like at module level, so patch the module
        # attribute -- that is what FallDetector and PostureMachine resolve.
        import fall_proto

        original = fall_proto.person_like

        def patched(obs, params, frame_w, frame_h, _r=ratio, _o=original):
            if obs.h < _r * obs.w:
                return False
            return _o(obs, params, frame_w, frame_h)

        fall_proto.person_like = patched
        hits = 0
        for clip, gt in falls:
            ok, _ = run(clip, p)
            hits += int(ok)
        fp_adl = 0
        for clip in adls:
            ok, _ = run(clip, p)
            fp_adl += int(ok)
        fp_crowd = 0
        for clip in crowds:
            ok, _ = run(clip, p)
            fp_crowd += int(ok)
        fall_proto.person_like = original
        print(f"{label:<22} fall {hits:2d}/{len(falls)}  "
              f"adl-fp {fp_adl:2d}/{len(adls)}  "
              f"crowd-fp {fp_crowd}/{len(crowds)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
