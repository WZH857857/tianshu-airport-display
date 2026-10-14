"""Full-length recall/false-positive score for the recall-fix candidates.

Runs one front-end + gate configuration over every GMDCSA24 Fall clip
(recall), every ADL clip (false positives) and every crowd clip (false
positives), with no frame caps, and writes the per-clip result to JSON so a
long run can be inspected while it is still going.

Usage::

    python tools/final_eval.py --erode 1 --dilate 2 --thr 12 ^
        --json out/_final_e1d2t12.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import imageio.v3 as iio

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fall_proto  # noqa: E402
from fall_proto import FallDetector, Params  # noqa: E402
from gate_set_eval import make_gate  # noqa: E402


def board_params(thr: int, erode: int, dilate: int) -> Params:
    p = Params()
    p.target_w, p.target_h = 320, 180
    p.motion_threshold = thr
    p.erode_radius = erode
    p.dilate_radius = dilate
    p.min_pixels = 4
    p.min_area = 160
    return p


def run(clip: Path, p: Params, plike, feed, cap: int) -> list[int]:
    original_plike = fall_proto.person_like
    original_feed = fall_proto.PostureMachine.feed
    fall_proto.person_like = plike
    fall_proto.PostureMachine.feed = feed
    try:
        det = FallDetector(p)
        alarms: list[int] = []
        for index, frame in enumerate(iio.imiter(clip, plugin="FFMPEG")):
            if index >= cap:
                break
            for det_item in det.update(frame):
                if det_item.flagged:
                    alarms.append(index)
    finally:
        fall_proto.person_like = original_plike
        fall_proto.PostureMachine.feed = original_feed
    return alarms


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", type=Path,
                    default=Path(__file__).resolve().parents[1]
                    / "datasets" / "gmdcsa24")
    ap.add_argument("--crowd", type=Path,
                    default=Path(__file__).resolve().parents[1]
                    / "datasets" / "crowd")
    ap.add_argument("--demo", type=Path,
                    default=Path(__file__).resolve().parents[1]
                    / "out" / "test_videos" / "fall_test.mp4",
                    help="looped fall montage; one alarm per detected event")
    ap.add_argument("--thr", type=int, default=12)
    ap.add_argument("--erode", type=int, default=1)
    # Defaults track the shipped board configuration (fall_camera_hdmi_multi_top
    # DIFF_THRESH=12, disk(1) erode + disk(3) dilate, bottom_tol 0.10).
    ap.add_argument("--dilate", type=int, default=3)
    ap.add_argument("--shrink-frac", type=float, default=1.00)
    ap.add_argument("--need-shrunk", type=int, default=1)
    ap.add_argument("--need-sunk", type=int, default=1)
    ap.add_argument("--keep-frac", type=float, default=0.30)
    ap.add_argument("--confirm", type=int, default=2)
    ap.add_argument("--bottom-tol", type=float, default=0.10)
    ap.add_argument("--crowd-frames", type=int, default=10 ** 9)
    ap.add_argument("--legacy", action="store_true",
                    help="old front end and old collapse gate, for an "
                         "apples-to-apples before/after under the same "
                         "any-alarm criterion: 320x240, thr 10, "
                         "disk(2)/disk(2), min 3/200, shrink 0.65, "
                         "collapse 18, bottom 0.15")
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args()

    falls: list[Path] = []
    adls: list[Path] = []
    for subject in sorted(args.root.glob("Subject *")):
        falls.extend(sorted((subject / "Fall").glob("*.mp4")))
        adls.extend(sorted((subject / "ADL").glob("*.mp4")))
    crowds = sorted(args.crowd.glob("*.mp4"))

    p = board_params(args.thr, args.erode, args.dilate)
    p.bottom_tol_frac = args.bottom_tol
    if args.legacy:
        p.target_w, p.target_h = 320, 240
        p.motion_threshold = 10
        p.erode_radius, p.dilate_radius = 2, 2
        p.min_pixels, p.min_area = 3, 200
        p.collapse_frames = 18
        p.bottom_tol_frac = 0.15
        args.shrink_frac = 0.65
        args.need_shrunk = 1
        args.keep_frac = 0.30
        args.confirm = 2
    plike, feed = make_gate(1.00, bool(args.need_shrunk), args.shrink_frac,
                            bool(args.need_sunk), args.keep_frac,
                            args.confirm)

    result: dict[str, object] = {
        # Read the numbers back off p: on a --legacy run the CLI defaults were
        # overridden, and the log has to describe the pipeline that actually
        # ran, not the flags that were typed.
        "thr": p.motion_threshold,
        "erode": p.erode_radius,
        "dilate": p.dilate_radius,
        "target_w": p.target_w, "target_h": p.target_h,
        "min_pixels": p.min_pixels, "min_area": p.min_area,
        "collapse_frames": p.collapse_frames,
        "bottom_tol_frac": p.bottom_tol_frac,
        "shrink_frac": args.shrink_frac, "need_shrunk": args.need_shrunk,
        "need_sunk": args.need_sunk, "keep_frac": args.keep_frac,
        "confirm": args.confirm,
        "legacy": bool(args.legacy),
    }

    if args.demo.exists():
        demo_alarms = run(args.demo, p, plike, feed, 10 ** 9)
        result["demo_events"] = len(demo_alarms)
        result["demo_alarms"] = demo_alarms
        print(f"DEMO {args.demo.name:<24} {len(demo_alarms):2d}/10  "
              f"{demo_alarms}", flush=True)
    else:
        print(f"SKIP demo {args.demo}", flush=True)

    fall_hits, fall_miss = [], []
    for clip in falls:
        alarms = run(clip, p, plike, feed, 10 ** 9)
        tag = f"{clip.parent.parent.name}/{clip.name}"
        (fall_hits if alarms else fall_miss).append(
            {"clip": tag, "alarms": alarms})
        print(f"FALL {tag:<24} {'HIT ' if alarms else 'MISS'} {alarms[:3]}",
              flush=True)
    result["fall_hits"] = fall_hits
    result["fall_miss"] = fall_miss
    result["fall_recall"] = f"{len(fall_hits)}/{len(falls)}"

    adl_hits = []
    for clip in adls:
        alarms = run(clip, p, plike, feed, 10 ** 9)
        tag = f"{clip.parent.parent.name}/{clip.name}"
        if alarms:
            adl_hits.append({"clip": tag, "alarms": alarms})
            print(f"ADL  {tag:<24} FP   {alarms[:3]}", flush=True)
    result["adl_fp"] = adl_hits
    result["adl_total"] = len(adls)

    crowd_hits = []
    for clip in crowds:
        alarms = run(clip, p, plike, feed, args.crowd_frames)
        if alarms:
            crowd_hits.append({"clip": clip.name, "alarms": alarms})
            print(f"CROWD {clip.name:<24} FP   {alarms[:3]}", flush=True)
    result["crowd_fp"] = crowd_hits
    result["crowd_total"] = len(crowds)

    print(f"SUMMARY thr={p.motion_threshold} erode={p.erode_radius} "
          f"dilate={p.dilate_radius} "
          f"target={p.target_w}x{p.target_h} "
          f"minpix={p.min_pixels} minarea={p.min_area} "
          f"bottom={p.bottom_tol_frac} "
          f"shrink={args.shrink_frac} need_shrunk={args.need_shrunk} "
          f"need_sunk={args.need_sunk} keep={args.keep_frac} "
          f"confirm={args.confirm}")
    print(f"  fall {len(fall_hits)}/{len(falls)}  "
          f"adl-fp {len(adl_hits)}/{len(adls)}  "
          f"crowd-fp {len(crowd_hits)}/{len(crowds)}")
    if args.json:
        result["summary"] = (f"fall {len(fall_hits)}/{len(falls)} "
                             f"adl-fp {len(adl_hits)}/{len(adls)} "
                             f"crowd-fp {len(crowd_hits)}/{len(crowds)}")
        args.json.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
