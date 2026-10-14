"""Score a matrix of collapse-gate settings on one clip set.

Follow-up to ``gate_set_eval.py``: the best demo variant (erode 1 / dilate 3,
diff 16, no shrink gate) reached 10/10 loops but lit up three crowd clips.
This runs a matrix of (shrink gate, drop fraction, keep fraction) at the same
front end so the cost of each gate can be read off separately.

Usage::

    python tools/gate_matrix.py --clips demo crowd --json out/_matrix.json
"""

from __future__ import annotations

import argparse
import itertools
import json
import sys
from pathlib import Path

import imageio.v3 as iio

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fall_proto  # noqa: E402
from fall_proto import FallDetector, Params, person_like  # noqa: E402
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
    ap.add_argument("--demo", type=Path,
                    default=Path(__file__).resolve().parents[1]
                    / "out" / "test_videos" / "fall_test.mp4")
    ap.add_argument("--crowd", type=Path,
                    default=Path(__file__).resolve().parents[1]
                    / "datasets" / "crowd")
    ap.add_argument("--root", type=Path,
                    default=Path(__file__).resolve().parents[1]
                    / "datasets" / "gmdcsa24")
    ap.add_argument("--thr", type=int, default=16)
    ap.add_argument("--erode", type=int, default=1)
    ap.add_argument("--dilate", type=int, default=3)
    ap.add_argument("--crowd-frames", type=int, default=600)
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--only-shrink", default=None,
                    help="run only variants whose shrink label matches")
    args = ap.parse_args()

    crowds = sorted(args.crowd.glob("*.mp4"))
    adls: list[Path] = []
    for subject in sorted(args.root.glob("Subject *")):
        adls.extend(sorted((subject / "ADL").glob("*.mp4")))

    shrink_choices = [
        ("shrink.65", True, 0.65),
        ("shrink.80", True, 0.80),
        ("shrink1.0", True, 1.00),
        ("noshrink", False, 0.65),
    ]
    drop_choices = [("drop.04", True), ("nodrop", False)]
    keep_choices = [0.30, 0.20, 0.10]

    rows = []
    for (s_label, need_shrunk, shrunk_frac), \
            (d_label, need_sunk), keep_frac in itertools.product(
                shrink_choices, drop_choices, keep_choices):
        if args.only_shrink and s_label != args.only_shrink:
            continue
        label = f"{s_label}-{d_label}-keep{keep_frac}"
        p = board_params(args.thr, args.erode, args.dilate)
        plike, feed = make_gate(1.00, need_shrunk, shrunk_frac,
                                need_sunk, keep_frac, 2)
        demo = run(args.demo, p, plike, feed, 10 ** 9)
        crowd_hits = []
        for clip in crowds:
            alarms = run(clip, p, plike, feed, args.crowd_frames)
            if alarms:
                crowd_hits.append(f"{clip.name}:{alarms[0]}")
        adl_hits = []
        for clip in adls:
            if run(clip, p, plike, feed, 10 ** 9):
                adl_hits.append(f"{clip.parent.parent.name}/{clip.name}")
        row = {
            "label": label,
            "demo": len(demo),
            "crowd": len(crowd_hits),
            "crowd_hits": crowd_hits,
            "adl": len(adl_hits),
            "adl_hits": adl_hits,
        }
        rows.append(row)
        print(f"{label:<34} demo {len(demo):2d}/10  "
              f"crowd {len(crowd_hits)} [{','.join(crowd_hits)}]  "
              f"adl {len(adl_hits)} [{','.join(adl_hits)}]")
        if args.json:
            args.json.write_text(json.dumps(rows, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
