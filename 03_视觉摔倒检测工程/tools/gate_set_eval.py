"""Which combination of collapse gates recovers the looped fall demo?

``criteria_probe.py`` showed that in the loops the board misses, the falling
blob grows (toward-camera fall) instead of shrinking, so the ``shrunk`` and
``sunk`` gates fail while ``flat`` + ``grounded`` hold.  This script scores
gate combinations over the ten loops of the demo clip and, for the best
ones, over the crowd negatives and the GMDCSA24 ADL clips.

Writes JSON so a long run can be inspected while it is still going.

Usage::

    python tools/gate_set_eval.py --json out/_gates.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import imageio.v3 as iio
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fall_proto  # noqa: E402
from fall_proto import FallDetector, Params, person_like  # noqa: E402


def board_params() -> Params:
    p = Params()
    p.target_w, p.target_h = 320, 180
    p.motion_threshold = 8
    p.erode_radius = 2
    p.dilate_radius = 2
    p.min_pixels = 4
    p.min_area = 160
    return p


def make_gate(h_ratio: float, need_shrunk: bool, shrunk_frac: float,
              need_sunk: bool, keep_frac: float, confirm: int):
    """Monkey-patchable person_like + a feed() that uses the new gates."""

    def patched_plike(obs, params, frame_w, frame_h):
        if obs.h < h_ratio * obs.w:
            return False
        return person_like(obs, params, frame_w, frame_h)

    def patched_feed(self, obs, frame=None):
        p = self.p
        if self.cooldown:
            self.cooldown -= 1
        if obs is None:
            self.flat_frames = 0
            if (self.pending_alarm and frame is not None
                    and frame - self.pending_frame >= p.recovery_window):
                return self._confirm()
            return False, self.state, "no-motion"
        if self.h_ref is not None and obs.frame - self.h_ref.frame > p.ref_ttl:
            self.h_ref = None
        if (self.person_like_ref(obs)
                and (self.h_ref is None or obs.h > self.h_ref.h)):
            self.h_ref = obs
        if self.h_ref is not None and obs.h >= p.tall_frac * self.h_ref.h:
            self.last_tall = obs.frame
        flat = obs.ratio <= p.lying_ratio
        self.flat_frames = self.flat_frames + 1 if flat else 0
        ref = self.h_ref
        if self.pending_alarm:
            rises = obs.y <= (self.pending_top
                              - p.recovery_rise_frac * self.pending_ref_h)
            stands = (ref is not None and obs.h >= 0.80 * ref.h
                      and obs.ratio >= p.upright_ratio)
            if rises or stands:
                self.pending_alarm = False
                self.state = "RECOVERED"
                self.flat_frames = 0
            elif obs.frame - self.pending_frame >= p.recovery_window:
                return self._confirm()
        if (ref is not None and self.cooldown == 0
                and not self.pending_alarm):
            tall = (ref.h >= p.min_height and self.person_like_ref(ref))
            shrunk = obs.h <= shrunk_frac * ref.h
            sunk = (obs.cy - ref.cy) >= p.drop_frac * self.frame_h
            grounded = obs.bottom >= ref.bottom - p.bottom_tol_frac * ref.h
            keep = obs.area >= keep_frac * ref.area
            fast = (obs.frame - self.last_tall) <= p.collapse_frames
            ok = (tall and flat and grounded and keep and fast
                  and (shrunk or not need_shrunk)
                  and (sunk or not need_sunk)
                  and self.flat_frames >= confirm)
            if ok:
                self.pending_alarm = True
                self.pending_frame = obs.frame
                self.pending_top = obs.y
                self.pending_ref_h = ref.h
                self.state = "PENDING"
                if p.recovery_window <= 1:
                    return self._confirm()
        if self.state == "FALL" and not flat:
            self.state = "RECOVERED"
        return False, self.state, ""

    return patched_plike, patched_feed


def score(clip: Path, p: Params, plike, feed) -> tuple[int, list[int]]:
    original_plike = fall_proto.person_like
    original_feed = fall_proto.PostureMachine.feed
    fall_proto.person_like = plike
    fall_proto.PostureMachine.feed = feed
    try:
        det = FallDetector(p)
        alarms: list[int] = []
        for index, frame in enumerate(iio.imiter(clip, plugin="FFMPEG")):
            for det_item in det.update(frame):
                if det_item.flagged:
                    alarms.append(index)
    finally:
        fall_proto.person_like = original_plike
        fall_proto.PostureMachine.feed = original_feed
    return len(alarms), alarms


def score_limited(clip: Path, p: Params, plike, feed,
                  max_frames: int) -> tuple[int, list[int]]:
    """score() with a frame cap, for the long crowd negatives."""
    original_plike = fall_proto.person_like
    original_feed = fall_proto.PostureMachine.feed
    fall_proto.person_like = plike
    fall_proto.PostureMachine.feed = feed
    try:
        det = FallDetector(p)
        alarms: list[int] = []
        for index, frame in enumerate(iio.imiter(clip, plugin="FFMPEG")):
            if index >= max_frames:
                break
            for det_item in det.update(frame):
                if det_item.flagged:
                    alarms.append(index)
    finally:
        fall_proto.person_like = original_plike
        fall_proto.PostureMachine.feed = original_feed
    return len(alarms), alarms


def probe_geometry(clip: Path, p: Params, limit: int) -> None:
    """Print the running maximum blob geometry, one line per 10 frames.

    Shows whether a person-sized blob ever exists at all: if the global
    maximum area never clears a few hundred pixels, the front end is the
    blocker, not the posture gates.
    """
    det = FallDetector(p)
    for index, frame in enumerate(iio.imiter(clip, plugin="FFMPEG")):
        if index >= limit:
            break
        det.update(frame)
        if index % 10:
            continue
        best = None
        for track in det.tracks:
            if track.last_bbox is None:
                continue
            x, y, w, h = track.last_bbox
            obs = det.observe(track.last_bbox, track.last_area)
            if best is None or obs.area > best.area:
                best = obs
        if best is None:
            print(f"f{index:4d} -")
        else:
            print(f"f{index:4d} {best.w:3d}x{best.h:3d} area={best.area:5d} "
                  f"ratio={best.ratio:.2f} fill={best.fill:.2f} "
                  f"plike={person_like(best, p, 320, 180)}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--crowd", type=Path,
                    default=Path(__file__).resolve().parents[1]
                    / "datasets" / "crowd")
    ap.add_argument("--root", type=Path,
                    default=Path(__file__).resolve().parents[1]
                    / "datasets" / "gmdcsa24")
    ap.add_argument("--demo",
                    type=Path,
                    default=Path(__file__).resolve().parents[1]
                    / "out" / "test_videos" / "fall_test.mp4")
    ap.add_argument("--quick", action="store_true",
                    help="demo only, no crowd/ADL validation")
    ap.add_argument("--probe", type=int, default=0,
                    help="print blob geometry for the first N demo frames")
    ap.add_argument("--crowd-frames", type=int, default=900,
                    help="frame cap per crowd clip (long negatives)")
    ap.add_argument("--only", nargs="*", default=None,
                    help="run only the variants whose label starts with one "
                         "of these strings")
    args = ap.parse_args()

    if args.probe:
        probe_geometry(args.demo, board_params(), args.probe)
        return 0

    variants = [
        # label, thr, erode, dilate, h_ratio, need_shrunk, shrunk_frac,
        # need_sunk, keep, confirm
        ("e1d3-t16", 16, 1, 3, 1.00, True, 0.65, True, 0.30, 2),
        ("e1d3-t16-noshrink", 16, 1, 3, 1.00, False, 0.65, False, 0.30, 2),
        ("e1d4-t20", 20, 1, 4, 1.00, True, 0.65, True, 0.30, 2),
        ("e1d4-t20-noshrink", 20, 1, 4, 1.00, False, 0.65, False, 0.30, 2),
        ("e1d3-t12", 12, 1, 3, 1.00, True, 0.65, True, 0.30, 2),
        ("e1d3-t12-noshrink", 12, 1, 3, 1.00, False, 0.65, False, 0.30, 2),
        ("minref800", 8, 2, 2, 1.00, True, 0.65, True, 0.30, 2),
        ("minref800-noshrink", 8, 2, 2, 1.00, False, 0.65, False, 0.30, 2),
        ("h0-noshrink", 8, 2, 2, 0.00, False, 0.65, False, 0.30, 2),
        ("base", 8, 2, 2, 1.00, True, 0.65, True, 0.30, 2),
        ("nosunk", 8, 2, 2, 1.00, True, 0.65, False, 0.30, 2),
        ("noshrink", 8, 2, 2, 1.00, False, 0.65, True, 0.30, 2),
        ("noshrink-nosunk-keep.2", 8, 2, 2, 1.00, False, 0.65, False, 0.20, 2),
        ("noshrink-nosunk-keep.2-c1", 8, 2, 2, 1.00, False, 0.65, False,
         0.20, 1),
    ]

    adls: list[Path] = []
    if not args.quick:
        for subject in sorted(args.root.glob("Subject *")):
            adls.extend(sorted((subject / "ADL").glob("*.mp4")))
    crowds = [] if args.quick else sorted(args.crowd.glob("*.mp4"))

    rows = []
    if args.only:
        variants = [v for v in variants
                    if any(v[0].startswith(prefix) for prefix in args.only)]

    for (label, thr, erode, dilate, h_ratio, need_shrunk, shrunk_frac, need_sunk,
         keep_frac, confirm) in variants:
        p = board_params()
        p.motion_threshold = thr
        p.erode_radius = erode
        p.dilate_radius = dilate
        if label.startswith("minref800"):
            p.min_area = 800
            p.min_ref_area = 800
        plike, feed = make_gate(h_ratio, need_shrunk, shrunk_frac,
                                need_sunk, keep_frac, confirm)
        n_demo, alarms = score(args.demo, p, plike, feed)
        row = {"label": label, "demo": n_demo, "alarms": alarms}
        if not args.quick:
            row["crowd"] = sum(1 for c in crowds
                               if score_limited(c, p, plike, feed,
                                                args.crowd_frames)[0])
            row["adl"] = sum(1 for c in adls
                             if score(c, p, plike, feed)[0])
        rows.append(row)
        print(f"{label:<24} demo {n_demo:2d}/10  "
              f"crowd-fp {row.get('crowd', '-')} adl-fp {row.get('adl', '-')} "
              f"alarms={alarms}")
        if args.json:
            args.json.write_text(json.dumps(rows, indent=2), encoding="utf-8")
    if args.json:
        args.json.write_text(json.dumps(rows, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
