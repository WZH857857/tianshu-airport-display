"""Which collapse criterion actually blocks the looped fall demo?

The demo clip is the same 4.3 s fall repeated ten times, so a criterion that
is mis-designed shows up as "true in some loops, false in others" with no
change in the foot geometry.  This probe replays the board parameters over
that clip and, at every observation that is flat and shrunk against its own
reference, prints which of the remaining gates were satisfied.

Usage::

    python tools/criteria_probe.py out/test_videos/fall_test.mp4
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import imageio.v3 as iio

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fall_proto import FallDetector, Params, person_like  # noqa: E402


def board_params() -> Params:
    p = Params()
    p.target_w, p.target_h = 320, 180
    p.motion_threshold = 8
    p.erode_radius = 2
    p.dilate_radius = 2
    p.min_pixels = 4
    p.min_area = 120
    p.min_ref_area = 120
    return p


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("clip", type=Path)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--end", type=int, default=0)
    args = ap.parse_args()
    p = board_params()
    det = FallDetector(p)
    end = args.end or 10 ** 9
    for index, frame in enumerate(iio.imiter(args.clip, plugin="FFMPEG")):
        if index < args.start:
            continue
        if index > end:
            break
        dets = det.update(frame)
        if index % 5:
            continue
        for det_item in dets:
            if det_item.bbox is None:
                continue
            x, y, w, h = det_item.bbox
            # find the machine that produced this detection
            track = next((t for t in det.tracks
                          if t.track_id == det_item.track_id), None)
            if track is None:
                continue
            m = track.machine
            ref = m.h_ref
            if ref is None:
                print(f"f{index:4d} t{det_item.track_id} {w}x{h} "
                      f"ratio={det_item.ratio:.2f} no-ref")
                continue
            obs = det.observe(det_item.bbox, det_item.area)
            tall = ref.h >= p.min_height and person_like(ref, p, 320, 180)
            shrunk = obs.h <= p.collapse_frac * ref.h
            flat = obs.ratio <= p.lying_ratio
            sunk = (obs.cy - ref.cy) >= p.drop_frac * 180
            grounded = obs.bottom >= ref.bottom - p.bottom_tol_frac * ref.h
            keep = obs.area >= p.min_fall_area_frac * ref.area
            fast = obs.frame - m.last_tall <= p.collapse_frames
            tags = "".join(c for c, ok in (
                ("T", tall), ("S", shrunk), ("F", flat), ("U", sunk),
                ("G", grounded), ("K", keep), ("N", fast)) if ok)
            print(f"f{index:4d} t{det_item.track_id} "
                  f"ref={ref.w}x{ref.h}@{ref.frame} cur={w}x{h} "
                  f"area={det_item.area} dy={obs.cy - ref.cy:6.1f} "
                  f"dbot={obs.bottom - ref.bottom:5d} "
                  f"speed={obs.frame - m.last_tall:3d} [{tags}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
