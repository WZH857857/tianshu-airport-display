"""Stage-by-stage probe for the seven GMDCSA24 fall clips the board misses.

The parameter sweep showed that no single posture threshold changes the
8/15 recall, which means the loss is upstream of the state machine.  This
script replicates ``FallDetector.update`` step by step and reports, per clip:

  mask    frames with any motion mask at all
  blob    frames whose strongest blob survives min_area / min_height
  plike   frames where some blob passes the standing person-likeness gate
  tallref frames where a track actually adopts an upright reference
  near    frames where the observation is within the collapse geometry of
          its own reference (shrunk + flat), i.e. the fall was seen at all

Board parameters are the ones compiled into the PNR build:
320x180, DIFF=8, MIN_PIX=4, MIN_AREA=160, erode 2, dilate 2.

Usage::

    python tools/fall_stage_probe.py [--root datasets/gmdcsa24]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import imageio.v3 as iio
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
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


def probe(path: Path, p: Params) -> dict:
    det = FallDetector(p)
    stats = {
        "clip": path.name,
        "frames": 0,
        "mask": 0,
        "blob": 0,
        "plike": 0,
        "tallref": 0,
        "near": 0,
        "max_area": 0,
        "max_h": 0,
        "max_ratio": 0.0,
        "min_ratio_seen": 99.0,
        "best": "",
        "alarms": [],
    }
    for index, frame in enumerate(iio.imiter(path, plugin="FFMPEG")):
        det.frame_index += 1
        gray = det.prepare(frame)
        mask = det.motion_mask(gray)
        det.mask_cache = mask
        frame_h, frame_w = gray.shape
        if mask.any():
            stats["mask"] += 1
            blobs = det.blobs_of(mask)
            # Same hybrid front end as FallDetector.update.
            def spans_raster(bbox) -> bool:
                return (bbox[2] >= frame_w - 8) or (bbox[3] >= frame_h - 8)
            blobs = [b for b in blobs if not spans_raster(b[0])]
            candidates = [det.observe(b, a) for b, a in blobs]
            plike_count = sum(
                1 for o in candidates
                if person_like(o, p, frame_w, frame_h))
            if plike_count < p.crowd_min_targets:
                bbox, area = det.bbox_of(mask)
                blobs = ([(bbox, area)]
                         if bbox is not None and not spans_raster(bbox)
                         else [])
            obs_list = [det.observe(b, a) for b, a in blobs]
        else:
            obs_list = []
        if obs_list:
            stats["blob"] += 1
        # ---- replicate _step_tracks without consuming machine state twice
        pairs = []
        for oi, obs in enumerate(obs_list):
            for ti, track in enumerate(det.tracks):
                cost = det._match_cost(track, obs)
                if cost is not None:
                    pairs.append((cost, oi, ti))
        pairs.sort(key=lambda item: item[0])
        obs_to_track: dict[int, int] = {}
        used_tracks: set[int] = set()
        for _, oi, ti in pairs:
            if oi in obs_to_track or ti in used_tracks:
                continue
            obs_to_track[oi] = ti
            used_tracks.add(ti)
        plike_count = sum(
            1 for o in obs_list
            if person_like(o, p, frame_w, frame_h))
        required = (p.crowd_confirm_frames
                    if plike_count >= p.crowd_min_targets
                    else p.confirm_frames)
        for ti, track in enumerate(det.tracks):
            if ti not in used_tracks:
                track.miss += 1
                track.machine.feed(None, det.frame_index)
        for oi, obs in enumerate(obs_list):
            ti = obs_to_track.get(oi)
            track = det.tracks[ti] if ti is not None else det._new_track(
                frame_w, frame_h)
            track.last_bbox = obs.bbox
            track.last_frame = obs.frame
            track.last_area = obs.area
            track.miss = 0
            track.hits += 1
            track.machine.confirm_required = required
            m = track.machine
            # reference that this feed() call will compare against
            ref = m.h_ref
            if person_like(obs, p, frame_w, frame_h):
                stats["plike"] += 1
                if m.h_ref is None or obs.h > m.h_ref.h:
                    stats["tallref"] += 1
                    stats["best"] = (f"f{det.frame_index} {obs.w}x{obs.h} "
                                     f"area={obs.area} "
                                     f"fill={obs.fill:.2f}")
            if ref is not None and obs.h <= p.collapse_frac * ref.h \
                    and obs.ratio <= p.lying_ratio:
                stats["near"] += 1
            stats["max_area"] = max(stats["max_area"], obs.area)
            stats["max_h"] = max(stats["max_h"], obs.h)
            stats["max_ratio"] = max(stats["max_ratio"], obs.ratio)
            stats["min_ratio_seen"] = min(stats["min_ratio_seen"], obs.ratio)
            flagged, _, _ = m.feed(obs, obs.frame)
            if flagged:
                stats["alarms"].append(det.frame_index)
        if det.tracks:
            det.tracks = [t for t in det.tracks if t.miss <= p.track_ttl]
        stats["frames"] += 1
    return stats


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path,
                        default=Path(__file__).resolve().parents[1]
                        / "datasets" / "gmdcsa24")
    parser.add_argument("--target-h", type=int, default=180)
    args = parser.parse_args()
    p = board_params()
    p.target_h = args.target_h
    clips = sorted(args.root.glob("Subject */Fall/*.mp4"))
    for clip in clips:
        s = probe(clip, p)
        verdict = "HIT " if s["alarms"] else "MISS"
        print(f"{verdict} {clip.parent.parent.name}/{clip.name:<8} "
              f"frames={s['frames']:4d} mask={s['mask']:4d} "
              f"blob={s['blob']:4d} plike={s['plike']:4d} "
              f"ref={s['tallref']:3d} near={s['near']:3d} "
              f"amax={s['max_area']:6d} hmax={s['max_h']:3d} "
              f"rmin={s['min_ratio_seen']:.2f} "
              f"best=({s['best']}) alarm={s['alarms'][:3]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
