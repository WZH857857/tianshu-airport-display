"""Crowd-path equivalence check: 4 slots through 4 posture machines.

``rtl_check.py`` replayed the detector's per-track streams through one
``posture_fsm``.  This does the same for the crowd path: the front end runs
over a clip, ``multi_bbox``'s four slots become four observation streams
(slot k of frame f is that slot's observation at frame f, valid=0 when the
slot came up empty), and each stream is replayed through the same RTL state
machine used by ``fall_core_multi_top.v``.

The point of the check is slot ordering: ``track_assoc`` keeps handing the
same person to the same slot, so each slot becomes a stable observation
stream.  The replay still runs through the RTL posture machine.

Usage::

    python multi_rtl_check.py [--clips a.mp4 ...] [--frames 60]
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

from fall_proto import Params, PostureMachine  # noqa: E402
from morph_check import masks_from_clip, morph_ref  # noqa: E402
from multi_model import frames_model  # noqa: E402
from track_model import track_slots  # noqa: E402
from rtl_check import (  # noqa: E402
    GEN, STATE_TO_INT, check_stream, find_tool, stream_params, write_params,
)

NUM_BLOBS = 4


def slot_streams(clip: Path, nframes: int) -> list[dict]:
    masks = masks_from_clip(clip, nframes)
    if masks is None:
        return []
    morphed = np.stack([morph_ref(m) for m in masks])
    slots = track_slots(frames_model(morphed))
    streams = []
    for k in range(NUM_BLOBS):
        rec = {"rows": [], "frame_w": 320, "frame_h": 180}
        for f, frame_slots in enumerate(slots):
            found, x, y, w, h, area = frame_slots[k]
            rec["rows"].append((f, found, x, y, w, h, area, 2, 0, False))
        streams.append(rec)
    return streams


def label_streams(streams: list[dict], p: Params) -> None:
    """Run each slot's observations through the Python state machine."""
    for rec in streams:
        machine = PostureMachine(p)
        machine.frame_w = rec["frame_w"]
        machine.frame_h = rec["frame_h"]
        for i, row in enumerate(rec["rows"]):
            frame, found, x, y, w, h, area = row[:7]
            obs = None
            if found:
                from fall_proto import Obs
                obs = Obs(frame, (x, y, w, h), area,
                          h / float(w) if w else 0.0,
                          x + (w - 1) / 2.0, y + (h - 1) / 2.0)
            alarm, state, _ = machine.feed(obs, frame)
            rec["rows"][i] = row[:7] + (2, STATE_TO_INT[state], bool(alarm))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--clips", type=Path, nargs="*", default=[])
    ap.add_argument("--frames", type=int, default=60)
    args = ap.parse_args()
    if not args.clips:
        print("no clips given")
        return 1

    p = Params.for_fps(30)
    vvp = find_tool("vvp")
    GEN.mkdir(parents=True, exist_ok=True)
    ok_all = True
    for clip in args.clips:
        if not clip.exists():
            print(f"SKIP missing {clip}")
            continue
        streams = slot_streams(clip, args.frames)
        if not streams:
            print(f"SKIP {clip.name}: not enough frames")
            continue
        label_streams(streams, p)
        tag_base = clip.stem.replace(" ", "_")[:20]
        oks = []
        for k, rec in enumerate(streams):
            tag = f"multi_{tag_base}_s{k}"
            write_params(GEN / "params.vh", stream_params(rec, p))
            good, msg = check_stream(vvp, tag, rec, p)
            oks.append(good)
            if not good:
                print(f"FAIL {clip.name} slot {k}: {msg}")
        ok_all &= all(oks)
        print(f"{'PASS' if all(oks) else 'FAIL'} {clip.name}: "
              f"4 slots x {len(streams[0]['rows'])} frames")
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
