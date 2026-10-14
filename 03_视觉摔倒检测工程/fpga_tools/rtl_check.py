"""Equivalence check: Python PostureMachine vs rtl/posture_fsm.v.

Runs the full Python front end over a clip, records the exact per-track
observation stream that every PostureMachine.feed() call received, then
replays each stream through the Verilog state machine with Icarus Verilog
and compares both the alarm frames and the per-row state trajectory.

Usage::

    python rtl_check.py [--clips clip1.mp4 clip2.mp4 ...]
                        [--max-tracks 4] [--min-rows 8] [--keep]

Exit code 0 means every replayed stream matched the Python reference.
"""

from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import sys
from pathlib import Path

import imageio.v3 as iio

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(ROOT / "tools"))

from fall_proto import FallDetector, Params, PostureMachine  # noqa: E402

RTL = FPGA / "rtl" / "posture_fsm.v"
TB = FPGA / "tb" / "tb_posture_fsm.v"
GEN = FPGA / "sim" / "gen"

STATE_TO_INT = {"SEEK": 0, "PENDING": 1, "FALL": 2, "RECOVERED": 3}

DEFAULT_CLIPS = [
    ROOT / "datasets" / "gmdcsa24" / "Subject 1" / "Fall" / "13.mp4",
    ROOT / "datasets" / "gmdcsa24" / "Subject 3" / "ADL" / "07.mp4",
    ROOT / "datasets" / "gmdcsa24" / "Subject 4" / "Fall" / "02.mp4",
    ROOT / "datasets" / "cctv_fall" / "day6_event019_confirmed.mp4",
]


def find_tool(name: str) -> str:
    found = shutil.which(name)
    if found:
        return found
    # The install directory is not always on PATH (a fresh shell may not have
    # it), so probe the usual spots before giving up.
    for root in (r"E:\iverilog\bin", r"C:\iverilog\bin"):
        cand = Path(root) / f"{name}.exe"
        if cand.exists():
            return str(cand)
    raise SystemExit(f"error: {name} not found (install Icarus Verilog)")


def clip_fps(clip: Path) -> float:
    meta = iio.immeta(clip, plugin="FFMPEG")
    return float(meta.get("fps") or 30.0)


def record_streams(clip: Path, params: Params) -> dict[int, dict]:
    """Run the Python detector once, recording every feed() call per track."""
    streams: dict[int, dict] = {}
    counter = [0]
    orig_init = PostureMachine.__init__
    orig_feed = PostureMachine.feed

    def patched_init(self, p):  # noqa: ANN001
        orig_init(self, p)
        counter[0] += 1
        self._mid = counter[0]
        streams[self._mid] = {"rows": [], "frame_w": 320, "frame_h": 240}

    def patched_feed(self, obs, frame=None):  # noqa: ANN001
        out = orig_feed(self, obs, frame)
        rec = streams[self._mid]
        rec["frame_w"] = self.frame_w
        rec["frame_h"] = self.frame_h
        if obs is None:
            row = (frame, 0, 0, 0, 0, 0, 0)
        else:
            row = (obs.frame, 1, obs.x, obs.y, obs.w, obs.h, obs.area)
        rec["rows"].append(row + (self.confirm_required,
                                  STATE_TO_INT[out[1]], bool(out[0])))
        return out

    PostureMachine.__init__ = patched_init
    PostureMachine.feed = patched_feed
    try:
        detector = FallDetector(params)
        for frame in iio.imiter(clip, plugin="FFMPEG"):
            detector.update(frame)
    finally:
        PostureMachine.__init__ = orig_init
        PostureMachine.feed = orig_feed
    return streams


def stream_params(rec: dict, p: Params) -> dict:
    fw, fh = rec["frame_w"], rec["frame_h"]
    return {
        "MIN_REF_AREA": p.min_ref_area,
        "MIN_REF_HEIGHT": p.min_ref_height,
        "FRAME_H": fh,
        "MAX_REF_W": math.floor(0.55 * fw + 1e-9),
        "MAX_REF_H": math.floor(0.95 * fh + 1e-9),
        "MAX_REF_AREA": math.floor(0.20 * fw * fh + 1e-6),
        "SUNK_CY2": math.ceil(0.08 * fh - 1e-9),
        "REF_TTL": p.ref_ttl,
        "COLLAPSE_FRAMES": p.collapse_frames,
        "RECOVERY_WINDOW": p.recovery_window,
        "COOLDOWN_FRAMES": p.cooldown,
    }


def write_params(path: Path, values: dict) -> None:
    lines = [f"`define {name} {value}" for name, value in values.items()]
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def write_vectors(path: Path, rec: dict) -> None:
    lines = [
        f"{frame} {valid} {x} {y} {w} {h} {area} {confirm}"
        for frame, valid, x, y, w, h, area, confirm, _, _ in rec["rows"]
    ]
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def compile_model(iverilog: str, tag: str) -> Path:
    out = GEN / f"sim_{tag}.vvp"
    cmd = [iverilog, "-g2005", "-I", str(GEN), "-o", str(out),
           str(TB), str(RTL)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"iverilog failed for {tag}:\n{proc.stdout}\n{proc.stderr}")
    return out


def run_model(vvp: str, model: Path, vectors: Path) -> tuple[list[int], list[tuple[int, int]]]:
    # +HOSTILE=1: the bench bumps the frame port the moment `en` drops, one
    # edge before posture_fsm consumes the observation -- the same thing the
    # vision-chain bench does.  The state machine must only look at what it
    # captured on the `en` edge, so the expected trace is unchanged.
    proc = subprocess.run([vvp, str(model), f"+VEC={vectors}", "+HOSTILE=1"],
                          capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"vvp failed:\n{proc.stdout}\n{proc.stderr}")
    alarms: list[int] = []
    states: list[tuple[int, int]] = []
    for line in proc.stdout.splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "ALARM":
            alarms.append(int(parts[1]))
        elif parts[0] == "ROW":
            states.append((int(parts[1]), int(parts[3])))
        elif parts[0] in ("PARSE-ERROR", "ERROR"):
            raise SystemExit(f"testbench error: {line}")
    return alarms, states


def check_stream(vvp: str, tag: str, rec: dict, p: Params) -> tuple[bool, str]:
    values = stream_params(rec, p)
    write_params(GEN / "params.vh", values)
    model = compile_model(find_tool("iverilog"), f"{tag}_{values['FRAME_H']}")
    vectors = GEN / f"vec_{tag}.txt"
    write_vectors(vectors, rec)
    got_alarms, got_states = run_model(vvp, model, vectors)

    rows = rec["rows"]
    want_alarms = [row[0] for row in rows if row[9]]
    want_states = [(row[0], row[8]) for row in rows]

    problems: list[str] = []
    if got_alarms != want_alarms:
        problems.append(f"alarm frames: python={want_alarms} rtl={got_alarms}")
    if len(got_states) != len(want_states):
        problems.append(
            f"row count: python={len(want_states)} rtl={len(got_states)}")
    else:
        for idx, (want, got) in enumerate(zip(want_states, got_states)):
            if want != got:
                problems.append(
                    f"row {idx}: frame {want[0]} state python={want[1]} "
                    f"rtl={got[1]}")
                if len(problems) >= 4:
                    break
    return (not problems), "; ".join(problems)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clips", type=Path, nargs="*", default=DEFAULT_CLIPS)
    parser.add_argument("--max-tracks", type=int, default=4,
                        help="longest-lived tracks replayed per clip")
    parser.add_argument("--min-rows", type=int, default=8,
                        help="skip tracks shorter than this many frames")
    args = parser.parse_args()

    vvp = find_tool("vvp")
    find_tool("iverilog")
    GEN.mkdir(parents=True, exist_ok=True)

    total = 0
    failed = 0
    for clip in args.clips:
        if not clip.exists():
            print(f"SKIP {clip} (missing)")
            continue
        fps = clip_fps(clip)
        params = Params.for_fps(fps)
        streams = record_streams(clip, params)
        ranked = sorted(streams.items(),
                        key=lambda kv: len(kv[1]["rows"]), reverse=True)
        picked = [(mid, rec) for mid, rec in ranked
                  if len(rec["rows"]) >= args.min_rows][: args.max_tracks]
        print(f"\n{clip.name}  fps={fps:g}  tracks={len(streams)}  "
              f"replaying={len(picked)}")
        for mid, rec in picked:
            tag = f"{clip.stem}_t{mid}".replace(" ", "_")
            ok, detail = check_stream(vvp, tag, rec, params)
            total += 1
            if ok:
                flags = sum(1 for row in rec["rows"] if row[9])
                print(f"  PASS track {mid:>3}  rows={len(rec['rows']):>4}  "
                      f"alarms={flags}")
            else:
                failed += 1
                print(f"  FAIL track {mid:>3}  {detail}")

    print(f"\n{total - failed}/{total} streams matched the Python reference")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
