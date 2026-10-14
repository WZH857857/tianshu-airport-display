"""End-to-end check of the RTL fall-detection chain vs the prototype.

Streams RGB565 frames through

    gray_front -> frame_diff -> motion_morph -> proj_bbox -> posture_fsm

in Icarus Verilog and compares the per-frame bbox result (found x y w h area)
plus the posture state trajectory and alarm frames against the reference
chain that mirrors every RTL operation::

    quantise565 -> 77/150/29 luma -> 2x2 box
                -> |cur - prev| > 12
                -> erosion(disk(1)) -> dilation(disk(3)), border 0
                -> two-pass projection bbox (largest_span, MIN_PIX/MIN_AREA)
                -> PostureMachine (one observation per frame, gaps included)

Frame 0 has no predecessor, so its mask is empty and its bbox is all zeros,
matching the offline prototype.

Usage::

    python chain_check.py [--clips a.mp4 b.mp4] [--frames 3]
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from pathlib import Path

import imageio.v3 as iio
import numpy as np

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(ROOT / "tools"))

from bbox_check import bbox_ref  # noqa: E402
from fall_proto import Obs, Params, PostureMachine  # noqa: E402
from morph_check import (  # noqa: E402
    GEN, H, KEEP_ROWS, W, diagnose, find_tool, morph_ref, pack_masks,
    unpack_masks,
)
from pixel_check import down2_box, luma_from_565, quantise565  # noqa: E402

RTL = [
    FPGA / "rtl" / "gray_front.v",
    FPGA / "rtl" / "frame_diff.v",
    FPGA / "rtl" / "motion_morph.v",
    FPGA / "rtl" / "proj_bbox.v",
    FPGA / "rtl" / "posture_fsm.v",
]
TB = FPGA / "tb" / "tb_vision_chain.v"

# Must match rtl/fall_camera_hdmi_multi_top.v (the board build) and the branch
# the testbench instantiates, otherwise the check proves a different pipeline.
DIFF_THRESH = 12

# The camera build is a fixed 320x180 / 30 fps path (the 3-of-4 row gate in
# gray_front keeps the front end inside the device's on-chip ERAM budget), so
# the posture constants are the Params defaults with the 30 fps frame counts.
CAMERA_W = 320
CAMERA_H = 180
CAMERA_FPS = 30.0
SOLO_CONFIRM = 2

# Which of the testbench's two candidate sampling edges drives the ROW/ALARM
# lines compared below (see tb_vision_chain.v).  The other edge is still
# printed as DBG<n> and reported on a row mismatch, so a wrong number of
# settle clocks can never be mistaken for a state-machine difference.
SAMPLE_RULE = 1

STATE_TO_INT = {"SEEK": 0, "PENDING": 1, "FALL": 2, "RECOVERED": 3}
PARAMS_HEADER = GEN / "params_chain.vh"

DEFAULT_CLIPS = [
    ROOT / "datasets" / "gmdcsa24" / "Subject 1" / "Fall" / "13.mp4",
    ROOT / "datasets" / "gmdcsa24" / "Subject 3" / "ADL" / "07.mp4",
]


def load565(clip: Path, nframes: int) -> np.ndarray | None:
    """Video -> normalised 640x480 RGB888 frames -> packed RGB565."""
    raw = []
    for i, frame in enumerate(iio.imiter(clip, plugin="FFMPEG")):
        if i >= nframes:
            break
        frame = np.asarray(frame)
        h, w = frame.shape[:2]
        if (w, h) != (640, 480):
            ys = np.minimum(np.arange(480) * h // 480, h - 1)
            xs = np.minimum(np.arange(640) * w // 640, w - 1)
            frame = frame[ys][:, xs]
        raw.append(frame)
    if not raw:
        return None
    return np.stack(raw)


def posture_params() -> Params:
    return Params.for_fps(CAMERA_FPS)


def write_params_header(path: Path, p: Params) -> None:
    """Mirror the Python posture constants into the testbench parameters."""
    values = {
        "CHAIN_MIN_REF_AREA":     p.min_ref_area,
        "CHAIN_MIN_REF_HEIGHT":   p.min_ref_height,
        "CHAIN_FRAME_H":          CAMERA_H,
        "CHAIN_MAX_REF_W":        math.floor(0.55 * CAMERA_W + 1e-9),
        "CHAIN_MAX_REF_H":        math.floor(0.95 * CAMERA_H + 1e-9),
        "CHAIN_MAX_REF_AREA":     math.floor(0.20 * CAMERA_W * CAMERA_H + 1e-6),
        "CHAIN_SUNK_CY2":         math.ceil(0.08 * CAMERA_H - 1e-9),
        "CHAIN_REF_TTL":          p.ref_ttl,
        "CHAIN_COLLAPSE_FRAMES":  p.collapse_frames,
        "CHAIN_RECOVERY_WINDOW":  p.recovery_window,
        "CHAIN_COOLDOWN_FRAMES":  p.cooldown,
    }
    lines = [f"`define {name} {value}" for name, value in values.items()]
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def posture_ref(boxes: list[tuple[int, ...]],
                p: Params) -> tuple[list[tuple[int, int, int]], list[int]]:
    """Reference state trajectory and alarm frames, one feed per frame."""
    machine = PostureMachine(p)
    # person_like() derives its two fraction gates from the frame size, so the
    # reference has to be told the working resolution.  Leaving it at the
    # 320x240 default made MAX_REF_H / MAX_REF_AREA 228 / 15360 against the
    # testbench's 171 / 11520: two different configurations compared as if
    # they were one.
    machine.frame_w = CAMERA_W
    machine.frame_h = CAMERA_H
    machine.confirm_required = SOLO_CONFIRM
    rows: list[tuple[int, int, int]] = []
    alarms: list[int] = []
    for frame, (found, x, y, w, h, area) in enumerate(boxes, start=1):
        obs = None
        if found:
            obs = Obs(frame, (x, y, w, h), area,
                      h / float(w) if w else 0.0,
                      x + (w - 1) / 2.0, y + (h - 1) / 2.0)
        flagged, state, _ = machine.feed(obs, frame)
        rows.append((frame, 1 if found else 0, STATE_TO_INT[state]))
        if flagged:
            alarms.append(frame)
    return rows, alarms


def chain_ref(frames: np.ndarray) -> tuple[np.ndarray, np.ndarray, list[tuple[int, ...]]]:
    """Reference chain: raw masks, morphed masks, bbox tuples per frame."""
    # Mirror gray_front's post-box row gate: rows 3,7,11,... are dropped.
    luma = np.stack([down2_box(luma_from_565(quantise565(f)))[
                        KEEP_ROWS]
                     for f in frames])
    masks = np.zeros_like(luma, dtype=bool)
    for i in range(1, len(luma)):
        diff = np.abs(luma[i].astype(np.int16) - luma[i - 1].astype(np.int16))
        masks[i] = diff > DIFF_THRESH
    morphed = np.stack([morph_ref(m) for m in masks])
    return masks, morphed, [bbox_ref(m) for m in morphed]


def run_rtl(pix: np.ndarray, tag: str, vvp: str) -> dict:
    n = len(pix)
    in_path = GEN / f"chain_{tag}.565"
    out_path = GEN / f"chain_{tag}.out"
    morph_path = GEN / f"chain_{tag}.morph.bin"
    diff_path = GEN / f"chain_{tag}.diff.bin"
    in_path.write_bytes(pix.astype("<u2").tobytes())

    model = GEN / "sim_vision_chain.vvp"
    cmd = [find_tool("iverilog"), "-g2005", "-I", str(GEN), "-o", str(model),
           *[str(p) for p in RTL], str(TB)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"iverilog failed:\n{proc.stdout}\n{proc.stderr}")

    proc = subprocess.run(
        [vvp, str(model), f"+IN={in_path}", f"+OUT={out_path}",
         f"+MORPH={morph_path}", f"+DIFF={diff_path}", f"+FRAMES={n}",
         f"+SAMPLE={SAMPLE_RULE}"],
        capture_output=True, text=True)
    if "DONE" not in proc.stdout:
        raise SystemExit(f"simulation failed:\n{proc.stdout}\n{proc.stderr}")

    rows = []
    for line in out_path.read_text().splitlines():
        if line.strip():
            rows.append(tuple(int(v) for v in line.split()))

    rtl_rows: list[tuple[int, int, int]] = []
    alarms: list[int] = []
    dbg: dict[str, list[tuple[int, int, int, int]]] = {}
    for line in proc.stdout.splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "ROW":
            rtl_rows.append((int(parts[1]), int(parts[2]), int(parts[3])))
        elif parts[0] == "ALARM":
            alarms.append(int(parts[1]))
        elif parts[0].startswith("DBG"):
            dbg.setdefault(parts[0][3:], []).append(
                tuple(int(v) for v in parts[1:]))

    return {
        "bbox": rows,
        "morph": morph_path.read_bytes(),
        "diff": diff_path.read_bytes(),
        "rows": rtl_rows,
        "alarms": alarms,
        "dbg": dbg,
        "stdout": proc.stdout,
    }


def check_clip(vvp: str, clip: Path, nframes: int) -> bool:
    frames = load565(clip, nframes)
    if frames is None:
        print(f"SKIP {clip.name}: no frames")
        return True
    n = len(frames)
    if n < 2:
        print(f"SKIP {clip.name}: single frame")
        return True

    pix = np.stack([quantise565(f) for f in frames])
    want_mask, want_morph, want_box = chain_ref(frames)
    params = posture_params()
    want_rows, want_alarms = posture_ref(want_box, params)
    tag = (clip.stem.replace(" ", "_")[:20] + f"_{n}f")
    got = run_rtl(pix, tag, vvp)

    box_ok = (len(got["bbox"]) == len(want_box) and
              all(g == w for g, w in zip(got["bbox"], want_box)))
    morph_ok = got["morph"] == pack_masks(want_morph)
    rows_ok = got["rows"] == want_rows
    alarms_ok = got["alarms"] == want_alarms
    ok = box_ok and morph_ok and rows_ok and alarms_ok
    status = "PASS" if ok else "FAIL"
    print(f"{status} {clip.name}: {n} frames, {len(got['bbox'])} bbox results, "
          f"morph {'match' if morph_ok else 'MISMATCH'}, "
          f"posture {'match' if rows_ok else 'MISMATCH'}, "
          f"alarms rtl={got['alarms']} ref={want_alarms}")
    if not box_ok:
        for i, (g, w) in enumerate(zip(got["bbox"], want_box)):
            if g != w:
                print(f"     frame {i}: got {g} want {w}")
        if len(got["bbox"]) != len(want_box):
            print(f"     result count: got {len(got['bbox'])} "
                  f"want {len(want_box)}")
    if not morph_ok:
        print("     morph: " + diagnose(unpack_masks(got["morph"]),
                                        want_morph.reshape(-1, W)))
    if got["diff"] != pack_masks(want_mask):
        print("     diff : " + diagnose(unpack_masks(got["diff"]),
                                        want_mask.reshape(-1, W)))
    if not rows_ok:
        for i, (g, w) in enumerate(zip(got["rows"], want_rows)):
            if g != w:
                other = " ".join(
                    f"DBG{rule}(frame {d[0]}) state={d[2]} alarm={d[3]}"
                    for rule, rows in sorted(got["dbg"].items())
                    for d in rows if d[0] == w[0])
                print(f"     row {i}: got {g} want {w}"
                      + (f"   [{other}]" if other else ""))
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clips", type=Path, nargs="*", default=DEFAULT_CLIPS)
    parser.add_argument("--frames", type=int, default=3)
    args = parser.parse_args()

    vvp = find_tool("vvp")
    GEN.mkdir(parents=True, exist_ok=True)
    write_params_header(PARAMS_HEADER, posture_params())
    ok = True
    for clip in args.clips:
        if not clip.exists():
            print(f"SKIP missing {clip}")
            continue
        ok &= check_clip(vvp, clip, args.frames)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
