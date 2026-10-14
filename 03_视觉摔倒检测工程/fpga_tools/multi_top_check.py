"""End-to-end check of fall_core_multi_top.v (the crowd path).

Streams RGB565 frames through the real top module in Icarus Verilog

    gray_front -> frame_diff -> motion_morph
        -> multi_bbox (4 exclusion passes) -> track_assoc -> posture_bank

and compares, for every frame and every slot,

  * the published box (found x y w h area) against ``multi_model.frames_model``
    run on the reference morph streams -- the same bit-exact equality that
    ``multi_bbox_check.py`` pins down, now with the full front end in front of
    it, and
  * the four posture machines' state trajectory and alarm frames against the
    Python ``PostureMachine`` replay used by ``multi_rtl_check.py``, and
  * motion_hist's whole-person boxes against ``hist_model.run`` -- the stick
    figure overlay's model.  The module publishes one frame behind the
    tracker, so RTL publish k must equal model row k.

Unlike ``multi_rtl_check.py`` this does not re-wire the stages in the bench:
the DUT is the top module that the board variant will instantiate.

Usage::

    python multi_top_check.py [--clips a.mp4 ...] [--frames 8]
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(ROOT / "tools"))

from fall_proto import Obs, Params, PostureMachine  # noqa: E402
from morph_check import (  # noqa: E402
    GEN, KEEP_ROWS, diagnose, find_tool, morph_ref, pack_masks, unpack_masks,
)
from multi_model import frames_model  # noqa: E402
from hist_model import CH as HIST_CH, CW as HIST_CW, run as hist_run  # noqa: E402
from pixel_check import down2_box, luma_from_565, quantise565  # noqa: E402
from rtl_check import STATE_TO_INT  # noqa: E402
from track_model import track_slots  # noqa: E402

RTL = [
    FPGA / "rtl" / "gray_front.v",
    FPGA / "rtl" / "frame_diff.v",
    FPGA / "rtl" / "motion_morph.v",
    FPGA / "rtl" / "multi_bbox.v",
    FPGA / "rtl" / "track_assoc.v",
    FPGA / "rtl" / "person_score.v",
    FPGA / "rtl" / "motion_hist.v",
    FPGA / "rtl" / "posture_fsm.v",
    FPGA / "rtl" / "posture_bank.v",
    FPGA / "rtl" / "fall_core_multi_top.v",
    FPGA / "tb" / "eg_phy_bram_stub.v",
]
TB = FPGA / "tb" / "tb_multi_top.v"

NUM_BLOBS = 4
# Must match rtl/fall_core_multi_top.v / fall_camera_hdmi_multi_top.v and the
# branch tb_multi_top instantiates, otherwise the check proves a different
# pipeline than the one that ships.
DIFF_THRESH = 12
CAMERA_W, CAMERA_H = 320, 180
CAMERA_FPS = 30.0
CROWD_CONFIRM = 2

DEFAULT_CLIPS = [
    ROOT / "datasets" / "crowd" / "crowd01.mp4",
    ROOT / "datasets" / "crowd" / "crowd05.mp4",
    ROOT / "datasets" / "gmdcsa24" / "Subject 1" / "Fall" / "13.mp4",
]


def load565(clip: Path, nframes: int) -> np.ndarray | None:
    import imageio.v3 as iio

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
    if len(raw) < 2:
        return None
    return np.stack(raw)


def chain_ref(frames: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Raw and morphed masks, mirroring the RTL front end."""
    luma = np.stack([down2_box(luma_from_565(quantise565(f)))[KEEP_ROWS]
                     for f in frames])
    masks = np.zeros_like(luma, dtype=bool)
    for i in range(1, len(luma)):
        diff = np.abs(luma[i].astype(np.int16) - luma[i - 1].astype(np.int16))
        masks[i] = diff > DIFF_THRESH
    morphed = np.stack([morph_ref(m) for m in masks])
    return masks, morphed


def cell_motion(morph: np.ndarray) -> np.ndarray:
    """Per-cell motion pixel counts (the ``cm`` of hist_model.run)."""
    n = morph.shape[0]
    return morph.reshape(n, HIST_CH, 4, HIST_CW, 4).sum(axis=(2, 4))


def pbox_row(entry) -> tuple:
    """Normalise a model row entry to the RTL's PBOX tuple."""
    if entry is None:
        return (0, 0, 0, 0, 0, 0)
    x, y, w, h, c = (int(v) for v in entry)
    return (1, x, y, w, h, c)


def posture_ref(slots, p: Params) -> tuple[list, list]:
    """Per-slot reference rows and alarm frames (same replay as rtl_check)."""
    rows = []          # rows[slot] = [(found, state, alarm), ...] per frame
    alarms = []        # alarms[slot] = [frame, ...]
    for k in range(NUM_BLOBS):
        machine = PostureMachine(p)
        machine.frame_w = CAMERA_W
        machine.frame_h = CAMERA_H
        machine.confirm_required = CROWD_CONFIRM
        slot_rows = []
        slot_alarms = []
        for frame, frame_slots in enumerate(slots, start=1):
            found, x, y, w, h, area = frame_slots[k]
            obs = None
            if found:
                obs = Obs(frame, (x, y, w, h), area,
                          h / float(w) if w else 0.0,
                          x + (w - 1) / 2.0, y + (h - 1) / 2.0)
            flagged, state, _ = machine.feed(obs, frame)
            slot_rows.append((1 if found else 0, STATE_TO_INT[state],
                              bool(flagged)))
            if flagged:
                slot_alarms.append(frame)
        rows.append(slot_rows)
        alarms.append(slot_alarms)
    return rows, alarms


def run_rtl(pix: np.ndarray, tag: str, vvp: str) -> dict:
    n = len(pix)
    in_path = GEN / f"multitop_{tag}.565"
    out_path = GEN / f"multitop_{tag}.out"
    morph_path = GEN / f"multitop_{tag}.morph.bin"
    diff_path = GEN / f"multitop_{tag}.diff.bin"
    in_path.write_bytes(pix.astype("<u2").tobytes())

    model = GEN / "sim_multi_top.vvp"
    cmd = [find_tool("iverilog"), "-g2005", "-o", str(model),
           *[str(p) for p in RTL], str(TB)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"iverilog failed:\n{proc.stdout}\n{proc.stderr}")

    proc = subprocess.run(
        [vvp, str(model), f"+IN={in_path}", f"+OUT={out_path}",
         f"+MORPH={morph_path}", f"+DIFF={diff_path}", f"+FRAMES={n}"],
        capture_output=True, text=True)
    if "DONE" not in proc.stdout:
        raise SystemExit(f"simulation failed:\n{proc.stdout}\n{proc.stderr}")

    boxes = []
    for line in out_path.read_text().splitlines():
        if line.strip():
            boxes.append(tuple(int(v) for v in line.split()))

    rows = [[] for _ in range(NUM_BLOBS)]
    alarms = [[] for _ in range(NUM_BLOBS)]
    dbg2 = [[] for _ in range(NUM_BLOBS)]
    pbox = {}                      # publish index -> list of four tuples
    for line in proc.stdout.splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "ROW":
            k = int(parts[1])
            rows[k].append((int(parts[2]), int(parts[3]), int(parts[4])))
        elif parts[0] == "ALARM":
            alarms[int(parts[1])].append(int(parts[2]))
        elif parts[0] == "DBG2":
            k = int(parts[1])
            dbg2[k].append((int(parts[2]), int(parts[3]), int(parts[4])))
        elif parts[0] == "PBOX":
            pub, k = int(parts[1]), int(parts[2])
            pbox.setdefault(pub, [None] * NUM_BLOBS)[k] = tuple(
                int(v) for v in parts[3:])

    return {
        "boxes": boxes,
        "rows": rows,
        "alarms": alarms,
        "dbg2": dbg2,
        "pbox": [pbox[i] for i in sorted(pbox)],
        "morph": morph_path.read_bytes(),
        "diff": diff_path.read_bytes(),
        "stdout": proc.stdout,
    }


def check_clip(vvp: str, clip: Path, nframes: int) -> bool:
    frames = load565(clip, nframes)
    if frames is None:
        print(f"SKIP {clip.name}: not enough frames")
        return True
    n = len(frames)
    pix = np.stack([quantise565(f) for f in frames])
    want_mask, want_morph = chain_ref(frames)
    want_slots = track_slots(frames_model(want_morph))
    p = Params.for_fps(CAMERA_FPS)
    want_rows, want_alarms = posture_ref(want_slots, p)
    want_pbox, _ = hist_run(cell_motion(want_morph), want_slots)

    tag = clip.stem.replace(" ", "_")[:20] + f"_{n}f"
    got = run_rtl(pix, tag, vvp)

    want_flat = [t for frame_slots in want_slots for t in frame_slots]
    box_ok = got["boxes"] == want_flat
    morph_ok = got["morph"] == pack_masks(want_morph)
    diff_ok = got["diff"] == pack_masks(want_mask)
    want_rows_k = [[(f, r[0], r[1]) for f, r in zip(range(1, n + 1), rows)]
                   for rows in want_rows]
    rows_ok = all(got["rows"][k] == want_rows_k[k] for k in range(NUM_BLOBS))
    alarm_ok = all(got["alarms"][k] == want_alarms[k]
                   for k in range(NUM_BLOBS))

    # motion_hist publishes one frame behind the tracker: RTL publish k must
    # equal model row k (row 0 is the empty prologue and is never published).
    npub = len(want_pbox) - 1
    want_pubs = [[pbox_row(e) for e in row] for row in want_pbox[1:]]
    got_pubs = [list(row) for row in got["pbox"][:npub]]
    pbox_ok = (len(got["pbox"]) >= npub) and (got_pubs == want_pubs)

    ok = box_ok and morph_ok and diff_ok and rows_ok and alarm_ok and pbox_ok
    print(f"{'PASS' if ok else 'FAIL'} {clip.name}: {n} frames, "
          f"{len(got['boxes'])} box lines "
          f"({len(got['boxes']) // NUM_BLOBS} frames x {NUM_BLOBS} slots), "
          f"morph {'match' if morph_ok else 'MISMATCH'}, "
          f"diff {'match' if diff_ok else 'MISMATCH'}, "
          f"posture {'match' if rows_ok else 'MISMATCH'}, "
          f"pbox {len(got_pubs)}/{npub} "
          f"{'match' if pbox_ok else 'MISMATCH'}")
    if not box_ok:
        shown = 0
        for i, (g, w) in enumerate(zip(got["boxes"], want_flat)):
            if g != w and shown < 6:
                print(f"     box {i // NUM_BLOBS + 1} slot {i % NUM_BLOBS}: "
                      f"got {g} want {w}")
                shown += 1
        if len(got["boxes"]) != len(want_flat):
            print(f"     box count: got {len(got['boxes'])} "
                  f"want {len(want_flat)}")
    if not morph_ok:
        print("     morph: " + diagnose(unpack_masks(got["morph"]),
                                        want_morph.reshape(-1, CAMERA_W)))
    if not diff_ok:
        print("     diff : " + diagnose(unpack_masks(got["diff"]),
                                        want_mask.reshape(-1, CAMERA_W)))
    if not rows_ok:
        for k in range(NUM_BLOBS):
            for i, (g, w) in enumerate(zip(got["rows"][k], want_rows_k[k])):
                if g != w:
                    print(f"     slot {k} frame {i + 1}: got {g} want {w}"
                          f"   [rule2 {got['dbg2'][k][i]}]")
                    break
    if not alarm_ok:
        for k in range(NUM_BLOBS):
            print(f"     slot {k} alarms: "
                  f"rtl={got['alarms'][k]} ref={want_alarms[k]}")
    if not pbox_ok:
        if len(got["pbox"]) < npub:
            print(f"     pbox publishes: got {len(got['pbox'])} want {npub}")
        for j, (g, w) in enumerate(zip(got_pubs, want_pubs)):
            if g != w:
                for k in range(NUM_BLOBS):
                    if g[k] != w[k]:
                        print(f"     pbox publish {j + 1} slot {k}: "
                              f"got {g[k]} want {w[k]}")
                break
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--clips", type=Path, nargs="*", default=DEFAULT_CLIPS)
    ap.add_argument("--frames", type=int, default=8)
    args = ap.parse_args()

    vvp = find_tool("vvp")
    GEN.mkdir(parents=True, exist_ok=True)
    ok = True
    for clip in args.clips:
        if not clip.exists():
            print(f"SKIP missing {clip}")
            continue
        ok &= check_clip(vvp, clip, args.frames)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
