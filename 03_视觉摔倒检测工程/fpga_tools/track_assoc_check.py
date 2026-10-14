"""Unit check for track_assoc.v against track_model.py.

The board path runs track_assoc between multi_bbox and posture_bank.  This
driver keeps the tracker in isolation so association bugs show up without a
full 640x480 video replay.

Usage::

    python track_assoc_check.py
"""

from __future__ import annotations

import subprocess
from pathlib import Path

from morph_check import GEN, find_tool
from person_roi import buckets, score
from track_model import GRACE_FRAMES, HOLD_FRAMES, track_slots

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
RTL = FPGA / "rtl" / "track_assoc.v"
PERSON_RTL = FPGA / "rtl" / "person_score.v"
TB = FPGA / "tb" / "tb_track_assoc.v"
NB = 4


def zbox(x: int = 0, y: int = 0, w: int = 0, h: int = 0,
         area: int | None = None) -> tuple[int, ...]:
    if area is None:
        area = w * h
    return (1 if w and h else 0, x, y, w, h, area)


def empty_slots() -> list[tuple[int, ...]]:
    return [(0, 0, 0, 0, 0, 0)] * NB


def cases() -> list[tuple[str, list[list[tuple[int, ...]]]]]:
    out: list[tuple[str, list[list[tuple[int, ...]]]]] = []

    out.append(("empty", [empty_slots(), empty_slots()]))

    out.append(("swap", [
        [zbox(20, 40, 30, 80), zbox(180, 40, 30, 80),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
        [zbox(180, 40, 30, 80), zbox(20, 40, 30, 80),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
    ]))

    out.append(("cross", [
        [zbox(30, 50, 28, 70), zbox(150, 50, 28, 70),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
        [zbox(160, 50, 28, 70), zbox(40, 50, 28, 70),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
        [zbox(50, 50, 28, 70), zbox(170, 50, 28, 70),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
    ]))

    out.append(("drop_one", [
        [zbox(20, 40, 30, 80), zbox(180, 40, 30, 80),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
        [zbox(182, 42, 30, 80), (0, 0, 0, 0, 0, 0),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
    ]))

    out.append(("new_box", [
        [zbox(20, 40, 30, 80), (0, 0, 0, 0, 0, 0),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
        [zbox(22, 42, 30, 80), zbox(180, 40, 30, 80),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
    ]))

    # Lock-on hold: a spawned box survives HOLD_FRAMES empty frames and is
    # released on the next one.
    hold_seq = [
        [zbox(20, 40, 30, 80), (0, 0, 0, 0, 0, 0),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
    ]
    hold_seq.extend(empty_slots() for _ in range(HOLD_FRAMES + 2))
    out.append(("hold_release", hold_seq))

    # A held slot is not free: a new detection goes to the lowest free slot
    # instead of stealing a box that is being held.
    out.append(("hold_protect", [
        [zbox(20, 40, 30, 80), (0, 0, 0, 0, 0, 0),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
        empty_slots(),
        [zbox(200, 40, 30, 80), (0, 0, 0, 0, 0, 0),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
    ]))

    # A one-frame flicker blob never earns ``est``, so it survives exactly the
    # grace window and then dies: long enough to bridge a frame-difference
    # hole on a real person, far too short to become the ten-second ghost the
    # wall lamp in the fall clip used to leave on screen.
    out.append(("flicker_grace", [
        [zbox(150, 20, 12, 14), (0, 0, 0, 0, 0, 0),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
        *[empty_slots() for _ in range(GRACE_FRAMES + 2)],
    ]))

    # The person leaves the frame next to a second, live track: the held box
    # of the first person is an old pose beside that live track, so the slot
    # is retired instead of publishing a duplicate box.
    out.append(("retire_dup", [
        [zbox(20, 40, 60, 120), zbox(100, 40, 20, 30),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
        [zbox(20, 40, 60, 120), zbox(95, 40, 20, 30),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
        [(0, 0, 0, 0, 0, 0), zbox(95, 40, 20, 30),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
    ]))

    # A detection too far away to overlap the held box, but close enough to
    # be the same person, adopts that slot instead of spawning: the ghost is
    # replaced by the fresh box and the slot number stays put.
    out.append(("claim_adopt", [
        [zbox(20, 40, 30, 80), zbox(180, 40, 30, 80),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
        [zbox(20, 40, 30, 80), (0, 0, 0, 0, 0, 0),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
        [zbox(20, 40, 30, 80), zbox(120, 40, 30, 80),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
    ]))

    # All four slots busy and the new detection is nowhere near them: the
    # stalest held slot makes way.  Slot 3 has held one frame longer than
    # slot 2, so slot 3 is the one that goes.
    out.append(("evict_stalest", [
        [zbox(20, 40, 30, 80), zbox(90, 40, 30, 80),
         zbox(160, 40, 30, 80), zbox(240, 40, 30, 80)],
        [zbox(20, 40, 30, 80), zbox(90, 40, 30, 80),
         zbox(160, 40, 30, 80), (0, 0, 0, 0, 0, 0)],
        [zbox(20, 40, 30, 80), zbox(90, 40, 30, 80),
         (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)],
        [zbox(20, 40, 30, 80), zbox(90, 40, 30, 80),
         zbox(290, 150, 20, 20), (0, 0, 0, 0, 0, 0)],
    ]))

    # Dense crowd-like sequence: boxes drift, swap area order, sometimes
    # disappear.  Coordinates stay inside the 320x180 processing frame.
    rng = __import__("random").Random(20261005)
    seq: list[list[tuple[int, ...]]] = []
    state = [
        [20.0, 40.0, 30.0, 80.0],
        [90.0, 45.0, 28.0, 72.0],
        [170.0, 35.0, 34.0, 88.0],
        [250.0, 50.0, 26.0, 66.0],
    ]
    alive = [True, True, True, True]
    for f in range(40):
        for k in range(NB):
            if alive[k] and rng.random() < 0.06:
                alive[k] = False
            if not alive[k] and rng.random() < 0.18:
                alive[k] = True
                state[k][0] = rng.uniform(10, 285)
                state[k][1] = rng.uniform(20, 100)
            if alive[k]:
                state[k][0] = min(310 - state[k][2], max(0.0,
                                      state[k][0] + rng.uniform(-8, 8)))
                state[k][1] = min(175 - state[k][3], max(0.0,
                                      state[k][1] + rng.uniform(-4, 4)))
        boxes = []
        for k in range(NB):
            if not alive[k]:
                boxes.append((0, 0, 0, 0, 0, 0))
            else:
                x, y, w, h = state[k]
                boxes.append(zbox(int(x), int(y), int(w), int(h),
                                  area=int(w * h * rng.uniform(0.65, 1.0))))
        boxes.sort(key=lambda t: t[5], reverse=True)
        seq.append(boxes)
    out.append(("random40", seq))
    return out


def run_rtl(raw: list[list[tuple[int, ...]]], tag: str, vvp: str
            ) -> tuple[list[list[tuple[int, ...]]],
                       list[list[int]]]:
    GEN.mkdir(parents=True, exist_ok=True)
    in_path = GEN / f"track_{tag}.in"
    out_path = GEN / f"track_{tag}.out"
    model = GEN / "sim_track_assoc.vvp"

    lines = []
    for frame in raw:
        vals = []
        for slot in frame:
            vals.extend(str(int(v)) for v in slot)
        lines.append(" ".join(vals))
    in_path.write_text("\n".join(lines) + "\n", encoding="ascii")

    proc = subprocess.run(
        [find_tool("iverilog"), "-g2005", "-o", str(model),
         str(RTL), str(PERSON_RTL), str(TB)],
        capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"iverilog failed:\n{proc.stdout}\n{proc.stderr}")
    proc = subprocess.run(
        [vvp, str(model), f"+IN={in_path}", f"+OUT={out_path}",
         f"+FRAMES={len(raw)}"],
        capture_output=True, text=True)
    if "DONE" not in proc.stdout:
        raise SystemExit(f"simulation failed:\n{proc.stdout}\n{proc.stderr}")

    rows = [tuple(int(v) for v in line.split())
            for line in out_path.read_text().splitlines() if line.strip()]
    if len(rows) != len(raw) * NB:
        raise SystemExit(f"{tag}: expected {len(raw)*NB} rows, got {len(rows)}")
    if any(len(r) != 7 for r in rows):
        raise SystemExit(f"{tag}: expected six box fields plus score")
    boxes = [[r[:6] for r in rows[i*NB:(i+1)*NB]]
             for i in range(len(raw))]
    scores = [[r[6] for r in rows[i*NB:(i+1)*NB]]
              for i in range(len(raw))]
    return boxes, scores


def score_of(box: tuple[int, ...], evid: int) -> int:
    if not box[0]:
        return 0
    w, h, area = int(box[3]), int(box[4]), int(box[5])
    _, aspect, fill, size = buckets(w, h, area, max(1, w * h), evid)
    return score(evid, aspect, fill, size)


def main() -> int:
    vvp = find_tool("vvp")
    ok_all = True
    for name, raw in cases():
        trace: list[list[int]] = []
        want = track_slots(raw, trace)
        got, got_score = run_rtl(raw, name, vvp)
        want_score = [[score_of(want[f][s], trace[f][s])
                       for s in range(NB)] for f in range(len(raw))]
        ok = got == want and got_score == want_score
        ok_all &= ok
        print(f"{'PASS' if ok else 'FAIL'} {name}: "
              f"{len(raw)} frames x {NB} slots")
        if not ok:
            for f, (g, w) in enumerate(zip(got, want)):
                if g != w:
                    print(f"     frame {f}: got  {g}")
                    print(f"                want {w}")
                    break
            for f, (g, w) in enumerate(zip(got_score, want_score)):
                if g != w:
                    print(f"     score frame {f}: got  {g}")
                    print(f"                     want {w}")
                    break
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
