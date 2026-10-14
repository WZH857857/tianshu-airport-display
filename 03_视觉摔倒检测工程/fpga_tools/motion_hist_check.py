"""Bit-exact check of motion_hist.v against tools/hist_model.py.

Streams the reference motion_morph masks into the module one pixel per
clock, publishes one tracker frame per step (the isolated timing the model
assumes), and compares

  * the frozen 2-bit map after every frame, dumped cell by cell, and
  * every published person box (found x y w h cells)

against ``hist_model.run``.  ``multi_top_check.py`` pins the same module
behind the real tracker; this bench isolates it, so a mismatch points at the
module instead of the board timing.

Usage::

    python motion_hist_check.py [--clips a.mp4 ...] [--frames 40]
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
sys.path.insert(0, str(HERE))

from hist_model import CH as HIST_CH, CW as HIST_CW  # noqa: E402
from hist_model import run as hist_run  # noqa: E402
from morph_check import GEN, find_tool, pack_masks  # noqa: E402
from multi_model import frames_model  # noqa: E402
from multi_top_check import cell_motion, chain_ref, load565, pbox_row  # noqa: E402
from track_model import track_slots  # noqa: E402

NUM_BLOBS = 4

RTL = [
    FPGA / "rtl" / "motion_hist.v",
    FPGA / "tb" / "eg_phy_bram_stub.v",
]
TB = FPGA / "tb" / "tb_motion_hist.v"

DEFAULT_CLIPS = [
    ROOT / "out" / "test_videos" / "fall_test.mp4",
    ROOT / "out" / "test_videos" / "crowd_test.mp4",
    ROOT / "datasets" / "cctv_fall" / "day11_event001_confirmed.mp4",
]


def write_inputs(tag: str, morph: np.ndarray, tracked) -> tuple[Path, Path]:
    mask_path = GEN / f"mhist_{tag}.mask"
    trk_path = GEN / f"mhist_{tag}.trk"
    mask_path.write_bytes(pack_masks(morph))
    lines = []
    for row in tracked:
        for slot in row:
            lines.append(" ".join(str(int(v)) for v in slot[:5]))
    trk_path.write_text("\n".join(lines) + "\n")
    return mask_path, trk_path


def run_rtl(mask_path: Path, trk_path: Path, tag: str, n: int) -> dict:
    out_path = GEN / f"mhist_{tag}.out"
    dump_path = GEN / f"mhist_{tag}.dump"
    model = GEN / "sim_motion_hist.vvp"
    cmd = [find_tool("iverilog"), "-g2005", "-o", str(model),
           *[str(p) for p in RTL], str(TB)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"iverilog failed:\n{proc.stdout}\n{proc.stderr}")
    proc = subprocess.run(
        [find_tool("vvp"), str(model),
         f"+IN={mask_path}", f"+TRK={trk_path}", f"+OUT={out_path}",
         f"+DUMP={dump_path}", f"+FRAMES={n}"],
        capture_output=True, text=True)
    if "DONE" not in proc.stdout:
        raise SystemExit(f"simulation failed:\n{proc.stdout}\n{proc.stderr}")

    pubs = {}
    dirs = {}
    for line in proc.stdout.splitlines():
        parts = line.split()
        if parts and parts[0] == "PBOX":
            pub, k = int(parts[1]), int(parts[2])
            pubs.setdefault(pub, [None] * NUM_BLOBS)[k] = tuple(
                int(v) for v in parts[3:])
        elif parts and parts[0] == "PDIR":
            pub, k = int(parts[1]), int(parts[2])
            dirs.setdefault(pub, [None] * NUM_BLOBS)[k] = int(parts[3])
    dump = np.frombuffer(dump_path.read_bytes(), dtype=np.uint8)
    maps = dump.reshape(-1, HIST_CH * HIST_CW) if dump.size else np.zeros(
        (0, HIST_CH * HIST_CW), dtype=np.uint8)
    return {"pubs": [pubs[i] for i in sorted(pubs)],
            "dirs": [dirs[i] for i in sorted(dirs)],
            "maps": maps, "stdout": proc.stdout}


def check_clip(vvp: str, clip: Path, nframes: int) -> bool:
    frames = load565(clip, nframes)
    if frames is None:
        print(f"SKIP {clip.name}: not enough frames")
        return True
    n = len(frames)
    want_mask, want_morph = chain_ref(frames)
    want_slots = track_slots(frames_model(want_morph))
    maps: list = []
    want_pbox, _ = hist_run(cell_motion(want_morph), want_slots, maps)
    want_maps = np.stack([m.reshape(-1) for m in maps]).astype(np.uint8)

    tag = clip.stem.replace(" ", "_")[:20] + f"_{n}f"
    mask_path, trk_path = write_inputs(tag, want_morph, want_slots)
    got = run_rtl(mask_path, trk_path, tag, n)

    # publishes: step f answers model row f+1, so the last (n-th) publish has
    # no model row to compare against.
    npub = n - 1
    want_pubs = [[pbox_row(e) for e in row] for row in want_pbox[1:]]
    got_pubs = [list(row) for row in got["pubs"][:npub]]
    pbox_ok = (len(got["pubs"]) >= npub) and (got_pubs == want_pubs)

    # Head end of a lying figure: the tracked blob's centre past the person
    # box's centre, in doubled coordinates so the halves stay integral.
    want_dirs = []
    for j in range(npub):
        row = []
        for k in range(NUM_BLOBS):
            pb = want_pbox[j + 1][k]
            s = want_slots[j][k]
            right = pb is not None and pb[2] > pb[3] and \
                (s[1] + s[3] / 2) > (pb[0] + pb[2] / 2)
            row.append(1 if right else 0)
        want_dirs.append(row)
    got_dirs = [list(row) for row in got["dirs"][:npub]]
    dir_ok = got_dirs == want_dirs

    # maps: the dump after frame f's publish is the snapshot the sweep of
    # frame f read, i.e. the map frame f left behind -- model row f.
    nmap = len(got["maps"])
    maps_ok = nmap >= n
    if maps_ok:
        maps_ok = np.array_equal(got["maps"][:n], want_maps[:n])

    ok = pbox_ok and maps_ok and dir_ok
    print(f"{'PASS' if ok else 'FAIL'} {clip.name}: {n} frames, "
          f"pbox {len(got_pubs)}/{npub} {'match' if pbox_ok else 'MISMATCH'}, "
          f"map {nmap}/{n} {'match' if maps_ok else 'MISMATCH'}, "
          f"dir {'match' if dir_ok else 'MISMATCH'}")

    if not pbox_ok:
        for j, (g, w) in enumerate(zip(got_pubs, want_pubs)):
            if g != w:
                for k in range(NUM_BLOBS):
                    if g[k] != w[k]:
                        print(f"     publish {j + 1} slot {k}: "
                              f"got {g[k]} want {w[k]}")
                break
    if not maps_ok and nmap:
        for j in range(min(nmap, n)):
            g = got["maps"][j].astype(int)
            w = want_maps[j].astype(int)
            if not np.array_equal(g, w):
                d = np.nonzero(g != w)[0]
                cells = [(int(i % HIST_CW), int(i // HIST_CW),
                          int(g[i]), int(w[i])) for i in d[:8]]
                print(f"     map after frame {j}: {len(d)} cells differ "
                      f"(x, y, rtl, model): {cells}")
                break
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--clips", type=Path, nargs="*", default=DEFAULT_CLIPS)
    ap.add_argument("--frames", type=int, default=40)
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
