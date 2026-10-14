"""Scratch: find the first frame where motion_hist.v's frozen map diverges.

Compares the RTL dump against both candidate model alignments so we can tell
a real geometry bug from an off-by-one in the check itself.

Usage::

    python fpga/tools/mhist_debug.py <clip> <frames> <tag>
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(HERE))

from hist_model import CH, CW, run as hist_run  # noqa: E402
from morph_check import GEN  # noqa: E402
from multi_model import frames_model  # noqa: E402
from multi_top_check import cell_motion, chain_ref, load565  # noqa: E402
from track_model import track_slots  # noqa: E402


def main() -> int:
    clip = Path(sys.argv[1])
    nframes = int(sys.argv[2])
    tag = sys.argv[3]
    frames = load565(clip, nframes)
    n = len(frames)
    _, want_morph = chain_ref(frames)
    want_slots = track_slots(frames_model(want_morph))
    maps: list = []
    _, _ = hist_run(cell_motion(want_morph), want_slots, maps)
    want = np.stack([m.reshape(-1) for m in maps]).astype(np.uint8)

    dump = np.frombuffer(
        (GEN / f"mhist_{tag}.dump").read_bytes(), dtype=np.uint8)
    got = dump.reshape(-1, CH * CW) if dump.size else np.zeros(
        (0, CH * CW), dtype=np.uint8)
    print(f"frames={n} rtl_maps={len(got)} model_maps={len(want)}")

    for shift in (0, 1):
        bad = None
        for j in range(len(got)):
            w = j + shift
            if w >= len(want):
                break
            if not np.array_equal(got[j], want[w]):
                bad = j
                break
        print(f"alignment got[j] vs want[j+{shift}]: "
              f"{'all match' if bad is None else f'first diff at j={bad}'}")
        if bad is not None:
            g = got[bad].astype(int)
            w = want[bad + shift].astype(int)
            d = np.nonzero(g != w)[0]
            cells = [(int(i % CW), int(i // CW), int(g[i]), int(w[i]))
                     for i in d[:6]]
            print(f"    {len(d)} cells differ, rtl={int((g > 0).sum())} "
                  f"live, model={int((w > 0).sum())} live, "
                  f"sample (x, y, rtl, model): {cells}")

    print()
    print("frame | tracked boxes (found x y w h)                    | "
          "model live | rtl live | cm cells>=2 | cm pixels")
    cm = cell_motion(want_morph)
    for f in range(min(n, 12)):
        boxes = " ".join(
            f"{int(b[0])}/{int(b[1])},{int(b[2])},{int(b[3])},{int(b[4])}"
            for b in want_slots[f])
        mlive = int((want[f] > 0).sum()) if f < len(want) else -1
        rlive = int((got[f] > 0).sum()) if f < len(got) else -1
        c = cm[f]
        print(f"{f:5d} | {boxes:52s} | {mlive:9d} | {rlive:8d} | "
              f"{int((c >= 2).sum()):11d} | {int(c.sum()):9d}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
