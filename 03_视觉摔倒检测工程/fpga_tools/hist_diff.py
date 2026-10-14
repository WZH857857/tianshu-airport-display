"""One-off diagnostic: compare RTL pass histograms against the Python model.

The RTL testbench gained a +HIST=<file> probe that dumps all 320 entries after
each pass's accumulation finishes.  This script feeds the same synthetic mask
that multi_bbox_check uses, then reproduces the reference histogram for each
pass and reports the first differing column.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))

from morph_check import pack_masks, synthetic_masks  # noqa: E402

FPGA = TOOLS.parent
GEN = FPGA / "td" / "gen"
W, H = 320, 180
NUM_BLOBS, MARGIN = 4, 8


def band_of(mask: np.ndarray) -> tuple[int, int] | None:
    rows = mask.sum(axis=1)
    active = rows >= 3
    best = None
    run = 0
    for y, a in enumerate(active):
        if a:
            run += 1
            if best is None or run > best[0]:
                best = (run, y - run + 1)
        else:
            run = 0
    return None if best is None else (best[1], best[1] + best[0])


def histogram(mask: np.ndarray, y0: int, y1: int,
              boxes: list[tuple[int, int, int, int]]) -> np.ndarray:
    band = mask[y0:y1].copy()
    for bx0, by0, bx1, by1 in boxes:
        lo = max(by0, y0) - y0
        hi = min(by1, y1) - y0
        if hi > lo:
            band[lo:hi, bx0:bx1] = False
    return band.sum(axis=0).astype(int)


def scan(profile: np.ndarray) -> tuple[int, int, int]:
    best = (0, 0, 0)
    run_len = run_start = run_sum = 0
    for x, v in enumerate(profile):
        if v >= 3:
            if run_len == 0:
                run_start, run_sum = x, 0
            run_len += 1
            run_sum += int(v)
            if run_len > best[0]:
                best = (run_len, run_start, run_sum)
        else:
            run_len = run_sum = 0
    return best[1], best[0], best[2]


def main() -> int:
    rng = np.random.default_rng(12345)
    mask = synthetic_masks(rng)[0]
    GEN.mkdir(parents=True, exist_ok=True)
    in_path = GEN / "hist_diff_in.bin"
    out_path = GEN / "hist_diff_out.txt"
    hist_path = GEN / "hist_diff_hist.txt"
    in_path.write_bytes(pack_masks(np.stack([mask])))

    vvp = GEN / "sim_multi_bbox.vvp"
    subprocess.run([
        r"E:\iverilog\bin\iverilog.exe", "-g2005", "-o", str(vvp),
        str(FPGA / "tb" / "tb_multi_bbox.v"),
        str(FPGA / "rtl" / "multi_bbox.v")], check=True)
    subprocess.run([
        r"E:\iverilog\bin\vvp.exe", str(vvp),
        f"+IN={in_path}", f"+OUT={out_path}", "+FRAMES=1",
        f"+HIST={hist_path}"], check=True)

    lines = hist_path.read_text().split()
    rtl: dict[int, np.ndarray] = {}
    i = 0
    while i < len(lines):
        assert lines[i] == "HIST", lines[i]
        p = int(lines[i + 1])
        rtl[p] = np.array([int(v) for v in lines[i + 2:i + 2 + W]])
        i += 2 + W

    span = band_of(mask)
    print(f"band = {span}")
    y0, y1 = span
    boxes: list[tuple[int, int, int, int]] = []
    for p in range(NUM_BLOBS):
        want = histogram(mask, y0, y1, boxes)
        got = rtl[p]
        diff = np.nonzero(got != want)[0]
        print(f"pass {p}: mismatched columns = {len(diff)}"
              f"  first = {diff[:8].tolist()}")
        for c in diff[:5]:
            print(f"   col {c}: rtl={got[c]} want={want[c]}")
        x, ln, area = scan(want)
        print(f"   model box x={x} w={ln} area={area}")
        if ln:
            boxes.append((x - MARGIN, y0 - MARGIN, x + ln + MARGIN,
                          y1 + MARGIN))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
