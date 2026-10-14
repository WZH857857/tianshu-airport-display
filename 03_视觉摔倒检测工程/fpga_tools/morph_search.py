"""Find the per-lane (delay tap, drain bit) that reproduce disk(3) dilation.

Each of the seven diamond lanes owns an independent horizontal OR-window, so
the lane taps can be solved one lane at a time: run the lane with every
(tap, drain) pair, compare its single-lane output with the reference
diamond's contribution at that vertical offset, and keep the exact match.

Usage::

    python fpga/tools/morph_search.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent / "tools"))
from fall_proto import disk  # noqa: E402

W, H = 320, 180
R = 3
LAT = 1 + R
WIDTH = [3, 3, 7, 5, 5, 3, 3]      # diamond window per dy = k-3


def lane_input(eroded: np.ndarray, tap: int) -> list[int]:
    """The lane's stream: the eroded stream delayed by 3 (tap 1) or 4 (tap 2).

    Returns a flat list of one bit per (delay, slot) position, in raster
    order, including the trailing padding slots and flush rows.
    """
    bits: list[int] = []
    d1 = d2 = 0
    for row in range(H + R):
        for slot in range(W + LAT):
            pix = int(eroded[row, slot]) if (row < H and slot < W) else 0
            new_d1 = ((d1 << 1) | pix) & 0x7F
            new_d2 = ((d2 << 1) | ((d1 >> 6) & 1)) & 0x7F
            d1, d2 = new_d1, new_d2
            bits.append((d1 >> 6) & 1 if tap == 1 else (d2 >> 6) & 1)
    return bits


def lane_target(eroded: np.ndarray, k: int) -> np.ndarray:
    """The pixels lane k alone must light: a horizontal window on row -dy."""
    dy = k - R
    half = (WIDTH[k] - 1) // 2
    acc = np.zeros_like(eroded)
    for dx in range(-half, half + 1):
        acc |= shift_ref(eroded, -dy, dx)
    return acc


def run_lane(bits: list[int], drain: int) -> np.ndarray:
    """One horizontal OR-window lane over the (H+R) x (W+LAT) stream."""
    n = W + LAT
    out = np.zeros((H, W), dtype=np.uint8)
    hist = 0
    cnt = 0
    for row in range(H + R):
        for slot in range(n):
            bbit = bits[row * n + slot]
            cnt += bbit - ((hist >> drain) & 1)
            hist = ((hist << 1) | bbit) & 0x7F
            if 1 <= slot < W + 1 and slot >= LAT and R <= row < H + R:
                out[row - R, slot - LAT] = int(cnt != 0)
    return out


def main() -> int:
    rng = np.random.default_rng(11)
    cases = []
    for thresh in (0.002, 0.02, 0.08):
        cases.append((rng.random((H, W)) < thresh).astype(np.uint8))
    blk = np.zeros((H, W), np.uint8)
    blk[60:120, 100:220] = 1
    cases.append(blk)
    iso = np.zeros((H, W), np.uint8)
    iso[:, ::37] = 1
    cases.append(iso)

    for k in range(7):
        # Solve the whole lane at once: for each (tap, drain) pair the lane
        # output is compared with every constant shift of its own horizontal
        # window, and the best triple is kept.
        best = None
        got_cache = {}
        for tap in (1, 2):
            for drain in range(7):
                got_cache[(tap, drain)] = [
                    run_lane(lane_input(eroded, tap), drain)
                    for eroded in cases]
        want_cache = {
            (dy, dx): [shift_ref(lane_target(eroded, k), dy, dx)
                       for eroded in cases]
            for dy in range(-8, 9) for dx in range(-8, 9)}
        for (tap, drain), gots in got_cache.items():
            for (dy, dx), wants in want_cache.items():
                err = sum(int((g != w).sum())
                          for g, w in zip(gots, wants))
                if best is None or err < best[0]:
                    best = (err, tap, drain, dy, dx)
        print(f"lane {k} (dy={k - R:+d}, width {WIDTH[k]}): "
              f"errors={best[0]} tap={best[1]} drain={best[2]} "
              f"shift=({best[3]},{best[4]})")
    return 0


def best_shift(got: np.ndarray, want: np.ndarray) -> tuple[int, int, int]:
    best = None
    for dy in range(-7, 8):
        for dx in range(-7, 8):
            cand = shift_ref(want, dy, dx)
            err = int((got != cand).sum())
            if best is None or err < best[0]:
                best = (err, dy, dx)
    return best


def shift_ref(a: np.ndarray, dy: int, dx: int) -> np.ndarray:
    out = np.zeros_like(a)
    ys0, ys1 = max(0, -dy), min(H, H - dy)
    xs0, xs1 = max(0, -dx), min(W, W - dx)
    out[ys0:ys1, xs0:xs1] = a[ys0 + dy:ys1 + dy, xs0 + dx:xs1 + dx]
    return out


if __name__ == "__main__":
    raise SystemExit(main())
