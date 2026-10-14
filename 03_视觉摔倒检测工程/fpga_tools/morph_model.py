"""Cycle model of the motion_morph disk(3) dilation lane taps.

The seven horizontal OR-windows of the diamond dilation are the one part of
the RTL that cannot be checked by reading the code: each lane's history and
sticky count has to stay aligned to the same output column while the newer
lanes ride one- and two-pixel-delayed copies of the eroded stream.  This
model mirrors the RTL state bit-for-bit and compares its output with
``scipy.ndimage.binary_dilation(disk(3))`` over synthetic patterns.

Only the dilation is modelled; the cross erosion is a single well-tested
stage and stays in RTL.

Usage::

    python fpga/tools/morph_model.py
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
DIL_LATENCY = 4      # column gate: out col = input col - 4
ROW_LATENCY = 3      # row gate
DRAIN = 3            # RTL flush rows


class DilateModel:
    """Mirrors the stack7 / eb_d1 / eb_d2 / h7 / c* datapath."""

    def __init__(self) -> None:
        self.err: list[str] = []
        self.stack7 = np.zeros((W, 7), dtype=np.uint8)
        self.eb_d1 = 0
        self.eb_d2 = 0
        self.h7 = [0] * 7
        self.c7 = np.zeros(7, dtype=np.int32)
        self.c5 = np.zeros(7, dtype=np.int32)
        self.c3 = np.zeros(7, dtype=np.int32)

    def run(self, eroded: np.ndarray) -> np.ndarray:
        """eroded: (H, W) uint8.  Returns the RTL output, (H, W)."""
        out = np.zeros((H + DRAIN, W), dtype=np.uint8)
        for row in range(H + DRAIN):
            for slot in range(W + DIL_LATENCY):
                pix = int(eroded[row, slot]) if (row < H and slot < W) else 0
                # ---- the erosion output fed to the dilation
                e_b = pix
                new_d1 = ((self.eb_d1 << 1) | e_b) & 0x7F
                new_d2 = ((self.eb_d2 << 1) | ((self.eb_d1 >> 6) & 1)) & 0x7F
                self.eb_d1, self.eb_d2 = new_d1, new_d2
                if slot < 1:
                    continue
                # ---- stack7 write (oldest row shifts out)
                if slot - 1 < W:
                    self.stack7[slot - 1, 1:] = self.stack7[slot - 1, :-1]
                    self.stack7[slot - 1, 0] = (new_d1 >> 6) & 1
                if slot - 1 >= W:
                    continue
                d_or = 0
                for kf in range(7):
                    # Delay taps: centre lanes (|dy| <= 1) ride the stream
                    # delayed by 3 while the outer lanes ride the stream
                    # delayed by 4; taking the window out of bits 0..N-1 of
                    # the lane's own history puts every lane on the centre
                    # column x-4.  The window width follows the diamond.
                    bbit = (new_d1 >> 6) & 1 if kf in (1, 3, 5) \
                        else (new_d2 >> 6) & 1
                    hist = self.h7[kf]
                    nh = ((hist << 1) | bbit) & 0x7F
                    if kf in (0, 6):
                        n = self.c3[kf] + bbit - ((hist >> 0) & 1)
                        self.c3[kf] = n
                    elif kf in (1, 5):
                        n = self.c3[kf] + bbit - ((hist >> 1) & 1)
                        self.c3[kf] = n
                    elif kf == 2:
                        n = self.c7[kf] + bbit - ((hist >> 5) & 1)
                        self.c7[kf] = n
                    elif kf == 3:
                        n = self.c5[kf] + bbit - ((hist >> 3) & 1)
                        self.c5[kf] = n
                    else:
                        n = self.c5[kf] + bbit - ((hist >> 4) & 1)
                        self.c5[kf] = n
                    val = n != 0
                    self.h7[kf] = nh
                    d_or |= int(val)
                if slot >= DIL_LATENCY and row >= ROW_LATENCY:
                    out[row, slot - DIL_LATENCY] = d_or
        return out[:H, :]


def shift_ref(a: np.ndarray, dy: int, dx: int) -> np.ndarray:
    out = np.zeros_like(a)
    ys0, ys1 = max(0, dy), min(H, H + dy)
    xs0, xs1 = max(0, dx), min(W, W + dx)
    ys, xs = slice(max(0, -dy), max(0, -dy) + (ys1 - ys0)), \
        slice(max(0, -dx), max(0, -dx) + (xs1 - xs0))
    out[ys, xs] = a[ys0:ys1, xs0:xs1]
    return out


def main() -> int:
    rng = np.random.default_rng(7)
    patterns: dict[str, np.ndarray] = {}
    patterns["sparse"] = (rng.random((H, W)) < 0.002).astype(np.uint8)
    patterns["dense"] = (rng.random((H, W)) < 0.05).astype(np.uint8)
    blk = np.zeros((H, W), np.uint8)
    blk[60:120, 100:220] = 1
    patterns["block"] = blk
    iso = np.zeros((H, W), np.uint8)
    iso[:, ::37] = 1
    patterns["isolated"] = iso

    for name, eroded in patterns.items():
        want = ndimage.binary_dilation(eroded, structure=disk(3),
                                       border_value=0).astype(np.uint8)
        got = DilateModel().run(eroded)
        # Find the alignment that removes the pipeline's constant offset.
        best = None
        for dy in range(-6, 7):
            for dx in range(-6, 7):
                cand = shift_ref(want, dy, dx)
                err = int((got != cand).sum())
                if best is None or err < best[0]:
                    best = (err, dy, dx, cand)
        err, dy, dx, want_shift = best
        diff = np.argwhere(got != want_shift)
        print(f"{name:<9} errors={len(diff)} "
              f"best shift=({dy},{dx}) "
              f"first={diff[0].tolist() if len(diff) else '-'}")
        if len(diff):
            y, x = diff[0]
            print(f"   got rows {got[:, max(0, x-2):x+3].sum(axis=0).tolist()} "
                  f"want {want_shift[:, max(0, x-2):x+3].sum(axis=0).tolist()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
