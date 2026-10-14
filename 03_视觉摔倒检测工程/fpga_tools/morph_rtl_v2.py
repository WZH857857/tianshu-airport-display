"""Cycle model of the redesigned motion_morph: disk(1) erode + disk(3) dilate.

The previous RTL fed every dilation lane the same 6-cycle-old copy of the
eroded stream, so the seven "lanes" all computed the same horizontal window
and the module was really a 1-row 7-wide dilation.  This model implements
the intended datapath:

  * erosion (unchanged): e_b at slot s of input row r is E[r-2][s-1]
  * stack7[c] is written with e_b at slot c+1, so bit i of the column holds
    E[r-2-i][c]; each row shifts every column exactly once
  * lane kf reads bit 6-kf of the freshly written 7-bit column word, which
    is the erosion pixel of row r-5+(kf-3) at that column
  * each lane runs a centred horizontal window of 1, 5 or 7 columns and the
    shorter windows are delayed so all seven land on the same output column

Output row y = r - 5 is emitted at slot s = u + 4, one bit per slot, so a
frame is 180 rows x 320 bits with ROW_LATENCY/FLUSH_ROWS = 5 flush rows.

Usage::

    python morph_rtl_v2.py         # compare against SciPy on patterns+clip
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(HERE))

from fall_proto import disk  # noqa: E402
from morph_rtl_model import H, W, synthetic  # noqa: E402

DIL_RADIUS = 3
DIL_LATENCY = 4               # out col = slot - 4
ROW_LATENCY = 5               # out row = input row - 5
ROW_SLOTS = W + DIL_LATENCY   # 325
LAST_SLOT = ROW_SLOTS - 1
FLUSH_ROWS = ROW_LATENCY

P_IDLE, P_ROW, P_FROW = 0, 1, 2

# lane kf -> (dy, half width h, window width 2h+1, vpipe tap)
#   tap 0 = lane value this cycle (h = 3)
#   tap 1 = one cycle old        (h = 2)
#   tap 3 = three cycles old     (h = 0)
LANES = [
    (-3, 0), (-2, 2), (-1, 2), (0, 3), (1, 2), (2, 2), (3, 0),
]


class MorphRtlV2:
    """Mirrors the redesigned datapath, one step per clock."""

    def __init__(self) -> None:
        self.stack3 = np.zeros(W, dtype=np.uint8)
        self.stack7 = np.zeros(W, dtype=np.uint8)
        self.p1a = self.p2a = self.q2a = self.q0a = 0
        self.h7 = np.zeros(7, dtype=np.uint8)     # per lane, bit0 = newest
        self.cnt = np.zeros(7, dtype=np.uint8)    # 3-bit running window OR
        self.vpipe = np.zeros(7, dtype=np.uint8)  # 3-bit delay of lane value
        self.phase = P_IDLE
        self.slot = 0
        self.row = 0
        self.flush_left = 0
        self.de_out = 0
        self.mask_out = 0
        self.ready = False
        self.clr_addr = 0
        self.clr_busy = True

    def step(self, de_in: int, mask_in: int) -> tuple[int, int]:
        process = 0
        pix = 0
        dslot = self.slot
        row_done = 0
        if self.ready and not self.clr_busy:
            if self.phase == P_IDLE:
                if de_in:
                    process, pix, dslot = 1, mask_in, 0
            elif self.phase == P_ROW:
                if self.slot < W:
                    if de_in:
                        process, pix, dslot = 1, mask_in, self.slot
                else:
                    process, dslot = 1, self.slot
                if process and dslot == LAST_SLOT:
                    row_done = 1
            else:
                process, dslot = 1, self.slot
                if dslot == LAST_SLOT:
                    row_done = 1

        self.de_out = 0
        if not self.ready:
            if self.clr_busy:
                if self.clr_addr == W - 1:
                    self.clr_busy = False
                    self.ready = True
                else:
                    self.clr_addr += 1
            return self.de_out, self.mask_out

        if process:
            # ------------------------------------------------ cross erosion
            w3a_cur = int(self.stack3[dslot]) if dslot < W else 0
            w3a_new = ((w3a_cur << 1) | pix) & 0x7
            e_a = (self.q2a & self.p1a & self.q0a & self.p2a
                   & ((w3a_cur >> 1) & 1))
            if dslot < W:
                self.stack3[dslot] = w3a_new
            self.p2a = self.p1a
            self.p1a = (w3a_cur >> 1) & 1
            self.q2a = (w3a_cur >> 2) & 1
            self.q0a = w3a_cur & 1
            e_b = e_a

            # --------------------------------------------- vertical columns
            col = dslot - 1
            lane_src = 0
            if 0 <= col < W:
                lane_src = ((int(self.stack7[col]) << 1) | e_b) & 0x7F
                self.stack7[col] = lane_src

            # ------------------------------------- centred horizontal windows
            d_or = 0
            for kf, (_dy, h) in enumerate(LANES):
                bbit = (lane_src >> (6 - kf)) & 1
                if h == 0:
                    v_now = bbit
                else:
                    width = 2 * h + 1
                    hist = int(self.h7[kf])
                    n = (int(self.cnt[kf]) + bbit
                         - ((hist >> (width - 1)) & 1)) & 0x7
                    self.cnt[kf] = n
                    self.h7[kf] = ((hist << 1) | bbit) & 0x7F
                    v_now = 1 if n else 0
                if h == 3:
                    d_or |= v_now
                elif h == 2:
                    d_or |= (int(self.vpipe[kf]) >> 0) & 1
                else:
                    d_or |= (int(self.vpipe[kf]) >> 2) & 1
                self.vpipe[kf] = ((int(self.vpipe[kf]) << 1) | v_now) & 0x7

            if dslot >= DIL_LATENCY and self.row >= ROW_LATENCY:
                self.mask_out = d_or
                self.de_out = 1

            # -------------------------------------------- row sequencing
            if row_done:
                self.slot = 0
                if self.phase == P_ROW:
                    if self.row == H - 1:
                        self.row = H
                        self.phase = P_FROW
                        self.flush_left = 0
                    else:
                        self.row += 1
                elif self.flush_left == FLUSH_ROWS - 1:
                    self.row = 0
                    self.phase = P_IDLE
                else:
                    self.flush_left += 1
            else:
                self.slot += 1
                if self.phase == P_IDLE:
                    self.phase = P_ROW
                    self.slot = 1
        return self.de_out, self.mask_out


def run_frames(masks: np.ndarray) -> np.ndarray:
    out = np.zeros_like(masks)
    m = MorphRtlV2()
    for _ in range(W + 20):
        m.step(0, 0)
    for f in range(len(masks)):
        bits: list[int] = []
        for r in range(H):
            for c in range(W):
                de, mo = m.step(1, int(masks[f, r, c]))
                if de:
                    bits.append(mo)
            for _ in range(DIL_LATENCY):
                de, mo = m.step(0, 0)
                if de:
                    bits.append(mo)
        for _ in range(FLUSH_ROWS * ROW_SLOTS + 8):
            de, mo = m.step(0, 0)
            if de:
                bits.append(mo)
        if len(bits) != H * W:
            raise RuntimeError(f"frame {f}: {len(bits)} output bits, "
                               f"want {H * W}")
        out[f] = np.array(bits, dtype=bool).reshape(H, W)
    return out


def morph_ref(mask: np.ndarray, erode_r: int, dilate_r: int) -> np.ndarray:
    if not mask.any():
        return np.zeros_like(mask)
    e = ndimage.binary_erosion(mask, structure=disk(erode_r), border_value=0)
    return ndimage.binary_dilation(e, structure=disk(dilate_r), border_value=0)


def main() -> int:
    masks = synthetic()
    want = np.stack([morph_ref(m, 1, 3) for m in masks])
    got = run_frames(masks)
    diff = got != want
    print(f"synthetic: {int(diff.sum())} bit errors "
          f"(extra={int((got & ~want).sum())} "
          f"missing={int((~got & want).sum())})")
    if diff.any():
        d = np.argwhere(diff)
        print("  first", d[0].tolist(), "per-frame",
              [int(diff[i].sum()) for i in range(len(masks))])

    clip = ROOT / "datasets" / "gmdcsa24" / "Subject 1" / "Fall" / "13.mp4"
    if clip.exists():
        sys.path.insert(0, str(HERE))
        from morph_check import masks_from_clip
        cm = masks_from_clip(clip, 8)
        if cm is not None:
            cw = np.stack([morph_ref(m, 1, 3) for m in cm])
            cg = run_frames(cm)
            cd = cg != cw
            print(f"clip: {int(cd.sum())} bit errors "
                  f"per-frame {[int(cd[i].sum()) for i in range(len(cm))]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
