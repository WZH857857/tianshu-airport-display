"""Cycle-exact Python model of rtl/motion_morph.v.

Mirrors the Verilog state machine bit-for-bit (stack3/stack7, eb_d1/eb_d2,
h7/c7/c5/c3, the P_IDLE/P_ROW/P_FROW sequencer) so the datapath can be
debugged and re-tuned in Python instead of waiting on 90 s Icarus runs.

``run_frames`` returns the output in the bit order the testbench sees:
320 bits per row, 180 rows per input frame, rows in raster order.

Usage::

    python morph_rtl_model.py            # impulse-response sanity check
"""

from __future__ import annotations

import numpy as np

W, H = 320, 180

DIL_RADIUS = 3
DIL_LATENCY = 1 + DIL_RADIUS
ROW_LATENCY = 3
ROW_SLOTS = W + DIL_LATENCY
LAST_SLOT = ROW_SLOTS - 1
FLUSH_ROWS = ROW_LATENCY

P_IDLE, P_ROW, P_FROW = 0, 1, 2


class MorphRtl:
    """Bit-exact mirror of the current motion_morph.v."""

    def __init__(self) -> None:
        self.stack3 = np.zeros(W, dtype=np.uint8)
        self.stack7 = np.zeros(W, dtype=np.uint8)
        self.p1a = self.p2a = self.q2a = self.q0a = 0
        self.eb_d1 = self.eb_d2 = 0
        self.h7 = np.zeros(7, dtype=np.uint8)
        self.c7 = np.zeros(7, dtype=np.uint8)
        self.c5 = np.zeros(7, dtype=np.uint8)
        self.c3 = np.zeros(7, dtype=np.uint8)
        self.phase = P_IDLE
        self.slot = 0
        self.row = 0
        self.flush_left = 0
        self.de_out = 0
        self.mask_out = 0
        self.ready = False
        self.clr_addr = 0
        self.clr_busy = True

    # -------------------------------------------------------------- one clock
    def step(self, de_in: int, mask_in: int) -> tuple[int, int]:
        """Advance one posedge.  Returns (de_out, mask_out) after the edge."""
        # ---- combinational block
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

        # ---- clocked block (de_out <= 0 first)
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

            # --------------------------------------------- disk(3) dilation
            self.eb_d2 = ((self.eb_d2 << 1) | ((self.eb_d1 >> 6) & 1)) & 0x7F
            self.eb_d1 = ((self.eb_d1 << 1) | e_b) & 0x7F

            if dslot >= 1:
                w7_cur = 0
                if dslot <= W:
                    w7_cur = int(self.stack7[dslot - 1])
                w7_new = ((w7_cur << 1) | e_b) & 0x7F
                if dslot <= W:
                    self.stack7[dslot - 1] = w7_new

                d_or = 0
                for kf in range(7):
                    bbit = (self.eb_d1 >> 6) & 1
                    hist = int(self.h7[kf])
                    nh = ((hist << 1) | bbit) & 0x7F
                    if kf in (0, 6):
                        n = (int(self.c3[kf]) + bbit
                             - ((hist >> 2) & 1)) & 0x7
                        self.c3[kf] = n
                    elif kf in (1, 5):
                        n = (int(self.c3[kf]) + bbit
                             - ((hist >> 1) & 1)) & 0x7
                        self.c3[kf] = n
                    elif kf == 2:
                        n = (int(self.c7[kf]) + bbit
                             - ((hist >> 6) & 1)) & 0x7
                        self.c7[kf] = n
                    elif kf == 3:
                        n = (int(self.c5[kf]) + bbit
                             - ((hist >> 4) & 1)) & 0x7
                        self.c5[kf] = n
                    else:
                        n = (int(self.c5[kf]) + bbit
                             - ((hist >> 4) & 1)) & 0x7
                        self.c5[kf] = n
                    self.h7[kf] = nh
                    d_or |= 1 if n else 0

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
    """masks: (F, H, W) bool.  Returns (F, H, W) bool, raster order."""
    out = np.zeros_like(masks)
    m = MorphRtl()
    for _ in range(W + 20):          # let the reset-time clear finish
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
        # flush rows plus slack for the output register
        for _ in range(FLUSH_ROWS * ROW_SLOTS + 8):
            de, mo = m.step(0, 0)
            if de:
                bits.append(mo)
        if len(bits) < H * W:
            raise RuntimeError(f"frame {f}: only {len(bits)} output bits")
        out[f] = np.array(bits[:H * W], dtype=bool).reshape(H, W)
    return out


def synthetic() -> np.ndarray:
    rng = np.random.default_rng(2026)
    zero = np.zeros((H, W), dtype=bool)
    frames = []

    impulse = zero.copy()
    for y, x in [(0, 0), (0, W - 1), (H - 1, 0), (H - 1, W - 1),
                 (H // 2, W // 2), (0, W // 2), (H // 2, 0),
                 (H - 1, W // 2), (H // 2, W - 1)]:
        impulse[y, x] = True
    frames.append(impulse)

    blocks = zero.copy()
    for y, x in [(0, 0), (H - 4, 0), (0, W - 4), (H - 4, W - 4)]:
        blocks[y:y + 4, x:x + 4] = True
    blocks[100:112, 150:165] = True
    blocks[170:171, 40:80] = True
    frames.append(blocks)

    blobs = zero.copy()
    yy, xx = np.ogrid[:H, :W]
    for cy, cx, r in [(60, 60, 3), (60, 200, 8), (150, 40, 15),
                      (0, 160, 6), (H - 1, 300, 6), (120, 319, 5)]:
        blobs |= ((yy - cy) ** 2 + (xx - cx) ** 2) <= r * r
    frames.append(blobs)

    noise = rng.random((H, W)) < 0.02
    for _ in range(6):
        y0 = int(rng.integers(0, H - 10))
        x0 = int(rng.integers(0, W - 10))
        noise[y0:y0 + int(rng.integers(3, 12)),
              x0:x0 + int(rng.integers(3, 12))] = True
    frames.append(noise)

    frames.append(zero.copy())
    frames.append(noise.copy())
    return np.stack(frames)


def main() -> int:
    # A plus-shaped input erodes to exactly one pixel, so the output is the
    # dilation structuring element placed at the pipeline's alignment.
    test = np.zeros((2, H, W), dtype=bool)
    for cy, cx in [(60, 60), (90, 160)]:
        test[0, cy, cx] = True
        test[0, cy - 1, cx] = True
        test[0, cy + 1, cx] = True
        test[0, cy, cx - 1] = True
        test[0, cy, cx + 1] = True
    got = run_frames(test)
    ys, xs = np.nonzero(got[0])
    print("cross impulse response:")
    for y in sorted(set(ys.tolist())):
        row = xs[ys == y]
        print(f"  y={y:3d} x={row.min()}..{row.max()} n={len(row)}")

    full = run_frames(synthetic())
    print("synthetic nonzero bits per frame:", [int(f.sum()) for f in full])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
