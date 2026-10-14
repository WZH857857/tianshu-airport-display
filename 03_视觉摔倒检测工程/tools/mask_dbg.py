"""Locate the first differing mask pixel between the RTL dump and NumPy.

Usage::

    python tools/mask_dbg.py CLIP.mp4 [--frames 2]

Writes <gen>/dbg_rtl_mask.bin and <gen>/dbg_ref_mask.bin and prints the
first differing pixel as (frame, row, col) plus a small neighbourhood of the
raw frame-difference values around it.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import imageio.v3 as iio
import numpy as np

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent / "fpga"
ROOT = FPGA.parent
sys.path.insert(0, str(FPGA / "tools"))
sys.path.insert(0, str(ROOT / "tools"))

from pixel_check import down2_box, luma_from_565, quantise565  # noqa: E402

W, H = 320, 180
KEEP = [y for y in range(240) if y % 4 != 3]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("clip", type=Path)
    ap.add_argument("--frames", type=int, default=2)
    args = ap.parse_args()

    gen = FPGA / "sim" / "gen"
    rtl = (gen / "pixel_13.mask.bin").read_bytes() \
        if (gen / "pixel_13.mask.bin").exists() else None
    if rtl is None:
        print("run pixel_check.py first")
        return 1
    rtl_gray = (gen / "pixel_13.gray.bin").read_bytes()

    frames = []
    for i, frame in enumerate(iio.imiter(args.clip, plugin="FFMPEG")):
        if i >= args.frames:
            break
        frames.append(np.asarray(frame))
    luma = np.stack([down2_box(luma_from_565(quantise565(f)))[KEEP]
                     for f in frames])
    ref = np.zeros_like(luma, dtype=bool)
    for i in range(1, len(luma)):
        d = np.abs(luma[i].astype(np.int16) - luma[i - 1].astype(np.int16))
        ref[i] = d > 10
    ref_b = np.stack([np.packbits(m, bitorder="big") for m in ref]).tobytes()

    (gen / "dbg_ref_mask.bin").write_bytes(ref_b)
    # The RTL gray dump is 57,600 bytes per frame; unpack it so a mismatch can
    # be tied back to the exact luma pair that produced the differing mask bit.
    got_gray = np.frombuffer(rtl_gray, dtype=np.uint8).reshape(-1, H, W)
    same_luma = (got_gray[: len(luma)] == luma).all()
    print(f"luma frames compared: {len(luma)}, identical: {same_luma}")
    n = min(len(rtl), len(ref_b))
    for i in range(n):
        if rtl[i] != ref_b[i]:
            px = i * 8
            f = px // (W * H)
            r = (px // W) % H
            c = px % W
            print(f"first byte diff at {i}: frame {f} row {r} col {c}")
            for dr in range(-1, 2):
                rr = r + dr
                if not 0 <= rr < H:
                    continue
                row_bits = "".join(
                    str(int(bit))
                    for bit in np.unpackbits(
                        np.frombuffer(rtl, dtype=np.uint8),
                        bitorder="big")[f * W * H + rr * W: 
                                          f * W * H + (rr + 1) * W])
                print(f"  rtl row {rr:3d}: {row_bits}")
            # Compare the two mask planes as bits for that neighbourhood.
            rbits = np.unpackbits(
                np.frombuffer(rtl, dtype=np.uint8), bitorder="big")
            wbits = np.unpackbits(
                np.frombuffer(ref_b, dtype=np.uint8), bitorder="big")
            plane = f * W * H
            a = rbits[plane:plane + W * H].reshape(H, W)
            b = wbits[plane:plane + W * H].reshape(H, W)
            print("  ref row:", "".join(str(int(v)) for v in b[r]))
            print("  diff pixels:", int((a != b).sum()), "of", a.size)
            # Luma values around the first differing column for both planes.
            cols = [c - 3, c - 2, c - 1, c, c + 1, c + 2, c + 3]
            cols = [cc for cc in cols if 0 <= cc < W]
            print("  col      :", " ".join(f"{cc:4d}" for cc in cols))
            print("  rtl prev :",
                  " ".join(f"{int(got_gray[f - 1, r, cc]):4d}" for cc in cols))
            print("  rtl cur  :",
                  " ".join(f"{int(got_gray[f, r, cc]):4d}" for cc in cols))
            print("  ref prev :",
                  " ".join(f"{int(luma[f - 1, r, cc]):4d}" for cc in cols))
            print("  ref cur  :",
                  " ".join(f"{int(luma[f, r, cc]):4d}" for cc in cols))
            return 0
    print("no byte difference")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
