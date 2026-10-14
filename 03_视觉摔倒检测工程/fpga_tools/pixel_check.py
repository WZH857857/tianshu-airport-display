"""Byte-exact check of the pixel front end against the Python prototype.

Streams RGB565 frames (quantised from the decoded video) through
``gray_front`` + ``frame_diff`` in Icarus Verilog, then compares the emitted
320x180 luma frames and the packed motion masks against a NumPy reference
that mirrors the RTL operations exactly:

    RGB565 -> 888 by bit replication -> (77R+150G+29B)>>8
           -> 2x2 box (sum of four)>>2
           -> keep rows 0,1,2,4,5,6,... (3 of every 4)
           -> |cur - prev| > 10

It also reports how much the RGB565 quantisation moves the luma away from
the RGB888 prototype (the offline detector still runs on RGB888).

Usage::

    python pixel_check.py [--clips a.mp4 b.mp4] [--frames 5]
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import imageio.v3 as iio
import numpy as np

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(ROOT / "tools"))

from fall_proto import to_gray, downsample_to  # noqa: E402

GEN = FPGA / "sim" / "gen"
RTL = [FPGA / "rtl" / "gray_front.v", FPGA / "rtl" / "frame_diff.v"]
TB = FPGA / "tb" / "tb_pixel_front.v"

# gray_front's vertical gate, in one place so every checker agrees with the
# RTL: the 2x2 box makes 240 rows and the mod-4 phase drops rows 3,7,11,...
CAMERA_ROWS = 240
KEEP_ROWS = np.array([y for y in range(CAMERA_ROWS) if y % 4 != 3])

DEFAULT_CLIPS = [
    ROOT / "datasets" / "gmdcsa24" / "Subject 1" / "Fall" / "13.mp4",
    ROOT / "datasets" / "gmdcsa24" / "Subject 4" / "Fall" / "02.mp4",
]


def find_tool(name: str) -> str:
    found = shutil.which(name)
    if found:
        return found
    # The install directory is not always on PATH (a fresh shell may not have
    # it), so probe the usual spots before giving up.
    for root in (r"E:\iverilog\bin", r"C:\iverilog\bin"):
        cand = Path(root) / f"{name}.exe"
        if cand.exists():
            return str(cand)
    raise SystemExit(f"error: {name} not found")


def quantise565(frame: np.ndarray) -> np.ndarray:
    """RGB888 -> RGB565, packed as {R[4:0], G[5:0], B[4:0]}."""
    r5 = (frame[..., 0].astype(np.uint16) >> 3)
    g6 = (frame[..., 1].astype(np.uint16) >> 2)
    b5 = (frame[..., 2].astype(np.uint16) >> 3)
    return ((r5 << 11) | (g6 << 5) | b5).astype(np.uint16)


def luma_from_565(pix: np.ndarray) -> np.ndarray:
    """RTL luma path: expand by bit replication, then 77/150/29 >> 8."""
    r5 = (pix >> 11) & 0x1F
    g6 = (pix >> 5) & 0x3F
    b5 = pix & 0x1F
    r8 = ((r5 << 3) | (r5 >> 2)).astype(np.uint32)
    g8 = ((g6 << 2) | (g6 >> 4)).astype(np.uint32)
    b8 = ((b5 << 3) | (b5 >> 2)).astype(np.uint32)
    return ((77 * r8 + 150 * g8 + 29 * b8) >> 8).astype(np.uint8)


def down2_box(gray: np.ndarray) -> np.ndarray:
    acc = (gray[0::2, 0::2].astype(np.uint16) + gray[0::2, 1::2] +
           gray[1::2, 0::2] + gray[1::2, 1::2])
    return (acc >> 2).astype(np.uint8)


def reference(frames: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    # gray_front applies a 3-of-4 row gate after the 2x2 box, so the stored
    # frames are 320x180.  Row 3,7,11,... is the dropped one.
    luma = np.stack([down2_box(luma_from_565(quantise565(f)))[KEEP_ROWS]
                     for f in frames])
    masks = np.zeros_like(luma, dtype=bool)
    for i in range(1, len(luma)):
        diff = np.abs(luma[i].astype(np.int16) - luma[i - 1].astype(np.int16))
        masks[i] = diff > 10
    packed = np.stack([np.packbits(m, bitorder="big") for m in masks])
    return luma, packed


def first_diff(a: bytes, b: bytes) -> str:
    n = min(len(a), len(b))
    for i in range(n):
        if a[i] != b[i]:
            return f"first mismatch at byte {i} (len {len(a)} vs {len(b)})"
    if len(a) != len(b):
        return f"length mismatch {len(a)} vs {len(b)}"
    return "no difference"


def check_clip(vvp: str, clip: Path, nframes: int) -> bool:
    frames: list[np.ndarray] = []
    for i, frame in enumerate(iio.imiter(clip, plugin="FFMPEG")):
        if i >= nframes:
            break
        frames.append(np.asarray(frame))
    if not frames:
        print(f"SKIP {clip.name}: no frames")
        return True
    h, w = frames[0].shape[:2]
    if (w, h) != (640, 480):
        print(f"SKIP {clip.name}: {w}x{h} is not the 640x480 camera path")
        return True
    if len(frames) < nframes:
        nframes = len(frames)

    tag = clip.stem.replace(" ", "_")
    in_path = GEN / f"pixel_{tag}.565"
    gray_path = GEN / f"pixel_{tag}.gray.bin"
    mask_path = GEN / f"pixel_{tag}.mask.bin"

    pix = np.stack([quantise565(f) for f in frames])
    in_path.write_bytes(pix.astype("<u2").tobytes())

    want_gray, want_mask = reference(frames)
    model = GEN / "sim_pixel_front.vvp"
    cmd = [find_tool("iverilog"), "-g2005", "-o", str(model),
           *[str(p) for p in RTL], str(TB)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"iverilog failed:\n{proc.stdout}\n{proc.stderr}")
    proc = subprocess.run(
        [vvp, str(model), f"+IN={in_path}", f"+GRAY={gray_path}",
         f"+MASK={mask_path}", f"+FRAMES={nframes}"],
        capture_output=True, text=True)
    if "DONE" not in proc.stdout:
        raise SystemExit(f"simulation failed:\n{proc.stdout}\n{proc.stderr}")

    got_gray = gray_path.read_bytes()
    got_mask = mask_path.read_bytes()
    want_gray_b = want_gray.tobytes()
    want_mask_b = want_mask.tobytes()

    ok_gray = got_gray == want_gray_b
    ok_mask = got_mask == want_mask_b

    proto = np.stack([downsample_to(to_gray(f), 320, 240)[0][
                          KEEP_ROWS]
                      for f in frames])
    delta = np.abs(proto.astype(np.int16) - want_gray.astype(np.int16))
    diff_pct = float((delta > 0).mean() * 100.0)

    status = "PASS" if (ok_gray and ok_mask) else "FAIL"
    print(f"{status} {clip.name}: {nframes} frames")
    print(f"     luma bytes {'match' if ok_gray else first_diff(got_gray, want_gray_b)}")
    print(f"     mask bytes {'match' if ok_mask else first_diff(got_mask, want_mask_b)}")
    print(f"     RGB565 vs RGB888 luma: mean|d|={delta.mean():.3f} "
          f"max={delta.max()} nonzero={diff_pct:.1f}%")
    return ok_gray and ok_mask


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clips", type=Path, nargs="*", default=DEFAULT_CLIPS)
    parser.add_argument("--frames", type=int, default=5)
    args = parser.parse_args()

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
