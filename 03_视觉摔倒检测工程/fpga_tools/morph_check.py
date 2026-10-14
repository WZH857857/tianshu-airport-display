"""Byte-exact check of motion_morph against the NumPy/SciPy prototype.

Feeds packed 320x180 binary masks (the frame_diff output) through
``motion_morph`` in Icarus Verilog and compares the packed output with::

    if mask.any():
        mask = binary_erosion(mask, structure=disk(1), border_value=0)
        mask = binary_dilation(mask, structure=disk(3), border_value=0)

The radii live in ERODE_R / DILATE_R below and must match the localparams in
rtl/motion_morph.v.

The default test set is a group of synthetic patterns that covers borders,
isolated pixels and random blobs.  Pass ``--clips`` to also replay real
video-derived masks.

Usage::

    python morph_check.py [--clips a.mp4 ...] [--frames 5]
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import imageio.v3 as iio
import numpy as np
from scipy import ndimage

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(ROOT / "tools"))

from fall_proto import disk  # noqa: E402
from pixel_check import (  # noqa: E402
    KEEP_ROWS, down2_box, luma_from_565, quantise565,
)

GEN = FPGA / "sim" / "gen"
RTL = [FPGA / "rtl" / "motion_morph.v"]
TB = FPGA / "tb" / "tb_motion_morph.v"

W, H = 320, 180

# Must match rtl/motion_morph.v (disk(1) erosion + radius-3 disk dilation).
ERODE_R = 1
DILATE_R = 3

# Stream shaper for the RTL replay.  The default is a dense stream; the real
# chain delivers one de_in pulse every second clock with long gaps between
# rows, so set GAP/ROW_GAP to reproduce that timing.
GAP = 0
ROW_GAP = None


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


def morph_ref(mask: np.ndarray) -> np.ndarray:
    if not mask.any():
        return np.zeros_like(mask)
    out = ndimage.binary_erosion(mask, structure=disk(ERODE_R), border_value=0)
    return ndimage.binary_dilation(out, structure=disk(DILATE_R),
                                   border_value=0)


def pack_masks(masks: np.ndarray) -> bytes:
    return np.stack([np.packbits(m, bitorder="big")
                     for m in masks]).tobytes()


def unpack_masks(raw: bytes) -> np.ndarray:
    if len(raw) % (W // 8):
        raise ValueError(f"output size {len(raw)} is not a whole number of rows")
    arr = np.frombuffer(raw, dtype=np.uint8)
    arr = arr.reshape(len(raw) // (W // 8), W // 8)
    return np.unpackbits(arr, axis=1, bitorder="big").astype(bool)


def synthetic_masks(rng: np.random.Generator) -> np.ndarray:
    """Patterns chosen to hit every border, isolated pixels and blobs."""
    frames = []
    zero = np.zeros((H, W), dtype=bool)

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
    blocks[170:171, 40:80] = True   # 1-pixel-high ridge, must erode away
    frames.append(blocks)

    # Disk-shaped blobs of several sizes, including ones clipped by borders.
    blobs = zero.copy()
    yy, xx = np.ogrid[:H, :W]
    for cy, cx, r in [(60, 60, 3), (60, 200, 8), (150, 40, 15),
                      (0, 160, 6), (H - 1, 300, 6), (120, 319, 5)]:
        blobs |= ((yy - cy) ** 2 + (xx - cx) ** 2) <= r * r
    frames.append(blobs)

    # Random salt-and-pepper plus random rectangles: worst-case noise.
    noise = rng.random((H, W)) < 0.02
    for _ in range(6):
        y0 = int(rng.integers(0, H - 10))
        x0 = int(rng.integers(0, W - 10))
        noise[y0:y0 + int(rng.integers(3, 12)),
              x0:x0 + int(rng.integers(3, 12))] = True
    frames.append(noise)

    frames.append(zero.copy())          # skip-path: all-zero mask
    frames.append(noise.copy())         # repeat to exercise stack reuse
    return np.stack(frames)


def masks_from_clip(clip: Path, nframes: int) -> np.ndarray | None:
    gray = []
    for i, frame in enumerate(iio.imiter(clip, plugin="FFMPEG")):
        if i >= nframes:
            break
        frame = np.asarray(frame)
        # The verified RTL front end is the 640x480 camera path; normalise
        # other source sizes with nearest-neighbour sampling so the 2x2 box
        # below still lands on 320x240, then apply the same 3-of-4 row gate
        # gray_front uses to reach 320x180.
        h, w = frame.shape[:2]
        if (w, h) != (640, 480):
            ys = np.minimum(np.arange(480) * h // 480, h - 1)
            xs = np.minimum(np.arange(640) * w // 640, w - 1)
            frame = frame[ys][:, xs]
        small_888 = luma_from_565(quantise565(frame))
        small = down2_box(small_888)
        gray.append(small[KEEP_ROWS])
    if len(gray) < 2:
        return None
    gray = np.stack(gray)
    masks = np.zeros_like(gray, dtype=bool)
    for i in range(1, len(gray)):
        diff = np.abs(gray[i].astype(np.int16) - gray[i - 1].astype(np.int16))
        masks[i] = diff > 10
    return masks


def run_rtl(masks: np.ndarray, tag: str, vvp: str) -> np.ndarray:
    n = len(masks)
    in_path = GEN / f"morph_{tag}.in"
    out_path = GEN / f"morph_{tag}.out"
    in_path.write_bytes(pack_masks(masks))
    model = GEN / "sim_motion_morph.vvp"
    cmd = [find_tool("iverilog"), "-g2005", "-o", str(model),
           *[str(p) for p in RTL], str(TB)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"iverilog failed:\n{proc.stdout}\n{proc.stderr}")
    plusargs = [f"+IN={in_path}", f"+OUT={out_path}", f"+FRAMES={n}"]
    if GAP:
        plusargs.append(f"+GAP={GAP}")
    if ROW_GAP is not None:
        plusargs.append(f"+ROW_GAP={ROW_GAP}")
    proc = subprocess.run([vvp, str(model), *plusargs],
                          capture_output=True, text=True)
    if "DONE" not in proc.stdout:
        raise SystemExit(f"simulation failed:\n{proc.stdout}\n{proc.stderr}")
    nrows = int(proc.stdout.split("rows=")[1].split()[0])
    return unpack_masks(out_path.read_bytes()), nrows


def diagnose(got: np.ndarray, want: np.ndarray) -> str:
    lines = []
    if got.shape != want.shape:
        lines.append(f"shape {got.shape} vs {want.shape}")
        return "; ".join(lines)
    diff = got != want
    total = int(diff.sum())
    if total == 0:
        return "no difference"
    first = np.argwhere(diff)[0]
    extra = int((got & ~want).sum())
    missing = int((~got & want).sum())
    lines.append(f"{total} bit errors (extra={extra} missing={missing}), "
                 f"first at row {first[0]} col {first[1]}")
    g_rows = got.sum(axis=1)
    w_rows = want.sum(axis=1)
    bad_rows = np.nonzero(g_rows - w_rows)[0]
    lines.append(f"rows with count mismatch: {len(bad_rows)} "
                 f"(first {bad_rows[:5].tolist()})")
    return "; ".join(lines)


def check_set(name: str, masks: np.ndarray, vvp: str) -> bool:
    want = np.stack([morph_ref(m) for m in masks])
    want = want.reshape(-1, W)
    got, nrows = run_rtl(masks, name, vvp)
    ok = got.shape == want.shape and np.array_equal(got, want)
    status = "PASS" if ok else "FAIL"
    print(f"{status} {name}: {len(masks)} frames, "
          f"{nrows} de_out rows (want {len(masks) * H})")
    if not ok:
        print(f"     {diagnose(got, want)}")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clips", type=Path, nargs="*", default=[])
    parser.add_argument("--frames", type=int, default=6)
    args = parser.parse_args()

    vvp = find_tool("vvp")
    GEN.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(2026)
    ok = check_set("synthetic", synthetic_masks(rng), vvp)

    for clip in args.clips:
        if not clip.exists():
            print(f"SKIP missing {clip}")
            continue
        masks = masks_from_clip(clip, args.frames)
        if masks is None:
            print(f"SKIP {clip.name}: not enough frames")
            continue
        tag = "clip_" + clip.stem.replace(" ", "_")[:24]
        ok &= check_set(tag, masks, vvp)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
