"""Check proj_bbox against the Python two-pass projection reference.

Takes the same masks as ``morph_check.py`` (synthetic patterns by default,
optionally video-derived), applies the SciPy morphology, feeds the morphed
masks through ``proj_bbox`` in Icarus Verilog and compares the per-frame
``found x y w h area`` values with ``FallDetector.bbox_of``'s reference
implementation built on ``largest_span``.

Usage::

    python bbox_check.py [--clips a.mp4 ...] [--frames 6]
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
sys.path.insert(0, str(ROOT / "tools"))

from fall_proto import largest_span  # noqa: E402
from morph_check import (  # noqa: E402
    GEN, W, H, find_tool, masks_from_clip, morph_ref, pack_masks,
    synthetic_masks,
)

RTL = [FPGA / "rtl" / "proj_bbox.v"]
TB = FPGA / "tb" / "tb_proj_bbox.v"

MIN_PIX = 3
MIN_AREA = 200


def bbox_ref(mask: np.ndarray) -> tuple[int, int, int, int, int, int]:
    """Reference: (found, x, y, w, h, area)."""
    if not mask.any():
        return (0, 0, 0, 0, 0, 0)
    span_y = largest_span(mask.sum(axis=1), MIN_PIX)
    if span_y is None:
        return (0, 0, 0, 0, 0, 0)
    y0, y1 = span_y
    band = mask[y0:y1]
    span_x = largest_span(band.sum(axis=0), MIN_PIX)
    if span_x is None:
        return (0, 0, 0, 0, 0, 0)
    x0, x1 = span_x
    area = int(band[:, x0:x1].sum())
    found = 1 if area >= MIN_AREA else 0
    return (found, x0, y0, x1 - x0, y1 - y0, area)


def run_rtl(masks: np.ndarray, tag: str, vvp: str) -> list[tuple[int, ...]]:
    n = len(masks)
    in_path = GEN / f"bbox_{tag}.in"
    out_path = GEN / f"bbox_{tag}.out"
    in_path.write_bytes(pack_masks(masks))
    model = GEN / "sim_proj_bbox.vvp"
    cmd = [find_tool("iverilog"), "-g2005", "-o", str(model),
           *[str(p) for p in RTL], str(TB)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"iverilog failed:\n{proc.stdout}\n{proc.stderr}")
    proc = subprocess.run(
        [vvp, str(model), f"+IN={in_path}", f"+OUT={out_path}",
         f"+FRAMES={n}"], capture_output=True, text=True)
    if "DONE" not in proc.stdout:
        raise SystemExit(f"simulation failed:\n{proc.stdout}\n{proc.stderr}")
    rows = []
    for line in out_path.read_text().splitlines():
        if line.strip():
            rows.append(tuple(int(v) for v in line.split()))
    return rows


def check_set(name: str, masks: np.ndarray, vvp: str) -> bool:
    morphed = np.stack([morph_ref(m) for m in masks])
    want = [bbox_ref(m) for m in morphed]
    got = run_rtl(morphed, name, vvp)
    ok = len(got) == len(want) and all(g == w for g, w in zip(got, want))
    status = "PASS" if ok else "FAIL"
    print(f"{status} {name}: {len(want)} frames, {len(got)} results")
    if not ok:
        for i, (g, w) in enumerate(zip(got, want)):
            if g != w:
                print(f"     frame {i}: got {g} want {w}")
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
