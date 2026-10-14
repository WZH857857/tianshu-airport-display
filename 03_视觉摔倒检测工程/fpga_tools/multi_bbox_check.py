"""Check multi_bbox against the Python exclusion-pass reference.

The RTL finds up to NUM_BLOBS boxes by re-scanning the mask and excluding the
rectangles already claimed, so the reference has to reproduce that sequence
exactly -- margin included -- or the comparison is meaningless.  This driver

  1. replays the same morphed masks as ``bbox_check.py`` through Icarus and
     requires the four slots to match the reference bit for bit, then
  2. reports how those boxes compare with a true 8-connected labelling, which
     is what the FPGA cannot afford (a real crowd frame holds 700-1500 runs,
     and storing them needs a 3-4 KB run array plus union-find).

Usage::

    python multi_bbox_check.py [--clips a.mp4 ...] [--frames 6]
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
    GEN, H, W, find_tool, masks_from_clip, morph_ref, pack_masks,
    synthetic_masks,
)

RTL = [FPGA / "rtl" / "multi_bbox.v"]
TB = FPGA / "tb" / "tb_multi_bbox.v"

# Must equal the defaults in rtl/multi_bbox.v (which are the shipped board
# values: fall_camera_hdmi_multi_top passes 4/160 through fall_core_multi_top).
MIN_PIX = 4
MIN_AREA = 160
NUM_BLOBS = 4
MARGIN = 8
# Blobs within this many pixels of the full raster are rejected (see the
# matching SPAN_*_LIMIT in multi_bbox.v).
SPAN_MARGIN = 8


def band_of(mask: np.ndarray) -> tuple[int, int] | None:
    """The row run the stream side picks, from the *whole* mask.

    multi_bbox.v computes this once per frame on the write side, before any
    exclusion is applied, and every pass then works inside that one band.
    """
    span = largest_span(mask.sum(axis=1), MIN_PIX)
    if span is None:
        return None
    return span


def project_in_band(mask: np.ndarray, y0: int, y1: int,
                    boxes: list[tuple[int, int, int, int]]
                    ) -> tuple[int, int, int, int, int, int]:
    """One pass: column run inside the fixed band, minus excluded rectangles.

    The box is then trimmed vertically -- the same second scan multi_bbox.v
    runs in ST_VSPAN -- so the published height is the mover's, not the
    band's.
    """
    band = mask[y0:y1].copy()
    for bx0, by0, bx1, by1 in boxes:
        lo = max(by0, y0) - y0
        hi = min(by1, y1) - y0
        if hi > lo:
            band[lo:hi, bx0:bx1] = False
    bh = y1 - y0
    span_x = largest_span(band.sum(axis=0), MIN_PIX)
    if span_x is None:
        return (0, 0, y0, 0, bh, 0)
    x0, x1 = span_x
    span_y = largest_span(band[:, x0:x1].sum(axis=1), MIN_PIX)
    if span_y is None:
        return (0, x0, y0, x1 - x0, 0, 0)
    y0b, y1b = span_y
    area = int(band[y0b:y1b, x0:x1].sum())
    frame_h, frame_w = mask.shape
    spans = ((x1 - x0 >= frame_w - SPAN_MARGIN) or
             (y1b - y0b >= frame_h - SPAN_MARGIN))
    found = 1 if (area >= MIN_AREA and not spans) else 0
    return (found, x0, y0 + y0b, x1 - x0, y1b - y0b, area)


def multi_ref(frames: np.ndarray) -> list[list[tuple[int, ...]]]:
    """Exclusion-pass reference, mirroring multi_bbox.v exactly.

    The exclusion rectangles survive across frames (a slot that finds nothing
    keeps last frame's box), so this is stateful exactly like the RTL.
    """
    h, w = frames.shape[1:]
    ex_valid = np.zeros(NUM_BLOBS, dtype=bool)
    ex_box = np.zeros((NUM_BLOBS, 4), dtype=np.int64)
    out: list[list[tuple[int, ...]]] = []

    for f in range(len(frames)):
        mask = frames[f]
        frame_res: list[tuple[int, ...]] = []
        band = band_of(mask)
        if band is None:
            # No vertical band: every slot is empty and the exclusion table
            # keeps last frame's rectangles untouched.
            out.append([(0, 0, 0, 0, 0, 0)] * NUM_BLOBS)
            continue
        y0, y1 = band
        for k in range(NUM_BLOBS):
            boxes: list[tuple[int, int, int, int]] = []
            for j in range(k):
                if not ex_valid[j]:
                    continue
                boxes.append(tuple(int(v) for v in ex_box[j]))
            res = project_in_band(mask, y0, y1, boxes)
            frame_res.append(res)
            found, x, y, bw, bh = res[0], res[1], res[2], res[3], res[4]
            if found:
                # Rectangle convention, mirroring multi_bbox.v: the left/top
                # edge stays on the box, while the right/bottom edge is
                # pushed out by MARGIN.  A slot that scores this frame
                # replaces its rectangle outright; only a slot that comes up
                # empty keeps last frame's box as a dead zone.
                ex_valid[k] = True
                ex_box[k] = (x, y, min(w, x + bw + MARGIN),
                             min(h, y + bh + MARGIN))
            else:
                ex_valid[k] = False
        out.append(frame_res)
    return out


def cc_blobs(mask: np.ndarray) -> list[tuple[int, int, int, int, int]]:
    """Ground-truth 8-connected blobs: (area, x, y, w, h), strongest first."""
    from scipy import ndimage
    labels, _ = ndimage.label(mask, structure=np.ones((3, 3), bool))
    blobs = []
    for label, slc in enumerate(ndimage.find_objects(labels), start=1):
        if slc is None:
            continue
        local = mask[slc] & (labels[slc] == label)
        area = int(local.sum())
        if area < MIN_AREA:
            continue
        ys, xs = np.nonzero(local)
        if ys.size == 0:
            continue
        bh = int(ys.max()) - int(ys.min()) + 1
        if bh < 14:
            continue
        bw = int(xs.max()) - int(xs.min()) + 1
        blobs.append((area, int(slc[1].start), int(slc[0].start), bw, bh))
    blobs.sort(reverse=True)
    return blobs


def run_rtl(masks: np.ndarray, tag: str, vvp: str) -> list[list[tuple[int, ...]]]:
    n = len(masks)
    in_path = GEN / f"multi_{tag}.in"
    out_path = GEN / f"multi_{tag}.out"
    in_path.write_bytes(pack_masks(masks))
    model = GEN / "sim_multi_bbox.vvp"
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
    if len(rows) != n * NUM_BLOBS:
        raise SystemExit(f"expected {n * NUM_BLOBS} rows, got {len(rows)}")
    return [rows[i * NUM_BLOBS:(i + 1) * NUM_BLOBS] for i in range(n)]


def iou(a: tuple[int, ...], b: tuple[int, ...]) -> float:
    ax, ay, aw, ah = a[1], a[2], a[3], a[4]
    bx, by, bw, bh = b[1], b[2], b[3], b[4]
    x0, y0 = max(ax, bx), max(ay, by)
    x1, y1 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    union = aw * ah + bw * bh - inter
    return inter / union if union else 0.0


def check_set(name: str, masks: np.ndarray, vvp: str) -> bool:
    morphed = np.stack([morph_ref(m) for m in masks])
    want = multi_ref(morphed)
    got = run_rtl(morphed, name, vvp)

    ok = len(got) == len(want)
    if ok:
        for f, (gslots, wslots) in enumerate(zip(got, want)):
            if list(gslots) != list(wslots):
                ok = False
                print(f"  frame {f}: got {gslots}")
                print(f"           want {wslots}")
                break
    print(f"{'PASS' if ok else 'FAIL'} {name}: RTL vs exclusion reference, "
          f"{len(want)} frames x {NUM_BLOBS} slots")

    # Secondary information: how the exclusion boxes line up with a true
    # 8-connected labelling.  This is a measurement, not a pass criterion --
    # merged blobs can legitimately differ.
    hit, total = 0, 0
    ious = []
    for mask, slots in zip(morphed, got):
        truth = cc_blobs(mask)
        for k, slot in enumerate(slots):
            if not slot[0]:
                continue
            total += 1
            if k < len(truth):
                v = iou(slot, (0, truth[k][1], truth[k][2],
                               truth[k][3], truth[k][4]))
                ious.append(v)
                if v >= 0.5:
                    hit += 1
    mean_iou = float(np.mean(ious)) if ious else 0.0
    print(f"     vs 8-connected truth: {hit}/{total} slots IoU>=0.5, "
          f"mean IoU {mean_iou:.3f}")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--clips", type=Path, nargs="*", default=[])
    ap.add_argument("--frames", type=int, default=6)
    args = ap.parse_args()

    vvp = find_tool("vvp")
    GEN.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(2026)
    ok = check_set("synthetic", synthetic_masks(rng), vvp)

    # The crowd case: a mask that fills the raster.  Every slot has to stay
    # empty -- publishing a frame-filling box would hide the real people --
    # while the small block in frame 2 must still survive the guard.
    span = np.zeros((4, H, W), dtype=bool)
    span[0] = True
    span[2, 40:110, 30:90] = True
    span[3] = True
    ok &= check_set("span", span, vvp)

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
