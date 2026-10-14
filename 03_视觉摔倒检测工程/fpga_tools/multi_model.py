"""Bit-exact Python model of multi_bbox.v, for debugging the RTL.

The RTL keeps the row projection on the write side (full mask, no
exclusions), freezes one band per frame, and then runs four exclusion passes
inside that band; each pass then trims its box vertically by scanning the
rows of the column run it just found.  A reference that recomputes the band
per pass silently diverges, so this model reproduces the RTL's exact order of
operations:

  band  -- from the whole mask, once per frame
  pass  -- column histogram of band minus excluded rectangles, 2-cycle scan
  trim  -- row histogram of the chosen column run, same longest-run rule
  freeze -- trimmed box + MARGIN becomes the next slot's exclusion rectangle

It is used by ``multi_bbox_check.py --model`` to bisect RTL/reference
differences down to a single pass.
"""

from __future__ import annotations

import numpy as np

# rtl/multi_bbox.v drives the row gate and the column gate from one parameter
# (MIN_PIX), so the two numbers here have to stay equal.  The values are the
# shipped board configuration: fall_camera_hdmi_multi_top passes 4/160 and
# fall_core_multi_top now carries the same defaults.
MIN_ROW_PIX = 4
MIN_PIX = 4
MIN_AREA = 160
NUM_BLOBS = 4
MARGIN = 8
# Blobs within this many pixels of the full raster are rejected outright:
# they are the whole-frame smear the crowd clips produce, not a person.
SPAN_MARGIN = 8


def band_of(mask: np.ndarray) -> tuple[int, int] | None:
    rows = mask.sum(axis=1)
    active = rows >= MIN_ROW_PIX
    best = None
    run = 0
    for y, a in enumerate(active):
        if a:
            run += 1
            if best is None or run > best[0]:
                best = (run, y - run + 1)
        else:
            run = 0
    if best is None:
        return None
    return (best[1], best[1] + best[0])


def scan(profile: np.ndarray) -> tuple[int, int, int]:
    """Longest run of columns with count >= MIN_PIX, plus its summed count."""
    active = profile >= MIN_PIX
    best_len = 0
    best_start = 0
    best_sum = 0
    run_len = 0
    run_start = 0
    run_sum = 0
    for x, a in enumerate(active):
        if a:
            if run_len == 0:
                run_start = x
                run_sum = 0
            run_len += 1
            run_sum += int(profile[x])
            if run_len > best_len:
                best_len = run_len
                best_start = run_start
                best_sum = run_sum
        else:
            run_len = 0
            run_sum = 0
    return best_start, best_len, best_sum


def project_band(mask: np.ndarray, y0: int, y1: int,
                 boxes: list[tuple[int, int, int, int]]
                 ) -> tuple[int, int, int, int, int, int]:
    band = mask[y0:y1].copy()
    for bx0, by0, bx1, by1 in boxes:
        lo = max(by0, y0) - y0
        hi = min(by1, y1) - y0
        if hi > lo:
            band[lo:hi, bx0:bx1] = False
    bh = y1 - y0
    x, w, _ = scan(band.sum(axis=0))
    if w == 0:
        return (0, 0, y0, 0, bh, 0)
    # Vertical trim: inside the chosen column run, keep the longest run of
    # rows that carry at least MIN_PIX of the run's pixels.  This is what
    # stops a box from wrapping every person the frame-wide band contains.
    ty, th, tsum = scan(band[:, x:x + w].sum(axis=1))
    frame_h, frame_w = mask.shape
    spans = (w >= frame_w - SPAN_MARGIN) or (th >= frame_h - SPAN_MARGIN)
    found = 1 if (th != 0 and tsum >= MIN_AREA and not spans) else 0
    return (found, x, y0 + ty, w, th, tsum)


def frames_model(frames: np.ndarray) -> list[list[tuple[int, ...]]]:
    h, w = frames.shape[1:]
    ex_valid = np.zeros(NUM_BLOBS, dtype=bool)
    ex_box = np.zeros((NUM_BLOBS, 4), dtype=np.int64)
    out = []
    for mask in frames:
        band = band_of(mask)
        if band is None:
            out.append([(0, 0, 0, 0, 0, 0)] * NUM_BLOBS)
            continue
        y0, y1 = band
        slots = []
        for k in range(NUM_BLOBS):
            boxes = [tuple(int(v) for v in ex_box[j])
                     for j in range(k) if ex_valid[j]]
            res = project_band(mask, y0, y1, boxes)
            slots.append(res)
            if res[0]:
                # Same rectangle convention as multi_bbox.v: left/top stay on
                # the box, right/bottom grow by MARGIN.
                ex_valid[k] = True
                ex_box[k] = (res[1], res[2],
                             min(w, res[1] + res[3] + MARGIN),
                             min(h, res[2] + res[4] + MARGIN))
            else:
                ex_valid[k] = False
        out.append(slots)
    return out
