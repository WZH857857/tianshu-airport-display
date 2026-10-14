"""Bit-exact Python model of rtl/motion_hist.v.

The module keeps a 2-bit "recent motion" map at 80x45 (one cell per 4x4
vision pixels) behind the tracker:

  * motion sets a cell to 3, a sweep drops every cell by one every DECAY
    frames -- but only while the frame's motion was big enough to be a
    person (MIN_GLOBAL cells), so a person who sits still keeps his map
    instead of fading out;
  * a cell only accepts motion that falls inside a window grown around one
    of the previous frame's tracked boxes, so a flickering wall edge cannot
    paint the map;
  * per slot, the bounding box of the live cells in that window is what the
    overlay draws the stick figure in.  When a slot's track is alive but the
    window has too few cells, the slot keeps its last box (the track is a
    person, so the box should not blink); it is dropped HOLD_MAX frames
    after the track itself died.

Units: ``cm`` is per-cell motion counts, boxes are (found, x, y, w, h) in
vision pixels (320x180).  Everything below is integer arithmetic so the RTL
can match it exactly.
"""

from __future__ import annotations

import numpy as np

CW, CH = 80, 45          # cell raster
CELL = 4                 # vision pixels per cell
NSLOT = 4

# Must match the motion_hist instance in fall_core_multi_top.v.
DECAY = 8                # frames between decay sweeps
MIN_CELL_PIX = 2         # mask pixels that light a cell
MIN_GLOBAL = 30          # cells of motion before the map may age
MIN_CELLS = 5            # live cells before a window's box is trusted
# A dead slot drops its figure at once: the tracker already holds a matched
# track for 120 frames, and stacking our own hold on top of that left three
# skeletons standing over one person in the fall clip.
HOLD_MAX = 0             # frames a dead slot keeps its last box
WIN_HW_MIN = 7           # half window width, cells
WIN_HH_MIN = 13          # half window height, cells
HW_MAX = 5               # tallest upright figure, as w * HW_MAX / 2


def windows(boxes):
    """Grow a cell-space window around each published box.

    The window leans upwards: a tracked blob is usually the legs or an arm,
    and the rest of the person -- the part the stick figure needs -- is above
    it.  ``up`` is 1.5 half-heights, ``down`` 0.5, so the window is twice as
    tall as half_h and sits mostly above the blob.
    """
    out = []
    for s in boxes:
        found, bx, by, bw, bh = s[0], s[1], s[2], s[3], s[4]
        if not found:
            out.append(None)
            continue
        cx = bx + (bw >> 1)
        cy = by + (bh >> 1)
        hw = max(WIN_HW_MIN, bw >> 2)
        hh = max(WIN_HH_MIN, (bh >> 2) + (bh >> 3))
        up = hh + (hh >> 1)
        dn = hh >> 1
        x0 = max(0, (cx >> 2) - hw)
        x1 = min(CW, (cx >> 2) + hw)
        y0 = max(0, (cy >> 2) - up)
        y1 = min(CH, (cy >> 2) + dn)
        out.append((x0, x1, y0, y1))
    return out


def run(cm, tracked, maps=None):
    """Replay the clip through the history map.

    ``cm``      -- (n, CH, CW) motion counts per cell
    ``tracked`` -- per frame, four (found, x, y, w, h) published boxes
    ``maps``    -- optional list; the map left behind by every frame is
                   appended to it (frame i leaves the map the sweep of
                   frame i reads)

    Returns ``(pbox, hist)`` where ``pbox[i][k]`` is either None or
    (x, y, w, h, cells) in vision pixels for slot k of frame i, and ``hist``
    is the map left behind by the last frame.
    """
    hist = np.zeros((CH, CW), dtype=np.int8)
    pbox = []
    age = [0] * NSLOT
    held = [None] * NSLOT
    dec = 0
    zero = [(0, 0, 0, 0, 0)] * NSLOT

    for i in range(len(cm)):
        boxes = tracked[i - 1] if i else zero
        wins = windows(boxes)

        rows = []
        for k in range(NSLOT):
            w = wins[k]
            if w is None:
                # Slot was not alive last frame: age the held box out.
                age[k] += 1
                if age[k] > HOLD_MAX:
                    held[k] = None
                rows.append(held[k])
                continue
            x0, x1, y0, y1 = w
            sub = hist[y0:y1, x0:x1] >= 1
            ys, xs = np.nonzero(sub)
            if xs.size < MIN_CELLS:
                rows.append(held[k])
                continue
            age[k] = 0
            bx, by, bw, bh = boxes[k][1], boxes[k][2], boxes[k][3], boxes[k][4]
            px0 = min((xs.min() + x0) * CELL, bx)
            py0 = min((ys.min() + y0) * CELL, by)
            px1 = max((xs.max() + x0 + 1) * CELL, bx + bw)
            py1 = max((ys.max() + y0 + 1) * CELL, by + bh)
            pw = int(px1 - px0)
            ph = int(py1 - py0)
            # A motion trail keeps where the person *was*: after a fall from
            # standing, the box still reaches up to the old head height.  A
            # person stands on the floor, so cap the height and keep the
            # bottom edge -- that trims the stale part and leaves the feet.
            if pw <= ph and ph > (pw * HW_MAX) >> 1:
                ph = (pw * HW_MAX) >> 1
                py0 = py1 - ph
            pb = (int(px0), int(py0), pw, ph, int(xs.size))
            held[k] = pb
            rows.append(pb)
        pbox.append(rows)

        # Write side: motion inside a live window sets the cell to 3.
        live = np.zeros((CH, CW), dtype=bool)
        for w in wins:
            if w is not None:
                live[w[2]:w[3], w[0]:w[1]] = True
        hit = (cm[i] >= MIN_CELL_PIX) & live
        hist = np.where(hit, np.int8(3), hist)

        # End of frame: age the map once the motion was person-sized.
        glob = int((cm[i] >= MIN_CELL_PIX).sum())
        dec = (dec + 1) % DECAY
        if dec == 0 and glob >= MIN_GLOBAL:
            hist = np.maximum(hist - 1, 0)
        if maps is not None:
            maps.append(hist.copy())

    return pbox, hist
