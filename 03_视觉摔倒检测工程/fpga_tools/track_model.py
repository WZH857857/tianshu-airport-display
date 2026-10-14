"""Reference model for track_assoc.v, second-generation hold policy.

The RTL and this model must agree on the whole association policy, not just
on the final box coordinates:

  * greedy rounds: scan i-major, j-minor; a strict cross-product winner
    replaces the current best, so equal scores keep the earlier pair;
  * a candidate is eligible only when inter != 0 and 4*inter >= union
    (IoU only -- the containment rule the first prototype tried changes no
    measured frame, so the hardware does not pay for it);
  * a slot earns the right to be held (``est``) after EST_MATCHES
    consecutive matches or one detection of at least AREA_EST pixels.
    A one-frame flicker blob never qualifies, so it dies immediately
    instead of turning into a ten-second ghost;
  * a held slot that has a matched box of the same frame within the claim
    radius is an old pose of that same person: it is retired, not published;
  * an unmatched detection adopts the lowest-index stale slot inside that
    radius, else the lowest free slot, else it evicts the stale slot with
    the least hold left.  A detection is dropped only when all four slots
    hold live tracks;
  * the radius is CLAIM_QUIET in a quiet scene (two detections or fewer) and
    CLAIM_NEAR in a crowd, so neighbours are not glued together.
  * since 2026-10-07 the published box is an *elastic lock box*, not the raw
    detection: every side the detection does not touch relaxes inward by
    SHRINK px per frame, while the box always grows instantly to cover the
    detection.  A limb-sized detection therefore settles into a body-sized
    box that keeps following the person instead of flickering at limb size.
    Association still runs on the raw detection boxes (``prev_*``), so the
    matching arithmetic above is unchanged.
    The trail is capped at LOCK_MARGIN px per side: a fast walker would
    otherwise stretch the box over its entire path.
  * since 2026-10-07 a slot that has not yet earned a full hold is still
    kept for GRACE_FRAMES after a missed detection.  Before that, one
    dropped frame in the first three killed the box outright, which is what
    made the display flicker on a person who was clearly still there.

The model consumes the ``multi_bbox`` tuples
``(found, x, y, w, h, area)`` and returns tuples in the same shape after
re-slotting.
"""

from __future__ import annotations

NUM_BLOBS = 4
MATCH_SCALE = 4
HOLD_FRAMES = 120
EST_MATCHES = 3
AREA_EST = 700
CLAIM_NEAR = 40
CLAIM_QUIET = 80
# Frames a not-yet-established track survives a detection dropout.  Six
# frames is 0.2 s at 30 fps: long enough to bridge the frame-difference
# holes on a slow-moving person, too short to leave a visible ghost.
GRACE_FRAMES = 6
# Elastic lock box (see the note above).  SHRINK is in vision pixels per
# frame: 2 px at 30 fps is 60 px/s -- fast enough that a walker's box does
# not trail, slow enough that the box never snaps between poses.
SHRINK = 2
# Hard cap on the trail: the lock box may never sit more than LOCK_MARGIN px
# off the detection on any side.  Without it a fast walker stretches the box
# over its whole path (measured on the board: one track grew 96x37 ->
# 234x113 px), which is exactly the "box too big" complaint the vertical trim
# had already fixed once.
LOCK_MARGIN = 8
FRAME_W = 320
FRAME_H = 180


def lock_update(lock, det, shrink: int | None = None):
    """Grow to cover ``det``, then relax every side toward it by ``shrink``.

    ``shrink`` defaults to the module constant at call time (not at import
    time) so a sweep can patch ``track_model.SHRINK``.
    """
    if shrink is None:
        shrink = SHRINK
    dx0, dy0 = det[1], det[2]
    dx1, dy1 = det[1] + det[3], det[2] + det[4]
    if lock is None:
        x0, y0, x1, y1 = dx0, dy0, dx1, dy1
    else:
        x0, y0 = lock[0], lock[1]
        x1, y1 = lock[0] + lock[2], lock[1] + lock[3]
    x0, y0 = min(x0, dx0), min(y0, dy0)
    x1, y1 = max(x1, dx1), max(y1, dy1)
    x0 = dx0 if x0 + shrink >= dx0 else x0 + shrink
    y0 = dy0 if y0 + shrink >= dy0 else y0 + shrink
    x1 = dx1 if x1 - shrink <= dx1 else x1 - shrink
    y1 = dy1 if y1 - shrink <= dy1 else y1 - shrink
    x0 = max(x0, dx0 - LOCK_MARGIN)
    y0 = max(y0, dy0 - LOCK_MARGIN)
    x1 = min(x1, dx1 + LOCK_MARGIN)
    y1 = min(y1, dy1 + LOCK_MARGIN)
    x1, y1 = min(x1, FRAME_W), min(y1, FRAME_H)
    return (x0, y0, x1 - x0, y1 - y0)


def _iou_parts(a: tuple[int, ...], b: tuple[int, ...]) -> tuple[int, int]:
    ax, ay, aw, ah = a[1], a[2], a[3], a[4]
    bx, by, bw, bh = b[1], b[2], b[3], b[4]
    x0, y0 = max(ax, bx), max(ay, by)
    x1, y1 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    area_a = int(aw) * int(ah)
    area_b = int(bw) * int(bh)
    if x1 <= x0 or y1 <= y0:
        return 0, area_a + area_b
    inter = (x1 - x0) * (y1 - y0)
    union = area_a + area_b - inter
    return inter, union


def _better(cand: tuple[int, int, int, int],
            best: tuple[int, int, int, int] | None) -> bool:
    """Return True when candidate IoU is strictly greater than best."""
    if best is None:
        return True
    _ci, _cj, ci, cu = cand
    _bi, _bj, bi, bu = best
    return ci * bu > bi * cu


def _centre(box: tuple[int, ...]) -> tuple[int, int]:
    """RTL centring: x + (w >> 1), y + (h >> 1)."""
    return box[1] + (box[3] >> 1), box[2] + (box[4] >> 1)


def _near(a: tuple[int, ...], b: tuple[int, ...], radius: int) -> bool:
    ax, ay = _centre(a)
    bx, by = _centre(b)
    return abs(ax - bx) <= radius and abs(ay - by) <= radius


def track_slots(raw_frames: list[list[tuple[int, ...]]],
                trace: list[list[int]] | None = None
                ) -> list[list[tuple[int, ...]]]:
    """Re-slot a whole sequence, mirroring ``track_assoc.v`` exactly.

    When ``trace`` is a list, one entry per frame is appended: the 2-bit
    evidence level the score module will use (0 = fresh match, 3 = a track
    that has earned a full hold).  Same registers the RTL already keeps, so
    the hardware pays one small mux per slot.
    """
    if not raw_frames:
        return []

    prev: list[tuple[int, ...] | None] = [None] * NUM_BLOBS
    hold = [0] * NUM_BLOBS
    run = [0] * NUM_BLOBS
    est = [False] * NUM_BLOBS
    lock: list[tuple[int, int, int, int] | None] = [None] * NUM_BLOBS
    out_frames: list[list[tuple[int, ...]]] = []

    for frame_slots in raw_frames:
        cur: list[tuple[int, ...] | None] = []
        for slot in frame_slots[:NUM_BLOBS]:
            cur.append(slot if slot[0] else None)
        while len(cur) < NUM_BLOBS:
            cur.append(None)

        n_cur = sum(1 for slot in cur if slot is not None)
        radius = CLAIM_QUIET if n_cur <= 2 else CLAIM_NEAR

        cur_used = [False] * NUM_BLOBS
        prev_used = [False] * NUM_BLOBS
        map_i = [0] * NUM_BLOBS

        # Greedy: four rounds at most.  The RTL runs until a scan finds no
        # eligible pair, which is the same stopping condition.
        for _round in range(NUM_BLOBS):
            best = None
            best_key = None
            for i in range(NUM_BLOBS):
                if cur[i] is None or cur_used[i]:
                    continue
                for j in range(NUM_BLOBS):
                    if prev[j] is None or prev_used[j]:
                        continue
                    inter, union = _iou_parts(cur[i], prev[j])
                    if inter == 0:
                        continue
                    if MATCH_SCALE * inter < union:
                        continue
                    cand = (i, j, inter, union)
                    if _better(cand, best):
                        best = cand
                        best_key = (i, j)
            if best is None:
                break
            i, j = best_key
            cur_used[i] = True
            prev_used[j] = True
            map_i[j] = i

        out: list[tuple[int, ...] | None] = [None] * NUM_BLOBS
        evid = [0] * NUM_BLOBS

        # ---- S_OUT_MATCH: keep matched slots, hold or retire the rest ----
        for j in range(NUM_BLOBS):
            if prev_used[j]:
                det = cur[map_i[j]]
                lock[j] = lock_update(lock[j], det)
                out[j] = (1, *lock[j], det[5])
                strong = (run[j] >= EST_MATCHES - 1 or
                          out[j][5] >= AREA_EST)
                if strong:
                    est[j] = True
                hold[j] = HOLD_FRAMES if est[j] else GRACE_FRAMES
                if run[j] != 3:
                    run[j] += 1
                evid[j] = 3 if est[j] else (2 if run[j] >= 2 else 1)
            elif prev[j] is not None and hold[j] != 0:
                # A held box that a matched box of this frame sits next to is
                # an old pose of that same person; free the slot instead of
                # publishing a duplicate.
                if any(prev_used[k] and _near(prev[j], cur[map_i[k]], radius)
                       for k in range(NUM_BLOBS)):
                    hold[j] = 0
                    run[j] = 0
                    est[j] = False
                    lock[j] = None
                else:
                    # Hold: keep publishing the lock box (frozen).
                    box = lock[j] if lock[j] is not None else prev[j][1:5]
                    out[j] = (1, *box, prev[j][5])
                    hold[j] -= 1
                    evid[j] = 3 if est[j] else (2 if run[j] >= 2 else 1)
            else:
                hold[j] = 0
                run[j] = 0
                est[j] = False
                lock[j] = None
                evid[j] = 0

        # ---- S_OUT_FILL: claim, free or evict a slot for every leftover ----
        for i in range(NUM_BLOBS):
            if cur[i] is None or cur_used[i]:
                continue
            tgt = None
            claimed = False
            for s in range(NUM_BLOBS):
                if not prev_used[s] and out[s] is not None and \
                        _near(out[s], cur[i], radius):
                    tgt = s
                    claimed = True
                    break
            if tgt is None:
                tgt = next((s for s in range(NUM_BLOBS)
                            if out[s] is None), None)
            if tgt is None:
                stale = [s for s in range(NUM_BLOBS)
                         if out[s] is not None and not prev_used[s]]
                if stale:
                    # min() keeps the lower index on a tie, like evict_sel.
                    tgt = min(stale, key=lambda s: hold[s])
            if tgt is None:
                continue        # four live tracks: the detection is dropped
            out[tgt] = cur[i]
            lock[tgt] = cur[i][1:5]
            prev_used[tgt] = True       # consumed for the rest of this pass
            if not claimed:
                run[tgt] = 1
                est[tgt] = cur[i][5] >= AREA_EST
            hold[tgt] = HOLD_FRAMES if est[tgt] else GRACE_FRAMES
            evid[tgt] = 3 if est[tgt] else (2 if run[tgt] >= 2 else 1)

        frame_out = []
        for slot in out:
            if slot is None:
                frame_out.append((0, 0, 0, 0, 0, 0))
            else:
                frame_out.append(tuple(int(v) for v in slot))
        out_frames.append(frame_out)
        if trace is not None:
            trace.append([int(v) for v in evid])
        prev = [slot if slot is not None and slot[0] else None for slot in out]

    return out_frames
