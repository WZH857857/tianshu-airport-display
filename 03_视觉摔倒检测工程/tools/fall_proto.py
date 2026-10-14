"""Offline prototype of the FPGA fall-detection pipeline.

Every stage is integer arithmetic with fixed structuring elements, so the
Python model maps 1:1 onto the RTL that will replace it:

    RGB -> luma -> 2x2 box downsample -> frame difference -> threshold
    -> erode/dilate -> row/column projection -> bounding box
    -> geometry history -> fast posture-collapse state machine

The detector never builds an explicit background model.  It only looks at how
the moving blob changes shape, which is what makes it robust when a person is
already standing in the first frame (the case that broke the background
subtraction attempt).

Usage::

    python fall_proto.py CLIP.mp4 [--out DIR] [--trace] [--render-every 6]
"""

from __future__ import annotations

import csv
import argparse
from dataclasses import dataclass, field
from pathlib import Path

import imageio.v3 as iio
import numpy as np
from scipy import ndimage


def to_gray(frame: np.ndarray) -> np.ndarray:
    """RGB565-style integer luma, kept in 8 bits."""
    r = frame[..., 0].astype(np.uint32)
    g = frame[..., 1].astype(np.uint32)
    b = frame[..., 2].astype(np.uint32)
    return ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)


def downsample_to(gray: np.ndarray, target_w: int = 320,
                  target_h: int = 240) -> tuple[np.ndarray, tuple[int, int]]:
    """Box filter down to a fixed working resolution.

    Clips in the dataset are 640x480 or 1280x720, so the factors differ per
    axis.  Working at one fixed resolution is what makes the pixel thresholds
    (areas, heights) comparable, and it is also what the RTL will do.
    """
    h, w = gray.shape
    fx = max(1, int(round(w / float(target_w))))
    fy = max(1, int(round(h / float(target_h))))
    if fx == 1 and fy == 1:
        return gray, (1, 1)
    h2, w2 = h - h % fy, w - w % fx
    small = (gray[:h2, :w2]
             .reshape(h2 // fy, fy, w2 // fx, fx)
             .mean(axis=(1, 3))
             .astype(np.uint8))
    return small, (fx, fy)


def disk(radius: int) -> np.ndarray:
    y, x = np.ogrid[-radius:radius + 1, -radius:radius + 1]
    return (x * x + y * y <= radius * radius)


def largest_span(profile: np.ndarray, thresh: int) -> tuple[int, int] | None:
    """Longest contiguous run of bins above ``thresh`` in a 1-D profile.

    RTL equivalent: a small run-length counter fed by one comparator.
    """
    idx = np.nonzero(profile >= thresh)[0]
    if idx.size == 0:
        return None
    best = None
    start = prev = int(idx[0])
    for raw in idx[1:]:
        cur = int(raw)
        if cur == prev + 1:
            prev = cur
            continue
        if best is None or (prev - start) > (best[1] - best[0] - 1):
            best = (start, prev + 1)
        start = prev = cur
    if best is None or (prev - start) > (best[1] - best[0] - 1):
        best = (start, prev + 1)
    return best


@dataclass
class Params:
    """All thresholds are integers so they map onto RTL registers."""

    target_w: int = 320          # every clip is normalised to this width
    target_h: int = 240
    motion_threshold: int = 10   # |cur - prev| threshold
    erode_radius: int = 1        # speckle removal
    dilate_radius: int = 3       # glue limbs/torso into one blob
    min_pixels: int = 3          # a row/column must carry this many mask pixels
    min_area: int = 200          # reject small blobs (320x240 scale)
    min_height: int = 14         # smallest bbox height that can be a person
    min_ref_area: int = 1600     # a standing person blob is at least this big
    min_ref_height: int = 25     # ... and at least this tall (320x240 scale)
    # Person-likeness gates on the standing reference.  A tram, bus or other
    # large moving object can otherwise become the "upright" pose and then
    # trigger a fall when it leaves the frame.  These are cheap integer
    # comparisons and map directly to RTL.
    min_ref_fill: float = 0.25   # mask pixels / bbox area
    max_ref_width_frac: float = 0.55
    max_ref_height_frac: float = 0.95
    max_ref_area_frac: float = 0.20
    upright_ratio: float = 1.00  # h/w at or above this counts as upright
    lying_ratio: float = 0.95    # h/w at or below this counts as no longer tall
    # The collapse gate used to require the bbox to shrink to 0.65 of the
    # standing height.  A fall toward the camera does the opposite - the
    # projected blob grows - so the gate is now "no taller than the
    # reference"; the flat, speed and keep-area tests do the discriminating.
    collapse_frac: float = 1.00  # bbox height must not exceed the reference
    tall_frac: float = 0.60      # "still upright" height, fraction of h_ref
    collapse_frames: int = 18    # the fast phase must fit in this window
    history_frames: int = 42     # geometry memory for the reference pose
    ref_ttl: int = 300           # how long a standing reference stays valid
    confirm_frames: int = 2      # flat frames needed to raise the alarm
    # Widening is left off by default: a fall toward the camera keeps a narrow
    # mask, and requiring width growth costs 3 of the 13 sample clips.
    widen_frac: float = 0.0      # bbox must get this much wider (0 = off)
    drop_frac: float = 0.04      # centroid must sink by this fraction of H
    cooldown: int = 60           # frames before the detector re-arms
    # Multi-target front end.  The crowd footage contains up to seven
    # simultaneous movers, so a single dominant box is not enough.  Each
    # surviving blob gets its own PostureMachine; the FPGA version replaces
    # ndimage.label() with a coarse-cell connected-component pass.
    max_targets: int = 4         # strongest motion blobs tracked per frame
    merge_gap: int = 0            # extra dilation before labelling (px)
    track_ttl: int = 45           # frames a track survives without an update
    track_near_frac: float = 0.75 # centroid jump allowed when associating
    max_assoc_gap: int = 3        # a track may only skip this many frames
    track_area_lo: float = 0.35   # blob area may not collapse between frames
    track_area_hi: float = 2.50   # ... nor explode (merged crowd)
    # A walking pair that merges and then splits leaves a torso blob whose
    # bottom sits ~11% of the standing height above the floor; the old 0.15
    # window admitted it.  A real fall keeps its mask on the floor, so 0.10
    # passes every GMDCSA24 fall while the crowd sweep drops to zero alarms.
    bottom_tol_frac: float = 0.10 # fallen box bottom stays near the ground
    min_fall_area_frac: float = 0.30  # fall mask / standing reference mask
    # A single-person scene can confirm a fast fall in two frames.  A crowded
    # scene needs a longer, quieter confirmation because fragments of walking
    # people produce the same short-lived geometry.
    crowd_min_targets: int = 2       # person-like movers that mean "crowd"
    crowd_confirm_frames: int = 4    # flat frames required in a crowd
    recovery_window: int = 2         # frames to watch for a stand-up recovery
    recovery_rise_frac: float = 0.25 # top rising by this much cancels a fall

    @classmethod
    def for_fps(cls, fps: float) -> "Params":
        """Same physical timing expressed in frames for a given frame rate.

        The RTL runs at a fixed 25/30 fps, so these become compile-time
        constants there.  Offline clips range from 1 fps (compressed CCTV
        event clips) to 30 fps, and re-using the 30 fps counters on a 1 fps
        clip would make every window meaningless.
        """
        p = cls()
        def frames(seconds: float, lo: int, hi: int) -> int:
            return max(lo, min(hi, int(round(seconds * fps))))
        p.collapse_frames = frames(1.20, 2, 40)
        p.confirm_frames = frames(0.065, 1, 3)
        p.crowd_confirm_frames = frames(0.13, 1, 4)
        p.recovery_window = frames(0.07, 1, 3)
        p.ref_ttl = frames(10.0, 30, 600)
        p.cooldown = frames(2.0, 2, 120)
        p.track_ttl = frames(1.5, 3, 90)
        p.max_assoc_gap = frames(0.10, 1, 6)
        return p


@dataclass
class Obs:
    frame: int
    bbox: tuple[int, int, int, int]
    area: int
    ratio: float
    cx: float
    cy: float

    @property
    def w(self) -> int:
        return self.bbox[2]

    @property
    def h(self) -> int:
        return self.bbox[3]

    @property
    def x(self) -> int:
        return self.bbox[0]

    @property
    def y(self) -> int:
        return self.bbox[1]

    @property
    def fill(self) -> float:
        """Fraction of the bounding box covered by motion pixels."""
        return self.area / float(self.w * self.h) if self.w and self.h else 0.0

    @property
    def bottom(self) -> int:
        return self.bbox[1] + self.bbox[3]


@dataclass
class Detection:
    frame: int
    flagged: bool
    track_id: int
    bbox: tuple[int, int, int, int] | None = None
    ratio: float = 0.0
    state: str = "IDLE"
    detail: str = ""
    area: int = 0


@dataclass
class Track:
    """One motion blob's trajectory and its private posture state machine."""

    track_id: int
    machine: PostureMachine
    last_bbox: tuple[int, int, int, int] | None = None
    last_frame: int = 0
    last_area: int = 0
    miss: int = 0
    hits: int = 0


def same_track(prev: tuple[int, int, int, int] | None,
               cur: tuple[int, int, int, int],
               near_frac: float = 0.75) -> bool:
    """True when ``cur`` continues the blob at ``prev``.

    Accepts either an overlapping box (IoU) or a nearby centroid, because a
    fall translates the blob faster than the overlap alone would tolerate.
    """
    if prev is None:
        return False
    ax, ay, aw, ah = prev
    bx, by, bw, bh = cur
    x0, y0 = max(ax, bx), max(ay, by)
    x1, y1 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    union = aw * ah + bw * bh - inter
    iou = inter / union if union else 0.0
    dx = (bx + bw / 2.0) - (ax + aw / 2.0)
    dy = (by + bh / 2.0) - (ay + ah / 2.0)
    near = (dx * dx + dy * dy) ** 0.5 <= near_frac * max(aw, ah, bw, bh)
    return iou > 0.05 or near


def person_like(obs: Obs, p: Params, frame_w: int, frame_h: int) -> bool:
    """True when a tall blob is dense and small enough to be a person.

    The reference decides the "standing height".  If a tram or another
    frame-filling mover is admitted here, every later shape change can be
    mistaken for a collapse.  Shape gates are deliberately applied only to
    the reference: partially occluded fall frames are allowed to be sparse,
    because the fall itself is judged by the geometry change.
    """
    frame_area = frame_w * frame_h
    return (obs.area >= p.min_ref_area
            and obs.h >= p.min_ref_height
            and obs.ratio >= p.upright_ratio
            and obs.fill >= p.min_ref_fill
            and obs.w <= p.max_ref_width_frac * frame_w
            and obs.h <= p.max_ref_height_frac * frame_h
            and obs.area <= p.max_ref_area_frac * frame_area)


class PostureMachine:
    """Posture state machine: an Obs stream in, alarm decisions out.

    Deliberately free of numpy so the parameter tuner can replay recorded
    geometry traces through exactly the same code the pixel pipeline uses.

    The rule it implements, per dominant moving blob:

      * remember the tallest upright pose (``h_ref``) seen recently,
      * remember when the blob was last still upright (``last_tall``),
      * raise an alarm when the blob becomes flat and wide, its centroid
        sinks, and it went from upright to flat within ``collapse_frames``.

    The speed term is what separates a fall from slowly sitting down.
    """

    def __init__(self, params: Params):
        self.p = params
        self.frame_w = 320          # working width, used by the width gate
        self.frame_h = 240          # working height, used by the sink test
        self.reset()

    def reset(self) -> None:
        self.h_ref: Obs | None = None
        self.last_tall: int = -10 ** 9
        self.flat_frames = 0
        self.cooldown = 0
        self.state = "SEEK"
        self.confirm_required = self.p.confirm_frames
        self.pending_alarm = False
        self.pending_frame = 0
        self.pending_top = 0
        self.pending_ref_h = 0
        self.pending_detail = ""

    def person_like_ref(self, obs: Obs) -> bool:
        return person_like(obs, self.p, self.frame_w, self.frame_h)

    def _confirm(self) -> tuple[bool, str, str]:
        self.state = "FALL"
        self.pending_alarm = False
        self.cooldown = self.p.cooldown
        return True, self.state, self.pending_detail

    def feed(self, obs: Obs | None, frame: int | None = None) -> tuple[bool, str, str]:
        """Feed one frame; returns (alarm, state, detail)."""
        p = self.p
        if self.cooldown:
            self.cooldown -= 1
        if obs is None:
            # A gap breaks the "continuously flat" evidence: otherwise a
            # torso fragment could re-associate after a long occlusion and
            # complete somebody else's collapse counter.
            self.flat_frames = 0
            # A candidate that survives the recovery window with no motion
            # at all is confirmed: the person went down and stopped moving.
            if (self.pending_alarm and frame is not None
                    and frame - self.pending_frame >= p.recovery_window):
                return self._confirm()
            return False, self.state, "no-motion"

        # The standing reference survives gaps with no motion: a person can
        # stand still for a while before falling, so only a TTL or a new
        # track drops it.
        if self.h_ref is not None and obs.frame - self.h_ref.frame > p.ref_ttl:
            self.h_ref = None
        if (self.person_like_ref(obs)
                and (self.h_ref is None or obs.h > self.h_ref.h)):
            self.h_ref = obs
        if self.h_ref is not None and obs.h >= p.tall_frac * self.h_ref.h:
            self.last_tall = obs.frame

        flat = obs.ratio <= p.lying_ratio
        self.flat_frames = self.flat_frames + 1 if flat else 0

        ref = self.h_ref
        # A pending candidate is cancelled if the blob stands back up:
        # either its top rises by a quarter of the standing height, or the
        # box grows back towards the upright height and aspect ratio.  This
        # is what separates "exercise: sit and stand" from a real fall.
        if self.pending_alarm:
            rises = obs.y <= (self.pending_top
                              - p.recovery_rise_frac * self.pending_ref_h)
            stands = (ref is not None
                      and obs.h >= 0.80 * ref.h
                      and obs.ratio >= p.upright_ratio)
            if rises or stands:
                self.pending_alarm = False
                self.state = "RECOVERED"
                self.flat_frames = 0
            elif obs.frame - self.pending_frame >= p.recovery_window:
                return self._confirm()

        if (ref is not None and self.cooldown == 0
                and not self.pending_alarm):
            tall = (ref.h >= p.min_height and self.person_like_ref(ref))
            shrunk = obs.h <= p.collapse_frac * ref.h
            wider = obs.w >= p.widen_frac * ref.w
            sunk = (obs.cy - ref.cy) >= p.drop_frac * self.frame_h
            # A walking person whose legs split off leaves a torso blob whose
            # bottom jumps upward, while a real fall keeps its bottom on the
            # floor.  This single comparison removes most crowd false alarms
            # and maps to one subtractor in RTL.
            grounded = obs.bottom >= ref.bottom - p.bottom_tol_frac * ref.h
            # Reject a collapse whose mask has shrunk to a tiny fragment of
            # the standing silhouette (a walking leg, a swinging arm).
            keep_area = obs.area >= p.min_fall_area_frac * ref.area
            speed = obs.frame - self.last_tall
            fast = speed <= p.collapse_frames
            if (tall and shrunk and flat and wider and sunk and grounded
                    and keep_area and fast
                    and self.flat_frames >= self.confirm_required):
                self.pending_alarm = True
                self.pending_frame = obs.frame
                self.pending_top = obs.y
                self.pending_ref_h = ref.h
                self.pending_detail = (
                    f"ref_h={ref.h}@{ref.frame} ratio={ref.ratio:.2f} "
                    f"cur={obs.w}x{obs.h} ratio={obs.ratio:.2f} "
                    f"dy={obs.cy - ref.cy:.0f} "
                    f"ground={obs.bottom - ref.bottom} speed={speed}")
                self.state = "PENDING"
                if p.recovery_window <= 1:
                    return self._confirm()

        if self.state == "FALL" and not flat:
            self.state = "RECOVERED"

        detail = (f"h_ref={ref.h if ref else 0} flat={int(flat)} "
                  f"speed={obs.frame - self.last_tall}"
                  if ref is not None else "")
        return False, self.state, detail


class FallDetector:
    """Frame-difference fall detector with one posture machine per track.

    The front end keeps the original integer morphology, then labels the
    motion mask into connected blobs.  The ``max_targets`` largest blobs are
    associated with existing trajectories by overlap/centroid proximity; a
    blob that matches nothing starts a new track.  Every track owns a
    ``PostureMachine``, so one person's sitting motion cannot reset the
    standing reference of somebody else in the same frame.

    ``max_targets == 1`` keeps a projection-based fallback for RTL bring-up:
    it is cheaper than connected components and is useful for comparing the
    two front ends on single-person clips.
    """

    def __init__(self, params: Params | None = None):
        self.p = params or Params()
        self.prev: np.ndarray | None = None
        self.frame_index = 0
        self.next_id = 1
        self.tracks: list[Track] = []
        self.mask_cache: np.ndarray | None = None
        self.scale: tuple[int, int] = (1, 1)
        self.trace: list[dict] = []

    # ---------------------------------------------------------------- frames
    def prepare(self, frame: np.ndarray) -> np.ndarray:
        small, self.scale = downsample_to(to_gray(frame), self.p.target_w,
                                          self.p.target_h)
        return small

    def motion_mask(self, gray: np.ndarray) -> np.ndarray:
        """|cur - prev| > threshold, opened then dilated."""
        p = self.p
        if self.prev is None:
            self.prev = gray.copy()
            return np.zeros_like(gray, dtype=bool)
        diff = np.abs(gray.astype(np.int16) - self.prev.astype(np.int16))
        mask = diff > p.motion_threshold
        self.prev = gray.copy()
        if mask.any():
            mask = ndimage.binary_erosion(mask, structure=disk(p.erode_radius))
            mask = ndimage.binary_dilation(mask, structure=disk(p.dilate_radius))
        return mask

    def bbox_of(self, mask: np.ndarray) -> tuple[tuple[int, int, int, int] | None, int]:
        """Bounding box of the dominant blob via two-pass projections.

        Kept as the single-target fallback and as the cheap RTL bring-up
        path.  The row run is found first, then the column run is measured
        *inside* that band only, so unrelated motion (a ceiling fan, sensor
        noise) cannot stretch the box to the frame.
        """
        if not mask.any():
            return None, 0
        p = self.p
        rows = mask.sum(axis=1)
        span_y = largest_span(rows, p.min_pixels)
        if span_y is None:
            return None, 0
        y0, y1 = span_y
        band = mask[y0:y1]
        span_x = largest_span(band.sum(axis=0), p.min_pixels)
        if span_x is None:
            return None, 0
        x0, x1 = span_x
        area = int(band[:, x0:x1].sum())
        if area < p.min_area:
            return None, area
        return (x0, y0, x1 - x0, y1 - y0), area

    def blobs_of(self,
                 mask: np.ndarray) -> list[tuple[tuple[int, int, int, int], int]]:
        """Connected motion blobs, strongest first, capped at max_targets.

        ``merge_gap`` optionally dilates the mask one more time before
        labelling so that a head and torso split by a one-pixel gap are
        treated as one person.  The default is zero because the 7x7 disk in
        ``motion_mask`` already glues most body parts.
        """
        if not mask.any():
            return []
        p = self.p
        grown = mask
        if p.merge_gap > 0:
            grown = ndimage.binary_dilation(mask, structure=disk(p.merge_gap))
        labels, count = ndimage.label(grown, structure=np.ones((3, 3), bool))
        blobs: list[tuple[tuple[int, int, int, int], int]] = []
        for label, slc in enumerate(ndimage.find_objects(labels), start=1):
            if slc is None:
                continue
            local = mask[slc] & (labels[slc] == label)
            area = int(local.sum())
            if area < p.min_area:
                continue
            ys, xs = np.nonzero(local)
            if ys.size == 0:
                continue
            y0 = slc[0].start + int(ys.min())
            y1 = slc[0].start + int(ys.max()) + 1
            x0 = slc[1].start + int(xs.min())
            x1 = slc[1].start + int(xs.max()) + 1
            if y1 - y0 < p.min_height:
                continue
            # A blob that spans (almost) the whole raster is not a person: it
            # is the crowd moving as one, camera shake, or an exposure step.
            # Keeping it eats a slot and hides the real people behind a box
            # the size of the screen, which is exactly what crowd_test.mp4 did
            # before this guard (one 320x240 track for the whole clip).
            if (x1 - x0) >= p.target_w - 8 or (y1 - y0) >= p.target_h - 8:
                continue
            blobs.append(((x0, y0, x1 - x0, y1 - y0), area))
        blobs.sort(key=lambda item: item[1], reverse=True)
        if p.max_targets > 0:
            blobs = blobs[:p.max_targets]
        return blobs

    # -------------------------------------------------------------- tracking
    def observe(self, bbox, area) -> Obs:
        x, y, w, h = bbox
        return Obs(self.frame_index, bbox, area,
                   h / float(w) if w else 0.0,
                   x + (w - 1) / 2.0,
                   y + (h - 1) / 2.0)

    def _match_cost(self, track: Track, obs: Obs):
        """Sort key for associating ``obs`` with ``track`` (lower = better).

        Returns ``None`` when the jump is too large to be the same blob.
        """
        prev = track.last_bbox
        if prev is None:
            return None
        p = self.p
        if obs.frame - track.last_frame > p.max_assoc_gap:
            return None
        if track.last_area:
            area_ratio = obs.area / float(track.last_area)
            if not (p.track_area_lo <= area_ratio <= p.track_area_hi):
                return None
        bbox = obs.bbox
        if not same_track(prev, bbox, p.track_near_frac):
            return None
        ax, ay, aw, ah = prev
        bx, by, bw, bh = bbox
        x0, y0 = max(ax, bx), max(ay, by)
        x1, y1 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
        inter = max(0, x1 - x0) * max(0, y1 - y0)
        union = aw * ah + bw * bh - inter
        iou = inter / union if union else 0.0
        dx = (bx + bw / 2.0) - (ax + aw / 2.0)
        dy = (by + bh / 2.0) - (ay + ah / 2.0)
        dist = (dx * dx + dy * dy) ** 0.5
        return (0 if iou > 0.05 else 1, -iou, dist)

    def _new_track(self, frame_w: int, frame_h: int) -> Track:
        machine = PostureMachine(self.p)
        machine.frame_w = frame_w
        machine.frame_h = frame_h
        track = Track(self.next_id, machine)
        self.next_id += 1
        self.tracks.append(track)
        return track

    def _step_tracks(self, obs_list: list[Obs],
                     frame_shape: tuple[int, int]) -> list[Detection]:
        p = self.p
        frame_h, frame_w = frame_shape

        # Greedy global matching: rank every compatible (track, blob) pair
        # once and take the best non-conflicting assignments.
        pairs: list[tuple[tuple, int, int]] = []
        for oi, obs in enumerate(obs_list):
            for ti, track in enumerate(self.tracks):
                cost = self._match_cost(track, obs)
                if cost is not None:
                    pairs.append((cost, oi, ti))
        pairs.sort(key=lambda item: item[0])
        obs_to_track: dict[int, int] = {}
        used_tracks: set[int] = set()
        for _, oi, ti in pairs:
            if oi in obs_to_track or ti in used_tracks:
                continue
            obs_to_track[oi] = ti
            used_tracks.add(ti)

        # Counting person-like movers decides how much confirmation the
        # scene needs: a lone walker can confirm in two frames, a dense
        # crowd needs four because torso/leg fragments are common there.
        person_like_count = sum(
            1 for obs in obs_list
            if person_like(obs, p, frame_w, frame_h))
        required = (p.crowd_confirm_frames
                    if person_like_count >= p.crowd_min_targets
                    else p.confirm_frames)

        # Existing tracks that were not matched this frame keep their
        # reference alive, but the posture machine sees the gap.
        missed_alarms: list[Detection] = []
        for ti, track in enumerate(self.tracks):
            if ti not in used_tracks:
                track.miss += 1
                flagged, state, detail = track.machine.feed(
                    None, self.frame_index)
                if flagged:
                    missed_alarms.append(Detection(
                        self.frame_index, True, track.track_id,
                        track.last_bbox, 0.0, state, detail, track.last_area))

        detections: list[Detection] = []
        for oi, obs in enumerate(obs_list):
            ti = obs_to_track.get(oi)
            if ti is None:
                track = self._new_track(frame_w, frame_h)
            else:
                track = self.tracks[ti]
            track.last_bbox = obs.bbox
            track.last_frame = obs.frame
            track.last_area = obs.area
            track.miss = 0
            track.hits += 1
            track.machine.confirm_required = required
            flagged, state, detail = track.machine.feed(obs, obs.frame)
            detections.append(Detection(obs.frame, flagged, track.track_id,
                                        obs.bbox, obs.ratio, state, detail,
                                        obs.area))
        detections.extend(missed_alarms)

        if self.tracks:
            self.tracks = [t for t in self.tracks if t.miss <= p.track_ttl]
        return detections

    # ---------------------------------------------------------------- update
    def update(self, frame: np.ndarray) -> list[Detection]:
        self.frame_index += 1
        gray = self.prepare(frame)
        mask = self.motion_mask(gray)
        self.mask_cache = mask
        if self.p.max_targets <= 1:
            bbox, area = self.bbox_of(mask)
            blobs = [(bbox, area)] if bbox is not None else []
        else:
            blobs = self.blobs_of(mask)
            frame_h, frame_w = gray.shape

            # Whole-raster boxes are crowd-wide motion, not people.  The
            # connected-component path filters them in ``blobs_of``; the
            # projection fallback below has the same failure mode (it is what
            # produced the single 320x240 track on crowd_test.mp4), so the
            # filter has to sit on this side too.
            def _spans_raster(bbox) -> bool:
                return (bbox[2] >= frame_w - 8) or (bbox[3] >= frame_h - 8)

            blobs = [item for item in blobs if not _spans_raster(item[0])]
            # Hybrid front end: a lone person is tracked with the cheaper
            # projection box (the configuration validated on GMDCSA24).
            # Connected components only take over once the scene actually
            # contains several person-like movers.
            candidates = [self.observe(b, a) for b, a in blobs]
            person_like_count = sum(
                1 for obs in candidates
                if person_like(obs, self.p, frame_w, frame_h))
            if person_like_count < self.p.crowd_min_targets:
                bbox, area = self.bbox_of(mask)
                blobs = ([(bbox, area)]
                         if bbox is not None and not _spans_raster(bbox)
                         else [])
        obs_list = [self.observe(bbox, area) for bbox, area in blobs]
        dets = self._step_tracks(obs_list, gray.shape)
        if dets:
            for det in dets:
                self.trace.append({
                    "frame": self.frame_index,
                    "track": det.track_id,
                    "area": det.area,
                    "bbox": det.bbox,
                    "ratio": det.ratio,
                    "state": det.state,
                    "flagged": det.flagged,
                    "detail": det.detail,
                })
        else:
            self.trace.append({
                "frame": self.frame_index,
                "track": 0,
                "area": 0,
                "bbox": None,
                "ratio": 0.0,
                "state": "SEEK",
                "flagged": False,
                "detail": "no-motion",
            })
        return dets


# Same palette as rtl/hdmi/box_overlay.v so the offline renders and the board
# picture read the same way: one hue per tracked slot, red when a fall is
# confirmed.  Slot order is stable, so a colour sticks to a person.
SLOT_COLOURS = [
    (0x00, 0xFF, 0x60),   # slot 0 green
    (0x00, 0xC8, 0xFF),   # slot 1 cyan
    (0xFF, 0xC4, 0x00),   # slot 2 amber
    (0xFF, 0x40, 0xFF),   # slot 3 magenta
]
ALARM_COLOUR = (0xFF, 0x00, 0x00)


def slot_colour(track_id: int) -> tuple[int, int, int]:
    return SLOT_COLOURS[track_id % len(SLOT_COLOURS)]


def annotate(frame: np.ndarray, mask: np.ndarray | None,
             detections: list[Detection],
             scale: tuple[int, int] = (1, 1)) -> np.ndarray:
    """Draw the mask overlay and the tracked box back onto the source frame."""
    fx, fy = scale
    out = frame.copy()
    if mask is not None:
        small = (mask * 255).astype(np.uint8)
        if fx > 1 or fy > 1:
            small = np.repeat(np.repeat(small, fy, axis=0), fx, axis=1)
        h, w = out.shape[:2]
        if small.shape[0] < h or small.shape[1] < w:
            padded = np.zeros((h, w), dtype=np.uint8)
            padded[:small.shape[0], :small.shape[1]] = small
            small = padded
        small = small[:h, :w]
        out = (out * 0.65 + np.dstack([small] * 3) * 0.35).astype(np.uint8)
    for det in detections:
        if det.bbox is None:
            continue
        x, y, w, h = det.bbox
        color = (ALARM_COLOUR if det.state == "FALL"
                 else slot_colour(det.track_id))
        x0, y0 = x * fx, y * fy
        x1, y1 = (x + w) * fx, (y + h) * fy
        out[max(0, y0 - 2):y0 + 2, x0:x1] = color
        out[y1 - 2:y1 + 2, x0:x1] = color
        out[y0:y1, max(0, x0 - 2):x0 + 2] = color
        out[y0:y1, x1 - 2:x1 + 2] = color
    return out


def run_clip(path: Path, params: Params, out_dir: Path | None,
             render_every: int = 5, trace: bool = False,
             dump: Path | None = None) -> list[Detection]:
    detector = FallDetector(params)
    results: list[Detection] = []
    if out_dir:
        out_dir.mkdir(parents=True, exist_ok=True)
    for index, frame in enumerate(iio.imiter(path, plugin="FFMPEG")):
        dets = detector.update(frame)
        results.extend(dets)
        if out_dir and index % render_every == 0:
            marked = annotate(frame, detector.mask_cache, dets, detector.scale)
            iio.imwrite(out_dir / f"annot_{index:04d}.png", marked)
    if trace:
        for row in detector.trace:
            bbox = row["bbox"]
            box = "-" if bbox is None else f"{bbox[0]},{bbox[1]} {bbox[2]}x{bbox[3]}"
            print(f"{row['frame']:4d} t{row.get('track', 0):<3d} "
                  f"area={row['area']:6d} bbox={box:<18} "
                  f"ratio={row['ratio']:.2f} {row['state']:<9} "
                  f"{'ALARM' if row['flagged'] else ''} {row['detail']}")
    if dump:
        dump.parent.mkdir(parents=True, exist_ok=True)
        with dump.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["frame", "track", "area", "x", "y", "w", "h",
                             "ratio", "state", "flagged", "detail"])
            for row in detector.trace:
                bbox = row["bbox"]
                x, y, w, h = bbox if bbox else ("", "", "", "")
                writer.writerow([row["frame"], row.get("track", 0),
                                 row["area"], x, y, w, h,
                                 f"{row['ratio']:.4f}", row["state"],
                                 int(row["flagged"]), row["detail"]])
    detector.trace = []
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("clip", type=Path)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--target-w", type=int, default=320)
    parser.add_argument("--target-h", type=int, default=240)
    parser.add_argument("--threshold", type=int, default=None,
                        help="frame-difference threshold (default 10)")
    parser.add_argument("--dilate", type=int, default=None,
                        help="dilation radius, 3 = the 7x7 kernel used in RTL")
    parser.add_argument("--erode", type=int, default=None,
                        help="erosion radius, 2 = the 5x5 kernel used in RTL")
    parser.add_argument("--render-every", type=int, default=5)
    parser.add_argument("--trace", action="store_true")
    parser.add_argument("--dump", type=Path, default=None)
    args = parser.parse_args()
    params = Params(target_w=args.target_w, target_h=args.target_h)
    if args.threshold is not None:
        params.motion_threshold = args.threshold
    if args.erode is not None:
        params.erode_radius = args.erode
    if args.dilate is not None:
        params.dilate_radius = args.dilate
    results = run_clip(args.clip, params, args.out, args.render_every,
                       args.trace, args.dump)
    alarms = [d.frame for d in results if d.flagged]
    print(f"alarm frames: {alarms if alarms else 'none'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
