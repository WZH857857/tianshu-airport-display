"""Render the HDMI overlay exactly as the board draws it.

Replays the RTL front end and the ``multi_model`` box finder over a clip and
paints the boxes with ``box_overlay.v``'s palette and geometry (vx = px/2,
vy = 3*py/8, one hue per slot, edge-only rectangles), so the picture matches
what the monitor shows for a given bitstream.

``--mode old`` uses the pre-ST_VSPAN band-height box, ``--mode new`` the
current trimmed box, ``--mode both`` writes a side-by-side comparison.  That
comparison is the evidence for "box too big" fixes without re-flashing.

``--mode track`` runs the same sequence through ``track_model`` (the reference
for ``track_assoc.v``) and paints the *published* boxes instead of the raw
detections, so the elastic lock box -- the thing that keeps one box glued to
one person -- can be judged before a PNR run.  ``--mode track-cmp`` writes a
still sheet with the raw detection on the left and the locked box on the
right.

Usage::

    python machine_view.py --clip CLIP.mp4 --frames 150 --mode both \
        --out ../out/machine_view_crowd
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(HERE))

import box_size_stats as B  # noqa: E402
import label_font as L  # noqa: E402
import multi_model as M  # noqa: E402
import person_roi as PR  # noqa: E402
import track_model as T  # noqa: E402

# box_overlay.v palette, in slot order.
SLOT_RGB = [(0x00, 0xFF, 0x60), (0x00, 0xC8, 0xFF),
            (0xFF, 0xC4, 0x00), (0xFF, 0x40, 0xFF)]
ALARM_RGB = (0xFF, 0x00, 0x00)

DISP_W, DISP_H = 640, 480


def score_of(slot, evid: int) -> int:
    """The same integer feature buckets and score formula the RTL uses."""
    if not slot[0]:
        return 0
    _, x0, y0, w, h, area = slot
    _, aspect, fill, size = PR.buckets(int(w), int(h), int(area),
                                      max(1, int(w) * int(h)), int(evid))
    return PR.score(evid, aspect, fill, size)


def normalise(frame: np.ndarray) -> np.ndarray:
    """Any source size -> the 640x480 raster the HDMI path carries."""
    h, w = frame.shape[:2]
    if (w, h) == (DISP_W, DISP_H):
        return frame
    ys = np.minimum(np.arange(DISP_H) * h // DISP_H, h - 1)
    xs = np.minimum(np.arange(DISP_W) * w // DISP_W, w - 1)
    return frame[ys][:, xs]


def paint(rgb: np.ndarray, slots, alarm=None, scores=None) -> np.ndarray:
    """Box overlay in box_overlay.v's style: boxes plus confidence tags."""
    out = rgb.copy()
    vx = np.arange(DISP_W) // 2                      # 0 .. 319
    vy = (np.arange(DISP_H) * 3) // 8                # 0 .. 179

    for k, slot in enumerate(slots):
        if not slot[0]:
            continue
        _, x0, y0, w, h, _ = slot
        x1, y1 = x0 + w, y0 + h
        inside_x = (vx >= x0) & (vx < x1)
        inside_y = (vy >= y0) & (vy < y1)
        band_x = (vx >= x0 - 1) & (vx <= x1)
        band_y = (vy >= y0 - 1) & (vy <= y1)
        core_x = (vx >= x0 + 1) & (vx <= x1 - 1)
        core_y = (vy >= y0 + 1) & (vy <= y1 - 1)
        edge = (band_y[:, None] & band_x[None, :] &
                ~(core_y[:, None] & core_x[None, :]))
        rgb_col = ALARM_RGB if (alarm is not None and alarm[k]) \
            else SLOT_RGB[k % len(SLOT_RGB)]
        out[edge] = rgb_col

    # Tags come last: box_overlay.v gives the label priority over the box edge
    # it overlaps.  The person readout is a 24x5 vision-pixel tag, the 摔倒
    # alarm the original 48x18 one, and each hangs 2 vision rows above the box
    # (rows 1.. below the top edge when the box is already in the top margin).
    if scores is not None:
        for k, slot in enumerate(slots):
            if not slot[0]:
                continue
            x0, y0 = int(slot[1]), int(slot[2])
            is_alarm = alarm is not None and alarm[k]
            mask = L.alarm_tag() if is_alarm else L.person_tag(int(scores[k]))
            colour = ALARM_RGB if is_alarm else SLOT_RGB[k % len(SLOT_RGB)]
            margin = mask.shape[0] + 1
            ly0 = y0 - margin if y0 >= margin else y0 + 1
            stamp(out, mask, x0, ly0, colour)
    return out


def stamp(img: np.ndarray, mask: np.ndarray, lx0: int, ly0: int,
          colour) -> None:
    """Blit a vision-space tag: two picture columns per vision column, and
    the picture rows whose vy = 3*py/8 lands on a mask row."""
    rows, cols = mask.shape
    col_of = np.arange(DISP_W) // 2
    row_of = (np.arange(DISP_H) * 3) // 8
    ys = np.where((row_of >= ly0) & (row_of < ly0 + rows))[0]
    xs = np.where((col_of >= lx0) & (col_of < lx0 + cols))[0]
    if not ys.size or not xs.size:
        return
    sub = mask[row_of[ys] - ly0][:, col_of[xs] - lx0]
    img[np.ix_(ys, xs)] = np.where(sub[..., None] != 0, colour, 0)


def label(img: np.ndarray, text: str) -> np.ndarray:
    from PIL import Image, ImageDraw, ImageFont

    pil = Image.fromarray(img)
    draw = ImageDraw.Draw(pil)
    try:
        font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 28)
    except OSError:
        font = ImageFont.load_default()
    draw.rectangle([0, 0, 620, 40], fill=(0, 0, 0))
    draw.text((10, 4), text, fill=(255, 255, 255), font=font)
    return np.asarray(pil)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--clip", type=Path, required=True)
    ap.add_argument("--frames", type=int, default=150)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--mode", default="both",
                    choices=["old", "new", "both", "track", "track-cmp"])
    ap.add_argument("--diff-thresh", type=int, default=8)
    ap.add_argument("--min-pix", type=int, default=4)
    ap.add_argument("--min-area", type=int, default=160)
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--samples", type=int, default=4,
                    help="frames in the still comparison")
    args = ap.parse_args()

    B.DIFF_THRESH = args.diff_thresh
    M.MIN_PIX = args.min_pix
    M.MIN_ROW_PIX = args.min_pix
    M.MIN_AREA = args.min_area

    import imageio.v3 as iio

    raw = []
    for i, frame in enumerate(iio.imiter(args.clip, plugin="FFMPEG")):
        if i >= args.frames:
            break
        raw.append(normalise(np.asarray(frame)))
    if len(raw) < 2:
        raise SystemExit("clip too short")

    masks = B.masks_of(args.clip, len(raw), 2, 2)
    n = min(len(raw), len(masks))
    raw, masks = raw[:n], masks[:n]
    print(f"{args.clip.name}: {n} frames, motion px/frame median "
          f"{int(np.median(masks.sum(axis=(1, 2))))}")

    modes = ["old", "new"] if args.mode == "both" else [args.mode]
    print(f"  params diff>{args.diff_thresh} min_pix={args.min_pix} "
          f"min_area={args.min_area}")
    raw_all = [B.boxes_of(m, 1) for m in masks]
    raw_model = [[tuple(int(v) for v in b) for b in f] for f in raw_all]
    trace: list[list[int]] = []
    if args.mode in ("track", "track-cmp"):
        lock_all = T.track_slots(raw_model, trace)
        score_all = [[score_of(lock_all[f][s], trace[f][s])
                      for s in range(T.NUM_BLOBS)] for f in range(n)]
    else:
        lock_all = T.track_slots(raw_model)
        score_all = None

    def stats(tag: str, frames) -> None:
        found = sum(1 for f in frames for s in f if s[0])
        heights = [s[4] for f in frames for s in f if s[0]]
        widths = [s[3] for f in frames for s in f if s[0]]
        print(f"  {tag:<10} found={found:<5} "
              f"w med={int(np.median(widths)) if widths else 0:<4} "
              f"h med={int(np.median(heights)) if heights else 0}")

    if args.mode in ("track", "track-cmp"):
        keep = [s for f in lock_all for s in f if s[0]]
        alive = sum(1 for f in lock_all if any(s[0] for s in f))
        print(f"  raw boxes/frame "
              f"{sum(1 for f in raw_all for s in f if s[0]) / max(1, n):.2f}"
              f" -> locked {len(keep) / max(1, n):.2f}, "
              f"frames with a lock {alive}/{n}")
        stats("raw", raw_all)
        stats("locked", lock_all)
        import imageio.v2 as iio2

        if args.mode == "track":
            writer = iio2.get_writer(str(args.out) + ".mp4", fps=30,
                                     codec="libx264", quality=8)
            for f, (rgb, slots) in enumerate(zip(raw, lock_all)):
                writer.append_data(paint(rgb, slots, scores=score_all[f]))
            writer.close()
            print(f"  wrote {args.out}.mp4")
        else:
            from PIL import Image

            idx = np.linspace(0, n - 1, args.samples).astype(int)
            rows = []
            for i in idx:
                left = label(paint(raw[i], raw_all[i]),
                             "原始检测框 (multi_bbox)")
                right = label(paint(raw[i], lock_all[i], scores=score_all[i]),
                              "锁定框 (track_assoc: 弹性锁定+宽限)")
                rows.append(np.concatenate([left, right], axis=1))
                rows.append(np.zeros((12, DISP_W * 2, 3), dtype=np.uint8))
            Image.fromarray(np.concatenate(rows[:-1], axis=0)).save(
                str(args.out) + ".png")
            print(f"  wrote {args.out}.png")
        return 0

    for mode in modes:
        tight = 0 if mode == "old" else 1
        tight_all = [B.boxes_of(m, tight) for m in masks]
        found = sum(1 for f in tight_all for s in f if s[0])
        heights = [s[4] for f in tight_all for s in f if s[0]]
        widths = [s[3] for f in tight_all for s in f if s[0]]
        med = int(np.median(heights)) if heights else 0
        medw = int(np.median(widths)) if widths else 0
        print(f"  {mode:<4} found={found:<5} w med={medw:<4} h med={med}")

        if args.mode == "both":
            continue
        import imageio.v2 as iio2

        writer = iio2.get_writer(str(args.out) + ".mp4", fps=30,
                                 codec="libx264", quality=8)
        for rgb, slots in zip(raw, tight_all):
            writer.append_data(paint(rgb, slots))
        writer.close()
        print(f"  wrote {args.out}.mp4")

    if args.mode == "both":
        from PIL import Image

        idx = np.linspace(0, n - 1, args.samples).astype(int)
        rows = []
        for i in idx:
            left = label(paint(raw[i], B.boxes_of(masks[i], 0)),
                         "旧 (PNR19): 整条运动带高度")
            right = label(paint(raw[i], B.boxes_of(masks[i], 1)),
                          "新: 收紧到人体范围")
            rows.append(np.concatenate([left, right], axis=1))
            rows.append(np.zeros((12, DISP_W * 2, 3), dtype=np.uint8))
        Image.fromarray(np.concatenate(rows[:-1], axis=0)).save(
            str(args.out) + ".png")
        print(f"  wrote {args.out}.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
