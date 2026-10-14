"""One row per tracked box: the features the FPGA can cheaply compute, plus a
context crop so a human can label it ("this box is a person, that one is a
bush / a plastic bag / a lamp reflection").

This is the raw material for the person-confidence score that the HDMI
overlay prints next to every box.  Nothing here is a neural network: the
features are compares and one small multiply, i.e. what
``rtl/person_score.v`` will be able to do in the same pixel window.

Features (each 2 bits, mirroring the planned RTL):

  evid    0 = no history, 1 = first match, 2 = two matches, 3 = established
  aspect  h vs w bucket            (0 very wide .. 3 tall)
  fill    blob area / box area     (0 hollow .. 3 solid)
  size    blob area bucket         (0 tiny .. 3 large)

Crops are taken from the *published* (lock) box scaled 1.6x so the montage
shows the person, not just the moving limb.

Usage::

    python person_roi.py --clips ../../datasets/crowd/crowd07_hogsvm.mp4 \
        --out ../../out/person_roi --frames 400 --sheet-samples 4
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(HERE))

import box_size_stats as B  # noqa: E402
import track_model as T  # noqa: E402

TILE = 72
COLS = 8
ROWS = 5


def vision_to_display(box, scale: float = 1.0):
    """Vision raster (320x180) box -> display raster (640x480) box."""
    x, y, w, h = (float(v) for v in box[1:5])
    cx = x + w / 2.0
    cy = y + h / 2.0
    w *= scale
    h *= scale
    px0 = 2.0 * (cx - w / 2.0)
    px1 = 2.0 * (cx + w / 2.0)
    py0 = 8.0 * (cy - h / 2.0) / 3.0
    py1 = 8.0 * (cy + h / 2.0) / 3.0
    return px0, py0, px1, py1


def crop(frame: np.ndarray, box, scale: float = 1.6) -> np.ndarray | None:
    from PIL import Image

    px0, py0, px1, py1 = vision_to_display(box, scale)
    h, w = frame.shape[:2]
    x0 = int(max(0, min(w - 1, px0)))
    x1 = int(max(x0 + 1, min(w, px1)))
    y0 = int(max(0, min(h - 1, py0)))
    y1 = int(max(y0 + 1, min(h, py1)))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    img = Image.fromarray(frame[y0:y1, x0:x1])
    return np.asarray(img.resize((TILE, TILE)))


def buckets(w: int, h: int, area: int, box_area: int, evid: int):
    """The 2-bit feature buckets, integer-only like the RTL."""
    if h * 2 < w:
        aspect = 0
    elif h < w:
        aspect = 1
    elif h < 2 * w:
        aspect = 2
    else:
        aspect = 3

    if area * 4 < box_area:
        fill = 0
    elif area * 2 < box_area:
        fill = 1
    elif area * 4 < 3 * box_area:
        fill = 2
    else:
        fill = 3

    if area < 160:
        size = 0
    elif area < 400:
        size = 1
    elif area < 900:
        size = 2
    else:
        size = 3
    return evid, aspect, fill, size


# The deployed RTL table, read from the same artifact person_score_fit.py
# writes and rtl/person_score.v is generated from.  Keeping it in one place
# stops the emulator and the bitstream drifting apart.
_SCORE_JSON = ROOT / "out" / "person_score" / "model.json"
SCORE_TABLE = json.loads(_SCORE_JSON.read_text(encoding="ascii"))["table"]


def score(evid: int, aspect: int, fill: int, size: int) -> int:
    """The deployed RTL confidence, in percent, through the same table."""
    idx = ((int(evid) & 3) << 6) | ((int(aspect) & 3) << 4) | \
          ((int(fill) & 3) << 2) | (int(size) & 3)
    return int(SCORE_TABLE[idx])


def tracks_of(seq):
    live = [None] * T.NUM_BLOBS
    out = []
    for f, frame in enumerate(seq):
        for s in range(T.NUM_BLOBS):
            slot = frame[s]
            if slot[0]:
                if live[s] is None:
                    live[s] = {"slot": s, "start": f, "frames": []}
                live[s]["frames"].append(f)
            elif live[s] is not None:
                out.append(live[s])
                live[s] = None
    out.extend(t for t in live if t is not None)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--clips", type=Path, nargs="+", required=True)
    ap.add_argument("--frames", type=int, default=400)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--mode", choices=["tracks", "raw"], default="tracks",
                    help="tracks: one row per frame of every locked track; "
                         "raw: every unfiltered multi_bbox detection (this is "
                         "where the plastic bag / bush / lamp blobs live)")
    ap.add_argument("--stride", type=int, default=2,
                    help="raw mode: keep one detection every N frames")
    ap.add_argument("--sheet-samples", type=int, default=4,
                    help="crops kept per track (evenly spaced)")
    ap.add_argument("--csv", type=Path, default=None)
    args = ap.parse_args()

    import imageio.v3 as iio
    from PIL import Image, ImageDraw

    args.out.mkdir(parents=True, exist_ok=True)
    csv_path = args.csv or (args.out / "features.csv")
    rows = []
    next_id = 0

    for clip in args.clips:
        frames = []
        for i, fr in enumerate(iio.imiter(clip, plugin="FFMPEG")):
            if i >= args.frames:
                break
            fr = np.asarray(fr)
            h, w = fr.shape[:2]
            if (w, h) != (640, 480):
                ys = np.minimum(np.arange(480) * h // 480, h - 1)
                xs = np.minimum(np.arange(640) * w // 640, w - 1)
                fr = fr[ys][:, xs]
            frames.append(fr)
        if not frames:
            print(f"SKIP {clip.name}: no frames")
            continue
        masks = B.masks_of(clip, len(frames), 2, 2)
        n = min(len(frames), len(masks))
        frames = frames[:n]
        raw = []
        for m in masks[:n]:
            boxes = [tuple(int(v) for v in b) for b in B.boxes_of(m, 1)]
            # multi_bbox publishes four slots every frame; an empty band
            # simply means all four are empty.
            boxes += [(0, 0, 0, 0, 0, 0)] * (T.NUM_BLOBS - len(boxes))
            raw.append(boxes[:T.NUM_BLOBS])
        trace: list[list[int]] = []
        lock = T.track_slots(raw, trace)
        trs = tracks_of(lock)
        print(f"{clip.name}: {n} frames, {len(trs)} tracks")

        if args.mode == "raw":
            tiles = []
            for k in range(0, n, max(1, args.stride)):
                for s in range(T.NUM_BLOBS):
                    box = raw[k][s]
                    if not box[0]:
                        continue
                    ev = trace[k][s]
                    as_, fi, si = buckets(box[3], box[4], box[5],
                                          max(1, box[3] * box[4]), ev)[1:]
                    rows.append({
                        "id": next_id, "clip": clip.name, "slot": s,
                        "frame": k, "life": 1,
                        "x": box[1], "y": box[2], "w": box[3], "h": box[4],
                        "area": box[5],
                        "evid": ev, "aspect": as_, "fill": fi, "size": si,
                        "sample": 1,
                    })
                    img = crop(frames[k], box, 2.4)
                    if img is not None:
                        Image.fromarray(img).save(
                            args.out / f"r{next_id:04d}_f{k:04d}.png")
                        tiles.append((next_id, img))
                    next_id += 1
            per_sheet = COLS * ROWS
            cell_w, cell_h = TILE + 4, TILE + 16
            for s in range(0, len(tiles), per_sheet):
                chunk = tiles[s:s + per_sheet]
                canvas = Image.new("RGB",
                                   (COLS * cell_w, ROWS * cell_h),
                                   (18, 18, 18))
                dr = ImageDraw.Draw(canvas)
                for i, (tid, img) in enumerate(chunk):
                    cx = (i % COLS) * cell_w + 2
                    cy = (i // COLS) * cell_h + 2
                    canvas.paste(Image.fromarray(img), (cx, cy))
                    dr.text((cx + 2, cy + TILE + 1), f"#{tid}",
                            fill=(255, 255, 0))
                canvas.save(args.out /
                            f"raw_{clip.stem[:16]}_{s // per_sheet}.png")
            print(f"  raw detections -> {len(tiles)} crops")
            continue

        sheets, tiles = [], []
        for t in trs:
            fl = t["frames"]
            picks = [fl[int(round((len(fl) - 1) * j /
                                  max(1, args.sheet_samples - 1)))]
                     for j in range(args.sheet_samples)]
            t["id"] = next_id
            next_id += 1
            picks = set(picks)
            tile_made = False
            for k in fl:
                box = lock[k][t["slot"]]
                ev = trace[k][t["slot"]]
                as_, fi, si = buckets(box[3], box[4], box[5],
                                      max(1, box[3] * box[4]), ev)[1:]
                rows.append({
                    "id": t["id"],
                    "clip": clip.name,
                    "slot": t["slot"],
                    "frame": k,
                    "life": len(fl),
                    "x": box[1], "y": box[2], "w": box[3], "h": box[4],
                    "area": box[5],
                    "evid": ev, "aspect": as_, "fill": fi, "size": si,
                    "sample": int(k in picks),
                })
                if k in picks or (not tile_made and k == fl[-1]):
                    img = crop(frames[k], box)
                    if img is not None:
                        Image.fromarray(img).save(
                            args.out / f"t{t['id']:04d}_f{k:04d}.png")
                        if not tile_made:
                            tiles.append((t["id"], img))
                            tile_made = True

        per_sheet = COLS * ROWS
        cell_w, cell_h = TILE + 4, TILE + 16
        for s in range(0, len(tiles), per_sheet):
            chunk = tiles[s:s + per_sheet]
            canvas = Image.new("RGB",
                               (COLS * cell_w, ROWS * cell_h), (18, 18, 18))
            dr = ImageDraw.Draw(canvas)
            for i, (tid, img) in enumerate(chunk):
                cx = (i % COLS) * cell_w + 2
                cy = (i // COLS) * cell_h + 2
                canvas.paste(Image.fromarray(img), (cx, cy))
                dr.text((cx + 2, cy + TILE + 1), f"#{tid}", fill=(255, 255, 0))
            path = args.out / f"sheet_{clip.stem[:16]}_{s // per_sheet}.png"
            canvas.save(path)
            sheets.append(path.name)
        print(f"  crops -> {len(tiles)} tiles, sheets {sheets}")

    with csv_path.open("w", newline="", encoding="ascii") as fh:
        wr = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)
    print(f"wrote {csv_path} ({len(rows)} rows)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
