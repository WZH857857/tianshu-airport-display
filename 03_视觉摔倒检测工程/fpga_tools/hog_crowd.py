#!/usr/bin/env python3
"""Run the HOG-SVM fall classifier over a crowd clip and draw its verdicts.

Training data is GMDCSA-24 (labelled falls vs activities of daily living);
the target clip is whatever the board will be pointed at.  The point of the
tool is the false-alarm count: a crowd of walkers must not light up, so the
picture shows every tracked ROI with the SVM's own score next to it.

Usage::

    python hog_crowd.py --clip ../../datasets/crowd/crowd07_hogsvm.mp4 \
        --out ../../out/crowd07_hog --frames 300 --roi-scale 2
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

import hog_svm as H  # noqa: E402
import machine_view as MV  # noqa: E402


def train_model(args) -> tuple[np.ndarray, float]:
    """Fit on every labelled clip and keep the raw (w, b) the RTL would use."""
    cache = Path(args.model)
    if args.roi_union:
        cache = cache.with_name(f"{cache.stem}_u{args.roi_union}"
                                f"{cache.suffix}")
    if cache.exists() and not args.retrain:
        blob = np.load(cache)
        print(f"model: {cache.name} ({blob['w'].shape[0]} weights)")
        return blob["w"].astype(np.float32), float(blob["b"])

    clips = H.load_clips(H.DATASET)
    print(f"training on {len(clips)} clips")
    data = H.build(args.config, clips, args.stride, args.per_clip,
                   False, "l2hys", args.shape_features, args.roi_scale,
                   args.roi_union)
    X, y = data["X"], data["y"]
    from sklearn.svm import LinearSVC

    clf = LinearSVC(C=args.C, class_weight="balanced", max_iter=8000,
                    dual=True, tol=1e-4)
    clf.fit(X, y)
    w = clf.coef_.reshape(-1).astype(np.float32)
    b = float(clf.intercept_[0])
    cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(cache, w=w, b=b, C=args.C,
                        config=args.config, roi_scale=args.roi_scale,
                        roi_union=args.roi_union,
                        shape_features=args.shape_features)
    print(f"model saved: {cache}  train acc "
          f"{float((clf.predict(X) == y).mean()):.3f}")
    return w, b


def features(gray: np.ndarray, box, args, masks=None, frame: int = 0):
    win, cell, bins = H.CONFIGS[args.config]
    if args.roi_union and masks is not None:
        rx, ry, rw, rh = H.union_box(masks, frame, args.roi_union,
                                     tuple(box[1:5]), 2.0, gray.shape)
    else:
        rx, ry, rw, rh = H.scale_box(tuple(box[1:5]), args.roi_scale,
                                     gray.shape)
    in_frame = rx >= 0 and ry >= 0 and rw > 0 and rh > 0
    if not in_frame:
        rx, ry, rw, rh = tuple(box[1:5])
    patch = H.letterbox(gray, (rx, ry, rw, rh), win)
    feat = H.hog(patch, win, cell, bins, False, "l2hys")
    if args.shape_features:
        area = box[5] if len(box) > 5 else 0
        extra = np.array([8.0 * rh / max(1, rw),
                          8.0 * area / max(1, rw * rh),
                          8.0 * min(1.0, area / 1200.0)], dtype=np.float32)
        feat = np.concatenate([feat, extra])
    return feat


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--clip", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--config", default="64")
    ap.add_argument("--C", type=float, default=0.1)
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--per-clip", type=int, default=120)
    ap.add_argument("--roi-scale", type=float, default=2.0)
    ap.add_argument("--roi-union", type=int, default=0)
    ap.add_argument("--shape-features", action="store_true", default=True)
    ap.add_argument("--model", default=str(ROOT / "out" / "hog_svm" /
                                           "model_64.npz"))
    ap.add_argument("--retrain", action="store_true")
    ap.add_argument("--sheet-frames", type=int, default=6)
    args = ap.parse_args()

    w, b = train_model(args)

    gray = H.luma_clip(args.clip)[:args.frames]
    rois, masks = H.tracked_rois(gray)
    import imageio.v2 as iio2
    import imageio.v3 as iio

    raw = []
    for i, frame in enumerate(iio.imiter(args.clip, plugin="FFMPEG")):
        if i >= len(gray):
            break
        raw.append(MV.normalise(np.asarray(frame)))

    hits = 0
    slots_seen = 0
    score_by_slot: dict[int, list[float]] = {}
    sheet_rows = []
    idx = set(np.linspace(0, len(gray) - 1, args.sheet_frames).astype(int))
    writer = iio2.get_writer(str(args.out) + ".mp4", fps=25,
                             codec="libx264", quality=8)
    for i, (rgb, slots) in enumerate(zip(raw, rois)):
        alarm = [False] * len(slots)
        texts = []
        for k, slot in enumerate(slots):
            if not slot or not slot[0]:
                continue
            slots_seen += 1
            feat = features(gray[i], slot, args, masks, i)
            s = float(w @ feat + b)
            score_by_slot.setdefault(k, []).append(s)
            if s > 0:
                alarm[k] = True
                hits += 1
            texts.append((k, slot, s))
        img = MV.paint(rgb, slots, alarm)
        if i in idx:
            from PIL import Image, ImageDraw, ImageFont
            pil = Image.fromarray(img)
            draw = ImageDraw.Draw(pil)
            try:
                font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 20)
            except OSError:
                font = ImageFont.load_default()
            for k, slot, s in texts:
                _f, x, y, bw, bh, _a = slot
                tag = "摔倒" if s > 0 else "人"
                col = (255, 60, 60) if s > 0 else (60, 255, 120)
                draw.rectangle([x * 2, y * 8 // 3 - 24, x * 2 + 150,
                                y * 8 // 3], fill=(0, 0, 0))
                draw.text((x * 2 + 4, y * 8 // 3 - 22),
                          f"{tag} {s:+.2f}", fill=col, font=font)
            sheet_rows.append(np.asarray(pil))
        writer.append_data(img)
    writer.close()

    from PIL import Image

    if sheet_rows:
        sheet = np.concatenate([np.concatenate(sheet_rows[i:i + 2], axis=1)
                                for i in range(0, len(sheet_rows) - 1, 2)],
                               axis=0)
        Image.fromarray(sheet).save(str(args.out) + "_sheet.png")
    print(f"{args.clip.name}: {len(gray)} frames, {slots_seen} ROI "
          f"evaluations, {hits} fall verdicts "
          f"({hits / max(1, slots_seen):.1%} of ROIs)")
    for k in sorted(score_by_slot):
        v = np.array(score_by_slot[k])
        print(f"  slot {k}: n={len(v):<5} score med={np.median(v):+.2f} "
              f"max={v.max():+.2f} >0: {(v > 0).mean():.1%}")
    print(f"  wrote {args.out}.mp4 and _sheet.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
