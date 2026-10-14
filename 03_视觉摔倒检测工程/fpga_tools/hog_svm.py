#!/usr/bin/env python3
"""Motion ROI + HOG + linear SVM: offline prototype for the EG4S20 build.

The current bitstream publishes tracker boxes from motion alone, and the
posture decision is a hand-tuned collapse rule.  This tool measures what the
next step -- classifying the tracked ROI with a HOG descriptor and a linear
SVM -- would buy, before any RTL is written:

    gray_front (320x180) -> frame_diff -> morph -> multi_bbox -> track_assoc
      -> crop the tracked box -> letterbox to a square window
      -> HOG (block-normalised cell histograms)
      -> linear SVM score

The front end and the tracker are the same Python models the RTL is checked
against (``multi_model.frames_model`` and ``track_model.track_slots``), so the
samples below are the crops the chip will actually see at run time -- motion
boxes only cover the moving limbs, and the tracker is what turns them into a
box around the whole person.

Labels come from the GMDCSA-24 per-clip CSVs: every second inside a
``Falling`` interval is positive, everything else (sitting, standing, walking,
sleeping, reading) is negative.  Evaluation is leave-one-subject-out, and the
hand-tuned box-shape rule is scored on the same folds as a baseline.

Usage::

    python hog_svm.py                       # full study, writes out/hog_svm/
    python hog_svm.py --configs 96 --folds 1
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(HERE))

from pixel_check import (  # noqa: E402
    KEEP_ROWS, down2_box, luma_from_565, quantise565,
)
from morph_check import morph_ref  # noqa: E402

import multi_model  # noqa: E402
import track_model  # noqa: E402

DATASET = ROOT / "datasets" / "gmdcsa24"
DIFF_THRESH = 10

CONFIGS = {
    # name: (window, cell px, orientation bins)
    "64": (64, 8, 9),
    "96": (96, 8, 9),
    "64x128": (64, 8, 9),
}


@dataclass
class Clip:
    path: Path
    subject: str
    kind: str
    intervals: list[tuple[float, float, bool]]

    @property
    def subject_index(self) -> int:
        m = re.search(r"(\d+)", self.subject)
        return int(m.group(1)) - 1 if m else 0

    def label(self, t: float) -> int | None:
        for t0, t1, fall in self.intervals:
            if t0 <= t < t1:
                return int(fall)
        return None


def parse_intervals(text: str) -> list[tuple[float, float, bool]]:
    out = []
    pat = re.compile(r"([A-Za-z][A-Za-z ]*?)\s*(?:\([^)]*\))?\s*"
                     r"\[\s*([\d.]+)\s*to\s*([\d.]+)\s*\]")
    for name, t0, t1 in pat.findall(text or ""):
        name = name.strip().lower()
        out.append((float(t0), float(t1), name.startswith("fall")))
    return out


def load_clips(dataset: Path) -> list[Clip]:
    clips = []
    for csv_path in sorted(dataset.glob("*/*.csv")):
        subject = csv_path.parent.name
        kind = csv_path.stem
        with open(csv_path, newline="", encoding="utf-8-sig") as fh:
            for row in csv.DictReader(fh):
                row = {(k or "").strip(): v for k, v in row.items()}
                name = (row.get("File Name") or "").strip()
                if not name:
                    continue
                clip = csv_path.parent / kind / name
                if not clip.exists():
                    continue
                intervals = parse_intervals(row.get("Classes", ""))
                if intervals:
                    clips.append(Clip(clip, subject, kind, intervals))
    return clips


def luma_clip(clip: Path) -> np.ndarray:
    """The RTL front end: RGB565 -> luma -> 2x2 box -> 320x180."""
    import imageio.v3 as iio

    frames = []
    for frame in iio.imiter(clip, plugin="FFMPEG"):
        frame = np.asarray(frame)
        h, w = frame.shape[:2]
        if (w, h) != (640, 480):
            ys = np.minimum(np.arange(480) * h // 480, h - 1)
            xs = np.minimum(np.arange(640) * w // 640, w - 1)
            frame = frame[ys][:, xs]
        small = down2_box(luma_from_565(quantise565(frame)))
        frames.append(small[KEEP_ROWS])
    if not frames:
        raise SystemExit(f"error: no frames decoded from {clip}")
    return np.stack(frames)


def masks_of(gray: np.ndarray) -> np.ndarray:
    diff = np.abs(gray[1:].astype(np.int16) - gray[:-1].astype(np.int16))
    raw = np.zeros_like(gray, dtype=bool)
    raw[1:] = diff > DIFF_THRESH
    return np.stack([morph_ref(m) for m in raw])


def tracked_rois(gray: np.ndarray):
    """multi_bbox -> track_assoc, exactly as the RTL orders them."""
    masks = masks_of(gray)
    return track_model.track_slots(multi_model.frames_model(masks)), masks


def union_box(masks: np.ndarray, i: int, span: int, seed,
              scale: float, shape: tuple[int, int]) -> tuple[int, int,
                                                             int, int]:
    """Bounding box of the motion in the last ``span`` frames near a seed.

    One frame of differencing only marks the pixels that moved *now*, so a
    walking person shows up as a pair of legs.  Accumulating a short window of
    masks and clipping to the tracker's neighbourhood gives a body-shaped ROI
    instead, which is what a descriptor needs.
    """
    region = masks[max(0, i - span + 1):i + 1].any(axis=0)
    x, y, w, h = scale_box(tuple(seed[:4]), scale, shape)
    sub = region[y:y + h, x:x + w]
    if not sub.any():
        return (x, y, w, h)
    ys, xs = np.where(sub)
    return (x + int(xs.min()), y + int(ys.min()),
            int(xs.max() - xs.min()) + 1, int(ys.max() - ys.min()) + 1)


def letterbox(img: np.ndarray, box: tuple[int, int, int, int],
              win: int) -> np.ndarray:
    """Nearest-neighbour fit of the ROI into a win x win window.

    Aspect ratio is preserved (the box-shape cue lives there); RTL would run
    this with a running error accumulator, one source pixel per output pixel.
    """
    h, w = img.shape
    x0, y0, bw, bh = box
    x0 = max(0, min(x0, w - 1))
    y0 = max(0, min(y0, h - 1))
    bw = max(1, min(bw, w - x0))
    bh = max(1, min(bh, h - y0))
    crop = img[y0:y0 + bh, x0:x0 + bw]
    if bh * win >= bw * win:           # taller: fit the height
        oh = win
        ow = max(1, int(round(bw * win / bh)))
    else:
        ow = win
        oh = max(1, int(round(bh * win / bw)))
    ys = np.minimum(np.arange(oh) * bh // oh, bh - 1)
    xs = np.minimum(np.arange(ow) * bw // ow, bw - 1)
    small = crop[ys][:, xs]
    out = np.zeros((win, win), dtype=np.uint8)
    top = (win - oh) // 2
    left = (win - ow) // 2
    out[top:top + oh, left:left + ow] = small
    return out


def scale_box(box: tuple[int, int, int, int], scale: float,
              shape: tuple[int, int]) -> tuple[int, int, int, int]:
    """Grow a tracked box around its centre, clamped to the raster.

    A frame-difference box only covers the pixels that moved, so the crop is
    a limb or a torso rather than a person.  Growing the box is the cheapest
    fix and the one the RTL can do with a shift.
    """
    if scale == 1.0:
        return box
    h, w = shape
    x, y, bw, bh = box
    cx, cy = x + bw / 2.0, y + bh / 2.0
    nw, nh = max(8, int(round(bw * scale))), max(8, int(round(bh * scale)))
    nx = int(round(cx - nw / 2.0))
    ny = int(round(cy - nh / 2.0))
    nx = max(0, min(nx, w - 1))
    ny = max(0, min(ny, h - 1))
    nw = min(nw, w - nx)
    nh = min(nh, h - ny)
    return (nx, ny, nw, nh)


def gradients(patch: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    g = patch.astype(np.int16)
    gx = np.empty_like(g)
    gy = np.empty_like(g)
    gx[:, 1:-1] = g[:, 2:] - g[:, :-2]
    gx[:, 0] = g[:, 1] - g[:, 0]
    gx[:, -1] = g[:, -1] - g[:, -2]
    gy[1:-1, :] = g[2:, :] - g[:-2, :]
    gy[0, :] = g[1, :] - g[0, :]
    gy[-1, :] = g[-1, :] - g[-2, :]
    mag = (np.abs(gx) + np.abs(gy)).astype(np.float32)
    ang = np.arctan2(gy.astype(np.float32), gx.astype(np.float32))
    ang = np.degrees(ang) % 180.0
    return mag, ang


def cell_hist(mag: np.ndarray, ang: np.ndarray, win: int, cell: int,
              bins: int, soft: bool) -> np.ndarray:
    n = win // cell
    pos = ang / (180.0 / bins) - 0.5
    lo = np.floor(pos).astype(np.int64)
    frac = (pos - lo).astype(np.float32)

    def fold(idx: np.ndarray, weight: np.ndarray) -> np.ndarray:
        idx = np.mod(idx, bins)
        m = mag.reshape(n, cell, n, cell).transpose(0, 2, 1, 3).reshape(n, n, -1)
        i = idx.reshape(n, cell, n, cell).transpose(0, 2, 1, 3).reshape(n, n, -1)
        w = weight.reshape(n, cell, n, cell).transpose(0, 2, 1, 3).reshape(n, n, -1)
        out = np.zeros((n, n, bins), dtype=np.float32)
        for b in range(bins):
            out[..., b] = np.where(i == b, m * w, 0.0).sum(axis=2)
        return out

    if soft:
        return fold(lo + 1, frac) + fold(lo, 1.0 - frac)
    return fold(np.rint(pos).astype(np.int64), np.ones_like(frac))


def block_norm(hist: np.ndarray, bins: int, norm: str) -> np.ndarray:
    from numpy.lib.stride_tricks import sliding_window_view

    ny, nx, _ = hist.shape
    view = sliding_window_view(hist, (2, 2, bins))[:, :, 0]
    v = view.reshape(ny - 1, nx - 1, 4 * bins).astype(np.float32)
    if norm == "none":
        return v.reshape(-1)
    if norm == "l1sqrt":
        v = v / (v.sum(axis=-1, keepdims=True) + 1e-3)
        return np.sqrt(v).reshape(-1)
    v = v / np.sqrt((v * v).sum(axis=-1, keepdims=True) + 1e-3)
    v = np.minimum(v, 0.2)
    v = v / np.sqrt((v * v).sum(axis=-1, keepdims=True) + 1e-3)
    return v.reshape(-1)


def hog(patch: np.ndarray, win: int, cell: int, bins: int, soft: bool,
        norm: str) -> np.ndarray:
    mag, ang = gradients(patch)
    hist = cell_hist(mag, ang, win, cell, bins, soft)
    return (block_norm(hist, bins, norm) * 128.0).astype(np.float32)


def build(config: str, clips: list[Clip], stride: int, cap: int,
          soft: bool, norm: str, shape_feat: bool,
          roi_scale: float = 1.0, roi_union: int = 0,
          roi_box_union: int = 0) -> dict:
    win, cell, bins = CONFIGS[config]
    X, y, meta, groups = [], [], [], []
    for clip in clips:
        gray = luma_clip(clip.path)
        rois, masks = tracked_rois(gray)
        idx = list(range(1, len(gray), stride))[:cap]
        hist: list[list[tuple[int, tuple[int, int, int, int]]]] = \
            [[] for _ in range(multi_model.NUM_BLOBS)]
        for i in idx:
            t = i / 25.0
            lab = clip.label(t)
            if lab is None:
                continue
            for slot, box in enumerate(rois[i]):
                if not box or not box[0]:
                    continue
                _, x, yy, w, h, area = box
                if w < 8 or h < 8 or area < 100:
                    continue
                rx, ry, rw, rh = scale_box((x, yy, w, h), roi_scale,
                                           gray[i].shape)
                if roi_union:
                    rx, ry, rw, rh = union_box(masks, i, roi_union,
                                               (x, yy, w, h), 2.0,
                                               gray[i].shape)
                if roi_box_union:
                    # Box-level accumulation: what the RTL can do with four
                    # registers per slot instead of a mask history.
                    bx0, by0 = x, yy
                    bx1, by1 = x + w, yy + h
                    for j, bj in hist[slot]:
                        if i - j > roi_box_union:
                            continue
                        bx0, by0 = min(bx0, bj[0]), min(by0, bj[1])
                        bx1, by1 = max(bx1, bj[0] + bj[2]), \
                            max(by1, bj[1] + bj[3])
                    rx, ry, rw, rh = scale_box((bx0, by0, bx1 - bx0,
                                                by1 - by0), roi_scale,
                                               gray[i].shape)
                patch = letterbox(gray[i], (rx, ry, rw, rh), win)
                feat = hog(patch, win, cell, bins, soft, norm)
                if shape_feat:
                    # The box geometry the RTL already carries: a fallen
                    # person's tracked box is wide and low, and the motion
                    # fills less of it than a standing torso does.
                    extra = np.array([8.0 * rh / max(1, rw),
                                      8.0 * area / max(1, rw * rh),
                                      8.0 * min(1.0, area / 1200.0)],
                                     dtype=np.float32)
                    feat = np.concatenate([feat, extra])
                X.append(feat)
                y.append(lab)
                meta.append((clip.subject, clip.path.name, i, slot,
                             rx, ry, rw, rh, area, lab))
                groups.append(clip.subject_index)
                if roi_box_union:
                    hist[slot].append((i, (x, yy, w, h)))
                    hist[slot] = [e for e in hist[slot]
                                  if i - e[0] <= roi_box_union]
        print(f"  {clip.subject}/{clip.kind}/{clip.path.name}: "
              f"{len(gray)} frames", flush=True)
    return {
        "X": np.stack(X) if X else np.zeros((0, 1), np.float32),
        "y": np.array(y, dtype=np.int8),
        "meta": meta,
        "group": np.array(groups, dtype=np.int64),
        "config": config,
        "win": win,
        "cell": cell,
        "bins": bins,
        "soft": soft,
        "norm": norm,
    }


def fit_predict(Xtr, ytr, Xte, C: float):
    from sklearn.svm import LinearSVC

    clf = LinearSVC(C=C, class_weight="balanced", max_iter=8000,
                    dual=True, tol=1e-4)
    clf.fit(Xtr, ytr)
    return clf, clf.decision_function(Xte)


def scores(y_true, y_pred) -> dict:
    y_true = np.asarray(y_true)
    y_pred = np.asarray(y_pred)
    tp = int(((y_true == 1) & (y_pred == 1)).sum())
    fp = int(((y_true == 0) & (y_pred == 1)).sum())
    fn = int(((y_true == 1) & (y_pred == 0)).sum())
    tn = int(((y_true == 0) & (y_pred == 0)).sum())
    prec = tp / (tp + fp) if tp + fp else 0.0
    rec = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2 * prec * rec / (prec + rec) if prec + rec else 0.0
    acc = (tp + tn) / max(1, tp + tn + fp + fn)
    return {"tp": tp, "fp": fp, "fn": fn, "tn": tn, "acc": acc,
            "prec": prec, "rec": rec, "f1": f1}


def shape_baseline(X, shape, folds) -> dict:
    """Tune one box-shape threshold on the train folds, score the test fold.

    This is what the board does today in spirit: a hand rule on the tracked
    box (height/width and how much of the box the motion fills), with no
    appearance model at all.
    """
    y = X["y"]
    hw = np.array([m[7] / max(1, m[6]) for m in X["meta"]], np.float32)
    fill = np.array([m[8] / max(1, m[6] * m[7]) for m in X["meta"]],
                    np.float32)
    pred = np.zeros_like(y)
    for sub in sorted(set(folds)):
        te = np.array([s == sub for s in folds])
        tr = ~te
        best, best_thr = -1.0, 1.0
        for thr in np.arange(0.2, 2.01, 0.05):
            cand = shape(hw[tr], fill[tr], thr).astype(np.int8)
            f1 = scores(y[tr], cand)["f1"]
            if f1 > best:
                best, best_thr = f1, thr
        pred[te] = shape(hw[te], fill[te], best_thr).astype(np.int8)
    return scores(y, pred)


def shape_rule(hw, fill, thr):
    return (hw < thr) & (fill > 0.3)


def run_study(out_dir: Path, configs, folds_wanted, args) -> dict:
    clips = load_clips(DATASET)
    if not clips:
        raise SystemExit(f"error: no labelled clips under {DATASET}")
    out_dir.mkdir(parents=True, exist_ok=True)
    subjects = sorted({c.subject for c in clips})
    print(f"{len(clips)} clips, {len(subjects)} subjects: {subjects}")

    report = {"clips": len(clips), "subjects": subjects, "configs": {}}
    cs = [float(c) for c in args.sweep_c.split(",") if c.strip()]
    for config in configs:
        win, cell, bins = CONFIGS[config]
        print(f"\n=== HOG {win}x{win}, cell {cell}, {bins} bins, "
              f"shape={args.shape_features}, C in {cs} ===")
        data = build(config, clips, args.stride, args.per_clip,
                     args.soft, args.norm, args.shape_features,
                     args.roi_scale, args.roi_union, args.roi_box_union)
        if len(data["y"]) == 0:
            print("  no samples")
            continue
        X, y = data["X"], data["y"]
        pos = int((y == 1).sum())
        print(f"  samples {len(y)}  fall {pos}  adl {len(y) - pos}  "
              f"dims {X.shape[1]}")
        # Cache before the SVM fits: the feature build is the slow half.
        np.savez_compressed(
            out_dir / f"hog_{config}{'_shape' if args.shape_features else ''}"
                      f"{'_soft' if args.soft else ''}"
                      f"{'' if args.roi_scale == 1.0 else f'_roi{args.roi_scale:g}'}"
                      f"{'' if not args.roi_union else f'_u{args.roi_union}'}"
                      f"{'' if not args.roi_box_union else f'_bu{args.roi_box_union}'}"
                      f".npz",
            X=X.astype(np.float32), y=y, group=data["group"],
            meta=np.array([(m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9])
                           for m in data["meta"]], dtype=np.int64))

        folds = [m[0] for m in data["meta"]]
        entry = {"win": win, "cell": cell, "bins": bins,
                 "samples": int(len(y)), "fall": pos,
                 "dims": int(X.shape[1]),
                 "shape_features": bool(args.shape_features),
                 "soft_bins": bool(args.soft),
                 "norm": args.norm, "sweep": {}}

        for C in cs:
            folds_out, perfold = {}, []
            for sub in subjects[:folds_wanted]:
                te = np.array([s == sub for s in folds])
                tr = ~te
                clf, dec = fit_predict(X[tr], y[tr], X[te], C)
                s = scores(y[te], (dec > 0).astype(np.int8))
                folds_out[sub] = s
                perfold.append(s)
                print(f"  C={C:<6g} LOSO {sub:<10} n={int(te.sum()):<5} "
                      f"acc={s['acc']:.3f} prec={s['prec']:.3f} "
                      f"rec={s['rec']:.3f} f1={s['f1']:.3f} "
                      f"(tp{s['tp']} fp{s['fp']} fn{s['fn']} tn{s['tn']})")
            entry["sweep"][str(C)] = {
                "folds": folds_out,
                "f1_mean": float(np.mean([s["f1"] for s in perfold])),
                "acc_mean": float(np.mean([s["acc"] for s in perfold])),
            }
        best_c = max(entry["sweep"], key=lambda k: entry["sweep"][k]["f1_mean"])
        entry["best_C"] = float(best_c)
        entry["loso_f1_mean"] = entry["sweep"][best_c]["f1_mean"]
        entry["loso_acc_mean"] = entry["sweep"][best_c]["acc_mean"]
        print(f"  best C={best_c}  f1={entry['loso_f1_mean']:.3f} "
              f"acc={entry['loso_acc_mean']:.3f}")
        entry["shape"] = shape_baseline(data, shape_rule, folds)
        print(f"  box-shape rule      acc={entry['shape']['acc']:.3f} "
              f"f1={entry['shape']['f1']:.3f}")
        report["configs"][config] = entry
    (out_dir / "hog_svm_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8")
    return report


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--configs", default="64,96")
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--per-clip", type=int, default=120)
    ap.add_argument("--folds", type=int, default=4)
    ap.add_argument("--sweep-c", default="0.003,0.01,0.03,0.1,0.3,1")
    ap.add_argument("--shape-features", action="store_true",
                    help="append box h/w, fill and area to the HOG vector")
    ap.add_argument("--roi-scale", type=float, default=1.0,
                    help="grow the tracked box around its centre before the "
                         "crop (1.0 = the raw motion box)")
    ap.add_argument("--roi-union", type=int, default=0,
                    help="bounding box of the motion over the last N frames "
                         "near the tracked box (0 = single-frame box)")
    ap.add_argument("--roi-box-union", type=int, default=0,
                    help="union of the last N tracked boxes of the slot "
                         "(the variant the RTL can afford)")
    ap.add_argument("--soft", action="store_true",
                    help="linear interpolation between orientation bins")
    ap.add_argument("--norm", default="l2hys",
                    choices=["l2hys", "l1sqrt", "none"])
    ap.add_argument("--out", default=str(ROOT / "out" / "hog_svm"))
    args = ap.parse_args()

    run_study(Path(args.out), [c.strip() for c in args.configs.split(",")
                               if c.strip()], args.folds, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
