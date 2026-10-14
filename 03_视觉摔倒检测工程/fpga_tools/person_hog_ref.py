"""The HOG + linear-SVM reference score, measured on the same labelled crops.

The person-confidence table in ``person_score.v`` is four 2-bit motion-shape
features.  HOG + a linear SVM is the classic appearance detector (the
reference the project keeps coming back to), so this script answers the
obvious question with numbers instead of opinion: on *these* labelled boxes,
how much better is HOG, and what does the answer cost?

    python fpga/tools/person_hog_ref.py

It writes ``out/person_score/hog_ref.json``; ``person_score_fit.py`` folds
the numbers into ``out/person_score/report.md``.  Nothing here runs on the
chip: a 64x64 window, 8x8 cells, 9 orientation bins, L2-Hys normalised
2x2 blocks is a 1764-dimension dot product per box, which is three orders of
magnitude past the ~1.9k LUT the current bitstream has left.
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(HERE))

import person_score_fit as P  # noqa: E402
from hog_svm import hog  # noqa: E402

DIRS = ["person_roi", "person_roi_raw", "person_roi_raw2"]
WIN, CELL, BINS = 64, 8, 9


def tiles() -> tuple[np.ndarray, np.ndarray, list[dict]]:
    """Load every saved crop with its label and its track key."""
    from PIL import Image

    X, y, meta = [], [], []
    for name in DIRS:
        d = ROOT / "out" / name
        csv_path = d / ("features.csv" if name == "person_roi"
                        else "features_raw.csv")
        if not csv_path.exists():
            continue
        clip_of = {}
        frame_of = {}
        with csv_path.open(newline="", encoding="ascii") as fh:
            for r in csv.DictReader(fh):
                clip_of[int(r["id"])] = r["clip"]
                frame_of[(int(r["id"]), int(r["frame"]))] = r
        for path in sorted(d.glob("[rt]*.png")):
            parts = path.stem.split("_")
            if len(parts) != 2 or parts[1][0] != "f":
                continue
            box_id = int(parts[0][1:])
            frame = int(parts[1][1:])
            clip = clip_of.get(box_id)
            if clip is None:
                continue
            lab = P.label_of(clip, box_id)
            if lab is None:
                continue
            img = Image.open(path).convert("L").resize((WIN, WIN))
            X.append(hog(np.asarray(img), WIN, CELL, BINS, False, "l2hys"))
            y.append(lab)
            meta.append({"src": name, "clip": clip, "id": box_id,
                         "frame": frame, "y": lab})
    return np.array(X, dtype=np.float32), np.array(y), meta


def fit_predict(Xtr, ytr, Xte, C=0.1):
    from sklearn.linear_model import LogisticRegression
    from sklearn.preprocessing import StandardScaler

    sc = StandardScaler().fit(Xtr)
    model = LogisticRegression(C=C, max_iter=3000)
    model.fit(sc.transform(Xtr), ytr)
    return model.predict_proba(sc.transform(Xte))[:, 1]


def folds_of(meta, k=5):
    """Same two protocols as person_score_fit: scene-internal and cross-scene."""
    scene = np.full(len(meta), -1)
    cross = np.zeros(len(meta), dtype=int)
    groups = sorted({(m["src"], m["clip"], m["id"]) for m in meta})
    for clip in sorted({m["clip"] for m in meta}):
        sub = sorted({(m["src"], m["id"]) for m in meta if m["clip"] == clip})
        assign = {g: i % k for i, g in enumerate(sub)}
        for i, m in enumerate(meta):
            if m["clip"] == clip:
                scene[i] = assign[(m["src"], m["id"])]
    for i, g in enumerate(groups):
        for j, m in enumerate(meta):
            if (m["src"], m["clip"], m["id"]) == g:
                cross[j] = i % k
    return scene, cross


def run_protocol(X, y, meta, fold, k, name):
    pred = np.zeros(len(y))
    if name == "scene":
        for clip in sorted({m["clip"] for m in meta}):
            idx = np.array([i for i, m in enumerate(meta) if m["clip"] == clip])
            for f in range(k):
                te = [i for i in idx if fold[i] == f]
                if not te:
                    continue
                tr = [i for i in range(len(y)) if i not in set(te)]
                pred[te] = fit_predict(X[tr], y[tr], X[te])
    else:
        for f in range(k):
            te = np.where(fold == f)[0]
            tr = np.where(fold != f)[0]
            if len(set(y[tr])) < 2:
                pred[te] = float(y[tr].mean())
            else:
                pred[te] = fit_predict(X[tr], y[tr], X[te])
    return pred


def main() -> int:
    X, y, meta = tiles()
    print(f"{len(y)} crops, {int(y.sum())} person / {int(len(y) - y.sum())} not")
    for clip in sorted({m["clip"] for m in meta}):
        sel = [m for m in meta if m["clip"] == clip]
        print(f"  {clip:32s} {len(sel):4d} crops, "
              f"{sum(m['y'] for m in sel):4d} person")
    scene_fold, cross_fold = folds_of(meta)

    metrics = {}
    for name, fold in (("within scene", scene_fold),
                       ("cross scene", cross_fold)):
        p = run_protocol(X, y, meta, fold, 5, name.split()[0])
        m = P.metrics(y, (p * 100).astype(int))
        m["auc"] = round(P.auc(y, p), 4)
        metrics[name] = m
        print(f"{name:14s} acc={m['acc']:.3f} prec={m['prec']:.3f} "
              f"rec={m['rec']:.3f} f1={m['f1']:.3f} auc={m['auc']:.3f}")

    out = {"window": WIN, "cell": CELL, "bins": BINS,
           "dims": int(X.shape[1]), "n": len(y),
           "person": int(y.sum()), "metrics": metrics,
           "note": "offline reference only; 1764-dim dot product per box, "
                   "does not fit the current EG4S20 bitstream"}
    path = ROOT / "out" / "person_score" / "hog_ref.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=1), encoding="ascii")
    print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
