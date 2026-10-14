"""Fit the per-box "this is a person" confidence that the HDMI overlay prints.

The board carries four 2-bit features per published box and has no room for
a multiplier farm, so the deployed model is a 256-entry table

    idx = {evid, aspect, fill, size}   ->   1 .. 99  (%)

How the table is produced:

  1. ``person_roi.py`` saved one crop per published box (and one row of
     features) for the test videos; the label rules below were read off the
     montage sheets by eye -- one rule per clip, naming the id ranges that
     are people and the ones that are tram body / window reflection / desk,
  2. a logistic regression is fitted on the *indicator* expansion of the
     four ordinal features (each level gets its own weight, so "big" can be
     bad while "medium" stays good -- a plain linear fit cannot say that),
  3. a weak Gaussian prior keeps the level ordering semantically sane: more
     track evidence, person-shaped aspect, solid fill and a body-sized blob
     all push the score up.  Without it the tram scene alone decides that
     "big blob = not a person", and a close-up person -- a face filling the
     box, exactly what the board showed the first time the camera looked at
     one -- would print 1 %.
  4. the fit is quantised to integer weights and turned into the table,
  5. accuracy is measured with **grouped 5-fold cross validation**: folds are
     split by track, never by frame, so no trajectory is seen in training and
     tested on itself.  Leave-one-clip-out is printed as well, but it is a
     scene-transfer number rather than the accuracy of the board, because
     dropping the tram clip drops nearly every negative in the set.

Outputs: ``out/person_score/model.json`` (table + metrics, also read by
machine_view.py so the offline video and the board print the same numbers),
``out/person_score/report.md`` and ``rtl/person_score.v``.

Usage::

    python fpga/tools/person_score_fit.py
"""

from __future__ import annotations

import csv
import json
import sys
import argparse
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent

FEATS = ("evid", "aspect", "fill", "size")
SCALE = 8          # logit units per integer weight step
CLIP_MIN = 1       # never print 0 %, never print 100 %
CLIP_MAX = 99
PRIOR_LAMBDA = 80  # pseudo-observations pulling the fit towards the prior

# (clip, id_lo, id_hi, is_person) -- first match wins, read off
# out/person_roi*/sheet_*.png and out/person_roi_*/raw_*.png.
LABEL_RULES: list[tuple[str, int, int, int]] = [
    # crowd07_hogsvm: pedestrians on a plaza; every crop is a person.
    ("crowd07_hogsvm.mp4", 0, 9999, 1),
    # crowd01: the two walkers in front of the tram are the only people,
    # everything else is tram body, window, reflection or door handle.
    ("crowd01.mp4", 59, 60, 1),
    ("crowd01.mp4", 61, 9999, 0),
    ("crowd01.mp4", 31, 31, 1),
    ("crowd01.mp4", 32, 9999, 0),
    # crowd02 / crowd03 / crowd04: close-up crowd video, people only.
    ("crowd02.mp4", 0, 9999, 1),
    ("crowd03.mp4", 23, 69, 1),
    ("crowd03.mp4", 70, 70, 0),          # fast motion blur, no body
    ("crowd04.mp4", 0, 9999, 1),
    # indoor fall clips: the box holding the body, vs desk / chairs.
    ("day11_event001_confirmed.mp4", 79, 79, 1),
    ("day11_event001_confirmed.mp4", 80, 9999, 0),
    ("day6_event019_confirmed.mp4", 0, 9999, 1),
    # single long tracks: a face close-up and an ADL body part.
    ("13.mp4", 0, 9999, 1),
    ("09.mp4", 0, 9999, 1),
]

SOURCES = [
    ROOT / "out" / "person_roi" / "features.csv",
    ROOT / "out" / "person_roi_raw" / "features_raw.csv",
    ROOT / "out" / "person_roi_raw2" / "features_raw.csv",
]

# Level weights of the semantic prior (level 0 = reference, so 0 by
# construction).  These say what "person-like" means before any data: a
# tracked, tall, solid, body-sized blob.
PRIOR_LEVELS = {
    "evid":   [0.0, 0.5, 1.0, 1.5],
    "aspect": [-1.5, -1.0, 0.5, 0.75],
    "fill":   [-1.0, 0.25, 0.25, 0.25],
    "size":   [-1.0, 0.75, 0.75, -0.25],
    "lifeq":  [-0.75, 0.25, 0.75, 1.25],
}


def label_of(clip: str, box_id: int) -> int | None:
    for rule_clip, lo, hi, lab in LABEL_RULES:
        if rule_clip == clip and lo <= box_id <= hi:
            return lab
    return None


def load_rows() -> list[dict]:
    rows: list[dict] = []
    for path in SOURCES:
        if not path.exists():
            print(f"missing {path}")
            continue
        with path.open(newline="", encoding="ascii") as fh:
            for r in csv.DictReader(fh):
                lab = label_of(r["clip"], int(r["id"]))
                if lab is None:
                    print(f"unlabelled {r['clip']}#{r['id']}")
                    continue
                rows.append({
                    "clip": r["clip"], "src": path.parent.name,
                    "id": int(r["id"]), "frame": int(r["frame"]),
                    "life": int(r["life"]),
                    "lifeq": life_bucket(int(r["life"])),
                    "area": int(r["area"]),
                    **{f: int(r[f]) for f in ("evid", "aspect", "fill", "size")},
                    "y": lab,
                })
    return rows


def life_bucket(life: int) -> int:
    """Frames published so far, bucketed like the RTL's 2-bit age counter."""
    if life < 4:
        return 0
    if life < 12:
        return 1
    if life < 40:
        return 2
    return 3


def one_hot(row_or_combo) -> np.ndarray:
    """[1, evid1..3, aspect1..3, fill1..3, size1..3] -- level 0 reference."""
    vals = [row_or_combo[f] if isinstance(row_or_combo, dict)
            else row_or_combo[i] for i, f in enumerate(FEATS)]
    x = [1.0]
    for f, v in zip(FEATS, vals):
        x.extend(1.0 if v == k else 0.0 for k in (1, 2, 3))
    return np.array(x)


def design(rows) -> np.ndarray:
    return np.array([one_hot(r) for r in rows])


def prior_vector() -> np.ndarray:
    w0 = [0.0]
    for f in FEATS:
        base = PRIOR_LEVELS[f][0]
        w0.extend(PRIOR_LEVELS[f][k] - base for k in (1, 2, 3))
    return np.array(w0)


def fit_logit(X: np.ndarray, y: np.ndarray, lam: float | None = None,
              w0: np.ndarray | None = None, iters: int = 40) -> np.ndarray:
    """IRLS logistic fit with a Gaussian prior centred on ``w0``."""
    if lam is None:
        lam = PRIOR_LAMBDA
    if w0 is None:
        w0 = prior_vector()
    w = w0.copy()
    eye = np.eye(len(w))
    for _ in range(iters):
        z = X @ w
        p = 1.0 / (1.0 + np.exp(-z))
        g = X.T @ (p - y) + 2.0 * lam * (w - w0)
        s = np.maximum(p * (1.0 - p), 1e-6) * X.T
        H = s @ X + 2.0 * lam * eye
        step = np.linalg.solve(H, g)
        w = w - step
        if np.max(np.abs(step)) < 1e-8:
            break
    return w


def quantise(w: np.ndarray) -> np.ndarray:
    return np.rint(w * SCALE).astype(int)


def combo_index(*vals: int) -> int:
    idx = 0
    for v in vals:
        idx = (idx << 2) | int(v)
    return idx


def table_from_int_weights(w: np.ndarray) -> np.ndarray:
    tab = np.zeros(4 ** len(FEATS), dtype=int)
    for idx in range(4 ** len(FEATS)):
        vals = tuple((idx >> (2 * (len(FEATS) - 1 - i))) & 3
                     for i in range(len(FEATS)))
        z = float(one_hot(vals) @ w) / SCALE
        p = 1.0 / (1.0 + np.exp(-z))
        tab[idx] = int(min(CLIP_MAX, max(CLIP_MIN, round(100 * p))))
    return tab


def table_score(rows, tab: np.ndarray) -> np.ndarray:
    return np.array([tab[combo_index(*[r[f] for f in FEATS])] for r in rows])


def metrics(y: np.ndarray, s: np.ndarray, thr: int = 50) -> dict:
    pred = s >= thr
    tp = int(np.sum(pred & (y == 1)))
    fp = int(np.sum(pred & (y == 0)))
    fn = int(np.sum(~pred & (y == 1)))
    tn = int(np.sum(~pred & (y == 0)))
    acc = (tp + tn) / max(1, len(y))
    prec = tp / max(1, tp + fp)
    rec = tp / max(1, tp + fn)
    f1 = 2 * prec * rec / max(1e-9, prec + rec)
    return {"n": int(len(y)), "tp": tp, "fp": fp, "fn": fn, "tn": tn,
            "acc": round(acc, 4), "prec": round(prec, 4),
            "rec": round(rec, 4), "f1": round(f1, 4),
            "auc": round(auc(y, s), 4)}


def auc(y: np.ndarray, s: np.ndarray) -> float:
    pos, neg = np.asarray(s)[y == 1], np.asarray(s)[y == 0]
    if len(pos) == 0 or len(neg) == 0:
        return float("nan")
    both = np.concatenate([pos, neg])
    try:
        from scipy.stats import rankdata
        ranks = rankdata(both)          # mid-ranks, so ties score 0.5
    except Exception:                    # pragma: no cover
        order = np.argsort(both, kind="stable")
        ranks = np.empty(len(both), dtype=float)
        ranks[order] = np.arange(1, len(both) + 1)
    r = ranks[:len(pos)].sum()
    return float((r - len(pos) * (len(pos) + 1) / 2) / (len(pos) * len(neg)))


def smooth_tracks(rows, s: np.ndarray, alpha: float = 0.25) -> np.ndarray:
    """EMA over the frames of one track, in playback order (RTL behaviour)."""
    out = np.array(s, dtype=float)
    key_prev, acc = None, 0.0
    for i in sorted(range(len(rows)),
                    key=lambda i: (rows[i]["src"], rows[i]["clip"],
                                   rows[i]["id"], rows[i]["frame"])):
        key = (rows[i]["src"], rows[i]["clip"], rows[i]["id"])
        if key != key_prev:
            acc = float(s[i])
            key_prev = key
        else:
            acc += alpha * (float(s[i]) - acc)
        out[i] = acc
    return out


def group_folds(rows, k: int = 5) -> np.ndarray:
    """Fold per *track*, not per frame: a trajectory never spans two folds."""
    groups = sorted({(r["src"], r["clip"], r["id"]) for r in rows})
    y = np.array([r["y"] for r in rows])
    keys = [(r["src"], r["clip"], r["id"]) for r in rows]
    try:
        from sklearn.model_selection import StratifiedGroupKFold
        splitter = StratifiedGroupKFold(n_splits=k, shuffle=True,
                                        random_state=7)
        assign = {}
        for fold, (_tr, te) in enumerate(splitter.split(rows, y, keys)):
            for i in te:
                assign[keys[i]] = fold
    except Exception:                    # pragma: no cover
        # Fall back to a greedy balance of the positive count per fold.
        counts = {}
        for g in groups:
            sel = [r for r in rows if (r["src"], r["clip"], r["id"]) == g]
            counts[g] = (sum(r["y"] for r in sel), len(sel))
        load = [[0, 0] for _ in range(k)]
        assign = {}
        for g in sorted(groups, key=lambda g: -counts[g][1]):
            f = min(range(k), key=lambda i: (load[i][0], load[i][1]))
            assign[g] = f
            load[f][0] += counts[g][0]
            load[f][1] += counts[g][1]
    return np.array([assign[(r["src"], r["clip"], r["id"])] for r in rows])


def cross_val(rows, y: np.ndarray, k: int = 5,
              use_table: bool = True) -> tuple[np.ndarray, list[dict]]:
    folds = group_folds(rows, k)
    pred = np.zeros(len(rows))
    per_fold = []
    for f in range(k):
        te = np.where(folds == f)[0]
        tr = np.where(folds != f)[0]
        w = fit_logit(design([rows[i] for i in tr]), y[tr])
        wq = quantise(w)
        tab = table_from_int_weights(wq)
        for i in te:
            pred[i] = float(table_score([rows[i]], tab)[0]) if use_table \
                else 100.0 / (1.0 + np.exp(-float(
                    design([rows[i]])[0] @ w)))
        per_fold.append({"fold": f, "n": len(te), **metrics(y[te], pred[te])})
    return pred, per_fold


def leave_one_clip_out(rows, y: np.ndarray) -> tuple[np.ndarray, list[dict]]:
    clips = sorted({r["clip"] for r in rows})
    pred = np.zeros(len(rows))
    per_clip = []
    for clip in clips:
        te = [i for i, r in enumerate(rows) if r["clip"] == clip]
        tr = [i for i, r in enumerate(rows) if r["clip"] != clip]
        ytr = y[tr]
        if len(set(ytr)) < 2:
            w = prior_vector()
        else:
            w = fit_logit(design([rows[i] for i in tr]), ytr)
        tab = table_from_int_weights(quantise(w))
        for i in te:
            pred[i] = float(table_score([rows[i]], tab)[0])
        per_clip.append({"clip": clip, "n": len(te),
                         **metrics(y[te], pred[te])})
    return pred, per_clip


def scene_cv(rows, y: np.ndarray, k: int = 4) -> tuple[np.ndarray, list[dict]]:
    """Fixed-camera protocol: test on held-out tracks *of the same scene*.

    A camera bolted to the ceiling of one hall sees one scene all day, so the
    question the board has to answer is "is this box a person in *this*
    picture" -- and that model may legitimately have been calibrated with
    earlier boxes from the same camera.  Folds are still split by track, so
    no trajectory is ever tested on frames it was trained on, and every other
    scene is kept in the training set.
    """
    pred = np.full(len(rows), np.nan)
    per_clip = []
    for clip in sorted({r["clip"] for r in rows}):
        idx = np.array([i for i, r in enumerate(rows) if r["clip"] == clip])
        groups = sorted({(rows[i]["src"], rows[i]["id"]) for i in idx})
        assign = {g: i % k for i, g in enumerate(groups)}
        for f in range(k):
            te = [i for i in idx
                  if assign[(rows[i]["src"], rows[i]["id"])] == f]
            if not te:
                continue
            tr = [i for i in range(len(rows)) if i not in set(te)]
            ytr = y[tr]
            w = fit_logit(design([rows[i] for i in tr]), ytr)
            tab = table_from_int_weights(quantise(w))
            for i in te:
                pred[i] = float(table_score([rows[i]], tab)[0])
        sel = idx
        per_clip.append({"clip": clip, "n": len(sel), **metrics(y[sel],
                                                                pred[sel])})
    return pred, per_clip


RTL_TEMPLATE = """\
//===========================================================================
// person_score.v -- the per-box "this is a person" confidence, in percent.
//
// GENERATED by fpga/tools/person_score_fit.py -- do not edit by hand.
//
// The index packs the four 2-bit features the pipeline already carries
//
//     idx = {{evid, aspect, fill, size}}
//
//   evid   0 no history .. 3 established track   (track_assoc run_cnt/est)
//   aspect h vs w    0 very wide .. 3 tall
//   fill   blob area / box area   0 hollow .. 3 solid
//   size   blob area 0 tiny .. 3 large
//
// The fitted table is a small read-only memory: the score only has to be
// ready once per camera frame, so track_assoc walks the slots through it.
// Written as an asynchronous read case so the synthesizer infers distributed
// ROM instead of a 256 x 7 LUT tree, which is what the first PNR build ran
// out of MSlices on (5073/4900).
//===========================================================================
`timescale 1ns / 1ps

module person_score (
    input  wire [1:0] evid,
    input  wire [1:0] aspect,
    input  wire [1:0] fill,
    input  wire [1:0] size,
    output reg  [6:0] score
);
    wire [7:0] idx = {{evid, aspect, fill, size}};

    always @(*) begin
        case (idx)
{TABLE}
                default: score = 7'd50;
        endcase
    end
endmodule
"""


def verilog_table(tab: np.ndarray) -> str:
    lines = []
    for i in range(0, 256, 4):
        cells = " ".join(f"8'd{i + k}: score = 7'd{tab[i + k]};"
                         for k in range(4))
        lines.append("                " + cells)
    return "\n".join(lines)


def markdown(res: dict, rows, tab, per_fold, per_clip, folds) -> str:
    w = res["int_weights"]
    L = []
    L.append("# Person-confidence score\n")
    L.append(f"{res['n']} labelled boxes "
             f"({res['person']} person / {res['n'] - res['person']} not), "
             f"features evid/aspect/fill/size.\n")
    L.append("## Integer weights (x8, hardware-exact)\n")
    L.append("| term | evid1 | evid2 | evid3 | asp1 | asp2 | asp3 | "
             "fill1 | fill2 | fill3 | size1 | size2 | size3 | bias |")
    L.append("|---" * 14 + "|")
    L.append("| w | " + " | ".join(str(v) for v in w[1:]) + f" | {w[0]} |")
    L.append("\n## Accuracy\n")
    L.append("| protocol | acc | prec | rec | F1 | AUC | TP | FP | FN | TN |")
    L.append("|---|---|---|---|---|---|---|---|---|---|")
    for key, name in (("scene_cv", "**within scene, tracks held out**"),
                      ("scene_cv_ema", "  + EMA(1/4) smoothing"),
                      ("cv_table", "cross scene, grouped 5-fold"),
                      ("cv_table_ema", "  + EMA(1/4) smoothing"),
                      ("cv_logit", "  (unquantised logit)"),
                      ("in_sample", "in-sample"),
                      ("loco_table", "leave-one-clip-out (scene transfer)")):
        m = res[key]
        L.append(f"| {name} | {m['acc']:.3f} | {m['prec']:.3f} | "
                 f"{m['rec']:.3f} | {m['f1']:.3f} | {m['auc']:.3f} | "
                 f"{m['tp']} | {m['fp']} | {m['fn']} | {m['tn']} |")
    L.append("\n## Within scene, per clip (fixed camera)\n")
    L.append("| clip | boxes | acc | prec | rec | F1 | AUC |")
    L.append("|---|---|---|---|---|---|---|")
    for c in res["scene_per_clip"]:
        L.append(f"| {c['clip']} | {c['n']} | {c['acc']:.3f} | "
                 f"{c['prec']:.3f} | {c['rec']:.3f} | {c['f1']:.3f} | "
                 f"{c['auc']:.3f} |")
    L.append("\n## Leave one clip out, per clip (new camera)\n")
    L.append("| clip | boxes | person | acc | prec | rec | F1 |")
    L.append("|---|---|---|---|---|---|---|")
    for c in per_clip:
        L.append(f"| {c['clip']} | {c['n']} | | {c['acc']:.3f} | "
                 f"{c['prec']:.3f} | {c['rec']:.3f} | {c['f1']:.3f} |")
    L.append("\n## Per group-fold\n")
    L.append("| fold | boxes | acc | prec | rec | F1 |")
    L.append("|---|---|---|---|---|---|")
    for c in per_fold:
        L.append(f"| {c['fold']} | {c['n']} | {c['acc']:.3f} | "
                 f"{c['prec']:.3f} | {c['rec']:.3f} | {c['f1']:.3f} |")
    L.append("\n## Deployed form\n")
    L.append("The FPGA reads this table from a 256x7 asynchronous\n")
    L.append("distributed ROM (`rtl/person_score.v`), generated by the same\n")
    L.append("script that writes `model.json`.  A plain LUT tree for the same\n")
    L.append("values pushed PNR to 5073/4900 MSlices, so the table is written\n")
    L.append("as a case statement that maps to distributed memory instead.\n")
    L.append(f"\n## Table (idx = evid*64 + aspect*16 + fill*4 + size)\n")
    for evid in range(4):
        for aspect in range(4):
            row = [tab[combo_index(evid, aspect, fill, size)]
                   for fill in range(4) for size in range(4)]
            L.append(f"evid{evid} asp{aspect}: " +
                     " ".join(f"{v:2d}" for v in row))
    hog = res.get("hog_ref")
    if hog:
        L.append("\n## HOG + linear SVM reference (offline, not on the FPGA)\n")
        L.append(f"window {hog['window']}, cell {hog['cell']}, "
                 f"{hog['bins']} bins, dims {hog['dims']}, "
                 f"{hog['n']} crops\n")
        L.append("| protocol | acc | prec | rec | F1 | AUC |")
        L.append("|---|---|---|---|---|---|")
        for name, m in hog["metrics"].items():
            L.append(f"| {name} | {m['acc']:.3f} | {m['prec']:.3f} | "
                     f"{m['rec']:.3f} | {m['f1']:.3f} | {m['auc']:.3f} |")
    if res.get("notes"):
        L.append("\n## Notes\n")
        L.extend(res["notes"])
    return "\n".join(L) + "\n"


def main() -> int:
    global PRIOR_LAMBDA, FEATS
    ap = argparse.ArgumentParser()
    ap.add_argument("--lam", type=float, default=PRIOR_LAMBDA,
                    help="prior strength in pseudo-observations")
    ap.add_argument("--feats", default=",".join(FEATS),
                    help="four 2-bit features, index order evid..size")
    ap.add_argument("--sweep", action="store_true",
                    help="print the score spread for a few lambdas and stop")
    args = ap.parse_args()
    PRIOR_LAMBDA = args.lam
    FEATS = tuple(args.feats.split(","))
    if len(FEATS) != 4:
        print("need exactly four 2-bit features")
        return 2
    rows = load_rows()
    if not rows:
        print("no labelled rows")
        return 1
    y = np.array([r["y"] for r in rows])
    print(f"{len(rows)} labelled boxes, {int(y.sum())} person / "
          f"{len(y) - int(y.sum())} not-person")
    for clip in sorted({r["clip"] for r in rows}):
        sel = [r for r in rows if r["clip"] == clip]
        print(f"  {clip:32s} {len(sel):5d} boxes, "
              f"{sum(r['y'] for r in sel):5d} person")

    X = design(rows)

    if args.sweep:
        for lam in (25, 50, 100, 200, 400, 800, 1600):
            PRIOR_LAMBDA = lam
            w = fit_logit(X, y, lam=lam)
            wq = quantise(w)
            tab = table_from_int_weights(wq)
            s = table_score(rows, tab)
            pred, _ = cross_val(rows, y, k=5, use_table=True)
            m = metrics(y, pred)
            spred, _ = scene_cv(rows, y, k=4)
            sm = metrics(y, spred)
            print(f"lam {lam:5d}  z {w.min():+.1f}..{w.max():+.1f}  "
                  f"score p05={np.percentile(s, 5):.0f} "
                  f"p50={np.percentile(s, 50):.0f} "
                  f"p95={np.percentile(s, 95):.0f}  "
                  f"scene auc={sm['auc']:.3f} acc={sm['acc']:.3f} | "
                  f"xscene auc={m['auc']:.3f} acc={m['acc']:.3f}")
        return 0

    w = fit_logit(X, y)
    wq = quantise(w)
    tab = table_from_int_weights(wq)
    print("\nlogit (levels 1..3 vs 0, then bias):",
          " ".join(f"{v:+.2f}" for v in w[1:]), f"b={w[0]:+.2f}")
    print("integer weights x8:", " ".join(f"{v:+d}" for v in wq[1:]),
          f"b={wq[0]:+d}")
    print("score spread: " + " ".join(
        f"asp{a}:{min(tab[(a << 4):(a << 4) + 16])}-"
        f"{max(tab[(a << 4):(a << 4) + 16])}%" for a in range(4)))

    res = {"n": len(rows), "person": int(y.sum()), "scale": SCALE,
           "int_weights": [int(v) for v in
                           np.concatenate([[wq[0]], wq[1:]])],
           "logit_weights": [round(float(v), 4) for v in
                             np.concatenate([[w[0]], w[1:]])],
           "prior_lambda": PRIOR_LAMBDA}

    s_all = table_score(rows, tab)
    res["in_sample"] = metrics(y, s_all)

    pred, per_fold = cross_val(rows, y, use_table=True)
    res["cv_table"] = metrics(y, pred)
    res["cv_table_ema"] = metrics(y, smooth_tracks(rows, pred))
    res["per_fold"] = per_fold
    pprob, _ = cross_val(rows, y, use_table=False)
    res["cv_logit"] = metrics(y, pprob)

    spred, sper_clip = scene_cv(rows, y, k=4)
    res["scene_cv"] = metrics(y, spred)
    res["scene_cv_ema"] = metrics(y, smooth_tracks(rows, np.nan_to_num(spred)))
    res["scene_per_clip"] = sper_clip

    lpred, per_clip = leave_one_clip_out(rows, y)
    res["loco_table"] = metrics(y, lpred)
    res["per_clip"] = per_clip

    hog_path = ROOT / "out" / "person_score" / "hog_ref.json"
    if hog_path.exists():
        res["hog_ref"] = json.loads(hog_path.read_text(encoding="ascii"))

    for key, name in (("scene_cv", "within-scene 4-fold (fixed camera)"),
                      ("scene_cv_ema", "  + EMA(1/4)"),
                      ("cv_table", "cross-scene grouped 5-fold"),
                      ("cv_table_ema", "  + EMA(1/4)"),
                      ("cv_logit", "  unquantised logit"),
                      ("in_sample", "in-sample"),
                      ("loco_table", "leave-one-clip-out")):
        m = res[key]
        print(f"{name:34s} acc={m['acc']:.3f} prec={m['prec']:.3f} "
              f"rec={m['rec']:.3f} f1={m['f1']:.3f} auc={m['auc']:.3f} "
              f"(tp{m['tp']} fp{m['fp']} fn{m['fn']} tn{m['tn']})")
    for c in res["per_clip"]:
        print(f"   loco {c['clip']:30s} n={c['n']:5d} acc={c['acc']:.3f}")
    for c in res["scene_per_clip"]:
        print(f"   scene {c['clip']:29s} n={c['n']:5d} acc={c['acc']:.3f} "
              f"prec={c['prec']:.3f} rec={c['rec']:.3f} auc={c['auc']:.3f}")

    outdir = ROOT / "out" / "person_score"
    outdir.mkdir(parents=True, exist_ok=True)

    (outdir / "model.json").write_text(json.dumps(
        {"features": list(FEATS), "scale": SCALE,
         "int_weights": res["int_weights"],
         "table": [int(v) for v in tab],
         "deployed": {
             "form": "256x7 asynchronous distributed ROM (case table)",
             "exact_on_table": True},
         "metrics": res,
         "labels": [{"clip": c, "lo": lo, "hi": hi, "y": lab}
                    for c, lo, hi, lab in LABEL_RULES]},
        indent=1), encoding="ascii")

    (outdir / "report.md").write_text(
        markdown(res, rows, tab, per_fold, per_clip, None),
        encoding="ascii")

    (FPGA / "rtl" / "person_score.v").write_text(
        RTL_TEMPLATE.format(TABLE=verilog_table(tab)), encoding="ascii")
    print(f"\nwrote {outdir / 'model.json'}")
    print(f"wrote {outdir / 'report.md'}")
    print(f"wrote {FPGA / 'rtl' / 'person_score.v'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
