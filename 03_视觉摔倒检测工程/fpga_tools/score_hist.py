#!/usr/bin/env python3
"""Per-frame confidence analysis for one clip, straight from the RTL table.

``out/person_roi/features.csv`` holds one row per published box per frame:
the four 2-bit features the FPGA carries.  Scoring every row through
``person_roi.score`` -- which reads the same 256-entry table as
``rtl/person_score.v`` -- answers the two questions the display threshold
depends on:

* what score does a person actually get on this material (distribution), and
* if the overlay only trusted boxes above score t, how many frames would
  still show somebody, and how many boxes would be dropped.

Outputs ``out/person_score/<clip>_scores.md`` and a two-panel figure
``out/person_score/<clip>_scores.png`` (score histogram, per-frame best score).

Usage::

    python score_hist.py --clip crowd07_hogsvm.mp4
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(HERE))

import person_roi as PR  # noqa: E402


def load_rows(path: Path, clip: str) -> list[dict]:
    out = []
    with open(path, newline="", encoding="utf-8") as fh:
        for rec in csv.DictReader(fh):
            if rec["clip"] != clip:
                continue
            out.append({k: (v if k == "clip" else int(v))
                        for k, v in rec.items()})
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--clip", default="crowd07_hogsvm.mp4")
    ap.add_argument("--csv", type=Path,
                    default=ROOT / "out" / "person_roi" / "features.csv")
    ap.add_argument("--outdir", type=Path,
                    default=ROOT / "out" / "person_score")
    args = ap.parse_args()

    rows = load_rows(args.csv, args.clip)
    if not rows:
        raise SystemExit(f"no rows for {args.clip} in {args.csv}")

    scores = np.array([PR.score(r["evid"], r["aspect"], r["fill"], r["size"])
                       for r in rows])
    frames = np.array([r["frame"] for r in rows])
    nframes = int(frames.max()) + 1

    best = np.full(nframes, -1.0)
    for f, s in zip(frames, scores):
        if s > best[f]:
            best[f] = s
    shown = best[best >= 0]

    lines = [f"# {args.clip} per-frame confidence",
             "",
             f"{len(rows)} published boxes over {nframes} frames "
             f"({len(shown)} frames carry at least one box).",
             "",
             "| statistic | boxes | frames (best box) |",
             "|---|---|---|",
             f"| median | {int(np.median(scores))} | {int(np.median(shown))} |",
             f"| mean | {scores.mean():.1f} | {shown.mean():.1f} |",
             f"| 10th pct | {int(np.percentile(scores, 10))} | "
             f"{int(np.percentile(shown, 10))} |",
             f"| 90th pct | {int(np.percentile(scores, 90))} | "
             f"{int(np.percentile(shown, 90))} |",
             f"| max | {int(scores.max())} | {int(shown.max())} |",
             "",
             "## Threshold sweep (what the overlay would show)",
             "",
             "| score >= | frames with someone | boxes kept | boxes dropped |",
             "|---|---|---|---|"]
    for t in (40, 50, 60, 70, 80, 85, 90, 95):
        frames_kept = int((shown >= t).sum())
        boxes_kept = int((scores >= t).sum())
        lines.append(f"| {t} | {frames_kept} / {nframes} "
                     f"({100.0 * frames_kept / nframes:.1f}%) | "
                     f"{boxes_kept} / {len(scores)} "
                     f"({100.0 * boxes_kept / len(scores):.1f}%) | "
                     f"{len(scores) - boxes_kept} |")

    hist, edges = np.histogram(scores, bins=np.arange(0, 105, 5))
    lines += ["", "## Histogram (5 % bins)", "",
              "| score | boxes |", "|---|---|"]
    for n, lo in zip(hist, edges[:-1]):
        if n:
            lines.append(f"| {int(lo)}-{int(lo) + 4} | {int(n)} |")

    args.outdir.mkdir(parents=True, exist_ok=True)
    stem = Path(args.clip).stem
    md = args.outdir / f"{stem}_scores.md"
    md.write_text("\n".join(lines) + "\n", encoding="utf-8")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6.4), dpi=110,
                                   gridspec_kw={"height_ratios": [1, 1.2]})
    ax1.bar(edges[:-1] + 2.5, hist, width=4.4, color="#38bdf8",
            edgecolor="#0b1020")
    ax1.set_xlim(0, 100)
    ax1.set_xlabel("person confidence printed by the overlay (%)")
    ax1.set_ylabel("boxes")
    ax1.set_title(f"{args.clip}: {len(scores)} published boxes")
    ax1.grid(alpha=0.25)

    ax2.plot(np.arange(nframes), np.where(best < 0, np.nan, best),
             lw=0.9, color="#34d399", label="best box in frame")
    if nframes > 20:
        k = 21
        pad = np.pad(np.where(best < 0, np.nan, best), k // 2,
                     constant_values=np.nan)
        roll = np.array([np.nanmedian(pad[i:i + k]) for i in range(nframes)])
        ax2.plot(np.arange(nframes), roll, lw=1.6, color="#fbbf24",
                 label="rolling median (21 frames)")
    ax2.axhline(60, color="#f87171", lw=1.2, ls="--",
                label="display threshold 60 %")
    ax2.set_ylim(0, 102)
    ax2.set_xlabel("frame")
    ax2.set_ylabel("confidence (%)")
    ax2.grid(alpha=0.25)
    ax2.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    png = args.outdir / f"{stem}_scores.png"
    fig.savefig(png)

    print(f"{args.clip}: {len(scores)} boxes, {nframes} frames, "
          f"best-box median {int(np.median(shown))} %, "
          f"score>=60 in {int((shown >= 60).sum())} frames")
    print(f"wrote {md}")
    print(f"wrote {png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
