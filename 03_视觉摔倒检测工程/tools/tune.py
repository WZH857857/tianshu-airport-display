"""Grid-search the posture thresholds against the recorded geometry traces.

The traces produced by ``dump_traces.py`` only depend on the front-end
(frame difference + morphology + projection box), so every candidate
threshold set can be scored without touching video again.  Replay uses the
same ``PostureMachine`` the pixel pipeline uses, so tuned values transfer.

Usage::

    python tune.py               # coarse sweep + per-clip report
    python tune.py --explain Subject1_Fall_13
"""

from __future__ import annotations

import argparse
import csv
import itertools
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import imageio.v3 as iio

sys.path.insert(0, str(Path(__file__).resolve().parent))
from batch_eval import read_annotations  # noqa: E402
from fall_proto import Obs, Params, PostureMachine, same_track  # noqa: E402


@dataclass
class Clip:
    tag: str
    subject: str
    kind: str
    name: str
    fps: float
    rows: list[Obs | None]
    gt: tuple[float, float] | None
    video: Path | None = None


def load_trace(path: Path) -> list[Obs | None]:
    rows: list[Obs | None] = []
    with path.open(newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            if not row["x"]:
                rows.append(None)
                continue
            x, y, w, h = (int(row[k]) for k in ("x", "y", "w", "h"))
            rows.append(Obs(
                frame=int(row["frame"]),
                bbox=(x, y, w, h),
                area=int(row["area"]),
                ratio=float(row["ratio"]),
                cx=x + (w - 1) / 2.0,
                cy=y + (h - 1) / 2.0,
            ))
    return rows


def load_clips(root: Path, trace_dir: Path, video_root: Path,
               airport: Path | None) -> list[Clip]:
    clips: list[Clip] = []
    for subject in sorted(root.glob("Subject *")):
        for kind in ("Fall", "ADL"):
            csv_path = subject / f"{kind}.csv"
            if not csv_path.exists():
                continue
            table = read_annotations(csv_path)
            for trace in sorted(trace_dir.glob(f"{subject.name.replace(' ', '')}_{kind}_*.csv")):
                name = f"{trace.stem.rsplit('_', 1)[1]}.mp4"
                video = video_root / subject.name / kind / name
                fps = 25.0
                if video.exists():
                    fps = float(iio.immeta(video, plugin="FFMPEG").get("fps") or 25.0)
                clips.append(Clip(trace.stem, subject.name, kind, name, fps,
                                  load_trace(trace), table.get(name), video))
    if airport is not None:
        for trace in sorted(trace_dir.glob("airport_*.csv")):
            name = trace.stem[len("airport_"):]
            video = (airport / f"{name}.mp4") if airport else None
            fps = 25.0
            if video is not None and video.exists():
                fps = float(iio.immeta(video, plugin="FFMPEG").get("fps") or 25.0)
            clips.append(Clip(trace.stem, "airport", "Neg", f"{name}.mp4",
                              fps, load_trace(trace), None, video))
    return clips


def replay(rows: list[Obs | None], params: Params) -> list[tuple[int, str]]:
    """Replay a trace through the posture machine, mirroring FallDetector."""
    machine = PostureMachine(params)
    alarms: list[tuple[int, str]] = []
    prev: tuple[int, int, int, int] | None = None
    miss = 0
    for obs in rows:
        if obs is None:
            miss += 1
            if miss > params.history_frames:
                prev = None
                machine.reset()
            else:
                machine.feed(None)
            continue
        miss = 0
        if not same_track(prev, obs.bbox):
            machine.reset()
        prev = obs.bbox
        flagged, _state, detail = machine.feed(obs)
        if flagged:
            alarms.append((obs.frame, detail))
    return alarms


def score(clips: list[Clip], params: Params) -> tuple[float, list[dict]]:
    details = []
    hits = 0
    falls = 0
    false_clips = 0
    for clip in clips:
        alarms = replay(clip.rows, params)
        times = [round(f / clip.fps, 2) for f, _ in alarms]
        if clip.gt is not None:
            falls += 1
            hit = any(clip.gt[0] - 0.5 <= t <= clip.gt[1] for t in times)
            hits += int(hit)
        else:
            hit = None
            false_clips += int(bool(alarms))
        details.append({"tag": clip.tag, "gt": clip.gt, "times": times, "hit": hit})
    total = hits - 3 * false_clips
    return total, [{"hits": hits, "falls": falls, "false_clips": false_clips}] + details


def grid() -> list[Params]:
    base = Params()
    space = {
        "lying_ratio": [0.7, 0.8, 0.85, 0.9, 0.95],
        "collapse_frac": [0.35, 0.45, 0.55, 0.65],
        "tall_frac": [0.5, 0.6, 0.7],
        "collapse_frames": [18, 26, 34],
        "confirm_frames": [1, 2],
        "widen_frac": [0.0],
        "drop_frac": [0.0, 0.04],
        "min_ref_area": [800, 1200, 1600],
        "min_ref_height": [25, 30],
    }
    keys = list(space)
    out = []
    for values in itertools.product(*(space[k] for k in keys)):
        out.append(replace(base, **dict(zip(keys, values))))
    return out


def explain(clip: Clip, params: Params) -> None:
    """Print the frames that came close, with each condition's verdict."""
    machine = PostureMachine(params)
    prev = None
    print(f"{clip.tag}: gt={clip.gt} fps={clip.fps:.2f} rows={len(clip.rows)}")
    print(f"{'frame':>5} {'t':>6} {'w':>4} {'h':>4} {'ratio':>6} "
          f"{'h_ref':>6} {'shrunk':>6} {'flat':>5} {'wide':>5} "
          f"{'sunk':>5} {'1v':>4} {'1h':>4}")
    for obs in clip.rows:
        if obs is None:
            machine.feed(None)
            continue
        if not same_track(prev, obs.bbox):
            machine.reset()
        prev = obs.bbox
        ref = machine.h_ref
        machine.feed(obs)
        if ref is None or obs.frame - ref.frame > params.ref_ttl:
            continue
        shrunk = obs.h <= params.collapse_frac * ref.h
        flat = obs.ratio <= params.lying_ratio
        wider = obs.w >= params.widen_frac * ref.w
        sunk = (obs.cy - ref.cy) >= params.drop_frac * 240
        speed = obs.frame - machine.last_tall if machine.last_tall > 0 else 999
        votes = sum([shrunk, flat, wider, sunk])
        if votes >= 3 and obs.ratio < 1.0:
            print(f"{obs.frame:5d} {obs.frame / clip.fps:6.2f} {obs.w:4d} {obs.h:4d} "
                  f"{obs.ratio:6.2f} {ref.h:6d} {int(shrunk):6d} {int(flat):5d} "
                  f"{int(wider):5d} {int(sunk):5d} {speed:4d} "
                  f"{int(speed <= params.collapse_frames):4d}")


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=root / "datasets" / "gmdcsa24")
    parser.add_argument("--traces", type=Path, default=root / "out_frames" / "traces")
    parser.add_argument("--airport", type=Path,
                        default=Path(r"E:\FPGA guo sai\EG4X20B\E104_Console\assets"))
    parser.add_argument("--explain", default=None)
    parser.add_argument("--top", type=int, default=10)
    args = parser.parse_args()

    clips = load_clips(args.root, args.traces, args.root, args.airport)
    print(f"{len(clips)} traces loaded\n")

    if args.explain:
        for clip in clips:
            if clip.tag == args.explain:
                explain(clip, Params())
                return 0
        print(f"no trace named {args.explain}")
        return 1

    ranked = []
    for params in grid():
        total, details = score(clips, params)
        ranked.append((total, details[0]["hits"], params, details))
    ranked.sort(key=lambda item: (item[0], item[1]), reverse=True)

    print(f"{'score':>5} {'hits':>5} {'false':>5}  thresholds")
    seen = set()
    shown = 0
    for total, hits, params, _details in ranked:
        key = (params.lying_ratio, params.collapse_frac, params.tall_frac,
               params.collapse_frames, params.widen_frac, params.drop_frac,
               params.min_ref_area, params.min_ref_height,
               params.confirm_frames)
        if key in seen:
            continue
        seen.add(key)
        print(f"{total:5.0f} {hits:5d} "
              f"{_details[0]['false_clips']:5d}  "
              f"lying<={params.lying_ratio} shrink<={params.collapse_frac} "
              f"tall>={params.tall_frac} speed<={params.collapse_frames} "
              f"widen>={params.widen_frac} drop>={params.drop_frac} "
              f"ref_area>={params.min_ref_area} ref_h>={params.min_ref_height}")
        shown += 1
        if shown >= args.top:
            break

    best = ranked[0]
    print(f"\nbest: score={best[0]:.0f} hits={best[1]}/{best[3][0]['falls']} "
          f"false={best[3][0]['false_clips']}")
    for row in best[3][1:]:
        if row["hit"] is None:
            mark = "FP  " if row["times"] else "OK  "
        else:
            mark = "HIT " if row["hit"] else "MISS"
        print(f"  {mark} {row['tag']:<28} gt={row['gt']} alarm={row['times']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
