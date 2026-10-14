"""Decode every clip once and store its per-frame blob geometry as CSV.

The trace only depends on the front-end (motion threshold + morphology +
projection box), so the posture state machine can be re-tuned offline from
these CSVs without decoding video again.

Usage::

    python dump_traces.py [--root DATASETS] [--airport DIR] [--out TRACE_DIR]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import imageio.v3 as iio

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fall_proto import FallDetector, Params  # noqa: E402


def dump_clip(clip: Path, out_path: Path, params: Params) -> int:
    detector = FallDetector(params)
    for frame in iio.imiter(clip, plugin="FFMPEG"):
        detector.update(frame)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    header = "frame,track,area,x,y,w,h,ratio,state,flagged\n"
    lines = []
    for row in detector.trace:
        bbox = row["bbox"]
        if bbox is None:
            lines.append(f"{row['frame']},{row.get('track', 0)},"
                         f"{row['area']},,,,,,\n")
        else:
            x, y, w, h = bbox
            lines.append(f"{row['frame']},{row.get('track', 0)},{row['area']},"
                         f"{x},{y},{w},{h},{row['ratio']:.4f},"
                         f"{row['state']},{int(row['flagged'])}\n")
    out_path.write_text(header + "".join(lines), encoding="utf-8")
    return len(detector.trace)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=root / "datasets" / "gmdcsa24")
    parser.add_argument("--airport", type=Path,
                        default=Path(r"E:\FPGA guo sai\EG4X20B\E104_Console\assets"))
    parser.add_argument("--out", type=Path, default=root / "out_frames" / "traces")
    args = parser.parse_args()

    params = Params()
    clips: list[tuple[str, Path]] = []
    for subject in sorted(args.root.glob("Subject *")):
        for kind in ("Fall", "ADL"):
            clip_dir = subject / kind
            if not clip_dir.is_dir():
                continue
            for clip in sorted(clip_dir.glob("*.mp4")):
                tag = f"{subject.name.replace(' ', '')}_{kind}_{clip.stem}"
                clips.append((tag, clip))
    if args.airport.is_dir():
        for clip in sorted(args.airport.glob("*.mp4")):
            clips.append((f"airport_{clip.stem}", clip))

    for tag, clip in clips:
        out_path = args.out / f"{tag}.csv"
        frames = dump_clip(clip, out_path, params)
        print(f"{tag:<28} {frames:4d} frames -> {out_path.name}")
    print(f"\n{len(clips)} traces written to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
