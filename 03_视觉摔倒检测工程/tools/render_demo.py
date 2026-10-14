"""Render annotated demo videos and contact sheets for a few selected clips.

Each output MP4 overlays the motion mask (green tint), the tracked bbox
(green while seeking, red once the fall state is latched) and a status banner,
so the whole pipeline can be judged by eye instead of by numbers.

Usage::

    python render_demo.py                 # default selection
    python render_demo.py --clip PATH --out DIR
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import imageio
import imageio.v3 as iio
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fall_proto import FallDetector, Params, annotate  # noqa: E402


def banner(frame: np.ndarray, text: str, color: tuple[int, int, int]) -> np.ndarray:
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    w = 34 + 9 * len(text)
    draw.rectangle([8, 8, 8 + w, 40], fill=(0, 0, 0))
    draw.text((16, 18), text, fill=color)
    return np.asarray(img)


def render(clip: Path, out_dir: Path, params: Params) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    fps = float(iio.immeta(clip, plugin="FFMPEG").get("fps") or 25.0)
    detector = FallDetector(params)
    # Subject folders share clip names (13.mp4 exists in every one), so keep
    # the parent folder in the output name.
    tag = f"{clip.parent.parent.name}_{clip.parent.name}_{clip.stem}".replace(" ", "")
    video_path = out_dir / f"{tag}_annotated.mp4"
    writer = imageio.get_writer(video_path, fps=fps, codec="libx264",
                                quality=7, macro_block_size=1)
    alarms: list[int] = []
    frames: list[np.ndarray] = []
    for index, frame in enumerate(iio.imiter(clip, plugin="FFMPEG")):
        dets = detector.update(frame)
        det = next((d for d in dets if d.flagged), None)
        if det is None and dets:
            det = max(dets, key=lambda d: d.area)
        if det is not None and det.flagged:
            alarms.append(index)
        marked = annotate(frame, detector.mask_cache, dets, detector.scale)
        state = det.state if det is not None else "SEEK"
        flagged = bool(det is not None and det.flagged)
        colour = (255, 80, 80) if flagged else (120, 255, 120)
        label = (f"FALL DETECTED f{index}" if flagged else
                 f"{state}  f{index}  {index / fps:5.2f}s")
        marked = banner(marked, label, colour)
        writer.append_data(marked)
        frames.append(marked)
    writer.close()

    sheets = []
    if alarms:
        first = alarms[0]
        picks = [f for f in range(first - 20, first + 16, 4) if 0 <= f < len(frames)]
        sheets.append(make_sheet(frames, picks, out_dir / f"{tag}_alarm_sheet.png"))
    return {
        "clip": tag,
        "video": str(video_path),
        "alarms": alarms,
        "alarm_times": [round(a / fps, 2) for a in alarms],
        "sheets": [str(s) for s in sheets if s],
    }


def make_sheet(frames: list[np.ndarray], picks: list[int],
               out_path: Path, cols: int = 3) -> Path | None:
    if not picks:
        return None
    tiles = [Image.fromarray(frames[i]).resize((426, 320)) for i in picks]
    rows = (len(tiles) + cols - 1) // cols
    canvas = Image.new("RGB", (426 * cols, 320 * rows), (15, 15, 15))
    for n, tile in enumerate(tiles):
        r, c = divmod(n, cols)
        canvas.paste(tile, (c * 426, r * 320))
    canvas.save(out_path)
    return out_path


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clip", type=Path, action="append", default=None)
    parser.add_argument("--out", type=Path, default=root / "out_frames" / "demo")
    args = parser.parse_args()

    if args.clip:
        clips = args.clip
    else:
        gmdcsa = root / "datasets" / "gmdcsa24"
        clips = [
            gmdcsa / "Subject 1" / "Fall" / "13.mp4",
            gmdcsa / "Subject 2" / "Fall" / "13.mp4",
            gmdcsa / "Subject 3" / "Fall" / "20.mp4",
        ]
        adl = sorted((gmdcsa / "Subject 3" / "ADL").glob("*.mp4"))
        if adl:
            clips.append(adl[0])
        airport = Path(r"E:\FPGA guo sai\EG4X20B\E104_Console\assets"
                       r"\cand_departure_360.mp4")
        if airport.exists():
            clips.append(airport)

    params = Params()
    for clip in clips:
        if not clip.exists():
            print(f"skip (missing): {clip}")
            continue
        info = render(Path(clip), args.out, params)
        alarm = info["alarm_times"]
        print(f"{info['clip']:<22} alarms={alarm if alarm else 'none'} -> {info['video']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
