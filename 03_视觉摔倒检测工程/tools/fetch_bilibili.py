#!/usr/bin/env python3
"""Grab a Bilibili clip for the crowd tests.

The page API is enough for the qualities that do not need a login: ask for
the DASH manifest, take the best video-only stream at or below the requested
height, and remux it to a plain mp4 with the ffmpeg that imageio bundles.

Usage::

    python fetch_bilibili.py BV1iA41147hB --out datasets/crowd/crowd07.mp4
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import requests

UA = {
    "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) "
                  "AppleWebKit/537.36 (KHTML, like Gecko) "
                  "Chrome/124.0 Safari/537.36",
    "Referer": "https://www.bilibili.com/",
}
API = "https://api.bilibili.com/x/web-interface/view"
PLAYURL = "https://api.bilibili.com/x/player/playurl"


def view(bvid: str) -> dict:
    r = requests.get(API, params={"bvid": bvid}, headers=UA, timeout=30)
    r.raise_for_status()
    data = r.json()
    if data.get("code") != 0:
        raise SystemExit(f"error: view API said {data.get('message')}")
    return data["data"]


def playurl(bvid: str, cid: int) -> dict:
    r = requests.get(PLAYURL, params={
        "bvid": bvid, "cid": cid, "qn": 80, "fnval": 16, "fourk": 0,
    }, headers=UA, timeout=30)
    r.raise_for_status()
    data = r.json()
    if data.get("code") != 0:
        raise SystemExit(f"error: playurl said {data.get('message')}")
    return data["data"]


def download(url: str, out: Path) -> None:
    with requests.get(url, headers=UA, stream=True, timeout=60) as resp:
        resp.raise_for_status()
        total = int(resp.headers.get("content-length", "0"))
        done = 0
        with open(out, "wb") as fh:
            for chunk in resp.iter_content(1 << 20):
                fh.write(chunk)
                done += len(chunk)
                if total:
                    print(f"\r  {done / 1e6:7.2f} / {total / 1e6:.2f} MB",
                          end="", flush=True)
    print()


def remux(src: Path, dst: Path) -> bool:
    try:
        import imageio_ffmpeg
        exe = imageio_ffmpeg.get_ffmpeg_exe()
    except Exception as exc:                       # pragma: no cover
        print(f"  no bundled ffmpeg ({exc}); keeping the raw stream")
        return False
    cmd = [exe, "-y", "-loglevel", "error", "-i", str(src),
           "-c", "copy", "-movflags", "+faststart", str(dst)]
    if subprocess.run(cmd).returncode == 0:
        return True
    # Some DASH streams refuse a straight copy; re-encode the video only.
    cmd = [exe, "-y", "-loglevel", "error", "-i", str(src),
           "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
           str(dst)]
    return subprocess.run(cmd).returncode == 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bvid")
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-height", type=int, default=720)
    args = ap.parse_args()

    info = view(args.bvid)
    cid = info["cid"]
    print(f"{info['title']}  ({info['duration']}s, cid {cid})")
    play = playurl(args.bvid, cid)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".stream.m4s")

    dash = play.get("dash") or {}
    streams = dash.get("video") or []
    if streams:
        pick = min(streams,
                   key=lambda s: (s.get("height", 0) > args.max_height,
                                  -s.get("height", 0), s.get("bandwidth", 0)))
        print(f"  dash {pick.get('width')}x{pick.get('height')} "
              f"{pick.get('codecs')} {pick.get('bandwidth', 0) // 1000} kbps")
        download(pick["baseUrl"], tmp)
    else:
        best = max(play["durl"], key=lambda d: d.get("size", 0))
        print(f"  durl {best.get('size', 0) / 1e6:.2f} MB")
        download(best["url"], tmp)

    if not remux(tmp, out):
        print("error: remux failed")
        return 1
    tmp.unlink(missing_ok=True)
    print(f"  -> {out}  ({out.stat().st_size / 1e6:.2f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
