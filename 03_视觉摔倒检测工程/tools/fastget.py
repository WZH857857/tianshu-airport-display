"""Parallel, resumable HTTP downloader for the fall-detection dataset.

GitHub's public mirrors throttle a single connection to a few kilobytes per
second on this network.  Splitting a file into byte ranges and fetching them
with several threads recovers usable throughput.

Usage::

    python fastget.py --url URL --out PATH [--threads 8]
    python fastget.py --list-file urls.txt --root DIR [--threads 8]
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import requests

CHUNK = 1 << 20


def probe(url: str) -> tuple[int, bool]:
    resp = requests.head(url, timeout=30, allow_redirects=True)
    resp.raise_for_status()
    size = int(resp.headers.get("content-length", "0"))
    ranges = resp.headers.get("accept-ranges", "").lower() == "bytes"
    return size, ranges


def fetch_range(url: str, out: Path, start: int, end: int,
                progress: dict, lock: threading.Lock) -> None:
    headers = {"Range": f"bytes={start}-{end - 1}"}
    for attempt in range(4):
        try:
            with requests.get(url, stream=True, timeout=(15, 60),
                              headers=headers) as resp:
                if resp.status_code not in (200, 206):
                    raise RuntimeError(f"status {resp.status_code}")
                with out.open("r+b") as fh:
                    fh.seek(start)
                    for chunk in resp.iter_content(1 << 16):
                        if chunk:
                            fh.write(chunk)
                            with lock:
                                progress["got"] += len(chunk)
            return
        except Exception as exc:  # noqa: BLE001 - retry the range
            if attempt == 3:
                raise
            time.sleep(1.5 * (attempt + 1))


def download(url: str, dest: Path, threads: int) -> None:
    size, ranges = probe(url)
    if size == 0:
        raise RuntimeError(f"no content-length for {url}")
    dest.parent.mkdir(parents=True, exist_ok=True)
    part = dest.with_suffix(dest.suffix + ".part")
    if ranges:
        with part.open("wb") as fh:
            fh.truncate(size)
        step = max(CHUNK, size // threads)
        spans = [(i, min(i + step, size)) for i in range(0, size, step)]
        progress = {"got": 0}
        lock = threading.Lock()
        workers = []
        for start, end in spans:
            t = threading.Thread(target=fetch_range,
                                 args=(url, part, start, end, progress, lock),
                                 daemon=True)
            t.start()
            workers.append(t)
        while any(t.is_alive() for t in workers):
            time.sleep(1.0)
            with lock:
                got = progress["got"]
            print(f"  {dest.name}: {got / 1e6:6.2f} / {size / 1e6:6.2f} MB",
                  flush=True)
        for t in workers:
            t.join()
    else:
        with requests.get(url, stream=True, timeout=(15, 120)) as resp:
            resp.raise_for_status()
            with part.open("wb") as fh:
                for chunk in resp.iter_content(CHUNK):
                    if chunk:
                        fh.write(chunk)
    if dest.exists():
        dest.unlink()
    part.replace(dest)
    print(f"ok {dest}  {dest.stat().st_size / 1e6:.2f} MB", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url")
    parser.add_argument("--out")
    parser.add_argument("--threads", type=int, default=8)
    args = parser.parse_args()
    if not args.url or not args.out:
        parser.error("--url and --out are required")
    started = time.time()
    download(args.url, Path(args.out), max(1, args.threads))
    print(f"elapsed {time.time() - started:.1f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
