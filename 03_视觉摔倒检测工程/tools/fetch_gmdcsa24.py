"""Download the GMDCSA24 fall-detection dataset (MIT licensed).

The upstream repository keeps every clip as a normal git blob, so individual
files can be streamed over HTTP.  Several GitHub mirrors are tried in turn and
partial downloads resume with a Range request.

Usage examples::

    python fetch_gmdcsa24.py --list
    python fetch_gmdcsa24.py --subject 1 --split Fall
    python fetch_gmdcsa24.py --max-mb 300
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import requests

REPO = "ekramalam/GMDCSA24-A-Dataset-for-Human-Fall-Detection-in-Videos"
BRANCH = "master"
TREE_API = f"https://api.github.com/repos/{REPO}/git/trees/{BRANCH}?recursive=1"
ROOT = Path(__file__).resolve().parents[1] / "datasets" / "gmdcsa24"


def mirrors(path: str) -> list[str]:
    quoted = requests.utils.quote(path)
    return [
        f"https://raw.githubusercontent.com/{REPO}/{BRANCH}/{quoted}",
        f"https://cdn.jsdelivr.net/gh/{REPO}@{BRANCH}/{quoted}",
        f"https://fastly.jsdelivr.net/gh/{REPO}@{BRANCH}/{quoted}",
        f"https://gcore.jsdelivr.net/gh/{REPO}@{BRANCH}/{quoted}",
    ]


def fetch_tree() -> list[dict]:
    resp = requests.get(TREE_API, timeout=60)
    resp.raise_for_status()
    payload = resp.json()
    if payload.get("truncated"):
        print("warning: git tree was truncated by the API", file=sys.stderr)
    return [item for item in payload["tree"] if item["type"] == "blob"]


def matches(path: str, subjects: set[str], splits: set[str]) -> bool:
    parts = path.split("/")
    if len(parts) < 3 or not parts[0].startswith("Subject"):
        return False
    if subjects and parts[0].split()[-1] not in subjects:
        return False
    if splits and parts[1] not in splits:
        return False
    return True


def download(path: str, dest: Path, retries: int = 3) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    part = dest.with_suffix(dest.suffix + ".part")
    for attempt in range(1, retries + 1):
        start = part.stat().st_size if part.exists() else 0
        for url in mirrors(path):
            headers = {"Range": f"bytes={start}-"} if start else {}
            try:
                with requests.get(url, stream=True, timeout=90,
                                  headers=headers) as resp:
                    if resp.status_code not in (200, 206):
                        continue
                    mode = "ab" if start and resp.status_code == 206 else "wb"
                    if mode == "wb":
                        start = 0
                    with part.open(mode) as fh:
                        for chunk in resp.iter_content(chunk_size=1 << 20):
                            if chunk:
                                fh.write(chunk)
                if dest.exists():
                    dest.unlink()
                part.replace(dest)
                return
            except requests.RequestException as exc:
                print(f"  retry {path}: {type(exc).__name__}", file=sys.stderr)
                continue
        time.sleep(1.0 * attempt)
    raise RuntimeError(f"all mirrors failed for {path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--subject", action="append", default=[],
                        help="subject number, repeatable (default: all)")
    parser.add_argument("--split", action="append", default=[],
                        choices=["ADL", "Fall"],
                        help="ADL or Fall, repeatable (default: both)")
    parser.add_argument("--list", action="store_true",
                        help="only print the matching files")
    parser.add_argument("--max-mb", type=float, default=0,
                        help="stop after downloading this many megabytes")
    args = parser.parse_args()

    subjects = set(args.subject)
    splits = set(args.split)
    items = [it for it in fetch_tree() if matches(it["path"], subjects, splits)]
    items.sort(key=lambda it: it["path"])

    total = sum(it.get("size", 0) for it in items)
    print(f"matched {len(items)} files, {total / 1e6:.1f} MB")
    if args.list:
        for it in items:
            print(f"{it.get('size', 0):>10}  {it['path']}")
        return 0

    limit = args.max_mb * 1e6 if args.max_mb else 0
    done = 0.0
    started = time.time()
    for it in items:
        if limit and done >= limit:
            print(f"reached --max-mb {args.max_mb}, stopping")
            break
        dest = ROOT / it["path"]
        if dest.exists() and dest.stat().st_size == it.get("size"):
            print(f"skip   {it['path']}")
            done += it.get("size", 0)
            continue
        try:
            download(it["path"], dest)
        except Exception as exc:  # noqa: BLE001 - report and keep going
            print(f"fail   {it['path']}: {exc}", file=sys.stderr)
            continue
        size = dest.stat().st_size
        print(f"ok     {it['path']}  {size / 1e6:.1f} MB")
        done += size
    print(f"done in {time.time() - started:.0f}s, "
          f"total {done / 1e6:.1f} MB under {ROOT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
