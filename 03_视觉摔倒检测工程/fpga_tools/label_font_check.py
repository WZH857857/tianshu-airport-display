#!/usr/bin/env python3
"""Check the RTL label font against tools/label_font.py.

Runs fpga/tb/tb_label_dump.v under Icarus, parses the character dump of each
tag area (24x5 for `87%`, 48x18 for `摔倒`) and compares it, pixel for pixel,
with the Python model of the same font.  Anything that only exists in the RTL
(the black backing rectangle, the picture pass-through, the slot colour) shows
up as a mismatch, so this is the gate between "the model looks nice" and "the
board draws that".

    python label_font_check.py
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(HERE))

import label_font as L  # noqa: E402

OUT = ROOT / "out_sim"
IVL = Path(r"E:\iverilog\bin")

def tool(name: str) -> str:
    exe = IVL / f"{name}.exe"
    if exe.exists():
        return str(exe)
    found = shutil.which(name)
    if not found:
        raise SystemExit(f"{name} not found (looked in {IVL} and PATH)")
    return found


def run_bench() -> dict[str, np.ndarray]:
    OUT.mkdir(parents=True, exist_ok=True)
    vvp = OUT / "tb_label_dump.vvp"
    cmd = [tool("iverilog"), "-g2012", "-o", str(vvp),
           str(FPGA / "tb" / "tb_label_dump.v"),
           str(FPGA / "rtl" / "hdmi" / "box_overlay.v")]
    subprocess.run(cmd, check=True)
    text = subprocess.run([tool("vvp"), str(vvp)], check=True,
                          capture_output=True, text=True).stdout

    dumps: dict[str, list[str]] = {}
    tag = None
    for line in text.splitlines():
        if line.startswith("TAG "):
            tag = line[4:].strip()
            dumps[tag] = []
        elif line.startswith("ROW ") and tag is not None:
            body = line.split(" ", 2)[2]
            dumps[tag].append(body)
    # Rows are padded to the widest one in the dump.
    out = {}
    for k, v in dumps.items():
        width = max(len(r) for r in v)
        out[k] = np.array([list(r.ljust(width, "-")) for r in v], dtype="<U1")
    return out


def expected(mask: np.ndarray, colour: str) -> np.ndarray:
    """The characters the RTL should print for one tag from its glyph mask."""
    rows, cols = mask.shape
    out = np.full((rows, cols), ".", dtype="<U1")
    for r in range(rows):
        for c in range(cols):
            if mask[r, c]:
                out[r, c] = colour
    return out


def main() -> int:
    dumps = run_bench()
    cases = {
        "person87": expected(L.person_tag(87), "#"),
        "person42": expected(L.person_tag(42), "#"),
        "person09": expected(L.person_tag(9), "#"),
        "alarm":    expected(L.alarm_tag(), "#"),
    }
    bad = 0
    for name, want in cases.items():
        got = dumps.get(name)
        if got is None:
            print(f"FAIL {name}: no dump")
            bad += 1
            continue
        if got.shape != want.shape or (got != want).any():
            diff = np.argwhere(got != want)
            print(f"FAIL {name}: {len(diff)} pixels differ "
                  f"(row, col) {[tuple(int(v) for v in p) for p in diff[:6]]}")
            print("RTL:")
            print("\n".join("".join(r) for r in got))
            print("model:")
            print("\n".join("".join(r) for r in want))
            bad += 1
        else:
            print(f"ok   {name}: {int((got == '#').sum())} glyph pixels match")
    if bad:
        print(f"FAIL label_font_check: {bad} of {len(cases)} tags differ")
        return 1
    print("PASS label_font_check: RTL font == label_font.py for "
          f"{len(cases)} tags")
    return 0


if __name__ == "__main__":
    os.environ.setdefault("PYTHONIOENCODING", "utf-8")
    raise SystemExit(main())
