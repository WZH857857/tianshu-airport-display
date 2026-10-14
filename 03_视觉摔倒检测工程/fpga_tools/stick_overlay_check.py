"""Pixel-exact check of rtl/hdmi/stick_overlay.v against the bar spec.

The overlay is combinatorial over one 640x480 raster.  This script builds a
handful of box sets (upright, lying both ways, tiny, huge, overlapped with an
alarm, empty), replays each through the real RTL in Icarus, renders the same
frame from the documented bar spec in numpy, and compares every pixel.

It also renders one case with the PIL prototype (scratch/_stick_render.py
bars, which the clips were signed off on) and reports how many pixels differ,
so a systematic geometry mistake cannot hide behind a matching model.

Usage::

    python fpga/tools/stick_overlay_check.py
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
ROOT = FPGA.parent
sys.path.insert(0, str(HERE))

from morph_check import GEN, find_tool  # noqa: E402

NB = 4
W, H = 640, 480
EMPTY = (0, 0, 0, 0, 0, 0, 0)
COLOURS = [(0, 255, 96), (0, 200, 255), (255, 196, 0), (255, 64, 255)]
ALARM_RGB = (255, 0, 0)

CASES = [
    # upright single person
    [(1, 0, 0, 100, 40, 40, 90), EMPTY, EMPTY, EMPTY],
    # lying, head on the left, then the same box with the head mirrored
    [(1, 0, 0, 60, 120, 120, 40), EMPTY, EMPTY, EMPTY],
    [(1, 1, 0, 60, 120, 120, 40), EMPTY, EMPTY, EMPTY],
    # too small for a figure, and a box that fills the raster
    [(1, 0, 0, 10, 10, 16, 30), EMPTY, EMPTY, EMPTY],
    [(1, 0, 0, 0, 0, 319, 179), (1, 0, 0, 300, 150, 10, 12), EMPTY, EMPTY],
    # two overlapping people, the second one alarming
    [(1, 0, 0, 100, 50, 60, 80), EMPTY, (1, 0, 1, 120, 60, 60, 80), EMPTY],
    # a lying figure with the tracker blob on the other end
    [EMPTY, EMPTY, EMPTY, (1, 1, 0, 250, 150, 60, 20)],
    # nothing found: the picture has to pass through untouched
    [EMPTY, EMPTY, EMPTY, EMPTY],
]


def pattern() -> np.ndarray:
    """The bench's background: R = px[7:0], G = py[7:0], B = 0x5A."""
    xs = np.arange(W, dtype=np.uint16)
    ys = np.arange(H, dtype=np.uint16)
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[..., 0] = (xs & 0xFF)[None, :]
    img[..., 1] = (ys & 0xFF)[:, None]
    img[..., 2] = 0x5A
    return img


def bone_geometry(w: int, h: int) -> dict:
    """The bar list of the header comment, in the box's local frame."""
    rot = w > h
    L, C = (w, h) if rot else (h, w)
    hs = max(3, min(12, L // 6))
    t = max(1, C // 16)
    c3 = C // 3
    c2 = C // 2
    return {
        "rot": rot,
        "L": L,
        "C": C,
        "hs": hs,
        "t": t,
        "qh": hs >> 1,
        "sh": hs + 2,
        "sh_hi": hs + 2 + t + 1,
        "hip": (3 * L) // 5,
        "hip_hi": (3 * L) // 5 + t + 1,
        "arm_end": (3 * L) // 5 - L // 6,
        "arm_lo": max(0, c2 - t - 1),
        "leg_lo": max(0, c3 - t - 1),
        "leg_hi": c3 + t,
        "c2": c2,
        "c3": c3,
    }


def slot_mask(g: dict, x: int, y: int, vx: np.ndarray,
              vy: np.ndarray, direction: int = 0) -> np.ndarray:
    """Pixels of one figure, in vision coordinates."""
    dx = vx - x
    dy = vy - y
    su = dy if g["rot"] else dx
    sv = dx if g["rot"] else dy
    inside = (su >= 0) & (su <= g["L"]) & (sv >= 0) & (sv <= g["C"])
    u = su.astype(np.int32)
    if g["rot"] and direction:
        # the exact reversal of a half-open span: u -> L-1-u
        u = g["L"] - 1 - u
    du = np.abs(sv.astype(np.int32) - (g["C"] // 2))

    head = (u < g["hs"]) & (du < g["qh"])
    spine = (u >= g["hs"]) & (u < g["hip"]) & (du < g["t"])
    sh = (u >= g["sh"]) & (u < g["sh_hi"]) & (du < g["c3"])
    hip = (u >= g["hip"]) & (u < g["hip_hi"]) & (du < g["c3"])
    arm = ((u >= g["sh"]) & (u < g["arm_end"]) &
           (du >= g["arm_lo"]) & (du < g["c2"]))
    leg = ((u >= g["hip"]) & (u < g["L"]) &
           (du >= g["leg_lo"]) & (du < g["leg_hi"]))
    return inside & (head | spine | sh | hip | arm | leg)


def reference(case) -> np.ndarray:
    vx = (np.arange(W, dtype=np.int32) >> 1)[None, :]
    vy = ((np.arange(H, dtype=np.int32) * 3) >> 3)[:, None]
    out = pattern()
    hit = np.zeros((H, W), dtype=bool)
    colour = np.full((H, W), 255, dtype=np.uint8)   # 255 = no slot drew here
    alarm = np.zeros((H, W), dtype=bool)

    for k in range(NB - 1, -1, -1):          # the lowest slot wins the colour
        found, direction, al, x, y, w, h = case[k]
        if not found:
            continue
        g = bone_geometry(w, h)
        m = slot_mask(g, x, y, vx, vy, direction)
        # drawn highest slot first so the lowest slot wins an overlap, the
        # same order the RTL's colour loop produces
        hit |= m
        colour[m] = k
        if al:
            alarm |= m

    for k in range(NB):
        out[colour == k] = COLOURS[k]
    out[alarm] = ALARM_RGB
    return out


def run_rtl(cases) -> np.ndarray:
    cfg = GEN / "stick_cases.cfg"
    dump = GEN / "stick_cases.rgb"
    lines = []
    for case in cases:
        for slot in case:
            lines.append(" ".join(str(int(v)) for v in slot))
    cfg.write_text("\n".join(lines) + "\n")

    vvp = GEN / "sim_stick_overlay.vvp"
    cmd = [find_tool("iverilog"), "-g2005", "-o", str(vvp),
           str(FPGA / "rtl" / "hdmi" / "stick_overlay.v"),
           str(FPGA / "tb" / "tb_stick_overlay.v")]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"iverilog failed:\n{proc.stdout}\n{proc.stderr}")
    proc = subprocess.run([find_tool("vvp"), str(vvp), f"+CFG={cfg}",
                           f"+OUT={dump}"], capture_output=True, text=True)
    if "DONE" not in proc.stdout:
        raise SystemExit(f"simulation failed:\n{proc.stdout}\n{proc.stderr}")
    raw = np.frombuffer(dump.read_bytes(), dtype=np.uint8)
    return raw.reshape(-1, H, W, 3)


def pil_compare(case, got: np.ndarray) -> float:
    """How close the RTL lands to the PIL prototype (informational)."""
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return -1.0
    sys.path.insert(0, str(Path(
        r"C:\Users\ASUS\Documents\Codex\2026-10-04"
        r"\files-mentioned-by-the-user-eg4s20ng88")))
    try:
        from _stick_render import bars
    except ImportError:
        return -1.0

    img = pattern()
    pil = Image.fromarray(img)
    d = ImageDraw.Draw(pil)
    for k, slot in enumerate(case):
        found, direction, al, x, y, w, h = slot
        if not found or w <= 0 or h <= 0:
            continue
        rot = w > h
        for u0, u1, v0, v1 in bars(x, y, w, h):
            if rot:
                if direction:
                    u0, u1 = w - u1, w - u0
                p0 = (x + v0, y + u0)
                p1 = (x + v1, y + u1)
            else:
                p0 = (x + u0, y + v0)
                p1 = (x + u1, y + v1)
            d.rectangle([p0[0] * 2, p0[1] * 8 // 3,
                         p1[0] * 2 - 1, p1[1] * 8 // 3 - 1],
                        fill=COLOURS[k])
    ref = np.asarray(pil)
    return float((ref != got).any(axis=2).mean())


def main() -> int:
    GEN.mkdir(parents=True, exist_ok=True)
    got = run_rtl(CASES)
    if got.shape[0] != len(CASES):
        print(f"FAIL case count: got {got.shape[0]} want {len(CASES)}")
        return 1
    ok = True
    for i, case in enumerate(CASES):
        want = reference(case)
        diff = (want != got[i]).any(axis=2)
        n = int(diff.sum())
        if n:
            ok = False
            yy, xx = np.nonzero(diff)
            print(f"FAIL case {i}: {n} pixels differ, first at "
                  f"({xx[0]}, {yy[0]}) rtl={got[i, yy[0], xx[0]]} "
                  f"want={want[yy[0], xx[0]]}")
        else:
            extra = pil_compare(case, got[i])
            note = (f", {extra * 100:.3f}% vs PIL prototype"
                    if extra >= 0 else "")
            print(f"PASS case {i}: {W}x{H} raster exact{note}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
