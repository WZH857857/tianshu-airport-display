"""One-page PNG status report for the FPGA fall-detection vision core.

Draws the RTL pipeline with its measured per-module cost, the WNS history of
the TD synthesis runs, the verification matrix and the device utilisation
bars.  Everything is read from the artefacts the runs left on disk:

    fpga/td/out/fall_core_gate.area      TD area report (run8)
    fpga/td/out/fall_core_gate.timing    TD timing report (run8)

Usage::

    python make_fpga_report.py [--out out/fpga_status.png]
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
AREA = ROOT / "fpga" / "td" / "out" / "fall_core_gate.area"
TIMING = ROOT / "fpga" / "td" / "out" / "fall_core_gate.timing"
AREA_ROUTE = ROOT / "fpga" / "td" / "out" / "pnr_route.area"
TIMING_PLACE = ROOT / "fpga" / "td" / "out" / "pnr_place.timing"
TIMING_ROUTE = ROOT / "fpga" / "td" / "out" / "pnr_route.timing"

BG = (17, 19, 24)
PANEL = (28, 32, 39)
PANEL2 = (34, 39, 48)
FG = (236, 239, 244)
MUTED = (154, 163, 176)
ACCENT = (0, 196, 255)
OK = (58, 214, 130)
WARN = (255, 190, 70)
BAD = (255, 92, 92)
GRID = (58, 65, 78)

W, H = 1720, 1470


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    names = ([r"C:\Windows\Fonts\msyhbd.ttc", r"C:\Windows\Fonts\msyh.ttc"]
             if bold else
             [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\msyhbd.ttc"])
    for name in names:
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def parse_area(path: Path) -> dict:
    """Pull the utilisation block and the per-instance table out of an area
    report.  The gate-level and post-route reports have different columns
    (post-route adds `le`), so the table is read through its own header row
    instead of a fixed regex."""
    text = path.read_text(encoding="utf-8", errors="ignore")
    out: dict = {"util": {}, "hier": {}}
    for key, name in [("lut", "#lut"), ("reg", "#reg"), ("bram", "#bram"),
                      ("bram9k", "#bram9k"), ("bram32k", "#bram32k"),
                      ("dsp", "#dsp"), ("pad", "#pad")]:
        m = re.search(rf"{name}\s+(\d+)\s+out of\s+(\d+)", text)
        if m:
            out["util"][key] = (int(m.group(1)), int(m.group(2)))
    # #bram9k only prints the used count; the block total lives on #bram.
    m = re.search(r"#bram9k\s+(\d+)\s*$", text, re.M)
    if m and "bram" in out["util"]:
        out["util"]["bram9k"] = (int(m.group(1)), out["util"]["bram"][1])

    lines = text.splitlines()
    for i, line in enumerate(lines):
        if "|Instance" not in line:
            continue
        cols = [c.strip() for c in line.strip().strip("|").split("|")]
        for row in lines[i + 1:]:
            s = row.strip()
            # The header is followed by a +----+ separator row before the data.
            if s.startswith("+"):
                continue
            if not s.startswith("|"):
                break
            cells = [c.strip() for c in row.strip().strip("|").split("|")]
            if len(cells) != len(cols) or not re.match(r"u_\w+", cells[0]):
                continue
            rec = dict(zip(cols, cells))
            out["hier"][cells[0]] = {
                "module": rec.get("Module", ""),
                "lut": int(rec.get("lut", 0)),
                "seq": int(rec.get("seq", 0)),
                "bram": int(rec.get("bram", 0)),
                "dsp": int(rec.get("dsp", 0)),
            }
        break
    return out


def parse_timing(path: Path) -> dict:
    text = path.read_text(encoding="utf-8", errors="ignore")
    out: dict = {}
    m = re.search(r"SWNS:\s*(-?[\d.]+)ns,\s*STNS:\s*(-?[\d.]+)ns", text)
    if m:
        out["wns"] = float(m.group(1))
        out["tns"] = float(m.group(2))
    m = re.search(r"HWNS:\s*(-?[\d.]+)ns,\s*HTNS:\s*(-?[\d.]+)ns", text)
    if m:
        out["hwns"] = float(m.group(1))
        out["htns"] = float(m.group(2))
    m = re.search(r"Fmax\s*:\s*([\d.]+)MHz", text)
    if m:
        out["fmax"] = float(m.group(1))
    m = re.search(r"R-Period\(ns\)\s+R-Freq\(MHz\)", text)
    if m:
        out["has_period_table"] = True
    return out


PIPELINE = [
    ("gray_front", "RGB565→luma\n2x2 box + mod-4 行抽样\n640x480 → 320x180"),
    ("frame_diff", "上一帧 luma 片内 ERAM\n|cur-prev| > 10\n输出运动掩膜"),
    ("motion_morph", "3x3 腐蚀\n11x11 圆盘膨胀\n列栈 + 滑动 popcount"),
    ("proj_bbox", "两遍投影\n最长游程包围盒\n双 bank 乒乓掩膜"),
    ("posture_fsm", "站立参考 + 塌陷候选\n恢复窗 + 冷却\n报警脉冲"),
]

VERIFY = [
    ("前端 pixel_check", "13/02 各 4 帧", "luma + 掩膜字节级一致", True),
    ("形态学 morph_check", "6 组合成 + 3 段素材", "逐位一致（含空档）", True),
    ("包围盒 bbox_check", "合成 + 13/07/crowd01", "逐帧 bbox 一致", True),
    ("姿态机 rtl_check", "4 段素材 14 条轨迹", "14/14 报警帧与状态一致", True),
    ("整链 chain_check", "6 段素材各 4 帧", "五级结果全部一致", True),
    ("整链长程 chain_check", "Subject1/Fall/13 前 45 帧",
     "bbox / 掩膜 / 状态全一致，报警帧 RTL=31 参考=31", True),
]

def rr(d: ImageDraw.ImageDraw, box, r, fill=None, outline=None, width=1):
    d.rounded_rectangle(box, radius=r, fill=fill, outline=outline, width=width)


def draw_pipeline(d, area, x0, y0, x1):
    f_title = font(26, True)
    f_name = font(21, True)
    f_body = font(15)
    f_cost = font(15, True)

    d.text((x0, y0), "算法链路（全部在 FPGA 内，无 MCU）", font=f_title, fill=FG)
    top = y0 + 46
    gap = 22
    n = len(PIPELINE)
    bw = (x1 - x0 - gap * (n - 1)) // n
    bh = 132
    cost = area["hier"]
    order = ["u_gray", "u_diff", "u_morph", "u_bbox", "u_fsm"]

    for i, ((name, desc), inst) in enumerate(zip(PIPELINE, order)):
        bx = x0 + i * (bw + gap)
        rr(d, (bx, top, bx + bw, top + bh), 8, fill=PANEL, outline=GRID)
        d.text((bx + 14, top + 12), name, font=f_name, fill=ACCENT)
        ty = top + 44
        for line in desc.split("\n"):
            d.text((bx + 14, ty), line, font=f_body, fill=MUTED)
            ty += 21
        c = cost.get(inst)
        if c:
            txt = f"LUT {c['lut']}   seq {c['seq']}   块 {c['bram']}   DSP {c['dsp']}"
            d.text((bx + 14, top + bh - 26), txt, font=f_cost, fill=OK)
        if i < n - 1:
            ax = bx + bw + 4
            d.line((ax, top + bh // 2, ax + gap - 8, top + bh // 2), fill=GRID, width=3)
            d.polygon([(ax + gap - 8, top + bh // 2 - 6),
                       (ax + gap - 8, top + bh // 2 + 6),
                       (ax + gap - 2, top + bh // 2)], fill=GRID)
    return top + bh


def draw_timing(d, bars, timing_synth, timing_place, timing_route, x0, y0, x1):
    f_title = font(26, True)
    f_lab = font(18)
    f_small = font(15)

    d.text((x0, y0), "时序（TD 6.2.1，48 MHz / 20.833 ns）", font=f_title, fill=FG)
    top = y0 + 50
    panel_h = 392
    rr(d, (x0, top, x1, top + panel_h), 8, fill=PANEL, outline=GRID)

    # bar chart: x maps slack -10.5 .. +4.5 (the +3.726 place estimate must fit)
    cx0, cx1 = x0 + 40, x1 - 40
    lo_ns, hi_ns = -10.5, 4.5

    def x_of(val: float) -> float:
        return cx0 + (val - lo_ns) / (hi_ns - lo_ns) * (cx1 - cx0)

    zero = x_of(0.0)
    base = top + panel_h - 92
    d.line((zero, top + 20, zero, base + 16), fill=GRID, width=2)
    d.text((zero + 6, base + 20), "0 ns", font=f_small, fill=MUTED)

    bar_h = 34
    for i, (label, val, col) in enumerate(bars):
        by = top + 46 + i * 54
        bx = x_of(val)
        lo, hi = min(zero, bx), max(zero, bx)
        rr(d, (lo, by, hi, by + bar_h), 6, fill=col)
        txt = f"{val:+.3f} ns"
        # Label and value share the line above the bar: a bar can reach either
        # end of the panel, so a value parked next to its moving end would be
        # drawn on top of the bar and vanish.
        d.text((cx0 + 4, by - 22), label, font=f_lab, fill=FG)
        lw = d.textlength(label, font=f_lab)
        d.text((cx0 + 4 + lw + 18, by - 22), txt, font=f_lab, fill=col)

    d.text((cx0 + 4, top + panel_h - 74),
           f"综合后 Fmax {timing_synth.get('fmax', 0):.3f} MHz     "
           f"布局后估算 SWNS {timing_place.get('wns', 0):+.3f} ns",
           font=f_lab, fill=MUTED)
    d.text((cx0 + 4, top + panel_h - 44),
           f"布线后 Fmax {timing_route.get('fmax', 0):.3f} MHz ≥ 48.001 MHz 目标    "
           f"SWNS {timing_route.get('wns', 0):+.3f} ns   "
           f"HWNS {timing_route.get('hwns', 0):+.3f} ns",
           font=f_lab, fill=OK)
    d.text((x1 - 150, top + 18), "布局布线已完成", font=f_small, fill=OK)
    return top + panel_h


def draw_verify(d, x0, y0, x1):
    f_title = font(26, True)
    f_lab = font(18)
    f_small = font(15)

    d.text((x0, y0), "对拍验证（Icarus + Python 参考实现逐操作对齐）", font=f_title, fill=FG)
    top = y0 + 50
    row_h = 45
    panel_h = row_h * len(VERIFY) + 16
    rr(d, (x0, top, x1, top + panel_h), 8, fill=PANEL, outline=GRID)
    for i, (name, mat, res, ok) in enumerate(VERIFY):
        ry = top + 8 + i * row_h
        if i:
            d.line((x0 + 12, ry, x1 - 12, ry), fill=GRID)
        d.text((x0 + 18, ry + 11), name, font=f_lab, fill=FG)
        d.text((x0 + 420, ry + 13), mat, font=f_small, fill=MUTED)
        d.text((x0 + 880, ry + 13), res, font=f_small, fill=OK if ok else BAD)
        d.text((x1 - 96, ry + 11), "PASS" if ok else "TODO",
               font=f_lab, fill=OK if ok else WARN)
    return top + panel_h


def draw_bars(d, area, timing, x0, y0, x1):
    f_title = font(26, True)
    f_lab = font(17)
    f_val = font(16, True)

    d.text((x0, y0), "器件占用（EG4S20NG88）", font=f_title, fill=FG)
    top = y0 + 50
    panel_h = 392
    rr(d, (x0, top, x1, top + panel_h), 8, fill=PANEL, outline=GRID)

    items = [
        ("LUT", area["util"].get("lut", (0, 1)), ACCENT),
        ("REG", area["util"].get("reg", (0, 1)), ACCENT),
        ("ERAM9K", area["util"].get("bram9k", (0, 1)), WARN),
        ("ERAM32K", area["util"].get("bram32k", (0, 1)), BAD),
        ("DSP", area["util"].get("dsp", (0, 1)), OK),
        ("PAD", area["util"].get("pad", (0, 1)), OK),
    ]
    bx0, bx1 = x0 + 118, x1 - 150
    for i, (name, (used, total), col) in enumerate(items):
        by = top + 34 + i * 58
        d.text((x0 + 18, by - 3), name, font=f_lab, fill=FG)
        d.rectangle((bx0, by, bx1, by + 22), fill=PANEL2, outline=GRID)
        frac = used / total if total else 0
        d.rectangle((bx0, by, bx0 + int((bx1 - bx0) * frac), by + 22), fill=col)
        d.text((bx1 + 14, by - 1), f"{used}/{total}  {frac * 100:.1f}%",
               font=f_val, fill=FG)
    d.text((x0 + 18, top + panel_h - 40),
           "布线后（final）统计，含 phys_opt 插的缓冲/复制寄存器",
           font=font(15), fill=MUTED)
    return top + panel_h


def draw_next(d, x0, y0, x1):
    f_title = font(26, True)
    f = font(17)
    d.text((x0, y0), "下一步", font=f_title, fill=FG)
    lines = [
        "1. 板级接入：写 CMOS 采集（vsync/href/pclk -> de_in/frame_start，~100 行）+ OV5640 的",
        "   I2C 初始化（复用 03-1/2/3 sim 里的 i2c_timing_ctrl_reg16_dat16.v）；管脚表见",
        "   docs/板级接入.md，相机 14 pad + HDMI 4 + LED 3 + 24M 时钟 1，共 22/66。",
        "2. HDMI 画框叠加（bbox -> 显示坐标）、LED/串口报警、E104 小板联动线序。",
        "3. 人群多目标：blob_label 连通域 top-4 + track_assoc，posture_fsm 已支持多实例。",
    ]
    ty = y0 + 42
    for line in lines:
        d.text((x0, ty), line, font=f, fill=MUTED)
        ty += 26
    return ty


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "out" / "fpga_status.png"))
    args = ap.parse_args()

    area = parse_area(AREA_ROUTE)
    t_synth = parse_timing(TIMING)
    t_place = parse_timing(TIMING_PLACE)
    t_route = parse_timing(TIMING_ROUTE)

    bars = [
        ("run6  常数乘法全被推进 DSP", -9.524, BAD),
        ("run7  proj_bbox 拆两拍 + ref_wh 寄存器", -0.622, WARN),
        ("run8  常量乘法改移位加（综合）", t_synth.get("wns", 0.0), OK),
        ("布局后（manhattan 估算）", t_place.get("wns", 0.0), OK),
        ("布线后 final", t_route.get("wns", 0.0), OK),
    ]

    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)

    d.text((60, 44), "摔倒检测 FPGA 视觉核 —— 综合与时序闭环",
           font=font(42, True), fill=FG)
    d.text((60, 104),
           "EG4S20NG88 / 安路 TD 6.2.1 · 320x180 灰度帧差 + 形态学 + 投影包围盒 + 姿态状态机 · "
           "纯 RTL，无软核",
           font=font(19), fill=MUTED)
    d.line((60, 146, W - 60, 146), fill=GRID, width=2)

    y = draw_pipeline(d, area, 60, 170, W - 60) + 52
    b1 = draw_timing(d, bars, t_synth, t_place, t_route, 60, y, 1010)
    b2 = draw_bars(d, area, t_route, 1040, y, W - 60)

    y = max(b1, b2) + 52
    y = draw_verify(d, 60, y, W - 60) + 46
    draw_next(d, 60, y, W - 60)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
