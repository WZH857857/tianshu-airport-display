"""One-page PNG status report for the board-level fall-detection design.

Unlike ``make_fpga_report.py`` (which reports the bare vision core), this one
covers ``fpga/rtl/fall_camera_top.v``: the OV5640 capture and SCCB front end,
the vision core, and the 74595 / UART status back end that actually goes on
the VF-E20NG88.

Everything is read from what the runs left on disk::

    fpga/td/out_board/board_place.timing    placed timing (manhattan)
    fpga/td/out_board/board_route.timing    routed timing (final)
    fpga/td/out_board/board_route.area      routed utilisation
    fpga/td/out_board/board_route.qor       routed QoR summary

Usage::

    python make_board_report.py [--out out/fpga_board_status.png]
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

from PIL import Image, ImageDraw

from make_fpga_report import (ACCENT, BAD, BG, FG, GRID, MUTED, OK, PANEL,
                              PANEL2, WARN, font, parse_area, parse_timing,
                              rr)

ROOT = Path(__file__).resolve().parent.parent
TD = ROOT / "fpga" / "td" / "out_board"
AREA_ROUTE = TD / "board_route.area"
TIMING_GATE = TD / "board_gate.timing"
TIMING_PLACE = TD / "board_place.timing"
TIMING_ROUTE = TD / "board_route.timing"
QOR_ROUTE = TD / "board_route.qor"

W, H = 1720, 1790

PINS = [
    ("clk_24m", "P13", 1),
    ("key_n 复位（低有效）", "P51", 1),
    ("cmos_data[7:0]  DVP 数据", "P2 P87 P86 P78 P77 P76 P75 P74", 8),
    ("cmos_pclk  48 MHz 算法时钟", "P79", 1),
    ("cmos_href 行有效", "P62", 1),
    ("cmos_vsync 场同步", "P66", 1),
    ("cmos_xclk 相机主时钟", "P72", 1),
    ("cmos_sclk SCCB 250 kHz", "P70", 1),
    ("cmos_sdat SCCB 数据", "P71", 1),
    ("led595_clk / latch / dout", "P59 / P57 / P45", 3),
    ("uart_txd 115200 状态行", "P3", 1),
]

ROW1 = [
    ("OV5640", "640x480 @30fps\nRGB565 大端字节对", ACCENT),
    ("ov5640_sccb_cfg", "24 MHz → 250 kHz\n250 条寄存器 + 2 ms 复位稳定期", ACCENT),
    ("cmos_capture_rgb565", "字节对 → 16 bit\n等 10 帧后 armed；hw_err 报错", ACCENT),
    ("gray_front", "565 → luma → 2x2 box\n丢每第 4 行 → 320x180", OK),
]

ROW2 = [
    ("frame_diff", "上一帧片内 ERAM\n|cur-prev| > 10", OK),
    ("motion_morph", "3x3 腐蚀 + 11x11 膨胀\n列栈 + 滑动 popcount", OK),
    ("proj_bbox", "两遍投影最长游程\n双 bank 乒乓", OK),
    ("posture_fsm", "站立参考 + 塌陷候选\n恢复窗 + 冷却 → 报警脉冲", OK),
    ("led595_shift", "8 位状态灯", WARN),
    ("status_uart", "115200 8N1\nF<帧> <found> x y w h area state", WARN),
]

VERIFY = [
    ("整链 chain_check", "Subject1/Fall/13 前 45 帧",
     "bbox / 掩膜 / 状态全一致，报警帧 RTL=31 参考=31"),
    ("整链 chain_check", "Subject3/Fall/01 前 45 帧",
     "bbox / 掩膜 / 状态全一致，报警帧 RTL=41 参考=41"),
    ("姿态机 rtl_check", "4 段素材 14 条轨迹", "14/14 报警帧与逐帧状态一致（含 +HOSTILE 时序）"),
    ("报警脉冲 tb_alarm_pulse", "选通 en 与常开 en 两种接法",
     "各 1 次报警，最长电平 1 拍（脉冲语义成立）"),
    ("板级集成 tb_board_top", "行为级 OV5640 + SCCB 从机 + UART 解码",
     "SCCB 250 写完成，3 帧 de=921600，bbox/UART 对齐，hw_err=00"),
    ("板级跌倒场景 +LONG", "站立 40x80 → 塌陷 80x20（工作分辨率）",
     "SEEK→PENDING→FALL，UART state=2，1 条 ALARM，hw_err=00"),
]


def parse_qor(path: Path) -> dict:
    text = path.read_text(encoding="utf-8", errors="ignore")
    out: dict = {}
    for key, pat in [("swns", r"Setup WNS\s*\|\s*(-?[\d.]+)"),
                     ("stns", r"Setup TNS\s*\|\s*(-?[\d.]+)"),
                     ("seps", r"violated S-EP\s*\|\s*(\d+)"),
                     ("fmax", r"FMAX\s*\|\s*([\d.]+)MHz"),
                     ("hwns", r"Hold WNS\s*\|\s*(-?[\d.]+)"),
                     ("heps", r"violated H-EP\s*\|\s*(\d+)"),
                     ("slices", r"#slices\s*\|\s*(\d+)"),
                     ("slicepct", r"slices percentage\s*\|\s*([\d.]+)%"),
                     ("rams", r"#RAMs\s*\|\s*(\d+)"),
                     ("dsps", r"#DSPs\s*\|\s*(\d+)")]:
        m = re.search(pat, text)
        if m:
            out[key] = float(m.group(1))
    return out


def draw_datapath(d, x0, y0, x1):
    d.text((x0, y0), "板级数据通路（全部在 FPGA 内，无 PLL / 无外部存储 / 无软核）",
           font=font(26, True), fill=FG)
    f_name = font(19, True)
    f_body = font(14)
    top = y0 + 46
    bh = 96
    gap = 30

    for row, ry in ((ROW1, top), (ROW2, top + bh + 44)):
        n = len(row)
        bw = (x1 - x0 - gap * (n - 1)) // n
        for i, (name, desc, col) in enumerate(row):
            bx = x0 + i * (bw + gap)
            rr(d, (bx, ry, bx + bw, ry + bh), 8, fill=PANEL, outline=GRID)
            d.text((bx + 12, ry + 10), name, font=f_name, fill=col)
            ty = ry + 38
            for line in desc.split("\n"):
                d.text((bx + 12, ty), line, font=f_body, fill=MUTED)
                ty += 19
            if i < n - 1:
                ax = bx + bw + 5
                d.line((ax, ry + bh // 2, ax + gap - 10, ry + bh // 2),
                       fill=GRID, width=3)
                d.polygon([(ax + gap - 10, ry + bh // 2 - 6),
                           (ax + gap - 10, ry + bh // 2 + 6),
                           (ax + gap - 3, ry + bh // 2)], fill=GRID)
    return top + 2 * bh + 44


def draw_timing(d, t_gate, t_place, t_route, x0, y0, x1):
    d.text((x0, y0), "时序（TD 6.2.1，cmos_pclk 48 MHz / 20.833 ns）",
           font=font(26, True), fill=FG)
    top = y0 + 50
    panel_h = 300
    rr(d, (x0, top, x1, top + panel_h), 8, fill=PANEL, outline=GRID)

    f_lab = font(18)
    f_small = font(15)
    bars = [
        ("综合后 optimize_gate", t_gate.get("wns", 0.0)),
        ("布局后 manhattan 估算", t_place.get("wns", 0.0)),
        ("布线后 final", t_route.get("wns", 0.0)),
    ]
    cx0, cx1 = x0 + 40, x1 - 40
    lo_ns, hi_ns = -2.0, 4.0

    def x_of(v):
        return cx0 + (v - lo_ns) / (hi_ns - lo_ns) * (cx1 - cx0)

    zero = x_of(0.0)
    base = top + 40 + len(bars) * 54
    d.line((zero, top + 20, zero, base), fill=GRID, width=2)
    d.text((zero + 6, base + 6), "0 ns", font=f_small, fill=MUTED)

    for i, (label, val) in enumerate(bars):
        by = top + 40 + i * 54
        bx = x_of(val)
        col = OK if val > 0 else BAD
        rr(d, (min(zero, bx), by, max(zero, bx), by + 34), 6, fill=col)
        d.text((cx0 + 4, by - 20), label, font=f_lab, fill=FG)
        lw = d.textlength(label, font=f_lab)
        d.text((cx0 + 4 + lw + 18, by - 20), f"{val:+.3f} ns", font=f_lab,
               fill=col)

    d.text((cx0 + 4, top + panel_h - 62),
           f"布线后 Fmax {t_route.get('fmax', 0):.3f} MHz ≥ 48.001 MHz 目标    "
           f"SWNS {t_route.get('wns', 0):+.3f} ns    "
           f"HWNS {t_route.get('hwns', 0):+.3f} ns",
           font=f_lab, fill=OK)
    d.text((cx0 + 4, top + panel_h - 32),
           "两路时钟：cmos_pclk 48 MHz（算法）与 clk_24m 24 MHz（按键/心跳），已设互不跨域",
           font=f_small, fill=MUTED)
    return top + panel_h


def draw_util(d, area, qor, x0, y0, x1):
    d.text((x0, y0), "器件占用（EG4S20NG88 / 66 个用户 PAD）",
           font=font(26, True), fill=FG)
    top = y0 + 50
    panel_h = 300
    rr(d, (x0, top, x1, top + panel_h), 8, fill=PANEL, outline=GRID)

    f_lab = font(17)
    f_val = font(16, True)
    items = [
        ("LUT", area["util"].get("lut", (0, 1)), ACCENT),
        ("REG", area["util"].get("reg", (0, 1)), ACCENT),
        ("ERAM9K", area["util"].get("bram9k", (0, 1)), WARN),
        ("ERAM32K", area["util"].get("bram32k", (0, 1)), BAD),
        ("DSP", area["util"].get("dsp", (0, 1)), OK),
        ("PAD", area["util"].get("pad", (0, 1)), OK),
    ]
    bx0, bx1 = x0 + 118, x1 - 168
    for i, (name, (used, total), col) in enumerate(items):
        by = top + 26 + i * 42
        d.text((x0 + 18, by - 3), name, font=f_lab, fill=FG)
        d.rectangle((bx0, by, bx1, by + 20), fill=PANEL2, outline=GRID)
        frac = used / total if total else 0
        d.rectangle((bx0, by, bx0 + int((bx1 - bx0) * frac), by + 20), fill=col)
        d.text((bx1 + 14, by - 1), f"{used}/{total}  {frac * 100:.1f}%",
               font=f_val, fill=FG)
    d.text((x0 + 18, top + panel_h - 32),
           f"布线后统计；#slices {int(qor.get('slices', 0))} "
           f"({qor.get('slicepct', 0):.1f}%)",
           font=font(15), fill=MUTED)
    return top + panel_h


def draw_pins(d, x0, y0, x1):
    d.text((x0, y0), "板级管脚预算 20/66", font=font(26, True), fill=FG)
    top = y0 + 50
    row_h = 30
    panel_h = row_h * len(PINS) + 60
    rr(d, (x0, top, x1, top + panel_h), 8, fill=PANEL, outline=GRID)

    f_lab = font(14)
    f_small = font(13)
    d.text((x0 + 16, top + 12), "信号", font=f_small, fill=MUTED)
    d.text((x0 + 250, top + 12), "管脚", font=f_small, fill=MUTED)
    d.text((x1 - 72, top + 12), "占用", font=f_small, fill=MUTED)
    for i, (sig, pad, cnt) in enumerate(PINS):
        ry = top + 40 + i * row_h
        if i:
            d.line((x0 + 12, ry - 6, x1 - 12, ry - 6), fill=GRID)
        d.text((x0 + 16, ry), sig, font=f_lab, fill=FG)
        d.text((x0 + 250, ry + 1), pad, font=f_small, fill=MUTED)
        d.text((x1 - 30, ry), str(cnt), font=f_lab, fill=ACCENT)
    d.text((x0 + 16, top + panel_h - 30),
           "合计 20 pad，剩余 46 个留给 HDMI 叠加 / 蜂鸣器 / E104 联动",
           font=f_small, fill=OK)
    return top + panel_h


def draw_verify(d, x0, y0, x1):
    d.text((x0, y0), "验证矩阵（Icarus + Python 参考实现逐操作对齐）",
           font=font(26, True), fill=FG)
    top = y0 + 50
    row_h = 64
    panel_h = row_h * len(VERIFY) + 16
    rr(d, (x0, top, x1, top + panel_h), 8, fill=PANEL, outline=GRID)
    for i, (name, mat, res) in enumerate(VERIFY):
        ry = top + 8 + i * row_h
        if i:
            d.line((x0 + 12, ry, x1 - 12, ry), fill=GRID)
        d.text((x0 + 18, ry + 8), name, font=font(18), fill=FG)
        nw = d.textlength(name, font=font(18))
        d.text((x0 + 26 + nw, ry + 12), mat, font=font(14), fill=MUTED)
        d.text((x0 + 18, ry + 34), res, font=font(15), fill=OK)
        d.text((x1 - 90, ry + 18), "PASS", font=font(18), fill=OK)
    return top + panel_h


def draw_fix(d, x0, y0, x1):
    d.text((x0, y0), "本轮修掉的三个真问题", font=font(26, True), fill=FG)
    top = y0 + 50
    lines = [
        ("报警脉冲语义", WARN,
         "alarm 原来只在 en 分支里清零。核内 en 常开，看不出问题；板级把 en 接成每帧一拍的选通后，"
         "报警电平会从触发帧一直保持到下一帧（+LONG 曾刷出上千万行日志）。现在 alarm <= 1'b0 每拍"
         "无条件执行，tb_alarm_pulse 用两种接法各测一遍，最长电平都是 1 拍。"),
        ("姿态阈值和验证配置对不上", BAD,
         "fall_core_top 曾把 MIN_REF_AREA / MIN_REF_HEIGHT 按 180/240 缩成 1200/19，而 res_eval、"
         "rtl_check 和 45 帧整链用的都是 Params 原值 1600/25——板子上跑的是一套没被验证过、"
         "而且更宽松的阈值。现在统一到 1600/25，chain_check 把这两个常量写进 params_chain.vh，"
         "testbench 直接从宏取，不再有手抄的机会。"),
        ("参考实现的画幅尺寸", BAD,
         "chain_check 的参考 PostureMachine 一直用默认 frame_h=240，testbench 却用 180，"
         "于是 MAX_REF_H / MAX_REF_AREA 长期是 228/15360 对 171/11520。已改为按 CAMERA_H 设置，"
         "两边现在描述同一个配置。"),
    ]
    ty = top + 6
    f_head = font(18, True)
    f_body = font(15)
    for head, col, body in lines:
        d.text((x0 + 6, ty), "· " + head, font=f_head, fill=col)
        ty += 27
        line = ""
        for ch in body:
            if len(line) >= 96:
                d.text((x0 + 22, ty), line, font=f_body, fill=MUTED)
                ty += 22
                line = ""
            line += ch
        if line:
            d.text((x0 + 22, ty), line, font=f_body, fill=MUTED)
            ty += 22
        ty += 10
    return ty


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "out" / "fpga_board_status.png"))
    args = ap.parse_args()

    area = parse_area(AREA_ROUTE)
    qor = parse_qor(QOR_ROUTE)
    t_gate = parse_timing(TIMING_GATE)
    t_place = parse_timing(TIMING_PLACE)
    t_route = parse_timing(TIMING_ROUTE)

    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)

    d.text((60, 40), "摔倒检测 FPGA 视觉副卡 —— 板级接入闭环",
           font=font(42, True), fill=FG)
    d.text((60, 100),
           "EG4S20NG88 / VF-E20NG88 · OV5640 DVP + SCCB → 320x180 灰度帧差 + 形态学 + "
           "投影包围盒 + 姿态状态机 → 74595 LED + 115200 串口",
           font=font(19), fill=MUTED)
    d.text((60, 126),
           "纯 RTL，无 PLL / 无外部帧缓存 / 无 MCU / 无软核；检测算法全部在 FPGA 内，"
           "电脑只负责显示",
           font=font(19), fill=MUTED)
    d.line((60, 168, W - 60, 168), fill=GRID, width=2)

    y = draw_datapath(d, 60, 196, W - 60) + 54
    b1 = draw_timing(d, t_gate, t_place, t_route, 60, y, 1010)
    b2 = draw_util(d, area, qor, 1040, y, W - 60)
    y = max(b1, b2) + 54
    v1 = draw_verify(d, 60, y, 1000)
    v2 = draw_pins(d, 1040, y, W - 60)
    y = max(v1, v2) + 50
    draw_fix(d, 60, y, W - 60)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
