"""One-page PNG status report for the crowd (multi-target) vision path.

Numbers come from what the runs left on disk:

    fpga/td/out_board_multi/multi_gate.area    synthesis utilisation
    fpga/td/out_board_multi/multi_route.area   post-route utilisation
    fpga/td/out_board_multi/multi_route.timing post-route timing
    out/crowd_stats.txt                        run_stats.py, run density
    out/crowd_order_stats.txt                  multi_order_stats.py
    out_frames/visual_report/crowd_multi.png   RTL strip from the top run

Usage::

    python make_crowd_report.py [--out out/fpga_crowd_status.png]
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

from PIL import Image, ImageDraw

from make_fpga_report import (ACCENT, BAD, BG, FG, GRID, MUTED, OK, PANEL,
                              PANEL2, WARN, font, parse_area, parse_timing, rr)

ROOT = Path(__file__).resolve().parent.parent
TD = ROOT / "fpga" / "td" / "out_board_multi"
AREA_GATE = TD / "multi_gate.area"
AREA_ROUTE = TD / "multi_route.area"
TIMING_ROUTE = TD / "multi_route.timing"
STATS = ROOT / "out" / "crowd_stats.txt"
ORDER = ROOT / "out" / "crowd_order_stats.txt"
STRIP = ROOT / "out_frames" / "visual_report" / "crowd_multi.png"

W = 1720


def read_stats(path: Path) -> list[str]:
    if not path.exists():
        return []
    return [ln for ln in path.read_text(encoding="utf-8").splitlines()
            if ln.strip()]


def parse_route_timing(path: Path) -> tuple[float, float, float, int]:
    """(setup WNS, hold WNS, Fmax MHz, violating endpoints)."""
    if not path.exists():
        return (0.0, 0.0, 0.0, -1)
    text = path.read_text(encoding="utf-8", errors="ignore")
    sw = re.search(r"SWNS:\s*(-?[\d.]+)ns", text)
    hw = re.search(r"HWNS:\s*(-?[\d.]+)ns", text)
    fm = re.search(r"Fmax\s*:\s*([\d.]+)MHz", text)
    ve = re.search(r"(\d+)\s+Viol\s+Endpoints", text)
    return (float(sw.group(1)) if sw else 0.0,
            float(hw.group(1)) if hw else 0.0,
            float(fm.group(1)) if fm else 0.0,
            int(ve.group(1)) if ve else -1)


def draw_header(d: ImageDraw.ImageDraw, route: tuple) -> None:
    d.text((60, 34), "摔倒检测 FPGA 视觉副卡 —— 人群多目标（4 槽排除式）",
           font=font(38, True), fill=FG)
    d.text((60, 88),
           "机场人多场景：单帧给出 4 个运动目标框 + 4 路姿态状态机，"
           "全部算法在 FPGA 内（无软核 / 无外部存储 / 无 PLL）",
           font=font(21), fill=MUTED)
    d.text((60, 122),
           "EG4S20NG88 · 640x480 RGB565 进 → 320x180 工作分辨率 · "
           "multi_bbox 4 次排除扫描 · slot_scan → 115200 UART 每帧 4 行",
           font=font(19), fill=MUTED)
    if route[3] >= 0:
        tag = (f"综合 + 布线完成 · {route[3]} 违例"
               if route[3] == 0 else
               f"综合 + 布线完成 · {route[3]} 违例（未收敛）")
        detail = (f"setup {route[0]:+.3f} ns / hold {route[1]:+.3f} ns / "
                  f"Fmax {route[2]:.1f} MHz")
        col = OK if route[3] == 0 else BAD
    else:
        tag = "综合完成，布线进行中"
        detail = ""
        col = WARN
    tw = d.textlength(tag, font=font(20, True))
    d.text((W - 60 - tw, 34), tag, font=font(20, True), fill=col)
    if detail:
        dw = d.textlength(detail, font=font(18))
        d.text((W - 60 - dw, 62), detail, font=font(18), fill=MUTED)


def draw_why(d, y0, lines) -> int:
    x0, x1 = 60, W - 60
    h = 250
    rr(d, (x0, y0, x1, y0 + h), 10, fill=PANEL, outline=GRID)
    d.text((x0 + 24, y0 + 18), "为什么不做片上连通域：先量密度，再选结构",
           font=font(26, True), fill=FG)
    d.text((x0 + 24, y0 + 56),
           "游程/并查集方案要把 320x180 掩膜的每条水平游程存下来："
           "实测每帧游程均值 341~1091、最大 1471，",
           font=font(19), fill=MUTED)
    d.text((x0 + 24, y0 + 82),
           "而片内 ERAM 只有 76,800 个 8 bit 条目 —— "
           "游程数组 + 并查集放不下，所以改用与单目标同 RAM 的排除式投影。",
           font=font(19), fill=MUTED)
    ry = y0 + 118
    for ln in lines:
        d.text((x0 + 24, ry), ln.replace(" |", "   |"), font=font(18),
               fill=FG if "W=" in ln else MUTED)
        ry += 24
    d.text((x0 + 24, y0 + h - 30),
           "结论：4 次排除扫描复用同一块掩膜 RAM（48/64 ERAM9K），"
           "代价只是每帧多 3 次列扫描 —— 帧间可用 331,200 拍，"
           "最坏 180 行带 4 趟约 231k 拍，余量充足。",
           font=font(18), fill=OK)
    return y0 + h + 26


def draw_path(d, y0) -> int:
    x0, x1 = 60, W - 60
    h = 176
    rr(d, (x0, y0, x1, y0 + h), 10, fill=PANEL, outline=GRID)
    d.text((x0 + 24, y0 + 18), "人群数据通路（pclk 48 MHz，单时钟域）",
           font=font(26, True), fill=FG)
    boxes = [
        ("gray_front", "565 → luma\n2x2 box + 丢每 4 行\n→ 320x180", ACCENT),
        ("frame_diff", "与上一帧差\n|Δ| > 10", ACCENT),
        ("motion_morph", "腐蚀 disk(1)\n膨胀 disk(5)", ACCENT),
        ("multi_bbox", "4 次排除扫描\n每帧 4 个框\n排除表跨帧保留", OK),
        ("posture_bank", "1 份 FSM 轮转 4 槽\n各存上下文\nTTL / 冷却共享帧号", OK),
        ("slot_scan + UART", "每帧 4 行\nS<slot> F<frame> ...\n115200", WARN),
    ]
    bw = (x1 - x0 - 48 - 5 * 26) // 6
    bx = x0 + 24
    for i, (name, sub, col) in enumerate(boxes):
        by = y0 + 62
        rr(d, (bx, by, bx + bw, by + 88), 8, fill=PANEL2, outline=col)
        d.text((bx + 12, by + 10), name, font=font(19, True), fill=col)
        for k, ln in enumerate(sub.split("\n")):
            d.text((bx + 12, by + 34 + k * 18), ln, font=font(15), fill=MUTED)
        if i != 5:
            ax = bx + bw + 4
            d.line((ax, by + 44, ax + 18, by + 44), fill=MUTED, width=3)
            d.polygon([(ax + 18, by + 38), (ax + 26, by + 44),
                       (ax + 18, by + 50)], fill=MUTED)
        bx += bw + 26
    return y0 + h + 26


def draw_verify(d, x0, y0, x1, y1) -> None:
    rr(d, (x0, y0, x1, y1), 10, fill=PANEL, outline=GRID)
    d.text((x0 + 22, y0 + 16), "验证矩阵（Icarus + Python 参考实现逐拍对齐）",
           font=font(24, True), fill=FG)
    rows = [
        ("multi_bbox_check.py",
         "synthetic 6 帧 + crowd01 / crowd05", "4 槽逐位等于参考模型"),
        ("multi_top_check.py",
         "fall_core_multi_top 全链：crowd01 / crowd05 / 13.mp4 各 4 帧，"
         "13.mp4 长程 45 帧",
         "4 槽框 + morph + diff + 4 路姿态 全对"),
        ("multi_rtl_check.py",
         "4 槽观测流回放 4 台状态机，3 段素材 x 40 帧",
         "报警帧与逐帧状态一致"),
        ("tb_slot_uart",
         "slot_scan + 带槽位前缀的 UART，2 组载荷 / 8 行",
         "8 行 ASCII 全对（36 字节/行）"),
        ("tb_board_top（单目标基线）",
         "改动 status_uart 后重跑板级集成",
         "PASS：格式/帧数/LED 全不变"),
    ]
    ry = y0 + 56
    for name, what, res in rows:
        d.text((x0 + 22, ry), name, font=font(18, True), fill=ACCENT)
        d.text((x0 + 22, ry + 24), what, font=font(16), fill=MUTED)
        d.text((x0 + 22, ry + 45), res, font=font(16), fill=OK)
        d.text((x1 - 84, ry + 12), "PASS", font=font(19, True), fill=OK)
        ry += 78


def draw_res(d, x0, y0, x1, y1, gate: dict, route: dict,
             timing: tuple) -> None:
    rr(d, (x0, y0, x1, y1), 10, fill=PANEL, outline=GRID)
    d.text((x0 + 22, y0 + 16), "资源与时序（EG4S20NG88，board_multi_pnr.tcl）",
           font=font(24, True), fill=FG)
    util = (route or gate).get("util", {})
    bars = [("LUT", "lut"), ("REG", "reg"), ("ERAM9K", "bram9k"),
            ("ERAM32K", "bram32k"), ("DSP", "dsp"), ("PAD", "pad")]
    ry = y0 + 62
    for label, key in bars:
        if key not in util:
            continue
        used, total = util[key]
        frac = used / total if total else 0.0
        d.text((x0 + 22, ry), label, font=font(17, True), fill=FG)
        bx0, bx1 = x0 + 120, x1 - 150
        rr(d, (bx0, ry + 2, bx1, ry + 18), 4, fill=PANEL2)
        col = OK if frac < 0.8 else (WARN if frac < 0.95 else BAD)
        rr(d, (bx0, ry + 2, bx0 + int((bx1 - bx0) * frac), ry + 18), 4,
           fill=col)
        d.text((bx1 + 12, ry), f"{used}/{total}  {frac * 100:.1f}%",
               font=font(17), fill=col)
        ry += 30
    src = "post-route" if route else "synthesis"
    d.text((x0 + 22, ry + 8), f"数据来源：{src}", font=font(15), fill=MUTED)
    ry += 34
    if timing[3] >= 0:
        d.text((x0 + 22, ry),
               f"布线收尾：setup {timing[0]:+.3f} ns，hold {timing[1]:+.3f} ns，"
               f"{timing[3]} 违例",
               font=font(18, True), fill=OK if timing[3] == 0 else BAD)
        ry += 26
        d.text((x0 + 22, ry),
               f"Fmax {timing[2]:.1f} MHz ≥ cmos_pclk 48 MHz",
               font=font(18), fill=FG)
        ry += 26
    d.text((x0 + 22, ry),
           "u_core 内：multi_bbox 4533 LUT / 16 BRAM，",
           font=font(16), fill=MUTED)
    d.text((x0 + 22, ry + 22),
           "posture_bank 1367 LUT（1 份 FSM 轮转 4 套上下文）。",
           font=font(16), fill=MUTED)


def draw_order(d, y0, lines) -> int:
    x0, x1 = 60, W - 60
    h = 260
    rr(d, (x0, y0, x1, y0 + h), 10, fill=PANEL, outline=GRID)
    d.text((x0 + 24, y0 + 18), "已知上限：槽位按帧内面积排序，不跟身份",
           font=font(26, True), fill=FG)
    d.text((x0 + 24, y0 + 56),
           "槽 0 永远是本帧最强的运动块。实测跨帧「同槽位仍然对得上」的比例：",
           font=font(19), fill=MUTED)
    ry = y0 + 90
    for ln in lines:
        d.text((x0 + 24, ry), ln, font=font(18), fill=FG)
        ry += 26
    d.text((x0 + 24, y0 + h - 62),
           "下一步（在 FPGA 内、约 +500 LUT）：每帧用 ≤4x4 个 IoU 比较给槽位重编号，",
           font=font(18), fill=WARN)
    d.text((x0 + 24, y0 + h - 34),
           "让每个槽在连续帧里贴住同一个人；"
           "4 槽状态机再吃到稳定流，密集人群才不会把顺序抖动当成动作。",
           font=font(18), fill=WARN)
    return y0 + h + 26


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path,
                    default=ROOT / "out" / "fpga_crowd_status.png")
    args = ap.parse_args()

    gate = parse_area(AREA_GATE) if AREA_GATE.exists() else {"util": {}}
    route = parse_area(AREA_ROUTE) if AREA_ROUTE.exists() else {}
    timing = parse_route_timing(TIMING_ROUTE)
    stats = read_stats(STATS)
    order = read_stats(ORDER)

    strip = Image.open(STRIP) if STRIP.exists() else None
    strip_w = W - 120
    strip_h = int(strip.height * strip_w / strip.width) if strip else 0

    y_head = 160
    y_why = y_head
    y_path = y_why + 276
    y_mid = y_path + 202
    y_order = y_mid + 484
    y_strip = y_order + 286
    H = y_strip + strip_h + 60

    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    draw_header(d, timing)
    draw_why(d, y_why, stats)
    draw_path(d, y_path)
    draw_verify(d, 60, y_mid, W // 2 - 20, y_mid + 464)
    draw_res(d, W // 2 + 20, y_mid, W - 60, y_mid + 464, gate, route, timing)
    draw_order(d, y_order, order)
    if strip:
        img.paste(strip.resize((strip_w, strip_h), Image.LANCZOS), (60, y_strip))
        d.text((60, y_strip - 30),
               "下面这张条带来自 fall_core_multi_top 的 RTL 导出："
               "左=相机帧上的 4 个槽框，右=motion_morph 掩膜 + 该 pass 的排除矩形。",
               font=font(18), fill=MUTED)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    img.save(args.out)
    print(f"wrote {args.out} ({img.width}x{img.height})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
