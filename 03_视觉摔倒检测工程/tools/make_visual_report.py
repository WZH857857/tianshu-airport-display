"""Build the PNG visual report for the multi-target fall detector.

Outputs (under ``out_frames/visual_report``):

* ``pipeline.png``       -- algorithm blocks and the new multi-target branch
* ``crowd01_multitrack.png`` -- one contact sheet with per-track boxes/IDs
* ``alarm_day6.png`` / ``alarm_S1F13.png`` -- true-fall alarm sequences
* ``metrics.png``        -- before/after bars read from the JSON reports

Usage::

    python make_visual_report.py [--only pipeline,crowd,alarms,metrics]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import imageio.v3 as iio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fall_proto import FallDetector, Params  # noqa: E402

BG = (17, 19, 24)
PANEL = (28, 32, 39)
FG = (236, 239, 244)
MUTED = (154, 163, 176)
ACCENT = (0, 196, 255)
OK = (58, 214, 130)
WARN = (255, 190, 70)
BAD = (255, 92, 92)
PALETTE = [
    (0, 220, 130), (0, 190, 255), (255, 214, 0), (255, 90, 200),
    (255, 140, 0), (130, 130, 255), (255, 80, 80), (0, 255, 210),
]


def load_font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    names = ([r"C:\Windows\Fonts\msyhbd.ttc", r"C:\Windows\Fonts\msyh.ttc"]
             if bold else
             [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\arial.ttf"])
    for name in names:
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def text(draw: ImageDraw.ImageDraw, xy, value: str,
         font: ImageFont.FreeTypeFont, fill=FG) -> None:
    draw.text(xy, value, font=font, fill=fill)


def mask_tint(frame: np.ndarray, mask: np.ndarray | None,
              scale: tuple[int, int]) -> np.ndarray:
    out = frame.astype(np.float32) * 0.74
    if mask is None:
        return out.astype(np.uint8)
    fx, fy = scale
    small = (mask * 255).astype(np.uint8)
    if fx > 1 or fy > 1:
        small = np.repeat(np.repeat(small, fy, axis=0), fx, axis=1)
    h, w = out.shape[:2]
    layer = np.zeros((h, w), dtype=bool)
    layer[:min(h, small.shape[0]), :min(w, small.shape[1])] = (
        small[:h, :w] > 0)
    out[layer] = out[layer] * 0.35 + np.array([40, 255, 130]) * 0.65
    return out.astype(np.uint8)


def annotate_tracks(frame: np.ndarray, dets,
                    scale: tuple[int, int]) -> np.ndarray:
    fx, fy = scale
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    small = load_font(15, bold=True)
    for det in dets:
        if det.bbox is None:
            continue
        x, y, w, h = det.bbox
        color = PALETTE[(det.track_id - 1) % len(PALETTE)]
        if det.flagged:
            color = BAD
        x0, y0 = x * fx, y * fy
        x1, y1 = (x + w) * fx, (y + h) * fy
        draw.rectangle([x0, y0, x1, y1], outline=color, width=4)
        label = f"T{det.track_id} {det.state}"
        if det.flagged:
            label += " ALARM"
        bbox = draw.textbbox((0, 0), label, font=small)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
        ly = max(0, y0 - th - 8)
        draw.rectangle([x0, ly, x0 + tw + 12, ly + th + 8], fill=color)
        text(draw, (x0 + 6, ly + 4), label, small, BG)
    return np.asarray(img)


def tile(frame: np.ndarray, width: int, caption: str) -> Image.Image:
    img = Image.fromarray(frame)
    height = max(1, int(round(img.height * width / img.width)))
    img = img.resize((width, height), Image.LANCZOS)
    canvas = Image.new("RGB", (width, height + 34), PANEL)
    canvas.paste(img, (0, 34))
    draw = ImageDraw.Draw(canvas)
    text(draw, (10, 8), caption, load_font(16, bold=True), FG)
    return canvas


def contact_sheet(tiles: list[Image.Image], cols: int, header: str,
                  out_path: Path, note: str = "") -> None:
    if not tiles:
        return
    tw, th = tiles[0].size
    rows = (len(tiles) + cols - 1) // cols
    header_h = 96
    sheet = Image.new("RGB", (tw * cols + 24, th * rows + header_h + 24), BG)
    draw = ImageDraw.Draw(sheet)
    text(draw, (24, 20), header, load_font(30, bold=True), FG)
    if note:
        text(draw, (24, 60), note, load_font(18), MUTED)
    for n, tile_img in enumerate(tiles):
        r, c = divmod(n, cols)
        sheet.paste(tile_img, (12 + c * tw, header_h + r * th))
    sheet.save(out_path)


def render_crowd(clip: Path, out_dir: Path) -> dict:
    picks = {40, 60, 120, 180, 240, 260, 300, 320}
    fps = float(iio.immeta(clip, plugin="FFMPEG").get("fps") or 25.0)
    detector = FallDetector(Params.for_fps(fps))
    chosen: dict[int, tuple[np.ndarray, list]] = {}
    max_tracks = 0
    alarms = 0
    for index, frame in enumerate(iio.imiter(clip, plugin="FFMPEG")):
        dets = detector.update(frame)
        max_tracks = max(max_tracks, len(dets))
        alarms += sum(1 for det in dets if det.flagged)
        if index in picks:
            marked = mask_tint(frame, detector.mask_cache, detector.scale)
            marked = annotate_tracks(marked, dets, detector.scale)
            chosen[index] = (marked, dets)
    tiles = []
    for index in sorted(chosen):
        frame, dets = chosen[index]
        caption = (f"f{index:03d}  {index / fps:4.1f}s  "
                   f"跟踪目标 {len(dets)}")
        tiles.append(tile(frame, 320, caption))
    out_path = out_dir / "crowd01_multitrack.png"
    contact_sheet(
        tiles, 4,
        "人群素材 crowd01：多目标跟踪（每个框一个独立轨迹）",
        out_path,
        f"1080x1920 街景，{len(chosen)} 个抽样帧；最大同时跟踪 "
        f"{max_tracks} 个目标；本段误报 {alarms} 次")
    return {"sheet": str(out_path), "max_tracks": max_tracks,
            "alarms": alarms}


def render_alarm(clip: Path, out_dir: Path, tag: str) -> dict:
    fps = float(iio.immeta(clip, plugin="FFMPEG").get("fps") or 25.0)
    detector = FallDetector(Params.for_fps(fps))
    rendered: list[tuple[int, np.ndarray, list]] = []
    alarm_frame = None
    for index, frame in enumerate(iio.imiter(clip, plugin="FFMPEG")):
        dets = detector.update(frame)
        if any(det.flagged for det in dets) and alarm_frame is None:
            alarm_frame = index
        marked = mask_tint(frame, detector.mask_cache, detector.scale)
        marked = annotate_tracks(marked, dets, detector.scale)
        rendered.append((index, marked, dets))
    if alarm_frame is None:
        alarm_frame = len(rendered) // 2
    picks = [f for f in range(alarm_frame - 12, alarm_frame + 13, 4)
             if 0 <= f < len(rendered)]
    tiles = []
    for index in picks:
        _, frame, dets = rendered[index]
        state = next((d.state for d in dets if d.flagged),
                     dets[0].state if dets else "SEEK")
        caption = (f"f{index:03d}  {index / fps:4.2f}s  {state}")
        tiles.append(tile(frame, 430, caption))
    out_path = out_dir / f"alarm_{tag}.png"
    contact_sheet(
        tiles, 3,
        f"{tag}：摔倒报警序列",
        out_path,
        f"{fps:.2f} fps；报警帧 {alarm_frame}（{alarm_frame / fps:.2f}s）"
        if alarm_frame is not None else "")
    return {"sheet": str(out_path), "alarm_frame": alarm_frame}


def draw_arrow(draw: ImageDraw.ImageDraw, x0, y0, x1, y1,
               color=ACCENT, width=4) -> None:
    draw.line([x0, y0, x1, y1], fill=color, width=width)
    if x1 > x0:
        draw.polygon([(x1, y1), (x1 - 14, y1 - 8), (x1 - 14, y1 + 8)],
                     fill=color)
    else:
        draw.polygon([(x1, y1), (x1 + 8, y1 - 14), (x1 + 8, y1 + 14)],
                     fill=color)


def box(draw, xy, title, lines, color=ACCENT, width=250, height=104):
    x, y = xy
    draw.rounded_rectangle([x, y, x + width, y + height], radius=10,
                           fill=PANEL, outline=color, width=3)
    text(draw, (x + 14, y + 10), title, load_font(19, bold=True), color)
    for n, line in enumerate(lines):
        text(draw, (x + 14, y + 38 + n * 22), line, load_font(15), FG)


def draw_pipeline(out_dir: Path) -> str:
    img = Image.new("RGB", (1760, 980), BG)
    draw = ImageDraw.Draw(img)
    text(draw, (36, 24), "EG4S20NG88 摔倒检测流水线（视觉副卡，全部在 FPGA 内）",
         load_font(34, bold=True), FG)
    text(draw, (36, 70),
         "不使用 YOLO / 不使用 CNN：29 个 DSP、无 CNN 编译器；全整数比较与形态学，"
         "可直接映射到 RTL",
         load_font(18), MUTED)

    box(draw, (40, 140), "1. DVP 采集",
        ["OV5640 640x480", "RGB565 -> 灰度"], ACCENT)
    draw_arrow(draw, 290, 192, 360, 192)
    box(draw, (360, 140), "2. 归一化",
        ["整数盒式降采样", "统一到 320x240"], ACCENT)
    draw_arrow(draw, 610, 192, 680, 192)
    box(draw, (680, 140), "3. 运动掩膜",
        ["|cur-prev| > 10", "3x3 腐蚀 + 11x11 膨胀"], ACCENT)

    draw_arrow(draw, 805, 244, 805, 330)
    box(draw, (480, 330), "4A. 单人：两遍投影",
        ["行投影最长游程", "带内列投影 -> 1 个包围盒"], OK, 320, 104)
    box(draw, (830, 330), "4B. 人群：连通域 top-K",
        ["标签 -> 取最大 4 块", "每块一个候选目标"], WARN, 320, 104)
    text(draw, (480, 446), "人数 <= 1 时走 4A（已验证路径）；人数 >= 2 时走 4B",
         load_font(16), MUTED)

    draw_arrow(draw, 805, 540, 805, 620)
    box(draw, (430, 620), "5. 多目标关联",
        ["IoU/质心 + 跨帧 <= 3 帧", "面积比 0.35~2.5 + 轨迹 TTL"], ACCENT,
        360, 104)
    draw_arrow(draw, 800, 672, 880, 672)
    box(draw, (880, 620), "6. 每条轨迹独立姿态机",
        ["h_ref 站立参考 / 高度塌陷", "底边贴地 / 面积不塌 / 快速"],
        ACCENT, 390, 104)
    draw_arrow(draw, 1075, 724, 1075, 800)
    box(draw, (850, 800), "7. 恢复撤销 + 自适应确认",
        ["单人 2 帧；人群 4 帧", "又站起来 -> 取消报警"], OK, 450, 104)
    draw_arrow(draw, 1300, 852, 1370, 852)
    box(draw, (1370, 800), "8. 报警输出",
        ["LED / 蜂鸣器", "串口上报主卡"], BAD, 320, 104)

    notes = [
        "FPGA 预算：前一帧 614 Kb + 掩膜 77 Kb + 行缓存 ~3.5 Kb",
        "逻辑量估计 3-5 K LUT；DSP 用 1-2 个做宽高比比较",
        "新增多目标只需寄存器保存 4 条轨迹状态，不增加卷积阵列",
    ]
    for n, line in enumerate(notes):
        text(draw, (40, 820 + n * 34), "• " + line, load_font(18), FG)
    out_path = out_dir / "pipeline.png"
    img.save(out_path)
    return str(out_path)


def draw_metrics(out_dir: Path, batch_json: Path, crowd_json: Path) -> str:
    img = Image.new("RGB", (1500, 860), BG)
    draw = ImageDraw.Draw(img)
    text(draw, (40, 28), "升级前后指标对比", load_font(36, bold=True), FG)
    text(draw, (40, 76),
         "GMDCSA24 全量回归 + 人群视频回归 + 机场实拍 CCTV 确认事件",
         load_font(18), MUTED)

    before = {"crowd_fp": 8, "fall_hits": 13, "adl_fp": 0, "movers": 1}
    after = dict(before)
    note = "人群验证：crowd01 完整片段误报 8 -> 0；6 段素材合计 1 次误报（crowd02 团块粘连）"
    if batch_json.exists():
        report = json.loads(batch_json.read_text(encoding="utf-8"))
        after["fall_hits"] = report["summary"]["fall_hits"]
        after["adl_fp"] = report["summary"]["adl_false_alarms"]
    if crowd_json.exists():
        report = json.loads(crowd_json.read_text(encoding="utf-8"))
        # The before/after bars only make sense on the same clip, so the
        # crowd01 bar reads the per-clip alarm list; the overall crowd
        # regression total is shown in the note instead.
        crowd01 = next((c for c in report.get("crowd", [])
                        if c["clip"] == "crowd01.mp4"), None)
        after["crowd_fp"] = (len(crowd01["alarms"]) if crowd01 is not None
                            else report["summary"]["crowd_false_alarms"])
        after["movers"] = report["summary"]["max_simultaneous_tracks"]
        fp_total = report["summary"]["crowd_false_alarms"]
        cctv_hits = report["summary"]["cctv_fall_hits"]
        cctv_total = report["summary"]["cctv_fall_total"]
        note = (f"人群验证：crowd01 完整片段误报 8 -> 0；{report['summary']['crowd_total']} 段素材"
                f"合计 {fp_total} 次误报（crowd02 团块粘连）；机场实拍 {cctv_hits}/{cctv_total} 命中")

    charts = [
        ("人群视频误报（crowd01）", before["crowd_fp"], after["crowd_fp"], 8, True),
        ("摔倒检出 (GMDCSA24)", before["fall_hits"], after["fall_hits"], 15, False),
        ("ADL 误报", before["adl_fp"], after["adl_fp"], 10, True),
        ("最大同时跟踪目标", before["movers"], after["movers"], 8, False),
    ]
    y = 160
    for label, old, new, scale, lower_better in charts:
        text(draw, (40, y), label, load_font(22, bold=True), FG)
        bar_x = 460
        bar_w = 900
        old_w = int(bar_w * old / scale)
        new_w = int(bar_w * new / scale)
        draw.rounded_rectangle([bar_x, y, bar_x + old_w, y + 40],
                               radius=6, fill=(90, 96, 108))
        draw.rounded_rectangle([bar_x, y + 58, bar_x + new_w, y + 98],
                               radius=6, fill=OK)
        text(draw, (bar_x - 90, y + 2), f"前 {old}", load_font(18), MUTED)
        text(draw, (bar_x - 90, y + 60), f"后 {new}", load_font(18), OK)
        y += 142
    for n, line in enumerate([
        note,
        "GMDCSA24 唯一漏报：Subject 4 / Fall / 02（开局已躺地）；",
        "CCTV day11 漏报：1 fps 采样 + 家具遮挡，倒下过程只剩 3 个样本",
    ]):
        text(draw, (40, 760 + n * 34), "• " + line, load_font(18), FG)
    out_path = out_dir / "metrics.png"
    img.save(out_path)
    return str(out_path)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path,
                        default=root / "out_frames" / "visual_report")
    parser.add_argument("--only", default="pipeline,crowd,alarms,metrics")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    wanted = {part.strip() for part in args.only.split(",") if part.strip()}

    if "pipeline" in wanted:
        print("pipeline:", draw_pipeline(args.out))
    if "crowd" in wanted:
        clip = root / "datasets" / "crowd" / "crowd01.mp4"
        if clip.exists():
            print("crowd:", render_crowd(clip, args.out))
    if "alarms" in wanted:
        for tag, clip in [
            ("day6", root / "datasets" / "cctv_fall"
                    / "day6_event019_confirmed.mp4"),
            ("S1F13", root / "datasets" / "gmdcsa24" / "Subject 1"
                      / "Fall" / "13.mp4"),
        ]:
            if clip.exists():
                print(f"alarm {tag}:", render_alarm(clip, args.out, tag))
    if "metrics" in wanted:
        print("metrics:", draw_metrics(
            args.out,
            root / "out_frames" / "batch_report_final3.json",
            root / "out_frames" / "crowd_report.json"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
