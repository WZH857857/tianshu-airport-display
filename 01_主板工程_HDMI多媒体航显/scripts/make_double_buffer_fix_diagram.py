from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "doc" / "double_buffer_fix_preview.png"


def font(size: int) -> ImageFont.FreeTypeFont:
    for candidate in (
        Path(r"C:\Windows\Fonts\msyh.ttc"),
        Path(r"C:\Windows\Fonts\msyhbd.ttc"),
    ):
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size)
    return ImageFont.load_default()


old = Image.open(ROOT / "sdcard" / "IMG02.BMP").convert("RGB")
new = Image.open(ROOT / "sdcard" / "IMG03.BMP").convert("RGB")

panel_w, panel_h = 640, 480
margin = 42
label_h = 62
title_h = 74
footer_h = 62
canvas = Image.new(
    "RGB",
    (margin * 3 + panel_w * 2, title_h + label_h + panel_h + footer_h + margin * 2),
    "#172033",
)
draw = ImageDraw.Draw(canvas)

title = "切图瞬间对比：旧版边写边显示，修正版整帧切换"
draw.text((margin, 24), title, font=font(30), fill="#f4f7fb")

left_x = margin
right_x = margin * 2 + panel_w
image_y = margin + title_h + label_h

# At this point in the old transfer, the bottom 38% has reached SDRAM.
split = int(panel_h * 0.62)
partial = old.copy()
partial.paste(new.crop((0, split, panel_w, panel_h)), (0, split))

canvas.paste(partial, (left_x, image_y))
canvas.paste(new, (right_x, image_y))

draw.rectangle(
    [left_x, image_y, left_x + panel_w - 1, image_y + panel_h - 1],
    outline="#e7edf7",
    width=2,
)
draw.rectangle(
    [right_x, image_y, right_x + panel_w - 1, image_y + panel_h - 1],
    outline="#e7edf7",
    width=2,
)

draw.text(
    (left_x, margin + title_h),
    "旧位流：第二帧误写到地址 0，屏幕扫到哪里就覆盖到哪里",
    font=font(22),
    fill="#ffb4a8",
)
draw.text(
    (right_x, margin + title_h),
    "修正版：第二帧写入地址 1，显示地址 0 完整不变",
    font=font(22),
    fill="#9fe3bd",
)

draw.line(
    [(left_x + 20, image_y + split), (left_x + panel_w - 20, image_y + split)],
    fill="#ffcc33",
    width=5,
)
draw.text(
    (left_x + 24, image_y + split + 12),
    "已写入的新图",
    font=font(24),
    fill="#ffe08a",
)
draw.text(
    (left_x + 24, image_y + 16),
    "仍在显示的旧图",
    font=font(24),
    fill="#ffffff",
)

draw.text(
    (left_x, image_y + panel_h + 16),
    "旧代码：~0 = 3，地址 2/3 实际接到 0",
    font=font(21),
    fill="#ffb4a8",
)
draw.text(
    (right_x, image_y + panel_h + 16),
    "修正：0/1 两个真实双缓冲地址",
    font=font(21),
    fill="#9fe3bd",
)

OUT.parent.mkdir(parents=True, exist_ok=True)
canvas.save(OUT, quality=95)
print(OUT)
