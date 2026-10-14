"""Render the three-stage fault LED timing diagram.

This is documentation only.  It shows how to split one repeating LED cycle
into the three values emitted by the current bring-up bitstream.
"""

import os

from PIL import Image, ImageDraw, ImageFont


W, H = 1600, 1120
BG = (24, 26, 30)
PANEL = (38, 41, 47)
TEXT = (235, 237, 242)
MUTED = (166, 171, 181)
ON = (68, 214, 126)
OFF = (67, 71, 80)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(HERE, "..", "doc", "fault_led_cycle.png"))
FONT_PATH = r"C:\Windows\Fonts\msyh.ttc"


def font(size):
    try:
        return ImageFont.truetype(FONT_PATH, size)
    except OSError:
        return ImageFont.load_default()


F_TITLE = font(46)
F_HEAD = font(30)
F_BODY = font(25)
F_SMALL = font(21)
F_MONO = font(24)


def centered(draw, box, text, fnt, fill=TEXT):
    x0, y0, x1, y1 = box
    left, top, right, bottom = draw.textbbox((0, 0), text, font=fnt)
    draw.text(
        (x0 + (x1 - x0 - (right - left)) / 2 - left,
         y0 + (y1 - y0 - (bottom - top)) / 2 - top),
        text,
        font=fnt,
        fill=fill,
    )


def led_row(draw, x, y, bits, labels=("LED4", "LED3", "LED2", "LED1")):
    for i, bit in enumerate(bits):
        cx = x + i * 92
        draw.ellipse([cx - 24, y - 24, cx + 24, y + 24],
                     fill=ON if bit else OFF,
                     outline=(208, 213, 222) if bit else (105, 109, 118),
                     width=3)
        centered(draw, (cx - 40, y + 35, cx + 40, y + 70), labels[i], F_SMALL, MUTED)


def main():
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)

    d.text((70, 50), "当前故障灯轮显：每次按三组分别记录", font=F_TITLE, fill=TEXT)
    d.text((72, 112),
           "只有看到“四灯亮 0.5 s -> 全灭 0.5 s -> 四灯再亮 0.5 s”的双闪，才说明下载的是当前位流。",
           font=F_BODY, fill=MUTED)
    d.text((72, 158),
           "故障发生后，四灯按下面的三组顺序循环；不要只截取某一瞬间的 0110。",
           font=F_BODY, fill=MUTED)

    # Startup marker
    d.rounded_rectangle([70, 215, 1530, 285], radius=12,
                        fill=(52, 56, 64), outline=(93, 99, 111), width=2)
    centered(d, (70, 215, 1530, 285),
             "启动标记：四灯亮 0.5 s  ->  全灭 0.5 s  ->  四灯再亮 0.5 s",
             F_HEAD)

    rows = [
        ("第 1 组", "扇区错误码", "显示 2.0 s", (1, 0, 0, 1),
         "常见：0100 包装层超时 / 1000 CMD17 被拒 / 1001 数据令牌超时"),
        ("第 2 组", "SD 控制器状态", "显示 2.0 s", (1, 0, 0, 0),
         "按 SD 状态表判读；例如 0110 才表示当时真的停在 S_CMD16"),
        ("第 3 组", "文件系统错误码", "显示 2.0 s", (0, 1, 0, 0),
         "通用扇区故障路径固定为 0100，用来确认错误路径"),
    ]
    top = 320
    for idx, (group, name, duration, bits, note) in enumerate(rows):
        y0 = top + idx * 205
        d.rounded_rectangle([70, y0, 1530, y0 + 145], radius=14,
                            fill=PANEL, outline=(84, 89, 100), width=2)
        d.text((105, y0 + 24), group, font=F_HEAD, fill=TEXT)
        d.text((105, y0 + 72), duration, font=F_BODY, fill=MUTED)
        led_row(d, 410, y0 + 72, bits)
        d.text((790, y0 + 28), name, font=F_HEAD, fill=TEXT)
        d.text((790, y0 + 78), note, font=F_SMALL, fill=MUTED)
        if idx != len(rows) - 1:
            cx = 800
            cy = y0 + 150
            d.line([cx, cy, cx, cy + 42], fill=(111, 117, 130), width=4)
            d.polygon([(cx, cy + 52), (cx - 9, cy + 37), (cx + 9, cy + 37)],
                      fill=(111, 117, 130))

    end_y = top + 3 * 205 - 20
    d.text((800, end_y),
           "此后全灭 3.5 s，再回到第 1 组重复",
           font=F_BODY, fill=MUTED)

    # Reading key
    d.rounded_rectangle([70, 995, 1530, 1090], radius=14,
                        fill=(31, 33, 38), outline=(78, 83, 94), width=2)
    d.text((105, 1015),
           "位序固定为 LED4 LED3 LED2 LED1。其他错误码：1010 数据长度不符 / 1011 CMD17 响应超时 / 1111 事务不在忙态。",
           font=F_BODY, fill=TEXT)
    d.text((105, 1055),
           "三组之间都有 0.5 s 全灭间隔；若没有双闪，先重新下载当前位流。",
           font=F_SMALL, fill=MUTED)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    img.save(OUT, format="PNG", optimize=True)
    print("wrote", OUT, os.path.getsize(OUT), "bytes")


if __name__ == "__main__":
    main()
