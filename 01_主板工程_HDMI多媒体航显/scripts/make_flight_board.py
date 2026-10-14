# Generate the 640 x 480 airport flight-information board used by the player.
#
# The player stores every picture as a 640 x 480 frame, so drawing the board
# at that exact size keeps the text pixel-sharp instead of going through the
# scaler.  The BMP is written as 24-bit BI_RGB, matching the other IMG0x.BMP
# files on the card.
from PIL import Image, ImageDraw, ImageFont

W, H = 640, 480
OUT_BMP = r"E:\FPGA guo sai\jing jie yao qiu\hdmi_mm_player_advanced\sdcard\adv03_demo\IMG07.BMP"
OUT_PNG = r"E:\FPGA guo sai\jing jie yao qiu\hdmi_mm_player_advanced\doc\adv13_flight_board_preview.png"

FONT_CN = r"C:\Windows\Fonts\msyh.ttc"
FONT_CN_B = r"C:\Windows\Fonts\msyhbd.ttc"
FONT_MONO = r"C:\Windows\Fonts\consolab.ttf"

NAVY_TOP = (8, 24, 46)
NAVY_BOT = (14, 42, 78)
HEAD_BG = (18, 52, 94)
ROW_A = (12, 34, 60)
ROW_B = (16, 42, 72)
LINE = (38, 76, 124)
TITLE = (238, 244, 252)
ACCENT = (94, 200, 255)
MUTED = (150, 178, 210)
GREEN = (86, 222, 150)
YELLOW = (255, 202, 74)
RED = (255, 112, 112)
WHITE = (236, 242, 250)

FLIGHTS = [
    ("CA1501", "北京首都", "PEK", "08:30", "12", "BOARDING", GREEN),
    ("MU5102", "上海虹桥", "SHA", "09:15", "07", "ON TIME", WHITE),
    ("CZ3305", "广州白云", "CAN", "10:40", "03", "DELAYED", YELLOW),
    ("HU7801", "深圳宝安", "SZX", "11:05", "15", "BOARDING", GREEN),
    ("3U8633", "成都天府", "TFU", "11:40", "09", "ON TIME", WHITE),
    ("MF8102", "厦门高崎", "XMN", "12:20", "21", "CANCELLED", RED),
]


def font(path, size):
    return ImageFont.truetype(path, size)


img = Image.new("RGB", (W, H), NAVY_TOP)
draw = ImageDraw.Draw(img)

# background gradient
for y in range(H):
    t = y / (H - 1)
    rgb = tuple(int(NAVY_TOP[i] + (NAVY_BOT[i] - NAVY_TOP[i]) * t) for i in range(3))
    draw.line([(0, y), (W, y)], fill=rgb)

f_title_cn = font(FONT_CN_B, 21)
f_title_en = font(FONT_MONO, 13)
f_head = font(FONT_CN_B, 13)
f_flight = font(FONT_MONO, 17)
f_cn = font(FONT_CN, 16)
f_code = font(FONT_MONO, 12)
f_time = font(FONT_MONO, 17)
f_status = font(FONT_MONO, 14)
f_note_cn = font(FONT_CN, 13)
f_note_en = font(FONT_MONO, 11)

# title bar
draw.rectangle([0, 0, W, 52], fill=(6, 20, 40))
draw.rectangle([0, 52, W, 54], fill=ACCENT)
draw.text((16, 14), "航班信息", font=f_title_cn, fill=TITLE)
draw.text((112, 21), "FLIGHT  INFORMATION", font=f_title_en, fill=ACCENT)
draw.text((470, 18), "T2  14:35", font=f_title_en, fill=MUTED)

# table header
draw.rectangle([0, 55, W, 83], fill=HEAD_BG)
draw.text((16, 63), "航班 FLIGHT", font=f_head, fill=MUTED)
draw.text((150, 63), "目的地 DESTINATION", font=f_head, fill=MUTED)
draw.text((340, 63), "时间 TIME", font=f_head, fill=MUTED)
draw.text((440, 63), "登机口", font=f_head, fill=MUTED)
draw.text((524, 63), "状态 STATUS", font=f_head, fill=MUTED)
draw.line([(0, 83), (W, 83)], fill=LINE)

# data rows
TOP = 84
ROW_H = 52
for i, (flt, city, code, tm, gate, status, color) in enumerate(FLIGHTS):
    y0 = TOP + i * ROW_H
    draw.rectangle([0, y0, W, y0 + ROW_H - 1], fill=(ROW_A if i % 2 == 0 else ROW_B))
    draw.text((16, y0 + 15), flt, font=f_flight, fill=WHITE)
    draw.text((150, y0 + 13), city, font=f_cn, fill=TITLE)
    draw.text((250, y0 + 19), code, font=f_code, fill=MUTED)
    draw.text((340, y0 + 15), tm, font=f_time, fill=WHITE)
    draw.text((456, y0 + 15), gate, font=f_time, fill=ACCENT)
    draw.text((524, y0 + 17), status, font=f_status, fill=color)
    draw.line([(0, y0 + ROW_H - 1), (W, y0 + ROW_H - 1)], fill=LINE)

# footer band
FOOT = TOP + len(FLIGHTS) * ROW_H
draw.rectangle([0, FOOT, W, H], fill=(6, 20, 40))
draw.rectangle([0, FOOT, W, FOOT + 2], fill=ACCENT)
draw.text((16, FOOT + 16), "请提前 30 分钟到达登机口", font=f_note_cn, fill=TITLE)
draw.text((16, FOOT + 38), "PLEASE ARRIVE AT THE GATE 30 MIN BEFORE DEPARTURE",
          font=f_note_en, fill=MUTED)
draw.text((470, FOOT + 16), "ANU2026", font=f_note_en, fill=ACCENT)

img.save(OUT_BMP, "BMP")
img.save(OUT_PNG, "PNG")
print("wrote", OUT_BMP)
print("wrote", OUT_PNG)
