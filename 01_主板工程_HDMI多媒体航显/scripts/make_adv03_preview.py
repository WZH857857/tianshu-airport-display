"""Build the ADV03 preview sheet from the real RTL scaling chain.

The script does not draw a mock up: it resamples a real test picture to two
non display sizes, streams the resulting 24 bit BMP files through
sim/tb_bmp_scale_dump.v (which instantiates rtl/storage/bmp_stream_parser.v and
rtl/storage/bmp_scaler.v unchanged) and then compares every dumped pixel with
the nearest neighbour mapping the RTL is supposed to implement.  Only when the
comparison is exact does it write doc/adv03_scale_preview.png.

run:
    <python with Pillow> scripts/make_adv03_preview.py
"""

import os
import struct
import subprocess
import sys

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
SIM = os.path.join(PROJ, "sim")
DOC = os.path.join(PROJ, "doc")
BUILD = os.path.join(SIM, "build_adv03")
SDCARD = os.path.join(PROJ, "sdcard")

IVERILOG = r"C:\iverilog\bin\iverilog.exe"
VVP = r"C:\iverilog\bin\vvp.exe"

DW, DH = 640, 480


def run(cmd, **kw):
    print("+ " + " ".join(cmd))
    subprocess.run(cmd, check=True, **kw)


def compile_dump():
    os.makedirs(BUILD, exist_ok=True)
    vvp_path = os.path.join(BUILD, "tb_bmp_scale_dump.vvp")
    run([
        IVERILOG, "-g2005", "-s", "tb_bmp_scale_dump", "-o", vvp_path,
        os.path.join(SIM, "tb_bmp_scale_dump.v"),
        os.path.join(PROJ, "rtl", "storage", "bmp_stream_parser.v"),
        os.path.join(PROJ, "rtl", "storage", "bmp_scaler.v"),
    ])
    return vvp_path


def make_source(path, size):
    """one of the real airport test pictures, resampled to `size`"""
    base = Image.open(os.path.join(SDCARD, "IMG03.BMP")).convert("RGB")
    if size is None:
        img = base
    else:
        img = base.resize(size, Image.LANCZOS)
    img.save(path)
    validate_bmp(path)
    return img


def validate_bmp(path):
    """the parser only accepts 24 bit, uncompressed, bottom up files"""
    with open(path, "rb") as fh:
        head = fh.read(54)
    magic, = struct.unpack_from("<H", head, 0)
    data_off, = struct.unpack_from("<I", head, 10)
    dib, = struct.unpack_from("<I", head, 14)
    width, = struct.unpack_from("<i", head, 18)
    height, = struct.unpack_from("<i", head, 22)
    planes, = struct.unpack_from("<H", head, 26)
    bpp, = struct.unpack_from("<H", head, 28)
    compression, = struct.unpack_from("<I", head, 30)
    row = (3 * width + 3) // 4 * 4
    size = os.path.getsize(path)
    assert magic == 0x4D42, "not a BMP"
    assert dib >= 40, "DIB header %d" % dib
    assert planes == 1 and bpp == 24 and compression == 0, \
        "planes=%d bpp=%d compression=%d" % (planes, bpp, compression)
    assert height > 0, "BMP is top down (height %d)" % height
    assert size == data_off + row * height, \
        "%s: %d bytes, header implies %d" % (path, size, data_off + row * height)


def dump_chain(vvp_path, bmp_path, rgb_path):
    run([
        VVP, vvp_path,
        "+bmp=" + bmp_path,
        "+out=" + rgb_path,
        "+size=%d" % os.path.getsize(bmp_path),
    ])


def load_dump(rgb_path):
    with open(rgb_path, "rb") as fh:
        raw = fh.read()
    assert len(raw) == DW * DH * 3, "dump is %d bytes" % len(raw)
    img = Image.frombytes("RGB", (DW, DH), raw)
    return img.transpose(Image.FLIP_TOP_BOTTOM)      # BMP row order, bottom first


def source_pixels(img):
    """the same pixels the parser sees, in file row order (bottom row first)"""
    return list(img.transpose(Image.FLIP_TOP_BOTTOM).getdata())


def nearest_reference(img, dst_w=DW, dst_h=DH):
    src = source_pixels(img)
    w, h = img.size
    out = []
    for y in range(dst_h):
        sy = (y * h) // dst_h
        row = sy * w
        for x in range(dst_w):
            out.append(src[row + (x * w) // dst_w])
    return out


def check(name, src_img, out_img, exact_identity=False):
    got = source_pixels(out_img)
    want = nearest_reference(src_img)
    if got != want:
        bad = sum(1 for a, b in zip(got, want) if a != b)
        raise SystemExit("FAIL %s: %d of %d pixels differ" % (name, bad, len(want)))
    if exact_identity and src_img.size == (DW, DH):
        if source_pixels(src_img) != got:
            raise SystemExit("FAIL %s: identity mapping is not pixel exact" % name)
    print("  %-24s %d x %d -> %d x %d : %d pixels match the nearest neighbour mapping"
          % (name, src_img.size[0], src_img.size[1], DW, DH, len(got)))


def build_sheet(cases, out_path):
    pad = 24
    label_h = 34
    col_w = DW
    thumb_w = 320
    height = pad + sum(label_h + DH + pad for _ in cases)
    sheet = Image.new("RGB", (pad + thumb_w + pad + col_w + pad, height), "white")
    draw = ImageDraw.Draw(sheet)
    y = pad
    for title, src_img, out_img in cases:
        draw.text((pad, y + 8), title, fill="black")
        y += label_h
        thumb = src_img.copy()
        thumb.thumbnail((thumb_w, DH))
        sheet.paste(thumb, (pad, y))
        sheet.paste(out_img, (pad + thumb_w + pad, y))
        y += DH + pad
    sheet.save(out_path)
    print("wrote", out_path)


def main():
    os.makedirs(BUILD, exist_ok=True)
    vvp_path = compile_dump()

    cases = []

    # 640 x 480: has to come out of the chain completely untouched
    src = make_source(os.path.join(BUILD, "src_640x480.bmp"), None)
    rgb = os.path.join(BUILD, "dump_640x480.rgb")
    dump_chain(vvp_path, os.path.join(BUILD, "src_640x480.bmp"), rgb)
    out = load_dump(rgb)
    check("identity 640x480", src, out, exact_identity=True)

    # small picture: upscaled on both axes
    src = make_source(os.path.join(BUILD, "src_320x240.bmp"), (320, 240))
    rgb = os.path.join(BUILD, "dump_320x240.rgb")
    dump_chain(vvp_path, os.path.join(BUILD, "src_320x240.bmp"), rgb)
    out_up = load_dump(rgb)
    check("upscale 320x240", src, out_up)
    cases.append(("320 x 240  ->  640 x 480   (up)", src, out_up))

    # large picture: downscaled on both axes
    src = make_source(os.path.join(BUILD, "src_960x720.bmp"), (960, 720))
    rgb = os.path.join(BUILD, "dump_960x720.rgb")
    dump_chain(vvp_path, os.path.join(BUILD, "src_960x720.bmp"), rgb)
    out_dn = load_dump(rgb)
    check("downscale 960x720", src, out_dn)
    cases.append(("960 x 720  ->  640 x 480   (down)", src, out_dn))

    # odd size, exercises the row padding path
    src = make_source(os.path.join(BUILD, "src_321x241.bmp"), (321, 241))
    rgb = os.path.join(BUILD, "dump_321x241.rgb")
    dump_chain(vvp_path, os.path.join(BUILD, "src_321x241.bmp"), rgb)
    out_odd = load_dump(rgb)
    check("odd 321x241", src, out_odd)

    build_sheet(cases, os.path.join(DOC, "adv03_scale_preview.png"))
    print("RESULT: PASS make_adv03_preview")


if __name__ == "__main__":
    sys.exit(main())
