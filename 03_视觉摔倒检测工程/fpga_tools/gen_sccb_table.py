#!/usr/bin/env python3
"""Emit the OV5640 640x480 RGB565 register table as a Verilog ``case`` block.

The register values are the public OV5640 application-note sequence that the
ALIENTEK/OpenEdv driver uses for 640x480 RGB565 (250 writes, PCLK 48 MHz with
a 24 MHz XVCLK).  Only the *values* are reused; the Verilog emitted here is
our own encoding for ``rtl/ov5640_sccb_cfg.v``.

Usage:
    python gen_sccb_table.py <reference_cfg.v> [out.vh]
    python gen_sccb_table.py <reference_cfg.v> --inject <rtl/ov5640_sccb_cfg.v>

The output is pasted into ``ov5640_sccb_cfg.v`` (kept out of the TD read_hdl
file list so the vendor flow never has to resolve an include path).

``--inject`` replaces the block between the OV5640_TABLE_BEGIN/END markers in
the RTL with the generated case arms.  That path is a bulk mechanical rewrite
of a generated constant table, not a hand edit.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REG_NUM = 250

# Runtime parameters of the reference driver, resolved to this project's mode.
RESOLVED = {
    "cmos_h_pixel": 640,
    "cmos_v_pixel": 480,
    "total_h_pixel": 1856,   # HTS = 0x0740
    "total_v_pixel": 984,    # VTS = 0x03D8
}

CASE_RE = re.compile(
    r"8'd(\d+)\s*:\s*i2c_data\s*<=\s*\{\s*([^;]+?)\}\s*;", re.S
)


def parse_rows(src: str) -> dict[int, tuple[int, int]]:
    body = src[src.find("case(init_reg_cnt)"):]
    rows: dict[int, tuple[int, int]] = {}
    for match in CASE_RE.finditer(body):
        idx = int(match.group(1))
        rhs = " ".join(match.group(2).split())
        lits = re.findall(r"(\d+)'h([0-9a-fA-F]+)", rhs)
        if len(lits) == 3:                       # {8'h30, 8'h17, 8'hff}
            addr = (int(lits[0][1], 16) << 8) | int(lits[1][1], 16)
            rows[idx] = (addr, int(lits[2][1], 16))
            continue
        if len(lits) == 2:                       # {16'h3808, 8'h02}
            addr = int(lits[0][1], 16)
            rows[idx] = (addr, int(lits[1][1], 16))
            continue
        if len(lits) == 1:                       # {16'h3808, <parameter>}
            addr = int(lits[0][1], 16)
            rows[idx] = (addr, resolve_data(idx, rhs))
            continue
        raise ValueError(f"index {idx}: cannot parse {rhs!r}")
    return rows


def resolve_data(idx: int, raw: str) -> int:
    """Resolve the eight entries that the reference feeds from parameters."""
    literal = re.fullmatch(r"8'h([0-9a-fA-F]{1,2})", raw)
    if literal:
        return int(literal.group(1), 16)
    for name, value in RESOLVED.items():
        if name in raw:
            if "[7:0]" in raw:
                return value & 0xFF
            if "[12:8]" in raw:
                return (value >> 8) & 0x1F
            if "[11:8]" in raw:
                return (value >> 8) & 0x0F
            if "[10:8]" in raw:
                return (value >> 8) & 0x07
    raise ValueError(f"index {idx}: cannot resolve {raw!r}")


def emit(rows: dict[int, tuple[int, int]]) -> str:
    missing = [i for i in range(REG_NUM) if i not in rows]
    if missing:
        raise ValueError(f"missing indices: {missing}")
    out = []
    for idx in range(REG_NUM):
        addr, data = rows[idx]
        out.append(
            "                8'd%-3d: begin tbl_addr = 16'h%04X; tbl_data = 8'h%02X; end"
            % (idx, addr, data)
        )
    return "\n".join(out) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("reference", type=Path)
    ap.add_argument("output", type=Path, nargs="?")
    ap.add_argument(
        "--inject",
        type=Path,
        help="rewrite the marker block inside this RTL file instead of writing output",
    )
    args = ap.parse_args()

    text = emit(parse_rows(args.reference.read_text(encoding="utf-8", errors="replace")))
    if args.inject:
        rtl = args.inject.read_text(encoding="ascii").splitlines()
        begin = [i for i, l in enumerate(rtl) if "OV5640_TABLE_BEGIN" in l]
        end = [i for i, l in enumerate(rtl) if "OV5640_TABLE_END" in l]
        if len(begin) != 1 or len(end) != 1 or end[0] <= begin[0]:
            raise SystemExit(f"{args.inject}: table markers not found/ambiguous")
        new = rtl[: begin[0] + 1] + text.strip("\n").splitlines() + rtl[end[0]:]
        args.inject.write_text("\n".join(new) + "\n", encoding="ascii")
        print(f"injected {len(text.splitlines())} entries into {args.inject}")
        return 0
    if args.output:
        args.output.write_text(text, encoding="ascii")
        print(f"wrote {args.output} ({len(text.splitlines())} entries)")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
