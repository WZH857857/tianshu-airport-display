#!/usr/bin/env python3
"""Dump the board's 115200 status lines and summarise what came back.

    python fpga/tools/serial_capture.py --port COM5 --seconds 12 \
           --out out/serial_pnr15_live.txt

The board prints four S lines per frame, one per slot:

    S<slot> F<frame:4> <found:1> <x:3> <y:3> <w:3> <h:3> <area:5> <state:1>
           <mean:2><shutter:3><gain:2><zone:1>

In the PBOX diagnostic bitstream the fields carry motion_hist's publish
instead of the tracker box: area is the sweep's trail cell count and state is
{published, pass saw a cell, sweep read a cell}.  The summary below reports
the field meanings it can see, so the same command serves both builds.
"""

import argparse
import collections
import re
import time

import serial

LINE = re.compile(
    r"^S([0-3]) F([0-9A-F]{4}) ([01]) ([0-9A-F]{3}) ([0-9A-F]{3}) "
    r"([0-9A-F]{3}) ([0-9A-F]{3}) ([0-9A-F]{5}) ([0-7])"
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--out", default="out/serial_live.txt")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    found = collections.Counter()
    states = collections.Counter()
    cells = collections.Counter()
    frames = set()
    slots = collections.Counter()
    total = 0
    boxed = 0

    t0 = time.time()
    with open(args.out, "w", encoding="ascii", errors="replace") as fh:
        while time.time() - t0 < args.seconds:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").rstrip("\r\n")
            if not line:
                continue
            fh.write(line + "\n")
            fh.flush()
            total += 1
            m = LINE.match(line)
            if not m:
                continue
            slot, frame, ok, x, y, w, h, area, state = m.groups()
            slot = int(slot)
            frames.add(frame)
            slots[slot] += 1
            states[int(state, 16)] += 1
            cells[int(area, 16)] += 1
            if ok == "1":
                found[slot] += 1
                boxed += 1
    ser.close()

    print(f"{total} lines -> {args.out}")
    print(f"frames seen: {len(frames)}  S lines with found=1: {boxed}")
    print("found per slot:", dict(sorted(found.items())))
    print("lines per slot:", dict(sorted(slots.items())))
    print("state digits:", dict(sorted(states.items())))
    top_cells = cells.most_common(5)
    print("area/cells values:", top_cells)


if __name__ == "__main__":
    main()
