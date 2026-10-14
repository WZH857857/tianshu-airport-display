# -*- coding: utf-8 -*-
"""Manual fire emergency switch for the HDMI board.

The normal trigger is automatic: the E104 console reports a flame alarm over
Bluetooth and the launcher mirrors it to the board with command 0x23.  This
script talks to the board directly, so the emergency screen, the buzzer and
the eight digit display can be demonstrated without the E104 board.

    python host/emg_test.py --state on
    python host/emg_test.py --state off
    python host/emg_test.py --state on --port COM6

Without --port the first CH340 / USB serial port is used.
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except Exception as exc:  # pragma: no cover - depends on the local install
    raise SystemExit("pyserial is required: %s" % exc)


HDR1 = 0xA5
HDR2 = 0x5A
CMD_PING = 0x01
CMD_EMG = 0x23
R_PING = 0x81
R_EMG = 0xA3
R_ERROR = 0xE0


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    body = bytes((cmd & 0xFF, len(payload) & 0xFF)) + bytes(payload)
    checksum = 0
    for byte in body:
        checksum ^= byte
    return bytes((HDR1, HDR2)) + body + bytes((checksum,))


def read_frame(port: "serial.Serial", timeout: float = 1.5):
    """Return (cmd, payload) of the next valid frame, or None."""
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = port.read(64)
        if chunk:
            buf.extend(chunk)
        while True:
            if len(buf) < 4:
                break
            if buf[0] != HDR1 or buf[1] != HDR2:
                del buf[0]
                continue
            length = buf[3]
            total = 5 + length
            if len(buf) < total:
                break
            raw = bytes(buf[:total])
            del buf[:total]
            checksum = 0
            for byte in raw[2:4 + length]:
                checksum ^= byte
            if checksum == raw[4 + length]:
                return raw[2], raw[4:4 + length]
    return None


def pick_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    ports = list(list_ports.comports())
    if not ports:
        raise SystemExit("no serial port found; connect the board first")
    for info in ports:
        desc = (getattr(info, "description", "") or "").lower()
        if "ch340" in desc or "usb-serial" in desc or "usb serial" in desc:
            return info.device
    return ports[0].device


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state", choices=("on", "off"), default="on",
                        help="on = fire emergency, off = back to normal")
    parser.add_argument("--port", default=None, help="main board COM port")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    port_name = pick_port(args.port)
    fire = 1 if args.state == "on" else 0

    with serial.Serial(port_name, args.baud, timeout=0.2) as port:
        time.sleep(0.2)
        port.reset_input_buffer()

        port.write(build_frame(CMD_PING))
        reply = read_frame(port)
        if reply is None:
            print("no reply on %s; is the board powered and is the port right?"
                  % port_name)
            return 2
        cmd, payload = reply
        if cmd == R_PING and payload:
            print("board firmware 0x%02X on %s" % (payload[0], port_name))
            if payload[0] < 0x1B:
                print("this firmware has no fire emergency command (needs 0x1B)")
                return 3

        port.write(build_frame(CMD_EMG, bytes((fire,))))
        reply = read_frame(port)
        if reply is None:
            print("the board did not answer the emergency command")
            return 4
        cmd, payload = reply
        if cmd == R_EMG:
            state = payload[0] if payload else fire
            print("fire emergency = %s" % ("ON" if state else "OFF"))
            return 0
        if cmd == R_ERROR:
            print("board rejected the command (error %d)"
                  % (payload[1] if len(payload) > 1 else -1))
            return 5
        print("unexpected reply 0x%02X %s" % (cmd, payload.hex()))
        return 6


if __name__ == "__main__":
    sys.exit(main())
