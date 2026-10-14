#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
airport_console.py - host side console for the EG4S20 HDMI player

The board brings the CH340 USB-UART bridge out on the same Type-C socket that
powers it.  This program can control the FPGA over that link or through a
paired Bluetooth SPP COM port.  Both paths use exactly the same byte protocol.
The FPGA stays the only processor: it owns the picture pipeline, the fade
engine, the audio stream and the glyph ROM.  Everything here is a remote
control.

Wire format, both directions:

    A5 5A | cmd | len | payload[len] | xor

The checksum is the exclusive or of cmd, len and every payload byte.  One
request is answered by exactly one reply, so the worker sends a frame and then
waits for its answer before the next one goes out.

    cmd                                reply
    01 PING      -                     81 ver
    02 TEXT      ascii, 0..32 bytes    82 eff_len text_len   (len 0 = built in)
    03 SLOT      slot 0..5             83 slot | E0 03 code
    04 MODE      0 manual / 1 auto /   84 mode
                 2 hand back to SW1
    05 PARAM     sel 0 bright, 1       85 sel val
                 contrast, val 0..7
    06 OSD       layout 0..3           86 layout
    07 BAUD      0..4                  87 index, sent at the new rate
    10 STATUS    -                     90 version slot total mode bright
                                           contrast osd cache textlen flags
    20 BOARD     row 0..5 + 40 bytes    A0 row
    21 BVIS      0 off / 1 on            A1 enable
    22 TIME      hh mm ss (BCD)          A2 hh mm ss

Error replies are E0 <cmd> <code> with the codes documented in
rtl/uart/uart_ctrl.v.

The glyph ROM in rtl/ui_overlay.v only carries space, - . / 0-9 : and A-Z, so
the text field folds to upper case and drops anything else instead of putting
a blank onto the screen.
"""

from __future__ import annotations

import argparse
import os
import queue
import sys
import time

from PyQt5.QtCore import (
    QEvent,
    QObject,
    QPointF,
    QRectF,
    QSize,
    Qt,
    QThread,
    QTimer,
    QUrl,
    pyqtSignal,
)
from PyQt5.QtGui import (
    QColor,
    QDesktopServices,
    QFont,
    QFontMetrics,
    QIcon,
    QImageReader,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
    QTextCursor,
)
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QStackedWidget,
    QStyle,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

try:
    import serial
    from serial.tools import list_ports
except ImportError:                                     # pragma: no cover
    serial = None
    list_ports = None


# ---------------------------------------------------------------------------
# protocol
# ---------------------------------------------------------------------------

HDR1 = 0xA5
HDR2 = 0x5A

CMD_PING = 0x01
CMD_TEXT = 0x02
CMD_SLOT = 0x03
CMD_MODE = 0x04
CMD_PARAM = 0x05
CMD_OSD = 0x06
CMD_BAUD = 0x07
CMD_STATUS = 0x10
CMD_BOARD = 0x20
CMD_BVIS = 0x21
CMD_TIME = 0x22

R_PING = 0x81
R_TEXT = 0x82
R_SLOT = 0x83
R_MODE = 0x84
R_PARAM = 0x85
R_OSD = 0x86
R_BAUD = 0x87
R_STATUS = 0x90
R_BOARD = 0xA0
R_BVIS = 0xA1
R_TIME = 0xA2
R_ERROR = 0xE0

ERR_TEXT = {
    1: "length out of range",
    2: "slot out of range",
    3: "slot not cached yet",
    4: "unknown command",
    5: "value out of range",
}


def image_dimensions(path: str) -> QSize:
    """Read the source dimensions without decoding the whole bitmap."""
    if not path or not os.path.exists(path):
        return QSize()
    size = QImageReader(path).size()
    return size if size.isValid() else QSize()


def dimension_text(size: QSize) -> str:
    if size.width() <= 0 or size.height() <= 0:
        return "--"
    return "%d×%d" % (size.width(), size.height())

BAUD_RATES = [115200, 230400, 460800, 921600, 1000000]
FALLBACK_RATE = 115200

PORT_KIND_USB = "usb"
PORT_KIND_BLUETOOTH = "bt"
PORT_KIND_SERIAL = "serial"

PORT_KIND_LABELS = {
    PORT_KIND_USB: "USB",
    PORT_KIND_BLUETOOTH: "蓝牙",
    PORT_KIND_SERIAL: "串口",
}

BLUETOOTH_PORT_TOKENS = (
    "BLUETOOTH",
    "BTHENUM",
    "BTHMODEM",
    "BLUETOOTH LINK",
    "SPP",
    "蓝牙",
)
USB_PORT_TOKENS = (
    "CH340",
    "CH341",
    "USB-SERIAL",
    "USB SERIAL",
    "CP210",
    "FTDI",
    "PROLIFIC",
    "SILICON LABS",
    "USB 串行",
)


def classify_serial_port(port_info) -> str:
    """Return the likely transport behind one pyserial port entry."""
    fields = " ".join(
        str(getattr(port_info, name, "") or "").upper()
        for name in ("device", "description", "manufacturer", "hwid")
    )
    if any(token in fields for token in BLUETOOTH_PORT_TOKENS):
        return PORT_KIND_BLUETOOTH
    if any(token in fields for token in USB_PORT_TOKENS):
        return PORT_KIND_USB
    return PORT_KIND_SERIAL


def port_kind_label(kind: str) -> str:
    return PORT_KIND_LABELS.get(kind, PORT_KIND_LABELS[PORT_KIND_SERIAL])

TEXT_MAX = 32
BOARD_ROWS = 6
BOARD_COLS = 40

# the glyph ROM can only draw these
SUPPORTED_CHARS = frozenset(" -./0123456789:ABCDEFGHIJKLMNOPQRSTUVWXYZ")

SLOT_COUNT = 6

# The FPGA keeps one row per 40-character frame.  A row is deliberately
# formatted like a conventional FIDS line so the operator can edit the same
# fixed columns that a real airport departure board uses.
def board_record(flight: str, airline: str, dest: str, std: str, etd: str,
                 gate: str, status: str) -> str:
    # column map used by the FPGA board and by the ruler under the panel:
    #   flight 0-5 | air 7-9 | dest 11-13 | std 15-19 |
    #   etd 21-25 | gate 27-29 | status 31-38
    text = "%-6s %-3s %-3s %-5s %-5s %-3s %-8s" % (
        flight, airline, dest, std, etd, gate, status,
    )
    return text[:BOARD_COLS].ljust(BOARD_COLS)


BOARD_DEFAULT = [
    board_record("FLIGHT", "AIR", "DEST", "STD", "ETD", "GATE", "STATUS"),
    board_record("CA1832", "CA", "PEK", "08:30", "08:45", "A12", "BOARDING"),
    board_record("MU5102", "MU", "SZX", "09:15", "09:15", "B07", "ON TIME"),
    board_record("CZ3305", "CZ", "CAN", "10:40", "11:00", "C03", "DELAYED"),
    board_record("HU7801", "HU", "CTU", "11:05", "11:05", "D15", "FINAL"),
    board_record("MF8127", "MF", "XIY", "11:40", "12:10", "E08", "CANCEL"),
]

# One page is a full six-row screen.  Every picture slot owns one page, so the
# host can show the six screens side by side and light the one on air.
BOARD_PAGES: list[tuple[str, list[str]]] = [
    ("IMG01 · 早高峰离港", BOARD_DEFAULT),
    ("IMG02 · 午高峰离港", [
        board_record("FLIGHT", "AIR", "DEST", "STD", "ETD", "GATE", "STATUS"),
        board_record("MU2157", "MU", "KMG", "12:20", "12:20", "A05", "ON TIME"),
        board_record("CA1943", "CA", "HGH", "12:55", "13:40", "B12", "DELAYED"),
        board_record("CZ6402", "CZ", "WUH", "13:10", "13:10", "C07", "BOARDING"),
        board_record("3U8881", "3U", "CKG", "13:45", "13:45", "D02", "ON TIME"),
        board_record("FM9203", "FM", "TSN", "14:05", "14:05", "E11", "FINAL"),
    ]),
    ("IMG03 · 进港到达", [
        board_record("FLIGHT", "AIR", "DEST", "STD", "ETD", "GATE", "STATUS"),
        board_record("MU5101", "MU", "SHA", "ARR", "14:20", "A12", "ARRIVED"),
        board_record("CA9831", "CA", "PEK", "ARR", "14:35", "B07", "ARRIVED"),
        board_record("CZ3512", "CZ", "CAN", "ARR", "15:05", "C03", "DELAYED"),
        board_record("HU7082", "HU", "HAK", "ARR", "15:30", "D15", "CANCEL"),
        board_record("ZH9817", "ZH", "SZX", "ARR", "15:50", "E08", "ON TIME"),
    ]),
    ("IMG04 · 中转联程", [
        board_record("FLIGHT", "AIR", "DEST", "STD", "ETD", "GATE", "STATUS"),
        board_record("CA1501", "CA", "PEK", "15:20", "15:20", "A03", "ON TIME"),
        board_record("MU2318", "MU", "SHA", "15:45", "16:10", "B11", "DELAYED"),
        board_record("CZ3876", "CZ", "CAN", "16:05", "16:05", "C06", "BOARDING"),
        board_record("HU7605", "HU", "SZX", "16:30", "16:30", "D09", "ON TIME"),
        board_record("3U8756", "3U", "CTU", "16:55", "16:55", "E04", "FINAL"),
    ]),
    ("IMG05 · 延误取消", [
        board_record("FLIGHT", "AIR", "DEST", "STD", "ETD", "GATE", "STATUS"),
        board_record("CA4318", "CA", "CTU", "17:10", "18:25", "A12", "DELAYED"),
        board_record("MU2931", "MU", "KMG", "17:35", "19:05", "B08", "DELAYED"),
        board_record("CZ6210", "CZ", "WUH", "17:50", "17:50", "C02", "CANCEL"),
        board_record("FM9316", "FM", "TSN", "18:15", "18:15", "D14", "ON TIME"),
        board_record("ZH9552", "ZH", "HAK", "18:40", "18:40", "E07", "BOARDING"),
    ]),
    ("IMG06 · 国际航班", [
        board_record("FLIGHT", "AIR", "DEST", "STD", "ETD", "GATE", "STATUS"),
        board_record("CA981", "CA", "JFK", "19:05", "19:05", "A01", "CHECK IN"),
        board_record("MU587", "MU", "LAX", "19:30", "19:30", "B03", "ON TIME"),
        board_record("CZ327", "CZ", "NRT", "19:55", "20:20", "C09", "DELAYED"),
        board_record("HU7907", "HU", "CDG", "20:15", "20:15", "D05", "BOARDING"),
        board_record("3U3891", "3U", "SGN", "20:40", "20:40", "E12", "FINAL"),
    ]),
]
ROTATE_MS = 10_000

# One page per picture: IMG01 shows page 0, IMG02 page 1, and so on.
PAGE_FOR_SLOT = (0, 1, 2, 3, 4, 5)

# the four overlay layouts the board understands
OSD_NAMES = ["FULL", "INFO", "ON", "OFF"]

# read only facts about the target, shown on the console for reference
BOARD_FACTS = [
    ("器件", "EG4S20BG256"),
    ("输出", "640x480 59.3Hz"),
    ("串口", "CH340 / 蓝牙 SPP"),
    ("音频", "HDMI 44.1kHz"),
    ("协议", "A5 5A v18 + BOARD 6x40 + TIME"),
]


def bcd_to_int(value: int) -> int:
    """Decode one packed BCD byte, e.g. 0x23 -> 23."""
    return (value >> 4) * 10 + (value & 0x0F)


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    """A5 5A cmd len payload xor, checksum over cmd, len and payload."""
    body = bytes([cmd & 0xFF, len(payload) & 0xFF]) + bytes(payload)
    chk = 0
    for b in body:
        chk ^= b
    return bytes([HDR1, HDR2]) + body + bytes([chk])


def fmt_frame(frame: bytes) -> str:
    return " ".join("%02X" % b for b in frame)


def sanitize_text(text: str) -> tuple[str, int]:
    """Fold to the glyph ROM's character set and clamp to the frame budget."""
    out = []
    dropped = 0
    for ch in text:
        up = ch.upper()
        if up in SUPPORTED_CHARS:
            out.append(up)
        else:
            dropped += 1
    return "".join(out[:TEXT_MAX]), dropped


def sanitize_board_row(text: str) -> tuple[str, int]:
    """Fold one board row to the shared 8x8 glyph ROM and pad it to 40."""
    out = []
    dropped = 0
    for ch in text:
        up = ch.upper()
        if up in SUPPORTED_CHARS:
            out.append(up)
        else:
            dropped += 1
    return ("".join(out[:BOARD_COLS]).ljust(BOARD_COLS)[:BOARD_COLS], dropped)


class FrameParser:
    """Byte stream in, (cmd, payload) pairs out, resynchronising on A5 5A."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buf += data
        frames: list[tuple[int, bytes]] = []
        while True:
            while len(self.buf) >= 2 and not (
                self.buf[0] == HDR1 and self.buf[1] == HDR2
            ):
                self.buf.pop(0)
            if len(self.buf) < 4:
                break
            ln = self.buf[3]
            total = 5 + ln
            if len(self.buf) < total:
                break
            raw = bytes(self.buf[:total])
            del self.buf[:total]
            chk = 0
            for b in raw[2 : 4 + ln]:
                chk ^= b
            if chk == raw[4 + ln]:
                frames.append((raw[2], raw[4 : 4 + ln]))
            else:
                self.bad += 1
        return frames


# ---------------------------------------------------------------------------
# serial worker
# ---------------------------------------------------------------------------


class SerialWorker(QThread):
    """Owns the port.  Sends one frame, waits for its reply, repeats."""

    opened = pyqtSignal(bool, str)
    link = pyqtSignal(bool)
    log = pyqtSignal(str, str)
    got_status = pyqtSignal(object)
    got_reply = pyqtSignal(int, object)
    rate_changed = pyqtSignal(int)

    POLL_INTERVAL = 1.0
    REPLY_TIMEOUT = 0.60
    TIME_SYNC_INTERVAL = 1.0

    def __init__(self, port: str, baud: int, parent=None) -> None:
        super().__init__(parent)
        self.port_name = port
        self.baud = baud
        self.outbox: "queue.Queue[tuple[int, bytes]]" = queue.Queue()
        self._stop = False
        self._send_now = False

    # -- api used from the GUI thread ------------------------------------
    def send(self, cmd: int, payload: bytes = b"") -> None:
        # BOARD is the one command with a 41-byte payload (row + 40 glyphs);
        # every other command keeps the original 32-byte text budget.
        limit = BOARD_COLS + 1 if cmd == CMD_BOARD else TEXT_MAX
        self.outbox.put((cmd, bytes(payload[:limit])))
        self._send_now = True

    def stop(self) -> None:
        self._stop = True

    # -- internals --------------------------------------------------------
    def _emit_tx(self, frame: bytes) -> None:
        self.log.emit("tx", fmt_frame(frame))

    def _handle(self, cmd: int, payload: bytes) -> None:
        self.log.emit("rx", "%02X  %s" % (cmd, payload.hex(" ").upper()))
        self.got_reply.emit(cmd, payload)
        if cmd == R_STATUS and len(payload) >= 10:
            self.got_status.emit(
                {
                    "ver": payload[0],
                    "slot": payload[1],
                    "total": payload[2],
                    "mode": payload[3] & 1,
                    "bright": payload[4] & 7,
                    "contrast": payload[5] & 7,
                    "osd": payload[6] & 3,
                    "cache": payload[7] & 0x3F,
                    "textlen": payload[8],
                    "link_seen": payload[9] & 1,
                    "card_ok": (payload[9] >> 1) & 1,
                }
            )
        elif cmd == R_ERROR and len(payload) >= 2:
            self.log.emit(
                "err",
                "board refused cmd %02X: %s"
                % (payload[0], ERR_TEXT.get(payload[1], "code %d" % payload[1])),
            )

    def run(self) -> None:                              # noqa: C901
        if serial is None:
            self.opened.emit(False, "pyserial is not installed")
            return

        try:
            port = serial.Serial(
                self.port_name, self.baud, timeout=0, write_timeout=0.5
            )
        except Exception as exc:                        # noqa: BLE001
            self.opened.emit(False, str(exc))
            return

        self.opened.emit(True, self.port_name)
        parser = FrameParser()
        pending = None
        next_poll = time.monotonic() + 0.05
        next_time_sync = time.monotonic()
        last_tick = time.monotonic()

        try:
            while not self._stop:
                # ---- collect the answer ---------------------------------
                try:
                    chunk = port.read(4096)
                except Exception as exc:                # noqa: BLE001
                    self.log.emit("err", "read failed: %s" % exc)
                    break

                if chunk:
                    bad_before = parser.bad
                    for cmd, payload in parser.feed(chunk):
                        self._handle(cmd, payload)
                        if pending is not None:
                            pending = None
                            self.link.emit(True)
                    if parser.bad != bad_before:
                        self.log.emit(
                            "err",
                            "dropped a reply with a bad checksum, check the "
                            "bitstream is the ADV13 build",
                        )
                    last_tick = time.monotonic()

                now = time.monotonic()

                # ---- retire a request that never answered ----------------
                if pending is not None and now > pending["deadline"]:
                    if pending["baud_back"] is not None:
                        # the board did not come back at the new rate
                        try:
                            port.baudrate = pending["baud_back"]
                        except Exception:               # noqa: BLE001
                            pass
                        self.rate_changed.emit(pending["baud_back"])
                        self.log.emit(
                            "err",
                            "no answer at %d baud, back to %d"
                            % (pending["baud_new"], pending["baud_back"]),
                        )
                    else:
                        self.log.emit(
                            "err",
                            "timeout waiting for %02X reply"
                            % (pending["expect"] or 0),
                        )
                        self.link.emit(False)
                    pending = None
                    next_poll = now + self.POLL_INTERVAL

                # ---- launch the next request ----------------------------
                if pending is None:
                    cmd = payload = None
                    if now >= next_time_sync:
                        lt = time.localtime()
                        cmd = CMD_TIME
                        payload = bytes([
                            ((lt.tm_hour // 10) << 4) | (lt.tm_hour % 10),
                            ((lt.tm_min // 10) << 4) | (lt.tm_min % 10),
                            ((lt.tm_sec // 10) << 4) | (lt.tm_sec % 10),
                        ])
                        next_time_sync = now + self.TIME_SYNC_INTERVAL
                    else:
                        try:
                            cmd, payload = self.outbox.get_nowait()
                        except queue.Empty:
                            pass

                    if cmd is None and now >= next_poll:
                        cmd, payload = CMD_STATUS, b""
                        next_poll = now + self.POLL_INTERVAL

                    if cmd is not None:
                        frame = build_frame(cmd, payload)
                        try:
                            port.write(frame)
                        except Exception as exc:        # noqa: BLE001
                            self.log.emit("err", "write failed: %s" % exc)
                            break
                        self._emit_tx(frame)

                        back = new = None
                        if cmd == CMD_BAUD and payload:
                            idx = payload[0]
                            if 0 <= idx < len(BAUD_RATES):
                                back = port.baudrate
                                new = BAUD_RATES[idx]
                                # the reply to BAUD leaves the board at the
                                # new rate, so switch here and verify with a
                                # STATUS frame instead of chasing that byte
                                time.sleep(0.05)
                                try:
                                    port.baudrate = new
                                    port.reset_input_buffer()
                                except Exception:       # noqa: BLE001
                                    pass
                                self.rate_changed.emit(new)
                                time.sleep(0.02)
                                probe = build_frame(CMD_STATUS, b"")
                                port.write(probe)
                                self._emit_tx(probe)
                                parser = FrameParser()

                        pending = {
                            "expect": cmd,
                            "deadline": time.monotonic() + self.REPLY_TIMEOUT,
                            "baud_back": back,
                            "baud_new": new,
                        }

                self._send_now = False
                self.msleep(4)
        finally:
            try:
                port.close()
            except Exception:                           # noqa: BLE001
                pass
            self.opened.emit(False, self.port_name)


# ---------------------------------------------------------------------------
# small widgets
# ---------------------------------------------------------------------------


def panel(title: str) -> tuple[QFrame, QVBoxLayout]:
    """A titled surface.  Cards are for real tools, so the radius stays at 8."""
    box = QFrame()
    box.setObjectName("panel")
    lay = QVBoxLayout(box)
    lay.setContentsMargins(16, 14, 16, 16)
    lay.setSpacing(12)
    if title:
        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)
        bar = QFrame()
        bar.setObjectName("accent")
        bar.setFixedSize(3, 13)
        head = QLabel(title)
        head.setObjectName("panelTitle")
        row.addWidget(bar)
        row.addWidget(head)
        row.addStretch(1)
        lay.addLayout(row)
    return box, lay


class Segmented(QWidget):
    """Row of exclusive buttons that reads as one control."""

    changed = pyqtSignal(int)

    def __init__(self, labels: list[str], current: int = 0, parent=None) -> None:
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)
        self.group = QButtonGroup(self)
        self.group.setExclusive(True)
        for idx, text in enumerate(labels):
            btn = QPushButton(text)
            btn.setObjectName("seg")
            btn.setCheckable(True)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setChecked(idx == current)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self.group.addButton(btn, idx)
            lay.addWidget(btn)
        self.group.idClicked.connect(self.changed.emit)

    def set_current(self, idx: int) -> None:
        btn = self.group.button(idx)
        if btn is not None and not btn.isChecked():
            btn.setChecked(True)


class MiniBoard(QFrame):
    """One slot's six rows on a small screen; lit while that slot is on air."""

    clicked = pyqtSignal(int)

    def __init__(self, index: int, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.index = index
        self.setObjectName("miniBoard")
        self.setCursor(Qt.PointingHandCursor)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 4, 6, 4)
        lay.setSpacing(2)

        self.head = QLabel()
        self.head.setObjectName("miniHead")
        lay.addWidget(self.head)

        self.body = QLabel()
        self.body.setObjectName("miniBody")
        font = QFont("Consolas")
        font.setStyleHint(QFont.Monospace)
        font.setPixelSize(13)
        self.body.setFont(font)
        self.body.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.body.setTextInteractionFlags(Qt.NoTextInteraction)
        lay.addWidget(self.body, 1)

        self.set_rows(BOARD_PAGES[index][1])
        self.set_live(False)

    def set_rows(self, rows: list[str]) -> None:
        self.body.setText("\n".join(rows))

    def set_live(self, live: bool) -> None:
        self.setProperty("live", "true" if live else "false")
        name = BOARD_PAGES[self.index][0]
        self.head.setText(("播放中 · " if live else "") + name)
        self.style().unpolish(self)
        self.style().polish(self)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self.clicked.emit(self.index)
        super().mousePressEvent(event)


class Slider(QWidget):
    """Label, track and a fixed width read-out so the row never shifts."""

    changed = pyqtSignal(int)

    def __init__(self, name: str, top: int, value: int, parent=None) -> None:
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)

        head = QHBoxLayout()
        head.setContentsMargins(0, 0, 0, 0)
        lbl = QLabel(name)
        lbl.setObjectName("rowLabel")
        self.value_lbl = QLabel(str(value))
        self.value_lbl.setObjectName("readout")
        self.value_lbl.setAlignment(Qt.AlignCenter)
        self.value_lbl.setFixedWidth(34)
        head.addWidget(lbl)
        head.addStretch(1)
        head.addWidget(self.value_lbl)
        lay.addLayout(head)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, top)
        self.slider.setValue(value)
        self.slider.setPageStep(1)
        self.slider.setSingleStep(1)
        self.slider.valueChanged.connect(self._on_change)
        lay.addWidget(self.slider)

    def _on_change(self, value: int) -> None:
        self.value_lbl.setText(str(value))
        self.changed.emit(value)

    def set_value(self, value: int) -> None:
        blocked = self.slider.blockSignals(True)
        self.slider.setValue(value)
        self.slider.blockSignals(blocked)
        self.value_lbl.setText(str(value))


class SlotCard(QFrame):
    """One picture slot: thumbnail, file name and a cache badge."""

    clicked = pyqtSignal(int)
    ICON = QSize(124, 93)

    def __init__(self, index: int, image_path: str, parent=None) -> None:
        super().__init__(parent)
        self.index = index
        self.image_dim = image_dimensions(image_path)
        self.cached = True
        self.setObjectName("slot")
        self.setCursor(Qt.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setFixedHeight(self.ICON.height() + 30)
        self.setProperty("selected", False)
        self.setProperty("cached", True)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 3)
        lay.setSpacing(3)

        self.thumb = QLabel()
        self.thumb.setObjectName("thumb")
        self.thumb.setFixedSize(self.ICON)
        self.thumb.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.thumb, 0, Qt.AlignHCenter)

        self.badge = QLabel("播放中", self.thumb)
        self.badge.setObjectName("slotBadge")
        self.badge.setAlignment(Qt.AlignCenter)
        self.badge.adjustSize()
        self.badge.move(self.ICON.width() - self.badge.width() - 7, 7)
        self.badge.hide()

        self.caption = QLabel()
        self.caption.setObjectName("slotCaption")
        self.caption.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.caption)

        self.selected = False
        self._bright = self._render(image_path, dim=False)
        self._dim = self._render(image_path, dim=True)
        self.set_cached(True)

    # -- painting helpers --------------------------------------------------
    def _placeholder(self) -> QPixmap:
        pix = QPixmap(self.ICON)
        pix.fill(QColor("#E9EFF6"))
        painter = QPainter(pix)
        painter.setPen(QColor("#738396"))
        painter.setFont(QFont("Consolas", 22, QFont.Bold))
        painter.drawText(pix.rect(), Qt.AlignCenter, "%02d" % (self.index + 1))
        painter.end()
        return pix

    def _render(self, path: str, dim: bool) -> QPixmap:
        canvas = QPixmap(self.ICON)
        canvas.fill(QColor("#E9EFF6"))
        src = QPixmap()
        if path and os.path.exists(path):
            src = QPixmap(path)
        if src.isNull():
            src = self._placeholder()
        else:
            src = src.scaled(
                self.ICON, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
        painter = QPainter(canvas)
        painter.drawPixmap(
            (self.ICON.width() - src.width()) // 2,
            (self.ICON.height() - src.height()) // 2,
            src,
        )
        if dim:
            painter.setCompositionMode(QPainter.CompositionMode_SourceOver)
            painter.fillRect(canvas.rect(), QColor(245, 248, 252, 170))
        painter.end()
        return canvas

    # -- state -------------------------------------------------------------
    def set_cached(self, cached: bool) -> None:
        self.cached = cached
        self.thumb.setPixmap(self._bright if cached else self._dim)
        mark = "IMG%02d" % (self.index + 1)
        caption = mark
        if self.image_dim.isValid():
            caption += "  " + dimension_text(self.image_dim)
        self.caption.setText(caption)
        if not cached:
            self.badge.setText("未缓存")
        else:
            self.badge.setText("播放中")
        self.badge.adjustSize()
        self.badge.move(self.ICON.width() - self.badge.width() - 7, 7)
        self.setProperty("cached", "true" if cached else "false")
        self.badge.setVisible(not cached or self.selected)
        self._repolish()
        detail = "画面槽位 %d" % (self.index + 1)
        if self.image_dim.isValid():
            detail += " · %s" % dimension_text(self.image_dim)
        if not cached:
            detail += " · 尚未进入帧缓存"
        self.setToolTip(detail)

    def set_selected(self, on: bool) -> None:
        self.selected = on
        self.setProperty("selected", "true" if on else "false")
        self.badge.setVisible(not self.cached or on)
        self._repolish()

    def _repolish(self) -> None:
        self.style().unpolish(self)
        self.style().polish(self)
        for child in (self.caption, self.thumb, self.badge):
            child.style().unpolish(child)
            child.style().polish(child)

    def mousePressEvent(self, event) -> None:           # noqa: N802
        if self.cached:
            self.clicked.emit(self.index)
        event.accept()


class AutoHideBar(QObject):
    """Keep a scroll area's vertical bar hidden until it is actually needed.

    A permanently visible groove along an otherwise full panel reads as an
    empty column, so the bar only appears when the content really overflows.
    """

    def __init__(self, area: QScrollArea, parent=None) -> None:
        super().__init__(parent)
        self._area = area
        area.verticalScrollBar().rangeChanged.connect(self._apply)

    def _apply(self, _lo: int, hi: int) -> None:
        want = Qt.ScrollBarAsNeeded if hi > 0 else Qt.ScrollBarAlwaysOff
        if self._area.verticalScrollBarPolicy() != want:
            self._area.setVerticalScrollBarPolicy(want)


class LinkPill(QLabel):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("pill")
        self.setAlignment(Qt.AlignCenter)
        self.setFixedHeight(30)
        self.setMinimumWidth(150)
        self.set_state("offline", "未连接")

    def set_state(self, state: str, text: str) -> None:
        self.setProperty("state", state)
        self.setText("●  %s" % text)
        self.style().unpolish(self)
        self.style().polish(self)


PLANE_OUTLINE = (
    (0.00, -0.95), (0.10, -0.60), (0.13, -0.18),
    (0.88, 0.48), (0.88, 0.62), (0.13, 0.26),
    (0.13, 0.60), (0.36, 0.82), (0.36, 0.92),
    (0.07, 0.78), (0.07, 0.96), (-0.07, 0.96),
    (-0.07, 0.78), (-0.36, 0.92), (-0.36, 0.82),
    (-0.13, 0.60), (-0.13, 0.26), (-0.88, 0.62),
    (-0.88, 0.48), (-0.13, -0.18), (-0.10, -0.60),
)


def brand_pixmap(size: int = 40) -> QPixmap:
    """An airliner climbing over a runway: the airport display mark."""
    pix = QPixmap(size, size)
    pix.fill(Qt.transparent)
    painter = QPainter(pix)
    painter.setRenderHint(QPainter.Antialiasing, True)
    painter.setPen(Qt.NoPen)
    painter.setBrush(QColor("#E8EFF7"))
    painter.drawRoundedRect(
        QRectF(0.5, 0.5, size - 1.0, size - 1.0), size * 0.26, size * 0.26
    )

    runway_y = size * 0.885
    painter.setPen(
        QPen(QColor(232, 163, 61, 105), max(1.0, size * 0.028),
             Qt.SolidLine, Qt.RoundCap)
    )
    painter.drawLine(
        QPointF(size * 0.14, runway_y), QPointF(size * 0.64, runway_y)
    )
    painter.setPen(
        QPen(QColor(232, 163, 61, 190), max(0.8, size * 0.016),
             Qt.SolidLine, Qt.FlatCap)
    )
    dash_x = 0.30
    for _ in range(3):
        painter.drawLine(
            QPointF(size * dash_x, runway_y),
            QPointF(size * (dash_x + 0.06), runway_y),
        )
        dash_x += 0.13

    painter.setPen(Qt.NoPen)
    painter.setBrush(QColor("#D88B22"))
    painter.translate(size * 0.50, size * 0.42)
    span = size * 0.345
    outline = QPainterPath()
    for idx, (px, py) in enumerate(PLANE_OUTLINE):
        point = QPointF(px * span, py * span)
        if idx == 0:
            outline.moveTo(point)
        else:
            outline.lineTo(point)
    outline.closeSubpath()
    painter.drawPath(outline)
    painter.end()
    return pix


class ScreenPreview(QWidget):
    """Miniature of the HDMI output: the selected slide with the amber ticker
    laid over it, so the operator sees what the wall display shows."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(88)
        self._src: QPixmap | None = None
        self._path = ""
        self._scaled: QPixmap | None = None
        self._scaled_for = QSize(0, 0)
        self._text = ""
        self._badge = "IMG01  MANUAL  ON"
        self._short = "IMG01"
        self._resolution = "--"
        self._live = False
        self._board_lines = [""] * BOARD_ROWS
        self._board_visible = False

    # -- state -------------------------------------------------------------
    def set_slide(self, path: str) -> None:
        if path == self._path and self._src is not None:
            return                      # the status poll repeats the same slot
        self._path = path
        pix = QPixmap(path) if path and os.path.exists(path) else QPixmap()
        self._src = None if pix.isNull() else pix
        self._resolution = (
            dimension_text(QSize(pix.width(), pix.height()))
            if not pix.isNull() else "--"
        )
        self._scaled = None
        self.update()

    def set_ticker(self, text: str) -> None:
        self._text = text.strip().upper()
        self.update()

    def set_badge(self, text: str) -> None:
        self._badge = text
        parts = text.split()
        self._short = parts[0] if parts else text
        self.update()

    def set_live(self, on: bool) -> None:
        if on != self._live:
            self._live = on
            self.update()

    def set_board(self, lines: list[str], visible: bool) -> None:
        rows = [sanitize_board_row(line)[0] for line in lines[:BOARD_ROWS]]
        rows += [""] * (BOARD_ROWS - len(rows))
        self._board_lines = rows
        self._board_visible = bool(visible)
        self.update()

    # -- painting ----------------------------------------------------------
    def _frame_rect(self) -> QRectF:
        w = max(1, self.width())
        h = max(1, self.height())
        if w * 3 <= h * 4:                      # panel output is 4:3
            fw = float(w)
            fh = fw * 3.0 / 4.0
        else:
            fh = float(h)
            fw = fh * 4.0 / 3.0
        return QRectF((w - fw) / 2.0, (h - fh) / 2.0, fw, fh)

    def paintEvent(self, _event) -> None:                   # noqa: N802
        box = self._frame_rect()
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)

        clip = QPainterPath()
        clip.addRoundedRect(box, 6.0, 6.0)
        painter.setClipPath(clip)
        painter.fillRect(box, QColor("#06080B"))

        if self._src is not None:
            want = QSize(int(box.width()), int(box.height()))
            if self._scaled is None or self._scaled_for != want:
                self._scaled = self._src.scaled(
                    want, Qt.KeepAspectRatioByExpanding,
                    Qt.SmoothTransformation,
                )
                self._scaled_for = want
            shot = self._scaled
            painter.drawPixmap(
                int(box.left() + (box.width() - shot.width()) / 2.0),
                int(box.top() + (box.height() - shot.height()) / 2.0),
                shot,
            )
            painter.fillRect(box, QColor(5, 7, 10, 92))

        board_top = box.top() + box.height() * (96.0 / 480.0)
        board_h = box.height() * (192.0 / 480.0)
        board = QRectF(box.left(), board_top, box.width(), board_h)
        if self._board_visible:
            painter.fillRect(board, QColor(2, 8, 16, 178))
            painter.setPen(QPen(QColor("#62D8FF"), 1))
            painter.drawLine(
                int(board.left()), int(board.top()),
                int(board.right()), int(board.top()),
            )
            painter.drawLine(
                int(board.left()), int(board.bottom()),
                int(board.right()), int(board.bottom()),
            )
            row_h = board.height() / float(BOARD_ROWS)
            board_font = QFont("Consolas", 7)
            board_font.setPixelSize(max(6, int(box.height() / 480.0 * 15.0)))
            board_font.setBold(True)
            painter.setFont(board_font)
            for row_idx, line in enumerate(self._board_lines):
                row_rect = QRectF(
                    board.left() + 5,
                    board.top() + row_idx * row_h,
                    board.width() - 10,
                    row_h,
                )
                painter.setPen(
                    QColor("#8EEBFF") if row_idx == 0 else QColor("#F2F5F8")
                )
                painter.drawText(
                    row_rect,
                    Qt.AlignVCenter | Qt.AlignLeft,
                    line.rstrip(),
                )

        bar_top = box.top() + box.height() * 0.70
        bar = QRectF(box.left(), bar_top, box.width(), box.height() * 0.30)
        painter.fillRect(bar, QColor(3, 5, 8, 196))
        painter.setPen(QPen(QColor(232, 163, 61, 150), 1))
        painter.drawLine(int(bar.left()), int(bar_top),
                         int(bar.right()), int(bar_top))

        tick = self._text or "AIRPORT INFORMATION DISPLAY"
        size = max(9, min(15, int(box.width() / 21)))
        font = QFont("Consolas", size)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QColor("#FFC45C"))
        metrics = QFontMetrics(font)
        room = int(bar.width() - 18)
        painter.drawText(
            QRectF(bar.left() + 9, bar.top(), room, bar.height()),
            Qt.AlignVCenter | Qt.AlignLeft,
            metrics.elidedText(tick, Qt.ElideRight, room),
        )

        live_text = "LIVE" if self._live else "STANDBY"
        live_color = QColor("#4FE0B0") if self._live else QColor("#8B929B")
        live_font = QFont("Consolas", max(7, size - 4))
        live_font.setBold(True)
        live_metrics = QFontMetrics(live_font)
        live_w = live_metrics.horizontalAdvance(live_text) + 22

        badge_font = QFont("Microsoft YaHei UI", max(7, size - 4))
        badge_font.setBold(True)
        badge_metrics = QFontMetrics(badge_font)
        room = max(46, int(box.width()) - 18 - int(live_w) - 8)
        avail = max(24, room - 16)
        source = self._badge
        if badge_metrics.horizontalAdvance(source) > avail and self._short:
            source = self._short
        badge_text = badge_metrics.elidedText(
            source, Qt.ElideRight, avail
        )
        badge_w = min(room, badge_metrics.horizontalAdvance(badge_text) + 16)
        painter.setFont(badge_font)
        badge = QRectF(box.left() + 9, box.top() + 9, badge_w, 20)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(3, 5, 8, 190))
        painter.drawRoundedRect(badge, 5.0, 5.0)
        painter.setPen(QColor("#D8DDE3"))
        painter.drawText(badge, Qt.AlignCenter, badge_text)

        res_font = QFont("Consolas", max(7, size - 5))
        res_font.setBold(True)
        res_metrics = QFontMetrics(res_font)
        res_text = "RES  %s" % self._resolution
        res_w = res_metrics.horizontalAdvance(res_text) + 16
        res = QRectF(box.left() + 9, box.top() + 33, res_w, 18)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(3, 5, 8, 178))
        painter.drawRoundedRect(res, 4.0, 4.0)
        painter.setFont(res_font)
        painter.setPen(QColor("#8CD6F5"))
        painter.drawText(res, Qt.AlignCenter, res_text)

        painter.setFont(live_font)
        live = QRectF(box.right() - live_w - 9, box.top() + 9, live_w, 20)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(3, 5, 8, 190))
        painter.drawRoundedRect(live, 5.0, 5.0)
        painter.setBrush(live_color)
        painter.drawEllipse(
            QPointF(live.left() + 10, live.center().y()), 3.0, 3.0
        )
        painter.setPen(QColor("#D8DDE3"))
        painter.drawText(
            QRectF(live.left() + 17, live.top(), live.width() - 20, live.height()),
            Qt.AlignVCenter | Qt.AlignLeft, live_text,
        )

        painter.setClipping(False)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor("#C6D2DF"), 1))
        painter.drawRoundedRect(box.adjusted(0.5, 0.5, -0.5, -0.5), 6.0, 6.0)
        painter.end()


# ---------------------------------------------------------------------------
# main window
# ---------------------------------------------------------------------------


class Console(QWidget):
    def __init__(self, assets: str, port: str = "", baud: int = 115200) -> None:
        super().__init__()
        self.assets = assets
        self.worker: SerialWorker | None = None
        self.connected = False
        self._status: dict | None = None
        self._counts = {"tx": 0, "rx": 0, "err": 0}
        self._resolution_cache: dict[int, QSize] = {}
        self._mode_cmd = 2                      # what we last told the board
        self._link_kind = PORT_KIND_USB
        self._active_link_kind = PORT_KIND_USB
        self._port_meta: dict[str, str] = {}
        self._param_timer = QTimer(self)
        self._param_timer.setSingleShot(True)
        self._param_timer.setInterval(120)
        self._param_timer.timeout.connect(self._flush_params)
        self._pending_param: tuple[int, int] | None = None
        self._board_dirty: set[int] = set()
        self._board_visible = True
        self._board_timer = QTimer(self)
        self._board_timer.setSingleShot(True)
        self._board_timer.setInterval(450)
        self._board_timer.timeout.connect(self._flush_board)
        # Whole-page rotation: page 0 is the template the board starts with.
        self._page_index = 0
        self._rotate_timer = QTimer(self)
        self._rotate_timer.setInterval(ROTATE_MS)
        self._rotate_timer.timeout.connect(self._advance_page)
        # Last picture the card reported; the status poll runs once a second,
        # so a picture change is noticed within one poll.
        self._last_slot: int | None = None
        self._mini_columns = 2
        self._mini_columns_wanted = 2

        self.setObjectName("root")
        self.setWindowTitle("天枢 · 空港智显   上位机控制台")
        # 1400 keeps the three columns at their natural widths: the left column
        # holds the longest value strings ("A5 5A v14 + BOARD 6x40") and starts
        # to clip its right edge below this window width.
        self.setMinimumSize(1220, 700)
        screen = QApplication.primaryScreen()
        if screen is not None:
            avail = screen.availableGeometry()
            self.resize(
                min(1500, max(1260, avail.width() - 80)),
                min(950, max(700, avail.height() - 80)),
            )
        else:
            self.resize(1420, 900)

        root = QVBoxLayout(self)
        root.setContentsMargins(18, 16, 18, 16)
        root.setSpacing(12)

        root.addWidget(self._build_header())

        body = QHBoxLayout()
        body.setSpacing(12)
        body.addWidget(self._build_left(), 0)
        body.addWidget(self._build_center(), 1)
        body.addWidget(self._build_right(), 0)
        root.addLayout(body, 1)

        self.refresh_ports()
        if port:
            idx = self.port_box.findText(port)
            if idx >= 0:
                self.port_box.setCurrentIndex(idx)
        idx = self.baud_box.findData(baud)
        if idx >= 0:
            self.baud_box.setCurrentIndex(idx)
        self._refresh_slots()
        self.preview.set_slide(os.path.join(self.assets, "IMG01.BMP"))
        self._sync_board_preview()
        self.log_line(
            "info", "选择 USB CH340 或已配对的蓝牙 SPP 串口，连接后即可控制板卡"
        )

        self._toast = QTimer(self)
        self._toast.setSingleShot(True)
        self._toast.timeout.connect(lambda: self.hint.setText(""))
        self._clock_timer = QTimer(self)
        self._clock_timer.setInterval(1000)
        self._clock_timer.timeout.connect(self._tick_clock)
        self._clock_timer.start()
        self._tick_clock()

    # -- construction ------------------------------------------------------
    def _tick_clock(self) -> None:
        self.clock.setText(time.strftime("%Y-%m-%d  %H:%M:%S"))

    def _build_header(self) -> QWidget:
        head = QFrame()
        head.setObjectName("header")
        lay = QHBoxLayout(head)
        lay.setContentsMargins(18, 12, 18, 12)
        lay.setSpacing(13)

        logo = QLabel()
        logo.setPixmap(brand_pixmap(42))
        logo.setFixedSize(42, 42)
        lay.addWidget(logo)

        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        title = QLabel("天枢 · 空港智显")
        title.setObjectName("appTitle")
        sub = QLabel("HDMI 航显播放器 · 串口控制台")
        sub.setObjectName("appSub")
        title_box.addWidget(title)
        title_box.addWidget(sub)
        lay.addLayout(title_box)

        chip = QLabel("v18 · HDMI A+B · USB / SPP · TIME")
        chip.setObjectName("brandChip")
        chip.setAlignment(Qt.AlignCenter)
        chip.setFixedHeight(22)
        lay.addWidget(chip, 0, Qt.AlignVCenter)
        lay.addStretch(1)

        self.clock = QLabel("--:--:--")
        self.clock.setObjectName("clock")
        self.clock.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lay.addWidget(self.clock)

        self.link_pill = LinkPill()
        lay.addWidget(self.link_pill)
        return head

    def _build_left(self) -> QWidget:
        col = QWidget()
        col.setMinimumWidth(370)
        lay = QVBoxLayout(col)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(14)

        box, bl = panel("连接方式")
        self.link_seg = Segmented(["USB 串口", "蓝牙 SPP"], current=0)
        self.link_seg.changed.connect(self._on_link_kind)
        bl.addWidget(self.link_seg)

        row = QHBoxLayout()
        row.setSpacing(6)
        self.port_box = QComboBox()
        self.port_box.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.refresh_btn = QToolButton()
        self.refresh_btn.setObjectName("iconBtn")
        self.refresh_btn.setIcon(
            self.style().standardIcon(QStyle.SP_BrowserReload)
        )
        self.refresh_btn.setToolTip("重新扫描串口")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        row.addWidget(self.port_box, 1)
        row.addWidget(self.refresh_btn)
        bl.addLayout(row)

        self.baud_box = QComboBox()
        for idx, rate in enumerate(BAUD_RATES):
            self.baud_box.addItem("%d" % rate, rate)
        self.baud_box.setCurrentIndex(0)
        baud_row = QHBoxLayout()
        baud_row.setSpacing(8)
        baud_lbl = QLabel("波特率")
        baud_lbl.setObjectName("rowLabel")
        baud_row.addWidget(baud_lbl)
        baud_row.addWidget(self.baud_box, 1)
        bl.addLayout(baud_row)

        link_row = QHBoxLayout()
        link_row.setSpacing(8)
        self.link_note = QLabel("CH340 USB-UART")
        self.link_note.setObjectName("caption")
        self.bt_settings_btn = QPushButton("蓝牙设置")
        self.bt_settings_btn.setObjectName("ghost")
        self.bt_settings_btn.setCursor(Qt.PointingHandCursor)
        self.bt_settings_btn.setVisible(False)
        self.bt_settings_btn.clicked.connect(self._open_bluetooth_settings)
        link_row.addWidget(self.link_note, 1)
        link_row.addWidget(self.bt_settings_btn)
        bl.addLayout(link_row)

        self.connect_btn = QPushButton("连接板卡")
        self.connect_btn.setObjectName("primary")
        self.connect_btn.setMinimumHeight(36)
        self.connect_btn.setCursor(Qt.PointingHandCursor)
        self.connect_btn.clicked.connect(self.toggle_link)
        bl.addWidget(self.connect_btn)

        self.hint = QLabel("")
        self.hint.setObjectName("hint")
        self.hint.setWordWrap(True)
        bl.addWidget(self.hint)
        lay.addWidget(box)

        box2, sl = panel("板卡状态")
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)
        grid.setColumnStretch(1, 1)
        self.status_labels: dict[str, QLabel] = {}
        rows = [
            ("version", "版本"),
            ("slot", "当前画面"),
            ("resolution", "图像分辨率"),
            ("mode", "运行模式"),
            ("cache", "已缓存"),
            ("textlen", "字幕字数"),
            ("card", "TF 卡"),
        ]
        for r, (key, name) in enumerate(rows):
            k = QLabel(name)
            k.setObjectName("rowLabel")
            v = QLabel("--")
            v.setObjectName("rowValue")
            v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(k, r, 0)
            grid.addWidget(v, r, 1)
            self.status_labels[key] = v
        sl.addLayout(grid)
        self.cache_bar = QProgressBar()
        self.cache_bar.setObjectName("cacheBar")
        self.cache_bar.setRange(0, SLOT_COUNT)
        self.cache_bar.setValue(0)
        self.cache_bar.setTextVisible(False)
        self.cache_bar.setFixedHeight(18)
        sl.addSpacing(2)
        sl.addWidget(self.cache_bar)
        lay.addWidget(box2)

        box3, fl = panel("板卡信息")
        facts = QGridLayout()
        facts.setHorizontalSpacing(10)
        facts.setVerticalSpacing(8)
        facts.setColumnStretch(1, 1)
        for r, (name, value) in enumerate(BOARD_FACTS):
            k = QLabel(name)
            k.setObjectName("rowLabel")
            v = QLabel(value)
            v.setObjectName("rowValue")
            v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            facts.addWidget(k, r, 0)
            facts.addWidget(v, r, 1)
        fl.addLayout(facts)
        lay.addWidget(box3)

        # The link log lives here rather than in the middle column: the middle
        # column has to fit the slot grid, the ticker and the flight board with
        # no scrolling, so anything extra would be pushed below the fold.
        lay.addWidget(self._build_log(), 1)

        scroll = QScrollArea()
        scroll.setObjectName("leftScroll")
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(col)
        # Hold the full panel width: a QScrollArea reports a tiny minimum of
        # its own, so without this the bodies inside get squeezed and their
        # right-hand values are cut off.
        scroll.setMinimumWidth(398)
        return scroll

    def _build_center(self) -> QWidget:
        col = QWidget()
        lay = QVBoxLayout(col)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)

        box, cl = panel("画面槽位")
        self.slot_cards: list[SlotCard] = []
        grid = QGridLayout()
        grid.setSpacing(6)
        for i in range(SLOT_COUNT):
            path = os.path.join(self.assets, "IMG%02d.BMP" % (i + 1))
            card = SlotCard(i, path)
            card.clicked.connect(self.send_slot)
            self.slot_cards.append(card)
            grid.addWidget(card, i // 3, i % 3)
        cl.addLayout(grid)
        lay.addWidget(box, 0)

        box2, tl = panel("航显字幕")
        self.text_edit = QLineEdit()
        self.text_edit.setObjectName("ticker")
        self.text_edit.setPlaceholderText("CA1832 GATE A12 BOARDING")
        self.text_edit.setMaxLength(96)
        self.text_edit.setMinimumHeight(40)
        self.text_edit.textChanged.connect(self._on_text_changed)
        self.text_edit.returnPressed.connect(self.send_text)
        tl.addWidget(self.text_edit)

        row = QHBoxLayout()
        row.setSpacing(10)
        self.counter = QLabel("0 / %d" % TEXT_MAX)
        self.counter.setObjectName("counter")
        self.preset_box = QComboBox()
        self.preset_box.setSizeAdjustPolicy(
            QComboBox.AdjustToMinimumContentsLengthWithIcon
        )
        self.preset_box.setMinimumContentsLength(10)
        self.preset_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.preset_box.addItem("常用播报", "")
        for text in PRESETS:
            self.preset_box.addItem(text, text)
        self.preset_box.currentIndexChanged.connect(self._apply_preset)
        row.addWidget(self.counter)
        row.addWidget(self.preset_box, 1)
        tl.addLayout(row)

        actions = QHBoxLayout()
        actions.setSpacing(8)
        actions.addStretch(1)
        restore = QPushButton("恢复内置字幕")
        restore.setObjectName("ghost")
        restore.setCursor(Qt.PointingHandCursor)
        restore.clicked.connect(lambda: self.send_text(builtin=True))
        send = QPushButton("发送字幕")
        send.setObjectName("primary")
        send.setCursor(Qt.PointingHandCursor)
        send.clicked.connect(self.send_text)
        actions.addWidget(restore)
        actions.addWidget(send)
        tl.addLayout(actions)
        lay.addWidget(box2, 0)

        box3, bl = panel("实时航班看板")
        board_head = QHBoxLayout()
        board_head.setSpacing(8)
        board_title = QLabel("FPGA 叠加 · 6 × 40")
        board_title.setObjectName("caption")
        board_head.addWidget(board_title)
        board_head.addStretch(1)
        self.board_counter = QLabel("%d / %d" % (BOARD_ROWS, BOARD_ROWS))
        self.board_counter.setObjectName("counter")
        board_head.addWidget(self.board_counter)
        self.board_switch = QPushButton("看板 ON")
        self.board_switch.setObjectName("seg")
        self.board_switch.setCheckable(True)
        self.board_switch.setChecked(True)
        self.board_switch.setCursor(Qt.PointingHandCursor)
        self.board_switch.toggled.connect(self.toggle_board)
        board_head.addWidget(self.board_switch)
        bl.addLayout(board_head)

        board_options = QHBoxLayout()
        board_options.setSpacing(12)
        self.follow_box = QCheckBox("跟随切图")
        self.follow_box.setToolTip("画面每换一张，航班信息就换到下一组")
        self.follow_box.setChecked(True)
        self.follow_box.toggled.connect(self._on_follow_toggled)
        board_options.addWidget(self.follow_box)
        self.rotate_box = QCheckBox("每 10 秒轮换")
        self.rotate_box.setToolTip("整组信息自动循环，播完一轮回到第一组")
        self.rotate_box.toggled.connect(self._on_rotate_toggled)
        board_options.addWidget(self.rotate_box)
        board_options.addStretch(1)
        bl.addLayout(board_options)

        # The panel flips between a wall of six live mini screens and the
        # six-row editor.  Both read the same pages, so a picture cut updates
        # either view without asking the card for anything.
        self.board_stack = QStackedWidget()

        wall = QWidget()
        wall_lay = QGridLayout(wall)
        wall_lay.setContentsMargins(0, 0, 0, 0)
        wall_lay.setSpacing(8)
        self.mini_boards: list[MiniBoard] = []
        self.mini_wall = wall
        self.mini_wall_layout = wall_lay
        for i in range(SLOT_COUNT):
            mini = MiniBoard(i)
            mini.clicked.connect(self._on_mini_clicked)
            self.mini_boards.append(mini)
            wall_lay.addWidget(mini, i // 2, i % 2)
        self.board_stack.addWidget(wall)

        edit = QWidget()
        edit_lay = QVBoxLayout(edit)
        edit_lay.setContentsMargins(0, 0, 0, 0)
        edit_lay.setSpacing(6)
        board_ruler = QLabel("FLIGHT AIR DEST  STD   ETD   GATE STATUS")
        board_ruler.setObjectName("boardRuler")
        edit_lay.addWidget(board_ruler)

        self.board_view = QPlainTextEdit()
        self.board_view.setObjectName("boardRows")
        self.board_view.setPlainText("\n".join(BOARD_DEFAULT))
        self.board_view.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.board_view.setTabChangesFocus(True)
        # Six rows always fit in the widget, so the bar would only ever be
        # noise.  The middle column itself still scrolls when the window is
        # made small, which is the control that actually matters.
        self.board_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        # Six 15 px text rows plus padding: never let the widget clip a row.
        self.board_view.setMinimumHeight(104)
        self.board_view.textChanged.connect(self._on_board_changed)
        edit_lay.addWidget(self.board_view, 1)
        self.board_stack.addWidget(edit)
        bl.addWidget(self.board_stack, 1)

        board_actions = QHBoxLayout()
        board_actions.setSpacing(8)
        self.board_mode = Segmented(["六屏", "编辑"], current=0)
        self.board_mode.changed.connect(self._on_board_mode)
        board_actions.addWidget(self.board_mode)
        self.page_box = QComboBox()
        for name, _page in BOARD_PAGES:
            self.page_box.addItem(name)
        self.page_box.setMinimumWidth(150)
        self.page_box.currentIndexChanged.connect(self._on_page_selected)
        board_actions.addWidget(self.page_box)
        bl.addLayout(board_actions)

        board_buttons = QHBoxLayout()
        board_buttons.setSpacing(8)
        board_buttons.addStretch(1)
        board_preset = QPushButton("恢复航班模板")
        board_preset.setObjectName("ghost")
        board_preset.clicked.connect(self._apply_board_preset)
        board_clear = QPushButton("清空")
        board_clear.setObjectName("ghost")
        board_clear.clicked.connect(self._clear_board)
        board_send = QPushButton("同步整页")
        board_send.setObjectName("primary")
        board_send.clicked.connect(self.send_board_all)
        board_buttons.addWidget(board_preset)
        board_buttons.addWidget(board_clear)
        board_buttons.addWidget(board_send)
        bl.addLayout(board_buttons)
        lay.addWidget(box3, 1)

        scroll = QScrollArea()
        scroll.setObjectName("leftScroll")
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(col)
        self._center_bar = AutoHideBar(scroll, self)
        self._center_scroll = scroll
        self._center_viewport = scroll.viewport()
        self._center_viewport.installEventFilter(self)
        return scroll

    def _schedule_mini_wall_mode(self) -> None:
        area = getattr(self, "_center_scroll", None)
        if area is None:
            return
        target = 2 if area.viewport().width() >= 680 else 1
        if target != self._mini_columns_wanted:
            self._mini_columns_wanted = target
            QTimer.singleShot(0, self._apply_mini_wall_mode)

    def eventFilter(self, obj, event) -> bool:          # noqa: N802
        if (
            obj is getattr(self, "_center_viewport", None)
            and event.type() == QEvent.Resize
        ):
            self._schedule_mini_wall_mode()
        return super().eventFilter(obj, event)

    def _apply_mini_wall_mode(self) -> None:
        """Reflow the six mini screens when the centre column gets narrow."""
        target = self._mini_columns_wanted
        if target == self._mini_columns:
            return
        layout = self.mini_wall_layout
        while layout.count():
            layout.takeAt(0)
        for i, mini in enumerate(self.mini_boards):
            layout.addWidget(mini, i // target, i % target)
        self._mini_columns = target
        self.mini_wall.updateGeometry()

    def resizeEvent(self, event) -> None:               # noqa: N802
        super().resizeEvent(event)
        self._schedule_mini_wall_mode()

    def _build_right(self) -> QWidget:
        col = QWidget()
        col.setMinimumWidth(320)
        lay = QVBoxLayout(col)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        box0, vl = panel("上屏预览")
        self.preview = ScreenPreview()
        self.preview.setMinimumHeight(186)
        vl.addWidget(self.preview, 1)
        lay.addWidget(box0, 1)

        box, ml = panel("切图模式")
        self.mode_seg = Segmented(["手动", "自动", "跟随开关"], current=2)
        self.mode_seg.changed.connect(self.send_mode)
        ml.addWidget(self.mode_seg)
        note = QLabel("SW1 决定手动或自动")
        note.setObjectName("caption")
        ml.addWidget(note)
        lay.addWidget(box)

        box2, pl = panel("显示参数")
        self.bright = Slider("亮度", 7, 7)
        self.contrast = Slider("对比度", 7, 3)
        self.bright.changed.connect(lambda v: self._queue_param(0, v))
        self.contrast.changed.connect(lambda v: self._queue_param(1, v))
        pl.addWidget(self.bright)
        pl.addWidget(self.contrast)
        lay.addWidget(box2)

        box3, ol = panel("信息面板")
        self.osd_seg = Segmented(
            ["FULL", "INFO", "ON", "OFF"], current=2
        )
        self.osd_seg.changed.connect(self.send_osd)
        ol.addWidget(self.osd_seg)
        lay.addWidget(box3)

        box4, cl2 = panel("链路统计")
        stat_row = QHBoxLayout()
        stat_row.setSpacing(8)
        self.chips: dict[str, QLabel] = {}
        for key, name in (("tx", "TX"), ("rx", "RX"), ("err", "ERR")):
            chip = QLabel("%s 0" % name)
            chip.setObjectName("chip")
            chip.setProperty("kind", key)
            chip.setProperty("on", "false")
            chip.setAlignment(Qt.AlignCenter)
            chip.setFixedHeight(26)
            self.chips[key] = chip
            stat_row.addWidget(chip, 1)
        cl2.addLayout(stat_row)
        lay.addWidget(box4)

        scroll = QScrollArea()
        scroll.setObjectName("rightScroll")
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(col)
        scroll.setMinimumWidth(340)
        self._right_bar = AutoHideBar(scroll, self)
        return scroll

    def _build_log(self) -> QWidget:
        box = QFrame()
        box.setObjectName("panel")
        lay = QVBoxLayout(box)
        lay.setContentsMargins(16, 14, 16, 16)
        lay.setSpacing(10)

        head = QHBoxLayout()
        head.setSpacing(8)
        title = QLabel("链路日志")
        title.setObjectName("panelTitle")
        head.addWidget(title)
        head.addStretch(1)
        self.clear_btn = QPushButton("清空")
        self.clear_btn.setObjectName("ghost")
        self.clear_btn.clicked.connect(lambda: self.log_view.clear())
        head.addWidget(self.clear_btn)
        lay.addLayout(head)

        self.log_view = QPlainTextEdit()
        self.log_view.setObjectName("log")
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(600)
        box.setMinimumHeight(150)
        lay.addWidget(self.log_view, 1)
        return box

    # -- ports -------------------------------------------------------------
    def _on_link_kind(self, index: int) -> None:
        self._link_kind = (
            PORT_KIND_BLUETOOTH if index == 1 else PORT_KIND_USB
        )
        is_bluetooth = self._link_kind == PORT_KIND_BLUETOOTH
        self.link_note.setText(
            "Windows 配对后的 SPP COM 口" if is_bluetooth else "CH340 USB-UART"
        )
        self.bt_settings_btn.setVisible(is_bluetooth)
        if is_bluetooth:
            idx = self.baud_box.findData(FALLBACK_RATE)
            if idx >= 0:
                self.baud_box.setCurrentIndex(idx)
        self.port_box.setCurrentIndex(-1)
        self.refresh_ports()
        if is_bluetooth:
            found = any(kind == PORT_KIND_BLUETOOTH
                        for kind in self._port_meta.values())
            if found:
                self.log_line("info", "蓝牙 SPP 模式：已发现可用串口")
            else:
                self.log_line("info", "蓝牙 SPP 模式：尚未发现已配对串口")
                self.show_hint("请先在 Windows 蓝牙设置中完成配对")
        else:
            self.log_line("info", "USB 串口模式")
        self.connect_btn.setText("连接蓝牙" if is_bluetooth else "连接板卡")

    def _open_bluetooth_settings(self) -> None:
        try:
            if sys.platform.startswith("win"):
                os.startfile("ms-settings:bluetooth")    # type: ignore[attr-defined]
            else:
                QDesktopServices.openUrl(QUrl("ms-settings:bluetooth"))
            self.log_line("info", "已打开系统蓝牙设置")
        except Exception as exc:                    # noqa: BLE001
            self.log_line("err", "无法打开蓝牙设置: %s" % exc)
            self.show_hint("请手动打开系统蓝牙设置")

    def refresh_ports(self) -> None:
        current = self.port_box.currentData()
        self.port_box.clear()
        if list_ports is None:
            self.port_box.addItem("pyserial missing", "")
            return
        ports = list(list_ports.comports())
        self._port_meta = {}
        entries = []
        for p in ports:
            kind = classify_serial_port(p)
            self._port_meta[p.device] = kind
            description = str(getattr(p, "description", "") or "")
            label = "[%s] %s" % (port_kind_label(kind), p.device)
            if description and description not in ("n/a",):
                label += "  ·  " + description
            entries.append((kind, label, p.device))
        kind_order = {self._link_kind: 0, PORT_KIND_SERIAL: 1}
        entries.sort(
            key=lambda item: (
                kind_order.get(item[0], 2),
                item[2],
            )
        )
        for _kind, label, device in entries:
            self.port_box.addItem(label, device)
        if not ports:
            self.port_box.addItem("未发现串口", "")
        elif current:
            idx = self.port_box.findData(current)
            if idx >= 0:
                self.port_box.setCurrentIndex(idx)
        else:
            preferred = -1
            for i in range(self.port_box.count()):
                device = self.port_box.itemData(i)
                if self._port_meta.get(device) == self._link_kind:
                    preferred = i
                    break
            if preferred >= 0:
                self.port_box.setCurrentIndex(preferred)
            elif self._link_kind == PORT_KIND_BLUETOOTH:
                self.port_box.insertItem(0, "未发现蓝牙 SPP 串口", "")
                item = self.port_box.model().item(0)
                if item is not None:
                    item.setEnabled(False)
                self.port_box.setCurrentIndex(0)
            else:
                self.port_box.setCurrentIndex(0)

    # -- link --------------------------------------------------------------
    def toggle_link(self) -> None:
        if self.connected:
            self.disconnect_board()
        else:
            self.connect_board()

    def connect_board(self) -> None:
        if serial is None:
            self.log_line("err", "pyserial is not installed")
            self.show_hint("缺少 pyserial，请先安装")
            return
        port = self.port_box.currentData()
        if not port:
            self.show_hint(
                "请选择已配对蓝牙串口"
                if self._link_kind == PORT_KIND_BLUETOOTH
                else "请选择串口"
            )
            return
        self._active_link_kind = self._port_meta.get(port, PORT_KIND_SERIAL)
        baud = self.baud_box.currentData()

        self.worker = SerialWorker(port, baud, self)
        self.worker.opened.connect(self._on_opened)
        self.worker.link.connect(self._on_link)
        self.worker.log.connect(self.log_line)
        self.worker.got_status.connect(self._on_status)
        self.worker.got_reply.connect(self._on_reply)
        self.worker.rate_changed.connect(self._on_rate)
        self.worker.start()
        self.connect_btn.setEnabled(False)

    def disconnect_board(self) -> None:
        if self.worker is not None:
            self.worker.stop()
            self.worker.wait(1500)
            self.worker = None
        self._set_connected(False)

    def _on_opened(self, ok: bool, info: str) -> None:
        if ok:
            transport = (
                "蓝牙 SPP " if self._active_link_kind == PORT_KIND_BLUETOOTH
                else "USB-UART " if self._active_link_kind == PORT_KIND_USB
                else ""
            )
            self.log_line("ok", "已打开 " + transport + info)
            self._set_connected(True)
            self.worker.send(CMD_STATUS)
        else:
            if self.worker is not None:
                self.log_line("err", "关闭 " + info)
            self._set_connected(False)

    def _set_connected(self, ok: bool) -> None:
        self.connected = ok
        self.connect_btn.setEnabled(True)
        is_bluetooth = self._active_link_kind == PORT_KIND_BLUETOOTH
        if ok:
            self.connect_btn.setText("断开蓝牙" if is_bluetooth else "断开连接")
        else:
            self.connect_btn.setText(
                "连接蓝牙" if self._link_kind == PORT_KIND_BLUETOOTH
                else "连接板卡"
            )
        self.connect_btn.setObjectName("danger" if ok else "primary")
        self.connect_btn.style().unpolish(self.connect_btn)
        self.connect_btn.style().polish(self.connect_btn)
        self.port_box.setEnabled(not ok)
        self.refresh_btn.setEnabled(not ok)
        self.link_seg.setEnabled(not ok)
        self.bt_settings_btn.setEnabled(not ok)
        if ok:
            if is_bluetooth:
                link_text = "蓝牙已连接"
            elif self._active_link_kind == PORT_KIND_USB:
                link_text = "USB 已连接"
            else:
                link_text = "串口已连接"
        else:
            link_text = "未连接"
        self.link_pill.set_state("online" if ok else "offline", link_text)
        self.preview.set_live(ok)
        if not ok:
            self._status = None
            self._refresh_status()
            self._refresh_slots()

    def _on_link(self, ok: bool) -> None:
        self.link_pill.set_state(
            "online" if ok else "error", "已连接" if ok else "无应答"
        )

    def _on_rate(self, rate: int) -> None:
        idx = self.baud_box.findData(rate)
        if idx >= 0:
            self.baud_box.blockSignals(True)
            self.baud_box.setCurrentIndex(idx)
            self.baud_box.blockSignals(False)
        self.log_line("info", "串口速率 %d" % rate)

    # -- sending -----------------------------------------------------------
    def _require_link(self) -> bool:
        if not self.connected or self.worker is None:
            self.show_hint("请先连接板卡")
            return False
        return True

    def send_slot(self, index: int) -> None:
        if not self._require_link():
            return
        self.worker.send(CMD_SLOT, bytes([index]))

    def send_mode(self, index: int) -> None:
        self._mode_cmd = index
        if not self._require_link():
            return
        self.worker.send(CMD_MODE, bytes([index]))

    def send_osd(self, index: int) -> None:
        if not self._require_link():
            return
        self.worker.send(CMD_OSD, bytes([index]))

    def _queue_param(self, sel: int, value: int) -> None:
        self._pending_param = (sel, value)
        self._param_timer.start()

    def _flush_params(self) -> None:
        if self._pending_param is None or not self._require_link():
            return
        sel, value = self._pending_param
        self._pending_param = None
        self.worker.send(CMD_PARAM, bytes([sel, value]))

    def send_text(self, builtin: bool = False) -> None:
        if not self._require_link():
            return
        if builtin:
            self.worker.send(CMD_TEXT, b"")
            self.text_edit.clear()
            return
        text, _ = sanitize_text(self.text_edit.text())
        if self.text_edit.text() != text:
            self.text_edit.blockSignals(True)
            self.text_edit.setText(text)
            self.text_edit.blockSignals(False)
            self.counter.setText("%d / %d" % (len(text), TEXT_MAX))
        self.worker.send(CMD_TEXT, text.encode("ascii"))

    def _apply_preset(self, _idx: int) -> None:
        text = self.preset_box.currentData()
        if text:
            self.text_edit.setText(text)

    # -- flight board ------------------------------------------------------
    def _board_lines(self) -> list[str]:
        raw = self.board_view.toPlainText().splitlines()
        rows = [sanitize_board_row(line)[0] for line in raw[:BOARD_ROWS]]
        rows += [" " * BOARD_COLS] * (BOARD_ROWS - len(rows))
        return rows

    def _sync_board_preview(self) -> None:
        if hasattr(self, "preview"):
            self.preview.set_board(self._board_lines(), self._board_visible)

    def _on_board_changed(self) -> None:
        if not hasattr(self, "board_view"):
            return
        raw = self.board_view.toPlainText().splitlines()
        dropped = 0
        for line in raw[:BOARD_ROWS]:
            dropped += sanitize_board_row(line)[1]
        if hasattr(self, "board_counter"):
            used = sum(1 for line in raw[:BOARD_ROWS] if line.strip())
            self.board_counter.setText("%d / %d" % (used, BOARD_ROWS))
        self._board_dirty.update(range(BOARD_ROWS))
        self._sync_board_preview()
        if hasattr(self, "mini_boards") and self.mini_boards:
            self.mini_boards[self._page_index].set_rows(self._board_lines())
        if dropped:
            self.show_hint("看板仅支持英文、数字和常用符号")
        elif len(raw) > BOARD_ROWS:
            self.show_hint("看板只保留前 %d 行" % BOARD_ROWS)
        self._board_timer.start()

    def _board_payload(self, row: int) -> bytes:
        text = self._board_lines()[row]
        return bytes([row & 0xFF]) + text.encode("ascii")

    def send_board_row(self, row: int) -> None:
        if not self._require_link():
            return
        self.worker.send(CMD_BOARD, self._board_payload(row))
        self._board_dirty.discard(row)

    def _flush_board(self) -> None:
        if not self.connected or self.worker is None or not self._board_dirty:
            return
        for row in sorted(self._board_dirty):
            self.worker.send(CMD_BOARD, self._board_payload(row))
        self._board_dirty.clear()

    def send_board_all(self) -> None:
        if not self._require_link():
            return
        self._board_timer.stop()
        self._board_dirty.clear()
        for row in range(BOARD_ROWS):
            self.worker.send(CMD_BOARD, self._board_payload(row))

    def toggle_board(self, on: bool) -> None:
        self._board_visible = bool(on)
        self.board_switch.setText("看板 ON" if on else "看板 OFF")
        self._sync_board_preview()
        if self.connected and self.worker is not None:
            self.worker.send(CMD_BVIS, bytes([1 if on else 0]))

    def _apply_board_preset(self) -> None:
        self.board_view.setPlainText("\n".join(BOARD_DEFAULT))
        self._board_timer.start()

    # -- board page rotation ----------------------------------------------
    def _store_page(self, index: int) -> None:
        """Keep whatever the operator typed for the page being left."""
        if 0 <= index < len(BOARD_PAGES):
            BOARD_PAGES[index][1][:] = self._board_lines()

    def _on_page_selected(self, index: int) -> None:
        if index == self._page_index or index < 0:
            return
        self._store_page(self._page_index)
        self._apply_page(index)
        self.log_line(
            "info", "航班信息组切换到「%s」" % BOARD_PAGES[index][0]
        )

    def _apply_page(self, index: int, send: bool = True) -> None:
        """Paint one information group into the editor, and usually the card.

        Selecting a small screen for editing passes send=False: changing what
        the wall highlights must not put rows on air that do not belong to the
        picture that is playing.
        """
        self._page_index = index
        self.board_view.blockSignals(True)
        self.board_view.setPlainText("\n".join(BOARD_PAGES[index][1]))
        self.board_view.blockSignals(False)
        self.page_box.blockSignals(True)
        self.page_box.setCurrentIndex(index)
        self.page_box.blockSignals(False)
        self._on_board_changed()
        if send and self.connected and self.worker is not None:
            self.send_board_all()

    def _on_rotate_toggled(self, on: bool) -> None:
        if on:
            self._rotate_timer.start()
            self.log_line(
                "info",
                "航班信息组每 %d 秒自动轮换，播完一轮从头再来" % (ROTATE_MS // 1000),
            )
        else:
            self._rotate_timer.stop()
            self.log_line("info", "航班信息组自动轮换已关闭")

    def _advance_page(self) -> None:
        self._store_page(self._page_index)
        index = (self._page_index + 1) % len(BOARD_PAGES)
        self._apply_page(index)

    def _follow_slot(self, slot: int) -> None:
        """A picture cut puts that slot's own rows up and pushes them out."""
        index = PAGE_FOR_SLOT[slot % len(PAGE_FOR_SLOT)] % len(BOARD_PAGES)
        self._store_page(self._page_index)
        self._apply_page(index)
        self.log_line(
            "info",
            "切到 IMG%02d，航班信息上新「%s」" % (slot + 1, BOARD_PAGES[index][0]),
        )

    def _on_board_mode(self, index: int) -> None:
        """Flip the panel between the six screen wall and the row editor."""
        self.board_stack.setCurrentIndex(index)

    def _on_mini_clicked(self, index: int) -> None:
        """Clicking a small screen opens that group in the editor."""
        self.board_mode.set_current(1)
        self.board_stack.setCurrentIndex(1)
        if index != self._page_index:
            self._store_page(self._page_index)
            self._apply_page(index, send=False)
            self.log_line(
                "info", "编辑航班信息组「%s」" % BOARD_PAGES[index][0]
            )
        self.board_view.setFocus()

    def _refresh_mini_live(self, slot: int) -> None:
        for mini in self.mini_boards:
            mini.set_live(mini.index == slot)

    def _on_follow_toggled(self, on: bool) -> None:
        if on:
            # Baseline against the picture already on screen, so flipping the
            # switch back on does not by itself jump a group.
            self._last_slot = self._status["slot"] if self._status else None
            self.log_line("info", "跟随切图已开启：画面每换一张，航班信息上新一组")
        else:
            self.log_line("info", "跟随切图已关闭：航班信息组不再随画面变化")

    def _clear_board(self) -> None:
        self.board_view.clear()
        self._board_timer.start()

    # -- text field --------------------------------------------------------
    def _on_text_changed(self, raw: str) -> None:
        text, dropped = sanitize_text(raw)
        if text != raw:
            self.text_edit.blockSignals(True)
            self.text_edit.setText(text)
            self.text_edit.setCursorPosition(len(text))
            self.text_edit.blockSignals(False)
        self.counter.setText("%d / %d" % (len(text), TEXT_MAX))
        self.counter.setProperty("full", len(text) >= TEXT_MAX)
        self.counter.style().unpolish(self.counter)
        self.counter.style().polish(self.counter)
        self.preview.set_ticker(text)
        if dropped:
            self.show_hint("已忽略 %d 个无法显示的字形" % dropped)

    # -- status ------------------------------------------------------------
    def _on_reply(self, cmd: int, payload: bytes) -> None:
        if cmd == R_TEXT and len(payload) >= 2:
            self.show_hint("字幕已更新，长度 %d" % payload[1])
        elif cmd == R_SLOT and payload:
            self.show_hint("已切到 IMG%02d" % (payload[0] + 1))
        elif cmd == R_MODE and payload:
            names = ["手动", "自动", "跟随开关"]
            self.show_hint("切图模式：%s" % names[min(payload[0], 2)])
        elif cmd == R_BOARD and payload:
            self.show_hint("看板第 %d 行已更新" % (payload[0] + 1))
        elif cmd == R_BVIS and payload:
            on = bool(payload[0] & 1)
            self.board_switch.blockSignals(True)
            self.board_switch.setChecked(on)
            self.board_switch.setText("看板 ON" if on else "看板 OFF")
            self.board_switch.blockSignals(False)
            self._board_visible = on
            self._sync_board_preview()
            self.show_hint("看板已%s" % ("显示" if on else "隐藏"))
        elif cmd == R_TIME and len(payload) >= 3:
            self.show_hint(
                "真实时间已同步 %02d:%02d:%02d"
                % (
                    bcd_to_int(payload[0]),
                    bcd_to_int(payload[1]),
                    bcd_to_int(payload[2]),
                )
            )

    def _on_status(self, st: dict) -> None:
        changed_cache = (
            self._status is None or self._status["cache"] != st["cache"]
        )
        self._status = st
        self._refresh_status()
        if changed_cache:
            self._refresh_slots()
        slot = st["slot"]
        changed_slot = self._last_slot is not None and slot != self._last_slot
        self._last_slot = slot
        self.select_slot(slot)
        self._refresh_mini_live(slot)
        self.link_pill.set_state("online", "已连接")
        if changed_slot and self.follow_box.isChecked():
            self._follow_slot(slot)

    def select_slot(self, index: int) -> None:
        for card in self.slot_cards:
            card.set_selected(card.index == index)
        self.preview.set_slide(
            os.path.join(self.assets, "IMG%02d.BMP" % (index + 1))
        )
        self.status_labels["resolution"].setText(
            dimension_text(self._slot_resolution(index))
        )

    def _slot_resolution(self, index: int) -> QSize:
        if index not in self._resolution_cache:
            self._resolution_cache[index] = image_dimensions(
                os.path.join(self.assets, "IMG%02d.BMP" % (index + 1))
            )
        return self._resolution_cache[index]

    def _refresh_status(self) -> None:
        st = self._status
        if st is None:
            for v in self.status_labels.values():
                v.setText("--")
            self.cache_bar.setValue(0)
            return
        self.status_labels["version"].setText("v%02X" % st["ver"])
        self.status_labels["slot"].setText("IMG%02d" % (st["slot"] + 1))
        self.status_labels["resolution"].setText(
            dimension_text(self._slot_resolution(st["slot"]))
        )
        self.status_labels["mode"].setText("自动" if st["mode"] else "手动")
        total = min(st["total"] if st["total"] else SLOT_COUNT, SLOT_COUNT)
        cached = bin(st["cache"]).count("1")
        self.status_labels["cache"].setText("%d / %d" % (cached, total))
        self.cache_bar.setRange(0, total)
        self.cache_bar.setValue(min(cached, total))
        self.status_labels["textlen"].setText(str(st["textlen"]))
        card = self.status_labels["card"]
        card.setText("正常" if st["card_ok"] else "未就绪")
        card.setProperty("ok", "true" if st["card_ok"] else "false")
        card.setProperty("bad", "false" if st["card_ok"] else "true")
        card.style().unpolish(card)
        card.style().polish(card)
        self.preview.set_badge(
            "IMG%02d  %s  %s"
            % (st["slot"] + 1,
               "AUTO" if st["mode"] else "MANUAL",
               OSD_NAMES[min(st["osd"], 3)])
        )
        self.bright.set_value(st["bright"])
        self.contrast.set_value(st["contrast"])
        self.osd_seg.set_current(st["osd"])

    def _refresh_slots(self) -> None:
        cache = self._status["cache"] if self._status else 0x3F
        for i, card in enumerate(self.slot_cards):
            card.set_cached(bool(cache & (1 << i)))
        if self._status is None and self.connected:
            for card in self.slot_cards:
                card.set_cached(True)

    def load_demo(self) -> None:
        """Fill the console with plausible board state, for screenshots."""
        self.connected = True
        self._status = {
            "ver": 0x13, "slot": 2, "total": 6, "mode": 1,
            "bright": 7, "contrast": 3, "osd": 2, "cache": 0x37,
            "textlen": 28, "link_seen": 1, "card_ok": 1,
        }
        self._refresh_status()
        self._refresh_slots()
        self.select_slot(self._status["slot"])
        self._refresh_mini_live(self._status["slot"])
        if self.port_box.count() == 0:
            self.port_box.addItem("[USB] COM7 · CH340", "COM7")
        self.text_edit.setText(PRESETS[0])
        self.link_pill.set_state("online", "已连接")
        self.preview.set_live(True)
        self.log_line("info", "控制台就绪，6 个画面槽位已探测")
        self.log_line("tx", "A5 5A 10 00 10")
        self.log_line("rx", "90  11 02 06 01 07 03 02 37 1C 03")
        self.log_line("tx", "A5 5A 02 1C 43 41 31 38 33 32 20 ... 4E 47 26")
        self.log_line("rx", "82  1C 1C 6E")
        self.log_line("info", "人声播报连续播放，切图不影响音频流")

    # -- log ---------------------------------------------------------------
    def log_line(self, level: str, text: str) -> None:
        colors = {
            "tx": "#A86418",
            "rx": "#137A59",
            "err": "#B62D35",
            "ok": "#137A59",
            "info": "#66788A",
        }
        stamps = {
            "tx": "TX",
            "rx": "RX",
            "err": "ERR",
            "ok": "OK",
            "info": "..",
        }
        stamp = time.strftime("%H:%M:%S")
        color = colors.get(level, "#66788A")
        tag = stamps.get(level, "..")
        if level in self._counts:
            self._counts[level] += 1
            chip = self.chips[level]
            chip.setText("%s %d" % (level.upper(), self._counts[level]))
            chip.setProperty("on", "true")
            chip.style().unpolish(chip)
            chip.style().polish(chip)
        self.log_view.appendHtml(
            '<span style="color:#8A98A6">%s</span> '
            '<span style="color:%s">%s</span>&nbsp;&nbsp;'
            '<span style="color:#263545">%s</span>'
            % (stamp, color, tag, text)
        )
        self.log_view.moveCursor(QTextCursor.End)

    def show_hint(self, text: str) -> None:
        self.hint.setText(text)
        self._toast.stop()
        if text:
            self._toast.start(4000)

    def closeEvent(self, event) -> None:                # noqa: N802
        self.disconnect_board()
        event.accept()


PRESETS = [
    "CA1832 GATE A12 NOW BOARDING",
    "MU5107 GATE B07 DELAYED 20 MIN",
    "CZ3456 FINAL CALL GATE C03",
    "PASSENGERS PROCEED TO GATE D05",
    "BAGGAGE CLAIM BELT 6",
    "WELCOME TO HONGQIAO AIRPORT",
]


QSS = """
QWidget {
    color: #263545;
    font-family: "Microsoft YaHei UI", "Segoe UI", sans-serif;
    font-size: 15px;
}
QWidget#root {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                stop:0 #F7FAFD, stop:1 #E8EFF6);
}
QScrollArea#leftScroll, QScrollArea#rightScroll {
    background: transparent;
    border: none;
}
QScrollArea#leftScroll > QWidget > QWidget,
QScrollArea#rightScroll > QWidget > QWidget {
    background: transparent;
}
QLabel { background: transparent; }
QFrame#header {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #FFFFFF, stop:1 #EDF3FA);
    border: 1px solid #D4DEE9;
    border-radius: 8px;
}
QFrame#accent {
    background: #D88B22;
    border: none;
    border-radius: 2px;
}
QLabel#appTitle {
    font-size: 23px;
    font-weight: 600;
    color: #15202B;
}
QLabel#appSub {
    font-size: 14px;
    color: #607080;
}
QLabel#brandChip {
    background: #EDF4FA;
    border: 1px solid #CDD9E6;
    border-radius: 11px;
    color: #4B6275;
    font-family: Consolas, monospace;
    font-size: 13px;
    padding: 0 10px;
}
QLabel#clock {
    color: #5A6B7C;
    font-family: Consolas, monospace;
    font-size: 14px;
}
QLabel#chip {
    background: #F3F6FA;
    border: 1px solid #D6E0EA;
    border-radius: 6px;
    color: #6E7E8F;
    font-family: Consolas, monospace;
    font-size: 14px;
    padding: 0 10px;
}
QLabel#chip[on="true"][kind="tx"] {
    background: #FFF3DD;
    border: 1px solid #F0C77B;
    color: #9A6518;
}
QLabel#chip[on="true"][kind="rx"] {
    background: #E7F7F0;
    border: 1px solid #9AD9C0;
    color: #137A59;
}
QLabel#chip[on="true"][kind="err"] {
    background: #FDECEC;
    border: 1px solid #F0B1B1;
    color: #B62D35;
}
QLabel#pill {
    border-radius: 8px;
    padding: 4px 14px;
    font-weight: 600;
    border: 1px solid #D4DEE9;
    background: #F3F6FA;
    color: #5C6E80;
}
QLabel#pill[state="online"] {
    background: #E7F7F0;
    border: 1px solid #9AD9C0;
    color: #137A59;
}
QLabel#pill[state="error"] {
    background: #FDECEC;
    border: 1px solid #F0B1B1;
    color: #B62D35;
}
QFrame#panel {
    background: #FFFFFF;
    border: 1px solid #D8E1EB;
    border-radius: 8px;
}
QLabel#panelTitle {
    font-size: 14px;
    font-weight: 600;
    color: #2C3B4A;
}
QLabel#rowLabel { color: #5B6C7E; }
QLabel#rowValue {
    color: #1F2A37;
    font-family: Consolas, monospace;
}
QLabel#rowValue[ok="true"] { color: #137A59; }
QLabel#rowValue[bad="true"] { color: #B62D35; }
QLabel#caption { color: #718092; font-size: 14px; }
QLabel#hint { color: #A86418; font-size: 14px; }
QLabel#counter {
    color: #65788A;
    font-family: Consolas, monospace;
    padding: 2px 8px;
}
QLabel#counter[full="true"] { color: #C23A42; }
QPushButton {
    background: #FFFFFF;
    border: 1px solid #C9D5E2;
    border-radius: 6px;
    padding: 7px 14px;
    color: #263545;
}
QPushButton:hover { background: #F2F6FB; border-color: #9FB4C8; }
QPushButton:pressed { background: #E6EDF5; }
QPushButton#primary {
    background: #E8A33D;
    border: 1px solid #D4932F;
    color: #2A1A05;
    font-weight: 600;
}
QPushButton#primary:hover { background: #F2B255; border-color: #E8A33D; }
QPushButton#primary:pressed { background: #CF8F31; }
QPushButton#danger {
    background: #FDECEC;
    border: 1px solid #E8A9A9;
    color: #B62D35;
    font-weight: 600;
}
QPushButton#danger:hover { background: #FADEDE; }
QPushButton#ghost {
    background: transparent;
    border: 1px solid #C9D5E2;
    color: #5A6C7D;
}
QPushButton#ghost:hover { background: #F0F5FA; color: #263545; }
QPushButton#seg {
    background: #F3F6FA;
    border: 1px solid #D6E0EA;
    border-radius: 6px;
    padding: 7px 6px;
    color: #66788A;
}
QPushButton#seg:hover { background: #E8EFF7; color: #314355; }
QPushButton#seg:checked {
    background: #FFFFFF;
    border: 1px solid #E8A33D;
    color: #8B5A14;
    font-weight: 600;
}
QToolButton#iconBtn {
    background: #FFFFFF;
    border: 1px solid #C9D5E2;
    border-radius: 6px;
    padding: 6px;
}
QToolButton#iconBtn:hover { background: #F0F5FA; }
QFrame#slot {
    background: #F7FAFD;
    border: 1px solid #D6E0EA;
    border-radius: 8px;
}
QFrame#slot:hover { border-color: #9FB4C8; background: #EEF4FA; }
QFrame#slot[selected="true"] {
    border: 1px solid #E8A33D;
    background: #FFF8EC;
}
QFrame#slot[cached="false"] {
    background: #F1F4F8;
    border-color: #E1E7EE;
}
QFrame#slot[cached="false"]:hover { border-color: #E1E7EE; }
QLabel#thumb {
    background: #E9EFF6;
    border: 1px solid #CDD9E6;
    border-radius: 5px;
}
QFrame#slot:hover QLabel#thumb { border-color: #9FB4C8; }
QFrame#slot[selected="true"] QLabel#thumb { border-color: #E8A33D; }
QLabel#slotBadge {
    background: #E8A33D;
    color: #2A1A05;
    border-radius: 7px;
    padding: 1px 7px;
    font-size: 12px;
    font-weight: 600;
}
QLabel#slotCaption {
    color: #5A6C7D;
    font-family: Consolas, monospace;
    font-size: 14px;
}
QFrame#slot[selected="true"] QLabel#slotCaption { color: #9A6518; }
QFrame#slot[cached="false"] QLabel#slotCaption { color: #98A5B2; }
QLineEdit#ticker {
    background: #FFFFFF;
    border: 1px solid #C9D5E2;
    border-radius: 6px;
    padding: 8px 12px;
    color: #A86418;
    font-family: Consolas, "Courier New", monospace;
    font-size: 18px;
    letter-spacing: 1px;
}
QLineEdit#ticker:focus { border-color: #E8A33D; }
QLabel#boardRuler {
    color: #66788A;
    font-family: Consolas, "Courier New", monospace;
    font-size: 13px;
    padding-left: 28px;
}
QLabel#boardIndex {
    background: #E8F5FC;
    border: 1px solid #B8DDEE;
    border-radius: 4px;
    color: #1679A5;
    font-family: Consolas, monospace;
    font-size: 12px;
    font-weight: 600;
}
QLineEdit#boardRow {
    background: #FFFFFF;
    border: 1px solid #C9D5E2;
    border-radius: 5px;
    padding: 5px 8px;
    color: #20303F;
    font-family: Consolas, "Courier New", monospace;
    font-size: 14px;
    letter-spacing: 0px;
}
QLineEdit#boardRow:focus {
    border-color: #1684B4;
    background: #F8FCFF;
}
QPlainTextEdit#boardRows {
    background: #FFFFFF;
    border: 1px solid #C9D5E2;
    border-radius: 5px;
    /* Right padding leaves room for the vertical bar: without it the 40-column
       rows push the document one pixel over and a bar appears on every edit. */
    padding: 3px 14px 3px 8px;
    color: #20303F;
    font-family: Consolas, "Courier New", monospace;
    font-size: 15px;
    selection-background-color: #D7E8F5;
}
QPlainTextEdit#boardRows:focus {
    border-color: #1684B4;
    background: #F8FCFF;
}
QPlainTextEdit#boardRows QScrollBar:vertical {
    background: transparent;
    width: 8px;
}
QPlainTextEdit#boardRows QScrollBar::handle:vertical {
    background: #D3DDE8;
    border-radius: 4px;
    min-height: 20px;
}
QFrame#miniBoard {
    background: #FFFFFF;
    border: 1px solid #C9D5E2;
    border-radius: 6px;
}
QFrame#miniBoard:hover { border-color: #E8A33D; }
QFrame#miniBoard[live="true"] {
    background: #FFF8EB;
    border: 1px solid #E8A33D;
}
QLabel#miniHead {
    color: #7A8CA0;
    font-size: 13px;
    font-weight: 600;
}
QFrame#miniBoard[live="true"] QLabel#miniHead { color: #A86418; }
QLabel#miniBody {
    color: #26394B;
    font-family: Consolas, "Courier New", monospace;
    font-size: 13px;
}
QCheckBox {
    color: #46586A;
    spacing: 6px;
}
QCheckBox::indicator {
    width: 15px;
    height: 15px;
    border: 1px solid #B9C7D6;
    border-radius: 4px;
    background: #FFFFFF;
}
QCheckBox::indicator:hover { border-color: #1684B4; }
QCheckBox::indicator:checked {
    background: #1684B4;
    border: 1px solid #1684B4;
    image: none;
}
QComboBox {
    background: #FFFFFF;
    border: 1px solid #C9D5E2;
    border-radius: 6px;
    padding: 6px 10px;
    min-height: 20px;
}
QComboBox:hover { border-color: #9FB4C8; }
QComboBox::drop-down { border: none; width: 22px; }
QComboBox QAbstractItemView {
    background: #FFFFFF;
    border: 1px solid #C9D5E2;
    selection-background-color: #E7F1FA;
    outline: none;
}
QSlider::groove:horizontal {
    height: 5px;
    background: #D9E2EC;
    border-radius: 3px;
}
QSlider::sub-page:horizontal {
    background: #E8A33D;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #FFFFFF;
    border: 1px solid #AFC0D2;
    width: 14px;
    margin: -6px 0;
    border-radius: 7px;
}
QSlider::handle:horizontal:hover { background: #F7FAFD; }
QLabel#readout {
    background: #F3F6FA;
    border: 1px solid #D6E0EA;
    border-radius: 4px;
    color: #9A6518;
    font-family: Consolas, monospace;
    font-weight: 600;
}
QProgressBar#cacheBar {
    background: #EFF3F8;
    border: 1px solid #D6E0EA;
    border-radius: 6px;
    color: #5A6C7D;
    font-family: Consolas, monospace;
    font-size: 13px;
    text-align: center;
}
QProgressBar#cacheBar::chunk {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #CF8F31, stop:1 #F2B255);
    border-radius: 5px;
}
QLineEdit#ticker:focus { selection-background-color: #FFE9C2; }
QPlainTextEdit#log {
    background: #F7FAFD;
    border: 1px solid #D6E0EA;
    border-radius: 6px;
    color: #263545;
    font-family: Consolas, "Courier New", monospace;
    font-size: 14px;
    padding: 6px;
}
QScrollBar:vertical {
    background: #EDF2F7; width: 12px; margin: 0;
}
QScrollBar::handle:vertical {
    background: #AEBECD; border-radius: 6px; min-height: 28px;
}
QScrollBar::handle:vertical:hover { background: #8FA5B8; }
QScrollBar::add-line, QScrollBar::sub-line { height: 0; }
QScrollBar:horizontal {
    background: #EDF2F7; height: 10px; margin: 0;
}
QScrollBar::handle:horizontal {
    background: #C5D1DE; border-radius: 5px; min-width: 24px;
}
QScrollBar::handle:horizontal:hover { background: #AEBECD; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
QPlainTextEdit#log { selection-background-color: #D7E8F5; }
QToolTip {
    background: #FFFFFF;
    color: #263545;
    border: 1px solid #C9D5E2;
    padding: 4px 6px;
}
"""


def frozen_root() -> str:
    """Home folder of the frozen build: where the .exe actually lives."""
    if getattr(sys, "frozen", False):
        return os.path.dirname(os.path.abspath(sys.executable))
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, ".."))


def default_assets() -> str:
    """An editable sdcard folder next to the exe wins over the bundled copy."""
    beside = os.path.join(frozen_root(), "sdcard")
    if os.path.isdir(beside):
        return beside
    if getattr(sys, "frozen", False):
        bundled = os.path.join(getattr(sys, "_MEIPASS", ""), "sdcard")
        if os.path.isdir(bundled):
            return bundled
    return beside


def selftest() -> int:
    """Byte level vectors, shared with sim/tb_uart_ctrl.v.

    Keeping both sides on the same literals is what stops the host and the
    board from drifting apart between builds.
    """
    fails: list[str] = []

    def eq(name: str, got, want) -> None:
        if got != want:
            fails.append("%s\n     got  %r\n     want %r" % (name, got, want))

    # ---- frames the host sends -----------------------------------------
    eq("PING", build_frame(CMD_PING), bytes.fromhex("A55A0100 01".replace(" ", "")))
    eq("STATUS", build_frame(CMD_STATUS),
       bytes.fromhex("A55A1000 10".replace(" ", "")))
    eq("SLOT 3", build_frame(CMD_SLOT, bytes([3])),
       bytes.fromhex("A55A0301 03 01".replace(" ", "")))
    eq("MODE auto", build_frame(CMD_MODE, bytes([1])),
       bytes.fromhex("A55A0401 01 04".replace(" ", "")))
    eq("PARAM bright 5", build_frame(CMD_PARAM, bytes([0, 5])),
       bytes.fromhex("A55A0502 0005 02".replace(" ", "")))
    eq("OSD 3", build_frame(CMD_OSD, bytes([3])),
       bytes.fromhex("A55A0601 03 04".replace(" ", "")))
    eq("BAUD 0", build_frame(CMD_BAUD, bytes([0])),
       bytes.fromhex("A55A0701 00 06".replace(" ", "")))
    eq("TEXT empty", build_frame(CMD_TEXT, b""),
       bytes.fromhex("A55A0200 02".replace(" ", "")))
    eq("TEXT GATE 3", build_frame(CMD_TEXT, b"GATE 3"),
       bytes.fromhex("A55A0206") + b"GATE 3" + bytes([0x00]))
    eq("BOARD row 2", build_frame(CMD_BOARD, bytes([2]) + b"A" * BOARD_COLS),
       bytes.fromhex("A55A2029 02".replace(" ", ""))
       + b"A" * BOARD_COLS + bytes([0x0B]))
    eq("BOARD row 5", build_frame(CMD_BOARD, bytes([5]) + b"B" * BOARD_COLS),
       bytes.fromhex("A55A2029 05".replace(" ", ""))
       + b"B" * BOARD_COLS + bytes([0x0C]))
    eq("BVIS on", build_frame(CMD_BVIS, bytes([1])),
       bytes.fromhex("A55A21010121"))
    eq("BVIS off", build_frame(CMD_BVIS, bytes([0])),
       bytes.fromhex("A55A21010020"))
    eq("TIME 12:34:56", build_frame(CMD_TIME, bytes([0x12, 0x34, 0x56])),
       bytes.fromhex("A55A220312345651"))
    eq("BCD decode", [bcd_to_int(0x00), bcd_to_int(0x09),
                      bcd_to_int(0x12), bcd_to_int(0x59)],
       [0, 9, 12, 59])

    # ---- replies the board sends ---------------------------------------
    ping = bytes.fromhex("A55A81011191")
    got = FrameParser().feed(ping)
    eq("PING reply", got, [(R_PING, bytes([0x11]))])

    status = bytes.fromhex("A55A900A110106010703002A0603A6")
    frames = FrameParser().feed(status)
    eq("STATUS accepted", len(frames), 1)
    if frames:
        _, pay = frames[0]
        eq("STATUS payload", pay.hex().upper(), "110106010703002A0603")
        eq("STATUS fields",
           dict(ver=pay[0], slot=pay[1], total=pay[2], mode=pay[3] & 1,
                bright=pay[4] & 7, contrast=pay[5] & 7, osd=pay[6] & 3,
                cache=pay[7] & 0x3F, textlen=pay[8], card_ok=(pay[9] >> 1) & 1),
           dict(ver=0x11, slot=1, total=6, mode=1, bright=7, contrast=3,
                osd=0, cache=0x2A, textlen=6, card_ok=1))

    err = bytes.fromhex("A55AE0020303E2")
    eq("error reply", FrameParser().feed(err), [(R_ERROR, bytes([3, 3]))])

    time_reply = bytes.fromhex("A55AA203123456D1")
    eq("TIME reply", FrameParser().feed(time_reply),
       [(R_TIME, bytes([0x12, 0x34, 0x56]))])

    # a reply split across reads must still assemble
    p = FrameParser()
    acc = []
    for byte in ping:
        acc += p.feed(bytes([byte]))
    eq("split reply", acc, [(R_PING, bytes([0x11]))])

    # leading junk and a corrupt checksum must both be survivable
    p = FrameParser()
    eq("resync", p.feed(bytes([0x00, 0x13]) + ping), [(R_PING, bytes([0x11]))])
    p = FrameParser()
    eq("bad checksum dropped", p.feed(ping[:-1] + bytes([0x00])), [])
    eq("bad checksum counted", p.bad, 1)

    # ---- text field -----------------------------------------------------
    eq("fold", sanitize_text("gate 3"), ("GATE 3", 0))
    eq("drop", sanitize_text("登机口 A"), (" A", 3))
    eq("clamp", sanitize_text("A" * 40), ("A" * TEXT_MAX, 0))
    row, dropped = sanitize_board_row("CA1832 GATE A12")
    eq("board pad", (len(row), dropped), (BOARD_COLS, 0))
    eq("board fold", sanitize_board_row("gate 3"), ("GATE 3" + " " * 34, 0))
    eq("board rows", BOARD_ROWS, 6)
    eq("board template width", [len(line) for line in BOARD_DEFAULT],
       [BOARD_COLS] * BOARD_ROWS)
    eq("one board page per slot", PAGE_FOR_SLOT, tuple(range(SLOT_COUNT)))
    eq("board page shapes",
       [(len(rows), sorted({len(r) for r in rows})) for _n, rows in BOARD_PAGES],
       [(BOARD_ROWS, [BOARD_COLS])] * SLOT_COUNT)

    # ---- serial transport classification --------------------------------
    class FakePort:
        def __init__(self, device: str, description: str, hwid: str) -> None:
            self.device = device
            self.description = description
            self.manufacturer = ""
            self.hwid = hwid

    eq("Bluetooth SPP port",
       classify_serial_port(
           FakePort("COM5", "Standard Serial over Bluetooth link", "BTHENUM")
       ),
       PORT_KIND_BLUETOOTH)
    eq("CH340 USB port",
       classify_serial_port(
           FakePort("COM7", "USB-SERIAL CH340", "USB VID:PID=1A86:7523")
       ),
       PORT_KIND_USB)
    eq("plain serial port",
       classify_serial_port(
           FakePort("COM9", "Communications Port", "ACPI")
       ),
       PORT_KIND_SERIAL)

    if fails:
        print("FAIL: host selftest")
        for f in fails:
            print("  - " + f)
        return 1
    print("PASS: host selftest (%d vectors)" % 9)
    return 0


def main(argv: list[str] | None = None) -> int:
    if sys.stdout is None:                  # windowed builds have no console
        sys.stdout = open(os.devnull, "w", encoding="utf-8")
    if sys.stderr is None:
        sys.stderr = open(os.devnull, "w", encoding="utf-8")
    ap = argparse.ArgumentParser(description="HDMI player serial console")
    ap.add_argument("--port", default="", help="serial port, e.g. COM7")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--bluetooth", action="store_true",
                    help="start with the Bluetooth SPP port filter selected")
    ap.add_argument("--assets", default=default_assets(),
                    help="folder holding IMG01.BMP .. IMG06.BMP")
    ap.add_argument("--shot", default="", help="render to PNG and exit")
    ap.add_argument("--demo", action="store_true",
                    help="seed sample board state before rendering")
    ap.add_argument("--selftest", action="store_true",
                    help="check the frame format against the RTL vectors")
    args = ap.parse_args(argv)

    if args.selftest:
        return selftest()

    # Windows commonly runs the desktop at 125% or 150%.  Without this the
    # Qt app treats every pixel as a physical pixel and the console looks tiny
    # on high-DPI screens even though the monitor itself is large.
    if hasattr(Qt, "AA_EnableHighDpiScaling"):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, "AA_UseHighDpiPixmaps"):
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setStyleSheet(QSS)
    app.setWindowIcon(QIcon(brand_pixmap(64)))
    win = Console(args.assets, args.port, args.baud)
    if args.bluetooth:
        win.link_seg.set_current(1)
        win._on_link_kind(1)
    if args.demo:
        win.load_demo()
    if args.shot:
        # park it outside the desktop so the grab uses the real font stack
        win.move(-4000, -3000)
    win.show()

    if args.shot:
        QTimer.singleShot(
            600,
            lambda: (
                win.grab().save(args.shot),
                app.quit(),
            ),
        )
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
