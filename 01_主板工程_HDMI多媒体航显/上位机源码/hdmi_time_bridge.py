# -*- coding: utf-8 -*-
"""Standalone HDMI real-time bridge.

This program is intentionally separate from the existing upper-computer
applications.  It reads the E104 RTC from its Bluetooth SPP port when that
port is free.  If the E104 console already owns the port, it falls back to
the PC clock so the HDMI OSD can still show real time.
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from typing import Iterable, Optional, Tuple

try:
    import serial
    from serial.tools import list_ports
except Exception:  # pragma: no cover - handled in the UI
    serial = None
    list_ports = None

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


HDR1 = 0xA5
HDR2 = 0x5A
CMD_STATUS = 0x10
R_STATUS = 0x90
CMD_TIME = 0x22
R_TIME = 0xA2
R_ERROR = 0xE0

DEFAULT_RTC_PORT = "COM11"
DEFAULT_FPGA_PORT = "COM6"
DEFAULT_RTC_BAUD = 9600
DEFAULT_FPGA_BAUD = 115200


class FrameParser:
    """Incremental A5 5A frame parser used by both serial paths."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data: bytes) -> list[Tuple[int, bytes]]:
        self.buf.extend(data)
        frames: list[Tuple[int, bytes]] = []
        while True:
            while len(self.buf) >= 2 and not (
                self.buf[0] == HDR1 and self.buf[1] == HDR2
            ):
                self.buf.pop(0)
            if len(self.buf) < 4:
                break
            length = self.buf[3]
            total = 5 + length
            if len(self.buf) < total:
                break
            raw = bytes(self.buf[:total])
            del self.buf[:total]
            checksum = 0
            for byte in raw[2 : 4 + length]:
                checksum ^= byte
            if checksum == raw[4 + length]:
                frames.append((raw[2], raw[4 : 4 + length]))
            else:
                self.bad += 1
        return frames


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    body = bytes((cmd & 0xFF, len(payload) & 0xFF)) + bytes(payload)
    checksum = 0
    for byte in body:
        checksum ^= byte
    return bytes((HDR1, HDR2)) + body + bytes((checksum,))


def _bcd_to_int(value: int) -> Optional[int]:
    if value & 0xF0 > 0x90 or value & 0x0F > 0x09:
        return None
    return ((value >> 4) & 0x0F) * 10 + (value & 0x0F)


def _int_to_bcd(value: int) -> int:
    return ((value // 10) << 4) | (value % 10)


def valid_bcd_time(hh: int, mm: int, ss: int) -> bool:
    h = _bcd_to_int(hh)
    m = _bcd_to_int(mm)
    s = _bcd_to_int(ss)
    return h is not None and m is not None and s is not None and (
        0 <= h <= 23 and 0 <= m <= 59 and 0 <= s <= 59
    )


def bcd_from_datetime(now: datetime) -> Tuple[int, int, int]:
    return (
        _int_to_bcd(now.hour),
        _int_to_bcd(now.minute),
        _int_to_bcd(now.second),
    )


def format_bcd_time(hh: int, mm: int, ss: int) -> str:
    h = _bcd_to_int(hh)
    m = _bcd_to_int(mm)
    s = _bcd_to_int(ss)
    if h is None or m is None or s is None:
        return "--:--:--"
    return "%02d:%02d:%02d" % (h, m, s)


class RtcReader:
    """Reads the E104 status frame and extracts the DS1307 RTC."""

    def __init__(self, port: str, baud: int = DEFAULT_RTC_BAUD) -> None:
        self.port_name = port
        self.baud = baud
        self.port = None

    def close(self) -> None:
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
            self.port = None

    def open(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self.close()
        self.port = serial.Serial(
            self.port_name,
            self.baud,
            timeout=0.04,
            write_timeout=0.35,
        )
        # HC-06 SPP needs a short settle time after the Windows COM handle opens.
        time.sleep(0.20)
        self.port.reset_input_buffer()

    def read_time(self, timeout: float = 1.0) -> Tuple[int, int, int]:
        if self.port is None:
            self.open()
        self.port.reset_input_buffer()
        self.port.write(build_frame(CMD_STATUS))
        parser = FrameParser()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            chunk = self.port.read(256)
            for cmd, payload in parser.feed(chunk or b""):
                if cmd != R_STATUS or len(payload) < 13:
                    continue
                if not (payload[7] & 0x02):
                    continue
                hh, mm, ss = payload[10], payload[11], payload[12]
                if valid_bcd_time(hh, mm, ss):
                    return hh, mm, ss
        raise TimeoutError("E104 RTC timeout")


class FpgaSender:
    """Sends the HDMI player CMD_TIME frame over the CH340 UART."""

    def __init__(self, port: str, baud: int = DEFAULT_FPGA_BAUD) -> None:
        self.port_name = port
        self.baud = baud
        self.port = None

    def close(self) -> None:
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
            self.port = None

    def open(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self.close()
        self.port = serial.Serial(
            self.port_name,
            self.baud,
            timeout=0.04,
            write_timeout=0.35,
        )
        self.port.reset_input_buffer()

    def query_status(self) -> Optional[bytes]:
        if self.port is None:
            self.open()
        self.port.reset_input_buffer()
        self.port.write(build_frame(CMD_STATUS))
        parser = FrameParser()
        deadline = time.monotonic() + 0.45
        while time.monotonic() < deadline:
            chunk = self.port.read(256)
            for cmd, payload in parser.feed(chunk or b""):
                if cmd == R_STATUS:
                    return payload
        return None

    def send_time(self, hh: int, mm: int, ss: int) -> str:
        if self.port is None:
            self.open()
        self.port.reset_input_buffer()
        self.port.write(build_frame(CMD_TIME, bytes((hh, mm, ss))))
        parser = FrameParser()
        deadline = time.monotonic() + 0.35
        while time.monotonic() < deadline:
            chunk = self.port.read(256)
            for cmd, payload in parser.feed(chunk or b""):
                if cmd == R_TIME:
                    return "ok"
                if cmd == R_ERROR:
                    if len(payload) >= 1 and payload[0] == CMD_TIME:
                        return "unsupported"
                    return "error"
        return "sent"


def _available_ports() -> list[str]:
    if list_ports is None:
        return []
    return [item.device for item in list_ports.comports()]


class TimeBridgeWindow(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("HDMI 时间同步")
        self.setMinimumSize(560, 430)

        self.rtc_reader: Optional[RtcReader] = None
        self.fpga_sender: Optional[FpgaSender] = None
        self.running = False
        self.rtc_retry_at = 0.0
        self.last_rtc_error = ""
        self.last_time: Optional[Tuple[int, int, int]] = None

        self.timer = QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self._tick)

        self._build_ui()
        self._refresh_ports()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(18, 16, 18, 16)
        root.setSpacing(12)

        title = QLabel("HDMI 时间同步")
        title.setObjectName("title")
        subtitle = QLabel("独立运行，不修改现有上位机")
        subtitle.setObjectName("subtitle")
        root.addWidget(title)
        root.addWidget(subtitle)

        port_row = QHBoxLayout()
        port_row.setSpacing(8)
        port_row.addWidget(QLabel("蓝牙 RTC"))
        self.rtc_box = QComboBox()
        self.rtc_box.setMinimumWidth(150)
        port_row.addWidget(self.rtc_box, 1)
        port_row.addWidget(QLabel("HDMI 串口"))
        self.fpga_box = QComboBox()
        self.fpga_box.setMinimumWidth(150)
        port_row.addWidget(self.fpga_box, 1)
        self.refresh_button = QPushButton("刷新")
        self.refresh_button.clicked.connect(self._refresh_ports)
        port_row.addWidget(self.refresh_button)
        root.addLayout(port_row)

        source_row = QHBoxLayout()
        source_row.setSpacing(8)
        source_row.addWidget(QLabel("时间来源"))
        self.source_box = QComboBox()
        self.source_box.addItem("自动（优先蓝牙 RTC）", "auto")
        self.source_box.addItem("电脑时间", "pc")
        self.source_box.addItem("仅蓝牙 RTC", "rtc")
        self.source_box.currentIndexChanged.connect(self._on_source_changed)
        source_row.addWidget(self.source_box, 1)
        root.addLayout(source_row)

        self.time_label = QLabel("--:--:--")
        self.time_label.setObjectName("clock")
        self.time_label.setAlignment(Qt.AlignCenter)
        root.addWidget(self.time_label)

        self.status_label = QLabel("未启动")
        self.status_label.setObjectName("status")
        self.status_label.setAlignment(Qt.AlignCenter)
        root.addWidget(self.status_label)

        button_row = QHBoxLayout()
        button_row.addStretch(1)
        self.start_button = QPushButton("启动")
        self.start_button.clicked.connect(self.start)
        self.stop_button = QPushButton("停止")
        self.stop_button.clicked.connect(self.stop)
        self.stop_button.setEnabled(False)
        button_row.addWidget(self.start_button)
        button_row.addWidget(self.stop_button)
        root.addLayout(button_row)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(120)
        self.log.setObjectName("log")
        root.addWidget(self.log, 1)

        self.setStyleSheet(
            """
            QWidget { background: #F4F7FA; color: #203040; font-size: 13px; }
            QLabel#title { font-size: 22px; font-weight: 700; color: #12324A; }
            QLabel#subtitle { color: #65788A; }
            QLabel#clock {
                font-size: 38px; font-weight: 700; color: #0B5B78;
                background: #FFFFFF; border: 1px solid #D6E1EA;
                border-radius: 8px; padding: 12px;
            }
            QLabel#status { color: #3C6176; padding: 4px; }
            QComboBox, QPlainTextEdit {
                background: #FFFFFF; border: 1px solid #D6E1EA;
                border-radius: 6px; padding: 5px;
            }
            QPushButton {
                background: #FFFFFF; border: 1px solid #C8D6E2;
                border-radius: 6px; padding: 7px 14px;
            }
            QPushButton:hover { background: #EAF3F8; }
            QPushButton:disabled { color: #9AA8B4; background: #EEF2F5; }
            """
        )

    def _refresh_ports(self) -> None:
        ports = _available_ports()
        for box, preferred in (
            (self.rtc_box, DEFAULT_RTC_PORT),
            (self.fpga_box, DEFAULT_FPGA_PORT),
        ):
            current = box.currentData() or box.currentText()
            box.clear()
            for port in ports:
                box.addItem(port, port)
            if preferred in ports:
                box.setCurrentIndex(ports.index(preferred))
            elif current in ports:
                box.setCurrentIndex(ports.index(current))
            elif ports:
                box.setCurrentIndex(0)

    def _on_source_changed(self) -> None:
        if self.running:
            self._prepare_rtc()

    def _selected_rtc(self) -> str:
        return self.rtc_box.currentData() or self.rtc_box.currentText().strip()

    def _selected_fpga(self) -> str:
        return self.fpga_box.currentData() or self.fpga_box.currentText().strip()

    def _source_mode(self) -> str:
        return self.source_box.currentData() or "auto"

    def _log(self, text: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log.appendPlainText("[%s] %s" % (stamp, text))

    def _prepare_rtc(self) -> None:
        mode = self._source_mode()
        if self.rtc_reader is not None:
            self.rtc_reader.close()
        self.rtc_reader = None
        self.last_rtc_error = ""
        if mode == "pc":
            return
        port = self._selected_rtc()
        if not port:
            self.last_rtc_error = "没有选择蓝牙 RTC 串口"
            return
        reader = RtcReader(port, DEFAULT_RTC_BAUD)
        try:
            reader.open()
            self.rtc_reader = reader
            self._log("蓝牙 RTC 已连接：%s" % port)
        except Exception as exc:
            reader.close()
            self.last_rtc_error = str(exc)
            self._log("蓝牙 RTC 暂不可用：%s" % exc)

    def _read_time(self) -> Tuple[Tuple[int, int, int], str]:
        mode = self._source_mode()
        if mode != "pc" and self.rtc_reader is not None:
            try:
                value = self.rtc_reader.read_time()
                return value, "蓝牙 RTC"
            except Exception as exc:
                self.last_rtc_error = str(exc)
                self.rtc_reader.close()
                self.rtc_reader = None
                self._log("蓝牙 RTC 读取失败，切换电脑时间：%s" % exc)

        if mode in ("auto", "rtc") and self.rtc_reader is None:
            now = time.monotonic()
            if now >= self.rtc_retry_at:
                self.rtc_retry_at = now + 8.0
                self._prepare_rtc()
                if self.rtc_reader is not None:
                    try:
                        value = self.rtc_reader.read_time()
                        return value, "蓝牙 RTC"
                    except Exception as exc:
                        self.last_rtc_error = str(exc)
                        self.rtc_reader.close()
                        self.rtc_reader = None

        if mode == "rtc":
            raise RuntimeError(self.last_rtc_error or "蓝牙 RTC 不可用")
        return bcd_from_datetime(datetime.now()), "电脑时间"

    def start(self) -> None:
        if serial is None:
            self._log("pyserial 未安装，无法启动")
            return
        port = self._selected_fpga()
        if not port:
            self._log("没有选择 HDMI 串口")
            return
        try:
            self.fpga_sender = FpgaSender(port, DEFAULT_FPGA_BAUD)
            self.fpga_sender.open()
            version = self.fpga_sender.query_status()
            version_text = "--"
            if version:
                version_text = "0x%02X" % version[0]
            self._log("HDMI 串口已连接：%s，版本 %s" % (port, version_text))
        except Exception as exc:
            if self.fpga_sender is not None:
                self.fpga_sender.close()
            self.fpga_sender = None
            self._log("HDMI 串口打开失败：%s" % exc)
            return

        self._prepare_rtc()
        self.running = True
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.rtc_box.setEnabled(False)
        self.fpga_box.setEnabled(False)
        self.source_box.setEnabled(False)
        self.refresh_button.setEnabled(False)
        self.status_label.setText("正在同步")
        self.timer.start()
        self._tick()

    def stop(self) -> None:
        self.timer.stop()
        self.running = False
        if self.rtc_reader is not None:
            self.rtc_reader.close()
            self.rtc_reader = None
        if self.fpga_sender is not None:
            self.fpga_sender.close()
            self.fpga_sender = None
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.rtc_box.setEnabled(True)
        self.fpga_box.setEnabled(True)
        self.source_box.setEnabled(True)
        self.refresh_button.setEnabled(True)
        self.status_label.setText("已停止")
        self.time_label.setText("--:--:--")
        self._log("已停止")

    def _tick(self) -> None:
        if not self.running or self.fpga_sender is None:
            return
        try:
            value, source = self._read_time()
            hh, mm, ss = value
            if not valid_bcd_time(hh, mm, ss):
                raise RuntimeError("时间数据无效")
            result = self.fpga_sender.send_time(hh, mm, ss)
            self.last_time = value
            self.time_label.setText(format_bcd_time(hh, mm, ss))
            if result == "ok":
                self.status_label.setText("已同步 · %s · HDMI 已确认" % source)
            elif result == "unsupported":
                self.status_label.setText("HDMI 固件不支持时间命令，请烧录 ADV18")
            elif result == "sent":
                self.status_label.setText("已发送 · %s · 等待 HDMI 确认" % source)
            else:
                self.status_label.setText("发送失败 · %s" % source)
        except Exception as exc:
            self.status_label.setText("同步失败：%s" % exc)

    def closeEvent(self, event) -> None:  # noqa: N802
        self.stop()
        event.accept()


def _selftest() -> int:
    frame = build_frame(CMD_STATUS)
    assert frame == bytes((0xA5, 0x5A, 0x10, 0x00, 0x10))
    parser = FrameParser()
    assert parser.feed(frame + b"\x00\x01") == [(0x10, b"")]
    time_frame = build_frame(CMD_TIME, bytes((0x12, 0x34, 0x56)))
    parser = FrameParser()
    assert parser.feed(time_frame) == [(CMD_TIME, bytes((0x12, 0x34, 0x56)))]
    assert valid_bcd_time(0x23, 0x59, 0x59)
    assert not valid_bcd_time(0x24, 0x00, 0x00)
    assert format_bcd_time(0x12, 0x34, 0x56) == "12:34:56"
    print("PASS: hdmi time bridge selftest")
    return 0


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args(list(argv) if argv is not None else None)
    if args.selftest:
        return _selftest()

    app = QApplication(sys.argv)
    app.setApplicationName("HDMI Time Bridge")
    window = TimeBridgeWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
