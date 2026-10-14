# -*- coding: utf-8 -*-
"""E104 应急检测装置 —— 天枢空港智显上位机（定制启动器）

原程序（airport_console.pyc）不做反编译，只用运行期补丁完成这些事：

  1. Console.__init__        : 把原界面整体包进 QTabWidget 的第 1 页，
                               新增第 2 页“应急监测”（只显示，不判定）。
  2. SerialWorker._handle    : 解析 25 字节 R_STATUS（兼容旧 10/13/18 字节），
                               提取温湿度、火焰 DO/AO、SR602、DS1307，
                               以及 MQ-2、水位、土壤湿度数据。
  3. Console._refresh_status : 主界面状态栏与应急监测页同步刷新。
  4. Console.load_demo       : --demo 时给应急页填充演示数据，便于离线截图。
  5. 界面布局与动态背景     : 日志面板上移加高；主界面和应急页使用蓝天白云
                               客机高清图做缓慢推拉的动态背景，不依赖视频解码器。
  6. 双链路同时在线         : 原“串口连接”继续连主板（CH340/USB-UART）；
                               新增独立的“蓝牙连接”走 HC-06 连 EG4X20B 应急装置。
                               两路各自收发、互不占用，可同时保持连接。

算法边界：温度 40 °C（连续 3 次）、湿度 95 %（连续 5 次）、火焰 DO（低电平）、
MQ-2 DO（低电平）和人体感应全部在 FPGA 内完成；ADC 百分比折算也在 FPGA 内
完成；本程序只做串口收发与显示，不做判定。
应急页“危险已解除”按钮只发送 0x11 清警命令，清除板卡已锁存的温湿度报警和
火焰报警。火焰 DO 触发后报警会一直保持，即使 DO 已恢复，也必须点击此按钮才会
解除；若清警时火焰信号仍在，板卡会立即重新锁存报警。人体检测只上屏、亮黄灯，
不驱动蜂鸣器；人离开后屏幕自动回到安全页，清警也不影响人体显示。

接线：主板走原串口连接（波特率按主板设置）；HC-06 出厂波特率 9600，
蓝牙链路固定在 9600-8-N-1。两路必须是两个不同的 COM 口。

用法：
  python e104_console.py --port COM3 --baud 115200
  python e104_console.py --demo --shot emergency_demo.png
  set E104_TAB=2 后配合 --demo --shot，可把截图切到应急监测页。
  蓝牙口默认优先选中 COM11；需要别的口时先设置环境变量 E104_BT_PORT=COMx。
"""

import math
import os
import re
import sys
import winreg
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import airport_console as ac  # noqa: E402  (patched module next to this file)

from PyQt5.QtCore import QRectF, Qt, QTimer, QUrl, pyqtSignal  # noqa: E402
from PyQt5.QtGui import (  # noqa: E402
    QColor,
    QDesktopServices,
    QIcon,
    QLinearGradient,
    QPainter,
    QPixmap,
)
from PyQt5.QtWidgets import (  # noqa: E402
    QApplication,
    QButtonGroup,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QStackedLayout,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

# ---------------------------------------------------------------------------
# HC-06 默认 9600：把 9600 加到波特率下拉框并设为默认值。
# ---------------------------------------------------------------------------
ac.BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
ac.FALLBACK_RATE = 9600

# 与应用层协议保持一致：0x11 清除 FPGA 已锁存的温湿度/火焰报警；人体实时跟随传感器。
CMD_CLEAR = 0x11
R_CLEAR = 0x91

EMG_QSS = """
QTabWidget#emgTabs::pane {
    border: 1px solid #D8E1EB;
    background: #F7FAFD;
    border-radius: 8px;
    top: -1px;
}
QTabBar::tab {
    background: #E8EFF6;
    border: 1px solid #D8E1EB;
    padding: 8px 26px;
    margin-right: 4px;
    border-top-left-radius: 7px;
    border-top-right-radius: 7px;
    color: #5B6C7E;
}
QTabBar::tab:selected {
    background: #FFFFFF;
    color: #1F2A37;
    font-weight: 600;
}
QTabBar::tab:hover { color: #2C3B4A; }

QFrame#emgBanner {
    background: #FFFFFF;
    border: 1px solid #D8E1EB;
    border-radius: 8px;
}
QLabel#emgTitle { font-size: 19px; font-weight: 600; color: #15202B; }
QLabel#emgSub { color: #607080; font-size: 13px; }
QLabel#emgPill {
    border-radius: 9px;
    padding: 5px 18px;
    border: 1px solid #D4DEE9;
    background: #F3F6FA;
    color: #5C6E80;
    font-weight: 600;
}
QLabel#emgPill[state="ok"] {
    background: #E7F7F0;
    border-color: #9AD9C0;
    color: #137A59;
}
QLabel#emgPill[state="warn"] {
    background: #FFF3DD;
    border-color: #F0C77B;
    color: #9A6518;
}
QLabel#emgPill[state="alarm"] {
    background: #FDECEC;
    border-color: #F0B1B1;
    color: #B62D35;
}
QPushButton#emgClearBtn {
    background: #B62D35;
    border: 1px solid #9F252C;
    border-radius: 6px;
    color: #FFFFFF;
    font-weight: 600;
    padding: 7px 16px;
}
QPushButton#emgClearBtn:hover { background: #C93A42; }
QPushButton#emgClearBtn:pressed { background: #962129; }
QPushButton#emgClearBtn:disabled {
    background: #E8EDF2;
    border-color: #D4DEE9;
    color: #98A6B5;
}
QLabel#emgChip {
    background: #F3F6FA;
    border: 1px solid #D6E0EA;
    border-radius: 6px;
    color: #5C6E80;
    font-family: Consolas, "Microsoft YaHei UI", monospace;
    font-size: 13px;
    padding: 4px 10px;
}
QLabel#emgChip[state="ok"] {
    background: #E7F7F0;
    border-color: #9AD9C0;
    color: #137A59;
}
QLabel#emgChip[state="bad"] {
    background: #FDECEC;
    border-color: #F0B1B1;
    color: #B62D35;
}

QFrame#emgCard {
    background: #FFFFFF;
    border: 1px solid #D8E1EB;
    border-radius: 8px;
}
QLabel#emgCardTitle { font-size: 14px; font-weight: 600; color: #2C3B4A; }
QLabel#emgValue {
    font-family: Consolas, "Microsoft YaHei UI", monospace;
    font-size: 34px;
    font-weight: 600;
    color: #1F2A37;
}
QLabel#emgValue[kind="ok"] { color: #137A59; }
QLabel#emgValue[kind="warn"] { color: #9A6518; }
QLabel#emgValue[kind="alarm"] { color: #B62D35; }
QLabel#emgUnit { color: #718092; font-size: 14px; }
QLabel#emgState { color: #5B6C7E; font-size: 14px; }
QLabel#emgState[kind="ok"] { color: #137A59; }
QLabel#emgState[kind="warn"] { color: #9A6518; font-weight: 600; }
QLabel#emgState[kind="alarm"] { color: #B62D35; font-weight: 600; }
QLabel#emgCaption { color: #8B9AAB; font-size: 13px; }

QFrame#emgFoot {
    background: #FFFFFF;
    border: 1px solid #D8E1EB;
    border-radius: 8px;
}
QLabel#emgFootText { color: #5B6C7E; font-size: 13px; }
QLabel#emgFootNote { color: #8B9AAB; font-size: 12px; }
"""

ac.QSS = ac.QSS + EMG_QSS

BG_QSS = """
QWidget#emgStage, QWidget#emgHostPage, QWidget#emgPage { background: transparent; }
QTabWidget#emgTabs { background: transparent; }
QTabWidget#emgTabs::pane {
    background: rgba(247, 250, 253, 0.20);
    border: 1px solid rgba(211, 222, 233, 0.88);
}
QTabBar { background: transparent; }
QTabBar::tab {
    background: rgba(232, 239, 246, 0.88);
    border: 1px solid rgba(211, 222, 233, 0.90);
}
QTabBar::tab:selected { background: rgba(255, 255, 255, 0.94); }
QFrame#header {
    background: rgba(255, 255, 255, 0.80);
    border: 1px solid rgba(211, 222, 233, 0.88);
}
QFrame#panel {
    background: rgba(255, 255, 255, 0.86);
    border: 1px solid rgba(211, 222, 233, 0.90);
}
QFrame#emgBanner, QFrame#emgCard, QFrame#emgFoot {
    background: rgba(255, 255, 255, 0.91);
}
QPlainTextEdit#log { background: rgba(247, 250, 253, 0.96); }
"""

ac.QSS = ac.QSS + BG_QSS


def _bcd(value):
    return ((value >> 4) & 0x0F) * 10 + (value & 0x0F)


def _set_state(widget, state):
    widget.setProperty("state", state)
    widget.style().unpolish(widget)
    widget.style().polish(widget)


def _set_kind(widget, kind):
    widget.setProperty("kind", kind)
    widget.style().unpolish(widget)
    widget.style().polish(widget)


class SkySceneBackground(QWidget):
    """蓝天白云 + 客机背景：高清静图缓慢推拉漂移，不依赖系统视频解码器。"""

    FPS = 15
    MAX_SOURCE_WIDTH = 2560

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("skyBg")
        self.setAttribute(Qt.WA_OpaquePaintEvent, True)

        self._source = self._load_source()
        self._cover = QPixmap()
        self._elapsed = 0.0

        self._timer = QTimer(self)
        self._timer.setTimerType(Qt.PreciseTimer)
        self._timer.setInterval(max(40, int(round(1000.0 / self.FPS))))
        self._timer.timeout.connect(self._advance)
        if not self._source.isNull():
            self._timer.start()

    def _load_source(self):
        path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "assets",
            "sky_bg",
            "sky_clouds_plane.jpg",
        )
        pixmap = QPixmap(path)
        if pixmap.isNull():
            return QPixmap()
        if pixmap.width() > self.MAX_SOURCE_WIDTH:
            pixmap = pixmap.scaledToWidth(
                self.MAX_SOURCE_WIDTH, Qt.SmoothTransformation
            )
        return pixmap

    def _rescale(self):
        if self._source.isNull() or self.width() <= 0 or self.height() <= 0:
            self._cover = QPixmap()
            return
        self._cover = self._source.scaled(
            self.size(), Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation
        )
        self.update()

    def _advance(self):
        if self._cover.isNull() or not self.isVisible():
            return
        window = self.window()
        if window is not None and window.isMinimized():
            return
        self._elapsed += 1.0 / self.FPS
        self.update()

    def _view_rect(self):
        """按时间取一块略小的取景窗，形成缓慢的推拉与漂移。"""
        width = self._cover.width()
        height = self._cover.height()
        if width <= 0 or height <= 0:
            return None
        t = self._elapsed
        zoom = 1.085 + 0.055 * math.sin(2.0 * math.pi * t / 48.0)
        center_x = 0.5 + 0.030 * math.sin(2.0 * math.pi * t / 67.0)
        center_y = 0.5 + 0.022 * math.sin(2.0 * math.pi * t / 53.0 + 1.1)
        window_w = width / zoom
        window_h = height / zoom
        left = min(max(center_x * width - window_w / 2.0, 0.0), width - window_w)
        top = min(max(center_y * height - window_h / 2.0, 0.0), height - window_h)
        return QRectF(left, top, window_w, window_h)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._rescale()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
        if self._cover.isNull():
            painter.fillRect(self.rect(), QColor("#8FC3EE"))
            return
        source = self._view_rect()
        if source is None:
            painter.drawPixmap(0, 0, self._cover)
        else:
            painter.drawPixmap(QRectF(self.rect()), self._cover, source)
        # 轻微提亮 + 底部淡蓝雾气：保留蓝天的通透，同时让半透明面板上的文字更清楚。
        wash = QLinearGradient(0.0, 0.0, 0.0, float(self.height()))
        wash.setColorAt(0.0, QColor(255, 255, 255, 16))
        wash.setColorAt(1.0, QColor(214, 235, 252, 64))
        painter.fillRect(self.rect(), wash)


class EmergencyPage(QWidget):
    """应急监测页：只显示 FPGA 已经算好的状态，不在这里做任何判定。"""

    clear_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("emgPage")

        root = QVBoxLayout(self)
        root.setContentsMargins(18, 14, 18, 16)
        root.setSpacing(12)

        root.addWidget(self._build_banner())

        grid = QGridLayout()
        grid.setSpacing(12)
        specs = [
            ("temp", "温度", "°C", "DHT11 · FPGA 阈值 40 °C，连续 3 次"),
            ("humi", "湿度", "%RH", "DHT11 · FPGA 阈值 95 %，连续 5 次"),
            ("flame", "火焰", "", "火焰 DO · 低电平报警"),
            ("flame_ao", "火焰模拟量", "V", "ADC CH3 · 12 bit 均值"),
            ("pir", "人体感应", "", "SR602 · 有人只上屏提示"),
            ("rtc", "实时时钟", "", "DS1307 · I2C 同步"),
            ("smoke", "烟雾", "%", "MQ-2 AO · DO 低电平预警"),
            ("water", "水位", "%", "水位 S · ADC CH5 均值"),
            ("soil", "土壤湿度", "%", "土壤 AO · ADC CH6 均值"),
        ]
        self.cards = {}
        for idx, (key, title, unit, caption) in enumerate(specs):
            card = self._build_card(title, unit, caption)
            self.cards[key] = card
            grid.addWidget(card["frame"], idx // 3, idx % 3)
        for row in range(3):
            grid.setRowStretch(row, 1)
        for col in range(3):
            grid.setColumnStretch(col, 1)
        root.addLayout(grid, 1)

        root.addWidget(self._build_foot())

        self.update_status({})

    # -- 构建 ---------------------------------------------------------------

    def _build_banner(self):
        bar = QFrame()
        bar.setObjectName("emgBanner")
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(18, 12, 18, 12)
        lay.setSpacing(10)

        box = QVBoxLayout()
        box.setSpacing(2)
        title = QLabel("应急检测装置 · 实时监测")
        title.setObjectName("emgTitle")
        sub = QLabel("EG4X20B 就地判定 · 蓝牙 9600-8-N-1 · 上位机仅显示")
        sub.setObjectName("emgSub")
        box.addWidget(title)
        box.addWidget(sub)
        lay.addLayout(box)
        lay.addStretch(1)

        self.chip_dht = QLabel("DHT11 --")
        self.chip_rtc = QLabel("时钟 --")
        self.chip_ble = QLabel("蓝牙 --")
        for chip in (self.chip_dht, self.chip_rtc, self.chip_ble):
            chip.setObjectName("emgChip")
            lay.addWidget(chip)

        self.btn_clear = QPushButton("危险已解除")
        self.btn_clear.setObjectName("emgClearBtn")
        self.btn_clear.setCursor(Qt.PointingHandCursor)
        self.btn_clear.setToolTip("清除 FPGA 已锁存的温湿度与火焰报警（人体显示实时跟随传感器）")
        self.btn_clear.clicked.connect(self.clear_requested.emit)
        lay.addWidget(self.btn_clear)

        self.pill = QLabel("等待数据")
        self.pill.setObjectName("emgPill")
        self.pill.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.pill)
        return bar

    def _build_card(self, title, unit, caption):
        frame = QFrame()
        frame.setObjectName("emgCard")
        frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        lay = QVBoxLayout(frame)
        lay.setContentsMargins(18, 14, 18, 14)
        lay.setSpacing(6)

        head = QLabel(title)
        head.setObjectName("emgCardTitle")
        lay.addWidget(head)

        row = QHBoxLayout()
        row.setSpacing(6)
        value = QLabel("--")
        value.setObjectName("emgValue")
        unit_label = QLabel(unit)
        unit_label.setObjectName("emgUnit")
        row.addWidget(value, 0, Qt.AlignBottom)
        row.addWidget(unit_label, 0, Qt.AlignBottom)
        row.addStretch(1)
        lay.addLayout(row)

        state = QLabel("等待数据")
        state.setObjectName("emgState")
        lay.addWidget(state)
        lay.addStretch(1)

        cap = QLabel(caption)
        cap.setObjectName("emgCaption")
        lay.addWidget(cap)

        return {"frame": frame, "value": value, "state": state}

    def _build_foot(self):
        foot = QFrame()
        foot.setObjectName("emgFoot")
        lay = QHBoxLayout(foot)
        lay.setContentsMargins(18, 10, 18, 10)
        lay.setSpacing(12)

        self.foot_out = QLabel("本地声光输出：--")
        self.foot_out.setObjectName("emgFootText")
        note = QLabel("全部阈值与报警判定在 FPGA 内完成")
        note.setObjectName("emgFootNote")
        lay.addWidget(self.foot_out)
        lay.addStretch(1)
        lay.addWidget(note)
        return foot

    def set_connected(self, connected):
        """只有链路建立后才允许发送清警命令。"""
        self.btn_clear.setEnabled(bool(connected))
        self.btn_clear.setToolTip(
            "清除 FPGA 已锁存的温湿度与火焰报警（人体显示实时跟随传感器）"
            if connected
            else "连接板卡后才能发送清警命令"
        )

    # -- 刷新 ---------------------------------------------------------------

    def update_status(self, st):
        def g(key, default=0):
            return st.get(key, default)

        temp_alarm = g("temp_alarm")
        humi_alarm = g("humi_alarm")
        flame_alarm = g("flame_alarm")
        smoke_alarm = g("smoke_alarm")
        pir_active = g("pir_active")
        alarm_any = g("alarm_any")
        dht_ok = g("dht_ok")
        rtc_valid = g("rtc_valid")
        ble_online = g("ble_online")

        # 顶部胶囊：只有火焰使用红色报警，温湿度和人体统一为黄色预警。
        if not st:
            self.pill.setText("等待数据")
            _set_state(self.pill, "idle")
        elif flame_alarm:
            self.pill.setText("报警 · 火焰")
            _set_state(self.pill, "alarm")
        elif smoke_alarm or temp_alarm or humi_alarm or pir_active:
            names = []
            if smoke_alarm:
                names.append("烟雾")
            if temp_alarm:
                names.append("高温")
            if humi_alarm:
                names.append("高湿")
            if pir_active:
                names.append("有人")
            self.pill.setText("预警 · " + "、".join(names))
            _set_state(self.pill, "warn")
        else:
            self.pill.setText("系统正常")
            _set_state(self.pill, "ok")

        # 顶部小标签
        if st:
            self.chip_dht.setText("DHT11 正常" if dht_ok else "DHT11 离线")
            _set_state(self.chip_dht, "ok" if dht_ok else "bad")
            self.chip_rtc.setText("时钟 已同步" if rtc_valid else "时钟 未同步")
            _set_state(self.chip_rtc, "ok" if rtc_valid else "bad")
            self.chip_ble.setText("蓝牙 已连接" if ble_online else "蓝牙 空闲")
            _set_state(self.chip_ble, "ok" if ble_online else "bad")
        else:
            self.chip_dht.setText("DHT11 --")
            self.chip_rtc.setText("时钟 --")
            self.chip_ble.setText("蓝牙 --")
            for chip in (self.chip_dht, self.chip_rtc, self.chip_ble):
                _set_state(chip, "idle")

        # 温度
        card = self.cards["temp"]
        if st and dht_ok:
            card["value"].setText(str(g("temp")))
            if temp_alarm:
                card["state"].setText("高温预警")
                _set_kind(card["value"], "warn")
                _set_kind(card["state"], "warn")
            else:
                card["state"].setText("温度正常")
                _set_kind(card["value"], "ok")
                _set_kind(card["state"], "ok")
        else:
            card["value"].setText("--")
            card["state"].setText("等待 DHT11")
            _set_kind(card["value"], "idle")
            _set_kind(card["state"], "idle")

        # 湿度
        card = self.cards["humi"]
        if st and dht_ok:
            card["value"].setText(str(g("humi")))
            if humi_alarm:
                card["state"].setText("高湿预警")
                _set_kind(card["value"], "warn")
                _set_kind(card["state"], "warn")
            else:
                card["state"].setText("湿度正常")
                _set_kind(card["value"], "ok")
                _set_kind(card["state"], "ok")
        else:
            card["value"].setText("--")
            card["state"].setText("等待 DHT11")
            _set_kind(card["value"], "idle")
            _set_kind(card["state"], "idle")

        # 火焰 DO
        card = self.cards["flame"]
        if st:
            if flame_alarm:
                card["value"].setText("有火焰")
                card["state"].setText("DO 报警 · 已触发")
                _set_kind(card["value"], "alarm")
                _set_kind(card["state"], "alarm")
            else:
                card["value"].setText("无")
                card["state"].setText("DO 正常")
                _set_kind(card["value"], "ok")
                _set_kind(card["state"], "ok")
        else:
            card["value"].setText("--")
            card["state"].setText("等待数据")
            _set_kind(card["value"], "idle")
            _set_kind(card["state"], "idle")

        # 火焰 AO
        card = self.cards["flame_ao"]
        if st:
            raw = g("flame_ao")
            volt = raw * 3.3 / 4095.0
            card["value"].setText("%.2f" % volt)
            card["state"].setText("0x%03X · %d" % (raw, raw))
            _set_kind(card["value"], "idle")
            _set_kind(card["state"], "idle")
        else:
            card["value"].setText("--")
            card["state"].setText("等待数据")
            _set_kind(card["value"], "idle")
            _set_kind(card["state"], "idle")

        # 人体红外
        card = self.cards["pir"]
        if st:
            if pir_active:
                card["value"].setText("有人")
                card["state"].setText("SR602 已触发")
                _set_kind(card["value"], "warn")
                _set_kind(card["state"], "warn")
            else:
                card["value"].setText("无人")
                card["state"].setText("SR602 监视中")
                _set_kind(card["value"], "ok")
                _set_kind(card["state"], "ok")
        else:
            card["value"].setText("--")
            card["state"].setText("等待数据")
            _set_kind(card["value"], "idle")
            _set_kind(card["state"], "idle")

        # 实时时钟
        card = self.cards["rtc"]
        if st and rtc_valid:
            card["value"].setText(
                "%02d:%02d:%02d"
                % (_bcd(g("rtc_h")), _bcd(g("rtc_m")), _bcd(g("rtc_s")))
            )
            card["state"].setText("DS1307 走时")
            _set_kind(card["value"], "idle")
            _set_kind(card["state"], "ok")
        else:
            card["value"].setText("--:--:--")
            card["state"].setText("等待时钟")
            _set_kind(card["value"], "idle")
            _set_kind(card["state"], "idle")

        # MQ-2 烟雾浓度
        card = self.cards["smoke"]
        if st:
            card["value"].setText(str(g("smoke")))
            if smoke_alarm:
                card["state"].setText("烟雾预警")
                _set_kind(card["value"], "warn")
                _set_kind(card["state"], "warn")
            else:
                card["state"].setText("浓度正常")
                _set_kind(card["value"], "ok")
                _set_kind(card["state"], "ok")
        else:
            card["value"].setText("--")
            card["state"].setText("等待 ADC CH2")
            _set_kind(card["value"], "idle")
            _set_kind(card["state"], "idle")

        # 水位：FPGA 只上报 0~100%，上位机不做二次判定。
        card = self.cards["water"]
        if st:
            card["value"].setText(str(g("water")))
            card["state"].setText("水位监测中")
            _set_kind(card["value"], "ok")
            _set_kind(card["state"], "ok")
        else:
            card["value"].setText("--")
            card["state"].setText("等待 ADC CH5")
            _set_kind(card["value"], "idle")
            _set_kind(card["state"], "idle")

        # 土壤湿度：AO 折算百分比；DO 只给干湿提示，不参与报警。
        card = self.cards["soil"]
        if st:
            card["value"].setText(str(g("soil")))
            if g("soil_wet"):
                card["state"].setText("探针判定湿润")
                _set_kind(card["value"], "ok")
                _set_kind(card["state"], "ok")
            else:
                card["state"].setText("探针判定干燥")
                _set_kind(card["value"], "idle")
                _set_kind(card["state"], "idle")
        else:
            card["value"].setText("--")
            card["state"].setText("等待 ADC CH6")
            _set_kind(card["value"], "idle")
            _set_kind(card["state"], "idle")

        # 底部：本地声光输出说明（由 FPGA 的报警位推导显示文本）
        if not st:
            self.foot_out.setText("本地声光输出：--")
        elif flame_alarm:
            self.foot_out.setText("本地声光输出：红灯快闪 · 蜂鸣器持续鸣响")
        elif smoke_alarm and pir_active:
            self.foot_out.setText("本地声光输出：黄灯常亮 · 蜂鸣器静音 · 屏幕提示 SMOKE/PERSON")
        elif smoke_alarm:
            self.foot_out.setText("本地声光输出：黄灯常亮 · 蜂鸣器静音 · 屏幕提示 SMOKE")
        elif pir_active and (temp_alarm or humi_alarm):
            self.foot_out.setText("本地声光输出：黄灯常亮 · 蜂鸣器静音 · 屏幕提示人体")
        elif pir_active:
            self.foot_out.setText("本地声光输出：黄灯常亮 · 蜂鸣器静音 · 屏幕提示 PERSON")
        elif temp_alarm or humi_alarm:
            self.foot_out.setText("本地声光输出：黄灯常亮 · 蜂鸣器静音")
        else:
            self.foot_out.setText("本地声光输出：绿灯心跳 · 蜂鸣器静音")


# ---------------------------------------------------------------------------
# 协议解析：R_STATUS 10 / 13 / 18 / 25 字节四种格式兼容
# ---------------------------------------------------------------------------


def e104_handle(self, cmd, payload):
    self.log.emit("rx", "%02X  %s" % (cmd, payload.hex().upper()))
    self.got_reply.emit(cmd, payload)

    if cmd == ac.R_STATUS and len(payload) >= 10:
        st = {
            "ver": payload[0],
            "slot": payload[1],
            "total": payload[2],
            "mode": payload[3] & 1,
            "bright": payload[4] & 7,
            "contrast": payload[5] & 7,
            "osd": payload[6] & 3,
            "cache": payload[7] & 63,
            "textlen": payload[8],
            "link_seen": payload[9] & 1,
            "card_ok": (payload[9] >> 1) & 1,
        }
        if len(payload) >= 13:
            st["rtc_h"] = payload[10]
            st["rtc_m"] = payload[11]
            st["rtc_s"] = payload[12]
        if len(payload) >= 18:
            st.update(
                {
                    "alarm_any": payload[3] & 1,
                    "temp_alarm": (payload[3] >> 1) & 1,
                    "humi_alarm": (payload[3] >> 2) & 1,
                    "flame_alarm": (payload[3] >> 3) & 1,
                    "pir_active": (payload[3] >> 4) & 1,
                    "smoke_alarm": (payload[3] >> 5) & 1,
                    "temp_level": payload[4] & 7,
                    "flame_level": payload[5] & 7,
                    "ble_online": payload[7] & 1,
                    "rtc_valid": (payload[7] >> 1) & 1,
                    "dht_ok": (payload[7] >> 5) & 1,
                    "temp": payload[13],
                    "humi": payload[14],
                    "flame_ao": payload[15] | ((payload[16] & 0x0F) << 8),
                    "features": payload[17],
                }
            )
        if len(payload) >= 25:
            st.update(
                {
                    "smoke_do": payload[18] & 1,
                    "soil_wet": (payload[18] >> 1) & 1,
                    "smoke": payload[19],
                    "water": payload[20],
                    "soil": payload[21],
                    "smoke_ao": payload[22] | ((payload[23] & 0x0F) << 8),
                    "features2": payload[24],
                }
            )
        self.got_status.emit(st)
        return

    if cmd == R_CLEAR:
        self.log.emit("info", "board cleared latched alarms and PIR tone")
        return

    if cmd == ac.R_ERROR and len(payload) >= 2:
        self.log.emit(
            "err",
            "board refused cmd %02X: %s"
            % (payload[0], ac.ERR_TEXT.get(payload[1], "code %d" % payload[1])),
        )
        return


# ---------------------------------------------------------------------------
# 界面补丁
# ---------------------------------------------------------------------------

_SPP_GUID = "00001101-0000-1000-8000-00805f9b34fb"
_BT_ENUM = r"SYSTEM\CurrentControlSet\Enum\BTHENUM"
_BT_DEVS = r"SYSTEM\CurrentControlSet\Services\BTHPORT\Parameters\Devices"

SEG_QSS = """
QPushButton#segBtn {
    border: 1px solid #C9D4DF;
    background: #F2F6FA;
    color: #33414E;
    border-radius: 6px;
    padding: 4px 16px;
}
QPushButton#segBtn:checked {
    background: #E8A33D;
    border-color: #E8A33D;
    color: #FFFFFF;
    font-weight: 600;
}
QPushButton#miniBtn {
    border: 1px solid #C9D4DF;
    background: #FFFFFF;
    color: #33414E;
    border-radius: 6px;
    padding: 4px 10px;
}
QPushButton#miniBtn:hover { border-color: #E8A33D; color: #B8730F; }
"""


def _bt_device_name(mac):
    """从注册表读取已配对蓝牙设备的名称（例如 HC-06）。"""
    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, _BT_DEVS + "\\" + mac)
        val, _ = winreg.QueryValueEx(key, "Name")
    except OSError:
        return ""
    if isinstance(val, bytes):
        val = val.split(b"\x00")[0].decode("utf-8", "ignore")
    return str(val).strip()


def list_bluetooth_ports():
    """列出已配对的蓝牙串口设备：[(名称, MAC, COM 口), ...]。

    Windows 会把蓝牙 SPP（HC-06 属于这一类）映射成虚拟串口，这里从
    BTHENUM 枚举读端口号、从 BTHPORT 读设备名，不需要额外依赖。
    """
    found = []
    try:
        root = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, _BT_ENUM)
    except OSError:
        return found

    i = 0
    while True:
        try:
            enum = winreg.EnumKey(root, i)
            i += 1
        except OSError:
            break
        if _SPP_GUID not in enum.lower():
            continue
        try:
            enum_key = winreg.OpenKey(root, enum)
        except OSError:
            continue
        j = 0
        while True:
            try:
                inst = winreg.EnumKey(enum_key, j)
                j += 1
            except OSError:
                break
            try:
                dev_key = winreg.OpenKey(enum_key, inst + "\\Device Parameters")
                port = winreg.QueryValueEx(dev_key, "PortName")[0]
            except OSError:
                continue
            match = re.search(r"&([0-9A-Fa-f]{12})_", inst)
            mac = match.group(1).lower() if match else ""
            if not mac or mac == "0" * 12:
                continue        # 本机蓝牙服务，不是外部设备
            name = _bt_device_name(mac) or ("蓝牙设备 " + mac.upper())
            found.append((name, mac, port))

    seen, uniq = set(), []
    for name, mac, port in found:
        if port in seen:
            continue
        seen.add(port)
        uniq.append((name, mac, port))
    return uniq


def _install_conn_mode_legacy(self):
    """旧版“串口 / 蓝牙”二选一实现，保留仅作参考，不再被调用。

    串口模式沿用原程序的 CH340 串口列表；蓝牙模式列出已配对的蓝牙 SPP
    设备（HC-06 → COM11），连接时仍走同一个串口通道，只是链路换成蓝牙。
    """
    box = getattr(self, "port_box", None)
    panel = box.parent() if box is not None else None
    lay = panel.layout() if panel is not None else None
    if box is None or lay is None:
        return

    hint = None
    for k in range(lay.count()):
        w = lay.itemAt(k).widget()
        if isinstance(w, QLabel) and w.objectName() == "hint":
            hint = w

    row = QHBoxLayout()
    row.setSpacing(6)
    row.addWidget(QLabel("连接方式"))
    btn_serial = QPushButton("串口")
    btn_bt = QPushButton("蓝牙")
    for b in (btn_serial, btn_bt):
        b.setObjectName("segBtn")
        b.setCheckable(True)
        b.setCursor(Qt.PointingHandCursor)
    group = QButtonGroup(self)
    group.setExclusive(True)
    group.addButton(btn_serial)
    group.addButton(btn_bt)
    btn_serial.setChecked(True)
    row.addWidget(btn_serial)
    row.addWidget(btn_bt)
    row.addStretch(1)

    btn_setup = QPushButton("配对蓝牙设备")
    btn_setup.setObjectName("miniBtn")
    btn_setup.setCursor(Qt.PointingHandCursor)
    btn_setup.setToolTip("打开 Windows 蓝牙设置，配对 HC-06（配对码 1234）")
    btn_setup.clicked.connect(
        lambda: QDesktopServices.openUrl(QUrl("ms-settings:bluetooth"))
    )
    row.addWidget(btn_setup)
    lay.insertLayout(1, row)
    panel.setStyleSheet(panel.styleSheet() + SEG_QSS)

    def apply_mode():
        bt_mode = btn_bt.isChecked()
        self._emg_mode = "bt" if bt_mode else "serial"
        if bt_mode:
            devs = list_bluetooth_ports()
            box.clear()
            for name, _mac, port in devs:
                box.addItem("%s · %s" % (name, port), port)
            idx = box.findData("COM11")
            if idx >= 0:
                box.setCurrentIndex(idx)
            baud = getattr(self, "baud_box", None)
            if baud is not None and baud.findData(9600) >= 0:
                baud.setCurrentIndex(baud.findData(9600))
            if hint is not None:
                hint.setText(
                    "选择已配对的蓝牙设备（HC-06 配对码 1234，默认 9600）；"
                    "列表为空时先在系统设置里配对并连接一次"
                )
            if hasattr(self, "log_line"):
                if devs:
                    self.log_line("info", "蓝牙模式：找到 %d 个已配对串口设备" % len(devs))
                else:
                    self.log_line("info", "未找到已配对的蓝牙串口设备，请先配对 HC-06")
        else:
            self.refresh_ports()
            if hint is not None:
                hint.setText("选择 USB-UART 对应的 CH340 串口，连接后即可控制板卡")

    btn_serial.clicked.connect(apply_mode)
    btn_bt.clicked.connect(apply_mode)

    refresh = getattr(self, "refresh_btn", None)
    if refresh is not None:
        # 原刷新按钮先扫串口，这里延后一拍按当前模式重填列表。
        refresh.clicked.connect(lambda: QTimer.singleShot(0, apply_mode))

    self._emg_conn_bt = btn_bt
    if os.environ.get("E104_CONN", "").lower() == "bt":
        btn_bt.setChecked(True)
        apply_mode()


def _all_port_choices():
    """返回 [(显示标签, 端口名)]：蓝牙 SPP 在前，普通串口在后，按端口去重。"""
    choices = []
    seen = set()
    for name, _mac, port in list_bluetooth_ports():
        if not port or port in seen:
            continue
        seen.add(port)
        choices.append(("%s · %s" % (name or "蓝牙设备", port), port))
    try:
        infos = list(ac.list_ports.comports())
    except Exception:
        infos = []
    for info in infos:
        port = str(getattr(info, "device", "") or "")
        if not port or port in seen:
            continue
        seen.add(port)
        desc = str(getattr(info, "description", "") or "").strip()
        if not desc or desc.lower() == "n/a":
            desc = "串口"
        choices.append(("%s · %s" % (desc, port), port))
    return choices


def _install_dual_link(self):
    """保留原串口连接，再挂一路独立的蓝牙应急链路。

    主串口仍由原界面的 port_box / 连接按钮控制；这里新增蓝牙设备下拉框、
    连接按钮和状态标签，两路使用各自的 SerialWorker，可同时在线。
    """
    box = getattr(self, "port_box", None)
    panel = box.parent() if box is not None else None
    lay = panel.layout() if panel is not None else None
    if box is None or lay is None:
        return

    hint = panel.findChild(QLabel, "hint")
    if hint is not None:
        hint.setText("主板默认 115200-8-N-1；蓝牙 HC-06 固定 9600，两路可同时连接")

    bt_box = QComboBox()
    bt_box.setSizeAdjustPolicy(QComboBox.AdjustToContents)
    bt_box.setMinimumWidth(110)
    bt_refresh = QPushButton("刷新")
    bt_refresh.setObjectName("miniBtn")
    bt_refresh.setCursor(Qt.PointingHandCursor)
    bt_refresh.setToolTip("重新扫描已配对的蓝牙 SPP 串口和普通串口")

    bt_btn = QPushButton("连接蓝牙")
    bt_btn.setObjectName("miniBtn")
    bt_btn.setCursor(Qt.PointingHandCursor)
    bt_btn.setToolTip("独立连接 HC-06；与主板串口互不影响，可同时在线")

    bt_pill = QLabel("蓝牙未连接")
    bt_pill.setObjectName("btPill")
    bt_pill.setAlignment(Qt.AlignCenter)
    bt_pill.setFixedHeight(27)

    bt_setup = QPushButton("配对")
    bt_setup.setObjectName("miniBtn")
    bt_setup.setCursor(Qt.PointingHandCursor)
    bt_setup.setToolTip("打开 Windows 蓝牙设置，配对 HC-06（配对码 1234）")
    bt_setup.clicked.connect(
        lambda: QDesktopServices.openUrl(QUrl("ms-settings:bluetooth"))
    )

    row1 = QHBoxLayout()
    row1.setSpacing(6)
    row1.addWidget(QLabel("蓝牙"))
    row1.addWidget(bt_box, 1)
    row1.addWidget(bt_refresh)
    row1.addWidget(bt_setup)

    row2 = QHBoxLayout()
    row2.setSpacing(6)
    row2.addWidget(bt_btn)
    row2.addWidget(bt_pill, 1)

    # 插到主板连接按钮之后；若找不到，退回端口选择行之后。
    anchor = None
    for target in (getattr(self, "connect_btn", None), box):
        if target is None:
            continue
        for i in range(lay.count()):
            item = lay.itemAt(i)
            widget = item.widget()
            sub = item.layout()
            if widget is target or (sub is not None and sub.indexOf(target) >= 0):
                anchor = i + 1
                break
        if anchor is not None:
            break
    if anchor is None:
        anchor = min(1, lay.count())
    lay.insertLayout(anchor, row1)
    lay.insertLayout(anchor + 1, row2)

    panel.setStyleSheet(
        panel.styleSheet()
        + SEG_QSS
        + """
QLabel#btPill {
    border: 1px solid #C9D4DF;
    border-radius: 6px;
    padding: 3px 8px;
    color: #6B7A89;
    background: #F7FAFD;
}
QLabel#btPill[state="ok"] {
    border-color: #9BD5B3;
    color: #1F7A45;
    background: #E9F7EF;
}
"""
    )

    self._emg_port_box = bt_box
    self._emg_connect_btn = bt_btn
    self._emg_link_pill = bt_pill

    bt_btn.clicked.connect(self._emg_toggle)
    bt_refresh.clicked.connect(self._emg_refresh_ports)
    self._emg_refresh_ports()
    self._emg_set_ui()


def _emg_refresh_ports(self):
    """重填蓝牙下拉框；已配对的 HC-06 会显示成 SPP 串口。"""
    box = getattr(self, "_emg_port_box", None)
    if box is None:
        return
    keep = box.currentData()
    box.blockSignals(True)
    box.clear()
    for label, port in _all_port_choices():
        box.addItem(label, port)
    want = os.environ.get("E104_BT_PORT", "COM11")
    idx = box.findData(keep) if keep else -1
    if idx < 0:
        idx = box.findData(want)
    if idx < 0 and box.count():
        idx = 0
    if idx >= 0:
        box.setCurrentIndex(idx)
    box.blockSignals(False)
    log = getattr(self, "log_line", None)
    if callable(log):
        log("info", "可选串口 %d 个；蓝牙请选 HC-06 对应的 COM 口" % box.count())


def _emg_set_ui(self):
    conn = bool(getattr(self, "_emg_connected", False))
    busy = bool(getattr(self, "_emg_busy", False))
    btn = getattr(self, "_emg_connect_btn", None)
    box = getattr(self, "_emg_port_box", None)
    pill = getattr(self, "_emg_link_pill", None)
    if btn is not None:
        btn.setText("断开蓝牙" if conn else ("连接中…" if busy else "连接蓝牙"))
        btn.setEnabled(not busy)
    if box is not None:
        box.setEnabled(not conn and not busy)
    if pill is not None:
        pill.setText("蓝牙已连接" if conn else ("正在连接…" if busy else "蓝牙未连接"))
        pill.setProperty("state", "ok" if conn else "idle")
        style = pill.style()
        style.unpolish(pill)
        style.polish(pill)


def _emg_on_status(self, st):
    """蓝牙链路收到的 R_STATUS：只更新应急页，不影响主板状态。"""
    self._emg_status = st
    page = getattr(self, "_emg_page", None)
    if page is not None:
        page.update_status(st)
        page.set_connected(bool(getattr(self, "_emg_connected", False)))


def _emg_on_log(self, level, text):
    log = getattr(self, "log_line", None)
    if not callable(log):
        return
    lv = level if level in ("info", "ok", "err") else "info"
    log(lv, "[蓝牙] %s" % text)


def _emg_on_reply(self, cmd, payload):
    if cmd == R_CLEAR:
        log = getattr(self, "log_line", None)
        if callable(log):
            log("ok", "蓝牙应急装置已解除锁存报警")


def _emg_on_opened(self, ok, info):
    self._emg_busy = False
    log = getattr(self, "log_line", None)
    if not ok:
        if callable(log):
            log("err", "蓝牙打开失败：%s" % info)
        self._emg_close()
        return
    if callable(log):
        log("ok", "蓝牙已连接：%s" % info)
    self._emg_connected = True
    timer = getattr(self, "_emg_poll_timer", None)
    if timer is not None:
        timer.start(500)
    worker = getattr(self, "_emg_worker", None)
    if worker is not None:
        worker.send(ac.CMD_STATUS)
    self._emg_set_ui()
    page = getattr(self, "_emg_page", None)
    if page is not None:
        page.set_connected(True)


def _emg_poll_tick(self):
    if not getattr(self, "_emg_connected", False):
        return
    worker = getattr(self, "_emg_worker", None)
    if worker is None:
        return
    try:
        worker.send(ac.CMD_STATUS)
    except Exception:
        pass


def _emg_open(self):
    box = getattr(self, "_emg_port_box", None)
    port = box.currentData() if box is not None else None
    if not port:
        show = getattr(self, "show_hint", None)
        if callable(show):
            show("请先选择蓝牙串口；HC-06 需先在 Windows 里配对")
        return
    main_box = getattr(self, "port_box", None)
    main_port = main_box.currentData() if main_box is not None else None
    if main_port and port == main_port and getattr(self, "connected", False):
        show = getattr(self, "show_hint", None)
        if callable(show):
            show("该 COM 口已被主板占用，请为蓝牙选择另一个口")
        return
    self._emg_busy = True
    self._emg_set_ui()
    worker = ac.SerialWorker(port, 9600, self)
    worker.opened.connect(self._emg_on_opened)
    worker.log.connect(self._emg_on_log)
    worker.got_status.connect(self._emg_on_status)
    worker.got_reply.connect(self._emg_on_reply)
    self._emg_worker = worker
    worker.start()


def _emg_close(self):
    timer = getattr(self, "_emg_poll_timer", None)
    if timer is not None:
        timer.stop()
    worker = getattr(self, "_emg_worker", None)
    self._emg_worker = None
    self._emg_connected = False
    self._emg_busy = False
    if worker is not None:
        try:
            worker.stop()
        except Exception:
            pass
        try:
            worker.wait(1500)
        except Exception:
            pass
    self._emg_set_ui()
    page = getattr(self, "_emg_page", None)
    if page is not None:
        page.set_connected(False)


def _emg_toggle(self):
    if getattr(self, "_emg_busy", False):
        return
    if getattr(self, "_emg_connected", False):
        self._emg_close()
        log = getattr(self, "log_line", None)
        if callable(log):
            log("info", "蓝牙连接已断开")
    else:
        self._emg_open()


def _find_panel_by_title(widget, title):
    for panel in widget.findChildren(QFrame):
        if panel.objectName() != "panel":
            continue
        label = panel.findChild(QLabel, "panelTitle")
        if label is not None and label.text().strip() == title:
            return panel
    return None


def _polish_console_layout(console):
    """把日志面板上移并加高，固定原面板高度，避免空间被均摊。"""
    log_panel = _find_panel_by_title(console, "链路日志")
    if log_panel is None:
        return

    parent = log_panel.parentWidget()
    layout = parent.layout() if parent is not None else None
    if layout is not None:
        layout.removeWidget(log_panel)
        layout.insertWidget(min(1, layout.count()), log_panel)

    # 原界面的四个左侧面板都是可拉伸策略；固定前三者的自然高度，
    # 把剩余空间稳定留给日志，而不是让每块面板一起变高。
    for title, height in (
        ("串口连接", 360),
        ("板卡状态", 258),
        ("板卡信息", 176),
    ):
        panel = _find_panel_by_title(console, title)
        if panel is not None:
            panel.setFixedHeight(height)

    log_panel.setMinimumHeight(280)
    log_panel.setMaximumHeight(280)
    log_panel.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
    panel_layout = log_panel.layout()
    if panel_layout is not None:
        panel_layout.setStretch(1, 1)

    log_edit = log_panel.findChild(QPlainTextEdit, "log")
    if log_edit is not None:
        log_edit.setMinimumHeight(210)
        log_edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)


_orig_init = ac.Console.__init__
_orig_refresh = ac.Console._refresh_status
_orig_load_demo = ac.Console.load_demo


def _e104_send_clear(self):
    """把“危险已解除”按钮变成一次 0x11 清警帧，通过蓝牙链路发出。"""
    worker = getattr(self, "_emg_worker", None)
    if worker is None or not getattr(self, "_emg_connected", False):
        show = getattr(self, "show_hint", None)
        if callable(show):
            show("请先连接蓝牙应急装置")
        return
    worker.send(CMD_CLEAR, b"")
    log = getattr(self, "log_line", None)
    if callable(log):
        log("info", "已通过蓝牙发送“危险已解除”命令 (11)")


def e104_init(self, assets="", port="", baud=115200):
    """把原界面整体放进第 1 页，再挂上“应急监测”第 2 页。"""
    _orig_init(self, assets, port, baud)

    # 蓝牙应急链路：独立于主串口，运行时状态全部放在实例属性上。
    self._emg_status = {}
    self._emg_worker = None
    self._emg_connected = False
    self._emg_busy = False
    self._emg_poll_timer = QTimer(self)
    self._emg_poll_timer.setInterval(500)
    self._emg_poll_timer.timeout.connect(self._emg_poll_tick)

    icon_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "e104_console.ico")
    if os.path.exists(icon_path):
        icon = QIcon(icon_path)
        app = QApplication.instance()
        if app is not None:
            app.setWindowIcon(icon)
        self.setWindowIcon(icon)
    self.setWindowTitle("天枢空港智显 · EG4X20B 应急监测")

    root = self.layout()
    items = []
    if root is not None:
        while root.count():
            stretch = root.stretch(0)
            items.append((root.takeAt(0), stretch))

    page1 = QWidget(self)
    page1.setObjectName("emgHostPage")
    lay1 = QVBoxLayout(page1)
    lay1.setContentsMargins(0, 0, 0, 0)
    lay1.setSpacing(0)
    for item, stretch in items:
        widget = item.widget()
        if widget is not None:
            lay1.addWidget(widget, stretch)
            continue
        sub = item.layout()
        if sub is not None:
            lay1.addLayout(sub, stretch)

    tabs = QTabWidget(self)
    tabs.setObjectName("emgTabs")
    tabs.addTab(page1, "天枢空港智显")
    page2 = EmergencyPage()
    page2.clear_requested.connect(self._e104_send_clear)
    tabs.addTab(page2, "应急监测")

    stage = QWidget(self)
    stage.setObjectName("emgStage")
    stack = QStackedLayout(stage)
    stack.setContentsMargins(0, 0, 0, 0)
    stack.setSpacing(0)
    stack.setStackingMode(QStackedLayout.StackAll)
    background = SkySceneBackground(stage)
    stack.addWidget(background)
    stack.addWidget(tabs)
    stack.setCurrentWidget(tabs)

    if root is not None:
        root.addWidget(stage, 1)
    else:
        outer = QVBoxLayout(self)
        outer.addWidget(stage)

    self._emg_tabs = tabs
    self._emg_page = page2
    self._emg_background = background
    page2.set_connected(self._emg_connected)

    # 波特率下拉框：HC-06 默认 9600，没有就补一项并选中
    box = getattr(self, "baud_box", None)
    if box is not None and box.findData(9600) < 0:
        box.addItem("9600", 9600)
    if box is not None and box.findData(baud) >= 0:
        box.setCurrentIndex(box.findData(baud))

    _install_dual_link(self)
    _polish_console_layout(self)

    if os.environ.get("E104_TAB", "") in ("2", "emg", "emergency"):
        tabs.setCurrentIndex(1)


def e104_refresh_status(self):
    """主界面状态栏用主板数据；应急页只认蓝牙应急装置的数据。"""
    _orig_refresh(self)
    page = getattr(self, "_emg_page", None)
    if page is not None:
        page.update_status(getattr(self, "_emg_status", None) or {})
        page.set_connected(bool(getattr(self, "_emg_connected", False)))


def e104_load_demo(self):
    """--demo：原演示数据 + 应急页演示数据。"""
    _orig_load_demo(self)
    page = getattr(self, "_emg_page", None)
    if page is None:
        return
    demo = {
        "alarm_any": 0,
        "temp_alarm": 0,
        "humi_alarm": 0,
        "flame_alarm": 0,
        "smoke_alarm": 0,
        "pir_active": 0,
        "dht_ok": 1,
        "rtc_valid": 1,
        "ble_online": 1,
        "temp": 26,
        "humi": 58,
        "flame_ao": 0x06A0,
        "smoke_ao": 0x02C0,
        "smoke": 17,
        "water": 42,
        "soil": 63,
        "smoke_do": 0,
        "soil_wet": 1,
        "rtc_h": 0x12,
        "rtc_m": 0x34,
        "rtc_s": 0x56,
    }
    self._emg_status = demo
    page.update_status(demo)
    page.set_connected(True)


ac.SerialWorker._handle = e104_handle
ac.Console._e104_send_clear = _e104_send_clear
ac.Console.__init__ = e104_init
ac.Console._refresh_status = e104_refresh_status
ac.Console.load_demo = e104_load_demo
ac.Console._emg_refresh_ports = _emg_refresh_ports
ac.Console._emg_set_ui = _emg_set_ui
ac.Console._emg_open = _emg_open
ac.Console._emg_close = _emg_close
ac.Console._emg_toggle = _emg_toggle
ac.Console._emg_poll_tick = _emg_poll_tick
ac.Console._emg_on_opened = _emg_on_opened
ac.Console._emg_on_status = _emg_on_status
ac.Console._emg_on_log = _emg_on_log
ac.Console._emg_on_reply = _emg_on_reply

_orig_close_event = ac.Console.closeEvent


def e104_close_event(self, event):
    """关闭窗口时同时停掉主板串口和蓝牙两路 worker。"""
    _emg_close(self)
    return _orig_close_event(self, event)


ac.Console.closeEvent = e104_close_event


if __name__ == "__main__":
    sys.exit(ac.main())
