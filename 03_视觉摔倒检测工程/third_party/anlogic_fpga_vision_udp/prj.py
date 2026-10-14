"""
FPGA综合控制系统 - 整合版本
整合q1.py（LED/数码管）、q2.py（图片发送）、q34.py（视频接收）的所有功能

功能模块：
1. LED模式（12个LED控制）- 发送0x0001
2. 数码管模式（8数码管+4单色LED）- 发送0x0002
3. 图片发送（640x480 RGB图片）- 发送图片数据
4. SD卡接收（接收SD卡BMP图片）- 发送0x0003
5. 摄像头接收（接收实时摄像头视频）- 发送0x0004
"""

import sys
import socket
import struct
import time
import threading
import queue
import numpy as np
import cv2
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QMessageBox, QFileDialog,
    QGroupBox, QGridLayout, QFrame, QComboBox, QStackedWidget,
    QAction, QFormLayout, QTextEdit, QSlider, QSpinBox, QCheckBox
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QColor
from PIL import Image

# ========================================================================
# 常量定义
# ========================================================================

# 网络配置
DEFAULT_FPGA_IP = "192.168.240.1"
DEFAULT_LOCAL_IP = "192.168.240.2"
DEFAULT_CONTROL_PORT = 4
DEFAULT_VIDEO_PORT = 6001

# 分辨率配置
RESOLUTION_CONFIGS = {
    '640x480': {
        'width': 640,
        'height': 480,
        'cmd': 0x0030,  # 显式设置640×480分辨率
        'display': '640×480 (VGA)',
        'pixels': 307200
    },
    '1280x720': {
        'width': 1280,
        'height': 720,
        'cmd': 0x0031,
        'display': '1280×720 (720p)',
        'pixels': 921600
    },
    '1920x1080': {
        'width': 1920,
        'height': 1080,
        'cmd': 0x0032,
        'display': '1920×1080 (1080p)',
        'pixels': 2073600
    }
}

# 图片参数（默认值）
IMG_WIDTH = 640
IMG_HEIGHT = 480
PACKET_SIZE_IMG = 140

# 视频接收参数
PACKET_HEADER_SIZE = 32
PACKET_DATA_SIZE = 636
TOTAL_PACKETS_PER_FRAME = 1450
BUFFER_SIZE = 8192
FRAME_TIMEOUT = 2.0

# LED颜色定义
LED_ON_COLOR = "#3B82F6"
LED_OFF_COLOR = "#9E9E9E"
LED_RED_ON = "#ff4d4f"
LED_GREEN_ON = "#52c41a"
STATUS_SUCCESS = "#52c41a"
STATUS_ERROR = "#e74c3c"

# ========================================================================
# UDP视频接收器类
# ========================================================================

class UDPCameraReceiver:
    """UDP摄像头数据接收器（后台线程）"""
    def __init__(self, host_ip, host_port, packet_queue):
        self.host_ip = host_ip
        self.host_port = host_port
        self.packet_queue = packet_queue
        self.sock = None
        self.running_event = threading.Event()

    def start(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 * 1024 * 1024)
            self.sock.bind((self.host_ip, self.host_port))
            self.sock.settimeout(1.0)
            self.running_event.set()
            threading.Thread(target=self._receive_loop, daemon=True).start()
            print(f"[INFO] UDP接收器启动: {self.host_ip}:{self.host_port}")
            return True
        except Exception as e:
            print(f"[ERROR] 无法启动UDP接收器: {e}")
            return False

    def stop(self):
        self.running_event.clear()
        if self.sock:
            self.sock.close()
        print("[INFO] UDP接收器已停止")

    def _receive_loop(self):
        while self.running_event.is_set():
            try:
                data, _ = self.sock.recvfrom(BUFFER_SIZE)
                if data:
                    try:
                        self.packet_queue.put_nowait(data)
                    except queue.Full:
                        pass
            except socket.timeout:
                continue
            except Exception:
                if self.running_event.is_set():
                    print("[ERROR] 接收循环异常")
                break

# ========================================================================
# LED控件
# ========================================================================

class LEDWidget(QFrame):
    """LED显示控件"""
    def __init__(self, color=LED_OFF_COLOR, width=60, height=40):
        super().__init__()
        self.is_on = False
        self.on_color = color
        self.off_color = LED_OFF_COLOR
        self.setFixedSize(width, height)
        self.setFrameShape(QFrame.Box)
        self.setLineWidth(2)
        self.update_color()

    def set_state(self, state):
        self.is_on = state
        self.update_color()

    def toggle(self):
        self.is_on = not self.is_on
        self.update_color()

    def update_color(self):
        color = self.on_color if self.is_on else self.off_color
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {color};
                border: 2px solid #555;
                border-radius: 8px;
            }}
        """)

    def mousePressEvent(self, event):
        self.toggle()

# ========================================================================
# 数码管选择器
# ========================================================================

class DigitSelector(QComboBox):
    """数码管数字选择器"""
    def __init__(self):
        super().__init__()
        self.addItems(['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ' '])
        self.setCurrentIndex(0)
        self.setFixedWidth(60)
        self.setStyleSheet("""
            QComboBox {
                font-size: 16px;
                font-weight: bold;
                padding: 5px;
            }
        """)

    def get_value(self):
        text = self.currentText()
        if text == ' ':
            return 0xE  # 空格/不显示
        else:
            return int(text)

# ========================================================================
# 主应用程序
# ========================================================================

class FPGAControlApp(QMainWindow):
    """FPGA综合控制系统主窗口"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("FPGA综合控制系统 - LED/数码管/图片/视频")
        self.setGeometry(100, 100, 1200, 800)

        # 网络配置
        self.fpga_ip = DEFAULT_FPGA_IP
        self.local_ip = DEFAULT_LOCAL_IP
        self.control_port = DEFAULT_CONTROL_PORT
        self.video_port = DEFAULT_VIDEO_PORT
        self.connected = False

        # LED/数码管状态
        self.mono_led_states = [False] * 4
        self.dual_red_states = [False] * 8
        self.dual_green_states = [False] * 8
        self.mono_led_widgets = []
        self.mono_led_widgets_seg = []
        self.dual_red_widgets = []
        self.dual_green_widgets = []
        self.digit_selectors = []

        # 图片发送状态
        self.selected_image = None
        self.current_resolution = '640x480'  # 当前选择的分辨率
        self.auto_select_resolution = True   # 默认启用自动选择分辨率

        # 视频接收状态
        self.receiver = None
        self.is_receiving = False
        self.packet_queue = queue.Queue(maxsize=TOTAL_PACKETS_PER_FRAME * 2)
        self.frame_packets = {}
        self.current_frame_seq = -1
        self.last_packet_time = 0
        self.fps = 0.0
        self.last_fps_time = time.time()
        self.fps_frame_count = 0
        self.cv2_window_name = "FPGA Camera"
        self.is_sd_card_mode = False  # 标识是否为SD卡模式

        # UDP带宽计算（单目）
        self.udp_bytes_received = 0
        self.udp_bandwidth_mbps = 0.0
        self.last_bandwidth_time = time.time()

        # 图片保存状态
        self.saved_image_counter = 0  # 保存图片计数器
        self.current_single_frame = None  # 当前单目帧（用于保存）

        # 灰度阈值配置（单目模式用）
        self.binary_threshold_low = 50
        self.binary_threshold_high = 200

        # RGB颜色阈值配置（双目测距模式用，默认识别纯红色）
        self.binary_threshold_r_low = 200
        self.binary_threshold_r_high = 255
        self.binary_threshold_g_low = 0
        self.binary_threshold_g_high = 80
        self.binary_threshold_b_low = 0
        self.binary_threshold_b_high = 80

        # 边框检测状态
        self.border_enable = False       # 默认关闭边框检测

        # 左右目选择状态（新增）
        self.current_camera_select = 'left'  # 'left' or 'right'
        self.current_image_proc_mode = 0x00  # 当前图像处理模式

        # 双目接收状态（新增）
        self.is_dual_cam_receiving = False
        self.dual_cam_receiver = None
        self.dual_cam_packet_queue = queue.Queue(maxsize=TOTAL_PACKETS_PER_FRAME * 2)
        self.dual_cam_frame_packets = {}
        self.dual_cam_current_frame_seq = -1
        self.dual_cam_last_packet_time = 0
        self.dual_cam_fps = 0.0
        self.dual_cam_last_fps_time = time.time()
        self.dual_cam_fps_frame_count = 0
        self.dual_cam_left_window_name = "左目图像"
        self.dual_cam_right_window_name = "右目图像"

        # 双目字节偏移配置（关键！用于修正颜色）
        self.dual_cam_shift_bytes = 1  # 全帧字节偏移量（0~3），根据FPGA数据流调整

        # 模板匹配状态
        self.template_image = None       # 32x32模板图像
        self.match_result = None         # 匹配结果：True=匹配, False=不匹配, None=未匹配（实时从FPGA接收）
        self.match_threshold = 50        # 匹配阈值（不同像素数）

        # 双目测距状态（新增）
        self.stereo_ranging_mode = False  # 双目测距模式：False=RGB, True=测距
        self.stereo_distance_cm = 0       # 当前测距距离（cm）

        # UDP带宽计算（双目）
        self.dual_cam_bytes_received = 0
        self.dual_cam_bandwidth_mbps = 0.0
        self.dual_cam_last_bandwidth_time = time.time()

        # 双目当前帧（用于保存）
        self.current_left_frame = None
        self.current_right_frame = None

        # 初始化UI
        self.init_ui()

        # 视频处理定时器
        self.process_timer = QTimer(self)
        self.process_timer.timeout.connect(self.process_video_packets)

        # 双目处理定时器（新增）
        self.dual_cam_process_timer = QTimer(self)
        self.dual_cam_process_timer.timeout.connect(self.process_dual_cam_packets)

    def init_ui(self):
        """初始化用户界面"""
        # 创建菜单栏
        self.create_menu_bar()

        # 创建中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # 网络配置区域
        main_layout.addWidget(self.create_network_group())

        # 堆叠控件（切换不同功能页面）
        self.stacked_widget = QStackedWidget()
        self.stacked_widget.addWidget(self.create_led_page())       # 页面0: LED模式
        self.stacked_widget.addWidget(self.create_segment_page())   # 页面1: 数码管模式
        self.stacked_widget.addWidget(self.create_image_page())     # 页面2: 图片发送
        self.stacked_widget.addWidget(self.create_video_page())     # 页面3: 单目视频接收
        self.stacked_widget.addWidget(self.create_dual_cam_page())  # 页面4: 双目视频接收（新增）
        main_layout.addWidget(self.stacked_widget)

        # 状态信息栏
        main_layout.addWidget(self.create_status_group())

        # 默认显示LED模式
        self.stacked_widget.setCurrentIndex(0)

    def create_menu_bar(self):
        """创建菜单栏"""
        menubar = self.menuBar()
        mode_menu = menubar.addMenu('功能切换')

        # LED模式
        led_action = QAction('1. LED模式 (12个LED)', self)
        led_action.setShortcut('Ctrl+1')
        led_action.triggered.connect(lambda: self.switch_mode(0, 0x0001, "LED"))
        mode_menu.addAction(led_action)

        # 数码管模式
        seg_action = QAction('2. 数码管模式 (8数码管+4单色LED)', self)
        seg_action.setShortcut('Ctrl+2')
        seg_action.triggered.connect(lambda: self.switch_mode(1, 0x0002, "数码管"))
        mode_menu.addAction(seg_action)

        # 图片发送
        img_action = QAction('3. 图片发送 (640x480)', self)
        img_action.setShortcut('Ctrl+3')
        img_action.triggered.connect(lambda: self.switch_mode(2, 0x0001, "图片发送"))
        mode_menu.addAction(img_action)

        mode_menu.addSeparator()

        # SD卡接收
        sd_action = QAction('4. SD卡接收 (BMP图片)', self)
        sd_action.setShortcut('Ctrl+4')
        sd_action.triggered.connect(lambda: self.switch_mode(3, 0x0003, "SD卡接收"))
        mode_menu.addAction(sd_action)

        # 摄像头接收
        cam_action = QAction('5. 摄像头接收 (单目实时视频)', self)
        cam_action.setShortcut('Ctrl+5')
        cam_action.triggered.connect(lambda: self.switch_mode(3, 0x0004, "单目摄像头接收"))
        mode_menu.addAction(cam_action)

        # 双目摄像头接收（新增）
        dual_cam_action = QAction('6. 双目摄像头接收', self)
        dual_cam_action.setShortcut('Ctrl+6')
        dual_cam_action.triggered.connect(lambda: self.switch_mode(4, 0x0011, "双目摄像头接收"))
        mode_menu.addAction(dual_cam_action)

        mode_menu.addSeparator()

        # 退出
        exit_action = QAction('退出', self)
        exit_action.setShortcut('Ctrl+Q')
        exit_action.triggered.connect(self.close)
        mode_menu.addAction(exit_action)

    def switch_mode(self, page_index, cmd_value, mode_name):
        """切换功能模式"""
        # 停止视频接收（单目和双目）
        if self.is_receiving:
            self.stop_video_receiving()
        if self.is_dual_cam_receiving:
            self.stop_dual_cam_receiving()

        # 设置SD卡模式标志（SD卡接收命令为0x0003）
        self.is_sd_card_mode = (cmd_value == 0x0003)

        # 切换页面
        self.stacked_widget.setCurrentIndex(page_index)

        # 发送模式切换命令
        if self.connected:
            self.send_mode_command(cmd_value)
            self.update_status(f"已切换到{mode_name}模式", STATUS_SUCCESS)
        else:
            self.update_status(f"已切换到{mode_name}模式（未连接）", STATUS_ERROR)

    def send_mode_command(self, cmd_value):
        """发送模式切换命令（2字节大端序）"""
        try:
            mode_data = struct.pack('!H', cmd_value)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sent = sock.sendto(mode_data, (self.fpga_ip, self.control_port))
            sock.close()
            print(f"[INFO] 发送模式命令: 0x{cmd_value:04X}, 已发送 {sent} 字节")
        except Exception as e:
            print(f"[ERROR] 发送模式命令失败: {e}")

    # ====================================================================
    # 网络配置区域
    # ====================================================================

    def create_network_group(self):
        """创建网络配置组"""
        group = QGroupBox("网络配置")
        layout = QHBoxLayout()

        layout.addWidget(QLabel("本机IP:"))
        self.local_ip_edit = QLineEdit(self.local_ip)
        self.local_ip_edit.setFixedWidth(120)
        layout.addWidget(self.local_ip_edit)

        layout.addWidget(QLabel("FPGA IP:"))
        self.fpga_ip_edit = QLineEdit(self.fpga_ip)
        self.fpga_ip_edit.setFixedWidth(120)
        layout.addWidget(self.fpga_ip_edit)

        layout.addWidget(QLabel("控制端口:"))
        self.control_port_edit = QLineEdit(str(self.control_port))
        self.control_port_edit.setFixedWidth(60)
        layout.addWidget(self.control_port_edit)

        layout.addWidget(QLabel("视频端口:"))
        self.video_port_edit = QLineEdit(str(self.video_port))
        self.video_port_edit.setFixedWidth(60)
        layout.addWidget(self.video_port_edit)

        self.connect_btn = QPushButton("连接")
        self.connect_btn.setFixedWidth(100)
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_btn)

        layout.addStretch()
        group.setLayout(layout)
        return group

    def toggle_connection(self):
        """切换连接状态"""
        if not self.connected:
            try:
                self.local_ip = self.local_ip_edit.text().strip()
                self.fpga_ip = self.fpga_ip_edit.text().strip()
                self.control_port = int(self.control_port_edit.text().strip())
                self.video_port = int(self.video_port_edit.text().strip())

                # 验证IP格式
                socket.inet_aton(self.local_ip)
                socket.inet_aton(self.fpga_ip)

                self.connected = True
                self.connect_btn.setText("断开")
                self.connect_btn.setStyleSheet("background-color: #e74c3c;")
                self.update_status(f"已连接: {self.local_ip} -> {self.fpga_ip}:{self.control_port}", STATUS_SUCCESS)

                # 禁用网络配置编辑
                self.local_ip_edit.setEnabled(False)
                self.fpga_ip_edit.setEnabled(False)
                self.control_port_edit.setEnabled(False)
                self.video_port_edit.setEnabled(False)

            except ValueError:
                QMessageBox.critical(self, "错误", "端口号必须为数字")
            except socket.error:
                QMessageBox.critical(self, "错误", "IP地址格式不正确")
        else:
            # 断开连接
            if self.is_receiving:
                self.stop_video_receiving()

            self.connected = False
            self.connect_btn.setText("连接")
            self.connect_btn.setStyleSheet("")
            self.update_status("已断开连接", STATUS_ERROR)

            # 启用网络配置编辑
            self.local_ip_edit.setEnabled(True)
            self.fpga_ip_edit.setEnabled(True)
            self.control_port_edit.setEnabled(True)
            self.video_port_edit.setEnabled(True)

    # ====================================================================
    # 页面1: LED模式
    # ====================================================================

    def create_led_page(self):
        """创建LED控制页面"""
        page = QWidget()
        layout = QVBoxLayout(page)

        # 标题
        title = QLabel("📍 LED模式：控制12个LED（4个单色 + 8个双色）")
        title.setStyleSheet("font-size: 14px; font-weight: bold; color: #3498db; padding: 10px;")
        layout.addWidget(title)

        # 单色LED区域
        mono_group = QGroupBox("单色LED (1-4)")
        mono_layout = QHBoxLayout()
        for i in range(4):
            led_layout = QVBoxLayout()
            led_layout.setAlignment(Qt.AlignCenter)
            led_layout.addWidget(QLabel(f"单色{i+1}", alignment=Qt.AlignCenter))
            led = LEDWidget(LED_ON_COLOR, 80, 50)
            led.mousePressEvent = lambda e, idx=i: self.toggle_mono_led(idx)
            self.mono_led_widgets.append(led)
            led_layout.addWidget(led)
            mono_layout.addLayout(led_layout)
        mono_group.setLayout(mono_layout)
        layout.addWidget(mono_group)

        # 双色LED区域
        dual_group = QGroupBox("双色LED (1-8)")
        dual_layout = QHBoxLayout()
        for i in range(8):
            led_container = QVBoxLayout()
            led_container.setAlignment(Qt.AlignCenter)
            led_container.addWidget(QLabel(f"双色{i+1}", alignment=Qt.AlignCenter))

            leds_layout = QHBoxLayout()
            # 红色
            red_layout = QVBoxLayout()
            red_layout.addWidget(QLabel("红", alignment=Qt.AlignCenter))
            red_led = LEDWidget(LED_RED_ON, 40, 50)
            red_led.mousePressEvent = lambda e, idx=i: self.toggle_dual_red(idx)
            self.dual_red_widgets.append(red_led)
            red_layout.addWidget(red_led)
            leds_layout.addLayout(red_layout)

            # 绿色
            green_layout = QVBoxLayout()
            green_layout.addWidget(QLabel("绿", alignment=Qt.AlignCenter))
            green_led = LEDWidget(LED_GREEN_ON, 40, 50)
            green_led.mousePressEvent = lambda e, idx=i: self.toggle_dual_green(idx)
            self.dual_green_widgets.append(green_led)
            green_layout.addWidget(green_led)
            leds_layout.addLayout(green_layout)

            led_container.addLayout(leds_layout)
            dual_layout.addLayout(led_container)
        dual_group.setLayout(dual_layout)
        layout.addWidget(dual_group)

        # 控制按钮
        btn_layout = QHBoxLayout()
        all_on_btn = QPushButton("全部打开")
        all_on_btn.clicked.connect(self.led_all_on)
        btn_layout.addWidget(all_on_btn)

        all_off_btn = QPushButton("全部关闭")
        all_off_btn.clicked.connect(self.led_all_off)
        btn_layout.addWidget(all_off_btn)

        send_btn = QPushButton("发送LED数据")
        send_btn.clicked.connect(self.send_led_data)
        btn_layout.addWidget(send_btn)

        btn_layout.addStretch()
        layout.addLayout(btn_layout)
        layout.addStretch()

        return page

    def toggle_mono_led(self, idx):
        """切换单色LED状态"""
        self.mono_led_states[idx] = not self.mono_led_states[idx]
        if idx < len(self.mono_led_widgets):
            self.mono_led_widgets[idx].set_state(self.mono_led_states[idx])
        if idx < len(self.mono_led_widgets_seg):
            self.mono_led_widgets_seg[idx].set_state(self.mono_led_states[idx])
        if self.connected:
            self.send_led_data()

    def toggle_dual_red(self, idx):
        """切换双色LED红色"""
        self.dual_red_states[idx] = not self.dual_red_states[idx]
        self.dual_red_widgets[idx].set_state(self.dual_red_states[idx])
        if self.connected:
            self.send_led_data()

    def toggle_dual_green(self, idx):
        """切换双色LED绿色"""
        self.dual_green_states[idx] = not self.dual_green_states[idx]
        self.dual_green_widgets[idx].set_state(self.dual_green_states[idx])
        if self.connected:
            self.send_led_data()

    def led_all_on(self):
        """打开所有LED"""
        for i in range(4):
            self.mono_led_states[i] = True
            if i < len(self.mono_led_widgets):
                self.mono_led_widgets[i].set_state(True)
            if i < len(self.mono_led_widgets_seg):
                self.mono_led_widgets_seg[i].set_state(True)
        for i in range(8):
            self.dual_red_states[i] = True
            self.dual_green_states[i] = True
            self.dual_red_widgets[i].set_state(True)
            self.dual_green_widgets[i].set_state(True)
        if self.connected:
            self.send_led_data()

    def led_all_off(self):
        """关闭所有LED"""
        for i in range(4):
            self.mono_led_states[i] = False
            if i < len(self.mono_led_widgets):
                self.mono_led_widgets[i].set_state(False)
            if i < len(self.mono_led_widgets_seg):
                self.mono_led_widgets_seg[i].set_state(False)
        for i in range(8):
            self.dual_red_states[i] = False
            self.dual_green_states[i] = False
            self.dual_red_widgets[i].set_state(False)
            self.dual_green_widgets[i].set_state(False)
        if self.connected:
            self.send_led_data()

    def send_led_data(self):
        """发送LED控制数据（3字节）"""
        if not self.connected:
            self.update_status("请先连接", STATUS_ERROR)
            return

        try:
            # 构建单色LED掩码
            mask_single = 0
            for i in range(4):
                if self.mono_led_states[i]:
                    mask_single |= (1 << (3 - i))

            # 绿色掩码（低电平有效）
            green_mask = 0xFF
            for i in range(8):
                if self.dual_green_states[i]:
                    green_mask &= ~(1 << i)

            # 红色掩码（低电平有效）
            red_mask = 0xFF
            for i in range(8):
                if self.dual_red_states[i]:
                    red_mask &= ~(1 << i)

            byte0 = ((mask_single & 0xF) << 4) | 0xF
            payload = bytes([byte0, green_mask, red_mask])

            # 发送数据
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sent = sock.sendto(payload, (self.fpga_ip, self.control_port))
            sock.close()

            if sent == len(payload):
                self.update_status(f"LED数据发送成功: {byte0:02X} {green_mask:02X} {red_mask:02X}", STATUS_SUCCESS)
            else:
                self.update_status(f"LED数据发送不完整 ({sent}/3)", STATUS_ERROR)

        except Exception as e:
            self.update_status(f"LED数据发送失败: {e}", STATUS_ERROR)

    # ====================================================================
    # 页面2: 数码管模式
    # ====================================================================

    def create_segment_page(self):
        """创建数码管控制页面"""
        page = QWidget()
        layout = QVBoxLayout(page)

        # 标题
        title = QLabel("📍 数码管模式：控制8个数码管 + 4个单色LED")
        title.setStyleSheet("font-size: 14px; font-weight: bold; color: #2ecc71; padding: 10px;")
        layout.addWidget(title)

        warning = QLabel("⚠️ 注意：数码管位选与8个双色LED引脚重复，此模式下双色LED不可用")
        warning.setStyleSheet("color: #f39c12; padding: 5px;")
        layout.addWidget(warning)

        # 单色LED区域（数码管模式专用）
        mono_group = QGroupBox("单色LED (1-4)")
        mono_layout = QHBoxLayout()
        for i in range(4):
            led_layout = QVBoxLayout()
            led_layout.setAlignment(Qt.AlignCenter)
            led_layout.addWidget(QLabel(f"单色{i+1}", alignment=Qt.AlignCenter))
            led = LEDWidget(LED_ON_COLOR, 80, 50)
            led.mousePressEvent = lambda e, idx=i: self.toggle_mono_led(idx)
            self.mono_led_widgets_seg.append(led)
            led_layout.addWidget(led)
            mono_layout.addLayout(led_layout)
        mono_group.setLayout(mono_layout)
        layout.addWidget(mono_group)

        # 数码管区域
        seg_group = QGroupBox("数码管控制 (8位)")
        seg_layout = QVBoxLayout()

        # 数码管选择器
        selectors_layout = QHBoxLayout()
        for i in range(8):
            digit_layout = QVBoxLayout()
            digit_layout.setAlignment(Qt.AlignCenter)
            digit_layout.addWidget(QLabel(f"LED{i}", alignment=Qt.AlignCenter))
            selector = DigitSelector()
            selector.currentIndexChanged.connect(self.send_segment_data)
            self.digit_selectors.append(selector)
            digit_layout.addWidget(selector)
            selectors_layout.addLayout(digit_layout)
        seg_layout.addLayout(selectors_layout)

        # 快捷输入
        quick_layout = QHBoxLayout()
        quick_layout.addWidget(QLabel("快捷输入（8位数字）:"))
        self.quick_input = QLineEdit()
        self.quick_input.setPlaceholderText("例如: 20251027")
        self.quick_input.setMaxLength(8)
        quick_layout.addWidget(self.quick_input)

        apply_btn = QPushButton("应用")
        apply_btn.clicked.connect(self.apply_quick_input)
        quick_layout.addWidget(apply_btn)

        clear_btn = QPushButton("清空")
        clear_btn.clicked.connect(self.clear_segments)
        quick_layout.addWidget(clear_btn)

        send_btn = QPushButton("发送数码管数据")
        send_btn.clicked.connect(self.send_segment_data)
        quick_layout.addWidget(send_btn)

        quick_layout.addStretch()
        seg_layout.addLayout(quick_layout)

        seg_group.setLayout(seg_layout)
        layout.addWidget(seg_group)
        layout.addStretch()

        return page

    def apply_quick_input(self):
        """应用快捷输入"""
        text = self.quick_input.text().strip()
        digits = ''.join(c for c in text if c.isdigit())
        if len(digits) == 0:
            return
        digits = digits[:8]
        if len(digits) < 8:
            digits = ' ' * (8 - len(digits)) + digits
        for i in range(8):
            if digits[i] == ' ':
                self.digit_selectors[i].setCurrentIndex(10)
            else:
                self.digit_selectors[i].setCurrentIndex(int(digits[i]))
        if self.connected:
            self.send_segment_data()

    def clear_segments(self):
        """清空所有数码管"""
        for selector in self.digit_selectors:
            selector.setCurrentIndex(10)
        if self.connected:
            self.send_segment_data()

    def send_segment_data(self):
        """发送数码管控制数据（8字节）"""
        if not self.connected:
            self.update_status("请先连接", STATUS_ERROR)
            return

        try:
            # 获取数码管值
            digits = [selector.get_value() for selector in self.digit_selectors]
            display_text = []
            for val in digits:
                if val == 0xE:
                    display_text.append(' ')
                else:
                    display_text.append(str(val))

            # 构建单色LED掩码
            mask_single = 0
            for i in range(4):
                if self.mono_led_states[i]:
                    mask_single |= (1 << (3 - i))

            # 构建8字节数据包
            byte0 = ((mask_single & 0xF) << 4) | 0x0
            byte1 = ((digits[0] & 0xF) << 4) | (digits[1] & 0xF)
            byte2 = ((digits[2] & 0xF) << 4) | (digits[3] & 0xF)
            byte3 = ((digits[4] & 0xF) << 4) | (digits[5] & 0xF)
            byte4 = ((digits[6] & 0xF) << 4) | (digits[7] & 0xF)

            payload = bytes([byte0, byte1, byte2, byte3, byte4, 0x00, 0x00, 0x00])

            # 发送数据
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sent = sock.sendto(payload, (self.fpga_ip, self.control_port))
            sock.close()

            if sent == len(payload):
                self.update_status(f"数码管+LED数据发送成功 | 显示: {''.join(display_text)}", STATUS_SUCCESS)
            else:
                self.update_status(f"数码管+LED数据发送不完整 ({sent}/8)", STATUS_ERROR)

        except Exception as e:
            self.update_status(f"数码管+LED数据发送失败: {e}", STATUS_ERROR)

    # ====================================================================
    # 页面3: 图片发送
    # ====================================================================

    def create_image_page(self):
        """创建图片发送页面"""
        page = QWidget()
        layout = QVBoxLayout(page)

        # 标题
        title = QLabel("📍 图片发送：发送RGB图片到FPGA HDMI显示（支持多种分辨率）")
        title.setStyleSheet("font-size: 14px; font-weight: bold; color: #e67e22; padding: 10px;")
        layout.addWidget(title)

        # 分辨率选择区域
        resolution_group = QGroupBox("分辨率选择")
        resolution_layout = QHBoxLayout()

        resolution_layout.addWidget(QLabel("目标分辨率:"))
        self.resolution_combo = QComboBox()
        for res_key in ['640x480', '1280x720', '1920x1080']:
            self.resolution_combo.addItem(RESOLUTION_CONFIGS[res_key]['display'], res_key)
        self.resolution_combo.setCurrentIndex(0)
        self.resolution_combo.currentIndexChanged.connect(self.on_resolution_changed)
        resolution_layout.addWidget(self.resolution_combo)

        self.apply_resolution_btn = QPushButton("应用分辨率")
        self.apply_resolution_btn.setStyleSheet("background-color: #3498db; color: white; padding: 5px;")
        self.apply_resolution_btn.clicked.connect(self.apply_resolution)
        resolution_layout.addWidget(self.apply_resolution_btn)

        # 添加自动选择复选框
        self.auto_resolution_checkbox = QCheckBox("智能选择分辨率")
        self.auto_resolution_checkbox.setChecked(True)
        self.auto_resolution_checkbox.setToolTip("根据图片尺寸自动推荐最合适的分辨率")
        self.auto_resolution_checkbox.stateChanged.connect(self.on_auto_resolution_toggled)
        resolution_layout.addWidget(self.auto_resolution_checkbox)

        resolution_hint = QLabel("提示：启用智能选择后，系统会根据图片尺寸自动推荐合适的分辨率")
        resolution_hint.setStyleSheet("color: #7f8c8d; padding: 5px;")
        resolution_layout.addWidget(resolution_hint)

        resolution_layout.addStretch()
        resolution_group.setLayout(resolution_layout)
        layout.addWidget(resolution_group)

        # 控制按钮
        btn_layout = QHBoxLayout()
        self.choose_img_btn = QPushButton("选择图片")
        self.choose_img_btn.clicked.connect(self.choose_image)
        btn_layout.addWidget(self.choose_img_btn)

        self.send_img_btn = QPushButton("发送图片")
        self.send_img_btn.clicked.connect(self.send_image)
        btn_layout.addWidget(self.send_img_btn)

        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        # 提示信息
        self.image_hint_label = QLabel("提示：选择的图片会自动调整为当前分辨率并水平翻转后发送")
        self.image_hint_label.setStyleSheet("color: #7f8c8d; padding: 5px;")
        layout.addWidget(self.image_hint_label)

        # 图片预览区域
        preview_frame = QFrame()
        preview_frame.setFrameShape(QFrame.Box)
        preview_frame.setMinimumHeight(400)
        preview_layout = QVBoxLayout(preview_frame)
        self.image_preview_label = QLabel("图片预览区域（选择后显示）")
        self.image_preview_label.setAlignment(Qt.AlignCenter)
        self.image_preview_label.setStyleSheet("color: #95a5a6; font-size: 16px;")
        preview_layout.addWidget(self.image_preview_label)
        layout.addWidget(preview_frame)

        # 状态标签
        self.img_status_label = QLabel("未选择图片")
        self.img_status_label.setStyleSheet("padding: 5px;")
        layout.addWidget(self.img_status_label)

        return page

    def recommend_resolution(self, img_width, img_height):
        """
        根据图片尺寸智能推荐分辨率
        规则：
        - 小于640x480 → 640x480
        - 640x480到1280x720 → 720p
        - 720p到1080p → 1080p
        - 大于1080p → 1080p
        """
        pixels = img_width * img_height

        if pixels < RESOLUTION_CONFIGS['640x480']['pixels']:
            # 小于VGA，推荐640x480
            return '640x480'
        elif pixels < RESOLUTION_CONFIGS['1280x720']['pixels']:
            # VGA到720p之间，推荐720p
            return '1280x720'
        else:
            # 720p及以上，推荐1080p
            return '1920x1080'

    def on_auto_resolution_toggled(self, state):
        """自动选择分辨率复选框切换事件"""
        self.auto_select_resolution = (state == 2)  # Qt.Checked = 2
        if self.auto_select_resolution:
            self.img_status_label.setText("✓ 已启用智能分辨率选择")
        else:
            self.img_status_label.setText("手动选择分辨率模式")

    def on_resolution_changed(self, index):
        """分辨率下拉框改变事件"""
        res_key = self.resolution_combo.currentData()
        config = RESOLUTION_CONFIGS[res_key]
        self.img_status_label.setText(f"已选择分辨率: {config['display']}")

    def send_resolution_command(self, res_key):
        """发送分辨率命令到FPGA（内部函数）"""
        config = RESOLUTION_CONFIGS[res_key]
        try:
            cmd_data = struct.pack('!H', config['cmd'])
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sent = sock.sendto(cmd_data, (self.fpga_ip, self.control_port))
            sock.close()
            print(f"[INFO] 发送分辨率命令: 0x{config['cmd']:04X}, 已发送 {sent} 字节")
            return True
        except Exception as e:
            print(f"[ERROR] 发送分辨率命令失败: {e}")
            return False

    def auto_resend_resolution(self):
        """自动重发分辨率命令（在RST后）"""
        if not self.connected:
            return

        res_key = self.current_resolution
        config = RESOLUTION_CONFIGS[res_key]

        if self.send_resolution_command(res_key):
            self.update_status(f"已自动重发分辨率命令: {config['display']}", STATUS_SUCCESS)
            self.img_status_label.setText(f"✓ 分辨率已恢复: {config['display']}")
            print(f"[INFO] 自动重发完成，当前分辨率: {config['display']}")
        else:
            self.update_status(f"自动重发失败", STATUS_ERROR)
            self.img_status_label.setText(f"❌ 自动重发失败，请手动重新设置")

    def apply_resolution(self):
        """应用分辨率设置（发送命令到FPGA）"""
        if not self.connected:
            QMessageBox.warning(self, "警告", "请先连接到FPGA")
            return

        res_key = self.resolution_combo.currentData()
        config = RESOLUTION_CONFIGS[res_key]

        # 发送分辨率切换命令
        if self.send_resolution_command(res_key):
            self.current_resolution = res_key
            self.update_status(f"分辨率已切换为 {config['display']}", STATUS_SUCCESS)
            self.img_status_label.setText(f"✓ 分辨率已应用: {config['display']}")

            # 显示复位提示并等待用户确认
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Information)
            msg.setWindowTitle("分辨率切换成功")
            msg.setText(f"✓ 分辨率已切换为 {config['display']}\n\n"
                       f"⚠️ 由于动态时钟切换，HDMI需要复位才能生效。\n\n"
                       f"📌 请按下FPGA开发板上的 RST 复位键。")
            msg.setInformativeText("复位后，上位机将自动重新发送分辨率命令。\n"
                                  "请按RST后点击确定，或等待3秒自动重发。")
            msg.setStandardButtons(QMessageBox.Ok)

            # 显示对话框（阻塞）
            msg.exec_()

            # 用户点击OK后，等待3秒，然后自动重发命令
            self.img_status_label.setText("⏳ 等待3秒后自动重发命令...")
            QApplication.processEvents()

            # 使用QTimer延迟3秒后重发
            QTimer.singleShot(3000, self.auto_resend_resolution)

        else:
            self.update_status(f"分辨率切换失败", STATUS_ERROR)
            self.img_status_label.setText(f"❌ 分辨率切换失败")
            QMessageBox.critical(self, "错误", f"分辨率命令发送失败，请检查网络连接")

    def choose_image(self):
        """选择图片"""
        config = RESOLUTION_CONFIGS[self.current_resolution]
        filepath, _ = QFileDialog.getOpenFileName(
            self,
            f"选择图片（会自动调整为{config['display']}）",
            "",
            "Image Files (*.png *.jpg *.jpeg *.bmp);;All Files (*.*)"
        )
        if not filepath:
            self.img_status_label.setText("未选择图片")
            return

        try:
            img = Image.open(filepath).convert("RGB")
            w, h = img.size
            self.selected_image = img

            # 智能推荐分辨率
            if self.auto_select_resolution:
                recommended_res = self.recommend_resolution(w, h)
                recommended_config = RESOLUTION_CONFIGS[recommended_res]

                # 如果推荐的分辨率与当前不同，提示用户并自动切换
                if recommended_res != self.current_resolution:
                    # 自动切换下拉框
                    for i in range(self.resolution_combo.count()):
                        if self.resolution_combo.itemData(i) == recommended_res:
                            self.resolution_combo.setCurrentIndex(i)
                            break

                    self.img_status_label.setText(
                        f"✓ 已选择: {w}×{h} | 智能推荐: {recommended_config['display']} "
                        f"（图片像素: {w*h:,}）"
                    )

                    # 显示提示信息
                    msg = QMessageBox()
                    msg.setIcon(QMessageBox.Information)
                    msg.setWindowTitle("智能分辨率推荐")
                    msg.setText(f"图片尺寸: {w}×{h} ({w*h:,} 像素)\n\n"
                               f"推荐分辨率: {recommended_config['display']}\n\n"
                               f"已自动切换到推荐分辨率，点击\"应用分辨率\"按钮通知FPGA。")
                    msg.setStandardButtons(QMessageBox.Ok)
                    msg.exec_()
                else:
                    self.img_status_label.setText(
                        f"✓ 已选择: {w}×{h} | 当前分辨率已是最佳: {config['display']}"
                    )
            else:
                # 手动模式，显示基本信息
                self.img_status_label.setText(
                    f"已选择: {w}×{h}（将调整为{config['display']}并水平翻转）"
                )

        except Exception as e:
            self.img_status_label.setText(f"选择失败: {e}")
            self.selected_image = None

    def send_image(self):
        """发送图片到FPGA"""
        if not self.connected:
            self.img_status_label.setText("请先连接")
            return

        if self.selected_image is None:
            self.img_status_label.setText("请先选择图片")
            return

        # 获取当前分辨率配置
        config = RESOLUTION_CONFIGS[self.current_resolution]
        target_width = config['width']
        target_height = config['height']

        img = self.selected_image
        original_size = img.size

        # 调整尺寸到目标分辨率
        if img.size != (target_width, target_height):
            self.img_status_label.setText(f"正在调整图片尺寸到 {config['display']}...")
            QApplication.processEvents()
            img = img.resize((target_width, target_height), Image.BILINEAR)

        # 水平翻转
        img = img.transpose(Image.FLIP_LEFT_RIGHT)

        try:
            pixel_data = img.tobytes()
            total = len(pixel_data)
            sent = 0

            self.img_status_label.setText("正在发送图片...")
            QApplication.processEvents()

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # 发送数据
            last_progress = -1
            while sent < total:
                chunk = pixel_data[sent: sent + PACKET_SIZE_IMG]
                sock.sendto(chunk, (self.fpga_ip, self.control_port))
                sent += len(chunk)

                # 更新进度
                progress = sent * 100 // total
                if progress - last_progress >= 5:
                    self.img_status_label.setText(f"正在发送... {progress}%")
                    QApplication.processEvents()
                    last_progress = progress

            sock.close()

            if original_size != (target_width, target_height):
                self.img_status_label.setText(
                    f"✓ 图片已发送: {original_size[0]}×{original_size[1]} → {config['display']}"
                )
            else:
                self.img_status_label.setText(f"✓ 图片发送完成 ({config['display']})")

            self.update_status(f"图片发送成功 ({config['display']})", STATUS_SUCCESS)

        except Exception as e:
            self.img_status_label.setText(f"发送失败: {e}")
            self.update_status(f"图片发送失败: {e}", STATUS_ERROR)

    # ====================================================================
    # 页面4: 视频接收
    # ====================================================================

    def create_video_page(self):
        """创建视频接收页面"""
        page = QWidget()
        layout = QVBoxLayout(page)

        # 标题
        title = QLabel("📍 视频接收：接收SD卡BMP图片或实时摄像头视频流")
        title.setStyleSheet("font-size: 14px; font-weight: bold; color: #9b59b6; padding: 10px;")
        layout.addWidget(title)

        # 提示信息
        hint = QLabel("提示：使用菜单切换SD卡接收(0x0003)或摄像头接收(0x0004)，然后点击开始接收")
        hint.setStyleSheet("color: #7f8c8d; padding: 5px;")
        layout.addWidget(hint)

        # 控制按钮
        btn_layout = QHBoxLayout()
        self.start_recv_btn = QPushButton("开始接收")
        self.start_recv_btn.setStyleSheet("background-color: #27ae60; color: white; padding: 8px;")
        self.start_recv_btn.clicked.connect(self.start_video_receiving)
        btn_layout.addWidget(self.start_recv_btn)

        self.stop_recv_btn = QPushButton("停止接收")
        self.stop_recv_btn.setStyleSheet("background-color: #e74c3c; color: white; padding: 8px;")
        self.stop_recv_btn.setEnabled(False)
        self.stop_recv_btn.clicked.connect(self.stop_video_receiving)
        btn_layout.addWidget(self.stop_recv_btn)

        self.save_frame_btn = QPushButton("保存当前图片")
        self.save_frame_btn.setStyleSheet("background-color: #f39c12; color: white; padding: 8px; font-weight: bold;")
        self.save_frame_btn.clicked.connect(self.save_current_single_frame)
        btn_layout.addWidget(self.save_frame_btn)

        self.fps_label = QLabel("FPS: --")
        self.fps_label.setStyleSheet("font-weight: bold; padding: 8px;")
        btn_layout.addWidget(self.fps_label)

        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        # 左右目选择按钮组（新增）
        camera_select_group = QGroupBox("摄像头选择（单目模式）")
        camera_select_layout = QHBoxLayout()

        self.left_cam_btn = QPushButton("左目摄像头")
        self.left_cam_btn.setStyleSheet("background-color: #27ae60; color: white; padding: 8px; min-width: 120px; font-weight: bold;")
        self.left_cam_btn.clicked.connect(lambda: self.switch_to_left_camera())
        camera_select_layout.addWidget(self.left_cam_btn)

        self.right_cam_btn = QPushButton("右目摄像头")
        self.right_cam_btn.setStyleSheet("background-color: #95a5a6; color: white; padding: 8px; min-width: 120px;")
        self.right_cam_btn.clicked.connect(lambda: self.switch_to_right_camera())
        camera_select_layout.addWidget(self.right_cam_btn)

        camera_select_hint = QLabel("提示：切换摄像头后，图像处理模式保持当前设置")
        camera_select_hint.setStyleSheet("color: #7f8c8d; padding: 5px;")
        camera_select_layout.addWidget(camera_select_hint)

        camera_select_layout.addStretch()
        camera_select_group.setLayout(camera_select_layout)
        layout.addWidget(camera_select_group)

        # 图像处理模式切换按钮组
        mode_group = QGroupBox("图像处理模式（仅单目模式有效）")
        mode_layout = QHBoxLayout()

        self.mode_original_btn = QPushButton("原始图像")
        self.mode_original_btn.setStyleSheet("background-color: #3498db; color: white; padding: 8px; min-width: 100px;")
        self.mode_original_btn.clicked.connect(lambda: self.switch_image_proc_mode(0x00))
        mode_layout.addWidget(self.mode_original_btn)

        self.mode_binary_btn = QPushButton("二值化")
        self.mode_binary_btn.setStyleSheet("background-color: #9b59b6; color: white; padding: 8px; min-width: 100px;")
        self.mode_binary_btn.clicked.connect(lambda: self.switch_image_proc_mode(0x01))
        mode_layout.addWidget(self.mode_binary_btn)

        self.mode_erode_btn = QPushButton("腐蚀")
        self.mode_erode_btn.setStyleSheet("background-color: #e67e22; color: white; padding: 8px; min-width: 100px;")
        self.mode_erode_btn.clicked.connect(lambda: self.switch_image_proc_mode(0x02))
        mode_layout.addWidget(self.mode_erode_btn)

        self.mode_opening_btn = QPushButton("开运算")
        self.mode_opening_btn.setStyleSheet("background-color: #16a085; color: white; padding: 8px; min-width: 100px;")
        self.mode_opening_btn.clicked.connect(lambda: self.switch_image_proc_mode(0x03))
        mode_layout.addWidget(self.mode_opening_btn)

        self.mode_relief_btn = QPushButton("浮雕效果")
        self.mode_relief_btn.setStyleSheet("background-color: #c0392b; color: white; padding: 8px; min-width: 100px;")
        self.mode_relief_btn.clicked.connect(lambda: self.switch_image_proc_mode(0x04))
        mode_layout.addWidget(self.mode_relief_btn)

        self.mode_laplacian_btn = QPushButton("拉普拉斯锐化")
        self.mode_laplacian_btn.setStyleSheet("background-color: #8e44ad; color: white; padding: 8px; min-width: 120px;")
        self.mode_laplacian_btn.clicked.connect(lambda: self.switch_image_proc_mode(0x05))
        mode_layout.addWidget(self.mode_laplacian_btn)

        self.mode_sobel_btn = QPushButton("Sobel边缘")
        self.mode_sobel_btn.setStyleSheet("background-color: #2c3e50; color: white; padding: 8px; min-width: 100px;")
        self.mode_sobel_btn.clicked.connect(lambda: self.switch_image_proc_mode(0x06))
        mode_layout.addWidget(self.mode_sobel_btn)

        self.mode_framediff_btn = QPushButton("帧差法")
        self.mode_framediff_btn.setStyleSheet("background-color: #d35400; color: white; padding: 8px; min-width: 100px;")
        self.mode_framediff_btn.clicked.connect(lambda: self.switch_image_proc_mode(0x07))
        mode_layout.addWidget(self.mode_framediff_btn)

        mode_layout.addStretch()
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)

        # 模板匹配功能区
        template_group = QGroupBox("实时模板匹配（32×32二值化图像）")
        template_layout = QVBoxLayout()

        # 匹配结果显示（大字）
        self.match_result_label = QLabel("等待上传模板...")
        self.match_result_label.setStyleSheet("""
            QLabel {
                font-size: 24px;
                font-weight: bold;
                color: #7f8c8d;
                padding: 15px;
                border: 2px solid #bdc3c7;
                border-radius: 8px;
                background-color: #ecf0f1;
            }
        """)
        self.match_result_label.setAlignment(Qt.AlignCenter)
        template_layout.addWidget(self.match_result_label)

        # 只保留上传模板按钮
        self.upload_template_btn = QPushButton("上传模板图像")
        self.upload_template_btn.setStyleSheet("background-color: #3498db; color: white; padding: 8px; min-width: 150px; font-weight: bold;")
        self.upload_template_btn.clicked.connect(self.upload_template_image)
        template_layout.addWidget(self.upload_template_btn)

        # 提示信息
        template_hint = QLabel("提示：上传32×32模板图像后，开启视频流，FPGA自动实时匹配，结果叠加显示在视频左上角（✓对/✗错）")
        template_hint.setStyleSheet("color: #7f8c8d; padding: 5px;")
        template_layout.addWidget(template_hint)

        template_group.setLayout(template_layout)
        layout.addWidget(template_group)

        # 边框检测开关区域
        border_group = QGroupBox("边框检测（检测二值化图像中的白色区域并绘制边框）")
        border_layout = QHBoxLayout()

        self.border_toggle_btn = QPushButton("边框检测：关闭")
        self.border_toggle_btn.setStyleSheet("background-color: #95a5a6; color: white; padding: 10px; min-width: 150px; font-weight: bold;")
        self.border_toggle_btn.clicked.connect(self.toggle_border_detection)
        border_layout.addWidget(self.border_toggle_btn)

        border_hint = QLabel("提示：边框使能后将显示二值化图像，并在原始彩色图像上用绿色框标记检测到的白色区域")
        border_hint.setStyleSheet("color: #7f8c8d; padding: 5px;")
        border_layout.addWidget(border_hint)

        border_layout.addStretch()
        border_group.setLayout(border_layout)
        layout.addWidget(border_group)

        # 灰度阈值配置区域（单目模式）
        threshold_group = QGroupBox("灰度二值化阈值配置（仅二值化/腐蚀/开运算模式有效）")
        threshold_layout = QVBoxLayout()

        # 低阈值调节
        low_threshold_layout = QHBoxLayout()
        low_threshold_layout.addWidget(QLabel("低阈值 (0-255):"))

        self.low_threshold_slider = QSlider(Qt.Horizontal)
        self.low_threshold_slider.setMinimum(0)
        self.low_threshold_slider.setMaximum(255)
        self.low_threshold_slider.setValue(self.binary_threshold_low)
        self.low_threshold_slider.setTickPosition(QSlider.TicksBelow)
        self.low_threshold_slider.setTickInterval(25)
        self.low_threshold_slider.valueChanged.connect(self.on_gray_low_threshold_changed)
        low_threshold_layout.addWidget(self.low_threshold_slider)

        self.low_threshold_spinbox = QSpinBox()
        self.low_threshold_spinbox.setMinimum(0)
        self.low_threshold_spinbox.setMaximum(255)
        self.low_threshold_spinbox.setValue(self.binary_threshold_low)
        self.low_threshold_spinbox.setFixedWidth(70)
        self.low_threshold_spinbox.valueChanged.connect(self.on_gray_low_threshold_changed)
        low_threshold_layout.addWidget(self.low_threshold_spinbox)

        threshold_layout.addLayout(low_threshold_layout)

        # 高阈值调节
        high_threshold_layout = QHBoxLayout()
        high_threshold_layout.addWidget(QLabel("高阈值 (0-255):"))

        self.high_threshold_slider = QSlider(Qt.Horizontal)
        self.high_threshold_slider.setMinimum(0)
        self.high_threshold_slider.setMaximum(255)
        self.high_threshold_slider.setValue(self.binary_threshold_high)
        self.high_threshold_slider.setTickPosition(QSlider.TicksBelow)
        self.high_threshold_slider.setTickInterval(25)
        self.high_threshold_slider.valueChanged.connect(self.on_gray_high_threshold_changed)
        high_threshold_layout.addWidget(self.high_threshold_slider)

        self.high_threshold_spinbox = QSpinBox()
        self.high_threshold_spinbox.setMinimum(0)
        self.high_threshold_spinbox.setMaximum(255)
        self.high_threshold_spinbox.setValue(self.binary_threshold_high)
        self.high_threshold_spinbox.setFixedWidth(70)
        self.high_threshold_spinbox.valueChanged.connect(self.on_gray_high_threshold_changed)
        high_threshold_layout.addWidget(self.high_threshold_spinbox)

        threshold_layout.addLayout(high_threshold_layout)

        # 阈值应用按钮
        apply_threshold_layout = QHBoxLayout()
        self.apply_gray_threshold_btn = QPushButton("应用灰度阈值")
        self.apply_gray_threshold_btn.setStyleSheet("background-color: #f39c12; color: white; padding: 8px; min-width: 150px;")
        self.apply_gray_threshold_btn.clicked.connect(self.send_gray_threshold_config)
        apply_threshold_layout.addWidget(self.apply_gray_threshold_btn)

        self.gray_threshold_status_label = QLabel(f"当前: 低={self.binary_threshold_low}, 高={self.binary_threshold_high}, 平均={(self.binary_threshold_low + self.binary_threshold_high) // 2}")
        self.gray_threshold_status_label.setStyleSheet("padding: 5px; color: #7f8c8d;")
        apply_threshold_layout.addWidget(self.gray_threshold_status_label)
        apply_threshold_layout.addStretch()

        threshold_layout.addLayout(apply_threshold_layout)

        threshold_group.setLayout(threshold_layout)
        layout.addWidget(threshold_group)

        # 日志显示
        log_group = QGroupBox("接收日志")
        log_layout = QVBoxLayout()
        self.video_log = QTextEdit()
        self.video_log.setReadOnly(True)
        self.video_log.setFont(QFont("Courier New", 9))
        self.video_log.setMaximumHeight(200)
        log_layout.addWidget(self.video_log)
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)

        layout.addStretch()
        return page

    def start_video_receiving(self):
        """开始视频接收"""
        if not self.connected:
            QMessageBox.warning(self, "警告", "请先连接到FPGA")
            return

        if self.is_receiving:
            return

        # 清空队列和缓存
        while not self.packet_queue.empty():
            self.packet_queue.get()
        self.frame_packets.clear()
        self.current_frame_seq = -1
        self.last_packet_time = time.time()

        # 启动UDP接收器
        self.receiver = UDPCameraReceiver(self.local_ip, self.video_port, self.packet_queue)
        if not self.receiver.start():
            self.video_log_append("[ERROR] 无法启动UDP接收器")
            return

        self.is_receiving = True
        self.start_recv_btn.setEnabled(False)
        self.stop_recv_btn.setEnabled(True)
        self.video_log_append(f"[INFO] 开始接收视频，监听 {self.local_ip}:{self.video_port}")

        # 创建OpenCV窗口
        cv2.namedWindow(self.cv2_window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.cv2_window_name, IMG_WIDTH, IMG_HEIGHT)

        # 启动处理定时器
        self.process_timer.start(20)

    def switch_to_left_camera(self):
        """切换到左目摄像头"""
        if not self.connected:
            self.video_log_append("[WARN] 请先连接到FPGA")
            return

        self.current_camera_select = 'left'
        # 更新按钮样式
        self.left_cam_btn.setStyleSheet("background-color: #27ae60; color: white; padding: 8px; min-width: 120px; font-weight: bold;")
        self.right_cam_btn.setStyleSheet("background-color: #95a5a6; color: white; padding: 8px; min-width: 120px;")

        # 发送左目+当前图像处理模式命令
        self._send_camera_command('left', self.current_image_proc_mode)

    def switch_to_right_camera(self):
        """切换到右目摄像头"""
        if not self.connected:
            self.video_log_append("[WARN] 请先连接到FPGA")
            return

        self.current_camera_select = 'right'
        # 更新按钮样式
        self.left_cam_btn.setStyleSheet("background-color: #95a5a6; color: white; padding: 8px; min-width: 120px;")
        self.right_cam_btn.setStyleSheet("background-color: #27ae60; color: white; padding: 8px; min-width: 120px; font-weight: bold;")

        # 发送右目+当前图像处理模式命令
        self._send_camera_command('right', self.current_image_proc_mode)

    def switch_image_proc_mode(self, proc_mode):
        """切换图像处理模式（作用于当前选中的摄像头）"""
        if not self.connected:
            self.video_log_append("[WARN] 请先连接到FPGA")
            return

        self.current_image_proc_mode = proc_mode
        # 发送当前摄像头+新图像处理模式命令
        self._send_camera_command(self.current_camera_select, proc_mode)

    def _send_camera_command(self, camera, proc_mode):
        """发送摄像头+图像处理模式命令"""
        # 命令映射表
        cmd_map = {
            ('left', 0x00): 0x0004,   # 左目+原始
            ('left', 0x01): 0x0005,   # 左目+二值化
            ('left', 0x02): 0x0006,   # 左目+腐蚀
            ('left', 0x03): 0x0007,   # 左目+开运算
            ('left', 0x04): 0x000A,   # 左目+浮雕效果
            ('left', 0x05): 0x000B,   # 左目+拉普拉斯锐化
            ('left', 0x06): 0x000C,   # 左目+Sobel边缘检测
            ('left', 0x07): 0x000D,   # 左目+帧差法
            ('right', 0x00): 0x0010,  # 右目+原始
            ('right', 0x01): 0x0012,  # 右目+二值化
            ('right', 0x02): 0x0013,  # 右目+腐蚀
            ('right', 0x03): 0x0014,  # 右目+开运算
            ('right', 0x04): 0x0015,  # 右目+浮雕效果
            ('right', 0x05): 0x0016,  # 右目+拉普拉斯锐化
            ('right', 0x06): 0x0017,  # 右目+Sobel边缘检测
        }

        mode_cmd = cmd_map.get((camera, proc_mode))
        if mode_cmd is None:
            self.video_log_append(f"[ERROR] 无效的摄像头/图像处理组合")
            return

        try:
            # 发送模式切换命令（2字节大端序）
            mode_data = struct.pack('!H', mode_cmd)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sent = sock.sendto(mode_data, (self.fpga_ip, self.control_port))
            sock.close()

            # 解析模式名称
            camera_name = "左目" if camera == 'left' else "右目"
            proc_names = {0x00: "原始图像", 0x01: "二值化", 0x02: "腐蚀", 0x03: "开运算", 0x04: "浮雕效果", 0x05: "拉普拉斯锐化", 0x06: "Sobel边缘检测", 0x07: "帧差法"}
            proc_name = proc_names.get(proc_mode, "未知")

            self.video_log_append(f"[INFO] 切换到: {camera_name} + {proc_name} (0x{mode_cmd:04X}), 已发送 {sent} 字节")
            self.update_status(f"{camera_name} + {proc_name}", STATUS_SUCCESS)

        except Exception as e:
            self.video_log_append(f"[ERROR] 模式切换失败: {e}")
            self.update_status(f"模式切换失败: {e}", STATUS_ERROR)

    def stop_video_receiving(self):
        """停止视频接收"""
        if not self.is_receiving:
            return

        self.is_receiving = False
        self.process_timer.stop()

        if self.receiver:
            self.receiver.stop()
            self.receiver = None

        try:
            cv2.destroyWindow(self.cv2_window_name)
        except:
            pass

        self.start_recv_btn.setEnabled(True)
        self.stop_recv_btn.setEnabled(False)
        self.video_log_append("[INFO] 视频接收已停止")

    def process_video_packets(self):
        """处理视频数据包（由QTimer调用）"""
        if not self.is_receiving:
            return

        try:
            # 处理队列中的数据包
            while not self.packet_queue.empty():
                packet = self.packet_queue.get_nowait()
                self.handle_video_packet(packet)
        except queue.Empty:
            pass

        # 检查超时
        if self.current_frame_seq != -1 and time.time() - self.last_packet_time > FRAME_TIMEOUT:
            self.video_log_append(f"[WARN] 帧 {self.current_frame_seq} 超时，已丢弃")
            self.frame_packets.clear()
            self.current_frame_seq = -1

        # 保持OpenCV窗口响应
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            self.stop_video_receiving()

    def handle_video_packet(self, packet):
        """处理单个视频数据包"""
        try:
            # 统计接收字节数（用于带宽计算）
            self.udp_bytes_received += len(packet)

            if len(packet) < PACKET_HEADER_SIZE:
                self.video_log_append(f"[WARN] 包太小: {len(packet)} < {PACKET_HEADER_SIZE}")
                return

            header = struct.unpack("<8I", packet[:PACKET_HEADER_SIZE])

            # 🔍 调试：打印包头信息
            if header[0] != 0xAA0055FF:
                self.video_log_append(f"[WARN] 无效魔数: 0x{header[0]:08X}，包长度: {len(packet)}")
                return

            frame_seq = header[5]
            packet_seq = header[6]

            # 解析模板匹配结果 (header[7])
            # 格式: bit 31=match_result, bits 9:0=hamming_distance
            match_result_header = header[7]
            match_result_bit = (match_result_header >> 31) & 0x1
            hamming_dist = match_result_header & 0x3FF

            # 更新匹配结果（每个packet都更新，保证实时性）
            self.match_result = (match_result_bit == 1)

            # 更新UI标签（仅在结果改变时更新以提高性能）
            if not hasattr(self, '_last_match_result') or self._last_match_result != self.match_result:
                self._last_match_result = self.match_result
                if self.match_result:
                    self.match_result_label.setText(f"✓ 对 (距离:{hamming_dist})")
                    self.match_result_label.setStyleSheet("""
                        QLabel {
                            font-size: 28px;
                            font-weight: bold;
                            color: white;
                            padding: 15px;
                            border: 3px solid #27ae60;
                            border-radius: 8px;
                            background-color: #27ae60;
                        }
                    """)
                else:
                    self.match_result_label.setText(f"✗ 错 (距离:{hamming_dist})")
                    self.match_result_label.setStyleSheet("""
                        QLabel {
                            font-size: 28px;
                            font-weight: bold;
                            color: white;
                            padding: 15px;
                            border: 3px solid #e74c3c;
                            border-radius: 8px;
                            background-color: #e74c3c;
                        }
                    """)

            # 🔍 调试：检查包序号范围
            if packet_seq >= TOTAL_PACKETS_PER_FRAME:
                self.video_log_append(f"[ERROR] 包序号越界: {packet_seq} >= {TOTAL_PACKETS_PER_FRAME}")
                return

            # 🔍 调试：检查数据长度
            data_len = len(packet) - PACKET_HEADER_SIZE
            if data_len != PACKET_DATA_SIZE:
                self.video_log_append(f"[WARN] 包 {packet_seq} 数据长度异常: {data_len} != {PACKET_DATA_SIZE}")

            if frame_seq != self.current_frame_seq:
                # 🔍 调试：打印帧切换信息
                if self.current_frame_seq != -1:
                    missing = TOTAL_PACKETS_PER_FRAME - len(self.frame_packets)
                    self.video_log_append(f"[WARN] 帧 {self.current_frame_seq} 不完整，丢失 {missing} 包")

                self.current_frame_seq = frame_seq
                self.frame_packets.clear()
                self.video_log_append(f"[INFO] 开始接收帧 {frame_seq}（包序号: {packet_seq}）")

            self.frame_packets[packet_seq] = packet[PACKET_HEADER_SIZE:]
            self.last_packet_time = time.time()

            # 检查是否接收完整
            if len(self.frame_packets) == TOTAL_PACKETS_PER_FRAME:
                # 🔍 调试：检查连续性
                missing_packets = [i for i in range(TOTAL_PACKETS_PER_FRAME) if i not in self.frame_packets]
                if missing_packets:
                    self.video_log_append(f"[ERROR] 帧 {frame_seq} 缺少包: {missing_packets[:10]}...")
                else:
                    self.video_log_append(f"[SUCCESS] 帧 {frame_seq} 完整接收，共 {len(self.frame_packets)} 包")

                self.assemble_and_display_frame()
                self.frame_packets.clear()
                self.current_frame_seq = -1

        except Exception as e:
            import traceback
            self.video_log_append(f"[ERROR] 处理数据包失败: {e}")
            self.video_log_append(f"[ERROR] 堆栈: {traceback.format_exc()}")

    def assemble_and_display_frame(self):
        """组装并显示完整帧"""
        # 组装数据
        full_frame_data = bytearray()
        missing_count = 0
        for i in range(TOTAL_PACKETS_PER_FRAME):
            if i in self.frame_packets:
                full_frame_data.extend(self.frame_packets[i])
            else:
                full_frame_data.extend(b'\x00' * PACKET_DATA_SIZE)
                missing_count += 1

        if missing_count > 0:
            self.video_log_append(f"[WARN] 用零填充了 {missing_count} 个丢失的包")

        expected_size = IMG_HEIGHT * IMG_WIDTH * 3
        actual_size = len(full_frame_data)

        # 🔍 调试：保存原始数据到文件（仅第一帧）
        if not hasattr(self, '_saved_raw_frame'):
            try:
                with open('debug_raw_frame.bin', 'wb') as f:
                    f.write(full_frame_data[:expected_size])
                self.video_log_append(f"[DEBUG] 已保存原始数据到 debug_raw_frame.bin ({expected_size} 字节)")
                self._saved_raw_frame = True
            except Exception as e:
                self.video_log_append(f"[WARN] 无法保存调试文件: {e}")

        if actual_size < expected_size:
            self.video_log_append(f"[ERROR] 数据不完整，期望 {expected_size} 字节，实际 {actual_size}")
            return

        final_data = full_frame_data[:expected_size]

        # 🔍 调试：检查数据统计
        data_array = np.frombuffer(final_data, dtype=np.uint8)
        zero_ratio = np.sum(data_array == 0) / len(data_array) * 100
        if zero_ratio > 50:
            self.video_log_append(f"[WARN] 数据中有 {zero_ratio:.1f}% 是零值，可能SD卡未读取完成")

        try:
            # 转换为numpy数组（默认RGB顺序）
            rgb_frame = np.frombuffer(final_data, dtype=np.uint8).reshape((IMG_HEIGHT, IMG_WIDTH, 3))

            # 🔍 调试：尝试不同的颜色通道顺序
            # 如果发现颜色错乱，取消下面注释尝试BGR顺序
            # rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

            # 🔍 调试：保存第一帧到图片文件
            if not hasattr(self, '_saved_first_frame'):
                cv2.imwrite('debug_first_frame_RGB.png', rgb_frame)
                # 也尝试BGR顺序
                bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
                cv2.imwrite('debug_first_frame_BGR.png', bgr_frame)
                self.video_log_append("[DEBUG] 已保存 debug_first_frame_RGB.png 和 BGR.png")
                self._saved_first_frame = True

            # 计算FPS
            self.fps_frame_count += 1
            elapsed = time.time() - self.last_fps_time
            if elapsed >= 1.0:
                self.fps = self.fps_frame_count / elapsed
                self.last_fps_time = time.time()
                self.fps_frame_count = 0
                self.fps_label.setText(f"FPS: {self.fps:.1f}")

            # 计算UDP带宽（Mbps）
            bandwidth_elapsed = time.time() - self.last_bandwidth_time
            if bandwidth_elapsed >= 1.0:
                self.udp_bandwidth_mbps = (self.udp_bytes_received * 8) / (bandwidth_elapsed * 1000000)
                self.last_bandwidth_time = time.time()
                self.udp_bytes_received = 0

            # 根据模式添加文本信息
            if not self.is_sd_card_mode:
                # 摄像头模式：显示FPS和带宽
                info_text = f"FPS: {self.fps:.1f} | Bandwidth: {self.udp_bandwidth_mbps:.2f} Mbps"
                cv2.putText(rgb_frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # 模板匹配结果叠加显示（左上角，FPS信息下方）
                if hasattr(self, 'match_result') and self.match_result is not None:
                    if self.match_result:
                        match_text = u"\u2713 \u5bf9"  # ✓ 对
                        match_color = (0, 255, 0)  # 绿色 (OpenCV uses BGR)
                    else:
                        match_text = u"\u2717 \u9519"  # ✗ 错
                        match_color = (0, 0, 255)  # 红色

                    cv2.putText(rgb_frame, match_text, (10, 70),
                               cv2.FONT_HERSHEY_SIMPLEX, 1.2, match_color, 3)
            # SD卡模式：不显示任何文本信息

            # 保存当前帧引用（用于保存功能）
            self.current_single_frame = rgb_frame.copy()

            # 显示图像
            cv2.imshow(self.cv2_window_name, rgb_frame)

        except Exception as e:
            import traceback
            self.video_log_append(f"[ERROR] 图像处理错误: {e}")
            self.video_log_append(f"[ERROR] 堆栈: {traceback.format_exc()}")

    def video_log_append(self, text):
        """添加日志信息"""
        timestamp = time.strftime("%H:%M:%S")
        self.video_log.append(f"[{timestamp}] {text}")

    def save_current_single_frame(self):
        """保存当前单目图片为BMP格式"""
        if self.current_single_frame is None:
            QMessageBox.warning(self, "警告", "当前没有可保存的图像，请先开始接收视频")
            self.video_log_append("[WARN] 没有可保存的图像")
            return

        try:
            # 生成文件名：FPGA_001.bmp, FPGA_002.bmp, ...
            self.saved_image_counter += 1
            filename = f"FPGA_{self.saved_image_counter:03d}.bmp"

            # 保存为BMP格式
            cv2.imwrite(filename, self.current_single_frame)

            self.video_log_append(f"[INFO] 图片已保存: {filename}")
            QMessageBox.information(self, "成功", f"图片已保存为 {filename}")

        except Exception as e:
            self.video_log_append(f"[ERROR] 保存图片失败: {e}")
            QMessageBox.critical(self, "错误", f"保存图片失败: {e}")

    # ====================================================================
    # 灰度阈值配置（单目模式）
    # ====================================================================

    def on_gray_low_threshold_changed(self, value):
        """灰度低阈值改变"""
        self.binary_threshold_low = value
        self.low_threshold_slider.blockSignals(True)
        self.low_threshold_slider.setValue(value)
        self.low_threshold_slider.blockSignals(False)
        self.low_threshold_spinbox.blockSignals(True)
        self.low_threshold_spinbox.setValue(value)
        self.low_threshold_spinbox.blockSignals(False)
        self.update_gray_threshold_status()

    def on_gray_high_threshold_changed(self, value):
        """灰度高阈值改变"""
        self.binary_threshold_high = value
        self.high_threshold_slider.blockSignals(True)
        self.high_threshold_slider.setValue(value)
        self.high_threshold_slider.blockSignals(False)
        self.high_threshold_spinbox.blockSignals(True)
        self.high_threshold_spinbox.setValue(value)
        self.high_threshold_spinbox.blockSignals(False)
        self.update_gray_threshold_status()

    def update_gray_threshold_status(self):
        """更新灰度阈值状态显示"""
        avg = (self.binary_threshold_low + self.binary_threshold_high) // 2
        status_text = f"当前: 低={self.binary_threshold_low}, 高={self.binary_threshold_high}, 平均={avg}"
        self.gray_threshold_status_label.setText(status_text)

    def send_gray_threshold_config(self):
        """发送灰度阈值配置到FPGA（4字节命令0x0008）"""
        if not self.connected:
            self.video_log_append("[WARN] 请先连接到FPGA")
            QMessageBox.warning(self, "警告", "请先连接到FPGA")
            return

        if self.binary_threshold_low >= self.binary_threshold_high:
            self.video_log_append("[ERROR] 低阈值必须小于高阈值")
            QMessageBox.critical(self, "错误", "低阈值必须小于高阈值！")
            return

        try:
            # 构造4字节命令：[0x00][0x08][low][high]
            cmd_packet = struct.pack('!HBB',
                                    0x0008,
                                    self.binary_threshold_low,
                                    self.binary_threshold_high)

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sent = sock.sendto(cmd_packet, (self.fpga_ip, self.control_port))
            sock.close()

            self.video_log_append(
                f"[INFO] 灰度阈值已发送: 低={self.binary_threshold_low}, "
                f"高={self.binary_threshold_high}, 已发送 {sent} 字节"
            )
            self.update_status("灰度阈值已更新", STATUS_SUCCESS)

        except Exception as e:
            self.video_log_append(f"[ERROR] 发送灰度阈值失败: {e}")
            QMessageBox.critical(self, "错误", f"发送灰度阈值失败: {e}")

    # ====================================================================
    # RGB颜色阈值配置（双目测距模式）
    # ====================================================================

    def sync_rgb_widgets(self, r_low, r_high, g_low, g_high, b_low, b_high):
        """同步双目UI的RGB控件"""
        # 更新R通道
        self.dual_r_low_slider.blockSignals(True)
        self.dual_r_low_slider.setValue(r_low)
        self.dual_r_low_slider.blockSignals(False)
        self.dual_r_low_spinbox.blockSignals(True)
        self.dual_r_low_spinbox.setValue(r_low)
        self.dual_r_low_spinbox.blockSignals(False)

        self.dual_r_high_slider.blockSignals(True)
        self.dual_r_high_slider.setValue(r_high)
        self.dual_r_high_slider.blockSignals(False)
        self.dual_r_high_spinbox.blockSignals(True)
        self.dual_r_high_spinbox.setValue(r_high)
        self.dual_r_high_spinbox.blockSignals(False)

        # 更新G通道
        self.dual_g_low_slider.blockSignals(True)
        self.dual_g_low_slider.setValue(g_low)
        self.dual_g_low_slider.blockSignals(False)
        self.dual_g_low_spinbox.blockSignals(True)
        self.dual_g_low_spinbox.setValue(g_low)
        self.dual_g_low_spinbox.blockSignals(False)

        self.dual_g_high_slider.blockSignals(True)
        self.dual_g_high_slider.setValue(g_high)
        self.dual_g_high_slider.blockSignals(False)
        self.dual_g_high_spinbox.blockSignals(True)
        self.dual_g_high_spinbox.setValue(g_high)
        self.dual_g_high_spinbox.blockSignals(False)

        # 更新B通道
        self.dual_b_low_slider.blockSignals(True)
        self.dual_b_low_slider.setValue(b_low)
        self.dual_b_low_slider.blockSignals(False)
        self.dual_b_low_spinbox.blockSignals(True)
        self.dual_b_low_spinbox.setValue(b_low)
        self.dual_b_low_spinbox.blockSignals(False)

        self.dual_b_high_slider.blockSignals(True)
        self.dual_b_high_slider.setValue(b_high)
        self.dual_b_high_slider.blockSignals(False)
        self.dual_b_high_spinbox.blockSignals(True)
        self.dual_b_high_spinbox.setValue(b_high)
        self.dual_b_high_spinbox.blockSignals(False)

    def apply_color_preset(self, color):
        """应用颜色预设"""
        presets = {
            'red':    (200, 255, 0, 80, 0, 80),      # 纯红色：高R，低G，低B
            'green':  (0, 80, 180, 255, 0, 80),      # 纯绿色：低R，高G，低B
            'blue':   (0, 80, 0, 80, 180, 255),      # 纯蓝色：低R，低G，高B
            'yellow': (180, 255, 180, 255, 0, 80),   # 黄色：高R，高G，低B
            'white':  (200, 255, 200, 255, 200, 255) # 白色：全高
        }

        if color in presets:
            r_low, r_high, g_low, g_high, b_low, b_high = presets[color]
            self.binary_threshold_r_low = r_low
            self.binary_threshold_r_high = r_high
            self.binary_threshold_g_low = g_low
            self.binary_threshold_g_high = g_high
            self.binary_threshold_b_low = b_low
            self.binary_threshold_b_high = b_high

            self.sync_rgb_widgets(r_low, r_high, g_low, g_high, b_low, b_high)
            self.update_rgb_threshold_status()
            self.dual_cam_log_append(f"[INFO] 已应用{color}颜色预设")

    def on_r_low_changed(self, value):
        """R通道低阈值改变"""
        self.binary_threshold_r_low = value
        self.update_rgb_threshold_status()

    def on_r_high_changed(self, value):
        """R通道高阈值改变"""
        self.binary_threshold_r_high = value
        self.update_rgb_threshold_status()

    def on_g_low_changed(self, value):
        """G通道低阈值改变"""
        self.binary_threshold_g_low = value
        self.update_rgb_threshold_status()

    def on_g_high_changed(self, value):
        """G通道高阈值改变"""
        self.binary_threshold_g_high = value
        self.update_rgb_threshold_status()

    def on_b_low_changed(self, value):
        """B通道低阈值改变"""
        self.binary_threshold_b_low = value
        self.update_rgb_threshold_status()

    def on_b_high_changed(self, value):
        """B通道高阈值改变"""
        self.binary_threshold_b_high = value
        self.update_rgb_threshold_status()

    def update_rgb_threshold_status(self):
        """更新RGB阈值状态显示"""
        status_text = (f"R:[{self.binary_threshold_r_low}-{self.binary_threshold_r_high}] "
                      f"G:[{self.binary_threshold_g_low}-{self.binary_threshold_g_high}] "
                      f"B:[{self.binary_threshold_b_low}-{self.binary_threshold_b_high}]")
        self.dual_threshold_status_label.setText(status_text)

    def send_rgb_threshold_config(self):
        """发送RGB颜色阈值配置到FPGA（8字节命令0x0009）"""
        if not self.connected:
            self.dual_cam_log_append("[WARN] 请先连接到FPGA")
            QMessageBox.warning(self, "警告", "请先连接到FPGA")
            return

        # 验证阈值有效性
        if (self.binary_threshold_r_low >= self.binary_threshold_r_high or
            self.binary_threshold_g_low >= self.binary_threshold_g_high or
            self.binary_threshold_b_low >= self.binary_threshold_b_high):
            self.dual_cam_log_append("[ERROR] 各通道低阈值必须小于高阈值")
            QMessageBox.critical(self, "错误", "各通道低阈值必须小于高阈值！")
            return

        try:
            # 构造8字节命令：[0x00][0x09][R_low][R_high][G_low][G_high][B_low][B_high]
            cmd_packet = struct.pack('!HBBBBBB',
                                    0x0009,                           # 命令码
                                    self.binary_threshold_r_low,      # R低阈值
                                    self.binary_threshold_r_high,     # R高阈值
                                    self.binary_threshold_g_low,      # G低阈值
                                    self.binary_threshold_g_high,     # G高阈值
                                    self.binary_threshold_b_low,      # B低阈值
                                    self.binary_threshold_b_high)     # B高阈值

            # 发送UDP命令
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sent = sock.sendto(cmd_packet, (self.fpga_ip, self.control_port))
            sock.close()

            self.dual_cam_log_append(
                f"[INFO] RGB颜色阈值已发送: "
                f"R:[{self.binary_threshold_r_low}-{self.binary_threshold_r_high}] "
                f"G:[{self.binary_threshold_g_low}-{self.binary_threshold_g_high}] "
                f"B:[{self.binary_threshold_b_low}-{self.binary_threshold_b_high}], "
                f"已发送 {sent} 字节"
            )
            self.update_status(
                f"RGB颜色阈值已更新",
                STATUS_SUCCESS
            )

        except Exception as e:
            self.dual_cam_log_append(f"[ERROR] 发送阈值配置失败: {e}")
            QMessageBox.critical(self, "错误", f"发送阈值配置失败: {e}")

    def toggle_border_detection(self):
        """切换边框检测使能（显示二值化图像+边框）"""
        if not self.connected:
            self.video_log_append("[WARN] 请先连接到FPGA")
            QMessageBox.warning(self, "警告", "请先连接到FPGA")
            return

        try:
            # 发送2字节命令切换边框使能
            cmd_packet = struct.pack('!H', 0x0009)  # 命令码 0x0009
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sent = sock.sendto(cmd_packet, (self.fpga_ip, self.control_port))
            sock.close()

            # 切换本地状态
            self.border_enable = not self.border_enable

            # 更新按钮样式
            if self.border_enable:
                self.border_toggle_btn.setText("边框检测：开启")
                self.border_toggle_btn.setStyleSheet(
                    "background-color: #27ae60; color: white; padding: 10px; min-width: 150px; font-weight: bold;"
                )
                status_text = "边框检测已开启（显示：二值化图像+绿色边框）"
                log_text = "[INFO] 边框检测已开启 - 将显示二值化图像及白色区域的绿色边框"
            else:
                self.border_toggle_btn.setText("边框检测：关闭")
                self.border_toggle_btn.setStyleSheet(
                    "background-color: #95a5a6; color: white; padding: 10px; min-width: 150px; font-weight: bold;"
                )
                status_text = "边框检测已关闭（按图像处理模式显示）"
                log_text = "[INFO] 边框检测已关闭 - 恢复当前图像处理模式"

            self.video_log_append(f"{log_text}，已发送 {sent} 字节")
            self.update_status(status_text, STATUS_SUCCESS)

        except Exception as e:
            self.video_log_append(f"[ERROR] 切换边框检测失败: {e}")
            QMessageBox.critical(self, "错误", f"切换边框检测失败: {e}")

    def upload_template_image(self):
        """上传模板图像（32×32二值化）"""
        if not self.connected:
            QMessageBox.warning(self, "警告", "请先连接到FPGA")
            return

        # 选择图像文件
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择模板图像", "", "图像文件 (*.png *.jpg *.bmp);;所有文件 (*)"
        )
        if not file_path:
            return

        try:
            # 加载图像并处理
            img = Image.open(file_path).convert('L')  # 转灰度
            img = img.resize((32, 32), Image.LANCZOS)  # 缩放到32×32
            img_array = np.array(img)
            binary = (img_array > 128).astype(np.uint8)  # 二值化

            # 保存到内存
            self.template_image = binary

            # 打包为128字节
            data = bytearray()
            for row in binary:
                for col_start in range(0, 32, 8):
                    byte_val = 0
                    for bit_idx in range(8):
                        pixel = row[col_start + bit_idx]
                        byte_val |= (pixel << (7 - bit_idx))
                    data.append(byte_val)

            # 发送UDP命令 0x00F0 + 128字节数据
            cmd_packet = struct.pack('!H', 0x00F0) + bytes(data)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sent = sock.sendto(cmd_packet, (self.fpga_ip, self.control_port))
            sock.close()

            self.video_log_append(f"[INFO] 模板图像已上传: {file_path} ({sent} 字节)")
            self.update_status("模板图像上传成功", STATUS_SUCCESS)
            self.upload_template_btn.setStyleSheet("background-color: #27ae60; color: white; padding: 8px; min-width: 150px; font-weight: bold;")
            self.match_result_label.setText("模板已上传，请开启视频流")
            self.video_log_append("[INFO] 实时模板匹配已启动，请开启视频流以查看匹配结果")

        except Exception as e:
            self.video_log_append(f"[ERROR] 上传模板图像失败: {e}")
            QMessageBox.critical(self, "错误", f"上传模板图像失败: {e}")

    # ====================================================================
    # 状态信息栏
    # ====================================================================

    def create_status_group(self):
        """创建状态显示组"""
        group = QGroupBox("状态信息")
        layout = QHBoxLayout()
        self.status_label = QLabel("就绪")
        self.status_label.setStyleSheet("font-size: 14px; padding: 5px;")
        layout.addWidget(self.status_label)
        layout.addStretch()
        group.setLayout(layout)
        return group

    def update_status(self, message, color=None):
        """更新状态信息"""
        self.status_label.setText(message)
        if color:
            self.status_label.setStyleSheet(f"font-size: 14px; padding: 5px; color: {color};")

    # ====================================================================
    # 页面5: 双目摄像头接收（新增）
    # ====================================================================

    def create_dual_cam_page(self):
        """创建双目摄像头接收页面"""
        page = QWidget()
        layout = QVBoxLayout(page)

        # 标题
        title = QLabel("📍 双目摄像头接收：同时显示左右目图像")
        title.setStyleSheet("font-size: 14px; font-weight: bold; color: #8e44ad; padding: 10px;")
        layout.addWidget(title)

        # 提示信息
        hint = QLabel("提示：双目模式发送4字节/像素（左RGB565+右RGB565），使用菜单或快捷键Ctrl+6切换到双目模式")
        hint.setStyleSheet("color: #7f8c8d; padding: 5px;")
        layout.addWidget(hint)

        # 字节偏移配置（新增）
        shift_group = QGroupBox("字节偏移配置（修正颜色显示）")
        shift_layout = QHBoxLayout()
        QLabel("全帧字节偏移:", parent=shift_group).setParent(None)
        shift_layout.addWidget(QLabel("全帧字节偏移:"))
        self.dual_shift_spinbox = QSpinBox()
        self.dual_shift_spinbox.setRange(0, 3)
        self.dual_shift_spinbox.setValue(1)
        self.dual_shift_spinbox.valueChanged.connect(self.on_dual_shift_changed)
        shift_layout.addWidget(self.dual_shift_spinbox)
        shift_hint = QLabel("（0~3，如果颜色不对请调整此值）")
        shift_hint.setStyleSheet("color: #7f8c8d;")
        shift_layout.addWidget(shift_hint)
        shift_layout.addStretch()
        shift_group.setLayout(shift_layout)
        layout.addWidget(shift_group)

        # RGB颜色阈值配置区域（双目模式，与单目共享）
        dual_threshold_group = QGroupBox("RGB颜色阈值配置（测距模式有效，与单目共享）")
        dual_threshold_layout = QVBoxLayout()

        # 创建RGB三通道滑块的辅助函数
        def create_dual_threshold_row(label_text, low_value, high_value, low_callback, high_callback, color_style):
            row_layout = QHBoxLayout()

            # 通道标签
            channel_label = QLabel(label_text)
            channel_label.setStyleSheet(f"font-weight: bold; color: {color_style}; min-width: 80px;")
            row_layout.addWidget(channel_label)

            # 低阈值
            row_layout.addWidget(QLabel("低:"))
            low_slider = QSlider(Qt.Horizontal)
            low_slider.setMinimum(0)
            low_slider.setMaximum(255)
            low_slider.setValue(low_value)
            low_slider.setTickPosition(QSlider.TicksBelow)
            low_slider.setTickInterval(50)
            low_slider.valueChanged.connect(low_callback)
            row_layout.addWidget(low_slider)

            low_spinbox = QSpinBox()
            low_spinbox.setMinimum(0)
            low_spinbox.setMaximum(255)
            low_spinbox.setValue(low_value)
            low_spinbox.setFixedWidth(60)
            low_spinbox.valueChanged.connect(low_callback)
            row_layout.addWidget(low_spinbox)

            # 高阈值
            row_layout.addWidget(QLabel("高:"))
            high_slider = QSlider(Qt.Horizontal)
            high_slider.setMinimum(0)
            high_slider.setMaximum(255)
            high_slider.setValue(high_value)
            high_slider.setTickPosition(QSlider.TicksBelow)
            high_slider.setTickInterval(50)
            high_slider.valueChanged.connect(high_callback)
            row_layout.addWidget(high_slider)

            high_spinbox = QSpinBox()
            high_spinbox.setMinimum(0)
            high_spinbox.setMaximum(255)
            high_spinbox.setValue(high_value)
            high_spinbox.setFixedWidth(60)
            high_spinbox.valueChanged.connect(high_callback)
            row_layout.addWidget(high_spinbox)

            return row_layout, low_slider, low_spinbox, high_slider, high_spinbox

        # R通道
        dual_r_layout, self.dual_r_low_slider, self.dual_r_low_spinbox, self.dual_r_high_slider, self.dual_r_high_spinbox = \
            create_dual_threshold_row("R通道:", self.binary_threshold_r_low, self.binary_threshold_r_high,
                                     self.on_r_low_changed, self.on_r_high_changed, "#e74c3c")
        dual_threshold_layout.addLayout(dual_r_layout)

        # G通道
        dual_g_layout, self.dual_g_low_slider, self.dual_g_low_spinbox, self.dual_g_high_slider, self.dual_g_high_spinbox = \
            create_dual_threshold_row("G通道:", self.binary_threshold_g_low, self.binary_threshold_g_high,
                                     self.on_g_low_changed, self.on_g_high_changed, "#27ae60")
        dual_threshold_layout.addLayout(dual_g_layout)

        # B通道
        dual_b_layout, self.dual_b_low_slider, self.dual_b_low_spinbox, self.dual_b_high_slider, self.dual_b_high_spinbox = \
            create_dual_threshold_row("B通道:", self.binary_threshold_b_low, self.binary_threshold_b_high,
                                     self.on_b_low_changed, self.on_b_high_changed, "#3498db")
        dual_threshold_layout.addLayout(dual_b_layout)

        # 颜色预设按钮
        preset_layout = QHBoxLayout()
        preset_layout.addWidget(QLabel("颜色预设:"))

        preset_red_btn = QPushButton("红色")
        preset_red_btn.setStyleSheet("background-color: #e74c3c; color: white; padding: 5px; min-width: 60px;")
        preset_red_btn.clicked.connect(lambda: self.apply_color_preset('red'))
        preset_layout.addWidget(preset_red_btn)

        preset_green_btn = QPushButton("绿色")
        preset_green_btn.setStyleSheet("background-color: #27ae60; color: white; padding: 5px; min-width: 60px;")
        preset_green_btn.clicked.connect(lambda: self.apply_color_preset('green'))
        preset_layout.addWidget(preset_green_btn)

        preset_blue_btn = QPushButton("蓝色")
        preset_blue_btn.setStyleSheet("background-color: #3498db; color: white; padding: 5px; min-width: 60px;")
        preset_blue_btn.clicked.connect(lambda: self.apply_color_preset('blue'))
        preset_layout.addWidget(preset_blue_btn)

        preset_yellow_btn = QPushButton("黄色")
        preset_yellow_btn.setStyleSheet("background-color: #f1c40f; color: black; padding: 5px; min-width: 60px;")
        preset_yellow_btn.clicked.connect(lambda: self.apply_color_preset('yellow'))
        preset_layout.addWidget(preset_yellow_btn)

        preset_white_btn = QPushButton("白色")
        preset_white_btn.setStyleSheet("background-color: #ecf0f1; color: black; padding: 5px; min-width: 60px;")
        preset_white_btn.clicked.connect(lambda: self.apply_color_preset('white'))
        preset_layout.addWidget(preset_white_btn)

        preset_layout.addStretch()
        dual_threshold_layout.addLayout(preset_layout)

        # 阈值应用按钮
        dual_apply_threshold_layout = QHBoxLayout()
        self.dual_apply_threshold_btn = QPushButton("应用颜色阈值")
        self.dual_apply_threshold_btn.setStyleSheet("background-color: #f39c12; color: white; padding: 8px; min-width: 150px; font-weight: bold;")
        self.dual_apply_threshold_btn.clicked.connect(self.send_rgb_threshold_config)
        dual_apply_threshold_layout.addWidget(self.dual_apply_threshold_btn)

        self.dual_threshold_status_label = QLabel(f"R:[{self.binary_threshold_r_low}-{self.binary_threshold_r_high}] G:[{self.binary_threshold_g_low}-{self.binary_threshold_g_high}] B:[{self.binary_threshold_b_low}-{self.binary_threshold_b_high}]")
        self.dual_threshold_status_label.setStyleSheet("padding: 5px; color: #7f8c8d;")
        dual_apply_threshold_layout.addWidget(self.dual_threshold_status_label)
        dual_apply_threshold_layout.addStretch()

        dual_threshold_layout.addLayout(dual_apply_threshold_layout)

        dual_threshold_group.setLayout(dual_threshold_layout)
        layout.addWidget(dual_threshold_group)

        # 控制按钮
        btn_layout = QHBoxLayout()
        self.dual_start_recv_btn = QPushButton("开始接收双目")
        self.dual_start_recv_btn.setStyleSheet("background-color: #27ae60; color: white; padding: 8px;")
        self.dual_start_recv_btn.clicked.connect(self.start_dual_cam_receiving)
        btn_layout.addWidget(self.dual_start_recv_btn)

        self.dual_stop_recv_btn = QPushButton("停止接收")
        self.dual_stop_recv_btn.setStyleSheet("background-color: #e74c3c; color: white; padding: 8px;")
        self.dual_stop_recv_btn.setEnabled(False)
        self.dual_stop_recv_btn.clicked.connect(self.stop_dual_cam_receiving)
        btn_layout.addWidget(self.dual_stop_recv_btn)

        # 双目测距开关按钮（新增）
        self.stereo_ranging_btn = QPushButton("开启双目测距")
        self.stereo_ranging_btn.setStyleSheet("background-color: #3498db; color: white; padding: 8px; font-weight: bold;")
        self.stereo_ranging_btn.clicked.connect(self.toggle_stereo_ranging)
        btn_layout.addWidget(self.stereo_ranging_btn)

        self.dual_save_frame_btn = QPushButton("保存当前图片")
        self.dual_save_frame_btn.setStyleSheet("background-color: #f39c12; color: white; padding: 8px; font-weight: bold;")
        self.dual_save_frame_btn.clicked.connect(self.save_current_dual_frames)
        btn_layout.addWidget(self.dual_save_frame_btn)

        self.dual_fps_label = QLabel("FPS: --")
        self.dual_fps_label.setStyleSheet("font-weight: bold; padding: 8px;")
        btn_layout.addWidget(self.dual_fps_label)

        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        # 日志显示
        log_group = QGroupBox("双目接收日志")
        log_layout = QVBoxLayout()
        self.dual_cam_log = QTextEdit()
        self.dual_cam_log.setReadOnly(True)
        self.dual_cam_log.setFont(QFont("Courier New", 9))
        self.dual_cam_log.setMaximumHeight(250)
        log_layout.addWidget(self.dual_cam_log)
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)

        layout.addStretch()
        return page

    def start_dual_cam_receiving(self):
        """开始双目摄像头接收"""
        if not self.connected:
            QMessageBox.warning(self, "警告", "请先连接到FPGA")
            return

        if self.is_dual_cam_receiving:
            return

        # 清空队列和缓存
        while not self.dual_cam_packet_queue.empty():
            self.dual_cam_packet_queue.get()
        self.dual_cam_frame_packets.clear()
        self.dual_cam_current_frame_seq = -1
        self.dual_cam_last_packet_time = time.time()

        # 启动UDP接收器
        self.dual_cam_receiver = UDPCameraReceiver(self.local_ip, self.video_port, self.dual_cam_packet_queue)
        if not self.dual_cam_receiver.start():
            self.dual_cam_log_append("[ERROR] 无法启动UDP接收器")
            return

        self.is_dual_cam_receiving = True
        self.dual_start_recv_btn.setEnabled(False)
        self.dual_stop_recv_btn.setEnabled(True)
        self.dual_cam_log_append(f"[INFO] 开始接收双目视频，监听 {self.local_ip}:{self.video_port}")

        # 创建OpenCV窗口
        cv2.namedWindow(self.dual_cam_left_window_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.dual_cam_right_window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.dual_cam_left_window_name, IMG_WIDTH, IMG_HEIGHT)
        cv2.resizeWindow(self.dual_cam_right_window_name, IMG_WIDTH, IMG_HEIGHT)

        # 启动处理定时器
        self.dual_cam_process_timer.start(20)

    def stop_dual_cam_receiving(self):
        """停止双目摄像头接收"""
        if not self.is_dual_cam_receiving:
            return

        self.is_dual_cam_receiving = False
        self.dual_cam_process_timer.stop()

        if self.dual_cam_receiver:
            self.dual_cam_receiver.stop()
            self.dual_cam_receiver = None

        try:
            cv2.destroyWindow(self.dual_cam_left_window_name)
            cv2.destroyWindow(self.dual_cam_right_window_name)
        except:
            pass

        self.dual_start_recv_btn.setEnabled(True)
        self.dual_stop_recv_btn.setEnabled(False)
        self.dual_cam_log_append("[INFO] 双目视频接收已停止")

    def process_dual_cam_packets(self):
        """处理双目视频数据包（由QTimer调用）"""
        if not self.is_dual_cam_receiving:
            return

        try:
            # 处理队列中的数据包
            while not self.dual_cam_packet_queue.empty():
                packet = self.dual_cam_packet_queue.get_nowait()
                self.handle_dual_cam_packet(packet)
        except queue.Empty:
            pass

        # 检查超时
        if self.dual_cam_current_frame_seq != -1 and time.time() - self.dual_cam_last_packet_time > FRAME_TIMEOUT:
            self.dual_cam_log_append(f"[WARN] 帧 {self.dual_cam_current_frame_seq} 超时，已丢弃")
            self.dual_cam_frame_packets.clear()
            self.dual_cam_current_frame_seq = -1

        # 保持OpenCV窗口响应
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            self.stop_dual_cam_receiving()

    def handle_dual_cam_packet(self, packet):
        """处理单个双目视频数据包"""
        try:
            # 统计接收字节数（用于带宽计算）
            self.dual_cam_bytes_received += len(packet)

            if len(packet) < PACKET_HEADER_SIZE:
                return

            header = struct.unpack("<8I", packet[:PACKET_HEADER_SIZE])

            if header[0] != 0xAA0055FF:
                return

            frame_seq = header[5]
            packet_seq = header[6]

            # 提取双目测距距离 (header[4][31:24])
            offset_with_distance = header[4]
            stereo_distance = (offset_with_distance >> 24) & 0xFF
            img_offset = offset_with_distance & 0xFFFFFF

            # 调试：每100帧打印一次信息
            if packet_seq == 0 and (frame_seq % 100 == 0):
                packet_data_len = len(packet) - PACKET_HEADER_SIZE
                self.dual_cam_log_append(f"[DEBUG] 帧{frame_seq}: 包数据大小={packet_data_len}字节, distance={stereo_distance}cm, mode={'测距' if self.stereo_ranging_mode else 'RGB'}")

            # 只在距离有效时更新（255表示无效测量）
            if stereo_distance < 255:
                self.stereo_distance_cm = stereo_distance
            else:
                # 如果距离无效，也记录一下
                if self.stereo_ranging_mode and packet_seq == 0:
                    self.dual_cam_log_append(f"[WARN] 距离无效(255)，可能未检测到目标或视差过小")

            if packet_seq >= TOTAL_PACKETS_PER_FRAME:
                self.dual_cam_log_append(f"[ERROR] 包序号越界: {packet_seq} >= {TOTAL_PACKETS_PER_FRAME}")
                return

            if frame_seq != self.dual_cam_current_frame_seq:
                if self.dual_cam_current_frame_seq != -1:
                    missing = TOTAL_PACKETS_PER_FRAME - len(self.dual_cam_frame_packets)
                    if missing > 0:
                        self.dual_cam_log_append(f"[WARN] 帧 {self.dual_cam_current_frame_seq} 不完整，丢失 {missing} 包")

                self.dual_cam_current_frame_seq = frame_seq
                self.dual_cam_frame_packets.clear()

            self.dual_cam_frame_packets[packet_seq] = packet[PACKET_HEADER_SIZE:]
            self.dual_cam_last_packet_time = time.time()

            # 检查是否接收完整
            if len(self.dual_cam_frame_packets) == TOTAL_PACKETS_PER_FRAME:
                self.assemble_and_display_dual_frame()
                self.dual_cam_frame_packets.clear()
                self.dual_cam_current_frame_seq = -1

        except Exception as e:
            self.dual_cam_log_append(f"[ERROR] 处理数据包失败: {e}")

    def assemble_and_display_dual_frame(self):
        """组装并显示完整双目帧"""
        # 根据模式确定每包数据大小
        if self.stereo_ranging_mode:
            packet_data_size = 424  # 测距模式：424字节/包
            expected_size = IMG_HEIGHT * IMG_WIDTH * 2  # 总共614,400字节
        else:
            packet_data_size = 848  # RGB模式：848字节/包
            expected_size = IMG_HEIGHT * IMG_WIDTH * 4  # 总共1,228,800字节

        # 组装数据
        full_frame_data = bytearray()
        for i in range(TOTAL_PACKETS_PER_FRAME):
            if i in self.dual_cam_frame_packets:
                full_frame_data.extend(self.dual_cam_frame_packets[i])
            else:
                full_frame_data.extend(b'\x00' * packet_data_size)

        if len(full_frame_data) < expected_size:
            self.dual_cam_log_append(f"[ERROR] 数据不完整，期望 {expected_size} 字节，实际 {len(full_frame_data)}")
            return

        final_data = full_frame_data[:expected_size]

        # ===== 关键修正：全帧字节偏移（用于修正FPGA数据流对齐问题）=====
        if self.dual_cam_shift_bytes:
            if self.stereo_ranging_mode:
                shift = self.dual_cam_shift_bytes % 2
            else:
                shift = self.dual_cam_shift_bytes % 4
            final_data = final_data[shift:] + final_data[:shift]

        try:
            frame_bytes = np.frombuffer(final_data, dtype=np.uint8)

            # 根据模式选择解码方式
            if self.stereo_ranging_mode:
                left_frame, right_frame = self._decode_dual_frame_binary(frame_bytes)
            else:
                left_frame, right_frame = self._decode_dual_frame(frame_bytes)

            # 计算FPS
            self.dual_cam_fps_frame_count += 1
            elapsed = time.time() - self.dual_cam_last_fps_time
            if elapsed >= 1.0:
                self.dual_cam_fps = self.dual_cam_fps_frame_count / elapsed
                self.dual_cam_last_fps_time = time.time()
                self.dual_cam_fps_frame_count = 0
                self.dual_fps_label.setText(f"FPS: {self.dual_cam_fps:.1f}")

            # 计算UDP带宽（Mbps）
            bandwidth_elapsed = time.time() - self.dual_cam_last_bandwidth_time
            if bandwidth_elapsed >= 1.0:
                self.dual_cam_bandwidth_mbps = (self.dual_cam_bytes_received * 8) / (bandwidth_elapsed * 1000000)
                self.dual_cam_last_bandwidth_time = time.time()
                self.dual_cam_bytes_received = 0

            # 添加FPS和带宽信息
            info_text = f"FPS: {self.dual_cam_fps:.1f} | Bandwidth: {self.dual_cam_bandwidth_mbps:.2f} Mbps"
            cv2.putText(left_frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(right_frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 添加左右目标签
            cv2.putText(left_frame, "Left Camera", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(right_frame, "Right Camera", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            # 显示双目测距结果（如果启用）
            if self.stereo_ranging_mode and self.stereo_distance_cm < 255:
                distance_text = f"Distance: {self.stereo_distance_cm} cm"
                cv2.putText(left_frame, distance_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(right_frame, distance_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            # 保存当前帧引用（用于保存功能）
            self.current_left_frame = left_frame.copy()
            self.current_right_frame = right_frame.copy()

            # 显示图像
            cv2.imshow(self.dual_cam_left_window_name, left_frame)
            cv2.imshow(self.dual_cam_right_window_name, right_frame)

        except Exception as e:
            self.dual_cam_log_append(f"[ERROR] 图像处理错误: {e}")

    def _decode_dual_frame(self, frame_bytes):
        """
        解码双目帧数据：4字节/像素（左RGB565 16bit + 右RGB565 16bit）
        格式：每像素 L_hi, L_lo, R_hi, R_lo（大端序 BE）
        """
        try:
            # 重塑为 (height*width, 4) 的数组
            pixel_pairs = frame_bytes.reshape((-1, 4))
        except ValueError:
            raise ValueError("帧数据长度无法被4整除")

        # 提取左右目RGB565数据（大端序：hi << 8 | lo）
        b0 = pixel_pairs[:, 0].astype(np.uint16)  # 左目高字节
        b1 = pixel_pairs[:, 1].astype(np.uint16)  # 左目低字节
        b2 = pixel_pairs[:, 2].astype(np.uint16)  # 右目高字节
        b3 = pixel_pairs[:, 3].astype(np.uint16)  # 右目低字节

        left_rgb565 = (b0 << 8) | b1
        right_rgb565 = (b2 << 8) | b3

        # 转换为BGR888
        left_bgr = self._rgb565_to_bgr(left_rgb565).reshape((IMG_HEIGHT, IMG_WIDTH, 3))
        right_bgr = self._rgb565_to_bgr(right_rgb565).reshape((IMG_HEIGHT, IMG_WIDTH, 3))

        return left_bgr, right_bgr

    @staticmethod
    def _rgb565_to_bgr(rgb565_values):
        """
        将RGB565转换为BGR888（高位扩展）
        RGB565 格式：RRRRR GGGGGG BBBBB（5-6-5位）
        """
        rgb565_values = rgb565_values.astype(np.uint16)

        # 提取RGB分量（5-6-5位）
        r5 = (rgb565_values >> 11) & 0x1F  # 5位红色
        g6 = (rgb565_values >> 5) & 0x3F   # 6位绿色
        b5 = rgb565_values & 0x1F          # 5位蓝色

        # 扩展到8位（高位扩展：复制高位到低位）
        r8 = ((r5 << 3) | (r5 >> 2)).astype(np.uint8)
        g8 = ((g6 << 2) | (g6 >> 4)).astype(np.uint8)
        b8 = ((b5 << 3) | (b5 >> 2)).astype(np.uint8)

        # 返回BGR顺序（OpenCV使用BGR）
        return np.stack((b8, g8, r8), axis=-1)

    def _decode_dual_frame_binary(self, frame_bytes):
        """
        解码双目二值化数据：2字节/像素（左二值化 + 右二值化）
        格式：每像素 left_gray, right_gray（各1字节，0x00或0xFF）
        """
        try:
            # 重塑为 (height*width, 2) 的数组
            pixel_pairs = frame_bytes.reshape((-1, 2))
        except ValueError:
            raise ValueError("帧数据长度无法被2整除")

        # 提取左右目灰度数据
        left_gray = pixel_pairs[:, 0]   # 左目灰度（0x00或0xFF）
        right_gray = pixel_pairs[:, 1]  # 右目灰度（0x00或0xFF）

        # 转换为BGR格式（灰度图，三通道相同）
        left_gray_2d = left_gray.reshape((IMG_HEIGHT, IMG_WIDTH))
        right_gray_2d = right_gray.reshape((IMG_HEIGHT, IMG_WIDTH))

        # 转换为3通道BGR（保持灰度）
        left_bgr = cv2.cvtColor(left_gray_2d, cv2.COLOR_GRAY2BGR)
        right_bgr = cv2.cvtColor(right_gray_2d, cv2.COLOR_GRAY2BGR)

        return left_bgr, right_bgr

    def dual_cam_log_append(self, text):
        """添加双目日志信息"""
        timestamp = time.strftime("%H:%M:%S")
        self.dual_cam_log.append(f"[{timestamp}] {text}")

    def save_current_dual_frames(self):
        """保存当前双目图片为BMP格式"""
        if self.current_left_frame is None or self.current_right_frame is None:
            QMessageBox.warning(self, "警告", "当前没有可保存的双目图像，请先开始接收视频")
            self.dual_cam_log_append("[WARN] 没有可保存的双目图像")
            return

        try:
            # 生成文件名：FPGA_001.bmp, FPGA_002.bmp, ...
            self.saved_image_counter += 1
            left_filename = f"FPGA_{self.saved_image_counter:03d}_left.bmp"
            right_filename = f"FPGA_{self.saved_image_counter:03d}_right.bmp"

            # 保存为BMP格式
            cv2.imwrite(left_filename, self.current_left_frame)
            cv2.imwrite(right_filename, self.current_right_frame)

            self.dual_cam_log_append(f"[INFO] 双目图片已保存: {left_filename}, {right_filename}")
            QMessageBox.information(self, "成功", f"双目图片已保存:\n{left_filename}\n{right_filename}")

        except Exception as e:
            self.dual_cam_log_append(f"[ERROR] 保存双目图片失败: {e}")
            QMessageBox.critical(self, "错误", f"保存双目图片失败: {e}")

    def toggle_stereo_ranging(self):
        """切换双目测距模式"""
        if not self.connected:
            QMessageBox.warning(self, "警告", "请先连接到FPGA")
            return

        # 切换模式
        self.stereo_ranging_mode = not self.stereo_ranging_mode

        try:
            # 发送UDP命令 0x00F5
            mode_byte = 0x01 if self.stereo_ranging_mode else 0x00
            cmd_packet = struct.pack('!HB', 0x00F5, mode_byte)

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sent = sock.sendto(cmd_packet, (self.fpga_ip, self.control_port))
            sock.close()

            # 更新UI
            if self.stereo_ranging_mode:
                self.stereo_ranging_btn.setText("关闭双目测距")
                self.stereo_ranging_btn.setStyleSheet("background-color: #27ae60; color: white; padding: 8px; font-weight: bold;")
                self.dual_cam_log_append(f"[INFO] 双目测距已启动，发送 {sent} 字节")
                self.dual_cam_log_append("[INFO] 显示格式：左右二值化图像 + 实时距离")
            else:
                self.stereo_ranging_btn.setText("开启双目测距")
                self.stereo_ranging_btn.setStyleSheet("background-color: #3498db; color: white; padding: 8px; font-weight: bold;")
                self.dual_cam_log_append(f"[INFO] 双目测距已关闭，恢复RGB模式，发送 {sent} 字节")

        except Exception as e:
            self.dual_cam_log_append(f"[ERROR] 切换双目测距模式失败: {e}")
            QMessageBox.critical(self, "错误", f"切换双目测距模式失败: {e}")

    def on_dual_shift_changed(self, value):
        """字节偏移量改变回调"""
        self.dual_cam_shift_bytes = value
        self.dual_cam_log_append(f"[INFO] 字节偏移已设置为: {value}")

    # ====================================================================
    # 窗口关闭事件
    # ====================================================================

    def closeEvent(self, event):
        """重写窗口关闭事件"""
        if self.is_receiving:
            self.stop_video_receiving()
        if self.is_dual_cam_receiving:
            self.stop_dual_cam_receiving()
        event.accept()

# ========================================================================
# 主函数
# ========================================================================

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = FPGAControlApp()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
