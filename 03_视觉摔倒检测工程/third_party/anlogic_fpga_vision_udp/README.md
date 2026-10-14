# 基于安路 FPGA 的智能视觉与以太网图像传输系统

> 全国大学生嵌入式芯片与系统设计竞赛 **FPGA 创新赛道 · 全国一等奖**
>
> 在安路 EG4S20BG256 FPGA 上实现的双模式视频处理系统：UDP 图像接收 → SDRAM 缓冲 → HDMI 显示（模式 0），以及 SD 卡 / 双 OV5640 摄像头采集 → SDRAM 缓冲 → UDP 发送（模式 1），支持双目立体视觉、实时图像处理与模板匹配目标识别。

## 技术栈

| 分类 | 技术 | 说明 |
|---|---|---|
| **FPGA 芯片** | 安路 **EG4S20BG256**（EG4 系列） | 2 万 LUT4 级逻辑容量 |
| **开发工具** | 安路 **TD**（工程 `prj.al`） | 综合 / 布局布线 / 下载 |
| **设计语言** | Verilog HDL（`import/` 全部源码） | 顶层 `top_final` |
| **以太网** | 千兆 **RGMII** + 安路 **TEMAC_CORE** IP + UDP/IP 协议栈 | `udp_ip_protocol_stack_merge.enc.v`（加密 IP）+ 自研 UDP 应用层 |
| **存储** | SDRAM 控制器 + 硬件仲裁器（`sdram_arbiter`） | 多通道地址空间隔离、互不干扰 |
| **视频输入** | 双 **OV5640** 摄像头（SCCB/I²C 配置 + DVP 8-bit 并行） | 双目立体视觉与距离测量 |
| **视频输出** | **HDMI**（差分 TMDS） | 多分辨率：VGA / 720p / 1080p |
| **图像处理** | 二值化、腐蚀、膨胀、形态学、帧差法 | 纯逻辑实时流水线实现 |
| **目标识别** | 模板匹配（`digit_recog`） | 数字识别，PC 端 `template_match_client.py` 协同 |
| **时钟方案** | `sys_pll` / `video_pll` / `pll_core` + 异步 FIFO ×3 | 多时钟域安全跨域 |
| **外设接口** | SD 卡 SPI、数码管、LED、按键 | `spi_master` / `seg_scan` / `ax_debounce` |
| **上位机** | Python 3 + **PyQt5** + numpy + OpenCV + Pillow + socket | `prj.py`（约 3000 行） |

## 系统架构

```
                    ┌────────────── top_final（EG4S20BG256）──────────────┐
 模式0 图像接收:    │  PC ──UDP──> udp_rx ──> SDRAM(地址空间A) ──> HDMI   │
                    │                                                     │
 模式1 摄像头采集:  │  双 OV5640 ─┐                                        │
                    │   SD 卡 BMP ─┼──> SDRAM(地址空间B) ──> udp_tx ──> PC │
                    │                                                     │
 公共通路:          │  clk_50 → sys_pll/video_pll → 各时钟域               │
                    │  RGMII ←→ TEMAC_CORE ←→ UDP/IP 协议栈              │
                    │  sdram_arbiter 硬件仲裁（读/写/刷新优先级）           │
                    │  图像处理流水线（二值化/腐蚀/膨胀/帧差/模板匹配）      │
                    └─────────────────────────────────────────────────────┘
 模式切换: UDP 命令（udp_mode_ctrl）或板载按键 key3
 控制链路: udp_cam_ctrl / udp2sdr / video_delay / video_timing_data
```

- **双模式工作**：UDP 命令或按键动态切换；SDRAM 按地址空间分区 + 硬件仲裁，两种模式互不干扰。
- **网络链路**：千兆 RGMII PHY → TEMAC_CORE → 加密 UDP/IP 协议栈 → 自研应用层（图像分包/组包、控制命令解析）。
- **双目视觉**：双 OV5640 同步采集（`dual_cam`），配合 `frame_fifo_read/write` 帧缓冲与 `line_buffer` 行缓存完成立体匹配与距离测量。
- **图像处理**：`color_bar` 测试图源、`bmp_read` BMP 解码、`video_define/video_delay/video_timing_data` 视频时序控制。
- **人机交互**：`led` / `seg_decoder` / `seg_scan` 状态显示，`ax_debounce` 按键消抖。

## 上位机（prj.py）功能

| 模式 | 功能 | 命令 |
|---|---|---|
| LED 模式 | 2 个 LED 控制 | `0x0001` |
| 数码管模式 | 8 位数码管 + 4 个单色 LED | `0x0002` |
| 图片发送 | 640×480 RGB 图片经 UDP 下发 FPGA → HDMI | 图片数据流 |
| SD 卡接收 | 接收 SD 卡 BMP 图片显示 | `0x0003` |
| 摄像头接收 | 实时接收摄像头视频流 | `0x0004` |

- 网络参数：FPGA `192.168.240.1`、PC `192.168.240.2`、视频端口 `6001`（可按需修改）。
- GUI：PyQt5 多页面控制台（QStackedWidget），OpenCV 解码显示视频流，Pillow 处理图片。

## 目录结构

```
├── import/                          # Verilog 源码
│   ├── top.v                        # 顶层 top_final（含完整端口与时序说明注释）
│   ├── top.sdc / top.adc            # 时序约束
│   ├── udp/                         # UDP/IP 协议栈（含加密 IP）与时钟
│   ├── sdram/                       # SDRAM 控制器与仲裁
│   ├── hdmi/                        # HDMI 输出（VGA/720p/1080p）
│   ├── ov5640/                      # OV5640 采集（cmos_capture_data / i2c_dri）
│   ├── dual_cam/                    # 双目立体视觉
│   ├── digit_recog/                 # 数字识别（模板匹配）
│   ├── ip_core/  other/             # IP 封装与行缓存
│   └── (根级) 帧缓冲/视频时序/BMP解码/LED/数码管/SD卡/SPI 等模块
├── al_ip/                           # 安路 IP：PLL×3、异步 FIFO×3、SDRAM、TEMAC_CORE
├── prj.al                           # 安路 TD 工程文件
├── prj.py                           # PyQt5 上位机
├── prj_devicesetting.cfg            # 器件配置
└── README.md
```

## 快速上手

### FPGA 构建

1. 安路 TD 打开 `prj.al`（器件 EG4S20BG256）
2. 重新生成 runs（`prj_Runs/` 构建产物不入库）后综合布线、生成 bit 流下载

### 上位机运行

```powershell
pip install PyQt5 numpy opencv-python pillow
python prj.py
```

## 备注

- `udp_ip_protocol_stack_merge.enc.v` 为安路加密 IP，仅可在此平台综合。
- 本仓库仅含源码与工程文件；`prj_Runs/`、`log/`、崩溃转储等产物已通过 `.gitignore` 排除。
