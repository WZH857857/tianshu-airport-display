# 摔倒检测 FPGA 工程（EG4S20NG88）

目标：把 `tools/fall_proto.py` 的算法逐模块翻译成 Verilog，全部逻辑在 FPGA
内完成，无 MCU / 无软核。硬件参考工程来自官方 HDK
`06_Anlogic_Examples/03-4_CMOS_OV5640_Color_1280720`（OV5640 采集 + SDRAM +
HDMI）与 `01-1_EG4_HDMI_TX_Test_640480`。

## 算法链路

```
OV5640 RGB565 640x480
   |
   v
gray_front      565 -> 888（位复制）-> 77/150/29 luma -> 2x2 box -> 320x240
                （再做 mod-4 行抽样，每 4 行丢第 4 行 -> 320x180）
   |
   v
frame_diff      上一帧 luma 存片内 RAM（read-first），|cur-prev| > 10
   |
   v
motion_morph    3x3 十字腐蚀 -> 半径 3 圆盘膨胀（列栈 + 滑动 popcount）
   |
   v
proj_bbox       行投影最长游程 -> 该带内列投影最长游程 -> 包围盒 + 面积
   |
   v
posture_fsm     站立参考 -> 塌陷候选 -> 恢复窗 -> 报警脉冲
```

全部为整数运算，无浮点。常量乘法（20*h、19*w、10*area 之类）一律写成
移位加，避免 TD 把它们推断成 DSP：不这样写时（run6/run7）TD 把 7 个
常量乘法塞进 `posture_fsm` 的 DSP，是当时 -9.524 / -0.622 ns 的主要
来源；改成移位加（run8）后 `posture_fsm` 只剩 1 个 DSP，即唯一的真
乘法 `w*h`，全链 DSP 4/29。

`posture_fsm` 的输入口还有一级流水（板上 `en` = 每帧一拍的 `bbox_valid`）：
第一拍把观测和它的乘积、比例比较存下来，第二拍状态机只读这些寄存器。这
不是为了省面积，而是为了在 48 MHz 下把原来「乘 → 移位加 → 相减 → 比较 →
优先编码」共 16 层逻辑的一条路径拆成两半——不拆的话整条路径只差 0.161 ns
收不回来（见下面的板级 P&R）。判决和 Python 参考完全一致，只是晚一个时钟
出结果。

### 人群多目标变体（4 槽）

上面的单目标链路假设「画面里只有一个最长游程」。机场大厅是人群场景，这版
另接了一条 `fall_core_multi_top`：

```
gray_front -> frame_diff -> motion_morph
   -> multi_bbox(4 槽, 8 连通域) -> posture_bank(1 x posture_fsm, 4 上下文)
```

`multi_bbox` 用的是**排除式投影**，不是片上连通域标记：每个运动连通域先算
行投影最长游程所在的竖直带，取最长的一条作为槽 0 的代表游程，把这个连通域
从掩膜里矩形抠除，再在剩下的像素上重复，直到 4 槽填满。

- 不需要 union-find / label 等价表，也不需要在片内平铺整帧掩膜；
- 槽位更新和 `proj_bbox` 一样是两遍行扫加一次列扫，复用同一套直方图 /
  游程结构。`multi_bbox` 综合后 4,533 LUT / 16 ERAM（三级流水版），是整条链
  的面积大头；
- 代价是槽的**顺序不稳定**：排除顺序取决于上一轮的游程长度，人群互相遮挡、
  掩膜抖动时槽号会串。

**为什么不用连通域 top-4**（决策记录）：先跑了游程密度统计
（`tools/run_stats.py`，结果在 `out/crowd_stats.txt`），crowd 素材在
320x180 下平均每帧 658~1091 条游程、p95 超过 1400 条。连通域标记要么给
每个游程建 label 表（片上存不下、访问带宽也不够），要么把掩膜整帧缓存
（ERAM32K 已经被 `frame_diff` 占满），所以改用排除式投影，把「人数上限」
做成固定 4 槽的硬件常量。

槽位顺序统计（`tools/multi_order_stats.py`，结果在
`out/crowd_order_stats.txt`，判据是逐帧同槽 IoU >= 0.3）：

| 素材 | 帧数 | 槽/帧 | 同槽匹配 | 跨槽漂移 | 新槽 | 平均 IoU |
| --- | --- | --- | --- | --- | --- | --- |
| crowd01 | 60 | 3.22 | 51.6% | 41 | 51 | 0.75 |
| crowd03 | 60 | 1.85 | 64.2% | 12 | 27 | 0.76 |
| crowd05 | 41 | 3.33 | 24.8% | 47 | 53 | 0.69 |

crowd05 是人贴人的密集段，同槽率掉到 24.8% —— 这是下一步要做片上 IoU
重编号 tracker 的直接依据。

板级变体 `fall_camera_multi_top` 和单目标 `fall_camera_top` 共用相机前端与
管脚表，只把核心换成 4 槽版，并在帧末用 `slot_scan` 把 4 槽快照依次按
`S<slot> F<frame> ...` 格式打成 4 行 UART（每行 36 字节，4 行共 1,440 bit，
在 115200 8N1 下约 12.5 ms，小于 33 ms 帧周期），单串口不用加宽。

### 人群变体的最终跑分（board_multi_synth.tcl + board_multi_pnr.tcl）

`multi_bbox` 的直方图读改写原来是「读 mux → 加一 → 写回」挤在一拍里，
布线后最差路径 21.693 ns（预算 20.833 ns），其中八成是线延时，直接导致
`SWNS -1.140 ns / 15 个违例 / slices 90.13%`，普通种子还会在
`u_core/u_bbox` 里撞出 `PHY-8023` 短接。把累加器拆成三级流水（地址级 →
读数级 → 写回级，见 `multi_bbox.v` 头部注释）之后，默认种子一次布通：

| 阶段 | SWNS | HWNS | 违例端点 | Fmax |
| --- | --- | --- | --- | --- |
| 综合后（board_multi_synth） | +1.388 ns | +0.509 ns | 0 | 51.427 MHz |
| 布局后（manhattan 估算） | +3.662 ns | — | 0 | 58.238 MHz |
| **布线后（final）** | **+2.000 ns** | **+0.037 ns** | **0** | **53.098 MHz** |

资源（`td/out_board_multi/multi_route.area`）：LUT 10,721/19,600（54.7%）、
REG 6,036（30.8%）、slices 6,062（61.9%）、ERAM9K 48/64、ERAM32K 16/16、
DSP 3/29、PAD 20/66。对比改之前的 P&R（LUT 13,600、slices 90.13%、
SWNS -1.140 ns、15 违例），LUT 降 21%，slices 降 28 个百分点。

流程末尾已经接上 `bitgen`，同一次运行直接吐出可下载的
`td/out_board_multi/multi.bit`（760,009 字节，EG4S20NG88）。重跑一遍的
QoR 与上表逐位相同（线长 1,521,784 也没变），说明固定种子下这条流程是
可复现的。

**这一版也踩过一个坑**：`mask_ram` 的 `ram_style = "bram_9k"` 提示只在
「读地址是寄存器、读数据也是寄存器」时才被 TD 采纳。第一版把
`mask_ram[acc_col]` 直接挂在活的列计数器上，结果整个 14,400 字节掩膜 bank
掉进触发器：`multi_bbox` 综合成 98,012 LUT / 118,366 seq，整体 528% LUT。
加回地址级寄存器（和 `proj_bbox.v` 同构）后是 4,533 LUT / 3,191 seq /
16 ERAM，掩膜仍在 ERAM 里。

**这一级流水踩过一次坑**：第一版只有观测数据进了寄存器，顺序块的判断仍在
读输入端口（`frame`/`valid`/`y`...）。板级顶层看不出问题（`bbox_*` 整帧
稳定），`tb_posture_fsm.v` 的向量回放也看不出问题（它把向量保持两拍），但
`tb_vision_chain.v` 在 bbox 脉冲那一拍就把帧计数器加一，于是状态机在第 2 拍
读到 `frame+1`：**候选的 `pending_frame` 记成 30（应为 29），报警从帧 31
漂到帧 32**。整链长程回归就是这么把它抓出来的：`alarms rtl=[32] ref=[31]`、
row 30 状态 PENDING ≠ FALL。现在第 2 拍全部只认 `p_*`，输入端口只要求在
`en` 那一拍有效；`tb_posture_fsm.v` 也加了 `+HOSTILE=1` 模式——在 `en`
拉低后立刻把 `frame` 加一，专门盯这条契约（`rtl_check.py` 默认打开它）。

## 目录

| 路径 | 作用 |
| --- | --- |
| `rtl/gray_front.v` | RGB565 → 8bit luma → 2x2 平均 → 3-of-4 行抽样（320x180） |
| `rtl/frame_diff.v` | 帧间差分，上一帧缓存 57.6 KB（ERAM，跨 32K/9K 两区拆分） |
| `rtl/motion_morph.v` | 3x3 腐蚀 + 7x7 圆盘膨胀，纯 de 流驱动 |
| `rtl/proj_bbox.v` | 两遍投影包围盒，双 bank 乒乓，后台 pass 不影响流侧 |
| `rtl/multi_bbox.v` | 人群 4 槽排除式投影包围盒（8 连通域候选，矩形抠除后重扫） |
| `rtl/posture_fsm.v` | 姿态塌陷状态机（`PostureMachine` 的整数 RTL 版） |
| `rtl/fall_core_multi_top.v` | 人群 4 槽视觉核：共享前端 + `multi_bbox` + `posture_bank` |
| `rtl/posture_bank.v` | 1 份 `posture_fsm` 分时服务 4 个槽，4 套上下文各自保存 |
| `rtl/cmos_capture_rgb565.v` | OV5640 DVP 8 bit 字节对 → RGB565，等 10 帧后放行，含 `hw_err` |
| `rtl/ov5640_sccb_cfg.v` | 250 条 SCCB 寄存器表（24 MHz → 250 kHz），复位后插 2 ms 稳定期 |
| `rtl/led595_shift.v` | 74595 移位驱动器（板载 8 位 LED） |
| `rtl/status_uart.v` | 115200 8N1 状态行；`SHOW_SLOT=0` 是单目标旧格式，`=1` 加 `S<slot> ` 前缀 |
| `rtl/slot_scan.v` | 帧末把 4 槽快照依次送进单个 `status_uart`，带超时防卡死 |
| `rtl/fall_camera_top.v` | **板级顶层**：相机 + 视觉核 + LED + 串口，无 PLL、无外部存储 |
| `rtl/fall_camera_multi_top.v` | **人群板级顶层**：同管脚/前端，核心换 4 槽 + `slot_scan` |
| `tb/tb_pixel_front.v` | 前端（gray+diff）字节级回放 |
| `tb/tb_motion_morph.v` | 形态学掩膜回放（可加像素/行空档） |
| `tb/tb_proj_bbox.v` | 包围盒掩膜回放 |
| `tb/tb_multi_bbox.v` | 排除式投影包围盒掩膜回放（逐槽 BOX 输出） |
| `tb/tb_multi_top.v` | **人群整核**：真实 `fall_core_multi_top`，逐槽 BOX/状态/报警 |
| `tb/tb_slot_uart.v` | `slot_scan` + `status_uart` 联合测试台，解码 4 行 ASCII |
| `tb/tb_vision_chain.v` | **整链**：RGB565 帧 → 包围盒 + 姿态状态 + 报警 |
| `tb/tb_posture_fsm.v` | 向量回放测试台：每行 = 一次 `feed()` 调用 |
| `tb/tb_alarm_pulse.v` | 报警脉冲宽度：`en` 单拍选通 vs 常开两种接法都必须只有 1 拍 |
| `tb/tb_board_top.v` | **板级集成**：行为级 OV5640 + SCCB 从机 + UART/74595 解码，`+LONG` 跑站立→跌倒 |
| `td/board_synth.tcl` / `td/board_pnr.tcl` | 板级顶层的非工程综合 / 完整 P&R |
| `td/board_multi_synth.tcl` / `td/board_multi_pnr.tcl` | 人群板级顶层的非工程综合 / 完整 P&R（复用同一份 `.adc/.sdc`） |
| `td/fall_camera_top.adc` / `.sdc` | 板级管脚（20 pad）与 `clk_24m` / `cmos_pclk` 约束 |
| `tools/pixel_check.py` | 前端对拍（luma + 掩膜字节级） |
| `tools/morph_check.py` | 形态学对拍（scipy 参考） |
| `tools/bbox_check.py` | 包围盒对拍（Python 两遍投影参考） |
| `tools/multi_bbox_check.py` | 4 槽排除式包围盒对拍（合成序列 + 参考模型） |
| `tools/multi_model.py` | 排除式投影的 Python 参考模型（RTL 逐操作对齐） |
| `tools/multi_top_check.py` | **人群整核对拍**：视频 → 4 槽框 + morph + diff + 4 路姿态 |
| `tools/multi_rtl_check.py` | 4 路姿态机轨迹对拍（逐帧状态与报警号） |
| `tools/multi_order_stats.py` | 槽位顺序稳定性统计（同槽 IoU 匹配率） |
| `tools/run_stats.py` | 游程/连通域密度统计，给「排除式 vs 连通域」做决策依据 |
| `tools/chain_check.py` | **整链对拍**：一帧一条结果，含姿态状态与报警帧 |
| `tools/rtl_check.py` | 姿态机对拍：Python 逐轨迹 `feed()` 序列 vs RTL |
| `tools/make_crowd_strip.py` | 把 RTL dump 画成人群 4 槽可视化条带 |
| `tools/make_crowd_report.py` | 汇总资源/时序/统计，生成一页 PNG 报告 |
| `sim/gen/` | 自动生成的 `params*.vh`、向量/输入文件、仿真可执行文件 |

## 模块接口契约

时序驱动的坑（务必遵守，违反会静默错位）：

- **驱动约定**：所有测试台必须 `@(negedge clk)` 摆数据、`@(posedge clk)` 采样。
  在 negedge 之前就把 `de_in`/像素摆好，会让首像素被采两次，整帧右移一格。
- `frame_start` 只允许一个 posedge 看到：在 negedge 拉高、下一个 negedge 拉低。
  用 `frame_start=1; @(negedge); @(posedge); frame_start=0;` 会被两个 posedge
  看到，`frame_cnt` 多计一帧，首帧读完未初始化 RAM（仿真 X、硬件随机）。
- `motion_morph` / `proj_bbox` 不需要 `frame_start`，靠 `de_in` 自行数行
  （320 像素 × 180 行一帧）；`motion_morph` 在帧末自动补 7 行 flush，
  这段时间必须落在垂直消隐里（7×326 拍 @74.25 MHz ≈ 31 us，720p30 消隐
  约 4.4 ms，余量充足）。
- `proj_bbox` 输出的是**上一帧**的包围盒（pass 侧与流侧乒乓并行），
  一帧一个 `out_valid` 脉冲，结果同拍有效。
- `posture_fsm` 有 `en` 时钟使能：整链里由 `out_valid` 驱动，保证空档和
  冷却期按**帧**计数而不是按**时钟**计数；独立回放测试台恒接 1。

## 对拍验证（当前已完成）

```powershell
$env:PYTHONIOENCODING='utf-8'
P=$env:PYTHONIOENCODING; $py='E:\anaconda3_install\python.exe'
$tools='E:\FPGA guo sai\fall_detection\fpga\tools'

# 1) 前端：luma + 掩膜字节级
& $py -X utf8 "$tools\pixel_check.py" --frames 4
# 2) 形态学：合成图案 + 真实素材
& $py -X utf8 "$tools\morph_check.py" --frames 3 --clips <clip.mp4> ...
# 3) 包围盒
& $py -X utf8 "$tools\bbox_check.py" --frames 4 --clips <clip.mp4> ...
# 4) 姿态机（逐轨迹 feed 序列）
& $py -X utf8 "$tools\rtl_check.py"
# 5) 整链：视频 -> 掩膜 -> 形态学 -> 包围盒 -> 姿态/报警
& $py -X utf8 "$tools\chain_check.py" --frames 4 --clips <clip.mp4> ...

# 6) 4 槽包围盒（合成序列 + 参考模型）
& $py -X utf8 "$tools\multi_bbox_check.py" --frames 4
# 7) 人群整核：视频 -> 4 槽框 + morph + diff + 4 路姿态
& $py -X utf8 "$tools\multi_top_check.py" --clips datasets\crowd\crowd01.mp4 --frames 4
# 8) 4 路姿态机轨迹
& $py -X utf8 "$tools\multi_rtl_check.py"
# 9) 槽位顺序稳定性（不比对，只出统计）
& $py -X utf8 "$tools\multi_order_stats.py" --frames 60

# 无梯子/无 Icarus 时：本机 iverilog 在 C:\iverilog\bin
```

`multi_top_check.py` 一次跑 4 帧约 2.5 分钟（每帧 307,200 像素，逐级
回放），长跑要留时间。

`chain_check.py` 的判据（任一不符即 FAIL）：

1. 逐帧包围盒 `found x y w h area` 完全一致；
2. 形态学输出掩膜逐位一致（可选 `+MORPH` 探针）；
3. 帧差掩膜逐位一致（可选 `+DIFF` 探针）；
4. 姿态状态轨迹（SEEK/PENDING/FALL/RECOVERED）逐帧一致；
5. 报警帧号完全一致。

参考实现与 RTL 逐操作对齐：`RGB565 → 位复制 → 77/150/29 luma → 2x2 box
→ |diff|>10 → erosion(disk 1) → dilation(disk 5) → 两遍投影 bbox
→ PostureMachine`。参数（`MAX_REF_W/H/AREA`、`SUNK_CY2`、TTL、塌陷窗、
恢复窗、冷却期）由 `chain_check.py` 从 `Params.for_fps(30)` 直接生成
`sim/gen/params_chain.vh`，不手抄。

已通过的对拍（2026-10-05）：

| 层次 | 素材 | 结果 |
| --- | --- | --- |
| 前端 `pixel_check` | 13 / 02，4 帧 | luma + 掩膜字节级一致 |
| 形态学 `morph_check` | 6 组合成图案 + 3 段素材 | 逐位一致（含空档时序） |
| 包围盒 `bbox_check` | 合成 + 13 / 07 / crowd01 | 逐帧 `found x y w h area` 一致 |
| 姿态机 `rtl_check` | 4 段素材 14 条轨迹，`+HOSTILE=1` | 报警帧 + 逐帧状态一致（`en` 拉低即换输入也能过） |
| 整链 `chain_check` | 13 / 02 / 19 / 07 / crowd01 / crowd03 / day6，4 帧 | 7/7 五级结果全部一致 |
| 整链长程 `chain_check` | Subject1/Fall/13 前 45 帧 | 45 帧 bbox / 掩膜 / 状态全一致，报警帧 RTL=31 = 参考=31 |
| 整链长程 `chain_check` | Subject3/Fall/01 前 45 帧 | 45 帧 bbox / 掩膜 / 状态全一致，报警帧 RTL=41 = 参考=41 |
| 4 槽包围盒 `multi_bbox_check` | 合成 4 帧 + crowd01/crowd05 | 合成 4/4 IoU>=0.5（mean 0.827）；crowd 素材与排除参考完全一致 |
| 人群整核 `multi_top_check` | crowd01 / crowd05 / Subject1-Fall-13，各 4 帧 | 4 槽框 + morph + diff + 4 路姿态逐帧全部一致 |
| 人群整核长程 `multi_top_check` | Subject1/Fall/13 前 45 帧 | 180 个槽框 + morph + diff + 4 路姿态逐帧全部一致 |
| 4 路姿态机 `multi_rtl_check` | 3 段素材 | 逐帧状态与报警号一致 |
| 串口分槽 `tb_slot_uart` | 两组载荷快照 | 8 行 ASCII 全对（覆盖 000/1FF/1FFFF 等边界） |
| 报警脉冲 `tb_alarm_pulse` | `en` 单拍选通 / 常开两种接法 | 各 1 次报警，最长电平 1 拍 |
| 板级集成 `tb_board_top` | 行为级 OV5640 + SCCB 从机 + UART/74595 解码 | SCCB 250 写完成，bbox / UART / hw_err 全对 |
| 板级跌倒 `tb_board_top +LONG` | 站立 → 塌陷 | SEEK→PENDING→FALL，UART `state=2`，1 条 ALARM |

### 阈值必须和验证配置一致

`posture_fsm` 的那组常量不是可以随手调的：`chain_check.py` 会把它们写进
`sim/gen/params_chain.vh`，`tb_vision_chain.v` / `tb_posture_fsm.v` 从宏取值，
所以 **testbench 和参考实现永远描述同一个配置**。板级顶层
`td/fall_core_top.v` 里的那份必须手工跟它对齐，规则是：

| 常量 | 来源 |
| --- | --- |
| `MIN_REF_AREA` / `MIN_REF_HEIGHT` | `Params` 原值，**不按分辨率缩放**（1600 / 25） |
| `MAX_REF_W` / `MAX_REF_H` / `MAX_REF_AREA` | 分辨率分数门（0.55W / 0.95H / 0.20WH），180 行时是 176 / 171 / 11520 |
| `SUNK_CY2` | `ceil(0.08 * FRAME_H)`，180 行时是 15 |
| `REF_TTL` / `COLLAPSE_FRAMES` / `RECOVERY_WINDOW` / `COOLDOWN_FRAMES` | `Params.for_fps(30)` 的帧数（300 / 36 / 2 / 60） |

这条规则是踩过坑才写下来的：`fall_core_top` 曾经把两个绝对门按 180/240 缩成
1200/19，而 `res_eval.py`、`rtl_check.py`、45 帧整链用的都是 1600/25 ——
板子上跑的是一套**没有任何验证支撑、而且更宽松**的阈值。同一时期
`chain_check.posture_ref` 还在用 `PostureMachine` 的默认 `frame_h=240`，
于是参考侧的 `MAX_REF_H` / `MAX_REF_AREA` 是 228 / 15360，testbench 是
171 / 11520，两边其实不是同一个配置。两处都已经改正，45 帧回归重跑通过。

长程回归用的是整帧 `$fread` 进内存数组的测试台（逐字节 `$fread` 版本要
2760 万次文件调用，45 帧跑不完）；跑一次约 20 分钟，日志留在
`out/chain_45f.txt`。

浮点阈值在 RTL 中全部改为整数交叉相乘（见 `posture_fsm.v` 头部注释）；
对拍集里未出现舍入差异。

## 综合与时序（TD 6.2.1，EG4S20NG88）

```powershell
cd 'E:\FPGA guo sai\fall_detection\fpga\td'
& 'E:\Anlogic\TD_6.2.1_Engineer_6.2.168.116\bin\td_commands_prompt.exe' 'synth.tcl'
```

`td/synth.tcl` 是**非工程模式**流程，跑到 `optimize_gate` 为止：此时
ERAM/DSP 映射已经定死、面积报告有意义，而 P&R 是单独一步（否则一次
长时间布局布线会把综合结果淹掉）。`td/fall_core_top.v` 是纯核顶层，
只保留算法本身的寄存器，不掺板级 I/O。

**时钟口径**：48 MHz / 20.833 ns，取自官方 HDK 例程
`06_Anlogic_Examples/03-3_CMOS_MT9V034_Gray_752480/sdc/EGS20NG88_Board.sdc`
对 `cmos_pclk` 的约束。640x480 RGB565 就是按这个像素时钟进来的，不是
HDMI 的 74.25 MHz。

run8（当前 RTL）实测：

| 项 | 实测 | 器件 | 占用 |
| --- | --- | --- | --- |
| LUT | 11,312 | 19,600 | 57.7% |
| REG | 5,936 | 19,600 | 30.3% |
| ERAM9K | 48 | 64 | 75.0% |
| ERAM32K | 16 | 16 | 100% |
| DSP | 4 | 29 | 13.8% |
| SWNS / STNS | +0.600 ns / 0.000 ns | | 满足 |
| Fmax | 49.424 MHz | | 对 48.001 MHz 目标 |

各子模块面积（综合报告）：

| 实例 | 模块 | LUT | seq | ERAM | DSP |
| --- | --- | --- | --- | --- | --- |
| `u_bbox` | proj_bbox | 7,885 | 5,444 | 16 | 1 |
| `u_morph` | motion_morph | 1,331 | 177 | 0 | 0 |
| `u_gray` | gray_front | 867 | 30 | 0 | 2 |
| `u_fsm` | posture_fsm | 451 | 195 | 0 | 1 |
| `u_diff` | frame_diff | 209 | 58 | 48 | 0 |

`proj_bbox` 是面积大头（两遍投影 + 双 bank 乒乓掩膜都在里面）。报告里
`#IO 79/66 = 119.7%` 是纯核顶层把内部信号全引成端口的结果，板级包装
只导出 bbox/状态/报警，不会真的超管脚。

把 WNS 从 -9.524 ns 拉到 +0.600 ns 的两处改动：

1. `proj_bbox` 的直方图扫描拆成两拍（`span_phase`）：比较器前面插一级
   读寄存器，长游程比较不再和 RAM 读叠在一拍里。
2. `posture_fsm` 常量乘法全部改写移位加，并把 `ref_h * ref_w` 的乘积
   存成寄存器 `ref_wh`。TD 原来把 7 个常量乘法塞进 DSP，MULT18 的
   3.5 ns × 2 直接压在关键路径上。

### 布局布线（td/pnr.tcl）

```powershell
cd 'E:\FPGA guo sai\fall_detection\fpga\td'
& 'E:\Anlogic\TD_6.2.1_Engineer_6.2.168.116\bin\td_commands_prompt.exe' 'pnr.tcl'
```

同一份 RTL 走完 `legalize_phy_inst -> place -> route`，时序：

| 阶段 | SWNS | HWNS | Fmax |
| --- | --- | --- | --- |
| 综合后（optimize_gate） | +0.600 ns | +0.509 ns | 49.424 MHz |
| 布局后（manhattan 估算） | +3.726 ns | +0.094 ns | — |
| **布线后（final）** | **+0.259 ns** | **+0.004 ns** | **48.605 MHz** |

布线后资源（`out/pnr_route.area`）：LUT 11,714/19,600（59.8%）、
REG 6,262（32.0%）、ERAM9K 48/64、ERAM32K 16/16、DSP 4/29、
PAD 34/66。比综合后略高是正常的：`phys_opt` 会插缓冲、复制高扇出寄存器，
而且布线后的统计口径还包含「用作 DRAM/shifter 的 LUT」。

**两个必须知道的坑**（都在这条流程里踩过）：

1. **器件只有 66 个用户 pad**。EG4S20NG88 是合封 SDRAM 的封装，SDRAM 走
   封装内部连接、不占用户 pad，但诊断顶层 `fall_core_top` 把内部信号
   全引出来，一共 79 个 pad，place 直接报
   `PHY-9016 ERROR: pad number = 79, exceeds the limit 66`。
   所以 P&R 用 `td/fall_core_pnr_top.v` 当顶层：核心原封不动，只把
   `bbox_x/y/w/h/area` 折成一个 8 位 XOR 签名（53 位全部参与折叠，
   综合无法把 bbox 逻辑裁掉），pad 数降到 34。板级包装不需要这个技巧，
   那时 bbox 直接接片内 HDMI 叠加逻辑。
2. `report_timing_path` 没有 `-nworst`，终点/路径条数是 `-ep_num` /
   `-path_num`（SWUG105 4.8.1），写错会让脚本在最后一步报
   `USR-8011` 中止。

约束文件 `td/fall_core_pins.adc` 只给 `clk` 指了真实管脚 P79（板上
OV5640 的 PCLK），其余端口交给工具自动布局——这一步只回答「核本身在
布线后还能不能收敛」，完整管脚表见 `docs/板级接入.md`。

### 板级顶层 P&R（td/board_pnr.tcl，当前 bitstream 口径）

上面那组核心数字是 `fall_core_top` 的旧跑分。真正要下板的是
`fall_camera_top`（SCCB + DVP 采集 + 视觉核 + 74595 + 115200 UART，20
个 pad），流程在 `td/board_pnr.tcl`。下面这组是**当前 RTL**（含
`posture_fsm` 输入流水与 `p_*` 端口修复）的完整跑分，`PNR_SEED=2`：

| 阶段 | SWNS | HWNS | 违例端点 | Fmax |
| --- | --- | --- | --- | --- |
| 综合后（board_synth） | +1.561 ns | +0.509 ns | 0 | 51.889 MHz |
| 布局后（manhattan 估算） | +5.442 ns | +0.030 ns | 0 | — |
| **布线后（final）** | **+1.208 ns** | **+0.030 ns** | **0** | **50.955 MHz** |

资源（`out_board/board_route.area`）：LUT 12,974/19,600（66.2%）、
REG 7,047（36.0%）、slices 7,440（75.9%）、ERAM9K 48/64、
ERAM32K 16/16、DSP 3/29、PAD 20/66。

流程里踩到的三件事：

1. **默认种子的布线会失败**。`HDL-5373` 修完之后综合后 LUT 从 12,185
   涨到 12,377，同一策略的默认种子在 `route` 最后剩 1 处
   `PHY-8023 短接`（`RUN-8102: Incremental route failed`）。
   `board_pnr.tcl` 现在认环境变量 `PNR_SEED`（`set_param place seed`），
   `PNR_SEED=2` 布通并留出 +1.208 ns；不设就是工具默认种子。
2. **`fix_hold` 要单独跑**。厂商 `DefaultFlow.tcl` 在 `route` 之后调用
   `fix_hold`，之前漏了这一步，HWNS 只有 +0.014 ns；补上之后是
   +0.030 ns。
3. 两条已知无害警告：`HDL-5001`（匿名 net 未重命名）、`PHY-5079`
   （`cmos_xclk` 单扇出输出走局部布线——它只驱动相机连接器的一个脚，
   不是时钟树）。

在加流水之前同一流程的最终结果是 SWNS -0.161 ns / 4 个 setup 违例，
最差路径 20.642 ns（预算 20.833 ns）：

```
bbox out_h_reg[1] --MULT18(w*h)--> plike --rh--> 20*h/13*rh
                   --cy2 减法--> sunk 比较 --> ground 比较
                   --priority mux--> pending_frame[*].ce   (16 逻辑层)
```

现在这条链拆成两段，每段各自有 20.833 ns；代价是
`posture_fsm` 多约 80 个寄存器（LUT +75、REG +122、DSP 不变），
对 19,600 的预算可以忽略。

## 片内存储预算（TD 实测口径）

EG4S20NG88 的 ERAM 不是一个连续的 1088 Kb，而是**两块独立区域**，TD 按
8 bit 条目计费时两块的有效容量也不同（用 `td/ramprobe*.v` 13 个探针工程
实测）：

| 区域 | 块数 | 每块有效条目（8 bit） | 小计 |
| --- | --- | --- | --- |
| ERAM9K | 64 | 600 | 38,400 |
| ERAM32K | 16 | 2,400 | 38,400 |
| 合计 | | | **76,800 条目** |

所以「1088 Kb = 136 KB」是位宽口径，真正能装的是 76,800 个字节条目，
320x240 的一帧（76,800）+ 乒乓掩膜（14,400）装不下，必须做行抽样。

当前占用：

| 存储 | 大小 | 映射 |
| --- | --- | --- |
| `frame_diff.prev_ram_lo` | 38,400 B | ERAM32K ×16（32K 区占满） |
| `frame_diff.prev_ram_hi` | 19,200 B | ERAM9K ×32 |
| `proj_bbox.mask_ram` | 2 × 7,200 B | ERAM9K ×24 |
| `motion_morph.stack3` | 320 × 3 bit | DRAM/slice |
| `motion_morph.stack11` | 320 × 11 bit | DRAM/slice |
| `proj_bbox.hist` | 320 × 16 bit | slice |

TD 综合报告（run8）实测：**ERAM9K 48/64、ERAM32K 16/16**，余量 16 个
9K 块。TD 的推断日志里 `prev_ram_lo` 报 `65536 x 8`、`prev_ram_hi`
`32768 x 8`、`mask_ram` 每 bank `16384 x 8`——那是按**地址端口位宽**报的
容量上界，不是实际写入深度，最终映射按实际深度裁到上表。32K 区（一帧
前半的 38.4 KB）已经占满，再加深任何一份 8 bit 存储都要掉到 9K 区或
外部存储。

### TD 映射的三条硬规则（改代码前先读）

1. **一个 always 块里只能有一个大数组**。两个数组写在同一 always 块时，
   TD 会试图把整组塞进同一个区，失败后把**全部**数组丢回 LUT，综合时间
   和资源都会爆掉（实测 112 万实例、10 分钟还没跑完 Stage 1.5）。
2. **地址端口位宽决定 TD 估算的容量**，不是数组声明深度。19,200 深的数组
   给 16 bit 端口 → TD 按 2^16 估算 → 判定装不下 → 退回 LUT。必须给
   `$clog2(深度)` 这么宽。
3. **ERAM32K 不支持 read-before-write**。同一地址同拍读写会报
   `SYN-8418 ERROR`。读、写必须拆到两个 always 块（SDP），并且写地址滞后
   读地址至少一拍，保证两端口不撞同一地址。

`ramprobe13`~`ramprobe18` 就是逐步验证这三条留下的探针工程。

## 下一步

前三步（DVP 采集 + SCCB 初始化、报警出到 74595/UART、48 MHz 综合）已经在
`fall_camera_top` + `board_pnr.tcl` 上闭环，见上面的 P&R 与验证矩阵。剩下：

1. **HDMI 画框**：用片内帧缓存（ERAM32K 已占满，需要先压缩灰度位宽或降
   显示分辨率）把 `proj_bbox` 的 x/y/w/h 映射成 640x480 显示坐标，叠加
   绿框 + 状态色条；管脚 P47/P49/P55/P61（LVDS25），参考官方
   `01-1_EG4_HDMI_TX_Test_640480`。
2. **槽位重编号 tracker（FPGA 内）**：4 槽排除式投影已经做完并通过对拍，
   但密集人群（crowd05）的同槽匹配率只有 24.8%，槽号会串。下一步在
   `multi_bbox` 和 `posture_fsm` 之间插一级 IoU 关联：用上一帧 4 槽框和
   本帧 4 槽框算 IoU 矩阵，贪心按最大重叠重排本帧槽号，让每条轨迹固定
   占一个槽。预算 ~500 LUT（LUT 余量 4,524），ERAM 不追加。做完之后
   重跑 `multi_order_stats.py`，目标是 crowd05 同槽率上到 80%+。
3. **阈值现场标定**：`MIN_REF_AREA` / `SUNK_CY2` / `COLLAPSE_FRAMES`
   现在按 30 fps 整定，机场远机位和近机位的人像像素尺度差很大，落地前
   要用现场录像重标定一版参数并重跑 `chain_check`。
4. **系统联动**：E104 小板接口线序 + 报警上报协议（主卡要什么格式的
   事件包），以及断线/相机失联时的自检上报。
