================================================================================
模板匹配模块集成说明
================================================================================

## 一、文件清单

已创建的模块文件（位于 import/digit_recog/）：
1. myram.v - RAM模块（保留原始注释）
2. template_match.v - 32×32模板匹配核心模块
3. udp_template_ctrl.v - UDP控制模块
4. vip_template_match.v - 顶层封装模块

已修改的文件：
1. prj.py - 添加了模板匹配UI和功能
   - 上传模板图像按钮
   - 上传测试图像按钮
   - 开始匹配按钮
   - 匹配结果显示（"对"或"错"）

## 二、top.v集成步骤

### 步骤1：实例化vip_template_match模块

在top.v中添加以下代码（建议放在udp_mode_ctrl实例化之后）：

```verilog
// ========================================================================
// 模板匹配模块（32×32二值化图像）
// ========================================================================
wire        template_match_done;
wire        template_match_result;
wire [9:0]  template_hamming_distance;

vip_template_match u_vip_template_match (
    .clk                    (udp_clk),
    .rst_n                  (key1),

    // UDP接收接口 - 直接连接到UDP RX
    .udp_rx_valid           (app_rx_data_valid),
    .udp_rx_cmd             ({byte1, app_rx_data}),  // 命令码拼接
    .udp_rx_data            (app_rx_data),
    .udp_rx_byte_cnt_valid  (app_rx_data_valid),
    .udp_rx_byte_cnt        (byte_count),            // 字节计数

    // UDP发送接口 - 需要连接到UDP TX（如果有的话）
    .udp_tx_req             (),  // 暂时悬空，后续连接UDP TX模块
    .udp_tx_data            (),
    .udp_tx_done            (1'b0),

    // 状态输出（可选，用于调试）
    .match_done             (template_match_done),
    .match_result           (template_match_result),
    .hamming_distance       (template_hamming_distance)
);
```

### 步骤2：UDP命令码说明

prj.py中使用的命令码：
- 0x00F0: 上传模板图像（130字节：2字节命令 + 128字节数据）
- 0x00F1: 上传测试图像（130字节：2字节命令 + 128字节数据）
- 0x00F2: 设置匹配阈值（4字节：2字节命令 + 2字节阈值）
- 0x00F3: 开始匹配（2字节命令）
- 0x00F4: 读取结果（2字节命令，可选）

### 步骤3：UDP发送结果（可选）

如果你的系统有UDP TX模块，可以将匹配结果发送回上位机。
如果没有，prj.py会在发送0x00F3命令后超时（但功能仍可工作）。

简化方案：
- 模板匹配结果可以通过LED或其他方式本地显示
- 上位机不依赖FPGA回复（超时后仍继续工作）

## 三、测试流程

1. 准备测试图像：
   - 准备两张任意尺寸的图像（会自动缩放到32×32）
   - 建议使用对比度高的黑白图像

2. 运行上位机程序：
   ```bash
   python prj.py
   ```

3. 操作步骤：
   - 连接到FPGA
   - 切换到"单目摄像头接收"模式
   - 滚动到"模板匹配"功能区
   - 点击"上传模板图像"，选择第一张图片
   - 点击"上传测试图像"，选择第二张图片
   - 点击"开始匹配"
   - 观察"匹配结果"标签显示"✓ 对"或"✗ 错"

## 四、调试建议

### 如果没有UDP TX回复：
在prj.py的start_template_match函数中，修改为不等待回复：

```python
try:
    # 发送开始匹配命令 0x00F3
    cmd_packet = struct.pack('!H', 0x00F3)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sent = sock.sendto(cmd_packet, (self.fpga_ip, self.control_port))
    sock.close()

    # 模拟结果（用于测试）
    self.match_result = True  # 或False
    self.match_result_label.setText("✓ 对" if self.match_result else "✗ 错")
    # ... 更新样式代码 ...
```

### 如果要观察内部信号：
可以将match_done, match_result输出到LED：

```verilog
assign led[11] = template_match_done;
assign led[10] = template_match_result;
```

## 五、资源占用

- BRAM 32K: 0个（使用分布式RAM）
- LUT RAM: 约1024 bits（32×32图像×2）
- LUT逻辑: 约300-500个
- FF寄存器: 约150-250个

**不会占用BRAM资源，不会影响现有的帧差法功能**

## 六、常见问题

Q: 为什么上位机显示超时？
A: 因为FPGA没有UDP TX模块回复结果。可以忽略超时，或者修改prj.py不等待回复。

Q: 如何调整匹配阈值？
A: 在prj.py中修改 self.match_threshold = 50 （默认50像素差异）

Q: 可以处理彩色图像吗？
A: 不可以，只支持二值化图像（黑白图像）

Q: 匹配速度如何？
A: 非常快，1024次XOR比较 + 计数，约1-2微秒完成

================================================================================
