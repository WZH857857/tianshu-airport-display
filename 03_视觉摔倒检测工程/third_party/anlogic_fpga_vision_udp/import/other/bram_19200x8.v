`timescale 1ns / 1ps
//********************************************************************************
// Module: bram_19200x8
// Description: 单端口RAM，用于存储降采样后的前一帧灰度图（160×120）
//
// 容量：19200×8bit = 150kbit ≈ 18.75KB
// 实现：使用安路FPGA的EG_LOGIC_BRAM原语（32K BRAM）
//
// 配置：
//   - MODE: SP (单端口)
//   - REGMODE_A: OUTREG (输出寄存器，1周期读延迟)
//   - WRITEMODE_A: READ_BEFORE_WRITE (先读后写)
//   - IMPLEMENT: 32K (使用BRAM 32k，综合工具会自动使用多个)
//********************************************************************************

module bram_19200x8 (
    input   wire        clk,
    input   wire        we,           // 写使能
    input   wire [14:0] addr,         // 15位地址（支持0~32767，实际使用0~19199）
    input   wire [7:0]  din,          // 写数据
    output  wire [7:0]  dout          // 读数据（1周期延迟）
);

// 参数定义
parameter DATA_WIDTH_A = 8;
parameter ADDR_WIDTH_A = 15;
parameter DATA_DEPTH_A = 32768;      // 向上对齐到2^15

// 使用安路EG_LOGIC_BRAM原语（单端口模式）
EG_LOGIC_BRAM #(
    .DATA_WIDTH_A   (DATA_WIDTH_A),
    .DATA_WIDTH_B   (DATA_WIDTH_A),  // 单端口模式下B端口未使用
    .ADDR_WIDTH_A   (ADDR_WIDTH_A),
    .ADDR_WIDTH_B   (ADDR_WIDTH_A),
    .DATA_DEPTH_A   (DATA_DEPTH_A),
    .DATA_DEPTH_B   (DATA_DEPTH_A),
    .MODE           ("SP"),           // 单端口模式
    .REGMODE_A      ("OUTREG"),       // 输出寄存器（1周期读延迟）
    .REGMODE_B      ("NOREG"),        // B端口未使用
    .WRITEMODE_A    ("READ_BEFORE_WRITE"), // 先读后写（关键！）
    .WRITEMODE_B    ("NORMAL"),
    .RESETMODE      ("SYNC"),
    .IMPLEMENT      ("32K"),          // 使用BRAM 32k
    .INIT_FILE      ("NONE"),
    .FILL_ALL       ("NONE")
) inst_bram (
    // Port A: 读写端口
    .dia    (din),
    .addra  (addr),
    .cea    (1'b1),          // 时钟使能始终开启
    .ocea   (1'b1),          // 输出寄存器时钟使能
    .clka   (clk),
    .wea    (we),            // 写使能
    .bea    (1'b0),          // 字节使能（8bit不需要）
    .rsta   (1'b0),          // 不使用异步复位
    .doa    (dout),          // 读数据输出

    // Port B: 单端口模式下未使用
    .dib    ({DATA_WIDTH_A{1'b0}}),
    .addrb  ({ADDR_WIDTH_A{1'b0}}),
    .ceb    (1'b0),
    .oceb   (1'b0),
    .clkb   (clk),
    .web    (1'b0),
    .beb    (1'b0),
    .rstb   (1'b0),
    .dob    ()
);

endmodule
