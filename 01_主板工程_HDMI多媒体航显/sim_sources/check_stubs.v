// Elaboration stubs for the blocks iverilog cannot read.
//
//   clk_wize, PLL_HDMI_AUDIO  TD generated PLL wrappers
//   EG_LOGIC_BUFG, EG_PHY_PLL  the primitives those wrappers instantiate
//   EG_PHY_SDRAM_2M_32        TD SDRAM PHY macro
//   sdr_as_ram                the encrypted SDRAM controller, see
//                             sim/sdr_as_ram_stub.v for its behaviour
//   EG_LOGIC_BRAM             the scaler line buffers, dual port RAM with an
//                             output register on port B, which is the mode
//                             rtl/storage/bmp_scaler.v asks for
//   hdmi_1_4b_transmitter_core_wrapper
//                             the vendor core is protected with an Anlogic
//                             pragma protect block, only TD can decrypt it
//
// They are empty boxes, so this file is only good for a syntax and
// connectivity check of the surrounding RTL. It is not part of the TD build.
`timescale 1ns / 1ps

module clk_wize (
    input  wire refclk,
    input  wire reset,
    output wire clk0_out,
    output wire clk1_out
);
    assign clk0_out = refclk;
    assign clk1_out = refclk;
endmodule

// The PLL primitive the two al_ip wrappers above (and al_ip/sys_pll.v) use.
// Only the port list matters here, the clock comes straight from refclk so the
// whole design elaborates and can be inspected.
module EG_LOGIC_BUFG (
    input  wire i,
    output wire o
);
    assign o = i;
endmodule

module EG_PHY_PLL #(
    parameter DPHASE_SOURCE    = "DISABLE",
    parameter DYNCFG           = "DISABLE",
    parameter FIN              = "50.000000",
    parameter FEEDBK_MODE      = "NORMAL",
    parameter FEEDBK_PATH      = "CLKC0_EXT",
    parameter STDBY_ENABLE     = "DISABLE",
    parameter PLLRST_ENA       = "DISABLE",
    parameter SYNC_ENABLE      = "DISABLE",
    parameter GMC_GAIN         = 2,
    parameter ICP_CURRENT      = 9,
    parameter KVCO             = 2,
    parameter LPF_CAPACITOR    = 1,
    parameter LPF_RESISTOR     = 8,
    parameter REFCLK_DIV       = 1,
    parameter FBCLK_DIV        = 1,
    parameter CLKC0_ENABLE     = "ENABLE",
    parameter CLKC0_DIV        = 1,
    parameter CLKC0_CPHASE     = 0,
    parameter CLKC0_FPHASE     = 0,
    parameter CLKC1_ENABLE     = "DISABLE",
    parameter CLKC1_DIV        = 1,
    parameter CLKC1_CPHASE     = 0,
    parameter CLKC1_FPHASE     = 0,
    parameter CLKC2_ENABLE     = "DISABLE",
    parameter CLKC2_DIV        = 1,
    parameter CLKC2_CPHASE     = 0,
    parameter CLKC2_FPHASE     = 0
)(
    input  wire       refclk,
    input  wire       reset,
    input  wire       stdby,
    output wire       extlock,
    input  wire       load_reg,
    input  wire       psclk,
    input  wire       psdown,
    input  wire       psstep,
    input  wire [2:0] psclksel,
    output wire       psdone,
    input  wire       dclk,
    input  wire       dcs,
    input  wire       dwe,
    input  wire [7:0] di,
    input  wire [5:0] daddr,
    output wire [7:0] do,
    input  wire       fbclk,
    output wire [4:0] clkc
);
    assign extlock = 1'b1;
    assign psdone  = 1'b1;
    assign do      = 8'd0;
    assign clkc    = {refclk, refclk, refclk, refclk, refclk};
endmodule

module PLL_HDMI_AUDIO (
    input  wire refclk,
    input  wire reset,
    output wire extlock,
    output wire clk0_out,
    output wire clk1_out,
    output wire clk2_out
);
    assign extlock  = 1'b1;
    assign clk0_out = refclk;
    assign clk1_out = refclk;
    assign clk2_out = refclk;
endmodule

module EG_PHY_SDRAM_2M_32 (
    input  wire        clk,
    input  wire        ras_n,
    input  wire        cas_n,
    input  wire        we_n,
    input  wire [10:0] addr,
    input  wire [1:0]  ba,
    inout  wire [31:0] dq,
    input  wire        cs_n,
    input  wire        dm0,
    input  wire        dm1,
    input  wire        dm2,
    input  wire        dm3,
    input  wire        cke
);
endmodule

module EG_LOGIC_ODDR (
    input  wire clk,
    input  wire rst,
    input  wire d0,
    input  wire d1,
    output wire q
);
    assign q = d0;
endmodule

// Scaler line buffer.  Port A writes, port B reads through an output register,
// both sides address the same memory.  This is the board macro's behaviour for
// the parameter set bmp_scaler uses (MODE "DP", REGMODE_A "NOREG",
// REGMODE_B "OUTREG"), so the scaled picture path can also be simulated.
module EG_LOGIC_BRAM #(
    parameter integer DATA_WIDTH_A = 24,
    parameter integer DATA_WIDTH_B = 24,
    parameter integer ADDR_WIDTH_A = 10,
    parameter integer ADDR_WIDTH_B = 10,
    parameter integer DATA_DEPTH_A = 1024,
    parameter integer DATA_DEPTH_B = 1024,
    parameter integer BYTE_ENABLE  = 0,
    parameter integer BYTE_A       = 1,
    parameter integer BYTE_B       = 1,
    parameter         MODE         = "DP",
    parameter         REGMODE_A    = "NOREG",
    parameter         REGMODE_B    = "OUTREG",
    parameter         WRITEMODE_A  = "NORMAL",
    parameter         WRITEMODE_B  = "NORMAL",
    parameter         RESETMODE    = "SYNC",
    parameter         DEBUGGABLE   = "NO",
    parameter         PACKABLE     = "NO",
    parameter         FORCE_KEEP   = "OFF",
    parameter         INIT_FILE    = "NONE",
    parameter         FILL_ALL     = "NONE",
    parameter         IMPLEMENT    = "9K"
)(
    output wire [DATA_WIDTH_A-1:0] doa,
    output wire [DATA_WIDTH_B-1:0] dob,
    input  wire [DATA_WIDTH_A-1:0] dia,
    input  wire [DATA_WIDTH_B-1:0] dib,
    input  wire [ADDR_WIDTH_A-1:0] addra,
    input  wire [ADDR_WIDTH_B-1:0] addrb,
    input  wire        cea,
    input  wire        ocea,
    input  wire        clka,
    input  wire        wea,
    input  wire        rsta,
    input  wire        ceb,
    input  wire        oceb,
    input  wire        clkb,
    input  wire        web,
    input  wire        rstb,
    input  wire        bea,
    input  wire        beb
);
    reg [DATA_WIDTH_A-1:0] mem [0:(1 << ADDR_WIDTH_A)-1];
    reg [DATA_WIDTH_A-1:0] doa_r;
    reg [DATA_WIDTH_B-1:0] dob_r;

    always @(posedge clka) begin
        if (cea && wea)
            mem[addra] <= dia;
        if (ocea)
            doa_r <= mem[addra];
    end

    always @(posedge clkb) begin
        if (ceb && oceb)
            dob_r <= mem[addrb];
    end

    assign doa = doa_r;
    assign dob = dob_r;
endmodule

module hdmi_1_4b_transmitter_core_wrapper #(
    parameter DEVICE            = "EG",
    parameter HTOTAL            = 2200,
    parameter HSA               = 44,
    parameter HFP               = 88,
    parameter HBP               = 148,
    parameter HACTIVE           = 1920,
    parameter VTOTAL            = 1125,
    parameter VSA               = 5,
    parameter VFP               = 4,
    parameter VBP               = 36,
    parameter VACTIVE           = 1080,
    parameter VIDEO_TPG         = "Enable",
    parameter VIDEO_FORMAT      = "RGB",
    parameter VIDEO_VIC         = 16,
    parameter AUDIO_SAMPLE_RATE = "48K",
    parameter IIC_SCL_DIV       = 125
)(
    input  wire       I_pixel_clk,
    input  wire       I_rst,
    input  wire       I_edid_read_trig,
    output wire       O_edid_read_valid,
    output wire [7:0] O_edid_read_data,
    input  wire       I_axis_s_user,
    input  wire       I_axis_s_valid,
    input  wire       I_axis_s_last,
    input  wire [23:0] I_axis_s_data,
    output wire       O_axis_s_ready,
    input  wire       I_audio_valid,
    input  wire [23:0] I_audio_left_data,
    input  wire [23:0] I_audio_right_data,
    input  wire       I_acr_valid,
    input  wire [19:0] I_acr_cts,
    input  wire [19:0] I_acr_n,
    output wire       O_video_locked,
    output wire       O_ddc_scl,
    inout  wire       IO_ddc_sda,
    output wire [9:0] O_ch0_tmds_data,
    output wire [9:0] O_ch1_tmds_data,
    output wire [9:0] O_ch2_tmds_data,
    output wire [9:0] O_clk_tmds_data
);
    assign O_edid_read_valid = 1'b0;
    assign O_edid_read_data  = 8'd0;
    assign O_axis_s_ready    = 1'b1;
    assign O_video_locked    = 1'b1;
    assign O_ddc_scl         = 1'b1;
    assign O_ch0_tmds_data   = 10'd0;
    assign O_ch1_tmds_data   = 10'd0;
    assign O_ch2_tmds_data   = 10'd0;
    assign O_clk_tmds_data   = 10'd0;
endmodule
