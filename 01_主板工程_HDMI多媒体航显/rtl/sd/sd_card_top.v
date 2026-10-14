`timescale 1ns / 1ps
//=============================================================================
// sd_card_top.v - TF card SPI wrapper for the multimedia player
//=============================================================================
module sd_card_top #(
    parameter [15:0] SPI_LOW_SPEED_DIV  = 16'd248,
    parameter [15:0] SPI_HIGH_SPEED_DIV = 16'd4,
    parameter [23:0] POWER_WAIT_CYCLES  = 24'd5_000_000,
    parameter [3:0]  CMD0_MAX_TRIES     = 4'd4
)(
    input  wire        clk,
    input  wire        rst,                  // active high

    output wire        SD_nCS,
    output wire        SD_DCLK,
    output wire        SD_MOSI,
    input  wire        SD_MISO,

    output wire        sd_init_done,
    output wire        sd_init_error,
    output wire [2:0]  init_fail_code,
    input  wire        sd_sec_read,
    input  wire [31:0] sd_sec_read_addr,
    output wire [7:0]  sd_sec_read_data,
    output wire        sd_sec_read_data_valid,
    output wire        sd_sec_read_end,
    output wire        sd_sec_read_error,
    output wire [3:0]  sd_sec_read_error_code,

    output wire [4:0]  dbg_state,
    output wire [4:0]  dbg_fault_state,
    output wire        dbg_error_seen
);

    wire [15:0] spi_clk_div;
    wire        cmd_req;
    wire        cmd_req_ack;
    wire        cmd_req_error;
    wire [47:0] cmd;
    wire [7:0]  cmd_r1;
    wire [15:0] cmd_data_len;

    wire        block_read_req;
    wire        block_read_valid;
    wire [7:0]  block_read_data;
    wire        block_read_req_ack;

    wire        nCS_ctrl;
    wire        spi_wr_req;
    wire        spi_wr_ack;
    wire [7:0]  spi_data_in;
    wire [7:0]  spi_data_out;
    wire [15:0] clk_div;

    sd_card_sec_read_write #(
        .SPI_LOW_SPEED_DIV  (SPI_LOW_SPEED_DIV),
        .SPI_HIGH_SPEED_DIV (SPI_HIGH_SPEED_DIV),
        .POWER_WAIT_CYCLES  (POWER_WAIT_CYCLES),
        .CMD0_MAX_TRIES     (CMD0_MAX_TRIES)
    ) u_sec_rw (
        .clk                   (clk),
        .rst                   (rst),
        .sd_init_done          (sd_init_done),
        .sd_init_error         (sd_init_error),
        .init_fail_code        (init_fail_code),
        .sd_sec_read           (sd_sec_read),
        .sd_sec_read_addr      (sd_sec_read_addr),
        .sd_sec_read_data      (sd_sec_read_data),
        .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end       (sd_sec_read_end),
        .sd_sec_read_error     (sd_sec_read_error),
        .sd_sec_read_error_code(sd_sec_read_error_code),
        .spi_clk_div           (spi_clk_div),
        .cmd_req               (cmd_req),
        .cmd_req_ack           (cmd_req_ack),
        .cmd_req_error         (cmd_req_error),
        .cmd                   (cmd),
        .cmd_r1                (cmd_r1),
        .cmd_data_len          (cmd_data_len),
        .block_read_req        (block_read_req),
        .block_read_valid      (block_read_valid),
        .block_read_data       (block_read_data),
        .block_read_req_ack    (block_read_req_ack),
        .dbg_state             (dbg_state),
        .dbg_fault_state       (dbg_fault_state),
        .dbg_error_seen        (dbg_error_seen)
    );

    sd_card_cmd u_cmd (
        .sys_clk               (clk),
        .rst                   (rst),
        .spi_clk_div           (spi_clk_div),
        .cmd_req               (cmd_req),
        .cmd_req_ack           (cmd_req_ack),
        .cmd_req_error         (cmd_req_error),
        .cmd                   (cmd),
        .cmd_r1                (cmd_r1),
        .cmd_data_len          (cmd_data_len),
        .block_read_req        (block_read_req),
        .block_read_valid      (block_read_valid),
        .block_read_data       (block_read_data),
        .block_read_req_ack    (block_read_req_ack),
        .block_write_req       (1'b0),
        .block_write_data      (8'h00),
        .block_write_data_rd   (),
        .block_write_req_ack   (),
        .nCS_ctrl              (nCS_ctrl),
        .clk_div               (clk_div),
        .spi_wr_req            (spi_wr_req),
        .spi_wr_ack            (spi_wr_ack),
        .spi_data_in           (spi_data_in),
        .spi_data_out          (spi_data_out)
    );

    spi_master u_spi (
        .sys_clk  (clk),
        .rst      (rst),
        .nCS      (SD_nCS),
        .DCLK     (SD_DCLK),
        .MOSI     (SD_MOSI),
        .MISO     (SD_MISO),
        .CPOL     (1'b1),
        .CPHA     (1'b1),
        .nCS_ctrl (nCS_ctrl),
        .clk_div  (clk_div),
        .wr_req   (spi_wr_req),
        .wr_ack   (spi_wr_ack),
        .data_in  (spi_data_in),
        .data_out (spi_data_out)
    );

endmodule
