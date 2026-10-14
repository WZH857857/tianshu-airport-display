`timescale 1ns / 1ps
//=============================================================================
// sector_reader.v - one 512 byte sector buffer in front of the SD controller
//
// The SD controller is fast enough that the rest of the design can consume a
// whole sector after it has arrived. Keeping the sector in a small RAM makes
// the FAT lookup and BMP parser independent of SPI byte timing.
//=============================================================================
module sector_reader #(
    parameter [23:0] TIMEOUT_CYCLES = 24'hFF_FFFF
)(
    input  wire        clk,
    input  wire        rst_n,

    // consumer side
    input  wire        start,
    input  wire [31:0] lba,
    output reg         busy,
    output reg         ready,
    output reg         error,
    output reg  [3:0]  error_code,
    input  wire [8:0]  raddr0,
    input  wire [8:0]  raddr1,
    input  wire [8:0]  raddr2,
    input  wire [8:0]  raddr3,
    output wire [7:0]  rdata0,
    output wire [7:0]  rdata1,
    output wire [7:0]  rdata2,
    output wire [7:0]  rdata3,
    input  wire        release_req,

    // SD controller side
    input  wire        sd_init_done,
    output reg         sd_sec_read,
    output reg  [31:0] sd_sec_read_addr,
    input  wire [7:0]  sd_sec_read_data,
    input  wire        sd_sec_read_data_valid,
    input  wire        sd_sec_read_end,
    input  wire        sd_sec_read_error,
    input  wire [3:0]  sd_sec_read_error_code
);

    reg [7:0]  mem [0:511];
    reg [9:0]  byte_cnt;
    reg [23:0] timeout_cnt;

    assign rdata0 = mem[raddr0];
    assign rdata1 = mem[raddr1];
    assign rdata2 = mem[raddr2];
    assign rdata3 = mem[raddr3];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy                <= 1'b0;
            ready               <= 1'b0;
            error               <= 1'b0;
            error_code          <= 4'd0;
            sd_sec_read         <= 1'b0;
            sd_sec_read_addr    <= 32'd0;
            byte_cnt            <= 10'd0;
            timeout_cnt         <= 24'd0;
        end else begin
            if (release_req) begin
                ready       <= 1'b0;
                error       <= 1'b0;
                timeout_cnt <= 24'd0;
            end

            if (!busy && !ready && start && sd_init_done) begin
                busy             <= 1'b1;
                ready            <= 1'b0;
                error            <= 1'b0;
                error_code       <= 4'd0;
                sd_sec_read      <= 1'b1;
                sd_sec_read_addr <= lba;
                byte_cnt         <= 10'd0;
                timeout_cnt      <= 24'd0;
            end

            if (busy) begin
                if (sd_sec_read_data_valid && (byte_cnt < 10'd512))
                    mem[byte_cnt[8:0]] <= sd_sec_read_data;

                if (sd_sec_read_data_valid && (byte_cnt < 10'd513))
                    byte_cnt <= byte_cnt + 10'd1;

                if (sd_sec_read_end || sd_sec_read_error) begin
                    busy         <= 1'b0;
                    sd_sec_read  <= 1'b0;
                    timeout_cnt  <= 24'd0;
                    if (!sd_sec_read_error && (byte_cnt == 10'd512))
                        ready <= 1'b1;
                    else begin
                        error <= 1'b1;
                        error_code <= sd_sec_read_error ?
                                      sd_sec_read_error_code : 4'd10;
                    end
                end else if (timeout_cnt >= TIMEOUT_CYCLES) begin
                    busy        <= 1'b0;
                    sd_sec_read <= 1'b0;
                    error       <= 1'b1;
                    error_code  <= 4'd4;
                    timeout_cnt <= 24'd0;
                end else begin
                    timeout_cnt <= timeout_cnt + 24'd1;
                end
            end
        end
    end

endmodule
