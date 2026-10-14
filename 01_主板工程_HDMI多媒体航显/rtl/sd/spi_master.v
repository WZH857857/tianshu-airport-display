`timescale 1ns / 1ps
//=============================================================================
// spi_master.v - byte oriented SPI master for the TF card
//
// Derived from the Anlogic TD 6.2.1 demo 18_sd_card_audio. It is kept as a
// byte engine so the SD command layer can own command framing and response
// timing.
//=============================================================================
module spi_master (
    input  wire        sys_clk,
    input  wire        rst,          // active high for this vendor-derived block

    output wire        nCS,
    output wire        DCLK,
    output wire        MOSI,
    input  wire        MISO,

    input  wire        CPOL,
    input  wire        CPHA,
    input  wire        nCS_ctrl,
    input  wire [15:0] clk_div,

    input  wire        wr_req,
    output wire        wr_ack,
    input  wire [7:0]  data_in,
    output wire [7:0]  data_out
);

    localparam IDLE            = 3'd0;
    localparam DCLK_EDGE       = 3'd1;
    localparam DCLK_IDLE       = 3'd2;
    localparam ACK             = 3'd3;
    localparam LAST_HALF_CYCLE = 3'd4;
    localparam ACK_WAIT        = 3'd5;

    reg        DCLK_reg;
    reg [7:0]  MOSI_shift;
    reg [7:0]  MISO_shift;
    reg [2:0]  state;
    reg [2:0]  next_state;
    reg [15:0] clk_cnt;
    reg [4:0]  clk_edge_cnt;

    assign MOSI     = MOSI_shift[7];
    assign DCLK     = DCLK_reg;
    assign data_out = MISO_shift;
    assign wr_ack   = (state == ACK);
    assign nCS      = nCS_ctrl;

    always @(posedge sys_clk or posedge rst) begin
        if (rst)
            state <= IDLE;
        else
            state <= next_state;
    end

    always @* begin
        case (state)
            IDLE:
                next_state = wr_req ? DCLK_IDLE : IDLE;

            DCLK_IDLE:
                next_state = (clk_cnt == clk_div) ? DCLK_EDGE : DCLK_IDLE;

            DCLK_EDGE:
                next_state = (clk_edge_cnt == 5'd15) ? LAST_HALF_CYCLE : DCLK_IDLE;

            LAST_HALF_CYCLE:
                next_state = (clk_cnt == clk_div) ? ACK : LAST_HALF_CYCLE;

            ACK:
                next_state = ACK_WAIT;

            ACK_WAIT:
                next_state = IDLE;

            default:
                next_state = IDLE;
        endcase
    end

    always @(posedge sys_clk or posedge rst) begin
        if (rst)
            DCLK_reg <= 1'b0;
        else if (state == IDLE)
            DCLK_reg <= CPOL;
        else if (state == DCLK_EDGE)
            DCLK_reg <= ~DCLK_reg;
    end

    always @(posedge sys_clk or posedge rst) begin
        if (rst)
            clk_cnt <= 16'd0;
        else if ((state == DCLK_IDLE) || (state == LAST_HALF_CYCLE))
            clk_cnt <= clk_cnt + 16'd1;
        else
            clk_cnt <= 16'd0;
    end

    always @(posedge sys_clk or posedge rst) begin
        if (rst)
            clk_edge_cnt <= 5'd0;
        else if (state == DCLK_EDGE)
            clk_edge_cnt <= clk_edge_cnt + 5'd1;
        else if (state == IDLE)
            clk_edge_cnt <= 5'd0;
    end

    always @(posedge sys_clk or posedge rst) begin
        if (rst)
            MOSI_shift <= 8'd0;
        else if ((state == IDLE) && wr_req)
            MOSI_shift <= data_in;
        else if (state == DCLK_EDGE) begin
            if ((CPHA == 1'b0) && (clk_edge_cnt[0] == 1'b1))
                MOSI_shift <= {MOSI_shift[6:0], MOSI_shift[7]};
            else if ((CPHA == 1'b1) && (clk_edge_cnt != 5'd0) &&
                     (clk_edge_cnt[0] == 1'b0))
                MOSI_shift <= {MOSI_shift[6:0], MOSI_shift[7]};
        end
    end

    always @(posedge sys_clk or posedge rst) begin
        if (rst)
            MISO_shift <= 8'd0;
        else if ((state == IDLE) && wr_req)
            MISO_shift <= 8'h00;
        else if (state == DCLK_EDGE) begin
            if ((CPHA == 1'b0) && (clk_edge_cnt[0] == 1'b0))
                MISO_shift <= {MISO_shift[6:0], MISO};
            else if ((CPHA == 1'b1) && (clk_edge_cnt[0] == 1'b1))
                MISO_shift <= {MISO_shift[6:0], MISO};
        end
    end

endmodule
