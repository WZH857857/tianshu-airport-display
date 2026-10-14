`timescale 1ns / 1ps
//=============================================================================
// i2c_rtc.v - small 100 kHz DS1307 reader
//
// The HDMI board only needs the current hour, minute and second.  This block
// reads registers 0x00..0x02 after each start_read pulse and returns the
// values as BCD.  SCL and SDA are open-drain: 0 drives low, 1 releases the
// line to the external 3.3 V pull-up.
//=============================================================================
module i2c_rtc (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start_read,
    output reg        busy,
    output reg        rtc_valid,
    output reg  [7:0] sec_bcd,
    output reg  [7:0] min_bcd,
    output reg  [7:0] hour_bcd,
    output reg        rtc_scl,
    inout  wire       rtc_sda
);

    // 50 MHz / (4 * 125) = 100 kHz.
    localparam [7:0] QUARTER_DIV = 8'd124;

    localparam [4:0] ST_IDLE     = 5'd0;
    localparam [4:0] ST_START    = 5'd1;
    localparam [4:0] ST_WRITE    = 5'd2;
    localparam [4:0] ST_ACK      = 5'd3;
    localparam [4:0] ST_RESTART  = 5'd4;
    localparam [4:0] ST_READ     = 5'd5;
    localparam [4:0] ST_READ_ACK = 5'd6;
    localparam [4:0] ST_STOP     = 5'd7;
    localparam [4:0] ST_ERROR    = 5'd8;

    localparam [1:0] CTX_ADDR_W  = 2'd0;
    localparam [1:0] CTX_REG     = 2'd1;
    localparam [1:0] CTX_ADDR_R  = 2'd2;

    reg [4:0] state;
    reg [1:0] phase;
    reg [7:0] div_cnt;
    reg [2:0] bit_cnt;
    reg [7:0] tx_byte;
    reg [7:0] rx_shift;
    reg [1:0] write_ctx;
    reg [1:0] ack_ctx;
    reg [1:0] read_idx;
    reg       start_pending;

    reg  scl_low;
    reg  sda_low;
    wire sda_in = rtc_sda;

    always @* begin
        rtc_scl = scl_low ? 1'b0 : 1'bz;
    end

    assign rtc_sda = sda_low ? 1'b0 : 1'bz;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            phase         <= 2'd0;
            div_cnt       <= 8'd0;
            bit_cnt       <= 3'd0;
            tx_byte       <= 8'h00;
            rx_shift      <= 8'h00;
            write_ctx     <= CTX_ADDR_W;
            ack_ctx       <= CTX_ADDR_W;
            read_idx      <= 2'd0;
            start_pending <= 1'b0;
            scl_low       <= 1'b0;
            sda_low       <= 1'b0;
            busy          <= 1'b0;
            rtc_valid     <= 1'b0;
            sec_bcd       <= 8'h00;
            min_bcd       <= 8'h00;
            hour_bcd      <= 8'h00;
        end else begin
            rtc_valid <= 1'b0;

            if (start_read)
                start_pending <= 1'b1;

            if (div_cnt != QUARTER_DIV) begin
                div_cnt <= div_cnt + 1'b1;
            end else begin
                div_cnt <= 8'd0;

                case (state)
                    ST_IDLE: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        busy    <= 1'b0;
                        if (start_pending) begin
                            busy          <= 1'b1;
                            start_pending <= 1'b0;
                            state         <= ST_START;
                            phase         <= 2'd0;
                        end
                    end

                    ST_START: begin
                        case (phase)
                            2'd0: begin scl_low <= 1'b0; sda_low <= 1'b0; end
                            2'd1: begin scl_low <= 1'b0; sda_low <= 1'b1; end
                            2'd2: begin scl_low <= 1'b1; sda_low <= 1'b1; end
                            default: begin
                                state     <= ST_WRITE;
                                write_ctx <= CTX_ADDR_W;
                                tx_byte   <= 8'hD0;
                                bit_cnt   <= 3'd0;
                                scl_low   <= 1'b1;
                                sda_low   <= 1'b1;
                            end
                        endcase
                        phase <= phase + 1'b1;
                    end

                    ST_WRITE: begin
                        case (phase)
                            2'd0: begin
                                scl_low <= 1'b1;
                                case (write_ctx)
                                    CTX_ADDR_W: sda_low <= ~tx_byte[7];
                                    CTX_REG:    sda_low <= ~tx_byte[7];
                                    CTX_ADDR_R: sda_low <= ~tx_byte[7];
                                    default:    sda_low <= 1'b1;
                                endcase
                            end
                            2'd1: begin
                                scl_low <= 1'b0;
                            end
                            2'd2: begin
                                scl_low <= 1'b0;
                            end
                            default: begin
                                scl_low <= 1'b1;
                                tx_byte <= {tx_byte[6:0], 1'b0};
                                if (bit_cnt == 3'd7) begin
                                    state   <= ST_ACK;
                                    ack_ctx <= write_ctx;
                                    bit_cnt <= 3'd0;
                                    sda_low <= 1'b0;
                                end else begin
                                    bit_cnt <= bit_cnt + 1'b1;
                                end
                            end
                        endcase
                        phase <= phase + 1'b1;
                    end

                    ST_ACK: begin
                        case (phase)
                            2'd0: begin
                                scl_low <= 1'b1;
                                sda_low <= 1'b0;
                            end
                            2'd1: begin
                                scl_low <= 1'b0;
                            end
                            2'd2: begin
                                scl_low <= 1'b0;
                            end
                            default: begin
                                scl_low <= 1'b1;

                                if (sda_in) begin
                                    state <= ST_STOP;
                                    phase <= 2'd0;
                                end else begin
                                    case (ack_ctx)
                                        CTX_ADDR_W: begin
                                            write_ctx <= CTX_REG;
                                            tx_byte   <= 8'h00;
                                            bit_cnt   <= 3'd0;
                                            state     <= ST_WRITE;
                                            phase     <= 2'd0;
                                        end
                                        CTX_REG: begin
                                            state <= ST_RESTART;
                                            phase <= 2'd0;
                                        end
                                        CTX_ADDR_R: begin
                                            state    <= ST_READ;
                                            read_idx <= 2'd0;
                                            bit_cnt  <= 3'd0;
                                            rx_shift <= 8'h00;
                                            phase    <= 2'd0;
                                        end
                                        default: begin
                                            state <= ST_STOP;
                                            phase <= 2'd0;
                                        end
                                    endcase
                                end
                            end
                        endcase
                        phase <= phase + 1'b1;
                    end

                    ST_RESTART: begin
                        case (phase)
                            2'd0: begin scl_low <= 1'b1; sda_low <= 1'b0; end
                            2'd1: begin scl_low <= 1'b0; sda_low <= 1'b0; end
                            2'd2: begin scl_low <= 1'b0; sda_low <= 1'b1; end
                            default: begin
                                state     <= ST_WRITE;
                                write_ctx <= CTX_ADDR_R;
                                tx_byte   <= 8'hD1;
                                bit_cnt   <= 3'd0;
                                scl_low   <= 1'b1;
                                sda_low   <= 1'b1;
                            end
                        endcase
                        phase <= phase + 1'b1;
                    end

                    ST_READ: begin
                        case (phase)
                            2'd0: begin
                                scl_low <= 1'b1;
                                sda_low <= 1'b0;
                            end
                            2'd1: begin
                                scl_low <= 1'b0;
                            end
                            2'd2: begin
                                scl_low <= 1'b0;
                                rx_shift <= {rx_shift[6:0], sda_in};
                            end
                            default: begin
                                scl_low <= 1'b1;
                                if (bit_cnt == 3'd7) begin
                                    state   <= ST_READ_ACK;
                                    bit_cnt <= 3'd0;
                                end else begin
                                    bit_cnt <= bit_cnt + 1'b1;
                                end
                            end
                        endcase
                        phase <= phase + 1'b1;
                    end

                    ST_READ_ACK: begin
                        case (phase)
                            2'd0: begin
                                scl_low <= 1'b1;
                                sda_low <= (read_idx == 2'd2) ? 1'b0 : 1'b1;
                            end
                            2'd1: begin
                                scl_low <= 1'b0;
                            end
                            2'd2: begin
                                scl_low <= 1'b0;
                            end
                            default: begin
                                scl_low <= 1'b1;

                                case (read_idx)
                                    2'd0: begin
                                        sec_bcd  <= rx_shift;
                                        read_idx <= 2'd1;
                                        rx_shift <= 8'h00;
                                        state    <= ST_READ;
                                    end
                                    2'd1: begin
                                        min_bcd  <= rx_shift;
                                        read_idx <= 2'd2;
                                        rx_shift <= 8'h00;
                                        state    <= ST_READ;
                                    end
                                    default: begin
                                        hour_bcd  <= rx_shift;
                                        rtc_valid <= 1'b1;
                                        state     <= ST_STOP;
                                    end
                                endcase
                                phase <= 2'd0;
                            end
                        endcase
                        if (phase != 2'd3)
                            phase <= phase + 1'b1;
                    end

                    ST_STOP: begin
                        case (phase)
                            2'd0: begin scl_low <= 1'b1; sda_low <= 1'b1; end
                            2'd1: begin scl_low <= 1'b0; sda_low <= 1'b1; end
                            2'd2: begin scl_low <= 1'b0; sda_low <= 1'b0; end
                            default: begin
                                scl_low <= 1'b0;
                                sda_low <= 1'b0;
                                busy    <= 1'b0;
                                state   <= ST_IDLE;
                                phase   <= 2'd0;
                            end
                        endcase
                        if (phase != 2'd3)
                            phase <= phase + 1'b1;
                    end

                    ST_ERROR: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        busy    <= 1'b0;
                        state   <= ST_IDLE;
                    end

                    default: begin
                        state <= ST_IDLE;
                    end
                endcase
            end
        end
    end

endmodule
