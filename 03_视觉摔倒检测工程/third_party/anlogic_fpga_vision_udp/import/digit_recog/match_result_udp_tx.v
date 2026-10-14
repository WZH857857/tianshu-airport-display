//****************************************Copyright (c)***********************************//
// File name:           match_result_udp_tx
// Last modified Date:  2025/01/05
// Last Version:        V1.0
// Descriptions:        实时发送模板匹配结果通过UDP
//----------------------------------------------------------------------------------------
// Created by:          Claude
// Created date:        2025/01/05
// Version:             V1.0
// Descriptions:        Send template match result via UDP in real-time
//                      - Triggered by match_result_valid (every frame)
//                      - Send 4 bytes: [match_result(1bit)][hamming_distance(10bit)][reserved(21bit)]
//                      - Command format: 0xF0 (match) or 0xF1 (not match) + distance (2 bytes)
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module match_result_udp_tx (
    input wire          clk,                    // UDP clock (udp_clk)
    input wire          rst_n,                  // Reset (active low)

    // Match result interface (from template matching module)
    input wire          match_result,           // Match result (1=match, 0=not match)
    input wire          match_result_valid,     // Result valid (pulse, once per frame)
    input wire [9:0]    hamming_distance,       // Hamming distance (0-1023)

    // UDP TX interface
    input wire          udp_tx_ready,           // UDP ready to send
    output reg          udp_tx_request,         // Request to send
    output reg          udp_tx_data_valid,      // Data valid
    output reg [7:0]    udp_tx_data,            // Data byte
    output reg [15:0]   udp_tx_length,          // Packet length
    input wire          udp_tx_ack              // Send complete acknowledge
);

//=============================================================================
// Parameter definitions
//=============================================================================
localparam PACKET_LENGTH = 16'd4;               // 4 bytes total

// State machine
localparam IDLE     = 3'd0;
localparam WAIT_RDY = 3'd1;
localparam SEND_B0  = 3'd2;
localparam SEND_B1  = 3'd3;
localparam SEND_B2  = 3'd4;
localparam SEND_B3  = 3'd5;
localparam WAIT_ACK = 3'd6;

//=============================================================================
// Signal definitions
//=============================================================================
reg [2:0]  state;
reg        match_result_latch;
reg [9:0]  hamming_distance_latch;
reg        match_result_valid_d1;
wire       match_result_valid_rise;

//=============================================================================
// Edge detection for match_result_valid
//=============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        match_result_valid_d1 <= 1'b0;
    else
        match_result_valid_d1 <= match_result_valid;
end

assign match_result_valid_rise = match_result_valid & ~match_result_valid_d1;

//=============================================================================
// State machine for UDP TX
//=============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state                  <= IDLE;
        udp_tx_request         <= 1'b0;
        udp_tx_data_valid      <= 1'b0;
        udp_tx_data            <= 8'd0;
        udp_tx_length          <= 16'd0;
        match_result_latch     <= 1'b0;
        hamming_distance_latch <= 10'd0;
    end else begin
        case (state)
            IDLE: begin
                udp_tx_request    <= 1'b0;
                udp_tx_data_valid <= 1'b0;

                // Trigger on match_result_valid rising edge
                if (match_result_valid_rise) begin
                    match_result_latch     <= match_result;
                    hamming_distance_latch <= hamming_distance;
                    state                  <= WAIT_RDY;
                end
            end

            WAIT_RDY: begin
                if (udp_tx_ready) begin
                    udp_tx_request <= 1'b1;
                    udp_tx_length  <= PACKET_LENGTH;
                    state          <= SEND_B0;
                end
            end

            SEND_B0: begin
                udp_tx_request    <= 1'b0;
                udp_tx_data_valid <= 1'b1;
                // Byte 0: 0xF0 (match) or 0xF1 (not match)
                udp_tx_data       <= match_result_latch ? 8'hF0 : 8'hF1;
                state             <= SEND_B1;
            end

            SEND_B1: begin
                udp_tx_data_valid <= 1'b1;
                // Byte 1: Hamming distance high byte (bits 9:8)
                udp_tx_data       <= {6'd0, hamming_distance_latch[9:8]};
                state             <= SEND_B2;
            end

            SEND_B2: begin
                udp_tx_data_valid <= 1'b1;
                // Byte 2: Hamming distance low byte (bits 7:0)
                udp_tx_data       <= hamming_distance_latch[7:0];
                state             <= SEND_B3;
            end

            SEND_B3: begin
                udp_tx_data_valid <= 1'b1;
                // Byte 3: Reserved (0x00)
                udp_tx_data       <= 8'h00;
                state             <= WAIT_ACK;
            end

            WAIT_ACK: begin
                udp_tx_data_valid <= 1'b0;
                if (udp_tx_ack) begin
                    state <= IDLE;
                end
            end

            default: state <= IDLE;
        endcase
    end
end

endmodule
