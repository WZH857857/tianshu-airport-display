//===========================================================================
// frame_diff.v -- |cur - prev| > threshold against the previous frame
//
// The previous 320x180 luma frame lives in on-chip ERAM.  TD's usable ERAM
// is 76,800 byte-equivalent *entries* on this device (64 9K blocks x 600
// plus 16 32K blocks x 2,400), and the frame plus the motion mask need
// 57,600 + 7,200, so the frame store is split across both regions:
//
//   prev_ram_lo  38,400 x 8, hinted bram_32k    rows   0..119
//   prev_ram_hi  19,200 x 8, hinted bram_9k     rows 120..179
//
// Three rules come straight from TD's ERAM mapping and all three are load
// bearing here (each one was measured with a probe project):
//
//   1. The two arrays need one always block each.  Putting both in a single
//      always block makes TD try to map the whole group into one region and
//      then drop all of it into LUTs.
//   2. The 19,200-entry half must take a 15-bit address.  With a 16-bit port
//      TD sizes the footprint from the port width and refuses the array.
//   3. The 32K block cannot do read-before-write, so the read and the write
//      live in separate always blocks (simple dual port) and the write runs
//      one pixel behind the read: the write of word a happens on the cycle
//      that reads word a+1, and the two ports never share an address.
//
// The read data arrives on the clock after the address, so the comparison
// runs two clocks behind the camera stream:
//
//   cycle n   : addr = a, gray_in = pixel a   (read old a, queue write a)
//   cycle n+1 : prev_q = old content of a     (previous frame's pixel a)
//               gray_q = pixel a, de_q = 1     (write a, read a+1)
//   cycle n+2 : mask_out for pixel a
//
// The output order is unchanged: exactly one mask bit per input pixel.
//
// frame_cnt counts frame_start pulses, so the first frame is streamed while
// frame_cnt == 1 and the first comparison happens on frame_cnt == 2.  The
// mask is gated until then; otherwise the buffer's power-up contents (X in
// simulation, arbitrary bits in ERAM) would be emitted as motion on the
// very first frame and poison the downstream morphology counters.
//
// frame_start pulses once per frame during blanking; de_in marks pixels.
//===========================================================================
`timescale 1ns / 1ps

module frame_diff #(
parameter integer WIDTH       = 320,
    parameter integer HEIGHT      = 180,
    parameter integer DIFF_THRESH = 10
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       frame_start,
    input  wire       de_in,
    input  wire [7:0] gray_in,
    output reg        de_out,
    output reg        mask_out
);

localparam integer PIXELS = WIDTH * HEIGHT;
localparam integer AWIDTH = $clog2(PIXELS);
localparam integer LO_PIX = 38400;   // exactly 16 x 2,400 32K-block entries
localparam integer HI_PIX = PIXELS - LO_PIX;
localparam integer HI_AW  = $clog2(HI_PIX);   // 15 for 320x180

(* ram_style = "bram_32k" *) reg [7:0] prev_ram_lo [0:LO_PIX-1];
(* ram_style = "bram_9k"  *) reg [7:0] prev_ram_hi [0:HI_PIX-1];
reg [AWIDTH-1:0] addr;      // read address: the pixel being compared
reg [AWIDTH-1:0] waddr;     // write address: one pixel behind addr
reg [7:0]        wdata;
reg              wvalid;
reg [15:0]       frame_cnt;

reg [7:0] q_lo, q_hi;  // ERAM outputs, one clock behind addr
reg       sel_q;       // bank of the address that produced q_lo/q_hi
// Muxed combinationally so the comparison stays aligned with the pixel that
// asked for this address: q_lo/q_hi land on the same edge that gray_q does.
wire [7:0] prev_q = sel_q ? q_hi : q_lo;
reg [7:0] gray_q;      // current pixel, delayed to meet prev_q
reg       de_q;

wire [8:0] diff_q = (gray_q > prev_q) ? (gray_q - prev_q)
                                      : (prev_q - gray_q);

// The bank boundary is LO_PIX (38,400), not the power of two above it: the
// low array is sized to fill the 32K ERAM region exactly, so selecting the
// bank with addr[15] would send rows 102..119 to the wrong array and leave
// holes in the low bank.  The read side and the write side each need their
// own comparison because the two addresses are a pixel apart.
wire              r_hi    = (addr >= LO_PIX[AWIDTH-1:0]);
wire              w_hi    = (waddr >= LO_PIX[AWIDTH-1:0]);
wire [AWIDTH-1:0] addr_hi = addr  - LO_PIX[AWIDTH-1:0];
wire [AWIDTH-1:0] waddr_hi= waddr - LO_PIX[AWIDTH-1:0];

// 1. One always block per array (rule 1), 3. read and write apart (rule 3).
always @(posedge clk) begin
    q_lo <= prev_ram_lo[addr];
end
always @(posedge clk) begin
    if (wvalid && !w_hi) prev_ram_lo[waddr] <= wdata;
end
// 2. The high half takes a 15-bit address, not the 16-bit addr_hi.
always @(posedge clk) begin
    q_hi <= prev_ram_hi[addr_hi[HI_AW-1:0]];
end
always @(posedge clk) begin
    if (wvalid && w_hi) prev_ram_hi[waddr_hi[HI_AW-1:0]] <= wdata;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        addr      <= {AWIDTH{1'b0}};
        waddr     <= {AWIDTH{1'b0}};
        wdata     <= 8'd0;
        wvalid    <= 1'b0;
        frame_cnt <= 16'd0;
        sel_q     <= 1'b0;
        gray_q    <= 8'd0;
        de_q      <= 1'b0;
        de_out    <= 1'b0;
        mask_out  <= 1'b0;
    end else begin
        de_out <= 1'b0;
        sel_q  <= r_hi;
        gray_q <= gray_in;
        de_q   <= de_in;

        // Queue the write for the next cycle.  addr still holds this pixel's
        // index because it only advances below.
        wvalid <= de_in;
        if (de_in) begin
            waddr <= addr;
            wdata <= gray_in;
        end

        if (frame_start) begin
            addr      <= {AWIDTH{1'b0}};
            frame_cnt <= frame_cnt + 16'd1;
        end else if (de_in) begin
            addr <= addr + 1'b1;
        end

        // One pixel of the previous frame is available with de_q.
        if (de_q) begin
            de_out   <= 1'b1;
            mask_out <= (frame_cnt >= 16'd2) && (diff_q > DIFF_THRESH);
        end
    end
end

endmodule
