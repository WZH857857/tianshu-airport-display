//===========================================================================
// stick_overlay.v -- draw each tracked person as a stick figure
//
// The box a slot publishes comes from motion_hist: it is the bounding box of
// the person's recent motion, grown to cover the whole body.  Drawing the box
// hid the person behind a rectangle, so this module draws the skeleton
// instead -- head, spine, shoulders, hips, arms and legs -- in the slot's
// own colour, and turns the whole figure red once the posture machine has
// confirmed a fall.
//
// Geometry.  Everything is axis aligned in a local frame whose u axis runs
// along the box's long side, so one set of bars covers both cases with no
// rotation hardware:
//
//   w <= h  standing   u = vy - y, v = vx - x   (u vertical on screen)
//   w >  h  lying      u = vx - x, v = vy - y   (u horizontal on screen)
//
// The bar list is the one the Python prototype (scratch/_stick_render.py)
// rendered and validated on the test clips, written with half-open spans and
// symmetric half widths so one |v - c| comparison covers both sides of a
// bone (the prototype's PIL rectangles are inclusive, which shortens one
// side of a bone by at most a pixel -- invisible at 640x480):
//
//   head   u [0, hs)        |v - c| < hs/2           filled
//   spine  u [hs, hip)      |v - c| < t
//   shldr  u [sh, sh+t+1)   |v - c| < C/3
//   hips   u [hip, hip+t+1) |v - c| < C/3
//   arms   u [sh, hip-L/6)  |v - c| in [C/2-t-1, C/2)
//   legs   u [hip, L)       |v - c| in [C/3-t-1, C/3+t)
//
// with hs = clamp(L/6, 3, 12), t = max(1, C/16), c = C/2, sh = hs+2.
// A lying figure whose head end is known mirrors u (u -> L-1-u, the exact
// reversal of a half-open span) first, which is how the figure faces the
// right way without any per-bar maths.
//
// Every constant division is a multiply by a reciprocal that is exact over
// the 0..319 range the raster can produce (L/6 = L*171/1024, 3L/5 =
// L*615/1024, C/3 = C*342/1024, the latter exact through C = 1533), so the
// bars land on the same pixels the prototype drew and the pixel path stays
// multiply-only-by-constant.
//
// Raster mapping is the one the old box overlay used: the 640x480 picture is
// read back as vision coordinates, vx = px/2 and vy = 3*py/8, the exact
// inverse of the 8/3 vertical stretch, so a bar edge always lands on the
// same picture pixel as the Python drawing.
//===========================================================================
`timescale 1ns / 1ps

module stick_overlay #(
    parameter integer NB = 4
) (
    input  wire            de,
    input  wire [9:0]      px,
    input  wire [9:0]      py,
    input  wire [23:0]     rgb_in,

    input  wire [NB-1:0]   found,
    input  wire [NB-1:0]   alarm,
    input  wire [NB-1:0]   head_right,
    input  wire [NB*9-1:0] bx,
    input  wire [NB*9-1:0] by,
    input  wire [NB*9-1:0] bw,
    input  wire [NB*9-1:0] bh,

    output reg  [23:0]     rgb_out
);

    wire [9:0]  vx  = {1'b0, px[9:1]};        // 0 .. 319
    wire [10:0] py3 = {1'b0, py} * 11'd3;     // 0 .. 1437
    wire [9:0]  vy  = py3[10:3];              // 0 .. 179
    wire [10:0] vxs = {1'b0, vx};
    wire [10:0] vys = {1'b0, vy};

    wire [NB-1:0] bone_hit;

    genvar gi;
    generate
        for (gi = 0; gi < NB; gi = gi + 1) begin : g_stick
            wire [8:0]  x0r = bx[gi*9 +: 9];
            wire [8:0]  y0r = by[gi*9 +: 9];
            wire [8:0]  wr  = bw[gi*9 +: 9];
            wire [8:0]  hr  = bh[gi*9 +: 9];

            wire        rot = (wr > hr);
            wire [9:0]  L   = rot ? {1'b0, wr} : {1'b0, hr};
            wire [9:0]  C   = rot ? {1'b0, hr} : {1'b0, wr};

            // local frame: u along the long side, v across it
            wire signed [10:0] dx = vxs - {2'b00, x0r};
            wire signed [10:0] dy = vys - {2'b00, y0r};
            wire signed [10:0] su = rot ? dy : dx;
            wire signed [10:0] sv = rot ? dx : dy;
            wire        in_box = (su >= 0) && (su <= {1'b0, L}) &&
                                 (sv >= 0) && (sv <= {1'b0, C});

            wire [9:0]  svu = sv[9:0];
            wire [9:0]  cs  = C >> 1;
            wire [9:0]  du  = (svu >= cs) ? (svu - cs) : (cs - svu);

            // bone geometry
            wire [15:0] l6v  = {6'b0, L} * 16'd171;      // L * 0.166992
            wire [9:0]  l6   = l6v[15:10];               // L / 6
            wire [9:0]  hs   = (l6 < 10'd3) ? 10'd3 :
                               ((l6 > 10'd12) ? 10'd12 : l6);
            wire [9:0]  qh   = hs >> 1;
            wire [9:0]  sh   = hs + 10'd2;
            wire [17:0] hipv = {8'b0, L} * 18'd615;      // L * 0.600586
            wire [9:0]  hip  = hipv[17:10];              // 3L / 5
            wire [9:0]  arm_end = hip - l6;
            wire [9:0]  t    = (C > 10'd16) ? (C >> 4) : 10'd1;
            wire [15:0] c3v  = {6'b0, C} * 16'd342;      // C * 0.333984
            wire [9:0]  c3   = c3v[15:10];               // C / 3
            wire [9:0]  c2   = C >> 1;
            wire [9:0]  arm_lo = (c2 > t + 10'd1) ? (c2 - t - 10'd1) : 10'd0;
            wire [9:0]  leg_lo = (c3 > t + 10'd1) ? (c3 - t - 10'd1) : 10'd0;
            wire [9:0]  leg_hi = c3 + t;
            wire [9:0]  sh_hi  = sh + t + 10'd1;
            wire [9:0]  hip_hi = hip + t + 10'd1;

            // a lying figure may need its long axis mirrored so the head end
            // lands on the side the tracker says the body is not
            wire [9:0]  suu = su[9:0];
            wire [9:0]  u   = (rot && head_right[gi]) ? (L - 10'd1 - suu)
                                                      : suu;

            wire in_head  = (u < hs) && (du < qh);
            wire in_spine = (u >= hs) && (u < hip) && (du < t);
            wire in_sh    = (u >= sh) && (u < sh_hi) && (du < c3);
            wire in_hip   = (u >= hip) && (u < hip_hi) && (du < c3);
            wire in_arm   = (u >= sh) && (u < arm_end) &&
                            (du >= arm_lo) && (du < c2);
            wire in_leg   = (u >= hip) && (u < L) &&
                            (du >= leg_lo) && (du < leg_hi);

            assign bone_hit[gi] = found[gi] && in_box &&
                                  (in_head | in_spine | in_sh | in_hip |
                                   in_arm | in_leg);
        end
    endgenerate

    // One hue per slot, same palette as the box overlay, so a person keeps
    // the colour the eye has already followed across the picture.
    function [23:0] slot_colour;
        input [1:0] s;
        begin
            case (s)
                2'd0:    slot_colour = 24'h00FF60;   // green
                2'd1:    slot_colour = 24'h00C8FF;   // cyan
                2'd2:    slot_colour = 24'hFFC400;   // amber
                default: slot_colour = 24'hFF40FF;   // magenta
            endcase
        end
    endfunction

    integer ci;
    reg [23:0] bone_rgb;
    always @(*) begin
        bone_rgb = 24'h00FF60;
        for (ci = NB - 1; ci >= 0; ci = ci - 1)
            if (bone_hit[ci]) bone_rgb = slot_colour(ci[1:0]);
    end

    wire any_bone  = |bone_hit;
    wire any_fall  = |(bone_hit & alarm);

    always @(*) begin
        if      (!de)       rgb_out = 24'h000000;
        else if (any_fall)  rgb_out = 24'hFF0000;   // confirmed fall
        else if (any_bone)  rgb_out = bone_rgb;     // tracked person
        else                rgb_out = rgb_in;
    end

endmodule
