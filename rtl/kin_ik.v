`timescale 1ns/1ps
// 2-link inverse kinematics (elbow-down), bit-accurate with model/ik_hw_model.py
// Inputs x,y: signed Q2.14.  Outputs t1,t2: signed Q3.13 (t1 in +-pi, t2 in [0,pi]).
// Throughput 1/clk. Latency 37 clocks.
module kin_ik (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               valid_in,
    input  wire signed [15:0] x_in,
    input  wire signed [15:0] y_in,
    output reg                valid_out,
    output reg  signed [15:0] t1_out,
    output reg  signed [15:0] t2_out
);
`include "kin_params.vh"
`include "ik_params.vh"

    // ---- Stage A (cycle 1): num = x^2 + y^2 - C28 (Q4.28)
    wire signed [31:0] xx = x_in * x_in;
    wire signed [31:0] yy = y_in * y_in;
    reg  signed [31:0] num_r;
    reg                va;
    always @(posedge clk) begin
        if (!rst_n) va <= 1'b0; else va <= valid_in;
        num_r <= xx + yy - C28_K;
    end

    // ---- Stage B (cycle 2): multiply by reciprocal, add rounding constant
    wire signed [63:0] nw = num_r;
    wire signed [63:0] iw = INV_K;
    reg  signed [63:0] prod_r;
    reg                vb;
    always @(posedge clk) begin
        if (!rst_n) vb <= 1'b0; else vb <= va;
        prod_r <= nw * iw + (64'sd1 <<< 31);
    end

    // ---- Stage C (cycle 3): shift by 32, clamp to +-C2MAX -> c2 (Q2.14)
    wire signed [63:0] sh   = prod_r >>> 32;
    wire signed [63:0] cmax = C2MAX_K;
    wire signed [63:0] cmin = -cmax;
    reg  signed [15:0] c2_r;
    reg                vc;
    always @(posedge clk) begin
        if (!rst_n) vc <= 1'b0; else vc <= vb;
        if (sh > cmax)      c2_r <= C2MAX_K;
        else if (sh < cmin) c2_r <= -C2MAX_K;
        else                c2_r <= sh[15:0];
    end

    // ---- Stage D (cycle 4): 2^28 - c2^2
    wire signed [31:0] csq = c2_r * c2_r;
    reg  [29:0]        c2sq_r;
    reg                vd;
    always @(posedge clk) begin
        if (!rst_n) vd <= 1'b0; else vd <= vc;
        c2sq_r <= 30'd268435456 - csq[29:0];
    end

    // ---- Square root (output at cycle 20)
    wire        vs;
    wire [15:0] s2_w;
    isqrt_pipe u_sq (.clk(clk), .rst_n(rst_n), .valid_in(vd), .d_in(c2sq_r),
                     .valid_out(vs), .root_out(s2_w));

    // c2 delay line: c2_r (cycle 3) -> c2d[16] (cycle 20)
    reg signed [15:0] c2d [0:16];
    integer k;
    always @(posedge clk) begin
        c2d[0] <= c2_r;
        for (k = 1; k <= 16; k = k + 1) c2d[k] <= c2d[k-1];
    end

    // ---- Stage E (cycle 21): k1 = L1 + L2*c2, k2 = L2*s2
    wire signed [15:0] s2s = s2_w;
    wire signed [15:0] c2s = c2d[16];
    wire signed [31:0] m1  = L2_K * c2s;
    wire signed [31:0] m2  = L2_K * s2s;
    reg  signed [15:0] k1_r, k2_r, c2e, s2e;
    reg                ve;
    always @(posedge clk) begin
        if (!rst_n) ve <= 1'b0; else ve <= vs;
        k1_r <= L1_K + (m1 >>> 14);
        k2_r <= m2 >>> 14;
        c2e  <= c2s;
        s2e  <= s2s;
    end

    // ---- Three atan2 cores
    wire               v_phi, v_t2, v_tg;
    wire signed [15:0] aphi, t2_w, atg;
    wire signed [15:0] m_phi, m_t2, m_tg;   // magnitudes, unused
    cordic_vec #(.ITER(14), .GUARD(4)) u_phi (.clk(clk), .rst_n(rst_n), .valid_in(ve),
        .x_in(k1_r), .y_in(k2_r), .valid_out(v_phi), .angle_out(aphi), .mag_out(m_phi));
    cordic_vec #(.ITER(14), .GUARD(4)) u_t2  (.clk(clk), .rst_n(rst_n), .valid_in(ve),
        .x_in(c2e),  .y_in(s2e),  .valid_out(v_t2),  .angle_out(t2_w), .mag_out(m_t2));
    cordic_vec #(.ITER(14), .GUARD(4)) u_tg  (.clk(clk), .rst_n(rst_n), .valid_in(valid_in),
        .x_in(x_in), .y_in(y_in), .valid_out(v_tg),  .angle_out(atg),  .mag_out(m_tg));

    // target-angle delay line: atg (cycle 15) -> atgd[20] (cycle 36)
    reg signed [15:0] atgd [0:20];
    integer j;
    always @(posedge clk) begin
        atgd[0] <= atg;
        for (j = 1; j <= 20; j = j + 1) atgd[j] <= atgd[j-1];
    end

    // ---- Stage F (cycle 37): t1 = atgt - aphi, wrapped to +-pi
    wire signed [17:0] pi18   = PI_K;
    wire signed [17:0] twopi  = pi18 <<< 1;
    wire signed [17:0] diff   = atgd[20] - aphi;
    wire signed [17:0] wrapd  = (diff < -pi18) ? (diff + twopi) :
                                (diff >  pi18) ? (diff - twopi) : diff;
    always @(posedge clk) begin
        if (!rst_n) valid_out <= 1'b0; else valid_out <= v_phi;
        t1_out <= wrapd[15:0];
        t2_out <= t2_w;
    end
endmodule
