`timescale 1ns/1ps
// 2-link forward kinematics: x = L1*cos(t1) + L2*cos(t1+t2), y = L1*sin(t1) + L2*sin(t1+t2)
// Inputs t1,t2: signed Q3.13 (t1 in [-pi,pi], t2 in [0,pi]).  Outputs x,y: signed Q2.14.
// Throughput 1/clk. Latency ITER+3 = 17 clocks.
module kin_fk (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               valid_in,
    input  wire signed [15:0] t1,
    input  wire signed [15:0] t2,
    output reg                valid_out,
    output reg  signed [15:0] x_out,
    output reg  signed [15:0] y_out
);
    localparam ITER = 14;
    localparam LAT  = ITER + 1;      // latency of cordic_pipe
`include "kin_params.vh"

    // Wrap to [-pi,pi], fold into [-pi/2,pi/2]. Returns {cos_flip, angle_Q2.14}
    function [16:0] qreduce(input signed [16:0] araw);
        reg signed [16:0] a, r, r2;
        reg sgn;
        begin
            a = araw;
            if (a > PI_K)       a = a - 2 * PI_K;
            else if (a < -PI_K) a = a + 2 * PI_K;
            sgn = 1'b0;
            r   = a;
            if (a > HALF_K)       begin r = PI_K - a;  sgn = 1'b1; end
            else if (a < -HALF_K) begin r = -PI_K - a; sgn = 1'b1; end
            r2 = r <<< 1;                      // Q3.13 -> Q2.14
            qreduce = {sgn, r2[15:0]};
        end
    endfunction

    // Stage A: angle sum + quadrant reduction
    wire signed [16:0] a1 = {t1[15], t1};
    wire signed [16:0] a2 = a1 + {t2[15], t2};
    reg  [16:0] q1, q2;
    reg         vq;
    always @(posedge clk) begin
        if (!rst_n) vq <= 1'b0; else vq <= valid_in;
        q1 <= qreduce(a1);
        q2 <= qreduce(a2);
    end

    // Stage B: two sin/cos cores
    wire               v1, v2;
    wire signed [15:0] cos1, sin1, cos2, sin2;
    cordic_pipe #(.ITER(ITER)) u_c1 (.clk(clk), .rst_n(rst_n), .valid_in(vq),
        .theta(q1[15:0]), .valid_out(v1), .cos_out(cos1), .sin_out(sin1));
    cordic_pipe #(.ITER(ITER)) u_c2 (.clk(clk), .rst_n(rst_n), .valid_in(vq),
        .theta(q2[15:0]), .valid_out(v2), .cos_out(cos2), .sin_out(sin2));

    // Delay the cos-flip flags to line up with the CORDIC outputs
    reg [1:0] sg [0:LAT-1];
    integer k;
    always @(posedge clk) begin
        sg[0] <= {q2[16], q1[16]};
        for (k = 1; k < LAT; k = k + 1) sg[k] <= sg[k-1];
    end

    // Stage C: apply cos sign, multiply by link lengths, sum
    wire signed [15:0] c1f = sg[LAT-1][0] ? -cos1 : cos1;
    wire signed [15:0] c2f = sg[LAT-1][1] ? -cos2 : cos2;
    wire signed [31:0] xs  = (L1_K * c1f + L2_K * c2f) >>> 14;
    wire signed [31:0] ys  = (L1_K * sin1 + L2_K * sin2) >>> 14;

    always @(posedge clk) begin
        if (!rst_n) valid_out <= 1'b0; else valid_out <= v1;
        x_out <= xs[15:0];
        y_out <= ys[15:0];
    end
endmodule
