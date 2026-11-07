`timescale 1ns/1ps
// Pipelined CORDIC, vectoring mode: atan2(y,x) and K*sqrt(x^2+y^2).
// Inputs x,y : signed Q2.14.  angle: signed Q3.13 (+-pi).  mag: Q2.14 (gain K not removed).
// GUARD extra fractional bits are carried inside the pipeline.
// Throughput: 1 result/clock. Latency: ITER+1 clocks.
module cordic_vec #(
    parameter ITER  = 14,
    parameter GUARD = 4
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               valid_in,
    input  wire signed [15:0] x_in,
    input  wire signed [15:0] y_in,
    output wire               valid_out,
    output wire signed [15:0] angle_out,
    output wire signed [15:0] mag_out
);
    localparam XW = 16 + GUARD;
`include "cordic_vec_atan.vh"

    reg signed [XW-1:0] x [0:ITER];
    reg signed [XW-1:0] y [0:ITER];
    reg signed [15:0]   z [0:ITER];
    reg [ITER:0]        v;

    // Stage 0: quadrant pre-rotation + guard bits
    wire               neg = x_in[15];
    wire signed [15:0] xr  = neg ? -x_in : x_in;
    wire signed [15:0] yr  = neg ? -y_in : y_in;
    wire signed [15:0] z0  = neg ? ((yr <= 0) ? PI_FX : -PI_FX) : 16'sd0;
    wire signed [XW-1:0] xe = xr;
    wire signed [XW-1:0] ye = yr;

    always @(posedge clk) begin
        if (!rst_n) v[0] <= 1'b0;
        else        v[0] <= valid_in;
        x[0] <= xe <<< GUARD;
        y[0] <= ye <<< GUARD;
        z[0] <= z0;
    end

    genvar i;
    generate
        for (i = 0; i < ITER; i = i + 1) begin : stage
            wire signed [XW-1:0] xs  = x[i] >>> i;
            wire signed [XW-1:0] ys  = y[i] >>> i;
            wire                 yneg = y[i][XW-1];
            always @(posedge clk) begin
                if (!rst_n) v[i+1] <= 1'b0;
                else        v[i+1] <= v[i];
                x[i+1] <= yneg ? (x[i] - ys) : (x[i] + ys);
                y[i+1] <= yneg ? (y[i] + xs) : (y[i] - xs);
                z[i+1] <= yneg ? (z[i] - atan_vec_lut(i)) : (z[i] + atan_vec_lut(i));
            end
        end
    endgenerate

    wire signed [XW-1:0] xfinal = x[ITER];
    assign mag_out   = xfinal[XW-1:GUARD];
    assign angle_out = z[ITER];
    assign valid_out = v[ITER];
endmodule
