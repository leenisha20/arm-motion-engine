`timescale 1ns/1ps
// Pipelined CORDIC, rotation mode: computes cos/sin of theta.
// Format: signed Q2.14 (16-bit). Input range |theta| <= pi/2.
// Throughput: 1 result/clock. Latency: ITER+1 clocks.
module cordic_pipe #(
    parameter ITER = 14          // max 14 with the current table
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               valid_in,
    input  wire signed [15:0] theta,
    output wire               valid_out,
    output wire signed [15:0] cos_out,
    output wire signed [15:0] sin_out
);
    localparam W = 16;
`include "cordic_atan.vh"

    reg signed [W-1:0] x [0:ITER];
    reg signed [W-1:0] y [0:ITER];
    reg signed [W-1:0] z [0:ITER];
    reg [ITER:0]       v;

    // Stage 0: input register, x starts at 1/K, y at 0
    always @(posedge clk) begin
        if (!rst_n) v[0] <= 1'b0;
        else        v[0] <= valid_in;
        x[0] <= X0;
        y[0] <= {W{1'b0}};
        z[0] <= theta;
    end

    genvar i;
    generate
        for (i = 0; i < ITER; i = i + 1) begin : stage
            wire signed [W-1:0] xs  = x[i] >>> i;
            wire signed [W-1:0] ys  = y[i] >>> i;
            wire                neg = z[i][W-1];
            always @(posedge clk) begin
                if (!rst_n) v[i+1] <= 1'b0;
                else        v[i+1] <= v[i];
                x[i+1] <= neg ? (x[i] + ys) : (x[i] - ys);
                y[i+1] <= neg ? (y[i] - xs) : (y[i] + xs);
                z[i+1] <= neg ? (z[i] + atan_lut(i)) : (z[i] - atan_lut(i));
            end
        end
    endgenerate

    assign cos_out   = x[ITER];
    assign sin_out   = y[ITER];
    assign valid_out = v[ITER];
endmodule
