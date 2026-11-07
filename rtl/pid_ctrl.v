`timescale 1ns/1ps
// Fixed-point positional PID with clamping anti-windup, bit-accurate with model/pid_model.py.
// r_in, p_in: signed Q3.13.  u_out: signed 16-bit (+-32767).  Latency 1 clock, throughput 1/clk.
// clear: treat this sample as the first of a new run (integrator and previous error = 0).
module pid_ctrl (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               clear,
    input  wire               valid_in,
    input  wire signed [15:0] r_in,
    input  wire signed [15:0] p_in,
    output reg                valid_out,
    output reg  signed [15:0] u_out
);
`include "pid_params.vh"

    reg signed [31:0] integ;
    reg signed [31:0] e_prev;

    wire signed [31:0] r32 = r_in;
    wire signed [31:0] p32 = p_in;
    wire signed [31:0] e   = r32 - p32;

    wire signed [31:0] integ_eff = clear ? 32'sd0 : integ;
    wire signed [31:0] eprev_eff = clear ? 32'sd0 : e_prev;

    wire signed [31:0] i_sum = integ_eff + e;
    wire signed [31:0] i_new = (i_sum >  IMAX_K) ?  IMAX_K :
                               (i_sum < -IMAX_K) ? -IMAX_K : i_sum;
    wire signed [31:0] d     = e - eprev_eff;

    wire signed [63:0] e64 = e;
    wire signed [63:0] i64 = i_new;
    wire signed [63:0] d64 = d;
    wire signed [63:0] acc = KP_K * e64 + KI_K * i64 + KD_K * d64;
    wire signed [63:0] u64 = acc >>> 8;

    wire signed [63:0] umax = UMAX_K;
    wire signed [63:0] umin = -umax;

    always @(posedge clk) begin
        if (!rst_n) begin
            integ     <= 32'sd0;
            e_prev    <= 32'sd0;
            valid_out <= 1'b0;
        end else begin
            valid_out <= valid_in;
            if (valid_in) begin
                integ  <= i_new;
                e_prev <= e;
                u_out  <= (u64 > umax) ? UMAX_K[15:0] :
                          (u64 < umin) ? -UMAX_K[15:0] : u64[15:0];
            end
        end
    end
endmodule
