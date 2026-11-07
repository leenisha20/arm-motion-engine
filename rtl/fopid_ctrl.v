`timescale 1ns/1ps
// Fractional-order PID (Gruenwald-Letnikov, short memory M), bit-accurate with model/fopid_model.py.
// r_in, p_in: signed Q3.13.  u_out: signed 16-bit.  Serial MAC: latency M+2 clocks,
// a new sample is accepted only when busy = 0.  clear: first sample of a new run (history = 0).
module fopid_ctrl #(
    parameter M = 16
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               clear,
    input  wire               valid_in,
    input  wire signed [15:0] r_in,
    input  wire signed [15:0] p_in,
    output reg                valid_out,
    output reg  signed [15:0] u_out,
    output wire               busy
);
`include "pid_params.vh"
`include "fopid_params.vh"

    reg signed [31:0] hist [0:M-1];
    reg signed [31:0] e_r;
    reg signed [63:0] acc_i, acc_d;
    reg        [7:0]  j;
    reg               running, fin;
    integer           m;
    assign busy = running | fin;

    wire signed [31:0] r32 = r_in;
    wire signed [31:0] p32 = p_in;
    wire signed [31:0] e   = r32 - p32;

    wire signed [63:0] wi64 = wi_lut(j);
    wire signed [63:0] wd64 = wd_lut(j);
    wire signed [63:0] h64  = hist[j];

    // final combination (uses accumulators after the last tap)
    wire signed [63:0] imax  = IMAX_K;
    wire signed [63:0] i_raw = acc_i >>> WS;
    wire signed [63:0] i_cl  = (i_raw > imax) ? imax : (i_raw < -imax) ? -imax : i_raw;
    wire signed [63:0] d_val = acc_d >>> WS;
    wire signed [63:0] e64   = e_r;
    wire signed [63:0] acc   = KP_K * e64 + KI_K * i_cl + KD_K * d_val;
    wire signed [63:0] u64   = acc >>> 8;
    wire signed [63:0] umax  = UMAX_K;

    always @(posedge clk) begin
        if (!rst_n) begin
            running <= 1'b0; fin <= 1'b0; valid_out <= 1'b0;
        end else begin
            valid_out <= 1'b0;
            if (valid_in && !busy) begin
                if (clear) for (m = 1; m < M; m = m + 1) hist[m] <= 32'sd0;
                else       for (m = 1; m < M; m = m + 1) hist[m] <= hist[m-1];
                hist[0] <= e;
                e_r     <= e;
                j       <= 8'd0;
                acc_i   <= 64'sd0;
                acc_d   <= 64'sd0;
                running <= 1'b1;
            end else if (running) begin
                acc_i <= acc_i + wi64 * h64;
                acc_d <= acc_d + wd64 * h64;
                if (j == M - 1) begin running <= 1'b0; fin <= 1'b1; end
                else            j <= j + 8'd1;
            end else if (fin) begin
                fin       <= 1'b0;
                valid_out <= 1'b1;
                u_out     <= (u64 > umax) ? UMAX_K[15:0] :
                             (u64 < -umax) ? -UMAX_K[15:0] : u64[15:0];
            end
        end
    end
endmodule
