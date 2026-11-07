`timescale 1ns/1ps
// Quintic trajectory generator, bit-accurate with model/traj_model.py (F = 20).
// start (1 clk pulse, when idle) with q0,q1 (Q3.13) and log2n (6..10) -> N+1 samples, 1 per clock.
// q(k) = q0 + (q1-q0) * (10t^3 - 15t^4 + 6t^5),  t = k/N
module traj_gen (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire signed [15:0] q0,
    input  wire signed [15:0] q1,
    input  wire        [3:0]  log2n,
    output reg                valid_out,
    output reg                last_out,
    output reg  signed [15:0] q_out,
    output wire               busy
);
    localparam integer F = 20;
    localparam signed [63:0] TEN = 64'sd10 << F;

    // ---- sequencer
    reg               running;
    reg        [10:0] k;
    reg        [3:0]  nl;
    reg signed [15:0] q0l, q1l;
    assign busy = running;

    // ---- pipeline registers
    reg signed [63:0] t0, t1r, t2_1, t3_2, poly_2, s_3;
    reg signed [15:0] q0_0, q0_1, q0_2, q0_3;
    reg signed [31:0] dq_0, dq_1, dq_2, dq_3;
    reg               v0, v1, v2, v3, l0, l1, l2, l3;
    reg signed [63:0] t_2;
    wire [31:0] kk = {21'd0, k};

    always @(posedge clk) begin
        if (!rst_n) begin
            running <= 1'b0;
            v0 <= 1'b0; v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0; valid_out <= 1'b0;
        end else begin
            // sequencer
            if (start && !running) begin
                q0l <= q0; q1l <= q1; nl <= log2n; k <= 11'd0; running <= 1'b1;
                v0 <= 1'b0;
            end else if (running) begin
                v0   <= 1'b1;
                t0   <= kk << (F - nl);
                q0_0 <= q0l;
                dq_0 <= q1l - q0l;
                l0   <= (k == (11'd1 << nl));
                if (k == (11'd1 << nl)) running <= 1'b0;
                else                    k <= k + 11'd1;
            end else begin
                v0 <= 1'b0;
            end
            v1 <= v0; v2 <= v1; v3 <= v2; valid_out <= v3;
        end
        // stage 1: t^2
        t1r  <= t0;
        t2_1 <= (t0 * t0) >>> F;
        q0_1 <= q0_0; dq_1 <= dq_0; l1 <= l0;
        // stage 2: t^3 and polynomial
        t_2    <= t1r;
        t3_2   <= (t2_1 * t1r) >>> F;
        poly_2 <= TEN - 15 * t1r + 6 * t2_1;
        q0_2 <= q0_1; dq_2 <= dq_1; l2 <= l1;
        // stage 3: s
        s_3  <= (t3_2 * poly_2) >>> F;
        q0_3 <= q0_2; dq_3 <= dq_2; l3 <= l2;
        // stage 4: output
        q_out    <= q0_3 + ((dq_3 * s_3) >>> F);
        last_out <= l3;
    end
endmodule
