`timescale 1ns/1ps
// One joint: quintic trajectory -> PID. Reference is replayed one sample per tick.
// Command: pulse start with q0, q1, log2n. u_out updates once per tick.
module joint_axis #(
    parameter TICK = 32
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire signed [15:0] q0,
    input  wire signed [15:0] q1,
    input  wire        [3:0]  log2n,
    input  wire signed [15:0] pos_in,
    output wire signed [15:0] ref_out,
    output wire signed [15:0] u_out,
    output wire               u_valid,
    output wire               moving
);
    wire               tv, tl, tbusy;
    wire signed [15:0] tq;
    traj_gen tg (.clk(clk), .rst_n(rst_n), .start(start), .q0(q0), .q1(q1),
                 .log2n(log2n), .valid_out(tv), .last_out(tl), .q_out(tq), .busy(tbusy));

    reg [7:0] tc;
    reg       tick;
    always @(posedge clk) begin
        tick <= 1'b0;
        if (!rst_n) tc <= 8'd0;
        else if (tc == TICK - 1) begin tc <= 8'd0; tick <= 1'b1; end
        else tc <= tc + 8'd1;
    end

    reg signed [15:0] rmem [0:2047];
    reg        [10:0] wr, rd;
    reg signed [15:0] ref_r;
    reg               first;
    always @(posedge clk) begin
        if (!rst_n) begin wr <= 0; rd <= 0; ref_r <= 0; first <= 1'b1; end
        else begin
            if (start && !tbusy) begin wr <= 0; rd <= 0; end
            if (tv) begin rmem[wr] <= tq; wr <= wr + 11'd1; end
            if (tick && rd < wr) begin ref_r <= rmem[rd]; rd <= rd + 11'd1; end
            if (tick) first <= 1'b0;
        end
    end
    assign ref_out = ref_r;
    assign moving  = tbusy | (rd < wr);

    pid_ctrl pid (.clk(clk), .rst_n(rst_n), .clear(first), .valid_in(tick),
                  .r_in(ref_r), .p_in(pos_in), .valid_out(u_valid), .u_out(u_out));
endmodule
