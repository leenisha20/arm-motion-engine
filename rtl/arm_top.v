`timescale 1ns/1ps
// Two-joint arm: target (x,y) -> IK -> two joint axes (trajectory + PID).
// Pulse go with x,y (Q2.14). When IK finishes, both joints start a quintic move
// from their current reference to the IK angles (Q3.13).
module arm_top #(
    parameter TICK = 32,
    parameter [3:0] LOG2N = 4'd9
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               go,
    input  wire signed [15:0] x_in,
    input  wire signed [15:0] y_in,
    input  wire signed [15:0] pos1,
    input  wire signed [15:0] pos2,
    output wire signed [15:0] ref1,
    output wire signed [15:0] ref2,
    output wire signed [15:0] u1,
    output wire signed [15:0] u2,
    output wire               u1_valid,
    output wire               u2_valid,
    output wire               moving
);
    wire               ik_v;
    wire signed [15:0] t1, t2;
    kin_ik ik (.clk(clk), .rst_n(rst_n), .valid_in(go), .x_in(x_in), .y_in(y_in),
               .valid_out(ik_v), .t1_out(t1), .t2_out(t2));

    wire m1, m2;
    joint_axis #(.TICK(TICK)) j1 (.clk(clk), .rst_n(rst_n), .start(ik_v), .q0(ref1), .q1(t1),
        .log2n(LOG2N), .pos_in(pos1), .ref_out(ref1), .u_out(u1), .u_valid(u1_valid), .moving(m1));
    joint_axis #(.TICK(TICK)) j2 (.clk(clk), .rst_n(rst_n), .start(ik_v), .q0(ref2), .q1(t2),
        .log2n(LOG2N), .pos_in(pos2), .ref_out(ref2), .u_out(u2), .u_valid(u2_valid), .moving(m2));
    assign moving = m1 | m2;
endmodule
