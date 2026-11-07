`timescale 1ns/1ps
// Two-arm leader-follower. Arm A gets target (x,y); arm B gets (x+DX, y+DY) DELAY+1 clocks later.
module arm_pair #(
    parameter TICK  = 32,
    parameter DELAY = 500,
    parameter signed [15:0] DX = -16'sd1500,
    parameter signed [15:0] DY =  16'sd1000
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               go,
    input  wire signed [15:0] x_in,
    input  wire signed [15:0] y_in,
    input  wire signed [15:0] pos1a, pos2a, pos1b, pos2b,
    output wire signed [15:0] ref1a, ref2a, ref1b, ref2b,
    output wire signed [15:0] u1a, u2a, u1b, u2b,
    output wire               u1a_v, u2a_v, u1b_v, u2b_v,
    output wire               moving
);
    reg [15:0]        cnt;
    reg               pend, go_b;
    reg signed [15:0] xb, yb;
    always @(posedge clk) begin
        go_b <= 1'b0;
        if (!rst_n) pend <= 1'b0;
        else if (go) begin
            pend <= 1'b1; cnt <= DELAY - 1;
            xb <= x_in + DX; yb <= y_in + DY;
        end else if (pend) begin
            if (cnt == 0) begin go_b <= 1'b1; pend <= 1'b0; end
            else cnt <= cnt - 16'd1;
        end
    end

    wire ma, mb;
    arm_top #(.TICK(TICK)) a (.clk(clk), .rst_n(rst_n), .go(go), .x_in(x_in), .y_in(y_in),
        .pos1(pos1a), .pos2(pos2a), .ref1(ref1a), .ref2(ref2a), .u1(u1a), .u2(u2a),
        .u1_valid(u1a_v), .u2_valid(u2a_v), .moving(ma));
    arm_top #(.TICK(TICK)) b (.clk(clk), .rst_n(rst_n), .go(go_b), .x_in(xb), .y_in(yb),
        .pos1(pos1b), .pos2(pos2b), .ref1(ref1b), .ref2(ref2b), .u1(u1b), .u2(u2b),
        .u1_valid(u1b_v), .u2_valid(u2b_v), .moving(mb));
    assign moving = ma | mb | pend;
endmodule
