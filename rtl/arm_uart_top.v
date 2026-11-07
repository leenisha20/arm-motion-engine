`timescale 1ns/1ps
// Host command path: UART -> frame parser -> target registers -> arm_top.
// cmd 1 = set X (Q2.14), cmd 2 = set Y (Q2.14), cmd 3 = go.
module arm_uart_top #(
    parameter TICK = 32,
    parameter CPB  = 434
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               rx,
    input  wire signed [15:0] pos1,
    input  wire signed [15:0] pos2,
    output wire signed [15:0] ref1,
    output wire signed [15:0] ref2,
    output wire signed [15:0] u1,
    output wire signed [15:0] u2,
    output wire               u1_valid,
    output wire               u2_valid,
    output wire               moving,
    output wire               bad_frame
);
    wire       bv, ferr;
    wire [7:0] bd;
    uart_rx #(.CLKS_PER_BIT(CPB)) urx (.clk(clk), .rst_n(rst_n), .rx(rx),
        .valid(bv), .data(bd), .frame_err(ferr));

    wire        cv;
    wire [7:0]  cmd;
    wire [15:0] cdata;
    cmd_parser cp (.clk(clk), .rst_n(rst_n), .byte_valid(bv), .byte_in(bd),
        .cmd_valid(cv), .cmd(cmd), .data(cdata), .bad_frame(bad_frame));

    reg signed [15:0] x_r, y_r;
    reg               go;
    always @(posedge clk) begin
        go <= 1'b0;
        if (!rst_n) begin x_r <= 16'sd0; y_r <= 16'sd0; end
        else if (cv) begin
            case (cmd)
                8'd1: x_r <= cdata;
                8'd2: y_r <= cdata;
                8'd3: go  <= 1'b1;
                default: ;
            endcase
        end
    end

    arm_top #(.TICK(TICK)) arm (.clk(clk), .rst_n(rst_n), .go(go), .x_in(x_r), .y_in(y_r),
        .pos1(pos1), .pos2(pos2), .ref1(ref1), .ref2(ref2), .u1(u1), .u2(u2),
        .u1_valid(u1_valid), .u2_valid(u2_valid), .moving(moving));
endmodule
