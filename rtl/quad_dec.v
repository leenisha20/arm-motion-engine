`timescale 1ns/1ps
// x4 quadrature decoder with synchronizer. pos increments on forward edges, decrements on reverse.
// err pulses for an illegal (two-bit) transition.
module quad_dec (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               a,
    input  wire               b,
    output reg  signed [15:0] pos,
    output reg                dir,     // 1 = last move forward
    output reg                err
);
    reg [1:0] s1, s2, prev;
    always @(posedge clk) begin
        if (!rst_n) begin
            s1 <= 2'b00; s2 <= 2'b00; prev <= 2'b00;
            pos <= 16'sd0; dir <= 1'b0; err <= 1'b0;
        end else begin
            s1 <= {a, b};
            s2 <= s1;
            prev <= s2;
            err <= 1'b0;
            case ({prev, s2})
                4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: begin pos <= pos + 16'sd1; dir <= 1'b1; end
                4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: begin pos <= pos - 16'sd1; dir <= 1'b0; end
                4'b00_11, 4'b11_00, 4'b01_10, 4'b10_01: err <= 1'b1;
                default: ;
            endcase
        end
    end
endmodule
