`timescale 1ns/1ps
// Center-aligned PWM, complementary outputs with dead time.
// cmd: signed, +-32767.  Duty = 50% + cmd/65534 (cmd=0 -> 50%, +max -> ~100%, -max -> ~0%).
// PERIOD: counter runs 0..PERIOD-1 up then down (carrier period = 2*PERIOD clocks).
module pwm_gen #(
    parameter PERIOD = 256,
    parameter DEAD   = 4
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] cmd,
    output reg                hi,
    output reg                lo
);
    reg [15:0] cnt;
    reg        up;
    reg [15:0] cmp;
    wire signed [31:0] scaled = (cmd * PERIOD) >>> 16;       // +-PERIOD/2
    wire signed [31:0] cmpv   = (PERIOD / 2) + scaled;

    always @(posedge clk) begin
        if (!rst_n) begin
            cnt <= 16'd0; up <= 1'b1; cmp <= PERIOD / 2;
        end else begin
            if (up) begin
                if (cnt == PERIOD - 1) begin up <= 1'b0; cmp <= cmpv[15:0]; end
                else cnt <= cnt + 16'd1;
            end else begin
                if (cnt == 0) up <= 1'b1;
                else cnt <= cnt - 16'd1;
            end
        end
    end

    wire raw = (cnt < cmp);
    // dead time: hi requires cnt < cmp - DEAD, lo requires cnt >= cmp + DEAD
    wire hi_n = (cnt + DEAD) < cmp;
    wire lo_n = (cnt >= cmp + DEAD);
    always @(posedge clk) begin
        if (!rst_n) begin hi <= 1'b0; lo <= 1'b0; end
        else begin hi <= hi_n; lo <= lo_n; end
    end
endmodule
