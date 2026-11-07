`timescale 1ns/1ps
// 8N1 UART receiver. valid pulses 1 clk when a byte is received (stop bit checked).
module uart_rx #(
    parameter CLKS_PER_BIT = 8
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg        valid,
    output reg  [7:0] data,
    output reg        frame_err
);
    reg [1:0] sync;
    always @(posedge clk) sync <= {sync[0], rx};
    wire rxs = sync[1];

    localparam IDLE = 0, START = 1, DATA = 2, STOP = 3;
    reg [1:0]  st;
    reg [15:0] cnt;
    reg [2:0]  bit_i;
    reg [7:0]  sh;

    always @(posedge clk) begin
        if (!rst_n) begin
            st <= IDLE; valid <= 1'b0; frame_err <= 1'b0; cnt <= 0; bit_i <= 0;
        end else begin
            valid <= 1'b0; frame_err <= 1'b0;
            case (st)
                IDLE: if (!rxs) begin st <= START; cnt <= 0; end
                START: begin
                    if (cnt == (CLKS_PER_BIT/2) - 1) begin
                        if (!rxs) begin st <= DATA; cnt <= 0; bit_i <= 0; end
                        else st <= IDLE;
                    end else cnt <= cnt + 1;
                end
                DATA: begin
                    if (cnt == CLKS_PER_BIT - 1) begin
                        cnt <= 0;
                        sh <= {rxs, sh[7:1]};
                        if (bit_i == 7) st <= STOP; else bit_i <= bit_i + 1;
                    end else cnt <= cnt + 1;
                end
                STOP: begin
                    if (cnt == CLKS_PER_BIT - 1) begin
                        cnt <= 0; st <= IDLE;
                        if (rxs) begin valid <= 1'b1; data <= sh; end
                        else frame_err <= 1'b1;
                    end else cnt <= cnt + 1;
                end
            endcase
        end
    end
endmodule
