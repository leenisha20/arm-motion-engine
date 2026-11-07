`timescale 1ns/1ps
// Frame: A5, cmd, data_hi, data_lo, chk (= cmd^hi^lo), 5A.
// Good frame -> cmd_valid pulse with cmd/data. Bad checksum/trailer -> bad_frame pulse.
module cmd_parser (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        byte_valid,
    input  wire [7:0]  byte_in,
    output reg         cmd_valid,
    output reg  [7:0]  cmd,
    output reg  [15:0] data,
    output reg         bad_frame
);
    reg [2:0] st;
    reg [7:0] c, hi, lo, ck;
    always @(posedge clk) begin
        if (!rst_n) begin
            st <= 0; cmd_valid <= 1'b0; bad_frame <= 1'b0;
        end else begin
            cmd_valid <= 1'b0; bad_frame <= 1'b0;
            if (byte_valid) begin
                case (st)
                    0: if (byte_in == 8'hA5) st <= 1;
                    1: begin c  <= byte_in; st <= 2; end
                    2: begin hi <= byte_in; st <= 3; end
                    3: begin lo <= byte_in; st <= 4; end
                    4: begin ck <= byte_in; st <= 5; end
                    5: begin
                        st <= 0;
                        if (byte_in == 8'h5A && ck == (c ^ hi ^ lo)) begin
                            cmd_valid <= 1'b1; cmd <= c; data <= {hi, lo};
                        end else bad_frame <= 1'b1;
                    end
                endcase
            end
        end
    end
endmodule
