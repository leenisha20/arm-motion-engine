`timescale 1ns/1ps
// Pipelined integer square root (digit-by-digit), floor(sqrt(d)).
// d_in: unsigned up to 2^28 (30-bit port).  root_out: unsigned, up to 2^14.
// Throughput 1/clk. Latency 16 clocks (1 input register + 15 stages).
module isqrt_pipe (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,
    input  wire [29:0] d_in,
    output wire        valid_out,
    output wire [15:0] root_out
);
    localparam STAGES = 15;

    reg [29:0]     n [0:STAGES];   // remainder
    reg [29:0]     r [0:STAGES];   // partial root (scaled)
    reg [STAGES:0] v;

    always @(posedge clk) begin
        if (!rst_n) v[0] <= 1'b0;
        else        v[0] <= valid_in;
        n[0] <= d_in;
        r[0] <= 30'd0;
    end

    genvar i;
    generate
        for (i = 0; i < STAGES; i = i + 1) begin : stage
            localparam [29:0] BIT = 30'd1 << (28 - 2 * i);
            wire [29:0] t  = r[i] + BIT;
            wire        ge = (n[i] >= t);
            always @(posedge clk) begin
                if (!rst_n) v[i+1] <= 1'b0;
                else        v[i+1] <= v[i];
                n[i+1] <= ge ? (n[i] - t) : n[i];
                r[i+1] <= (r[i] >> 1) + (ge ? BIT : 30'd0);
            end
        end
    endgenerate

    assign root_out  = r[STAGES][15:0];
    assign valid_out = v[STAGES];
endmodule
