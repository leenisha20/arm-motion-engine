`timescale 1ns/1ps
module tb_cordic;
    localparam N = 201;

    reg                clk = 0;
    reg                rst_n = 0;
    reg                valid_in = 0;
    reg  signed [15:0] theta = 0;
    wire               valid_out;
    wire signed [15:0] cos_o, sin_o;

    cordic_pipe #(.ITER(14)) dut (
        .clk(clk), .rst_n(rst_n), .valid_in(valid_in), .theta(theta),
        .valid_out(valid_out), .cos_out(cos_o), .sin_out(sin_o)
    );

    always #5 clk = ~clk;

    reg [47:0] vec [0:N-1];
    integer sent = 0, got = 0, errors = 0, cycles = 0;
    reg signed [15:0] exp_c, exp_s;

    initial begin
        $readmemh("tb/cordic_vectors.hex", vec);
        $dumpfile("sim/tb_cordic.vcd");
        $dumpvars(0, tb_cordic);
        repeat (4) @(posedge clk);
        #1 rst_n = 1;
    end

    // Driver: one angle per clock
    always @(posedge clk) begin
        cycles <= cycles + 1;
        if (rst_n && sent < N) begin
            theta    <= vec[sent][47:32];
            valid_in <= 1'b1;
            sent     <= sent + 1;
        end else begin
            valid_in <= 1'b0;
        end
    end

    // Checker: exact match against golden model
    always @(posedge clk) begin
        if (valid_out && got < N) begin
            exp_c = vec[got][31:16];
            exp_s = vec[got][15:0];
            if (cos_o !== exp_c || sin_o !== exp_s) begin
                errors = errors + 1;
                $display("MISMATCH #%0d theta=%h: cos got %h exp %h | sin got %h exp %h",
                         got, vec[got][47:32], cos_o, exp_c, sin_o, exp_s);
            end
            got = got + 1;
            if (got == N) begin
                $display("----------------------------------------");
                $display("Vectors checked : %0d", N);
                $display("Mismatches      : %0d", errors);
                $display("Total cycles    : %0d (pipelined, 1 result/clk)", cycles);
                if (errors == 0) $display("RESULT: PASS");
                else             $display("RESULT: FAIL");
                $finish;
            end
        end
    end

    initial begin
        #100000;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
