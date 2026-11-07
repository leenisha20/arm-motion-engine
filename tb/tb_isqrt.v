`timescale 1ns/1ps
module tb_isqrt;
    integer N = 0;

    reg         clk = 0;
    reg         rst_n = 0;
    reg         valid_in = 0;
    reg  [29:0] d_in = 0;
    wire        valid_out;
    wire [15:0] root_o;

    isqrt_pipe dut (.clk(clk), .rst_n(rst_n), .valid_in(valid_in),
                    .d_in(d_in), .valid_out(valid_out), .root_out(root_o));

    always #5 clk = ~clk;

    reg [63:0] vec [0:8191];
    integer sent = 0, got = 0, errors = 0, cycles = 0, fd, rc;
    reg [15:0] exp_r;

    initial begin
        fd = $fopen("tb/isqrt_count.txt", "r");
        rc = $fscanf(fd, "%d", N);
        $fclose(fd);
        $readmemh("tb/isqrt_vectors.hex", vec);
        $dumpfile("sim/tb_isqrt.vcd");
        $dumpvars(0, tb_isqrt);
        repeat (4) @(posedge clk);
        #1 rst_n = 1;
    end

    always @(posedge clk) begin
        cycles <= cycles + 1;
        if (rst_n && sent < N) begin
            d_in     <= vec[sent][61:32];
            valid_in <= 1'b1;
            sent     <= sent + 1;
        end else begin
            valid_in <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (valid_out && got < N) begin
            exp_r = vec[got][15:0];
            if (root_o !== exp_r) begin
                errors = errors + 1;
                if (errors <= 10)
                    $display("MISMATCH #%0d d=%0d: got %0d exp %0d",
                             got, vec[got][61:32], root_o, exp_r);
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
        #1000000;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
