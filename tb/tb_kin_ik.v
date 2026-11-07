`timescale 1ns/1ps
module tb_kin_ik;
    integer N = 0;

    reg                clk = 0;
    reg                rst_n = 0;
    reg                valid_in = 0;
    reg  signed [15:0] x_in = 0, y_in = 0;
    wire               valid_out;
    wire signed [15:0] t1_o, t2_o;

    kin_ik dut (.clk(clk), .rst_n(rst_n), .valid_in(valid_in),
                .x_in(x_in), .y_in(y_in),
                .valid_out(valid_out), .t1_out(t1_o), .t2_out(t2_o));

    always #5 clk = ~clk;

    reg [63:0] vec [0:4095];
    integer sent = 0, got = 0, errors = 0, cycles = 0, fd, rc;
    reg signed [15:0] exp_t1, exp_t2;

    initial begin
        fd = $fopen("tb/ik_count.txt", "r");
        rc = $fscanf(fd, "%d", N);
        $fclose(fd);
        $readmemh("tb/ik_vectors.hex", vec);
        $dumpfile("sim/tb_kin_ik.vcd");
        $dumpvars(0, tb_kin_ik);
        repeat (4) @(posedge clk);
        #1 rst_n = 1;
    end

    always @(posedge clk) begin
        cycles <= cycles + 1;
        if (rst_n && sent < N) begin
            x_in     <= vec[sent][63:48];
            y_in     <= vec[sent][47:32];
            valid_in <= 1'b1;
            sent     <= sent + 1;
        end else begin
            valid_in <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (valid_out && got < N) begin
            exp_t1 = vec[got][31:16];
            exp_t2 = vec[got][15:0];
            if (t1_o !== exp_t1 || t2_o !== exp_t2) begin
                errors = errors + 1;
                if (errors <= 10)
                    $display("MISMATCH #%0d x=%h y=%h: t1 got %h exp %h | t2 got %h exp %h",
                             got, vec[got][63:48], vec[got][47:32], t1_o, exp_t1, t2_o, exp_t2);
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
