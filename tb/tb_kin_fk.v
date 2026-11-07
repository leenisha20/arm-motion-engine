`timescale 1ns/1ps
module tb_kin_fk;
    integer N = 0;

    reg                clk = 0;
    reg                rst_n = 0;
    reg                valid_in = 0;
    reg  signed [15:0] t1 = 0, t2 = 0;
    wire               valid_out;
    wire signed [15:0] x_o, y_o;

    kin_fk dut (.clk(clk), .rst_n(rst_n), .valid_in(valid_in),
                .t1(t1), .t2(t2), .valid_out(valid_out), .x_out(x_o), .y_out(y_o));

    always #5 clk = ~clk;

    reg [63:0] vec [0:4095];
    integer sent = 0, got = 0, errors = 0, cycles = 0, fd, rc;
    reg signed [15:0] exp_x, exp_y;

    initial begin
        fd = $fopen("tb/fk_count.txt", "r");
        rc = $fscanf(fd, "%d", N);
        $fclose(fd);
        $readmemh("tb/fk_vectors.hex", vec);
        $dumpfile("sim/tb_kin_fk.vcd");
        $dumpvars(0, tb_kin_fk);
        repeat (4) @(posedge clk);
        #1 rst_n = 1;
    end

    always @(posedge clk) begin
        cycles <= cycles + 1;
        if (rst_n && sent < N) begin
            t1 <= vec[sent][63:48];
            t2 <= vec[sent][47:32];
            valid_in <= 1'b1;
            sent <= sent + 1;
        end else begin
            valid_in <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (valid_out && got < N) begin
            exp_x = vec[got][31:16];
            exp_y = vec[got][15:0];
            if (x_o !== exp_x || y_o !== exp_y) begin
                errors = errors + 1;
                if (errors <= 10)
                    $display("MISMATCH #%0d t1=%h t2=%h: x got %h exp %h | y got %h exp %h",
                             got, vec[got][63:48], vec[got][47:32], x_o, exp_x, y_o, exp_y);
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
        #500000;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
