`timescale 1ns/1ps
module tb_cordic_vec;
    integer N = 0;

    reg                clk = 0;
    reg                rst_n = 0;
    reg                valid_in = 0;
    reg  signed [15:0] x_in = 0, y_in = 0;
    wire               valid_out;
    wire signed [15:0] ang_o, mag_o;

    cordic_vec #(.ITER(14), .GUARD(4)) dut (
        .clk(clk), .rst_n(rst_n), .valid_in(valid_in),
        .x_in(x_in), .y_in(y_in),
        .valid_out(valid_out), .angle_out(ang_o), .mag_out(mag_o)
    );

    always #5 clk = ~clk;

    reg [63:0] vec [0:4095];
    integer sent = 0, got = 0, errors = 0, cycles = 0, fd, rc;
    reg signed [15:0] exp_a, exp_m;

    initial begin
        fd = $fopen("tb/cordic_vec_count.txt", "r");
        rc = $fscanf(fd, "%d", N);
        $fclose(fd);
        $readmemh("tb/cordic_vec_vectors.hex", vec);
        $dumpfile("sim/tb_cordic_vec.vcd");
        $dumpvars(0, tb_cordic_vec);
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
            exp_a = vec[got][31:16];
            exp_m = vec[got][15:0];
            if (ang_o !== exp_a || mag_o !== exp_m) begin
                errors = errors + 1;
                if (errors <= 10)
                    $display("MISMATCH #%0d x=%h y=%h: ang got %h exp %h | mag got %h exp %h",
                             got, vec[got][63:48], vec[got][47:32], ang_o, exp_a, mag_o, exp_m);
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
        #200000;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
