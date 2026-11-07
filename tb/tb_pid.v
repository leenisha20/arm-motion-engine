`timescale 1ns/1ps
module tb_pid;
    integer N = 0;

    reg                clk = 0;
    reg                rst_n = 0;
    reg                clear = 0;
    reg                valid_in = 0;
    reg  signed [15:0] r_in = 0, p_in = 0;
    wire               valid_out;
    wire signed [15:0] u_o;

    pid_ctrl dut (.clk(clk), .rst_n(rst_n), .clear(clear), .valid_in(valid_in),
                  .r_in(r_in), .p_in(p_in), .valid_out(valid_out), .u_out(u_o));

    always #5 clk = ~clk;

    reg [63:0] vec [0:8191];
    integer sent = 0, got = 0, errors = 0, cycles = 0, fd, rc;
    reg signed [15:0] exp_u;

    initial begin
        fd = $fopen("tb/pid_count.txt", "r");
        rc = $fscanf(fd, "%d", N);
        $fclose(fd);
        $readmemh("tb/pid_vectors.hex", vec);
        $dumpfile("sim/tb_pid.vcd");
        $dumpvars(0, tb_pid);
        repeat (4) @(posedge clk);
        #1 rst_n = 1;
    end

    always @(posedge clk) begin
        cycles <= cycles + 1;
        if (rst_n && sent < N) begin
            r_in     <= vec[sent][63:48];
            p_in     <= vec[sent][47:32];
            clear    <= vec[sent][0];
            valid_in <= 1'b1;
            sent     <= sent + 1;
        end else begin
            valid_in <= 1'b0;
            clear    <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (valid_out && got < N) begin
            exp_u = vec[got][31:16];
            if (u_o !== exp_u) begin
                errors = errors + 1;
                if (errors <= 10)
                    $display("MISMATCH #%0d r=%0d p=%0d: u got %0d exp %0d",
                             got, $signed(vec[got][63:48]), $signed(vec[got][47:32]), u_o, exp_u);
            end
            got = got + 1;
            if (got == N) begin
                $display("----------------------------------------");
                $display("Samples checked : %0d", N);
                $display("Mismatches      : %0d", errors);
                $display("Total cycles    : %0d (1 sample/clk)", cycles);
                if (errors == 0) $display("RESULT: PASS");
                else             $display("RESULT: FAIL");
                $finish;
            end
        end
    end

    initial begin
        #2000000;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
