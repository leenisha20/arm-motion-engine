`timescale 1ns/1ps
module tb_traj;
    integer NM = 0;

    reg                clk = 0;
    reg                rst_n = 0;
    reg                start = 0;
    reg  signed [15:0] q0 = 0, q1 = 0;
    reg         [3:0]  log2n = 0;
    wire               valid_out, last_out, busy;
    wire signed [15:0] q_o;

    traj_gen dut (.clk(clk), .rst_n(rst_n), .start(start), .q0(q0), .q1(q1),
                  .log2n(log2n), .valid_out(valid_out), .last_out(last_out),
                  .q_out(q_o), .busy(busy));

    always #5 clk = ~clk;

    reg [47:0] mv  [0:255];
    reg [15:0] exp [0:16383];
    integer i, fd, rc, total = 0, got = 0, errors = 0, cycles = 0, lasts = 0;
    reg signed [15:0] e;

    always @(posedge clk) cycles <= cycles + 1;

    always @(posedge clk) begin
        if (valid_out) begin
            e = exp[got];
            if (q_o !== e) begin
                errors = errors + 1;
                if (errors <= 10)
                    $display("MISMATCH sample %0d: got %0d exp %0d", got, q_o, e);
            end
            got = got + 1;
            if (last_out) lasts = lasts + 1;
        end
    end

    initial begin
        fd = $fopen("tb/traj_count.txt", "r");
        rc = $fscanf(fd, "%d", NM);
        $fclose(fd);
        $readmemh("tb/traj_moves.hex", mv);
        $readmemh("tb/traj_expected.hex", exp);
        $dumpfile("sim/tb_traj.vcd");
        $dumpvars(0, tb_traj);
        repeat (4) @(posedge clk);
        #1 rst_n = 1;
        for (i = 0; i < NM; i = i + 1) begin
            @(posedge clk); #1;
            q0 = mv[i][47:32]; q1 = mv[i][31:16]; log2n = mv[i][3:0];
            total = total + (1 << mv[i][3:0]) + 1;
            start = 1;
            @(posedge clk); #1 start = 0;
            while (got < total) @(posedge clk);
        end
        repeat (3) @(posedge clk);
        $display("----------------------------------------");
        $display("Moves           : %0d", NM);
        $display("Samples checked : %0d", got);
        $display("Moves finished  : %0d (last flags)", lasts);
        $display("Mismatches      : %0d", errors);
        $display("Total cycles    : %0d", cycles);
        if (errors == 0 && got == total && lasts == NM) $display("RESULT: PASS");
        else                                            $display("RESULT: FAIL");
        $finish;
    end

    initial begin
        #5000000;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
