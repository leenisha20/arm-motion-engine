`timescale 1ns/1ps
module tb_periph;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // ---------- encoder ----------
    reg a = 0, b = 0;
    wire signed [15:0] pos;
    wire dir, qerr;
    quad_dec qd (.clk(clk), .rst_n(rst_n), .a(a), .b(b), .pos(pos), .dir(dir), .err(qerr));

    // ---------- PWM ----------
    reg signed [15:0] cmd = 0;
    wire hi, lo;
    pwm_gen #(.PERIOD(256), .DEAD(4)) pw (.clk(clk), .rst_n(rst_n), .cmd(cmd), .hi(hi), .lo(lo));

    integer errors = 0, i;
    integer hi_cnt, tot, overlap;
    integer duty_pct;

    // forward sequence 00 -> 01 -> 11 -> 10 -> 00 ; reverse is the opposite
    task step_fwd; begin
        {a,b} = 2'b01; repeat (6) @(posedge clk);
        {a,b} = 2'b11; repeat (6) @(posedge clk);
        {a,b} = 2'b10; repeat (6) @(posedge clk);
        {a,b} = 2'b00; repeat (6) @(posedge clk);
    end endtask
    task step_rev; begin
        {a,b} = 2'b10; repeat (6) @(posedge clk);
        {a,b} = 2'b11; repeat (6) @(posedge clk);
        {a,b} = 2'b01; repeat (6) @(posedge clk);
        {a,b} = 2'b00; repeat (6) @(posedge clk);
    end endtask

    task measure(input integer periods);
        integer c;
        begin
            hi_cnt = 0; tot = 0; overlap = 0;
            for (c = 0; c < periods * 512; c = c + 1) begin
                @(posedge clk);
                tot = tot + 1;
                if (hi) hi_cnt = hi_cnt + 1;
                if (hi && lo) overlap = overlap + 1;
            end
            duty_pct = (hi_cnt * 100) / tot;
        end
    endtask

    initial begin
        $dumpfile("sim/tb_periph.vcd");
        $dumpvars(0, tb_periph);
        repeat (4) @(posedge clk);
        #1 rst_n = 1;
        repeat (4) @(posedge clk);

        // --- encoder: 10 forward cycles = +40 counts, then 25 reverse cycles = -100 => -60
        for (i = 0; i < 10; i = i + 1) step_fwd;
        if (pos !== 16'sd40) begin errors = errors + 1; $display("ENC FAIL fwd: pos=%0d exp 40", pos); end
        for (i = 0; i < 25; i = i + 1) step_rev;
        if (pos !== -16'sd60) begin errors = errors + 1; $display("ENC FAIL rev: pos=%0d exp -60", pos); end

        // illegal jump 00 -> 11 must raise err and not move pos
        begin : illegal
            integer seen; integer p0;
            seen = 0; p0 = pos;
            {a,b} = 2'b11;
            repeat (6) begin @(posedge clk); if (qerr) seen = 1; end
            if (!seen) begin errors = errors + 1; $display("ENC FAIL: illegal transition not flagged"); end
            if (pos !== p0) begin errors = errors + 1; $display("ENC FAIL: pos moved on illegal"); end
        end

        // --- PWM duty checks
        cmd = 0;      repeat (600) @(posedge clk); measure(4);
        $display("PWM cmd=0     : duty %0d%% (expect ~46-50 after dead time), overlap %0d", duty_pct, overlap);
        if (duty_pct < 44 || duty_pct > 51 || overlap != 0) errors = errors + 1;

        cmd = 16'sd16000; repeat (600) @(posedge clk); measure(4);
        $display("PWM cmd=+16000: duty %0d%% (expect ~69-75), overlap %0d", duty_pct, overlap);
        if (duty_pct < 67 || duty_pct > 76 || overlap != 0) errors = errors + 1;

        cmd = -16'sd16000; repeat (600) @(posedge clk); measure(4);
        $display("PWM cmd=-16000: duty %0d%% (expect ~22-28), overlap %0d", duty_pct, overlap);
        if (duty_pct < 20 || duty_pct > 29 || overlap != 0) errors = errors + 1;

        $display("----------------------------------------");
        $display("Mismatches : %0d", errors);
        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL");
        $finish;
    end

    initial begin
        #50000000;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
