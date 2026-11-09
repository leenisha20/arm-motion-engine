`timescale 1ns/1ps
module tb_pair;
    localparam DELAY = 500;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    reg go = 0;
    reg signed [15:0] x_in = 0, y_in = 0;
    reg signed [15:0] p1a = 0, p2a = 0, p1b = 0, p2b = 0;
    wire signed [15:0] r1a, r2a, r1b, r2b, u1a, u2a, u1b, u2b;
    wire v1a, v2a, v1b, v2b, mv;
    arm_pair #(.DELAY(DELAY)) dut (.clk(clk), .rst_n(rst_n), .go(go), .x_in(x_in), .y_in(y_in),
        .pos1a(p1a), .pos2a(p2a), .pos1b(p1b), .pos2b(p2b),
        .ref1a(r1a), .ref2a(r2a), .ref1b(r1b), .ref2b(r2b),
        .u1a(u1a), .u2a(u2a), .u1b(u1b), .u2b(u2b),
        .u1a_v(v1a), .u2a_v(v2a), .u1b_v(v1b), .u2b_v(v2b), .moving(mv));

    real w1=0.0, q1=0.0, w2=0.0, q2=0.0, w3=0.0, q3=0.0, w4=0.0, q4=0.0, ua, ub, uc, ud;
    always @(posedge clk) if (v1a) begin ua = u1a; w1 = w1 + (400.0*ua - 4.0*w1)*1.0e-3; q1 = q1 + w1*1.0e-3;
        p1a <= $rtoi(q1 >= 0.0 ? q1 + 0.5 : q1 - 0.5); end
    always @(posedge clk) if (v2a) begin ub = u2a; w2 = w2 + (400.0*ub - 4.0*w2)*1.0e-3; q2 = q2 + w2*1.0e-3;
        p2a <= $rtoi(q2 >= 0.0 ? q2 + 0.5 : q2 - 0.5); end
    always @(posedge clk) if (v1b) begin uc = u1b; w3 = w3 + (400.0*uc - 4.0*w3)*1.0e-3; q3 = q3 + w3*1.0e-3;
        p1b <= $rtoi(q3 >= 0.0 ? q3 + 0.5 : q3 - 0.5); end
    always @(posedge clk) if (v2b) begin ud = u2b; w4 = w4 + (400.0*ud - 4.0*w4)*1.0e-3; q4 = q4 + w4*1.0e-3;
        p2b <= $rtoi(q4 >= 0.0 ? q4 + 0.5 : q4 - 0.5); end

    wire fva, fvb;
    wire signed [15:0] fxa, fya, fxb, fyb;
    reg fk_go = 0;
    kin_fk fka (.clk(clk), .rst_n(rst_n), .valid_in(fk_go), .t1(p1a), .t2(p2a),
                .valid_out(fva), .x_out(fxa), .y_out(fya));
    kin_fk fkb (.clk(clk), .rst_n(rst_n), .valid_in(fk_go), .t1(p1b), .t2(p2b),
                .valid_out(fvb), .x_out(fxb), .y_out(fyb));

    // start-time lag between the two IK results
    integer ta = -1, tb = -1;
    always @(posedge clk) begin
        if (dut.a.ik_v && ta < 0) ta = $time;
        if (dut.b.ik_v && tb < 0) tb = $time;
    end

    integer errors = 0, t, ex, ey, lag;
    real ea, eb;
    // log joint angles every 200 clocks for the animation
    integer lc = 0, fdp;
    initial begin
        fdp = $fopen("results/pair_trace.csv", "w");
        $fdisplay(fdp, "clk,p1a,p2a,p1b,p2b");
    end
    always @(posedge clk) begin
        lc <= lc + 1;
        if (rst_n && (lc % 200 == 0)) $fdisplay(fdp, "%0d,%0d,%0d,%0d,%0d", lc, p1a, p2a, p1b, p2b);
    end

    initial begin
        $dumpfile("sim/tb_pair.vcd"); $dumpvars(0, tb_pair);
        repeat (4) @(posedge clk); #1 rst_n = 1;
        repeat (100) @(posedge clk);
        x_in = 6000; y_in = 3000;
        #1 go = 1; @(posedge clk); #1 go = 0;
        repeat (2000) @(posedge clk);
        t = 0;
        while (mv && t < 400000) begin @(posedge clk); t = t + 1; end
        repeat (32 * 600) @(posedge clk);
        #1 fk_go = 1; @(posedge clk); #1 fk_go = 0;
        while (!fva) @(posedge clk);
        ex = fxa - 6000; ey = fya - 3000;
        ea = $sqrt(ex*ex + ey*ey*1.0) / 16384.0 / 0.55 * 1000.0;
        ex = fxb - 4500; ey = fyb - 4000;
        eb = $sqrt(ex*ex + ey*ey*1.0) / 16384.0 / 0.55 * 1000.0;
        lag = (tb - ta) / 10;
        $display("Arm A target (6000,3000): reached (%0d,%0d), error %0f mm", fxa, fya, ea);
        $display("Arm B target (4500,4000): reached (%0d,%0d), error %0f mm", fxb, fyb, eb);
        $display("Start lag B after A: %0d clocks (expected %0d)", lag, DELAY + 1);
        if (ea > 5.0 || eb > 5.0) errors = errors + 1;
        if (lag != DELAY + 1) errors = errors + 1;
        $fclose(fdp);
        $display("Mismatches : %0d", errors);
        if (errors == 0) $display("RESULT: PASS"); else $display("RESULT: FAIL");
        $finish;
    end
    initial begin #2000000000; $display("RESULT: FAIL (timeout)"); $finish; end
endmodule
