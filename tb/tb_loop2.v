`timescale 1ns/1ps
// Hardware loop comparison: same reference, same plant constants, PID vs FO-PID.
// One controller sample every TICK clocks (FO-PID needs 18 clocks per sample).
module tb_loop2;
    localparam TICK = 32;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg                start = 0;
    reg  signed [15:0] q0 = 0, q1 = 0;
    reg         [3:0]  log2n = 0;
    wire               tv, tl, tbusy;
    wire signed [15:0] tq;
    traj_gen tg (.clk(clk), .rst_n(rst_n), .start(start), .q0(q0), .q1(q1),
                 .log2n(log2n), .valid_out(tv), .last_out(tl), .q_out(tq), .busy(tbusy));

    reg signed [15:0] ref_r = 0;
    // sample tick
    reg [7:0] tc = 0;
    reg       tick = 0, first = 0;
    always @(posedge clk) begin
        tick <= 1'b0;
        if (rst_n) begin
            if (tc == TICK - 1) begin tc <= 0; tick <= 1'b1; end
            else tc <= tc + 1;
        end
    end

    // store trajectory, replay one sample per controller tick
    reg signed [15:0] rmem [0:2047];
    integer wr = 0, rd = 0;
    always @(posedge clk) begin
        if (tv) begin rmem[wr] <= tq; wr <= wr + 1; end
        if (tick && rd < wr) begin ref_r <= rmem[rd]; rd <= rd + 1; end
    end

    // controller A: PID
    reg  signed [15:0] posA = 0, posB = 0;
    wire               pvA, pvB, busyB;
    wire signed [15:0] uA, uB;
    pid_ctrl #(.USE_OVR(1), .KP_OVR(4096), .KI_OVR(0), .KD_OVR(614400)) ca (.clk(clk), .rst_n(rst_n), .clear(first), .valid_in(tick),
                   .r_in(ref_r), .p_in(posA), .valid_out(pvA), .u_out(uA));
    fopid_ctrl #(.M(16)) cb (.clk(clk), .rst_n(rst_n), .clear(first), .valid_in(tick),
                   .r_in(ref_r), .p_in(posB), .valid_out(pvB), .u_out(uB), .busy(busyB));

    // plants
    real vA = 0.0, pA = 0.0, vB = 0.0, pB = 0.0, ua, ub;
    always @(posedge clk) if (pvA) begin
        ua = uA;
        vA = vA + (400.0 * ua - 4.0 * vA) * 1.0e-3;
        pA = pA + vA * 1.0e-3;
        posA <= $rtoi(pA >= 0.0 ? pA + 0.5 : pA - 0.5);
    end
    always @(posedge clk) if (pvB) begin
        ub = uB;
        vB = vB + (400.0 * ub - 4.0 * vB) * 1.0e-3;
        pB = pB + vB * 1.0e-3;
        posB <= $rtoi(pB >= 0.0 ? pB + 0.5 : pB - 0.5);
    end

    // metrics (sampled once per tick)
    integer eA, eB, maxA = 0, maxB = 0, pmaxA = -100000, pmaxB = -100000,
            pminA = 100000, pminB = 100000, n = 0, fd, errors = 0, cyc = 0;
    real    sA = 0.0, sB = 0.0;
    reg     measure = 0;
    always @(posedge clk) begin
        cyc <= cyc + 1;
        if (tick && measure) begin
            eA = ref_r - posA; eB = ref_r - posB;
            sA = sA + eA * eA; sB = sB + eB * eB; n = n + 1;
            if (eA < 0) eA = -eA;
            if (eB < 0) eB = -eB;
            if (eA > maxA) maxA = eA;
            if (eB > maxB) maxB = eB;
            if (posA > pmaxA) pmaxA = posA;
            if (posB > pmaxB) pmaxB = posB;
            if (posA < pminA) pminA = posA;
            if (posB < pminB) pminB = posB;
            $fdisplay(fd, "%0d,%0d,%0d,%0d", n, ref_r, posA, posB);
        end
    end

    task do_move(input signed [15:0] a, input signed [15:0] b, input [3:0] nn, input integer target);
        integer fa, fb;
        begin
            maxA = 0; maxB = 0; pmaxA = -100000; pmaxB = -100000;
            pminA = 100000; pminB = 100000; sA = 0.0; sB = 0.0; n = 0;
            q0 = a; q1 = b; log2n = nn;
            #1 start = 1;
            @(posedge clk); #1 start = 0;
            @(posedge clk);
            while (!(tv && tl)) @(posedge clk);
            repeat (TICK * 600) @(posedge clk);
            fa = ref_r - posA; if (fa < 0) fa = -fa;
            fb = ref_r - posB; if (fb < 0) fb = -fb;
            $display("Move -> %0d | PID   : max err %0d LSB, rms %0f, overshoot %0d, final %0d",
                     target, maxA, $sqrt(sA / n), (b > a) ? pmaxA - target : target - pminA, fa);
            $display("Move -> %0d | FO-PID: max err %0d LSB, rms %0f, overshoot %0d, final %0d",
                     target, maxB, $sqrt(sB / n), (b > a) ? pmaxB - target : target - pminB, fb);
            if (maxA > 1500 || maxB > 1500 || fa > 10 || fb > 10) errors = errors + 1;
        end
    endtask

    initial begin
        fd = $fopen("results/loop2_trace.csv", "w");
        $fdisplay(fd, "n,ref,pid,fopid");
        $dumpfile("sim/tb_loop2.vcd");
        $dumpvars(0, tb_loop2);
        repeat (4) @(posedge clk);
        #1 rst_n = 1;
        first = 1; repeat (TICK + 2) @(posedge clk); #1 first = 0;
        measure = 1;
        do_move(16'sd0, 16'sd12288, 4'd8, 12288);
        do_move(16'sd12288, 16'sd0, 4'd8, 0);
        $fclose(fd);
        $display("----------------------------------------");
        $display("Mismatches : %0d", errors);
        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL");
        $finish;
    end

    initial begin
        #200000000;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
