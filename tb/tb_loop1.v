`timescale 1ns/1ps
// Closed loop: traj_gen -> pid_ctrl -> DC-motor plant (real arithmetic) -> position feedback.
// Plant constants must match model/pid_model.py: KU=400, KD_PLANT=4, DT=1e-3.
module tb_loop1;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // trajectory generator
    reg                start = 0;
    reg  signed [15:0] q0 = 0, q1 = 0;
    reg         [3:0]  log2n = 0;
    wire               tv, tl, tbusy;
    wire signed [15:0] tq;
    traj_gen tg (.clk(clk), .rst_n(rst_n), .start(start), .q0(q0), .q1(q1),
                 .log2n(log2n), .valid_out(tv), .last_out(tl), .q_out(tq), .busy(tbusy));

    // reference hold (reference stays at the last trajectory sample)
    reg signed [15:0] ref_r = 0;
    always @(posedge clk) if (tv) ref_r <= tq;

    // controller
    reg                pid_en = 0, first = 0;
    reg  signed [15:0] pos = 0;
    wire               pv;
    wire signed [15:0] u_o;
    pid_ctrl pid (.clk(clk), .rst_n(rst_n), .clear(first), .valid_in(pid_en),
                  .r_in(ref_r), .p_in(pos), .valid_out(pv), .u_out(u_o));

    // plant
    real v = 0.0, p = 0.0, uu;
    always @(posedge clk) if (pv) begin
        uu = u_o;
        v  = v + (400.0 * uu - 4.0 * v) * 1.0e-3;
        p  = p + v * 1.0e-3;
        pos <= $rtoi(p >= 0.0 ? p + 0.5 : p - 0.5);
    end

    // metrics + trace
    integer e, emax = 0, pmax = -100000, pmin = 100000, cyc = 0, fd, errors = 0;
    reg measure = 0;
    always @(posedge clk) begin
        cyc <= cyc + 1;
        if (measure) begin
            e = ref_r - pos;
            if (e < 0) e = -e;
            if (e > emax) emax = e;
            if (pos > pmax) pmax = pos;
            if (pos < pmin) pmin = pos;
            $fdisplay(fd, "%0d,%0d,%0d,%0d", cyc, ref_r, pos, u_o);
        end
    end

    task do_move(input signed [15:0] a, input signed [15:0] b, input [3:0] n);
        begin
            emax = 0; pmax = -100000; pmin = 100000;
            q0 = a; q1 = b; log2n = n;
            #1 start = 1;
            @(posedge clk); #1 start = 0;
            @(posedge clk);
            while (!(tv && tl)) @(posedge clk);
            repeat (600) @(posedge clk);
        end
    endtask

    integer fin;
    initial begin
        fd = $fopen("results/loop1_trace.csv", "w");
        $fdisplay(fd, "cycle,ref,pos,u");
        $dumpfile("sim/tb_loop1.vcd");
        $dumpvars(0, tb_loop1);
        repeat (4) @(posedge clk);
        #1 rst_n = 1;
        @(posedge clk); #1 pid_en = 1; first = 1;
        @(posedge clk); #1 first = 0;
        repeat (50) @(posedge clk);
        measure = 1;

        do_move(16'sd0, 16'sd12288, 4'd8);
        fin = ref_r - pos; if (fin < 0) fin = -fin;
        $display("Move 0 -> 12288 : max tracking err %0d LSB (%0f rad), overshoot %0d LSB, final err %0d LSB",
                 emax, emax / 8192.0, pmax - 12288, fin);
        if (emax > 1500 || fin > 5) errors = errors + 1;

        do_move(16'sd12288, 16'sd0, 4'd8);
        fin = ref_r - pos; if (fin < 0) fin = -fin;
        $display("Move 12288 -> 0 : max tracking err %0d LSB (%0f rad), overshoot %0d LSB, final err %0d LSB",
                 emax, emax / 8192.0, 0 - pmin, fin);
        if (emax > 1500 || fin > 5) errors = errors + 1;

        $fclose(fd);
        $display("----------------------------------------");
        $display("Mismatches : %0d", errors);
        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL");
        $finish;
    end

    initial begin
        #5000000;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
