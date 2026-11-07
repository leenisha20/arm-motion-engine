`timescale 1ns/1ps
module tb_arm;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    reg go = 0;
    reg signed [15:0] x_in = 0, y_in = 0;
    reg signed [15:0] pos1 = 0, pos2 = 0;
    wire signed [15:0] r1, r2, u1, u2;
    wire v1, v2, mv;
    arm_top dut (.clk(clk), .rst_n(rst_n), .go(go), .x_in(x_in), .y_in(y_in),
                 .pos1(pos1), .pos2(pos2), .ref1(r1), .ref2(r2), .u1(u1), .u2(u2),
                 .u1_valid(v1), .u2_valid(v2), .moving(mv));

    real va = 0.0, pa = 0.0, vb = 0.0, pb = 0.0, ua, ub;
    always @(posedge clk) if (v1) begin
        ua = u1; va = va + (400.0 * ua - 4.0 * va) * 1.0e-3; pa = pa + va * 1.0e-3;
        pos1 <= $rtoi(pa >= 0.0 ? pa + 0.5 : pa - 0.5);
    end
    always @(posedge clk) if (v2) begin
        ub = u2; vb = vb + (400.0 * ub - 4.0 * vb) * 1.0e-3; pb = pb + vb * 1.0e-3;
        pos2 <= $rtoi(pb >= 0.0 ? pb + 0.5 : pb - 0.5);
    end

    // forward kinematics check on the final joint angles
    wire fk_v;
    wire signed [15:0] fx, fy;
    reg  fk_go = 0;
    kin_fk fk (.clk(clk), .rst_n(rst_n), .valid_in(fk_go), .t1(pos1), .t2(pos2),
               .valid_out(fk_v), .x_out(fx), .y_out(fy));

    integer errors = 0, t, ex, ey, k;
    real err_mm;
    reg signed [15:0] tx [0:2], ty [0:2];

    task run_target(input signed [15:0] x, input signed [15:0] y);
        begin
            x_in = x; y_in = y;
            #1 go = 1; @(posedge clk); #1 go = 0;
            t = 0;
            while (t < 200 || (mv && t < 400000)) begin @(posedge clk); t = t + 1; end
            repeat (32 * 600) @(posedge clk);
            #1 fk_go = 1; @(posedge clk); #1 fk_go = 0;
            while (!fk_v) @(posedge clk);
            ex = fx - x; ey = fy - y;
            err_mm = $sqrt(ex * ex + ey * ey * 1.0) / 16384.0 / 0.55 * 1000.0;
            $display("Target (%0d,%0d): reached (%0d,%0d), error %0f mm (reach 550 mm), joints %0d %0d",
                     x, y, fx, fy, err_mm, pos1, pos2);
            if (err_mm > 5.0) errors = errors + 1;
        end
    endtask

    initial begin
        $dumpfile("sim/tb_arm.vcd"); $dumpvars(0, tb_arm);
        repeat (4) @(posedge clk); #1 rst_n = 1;
        repeat (100) @(posedge clk);
        run_target(16'sd6000, 16'sd3000);
        $display("Mismatches : %0d", errors);
        if (errors == 0) $display("RESULT: PASS"); else $display("RESULT: FAIL");
        $finish;
    end
    initial begin #2000000000; $display("RESULT: FAIL (timeout)"); $finish; end
endmodule
