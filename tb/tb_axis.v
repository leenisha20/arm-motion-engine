`timescale 1ns/1ps
module tb_axis;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    reg start = 0;
    reg signed [15:0] q0 = 0, q1 = 0, pos = 0;
    reg [3:0] log2n = 0;
    wire signed [15:0] ref_o, u_o;
    wire uv, mv;
    joint_axis ax (.clk(clk), .rst_n(rst_n), .start(start), .q0(q0), .q1(q1), .log2n(log2n),
                   .pos_in(pos), .ref_out(ref_o), .u_out(u_o), .u_valid(uv), .moving(mv));

    real v = 0.0, p = 0.0, uu;
    always @(posedge clk) if (uv) begin
        uu = u_o;
        v = v + (400.0 * uu - 4.0 * v) * 1.0e-3;
        p = p + v * 1.0e-3;
        pos <= $rtoi(p >= 0.0 ? p + 0.5 : p - 0.5);
    end

    integer e, emax = 0, fin, errors = 0, t;
    always @(posedge clk) if (mv) begin
        e = ref_o - pos; if (e < 0) e = -e;
        if (e > emax) emax = e;
    end

    initial begin
        $dumpfile("sim/tb_axis.vcd"); $dumpvars(0, tb_axis);
        repeat (4) @(posedge clk); #1 rst_n = 1;
        repeat (100) @(posedge clk);
        q0 = 0; q1 = 12288; log2n = 4'd8;
        #1 start = 1; @(posedge clk); #1 start = 0;
        t = 0;
        while (t < 200000 && !(t > 100 && !mv)) begin @(posedge clk); t = t + 1; end
        repeat (32 * 600) @(posedge clk);
        fin = ref_o - pos; if (fin < 0) fin = -fin;
        $display("Axis move 0 -> 12288: max tracking err %0d LSB, final err %0d LSB", emax, fin);
        if (fin > 10 || emax > 1500) errors = errors + 1;
        $display("Mismatches : %0d", errors);
        if (errors == 0) $display("RESULT: PASS"); else $display("RESULT: FAIL");
        $finish;
    end
    initial begin #400000000; $display("RESULT: FAIL (timeout)"); $finish; end
endmodule
