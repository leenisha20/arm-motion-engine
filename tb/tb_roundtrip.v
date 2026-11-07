`timescale 1ns/1ps
module tb_roundtrip;
    localparam LAT = 54;               // IK 37 + FK 17
    integer N = 0;

    reg                clk = 0;
    reg                rst_n = 0;
    reg                valid_in = 0;
    reg  signed [15:0] x_in = 0, y_in = 0;
    wire               v_ik, v_fk;
    wire signed [15:0] t1, t2, x_o, y_o;

    kin_ik u_ik (.clk(clk), .rst_n(rst_n), .valid_in(valid_in),
                 .x_in(x_in), .y_in(y_in),
                 .valid_out(v_ik), .t1_out(t1), .t2_out(t2));
    kin_fk u_fk (.clk(clk), .rst_n(rst_n), .valid_in(v_ik),
                 .t1(t1), .t2(t2),
                 .valid_out(v_fk), .x_out(x_o), .y_out(y_o));

    always #5 clk = ~clk;

    reg [63:0] vec [0:4095];
    reg signed [15:0] xd [0:LAT-1];
    reg signed [15:0] yd [0:LAT-1];
    integer sent = 0, got = 0, cycles = 0, fd, rc, k;
    integer ex, ey, mag2, max2 = 0, bad = 0;
    real    sum2 = 0.0;

    initial begin
        fd = $fopen("tb/ik_count.txt", "r");
        rc = $fscanf(fd, "%d", N);
        $fclose(fd);
        $readmemh("tb/ik_vectors.hex", vec);
        $dumpfile("sim/tb_roundtrip.vcd");
        $dumpvars(0, tb_roundtrip);
        repeat (4) @(posedge clk);
        #1 rst_n = 1;
    end

    always @(posedge clk) begin
        cycles <= cycles + 1;
        if (rst_n && sent < N) begin
            x_in <= vec[sent][63:48];
            y_in <= vec[sent][47:32];
            valid_in <= 1'b1;
            sent <= sent + 1;
        end else valid_in <= 1'b0;
        xd[0] <= x_in;
        yd[0] <= y_in;
        for (k = 1; k < LAT; k = k + 1) begin
            xd[k] <= xd[k-1];
            yd[k] <= yd[k-1];
        end
    end

    always @(posedge clk) begin
        if (v_fk && got < N) begin
            ex = x_o - xd[LAT-2];
            ey = y_o - yd[LAT-2];
            mag2 = ex*ex + ey*ey;
            if (mag2 > max2) max2 = mag2;
            sum2 = sum2 + mag2;
            if (mag2 > 400) bad = bad + 1;       // > 20 LSB (~0.67 mm)
            got = got + 1;
            if (got == N) begin
                $display("----------------------------------------");
                $display("Targets checked        : %0d", N);
                $display("Max error (LSB, Q2.14) : %0f", $sqrt(max2*1.0));
                $display("Max error (mm @550mm)  : %0f", $sqrt(max2*1.0)/16384.0/0.55*1000.0);
                $display("RMS error (LSB)        : %0f", $sqrt(sum2/N));
                $display("Targets over 20 LSB    : %0d", bad);
                if (bad == 0) $display("RESULT: PASS");
                else          $display("RESULT: FAIL");
                $finish;
            end
        end
    end

    initial begin
        #1000000;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
