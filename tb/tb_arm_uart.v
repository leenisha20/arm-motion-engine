`timescale 1ns/1ps
module tb_arm_uart;
    localparam CPB = 8;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // host side UART transmitter
    reg        h_start = 0;
    reg  [7:0] h_data = 0;
    wire       line, h_busy;
    uart_tx #(.CLKS_PER_BIT(CPB)) host (.clk(clk), .rst_n(rst_n), .start(h_start),
                                        .data(h_data), .tx(line), .busy(h_busy));

    reg signed [15:0] pos1 = 0, pos2 = 0;
    wire signed [15:0] r1, r2, u1, u2;
    wire v1, v2, mv, badf;
    arm_uart_top #(.CPB(CPB)) dut (.clk(clk), .rst_n(rst_n), .rx(line),
        .pos1(pos1), .pos2(pos2), .ref1(r1), .ref2(r2), .u1(u1), .u2(u2),
        .u1_valid(v1), .u2_valid(v2), .moving(mv), .bad_frame(badf));

    integer bad_cnt = 0;
    always @(posedge clk) if (badf) bad_cnt = bad_cnt + 1;

    real va = 0.0, pa = 0.0, vb = 0.0, pb = 0.0, ua, ub;
    always @(posedge clk) if (v1) begin
        ua = u1; va = va + (400.0 * ua - 4.0 * va) * 1.0e-3; pa = pa + va * 1.0e-3;
        pos1 <= $rtoi(pa >= 0.0 ? pa + 0.5 : pa - 0.5);
    end
    always @(posedge clk) if (v2) begin
        ub = u2; vb = vb + (400.0 * ub - 4.0 * vb) * 1.0e-3; pb = pb + vb * 1.0e-3;
        pos2 <= $rtoi(pb >= 0.0 ? pb + 0.5 : pb - 0.5);
    end

    wire fk_v;
    wire signed [15:0] fx, fy;
    reg  fk_go = 0;
    kin_fk fk (.clk(clk), .rst_n(rst_n), .valid_in(fk_go), .t1(pos1), .t2(pos2),
               .valid_out(fk_v), .x_out(fx), .y_out(fy));

    task send_byte(input [7:0] b);
        begin
            @(posedge clk); #1;
            while (h_busy) begin @(posedge clk); #1; end
            h_data = b; h_start = 1;
            @(posedge clk); #1 h_start = 0;
        end
    endtask
    task send_frame(input [7:0] c, input [15:0] d, input [7:0] chk_xor);
        begin
            send_byte(8'hA5); send_byte(c); send_byte(d[15:8]); send_byte(d[7:0]);
            send_byte(c ^ d[15:8] ^ d[7:0] ^ chk_xor); send_byte(8'h5A);
        end
    endtask

    integer errors = 0, t, ex, ey;
    real err_mm;
    initial begin
        $dumpfile("sim/tb_arm_uart.vcd"); $dumpvars(0, tb_arm_uart);
        repeat (4) @(posedge clk); #1 rst_n = 1;
        repeat (100) @(posedge clk);

        send_frame(8'd1, 16'h7FFF, 8'hFF);      // corrupted checksum: must be ignored
        send_frame(8'd1, 16'd6000, 8'h00);      // X
        send_frame(8'd2, 16'd3000, 8'h00);      // Y
        send_frame(8'd3, 16'd0,    8'h00);      // go
        repeat (2000) @(posedge clk);
        t = 0;
        while (mv && t < 400000) begin @(posedge clk); t = t + 1; end
        repeat (32 * 600) @(posedge clk);

        #1 fk_go = 1; @(posedge clk); #1 fk_go = 0;
        while (!fk_v) @(posedge clk);
        ex = fx - 6000; ey = fy - 3000;
        err_mm = $sqrt(ex * ex + ey * ey * 1.0) / 16384.0 / 0.55 * 1000.0;
        $display("UART target (6000,3000): reached (%0d,%0d), error %0f mm, rejected frames %0d",
                 fx, fy, err_mm, bad_cnt);
        if (err_mm > 5.0) errors = errors + 1;
        if (bad_cnt != 1) errors = errors + 1;
        $display("Mismatches : %0d", errors);
        if (errors == 0) $display("RESULT: PASS"); else $display("RESULT: FAIL");
        $finish;
    end
    initial begin #2000000000; $display("RESULT: FAIL (timeout)"); $finish; end
endmodule
