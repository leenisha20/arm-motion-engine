`timescale 1ns/1ps
module tb_uart;
    localparam CPB = 8;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  tx_start = 0;
    reg  [7:0] tx_data = 0;
    wire line, tx_busy;
    uart_tx #(.CLKS_PER_BIT(CPB)) utx (.clk(clk), .rst_n(rst_n), .start(tx_start),
                                       .data(tx_data), .tx(line), .busy(tx_busy));

    wire rx_valid, rx_ferr;
    wire [7:0] rx_data;
    uart_rx #(.CLKS_PER_BIT(CPB)) urx (.clk(clk), .rst_n(rst_n), .rx(line),
                                       .valid(rx_valid), .data(rx_data), .frame_err(rx_ferr));

    wire cmd_valid, bad_frame;
    wire [7:0] cmd;
    wire [15:0] cdata;
    cmd_parser cp (.clk(clk), .rst_n(rst_n), .byte_valid(rx_valid), .byte_in(rx_data),
                   .cmd_valid(cmd_valid), .cmd(cmd), .data(cdata), .bad_frame(bad_frame));

    integer errors = 0, good = 0, bad = 0, bytes = 0;
    reg [7:0]  last_cmd;
    reg [15:0] last_data;
    reg [7:0]  last_byte;

    always @(posedge clk) begin
        if (rx_valid) begin bytes = bytes + 1; last_byte = rx_data; end
        if (cmd_valid) begin good = good + 1; last_cmd = cmd; last_data = cdata; end
        if (bad_frame) bad = bad + 1;
        if (rx_ferr) begin errors = errors + 1; $display("UNEXPECTED frame error"); end
    end

    task send_byte(input [7:0] b);
        begin
            @(posedge clk); #1;
            while (tx_busy) begin @(posedge clk); #1; end
            tx_data = b; tx_start = 1;
            @(posedge clk); #1 tx_start = 0;
        end
    endtask

    task send_frame(input [7:0] c, input [15:0] d, input [7:0] chk_xor, input [7:0] trailer);
        begin
            send_byte(8'hA5);
            send_byte(c);
            send_byte(d[15:8]);
            send_byte(d[7:0]);
            send_byte(c ^ d[15:8] ^ d[7:0] ^ chk_xor);
            send_byte(trailer);
        end
    endtask

    task wait_idle;
        begin
            repeat (4) @(posedge clk);
            while (tx_busy) @(posedge clk);
            repeat (CPB * 4) @(posedge clk);
        end
    endtask

    initial begin
        $dumpfile("sim/tb_uart.vcd");
        $dumpvars(0, tb_uart);
        repeat (4) @(posedge clk);
        #1 rst_n = 1;
        repeat (10) @(posedge clk);

        // raw byte loopback check
        send_byte(8'h3C); wait_idle;
        if (last_byte !== 8'h3C || bytes != 1) begin errors = errors + 1; $display("LOOPBACK FAIL: got %h", last_byte); end

        // good frame 1
        send_frame(8'h01, 16'h1234, 8'h00, 8'h5A); wait_idle;
        if (good != 1 || last_cmd !== 8'h01 || last_data !== 16'h1234) begin
            errors = errors + 1; $display("FRAME 1 FAIL good=%0d cmd=%h data=%h", good, last_cmd, last_data);
        end

        // bad checksum -> must be rejected
        send_frame(8'h02, 16'hBEEF, 8'hFF, 8'h5A); wait_idle;
        if (good != 1 || bad != 1) begin errors = errors + 1; $display("BAD-CHK FAIL good=%0d bad=%0d", good, bad); end

        // garbage bytes before a good frame
        send_byte(8'h00); send_byte(8'hFF); send_byte(8'h77);
        send_frame(8'h03, 16'h8001, 8'h00, 8'h5A); wait_idle;
        if (good != 2 || last_cmd !== 8'h03 || last_data !== 16'h8001) begin
            errors = errors + 1; $display("FRAME 3 FAIL good=%0d cmd=%h data=%h", good, last_cmd, last_data);
        end

        // wrong trailer -> rejected
        send_frame(8'h04, 16'h0042, 8'h00, 8'h00); wait_idle;
        if (good != 2 || bad != 2) begin errors = errors + 1; $display("BAD-TRAILER FAIL good=%0d bad=%0d", good, bad); end

        $display("----------------------------------------");
        $display("Good frames: %0d (expect 2), rejected: %0d (expect 2)", good, bad);
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
