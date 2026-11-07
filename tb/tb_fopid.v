`timescale 1ns/1ps
module tb_fopid;
    integer N = 0;

    reg                clk = 0;
    reg                rst_n = 0;
    reg                clear = 0;
    reg                valid_in = 0;
    reg  signed [15:0] r_in = 0, p_in = 0;
    wire               valid_out, busy;
    wire signed [15:0] u_o;

    fopid_ctrl #(.M(16)) dut (.clk(clk), .rst_n(rst_n), .clear(clear), .valid_in(valid_in),
                  .r_in(r_in), .p_in(p_in), .valid_out(valid_out), .u_out(u_o), .busy(busy));

    always #5 clk = ~clk;

    reg [63:0] vec [0:8191];
    integer i, fd, rc, errors = 0, cycles = 0, lat = 0, lat_first = 0;
    reg signed [15:0] exp_u;

    always @(posedge clk) cycles <= cycles + 1;

    initial begin
        fd = $fopen("tb/fopid_count.txt", "r");
        rc = $fscanf(fd, "%d", N);
        $fclose(fd);
        $readmemh("tb/fopid_vectors.hex", vec);
        $dumpfile("sim/tb_fopid.vcd");
        $dumpvars(0, tb_fopid);
        repeat (4) @(posedge clk);
        #1 rst_n = 1;
        for (i = 0; i < N; i = i + 1) begin
            @(posedge clk); #1;
            r_in = vec[i][63:48]; p_in = vec[i][47:32]; clear = vec[i][0];
            valid_in = 1;
            @(posedge clk); #1;
            valid_in = 0; clear = 0;
            lat = 1;
            while (!valid_out) begin @(posedge clk); #1; lat = lat + 1; end
            if (i == 0) lat_first = lat;
            exp_u = vec[i][31:16];
            if (u_o !== exp_u) begin
                errors = errors + 1;
                if (errors <= 10)
                    $display("MISMATCH #%0d r=%0d p=%0d: u got %0d exp %0d",
                             i, $signed(vec[i][63:48]), $signed(vec[i][47:32]), u_o, exp_u);
            end
        end
        $display("----------------------------------------");
        $display("Samples checked : %0d", N);
        $display("Mismatches      : %0d", errors);
        $display("Latency (clks)  : %0d", lat_first);
        $display("Total cycles    : %0d", cycles);
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
