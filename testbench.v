`timescale 1ns / 1ps

// I2C Master + AXI4-Lite Slave TB

module i2c_to_axi4lite_tb;

    // clock
    reg clk, rst_n;
    initial clk = 0;
    always #5 clk = ~clk;

    // I2C lines (open-drain model)
    reg scl_lo, sda_lo;

    wire dut_sda_o, dut_sda_oe;

    wire scl_i   = scl_lo ? 1'b0 : 1'b1;
    wire sda_bus = (sda_lo || (dut_sda_oe && !dut_sda_o)) ? 1'b0 : 1'b1;

    // AXI signals
    wire [15:0] awaddr;  wire awvalid;  reg awready;
    wire [31:0] wdata;   wire [3:0] wstrb;
    wire wvalid;         reg wready;
    reg  [1:0] bresp;    reg bvalid;    wire bready;

    wire [15:0] araddr;  wire arvalid;  reg arready;
    reg  [31:0] rdata;   reg [1:0] rresp;
    reg rvalid;          wire rready;

    wire busy, axi_error;

    // DUT
    i2c_to_axi4lite DUT (
        .clk(clk), .rst_n(rst_n),
        .scl_i(scl_i), .sda_i(sda_bus),
        .sda_o(dut_sda_o), .sda_oe(dut_sda_oe),

        .m_axi_awaddr(awaddr), .m_axi_awvalid(awvalid), .m_axi_awready(awready),
        .m_axi_wdata(wdata),   .m_axi_wstrb(wstrb),
        .m_axi_wvalid(wvalid), .m_axi_wready(wready),
        .m_axi_bresp(bresp),   .m_axi_bvalid(bvalid), .m_axi_bready(bready),

        .m_axi_araddr(araddr), .m_axi_arvalid(arvalid), .m_axi_arready(arready),
        .m_axi_rdata(rdata),   .m_axi_rresp(rresp),
        .m_axi_rvalid(rvalid), .m_axi_rready(rready),

        .busy(busy), .axi_error(axi_error)
    );

    // timing
    localparam T = 500;

    // I2C helpers 

    task i2c_start;
    begin
        scl_lo=0; sda_lo=0; #T;
        sda_lo=1; #T;
        scl_lo=1; #T;
    end
    endtask

    task i2c_stop;
    begin
        scl_lo=1; sda_lo=1; #T;
        scl_lo=0; #T;
        sda_lo=0; #T;
        #T;
    end
    endtask

    task send_byte;
        input [7:0] b;
        output nack;
        integer i;
    begin
        for (i=7;i>=0;i=i-1) begin
            scl_lo=1;
            sda_lo = (b[i]==0) ? 1:0;
            #T;
            scl_lo=0; #T;
            scl_lo=1; #T;
        end

        // ACK
        sda_lo=0;
        #T;
        scl_lo=0; #T;
        nack = sda_bus;
        scl_lo=1; #T;
    end
    endtask

    task read_byte;
        output [7:0] b;
        input do_nack;
        integer i;
    begin
        b = 0;
        sda_lo = 0;

        // make sure SCL starts high
        scl_lo = 0;
        #T;

        for (i=7;i>=0;i=i-1) begin
            scl_lo=1; #T;
            scl_lo=0; #T;
            b[i] = sda_bus;
        end

        // send ACK/NACK
        scl_lo=1;
        sda_lo=do_nack;
        #T;
        scl_lo=0; #T;
        scl_lo=1;
        sda_lo=0;
        #T;
    end
    endtask

    // AXI slave 

    reg [31:0] axi_mem;
    reg aw_done, w_done;
    reg [15:0] saved_addr;
    reg [31:0] saved_data;

    function [7:0] read_val;
        input [15:0] addr;
        case(addr)
            16'hB008: read_val = 8'hC3;
            16'hD020: read_val = 8'h7E;
            default:  read_val = 8'hAA;
        endcase
    endfunction

    initial begin
        awready=0; wready=0; bvalid=0;
        arready=0; rvalid=0;
        axi_mem=0; aw_done=0; w_done=0;
    end

    always @(posedge clk) begin
        if (!rst_n) begin awready<=0; aw_done<=0; end
        else begin
            if (awvalid && !aw_done) begin
                saved_addr <= awaddr;
                awready <= 1;
            end else awready <= 0;

            if (awready) aw_done <= 1;
            if (bvalid && bready) aw_done <= 0;
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin wready<=0; w_done<=0; end
        else begin
            if (wvalid && !w_done) begin
                saved_data <= wdata;
                axi_mem <= wdata;
                wready <= 1;
            end else wready <= 0;

            if (wready) w_done <= 1;
            if (bvalid && bready) w_done <= 0;
        end
    end

    always @(posedge clk) begin
        if (!rst_n) bvalid<=0;
        else begin
            if (aw_done && w_done && !bvalid) begin
                bvalid <= 1;
                $display("[AXI] write addr=0x%h data=0x%h", saved_addr, saved_data);
            end
            if (bvalid && bready) bvalid <= 0;
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin arready<=0; rvalid<=0; end
        else begin
            if (arvalid && !rvalid) arready<=1;
            else arready<=0;

            if (arready) begin
                rdata  <= {24'd0, read_val(araddr)};
                rvalid <= 1;
                $display("[AXI] read addr=0x%h", araddr);
            end

            if (rvalid && rready) rvalid<=0;
        end
    end

    // tests 

    reg nack;
    reg [7:0] data;
    integer pass;

    initial begin
        scl_lo=0; sda_lo=0; rst_n=0; pass=0;

        repeat(10) @(posedge clk);
        rst_n=1;

        // ---- Test 1 (write)
        i2c_start;
        send_byte(8'hAA,nack);
        send_byte(8'hA0,nack);
        send_byte(8'h04,nack);
        send_byte(8'hAB,nack);
        i2c_stop;

        repeat(100) @(posedge clk);

        if (axi_mem[7:0]==8'hAB) begin
            $display("TEST1 PASS"); pass++;
        end else $display("TEST1 FAIL");

        // Test 2 (read)
        i2c_start;
        send_byte(8'hAA,nack);
        send_byte(8'hB0,nack);
        send_byte(8'h08,nack);

        i2c_start;
        send_byte(8'hAB,nack);

        scl_lo=1;
        repeat(150) @(posedge clk);

        read_byte(data,1);
        i2c_stop;

        if (data==8'hC3) begin
            $display("TEST2 PASS"); pass++;
        end else $display("TEST2 FAIL");

        // final output
        $display("PASS COUNT = %0d", pass);
        $finish;
    end

endmodule
