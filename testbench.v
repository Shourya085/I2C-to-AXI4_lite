`timescale 1ns / 1ps

module i2c_to_axi4lite_tb1;

    // Clock: 100 MHz 
    reg clk, rst_n;
    initial clk = 0;
    always #5 clk = ~clk;

    reg  scl_lo, sda_lo;

    wire dut_sda_o, dut_sda_oe;

    wire scl_i   = scl_lo ? 1'b0 : 1'b1;
    wire sda_bus = (sda_lo || (dut_sda_oe && !dut_sda_o))
                   ? 1'b0 : 1'b1;

    // AXI4-Lite signals 
    wire [15:0] awaddr;  wire awvalid;  reg  awready;
    wire [31:0] wdata;   wire [3:0] wstrb;
    wire        wvalid;  reg  wready;
    reg  [1:0]  bresp;   reg  bvalid;   wire bready;
    wire [15:0] araddr;  wire arvalid;  reg  arready;
    reg  [31:0] rdata;   reg  [1:0] rresp;
    reg         rvalid;  wire rready;
    wire        busy, axi_error;

    //  DUT 
    i2c_to_axi4lite #(.I2C_ADDR(7'h55)) DUT (
        .clk          (clk),
        .rst_n        (rst_n),
        .scl_i        (scl_i),
        .sda_i        (sda_bus),
        .sda_o        (dut_sda_o),
        .sda_oe       (dut_sda_oe),
        .m_axi_awaddr (awaddr),   .m_axi_awvalid(awvalid),
        .m_axi_awready(awready),
        .m_axi_wdata  (wdata),    .m_axi_wstrb  (wstrb),
        .m_axi_wvalid (wvalid),   .m_axi_wready (wready),
        .m_axi_bresp  (bresp),    .m_axi_bvalid (bvalid),
        .m_axi_bready (bready),
        .m_axi_araddr (araddr),   .m_axi_arvalid(arvalid),
        .m_axi_arready(arready),
        .m_axi_rdata  (rdata),    .m_axi_rresp  (rresp),
        .m_axi_rvalid (rvalid),   .m_axi_rready (rready),
        .busy         (busy),
        .axi_error    (axi_error)
    );

    // I2C timing: 1 MHz (500 ns half-period) 
    localparam T = 500; // ns

    // START: SDA falls while SCL high, then SCL falls
    task i2c_start;
        begin
            scl_lo=0; sda_lo=0; #T; 
            sda_lo=1;           #T;  
            scl_lo=1;           #T;  
        end
    endtask

    // STOP: SCL rises, then SDA rises while SCL high
    task i2c_stop;
        begin
            scl_lo=1; sda_lo=1; #T; 
            scl_lo=0;           #T; 
            sda_lo=0;           #T; 
            #T;                     
        end
    endtask

    task send_byte;
        input [7:0] b;
        output      nack;
        integer i;
        begin
            for (i=7; i>=0; i=i-1) begin
                scl_lo = 1;                       
                sda_lo = (b[i]==1'b0) ? 1 : 0;    
                #T;                               
                scl_lo = 0;                       
                #T;                               
                scl_lo = 1;                       
                #T;
            end
            // ACK clock: release SDA for slave to pull low
            sda_lo = 0;
            #T;
            scl_lo = 0;       
            #T;
            nack = sda_bus;   
            scl_lo = 1;       
            #T;
            
        end
    endtask

    task read_byte;
        output [7:0] b;
        input        do_nack;  
        integer i;
        begin
            b      = 8'h00;
            sda_lo = 0;        

            
            scl_lo = 0;       
            #T;                

            for (i=7; i>=0; i=i-1) begin
                scl_lo = 1;    
                #T;          
                scl_lo = 0;    
                #T;            
                b[i] = sda_bus;
               
            end
            scl_lo = 1;        
            sda_lo = do_nack;  
            #T;
            scl_lo = 0;        
            #T;
            scl_lo = 1;        
            sda_lo = 0;        
            #T;
        end
    endtask.

    reg [31:0] axi_mem;
    reg        aw_done, w_done;
    reg [15:0] saved_awaddr;
    reg [31:0] saved_wdata;

    // Address-based read return value
    function [7:0] read_val;
        input [15:0] addr;
        case (addr)
            16'hB008: read_val = 8'hC3;
            16'hD020: read_val = 8'h7E;
            default:  read_val = 8'hAA;
        endcase
    endfunction

    initial begin
        awready=0; wready=0; bvalid=0; bresp=2'b00;
        arready=0; rvalid=0; rdata=32'h0; rresp=2'b00;
        axi_mem=32'h0; aw_done=0; w_done=0;
        saved_awaddr=16'h0; saved_wdata=32'h0;
    end

    // Write address channel
    always @(posedge clk) begin
        if (!rst_n) begin awready<=0; aw_done<=0; end
        else begin
            if (awvalid && !awready && !aw_done) begin
                saved_awaddr <= awaddr;
                awready      <= 1;
            end else if (awready) begin
                awready <= 0;
                aw_done <= 1;
            end
            if (bvalid && bready) aw_done <= 0;
        end
    end

    // Write data channel
    always @(posedge clk) begin
        if (!rst_n) begin wready<=0; w_done<=0; end
        else begin
            if (wvalid && !wready && !w_done) begin
                saved_wdata <= wdata;
                axi_mem     <= wdata;
                wready      <= 1;
            end else if (wready) begin
                wready <= 0;
                w_done <= 1;
            end
            if (bvalid && bready) w_done <= 0;
        end
    end

    // Write response: assert bvalid when both channels done
    always @(posedge clk) begin
        if (!rst_n) begin bvalid<=0; end
        else begin
            if (aw_done && w_done && !bvalid) begin
                bvalid <= 1;
                bresp  <= 2'b00;
                $display("  [AXI]   Write addr=0x%08h data=0x%08h -> OKAY",
                          saved_awaddr, saved_wdata);
            end
            if (bvalid && bready) bvalid <= 0;
        end
    end

    // Read channel
    always @(posedge clk) begin
        if (!rst_n) begin arready<=0; rvalid<=0; end
        else begin
            if (arvalid && !arready && !rvalid) arready <= 1;
            else                                arready <= 0;
            if (arready) begin
                rdata  <= {24'd0, read_val(araddr)};
                rvalid <= 1;
                rresp  <= 2'b00;
                $display("  [AXI]   Read  addr=0x%08h -> returning 0x%02h",
                          araddr, read_val(araddr));
            end
            if (rvalid && rready) rvalid <= 0;
        end
    end

    // Main test sequence 
    reg       nack_flag;
    reg [7:0] rx_byte;
    integer   pass_cnt;

    initial begin
        scl_lo=0; sda_lo=0; rst_n=0; pass_cnt=0;
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(5)  @(posedge clk);
        $display("[RESET] Done");

        $display("");
        $display("[TEST 1] I2C Write -> AXI Write");
        $display("  [I2C-M] addr=0x55 W, AXI=0xA004, data=0xAB");

        i2c_start;
        send_byte(8'hAA, nack_flag);  // 0x55<<1 | W = 0xAA
        send_byte(8'hA0, nack_flag);  // AXI addr high
        send_byte(8'h04, nack_flag);  // AXI addr low
        send_byte(8'hAB, nack_flag);  // write data
        i2c_stop;

        repeat(100) @(posedge clk);

        if (axi_mem[7:0]==8'hAB && !axi_error) begin
            $display("[TEST 1] PASS"); pass_cnt=pass_cnt+1;
        end else
            $display("[TEST 1] FAIL (mem=0x%02h err=%0b)",
                      axi_mem[7:0], axi_error);

        repeat(30) @(posedge clk);

        $display("");
        $display("[TEST 2] I2C Read -> AXI Read -> I2C");
        $display("  [I2C-M] addr=0x55 W+R, AXI=0xB008");

        i2c_start;
        send_byte(8'hAA, nack_flag);   
        send_byte(8'hB0, nack_flag);  
        send_byte(8'h08, nack_flag);  
        i2c_start;                     
        send_byte(8'hAB, nack_flag);   

        scl_lo = 1;
        repeat(150) @(posedge clk);

        read_byte(rx_byte, 1'b1);     
        i2c_stop;

        $display("  [I2C-M] Received = 0x%02h", rx_byte);
        if (rx_byte==8'hC3 && !axi_error) begin
            $display("[TEST 2] PASS"); pass_cnt=pass_cnt+1;
        end else
            $display("[TEST 2] FAIL (got=0x%02h err=%0b)",
                      rx_byte, axi_error);

        repeat(30) @(posedge clk);

        $display("");
        $display("[TEST 3] I2C Write -> AXI Write (2nd cycle)");
        $display("  [I2C-M] addr=0x55 W, AXI=0xC010, data=0xF0");

        i2c_start;
        send_byte(8'hAA, nack_flag);
        send_byte(8'hC0, nack_flag);
        send_byte(8'h10, nack_flag);
        send_byte(8'hF0, nack_flag);
        i2c_stop;

        repeat(100) @(posedge clk);

        if (axi_mem[7:0]==8'hF0 && !axi_error) begin
            $display("[TEST 3] PASS"); pass_cnt=pass_cnt+1;
        end else
            $display("[TEST 3] FAIL (mem=0x%02h err=%0b)",
                      axi_mem[7:0], axi_error);

        repeat(30) @(posedge clk);
        $display("");
        $display("[TEST 4] I2C Read -> AXI Read -> I2C (2nd cycle)");
        $display("  [I2C-M] addr=0x55 W+R, AXI=0xD020 (expect 0x7E)");

        i2c_start;
        send_byte(8'hAA, nack_flag);
        send_byte(8'hD0, nack_flag);
        send_byte(8'h20, nack_flag);

        i2c_start;
        send_byte(8'hAB, nack_flag);

        scl_lo = 1;
        repeat(150) @(posedge clk);

        read_byte(rx_byte, 1'b1);
        i2c_stop;

        $display("  [I2C-M] Received = 0x%02h", rx_byte);
        if (rx_byte==8'h7E && !axi_error) begin
            $display("[TEST 4] PASS"); pass_cnt=pass_cnt+1;
        end else
            $display("[TEST 4] FAIL (expected 0x7E, got 0x%02h err=%0b)",
                      rx_byte, axi_error);

        // final result 
        repeat(20) @(posedge clk);
        $display("");
        $display("=========================================");
        if (pass_cnt==4)
            $display(" ALL TESTS PASSED (%0d/4)", pass_cnt);
        else
            $display(" %0d/4 TESTS PASSED", pass_cnt);
        $display("=========================================");
        $finish;
    end

    // Watchdog: kill if stuck
    initial begin
        #30_000_000;
        $display("WATCHDOG: timeout - simulation hung");
        $finish;
    end

endmodule
