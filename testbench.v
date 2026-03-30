`timescale 1ns/1ps

module i2c_to_axi4lite_tb;

  // Clock / Reset
  reg clk, rst_n;
  initial clk = 0;
  always #5 clk = ~clk; // 100MHz

  // AXI
  reg  [4:0]  s_awaddr;   reg  s_awvalid;  wire s_awready;
  reg  [31:0] s_wdata;    reg  s_wvalid;   wire s_wready;
  wire [1:0]  s_bresp;    wire s_bvalid;   reg  s_bready;

  reg  [4:0]  s_araddr;   reg  s_arvalid;  wire s_arready;
  wire [31:0] s_rdata;    wire [1:0] s_rresp;
  wire        s_rvalid;   reg  s_rready;

  // I2C
  wire scl;
  wire sda;
  reg  tb_sda_drive;
  assign sda = tb_sda_drive ? 1'b0 : 1'bz;
  pullup(sda);

  // DUT
  i2c_to_axi4lite #(
    .CLK_FREQ(100_000_000),
    .I2C_FREQ(5_000_000)
  ) DUT (
    .clk(clk), .rst_n(rst_n),
    .s_awaddr(s_awaddr), .s_awvalid(s_awvalid), .s_awready(s_awready),
    .s_wdata(s_wdata),   .s_wvalid(s_wvalid),   .s_wready(s_wready),
    .s_bresp(s_bresp),   .s_bvalid(s_bvalid),   .s_bready(s_bready),
    .s_araddr(s_araddr), .s_arvalid(s_arvalid), .s_arready(s_arready),
    .s_rdata(s_rdata),   .s_rresp(s_rresp),     .s_rvalid(s_rvalid),
    .s_rready(s_rready),
    .scl(scl), .sda(sda)
  );

  reg       slave_do_ack;
  reg [7:0] slave_tx_byte;

  localparam SL_IDLE        = 3'd0;
  localparam SL_RX_ADDR     = 3'd1;
  localparam SL_ACK_ADDR_0  = 3'd2;
  localparam SL_ACK_ADDR_1  = 3'd3;
  localparam SL_RX_DATA     = 3'd4;
  localparam SL_ACK_DATA_0  = 3'd5;
  localparam SL_ACK_DATA_1  = 3'd6;
  localparam SL_TX_DATA     = 3'd7;

  reg [2:0] sl_state;
  reg [2:0] sl_bit;
  reg [7:0] sl_rx_shift;
  reg [7:0] sl_tx_shift;
  reg       sl_rw;

  // START detect
  always @(negedge sda) begin
    if (rst_n && scl === 1'b1) begin
      $display("[BUS] START at time %0t", $time);
      sl_state     <= SL_RX_ADDR;
      sl_bit       <= 3'd7;
      sl_rx_shift  <= 8'h00;
      tb_sda_drive <= 1'b0;
    end
  end

  // STOP detect
  always @(posedge sda) begin
    if (rst_n && scl === 1'b1) begin
      $display("[BUS] STOP  at time %0t", $time);
      sl_state     <= SL_IDLE;
      tb_sda_drive <= 1'b0;
    end
  end

  // Sample on SCL rising
  always @(posedge scl or negedge rst_n) begin
    if (!rst_n) begin
      sl_state     <= SL_IDLE;
      sl_bit       <= 3'd7;
      sl_rx_shift  <= 8'h00;
      sl_tx_shift  <= 8'h00;
      sl_rw        <= 1'b0;
      tb_sda_drive <= 1'b0;
    end else begin
      case (sl_state)
        SL_RX_ADDR: begin
          sl_rx_shift[sl_bit] <= sda;
          if (sl_bit == 0) begin
            sl_rw <= sda;
            $display("[SLAVE] Addr byte = 0x%02h (%s)",
                     {sl_rx_shift[7:1], sda}, (sda ? "R" : "W"));
            sl_state <= SL_ACK_ADDR_0;
          end else begin
            sl_bit <= sl_bit - 1;
          end
        end

        SL_RX_DATA: begin
          sl_rx_shift[sl_bit] <= sda;
          if (sl_bit == 0) begin
            $display("[SLAVE] Data byte received = 0x%02h -> ACK",
                     {sl_rx_shift[7:1], sda});
            sl_state <= SL_ACK_DATA_0;
          end else begin
            sl_bit <= sl_bit - 1;
          end
        end

        default: ;
      endcase
    end
  end

  // Drive on SCL falling
  always @(negedge scl or negedge rst_n) begin
    if (!rst_n) begin
      tb_sda_drive <= 1'b0;
    end else begin
      case (sl_state)
        SL_ACK_ADDR_0: begin
          if (slave_do_ack) begin
            tb_sda_drive <= 1'b1; // ACK
            $display("[SLAVE] -> ACK");
          end else begin
            tb_sda_drive <= 1'b0; // NACK
            $display("[SLAVE] -> NACK");
          end
          sl_state <= SL_ACK_ADDR_1;
        end

        SL_ACK_ADDR_1: begin
          tb_sda_drive <= 1'b0; // release
          if (!slave_do_ack) begin
            sl_state <= SL_IDLE;
          end else if (sl_rw == 1'b0) begin
            sl_state    <= SL_RX_DATA;
            sl_bit      <= 3'd7;
            sl_rx_shift <= 8'h00;
          end else begin
            sl_state    <= SL_TX_DATA;
            sl_bit      <= 3'd7;
            sl_tx_shift <= slave_tx_byte;
            $display("[SLAVE] Sending data byte 0x%02h", slave_tx_byte);

            tb_sda_drive <= (slave_tx_byte[7] == 1'b0);
            sl_tx_shift  <= {slave_tx_byte[6:0], 1'b0};
            sl_bit       <= 3'd6;
          end
        end

        SL_ACK_DATA_0: begin
          tb_sda_drive <= 1'b1; // ACK
          sl_state <= SL_ACK_DATA_1;
        end

        SL_ACK_DATA_1: begin
          tb_sda_drive <= 1'b0; // release
          sl_state <= SL_IDLE;
        end

        SL_TX_DATA: begin
          tb_sda_drive <= (sl_tx_shift[7] == 1'b0);
          sl_tx_shift  <= {sl_tx_shift[6:0], 1'b0};
          if (sl_bit == 0) begin
            tb_sda_drive <= 1'b0;
            sl_state <= SL_IDLE;
          end else begin
            sl_bit <= sl_bit - 1;
          end
        end

        default: ;
      endcase
    end
  end

  task cpu_write;
    input [4:0]  addr;
    input [31:0] data;
    begin
   
      @(posedge clk); #1;
      s_awaddr  = addr;
      s_awvalid = 1;
      while (!s_awready) @(posedge clk);
      @(posedge clk); #1;
      s_awvalid = 0;

    
      s_wdata  = data;
      s_wvalid = 1;
      while (!s_wready) @(posedge clk);
      @(posedge clk); #1;
      s_wvalid = 0;

      s_bready = 1;
      while (!s_bvalid) @(posedge clk);
      @(posedge clk); #1;
      s_bready = 0;
    end
  endtask

  task cpu_read;
    input  [4:0]  addr;
    output [31:0] data;
    begin
      @(posedge clk); #1;
      s_araddr  = addr; s_arvalid = 1; s_rready = 1;
      while (!s_arready) @(posedge clk);
      @(posedge clk); #1 s_arvalid = 0;
      while (!s_rvalid) @(posedge clk);
      data = s_rdata;
      @(posedge clk); #1 s_rready = 0;
    end
  endtask

  task wait_done;
    reg [31:0] st;
    begin
      st = 0;
      while (st[1] == 0) begin
        repeat(20) @(posedge clk);
        cpu_read(5'h10, st);
      end
    end
  endtask

  reg [31:0] rval;

  initial begin
    rst_n=0;
    s_awaddr=0; s_awvalid=0;
    s_wdata=0;  s_wvalid=0;
    s_bready=0;
    s_araddr=0; s_arvalid=0;
    s_rready=0;

    tb_sda_drive=0;
    slave_do_ack=1;
    slave_tx_byte=8'h00;

    repeat(20) @(posedge clk);
    rst_n=1;
    repeat(10) @(posedge clk);
    $display("[RESET] Done at time %0t", $time);

    // TEST 1
    $display("");
    $display("[TEST 1] WRITE slave=0x55 data=0xAB");
    slave_do_ack  = 1;
    slave_tx_byte = 8'h00;

    cpu_write(5'h00, 32'h0000_0055);
    cpu_write(5'h08, 32'h0000_00AB);
    cpu_write(5'h0C, 32'h0000_0001);

    wait_done;
    cpu_read(5'h10, rval);
    $display("[TEST 1] STATUS=0x%08h", rval);

    // TEST 2
    $display("");
    $display("[TEST 2] READ slave=0x55 expect=0xC3");
    slave_do_ack  = 1;
    slave_tx_byte = 8'hC3;

    cpu_write(5'h00, 32'h0000_0055);
    cpu_write(5'h0C, 32'h0000_0003);

    wait_done;
    cpu_read(5'h14, rval);
    $display("[TEST 2] RDATA=0x%08h", rval);

    // TEST 3
    $display("");
    $display("[TEST 3] NACK slave=0x77");
    slave_do_ack = 0;

    cpu_write(5'h00, 32'h0000_0077);
    cpu_write(5'h0C, 32'h0000_0001);

    wait_done;
    cpu_read(5'h10, rval);
    $display("[TEST 3] STATUS=0x%08h", rval);

    $finish;
  end

 

endmodule
