`timescale 1ns / 1ps

module i2c_to_axi4lite #(
    parameter [6:0] I2C_ADDR = 7'h55
)(
    input  wire        clk,
    input  wire        rst_n,

    // I2C Slave (open-drain SDA)
    input  wire        scl_i,
    input  wire        sda_i,
    output reg         sda_o,
    output reg         sda_oe,

    // AXI4-Lite Master
    output reg  [15:0] m_axi_awaddr,
    output reg         m_axi_awvalid,
    input  wire        m_axi_awready,
    output reg  [31:0] m_axi_wdata,
    output reg  [3:0]  m_axi_wstrb,
    output reg         m_axi_wvalid,
    input  wire        m_axi_wready,
    input  wire [1:0]  m_axi_bresp,
    input  wire        m_axi_bvalid,
    output reg         m_axi_bready,
    output reg  [15:0] m_axi_araddr,
    output reg         m_axi_arvalid,
    input  wire        m_axi_arready,
    input  wire [31:0] m_axi_rdata,
    input  wire [1:0]  m_axi_rresp,
    input  wire        m_axi_rvalid,
    output reg         m_axi_rready,

    output reg         busy,
    output reg         axi_error
);

    // 2-FF synchronisers 
    reg [1:0] scl_ff, sda_ff;
    wire scl = scl_ff[1];
    wire sda = sda_ff[1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin scl_ff <= 2'b11; sda_ff <= 2'b11; end
        else begin
            scl_ff <= {scl_ff[0], scl_i};
            sda_ff <= {sda_ff[0], sda_i};
        end
    end

    reg scl_d, sda_d;
    always @(posedge clk) begin scl_d <= scl; sda_d <= sda; end

    wire scl_rise  =  scl & ~scl_d;
    wire scl_fall  = ~scl &  scl_d;
    wire i2c_start = ~sda &  sda_d & scl;
    wire i2c_stop  =  sda & ~sda_d & scl;

    reg [2:0] rx_seq;

    
    reg [7:0] rx_byte;     
    reg [7:0] addr_byte;   
    reg [7:0] axih_byte;   
    reg [7:0] axilo_byte; 
    reg [7:0] data_byte;  
    reg [2:0] bit_cnt;

    // TX shift register for read result
    reg [7:0] tx_byte;
    reg [2:0] tx_cnt;

    // State encoding 
    localparam [3:0]
        S2_IDLE     = 4'd0,
        S2_RX       = 4'd1,   
        S2_ACK      = 4'd2,   
        S2_ACK_REL  = 4'd3,   
        S2_AXI_WR   = 4'd4,   
        S2_AXI_WR_W = 4'd5, 
        S2_AXI_RD   = 4'd6,  
        S2_AXI_RD_W = 4'd7,  
        S2_TX       = 4'd8,   
        S2_WAIT_NK  = 4'd9; 

    reg [3:0] state;

    // Main FSM 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S2_IDLE;
            rx_seq        <= 3'd0;
            rx_byte       <= 8'h00;
            addr_byte     <= 8'h00;
            axih_byte     <= 8'h00;
            axilo_byte    <= 8'h00;
            data_byte     <= 8'h00;
            bit_cnt       <= 3'd7;
            tx_byte       <= 8'h00;
            tx_cnt        <= 3'd7;
            sda_o         <= 1'b1;
            sda_oe        <= 1'b0;
            busy          <= 1'b0;
            axi_error     <= 1'b0;
            m_axi_awvalid <= 1'b0;
            m_axi_wvalid  <= 1'b0;
            m_axi_wstrb   <= 4'hF;
            m_axi_bready  <= 1'b0;
            m_axi_arvalid <= 1'b0;
            m_axi_rready  <= 1'b0;
            m_axi_awaddr  <= 16'h0;
            m_axi_araddr  <= 16'h0;
            m_axi_wdata   <= 32'h0;
        end else begin

            // Global: STOP resets from any state
            if (i2c_stop) begin
                state  <= S2_IDLE;
                sda_oe <= 1'b0;
                busy   <= 1'b0;
            end
            //  Global: START / REPEATED START 
            else if (i2c_start) begin
                state   <= S2_RX;
                rx_seq  <= 3'd0;
                rx_byte <= 8'h00;
                bit_cnt <= 3'd7;
                sda_oe  <= 1'b0;
                busy    <= 1'b1;
            end
            else case (state)

                S2_IDLE: begin
                    sda_oe <= 1'b0;
                    busy   <= 1'b0;
                end

                S2_RX: begin
                    if (scl_rise) begin
                        rx_byte <= {rx_byte[6:0], sda};
                        if (bit_cnt == 3'd0) begin
                            case (rx_seq)
                                3'd0: addr_byte  <= {rx_byte[6:0], sda};
                                3'd1: axih_byte  <= {rx_byte[6:0], sda};
                                3'd2: axilo_byte <= {rx_byte[6:0], sda};
                                3'd3: data_byte  <= {rx_byte[6:0], sda};
                                default: ;
                            endcase
                            state   <= S2_ACK;
                            bit_cnt <= 3'd7;
                            rx_byte <= 8'h00;
                        end else
                            bit_cnt <= bit_cnt - 1;
                    end
                end

                S2_ACK: begin
                    if (scl_fall) begin
                        if (rx_seq == 3'd0) begin
                            if (addr_byte[7:1] == I2C_ADDR) begin
                                sda_o  <= 1'b0;
                                sda_oe <= 1'b1;      // ACK
                                state  <= S2_ACK_REL;
                            end else begin
                                sda_oe <= 1'b0;      // NACK
                                state  <= S2_IDLE;
                            end
                        end else begin
                            sda_o  <= 1'b0;
                            sda_oe <= 1'b1;          // ACK
                            state  <= S2_ACK_REL;
                        end
                    end
                end

                S2_ACK_REL: begin
                    if (scl_fall) begin
                        sda_oe <= 1'b0;

                        case (rx_seq)
                            3'd0: begin
                                // Address byte done
                                if (addr_byte[0] == 1'b0) begin
                                    // Write mode: receive AXI addr high
                                    rx_seq  <= 3'd1;
                                    rx_byte <= 8'h00;
                                    bit_cnt <= 3'd7;
                                    state   <= S2_RX;
                                end else begin
                                    state <= S2_AXI_RD;
                                end
                            end
                            3'd1: begin
                                // AXI addr high done, get low byte
                                rx_seq  <= 3'd2;
                                rx_byte <= 8'h00;
                                bit_cnt <= 3'd7;
                                state   <= S2_RX;
                            end
                            3'd2: begin
                                // AXI addr low done, get data byte
                                rx_seq  <= 3'd3;
                                rx_byte <= 8'h00;
                                bit_cnt <= 3'd7;
                                state   <= S2_RX;
                            end
                            3'd3: begin
                                // Data byte done, fire AXI write
                                state <= S2_AXI_WR;
                            end
                            default: state <= S2_IDLE;
                        endcase
                    end
                end

                // AXI Write 
                S2_AXI_WR: begin
                    m_axi_awaddr  <= {axih_byte, axilo_byte};
                    m_axi_awvalid <= 1'b1;
                    m_axi_wdata   <= {24'd0, data_byte};
                    m_axi_wstrb   <= 4'hF;
                    m_axi_wvalid  <= 1'b1;
                    state         <= S2_AXI_WR_W;
                end

                S2_AXI_WR_W: begin
                    if (m_axi_awready) m_axi_awvalid <= 1'b0;
                    if (m_axi_wready)  m_axi_wvalid  <= 1'b0;
                    if (m_axi_bvalid) begin
                        m_axi_bready <= 1'b1;
                        axi_error    <= |m_axi_bresp;
                        state        <= S2_IDLE;
                        busy         <= 1'b0;
                    end
                    if (m_axi_bready) m_axi_bready <= 1'b0;
                end

                // AXI Read 
                S2_AXI_RD: begin
                    m_axi_araddr  <= {axih_byte, axilo_byte};
                    m_axi_arvalid <= 1'b1;
                    m_axi_rready  <= 1'b1;
                    state         <= S2_AXI_RD_W;
                end

                S2_AXI_RD_W: begin
                    if (m_axi_arready) m_axi_arvalid <= 1'b0;
                    if (m_axi_rvalid) begin
                        m_axi_rready <= 1'b0;
                        axi_error    <= |m_axi_rresp;
                        tx_byte      <= m_axi_rdata[7:0];
                        tx_cnt       <= 3'd7;
                        state        <= S2_TX;
                    end
                end

                S2_TX: begin
                    if (scl_fall) begin
                        sda_o  = tx_byte[7];    // blocking: immediate
                        sda_oe <= 1'b1;
                        if (tx_cnt == 3'd0) begin
                            state <= S2_WAIT_NK;
                        end else begin
                            tx_byte <= {tx_byte[6:0], 1'b0};
                            tx_cnt  <= tx_cnt - 1;
                        end
                    end
                end
                S2_WAIT_NK: begin
                    if (scl_fall) begin
                        sda_oe <= 1'b0;   // release after master sampled
                        state  <= S2_IDLE;
                        busy   <= 1'b0;
                    end
                end

                default: state <= S2_IDLE;
            endcase
        end
    end

endmodule
