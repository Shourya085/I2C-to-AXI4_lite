`timescale 1ns / 1ps

// I2C Slave to AXI4-Lite Master bridge

module i2c_to_axi4lite #(
    parameter [6:0] I2C_ADDR = 7'h55
)(
    input  wire clk,
    input  wire rst_n,

    // I2C
    input  wire scl_i,
    input  wire sda_i,
    output reg  sda_o,
    output reg  sda_oe,

    // AXI4-Lite
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

    output reg busy,
    output reg axi_error
);

    // ----------------------------------------------------
    // sync I2C lines
    reg [1:0] scl_ff, sda_ff;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_ff <= 2'b11;
            sda_ff <= 2'b11;
        end else begin
            scl_ff <= {scl_ff[0], scl_i};
            sda_ff <= {sda_ff[0], sda_i};
        end
    end

    wire scl = scl_ff[1];
    wire sda = sda_ff[1];

    // edge detect
    reg scl_d, sda_d;

    always @(posedge clk) begin
        scl_d <= scl;
        sda_d <= sda;
    end

    wire scl_rise =  scl & ~scl_d;
    wire scl_fall = ~scl &  scl_d;

    wire i2c_start = ~sda &  sda_d & scl;
    wire i2c_stop  =  sda & ~sda_d & scl;

    // ----------------------------------------------------
    // registers

    reg [2:0] rx_seq;

    reg [7:0] rx_byte;
    reg [7:0] addr_byte;
    reg [7:0] axih_byte;
    reg [7:0] axilo_byte;
    reg [7:0] data_byte;

    reg [2:0] bit_cnt;

    reg [7:0] tx_byte;
    reg [2:0] tx_cnt;

    // ----------------------------------------------------
    // states

    localparam [3:0]
        IDLE      = 4'd0,
        RX        = 4'd1,
        ACK       = 4'd2,
        ACK_REL   = 4'd3,
        AXI_WR    = 4'd4,
        AXI_WR_W  = 4'd5,
        AXI_RD    = 4'd6,
        AXI_RD_W  = 4'd7,
        TX        = 4'd8,
        WAIT_NACK = 4'd9;

    reg [3:0] state;

    // ----------------------------------------------------
    // main FSM

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            rx_seq <= 0;
            rx_byte <= 0;
            addr_byte <= 0;
            axih_byte <= 0;
            axilo_byte <= 0;
            data_byte <= 0;
            bit_cnt <= 7;
            tx_byte <= 0;
            tx_cnt <= 7;

            sda_o <= 1;
            sda_oe <= 0;

            busy <= 0;
            axi_error <= 0;

            m_axi_awvalid <= 0;
            m_axi_wvalid  <= 0;
            m_axi_wstrb   <= 4'hF;
            m_axi_bready  <= 0;
            m_axi_arvalid <= 0;
            m_axi_rready  <= 0;

        end else begin

            // STOP condition
            if (i2c_stop) begin
                state  <= IDLE;
                sda_oe <= 0;
                busy   <= 0;
            end

            // START or repeated START
            else if (i2c_start) begin
                state   <= RX;
                rx_seq  <= 0;
                rx_byte <= 0;
                bit_cnt <= 7;
                sda_oe  <= 0;
                busy    <= 1;
            end

            else case (state)

                IDLE: begin
                    busy   <= 0;
                    sda_oe <= 0;
                end

                // receive byte
                RX: begin
                    if (scl_rise) begin
                        rx_byte <= {rx_byte[6:0], sda};

                        if (bit_cnt == 0) begin
                            case (rx_seq)
                                0: addr_byte  <= {rx_byte[6:0], sda};
                                1: axih_byte  <= {rx_byte[6:0], sda};
                                2: axilo_byte <= {rx_byte[6:0], sda};
                                3: data_byte  <= {rx_byte[6:0], sda};
                            endcase

                            state   <= ACK;
                            bit_cnt <= 7;
                            rx_byte <= 0;
                        end else begin
                            bit_cnt <= bit_cnt - 1;
                        end
                    end
                end

                // send ACK
                ACK: begin
                    if (scl_fall) begin
                        if (rx_seq == 0) begin
                            if (addr_byte[7:1] == I2C_ADDR) begin
                                sda_o  <= 0;
                                sda_oe <= 1;
                                state  <= ACK_REL;
                            end else begin
                                sda_oe <= 0;
                                state  <= IDLE;
                            end
                        end else begin
                            sda_o  <= 0;
                            sda_oe <= 1;
                            state  <= ACK_REL;
                        end
                    end
                end

                // release SDA after ACK
                ACK_REL: begin
                    if (scl_fall) begin
                        sda_oe <= 0;

                        case (rx_seq)
                            0: begin
                                if (addr_byte[0] == 0) begin
                                    rx_seq <= 1;
                                    state  <= RX;
                                end else begin
                                    state <= AXI_RD;
                                end
                            end

                            1: begin
                                rx_seq <= 2;
                                state  <= RX;
                            end

                            2: begin
                                rx_seq <= 3;
                                state  <= RX;
                            end

                            3: begin
                                state <= AXI_WR;
                            end
                        endcase
                    end
                end

                // AXI write
                AXI_WR: begin
                    m_axi_awaddr  <= {axih_byte, axilo_byte};
                    m_axi_awvalid <= 1;
                    m_axi_wdata   <= {24'd0, data_byte};
                    m_axi_wvalid  <= 1;
                    state         <= AXI_WR_W;
                end

                AXI_WR_W: begin
                    if (m_axi_awready) m_axi_awvalid <= 0;
                    if (m_axi_wready)  m_axi_wvalid  <= 0;

                    if (m_axi_bvalid) begin
                        m_axi_bready <= 1;
                        axi_error    <= |m_axi_bresp;
                        state        <= IDLE;
                        busy         <= 0;
                    end

                    if (m_axi_bready)
                        m_axi_bready <= 0;
                end

                // AXI read
                AXI_RD: begin
                    m_axi_araddr  <= {axih_byte, axilo_byte};
                    m_axi_arvalid <= 1;
                    m_axi_rready  <= 1;
                    state         <= AXI_RD_W;
                end

                AXI_RD_W: begin
                    if (m_axi_arready)
                        m_axi_arvalid <= 0;

                    if (m_axi_rvalid) begin
                        m_axi_rready <= 0;
                        axi_error    <= |m_axi_rresp;
                        tx_byte      <= m_axi_rdata[7:0];
                        tx_cnt       <= 7;
                        state        <= TX;
                    end
                end

                // transmit byte
                TX: begin
                    if (scl_fall) begin
                        sda_o  = tx_byte[7];
                        sda_oe <= 1;

                        if (tx_cnt == 0) begin
                            state <= WAIT_NACK;
                        end else begin
                            tx_byte <= {tx_byte[6:0], 1'b0};
                            tx_cnt  <= tx_cnt - 1;
                        end
                    end
                end

                // wait for last clock, then release
                WAIT_NACK: begin
                    if (scl_fall) begin
                        sda_oe <= 0;
                        state  <= IDLE;
                        busy   <= 0;
                    end
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule
