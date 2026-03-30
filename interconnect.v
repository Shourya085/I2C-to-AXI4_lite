// ============================================================
// File    : i2c_to_axi4lite.v  (FINAL v4 - CLEAN AND CORRECT)
// ============================================================

`timescale 1ns / 1ps

module i2c_to_axi4lite #(
    parameter CLK_FREQ = 100_000_000,
    parameter I2C_FREQ = 100_000
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [4:0]  s_awaddr,
    input  wire        s_awvalid,
    output reg         s_awready,

    input  wire [31:0] s_wdata,
    input  wire        s_wvalid,
    output reg         s_wready,

    output reg  [1:0]  s_bresp,
    output reg         s_bvalid,
    input  wire        s_bready,

    input  wire [4:0]  s_araddr,
    input  wire        s_arvalid,
    output reg         s_arready,

    output reg  [31:0] s_rdata,
    output reg  [1:0]  s_rresp,
    output reg         s_rvalid,
    input  wire        s_rready,

    output reg         scl,
    inout  wire        sda
);

    localparam integer HALF = CLK_FREQ / (I2C_FREQ * 2);

    // Config registers
    reg [6:0]  reg_addr;
    reg [23:0] reg_axia;
    reg [7:0]  reg_wdata;
    reg        reg_start, reg_rw;
    reg        reg_busy, reg_done, reg_nack;
    reg [7:0]  reg_rdata;

    // SDA
    reg sda_oe, sda_o;
    assign sda = (sda_oe && !sda_o) ? 1'b0 : 1'bz;

    // States
    localparam [4:0]
        IDLE     = 0,
        START1   = 1,   // SDA↓ SCL=1
        START2   = 2,   // SCL↓, load byte
        TX_LO    = 3,   // SCL=0, put bit on SDA
        TX_HI    = 4,   // SCL=1, hold (slave samples)
        TX_NEXT  = 5,   // SCL=1→0, advance to next bit or ACK
        ACK_LO   = 6,   // SCL=0, release SDA
        ACK_HI   = 7,   // SCL=1, sample ACK
        ACK_NEXT = 8,   // SCL=1→0, decide what's next
        RX_LO    = 9,   // SCL=0, slave drives SDA
        RX_HI    = 10,  // SCL=1, sample bit
        RX_NEXT  = 11,  // SCL=1→0, advance
        NACK_LO  = 12,  // SCL=0, SDA=1
        NACK_HI  = 13,  // SCL=1
        STOP1    = 14,  // SCL=0, SDA=0
        STOP2    = 15,  // SCL=1, SDA=0
        STOP3    = 16,  // SDA↑ = STOP
        DONEST   = 17;

    reg [4:0]  st;
    reg [15:0] cnt;
    reg        tick;
    reg [3:0]  bidx;      // bit index 7..0
    reg [7:0]  txr;       // TX shift register
    reg [7:0]  rxr;       // RX shift register
    reg        adone;     // address byte sent flag
    reg        sda_sample; // latched SDA value

    // Tick
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin cnt <= 0; tick <= 0; end
        else if (st == IDLE) begin cnt <= 0; tick <= 0; end
        else begin
            tick <= 0;
            if (cnt == HALF - 1) begin cnt <= 0; tick <= 1; end
            else cnt <= cnt + 1;
        end
    end

    // FSM
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= IDLE; scl <= 1; sda_o <= 1; sda_oe <= 0;
            bidx <= 0; txr <= 0; rxr <= 0; adone <= 0;
            reg_busy <= 0; reg_done <= 0; reg_nack <= 0; reg_rdata <= 0;
            sda_sample <= 0;
        end else begin
            case (st)

                IDLE: begin
                    scl <= 1; sda_o <= 1; sda_oe <= 0; adone <= 0;
                    if (reg_start) begin
                        reg_busy <= 1; reg_done <= 0; reg_nack <= 0;
                        st <= START1;
                    end
                end

                // ---- START ----
                START1: if (tick) begin
                    scl <= 1; sda_o <= 0; sda_oe <= 1;  // SDA falls
                    st <= START2;
                end

                START2: if (tick) begin
                    scl  <= 0;                           // SCL falls
                    txr  <= {reg_addr, reg_rw};
                    bidx <= 4'd7;
                    adone <= 0;
                    st   <= TX_LO;
                end

                // ---- TRANSMIT BIT ----
                // TX_LO:   SCL is already 0 (set by previous state)
                //          Drive SDA with txr[bidx]
                //          (We use txr[7] and shift after each bit)
                TX_LO: if (tick) begin
                    // SCL stays 0 (already low)
                    // But we need to KEEP scl=0 explicitly
                    scl    <= 0;
                    sda_o  <= txr[7];
                    sda_oe <= 1;
                    st     <= TX_HI;
                end

                // TX_HI: Raise SCL. SDA is stable. Slave samples.
                TX_HI: if (tick) begin
                    scl <= 1;
                    st  <= TX_NEXT;
                end

                // TX_NEXT: SCL has been high for one HALF. Now pull low.
                //          Advance bit counter.
                TX_NEXT: if (tick) begin
                    scl <= 0;
                    if (bidx == 0) begin
                        // All 8 bits sent, go to ACK
                        st <= ACK_LO;
                    end else begin
                        txr  <= {txr[6:0], 1'b0};
                        bidx <= bidx - 1;
                        st   <= TX_LO;
                    end
                end

                // ---- ACK ----
                ACK_LO: if (tick) begin
                    scl    <= 0;
                    sda_oe <= 0;  // release SDA
                    st     <= ACK_HI;
                end

                ACK_HI: if (tick) begin
                    scl <= 1;
                    sda_sample <= sda;  // latch SDA
                    st  <= ACK_NEXT;
                end

                ACK_NEXT: if (tick) begin
                    scl <= 0;
                    if (sda_sample == 1'b1) begin
                        // NACK
                        reg_nack <= 1;
                        st <= STOP1;
                    end else if (!adone) begin
                        // Address ACK, go to data
                        adone <= 1;
                        bidx  <= 4'd7;
                        if (reg_rw) begin
                            rxr <= 0;
                            st  <= RX_LO;
                        end else begin
                            txr <= reg_wdata;
                            st  <= TX_LO;
                        end
                    end else begin
                        // Data ACK, done
                        st <= STOP1;
                    end
                end

                // ---- RECEIVE BIT ----
                RX_LO: if (tick) begin
                    scl    <= 0;
                    sda_oe <= 0;
                    st     <= RX_HI;
                end

                RX_HI: if (tick) begin
                    scl <= 1;
                    sda_sample <= sda;
                    st  <= RX_NEXT;
                end

                RX_NEXT: if (tick) begin
                    scl <= 0;
                    rxr <= {rxr[6:0], sda_sample};
                    if (bidx == 0) begin
                        reg_rdata <= {rxr[6:0], sda_sample};
                        st <= NACK_LO;
                    end else begin
                        bidx <= bidx - 1;
                        st   <= RX_LO;
                    end
                end

                // ---- MASTER NACK ----
                NACK_LO: if (tick) begin
                    scl    <= 0;
                    sda_o  <= 1;   // NACK = high
                    sda_oe <= 1;
                    st     <= NACK_HI;
                end

                NACK_HI: if (tick) begin
                    scl <= 1;
                    st  <= STOP1;
                end

                // ---- STOP ----
                STOP1: if (tick) begin
                    scl    <= 0;
                    sda_o  <= 0;
                    sda_oe <= 1;
                    st     <= STOP2;
                end

                STOP2: if (tick) begin
                    scl <= 1;      // SCL rises, SDA still low
                    st  <= STOP3;
                end

                STOP3: if (tick) begin
                    sda_o  <= 1;   // SDA rises = STOP
                    sda_oe <= 0;
                    st     <= DONEST;
                end

                DONEST: if (tick) begin
                    reg_busy  <= 0;
                    reg_done  <= 1;
                    reg_start <= 0;
                    st        <= IDLE;
                end

                default: st <= IDLE;
            endcase
        end
    end

    // ============================================================
    // AXI Write
    // ============================================================
    reg [4:0] awl;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_awready <= 0; s_wready <= 0; s_bvalid <= 0; s_bresp <= 0;
            awl <= 0; reg_addr <= 0; reg_axia <= 0; reg_wdata <= 0;
            reg_start <= 0; reg_rw <= 0;
        end else begin
            if (s_awvalid && !s_awready) begin
                s_awready <= 1; awl <= s_awaddr;
            end else s_awready <= 0;

            if (s_wvalid && !s_wready) begin
                s_wready <= 1;
                case (awl[4:2])
                    3'd0: reg_addr  <= s_wdata[6:0];
                    3'd1: reg_axia  <= s_wdata[23:0];
                    3'd2: reg_wdata <= s_wdata[7:0];
                    3'd3: begin reg_start <= s_wdata[0]; reg_rw <= s_wdata[1]; end
                    default: ;
                endcase
            end else s_wready <= 0;

            if (s_wready && !s_bvalid) begin
                s_bvalid <= 1; s_bresp <= 2'b00;
            end else if (s_bvalid && s_bready) s_bvalid <= 0;
        end
    end

    // ============================================================
    // AXI Read
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_arready <= 0; s_rvalid <= 0; s_rdata <= 0; s_rresp <= 0;
        end else begin
            if (s_arvalid && !s_arready) begin
                s_arready <= 1; s_rvalid <= 1; s_rresp <= 2'b00;
                case (s_araddr[4:2])
                    3'd0: s_rdata <= {25'd0, reg_addr};
                    3'd1: s_rdata <= {8'd0, reg_axia};
                    3'd2: s_rdata <= {24'd0, reg_wdata};
                    3'd3: s_rdata <= {30'd0, reg_rw, reg_start};
                    3'd4: s_rdata <= {29'd0, reg_nack, reg_done, reg_busy};
                    3'd5: s_rdata <= {24'd0, reg_rdata};
                    default: s_rdata <= 32'd0;
                endcase
            end else begin
                s_arready <= 0;
                if (s_rvalid && s_rready) s_rvalid <= 0;
            end
        end
    end

endmodule