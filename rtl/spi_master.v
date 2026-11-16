/*****************************************************************************
 Module: spi_master
 Purpose: Mode-0 SPI master serializer/deserializer.

 Description:
   Implements a 3-state FSM (IDLE/TRANSFER/FINISH) that shifts TX data out
   on MOSI and captures RX data from MISO. A clock divider generates a
   25 MHz SCLK from the system clock, with edge detection driving the shift
   registers. Used by spi_top to communicate with sensor and processing slaves.

 Inputs:
   clk_i     - System clock
   rst_i     - Asynchronous active-high reset
   start_i   - Pulse to begin a new SPI transaction
   miso_i    - Master-In Slave-Out serial data
   tx_data_i - Parallel transmit data (DATA_WIDTH bits)

 Outputs:
   mosi_o    - Master-Out Slave-In serial data
   sclk_o    - SPI clock output
   rx_data_o - Parallel received data (DATA_WIDTH bits)
   busy_o    - High while a transfer is in progress
   done_o    - Single-cycle pulse when transfer completes
*****************************************************************************/

module spi_master #(parameter DATA_WIDTH = 50)(
    input clk_i,
    input rst_i,
    input start_i,
    input miso_i,
    input wire [DATA_WIDTH-1:0] tx_data_i,

    output reg mosi_o,
    output reg sclk_o,
    output reg [DATA_WIDTH-1:0] rx_data_o,
    output reg busy_o,
    output reg done_o
);

localparam IDLE     = 2'd0;
localparam TRANSFER = 2'd1;
localparam FINISH   = 2'd2;

reg [1:0] state;
// bit_cnt needs to count up to DATA_WIDTH (50). 6 bits covers 0-63.
reg [5:0] bit_cnt;

reg [DATA_WIDTH-1:0] shift_tx;
reg [DATA_WIDTH-1:0] shift_rx;

reg [0:0] clk_div;
reg       sclk_en;

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
        clk_div <= 0;
        sclk_o  <= 0;
    end else if (sclk_en) begin
        if (clk_div == 1'b1) begin
            sclk_o  <= ~sclk_o;
            clk_div <= 0;
        end else begin
            clk_div <= clk_div + 1;
        end
    end else begin
        clk_div <= 0;
        sclk_o  <= 0;
    end
end

reg sclk_d;

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) sclk_d <= 0;
    else        sclk_d <= sclk_o;
end

wire sclk_rise = ( sclk_o & ~sclk_d);
wire sclk_fall = (~sclk_o &  sclk_d);

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
        state     <= IDLE;
        mosi_o    <= 0;
        sclk_en   <= 0;
        busy_o    <= 0;
        done_o    <= 0;
        bit_cnt   <= 0;
        shift_tx  <= 0;
        shift_rx  <= 0;
        rx_data_o <= 0;
    end
    else begin
        done_o <= 0;

        case (state)

        IDLE: begin
            sclk_en <= 0;
            if (start_i) begin
                busy_o   <= 1;

                shift_tx <= {tx_data_i[DATA_WIDTH-2:0], 1'b0};

                mosi_o   <= tx_data_i[DATA_WIDTH-1];
                bit_cnt  <= 0;
                sclk_en  <= 1;
                state    <= TRANSFER;
            end else begin
                busy_o <= 0;
            end
        end

        TRANSFER: begin
            if (sclk_fall && (bit_cnt < DATA_WIDTH)) begin
                mosi_o   <= shift_tx[DATA_WIDTH-1];
                shift_tx <= {shift_tx[DATA_WIDTH-2:0], 1'b0};
            end

            if (sclk_rise) begin
                shift_rx <= {shift_rx[DATA_WIDTH-2:0], miso_i};
                bit_cnt  <= bit_cnt + 1;
            end

            if (sclk_fall && (bit_cnt == DATA_WIDTH)) begin
                sclk_en   <= 0;
                state     <= FINISH;
            end
        end

        FINISH: begin
            rx_data_o <= shift_rx;
            done_o <= 1;
            busy_o <= 0;
            state  <= IDLE;
        end

        default: state <= IDLE;
        endcase
    end
end

endmodule
