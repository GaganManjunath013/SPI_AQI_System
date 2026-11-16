/*****************************************************************************
 Module: uart_tx
 Purpose: UART transmit serializer.

 Description:
   Implements a 4-state FSM (IDLE/START/DATA/STOP) that serializes an 8-bit
   byte onto the TX line with standard UART framing (1 start bit, 8 data
   bits LSB-first, 1 stop bit). A bit-time counter counts CLKS_PER_BIT
   system clocks per bit period. Used by spi_top's FIFO drain engine to
   send data to the host.

 Inputs:
   clk      - System clock
   rst      - Asynchronous active-high reset
   tx_start - Pulse to begin transmitting tx_data
   tx_data  - 8-bit parallel data to transmit

 Outputs:
   tx       - Serial transmit line (idles high)
   tx_done  - Single-cycle pulse when byte transmission completes
   tx_busy  - High while a transmission is in progress
*****************************************************************************/

module uart_tx #(parameter CLKS_PER_BIT = 109)(
    input  wire       clk,
    input  wire       rst,
    input  wire       tx_start,
    input  wire [7:0] tx_data,
    output reg        tx,
    output reg        tx_done,
    output wire       tx_busy
);

localparam IDLE      = 2'd0;
localparam START_BIT = 2'd1;
localparam DATA_BITS = 2'd2;
localparam STOP_BIT  = 2'd3;

reg [1:0] state;
reg [9:0] clk_cnt;
reg [2:0] bit_cnt;
reg [7:0] tx_reg;

assign tx_busy = (state != IDLE);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state   <= IDLE;
        tx      <= 1'b1;
        tx_done <= 1'b0;
        clk_cnt <= 0;
        bit_cnt <= 0;
        tx_reg  <= 0;
    end else begin
        tx_done <= 1'b0;

        case (state)

        IDLE: begin
            tx <= 1'b1;
            if (tx_start) begin
                tx_reg  <= tx_data;
                clk_cnt <= 0;
                state   <= START_BIT;
            end
        end

        START_BIT: begin
            tx <= 1'b0;
            if (clk_cnt < CLKS_PER_BIT - 1)
                clk_cnt <= clk_cnt + 1;
            else begin
                clk_cnt <= 0;
                state   <= DATA_BITS;
            end
        end

        DATA_BITS: begin
            tx <= tx_reg[bit_cnt];
            if (clk_cnt < CLKS_PER_BIT - 1)
                clk_cnt <= clk_cnt + 1;
            else begin
                clk_cnt <= 0;
                if (bit_cnt < 3'd7)
                    bit_cnt <= bit_cnt + 1;
                else begin
                    bit_cnt <= 0;
                    state   <= STOP_BIT;
                end
            end
        end

        STOP_BIT: begin
            tx <= 1'b1;
            if (clk_cnt < CLKS_PER_BIT - 1)
                clk_cnt <= clk_cnt + 1;
            else begin
                clk_cnt <= 0;
                tx_done <= 1'b1;
                state   <= IDLE;
            end
        end

        default: state <= IDLE;
        endcase
    end
end

endmodule
