/*****************************************************************************
 Module: uart_rx
 Purpose: UART receive deserializer with host command decode.

 Description:
   Implements a 4-state FSM (IDLE/START/DATA/STOP) that deserializes an
   8-bit byte from the RX line using mid-bit sampling (HALF_BIT offset).
   A 2-FF synchronizer conditions the asynchronous rx input. On valid stop
   bit, the received byte is decoded into single-cycle command pulses for
   five host commands (0x01-0x04, 0x06). Used by spi_top for host control.

 Inputs:
   clk - System clock
   rst - Asynchronous active-high reset
   rx  - Serial receive line from host

 Outputs:
   rx_done          - Single-cycle pulse on valid byte reception
   rx_data          - 8-bit received data
   cmd_get_live_o   - Pulse on 0x01: request live sensor reading
   cmd_dump_log_o   - Pulse on 0x02: dump alarm log
   cmd_clear_log_o  - Pulse on 0x03: clear alarm log
   cmd_wake_o       - Pulse on 0x04: wake sensor from sleep
   cmd_flash_dump_o - Pulse on 0x06: dump flash contents
*****************************************************************************/

module uart_rx #(parameter CLKS_PER_BIT = 109)(
    input  wire       clk,
    input  wire       rst,
    input  wire       rx,

    output reg        rx_done,
    output reg  [7:0] rx_data,

    output reg        cmd_get_live_o,
    output reg        cmd_dump_log_o,
    output reg        cmd_clear_log_o,
    output reg        cmd_wake_o,
    output reg        cmd_flash_dump_o
);

localparam IDLE      = 2'd0;
localparam START_BIT = 2'd1;
localparam DATA_BITS = 2'd2;
localparam STOP_BIT  = 2'd3;

localparam HALF_BIT  = (CLKS_PER_BIT + 1) / 2;

reg [1:0]  state;
reg [9:0]  clk_cnt;
reg [2:0]  bit_cnt;
reg [7:0]  rx_shift;

// 2-FF synchronizer for asynchronous rx input
reg rx_s1, rx_s2;

always @(posedge clk or posedge rst) begin
    if (rst) begin rx_s1 <= 1'b1; rx_s2 <= 1'b1; end
    else     begin rx_s1 <= rx;    rx_s2 <= rx_s1; end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state            <= IDLE;
        clk_cnt          <= 10'd0;
        bit_cnt          <= 3'd0;
        rx_shift         <= 8'd0;
        rx_done          <= 1'b0;
        rx_data          <= 8'd0;
        cmd_get_live_o   <= 1'b0;
        cmd_dump_log_o   <= 1'b0;
        cmd_clear_log_o  <= 1'b0;
        cmd_wake_o       <= 1'b0;
        cmd_flash_dump_o <= 1'b0;
    end else begin

        rx_done          <= 1'b0;
        cmd_get_live_o   <= 1'b0;
        cmd_dump_log_o   <= 1'b0;
        cmd_clear_log_o  <= 1'b0;
        cmd_wake_o       <= 1'b0;
        cmd_flash_dump_o <= 1'b0;

        case (state)

        IDLE: begin
            if (rx_s2 == 1'b0) begin

                clk_cnt <= 10'd0;
                state   <= START_BIT;
            end
        end

        START_BIT: begin

            if (clk_cnt < HALF_BIT - 1) begin
                clk_cnt <= clk_cnt + 1;
            end else begin
                clk_cnt <= 10'd0;
                bit_cnt <= 3'd0;
                if (rx_s2 == 1'b0)
                    state <= DATA_BITS;
                else
                    state <= IDLE;
            end
        end

        DATA_BITS: begin
            if (clk_cnt < CLKS_PER_BIT - 1) begin
                clk_cnt <= clk_cnt + 1;
            end else begin
                clk_cnt          <= 10'd0;

                rx_shift         <= {rx_s2, rx_shift[7:1]};
                if (bit_cnt < 3'd7) begin
                    bit_cnt <= bit_cnt + 1;
                end else begin
                    bit_cnt <= 3'd0;
                    state   <= STOP_BIT;
                end
            end
        end

        STOP_BIT: begin
            if (clk_cnt < CLKS_PER_BIT - 1) begin
                clk_cnt <= clk_cnt + 1;
            end else begin
                clk_cnt <= 10'd0;
                state   <= IDLE;
                if (rx_s2 == 1'b1) begin

                    rx_data <= rx_shift;
                    rx_done <= 1'b1;

                    case (rx_shift)
                        8'h01: cmd_get_live_o    <= 1'b1;
                        8'h02: cmd_dump_log_o    <= 1'b1;
                        8'h03: cmd_clear_log_o   <= 1'b1;
                        8'h04: cmd_wake_o        <= 1'b1;
                        8'h06: cmd_flash_dump_o  <= 1'b1;
                        default: ;
                    endcase
                end

            end
        end

        default: state <= IDLE;
        endcase
    end
end

endmodule
