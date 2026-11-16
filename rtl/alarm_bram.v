/*****************************************************************************
 Module: alarm_bram
 Purpose: 64-entry x 64-bit circular BRAM buffer for alarm event logging
          with byte-wise XOR parity error detection.

 Description:
   Stores alarm events into a 64-deep memory array on each alarm_rising_i
   pulse, packing timestamp, AQI, category, error flags, and alarm state
   into a single 64-bit entry. Each entry is protected by byte-wise XOR
   parity (8 parity bits per 64-bit word) enabling single-bit error
   detection on read-back. A write pointer wraps circularly, with an
   entry counter and overflow flag tracking buffer fullness. Supports
   synchronous read and clear.

 Parity scheme:
   - Internal storage: 72 bits per entry (64 data + 8 parity)
   - Parity: XOR reduction of all 8 bytes in the 64-bit data word
   - ecc_ded_o asserts when read-back parity does not match stored parity
   - ecc_sec_o is permanently deasserted (no correction capability)

 Write entry format (64 bits):
   [63:32] timestamp_i    (32 bits)
   [31:26] reserved       ( 6 bits, zero)
   [25:17] aqi_i          ( 9 bits)
   [16:14] category_i     ( 3 bits)
   [13:10] input_err_i    ( 4 bits)
   [9]     crc_err_i      ( 1 bit)
   [8]     alarm_state_i  ( 1 bit)
   [7:0]   reserved       ( 8 bits, zero)

 Used by spi_top for volatile alarm log storage.

 Inputs:
   clk            - System clock
   rst            - Asynchronous active-high reset
   alarm_rising_i - Single-cycle pulse to log a new alarm event
   timestamp_i    - 32-bit timestamp for the event
   aqi_i          - 9-bit air quality index value
   category_i     - 3-bit alarm category
   input_err_i    - 4-bit input error flags
   crc_err_i      - CRC error flag
   alarm_state_i  - Current alarm state
   rd_addr_i      - 6-bit read address into the buffer
   clr_i          - Pulse to clear the buffer

 Outputs:
   rd_data_o     - 64-bit read data at rd_addr_i
   entry_count_o - Number of valid entries (saturates at 64)
   wr_ptr_o      - Current write pointer (registered)
   overflow_o    - Set when buffer has wrapped and old data is overwritten
   ecc_sec_o     - Single-error corrected flag, always 0 (reserved)
   ecc_ded_o     - Parity error detected on last read
*****************************************************************************/

module alarm_bram (
    input  wire        clk,
    input  wire        rst,

    input  wire        alarm_rising_i,
    input  wire [31:0] timestamp_i,
    input  wire [8:0]  aqi_i,
    input  wire [2:0]  category_i,
    input  wire [3:0]  input_err_i,
    input  wire        crc_err_i,
    input  wire        alarm_state_i,

    input  wire [5:0]  rd_addr_i,
    output wire [63:0] rd_data_o,

    output reg  [6:0]  entry_count_o,
    output reg  [5:0]  wr_ptr_o,
    output reg         overflow_o,

    output wire        ecc_sec_o,
    output wire        ecc_ded_o,

    input  wire        clr_i
);

// ---------------------------------------------------------------------------
// BRAM array: 64 entries x 72 bits (64 data + 8 parity)
// ---------------------------------------------------------------------------
reg [71:0] mem [0:63];

reg [5:0] wr_ptr;

// ---------------------------------------------------------------------------
// Byte-wise XOR parity computation
// ---------------------------------------------------------------------------
function [7:0] calc_parity;
    input [63:0] d;
    begin
        calc_parity = d[7:0] ^ d[15:8] ^ d[23:16] ^ d[31:24] ^
                      d[39:32] ^ d[47:40] ^ d[55:48] ^ d[63:56];
    end
endfunction

// ---------------------------------------------------------------------------
// Write entry packing
// ---------------------------------------------------------------------------
wire [63:0] wr_entry = {
    timestamp_i,
    6'b0, aqi_i,
    category_i, input_err_i,
    crc_err_i, alarm_state_i,
    8'h00
};

wire [7:0] wr_parity = calc_parity(wr_entry);

// ---------------------------------------------------------------------------
// Write logic with parity
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        wr_ptr        <= 6'd0;
        entry_count_o <= 7'd0;
        overflow_o    <= 1'b0;
    end else if (clr_i) begin
        wr_ptr        <= 6'd0;
        entry_count_o <= 7'd0;
        overflow_o    <= 1'b0;
    end else if (alarm_rising_i) begin
        mem[wr_ptr]   <= {wr_parity, wr_entry};
        wr_ptr        <= wr_ptr + 1;
        if (entry_count_o < 7'd64) begin
            entry_count_o <= entry_count_o + 7'd1;
            if (entry_count_o == 7'd63)
                overflow_o <= 1'b1;  // about to become full
        end
    end
end

// ---------------------------------------------------------------------------
// Write pointer output register
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) wr_ptr_o <= 6'd0;
    else     wr_ptr_o <= wr_ptr;
end

// ---------------------------------------------------------------------------
// Synchronous read with parity check (1-cycle latency)
// ---------------------------------------------------------------------------
reg [71:0] rd_raw;
always @(posedge clk or posedge rst) begin
    if (rst)
        rd_raw <= 72'd0;
    else
        rd_raw <= mem[rd_addr_i];
end

wire [63:0] rd_data_raw    = rd_raw[63:0];
wire [7:0]  rd_parity_raw  = rd_raw[71:64];
wire [7:0]  rd_parity_calc = calc_parity(rd_data_raw);

assign rd_data_o = rd_data_raw;
assign ecc_sec_o = 1'b0;
assign ecc_ded_o = (rd_parity_raw != rd_parity_calc);

endmodule
