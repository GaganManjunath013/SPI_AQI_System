/*****************************************************************************
 Module: spi_processing_slave  (Processing-Centric Architecture — v4.0)
 Purpose: Sole AQI computation authority for the SPI Air-Quality System.

 Architectural Change (v4.0):
   This module now receives a FORWARDED AVERAGED frame from spi_top
   (containing MAF-smoothed concentrations + a fresh CRC-8) rather than
   the raw Sensor A frame. It is the sole authority for:
     1. Validating the CRC-8 of the internal averaged frame.
     2. Computing US-EPA piecewise-linear AQI for PM2.5 and PM10.
     3. Computing simplified AQI for VOC and CO (value >> 1).
     4. Selecting the maximum channel AQI, clamped to 500.
     5. Classifying the result into one of 6 EPA health categories.
     6. Returning aqi_value and category to spi_top for system outputs.

 The processing slave uses a 2-stage pipeline:
   Stage 1 (registered): breakpoint segment selection per channel.
   Stage 2 (registered): multiply-accumulate → final AQI per channel.

 Input Frame Layout (50-bit, identical to sensor frame format):
   [49:40] avg_pm25  [39:30] avg_pm10  [29:20] avg_voc
   [19:10] avg_co    [9:8]   reserved  [7:0]   CRC-8

 AQI formula:  AQI = ((SF × (C − BP_lo)) >> 8) + I_lo

 PM2.5 Breakpoint Table:
   Seg | BP_lo | BP_hi | I_lo | I_hi | SF(Q8)
    0  |   0   |  12   |  0   |  50  | 1067
    1  |  13   |  35   | 51   | 100  |  570
    2  |  36   |  55   | 101  | 150  |  660
    3  |  56   | 150   | 151  | 200  |  133
    4  | 151   | 250   | 201  | 300  |  256
    5  | 251   | 350   | 301  | 400  |  256
    6  | 351   | 499   | 401  | 500  |  171

 PM10 Breakpoint Table:
   Seg | BP_lo | BP_hi | I_lo | I_hi | SF(Q8)
    0  |   0   |  54   |  0   |  50  |  237
    1  |  55   | 154   | 51   | 100  |  127
    2  | 155   | 254   | 101  | 150  |  127
    3  | 255   | 354   | 151  | 200  |  127
    4  | 355   | 424   | 201  | 300  |  367
    5  | 425   | 504   | 301  | 400  |  321
    6  | 505   | 599   | 401  | 500  |  270

 Category Mapping:
   0-50 Good | 51-100 Moderate | 101-200 Unhealthy
   201-300 Severe | 301-400 Very Severe | 401-500 Hazardous

 crc_err_flag behavior:
   Asserts ONE cycle on CS rising edge when the received CRC
   does not match the recomputed CRC over the 40-bit payload.
   This now detects errors on the INTERNAL averaged frame,
   not just the raw sensor A frame.

 Output validity:
   aqi_value / category are computed straight from the receive shift
   register, which is cleared two clocks after CS rises. They are valid in
   the clock cycle where CS is first seen high, and spi_top samples them in
   exactly that cycle (first cycle of LATCH_DATA).

 Inputs:
   clk          - System clock (100 MHz)
   rst          - Active-high async reset
   sclk         - SPI clock from master (synchronized internally)
   cs           - SPI chip-select (active-low), driven by cs2
   mosi         - SPI data in (averaged frame from spi_top)

 Outputs:
   aqi_value    - 9-bit final AQI (0–500), sourced from averaged data
   category     - 3-bit EPA health category (0–5)
   crc_err_flag - One-cycle pulse on CRC mismatch of internal frame
*****************************************************************************/

module spi_processing_slave(
    input  wire        clk,
    input  wire        rst,
    input  wire        sclk,
    input  wire        cs,
    input  wire        mosi,
    output wire [8:0]  aqi_value,
    output wire [2:0]  category,
    output wire        crc_err_flag
);

// ---------------------------------------------------------------------------
// 2-FF synchronizer for external SCLK
// ---------------------------------------------------------------------------
reg sclk_q1, sclk_q2;
always @(posedge clk or posedge rst) begin
    if (rst) begin sclk_q1 <= 1'b0; sclk_q2 <= 1'b0; end
    else     begin sclk_q1 <= sclk;  sclk_q2 <= sclk_q1; end
end
wire sclk_rise = (sclk_q1 == 1'b1) && (sclk_q2 == 1'b0);

// ---------------------------------------------------------------------------
// CS rising-edge detector (2-FF)
// ---------------------------------------------------------------------------
reg cs_q1, cs_q2;
always @(posedge clk or posedge rst) begin
    if (rst) begin cs_q1 <= 1'b1; cs_q2 <= 1'b1; end
    else     begin cs_q1 <= cs;    cs_q2 <= cs_q1; end
end
wire cs_rise = (cs == 1'b1) && (cs_q1 == 1'b0);

// ---------------------------------------------------------------------------
// 50-bit SPI shift register — accumulates the averaged frame
// ---------------------------------------------------------------------------
reg [49:0] rx_reg;
always @(posedge clk or posedge rst) begin
    if      (rst)       rx_reg <= 50'b0;
    else if (cs_q1)    rx_reg <= 50'b0;
    else if (sclk_rise) rx_reg <= {rx_reg[48:0], mosi};
end

// Decode received averaged concentrations + CRC
wire [9:0] avg_pm25 = rx_reg[49:40];
wire [9:0] avg_pm10 = rx_reg[39:30];
wire [9:0] avg_voc  = rx_reg[29:20];
wire [9:0] avg_co   = rx_reg[19:10];
wire [7:0] rx_crc   = rx_reg[ 7: 0];

wire [39:0] data_40bit = {avg_pm25, avg_pm10, avg_voc, avg_co};

// ---------------------------------------------------------------------------
// CRC-8 recomputation (combinational, poly=0x07, seed=0xFF)
// Validates the INTERNAL forwarded frame's CRC (not sensor A raw frame)
// ---------------------------------------------------------------------------
reg [7:0] crc8_calc;
integer b;
always @(*) begin
    crc8_calc = 8'hFF;
    for (b = 39; b >= 0; b = b - 1) begin
        if (crc8_calc[7] ^ data_40bit[b])
            crc8_calc = (crc8_calc << 1) ^ 8'h07;
        else
            crc8_calc = (crc8_calc << 1);
    end
end

wire crc_ok = (rx_crc == crc8_calc);
assign crc_err_flag = cs_rise & ~crc_ok;

// ---------------------------------------------------------------------------
// Pipeline Stage 1: breakpoint segment selection
// Operates on avg_pm25 / avg_pm10 from the averaged payload
// ---------------------------------------------------------------------------
reg [10:0] s1_sf_25,   s1_sf_10;
reg  [9:0] s1_diff_25, s1_diff_10;
reg  [8:0] s1_ilo_25,  s1_ilo_10;
reg        s1_sat_25,  s1_sat_10;
reg  [8:0] s1_aqi_voc, s1_aqi_co;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        s1_sf_25 <= 11'd0; s1_sf_10 <= 11'd0;
        s1_diff_25 <= 10'd0; s1_diff_10 <= 10'd0;
        s1_ilo_25  <=  9'd0; s1_ilo_10  <=  9'd0;
        s1_sat_25  <=  1'b0; s1_sat_10  <=  1'b0;
        s1_aqi_voc <=  9'd0; s1_aqi_co  <=  9'd0;
    end else begin
        // PM2.5 segment selection
        if      (avg_pm25 <= 10'd12) begin
            s1_sf_25 <= 11'd1067; s1_diff_25 <= avg_pm25;
            s1_ilo_25 <=  9'd0;   s1_sat_25  <= 1'b0;
        end else if (avg_pm25 <= 10'd35) begin
            s1_sf_25 <= 11'd570;  s1_diff_25 <= avg_pm25 - 10'd13;
            s1_ilo_25 <=  9'd51;  s1_sat_25  <= 1'b0;
        end else if (avg_pm25 <= 10'd55) begin
            s1_sf_25 <= 11'd660;  s1_diff_25 <= avg_pm25 - 10'd36;
            s1_ilo_25 <=  9'd101; s1_sat_25  <= 1'b0;
        end else if (avg_pm25 <= 10'd150) begin
            s1_sf_25 <= 11'd133;  s1_diff_25 <= avg_pm25 - 10'd56;
            s1_ilo_25 <=  9'd151; s1_sat_25  <= 1'b0;
        end else if (avg_pm25 <= 10'd250) begin
            s1_sf_25 <= 11'd256;  s1_diff_25 <= avg_pm25 - 10'd151;
            s1_ilo_25 <=  9'd201; s1_sat_25  <= 1'b0;
        end else if (avg_pm25 <= 10'd350) begin
            s1_sf_25 <= 11'd256;  s1_diff_25 <= avg_pm25 - 10'd251;
            s1_ilo_25 <=  9'd301; s1_sat_25  <= 1'b0;
        end else if (avg_pm25 <= 10'd499) begin
            s1_sf_25 <= 11'd171;  s1_diff_25 <= avg_pm25 - 10'd351;
            s1_ilo_25 <=  9'd401; s1_sat_25  <= 1'b0;
        end else begin
            s1_sf_25 <= 11'd0; s1_diff_25 <= 10'd0;
            s1_ilo_25 <= 9'd0; s1_sat_25  <= 1'b1;
        end

        // PM10 segment selection
        if      (avg_pm10 <= 10'd54) begin
            s1_sf_10 <= 11'd237; s1_diff_10 <= avg_pm10;
            s1_ilo_10 <=  9'd0;  s1_sat_10  <= 1'b0;
        end else if (avg_pm10 <= 10'd154) begin
            s1_sf_10 <= 11'd127; s1_diff_10 <= avg_pm10 - 10'd55;
            s1_ilo_10 <=  9'd51; s1_sat_10  <= 1'b0;
        end else if (avg_pm10 <= 10'd254) begin
            s1_sf_10 <= 11'd127; s1_diff_10 <= avg_pm10 - 10'd155;
            s1_ilo_10 <=  9'd101; s1_sat_10 <= 1'b0;
        end else if (avg_pm10 <= 10'd354) begin
            s1_sf_10 <= 11'd127; s1_diff_10 <= avg_pm10 - 10'd255;
            s1_ilo_10 <=  9'd151; s1_sat_10 <= 1'b0;
        end else if (avg_pm10 <= 10'd424) begin
            s1_sf_10 <= 11'd367; s1_diff_10 <= avg_pm10 - 10'd355;
            s1_ilo_10 <=  9'd201; s1_sat_10 <= 1'b0;
        end else if (avg_pm10 <= 10'd504) begin
            s1_sf_10 <= 11'd321; s1_diff_10 <= avg_pm10 - 10'd425;
            s1_ilo_10 <=  9'd301; s1_sat_10 <= 1'b0;
        end else if (avg_pm10 <= 10'd599) begin
            s1_sf_10 <= 11'd270; s1_diff_10 <= avg_pm10 - 10'd505;
            s1_ilo_10 <=  9'd401; s1_sat_10 <= 1'b0;
        end else begin
            s1_sf_10 <= 11'd0; s1_diff_10 <= 10'd0;
            s1_ilo_10 <= 9'd0; s1_sat_10  <= 1'b1;
        end

        // VOC and CO: simplified AQI = value >> 1
        s1_aqi_voc <= avg_voc[9:1];
        s1_aqi_co  <= avg_co[9:1];
    end
end

// ---------------------------------------------------------------------------
// Pipeline Stage 2: multiply-accumulate
// ---------------------------------------------------------------------------
reg [8:0] s2_aqi_pm25, s2_aqi_pm10, s2_aqi_voc, s2_aqi_co;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        s2_aqi_pm25 <= 9'd0; s2_aqi_pm10 <= 9'd0;
        s2_aqi_voc  <= 9'd0; s2_aqi_co   <= 9'd0;
    end else begin
        if (s1_sat_25) begin
            s2_aqi_pm25 <= 9'd500;
        end else begin : pm25_mac
            reg [20:0] prod_25;
            prod_25     = s1_sf_25 * s1_diff_25;
            s2_aqi_pm25 <= prod_25[16:8] + s1_ilo_25;
        end

        if (s1_sat_10) begin
            s2_aqi_pm10 <= 9'd500;
        end else begin : pm10_mac
            reg [20:0] prod_10;
            prod_10     = s1_sf_10 * s1_diff_10;
            s2_aqi_pm10 <= prod_10[16:8] + s1_ilo_10;
        end

        s2_aqi_voc <= s1_aqi_voc;
        s2_aqi_co  <= s1_aqi_co;
    end
end

// ---------------------------------------------------------------------------
// Final AQI: max of all 4 channels, clamped to 500
// ---------------------------------------------------------------------------
wire [8:0] max_25_10  = (s2_aqi_pm25 >= s2_aqi_pm10) ? s2_aqi_pm25 : s2_aqi_pm10;
wire [8:0] max_voc_co = (s2_aqi_voc  >= s2_aqi_co)   ? s2_aqi_voc  : s2_aqi_co;
wire [8:0] aqi_raw    = (max_25_10   >= max_voc_co)   ? max_25_10   : max_voc_co;
wire [8:0] aqi_clamped = (aqi_raw > 9'd500) ? 9'd500 : aqi_raw;

assign aqi_value = aqi_clamped;

// ---------------------------------------------------------------------------
// Category classification
// ---------------------------------------------------------------------------
reg [2:0] w_category;
always @(*) begin
    if      (aqi_clamped <= 9'd50)  w_category = 3'd0; // Good
    else if (aqi_clamped <= 9'd100) w_category = 3'd1; // Moderate
    else if (aqi_clamped <= 9'd200) w_category = 3'd2; // Unhealthy
    else if (aqi_clamped <= 9'd300) w_category = 3'd3; // Severe
    else if (aqi_clamped <= 9'd400) w_category = 3'd4; // Very Severe
    else                            w_category = 3'd5; // Hazardous
end

assign category = w_category;

endmodule