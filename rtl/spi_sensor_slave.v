/*****************************************************************************
 Module: spi_sensor_slave
 Purpose: Sensor data framer with input validation, linear scaling,
          5-stage pipelined CRC-8 generation, and SPI shift-out.

 Description:
   Validates signed 13-bit raw ADC inputs (clamps negatives to zero),
   scales to 10-bit physical units (PM2.5*500, PM10*600, VOC*1000,
   CO*1000 all >>12), computes CRC-8 (poly 0x07, seed 0xFF) over the
   40-bit data payload using a 5-stage pipeline with data shadowing for
   coherency, and loads a 50-bit SPI frame on CS assertion.

 Scaling: value = (safe_raw * full_scale) >> 12
 CRC:     poly 0x07, seed 0xFF, 40-bit payload MSB-first, 5 pipeline
          stages each processing one byte using shadowed data bytes so
          all stages operate on one coherent snapshot.

 Frame layout: [49:40]PM2.5 [39:30]PM10 [29:20]VOC [19:10]CO [9:8]00 [7:0]CRC

 Three instances in spi_top (sensors A, B, C).

 Inputs:
   clk       - system clock
   rst       - active-high async reset
   sclk      - SPI clock from master
   cs        - SPI chip-select (active-low)
   raw_pm25  - signed 13-bit raw ADC for PM2.5
   raw_pm10  - signed 13-bit raw ADC for PM10
   raw_voc   - signed 13-bit raw ADC for VOC
   raw_co    - signed 13-bit raw ADC for CO

 Outputs:
   miso        - SPI data out (active when cs low)
   input_err_o - per-channel error flags {PM2.5, PM10, VOC, CO}
*****************************************************************************/

module spi_sensor_slave(
    input  wire        clk,
    input  wire        rst,
    input  wire        sclk,
    input  wire        cs,

    input  wire signed [12:0] raw_pm25,
    input  wire signed [12:0] raw_pm10,
    input  wire signed [12:0] raw_voc,
    input  wire signed [12:0] raw_co,

    output wire        miso,
    output wire [3:0]  input_err_o
);

wire pm25_bad = (raw_pm25 <= 13'sh0);
wire pm10_bad = (raw_pm10 <= 13'sh0);
wire voc_bad  = (raw_voc  <= 13'sh0);
wire co_bad   = (raw_co   <= 13'sh0);

// Bit order: [3]=PM2.5, [2]=PM10, [1]=VOC, [0]=CO
assign input_err_o = { pm25_bad, pm10_bad, voc_bad, co_bad };

wire [11:0] safe_pm25 = pm25_bad ? 12'd0 : raw_pm25[11:0];
wire [11:0] safe_pm10 = pm10_bad ? 12'd0 : raw_pm10[11:0];
wire [11:0] safe_voc  = voc_bad  ? 12'd0 : raw_voc [11:0];
wire [11:0] safe_co   = co_bad   ? 12'd0 : raw_co  [11:0];

wire [23:0] pm25_calc = {12'b0, safe_pm25} * 24'd500;
wire [23:0] pm10_calc = {12'b0, safe_pm10} * 24'd600;
wire [23:0] voc_calc  = {12'b0, safe_voc}  * 24'd1000;
wire [23:0] co_calc   = {12'b0, safe_co}   * 24'd1000;

wire [9:0] pm25_val = pm25_calc[21:12];
wire [9:0] pm10_val = pm10_calc[21:12];
wire [9:0] voc_val  = voc_calc [21:12];
wire [9:0] co_val   = co_calc  [21:12];

function [7:0] crc8_step8;
    input [7:0] state;
    input [7:0] data8;
    integer i;
    reg [7:0] s;
    begin
        s = state;
        for (i = 7; i >= 0; i = i - 1) begin
            if (s[7] ^ data8[i])
                s = (s << 1) ^ 8'h07;
            else
                s = (s << 1);
        end
        crc8_step8 = s;
    end
endfunction

wire [39:0] data_40bit = {pm25_val, pm10_val, voc_val, co_val};

// CRC-8 pipeline: stage 0 is combinational seed, stages 1-5 are registered
wire [7:0] crc_s0_in = 8'hFF;
wire [7:0] crc_s1_in = crc8_step8(crc_s0_in,       data_40bit[39:32]);
wire [7:0] crc_s2_in;
wire [7:0] crc_s3_in;
wire [7:0] crc_s4_in;
wire [7:0] crc_s5_in;

reg [7:0] crc_p1, crc_p2, crc_p3, crc_p4;
reg [7:0] crc8_reg;
reg [7:0] byte1_s1, byte2_s1, byte3_s1, byte4_s1;
reg [7:0] byte2_s2, byte3_s2, byte4_s2;
reg [7:0] byte3_s3, byte4_s3;
reg [7:0] byte4_s4;

// Shadowed data bytes feed each pipeline stage for coherency
assign crc_s2_in = crc8_step8(crc_p1, byte1_s1);
assign crc_s3_in = crc8_step8(crc_p2, byte2_s2);
assign crc_s4_in = crc8_step8(crc_p3, byte3_s3);
assign crc_s5_in = crc8_step8(crc_p4, byte4_s4);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        crc_p1   <= 8'hFF;
        crc_p2   <= 8'hFF;
        crc_p3   <= 8'hFF;
        crc_p4   <= 8'hFF;
        crc8_reg <= 8'hFF;
        byte1_s1 <= 8'h00;
        byte2_s1 <= 8'h00;
        byte3_s1 <= 8'h00;
        byte4_s1 <= 8'h00;
        byte2_s2 <= 8'h00;
        byte3_s2 <= 8'h00;
        byte4_s2 <= 8'h00;
        byte3_s3 <= 8'h00;
        byte4_s3 <= 8'h00;
        byte4_s4 <= 8'h00;
    end else begin
        byte1_s1 <= data_40bit[31:24];
        byte2_s1 <= data_40bit[23:16];
        byte3_s1 <= data_40bit[15: 8];
        byte4_s1 <= data_40bit[ 7: 0];
        crc_p1   <= crc_s1_in;

        byte2_s2 <= byte2_s1;
        byte3_s2 <= byte3_s1;
        byte4_s2 <= byte4_s1;
        crc_p2   <= crc_s2_in;

        byte3_s3 <= byte3_s2;
        byte4_s3 <= byte4_s2;
        crc_p3   <= crc_s3_in;

        byte4_s4 <= byte4_s3;
        crc_p4   <= crc_s4_in;
        crc8_reg <= crc_s5_in;
    end
end

// 2-FF synchronizer for external SCLK
reg sclk_q1, sclk_q2;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        sclk_q1 <= 1'b0;
        sclk_q2 <= 1'b0;
    end else begin
        sclk_q1 <= sclk;
        sclk_q2 <= sclk_q1;
    end
end
wire sclk_falling = (sclk_q2 == 1'b1) && (sclk_q1 == 1'b0);

reg [49:0] shift_reg;

always @(posedge clk or posedge rst) begin
    if (rst)
        shift_reg <= 50'b0;
    else if (cs)
        shift_reg <= {pm25_val, pm10_val, voc_val, co_val, 2'b00, crc8_reg};
    else if (sclk_falling)
        shift_reg <= {shift_reg[48:0], 1'b0};
end

assign miso = (cs == 1'b0) ? shift_reg[49] : 1'b0;

endmodule
