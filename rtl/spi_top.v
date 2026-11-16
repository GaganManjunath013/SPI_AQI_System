
/*****************************************************************************
 Module: spi_top  (Processing-Centric Architecture — v4.0)
 Purpose: Top-level orchestrator for the SPI Air-Quality Monitoring System.

 Architectural Change (v4.0):
   AQI computation is fully offloaded to spi_processing_slave.
   spi_top now:
     1. Validates CRC-8 of every incoming sensor frame (A, B, C).
     2. Applies TMR median voting on validated concentrations.
     3. Applies 8-sample MAF per channel.
     4. Packs averaged concentrations into a NEW 50-bit frame.
     5. Generates a NEW CRC-8 over that averaged frame.
     6. Forwards averaged frame + CRC to processing slave via SPI.
     7. Receives final aqi_value and category BACK from the slave
        through registered output wires (proc_aqi, proc_cat).
     8. The processing slave also asserts proc_crc_err if the
        internal forwarded frame's CRC fails.

 Internal Transfer Frame (50-bit, same layout as sensor frame):
   [49:40] avg_pm25  [39:30] avg_pm10  [29:20] avg_voc
   [19:10] avg_co    [9:8]   2'b00     [7:0]   CRC-8

 Timing notes:
   - The median voter result is registered in MEDIAN_VOTE; MAF_UPDATE and the
     sensor-fault comparators work from that register.
   - Everything that samples the processing slave (AQI latch, alarm counters,
     AQI moving average, trend flag) does so in the first LATCH_DATA cycle.

 FSM states (extended):
   IDLE(0) -> READ_SENS_A(1) -> WAIT_SENS_A(2) ->
   CRC_CHK_A(18) -> READ_SENS_B(3) -> WAIT_SENS_B(4) ->
   CRC_CHK_B(19) -> READ_SENS_C(5) -> WAIT_SENS_C(6) ->
   CRC_CHK_C(20) -> MEDIAN_VOTE(7) -> MAF_UPDATE(21) ->
   BUILD_PROC_FRAME(22) -> SEND_PROC(8) -> WAIT_PROC(9) ->
   LATCH_DATA(10) -> CHECK_CMD(11)
   BRAM_WAIT(13) -> BRAM_RD_WAIT(17) -> BRAM_LOAD(12)
   FD_RD_REQ(14) -> FD_RD_WAIT(15) -> FD_LOAD(16)
*****************************************************************************/

module spi_top #(
    parameter DATA_WIDTH      = 50,
    parameter CLKS_PER_BIT    = 109,
    parameter FAULT_THRESH    = 10'd50,
    parameter TREND_THRESHOLD = 9'd20,
    parameter ALARM_ON_THRESH  = 3'd3,
    parameter ALARM_OFF_THRESH = 3'd2,
    parameter N_ON             = 4'd4,
    parameter N_OFF            = 4'd8,
    parameter MAF_DEPTH = 8,
    parameter MAF_N     = 3
)(
    input  wire        clk_i,
    input  wire        rst_i,
    input  wire        start_i,
    input  wire signed [12:0] raw_pm25_i, raw_pm10_i, raw_voc_i, raw_co_i,
    input  wire signed [12:0] raw_pm25_b_i, raw_pm10_b_i, raw_voc_b_i, raw_co_b_i,
    input  wire signed [12:0] raw_pm25_c_i, raw_pm10_c_i, raw_voc_c_i, raw_co_c_i,
    input  wire        miso_a_i, miso_b_i, miso_c_i,
    input  wire        sleep_i,
    output reg         wakeup_o,
    output wire        uart_tx_o,
    input  wire        uart_rx_i,
    output reg         done_o,
    output wire [15:0] aqi_out_o,
    output wire [2:0]  cat_out_o,
    output wire        alarm_o,
    output wire        crc_err_o,
    output wire [3:0]  input_err_o,
    output wire        sensor_timeout_o,
    output reg  [2:0]  sensor_fault_o,
    output reg         cs3, cs4,
    input  wire        sclk_s_i, cs_s_i, mosi_s_i,
    output wire        miso_s_o,
    output reg         trend_rising_o,
    output reg         flash_sclk_o, flash_cs_o, flash_mosi_o,
    input  wire        flash_miso_i,
    input  wire        bod_i,
    output wire        bod_flag_o,
    output wire        flash_write_err_o
);

// ---------------------------------------------------------------------------
// Forward Declarations (Moved to top to prevent implicit wire errors)
// ---------------------------------------------------------------------------
reg  [9:0]  fault_thresh_dyn;
reg  [8:0]  trend_thresh_dyn;
wire        cmd_get_live, cmd_dump_log, cmd_clear_log, cmd_wake, cmd_flash_dump;
wire [6:0]  bram_entry_count;
wire [11:0] flash_wr_addr_out;
wire [8:0]  flash_rec_count = flash_wr_addr_out[11:3];
reg  [8:0]  fd_idx;
reg  [6:0]  dump_cnt;

// ---------------------------------------------------------------------------
// FSM State Encoding
// ---------------------------------------------------------------------------
localparam IDLE             = 7'd0;
localparam READ_SENS_A      = 7'd1;
localparam WAIT_SENS_A      = 7'd2;
localparam READ_SENS_B      = 7'd3;
localparam WAIT_SENS_B      = 7'd4;
localparam READ_SENS_C      = 7'd5;
localparam WAIT_SENS_C      = 7'd6;
localparam MEDIAN_VOTE      = 7'd7;
localparam SEND_PROC        = 7'd8;
localparam WAIT_PROC        = 7'd9;
localparam LATCH_DATA       = 7'd10;
localparam CHECK_CMD        = 7'd11;
localparam BRAM_LOAD        = 7'd12;
localparam BRAM_WAIT        = 7'd13;
localparam FD_RD_REQ        = 7'd14;
localparam FD_RD_WAIT       = 7'd15;
localparam FD_LOAD          = 7'd16;
localparam BRAM_RD_WAIT     = 7'd17;
// New states for processing-centric arch
localparam CRC_CHK_A        = 7'd18;
localparam CRC_CHK_B        = 7'd19;
localparam CRC_CHK_C        = 7'd20;
localparam MAF_UPDATE       = 7'd21;
localparam BUILD_PROC_FRAME = 7'd22;

reg [6:0] state;
reg        master_start;
reg  [DATA_WIDTH-1:0] master_tx_data;
wire [DATA_WIDTH-1:0] master_rx_data;
wire master_done, master_busy, mosi, sclk, miso_sensor_a;
reg  cs1, cs2;

wire miso_active = (state == WAIT_SENS_A) ? miso_a_i :
                   (state == WAIT_SENS_B) ? miso_b_i :
                   (state == WAIT_SENS_C) ? miso_c_i :
                   (state == WAIT_PROC)   ? 1'b0     : 1'b0;

wire [3:0] sens_a_input_err, sens_b_input_err, sens_c_input_err;

spi_sensor_slave sensor_a (.clk(clk_i), .rst(rst_i), .sclk(sclk), .cs(cs1),
    .raw_pm25(raw_pm25_i), .raw_pm10(raw_pm10_i), .raw_voc(raw_voc_i), .raw_co(raw_co_i),
    .miso(miso_sensor_a), .input_err_o(sens_a_input_err));
spi_sensor_slave sensor_b (.clk(clk_i), .rst(rst_i), .sclk(sclk), .cs(cs3),
    .raw_pm25(raw_pm25_b_i), .raw_pm10(raw_pm10_b_i), .raw_voc(raw_voc_b_i), .raw_co(raw_co_b_i),
    .miso(), .input_err_o(sens_b_input_err));
spi_sensor_slave sensor_c (.clk(clk_i), .rst(rst_i), .sclk(sclk), .cs(cs4),
    .raw_pm25(raw_pm25_c_i), .raw_pm10(raw_pm10_c_i), .raw_voc(raw_voc_c_i), .raw_co(raw_co_c_i),
    .miso(), .input_err_o(sens_c_input_err));

assign input_err_o = sens_a_input_err | sens_b_input_err | sens_c_input_err;

// ---------------------------------------------------------------------------
// Sleep / Clock Gating (unchanged)
// ---------------------------------------------------------------------------
reg sleep_s1, sleep_s2;
always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin sleep_s1 <= 1'b0; sleep_s2 <= 1'b0; end
    else       begin sleep_s1 <= sleep_i; sleep_s2 <= sleep_s1; end
end
wire sleep_sync = sleep_s2;

reg sleep_latch;
always @(negedge clk_i or posedge rst_i) begin
    if (rst_i) sleep_latch <= 1'b0;
    else       sleep_latch <= sleep_sync;
end
wire gated_clk = clk_i & ~sleep_latch;

spi_master #(.DATA_WIDTH(DATA_WIDTH)) master (
    .clk_i(gated_clk), .rst_i(rst_i), .start_i(master_start),
    .miso_i(miso_active), .tx_data_i(master_tx_data),
    .mosi_o(mosi), .sclk_o(sclk), .rx_data_o(master_rx_data),
    .busy_o(master_busy), .done_o(master_done));

// ---------------------------------------------------------------------------
// Processing Slave — receives averaged frame, returns AQI + category
// ---------------------------------------------------------------------------
wire [8:0] proc_aqi;
wire [2:0] proc_cat;
wire       proc_crc_err;

spi_processing_slave proc (
    .clk(clk_i), .rst(rst_i),
    .sclk(sclk), .cs(cs2), .mosi(mosi),
    .aqi_value(proc_aqi), .category(proc_cat),
    .crc_err_flag(proc_crc_err));

// ---------------------------------------------------------------------------
// CRC-8 verification (poly=0x07, init=0xFF) over sensor frames
// Used to gate which sensor frames enter the TMR voter
// ---------------------------------------------------------------------------
function [7:0] crc8_top;
    input [39:0] payload;
    reg   [7:0]  crc;
    integer      i;
    begin
        crc = 8'hFF;
        for (i = 39; i >= 0; i = i - 1) begin
            if (crc[7] ^ payload[i])
                crc = (crc << 1) ^ 8'h07;
            else
                crc = (crc << 1);
        end
        crc8_top = crc;
    end
endfunction

// ---------------------------------------------------------------------------
// CRC-8 generation for averaged frame forwarded to processing slave
// Same polynomial / seed
// ---------------------------------------------------------------------------
function [7:0] crc8_gen;
    input [39:0] payload;
    reg   [7:0]  crc;
    integer      i;
    begin
        crc = 8'hFF;
        for (i = 39; i >= 0; i = i - 1) begin
            if (crc[7] ^ payload[i])
                crc = (crc << 1) ^ 8'h07;
            else
                crc = (crc << 1);
        end
        crc8_gen = crc;
    end
endfunction

// ---------------------------------------------------------------------------
// Received sensor frames
// ---------------------------------------------------------------------------
reg [DATA_WIDTH-1:0] sens_a_frame, sens_b_frame, sens_c_frame;

// CRC validation flags per sensor (registered in CRC_CHK_x states)
reg sens_a_crc_ok, sens_b_crc_ok, sens_c_crc_ok;

// Sensor-level CRC check wires (combinational on latched frames)
wire [7:0] crc_calc_a = crc8_top(sens_a_frame[49:10]);
wire [7:0] crc_calc_b = crc8_top(sens_b_frame[49:10]);
wire [7:0] crc_calc_c = crc8_top(sens_c_frame[49:10]);

wire crc_ok_a = (crc_calc_a == sens_a_frame[7:0]);
wire crc_ok_b = (crc_calc_b == sens_b_frame[7:0]);
wire crc_ok_c = (crc_calc_c == sens_c_frame[7:0]);

// Expose sensor CRC error flags to testbench
wire sens_a_crc_err = ~sens_a_crc_ok;
wire sens_b_crc_err = ~sens_b_crc_ok;
wire sens_c_crc_err = ~sens_c_crc_ok;

// ---------------------------------------------------------------------------
// TMR: channel fields from each sensor frame
// If a sensor's CRC failed, its values are replaced with the median of the
// other two to minimise distortion (graceful degradation).
// ---------------------------------------------------------------------------
wire [9:0] pm25_a_raw = sens_a_frame[49:40], pm10_a_raw = sens_a_frame[39:30],
           voc_a_raw  = sens_a_frame[29:20], co_a_raw   = sens_a_frame[19:10];
wire [9:0] pm25_b_raw = sens_b_frame[49:40], pm10_b_raw = sens_b_frame[39:30],
           voc_b_raw  = sens_b_frame[29:20], co_b_raw   = sens_b_frame[19:10];
wire [9:0] pm25_c_raw = sens_c_frame[49:40], pm10_c_raw = sens_c_frame[39:30],
           voc_c_raw  = sens_c_frame[29:20], co_c_raw   = sens_c_frame[19:10];

// Substitute failed sensor channel with midpoint of valid pair to keep voter
// working even when one sensor frame had a CRC fault
wire [9:0] pm25_a = sens_a_crc_ok ? pm25_a_raw : ({1'b0,pm25_b_raw} + {1'b0,pm25_c_raw}) >> 1;
wire [9:0] pm25_b = sens_b_crc_ok ? pm25_b_raw : ({1'b0,pm25_a_raw} + {1'b0,pm25_c_raw}) >> 1;
wire [9:0] pm25_c = sens_c_crc_ok ? pm25_c_raw : ({1'b0,pm25_a_raw} + {1'b0,pm25_b_raw}) >> 1;

wire [9:0] pm10_a = sens_a_crc_ok ? pm10_a_raw : ({1'b0,pm10_b_raw} + {1'b0,pm10_c_raw}) >> 1;
wire [9:0] pm10_b = sens_b_crc_ok ? pm10_b_raw : ({1'b0,pm10_a_raw} + {1'b0,pm10_c_raw}) >> 1;
wire [9:0] pm10_c = sens_c_crc_ok ? pm10_c_raw : ({1'b0,pm10_a_raw} + {1'b0,pm10_b_raw}) >> 1;

wire [9:0] voc_a  = sens_a_crc_ok ? voc_a_raw  : ({1'b0,voc_b_raw}  + {1'b0,voc_c_raw})  >> 1;
wire [9:0] voc_b  = sens_b_crc_ok ? voc_b_raw  : ({1'b0,voc_a_raw}  + {1'b0,voc_c_raw})  >> 1;
wire [9:0] voc_c  = sens_c_crc_ok ? voc_c_raw  : ({1'b0,voc_a_raw}  + {1'b0,voc_b_raw})  >> 1;

wire [9:0] co_a   = sens_a_crc_ok ? co_a_raw   : ({1'b0,co_b_raw}   + {1'b0,co_c_raw})   >> 1;
wire [9:0] co_b   = sens_b_crc_ok ? co_b_raw   : ({1'b0,co_a_raw}   + {1'b0,co_c_raw})   >> 1;
wire [9:0] co_c   = sens_c_crc_ok ? co_c_raw   : ({1'b0,co_a_raw}   + {1'b0,co_b_raw})   >> 1;

// ---------------------------------------------------------------------------
// Median voter
// ---------------------------------------------------------------------------
function [9:0] median3;
    input [9:0] a, b, c;
    begin
        median3 = (a >= b) ?
                    ((b >= c) ? b : ((a >= c) ? c : a)) :
                    ((a >= c) ? a : ((b >= c) ? c : b));
    end
endfunction

wire [9:0] med_pm25 = median3(pm25_a, pm25_b, pm25_c);
wire [9:0] med_pm10 = median3(pm10_a, pm10_b, pm10_c);
wire [9:0] med_voc  = median3(voc_a,  voc_b,  voc_c);
wire [9:0] med_co   = median3(co_a,   co_b,   co_c);

// ---------------------------------------------------------------------------
// Voter output register
// The CRC-fail substitution and the median compare tree are a long
// combinational cone. Their result is captured here during MEDIAN_VOTE so the
// MAF adders and the deviation comparators start from a flop instead of
// sharing a cycle with that cone. The three frames and their CRC flags are
// all valid by MEDIAN_VOTE, so the registered values are always coherent.
// ---------------------------------------------------------------------------
reg [9:0] med_pm25_q, med_pm10_q, med_voc_q, med_co_q;
reg [9:0] pm25_a_q, pm25_b_q, pm25_c_q;

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
        med_pm25_q <= 10'd0; med_pm10_q <= 10'd0;
        med_voc_q  <= 10'd0; med_co_q   <= 10'd0;
        pm25_a_q   <= 10'd0; pm25_b_q   <= 10'd0; pm25_c_q <= 10'd0;
    end else if (state == MEDIAN_VOTE) begin
        med_pm25_q <= med_pm25; med_pm10_q <= med_pm10;
        med_voc_q  <= med_voc;  med_co_q   <= med_co;
        pm25_a_q   <= pm25_a;   pm25_b_q   <= pm25_b;   pm25_c_q <= pm25_c;
    end
end

// TMR deviation (PM2.5 only), taken from the registered voter result
wire [10:0] sens_a_dev = (pm25_a_q >= med_pm25_q) ? (pm25_a_q - med_pm25_q) : (med_pm25_q - pm25_a_q);
wire [10:0] sens_b_dev = (pm25_b_q >= med_pm25_q) ? (pm25_b_q - med_pm25_q) : (med_pm25_q - pm25_b_q);
wire [10:0] sens_c_dev = (pm25_c_q >= med_pm25_q) ? (pm25_c_q - med_pm25_q) : (med_pm25_q - pm25_c_q);

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) sensor_fault_o <= 3'b000;
    else begin
        sensor_fault_o[2] <= (sens_a_dev > fault_thresh_dyn);
        sensor_fault_o[1] <= (sens_b_dev > fault_thresh_dyn);
        sensor_fault_o[0] <= (sens_c_dev > fault_thresh_dyn);
    end
end

// ---------------------------------------------------------------------------
// 8-sample MAF per channel (updated in MAF_UPDATE state)
// ---------------------------------------------------------------------------
reg [9:0]  maf_pm25 [0:MAF_DEPTH-1], maf_pm10 [0:MAF_DEPTH-1],
           maf_voc  [0:MAF_DEPTH-1], maf_co   [0:MAF_DEPTH-1];
reg [13:0] acc_pm25, acc_pm10, acc_voc, acc_co;
reg [2:0]  maf_ptr;
reg        maf_valid;

wire [9:0] avg_pm25 = acc_pm25[12:3];
wire [9:0] avg_pm10 = acc_pm10[12:3];
wire [9:0] avg_voc  = acc_voc [12:3];
wire [9:0] avg_co   = acc_co  [12:3];

integer maf_i;
always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
        maf_ptr  <= 3'd0; maf_valid <= 1'b0;
        acc_pm25 <= 14'd0; acc_pm10 <= 14'd0;
        acc_voc  <= 14'd0; acc_co   <= 14'd0;
        for (maf_i = 0; maf_i < MAF_DEPTH; maf_i = maf_i + 1) begin
            maf_pm25[maf_i] <= 10'd0; maf_pm10[maf_i] <= 10'd0;
            maf_voc [maf_i] <= 10'd0; maf_co  [maf_i] <= 10'd0;
        end
    end else if (state == MAF_UPDATE) begin
        if (!maf_valid) begin
            for (maf_i = 0; maf_i < MAF_DEPTH; maf_i = maf_i + 1) begin
                maf_pm25[maf_i] <= med_pm25_q; maf_pm10[maf_i] <= med_pm10_q;
                maf_voc [maf_i] <= med_voc_q;  maf_co  [maf_i] <= med_co_q;
            end
            acc_pm25  <= {4'b0, med_pm25_q} << MAF_N;
            acc_pm10  <= {4'b0, med_pm10_q} << MAF_N;
            acc_voc   <= {4'b0, med_voc_q}  << MAF_N;
            acc_co    <= {4'b0, med_co_q}   << MAF_N;
            maf_valid <= 1'b1;
        end else begin
            acc_pm25 <= acc_pm25 - maf_pm25[maf_ptr] + med_pm25_q;
            acc_pm10 <= acc_pm10 - maf_pm10[maf_ptr] + med_pm10_q;
            acc_voc  <= acc_voc  - maf_voc [maf_ptr] + med_voc_q;
            acc_co   <= acc_co   - maf_co  [maf_ptr] + med_co_q;
            maf_pm25[maf_ptr] <= med_pm25_q; maf_pm10[maf_ptr] <= med_pm10_q;
            maf_voc [maf_ptr] <= med_voc_q;  maf_co  [maf_ptr] <= med_co_q;
            maf_ptr <= maf_ptr + 1;
        end
    end
end

// ---------------------------------------------------------------------------
// Build Internal Transfer Frame (averaged concentrations + new CRC)
// Computed combinationally from avg values; latched in BUILD_PROC_FRAME state
// ---------------------------------------------------------------------------
wire [39:0] avg_payload      = {avg_pm25, avg_pm10, avg_voc, avg_co};
wire [7:0]  avg_crc          = crc8_gen(avg_payload);
wire [DATA_WIDTH-1:0] proc_frame_wire = {avg_pm25, avg_pm10, avg_voc, avg_co, 2'b00, avg_crc};

reg [DATA_WIDTH-1:0] proc_frame_latched;

// ---------------------------------------------------------------------------
// AQI / Category — now come ONLY from the processing slave
// ---------------------------------------------------------------------------
reg [8:0] aqi_latched;
reg [2:0] cat_latched;
reg       alarm_state, crc_err_latched, proc_crc_seen;
reg       latch_done_sent;

// LATCH_DATA can last several cycles when the UART FIFO is busy, but the
// processing slave only holds its result for a short window after CS rises.
// latch_first marks the first LATCH_DATA cycle; the AQI latch, the alarm
// counters, the AQI moving average and the trend flag sample on it only.
wire latch_first = (state == LATCH_DATA) && !latch_done_sent;

// Value reported in the live UART packet: straight from the slave in the first
// cycle, from the latched copy if the packet had to wait for FIFO space.
wire [8:0] live_aqi = latch_first ? proc_aqi : aqi_latched;
wire [2:0] live_cat = latch_first ? proc_cat : cat_latched;

assign aqi_out_o = {7'b0, aqi_latched};
assign cat_out_o = cat_latched;
assign alarm_o   = alarm_state;
// crc_err_o now ORs sensor-level CRC errors with the internal transfer CRC error
assign crc_err_o = crc_err_latched;

// ---------------------------------------------------------------------------
// 8-sample AQI MAF (for trend detection only)
// ---------------------------------------------------------------------------
reg [9:0]  maf_aqi [0:MAF_DEPTH-1];
reg [13:0] acc_aqi;
reg [2:0]  aqi_maf_ptr;
reg        aqi_maf_valid;
wire [8:0] avg_aqi = acc_aqi[12:3];

integer aqi_i;
always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
        acc_aqi <= 14'd0; aqi_maf_ptr <= 3'd0; aqi_maf_valid <= 1'b0;
        for (aqi_i = 0; aqi_i < MAF_DEPTH; aqi_i = aqi_i + 1) maf_aqi[aqi_i] <= 10'd0;
    end else if (latch_first) begin
        if (!aqi_maf_valid) begin
            for (aqi_i = 0; aqi_i < MAF_DEPTH; aqi_i = aqi_i + 1)
                maf_aqi[aqi_i] <= {1'b0, proc_aqi};
            acc_aqi       <= {5'b0, proc_aqi} << MAF_N;
            aqi_maf_valid <= 1'b1;
        end else begin
            acc_aqi <= acc_aqi - maf_aqi[aqi_maf_ptr] + {1'b0, proc_aqi};
            maf_aqi[aqi_maf_ptr] <= {1'b0, proc_aqi};
            aqi_maf_ptr <= aqi_maf_ptr + 1;
        end
    end
end

wire signed [10:0] trend_delta = {1'b0, proc_aqi} - {1'b0, avg_aqi};
// Evaluated once per measurement and held until the next one, so the flag is
// a stable level for the MCU register read instead of a few-cycle pulse.
always @(posedge clk_i or posedge rst_i) begin
    if (rst_i)            trend_rising_o <= 1'b0;
    else if (latch_first) trend_rising_o <= (trend_delta > $signed({1'b0, trend_thresh_dyn}));
end

// ---------------------------------------------------------------------------
// Alarm hysteresis (uses proc_cat latched as cat_latched)
// ---------------------------------------------------------------------------
reg [3:0] on_ctr, off_ctr;
reg       alarm_rising;

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
        on_ctr <= 4'd0; off_ctr <= 4'd0; alarm_state <= 1'b0; alarm_rising <= 1'b0;
    end else begin
        alarm_rising <= 1'b0;
        if (latch_first) begin
            if (!alarm_state) begin
                if (proc_cat >= ALARM_ON_THRESH) begin
                    on_ctr <= on_ctr + 1; off_ctr <= 4'd0;
                    if (on_ctr == N_ON - 1) begin alarm_state <= 1'b1; alarm_rising <= 1'b1; end
                end else on_ctr <= 4'd0;
            end else begin
                if (proc_cat < ALARM_OFF_THRESH) begin
                    off_ctr <= off_ctr + 1; on_ctr <= 4'd0;
                    if (off_ctr == N_OFF - 1) alarm_state <= 1'b0;
                end else off_ctr <= 4'd0;
            end
        end
    end
end

// ---------------------------------------------------------------------------
// Timestamp counter
// ---------------------------------------------------------------------------
reg [31:0] timestamp_cnt;
always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) timestamp_cnt <= 32'd0;
    else       timestamp_cnt <= timestamp_cnt + 1;
end

// ---------------------------------------------------------------------------
// Watchdog (unchanged)
// ---------------------------------------------------------------------------
reg [9:0] wdog_cnt;
reg       wdog_en;
wire      wdog_timeout = (wdog_cnt == 10'd1023);
reg       sensor_timeout_latched;

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) wdog_cnt <= 10'd0;
    else if (!wdog_en) wdog_cnt <= 10'd0;
    else if (!wdog_timeout) wdog_cnt <= wdog_cnt + 1;
end
assign sensor_timeout_o = wdog_timeout | sensor_timeout_latched;

// ---------------------------------------------------------------------------
// Sleep / Wake
// ---------------------------------------------------------------------------
reg sleep_prev;
always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin sleep_prev <= 1'b0; wakeup_o <= 1'b0; end
    else begin
        sleep_prev <= sleep_sync;
        wakeup_o   <= (sleep_prev && !sleep_sync) || cmd_wake;
    end
end

// ---------------------------------------------------------------------------
// 128-byte UART FIFO + push engine (unchanged)
// ---------------------------------------------------------------------------
parameter FIFO_DEPTH = 128;
reg [7:0]  fifo_mem [0:FIFO_DEPTH-1];
reg [6:0]  fifo_wr, fifo_rd;
wire       fifo_empty = (fifo_wr == fifo_rd);
wire       fifo_full  = ((fifo_wr + 7'd1) == fifo_rd);

reg        uart_start;
reg  [7:0] uart_tx_data;
wire       uart_done, uart_busy;

uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) uart_tx_inst (
    .clk(clk_i), .rst(rst_i), .tx_start(uart_start),
    .tx_data(uart_tx_data), .tx(uart_tx_o),
    .tx_done(uart_done), .tx_busy(uart_busy));

reg [7:0] push_buf [0:9];
reg [3:0] push_len;
reg       push_go;
reg [3:0] push_ptr;
wire      push_idle = (push_ptr == 4'd0);

integer fi2;
always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
        uart_start <= 1'b0; uart_tx_data <= 8'd0;
        fifo_rd <= 7'd0; fifo_wr <= 7'd0; push_ptr <= 4'd0;
        for (fi2 = 0; fi2 < FIFO_DEPTH; fi2 = fi2 + 1) fifo_mem[fi2] <= 8'd0;
    end else begin
        uart_start <= 1'b0;
        if (push_go && push_ptr == 4'd0) begin
            if (!fifo_full) begin
                fifo_mem[fifo_wr] <= push_buf[0];
                fifo_wr           <= fifo_wr + 7'd1;
                push_ptr          <= (push_len > 4'd1) ? 4'd1 : 4'd0;
            end
        end else if (push_ptr > 4'd0 && push_ptr < push_len) begin
            if (!fifo_full) begin
                fifo_mem[fifo_wr] <= push_buf[push_ptr];
                fifo_wr           <= fifo_wr + 7'd1;
                push_ptr          <= push_ptr + 4'd1;
            end
        end else if (push_ptr == push_len && push_len > 4'd0) begin
            push_ptr <= 4'd0;
        end
        if (!fifo_empty && !uart_busy && !uart_start) begin
            uart_tx_data <= fifo_mem[fifo_rd];
            fifo_rd      <= fifo_rd + 7'd1;
            uart_start   <= 1'b1;
        end
    end
end

// ---------------------------------------------------------------------------
// UART RX + command decode (unchanged)
// ---------------------------------------------------------------------------
wire rx_done;
wire [7:0] rx_data;

uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) rx_uart (
    .clk(clk_i), .rst(rst_i), .rx(uart_rx_i),
    .rx_done(rx_done), .rx_data(rx_data),
    .cmd_get_live_o(cmd_get_live), .cmd_dump_log_o(cmd_dump_log),
    .cmd_clear_log_o(cmd_clear_log), .cmd_wake_o(cmd_wake),
    .cmd_flash_dump_o(cmd_flash_dump));

reg cmd_pending_dump, cmd_pending_flash_dump, ack_pending, err_pending;
reg [7:0] err_code;
reg sensor_timeout_prev;

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) sensor_timeout_prev <= 1'b0;
    else       sensor_timeout_prev <= sensor_timeout_latched;
end

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
        cmd_pending_dump <= 1'b0; cmd_pending_flash_dump <= 1'b0;
        ack_pending <= 1'b0; err_pending <= 1'b0; err_code <= 8'h00;
    end else begin
        if (cmd_dump_log)   cmd_pending_dump       <= 1'b1;
        if (cmd_clear_log)  cmd_pending_dump       <= 1'b0;
        if (cmd_flash_dump) cmd_pending_flash_dump <= 1'b1;
        if (cmd_get_live || cmd_wake) ack_pending  <= 1'b1;
        if ((wdog_timeout || (sensor_timeout_latched && !sensor_timeout_prev))
            && state != CHECK_CMD) begin
            err_pending <= 1'b1;
            err_code    <= 8'h01;
        end
        if (state == BRAM_LOAD && dump_cnt + 7'd1 >= bram_entry_count) cmd_pending_dump <= 1'b0;
        if (state == FD_LOAD   && fd_idx   + 1'b1 >= flash_rec_count)  cmd_pending_flash_dump <= 1'b0;
        if ((state == IDLE || state == CHECK_CMD) && cmd_pending_dump && bram_entry_count == 0)
            cmd_pending_dump <= 1'b0;
        if ((state == IDLE || state == CHECK_CMD) && cmd_pending_flash_dump && flash_rec_count == 0)
            cmd_pending_flash_dump <= 1'b0;
        if (state == CHECK_CMD && ack_pending) ack_pending <= 1'b0;
        if (state == CHECK_CMD && err_pending) err_pending <= 1'b0;
    end
end

// ---------------------------------------------------------------------------
// BRAM alarm log (unchanged)
// ---------------------------------------------------------------------------
reg  [5:0]  bram_rd_addr;
wire [63:0] bram_rd_data;
wire [5:0]  bram_wr_ptr;
wire        bram_overflow, bram_ecc_sec, bram_ecc_ded;

alarm_bram bram (
    .clk(clk_i), .rst(rst_i),
    .alarm_rising_i(alarm_rising),
    .timestamp_i(timestamp_cnt),
    .aqi_i(aqi_latched), .category_i(cat_latched),
    .input_err_i(input_err_o), .crc_err_i(crc_err_latched),
    .alarm_state_i(alarm_state),
    .rd_addr_i(bram_rd_addr), .rd_data_o(bram_rd_data),
    .entry_count_o(bram_entry_count), .wr_ptr_o(bram_wr_ptr),
    .overflow_o(bram_overflow), .ecc_sec_o(bram_ecc_sec),
    .ecc_ded_o(bram_ecc_ded), .clr_i(cmd_clear_log));

// ---------------------------------------------------------------------------
// Flash write queue (unchanged)
// ---------------------------------------------------------------------------
reg        flash_wr_pending;
reg [63:0] latched_flash_write_data;
reg        flash_wr_overflow, flash_wr_hold_valid;
reg [63:0] flash_wr_hold_data;

wire flash_busy, flash_done;
wire [63:0] alarm_flash_entry = {
    timestamp_cnt,
    6'b0, aqi_latched,
    cat_latched, input_err_o, crc_err_latched, 1'b1,
    8'h00
};

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
        flash_wr_pending <= 1'b0; latched_flash_write_data <= 64'd0;
        flash_wr_hold_valid <= 1'b0; flash_wr_hold_data <= 64'd0;
        flash_wr_overflow <= 1'b0;
    end else begin
        if (flash_wr_pending && !flash_busy && !alarm_rising)
            flash_wr_pending <= 1'b0;
        if (!flash_wr_pending && !alarm_rising && flash_wr_hold_valid) begin
            flash_wr_pending         <= 1'b1;
            latched_flash_write_data <= flash_wr_hold_data;
            flash_wr_hold_valid      <= 1'b0;
        end
        if (alarm_rising) begin
            if (!flash_wr_pending) begin
                flash_wr_pending         <= 1'b1;
                latched_flash_write_data <= alarm_flash_entry;
            end else if (!flash_wr_hold_valid) begin
                flash_wr_hold_valid <= 1'b1;
                flash_wr_hold_data  <= alarm_flash_entry;
            end else begin
                flash_wr_hold_data <= alarm_flash_entry;
                flash_wr_overflow  <= 1'b1;
            end
        end
        if (cmd_clear_log) flash_wr_overflow <= 1'b0;
    end
end

reg        flash_rd_req;
reg [11:0] flash_rd_addr;
wire [63:0] flash_rd_data;
reg [63:0]  fd_entry;

wire flash_sclk_int, flash_cs_int, flash_mosi_int;

spi_flash_ctrl flash_ctrl (
    .clk(clk_i), .rst(rst_i),
    .write_req_i(flash_wr_pending), .write_data_i(latched_flash_write_data),
    .read_req_i(flash_rd_req), .read_addr_i(flash_rd_addr),
    .read_data_o(flash_rd_data), .busy_o(flash_busy), .done_o(flash_done),
    .flash_wr_addr_o(flash_wr_addr_out), .flash_write_err_o(flash_write_err_o),
    .bod_i(bod_i), .bod_flag_o(bod_flag_o),
    .flash_sclk(flash_sclk_int), .flash_cs(flash_cs_int),
    .flash_mosi(flash_mosi_int), .flash_miso(flash_miso_i));

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin flash_sclk_o <= 1'b0; flash_cs_o <= 1'b1; flash_mosi_o <= 1'b0; end
    else begin
        flash_sclk_o <= flash_sclk_int;
        flash_cs_o   <= flash_cs_int;
        flash_mosi_o <= flash_mosi_int;
    end
end

// ---------------------------------------------------------------------------
// MCU SPI slave (unchanged)
// ---------------------------------------------------------------------------
wire slave_wr_valid; wire [6:0] slave_wr_addr; wire [7:0] slave_wr_data;
spi_slave slave_inst (
    .clk(clk_i), .rst(rst_i),
    .sclk_s(sclk_s_i), .cs_s(cs_s_i), .mosi_s(mosi_s_i), .miso_s(miso_s_o),
    .reg_pm25({6'b0, avg_pm25}), .reg_pm10({6'b0, avg_pm10}),
    .reg_voc({6'b0, avg_voc}),   .reg_co({6'b0, avg_co}),
    .reg_aqi({7'b0, aqi_latched}), .reg_category({5'b0, cat_latched}),
    .reg_status({alarm_state, crc_err_latched, wdog_timeout, sleep_sync, 4'b0}),
    .reg_faults({5'b0, sensor_fault_o}), .reg_trend({7'b0, trend_rising_o}),
    .wr_valid_o(slave_wr_valid), .wr_addr_o(slave_wr_addr), .wr_data_o(slave_wr_data));

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin fault_thresh_dyn <= FAULT_THRESH; trend_thresh_dyn <= TREND_THRESHOLD; end
    else if (slave_wr_valid) case (slave_wr_addr)
        7'h07: fault_thresh_dyn <= {2'b0, slave_wr_data};
        7'h08: trend_thresh_dyn <= {1'b0, slave_wr_data};
        default: ;
    endcase
end

// ---------------------------------------------------------------------------
// BRAM dump helpers (unchanged)
// ---------------------------------------------------------------------------
reg [63:0] dump_entry;
wire [31:0] dump_ts     = dump_entry[63:32];
wire [7:0]  dump_aqi_lo = dump_entry[23:16];
reg  [5:0]  dump_idx;

// ---------------------------------------------------------------------------
// MAIN FSM
// ---------------------------------------------------------------------------
always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
        state <= IDLE; cs1 <= 1'b1; cs2 <= 1'b1; cs3 <= 1'b1; cs4 <= 1'b1;
        master_start <= 1'b0; master_tx_data <= {DATA_WIDTH{1'b0}};
        sens_a_frame <= {DATA_WIDTH{1'b0}};
        sens_b_frame <= {DATA_WIDTH{1'b0}};
        sens_c_frame <= {DATA_WIDTH{1'b0}};
        sens_a_crc_ok <= 1'b1; sens_b_crc_ok <= 1'b1; sens_c_crc_ok <= 1'b1;
        proc_frame_latched <= {DATA_WIDTH{1'b0}};
        aqi_latched <= 9'd0; cat_latched <= 3'd0;
        crc_err_latched <= 1'b0;
        proc_crc_seen <= 1'b0; done_o <= 1'b0; latch_done_sent <= 1'b0;
        wdog_en <= 1'b0; sensor_timeout_latched <= 1'b0;
        bram_rd_addr <= 6'd0;
        dump_idx <= 6'd0; dump_entry <= 64'd0; dump_cnt <= 7'd0;
        flash_rd_req <= 1'b0; flash_rd_addr <= 12'd0;
        fd_idx <= 9'd0; fd_entry <= 64'd0;
        push_go <= 1'b0; push_len <= 4'd0;
    end else begin
        master_start <= 1'b0; done_o <= 1'b0;
        flash_rd_req <= 1'b0; push_go <= 1'b0;

        case (state)

        // ------------------------------------------------------------------
        // IDLE — command dispatcher and start gate
        // ------------------------------------------------------------------
        IDLE: begin
            cs1 <= 1'b1; cs2 <= 1'b1; cs3 <= 1'b1; cs4 <= 1'b1; wdog_en <= 1'b0;
            if (cmd_clear_log) begin
                sensor_timeout_latched <= 1'b0; state <= IDLE;
            end else if (err_pending && push_idle) begin
                push_buf[0] <= 8'hEE; push_buf[1] <= err_code;
                push_len    <= 4'd2;  push_go <= 1'b1;
                state <= CHECK_CMD;
            end else if (ack_pending && push_idle) begin
                push_buf[0] <= 8'hCC;
                push_len    <= 4'd1; push_go <= 1'b1;
                state <= CHECK_CMD;
            end else if (cmd_pending_flash_dump && flash_rec_count > 0) begin
                fd_idx <= 9'd0; flash_rd_addr <= 12'd0; state <= FD_RD_REQ;
            end else if (cmd_pending_flash_dump && flash_rec_count == 0) begin
                // flag cleared in always block
            end else if (cmd_pending_dump && bram_entry_count > 0) begin
                dump_idx     <= bram_overflow ? bram_wr_ptr : 6'd0;
                bram_rd_addr <= bram_overflow ? bram_wr_ptr : 6'd0;
                dump_cnt     <= 7'd0; state <= BRAM_WAIT;
            end else if (cmd_pending_dump && bram_entry_count == 0) begin
                // flag cleared
            end else if (!sleep_latch && start_i) state <= READ_SENS_A;
        end

        // ------------------------------------------------------------------
        // Sensor A read
        // ------------------------------------------------------------------
        READ_SENS_A: begin
            proc_crc_seen <= 1'b0;
            sensor_timeout_latched <= 1'b0;
            cs1 <= 1'b0; master_tx_data <= {DATA_WIDTH{1'b0}};
            master_start <= 1'b1; wdog_en <= 1'b1; state <= WAIT_SENS_A;
        end
        WAIT_SENS_A: begin
            if (wdog_timeout) begin
                cs1 <= 1'b1; wdog_en <= 1'b0;
                sensor_timeout_latched <= 1'b1; state <= IDLE;
            end else if (master_done) begin
                cs1 <= 1'b1; wdog_en <= 1'b0;
                if (master_rx_data == {DATA_WIDTH{1'b0}}) begin
                    sensor_timeout_latched <= 1'b1; state <= IDLE;
                end else begin
                    sensor_timeout_latched <= 1'b0;
                    sens_a_frame <= master_rx_data; state <= CRC_CHK_A;
                end
            end
        end
        // Validate CRC of Sensor A frame
        CRC_CHK_A: begin
            sens_a_crc_ok <= crc_ok_a;
            state <= READ_SENS_B;
        end

        // ------------------------------------------------------------------
        // Sensor B read
        // ------------------------------------------------------------------
        READ_SENS_B: begin
            cs3 <= 1'b0; master_tx_data <= {DATA_WIDTH{1'b0}};
            master_start <= 1'b1; wdog_en <= 1'b1; state <= WAIT_SENS_B;
        end
        WAIT_SENS_B: begin
            if (wdog_timeout) begin
                cs3 <= 1'b1; wdog_en <= 1'b0;
                sensor_timeout_latched <= 1'b1; state <= IDLE;
            end else if (master_done) begin
                cs3 <= 1'b1; wdog_en <= 1'b0;
                if (master_rx_data == {DATA_WIDTH{1'b0}}) begin
                    sensor_timeout_latched <= 1'b1; state <= IDLE;
                end else begin
                    sensor_timeout_latched <= 1'b0;
                    sens_b_frame <= master_rx_data; state <= CRC_CHK_B;
                end
            end
        end
        CRC_CHK_B: begin
            sens_b_crc_ok <= crc_ok_b;
            state <= READ_SENS_C;
        end

        // ------------------------------------------------------------------
        // Sensor C read
        // ------------------------------------------------------------------
        READ_SENS_C: begin
            cs4 <= 1'b0; master_tx_data <= {DATA_WIDTH{1'b0}};
            master_start <= 1'b1; wdog_en <= 1'b1; state <= WAIT_SENS_C;
        end
        WAIT_SENS_C: begin
            if (wdog_timeout) begin
                cs4 <= 1'b1; wdog_en <= 1'b0;
                sensor_timeout_latched <= 1'b1; state <= IDLE;
            end else if (master_done) begin
                cs4 <= 1'b1; wdog_en <= 1'b0;
                if (master_rx_data == {DATA_WIDTH{1'b0}}) begin
                    sensor_timeout_latched <= 1'b1; state <= IDLE;
                end else begin
                    sensor_timeout_latched <= 1'b0;
                    sens_c_frame <= master_rx_data; state <= CRC_CHK_C;
                end
            end
        end
        CRC_CHK_C: begin
            sens_c_crc_ok <= crc_ok_c;
            state <= MEDIAN_VOTE;
        end

        // ------------------------------------------------------------------
        // Median vote: purely combinational, just a pipeline stage
        // ------------------------------------------------------------------
        MEDIAN_VOTE: state <= MAF_UPDATE;

        // ------------------------------------------------------------------
        // MAF update (replaces old MEDIAN_VOTE inline MAF)
        // ------------------------------------------------------------------
        MAF_UPDATE: state <= BUILD_PROC_FRAME;

        // ------------------------------------------------------------------
        // Latch the averaged frame with its fresh CRC
        // ------------------------------------------------------------------
        BUILD_PROC_FRAME: begin
            proc_frame_latched <= proc_frame_wire;
            state <= SEND_PROC;
        end

        // ------------------------------------------------------------------
        // Forward averaged frame to processing slave
        // ------------------------------------------------------------------
        SEND_PROC: begin
            cs2 <= 1'b0;
            master_tx_data <= proc_frame_latched;
            master_start   <= 1'b1; wdog_en <= 1'b1; state <= WAIT_PROC;
        end
        WAIT_PROC: begin
            if (proc_crc_err) proc_crc_seen <= 1'b1;
            if (wdog_timeout) begin
                cs2 <= 1'b1; wdog_en <= 1'b0;
                sensor_timeout_latched <= 1'b1; state <= IDLE;
            end else if (master_done) begin
                cs2 <= 1'b1; wdog_en <= 1'b0; state <= LATCH_DATA;
            end
        end

        // ------------------------------------------------------------------
        // Latch AQI / Category from processing slave output
        // ------------------------------------------------------------------
        LATCH_DATA: begin
            // AQI and category come EXCLUSIVELY from the processing slave.
            // They are captured in the first cycle only (see latch_first).
            if (latch_first) begin
                aqi_latched     <= proc_aqi;
                cat_latched     <= proc_cat;
                // crc_err aggregates: sensor-level faults + internal transfer fault
                crc_err_latched <= (~sens_a_crc_ok | ~sens_b_crc_ok |
                                    ~sens_c_crc_ok | proc_crc_seen | proc_crc_err);
                // keep the one-cycle processing-slave CRC pulse for a delayed packet
                proc_crc_seen   <= proc_crc_seen | proc_crc_err;
                done_o          <= 1'b1;
                latch_done_sent <= 1'b1;
            end
            if (!sleep_latch && push_idle && !fifo_full) begin
                push_buf[0] <= 8'hAA;
                push_buf[1] <= {7'b0, live_aqi[8]};
                push_buf[2] <= live_aqi[7:0];
                push_buf[3] <= {input_err_o,
                                (proc_crc_seen | proc_crc_err),
                                live_cat};
                push_len    <= 4'd4;
                push_go     <= 1'b1;
            end
            if (sleep_latch || (push_idle && !fifo_full)) begin
                state           <= CHECK_CMD;
                latch_done_sent <= 1'b0;
            end
        end

        // ------------------------------------------------------------------
        // Command processing (unchanged)
        // ------------------------------------------------------------------
        CHECK_CMD: begin
            if (cmd_clear_log) state <= IDLE;
            else if (cmd_pending_flash_dump && flash_rec_count > 0) begin
                fd_idx <= 9'd0; flash_rd_addr <= 12'd0; state <= FD_RD_REQ;
            end else if (cmd_pending_flash_dump && flash_rec_count == 0) begin
                state <= IDLE;
            end else if (cmd_pending_dump && bram_entry_count > 0) begin
                dump_idx     <= bram_overflow ? bram_wr_ptr : 6'd0;
                bram_rd_addr <= bram_overflow ? bram_wr_ptr : 6'd0;
                dump_cnt     <= 7'd0; state <= BRAM_WAIT;
            end else if (cmd_pending_dump && bram_entry_count == 0) begin
                state <= IDLE;
            end else state <= IDLE;
        end

        BRAM_WAIT: begin
            bram_rd_addr <= dump_idx; state <= BRAM_RD_WAIT;
        end
        BRAM_RD_WAIT: state <= BRAM_LOAD;
        BRAM_LOAD: begin
            dump_entry <= bram_rd_data;
            if (push_idle && !fifo_full) begin
                push_buf[0] <= 8'hBB;
                push_buf[1] <= {2'b0, dump_idx};
                push_buf[2] <= bram_rd_data[63:56];
                push_buf[3] <= bram_rd_data[55:48];
                push_buf[4] <= bram_rd_data[47:40];
                push_buf[5] <= bram_rd_data[39:32];
                push_buf[6] <= bram_rd_data[31:24];
                push_buf[7] <= bram_rd_data[23:16];
                push_buf[8] <= bram_rd_data[15:8];
                push_len    <= 4'd9;
                if (dump_cnt + 7'd1 < bram_entry_count) begin
                    push_go      <= 1'b1;
                    dump_cnt     <= dump_cnt + 7'd1;
                    dump_idx     <= dump_idx + 6'd1;
                    bram_rd_addr <= dump_idx + 6'd1;
                    state        <= BRAM_WAIT;
                end else begin
                    push_buf[9] <= 8'hDD; push_len <= 4'd10;
                    push_go <= 1'b1; state <= IDLE;
                end
            end
        end

        FD_RD_REQ: begin
            flash_rd_addr <= {fd_idx[8:0], 3'b000};
            flash_rd_req  <= 1'b1; state <= FD_RD_WAIT;
        end
        FD_RD_WAIT: begin
            if (flash_done) begin fd_entry <= flash_rd_data; state <= FD_LOAD; end
        end
        FD_LOAD: begin
            if (push_idle && !fifo_full) begin
                push_buf[0] <= 8'hBF;
                push_buf[1] <= fd_idx[7:0];
                push_buf[2] <= flash_rd_data[63:56];
                push_buf[3] <= flash_rd_data[55:48];
                push_buf[4] <= flash_rd_data[47:40];
                push_buf[5] <= flash_rd_data[39:32];
                push_buf[6] <= flash_rd_data[31:24];
                push_buf[7] <= flash_rd_data[23:16];
                push_buf[8] <= flash_rd_data[15:8];
                push_len    <= 4'd9;
                if (fd_idx + 9'd1 < flash_rec_count) begin
                    push_go <= 1'b1; fd_idx <= fd_idx + 9'd1; state <= FD_RD_REQ;
                end else begin
                    push_buf[9] <= 8'hDD; push_len <= 4'd10;
                    push_go <= 1'b1; state <= IDLE;
                end
            end
        end

        default: state <= IDLE;
        endcase
    end
end
endmodule
