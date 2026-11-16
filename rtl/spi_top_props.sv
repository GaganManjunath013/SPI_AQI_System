/******************************************************************************
 Module Name : spi_top_props
 Purpose     : Concurrent Protocol Monitors & Formal Properties for spi_top
 Description : Combines Yosys-compatible $past() monitors with comprehensive
               SVA-style range/consistency checks. Suitable for both formal
               verification (sby/JasperGold) and simulation-based assertion
               checking.

 Properties Verified:
   1.  CS mutual exclusion (at most one active)
   2.  done_o is a single-cycle pulse
   3.  alarm_rising is a single-cycle pulse
   4.  Watchdog timeout releases all chip selects
   5.  Flash write integrity (no write while busy)
   6.  Sleep mode blocks alarm triggers
   7.  AQI output range [0, 500]
   8.  Category range [0, 5]
   9.  Category-AQI consistency (6 sub-properties)
   10. Alarm on_ctr bounded by N_ON
   11. Alarm off_ctr bounded by N_OFF
   12. aqi_out_o upper bits always zero
   13. Median voter outputs bounded by 10-bit range
   14. AQI/category latched from the processing slave on entry to LATCH_DATA
******************************************************************************/

module spi_top_props (
    input wire        clk_i,
    input wire        rst_i,
    input wire        done_o,
    input wire        alarm_rising,
    input wire        cs1,
    input wire        cs2,
    input wire        cs3,
    input wire        cs4,
    input wire        flash_busy,
    input wire        flash_write_en,
    input wire        wdog_timeout,
    input wire        sleep_mode,
    input wire [8:0]  aqi_latched,
    input wire [2:0]  cat_latched,
    input wire [15:0] aqi_out_o,
    input wire [3:0]  on_ctr,
    input wire [3:0]  off_ctr,
    input wire [9:0]  med_pm25,
    input wire [9:0]  med_pm10,
    input wire [9:0]  med_voc,
    input wire [9:0]  med_co,
    input wire        bram_ecc_ded,
    
    // --- ADDED PORTS FOR PROPERTY 14 ---
    input wire [6:0]  state,
    input wire [8:0]  proc_aqi,
    input wire [2:0]  proc_cat
);

    // ========================================================================
    // CONCURRENT PROTOCOL MONITORS (Yosys $past() compatible)
    // ========================================================================
    always @(posedge clk_i) begin
        if (!rst_i) begin

            // ----------------------------------------------------------------
            // 1. CS MUTEX — at most one chip select active at a time
            // ----------------------------------------------------------------
            if (!cs1) assert(cs2 && cs3 && cs4);
            if (!cs2) assert(cs1 && cs3 && cs4);
            if (!cs3) assert(cs1 && cs2 && cs4);
            if (!cs4) assert(cs1 && cs2 && cs3);

            // ----------------------------------------------------------------
            // 2. done_o is a single-cycle pulse
            // ----------------------------------------------------------------
            if ($past(done_o))
                assert(!done_o);

            // ----------------------------------------------------------------
            // 3. alarm_rising is a single-cycle pulse
            // ----------------------------------------------------------------
            if ($past(alarm_rising))
                assert(!alarm_rising);

            // ----------------------------------------------------------------
            // 4. Watchdog timeout releases all chip selects
            // ----------------------------------------------------------------
            if ($past(wdog_timeout))
                assert(cs1 && cs2 && cs3 && cs4);

            // ----------------------------------------------------------------
            // 5. Flash write integrity — no new write while busy
            // ----------------------------------------------------------------
            if (flash_busy)
                assert(!flash_write_en);

            // ----------------------------------------------------------------
            // 6. Sleep mode blocks alarm triggers
            // ----------------------------------------------------------------
            if (sleep_mode)
                assert(!alarm_rising);

            // ----------------------------------------------------------------
            // 7. AQI output range [0, 500]
            // ----------------------------------------------------------------
            assert(aqi_latched <= 9'd500);

            // ----------------------------------------------------------------
            // 8. Category range [0, 5]
            // ----------------------------------------------------------------
            assert(cat_latched <= 3'd5);

            // ----------------------------------------------------------------
            // 9. Category-AQI consistency
            // ----------------------------------------------------------------
            if (aqi_latched <= 9'd50)
                assert(cat_latched == 3'd0);
            if (aqi_latched > 9'd50  && aqi_latched <= 9'd100)
                assert(cat_latched == 3'd1);
            if (aqi_latched > 9'd100 && aqi_latched <= 9'd200)
                assert(cat_latched == 3'd2);
            if (aqi_latched > 9'd200 && aqi_latched <= 9'd300)
                assert(cat_latched == 3'd3);
            if (aqi_latched > 9'd300 && aqi_latched <= 9'd400)
                assert(cat_latched == 3'd4);
            if (aqi_latched > 9'd400)
                assert(cat_latched == 3'd5);

            // ----------------------------------------------------------------
            // 10. Alarm on_ctr bounded by N_ON (4)
            // ----------------------------------------------------------------
            assert(on_ctr <= 4'd4);

            // ----------------------------------------------------------------
            // 11. Alarm off_ctr bounded by N_OFF (8)
            // ----------------------------------------------------------------
            assert(off_ctr <= 4'd8);

            // ----------------------------------------------------------------
            // 12. aqi_out_o upper bits always zero
            // ----------------------------------------------------------------
            assert(aqi_out_o[15:9] == 7'd0);

            // ----------------------------------------------------------------
            // 13. Median voter outputs bounded by 10-bit range
            // ----------------------------------------------------------------
            assert(med_pm25 <= 10'd1023);
            assert(med_pm10 <= 10'd1023);
            assert(med_voc  <= 10'd1023);
            assert(med_co   <= 10'd1023);

            // ----------------------------------------------------------------
            // 14. Processing-Centric Pipeline Transfer Synchronization
            // ----------------------------------------------------------------
            // spi_top captures the slave outputs in the first LATCH_DATA
            // cycle only, so the check is made one clock after entry.
            if ($past(state) == 7'd10 && $past(state, 2) != 7'd10) begin
                assert(aqi_latched == $past(proc_aqi));
                assert(cat_latched == $past(proc_cat));
            end
        end
    end

endmodule

// ============================================================================
// TOP-LEVEL BINDING
// ============================================================================
bind spi_top spi_top_props u_formal_props (
    .clk_i          (clk_i),
    .rst_i          (rst_i),
    .done_o         (done_o),
    .alarm_rising   (alarm_rising),
    .cs1            (cs1),
    .cs2            (cs2),
    .cs3            (cs3),
    .cs4            (cs4),
    .flash_busy     (flash_busy),         
    .flash_write_en (flash_wr_pending),
    .wdog_timeout   (sensor_timeout_o),
    .sleep_mode     (sleep_latch),
    .aqi_latched    (aqi_latched),
    .cat_latched    (cat_latched),
    .aqi_out_o      (aqi_out_o),
    .on_ctr         (on_ctr),
    .off_ctr        (off_ctr),
    .med_pm25       (med_pm25),
    .med_pm10       (med_pm10),
    .med_voc        (med_voc),
    .med_co         (med_co),
    .bram_ecc_ded   (bram_ecc_ded),
    
    // --- ADDED MAPPINGS FOR PROPERTY 14 ---
    .state          (state),
    .proc_aqi       (proc_aqi),
    .proc_cat       (proc_cat)
);
