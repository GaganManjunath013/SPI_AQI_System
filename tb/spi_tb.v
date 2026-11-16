
/******************************************************************************
 Module Name : spi_tb  (Processing-Centric Architecture)

 Purpose     : System-level verification harness for the refactored
               Processing-Centric SPI Air-Quality Monitoring System.

 Key Features:
   - Monitors the internal data path:
       Sensor A/B/C -> spi_top (CRC validate, TMR, MAF) -> Processing Slave
   - All AQI checks target proc.aqi_value / proc.category from the slave.
   - Sensor-level CRC error injection tested separately from internal
     (averaged-frame) CRC injection.
   - Professional console output with ASCII section banners, live dashboard,
     and consolidated per-test summary.
   - Performance profiling: cycle latency, proc slave latency, UART TX
     latency, BRAM dump latency, flash write latency.

 Logging format:
   [PASS] / [FAIL] tags  +  timestamped result line per assertion.

 Dashboard printed at every LATCH_DATA milestone showing:
   - Sensor A/B/C CRC validity
   - Voted medians (pm25/pm10/voc/co)
   - MAF averages (avg_pm25/avg_pm10)
   - Internal transfer frame content
   - Processing slave AQI + category

 Summary:
   Section A  -- Performance (sim time, cycles)
   Section B  -- Functional Accuracy (pass/fail counts)
   Section C  -- Component Health (math, SPI/CRC, sensor reliability)
   Section D  -- Per-test breakdown table
******************************************************************************/

module spi_tb;

// -----------------------------------------------------------------------------
//  DUT STIMULUS SIGNALS
// -----------------------------------------------------------------------------
reg        clk, reset, start, sleep;
reg signed [12:0] raw_pm25,   raw_pm10,   raw_voc,   raw_co;
reg signed [12:0] raw_pm25_b, raw_pm10_b, raw_voc_b, raw_co_b;
reg signed [12:0] raw_pm25_c, raw_pm10_c, raw_voc_c, raw_co_c;
reg        sclk_s, cs_s, mosi_s;
reg        bod_i_drive;
reg        uart_rx_drive;

// -----------------------------------------------------------------------------
//  DUT OUTPUT WIRES
// -----------------------------------------------------------------------------
wire       done, alarm_pin, uart_tx, crc_err, wakeup, sensor_timeout, trend_rising;
wire [15:0] final_aqi;
wire [2:0]  final_category, sensor_fault;
wire [3:0]  input_err;
wire        flash_sclk, flash_cs, flash_mosi, flash_write_err, bod_flag, miso_s_out;
wire        miso_b_wire, miso_c_wire, flash_miso_wire;

// -----------------------------------------------------------------------------
//  FAULT INJECTION CONTROLS
// -----------------------------------------------------------------------------
reg        enable_print = 1'b1;

reg        corrupt_enable, mock_sensor_enable;
reg [49:0] corrupt_mask, corrupt_val;
reg [5:0]  corrupt_bit_cnt;

// Sensor A MISO fault injection
always @(posedge uut.sclk or posedge reset) begin
    if (reset)               corrupt_bit_cnt <= 6'd0;
    else if (uut.cs1==1'b1)  corrupt_bit_cnt <= 6'd0;
    else                     corrupt_bit_cnt <= corrupt_bit_cnt + 1;
end
wire corrupt_bit            = corrupt_enable ? 1'b1 : 1'b0;
wire miso_sensor_a_internal = uut.miso_sensor_a;
wire miso_sensor_a_tb       = mock_sensor_enable ? 1'b0 :
                              (miso_sensor_a_internal ^ corrupt_bit);

// Internal transfer frame injection (averaged frame to proc slave)
reg internal_corrupt_enable;

// Helper reg for force workaround (iverilog does not support expressions on RHS of force)
reg [49:0] force_tx_data_val;

// -----------------------------------------------------------------------------
//  DUT INSTANTIATION
// -----------------------------------------------------------------------------
spi_top #(.DATA_WIDTH(50)) uut (
    .clk_i(clk), .rst_i(reset), .start_i(start),
    .raw_pm25_i(raw_pm25),   .raw_pm10_i(raw_pm10),   .raw_voc_i(raw_voc),   .raw_co_i(raw_co),
    .raw_pm25_b_i(raw_pm25_b),.raw_pm10_b_i(raw_pm10_b),.raw_voc_b_i(raw_voc_b),.raw_co_b_i(raw_co_b),
    .raw_pm25_c_i(raw_pm25_c),.raw_pm10_c_i(raw_pm10_c),.raw_voc_c_i(raw_voc_c),.raw_co_c_i(raw_co_c),
    .miso_a_i(miso_sensor_a_tb), .miso_b_i(miso_b_wire), .miso_c_i(miso_c_wire),
    .sleep_i(sleep), .wakeup_o(wakeup),
    .uart_tx_o(uart_tx), .uart_rx_i(uart_rx_drive),
    .done_o(done), .aqi_out_o(final_aqi), .cat_out_o(final_category),
    .alarm_o(alarm_pin), .crc_err_o(crc_err), .input_err_o(input_err),
    .sensor_timeout_o(sensor_timeout), .sensor_fault_o(sensor_fault),
    .cs3(), .cs4(),
    .sclk_s_i(sclk_s), .cs_s_i(cs_s), .mosi_s_i(mosi_s), .miso_s_o(miso_s_out),
    .trend_rising_o(trend_rising),
    .flash_sclk_o(flash_sclk), .flash_cs_o(flash_cs),
    .flash_mosi_o(flash_mosi), .flash_miso_i(flash_miso_wire),
    .flash_write_err_o(flash_write_err), .bod_i(bod_i_drive), .bod_flag_o(bod_flag)
);

assign miso_b_wire = uut.sensor_b.miso;
assign miso_c_wire = uut.sensor_c.miso;

spi_flash_model #(.DEPTH(4096)) flash_model (
    .sclk(flash_sclk), .cs(flash_cs), .mosi(flash_mosi), .miso(flash_miso_wire)
);

// -----------------------------------------------------------------------------
//  CLOCK
// -----------------------------------------------------------------------------
initial clk = 0;
always  #5 clk = ~clk;
integer global_clk_ctr;
// -----------------------------------------------------------------------------
//  UART BYTE MONITORS
// -----------------------------------------------------------------------------
reg global_saw_cc, global_saw_ee, global_saw_bb, global_saw_bf, global_saw_dd;

always @(posedge clk) begin
    if (uut.uart_start) begin
        case (uut.uart_tx_data)
            8'hCC: global_saw_cc <= 1'b1;
            8'hEE: global_saw_ee <= 1'b1;
            8'hBB: global_saw_bb <= 1'b1;
            8'hBF: global_saw_bf <= 1'b1;
            8'hDD: global_saw_dd <= 1'b1;
        endcase
    end
end

// -----------------------------------------------------------------------------
//  COMPONENT HEALTH COUNTERS
// -----------------------------------------------------------------------------
integer math_err_count;
integer spi_proto_err_count;
integer sensor_rel_err_count;

// -----------------------------------------------------------------------------
//  PERFORMANCE PROFILING REGISTERS
// -----------------------------------------------------------------------------
// A. Complete System Measurement Cycle Latency (start_i -> done_o)
integer  prof_cycle_clk_start;
real     prof_cycle_time_start;
integer  prof_cycle_clk_total;
real     prof_cycle_time_total;
integer  prof_cycle_samples;

// B. Processing Slave Calculation Latency (cs2 drops -> cs2 rises)
integer  prof_proc_clk_start;
real     prof_proc_time_start;
integer  prof_proc_clk_total;
real     prof_proc_time_total;
integer  prof_proc_samples;

// C. Live Data UART Transmission Latency (LATCH_DATA -> fifo_empty & !uart_busy)
integer  prof_uart_clk_start;
real     prof_uart_time_start;
integer  prof_uart_clk_total;
real     prof_uart_time_total;
integer  prof_uart_samples;

// D. BRAM Dump UART Transmission Latency (0x02 cmd decoded -> 0xDD fully sent)
integer  prof_bram_dump_clk_start;
real     prof_bram_dump_time_start;
integer  prof_bram_dump_clk_total;
real     prof_bram_dump_time_total;
integer  prof_bram_dump_samples;

// E. SPI Flash Write Latency (alarm_rising -> flash_done)
integer  prof_flash_wr_clk_start;
real     prof_flash_wr_time_start;
integer  prof_flash_wr_clk_total;
real     prof_flash_wr_time_total;
integer  prof_flash_wr_samples;

// -- Profiling edge-detect helpers --
reg prof_start_prev, prof_done_prev;
reg prof_alarm_rising_prev, prof_flash_done_prev;
reg prof_uart_measuring, prof_bram_dump_measuring, prof_flash_wr_measuring;
reg prof_bram_dump_draining;  // waiting for 0xDD to fully shift out

// -- A. System Cycle Latency: start_i rising -> done_o rising --
always @(posedge clk) begin
    if (reset) begin
        prof_start_prev <= 1'b0; prof_done_prev <= 1'b0;
        prof_cycle_clk_total <= 0; prof_cycle_time_total <= 0.0;
        prof_cycle_samples <= 0;
    end else begin
        prof_start_prev <= start;
        prof_done_prev  <= done;
        if (start && !prof_start_prev) begin
            prof_cycle_clk_start  <= global_clk_ctr;
            prof_cycle_time_start <= $realtime;
        end
        if (done && !prof_done_prev) begin
            prof_cycle_clk_total  <= global_clk_ctr - prof_cycle_clk_start;
            prof_cycle_time_total <= $realtime - prof_cycle_time_start;
            prof_cycle_samples    <= prof_cycle_samples + 1;
        end
    end
end

// -- B. Processing Slave Latency: SEND_PROC entry -> LATCH_DATA entry --
reg [6:0] prof_state_prev;
always @(posedge clk) begin
    if (reset) begin
        prof_state_prev <= 7'd0;
        prof_proc_clk_total <= 0; prof_proc_time_total <= 0.0;
        prof_proc_samples <= 0;
    end else begin
        prof_state_prev <= uut.state;
        // Start when FSM enters SEND_PROC (state 8)
        if (uut.state == 7'd8 && prof_state_prev != 7'd8) begin
            prof_proc_clk_start  <= global_clk_ctr;
            prof_proc_time_start <= $realtime;
        end
        // End when FSM enters LATCH_DATA (state 10)
        if (uut.state == 7'd10 && prof_state_prev != 7'd10) begin
            prof_proc_clk_total  <= global_clk_ctr - prof_proc_clk_start;
            prof_proc_time_total <= $realtime - prof_proc_time_start;
            prof_proc_samples    <= prof_proc_samples + 1;
        end
    end
end

// -- C. Live UART TX Latency: LATCH_DATA entry -> fifo_empty & !uart_busy & push_idle --
reg prof_uart_push_seen;  // guard: wait until push engine has started at least once
always @(posedge clk) begin
    if (reset) begin
        prof_uart_measuring <= 1'b0;
        prof_uart_push_seen <= 1'b0;
        prof_uart_clk_total <= 0; prof_uart_time_total <= 0.0;
        prof_uart_samples <= 0;
    end else begin
        // Start when FSM enters LATCH_DATA
        if (uut.state == 7'd10 && !prof_uart_measuring) begin
            prof_uart_clk_start  <= global_clk_ctr;
            prof_uart_time_start <= $realtime;
            prof_uart_measuring  <= 1'b1;
            prof_uart_push_seen  <= 1'b0;
        end
        // Detect that the push engine has been activated (push_go or FIFO not empty)
        if (prof_uart_measuring && (!uut.fifo_empty || !uut.push_idle)) begin
            prof_uart_push_seen <= 1'b1;
        end
        // End when push has been seen AND FIFO is drained, UART is idle, push engine is idle
        if (prof_uart_measuring && prof_uart_push_seen &&
            uut.fifo_empty && !uut.uart_busy && uut.push_idle) begin
            prof_uart_clk_total  <= global_clk_ctr - prof_uart_clk_start;
            prof_uart_time_total <= $realtime - prof_uart_time_start;
            prof_uart_samples    <= prof_uart_samples + 1;
            prof_uart_measuring  <= 1'b0;
            prof_uart_push_seen  <= 1'b0;
        end
    end
end

// -- D. BRAM Dump Latency: cmd_dump_log decoded -> 0xDD fully shifted out --
always @(posedge clk) begin
    if (reset) begin
        prof_bram_dump_measuring <= 1'b0;
        prof_bram_dump_draining  <= 1'b0;
        prof_bram_dump_clk_total <= 0; prof_bram_dump_time_total <= 0.0;
        prof_bram_dump_samples <= 0;
    end else begin
        // Start when 0x02 command is decoded
        if (uut.cmd_dump_log && !prof_bram_dump_measuring) begin
            prof_bram_dump_clk_start  <= global_clk_ctr;
            prof_bram_dump_time_start <= $realtime;
            prof_bram_dump_measuring  <= 1'b1;
            prof_bram_dump_draining   <= 1'b0;
        end
        // Detect when 0xDD is pushed into the FIFO
        if (prof_bram_dump_measuring && !prof_bram_dump_draining &&
            uut.uart_start && uut.uart_tx_data == 8'hDD) begin
            prof_bram_dump_draining <= 1'b1;
        end
        // End when 0xDD has been fully shifted out (FIFO empty, UART idle, push idle)
        if (prof_bram_dump_draining && uut.fifo_empty && !uut.uart_busy && uut.push_idle) begin
            prof_bram_dump_clk_total  <= global_clk_ctr - prof_bram_dump_clk_start;
            prof_bram_dump_time_total <= $realtime - prof_bram_dump_time_start;
            prof_bram_dump_samples    <= prof_bram_dump_samples + 1;
            prof_bram_dump_measuring  <= 1'b0;
            prof_bram_dump_draining   <= 1'b0;
        end
    end
end

// -- E. Flash Write Latency: alarm_rising -> flash_done (or busy_o drops) --
reg prof_flash_busy_prev;
always @(posedge clk) begin
    if (reset) begin
        prof_alarm_rising_prev <= 1'b0; prof_flash_done_prev <= 1'b0;
        prof_flash_busy_prev <= 1'b0;
        prof_flash_wr_measuring <= 1'b0;
        prof_flash_wr_clk_total <= 0; prof_flash_wr_time_total <= 0.0;
        prof_flash_wr_samples <= 0;
    end else begin
        prof_alarm_rising_prev <= uut.alarm_rising;
        prof_flash_done_prev   <= uut.flash_ctrl.done_o;
        prof_flash_busy_prev   <= uut.flash_ctrl.busy_o;
        // Start on alarm_rising posedge
        if (uut.alarm_rising && !prof_alarm_rising_prev && !prof_flash_wr_measuring) begin
            prof_flash_wr_clk_start  <= global_clk_ctr;
            prof_flash_wr_time_start <= $realtime;
            prof_flash_wr_measuring  <= 1'b1;
        end
        // End on flash_done posedge or busy_o falling edge
        if (prof_flash_wr_measuring &&
            ((uut.flash_ctrl.done_o && !prof_flash_done_prev) ||
             (!uut.flash_ctrl.busy_o && prof_flash_busy_prev))) begin
            prof_flash_wr_clk_total  <= global_clk_ctr - prof_flash_wr_clk_start;
            prof_flash_wr_time_total <= $realtime - prof_flash_wr_time_start;
            prof_flash_wr_samples    <= prof_flash_wr_samples + 1;
            prof_flash_wr_measuring  <= 1'b0;
        end
    end
end

// -----------------------------------------------------------------------------
//  VCD SIGNAL ALIASES (for GTKWave-friendly names)
// -----------------------------------------------------------------------------
wire [6:0]  fsm_state        = uut.state;
wire signed [12:0] raw_pm25_a = raw_pm25;
wire signed [12:0] raw_pm10_a = raw_pm10;
wire signed [12:0] raw_voc_a  = raw_voc;
wire signed [12:0] raw_co_a   = raw_co;
wire [9:0]  med_pm25         = uut.med_pm25;
wire [9:0]  med_pm10         = uut.med_pm10;
wire [9:0]  med_voc          = uut.med_voc;
wire [9:0]  med_co           = uut.med_co;
wire [9:0]  avg_pm25         = uut.avg_pm25;
wire        maf_valid        = uut.maf_valid;
wire [8:0]  aqi_latched      = uut.aqi_latched;
wire [2:0]  cat_latched      = uut.cat_latched;
wire        crc_err_lat      = uut.crc_err_latched;
wire        alarm_state      = uut.alarm_state;
wire        alarm_rising     = uut.alarm_rising;
wire        watchdog_timeout = uut.wdog_timeout;
wire        cs_sensor_a      = uut.cs1;
wire        cs_processing    = uut.cs2;
wire        cs_sensor_b      = uut.cs3;
wire        cs_sensor_c      = uut.cs4;
wire        spi_sclk         = uut.sclk;
wire        spi_mosi         = uut.mosi;
wire        fifo_empty       = uut.fifo_empty;
wire        fifo_full        = uut.fifo_full;
wire [6:0]  fifo_wr_ptr      = uut.fifo_wr;
wire [6:0]  fifo_rd_ptr      = uut.fifo_rd;
wire        push_idle        = uut.push_idle;

// -----------------------------------------------------------------------------
//  GLOBAL TEST ACCOUNTING
// -----------------------------------------------------------------------------
integer pass_count, fail_count, test_num;
integer test_pass  [1:80];
integer test_fail  [1:80];
real    test_start [1:80];
real    test_end   [1:80];
integer clk_start  [1:80];
integer clk_end    [1:80];

reg [511:0] test_name [1:80];

always @(posedge clk) global_clk_ctr = global_clk_ctr + 1;

task begin_test;
    input [511:0] name;
    begin
        test_name[test_num]  = name;
        test_start[test_num] = $realtime;
        clk_start[test_num]  = global_clk_ctr;
        test_pass[test_num]  = 0;
        test_fail[test_num]  = 0;
        $display("  +- T%02d  %0s", test_num, name);
    end
endtask

task end_test;
    begin
        test_end[test_num] = $realtime;
        clk_end[test_num]  = global_clk_ctr;
        pass_count = pass_count + test_pass[test_num];
        fail_count = fail_count + test_fail[test_num];
        if (test_fail[test_num] == 0)
            $display("  +- T%02d  [PASSED]  Assertions: %0d passed, %0d failed  Cycles:%0d",
                     test_num, test_pass[test_num], test_fail[test_num],
                     clk_end[test_num] - clk_start[test_num]);
        else
            $display("  +- T%02d  [FAILED]  Assertions: %0d passed, %0d failed  Cycles:%0d",
                     test_num, test_pass[test_num], test_fail[test_num],
                     clk_end[test_num] - clk_start[test_num]);
        $display("");
        test_num = test_num + 1;
    end
endtask

// -----------------------------------------------------------------------------
//  PRIMARY LOGGING TASK
// -----------------------------------------------------------------------------
task log_result;
    input [511:0] tname;
    input [63:0]  inp_val;
    input [63:0]  out_val;
    input [63:0]  exp_val;
    input         pass;
    begin
        if (pass) begin
            $display("  |  [PASS] %-48s | got=%04h  exp=%04h | @%0t",
                     tname, out_val[15:0], exp_val[15:0], $time);
            test_pass[test_num] = test_pass[test_num] + 1;
        end else begin
            $display("  |  [FAIL] %-48s | got=%04h  exp=%04h | @%0t  <<<",
                     tname, out_val[15:0], exp_val[15:0], $time);
            test_fail[test_num] = test_fail[test_num] + 1;
        end
    end
endtask

task chk;
    input        condition;
    input [511:0] msg;
    begin
        log_result(msg, 64'd0, 64'd0, 64'd0, condition);
        if (!condition)
            $display("  |  [ASSERT FAIL] %0s", msg);
    end
endtask

// -----------------------------------------------------------------------------
//  LIVE PIPELINE DASHBOARD
//  Printed each time the FSM reaches LATCH_DATA (state==10)
// -----------------------------------------------------------------------------
reg dashboard_sent;

always @(posedge clk) begin
    if (!reset && (uut.state == 7'd10) && !dashboard_sent && enable_print) begin
        dashboard_sent <= 1'b1;
        $display("");
        $display("  +==============================================================+");
        $display("  |  PIPELINE DASHBOARD  [Test T%02d]", test_num);
        $display("  +==============================================================+");
        $display("  |  Sensor CRC Status : A=%0s  B=%0s  C=%0s",
                 uut.sens_a_crc_ok ? "OK  " : "FAIL",
                 uut.sens_b_crc_ok ? "OK  " : "FAIL",
                 uut.sens_c_crc_ok ? "OK  " : "FAIL");
        $display("  |  Voted  Medians    : PM2.5=%0d  PM10=%0d  VOC=%0d  CO=%0d",
                 uut.med_pm25, uut.med_pm10, uut.med_voc, uut.med_co);
        $display("  |  MAF   Averages    : avg_PM25=%0d  avg_PM10=%0d  avg_VOC=%0d  avg_CO=%0d",
                 uut.avg_pm25, uut.avg_pm10, uut.avg_voc, uut.avg_co);
        $display("  |  Internal Frame    : [%012h] CRC=%02h",
                 uut.proc_frame_latched[49:8], uut.proc_frame_latched[7:0]);
        $display("  |  Processing Slave  : AQI=%0d  Category=%0d  CRC_err=%0b",
                 uut.proc.aqi_value, uut.proc.category, uut.proc.crc_err_flag);
        $display("  |  System Outputs    : aqi_latched=%0d  cat=%0d  alarm=%0b  trend=%0b",
                 uut.aqi_latched, uut.cat_latched, uut.alarm_state, uut.trend_rising_o);
        $display("  +==============================================================+");
        $display("");
    end
    if (uut.state != 7'd10) dashboard_sent <= 1'b0;
end

// -----------------------------------------------------------------------------
//  HELPERS
// -----------------------------------------------------------------------------
localparam UART_BIT_CLKS = 109;
localparam UART_BIT_NS   = UART_BIT_CLKS * 10;

function signed [12:0] phys_to_raw;
    input real val, full_scale;
    begin
        if (val <= 0.0) phys_to_raw = 13'sd0;
        else            phys_to_raw = $rtoi((val / full_scale) * 4095.0 + 0.5);
    end
endfunction

task set_all_sensors;
    input signed [12:0] p25, p10, voc, co;
    begin
        raw_pm25 = p25; raw_pm10 = p10; raw_voc = voc; raw_co = co;
        raw_pm25_b = p25; raw_pm10_b = p10; raw_voc_b = voc; raw_co_b = co;
        raw_pm25_c = p25; raw_pm10_c = p10; raw_voc_c = voc; raw_co_c = co;
    end
endtask

task wait_idle;
    begin
        repeat(5) @(posedge clk);
        wait (uut.state == 0 && !uut.err_pending && !uut.ack_pending &&
              !uut.cmd_pending_dump && !uut.cmd_pending_flash_dump &&
              !uut.uart_busy && uut.fifo_empty);
    end
endtask

task run_cycle;
    begin
        wait_idle;
        @(posedge clk); #1;
        start = 1'b1;
        @(posedge clk); #1;
        start = 1'b0;
        @(posedge done);
    end
endtask

task run_silent_cycles;
    input integer num_cycles;
    integer i;
    begin
        enable_print = 1'b0;
        for (i = 0; i < num_cycles; i = i + 1) begin
            run_cycle;
        end
        enable_print = 1'b1;
    end
endtask

task uart_send;
    input [7:0] data;
    integer i;
    begin
        uart_rx_drive = 1'b0; #(UART_BIT_NS);
        for (i = 0; i < 8; i = i + 1) begin
            uart_rx_drive = data[i]; #(UART_BIT_NS);
        end
        uart_rx_drive = 1'b1; #(UART_BIT_NS);
    end
endtask

task spi_slave_read;
    input  [7:0]  addr;
    output [15:0] rdata;
    integer i; reg [15:0] sh;
    begin
        cs_s = 0; sh = 0; #100;
        for (i=7; i>=0; i=i-1) begin mosi_s=addr[i]; #50; sclk_s=1; #50; sclk_s=0; #50; end
        mosi_s = 0; #50; sh[15] = miso_s_out;
        for (i=14; i>=0; i=i-1) begin #50; sclk_s=1; #50; sh[i]=miso_s_out; sclk_s=0; #50; end
        #100; cs_s = 1; rdata = sh; #100;
    end
endtask

task spi_slave_write;
    input [7:0] addr, data;
    integer i;
    begin
        cs_s = 0; #100;
        for (i=7; i>=0; i=i-1) begin mosi_s=addr[i]; #50; sclk_s=1; #50; sclk_s=0; #50; end
        for (i=7; i>=0; i=i-1) begin mosi_s=data[i]; #50; sclk_s=1; #50; sclk_s=0; #50; end
        mosi_s=0; #100; cs_s=1; repeat(10) @(posedge clk);
    end
endtask

// -----------------------------------------------------------------------------
//  VCD DUMP -- Golden GTKWave Configuration
// -----------------------------------------------------------------------------
initial begin
    $dumpfile("dump.vcd");

    // -- 1. Clock & Control --
    $dumpvars(1, clk);
    $dumpvars(1, reset);
    $dumpvars(1, start);
    $dumpvars(1, done);
    $dumpvars(1, sleep);
    $dumpvars(1, bod_i_drive);
    $dumpvars(1, corrupt_enable);
    $dumpvars(1, mock_sensor_enable);

    // -- 2. FSM State --
    $dumpvars(1, fsm_state);
    $dumpvars(1, uut.master_start);
    $dumpvars(1, uut.master_done);
    $dumpvars(1, uut.master_busy);
    $dumpvars(1, uut.wdog_en);

    // -- 3. Sensor A Raw ADC Inputs --
    $dumpvars(1, raw_pm25_a);
    $dumpvars(1, raw_pm10_a);
    $dumpvars(1, raw_voc_a);
    $dumpvars(1, raw_co_a);

    // -- 4. Sensor B Raw ADC Inputs --
    $dumpvars(1, raw_pm25_b);
    $dumpvars(1, raw_pm10_b);
    $dumpvars(1, raw_voc_b);
    $dumpvars(1, raw_co_b);

    // -- 5. Sensor C Raw ADC Inputs --
    $dumpvars(1, raw_pm25_c);
    $dumpvars(1, raw_pm10_c);
    $dumpvars(1, raw_voc_c);
    $dumpvars(1, raw_co_c);

    // -- 6. Sensor A Converted Values + Error Flags --
    $dumpvars(1, uut.sensor_a.pm25_val);
    $dumpvars(1, uut.sensor_a.pm10_val);
    $dumpvars(1, uut.sensor_a.voc_val);
    $dumpvars(1, uut.sensor_a.co_val);
    $dumpvars(1, uut.sensor_a.input_err_o);

    // -- 7. Sensor B Converted Values + Error Flags --
    $dumpvars(1, uut.sensor_b.pm25_val);
    $dumpvars(1, uut.sensor_b.pm10_val);
    $dumpvars(1, uut.sensor_b.voc_val);
    $dumpvars(1, uut.sensor_b.co_val);
    $dumpvars(1, uut.sensor_b.input_err_o);

    // -- 8. Sensor C Converted Values + Error Flags --
    $dumpvars(1, uut.sensor_c.pm25_val);
    $dumpvars(1, uut.sensor_c.pm10_val);
    $dumpvars(1, uut.sensor_c.voc_val);
    $dumpvars(1, uut.sensor_c.co_val);
    $dumpvars(1, uut.sensor_c.input_err_o);

    // -- 9. TMR Median Voter + Fault Detection --
    $dumpvars(1, uut.sens_a_frame);
    $dumpvars(1, uut.sens_b_frame);
    $dumpvars(1, uut.sens_c_frame);
    $dumpvars(1, med_pm25);
    $dumpvars(1, med_pm10);
    $dumpvars(1, med_voc);
    $dumpvars(1, med_co);
    $dumpvars(1, uut.sens_a_dev);
    $dumpvars(1, uut.sens_b_dev);
    $dumpvars(1, uut.sens_c_dev);
    $dumpvars(1, sensor_fault);

    // -- 10. MAF Accumulator + Average --
    $dumpvars(1, uut.acc_pm25);
    $dumpvars(1, avg_pm25);
    $dumpvars(1, uut.maf_ptr);
    $dumpvars(1, maf_valid);

    // -- 11. AQI Pipeline + Category --
    $dumpvars(1, aqi_latched);
    $dumpvars(1, cat_latched);
    $dumpvars(1, crc_err_lat);
    $dumpvars(1, uut.proc.aqi_value);
    $dumpvars(1, uut.proc.category);
    $dumpvars(1, uut.proc.crc_err_flag);

    // -- 12. Top-Level Outputs --
    $dumpvars(1, final_aqi);
    $dumpvars(1, final_category);
    $dumpvars(1, alarm_pin);
    $dumpvars(1, crc_err);
    $dumpvars(1, input_err);
    $dumpvars(1, sensor_timeout);
    $dumpvars(1, trend_rising);
    $dumpvars(1, wakeup);
    $dumpvars(1, bod_flag);
    $dumpvars(1, flash_write_err);

    // -- 13. Alarm + Hysteresis --
    $dumpvars(1, alarm_state);
    $dumpvars(1, alarm_rising);
    $dumpvars(1, uut.on_ctr);
    $dumpvars(1, uut.off_ctr);
    $dumpvars(1, watchdog_timeout);
    $dumpvars(1, uut.sensor_timeout_latched);

    // -- 14. Chip Selects --
    $dumpvars(1, cs_sensor_a);
    $dumpvars(1, cs_processing);
    $dumpvars(1, cs_sensor_b);
    $dumpvars(1, cs_sensor_c);

    // -- 15. SPI Bus --
    $dumpvars(1, spi_sclk);
    $dumpvars(1, spi_mosi);
    $dumpvars(1, miso_sensor_a_tb);
    $dumpvars(1, miso_b_wire);
    $dumpvars(1, miso_c_wire);

    // -- 16. UART TX Internals --
    $dumpvars(1, uart_tx);
    $dumpvars(1, uart_rx_drive);
    $dumpvars(1, uut.uart_start);
    $dumpvars(1, uut.uart_tx_data);
    $dumpvars(1, uut.uart_busy);

    // -- 17. FIFO Status --
    $dumpvars(1, fifo_empty);
    $dumpvars(1, fifo_full);
    $dumpvars(1, fifo_wr_ptr);
    $dumpvars(1, fifo_rd_ptr);
    $dumpvars(1, push_idle);

    // -- 18. Flash Bus + Controller Status --
    $dumpvars(1, flash_sclk);
    $dumpvars(1, flash_cs);
    $dumpvars(1, flash_mosi);
    $dumpvars(1, flash_miso_wire);
    $dumpvars(1, uut.flash_ctrl.busy_o);
    $dumpvars(1, uut.flash_ctrl.done_o);
    $dumpvars(1, uut.flash_ctrl.flash_wr_addr_o);

    // -- 19. Alarm BRAM Status --
    $dumpvars(1, uut.bram.entry_count_o);
    $dumpvars(1, uut.bram.wr_ptr_o);
    $dumpvars(1, uut.bram_rd_addr);
    $dumpvars(1, uut.bram_rd_data);
    $dumpvars(1, uut.bram_ecc_ded);

    // -- 20. SPI Slave (MCU) Bus + State --
    $dumpvars(1, sclk_s);
    $dumpvars(1, cs_s);
    $dumpvars(1, mosi_s);
    $dumpvars(1, miso_s_out);
    $dumpvars(1, uut.slave_inst.addr_reg);
    $dumpvars(1, uut.slave_inst.tx_active);
    $dumpvars(1, uut.slave_inst.selected_reg);
end

// -----------------------------------------------------------------------------
//  SECTION BANNER TASK
// -----------------------------------------------------------------------------
task print_section;
    input [511:0] title;
    input [255:0] subtitle;
    input [63:0]  t_range;
    begin
        $display("");
        $display("  ================================================================");
        $display("  SECTION  %0s  (%0s)", title, t_range);
        $display("  %0s", subtitle);
        $display("  ================================================================");
    end
endtask

// -----------------------------------------------------------------------------
//  MAIN TEST SEQUENCE
// -----------------------------------------------------------------------------
integer rand_seed;

initial begin
    pass_count=0; fail_count=0; test_num=1; global_clk_ctr=0;
    math_err_count=0; spi_proto_err_count=0; sensor_rel_err_count=0;
    reset=1; start=0; sleep=0; bod_i_drive=0; uart_rx_drive=1;
    sclk_s=0; cs_s=1; mosi_s=0; rand_seed=42;
    corrupt_enable=0; internal_corrupt_enable=0; mock_sensor_enable=0;
    global_saw_cc=0; global_saw_ee=0; global_saw_bb=0; global_saw_bf=0; global_saw_dd=0;
    set_all_sensors(13'sd0, 13'sd0, 13'sd0, 13'sd1);
    dashboard_sent = 1'b0;
    repeat(4) @(posedge clk); reset=0; repeat(2) @(posedge clk);

    $display("");
    $display("  +==================================================================+");
    $display("  |   SPI Air-Quality Monitoring System -- Processing-Centric Arch   |");
    $display("  |   100 MHz clk | 25 MHz SPI | 921600 baud UART | 50-bit frames   |");
    $display("  +==================================================================+");
    $display("  |  DATA PIPELINE:                                                  |");
    $display("  |   Sensor A/B/C -> spi_top CRC-validate -> TMR median ->          |");
    $display("  |   8-sample MAF -> Build averaged frame + CRC ->                  |");
    $display("  |   Processing Slave (AQI authority) -> aqi_out / category         |");
    $display("  +==================================================================+");
    $display("  |  CRC-8 poly=0x07 seed=0xFF applied at TWO checkpoints:           |");
    $display("  |   1. spi_top validates each incoming sensor frame CRC            |");
    $display("  |   2. Processing slave validates the internal averaged frame CRC  |");
    $display("  +==================================================================+");
    $display("");

    // ========================================================================
    //  SECTION A -- AQI via Processing Slave: All 6 EPA Categories
    // ========================================================================
    print_section("A  AQI via Processing Slave -- All 6 EPA Categories",
                  "AQI sourced from proc.aqi_value; proc.category used for cat checks",
                  "T01-T10");
    #1000;

    begin_test("T01 AQI Good   PM2.5=10");
    set_all_sensors(13'sd82, 13'sd28, 13'sd21, 13'sd1); run_cycle;
    log_result("AQI from proc slave <= 50",
               {51'b0, raw_pm25}, {55'b0, uut.proc.aqi_value}, 64'd50,
               uut.proc.aqi_value <= 9'd50);
    log_result("proc.category = 0 (Good)",
               64'd0, {61'b0, uut.proc.category}, 64'd0, uut.proc.category===3'd0);
    log_result("No internal CRC error",
               64'd0, {63'b0, uut.proc.crc_err_flag}, 64'd0, crc_err===1'b0);
    if (uut.proc.aqi_value > 9'd50)   math_err_count=math_err_count+1;
    if (uut.proc.category !== 3'd0)   math_err_count=math_err_count+1;
    end_test;

    begin_test("T02 AQI Moderate PM2.5=35");
    set_all_sensors(13'sd287, 13'sd137, 13'sd21, 13'sd1); run_cycle;
    log_result("proc AQI 51-100",
               {51'b0, raw_pm25}, {55'b0, uut.proc.aqi_value}, 64'd51,
               uut.proc.aqi_value>=9'd51 && uut.proc.aqi_value<=9'd100);
    log_result("proc.category = 1",
               64'd0, {61'b0, uut.proc.category}, 64'd1, uut.proc.category===3'd1);
    if (uut.proc.category !== 3'd1) math_err_count=math_err_count+1;
    end_test;

    begin_test("T03 AQI Unhealthy PM2.5=80");
    set_all_sensors(13'sd656, 13'sd274, 13'sd62, 13'sd1); run_silent_cycles(7); run_cycle;
    log_result("proc AQI 101-200",
               {51'b0, raw_pm25}, {55'b0, uut.proc.aqi_value}, 64'd101,
               uut.proc.aqi_value>=9'd101 && uut.proc.aqi_value<=9'd200);
    log_result("proc.category = 2",
               64'd0, {61'b0, uut.proc.category}, 64'd2, uut.proc.category===3'd2);
    if (uut.proc.category !== 3'd2) math_err_count=math_err_count+1;
    end_test;

    begin_test("T04 AQI Severe PM2.5=200");
    set_all_sensors(13'sd1639, 13'sd96, 13'sd50, 13'sd1); run_silent_cycles(7); run_cycle;
    log_result("proc AQI 201-300",
               {51'b0, raw_pm25}, {55'b0, uut.proc.aqi_value}, 64'd201,
               uut.proc.aqi_value>=9'd201 && uut.proc.aqi_value<=9'd300);
    log_result("proc.category = 3",
               64'd0, {61'b0, uut.proc.category}, 64'd3, uut.proc.category===3'd3);
    if (uut.proc.category !== 3'd3) math_err_count=math_err_count+1;
    end_test;

    begin_test("T05 AQI Very Severe PM2.5=300");
    set_all_sensors(13'sd2458, 13'sd96, 13'sd50, 13'sd1); run_silent_cycles(7); run_cycle;
    log_result("proc AQI 301-400",
               {51'b0, raw_pm25}, {55'b0, uut.proc.aqi_value}, 64'd301,
               uut.proc.aqi_value>=9'd301 && uut.proc.aqi_value<=9'd400);
    log_result("proc.category = 4",
               64'd0, {61'b0, uut.proc.category}, 64'd4, uut.proc.category===3'd4);
    if (uut.proc.category !== 3'd4) math_err_count=math_err_count+1;
    end_test;

    begin_test("T06 AQI Hazardous PM2.5=450");
    set_all_sensors(13'sd3686, 13'sd96, 13'sd50, 13'sd1); run_silent_cycles(7); run_cycle;
    log_result("proc AQI 401-500",
               {51'b0, raw_pm25}, {55'b0, uut.proc.aqi_value}, 64'd401,
               uut.proc.aqi_value>=9'd401 && uut.proc.aqi_value<=9'd500);
    log_result("proc AQI clamped<=500",
               64'd0, {55'b0, uut.proc.aqi_value}, 64'd500,
               uut.proc.aqi_value<=9'd500);
    log_result("proc.category = 5",
               64'd0, {61'b0, uut.proc.category}, 64'd5, uut.proc.category===3'd5);
    if (uut.proc.category !== 3'd5) math_err_count=math_err_count+1;
    end_test;

    begin_test("T07 PM2.5 seg0/seg1 boundary");
    set_all_sensors(13'sd98, 13'sd28, 13'sd21, 13'sd1); run_silent_cycles(7); run_cycle;
    log_result("proc AQI<=50 at seg0 boundary",
               64'd98, {55'b0, uut.proc.aqi_value}, 64'd50,
               uut.proc.aqi_value<=9'd50);
    end_test;

    begin_test("T08 Decimal inputs PM2.5/PM10");
    set_all_sensors(phys_to_raw(35.7,499.0), phys_to_raw(52.3,599.0),
                    phys_to_raw(120.5,999.0), 13'sd1); run_cycle;
    log_result("proc cat<=Unhealthy",
               64'd0, {61'b0, uut.proc.category}, 64'd2,
               uut.proc.category<=3'd2);
    log_result("No CRC error",
               64'd0, {63'b0, crc_err}, 64'd0, crc_err===1'b0);
    end_test;

    begin_test("T09 VOC dominant AQI");
    set_all_sensors(13'sd82, 13'sd28, phys_to_raw(900.0,999.0), 13'sd1); run_cycle;
    log_result("proc.aqi reflects VOC dominance",
               64'd0, {55'b0, uut.proc.aqi_value}, 64'd1,
               uut.proc.aqi_value >= 9'd1);
    log_result("proc cat>=Moderate",
               64'd0, {61'b0, uut.proc.category}, 64'd1,
               uut.proc.category>=3'd1);
    end_test;

    begin_test("T10 PM10 dominant AQI");
    set_all_sensors(13'sd82, phys_to_raw(500.0,599.0), 13'sd21, 13'sd1); run_silent_cycles(7); run_cycle;
    log_result("proc AQI>=Severe due to PM10",
               64'd0, {55'b0, uut.proc.aqi_value}, 64'd201,
               uut.proc.aqi_value>=9'd201);
    log_result("proc cat>=Severe",
               64'd0, {61'b0, uut.proc.category}, 64'd3,
               uut.proc.category>=3'd3);
    end_test;

    // ========================================================================
    //  SECTION B -- Input Validation (unchanged behaviour)
    // ========================================================================
    print_section("B  Input Validation & Clamping",
                  "Negative/zero inputs, per-channel error flags, AQI=0",
                  "T11-T15");
    #1000;

    begin_test("T11 All channels negative -> AQI=0");
    set_all_sensors(-13'sd100, -13'sd50, -13'sd30, -13'sd10); run_silent_cycles(7); run_cycle;
    log_result("input_err=1111",
               64'hF, {60'b0, input_err}, 64'hF, input_err[3:0]===4'b1111);
    log_result("proc AQI=0",
               64'd0, {55'b0, uut.proc.aqi_value}, 64'd0, uut.proc.aqi_value===9'd0);
    end_test;

    begin_test("T12 PM2.5 negative only");
    set_all_sensors(-13'sd82, 13'sd28, 13'sd21, 13'sd1); run_cycle;
    log_result("input_err[3]=1",
               64'd0, {63'b0, input_err[3]}, 64'd1, input_err[3]===1'b1);
    log_result("input_err[2]=0",
               64'd0, {63'b0, input_err[2]}, 64'd0, input_err[2]===1'b0);
    end_test;

    begin_test("T13 PM2.5=0 invalid");
    set_all_sensors(13'sd0, 13'sd28, 13'sd21, 13'sd1); run_cycle;
    log_result("input_err[3]=1",
               64'd0, {63'b0, input_err[3]}, 64'd1, input_err[3]===1'b1);
    end_test;

    begin_test("T14 CO channel negative");
    set_all_sensors(13'sd82, 13'sd28, 13'sd21, -13'sd50); run_cycle;
    log_result("input_err[0]=1 (CO bad)",
               64'd0, {63'b0, input_err[0]}, 64'd1, input_err[0]===1'b1);
    log_result("PM/VOC valid",
               64'd0, {61'b0, input_err[3:1]}, 64'd0, input_err[3:1]===3'b000);
    end_test;

    begin_test("T15 PM10+CO bad PM2.5+VOC valid");
    set_all_sensors(13'sd82, -13'sd28, 13'sd21, -13'sd1); run_cycle;
    log_result("input_err[2]=1",
               64'd0, {63'b0, input_err[2]}, 64'd1, input_err[2]===1'b1);
    log_result("input_err[0]=1",
               64'd0, {63'b0, input_err[0]}, 64'd1, input_err[0]===1'b1);
    end_test;

    // ========================================================================
    //  SECTION C -- TMR Median Voter
    // ========================================================================
    print_section("C  Triple Redundancy Median Voter",
                  "Outlier rejection, fault flags, AQI stability via processing slave",
                  "T16-T20");
    #1000;

    begin_test("T16 All 3 sensors equal -- no fault");
    set_all_sensors(13'sd82, 13'sd28, 13'sd21, 13'sd1); run_cycle;
    log_result("sensor_fault=0",
               64'd0, {61'b0, sensor_fault}, 64'd0, sensor_fault===3'b000);
    log_result("med_pm25 == pm25_a",
               64'd0, {54'b0, uut.med_pm25}, {54'b0, uut.pm25_a},
               uut.med_pm25===uut.pm25_a);
    end_test;

    begin_test("T17 Sensor C spike -- outlier rejected");
    run_silent_cycles(3);
    raw_pm25=13'sd82;   raw_pm10=13'sd28;   raw_voc=13'sd21;   raw_co=13'sd1;
    raw_pm25_b=13'sd82; raw_pm10_b=13'sd28; raw_voc_b=13'sd21; raw_co_b=13'sd1;
    raw_pm25_c=13'sd4000; raw_pm10_c=13'sd28; raw_voc_c=13'sd21; raw_co_c=13'sd1;
    run_cycle;
    log_result("fault[0]=1 (C deviated)",
               64'd4000, {61'b0, sensor_fault}, 64'd1, sensor_fault[0]===1'b1);
    log_result("proc AQI unaffected by spike",
               64'd0, {55'b0, uut.proc.aqi_value}, 64'd80,
               uut.proc.aqi_value<9'd80);
    if (sensor_fault[0]!==1'b1) sensor_rel_err_count=sensor_rel_err_count+1;
    end_test;

    begin_test("T18 Sensor A spike -- B+C median");
    raw_pm25=13'sd1639; raw_pm10=13'sd28; raw_voc=13'sd21; raw_co=13'sd1;
    raw_pm25_b=13'sd82; raw_pm10_b=13'sd28; raw_voc_b=13'sd21; raw_co_b=13'sd1;
    raw_pm25_c=13'sd82; raw_pm10_c=13'sd28; raw_voc_c=13'sd21; raw_co_c=13'sd1;
    run_cycle;
    log_result("fault[2]=1 (A outlier)",
               64'd1639, {61'b0, sensor_fault}, 64'd4, sensor_fault[2]===1'b1);
    log_result("proc AQI stays low",
               64'd0, {55'b0, uut.proc.aqi_value}, 64'd50,
               uut.proc.aqi_value<=9'd80);
    end_test;

    begin_test("T19 Two sensors spike -- majority wins");
    raw_pm25=13'sd1639;   raw_pm10=13'sd28; raw_voc=13'sd21; raw_co=13'sd1;
    raw_pm25_b=13'sd1639; raw_pm10_b=13'sd28; raw_voc_b=13'sd21; raw_co_b=13'sd1;
    raw_pm25_c=13'sd82;   raw_pm10_c=13'sd28; raw_voc_c=13'sd21; raw_co_c=13'sd1;
    run_cycle;
    enable_print = 1'b0;
    repeat(6) begin
        raw_pm25=13'sd1639;   raw_pm10=13'sd28; raw_voc=13'sd21; raw_co=13'sd1;
        raw_pm25_b=13'sd1639; raw_pm10_b=13'sd28; raw_voc_b=13'sd21; raw_co_b=13'sd1;
        raw_pm25_c=13'sd82;   raw_pm10_c=13'sd28; raw_voc_c=13'sd21; raw_co_c=13'sd1;
        run_cycle;
    end
    enable_print = 1'b1;
    begin
        raw_pm25=13'sd1639;   raw_pm10=13'sd28; raw_voc=13'sd21; raw_co=13'sd1;
        raw_pm25_b=13'sd1639; raw_pm10_b=13'sd28; raw_voc_b=13'sd21; raw_co_b=13'sd1;
        raw_pm25_c=13'sd82;   raw_pm10_c=13'sd28; raw_voc_c=13'sd21; raw_co_c=13'sd1;
        run_cycle;
    end
    log_result("fault[0]=1 (C laggard)",
               64'd82, {61'b0, sensor_fault}, 64'd1, sensor_fault[0]===1'b1);
    log_result("proc AQI rising (majority high)",
               64'd0, {55'b0, uut.proc.aqi_value}, 64'd50,
               uut.proc.aqi_value>9'd50);
    end_test;

    begin_test("T20 Sensor B drift");
    raw_pm25=13'sd82;     raw_pm10=13'sd96; raw_voc=13'sd50; raw_co=13'sd50;
    raw_pm25_b=13'sd1638; raw_pm10_b=13'sd96; raw_voc_b=13'sd50; raw_co_b=13'sd50;
    raw_pm25_c=13'sd82;   raw_pm10_c=13'sd96; raw_voc_c=13'sd50; raw_co_c=13'sd50;
    run_cycle;
    log_result("fault[1]=1 (B drifted)",
               64'd1638, {61'b0, sensor_fault}, 64'd2, sensor_fault[1]===1'b1);
    log_result("fault[2]=0 (A ok)",
               64'd0, {63'b0, sensor_fault[2]}, 64'd0, sensor_fault[2]===1'b0);
    if (sensor_fault[1]!==1'b1) sensor_rel_err_count=sensor_rel_err_count+1;
    end_test;

    // ========================================================================
    //  SECTION D -- 8-Sample MAF Reflected in Averaged Frame
    // ========================================================================
    print_section("D  MAF Smoothing -- Verified via Internal Transfer Frame",
                  "avg_pm25/avg_pm10 must appear in proc_frame_latched[49:10]",
                  "T21-T24");
    #1000;

    set_all_sensors(13'sd82, 13'sd28, 13'sd21, 13'sd1);
    run_silent_cycles(10);

    begin_test("T21 MAF pre-fill -- avg converges");
    enable_print = 1'b0;
    repeat(7) begin set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle; end
    enable_print = 1'b1;
    begin set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle; end
    log_result("avg_pm25<20",
               64'd0, {54'b0, uut.avg_pm25}, 64'd20, uut.avg_pm25<10'd20);
    log_result("maf_valid=1",
               64'd0, {63'b0, uut.maf_valid}, 64'd1, uut.maf_valid===1'b1);
    log_result("Internal frame[49:40] == avg_pm25",
               {54'b0, uut.avg_pm25},
               {54'b0, uut.proc_frame_latched[49:40]},
               {54'b0, uut.avg_pm25},
               uut.proc_frame_latched[49:40] === uut.avg_pm25);
    end_test;

    begin_test("T22 MAF step Good->Severe 8 cyc");
    begin : maf_step
        integer cyc; reg [9:0] prev_avg; reg ok;
        prev_avg = uut.avg_pm25; ok = 1;
        enable_print = 1'b0;
        for (cyc=1; cyc<=7; cyc=cyc+1) begin
            set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle;
            if (uut.avg_pm25 < prev_avg) ok = 0;
            prev_avg = uut.avg_pm25;
        end
        enable_print = 1'b1;
        begin
            set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle;
            if (uut.avg_pm25 < prev_avg) ok = 0;
            prev_avg = uut.avg_pm25;
        end
        log_result("avg_pm25 monotone rises",
                   64'd0, {54'b0, uut.avg_pm25}, 64'd100, ok);
        log_result("avg_pm25>100 after 8 Severe",
                   64'd0, {54'b0, uut.avg_pm25}, 64'd100, uut.avg_pm25>10'd100);
    end
    end_test;

    begin_test("T23 Single spike dampened by MAF");
    enable_print = 1'b0;
    repeat(7) begin set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle; end
    enable_print = 1'b1;
    begin set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle; end
    begin : spike_test
        reg [9:0] avg_before, avg_after;
        avg_before = uut.avg_pm25;
        set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle;
        avg_after = uut.avg_pm25;
        log_result("Spike delta<50",
                   {54'b0, avg_before}, {54'b0, avg_after},
                   {54'b0, avg_before + 10'd50},
                   avg_after < avg_before + 10'd50);
    end
    end_test;

    begin_test("T24 MAF recovery after spike");
    set_all_sensors(13'sd82, 13'sd28, 13'sd21, 13'sd1); run_cycle;
    log_result("All faults clear",
               64'd0, {61'b0, sensor_fault}, 64'd0, sensor_fault===3'b000);
    log_result("proc AQI<100 after recovery",
               64'd0, {55'b0, uut.proc.aqi_value}, 64'd100,
               uut.proc.aqi_value<9'd100);
    end_test;

    // ========================================================================
    //  SECTION E -- Alarm Hysteresis (category from proc slave)
    // ========================================================================
    print_section("E  Alarm Hysteresis via Processing Slave Category",
                  "N_ON=4 assert, N_OFF=8 deassert; cat comes from proc.category",
                  "T25-T28");
    #1000;

    run_silent_cycles(12);

    // Reset MAF to Good baseline first
    run_silent_cycles(8);

    begin_test("T25 Alarm OFF before threshold reached");
    // Send 8 Severe to fill MAF, then check alarm is still OFF (needs N_ON=4 consecutive)
    // MAF converges around cycle 8, so alarm counter starts late
    begin : t25b
        integer i;
        enable_print = 1'b0;
        for (i=0; i<7; i=i+1) begin
            set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle;
        end
        enable_print = 1'b1;
        set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle;
    end
    // After 8 cycles, MAF just converged. Alarm counter may have 0-2 counts.
    // Check alarm is still OFF (hasn't reached N_ON=4 yet)
    @(posedge clk); #1;
    log_result("Alarm OFF before N_ON reached",
               64'd0, {63'b0, uut.alarm_state}, 64'd0, uut.alarm_state===1'b0);
    end_test;

    begin_test("T26 Continued Severe -- alarm asserts");
    // Continue Severe cycles until alarm triggers
    begin : t26b
        integer i;
        enable_print = 1'b0;
        for (i=0; i<5; i=i+1) begin
            set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle;
        end
        enable_print = 1'b1;
        set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle;
    end
    @(posedge clk); #1;
    log_result("alarm_state=1",
               64'd4, {63'b0, uut.alarm_state}, 64'd1, uut.alarm_state===1'b1);
    log_result("alarm_pin HIGH",
               64'd0, {63'b0, alarm_pin}, 64'd1, alarm_pin===1'b1);
    end_test;

    begin_test("T27 7 Good readings -- alarm still ON");
    begin : t27b
        integer i;
        enable_print = 1'b0;
        for (i=0; i<6; i=i+1) begin
            set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle;
        end
        enable_print = 1'b1;
        set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle;
    end
    @(posedge clk); #1;
    log_result("Alarm ON at N_OFF-1",
               64'd0, {63'b0, uut.alarm_state}, 64'd1, uut.alarm_state===1'b1);
    end_test;

    begin_test("T28 8th Good -- alarm deasserts");
    begin : t28b
        integer i;
        enable_print = 1'b0;
        for (i=0; i<8; i=i+1) begin
            set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle;
        end
        enable_print = 1'b1;
        set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle;
    end
    @(posedge clk); #1;
    log_result("alarm_state=0",
               64'd0, {63'b0, uut.alarm_state}, 64'd0, uut.alarm_state===1'b0);
    log_result("alarm_pin LOW",
               64'd0, {63'b0, alarm_pin}, 64'd0, alarm_pin===1'b0);
    end_test;

    // ========================================================================
    //  SECTION F -- NEW: Sensor-Level CRC Fault Injection
    //  Tests that spi_top detects bad sensor frames and excludes them from TMR
    // ========================================================================
    print_section("F  Sensor-Level CRC Fault Injection",
                  "spi_top checks each sensor CRC; bad sensor excluded from TMR",
                  "T29-T32");
    #1000;

    begin_test("T29 Clean frames -- all sensor CRCs OK");
    corrupt_enable = 0;
    set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle;
    @(posedge clk); #1;
    log_result("sens_a_crc_ok=1",
               64'd0, {63'b0, uut.sens_a_crc_ok}, 64'd1, uut.sens_a_crc_ok===1'b1);
    log_result("sens_b_crc_ok=1",
               64'd0, {63'b0, uut.sens_b_crc_ok}, 64'd1, uut.sens_b_crc_ok===1'b1);
    log_result("sens_c_crc_ok=1",
               64'd0, {63'b0, uut.sens_c_crc_ok}, 64'd1, uut.sens_c_crc_ok===1'b1);
    log_result("system crc_err=0",
               64'd0, {63'b0, crc_err}, 64'd0, crc_err===1'b0);
    end_test;

    begin_test("T30 Sensor A bit-flip -- CRC fail");
    set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1);
    fork
        begin : t30i
            wait(uut.cs1===1'b0);
            corrupt_enable = 1'b1;
            @(posedge uut.cs1);
            corrupt_enable = 1'b0;
        end
        run_cycle;
    join
    @(posedge clk); #1;
    log_result("sens_a_crc_ok=0 detected by spi_top",
               64'd0, {63'b0, uut.sens_a_crc_ok}, 64'd0, uut.sens_a_crc_ok===1'b0);
    log_result("crc_err=1 propagated to output",
               64'd0, {63'b0, crc_err}, 64'd1, crc_err===1'b1);
    if (uut.sens_a_crc_ok!==1'b0) spi_proto_err_count=spi_proto_err_count+1;
    end_test;

    begin_test("T31 Sensor A CRC fail: B+C valid");
    // After sensor A CRC fault, TMR uses substituted value; proc slave still gets valid frame
    set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1);
    fork
        begin : t31i
            wait(uut.cs1===1'b0);
            corrupt_enable = 1'b1;
            @(posedge uut.cs1);
            corrupt_enable = 1'b0;
        end
        run_cycle;
    join
    @(posedge clk); #1;
    log_result("proc slave still produces valid AQI",
               64'd0, {55'b0, uut.proc.aqi_value}, 64'd500,
               uut.proc.aqi_value <= 9'd500);
    log_result("proc.category in valid range",
               64'd0, {61'b0, uut.proc.category}, 64'd5,
               uut.proc.category <= 3'd5);
    end_test;

    begin_test("T32 Clean frame -- crc_err clears");
    corrupt_enable = 0;
    set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle;
    log_result("crc_err clears after clean cycle",
               64'd0, {63'b0, crc_err}, 64'd0, crc_err===1'b0);
    log_result("sens_a_crc_ok=1 restored",
               64'd0, {63'b0, uut.sens_a_crc_ok}, 64'd1, uut.sens_a_crc_ok===1'b1);
    end_test;

    // ========================================================================
    //  SECTION G -- NEW: Internal Transfer Frame CRC Injection
    //  Inject error into the averaged frame sent to processing slave
    // ========================================================================
    print_section("G  Internal Transfer Frame CRC Injection",
                  "Force bad CRC in proc_frame_latched; slave reports crc_err_flag",
                  "T33-T35");
    #1000;

    begin_test("T33 Internal 1-bit flip: CRC err");
    set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1);
    fork
        begin : t33i
            wait(uut.state == 7'd8); // SEND_PROC
            force_tx_data_val = uut.proc_frame_latched ^ 50'h0000000000001;
            force uut.master_tx_data = force_tx_data_val;
            @(posedge clk); #1;
            release uut.master_tx_data;
        end
        run_cycle;
    join
    log_result("proc.crc_err_flag detected internal frame corruption",
               50'h0000000000001, {63'b0, crc_err}, 64'd1, crc_err===1'b1);
    if (crc_err!==1'b1) spi_proto_err_count=spi_proto_err_count+1;
    end_test;

    begin_test("T34 Internal 8-bit burst: reject");
    set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1);
    fork
        begin : t34i
            wait(uut.state == 7'd8);
            force_tx_data_val = uut.proc_frame_latched ^ 50'h00000000000FF;
            force uut.master_tx_data = force_tx_data_val;
            @(posedge clk); #1;
            release uut.master_tx_data;
        end
        run_cycle;
    join
    log_result("proc slave detects 8-bit burst error in internal frame",
               50'h00000000000FF, {63'b0, crc_err}, 64'd1, crc_err===1'b1);
    if (crc_err!==1'b1) spi_proto_err_count=spi_proto_err_count+1;
    end_test;

    begin_test("T35 Clean frame after injection");
    set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle;
    log_result("crc_err clears after clean internal frame",
               64'd0, {63'b0, crc_err}, 64'd0, crc_err===1'b0);
    log_result("proc AQI valid after recovery",
               64'd0, {55'b0, uut.proc.aqi_value}, 64'd500,
               uut.proc.aqi_value<=9'd500);
    end_test;

    // ========================================================================
    //  SECTION H -- Watchdog & Sensor Fault
    // ========================================================================
    print_section("H  Watchdog Timer & Sensor Fault",
                  "1024-cycle timeout, sensor dropout, recovery",
                  "T36-T38");
    #1000;

    begin_test("T36 Normal transaction -- no watchdog");
    set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle;
    log_result("No timeout",
               64'd0, {63'b0, sensor_timeout}, 64'd0, sensor_timeout===1'b0);
    end_test;

    begin_test("T37 Sensor silent -- watchdog fires");
    wait_idle;
    mock_sensor_enable = 1;
    start = 1; @(posedge clk); #1; start = 0;
    wait(uut.state != 0); wait(uut.state == 0); #1;
    log_result("sensor_timeout=1",
               64'd0, {63'b0, sensor_timeout}, 64'd1, sensor_timeout===1'b1);
    log_result("CS1 released",
               64'd0, {63'b0, uut.cs1}, 64'd1, uut.cs1===1'b1);
    mock_sensor_enable = 0;
    sensor_rel_err_count = sensor_rel_err_count + 1;
    set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle;
    log_result("Timeout clears after recovery",
               64'd0, {63'b0, sensor_timeout}, 64'd0, sensor_timeout===1'b0);
    end_test;

    begin_test("T38 Post-recovery proc AQI correct");
    set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle;
    log_result("proc AQI<=50 post-recovery",
               64'd0, {55'b0, uut.proc.aqi_value}, 64'd50,
               uut.proc.aqi_value<=9'd50);
    log_result("crc_err=0",
               64'd0, {63'b0, crc_err}, 64'd0, crc_err===1'b0);
    end_test;

    // ========================================================================
    //  SECTION I -- Sleep / Wake
    // ========================================================================
    print_section("I  Sleep / Wake Mode",
                  "Sleep blocks FSM; UART 0x04 wake command; post-wake AQI verify",
                  "T39-T41");
    #1000;

    begin_test("T39 Sleep blocks start_i");
    wait_idle; sleep = 1;
    repeat(4) @(posedge clk); #1;
    start=1; @(posedge clk); #1; start=0;
    repeat(20) @(posedge clk);
    log_result("FSM stays IDLE",
               64'd0, {57'b0, uut.state}, 64'd0, uut.state==0);
    log_result("CS lines HIGH",
               64'd0, {62'b0, uut.cs1, uut.cs2}, 64'h3,
               uut.cs1===1'b1 && uut.cs2===1'b1);
    sleep = 0;
    end_test;

    begin_test("T40 UART 0x04 triggers wakeup_o");
    wait_idle;
    begin : t40p
        reg saw_wakeup; saw_wakeup=0;
        fork
            uart_send(8'h04);
            begin : t40m
                integer w;
                for (w=0; w<2000; w=w+1) begin
                    @(posedge clk);
                    if (wakeup) begin saw_wakeup=1; disable t40m; end
                end
            end
        join
        log_result("wakeup_o pulsed",
                   64'h04, {63'b0, saw_wakeup}, 64'd1, saw_wakeup===1'b1);
    end
    @(posedge clk); @(negedge clk);
    log_result("wakeup_o LOW after 1 cycle",
               64'd0, {63'b0, wakeup}, 64'd0, wakeup===1'b0);
    end_test;

    begin_test("T41 Post-wake proc AQI correct");
    set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle;
    log_result("proc AQI<=50 post-wake",
               64'd0, {55'b0, uut.proc.aqi_value}, 64'd50,
               uut.proc.aqi_value<=9'd50);
    log_result("crc_err=0 post-wake",
               64'd0, {63'b0, crc_err}, 64'd0, crc_err===1'b0);
    end_test;

    // ========================================================================
    //  SECTION J -- UART DMA & Commands
    // ========================================================================
    print_section("J  UART DMA Non-Blocking & Commands",
                  "done_o before FIFO drains; 0xAA/CC/EE/BB/BF/DD packets",
                  "T42-T47");
    #1000;

    begin_test("T42 done_o fires before FIFO drains");
    begin : t42b
        real t_done, t_fe;
        wait_idle;
        set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1);
        @(posedge clk); #1; start=1; @(posedge clk); #1; start=0;
        @(posedge done); t_done = $realtime;
        wait(uut.fifo_empty && !uut.uart_busy);
        t_fe = $realtime;
        log_result("FIFO drains AFTER done_o",
                   64'd0, 64'd0, 64'd0, t_fe > t_done);
    end
    end_test;

    begin_test("T43 Live packet starts with 0xAA");
    begin : t43b
        reg saw_aa; saw_aa=0;
        wait_idle; set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle;
        begin : aam
            integer w;
            for (w=0; w<5000; w=w+1) begin
                @(posedge clk);
                if (uut.uart_start && uut.uart_tx_data==8'hAA)
                    begin saw_aa=1; disable aam; end
            end
        end
        log_result("0xAA header seen",
                   64'hAA, {63'b0, saw_aa}, 64'd1, saw_aa===1'b1);
    end
    end_test;

    begin_test("T44 UART 0x01 -> 0xCC ACK");
    global_saw_cc=0;
    wait_idle; set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle;
    fork uart_send(8'h01); join
    begin : ccm
        integer w;
        for (w=0; w<10000; w=w+1) begin @(posedge clk); if (global_saw_cc) disable ccm; end
    end
    log_result("0xCC ACK seen",
               64'h01, {63'b0, global_saw_cc}, 64'd1, global_saw_cc===1'b1);
    end_test;

    begin_test("T45 Watchdog -> 0xEE error packet");
    global_saw_ee=0;
    wait_idle; mock_sensor_enable=1;
    start=1; @(posedge clk); #1; start=0;
    wait(uut.state != 0); wait(uut.state == 0);
    mock_sensor_enable=0;
    begin : eem
        integer w;
        for (w=0; w<10000; w=w+1) begin @(posedge clk); if (global_saw_ee) disable eem; end
    end
    log_result("0xEE error header seen",
               64'd0, {63'b0, global_saw_ee}, 64'd1, global_saw_ee===1'b1);
    set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle;
    end_test;

    begin_test("T46 UART 0x02 -> BRAM dump BB/DD");
    global_saw_bb=0; global_saw_dd=0;
    enable_print = 1'b0;
    repeat(20) begin set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle; end
    repeat(3)  begin set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle; end
    enable_print = 1'b1;
    begin set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle; end
    wait_idle; fork uart_send(8'h02); join
    repeat(30) @(posedge clk);
    begin : bbm
        integer w;
        for (w=0; w<240000; w=w+1) begin
            @(posedge clk); if (global_saw_bb && global_saw_dd) disable bbm;
        end
    end
    log_result("BRAM has entries",
               64'd0, {57'b0, uut.bram.entry_count_o}, 64'd1,
               uut.bram.entry_count_o>0);
    log_result("0xBB header seen",
               64'd0, {63'b0, global_saw_bb}, 64'd1, global_saw_bb===1'b1);
    log_result("0xDD end marker",
               64'd0, {63'b0, global_saw_dd}, 64'd1, global_saw_dd===1'b1);
    fork uart_send(8'h03); join
    repeat(20) @(posedge clk);
    end_test;

    begin_test("T47 UART 0x06 -> flash dump with 0xBF");
    global_saw_bf=0; global_saw_dd=0;
    wait_idle;
    begin : wait_flash_idle
        integer fw;
        for (fw=0; fw<200000; fw=fw+1) begin
            @(posedge clk);
            if (!uut.flash_ctrl.busy_o && !uut.flash_wr_pending) disable wait_flash_idle;
        end
    end
    repeat(10) @(posedge clk);
    fork uart_send(8'h06); join
    repeat(30) @(posedge clk);
    begin : bfm
        integer w;
        for (w=0; w<20000; w=w+1) begin @(posedge clk); if (global_saw_bf) disable bfm; end
    end
    log_result("0xBF flash header seen",
               64'd0, {63'b0, global_saw_bf}, 64'd1, global_saw_bf===1'b1);
    begin : fdm
        integer w;
        for (w=0; w<120000; w=w+1) begin
            @(posedge clk);
            if (!uut.cmd_pending_flash_dump && uut.state==0) disable fdm;
        end
    end
    repeat(10) @(posedge clk);
    end_test;

    // ========================================================================
    //  SECTION K -- Flash Persistence
    // ========================================================================
    print_section("K  SPI Flash Persistence",
                  "Flash pointer advance, verify-retry, BOD safe-abort",
                  "T48-T50");
    #1000;

    begin_test("T48 Alarm -- flash ptr advances");
    begin : t48b
        reg [11:0] addr_before;
        enable_print = 1'b0;
        repeat(20) begin set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle; end
        enable_print = 1'b1;
        addr_before = uut.flash_ctrl.flash_wr_addr_o;
        enable_print = 1'b0;
        repeat(11) begin set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle; end
        enable_print = 1'b1;
        begin set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle; end
        repeat(50) @(posedge clk);
        begin : wf48
            integer w;
            for (w=0; w<200000; w=w+1) begin
                @(posedge clk); if (!uut.flash_ctrl.busy_o) disable wf48;
            end
        end
        repeat(10) @(posedge clk);
        log_result("Flash pointer advanced",
                   {52'b0, addr_before}, {52'b0, uut.flash_ctrl.flash_wr_addr_o},
                   {52'b0, addr_before + 12'd1},
                   uut.flash_ctrl.flash_wr_addr_o > addr_before);
    end
    end_test;

    begin_test("T49 Verify fail -> flash_write_err");
    begin : t49b
        enable_print = 1'b0;
        repeat(20) begin set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle; end
        enable_print = 1'b1;
        flash_model.corrupt_readback = 1'b1;
        enable_print = 1'b0;
        repeat(20) begin set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle; end
        repeat(3)  begin set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle; end
        enable_print = 1'b1;
        begin set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle; end
        begin : wf49
            integer w;
            for (w=0; w<200000; w=w+1) begin
                @(posedge clk); if (!uut.flash_ctrl.busy_o) disable wf49;
            end
        end
        #1;
        log_result("flash_write_err=1 after retry",
                   64'd0, {63'b0, flash_write_err}, 64'd1, flash_write_err===1'b1);
        flash_model.corrupt_readback = 1'b0;
        repeat(50) @(posedge clk);
        enable_print = 1'b0;
        repeat(20) begin set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle; end
        repeat(3)  begin set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle; end
        enable_print = 1'b1;
        begin set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle; end
        force uut.flash_wr_pending = 1'b1;
        @(posedge clk); #1; release uut.flash_wr_pending;
        begin : wf49b
            integer w;
            for (w=0; w<200000; w=w+1) begin
                @(posedge clk); if (!uut.flash_ctrl.busy_o) disable wf49b;
            end
        end
        repeat(10) @(posedge clk);
        log_result("flash_write_err clears",
                   64'd0, {63'b0, flash_write_err}, 64'd0, flash_write_err===1'b0);
    end
    end_test;

    begin_test("T50 BOD mid-write -- safe abort");
    begin : t50b
        enable_print = 1'b0;
        repeat(7) begin set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle; end
        enable_print = 1'b1;
        begin set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle; end
        fork
            begin : t50a
                enable_print = 1'b0;
                repeat(11) begin
                    set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle;
                end
                enable_print = 1'b1;
                begin
                    set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle;
                end
            end
            begin : t50bi
                // Timeout: wait up to 500000 clocks for flash_cs to go low
                reg t50bi_got_cs;
                integer t50bi_w;
                t50bi_got_cs = 0;
                begin : t50bi_poll
                    for (t50bi_w = 0; t50bi_w < 500000; t50bi_w = t50bi_w + 1) begin
                        @(posedge clk);
                        if (flash_cs === 1'b0) begin
                            t50bi_got_cs = 1;
                            disable t50bi_poll;
                        end
                    end
                end
                if (!t50bi_got_cs) begin
                    $display("  |  [SKIP] T50 BOD: flash_cs never went low (timeout after 500000 clks)");
                    log_result("flash_cs went low for BOD test",
                               64'd0, {63'b0, flash_cs}, 64'd0, 1'b0);
                end else begin
                    bod_i_drive=1;
                    repeat(4) @(posedge clk); #1;
                    log_result("bod_flag asserted",
                               64'd1, {63'b0, bod_flag}, 64'd1, bod_flag===1'b1);
                    log_result("flash_cs released",
                               64'd0, {63'b0, flash_cs}, 64'd1, flash_cs===1'b1);
                    log_result("S_BOD_WAIT state",
                               64'd0, {58'b0, uut.flash_ctrl.state}, 64'd31,
                               uut.flash_ctrl.state==31);
                    bod_i_drive=0;
                    repeat(3) @(posedge clk); #1;
                    log_result("Not busy after BOD deassert",
                               64'd0, {63'b0, uut.flash_ctrl.busy_o}, 64'd0,
                               !uut.flash_ctrl.busy_o);
                end
            end
        join
        bod_i_drive=0; repeat(20) @(posedge clk);
    end
    end_test;

    // ========================================================================
    //  SECTION L -- MCU SPI Slave Register Map
    // ========================================================================
    print_section("L  SPI Slave Register Map",
                  "MCU reads AQI/Category (now from proc slave); dynamic thresholds",
                  "T51-T53");
    #1000;

    begin_test("T51 MCU reads AQI register");
    begin : t51b
        reg [15:0] rd;
        enable_print = 1'b0;
        repeat(7) begin set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle; end
        enable_print = 1'b1;
        begin set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle; end
        wait_idle; #100;
        spi_slave_read(8'h84, rd);
        log_result("AQI register Good range (proc slave)",
                   64'h84, {55'b0, rd[8:0]}, 64'd50, rd[8:0]<=9'd50);
        log_result("AQI upper bits=0",
                   64'd0, {49'b0, rd[15:9]}, 64'd0, rd[15:9]===7'd0);
    end
    end_test;

    begin_test("T52 MCU reads Category register");
    begin : t52b
        reg [15:0] rd;
        spi_slave_read(8'h85, rd);
        log_result("Cat in 0-5",
                   64'd0, {61'b0, rd[2:0]}, 64'd5, rd[2:0]<=3'd5);
        log_result("Upper bits=0",
                   64'd0, {51'b0, rd[15:3]}, 64'd0, rd[15:3]===13'd0);
    end
    end_test;

    begin_test("T53 MCU writes fault threshold");
    spi_slave_write(8'h07, 8'h14); repeat(10) @(posedge clk);
    log_result("fault_thresh_dyn=20",
               64'h14, {54'b0, uut.fault_thresh_dyn}, 64'd20,
               uut.fault_thresh_dyn===10'd20);
    spi_slave_write(8'h07, 8'h32); repeat(10) @(posedge clk);
    log_result("fault_thresh restored to 50",
               64'h32, {54'b0, uut.fault_thresh_dyn}, 64'd50,
               uut.fault_thresh_dyn===10'd50);
    end_test;

    // ========================================================================
    //  SECTION M -- Extreme Boundary Sweeps
    // ========================================================================
    print_section("M  Extreme Boundary Sweeps",
                  "Max saturation, min valid, breakpoint sweeps, random, CS mutex",
                  "T54-T60");
    #1000;

    begin_test("T54 Max Saturation 0x0FFF");
    set_all_sensors(13'sh0FFF, 13'sh0FFF, 13'sh0FFF, 13'sh0FFF); run_silent_cycles(7); run_cycle;
    log_result("proc AQI clamped<=500",
               64'h0FFF, {55'b0, uut.proc.aqi_value}, 64'd500,
               uut.proc.aqi_value<=9'd500);
    log_result("proc cat=5 Hazardous",
               64'h0FFF, {61'b0, uut.proc.category}, 64'd5,
               uut.proc.category===3'd5);
    log_result("No CRC error at max",
               64'd0, {63'b0, crc_err}, 64'd0, crc_err===1'b0);
    if (uut.proc.aqi_value > 9'd500) math_err_count=math_err_count+1;
    end_test;

    begin_test("T55 Minimum valid input (raw=1)");
    set_all_sensors(13'sd1, 13'sd1, 13'sd1, 13'sd1); run_silent_cycles(7); run_cycle;
    log_result("proc AQI=0 at min",
               64'd1, {55'b0, uut.proc.aqi_value}, 64'd0, uut.proc.aqi_value===9'd0);
    log_result("cat=Good at min",
               64'd0, {61'b0, uut.proc.category}, 64'd0, uut.proc.category===3'd0);
    log_result("No input_err at min valid",
               64'd0, {60'b0, input_err}, 64'd0, input_err===4'b0000);
    end_test;

    begin_test("T56 Randomized noise sweep (32)");
    begin : rand_sweep
        integer ri;
        reg [12:0] r25, r10, rv, rc;
        reg valid_frame;
        integer rand_pass_local, rand_fail_local;
        rand_pass_local=0; rand_fail_local=0;
        for (ri=0; ri<32; ri=ri+1) begin
            r25 = {1'b0, $random(rand_seed)} % 13'd4096;
            r10 = {1'b0, $random(rand_seed)} % 13'd4096;
            rv  = {1'b0, $random(rand_seed)} % 13'd4096;
            rc  = {1'b0, $random(rand_seed)} % 13'd4096;
            if (r25==0) r25=13'd1; if (r10==0) r10=13'd1;
            if (rv==0)  rv=13'd1;  if (rc==0)  rc=13'd1;
            set_all_sensors(r25, r10, rv, rc); run_cycle;
            valid_frame = (uut.proc.aqi_value<=9'd500) &&
                          (uut.proc.category<=3'd5) &&
                          (crc_err===1'b0);
            log_result("Rand frame proc-slave invariants",
                       {51'b0, r25}, {55'b0, uut.proc.aqi_value},
                       64'd500, valid_frame);
            if (!valid_frame) begin
                math_err_count=math_err_count+1;
                rand_fail_local=rand_fail_local+1;
            end else rand_pass_local=rand_pass_local+1;
        end
        $display("  |  [INFO] Randomized: %0d/32 passed", rand_pass_local);
    end
    end_test;

    begin_test("T57 All channels max-neg -> AQI=0");
    set_all_sensors(-13'sh1000, -13'sh1000, -13'sh1000, -13'sh1000); run_silent_cycles(7); run_cycle;
    log_result("input_err=1111",
               64'hF000, {60'b0, input_err}, 64'hF, input_err===4'b1111);
    log_result("proc AQI=0",
               64'd0, {55'b0, uut.proc.aqi_value}, 64'd0, uut.proc.aqi_value===9'd0);
    log_result("proc cat=Good(0)",
               64'd0, {61'b0, uut.proc.category}, 64'd0, uut.proc.category===3'd0);
    end_test;

    begin_test("T58 Trend rising after AQI jump");
    begin : trend_test
        enable_print = 1'b0;
        repeat(8) begin set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1); run_cycle; end
        repeat(3) begin set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle; end
        enable_print = 1'b1;
        begin set_all_sensors(13'sd1639,13'sd96,13'sd50,13'sd1); run_cycle; end
        @(posedge clk); #1;
        log_result("trend_rising=1 after sharp jump",
                   64'd0, {63'b0, trend_rising}, 64'd1, trend_rising===1'b1);
    end
    end_test;

    begin_test("T59 CS Mutex never violated");
    begin : cs_mon
        integer w; reg mutex_ok;
        mutex_ok=1;
        set_all_sensors(13'sd82,13'sd28,13'sd21,13'sd1);
        begin : cs_watch
            integer ci;
            fork
                begin : stim
                    for (ci=0; ci<5; ci=ci+1) run_cycle;
                    disable mon;
                end
                begin : mon
                    integer m;
                    for (m=0; m<50000; m=m+1) begin
                        @(posedge clk);
                        if (({uut.cs4,uut.cs3,uut.cs2,uut.cs1}!=4'b1111 &&
                             {uut.cs4,uut.cs3,uut.cs2,uut.cs1}!=4'b1110 &&
                             {uut.cs4,uut.cs3,uut.cs2,uut.cs1}!=4'b1101 &&
                             {uut.cs4,uut.cs3,uut.cs2,uut.cs1}!=4'b1011 &&
                             {uut.cs4,uut.cs3,uut.cs2,uut.cs1}!=4'b0111)) begin
                            mutex_ok=0; spi_proto_err_count=spi_proto_err_count+1;
                        end
                    end
                end
            join
        end
        log_result("CS mutex not violated",
                   64'd0, {63'b0, mutex_ok}, 64'd1, mutex_ok===1'b1);
    end
    end_test;

    begin_test("T60 proc AQI=500 at saturation");
    begin : sat_test
        enable_print = 1'b0;
        repeat(7) begin
            set_all_sensors(13'sd4095, 13'sd4095, 13'sd4095, 13'sd4095); run_cycle;
        end
        enable_print = 1'b1;
        begin
            set_all_sensors(13'sd4095, 13'sd4095, 13'sd4095, 13'sd4095); run_cycle;
        end
        log_result("proc AQI=500 at saturation",
                   64'd0, {55'b0, uut.proc.aqi_value}, 64'd500,
                   uut.proc.aqi_value===9'd500);
        log_result("proc cat=5 Hazardous",
                   64'd0, {61'b0, uut.proc.category}, 64'd5,
                   uut.proc.category===3'd5);
    end
    end_test;

    // ========================================================================
    //  CONCURRENT PROPERTY MONITORS
    // ========================================================================
    #2000;

    // ========================================================================
    //  FINAL SUMMARY
    // ========================================================================
    $display("");
    $display("  +==================================================================+");
    $display("  |         SIMULATION COMPLETE -- FULL VERIFICATION REPORT          |");
    $display("  +==================================================================+");
    $display("  |  Architecture : Processing-Centric SPI Air-Quality System       |");
    $display("  |  AQI Source   : spi_processing_slave.proc.aqi_value             |");
    $display("  +==================================================================+");

    $display("");
    $display("  +---------------------------------------------------------------+");
    $display("  |  SECTION A  PERFORMANCE                                       |");
    $display("  +---------------------------------------------------------------+");
    $display("  |  Total Simulation Time  : %.0f ns", $realtime);
    $display("  |  Total Clock Cycles     : %0d", global_clk_ctr);
    $display("  |  Avg Cycles / Test      : %0d", global_clk_ctr / (test_num - 1));
    $display("  +---------------------------------------------------------------+");
    $display("  +---------------------------------------------------------------+");
    $display("  |  LATENCY PROFILING (last measured sample)                     |");
    $display("  +---------------------------------------------------------------+");
    $display("  |  A. System Cycle (start->done)                                |");
    $display("  |       Clocks : %-8d  Time : %.0f ns",
             prof_cycle_clk_total, prof_cycle_time_total);
    $display("  |  B. Processing Slave (SEND_PROC->LATCH_DATA)                  |");
    $display("  |       Clocks : %-8d  Time : %.0f ns",
             prof_proc_clk_total, prof_proc_time_total);
    $display("  |  C. Live UART TX (LATCH_DATA->fifo_empty)                     |");
    $display("  |       Clocks : %-8d  Time : %.0f ns",
             prof_uart_clk_total, prof_uart_time_total);
    $display("  |  D. BRAM Dump UART (0x02 cmd->0xDD sent)                      |");
    if (prof_bram_dump_samples > 0)
    $display("  |       Clocks : %-8d  Time : %.0f ns",
             prof_bram_dump_clk_total, prof_bram_dump_time_total);
    else
    $display("  |       (no BRAM dump measured)                                 |");
    $display("  |  E. Flash Write (alarm_rising->flash_done)                    |");
    if (prof_flash_wr_samples > 0)
    $display("  |       Clocks : %-8d  Time : %.0f ns",
             prof_flash_wr_clk_total, prof_flash_wr_time_total);
    else
    $display("  |       (no flash write measured)                               |");
    $display("  +---------------------------------------------------------------+");

    $display("");
    $display("  +---------------------------------------------------------------+");
    $display("  |  SECTION B  FUNCTIONAL ACCURACY                               |");
    $display("  +---------------------------------------------------------------+");
    $display("  |  Total Tests Run       : %0d", test_num - 1);
    $display("  |  Total Assertions PASS : %0d", pass_count);
    $display("  |  Total Assertions FAIL : %0d", fail_count);
    begin : rate_calc
        real rate;
        rate = (pass_count + fail_count > 0) ?
               (100.0 * pass_count / (pass_count + fail_count)) : 0.0;
        $display("  |  Pass Rate             : %0.1f %%", rate);
    end
    $display("  |  Result                : %0s",
             (fail_count==0) ? "*** ALL TESTS PASSED ***" : "!!! FAILURES DETECTED !!!");
    $display("  +---------------------------------------------------------------+");

    $display("");
    $display("  +---------------------------------------------------------------+");
    $display("  |  SECTION C  COMPONENT HEALTH                                  |");
    $display("  +---------------------------------------------------------------+");
    $display("  |  Math Engine (proc slave AQI/cat) : %0s  (%0d errors)",
             (math_err_count==0)      ? "OK     " : "DEGRADED", math_err_count);
    $display("  |  SPI/CRC Protocol                 : %0s  (%0d violations)",
             (spi_proto_err_count==0) ? "OK     " : "DEGRADED", spi_proto_err_count);
    $display("  |  Sensor Reliability               : %0s  (%0d fault/timeout events)",
             (sensor_rel_err_count<=4)? "OK     " : "DEGRADED", sensor_rel_err_count);
    $display("  +---------------------------------------------------------------+");

    $display("");
    $display("  +------+------------------------------------------+------+------+---------+--------+");
    $display("  |                         SECTION D  PER-TEST BREAKDOWN                              |");
    $display("  +------+------------------------------------------+------+------+---------+--------+");
    $display("  | Num  | Test Name                                | Pass | Fail |   Clks  | Result |");
    $display("  +------+------------------------------------------+------+------+---------+--------+");
    begin : summary_loop
        integer t;
        for (t = 1; t < test_num; t = t + 1) begin
            $display("  | T%02d  | %-40s | %4d | %4d | %7d | %s |",
                     t, test_name[t],
                     test_pass[t], test_fail[t],
                     clk_end[t] - clk_start[t],
                     (test_fail[t] == 0) ? "PASS  " : "FAIL  ");
        end
    end
    $display("  +------+------------------------------------------+------+------+---------+--------+");
    $display("  | TOT  | All Tests                                | %4d | %4d | %7d |        |",
             pass_count, fail_count, global_clk_ctr);
    $display("  +------+------------------------------------------+------+------+---------+--------+");

    $finish;
end

// -----------------------------------------------------------------------------
//  CONCURRENT PROPERTY MONITORS
// -----------------------------------------------------------------------------
reg p1_prev_alarm_rising;
always @(posedge clk) begin
    if (!reset) begin
        p1_prev_alarm_rising <= uut.alarm_rising;
        if (p1_prev_alarm_rising && uut.alarm_rising)
            $display("  [PROP-FAIL] P1: alarm_rising held HIGH 2 cycles at %0t", $time);
    end
end

always @(posedge clk) begin
    if (!reset) begin : p4_chk
        reg [3:0] cs_bus;
        cs_bus = {uut.cs4, uut.cs3, uut.cs2, uut.cs1};
        if (cs_bus!=4'b1111 && cs_bus!=4'b1110 && cs_bus!=4'b1101 &&
            cs_bus!=4'b1011 && cs_bus!=4'b0111)
            $display("  [PROP-FAIL] P4: Multiple CS active (%04b) at %0t", cs_bus, $time);
    end
end

reg p7_prev_done;
always @(posedge clk) begin
    if (!reset) begin
        p7_prev_done <= done;
        if (p7_prev_done && done)
            $display("  [PROP-FAIL] P7: done_o held HIGH 2 cycles at %0t", $time);
    end
end

reg p9_bod_prev;
reg p9_rose_d1,p9_rose_d2,p9_rose_d3,p9_rose_d4,p9_rose_d5,p9_rose_d6;
reg p9_flag_d1,p9_flag_d2,p9_flag_d3,p9_flag_d4,p9_flag_d5,p9_flag_d6;
always @(posedge clk) begin
    if (!reset) begin
        p9_bod_prev  <= bod_i_drive;
        p9_rose_d1 <= (bod_i_drive && !p9_bod_prev);
        p9_rose_d2 <= p9_rose_d1; p9_rose_d3 <= p9_rose_d2;
        p9_rose_d4 <= p9_rose_d3; p9_rose_d5 <= p9_rose_d4; p9_rose_d6 <= p9_rose_d5;
        p9_flag_d1 <= bod_flag; p9_flag_d2 <= p9_flag_d1; p9_flag_d3 <= p9_flag_d2;
        p9_flag_d4 <= p9_flag_d3; p9_flag_d5 <= p9_flag_d4; p9_flag_d6 <= p9_flag_d5;
        if (p9_rose_d6 && !(p9_flag_d2||p9_flag_d3||p9_flag_d4||p9_flag_d5||p9_flag_d6))
            $display("  [PROP-FAIL] P9: bod_flag not asserted within cycles at %0t", $time);
    end
end

// AQI range check from proc slave
always @(posedge clk) begin
    if (!reset) begin
        if (uut.proc.aqi_value > 9'd500)
            $display("  [PROP-FAIL] AQI_RANGE: proc.aqi_value=%0d > 500 at %0t",
                     uut.proc.aqi_value, $time);
        if (uut.proc.category > 3'd5)
            $display("  [PROP-FAIL] CAT_RANGE: proc.category=%0d > 5 at %0t",
                     uut.proc.category, $time);
    end
end

endmodule
