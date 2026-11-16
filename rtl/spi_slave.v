/*****************************************************************************
 Module: spi_slave
 Purpose: External SPI register-map slave for host MCU access.

 Description:
   Provides read/write access to 9 internal registers (PM2.5, PM10,
   VOC, CO, AQI, category, status, faults, trend). Uses 2-FF
   synchronizers on all external SPI signals (sclk_s, cs_s, mosi_s)
   and 2-stage CDC flip-flops on all register inputs. Protocol: 8-bit
   address byte (bit[7]=1 for read, 0 for write), then 8 or 16-bit
   data phase depending on register width.

 Register map (read):
   0x00 PM2.5 (16-bit)    0x05 category (8-bit)
   0x01 PM10  (16-bit)    0x06 status   (8-bit)
   0x02 VOC   (16-bit)    0x07 faults   (8-bit)
   0x03 CO    (16-bit)    0x08 trend    (8-bit)
   0x04 AQI   (16-bit)

 Instantiated in spi_top.

 Inputs:
   clk          - system clock
   rst          - active-high async reset
   sclk_s       - SPI clock from host
   cs_s         - SPI chip-select from host (active-low)
   mosi_s       - SPI data in from host
   reg_pm25     - 16-bit PM2.5 register value
   reg_pm10     - 16-bit PM10 register value
   reg_voc      - 16-bit VOC register value
   reg_co       - 16-bit CO register value
   reg_aqi      - 16-bit AQI register value
   reg_category - 8-bit category code
   reg_status   - 8-bit status flags
   reg_faults   - 8-bit fault flags
   reg_trend    - 8-bit trend indicator

 Outputs:
   miso_s    - SPI data out to host
   wr_valid_o - write strobe (one clk pulse)
   wr_addr_o  - 7-bit write address
   wr_data_o  - 8-bit write data
*****************************************************************************/

module spi_slave (
    input  wire        clk,
    input  wire        rst,

    input  wire        sclk_s,
    input  wire        cs_s,
    input  wire        mosi_s,
    output wire        miso_s,

    input  wire [15:0] reg_pm25,
    input  wire [15:0] reg_pm10,
    input  wire [15:0] reg_voc,
    input  wire [15:0] reg_co,
    input  wire [15:0] reg_aqi,
    input  wire [7:0]  reg_category,
    input  wire [7:0]  reg_status,
    input  wire [7:0]  reg_faults,
    input  wire [7:0]  reg_trend,

    output reg         wr_valid_o,
    output reg [6:0]   wr_addr_o,
    output reg [7:0]   wr_data_o
);

// 2-FF synchronizer for SCLK
reg sclk_s1, sclk_s2;

always @(posedge clk or posedge rst) begin
    if (rst) begin sclk_s1 <= 1'b0; sclk_s2 <= 1'b0; end
    else     begin sclk_s1 <= sclk_s; sclk_s2 <= sclk_s1; end
end
wire sclk_rise = (sclk_s1 == 1'b1) && (sclk_s2 == 1'b0);
wire sclk_fall = (sclk_s1 == 1'b0) && (sclk_s2 == 1'b1);

// 2-FF synchronizer for CS
reg cs_s1, cs_s2;

always @(posedge clk or posedge rst) begin
    if (rst) begin cs_s1 <= 1'b1; cs_s2 <= 1'b1; end
    else     begin cs_s1 <= cs_s; cs_s2 <= cs_s1; end
end
wire cs_active = ~cs_s2;

// 2-FF synchronizer for MOSI
reg mosi_s1, mosi_s2;

always @(posedge clk or posedge rst) begin
    if (rst) begin mosi_s1 <= 1'b0; mosi_s2 <= 1'b0; end
    else     begin mosi_s1 <= mosi_s; mosi_s2 <= mosi_s1; end
end

// 2-stage CDC flip-flops for all register inputs
reg [15:0] r_pm25_ff1,   r_pm25_ff2;
reg [15:0] r_pm10_ff1,   r_pm10_ff2;
reg [15:0] r_voc_ff1,    r_voc_ff2;
reg [15:0] r_co_ff1,     r_co_ff2;
reg [15:0] r_aqi_ff1,    r_aqi_ff2;
reg [7:0]  r_cat_ff1,    r_cat_ff2;
reg [7:0]  r_stat_ff1,   r_stat_ff2;
reg [7:0]  r_fault_ff1,  r_fault_ff2;
reg [7:0]  r_trend_ff1,  r_trend_ff2;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        r_pm25_ff1  <= 16'd0; r_pm25_ff2  <= 16'd0;
        r_pm10_ff1  <= 16'd0; r_pm10_ff2  <= 16'd0;
        r_voc_ff1   <= 16'd0; r_voc_ff2   <= 16'd0;
        r_co_ff1    <= 16'd0; r_co_ff2    <= 16'd0;
        r_aqi_ff1   <= 16'd0; r_aqi_ff2   <= 16'd0;
        r_cat_ff1   <= 8'd0;  r_cat_ff2   <= 8'd0;
        r_stat_ff1  <= 8'd0;  r_stat_ff2  <= 8'd0;
        r_fault_ff1 <= 8'd0;  r_fault_ff2 <= 8'd0;
        r_trend_ff1 <= 8'd0;  r_trend_ff2 <= 8'd0;
    end else begin

        r_pm25_ff1  <= reg_pm25;    r_pm10_ff1  <= reg_pm10;
        r_voc_ff1   <= reg_voc;     r_co_ff1    <= reg_co;
        r_aqi_ff1   <= reg_aqi;     r_cat_ff1   <= reg_category;
        r_stat_ff1  <= reg_status;  r_fault_ff1 <= reg_faults;
        r_trend_ff1 <= reg_trend;

        r_pm25_ff2  <= r_pm25_ff1;    r_pm10_ff2  <= r_pm10_ff1;
        r_voc_ff2   <= r_voc_ff1;     r_co_ff2    <= r_co_ff1;
        r_aqi_ff2   <= r_aqi_ff1;     r_cat_ff2   <= r_cat_ff1;
        r_stat_ff2  <= r_stat_ff1;    r_fault_ff2 <= r_fault_ff1;
        r_trend_ff2 <= r_trend_ff1;
    end
end

reg [7:0]  addr_reg;
reg [15:0] tx_shift;
reg [4:0]  tx_bits;
reg [4:0]  tx_cnt;
reg [3:0]  rx_cnt;
reg        tx_active;
reg [15:0] selected_reg;

reg        is_write;
reg        rx_data_phase;
reg [7:0]  rx_data_shift;
reg [3:0]  rx_data_cnt;
wire [7:0] full_addr_byte = {addr_reg[6:0], mosi_s2};

assign miso_s = tx_active ? tx_shift[15] : 1'b0;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        addr_reg      <= 8'd0;
        tx_shift      <= 16'd0;
        tx_bits       <= 5'd0;
        tx_cnt        <= 5'd0;
        rx_cnt        <= 4'd0;
        tx_active     <= 1'b0;
        selected_reg  <= 16'd0;
        is_write      <= 1'b0;
        rx_data_phase <= 1'b0;
        rx_data_shift <= 8'd0;
        rx_data_cnt   <= 4'd0;
        wr_valid_o    <= 1'b0;
        wr_addr_o     <= 7'd0;
        wr_data_o     <= 8'd0;
    end else if (!cs_active) begin

        addr_reg      <= 8'd0;
        tx_shift      <= 16'd0;
        tx_bits       <= 5'd0;
        tx_cnt        <= 5'd0;
        rx_cnt        <= 4'd0;
        tx_active     <= 1'b0;
        selected_reg  <= 16'd0;
        is_write      <= 1'b0;
        rx_data_phase <= 1'b0;
        rx_data_shift <= 8'd0;
        rx_data_cnt   <= 4'd0;
        wr_valid_o    <= 1'b0;
    end else begin

        wr_valid_o <= 1'b0;

        if (sclk_rise) begin
            if (rx_data_phase) begin

                rx_data_shift <= {rx_data_shift[6:0], mosi_s2};
                rx_data_cnt   <= rx_data_cnt + 1'b1;
                if (rx_data_cnt == 4'd7) begin

                    wr_valid_o    <= 1'b1;
                    wr_addr_o     <= addr_reg[6:0];
                    wr_data_o     <= {rx_data_shift[6:0], mosi_s2};
                    rx_data_phase <= 1'b0;
                end
            end else if (!tx_active) begin

                addr_reg  <= {addr_reg[6:0], mosi_s2};
                rx_cnt    <= rx_cnt + 1'b1;
                if (rx_cnt == 4'd7) begin

                    is_write <= ~full_addr_byte[7];
                    addr_reg <= full_addr_byte;
                    if (full_addr_byte[7]) begin
                        // 8-bit regs left-justify into tx_shift[15:8]
                        case (full_addr_byte[6:0])
                            7'h00: begin selected_reg <= r_pm25_ff2;          tx_shift <= r_pm25_ff2;             tx_bits <= 5'd16; end
                            7'h01: begin selected_reg <= r_pm10_ff2;          tx_shift <= r_pm10_ff2;             tx_bits <= 5'd16; end
                            7'h02: begin selected_reg <= r_voc_ff2;           tx_shift <= r_voc_ff2;              tx_bits <= 5'd16; end
                            7'h03: begin selected_reg <= r_co_ff2;            tx_shift <= r_co_ff2;               tx_bits <= 5'd16; end
                            7'h04: begin selected_reg <= r_aqi_ff2;           tx_shift <= r_aqi_ff2;              tx_bits <= 5'd16; end
                            7'h05: begin selected_reg <= {8'd0, r_cat_ff2};   tx_shift <= {r_cat_ff2, 8'b0};      tx_bits <= 5'd8;  end
                            7'h06: begin selected_reg <= {8'd0, r_stat_ff2};  tx_shift <= {r_stat_ff2, 8'b0};     tx_bits <= 5'd8;  end
                            7'h07: begin selected_reg <= {8'd0, r_fault_ff2}; tx_shift <= {r_fault_ff2, 8'b0};    tx_bits <= 5'd8;  end
                            7'h08: begin selected_reg <= {8'd0, r_trend_ff2}; tx_shift <= {r_trend_ff2, 8'b0};    tx_bits <= 5'd8;  end
                            default: begin selected_reg <= 16'd0;              tx_shift <= 16'd0;                  tx_bits <= 5'd8;  end
                        endcase
                        tx_cnt    <= 5'd0;
                        tx_active <= 1'b1;
                    end else begin

                        rx_data_phase <= 1'b1;
                        rx_data_shift <= 8'd0;
                        rx_data_cnt   <= 4'd0;
                        tx_active     <= 1'b0;
                    end
                end
            end
        end else if (sclk_fall && tx_active) begin

            tx_shift <= {tx_shift[14:0], 1'b0};
            tx_cnt   <= tx_cnt + 1'b1;
            if (tx_cnt == (tx_bits - 1'b1))
                tx_active <= 1'b0;
        end
    end
end

endmodule
