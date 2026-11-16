/*****************************************************************************
 Module: spi_flash_ctrl
 Purpose: SPI flash command controller for persistent alarm logging.

 Description:
   Sequences WREN -> Sector-Erase -> WREN -> Page-Program -> Verify-Read
   operations with up to 3 retries on verify failure. Includes a 2-FF
   BOD synchronizer with safe abort to S_BOD_WAIT state. Generates its
   own divided SCLK from the system clock.

 Command opcodes: WREN=0x0C, PP=0x04, READ=0x06, SE=0x40

 Instantiated in spi_top.

 Inputs:
   clk          - system clock
   rst          - active-high async reset
   write_req_i  - write request strobe
   write_data_i - 64-bit data to program
   read_req_i   - read request strobe
   read_addr_i  - 12-bit flash read address
   bod_i        - brown-out detect input (async)
   flash_miso   - SPI data in from flash

 Outputs:
   read_data_o       - 64-bit read data
   busy_o            - high while operation in progress
   done_o            - single-cycle completion strobe
   flash_wr_addr_o   - current 12-bit write address pointer
   flash_write_err_o - high if verify failed after all retries
   bod_flag_o        - latched BOD event indicator
   flash_sclk        - SPI clock to flash
   flash_cs          - SPI chip-select to flash (active-low)
   flash_mosi        - SPI data out to flash
*****************************************************************************/

module spi_flash_ctrl (
    input  wire        clk,
    input  wire        rst,
    input  wire        write_req_i,
    input  wire [63:0] write_data_i,
    input  wire        read_req_i,
    input  wire [11:0] read_addr_i,
    output reg  [63:0] read_data_o,
    output reg         busy_o,
    output reg         done_o,
    output wire [11:0] flash_wr_addr_o,
    output reg         flash_write_err_o,
    input  wire        bod_i,
    output reg         bod_flag_o,
    output reg         flash_sclk,
    output reg         flash_cs,
    output reg         flash_mosi,
    input  wire        flash_miso
);
reg [11:0] flash_wr_addr;
assign flash_wr_addr_o = flash_wr_addr;

localparam MAX_RETRY = 2'd3;


reg [2:0]  clk_div;
reg        sclk_en;
reg        sclk_q;
reg        bod_s1, bod_s2;
wire       bod_sync = bod_s2;

// Divided SCLK generator (div-by-4 when enabled)
always @(posedge clk or posedge rst) begin
    if (rst) begin
        clk_div    <= 3'd0;
        flash_sclk <= 1'b0;
        sclk_q     <= 1'b0;
    end else if (sclk_en) begin
        sclk_q <= flash_sclk;
        if (clk_div == 3'd1) begin
            flash_sclk <= ~flash_sclk;
            clk_div    <= 3'd0;
        end else begin
            clk_div <= clk_div + 1;
        end
    end else begin
        clk_div    <= 3'd0;
        flash_sclk <= 1'b0;
        sclk_q     <= 1'b0;
    end
end

wire sclk_rise_f = ( flash_sclk & ~sclk_q);
wire sclk_fall_f = (~flash_sclk &  sclk_q);

// FSM state encoding
localparam S_IDLE           = 6'd0;
localparam S_WEN_CS_LOW     = 6'd1;
localparam S_WEN_SEND       = 6'd2;
localparam S_WEN_CS_HIGH    = 6'd3;
localparam S_WEN_GAP        = 6'd4;
localparam S_PP_CS_LOW      = 6'd5;
localparam S_PP_CMD         = 6'd6;
localparam S_PP_ADDR2       = 6'd7;
localparam S_PP_ADDR1       = 6'd8;
localparam S_PP_ADDR0       = 6'd9;
localparam S_PP_DATA        = 6'd10;
localparam S_PP_CS_HIGH     = 6'd11;
localparam S_VFY_RD_REQ     = 6'd12;
localparam S_VFY_CS_LOW     = 6'd13;
localparam S_VFY_CMD        = 6'd14;
localparam S_VFY_ADDR2      = 6'd15;
localparam S_VFY_ADDR1      = 6'd16;
localparam S_VFY_ADDR0      = 6'd17;
localparam S_VFY_DATA       = 6'd18;
localparam S_VFY_CS_HIGH    = 6'd19;
localparam S_VFY_CHECK      = 6'd20;
localparam S_WR_DONE        = 6'd21;
localparam S_WR_ERR         = 6'd22;
localparam S_RD_CS_LOW      = 6'd23;
localparam S_RD_CMD         = 6'd24;
localparam S_RD_ADDR2       = 6'd25;
localparam S_RD_ADDR1       = 6'd26;
localparam S_RD_ADDR0       = 6'd27;
localparam S_RD_DATA        = 6'd28;
localparam S_RD_CS_HIGH     = 6'd29;
localparam S_RD_DONE        = 6'd30;
localparam S_BOD_WAIT       = 6'd31;
localparam S_SE_WEN_CS_LOW  = 6'd40;
localparam S_SE_WEN_SEND    = 6'd41;
localparam S_SE_WEN_CS_HIGH = 6'd42;
localparam S_SE_WEN_GAP     = 6'd43;
localparam S_SE_CS_LOW      = 6'd44;
localparam S_SE_CMD         = 6'd45;
localparam S_SE_ADDR2       = 6'd46;
localparam S_SE_ADDR1       = 6'd47;
localparam S_SE_ADDR0       = 6'd48;
localparam S_SE_CS_HIGH     = 6'd49;
localparam S_SE_WAIT        = 6'd50;

localparam [4:0] SE_WAIT_CYCLES = 5'd15;
reg [4:0] se_wait_cnt;

reg [5:0] state;
reg [7:0]  tx_byte;
reg [2:0]  bit_cnt, byte_cnt;
reg [63:0] data_buf, orig_data_buf, verify_buf;
reg [11:0] addr_buf;
reg        is_write;
reg [1:0]  retry_cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state             <= S_IDLE;
        flash_cs          <= 1'b1;
        flash_mosi        <= 1'b0;
        sclk_en           <= 1'b0;
        busy_o            <= 1'b0;
        done_o            <= 1'b0;
        flash_write_err_o <= 1'b0;
        bod_flag_o        <= 1'b0;
        flash_wr_addr     <= 12'd0;
        read_data_o       <= 64'd0;
        tx_byte           <= 8'd0;
        bit_cnt           <= 3'd0;
        byte_cnt          <= 3'd0;
        data_buf          <= 64'd0;
        orig_data_buf     <= 64'd0;
        verify_buf        <= 64'd0;
        addr_buf          <= 12'd0;
        is_write          <= 1'b0;
        retry_cnt         <= 2'd0;
        se_wait_cnt       <= 5'd0;
        bod_s1            <= 1'b0;
        bod_s2            <= 1'b0;
    end else begin
        // 2-FF BOD synchronizer
        bod_s1 <= bod_i;
        bod_s2 <= bod_s1;
        done_o <= 1'b0;

        // BOD safe-abort: deassert flash lines and wait for recovery
        if (bod_sync && state != S_IDLE && state != S_BOD_WAIT) begin
            flash_cs   <= 1'b1;
            sclk_en    <= 1'b0;
            flash_mosi <= 1'b0;
            busy_o     <= 1'b1;
            bod_flag_o <= 1'b1;
            state      <= S_BOD_WAIT;
        end else begin

        case (state)

        S_IDLE: begin
            flash_cs   <= 1'b1;
            sclk_en    <= 1'b0;
            busy_o     <= 1'b0;
            if (write_req_i && !bod_sync) begin
                flash_write_err_o <= 1'b0;
                data_buf      <= write_data_i;
                orig_data_buf <= write_data_i;
                addr_buf      <= flash_wr_addr;
                is_write      <= 1'b1;
                busy_o        <= 1'b1;
                retry_cnt     <= 2'd0;
                state         <= S_SE_WEN_CS_LOW;
            end else if (read_req_i && !bod_sync) begin
                addr_buf  <= read_addr_i;
                is_write  <= 1'b0;
                busy_o    <= 1'b1;
                state     <= S_RD_CS_LOW;
            end
        end

        // --- WREN before Page-Program ---
        S_WEN_CS_LOW: begin
            flash_cs   <= 1'b0;
            tx_byte    <= 8'h0C;
            flash_mosi <= 1'b0;
            bit_cnt    <= 3'd1;
            sclk_en    <= 1'b1;
            state      <= S_WEN_SEND;
        end
        S_WEN_SEND: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    sclk_en <= 1'b0;
                    state   <= S_WEN_CS_HIGH;
                end
            end
        end
        S_WEN_CS_HIGH: begin flash_cs <= 1'b1; state <= S_WEN_GAP; end
        S_WEN_GAP: begin state <= S_PP_CS_LOW; end

        // --- Page-Program: cmd + 3-byte addr + 8 data bytes ---
        S_PP_CS_LOW: begin
            flash_cs   <= 1'b0;
            tx_byte    <= 8'h04;
            flash_mosi <= 1'b0;
            bit_cnt    <= 3'd1;
            sclk_en    <= 1'b1;
            state      <= S_PP_CMD;
        end
        S_PP_CMD: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    tx_byte    <= 8'h00;
                    bit_cnt    <= 3'd0;
                    state      <= S_PP_ADDR2;
                end
            end
        end
        S_PP_ADDR2: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    tx_byte    <= {4'b0, addr_buf[11:8]};
                    bit_cnt    <= 3'd0;
                    state      <= S_PP_ADDR1;
                end
            end
        end
        S_PP_ADDR1: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    tx_byte    <= {addr_buf[7:0]};
                    bit_cnt    <= 3'd0;
                    state      <= S_PP_ADDR0;
                end
            end
        end
        S_PP_ADDR0: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    tx_byte    <= data_buf[63:56];
                    bit_cnt    <= 3'd0;
                    byte_cnt   <= 3'd0;
                    state      <= S_PP_DATA;
                end
            end
        end
        S_PP_DATA: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    byte_cnt <= byte_cnt + 1;
                    if (byte_cnt == 3'd7) begin
                        sclk_en <= 1'b0;
                        state   <= S_PP_CS_HIGH;
                    end else begin
                        data_buf   <= {data_buf[55:0], 8'h00};
                        tx_byte    <= data_buf[55:48];
                        bit_cnt    <= 3'd0;
                    end
                end
            end
        end
        S_PP_CS_HIGH: begin flash_cs <= 1'b1; state <= S_VFY_RD_REQ; end

        // --- Verify-Read: read back and compare ---
        S_VFY_RD_REQ: begin verify_buf <= 64'd0; state <= S_VFY_CS_LOW; end
        S_VFY_CS_LOW: begin
            flash_cs   <= 1'b0;
            tx_byte    <= 8'h06;
            flash_mosi <= 1'b0;
            bit_cnt    <= 3'd1;
            sclk_en    <= 1'b1;
            state      <= S_VFY_CMD;
        end
        S_VFY_CMD: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    tx_byte    <= 8'h00;
                    bit_cnt    <= 3'd0;
                    state      <= S_VFY_ADDR2;
                end
            end
        end
        S_VFY_ADDR2: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    tx_byte    <= {4'b0, addr_buf[11:8]};
                    bit_cnt    <= 3'd0;
                    state      <= S_VFY_ADDR1;
                end
            end
        end
        S_VFY_ADDR1: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    tx_byte    <= addr_buf[7:0];
                    bit_cnt    <= 3'd0;
                    state      <= S_VFY_ADDR0;
                end
            end
        end
        S_VFY_ADDR0: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    flash_mosi <= 1'b0;
                    bit_cnt    <= 3'd0;
                    byte_cnt   <= 3'd0;
                    state      <= S_VFY_DATA;
                end
            end
        end
        S_VFY_DATA: begin
            if (sclk_rise_f) begin
                verify_buf <= {verify_buf[62:0], flash_miso};
                bit_cnt    <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    byte_cnt   <= byte_cnt + 1;
                    bit_cnt    <= 3'd0;
                    if (byte_cnt == 3'd7) begin
                        sclk_en <= 1'b0;
                        state   <= S_VFY_CS_HIGH;
                    end
                end
            end
        end
        S_VFY_CS_HIGH: begin flash_cs <= 1'b1; state <= S_VFY_CHECK; end

        S_VFY_CHECK: begin
            if (verify_buf == orig_data_buf) begin
                flash_wr_addr     <= flash_wr_addr + 12'd8;
                flash_write_err_o <= 1'b0;
                state             <= S_WR_DONE;
            end else begin
                state <= S_WR_ERR;
            end
        end
        S_WR_ERR: begin
            // Retry from sector-erase if attempts remain
            if (retry_cnt < MAX_RETRY) begin
                retry_cnt  <= retry_cnt + 2'd1;
                data_buf   <= orig_data_buf;
                state      <= S_SE_WEN_CS_LOW;
            end else begin
                flash_write_err_o <= 1'b1;
                flash_wr_addr     <= flash_wr_addr + 12'd8;
                state             <= S_WR_DONE;
            end
        end
        S_WR_DONE: begin done_o <= 1'b1; busy_o <= 1'b0; state <= S_IDLE; end

        // --- Standalone Read ---
        S_RD_CS_LOW: begin
            flash_cs   <= 1'b0;
            tx_byte    <= 8'h06;
            flash_mosi <= 1'b0;
            bit_cnt    <= 3'd1;
            sclk_en    <= 1'b1;
            state      <= S_RD_CMD;
        end
        S_RD_CMD: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    tx_byte    <= 8'h00;
                    bit_cnt    <= 3'd0;
                    state      <= S_RD_ADDR2;
                end
            end
        end
        S_RD_ADDR2: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    tx_byte    <= {4'b0, addr_buf[11:8]};
                    bit_cnt    <= 3'd0;
                    state      <= S_RD_ADDR1;
                end
            end
        end
        S_RD_ADDR1: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    tx_byte    <= addr_buf[7:0];
                    bit_cnt    <= 3'd0;
                    state      <= S_RD_ADDR0;
                end
            end
        end
        S_RD_ADDR0: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    flash_mosi <= 1'b0;
                    bit_cnt    <= 3'd0;
                    byte_cnt   <= 3'd0;
                    data_buf   <= 64'd0;
                    state      <= S_RD_DATA;
                end
            end
        end
        S_RD_DATA: begin
            if (sclk_rise_f) begin
                data_buf <= {data_buf[62:0], flash_miso};
                bit_cnt  <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    byte_cnt <= byte_cnt + 1;
                    bit_cnt  <= 3'd0;
                    if (byte_cnt == 3'd7) begin
                        sclk_en <= 1'b0;
                        state   <= S_RD_CS_HIGH;
                    end
                end
            end
        end
        S_RD_CS_HIGH: begin flash_cs <= 1'b1; read_data_o <= data_buf; state <= S_RD_DONE; end
        S_RD_DONE: begin done_o <= 1'b1; busy_o <= 1'b0; state <= S_IDLE; end

        // --- Sector-Erase sequence: WREN + SE cmd + 3-byte addr ---
        S_SE_WEN_CS_LOW: begin
            flash_cs   <= 1'b0;
            tx_byte    <= 8'h0C;
            flash_mosi <= 1'b0;
            bit_cnt    <= 3'd1;
            sclk_en    <= 1'b1;
            state      <= S_SE_WEN_SEND;
        end
        S_SE_WEN_SEND: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    sclk_en <= 1'b0;
                    state   <= S_SE_WEN_CS_HIGH;
                end
            end
        end
        S_SE_WEN_CS_HIGH: begin flash_cs <= 1'b1; state <= S_SE_WEN_GAP; end
        S_SE_WEN_GAP: begin state <= S_SE_CS_LOW; end
        S_SE_CS_LOW: begin
            flash_cs   <= 1'b0;
            tx_byte    <= 8'h40;
            flash_mosi <= 1'b0;
            bit_cnt    <= 3'd1;
            sclk_en    <= 1'b1;
            state      <= S_SE_CMD;
        end
        S_SE_CMD: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    tx_byte    <= 8'h00;
                    bit_cnt    <= 3'd0;
                    state      <= S_SE_ADDR2;
                end
            end
        end
        S_SE_ADDR2: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    tx_byte    <= {4'b0, addr_buf[11:8]};
                    bit_cnt    <= 3'd0;
                    state      <= S_SE_ADDR1;
                end
            end
        end
        S_SE_ADDR1: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    tx_byte    <= addr_buf[7:0];
                    bit_cnt    <= 3'd0;
                    state      <= S_SE_ADDR0;
                end
            end
        end
        S_SE_ADDR0: begin
            if (sclk_fall_f) begin
                flash_mosi <= tx_byte[7];
                tx_byte    <= {tx_byte[6:0], 1'b0};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == 3'd7) begin
                    sclk_en <= 1'b0;
                    state   <= S_SE_CS_HIGH;
                end
            end
        end
        S_SE_CS_HIGH: begin flash_cs <= 1'b1; se_wait_cnt <= 5'd0; state <= S_SE_WAIT; end
        S_SE_WAIT: begin if (se_wait_cnt == SE_WAIT_CYCLES) begin state <= S_WEN_CS_LOW; end else begin se_wait_cnt <= se_wait_cnt + 1'b1; end end

        S_BOD_WAIT: begin flash_cs <= 1'b1; sclk_en <= 1'b0; flash_mosi <= 1'b0; busy_o <= 1'b1; if (!bod_sync) begin busy_o <= 1'b0; state <= S_IDLE; end end
        default: state <= S_IDLE;
        endcase
        end
    end
end
endmodule
