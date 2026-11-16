/*****************************************************************************
 Module: spi_flash_model (SIMULATION ONLY)
 Purpose: Behavioral SPI flash device model for verification.

 Description:
   Simulation-only behavioral model of an SPI flash device with byte-
   addressable memory array. Decodes command bytes matching the
   spi_flash_ctrl opcodes (WREN=0x0C, PP=0x04, READ=0x06, SE=0x40),
   supports write-enable latch, sector erase, page program, and read
   operations. Includes a corrupt_readback flag for verify-failure testing.

   NOT synthesizable — uses initial blocks, posedge cs triggers, and
   behavioral memory operations.
*****************************************************************************/


module spi_flash_model #(
    parameter DEPTH = 4096
)(
    input  wire       sclk,
    input  wire       cs,
    input  wire       mosi,
    output reg        miso
);

reg [7:0] flash_mem [0:DEPTH-1];

integer init_i;
initial begin
    for (init_i = 0; init_i < DEPTH; init_i = init_i + 1)
        flash_mem[init_i] = 8'hFF;
    miso = 1'b0;
end

reg [7:0]  shift_reg;
reg [2:0]  bit_cnt;
reg [7:0]  cmd;
reg [23:0] addr;
reg [7:0]  wr_byte;
reg [7:0]  rd_byte;
reg        wel;
reg        busy;

localparam PH_CMD      = 3'd0;
localparam PH_ADDR2    = 3'd1;
localparam PH_ADDR1    = 3'd2;
localparam PH_ADDR0    = 3'd3;
localparam PH_DATA_WR  = 3'd4;
localparam PH_DATA_RD  = 3'd5;
localparam PH_STATUS   = 3'd6;
localparam PH_SE_WAIT  = 3'd7;

reg [2:0]  phase;
reg [11:0] byte_addr;

reg corrupt_readback;
initial corrupt_readback = 1'b0;

integer se_i;

// Controller opcodes: WREN=0x0C, PP=0x04, READ=0x06, SE=0x40
always @(posedge cs) begin

    if (phase == PH_CMD && cmd == 8'h0C && bit_cnt == 3'd0)
        wel <= 1'b1;

    if (phase == PH_DATA_WR)
        wel <= 1'b0;

    if (phase == PH_SE_WAIT && wel) begin : do_erase
        integer ei;
        reg [23:0] sector_base;
        sector_base = {addr[23:12], 12'b0};
        for (ei = 0; ei < 4096; ei = ei + 1) begin
            if ((sector_base + ei) < DEPTH)
                flash_mem[sector_base + ei] = 8'hFF;
        end
        wel <= 1'b0;
    end
    phase   <= PH_CMD;
    bit_cnt <= 3'd0;
    shift_reg <= 8'h00;
    miso    <= 1'b0;
end

always @(posedge sclk) begin
    if (!cs) begin
        shift_reg <= {shift_reg[6:0], mosi};
        bit_cnt   <= bit_cnt + 1;

        if (bit_cnt == 3'd7) begin

            case (phase)

            PH_CMD: begin
                cmd   <= {shift_reg[6:0], mosi};
                phase <= PH_ADDR2;

                if ({shift_reg[6:0], mosi} == 8'h05)
                    phase <= PH_STATUS;
                else if ({shift_reg[6:0], mosi} == 8'h0C)
                    phase <= PH_CMD;
                else if ({shift_reg[6:0], mosi} == 8'h40)
                    phase <= PH_ADDR2;
            end

            PH_ADDR2: begin
                addr[23:16] <= {shift_reg[6:0], mosi};
                phase       <= PH_ADDR1;
            end

            PH_ADDR1: begin
                addr[15:8] <= {shift_reg[6:0], mosi};
                phase      <= PH_ADDR0;
            end

            PH_ADDR0: begin
                addr[7:0] <= {shift_reg[6:0], mosi};
                byte_addr <= {addr[11:8], shift_reg[6:0], mosi};
                if (cmd == 8'h04 && wel)
                    phase <= PH_DATA_WR;
                else if (cmd == 8'h06)
                    phase <= PH_DATA_RD;
                else if (cmd == 8'h40 && wel)
                    phase <= PH_SE_WAIT;
                else
                    phase <= PH_CMD;
            end

            PH_DATA_WR: begin

                flash_mem[byte_addr] <= {shift_reg[6:0], mosi};
                byte_addr <= byte_addr + 1;

            end

            default: ;
            endcase

            bit_cnt <= 3'd0;
        end
    end
end

always @(negedge sclk) begin
    if (!cs) begin

        case (phase)

        PH_DATA_RD: begin
            if (bit_cnt == 3'd0) begin

                rd_byte <= flash_mem[byte_addr];
                miso <= flash_mem[byte_addr][7] ^ corrupt_readback;
            end else begin
                miso <= rd_byte[7 - bit_cnt] ^ corrupt_readback;
            end
            if (bit_cnt == 3'd7)
                byte_addr <= byte_addr + 1;
        end

        PH_STATUS: begin

            begin : status_sel
                reg [7:0] status_byte;
                status_byte = {6'b0, wel, busy};
                miso <= status_byte[7 - bit_cnt];
            end
        end

        default: miso <= 1'b0;
        endcase
    end
end

endmodule
// synthesis translate_on
