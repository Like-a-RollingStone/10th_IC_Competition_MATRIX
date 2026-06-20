module matmul_axi_slave (
    input              clk,
    input              resetn,

    input              s_awvalid,
    output reg         s_awready,
    input      [31:0]  s_awaddr,
    input      [4:0]   s_awid,
    input      [7:0]   s_awlen,
    input      [2:0]   s_awsize,
    input      [1:0]   s_awburst,
    input              s_awlock,
    input      [3:0]   s_awcache,
    input      [2:0]   s_awprot,

    input              s_wvalid,
    output reg         s_wready,
    input      [31:0]  s_wdata,
    input      [3:0]   s_wstrb,
    input              s_wlast,

    output reg         s_bvalid,
    input              s_bready,
    output reg [4:0]   s_bid,
    output reg [1:0]   s_bresp,

    output     [3:0]   m_arid,
    output reg [31:0]  m_araddr,
    output     [7:0]   m_arlen,
    output     [2:0]   m_arsize,
    output     [1:0]   m_arburst,
    output             m_arlock,
    output     [3:0]   m_arcache,
    output     [2:0]   m_arprot,
    output reg         m_arvalid,
    input              m_arready,
    input      [3:0]   m_rid,
    input      [31:0]  m_rdata,
    input      [1:0]   m_rresp,
    input              m_rlast,
    input              m_rvalid,
    output             m_rready,
    output     [3:0]   m_awid,
    output reg [31:0]  m_awaddr,
    output     [7:0]   m_awlen,
    output     [2:0]   m_awsize,
    output     [1:0]   m_awburst,
    output             m_awlock,
    output     [3:0]   m_awcache,
    output     [2:0]   m_awprot,
    output reg         m_awvalid,
    input              m_awready,
    output reg [31:0]  m_wdata,
    output     [3:0]   m_wstrb,
    output reg         m_wlast,
    output reg         m_wvalid,
    input              m_wready,
    input      [3:0]   m_bid,
    input      [1:0]   m_bresp,
    input              m_bvalid,
    output             m_bready,

    input              s_arvalid,
    output reg         s_arready,
    input      [31:0]  s_araddr,
    input      [4:0]   s_arid,
    input      [7:0]   s_arlen,
    input      [2:0]   s_arsize,
    input      [1:0]   s_arburst,
    input              s_arlock,
    input      [3:0]   s_arcache,
    input      [2:0]   s_arprot,

    output reg         s_rvalid,
    input              s_rready,
    output reg [31:0]  s_rdata,
    output reg [4:0]   s_rid,
    output reg [1:0]   s_rresp,
    output reg         s_rlast,

    output             direct_ext_active,
    output reg [19:0]  direct_ext_addr,
    output reg [3:0]   direct_ext_be_n,
    output             direct_ext_ce_n,
    output             direct_ext_oe_n,
    output             direct_ext_we_n,
    output reg [31:0]  direct_ext_wdata,
    input      [31:0]  direct_ext_rdata
);

localparam [11:0] CTRL_ADDR    = 12'h000;
localparam [11:0] STATUS_ADDR  = 12'h004;
localparam [11:0] VERSION_ADDR = 12'h008;
localparam [11:0] A_BASE_ADDR  = 12'h010;
localparam [11:0] B_BASE_ADDR  = 12'h050;
localparam [11:0] C_BASE_ADDR  = 12'h090;
localparam [11:0] SRC_BASE_ADDR   = 12'h150;
localparam [11:0] DST_BASE_ADDR   = 12'h154;
localparam [11:0] GROUP_COUNT_ADDR = 12'h158;
localparam [11:0] CRC32_ADDR      = 12'h15c;

localparam CTRL_START_BIT    = 0;
localparam CTRL_SOFT_RST_BIT = 1;

localparam STATUS_BUSY_BIT   = 0;
localparam STATUS_DONE_BIT   = 1;
localparam STATUS_ERROR_BIT  = 2;

localparam VERSION_VALUE     = 32'h4d54_4d31;  // "MTM1"
localparam DMA_IDLE          = 3'd0;
localparam DMA_READ_AR       = 3'd1;
localparam DMA_READ_R        = 3'd2;
localparam DMA_COMPUTE       = 3'd3;
localparam DMA_WRITE_AW      = 3'd4;
localparam DMA_WRITE_W       = 3'd5;
localparam DMA_WRITE_B       = 3'd6;

integer i;

reg [31:0] a_regs [0:15];
reg [31:0] b_regs [0:15];
reg [31:0] c_regs [0:47];
reg [31:0] c_regs_alt [0:47];

reg        busy;
reg        done;
reg        error;
reg        compute_active;
reg        dma_active;
reg [31:0] ctrl_shadow;
reg [31:0] src_base_reg;
reg [31:0] dst_base_reg;
reg [31:0] group_count_reg;
reg [31:0] crc_acc;
reg [31:0] crc_result_reg;

reg [31:0] awaddr_latched;
reg [4:0]  awid_latched;
reg        aw_pending;

reg [31:0] reg_rdata;
reg [2:0]  dma_state;
reg [31:0] dma_group;
reg [31:0] dma_write_group;
reg [5:0]  dma_read_word;
reg [5:0]  dma_write_word;
reg [31:0] dma_write_data;
reg        compute_slot;
reg        dma_write_slot;
reg        pending_write_valid;
reg        pending_write_slot;
reg [31:0] pending_write_group;
reg        compute_done_pending;
reg        compute_done_slot;
reg [31:0] compute_done_group;
reg        direct_active_reg;
reg        direct_read_active;
reg        direct_write_strobe;
reg [1:0]  calc_row;
reg [1:0]  calc_col;
reg [1:0]  calc_k;
reg [5:0]  mul_bit;
reg        mul_finish;
reg [65:0] sum_acc;
reg [65:0] product_acc;
reg [31:0] multiplicand_reg;
reg [31:0] multiplier_reg;
reg [65:0] sum_acc1;
reg [65:0] product_acc1;
reg [31:0] multiplicand_reg1;
reg [31:0] multiplier_reg1;
reg [65:0] sum_acc2;
reg [65:0] product_acc2;
reg [31:0] multiplicand_reg2;
reg [31:0] multiplier_reg2;
reg [65:0] sum_acc3;
reg [65:0] product_acc3;
reg [31:0] multiplicand_reg3;
reg [31:0] multiplier_reg3;
reg [65:0] sum_acc4;
reg [65:0] product_acc4;
reg [31:0] multiplicand_reg4;
reg [31:0] multiplier_reg4;
reg [65:0] sum_acc5;
reg [65:0] product_acc5;
reg [31:0] multiplicand_reg5;
reg [31:0] multiplier_reg5;
reg [65:0] sum_acc6;
reg [65:0] product_acc6;
reg [31:0] multiplicand_reg6;
reg [31:0] multiplier_reg6;
reg [65:0] sum_acc7;
reg [65:0] product_acc7;
reg [31:0] multiplicand_reg7;
reg [31:0] multiplier_reg7;
// Register window spans byte offsets 0x000..0x14c (C region reaches 0x14c),
// so decode on the low 12 bits, not just [7:0]; the upper C addresses would
// otherwise alias down into CTRL/STATUS/A.
wire [11:0] ar_word_addr = s_araddr[11:0];
wire [11:0] aw_word_addr = awaddr_latched[11:0];
wire [31:0] ctrl_wdata = apply_wstrb(ctrl_shadow, s_wdata, s_wstrb);
wire [1:0]  next_calc_row = calc_row + 2'd2;
wire [1:0]  calc_row_1 = calc_row + 2'd1;
wire [65:0] shifted_multiplicand = {34'b0, multiplicand_reg} << mul_bit;
wire [65:0] product_acc_next = product_acc
    + (multiplier_reg[0] ? shifted_multiplicand : 66'b0)
    + (multiplier_reg[1] ? (shifted_multiplicand << 1) : 66'b0)
    + (multiplier_reg[2] ? (shifted_multiplicand << 2) : 66'b0)
    + (multiplier_reg[3] ? (shifted_multiplicand << 3) : 66'b0)
    + (multiplier_reg[4] ? (shifted_multiplicand << 4) : 66'b0)
    + (multiplier_reg[5] ? (shifted_multiplicand << 5) : 66'b0)
    + (multiplier_reg[6] ? (shifted_multiplicand << 6) : 66'b0)
    + (multiplier_reg[7] ? (shifted_multiplicand << 7) : 66'b0);
wire [65:0] sum_acc_finish = sum_acc + product_acc;
wire [65:0] shifted_multiplicand1 = {34'b0, multiplicand_reg1} << mul_bit;
wire [65:0] product_acc_next1 = product_acc1
    + (multiplier_reg1[0] ? shifted_multiplicand1 : 66'b0)
    + (multiplier_reg1[1] ? (shifted_multiplicand1 << 1) : 66'b0)
    + (multiplier_reg1[2] ? (shifted_multiplicand1 << 2) : 66'b0)
    + (multiplier_reg1[3] ? (shifted_multiplicand1 << 3) : 66'b0)
    + (multiplier_reg1[4] ? (shifted_multiplicand1 << 4) : 66'b0)
    + (multiplier_reg1[5] ? (shifted_multiplicand1 << 5) : 66'b0)
    + (multiplier_reg1[6] ? (shifted_multiplicand1 << 6) : 66'b0)
    + (multiplier_reg1[7] ? (shifted_multiplicand1 << 7) : 66'b0);
wire [65:0] sum_acc_finish1 = sum_acc1 + product_acc1;
wire [65:0] shifted_multiplicand2 = {34'b0, multiplicand_reg2} << mul_bit;
wire [65:0] product_acc_next2 = product_acc2
    + (multiplier_reg2[0] ? shifted_multiplicand2 : 66'b0)
    + (multiplier_reg2[1] ? (shifted_multiplicand2 << 1) : 66'b0)
    + (multiplier_reg2[2] ? (shifted_multiplicand2 << 2) : 66'b0)
    + (multiplier_reg2[3] ? (shifted_multiplicand2 << 3) : 66'b0)
    + (multiplier_reg2[4] ? (shifted_multiplicand2 << 4) : 66'b0)
    + (multiplier_reg2[5] ? (shifted_multiplicand2 << 5) : 66'b0)
    + (multiplier_reg2[6] ? (shifted_multiplicand2 << 6) : 66'b0)
    + (multiplier_reg2[7] ? (shifted_multiplicand2 << 7) : 66'b0);
wire [65:0] sum_acc_finish2 = sum_acc2 + product_acc2;
wire [65:0] shifted_multiplicand3 = {34'b0, multiplicand_reg3} << mul_bit;
wire [65:0] product_acc_next3 = product_acc3
    + (multiplier_reg3[0] ? shifted_multiplicand3 : 66'b0)
    + (multiplier_reg3[1] ? (shifted_multiplicand3 << 1) : 66'b0)
    + (multiplier_reg3[2] ? (shifted_multiplicand3 << 2) : 66'b0)
    + (multiplier_reg3[3] ? (shifted_multiplicand3 << 3) : 66'b0)
    + (multiplier_reg3[4] ? (shifted_multiplicand3 << 4) : 66'b0)
    + (multiplier_reg3[5] ? (shifted_multiplicand3 << 5) : 66'b0)
    + (multiplier_reg3[6] ? (shifted_multiplicand3 << 6) : 66'b0)
    + (multiplier_reg3[7] ? (shifted_multiplicand3 << 7) : 66'b0);
wire [65:0] sum_acc_finish3 = sum_acc3 + product_acc3;
wire [65:0] shifted_multiplicand4 = {34'b0, multiplicand_reg4} << mul_bit;
wire [65:0] product_acc_next4 = product_acc4
    + (multiplier_reg4[0] ? shifted_multiplicand4 : 66'b0)
    + (multiplier_reg4[1] ? (shifted_multiplicand4 << 1) : 66'b0)
    + (multiplier_reg4[2] ? (shifted_multiplicand4 << 2) : 66'b0)
    + (multiplier_reg4[3] ? (shifted_multiplicand4 << 3) : 66'b0)
    + (multiplier_reg4[4] ? (shifted_multiplicand4 << 4) : 66'b0)
    + (multiplier_reg4[5] ? (shifted_multiplicand4 << 5) : 66'b0)
    + (multiplier_reg4[6] ? (shifted_multiplicand4 << 6) : 66'b0)
    + (multiplier_reg4[7] ? (shifted_multiplicand4 << 7) : 66'b0);
wire [65:0] sum_acc_finish4 = sum_acc4 + product_acc4;
wire [65:0] shifted_multiplicand5 = {34'b0, multiplicand_reg5} << mul_bit;
wire [65:0] product_acc_next5 = product_acc5
    + (multiplier_reg5[0] ? shifted_multiplicand5 : 66'b0)
    + (multiplier_reg5[1] ? (shifted_multiplicand5 << 1) : 66'b0)
    + (multiplier_reg5[2] ? (shifted_multiplicand5 << 2) : 66'b0)
    + (multiplier_reg5[3] ? (shifted_multiplicand5 << 3) : 66'b0)
    + (multiplier_reg5[4] ? (shifted_multiplicand5 << 4) : 66'b0)
    + (multiplier_reg5[5] ? (shifted_multiplicand5 << 5) : 66'b0)
    + (multiplier_reg5[6] ? (shifted_multiplicand5 << 6) : 66'b0)
    + (multiplier_reg5[7] ? (shifted_multiplicand5 << 7) : 66'b0);
wire [65:0] sum_acc_finish5 = sum_acc5 + product_acc5;
wire [65:0] shifted_multiplicand6 = {34'b0, multiplicand_reg6} << mul_bit;
wire [65:0] product_acc_next6 = product_acc6
    + (multiplier_reg6[0] ? shifted_multiplicand6 : 66'b0)
    + (multiplier_reg6[1] ? (shifted_multiplicand6 << 1) : 66'b0)
    + (multiplier_reg6[2] ? (shifted_multiplicand6 << 2) : 66'b0)
    + (multiplier_reg6[3] ? (shifted_multiplicand6 << 3) : 66'b0)
    + (multiplier_reg6[4] ? (shifted_multiplicand6 << 4) : 66'b0)
    + (multiplier_reg6[5] ? (shifted_multiplicand6 << 5) : 66'b0)
    + (multiplier_reg6[6] ? (shifted_multiplicand6 << 6) : 66'b0)
    + (multiplier_reg6[7] ? (shifted_multiplicand6 << 7) : 66'b0);
wire [65:0] sum_acc_finish6 = sum_acc6 + product_acc6;
wire [65:0] shifted_multiplicand7 = {34'b0, multiplicand_reg7} << mul_bit;
wire [65:0] product_acc_next7 = product_acc7
    + (multiplier_reg7[0] ? shifted_multiplicand7 : 66'b0)
    + (multiplier_reg7[1] ? (shifted_multiplicand7 << 1) : 66'b0)
    + (multiplier_reg7[2] ? (shifted_multiplicand7 << 2) : 66'b0)
    + (multiplier_reg7[3] ? (shifted_multiplicand7 << 3) : 66'b0)
    + (multiplier_reg7[4] ? (shifted_multiplicand7 << 4) : 66'b0)
    + (multiplier_reg7[5] ? (shifted_multiplicand7 << 5) : 66'b0)
    + (multiplier_reg7[6] ? (shifted_multiplicand7 << 6) : 66'b0)
    + (multiplier_reg7[7] ? (shifted_multiplicand7 << 7) : 66'b0);
wire [65:0] sum_acc_finish7 = sum_acc7 + product_acc7;
wire [31:0] dma_src_addr = src_base_reg + (dma_group << 7) + {24'b0, dma_read_word, 2'b00};
wire [31:0] dma_dst_addr = dst_base_reg + (dma_write_group << 7) + (dma_write_group << 6) + {24'b0, dma_write_word, 2'b00};
wire        dma_write_last_group = (dma_write_group == (group_count_reg - 32'd1));
wire        dma_last_write = (dma_write_word == 6'd47);
wire [31:0] dma_current_write_data = dma_write_slot ? c_regs_alt[dma_write_word] : c_regs[dma_write_word];
wire [31:0] crc_next_word = crc32_update_word(crc_acc, dma_current_write_data);

assign direct_ext_active = direct_active_reg;
assign direct_ext_ce_n = ~direct_active_reg;
assign direct_ext_oe_n = ~(direct_active_reg && direct_read_active);
assign direct_ext_we_n = ~(direct_active_reg && direct_write_strobe && !clk);

assign m_arid    = 4'b0;
assign m_arlen   = 8'd31;
assign m_arsize  = 3'b010;
assign m_arburst = 2'b01;
assign m_arlock  = 1'b0;
assign m_arcache = 4'b0011;
assign m_arprot  = 3'b000;
assign m_rready  = 1'b1;
assign m_awid    = 4'b0;
assign m_awlen   = 8'd47;
assign m_awsize  = 3'b010;
assign m_awburst = 2'b01;
assign m_awlock  = 1'b0;
assign m_awcache = 4'b0011;
assign m_awprot  = 3'b000;
assign m_wstrb   = 4'b1111;
assign m_bready  = 1'b1;

function [5:0] c_word_index;
    input [1:0] row_idx;
    input [1:0] col_idx;
    reg   [3:0] element_idx;
    reg   [5:0] c_base_idx;
    begin
        element_idx = {row_idx, 2'b00} + {2'b00, col_idx};
        c_base_idx = {element_idx, 1'b0};
        c_word_index = c_base_idx + {2'b00, element_idx};
    end
endfunction

function [3:0] a_word_index;
    input [1:0] row_idx;
    input [1:0] k_idx;
    begin
        a_word_index = {row_idx, 2'b00} + {2'b00, k_idx};
    end
endfunction

function [3:0] b_word_index;
    input [1:0] k_idx;
    input [1:0] col_idx;
    begin
        b_word_index = {k_idx, 2'b00} + {2'b00, col_idx};
    end
endfunction

function [31:0] apply_wstrb;
    input [31:0] current;
    input [31:0] data;
    input [3:0]  strb;
    begin
        apply_wstrb = current;
        if (strb[0]) apply_wstrb[7:0]   = data[7:0];
        if (strb[1]) apply_wstrb[15:8]  = data[15:8];
        if (strb[2]) apply_wstrb[23:16] = data[23:16];
        if (strb[3]) apply_wstrb[31:24] = data[31:24];
    end
endfunction

function [31:0] crc32_update_byte;
    input [31:0] crc_in;
    input [7:0]  data_in;
    reg [31:0] crc;
    integer bit_idx;
    begin
        crc = crc_in ^ {24'b0, data_in};
        for (bit_idx = 0; bit_idx < 8; bit_idx = bit_idx + 1) begin
            if (crc[0]) begin
                crc = (crc >> 1) ^ 32'hedb8_8320;
            end else begin
                crc = crc >> 1;
            end
        end
        crc32_update_byte = crc;
    end
endfunction

function [31:0] crc32_update_word;
    input [31:0] crc_in;
    input [31:0] data_in;
    reg [31:0] crc;
    begin
        crc = crc32_update_byte(crc_in, data_in[7:0]);
        crc = crc32_update_byte(crc, data_in[15:8]);
        crc = crc32_update_byte(crc, data_in[23:16]);
        crc = crc32_update_byte(crc, data_in[31:24]);
        crc32_update_word = crc;
    end
endfunction

always @(*) begin
    reg_rdata = 32'b0;

    if (ar_word_addr == CTRL_ADDR) begin
        reg_rdata = ctrl_shadow;
    end else if (ar_word_addr == STATUS_ADDR) begin
        reg_rdata = {29'b0, error, done, busy};
    end else if (ar_word_addr == VERSION_ADDR) begin
        reg_rdata = VERSION_VALUE;
    end else if ((ar_word_addr >= A_BASE_ADDR) && (ar_word_addr < (A_BASE_ADDR + 12'h040))) begin
        reg_rdata = a_regs[(ar_word_addr - A_BASE_ADDR) >> 2];
    end else if ((ar_word_addr >= B_BASE_ADDR) && (ar_word_addr < (B_BASE_ADDR + 12'h040))) begin
        reg_rdata = b_regs[(ar_word_addr - B_BASE_ADDR) >> 2];
    end else if ((ar_word_addr >= C_BASE_ADDR) && (ar_word_addr < (C_BASE_ADDR + 12'h0c0))) begin
        reg_rdata = c_regs[(ar_word_addr - C_BASE_ADDR) >> 2];
    end else if (ar_word_addr == SRC_BASE_ADDR) begin
        reg_rdata = src_base_reg;
    end else if (ar_word_addr == DST_BASE_ADDR) begin
        reg_rdata = dst_base_reg;
    end else if (ar_word_addr == GROUP_COUNT_ADDR) begin
        reg_rdata = group_count_reg;
    end else if (ar_word_addr == CRC32_ADDR) begin
        reg_rdata = crc_result_reg;
    end
end

always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
        s_awready <= 1'b1;
        s_wready  <= 1'b1;
        s_bvalid  <= 1'b0;
        s_bid     <= 5'b0;
        s_bresp   <= 2'b0;
        m_arvalid <= 1'b0;
        m_araddr  <= 32'b0;
        m_awvalid <= 1'b0;
        m_awaddr  <= 32'b0;
        m_wvalid  <= 1'b0;
        m_wdata   <= 32'b0;
        m_wlast   <= 1'b0;
        s_arready <= 1'b1;
        s_rvalid  <= 1'b0;
        s_rdata   <= 32'b0;
        s_rid     <= 5'b0;
        s_rresp   <= 2'b0;
        s_rlast   <= 1'b0;
        awaddr_latched <= 32'b0;
        awid_latched   <= 5'b0;
        aw_pending     <= 1'b0;
        busy           <= 1'b0;
        done           <= 1'b0;
        error          <= 1'b0;
        compute_active <= 1'b0;
        dma_active     <= 1'b0;
        ctrl_shadow    <= 32'b0;
        src_base_reg   <= 32'b0;
        dst_base_reg   <= 32'b0;
        group_count_reg <= 32'b0;
        crc_acc        <= 32'hffff_ffff;
        crc_result_reg <= 32'b0;
        dma_state      <= DMA_IDLE;
        dma_group      <= 32'b0;
        dma_write_group <= 32'b0;
        dma_read_word  <= 6'b0;
        dma_write_word <= 6'b0;
        dma_write_data <= 32'b0;
        compute_slot   <= 1'b0;
        dma_write_slot <= 1'b0;
        pending_write_valid <= 1'b0;
        pending_write_slot <= 1'b0;
        pending_write_group <= 32'b0;
        compute_done_pending <= 1'b0;
        compute_done_slot <= 1'b0;
        compute_done_group <= 32'b0;
        direct_active_reg <= 1'b0;
        direct_read_active <= 1'b0;
        direct_write_strobe <= 1'b0;
        direct_ext_addr <= 20'b0;
        direct_ext_be_n <= 4'b1111;
        direct_ext_wdata <= 32'b0;
        calc_row       <= 2'b0;
        calc_col       <= 2'b0;
        calc_k         <= 2'b0;
        mul_bit        <= 6'b0;
        mul_finish     <= 1'b0;
        sum_acc        <= 66'b0;
        product_acc    <= 66'b0;
        multiplicand_reg <= 32'b0;
        multiplier_reg   <= 32'b0;
        sum_acc1       <= 66'b0;
        product_acc1   <= 66'b0;
        multiplicand_reg1 <= 32'b0;
        multiplier_reg1   <= 32'b0;
        sum_acc2       <= 66'b0;
        product_acc2   <= 66'b0;
        multiplicand_reg2 <= 32'b0;
        multiplier_reg2   <= 32'b0;
        sum_acc3       <= 66'b0;
        product_acc3   <= 66'b0;
        multiplicand_reg3 <= 32'b0;
        multiplier_reg3   <= 32'b0;
        sum_acc4       <= 66'b0;
        product_acc4   <= 66'b0;
        multiplicand_reg4 <= 32'b0;
        multiplier_reg4   <= 32'b0;
        sum_acc5       <= 66'b0;
        product_acc5   <= 66'b0;
        multiplicand_reg5 <= 32'b0;
        multiplier_reg5   <= 32'b0;
        sum_acc6       <= 66'b0;
        product_acc6   <= 66'b0;
        multiplicand_reg6 <= 32'b0;
        multiplier_reg6   <= 32'b0;
        sum_acc7       <= 66'b0;
        product_acc7   <= 66'b0;
        multiplicand_reg7 <= 32'b0;
        multiplier_reg7   <= 32'b0;
        for (i = 0; i < 16; i = i + 1) begin
            a_regs[i] <= 32'b0;
            b_regs[i] <= 32'b0;
        end
        for (i = 0; i < 48; i = i + 1) begin
            c_regs[i] <= 32'b0;
            c_regs_alt[i] <= 32'b0;
        end
    end else begin
        s_awready <= (!aw_pending) && (!s_bvalid);
        s_wready  <= aw_pending && (!s_bvalid);
        s_arready <= !s_rvalid;

        if (s_bvalid && s_bready) begin
            s_bvalid <= 1'b0;
        end
        if (s_rvalid && s_rready) begin
            s_rvalid <= 1'b0;
            s_rlast  <= 1'b0;
        end

        if (s_awvalid && s_awready) begin
            awaddr_latched <= s_awaddr;
            awid_latched   <= s_awid;
            aw_pending     <= 1'b1;
            if ((s_awlen != 8'b0) || (s_awsize != 3'b010) || (s_awburst != 2'b01)) begin
                error <= 1'b1;
            end
        end

        if (aw_pending && s_wvalid && s_wready) begin
            aw_pending <= 1'b0;
            s_bvalid   <= 1'b1;
            s_bid      <= awid_latched;
            s_bresp    <= 2'b00;

            if (!s_wlast) begin
                error <= 1'b1;
            end

            if (aw_word_addr == CTRL_ADDR) begin
                ctrl_shadow <= ctrl_wdata & 32'h0000_0003;

                if (ctrl_wdata[CTRL_SOFT_RST_BIT]) begin
                    busy  <= 1'b0;
                    done  <= 1'b0;
                    error <= 1'b0;
                    compute_active <= 1'b0;
                    dma_active <= 1'b0;
                    dma_state <= DMA_IDLE;
                    m_arvalid <= 1'b0;
                    m_awvalid <= 1'b0;
                    m_wvalid <= 1'b0;
                    m_wlast <= 1'b0;
                    dma_group <= 32'b0;
                    dma_write_group <= 32'b0;
                    dma_read_word <= 6'b0;
                    dma_write_word <= 6'b0;
                    dma_write_data <= 32'b0;
                    compute_slot <= 1'b0;
                    dma_write_slot <= 1'b0;
                    pending_write_valid <= 1'b0;
                    pending_write_slot <= 1'b0;
                    pending_write_group <= 32'b0;
                    compute_done_pending <= 1'b0;
                    compute_done_slot <= 1'b0;
                    compute_done_group <= 32'b0;
                    direct_active_reg <= 1'b0;
                    direct_read_active <= 1'b0;
                    direct_write_strobe <= 1'b0;
                    direct_ext_addr <= 20'b0;
                    direct_ext_be_n <= 4'b1111;
                    direct_ext_wdata <= 32'b0;
                    crc_acc <= 32'hffff_ffff;
                    crc_result_reg <= 32'b0;
                    calc_row <= 2'b0;
                    calc_col <= 2'b0;
                    calc_k   <= 2'b0;
                    mul_bit  <= 6'b0;
                    mul_finish <= 1'b0;
                    sum_acc  <= 66'b0;
                    product_acc <= 66'b0;
                    multiplicand_reg <= 32'b0;
                    multiplier_reg   <= 32'b0;
                    sum_acc1 <= 66'b0;
                    product_acc1 <= 66'b0;
                    multiplicand_reg1 <= 32'b0;
                    multiplier_reg1   <= 32'b0;
                    sum_acc2 <= 66'b0;
                    product_acc2 <= 66'b0;
                    multiplicand_reg2 <= 32'b0;
                    multiplier_reg2   <= 32'b0;
                    sum_acc3 <= 66'b0;
                    product_acc3 <= 66'b0;
                    multiplicand_reg3 <= 32'b0;
                    multiplier_reg3   <= 32'b0;
                    sum_acc4 <= 66'b0;
                    product_acc4 <= 66'b0;
                    multiplicand_reg4 <= 32'b0;
                    multiplier_reg4   <= 32'b0;
                    sum_acc5 <= 66'b0;
                    product_acc5 <= 66'b0;
                    multiplicand_reg5 <= 32'b0;
                    multiplier_reg5   <= 32'b0;
                    sum_acc6 <= 66'b0;
                    product_acc6 <= 66'b0;
                    multiplicand_reg6 <= 32'b0;
                    multiplier_reg6   <= 32'b0;
                    sum_acc7 <= 66'b0;
                    product_acc7 <= 66'b0;
                    multiplicand_reg7 <= 32'b0;
                    multiplier_reg7   <= 32'b0;
                    for (i = 0; i < 48; i = i + 1) begin
                        c_regs[i] <= 32'b0;
                        c_regs_alt[i] <= 32'b0;
                    end
                end else if (ctrl_wdata[CTRL_START_BIT]) begin
                    if (busy) begin
                        error <= 1'b1;
                    end else begin
                        busy <= 1'b1;
                        done <= 1'b0;
                        error <= 1'b0;
                        calc_row <= 2'b0;
                        calc_col <= 2'b0;
                        calc_k   <= 2'b0;
                        mul_bit  <= 6'b0;
                        mul_finish <= 1'b0;
                        sum_acc  <= 66'b0;
                        product_acc <= 66'b0;
                        sum_acc1  <= 66'b0;
                        product_acc1 <= 66'b0;
                        sum_acc2  <= 66'b0;
                        product_acc2 <= 66'b0;
                        sum_acc3  <= 66'b0;
                        product_acc3 <= 66'b0;
                        sum_acc4  <= 66'b0;
                        product_acc4 <= 66'b0;
                        sum_acc5  <= 66'b0;
                        product_acc5 <= 66'b0;
                        sum_acc6  <= 66'b0;
                        product_acc6 <= 66'b0;
                        sum_acc7  <= 66'b0;
                        product_acc7 <= 66'b0;
                        if (group_count_reg != 32'b0) begin
                            compute_active <= 1'b0;
                            dma_active <= 1'b1;
                            dma_state <= DMA_READ_AR;
                            dma_group <= 32'b0;
                            dma_write_group <= 32'b0;
                            dma_read_word <= 6'b0;
                            dma_write_word <= 6'b0;
                            compute_slot <= 1'b0;
                            dma_write_slot <= 1'b0;
                            pending_write_valid <= 1'b0;
                            pending_write_slot <= 1'b0;
                            pending_write_group <= 32'b0;
                            compute_done_pending <= 1'b0;
                            compute_done_slot <= 1'b0;
                            compute_done_group <= 32'b0;
                            direct_active_reg <= 1'b1;
                            direct_read_active <= 1'b0;
                            direct_write_strobe <= 1'b0;
                            direct_ext_addr <= src_base_reg[21:2];
                            direct_ext_be_n <= 4'b0000;
                            direct_ext_wdata <= 32'b0;
                            crc_acc <= 32'hffff_ffff;
                            crc_result_reg <= 32'b0;
                        end else begin
                            compute_active <= 1'b1;
                            dma_active <= 1'b0;
                            dma_state <= DMA_IDLE;
                            compute_slot <= 1'b0;
                            multiplicand_reg <= a_regs[0];
                            multiplier_reg   <= b_regs[0];
                            multiplicand_reg1 <= a_regs[0];
                            multiplier_reg1   <= b_regs[1];
                            multiplicand_reg2 <= a_regs[0];
                            multiplier_reg2   <= b_regs[2];
                            multiplicand_reg3 <= a_regs[0];
                            multiplier_reg3   <= b_regs[3];
                            multiplicand_reg4 <= a_regs[4];
                            multiplier_reg4   <= b_regs[0];
                            multiplicand_reg5 <= a_regs[4];
                            multiplier_reg5   <= b_regs[1];
                            multiplicand_reg6 <= a_regs[4];
                            multiplier_reg6   <= b_regs[2];
                            multiplicand_reg7 <= a_regs[4];
                            multiplier_reg7   <= b_regs[3];
                        end
                    end
                end
            end else if ((aw_word_addr >= A_BASE_ADDR) && (aw_word_addr < (A_BASE_ADDR + 12'h040))) begin
                a_regs[(aw_word_addr - A_BASE_ADDR) >> 2] <=
                    apply_wstrb(a_regs[(aw_word_addr - A_BASE_ADDR) >> 2], s_wdata, s_wstrb);
            end else if ((aw_word_addr >= B_BASE_ADDR) && (aw_word_addr < (B_BASE_ADDR + 12'h040))) begin
                b_regs[(aw_word_addr - B_BASE_ADDR) >> 2] <=
                    apply_wstrb(b_regs[(aw_word_addr - B_BASE_ADDR) >> 2], s_wdata, s_wstrb);
            end else if ((aw_word_addr >= C_BASE_ADDR) && (aw_word_addr < (C_BASE_ADDR + 12'h0c0))) begin
                c_regs[(aw_word_addr - C_BASE_ADDR) >> 2] <=
                    apply_wstrb(c_regs[(aw_word_addr - C_BASE_ADDR) >> 2], s_wdata, s_wstrb);
            end else if (aw_word_addr == SRC_BASE_ADDR) begin
                src_base_reg <= apply_wstrb(src_base_reg, s_wdata, s_wstrb);
            end else if (aw_word_addr == DST_BASE_ADDR) begin
                dst_base_reg <= apply_wstrb(dst_base_reg, s_wdata, s_wstrb);
            end else if (aw_word_addr == GROUP_COUNT_ADDR) begin
                group_count_reg <= apply_wstrb(group_count_reg, s_wdata, s_wstrb);
            end else if (aw_word_addr == CRC32_ADDR) begin
                crc_result_reg <= apply_wstrb(crc_result_reg, s_wdata, s_wstrb);
            end else begin
                error <= 1'b1;
            end
        end

        if (s_arvalid && s_arready) begin
            s_rvalid <= 1'b1;
            s_rid    <= s_arid;
            s_rresp  <= 2'b00;
            s_rlast  <= 1'b1;
            s_rdata  <= reg_rdata;

            if ((s_arlen != 8'b0) || (s_arsize != 3'b010) || (s_arburst != 2'b01)) begin
                error <= 1'b1;
            end
        end

        if (compute_active) begin
            if (mul_finish) begin
                if (calc_k == 2'd3) begin
                    if (compute_slot) begin
                        c_regs_alt[c_word_index(calc_row, 2'd0) + 6'd0] <= sum_acc_finish[31:0];
                        c_regs_alt[c_word_index(calc_row, 2'd0) + 6'd1] <= sum_acc_finish[63:32];
                        c_regs_alt[c_word_index(calc_row, 2'd0) + 6'd2] <= {30'b0, sum_acc_finish[65:64]};
                        c_regs_alt[c_word_index(calc_row, 2'd1) + 6'd0] <= sum_acc_finish1[31:0];
                        c_regs_alt[c_word_index(calc_row, 2'd1) + 6'd1] <= sum_acc_finish1[63:32];
                        c_regs_alt[c_word_index(calc_row, 2'd1) + 6'd2] <= {30'b0, sum_acc_finish1[65:64]};
                        c_regs_alt[c_word_index(calc_row, 2'd2) + 6'd0] <= sum_acc_finish2[31:0];
                        c_regs_alt[c_word_index(calc_row, 2'd2) + 6'd1] <= sum_acc_finish2[63:32];
                        c_regs_alt[c_word_index(calc_row, 2'd2) + 6'd2] <= {30'b0, sum_acc_finish2[65:64]};
                        c_regs_alt[c_word_index(calc_row, 2'd3) + 6'd0] <= sum_acc_finish3[31:0];
                        c_regs_alt[c_word_index(calc_row, 2'd3) + 6'd1] <= sum_acc_finish3[63:32];
                        c_regs_alt[c_word_index(calc_row, 2'd3) + 6'd2] <= {30'b0, sum_acc_finish3[65:64]};
                        c_regs_alt[c_word_index(calc_row_1, 2'd0) + 6'd0] <= sum_acc_finish4[31:0];
                        c_regs_alt[c_word_index(calc_row_1, 2'd0) + 6'd1] <= sum_acc_finish4[63:32];
                        c_regs_alt[c_word_index(calc_row_1, 2'd0) + 6'd2] <= {30'b0, sum_acc_finish4[65:64]};
                        c_regs_alt[c_word_index(calc_row_1, 2'd1) + 6'd0] <= sum_acc_finish5[31:0];
                        c_regs_alt[c_word_index(calc_row_1, 2'd1) + 6'd1] <= sum_acc_finish5[63:32];
                        c_regs_alt[c_word_index(calc_row_1, 2'd1) + 6'd2] <= {30'b0, sum_acc_finish5[65:64]};
                        c_regs_alt[c_word_index(calc_row_1, 2'd2) + 6'd0] <= sum_acc_finish6[31:0];
                        c_regs_alt[c_word_index(calc_row_1, 2'd2) + 6'd1] <= sum_acc_finish6[63:32];
                        c_regs_alt[c_word_index(calc_row_1, 2'd2) + 6'd2] <= {30'b0, sum_acc_finish6[65:64]};
                        c_regs_alt[c_word_index(calc_row_1, 2'd3) + 6'd0] <= sum_acc_finish7[31:0];
                        c_regs_alt[c_word_index(calc_row_1, 2'd3) + 6'd1] <= sum_acc_finish7[63:32];
                        c_regs_alt[c_word_index(calc_row_1, 2'd3) + 6'd2] <= {30'b0, sum_acc_finish7[65:64]};
                    end else begin
                        c_regs[c_word_index(calc_row, 2'd0) + 6'd0] <= sum_acc_finish[31:0];
                        c_regs[c_word_index(calc_row, 2'd0) + 6'd1] <= sum_acc_finish[63:32];
                        c_regs[c_word_index(calc_row, 2'd0) + 6'd2] <= {30'b0, sum_acc_finish[65:64]};
                        c_regs[c_word_index(calc_row, 2'd1) + 6'd0] <= sum_acc_finish1[31:0];
                        c_regs[c_word_index(calc_row, 2'd1) + 6'd1] <= sum_acc_finish1[63:32];
                        c_regs[c_word_index(calc_row, 2'd1) + 6'd2] <= {30'b0, sum_acc_finish1[65:64]};
                        c_regs[c_word_index(calc_row, 2'd2) + 6'd0] <= sum_acc_finish2[31:0];
                        c_regs[c_word_index(calc_row, 2'd2) + 6'd1] <= sum_acc_finish2[63:32];
                        c_regs[c_word_index(calc_row, 2'd2) + 6'd2] <= {30'b0, sum_acc_finish2[65:64]};
                        c_regs[c_word_index(calc_row, 2'd3) + 6'd0] <= sum_acc_finish3[31:0];
                        c_regs[c_word_index(calc_row, 2'd3) + 6'd1] <= sum_acc_finish3[63:32];
                        c_regs[c_word_index(calc_row, 2'd3) + 6'd2] <= {30'b0, sum_acc_finish3[65:64]};
                        c_regs[c_word_index(calc_row_1, 2'd0) + 6'd0] <= sum_acc_finish4[31:0];
                        c_regs[c_word_index(calc_row_1, 2'd0) + 6'd1] <= sum_acc_finish4[63:32];
                        c_regs[c_word_index(calc_row_1, 2'd0) + 6'd2] <= {30'b0, sum_acc_finish4[65:64]};
                        c_regs[c_word_index(calc_row_1, 2'd1) + 6'd0] <= sum_acc_finish5[31:0];
                        c_regs[c_word_index(calc_row_1, 2'd1) + 6'd1] <= sum_acc_finish5[63:32];
                        c_regs[c_word_index(calc_row_1, 2'd1) + 6'd2] <= {30'b0, sum_acc_finish5[65:64]};
                        c_regs[c_word_index(calc_row_1, 2'd2) + 6'd0] <= sum_acc_finish6[31:0];
                        c_regs[c_word_index(calc_row_1, 2'd2) + 6'd1] <= sum_acc_finish6[63:32];
                        c_regs[c_word_index(calc_row_1, 2'd2) + 6'd2] <= {30'b0, sum_acc_finish6[65:64]};
                        c_regs[c_word_index(calc_row_1, 2'd3) + 6'd0] <= sum_acc_finish7[31:0];
                        c_regs[c_word_index(calc_row_1, 2'd3) + 6'd1] <= sum_acc_finish7[63:32];
                        c_regs[c_word_index(calc_row_1, 2'd3) + 6'd2] <= {30'b0, sum_acc_finish7[65:64]};
                    end

                    if (calc_row == 2'd2) begin
                        compute_active <= 1'b0;
                        mul_finish <= 1'b0;
                        if (dma_active) begin
                            compute_done_pending <= 1'b1;
                            compute_done_slot <= compute_slot;
                            compute_done_group <= dma_group;
                        end else begin
                            busy <= 1'b0;
                            done <= 1'b1;
                        end
                    end else begin
                        calc_row <= next_calc_row;
                        calc_col <= 2'd0;
                        calc_k   <= 2'd0;
                        mul_bit  <= 6'd0;
                        mul_finish <= 1'b0;
                        sum_acc  <= 66'b0;
                        product_acc <= 66'b0;
                        sum_acc1  <= 66'b0;
                        product_acc1 <= 66'b0;
                        sum_acc2  <= 66'b0;
                        product_acc2 <= 66'b0;
                        sum_acc3  <= 66'b0;
                        product_acc3 <= 66'b0;
                        sum_acc4  <= 66'b0;
                        product_acc4 <= 66'b0;
                        sum_acc5  <= 66'b0;
                        product_acc5 <= 66'b0;
                        sum_acc6  <= 66'b0;
                        product_acc6 <= 66'b0;
                        sum_acc7  <= 66'b0;
                        product_acc7 <= 66'b0;
                        multiplicand_reg <= a_regs[a_word_index(next_calc_row, 2'd0)];
                        multiplier_reg   <= b_regs[b_word_index(2'd0, 2'd0)];
                        multiplicand_reg1 <= a_regs[a_word_index(next_calc_row, 2'd0)];
                        multiplier_reg1   <= b_regs[b_word_index(2'd0, 2'd1)];
                        multiplicand_reg2 <= a_regs[a_word_index(next_calc_row, 2'd0)];
                        multiplier_reg2   <= b_regs[b_word_index(2'd0, 2'd2)];
                        multiplicand_reg3 <= a_regs[a_word_index(next_calc_row, 2'd0)];
                        multiplier_reg3   <= b_regs[b_word_index(2'd0, 2'd3)];
                        multiplicand_reg4 <= a_regs[a_word_index(next_calc_row + 2'd1, 2'd0)];
                        multiplier_reg4   <= b_regs[b_word_index(2'd0, 2'd0)];
                        multiplicand_reg5 <= a_regs[a_word_index(next_calc_row + 2'd1, 2'd0)];
                        multiplier_reg5   <= b_regs[b_word_index(2'd0, 2'd1)];
                        multiplicand_reg6 <= a_regs[a_word_index(next_calc_row + 2'd1, 2'd0)];
                        multiplier_reg6   <= b_regs[b_word_index(2'd0, 2'd2)];
                        multiplicand_reg7 <= a_regs[a_word_index(next_calc_row + 2'd1, 2'd0)];
                        multiplier_reg7   <= b_regs[b_word_index(2'd0, 2'd3)];
                    end
                end else begin
                    calc_k   <= calc_k + 2'd1;
                    mul_bit  <= 6'd0;
                    mul_finish <= 1'b0;
                    sum_acc  <= sum_acc_finish;
                    product_acc <= 66'b0;
                    sum_acc1 <= sum_acc_finish1;
                    product_acc1 <= 66'b0;
                    sum_acc2 <= sum_acc_finish2;
                    product_acc2 <= 66'b0;
                    sum_acc3 <= sum_acc_finish3;
                    product_acc3 <= 66'b0;
                    sum_acc4 <= sum_acc_finish4;
                    product_acc4 <= 66'b0;
                    sum_acc5 <= sum_acc_finish5;
                    product_acc5 <= 66'b0;
                    sum_acc6 <= sum_acc_finish6;
                    product_acc6 <= 66'b0;
                    sum_acc7 <= sum_acc_finish7;
                    product_acc7 <= 66'b0;
                    multiplicand_reg <= a_regs[a_word_index(calc_row, calc_k + 2'd1)];
                    multiplier_reg   <= b_regs[b_word_index(calc_k + 2'd1, 2'd0)];
                    multiplicand_reg1 <= a_regs[a_word_index(calc_row, calc_k + 2'd1)];
                    multiplier_reg1   <= b_regs[b_word_index(calc_k + 2'd1, 2'd1)];
                    multiplicand_reg2 <= a_regs[a_word_index(calc_row, calc_k + 2'd1)];
                    multiplier_reg2   <= b_regs[b_word_index(calc_k + 2'd1, 2'd2)];
                    multiplicand_reg3 <= a_regs[a_word_index(calc_row, calc_k + 2'd1)];
                    multiplier_reg3   <= b_regs[b_word_index(calc_k + 2'd1, 2'd3)];
                    multiplicand_reg4 <= a_regs[a_word_index(calc_row_1, calc_k + 2'd1)];
                    multiplier_reg4   <= b_regs[b_word_index(calc_k + 2'd1, 2'd0)];
                    multiplicand_reg5 <= a_regs[a_word_index(calc_row_1, calc_k + 2'd1)];
                    multiplier_reg5   <= b_regs[b_word_index(calc_k + 2'd1, 2'd1)];
                    multiplicand_reg6 <= a_regs[a_word_index(calc_row_1, calc_k + 2'd1)];
                    multiplier_reg6   <= b_regs[b_word_index(calc_k + 2'd1, 2'd2)];
                    multiplicand_reg7 <= a_regs[a_word_index(calc_row_1, calc_k + 2'd1)];
                    multiplier_reg7   <= b_regs[b_word_index(calc_k + 2'd1, 2'd3)];
                end
            end else if (mul_bit == 6'd24) begin
                mul_finish <= 1'b1;
                product_acc <= product_acc_next;
                product_acc1 <= product_acc_next1;
                product_acc2 <= product_acc_next2;
                product_acc3 <= product_acc_next3;
                product_acc4 <= product_acc_next4;
                product_acc5 <= product_acc_next5;
                product_acc6 <= product_acc_next6;
                product_acc7 <= product_acc_next7;
            end else begin
                mul_bit <= mul_bit + 6'd8;
                product_acc <= product_acc_next;
                multiplier_reg <= {8'b0, multiplier_reg[31:8]};
                product_acc1 <= product_acc_next1;
                multiplier_reg1 <= {8'b0, multiplier_reg1[31:8]};
                product_acc2 <= product_acc_next2;
                multiplier_reg2 <= {8'b0, multiplier_reg2[31:8]};
                product_acc3 <= product_acc_next3;
                multiplier_reg3 <= {8'b0, multiplier_reg3[31:8]};
                product_acc4 <= product_acc_next4;
                multiplier_reg4 <= {8'b0, multiplier_reg4[31:8]};
                product_acc5 <= product_acc_next5;
                multiplier_reg5 <= {8'b0, multiplier_reg5[31:8]};
                product_acc6 <= product_acc_next6;
                multiplier_reg6 <= {8'b0, multiplier_reg6[31:8]};
                product_acc7 <= product_acc_next7;
                multiplier_reg7 <= {8'b0, multiplier_reg7[31:8]};
            end
        end

        if (dma_active) begin
            case (dma_state)
                DMA_READ_AR: begin
                    direct_read_active <= 1'b1;
                    direct_write_strobe <= 1'b0;
                    direct_ext_be_n <= 4'b0000;
                    direct_ext_addr <= dma_src_addr[21:2];
                    dma_state <= DMA_READ_R;
                end

                DMA_READ_R: begin
                    if (dma_read_word < 6'd16) begin
                        a_regs[dma_read_word[3:0]] <= direct_ext_rdata;
                    end else begin
                        b_regs[dma_read_word[3:0]] <= direct_ext_rdata;
                    end

                    if (dma_read_word == 6'd31) begin
                        direct_read_active <= 1'b0;
                        compute_active <= 1'b1;
                        calc_row <= 2'b0;
                        calc_col <= 2'b0;
                        calc_k   <= 2'b0;
                        mul_bit  <= 6'b0;
                        mul_finish <= 1'b0;
                        sum_acc  <= 66'b0;
                        product_acc <= 66'b0;
                        sum_acc1 <= 66'b0;
                        product_acc1 <= 66'b0;
                        sum_acc2 <= 66'b0;
                        product_acc2 <= 66'b0;
                        sum_acc3 <= 66'b0;
                        product_acc3 <= 66'b0;
                        sum_acc4 <= 66'b0;
                        product_acc4 <= 66'b0;
                        sum_acc5 <= 66'b0;
                        product_acc5 <= 66'b0;
                        sum_acc6 <= 66'b0;
                        product_acc6 <= 66'b0;
                        sum_acc7 <= 66'b0;
                        product_acc7 <= 66'b0;
                        multiplicand_reg <= a_regs[0];
                        multiplier_reg   <= b_regs[0];
                        multiplicand_reg1 <= a_regs[0];
                        multiplier_reg1   <= b_regs[1];
                        multiplicand_reg2 <= a_regs[0];
                        multiplier_reg2   <= b_regs[2];
                        multiplicand_reg3 <= a_regs[0];
                        multiplier_reg3   <= b_regs[3];
                        multiplicand_reg4 <= a_regs[4];
                        multiplier_reg4   <= b_regs[0];
                        multiplicand_reg5 <= a_regs[4];
                        multiplier_reg5   <= b_regs[1];
                        multiplicand_reg6 <= a_regs[4];
                        multiplier_reg6   <= b_regs[2];
                        multiplicand_reg7 <= a_regs[4];
                        multiplier_reg7   <= b_regs[3];
                        if (pending_write_valid) begin
                            dma_write_group <= pending_write_group;
                            dma_write_slot <= pending_write_slot;
                            pending_write_valid <= 1'b0;
                            dma_write_word <= 6'b0;
                            dma_state <= DMA_WRITE_AW;
                        end else begin
                            dma_state <= DMA_COMPUTE;
                        end
                    end else begin
                        dma_read_word <= dma_read_word + 6'd1;
                        direct_ext_addr <= dma_src_addr[21:2] + 20'd1;
                        dma_state <= DMA_READ_R;
                    end
                end

                DMA_COMPUTE: begin
                    if (compute_done_pending) begin
                        if (compute_done_group == (group_count_reg - 32'd1)) begin
                            dma_write_group <= compute_done_group;
                            dma_write_slot <= compute_done_slot;
                            compute_done_pending <= 1'b0;
                            dma_write_word <= 6'b0;
                            dma_state <= DMA_WRITE_AW;
                        end else begin
                            pending_write_valid <= 1'b1;
                            pending_write_group <= compute_done_group;
                            pending_write_slot <= compute_done_slot;
                            compute_slot <= ~compute_done_slot;
                            compute_done_pending <= 1'b0;
                            dma_group <= compute_done_group + 32'd1;
                            dma_read_word <= 6'b0;
                            dma_write_word <= 6'b0;
                            dma_state <= DMA_READ_AR;
                        end
                    end
                end

                DMA_WRITE_AW: begin
                    direct_read_active <= 1'b0;
                    direct_write_strobe <= 1'b1;
                    direct_ext_be_n <= 4'b0000;
                    direct_ext_addr <= dma_dst_addr[21:2];
                    direct_ext_wdata <= dma_current_write_data;
                    dma_state <= DMA_WRITE_W;
                end

                DMA_WRITE_W: begin
                    crc_acc <= crc_next_word;
                    if (dma_last_write) begin
                        direct_write_strobe <= 1'b0;
                        dma_state <= DMA_WRITE_B;
                    end else begin
                        dma_write_word <= dma_write_word + 6'd1;
                        direct_ext_addr <= dma_dst_addr[21:2] + 20'd1;
                        direct_ext_wdata <= dma_write_slot ? c_regs_alt[dma_write_word + 6'd1] : c_regs[dma_write_word + 6'd1];
                    end
                end

                DMA_WRITE_B: begin
                    if (dma_write_last_group) begin
                        crc_result_reg <= crc_acc ^ 32'hffff_ffff;
                        busy <= 1'b0;
                        done <= 1'b1;
                        dma_active <= 1'b0;
                        direct_active_reg <= 1'b0;
                        direct_read_active <= 1'b0;
                        direct_write_strobe <= 1'b0;
                        direct_ext_be_n <= 4'b1111;
                        dma_state <= DMA_IDLE;
                    end else if (compute_done_pending) begin
                        if (compute_done_group == (group_count_reg - 32'd1)) begin
                            dma_write_group <= compute_done_group;
                            dma_write_slot <= compute_done_slot;
                            compute_done_pending <= 1'b0;
                            dma_write_word <= 6'b0;
                            dma_state <= DMA_WRITE_AW;
                        end else begin
                            pending_write_valid <= 1'b1;
                            pending_write_group <= compute_done_group;
                            pending_write_slot <= compute_done_slot;
                            compute_slot <= ~compute_done_slot;
                            compute_done_pending <= 1'b0;
                            dma_group <= compute_done_group + 32'd1;
                            dma_read_word <= 6'b0;
                            dma_write_word <= 6'b0;
                            dma_state <= DMA_READ_AR;
                        end
                    end else begin
                        dma_state <= DMA_COMPUTE;
                    end
                end

                default: begin
                    dma_state <= DMA_IDLE;
                end
            endcase
        end
    end
end

endmodule
