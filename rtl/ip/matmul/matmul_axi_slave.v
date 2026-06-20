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
    output reg         s_rlast
);

localparam [11:0] CTRL_ADDR    = 12'h000;
localparam [11:0] STATUS_ADDR  = 12'h004;
localparam [11:0] VERSION_ADDR = 12'h008;
localparam [11:0] A_BASE_ADDR  = 12'h010;
localparam [11:0] B_BASE_ADDR  = 12'h050;
localparam [11:0] C_BASE_ADDR  = 12'h090;

localparam CTRL_START_BIT    = 0;
localparam CTRL_SOFT_RST_BIT = 1;

localparam STATUS_BUSY_BIT   = 0;
localparam STATUS_DONE_BIT   = 1;
localparam STATUS_ERROR_BIT  = 2;

localparam VERSION_VALUE     = 32'h4d54_4d31;  // "MTM1"

integer i;

reg [31:0] a_regs [0:15];
reg [31:0] b_regs [0:15];
reg [31:0] c_regs [0:47];

reg        busy;
reg        done;
reg        error;
reg [31:0] ctrl_shadow;

reg [31:0] awaddr_latched;
reg [4:0]  awid_latched;
reg        aw_pending;

reg [31:0] reg_rdata;
reg [1:0]  calc_row;
reg [1:0]  calc_col;
// Register window spans byte offsets 0x000..0x14c (C region reaches 0x14c),
// so decode on the low 12 bits, not just [7:0]; the upper C addresses would
// otherwise alias down into CTRL/STATUS/A.
wire [11:0] ar_word_addr = s_araddr[11:0];
wire [11:0] aw_word_addr = awaddr_latched[11:0];
wire [31:0] ctrl_wdata = apply_wstrb(ctrl_shadow, s_wdata, s_wstrb);
wire [1:0]  next_calc_row = (calc_col == 2'd3) ? (calc_row + 2'd1) : calc_row;
wire [1:0]  next_calc_col = (calc_col == 2'd3) ? 2'd0 : (calc_col + 2'd1);
wire [63:0] product_0 = lut_mul32(a_regs[a_word_index(calc_row, 2'd0)],
                                  b_regs[b_word_index(2'd0, calc_col)]);
wire [63:0] product_1 = lut_mul32(a_regs[a_word_index(calc_row, 2'd1)],
                                  b_regs[b_word_index(2'd1, calc_col)]);
wire [63:0] product_2 = lut_mul32(a_regs[a_word_index(calc_row, 2'd2)],
                                  b_regs[b_word_index(2'd2, calc_col)]);
wire [63:0] product_3 = lut_mul32(a_regs[a_word_index(calc_row, 2'd3)],
                                  b_regs[b_word_index(2'd3, calc_col)]);
wire [65:0] c_sum =
    {2'b0, product_0} + {2'b0, product_1} + {2'b0, product_2} + {2'b0, product_3};

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

function [63:0] lut_mul32;
    input [31:0] lhs;
    input [31:0] rhs;
    integer bit_idx;
    reg [63:0] acc;
    reg [63:0] lhs_ext;
    begin
        acc = 64'b0;
        lhs_ext = {32'b0, lhs};
        for (bit_idx = 0; bit_idx < 32; bit_idx = bit_idx + 1) begin
            if (rhs[bit_idx]) begin
                acc = acc + (lhs_ext << bit_idx);
            end
        end
        lut_mul32 = acc;
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
    end
end

always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
        s_awready <= 1'b1;
        s_wready  <= 1'b1;
        s_bvalid  <= 1'b0;
        s_bid     <= 5'b0;
        s_bresp   <= 2'b0;
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
        ctrl_shadow    <= 32'b0;
        calc_row       <= 2'b0;
        calc_col       <= 2'b0;
        for (i = 0; i < 16; i = i + 1) begin
            a_regs[i] <= 32'b0;
            b_regs[i] <= 32'b0;
        end
        for (i = 0; i < 48; i = i + 1) begin
            c_regs[i] <= 32'b0;
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
                    calc_row <= 2'b0;
                    calc_col <= 2'b0;
                    for (i = 0; i < 48; i = i + 1) begin
                        c_regs[i] <= 32'b0;
                    end
                end else if (ctrl_wdata[CTRL_START_BIT]) begin
                    if (busy) begin
                        error <= 1'b1;
                    end else begin
                        busy <= 1'b1;
                        done <= 1'b0;
                        calc_row <= 2'b0;
                        calc_col <= 2'b0;
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

        if (busy) begin
            c_regs[c_word_index(calc_row, calc_col) + 6'd0] <= c_sum[31:0];
            c_regs[c_word_index(calc_row, calc_col) + 6'd1] <= c_sum[63:32];
            c_regs[c_word_index(calc_row, calc_col) + 6'd2] <= {30'b0, c_sum[65:64]};

            if ((calc_row == 2'd3) && (calc_col == 2'd3)) begin
                busy <= 1'b0;
                done <= 1'b1;
            end else begin
                calc_row <= next_calc_row;
                calc_col <= next_calc_col;
            end
        end
    end
end

endmodule
