/*------------------------------------------------------------------------------
--------------------------------------------------------------------------------
Copyright (c) 2016, Loongson Technology Corporation Limited.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of Loongson Technology Corporation Limited nor the names of 
its contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL LOONGSON TECHNOLOGY CORPORATION LIMITED BE LIABLE
TO ANY PARTY FOR DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------
------------------------------------------------------------------------------*/

module axi_wrap_ram_sp_external (
    input         aclk,
    input         aresetn,
    //ar
    input  [4 :0] axi_arid   ,
    input  [31:0] axi_araddr ,
    input  [7 :0] axi_arlen  ,
    input  [2 :0] axi_arsize ,
    input  [1 :0] axi_arburst,
    input         axi_arlock ,
    input  [3 :0] axi_arcache,
    input  [2 :0] axi_arprot ,
    input         axi_arvalid,
    output        axi_arready,
    //r
    output [4 :0] axi_rid    ,
    output [31:0] axi_rdata  ,
    output [1 :0] axi_rresp  ,
    output        axi_rlast  ,
    output        axi_rvalid ,
    input         axi_rready ,
    //aw
    input  [4 :0] axi_awid   ,
    input  [31:0] axi_awaddr ,
    input  [7 :0] axi_awlen  ,
    input  [2 :0] axi_awsize ,
    input  [1 :0] axi_awburst,
    input         axi_awlock ,
    input  [3 :0] axi_awcache,
    input  [2 :0] axi_awprot ,
    input         axi_awvalid,
    output        axi_awready,
    //w
    input  [31:0] axi_wdata  ,
    input  [3 :0] axi_wstrb  ,
    input         axi_wlast  ,
    input         axi_wvalid ,
    output        axi_wready ,
    //b
    output [4 :0] axi_bid    ,
    output [1 :0] axi_bresp  ,
    output        axi_bvalid ,
    input         axi_bready ,

    //BaseRAM信号
    inout  [31:0] base_ram_data,  //BaseRAM数据，低8位与CPLD串口控制器共享
    output [19:0] base_ram_addr, //BaseRAM地址
    output [ 3:0] base_ram_be_n,  //BaseRAM字节使能，低有效。如果不使用字节使能，请保持为0
    output  base_ram_ce_n,       //BaseRAM片选，低有效
    output  base_ram_oe_n,       //BaseRAM读使能，低有效
    output  base_ram_we_n,       //BaseRAM写使能，低有效

    //ExtRAM信号
    inout  [31:0] ext_ram_data,  //ExtRAM数据
    output [19:0] ext_ram_addr, //ExtRAM地址
    output [ 3:0] ext_ram_be_n,  //ExtRAM字节使能，低有效。如果不使用字节使能，请保持为0
    output  ext_ram_ce_n,       //ExtRAM片选，低有效
    output  ext_ram_oe_n,       //ExtRAM读使能，低有效
    output  ext_ram_we_n,      //ExtRAM写使能，低有效

    // Full-cycle fast reader for the matrix accelerator.
    input         direct_fast_clk,
    input         direct_fast_resetn,
    input         direct_fast_enable,
    input  [31:0] direct_fast_pair_count,
    output [63:0] direct_fast_pair_data,
    output        direct_fast_pair_valid,
    input         direct_fast_pair_ready,
    output        direct_fast_done,
    output        direct_fast_error,

    // Direct ExtRAM port for the matrix accelerator.
    input         direct_ext_active,
    input  [19:0] direct_ext_addr,
    input  [ 3:0] direct_ext_be_n,
    input         direct_ext_ce_n,
    input         direct_ext_oe_n,
    input         direct_ext_we_n,
    input  [31:0] direct_ext_wdata,
    output [31:0] direct_ext_rdata
);


//ram axi
//ar
wire [4 :0] ram_arid   ;
wire [31:0] ram_araddr ;
wire [7 :0] ram_arlen  ;
wire [2 :0] ram_arsize ;
wire [1 :0] ram_arburst;
wire        ram_arlock ;
wire [3 :0] ram_arcache;
wire [2 :0] ram_arprot ;
wire        ram_arvalid;
wire        ram_arready;
//r
wire [4 :0] ram_rid    ;
wire [31:0] ram_rdata  ;
wire [1 :0] ram_rresp  ;
wire        ram_rlast  ;
wire        ram_rvalid ;
wire        ram_rready ;
//aw
wire [4 :0] ram_awid   ;
wire [31:0] ram_awaddr ;
wire [7 :0] ram_awlen  ;
wire [2 :0] ram_awsize ;
wire [1 :0] ram_awburst;
wire        ram_awlock ;
wire [3 :0] ram_awcache;
wire [2 :0] ram_awprot ;
wire        ram_awvalid;
wire        ram_awready;
//w
wire [31:0] ram_wdata  ;
wire [3 :0] ram_wstrb  ;
wire        ram_wlast  ;
wire        ram_wvalid ;
wire        ram_wready ;
//b
wire [4 :0] ram_bid    ;
wire [1 :0] ram_bresp  ;
wire        ram_bvalid ;
wire        ram_bready ;

//sram signal
wire  [31:0]    soc_sram_addr;
wire            soc_sram_cs;
wire            soc_sram_we;
wire  [3:0]     soc_sram_be;
wire  [31:0]    soc_sram_wdata;
wire  [31:0]    soc_sram_rdata;

//ar
assign ram_arid    = axi_arid   ;
assign ram_araddr  = axi_araddr ;
assign ram_arlen   = axi_arlen  ;
assign ram_arsize  = axi_arsize ;
assign ram_arburst = axi_arburst;
assign ram_arlock  = axi_arlock ;
assign ram_arcache = axi_arcache;
assign ram_arprot  = axi_arprot ;
assign ram_arvalid = axi_arvalid;
assign axi_arready = ram_arready;
//r
assign axi_rid    = axi_rvalid ? ram_rid   :  5'd0 ;
assign axi_rdata  = axi_rvalid ? ram_rdata : 32'd0 ;
assign axi_rresp  = axi_rvalid ? ram_rresp :  2'd0 ;
assign axi_rlast  = axi_rvalid ? ram_rlast :  1'd0 ;
assign axi_rvalid = ram_rvalid;
assign ram_rready = axi_rready;
//aw
assign ram_awid    = axi_awid   ;
assign ram_awaddr  = axi_awaddr ;
assign ram_awlen   = axi_awlen  ;
assign ram_awsize  = axi_awsize ;
assign ram_awburst = axi_awburst;
assign ram_awlock  = axi_awlock ;
assign ram_awcache = axi_awcache;
assign ram_awprot  = axi_awprot ;
assign ram_awvalid = axi_awvalid;
assign axi_awready = ram_awready;
//w
assign ram_wdata  = axi_wdata  ;
assign ram_wstrb  = axi_wstrb  ;
assign ram_wlast  = axi_wlast  ;
assign ram_wvalid = axi_wvalid ;
assign axi_wready = ram_wready ;
//b
assign axi_bid    = axi_bvalid ? ram_bid   : 5'd0 ;
assign axi_bresp  = axi_bvalid ? ram_bresp : 2'd0 ;
assign axi_bvalid = ram_bvalid ;
assign ram_bready = axi_bready ;


axi2sram_sp_external #(
    .AXI_ID_WIDTH   ( 5  ),
    .AXI_ADDR_WIDTH ( 32 ),
    .AXI_DATA_WIDTH ( 32 ))
 u_axi_sram_sp (
    .clk                     ( aclk         ),
    .resetn                  ( aresetn      ),

    .s_araddr                ( ram_araddr    ),
    .s_arburst               ( ram_arburst   ),
    .s_arcache               ( ram_arcache   ),
    .s_arid                  ( ram_arid      ),
    .s_arlen                 ( ram_arlen     ),
    .s_arlock                ( ram_arlock    ),
    .s_arprot                ( ram_arprot    ),
    .s_arsize                ( ram_arsize    ),
    .s_arvalid               ( ram_arvalid   ),
    .s_awaddr                ( ram_awaddr    ),
    .s_awburst               ( ram_awburst   ),
    .s_awcache               ( ram_awcache   ),
    .s_awid                  ( ram_awid      ),
    .s_awlen                 ( ram_awlen     ),
    .s_awlock                ( ram_awlock    ),
    .s_awprot                ( ram_awprot    ),
    .s_awsize                ( ram_awsize    ),
    .s_awvalid               ( ram_awvalid   ),
    .s_bready                ( ram_bready    ),
    .s_rready                ( ram_rready    ),
    .s_wdata                 ( ram_wdata     ),
    .s_wlast                 ( ram_wlast     ),
    .s_wstrb                 ( ram_wstrb     ),
    .s_wvalid                ( ram_wvalid    ),
    .s_arready               ( ram_arready   ),
    .s_awready               ( ram_awready   ),
    .s_bid                   ( ram_bid       ),
    .s_bresp                 ( ram_bresp     ),
    .s_bvalid                ( ram_bvalid    ),
    .s_rdata                 ( ram_rdata     ),
    .s_rid                   ( ram_rid       ),
    .s_rlast                 ( ram_rlast     ),
    .s_rresp                 ( ram_rresp     ),
    .s_rvalid                ( ram_rvalid    ),
    .s_wready                ( ram_wready    ),

    .req_o                   ( soc_sram_cs       ),
    .we_o                    ( soc_sram_we       ),
    .addr_o                  ( soc_sram_addr     ),
    .be_o                    ( soc_sram_be       ),
    .data_o                  ( soc_sram_wdata    ),
    .data_i                  ( soc_sram_rdata    )
);

wire choose_sram = soc_sram_addr[22];//1:ExtRAM 0:BaseRAM
wire [3:0] be_out = soc_sram_we ? soc_sram_be : 4'b1111;
wire normal_ext_we_n = choose_sram ? ~soc_sram_we : 1'b1;
wire ext_ram_we_pos = direct_ext_active ? 1'b1 : normal_ext_we_n;
wire ext_ram_we_neg = direct_ext_active ? direct_ext_we_n : 1'b1;

localparam FAST_FIFO_ADDR_WIDTH = 6;
localparam FAST_FIFO_PTR_WIDTH = FAST_FIFO_ADDR_WIDTH + 1;

(* ram_style = "block" *) reg [63:0] fast_fifo_mem [0:(1 << FAST_FIFO_ADDR_WIDTH)-1];
reg [FAST_FIFO_PTR_WIDTH-1:0] fast_wr_bin;
reg [FAST_FIFO_PTR_WIDTH-1:0] fast_wr_gray;
reg [FAST_FIFO_PTR_WIDTH-1:0] fast_rd_bin;
reg [FAST_FIFO_PTR_WIDTH-1:0] fast_rd_gray;
(* ASYNC_REG = "TRUE" *) reg [FAST_FIFO_PTR_WIDTH-1:0] fast_rd_gray_sync1;
(* ASYNC_REG = "TRUE" *) reg [FAST_FIFO_PTR_WIDTH-1:0] fast_rd_gray_sync2;
(* ASYNC_REG = "TRUE" *) reg [FAST_FIFO_PTR_WIDTH-1:0] fast_wr_gray_sync1;
(* ASYNC_REG = "TRUE" *) reg [FAST_FIFO_PTR_WIDTH-1:0] fast_wr_gray_sync2;

(* ASYNC_REG = "TRUE" *) reg [1:0] fast_enable_sync;
(* ASYNC_REG = "TRUE" *) reg [1:0] fast_done_sync;
(* ASYNC_REG = "TRUE" *) reg [1:0] fast_error_sync;
reg        fast_session_started;
reg        fast_session_done;
reg        fast_session_error;
reg [19:0] fast_ext_addr;
reg [31:0] fast_pairs_remaining;
reg        fast_word_phase;
reg [31:0] fast_even_word;
reg [63:0] fast_pair_data_reg;
reg        fast_pair_valid_reg;

wire [FAST_FIFO_PTR_WIDTH-1:0] fast_wr_bin_next = fast_wr_bin + {{(FAST_FIFO_PTR_WIDTH-1){1'b0}}, 1'b1};
wire [FAST_FIFO_PTR_WIDTH-1:0] fast_wr_gray_next =
    (fast_wr_bin_next >> 1) ^ fast_wr_bin_next;
wire fast_fifo_full =
    fast_wr_gray_next
    == {~fast_rd_gray_sync2[FAST_FIFO_PTR_WIDTH-1:FAST_FIFO_PTR_WIDTH-2],
        fast_rd_gray_sync2[FAST_FIFO_PTR_WIDTH-3:0]};
wire fast_fifo_empty = (fast_rd_gray == fast_wr_gray_sync2);

assign direct_fast_pair_data = fast_pair_data_reg;
assign direct_fast_pair_valid = fast_pair_valid_reg;
assign direct_fast_done = fast_done_sync[1];
assign direct_fast_error = fast_error_sync[1];

always @(posedge direct_fast_clk) begin
    if (!direct_fast_resetn) begin
        fast_enable_sync <= 2'b0;
        fast_rd_gray_sync1 <= {FAST_FIFO_PTR_WIDTH{1'b0}};
        fast_rd_gray_sync2 <= {FAST_FIFO_PTR_WIDTH{1'b0}};
        fast_wr_bin <= {FAST_FIFO_PTR_WIDTH{1'b0}};
        fast_wr_gray <= {FAST_FIFO_PTR_WIDTH{1'b0}};
        fast_session_started <= 1'b0;
        fast_session_done <= 1'b0;
        fast_session_error <= 1'b0;
        fast_ext_addr <= 20'b0;
        fast_pairs_remaining <= 32'b0;
        fast_word_phase <= 1'b0;
        fast_even_word <= 32'b0;
    end else begin
        fast_enable_sync <= {fast_enable_sync[0], direct_fast_enable};
        fast_rd_gray_sync1 <= fast_rd_gray;
        fast_rd_gray_sync2 <= fast_rd_gray_sync1;

        if (fast_enable_sync[1] && !fast_session_started) begin
            fast_session_started <= 1'b1;
            fast_session_done <= 1'b0;
            fast_session_error <= (direct_fast_pair_count == 32'b0);
            fast_ext_addr <= direct_ext_addr;
            fast_pairs_remaining <= direct_fast_pair_count;
            fast_word_phase <= 1'b0;
        end else if (fast_session_started && !fast_session_done && !fast_session_error) begin
            if (!fast_word_phase) begin
                fast_even_word <= ext_ram_data;
                fast_ext_addr <= fast_ext_addr + 20'd1;
                fast_word_phase <= 1'b1;
            end else if (!fast_fifo_full) begin
                fast_fifo_mem[fast_wr_bin[FAST_FIFO_ADDR_WIDTH-1:0]]
                    <= {ext_ram_data, fast_even_word};
                fast_wr_bin <= fast_wr_bin_next;
                fast_wr_gray <= fast_wr_gray_next;
                fast_word_phase <= 1'b0;
                if (fast_pairs_remaining == 32'd1) begin
                    fast_pairs_remaining <= 32'b0;
                    fast_session_done <= 1'b1;
                end else begin
                    fast_pairs_remaining <= fast_pairs_remaining - 32'd1;
                    fast_ext_addr <= fast_ext_addr + 20'd1;
                end
            end
        end
    end
end

always @(posedge aclk) begin
    if (!aresetn) begin
        fast_wr_gray_sync1 <= {FAST_FIFO_PTR_WIDTH{1'b0}};
        fast_wr_gray_sync2 <= {FAST_FIFO_PTR_WIDTH{1'b0}};
        fast_rd_bin <= {FAST_FIFO_PTR_WIDTH{1'b0}};
        fast_rd_gray <= {FAST_FIFO_PTR_WIDTH{1'b0}};
        fast_pair_data_reg <= 64'b0;
        fast_pair_valid_reg <= 1'b0;
        fast_done_sync <= 2'b0;
        fast_error_sync <= 2'b0;
    end else begin
        fast_wr_gray_sync1 <= fast_wr_gray;
        fast_wr_gray_sync2 <= fast_wr_gray_sync1;
        fast_done_sync <= {fast_done_sync[0], fast_session_done};
        fast_error_sync <= {fast_error_sync[0], fast_session_error};

        if ((!fast_pair_valid_reg || direct_fast_pair_ready) && !fast_fifo_empty) begin
            fast_pair_data_reg <= fast_fifo_mem[fast_rd_bin[FAST_FIFO_ADDR_WIDTH-1:0]];
            fast_rd_bin <= fast_rd_bin + {{(FAST_FIFO_PTR_WIDTH-1){1'b0}}, 1'b1};
            fast_rd_gray <= ((fast_rd_bin + {{(FAST_FIFO_PTR_WIDTH-1){1'b0}}, 1'b1}) >> 1)
                          ^ (fast_rd_bin + {{(FAST_FIFO_PTR_WIDTH-1){1'b0}}, 1'b1});
            fast_pair_valid_reg <= 1'b1;
        end else if (fast_pair_valid_reg && direct_fast_pair_ready) begin
            fast_pair_valid_reg <= 1'b0;
        end
    end
end

assign base_ram_addr = soc_sram_addr[21:2];
assign base_ram_be_n = choose_sram ? 4'b1111 : ~be_out;
assign base_ram_ce_n = ~(soc_sram_cs & (~choose_sram));
assign base_ram_oe_n = soc_sram_we | choose_sram;
assign base_ram_we_n = ~(soc_sram_we & (~choose_sram));
assign base_ram_data = ((~choose_sram) & soc_sram_cs & soc_sram_we) ? soc_sram_wdata : 32'hzzzzzzzz;

assign ext_ram_addr = direct_fast_enable ? fast_ext_addr
                    : direct_ext_active ? direct_ext_addr
                    : soc_sram_addr[21:2];
assign ext_ram_be_n = direct_ext_active ? direct_ext_be_n : (choose_sram ? ~be_out : 4'b1111);
assign ext_ram_ce_n = direct_ext_active ? direct_ext_ce_n : (choose_sram ? ~soc_sram_cs : 1'b1);
assign ext_ram_oe_n = direct_ext_active ? direct_ext_oe_n : (choose_sram ? soc_sram_we : 1'b1);
`ifdef MODELSIM_BUILD
assign ext_ram_we_n = direct_ext_active ? (direct_ext_we_n | aclk) : normal_ext_we_n;
`elsif VERILATOR
assign ext_ram_we_n = direct_ext_active ? (direct_ext_we_n | aclk) : normal_ext_we_n;
`else
ODDR #(
    .DDR_CLK_EDGE("OPPOSITE_EDGE"),
    .INIT(1'b1),
    .SRTYPE("SYNC")
) ext_ram_we_n_oddr (
    .Q(ext_ram_we_n),
    .C(aclk),
    .CE(1'b1),
    .D1(ext_ram_we_pos),
    .D2(ext_ram_we_neg),
    .R(1'b0),
    .S(1'b0)
);
`endif
assign ext_ram_data = direct_ext_active
                    ? (direct_ext_oe_n ? direct_ext_wdata : 32'hzzzzzzzz)
                    : (((choose_sram) & soc_sram_cs & soc_sram_we) ? soc_sram_wdata : 32'hzzzzzzzz);

assign soc_sram_rdata = choose_sram ? ext_ram_data : base_ram_data;
assign direct_ext_rdata = ext_ram_data;

endmodule
