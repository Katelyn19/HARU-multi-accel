/* MIT License

Copyright (c) 2022 Po Jui Shih
Copyright (c) 2022 Hassaan Saadat
Copyright (c) 2022 Sri Parameswaran
Copyright (c) 2022 Hasindu Gamaarachchi

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

`timescale 1ps / 1ps

`define MAJOR_VERSION       1
`define MINOR_VERSION       0
`define REVISION            0

`define MAJOR_RANGE         31:28
`define MINOR_RANGE         27:20
`define REVISION_RANGE      19:16
`define VERSION_PAD_RANGE   15:0

module dtw_accel #(
    parameter ADDR_WIDTH            = 16,
    parameter DATA_WIDTH            = 32,

    parameter AXIS_DATA_WIDTH       = 32,
    parameter AXIS_DEST_WIDTH       = 4,
    parameter AXIS_ID_WIDTH        = 8,
    parameter AXIS_KEEP_WIDTH       = (AXIS_DATA_WIDTH / 8),
    parameter AXIS_DATA_USER_WIDTH  = 0,

    parameter FIFO_DATA_WIDTH       = AXIS_DATA_WIDTH,
    parameter FIFO_DEPTH            = 4,
    
    parameter INVERT_AXI_RESET      = 1,
    parameter INVERT_AXIS_RESET     = 1,

    parameter DTW_DATA_WIDTH        = 16,

    parameter NUM_CORES             = 2
)(
    input  wire                             S_AXI_clk,
    input  wire                             S_AXI_rst,

    // Write Address Channel
    input  wire                             S_AXI_awvalid,
    input  wire [ADDR_WIDTH - 1: 0]         S_AXI_awaddr,
    output wire                             S_AXI_awready,

    // Write Data Channel
    input  wire                             S_AXI_wvalid,
    output wire                             S_AXI_wready,
    input  wire [DATA_WIDTH - 1: 0]         S_AXI_wdata,

    // Write Response Channel
    output wire                             S_AXI_bvalid,
    input  wire                             S_AXI_bready,
    output wire [1:0]                       S_AXI_bresp,

    // Read Address Channel
    input  wire                             S_AXI_arvalid,
    output wire                             S_AXI_arready,
    input  wire [ADDR_WIDTH - 1: 0]         S_AXI_araddr,

    // Read Data Channel
    output wire                             S_AXI_rvalid,
    input  wire                             S_AXI_rready,
    output wire [1:0]                       S_AXI_rresp,
    output wire [DATA_WIDTH - 1: 0]         S_AXI_rdata,

    // // AXI Stream

    // // Input AXI Stream
    // input  wire                             SRC_AXIS_clk,
    // input  wire                             SRC_AXIS_rst,
    // input  wire                             SRC_AXIS_tuser,
    // input  wire                             SRC_AXIS_tvalid,
    // output wire                             SRC_AXIS_tready,
    // input  wire                             SRC_AXIS_tlast,
    // input  wire [AXIS_DATA_WIDTH - 1:0]     SRC_AXIS_tdata,

    // // Output AXI Stream
    // input  wire                             SINK_AXIS_clk,
    // input  wire                             SINK_AXIS_rst,
    // output wire                             SINK_AXIS_tuser,
    // output wire                             SINK_AXIS_tvalid,
    // input  wire                             SINK_AXIS_tready,
    // output wire                             SINK_AXIS_tlast,
    // output wire [AXIS_DATA_WIDTH - 1:0]     SINK_AXIS_tdata,

    // AXI Multi-channel Stream

    // Input AXI Stream (from the Master mm2s port of the MCDMA)
    input  wire                             SRC_AXIS_clk,
    input  wire                             SRC_AXIS_rst,
    input  wire [AXIS_DATA_WIDTH - 1:0]     SRC_AXIS_tdata,
    input  wire [AXIS_DEST_WIDTH - 1:0]     SRC_AXIS_tdest,
    input  wire [AXIS_ID_WIDTH - 1:0]       SRC_AXIS_tid,
    input  wire [AXIS_KEEP_WIDTH - 1:0]     SRC_AXIS_tkeep,
    input  wire                             SRC_AXIS_tlast,
    input  wire                             SRC_AXIS_tuser,
    output wire                             SRC_AXIS_tready,
    input  wire                             SRC_AXIS_tvalid,

    // Output AXI Stream (to the Slave s2mm port of the MCDMA)
    input  wire                             SINK_AXIS_clk,
    input  wire                             SINK_AXIS_rst,
    output wire [AXIS_DATA_WIDTH - 1:0]     SINK_AXIS_tdata,
    output wire [AXIS_DEST_WIDTH - 1:0]     SINK_AXIS_tdest,
    output wire [AXIS_ID_WIDTH - 1:0]       SINK_AXIS_tid,
    output wire [AXIS_KEEP_WIDTH - 1:0]     SINK_AXIS_tkeep,
    output wire                             SINK_AXIS_tlast,
    input  wire                             SINK_AXIS_tready,
    output wire                             SINK_AXIS_tuser,
    output wire                             SINK_AXIS_tvalid,

    // debug
    output wire [31:0]                      dbg_ref_addr,
    output wire [1:0]                       dbg_ref_state,
    output wire [2:0]                       dbg_dtw_state,
    output wire                             dbg_load_done
);

/* ===============================
 * local parameters
 * =============================== */
// Address Map
// localparam  REG_CONTROL      = 0 << 2;
// localparam  REG_STATUS       = 1 << 2;
// localparam  REG_REF_LEN      = 2 << 2;
// localparam  REG_VERSION      = 3 << 2;
// localparam  REG_KEY          = 4 << 2;
localparam  REG_CONTROL      = 0;
localparam  REG_STATUS       = 1;
localparam  REG_REF_LEN      = 2;
localparam  REG_VERSION      = 3;
localparam  REG_KEY          = 4;
localparam  REG_REF_ADDR     = 5;
localparam  REG_REF_DIN      = 6;
localparam  REG_REF_DOUT     = 7;
localparam  REG_CYCLE_CNT    = 8;
localparam  REG_CORE_REF_ADDR= 9;
localparam  REG_NQUERY       = 10;
localparam  REG_CURR_QID     = 11;

localparam  integer ADDR_LSB = (DATA_WIDTH / 32) + 1;
localparam  integer ADDR_BITS = 3;

localparam  MAX_ADDR = REG_KEY;

localparam REFMEM_PTR_WIDTH = 18;

/* ===============================
 * registers/wires
 * =============================== */
// User Interface
wire                            w_axi_rst;
wire                            w_axis_rst;
wire  [ADDR_WIDTH - 1 : 0]      w_reg_address;
reg                             r_reg_invalid_addr;

wire                            w_reg_in_rdy;
reg                             r_reg_in_ack_stb;
wire  [DATA_WIDTH - 1 : 0]      w_reg_in_data;

wire                            w_reg_out_req;
reg                             r_reg_out_rdy_stb;
reg   [DATA_WIDTH - 1 : 0]      r_reg_out_data;

// DTW accel
reg   [DATA_WIDTH - 1 : 0]      r_control;
wire  [DATA_WIDTH - 1 : 0]      w_status;
reg   [DATA_WIDTH - 1 : 0]      r_ref_len;
wire  [DATA_WIDTH - 1 : 0]      w_version;
wire  [DATA_WIDTH - 1 : 0]      w_key;
reg   [DATA_WIDTH - 1 : 0]      r_dbg_ref_addr;
reg   [DATA_WIDTH - 1 : 0]      r_dbg_ref_din;
wire  [DATA_WIDTH - 1 : 0]      w_dbg_ref_dout;
wire  [DATA_WIDTH - 1 : 0]      w_dtw_core_cycle_counter;

// Control Register bits
wire                            w_dtw_core_rst;
wire                            w_dtw_core_rs;
wire                            w_dtw_core_mode;

// Status Register bits
wire                            w_dtw_core_busy [NUM_CORES-1:0];
wire                            w_dtw_core_load_done;

// Src FIFO
wire  [NUM_CORES-1:0]            w_src_fifo_clear;
wire  [FIFO_DATA_WIDTH - 1:0]   w_src_fifo_w_data[NUM_CORES-1:0];
wire  [NUM_CORES-1:0]            w_src_fifo_w_stb;
wire  [NUM_CORES-1:0]            w_src_fifo_full;
wire  [NUM_CORES-1:0]            w_src_fifo_not_full;

wire  [FIFO_DATA_WIDTH - 1:0]   w_src_fifo_r_data[NUM_CORES-1:0];
wire  [NUM_CORES-1:0]           w_src_fifo_r_stb;
wire  [NUM_CORES-1:0]           w_src_fifo_empty;
wire  [NUM_CORES-1:0]           w_src_fifo_not_empty;

// Sink FIFO
wire  [FIFO_DATA_WIDTH - 1:0]   w_sink_fifo_w_data[NUM_CORES-1:0];
wire  [NUM_CORES-1:0]           w_sink_fifo_w_stb;
wire  [NUM_CORES-1:0]           w_sink_fifo_full;
wire  [NUM_CORES-1:0]           w_sink_fifo_not_full;
wire  [NUM_CORES-1:0]           w_sink_fifo_r_last;

wire  [FIFO_DATA_WIDTH - 1:0]   w_sink_fifo_r_data[NUM_CORES-1:0];
wire  [NUM_CORES-1:0]           w_sink_fifo_r_stb;
wire  [NUM_CORES-1:0]           w_sink_fifo_empty;
wire  [NUM_CORES-1:0]           w_sink_fifo_not_empty;

// Core and ref interfacing signals
wire  [NUM_CORES-1:0]           w_dtw_src_fifo_r_stb;
wire  [NUM_CORES-1:0]           w_dtw_src_fifo_clear;
wire  [NUM_CORES-1:0]           w_dtw_core_done;
wire                            w_ref_src_fifo_r_stb;
wire                            w_ref_src_fifo_clear;
wire  [REFMEM_PTR_WIDTH - 1:0]  w_dtw_ref_addr [NUM_CORES-1:0];
wire  [DTW_DATA_WIDTH - 1 :0]   w_dtw_ref_data;
wire                            w_dtw_ref_busy;

// dtw core debug
wire  [2:0]                     w_dtw_core_state;
wire  [REFMEM_PTR_WIDTH-1:0]   w_dtw_core_addr_ref;
wire  [31:0]                    w_dtw_core_nquery;
wire  [31:0]                    w_dtw_core_curr_qid;


/* ===============================
 * initialization
 * =============================== */
initial begin
    r_control <= 0;
    r_ref_len <= 4000;
end

/* ===============================
 * submodules
 * =============================== */
// Convert AXI Slave bus to a simple register/address strobe
axi_lite_slave #(
    .ADDR_WIDTH         (ADDR_WIDTH),
    .DATA_WIDTH         (DATA_WIDTH)
) axi_lite_reg_interface (
    .clk                (S_AXI_clk),
    .rst                (w_axi_rst),

    .i_awvalid          (S_AXI_awvalid),
    .i_awaddr           (S_AXI_awaddr),
    .o_awready          (S_AXI_awready),

    .i_wvalid           (S_AXI_wvalid),
    .o_wready           (S_AXI_wready),
    .i_wdata            (S_AXI_wdata),

    .o_bvalid           (S_AXI_bvalid),
    .i_bready           (S_AXI_bready),
    .o_bresp            (S_AXI_bresp),

    .i_arvalid          (S_AXI_arvalid),
    .o_arready          (S_AXI_arready),
    .i_araddr           (S_AXI_araddr),

    .o_rvalid           (S_AXI_rvalid),
    .i_rready           (S_AXI_rready),
    .o_rresp            (S_AXI_rresp),
    .o_rdata            (S_AXI_rdata),


    // Register Interface
    .o_reg_address      (w_reg_address),
    .i_reg_invalid_addr (r_reg_invalid_addr),

    // From Master
    .o_reg_in_rdy       (w_reg_in_rdy),
    .i_reg_in_ack_stb   (r_reg_in_ack_stb),
    .o_reg_in_data      (w_reg_in_data),

    // To Master
    .o_reg_out_req      (w_reg_out_req),
    .i_reg_out_rdy_stb  (r_reg_out_rdy_stb),
    .i_reg_out_data     (r_reg_out_data)
);


// AXIS src -> src FIFO
// axis_2_fifo_adapter #(
//     .AXIS_DATA_WIDTH    (AXIS_DATA_WIDTH)
// ) a2fa (
//     .i_axis_tuser       (SRC_AXIS_tuser),
//     .i_axis_tvalid      (SRC_AXIS_tvalid),
//     .o_axis_tready      (SRC_AXIS_tready),
//     .i_axis_tlast       (SRC_AXIS_tlast),
//     .i_axis_tdata       (SRC_AXIS_tdata),

//     .o_fifo_data        (w_src_fifo_w_data),
//     .o_fifo_w_stb       (w_src_fifo_w_stb),
//     .i_fifo_not_full    (w_src_fifo_not_full)
// );

// AXIS src -> src FIFO(s)
mm2s_packet_filter #(
    .AXIS_DATA_WIDTH    (AXIS_DATA_WIDTH),
    .FIFO_DATA_WIDTH    (FIFO_DATA_WIDTH),
    .AXIS_DEST_WIDTH    (AXIS_DEST_WIDTH),
    .NUM_FIFOS          (NUM_CORES)
) mm2s_pf (
    .SRC_AXIS_tdata_in  (SRC_AXIS_tdata),
    .SRC_AXIS_tdest_in  (SRC_AXIS_tdest),
    .SRC_AXIS_tvalid_in (SRC_AXIS_tvalid),
    .SRC_AXIS_tready_out(SRC_AXIS_tready),

    .fifo_wren_out      (w_src_fifo_w_stb),
    .fifo_full_in       (w_src_fifo_full),
    .fifo_data_out      (w_src_fifo_w_data)
);

genvar i;
generate
for (i = 0; i < NUM_CORES; i = i + 1) begin
    fifo #(
        .DEPTH              (FIFO_DEPTH),
        .WIDTH              (FIFO_DATA_WIDTH)
    ) src_fifo (
        .clk                (SRC_AXIS_clk),
        .rst                (w_axis_rst | w_src_fifo_clear[i]),

        .i_fifo_w_stb       (w_src_fifo_w_stb[i]),
        .i_fifo_w_data      (w_src_fifo_w_data[i]),
        .o_fifo_full        (w_src_fifo_full[i]),
        .o_fifo_not_full    (w_src_fifo_not_full[i]),

        .i_fifo_r_stb       (w_src_fifo_r_stb[i]),
        .o_fifo_r_data      (w_src_fifo_r_data[i]),
        .o_fifo_empty       (w_src_fifo_empty[i]),
        .o_fifo_not_empty   (w_src_fifo_not_empty[i])
    );

    // DTW core
    dtw_core #(
        .WIDTH              (DTW_DATA_WIDTH),
        .AXIS_WIDTH         (AXIS_DATA_WIDTH),
        .REF_INIT           (0),
        .REFMEM_PTR_WIDTH   (REFMEM_PTR_WIDTH)
    ) dc (
        .clk                (S_AXI_clk),
        .rst                (w_dtw_core_rst),
        .rs                 (w_dtw_core_rs),

        .ref_len            (r_ref_len),
        .op_mode            (w_dtw_core_mode),
        .busy               (w_dtw_core_busy[i]),

        .load_done          (w_dtw_core_load_done),
        .ref_data           (w_dtw_ref_data),
        .dtw_done           (w_dtw_core_done[i]),
        .addr_ref           (w_dtw_ref_addr[i]),

        .src_fifo_clear     (w_dtw_src_fifo_clear[i]),
        .src_fifo_rden      (w_dtw_src_fifo_r_stb[i]),
        .src_fifo_empty     (w_src_fifo_empty[i]),
        .src_fifo_data      (w_src_fifo_r_data[i]),

        .sink_fifo_wren     (w_sink_fifo_w_stb[i]),
        .sink_fifo_full     (w_sink_fifo_full[i]),
        .sink_fifo_data     (w_sink_fifo_w_data[i]),
        .sink_fifo_last     (w_sink_fifo_r_last[i])
    );

    fifo #(
        .DEPTH              (FIFO_DEPTH),
        .WIDTH              (FIFO_DATA_WIDTH)
    ) sink_fifo (
        .clk                (SRC_AXIS_clk),
        .rst                (w_axis_rst),

        .i_fifo_w_stb       (w_sink_fifo_w_stb[i]),
        .i_fifo_w_data      (w_sink_fifo_w_data[i]),
        .o_fifo_full        (w_sink_fifo_full[i]),
        .o_fifo_not_full    (w_sink_fifo_not_full[i]),

        .i_fifo_r_stb       (w_sink_fifo_r_stb[i]),
        .o_fifo_r_data      (w_sink_fifo_r_data[i]),
        .o_fifo_empty       (w_sink_fifo_empty[i]),
        .o_fifo_not_empty   (w_sink_fifo_not_empty[i])
    );
end
endgenerate

dtw_ref #(
    .WIDTH              (DTW_DATA_WIDTH),   // Data width
    .AXIS_WIDTH         (AXIS_DATA_WIDTH),   // AXI data width
    .REF_INIT           (0),
    .REFMEM_PTR_WIDTH   (REFMEM_PTR_WIDTH)
) dr (
    // Main control signals
    .clk                (S_AXI_clk),
    .rst                (w_dtw_core_rst),
    .rs                 (w_dtw_core_rs),

    .ref_len_in         (r_ref_len[REFMEM_PTR_WIDTH-1:0]),
    .op_mode_in         (w_dtw_core_mode),            // Reference mode: 0, query mode: 1
    .busy_out           (w_dtw_ref_busy),               // Idle: 0, busy_out: 1

    .dtw_done_in        (w_dtw_core_done[0]),
    .dtw_read_addr_in   (w_dtw_ref_addr[0]),
    .ref_data_out       (w_dtw_ref_data),
    .load_done_out      (w_dtw_core_load_done),

    .src_fifo_clear_out (w_ref_src_fifo_clear),
    .src_fifo_rden_out  (w_ref_src_fifo_r_stb),
    .src_fifo_empty     (w_src_fifo_empty),
    .src_fifo_data_in   (w_src_fifo_r_data[0]),

    .dbg_ref_state_out  (dbg_ref_state),
    .dbg_addr_ref_out   (dbg_ref_addr[REFMEM_PTR_WIDTH-1:0])
);

// sink FIFO -> AXIS sink
// fifo_2_axis_adapter #(
//     .AXIS_DATA_WIDTH    (AXIS_DATA_WIDTH)
// )f2aa(
//     .o_fifo_r_stb       (w_sink_fifo_r_stb),
//     .i_fifo_data        (w_sink_fifo_r_data),
//     .i_fifo_not_empty   (w_sink_fifo_not_empty),
//     .i_fifo_last        (w_sink_fifo_r_last),

//     .o_axis_tuser       (SINK_AXIS_tuser),
//     .o_axis_tdata       (SINK_AXIS_tdata),
//     .o_axis_tvalid      (SINK_AXIS_tvalid),
//     .i_axis_tready      (SINK_AXIS_tready),
//     .o_axis_tlast       (SINK_AXIS_tlast)
// );

s2mm_packet_filter #(
    .AXIS_DATA_WIDTH        (AXIS_DATA_WIDTH),
    .FIFO_DATA_WIDTH        (FIFO_DATA_WIDTH),
    .AXIS_KEEP_WIDTH        (AXIS_KEEP_WIDTH),
    .AXIS_DEST_WIDTH        (AXIS_DEST_WIDTH),
    .NUM_CHANNELS           (NUM_CORES)
) s2mm_pf (
    .clk_in                 (SRC_AXIS_clk),
    .rst_in                 (S_AXI_rst),

    .SINK_AXIS_tready_in    (SINK_AXIS_tready),
    .SINK_AXIS_tdata_out    (SINK_AXIS_tdata),
    .SINK_AXIS_tdest_out    (SINK_AXIS_tdest),
    .SINK_AXIS_tkeep_out    (SINK_AXIS_tkeep),
    .SINK_AXIS_tlast_out    (SINK_AXIS_tlast),
    .SINK_AXIS_tuser_out    (SINK_AXIS_tuser),
    .SINK_AXIS_tvalid_out   (SINK_AXIS_tvalid),

    .fifo_data_in           (w_sink_fifo_r_data),
    .fifo_not_empty_in      (w_sink_fifo_not_empty),
    .fifo_last_in           (w_sink_fifo_r_last),
    .fifo_r_stb_out         (w_sink_fifo_r_stb)
);

/* ===============================
 * asynchronous logic
 * =============================== */
assign w_axi_rst                        = INVERT_AXI_RESET  ? ~S_AXI_rst    : S_AXI_rst;
assign w_axis_rst                       = INVERT_AXIS_RESET ? ~SRC_AXIS_rst : SRC_AXIS_rst;
assign w_version[`MAJOR_RANGE]          = `MAJOR_VERSION;
assign w_version[`MINOR_RANGE]          = `MINOR_VERSION;
assign w_version[`REVISION_RANGE]       = `REVISION;
assign w_version[`VERSION_PAD_RANGE]    = 0;
assign w_key                            = 32'h0ca7cafe;

assign w_dtw_core_rst                   = r_control[0];
assign w_dtw_core_rs                    = r_control[1];
assign w_dtw_core_mode                  = r_control[2];

assign w_status[0]                      = |w_dtw_core_busy;
assign w_status[1]                      = w_dtw_core_load_done;
assign w_status[2]                      = w_src_fifo_empty;
assign w_status[3]                      = w_src_fifo_full;
assign w_status[4]                      = w_sink_fifo_empty;
assign w_status[5]                      = w_sink_fifo_full;
assign w_status[8:6]                    = w_dtw_core_state;
// assign w_status[23:9]                   = w_dtw_core_addrW_ref;
// assign w_status[31:24]                  = w_dtw_core_addrR_ref[7:0];
assign w_status[31:9]                   = 0;
assign SINK_AXIS_tid [AXIS_ID_WIDTH - 1:0]                      = {AXIS_ID_WIDTH{1'b0}};
// assign SINK_AXIS_tdest [AXIS_DEST_WIDTH - 1:0]                  = {AXIS_DEST_WIDTH{1'b0}};
// assign SINK_AXIS_tkeep [AXIS_KEEP_WIDTH - 1:0]                  = {AXIS_KEEP_WIDTH{1'b1}};

assign w_src_fifo_r_stb = w_dtw_src_fifo_r_stb | w_ref_src_fifo_r_stb;
assign w_src_fifo_clear = w_dtw_src_fifo_clear | w_ref_src_fifo_clear;

assign dbg_load_done = w_dtw_core_load_done;


/* ===============================
 * synchronous logic
 * =============================== */
always @ (posedge S_AXI_clk) begin
    // De-assert Strobes
    r_reg_in_ack_stb    <=  0;
    r_reg_out_rdy_stb   <=  0;
    r_reg_invalid_addr  <=  0;

    if (w_axi_rst) begin
        r_reg_out_data  <=  0;

        // Reset registers
        r_control       <=  0;
        r_ref_len       <=  0;
    end else begin
        if (w_reg_in_rdy) begin
            // M_AXI to here
            case (w_reg_address[ADDR_LSB + ADDR_BITS:ADDR_LSB])
            REG_CONTROL: begin
                r_control <= w_reg_in_data;
            end
            REG_STATUS: begin
            end
            REG_REF_LEN: begin
                r_ref_len <= w_reg_in_data;
            end
            REG_VERSION: begin
            end
            REG_KEY: begin
            end
            REG_REF_ADDR: begin
                r_dbg_ref_addr <= w_reg_in_data;
            end
            REG_REF_DIN: begin
                r_dbg_ref_din <= w_reg_in_data;
            end
            REG_REF_DOUT: begin
            end
            REG_CYCLE_CNT: begin
            end
            REG_CORE_REF_ADDR: begin
            end
            REG_NQUERY: begin
            end
            REG_CURR_QID: begin
            end
            default: begin // unknown address
                $display ("Unknown address: 0x%h", w_reg_address);
                r_reg_invalid_addr <= 1;
            end
            endcase
            r_reg_in_ack_stb <= 1; // Tell AXI Slave we are done with the data
        end else if (w_reg_out_req) begin
            // Here to M_AXI
            case (w_reg_address[ADDR_LSB + ADDR_BITS:ADDR_LSB])
            REG_CONTROL: begin
                r_reg_out_data <= r_control;
            end
            REG_STATUS: begin
                r_reg_out_data <= w_status;
            end
            REG_REF_LEN: begin
                r_reg_out_data <= r_ref_len;
            end
            REG_VERSION: begin
                r_reg_out_data <= w_version;
            end
            REG_KEY: begin
                r_reg_out_data <= w_key;
            end
            REG_REF_ADDR: begin
                r_reg_out_data <= r_dbg_ref_addr;
            end
            REG_REF_DIN: begin
                r_reg_out_data <= r_dbg_ref_din;
            end
            REG_REF_DOUT: begin
                r_reg_out_data <= w_dbg_ref_dout;
            end
            REG_CYCLE_CNT: begin
                r_reg_out_data <= w_dtw_core_cycle_counter;
            end
            REG_CORE_REF_ADDR: begin
                r_reg_out_data <= {12'h0, w_dtw_core_addr_ref};
            end
            REG_NQUERY: begin
                r_reg_out_data <= w_dtw_core_nquery;
            end
            REG_CURR_QID: begin
                r_reg_out_data <= w_dtw_core_curr_qid;
            end
            default: begin // Unknown address
                r_reg_out_data      <= 32'h00;
                r_reg_invalid_addr  <= 1;
            end
            endcase
            r_reg_out_rdy_stb <= 1; // Tell AXI Slave to send back this packet
        end
    end
end

endmodule