/*
Distributed under the MIT license.
Copyright (c) 2022 Elton Shih (beebdev@gmail.com)

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
 * Author: Elton Shih (beebdev@gmail.com)
 * Description: Core implementation for pipelined subsequence DTW algorithm
 *
 * Changes:     Author         Description
 *  03/31/2022  Elton Shih     Initial Commit
 */

`timescale 1ns / 1ps

module dtw_core #(
    parameter WIDTH         = 16,   // Data width
    parameter AXIS_WIDTH    = 32,   // AXI data width
    parameter SQG_SIZE      = 250,  // Squiggle size
    parameter REF_INIT      = 0,
    parameter REFMEM_PTR_WIDTH = 20
)(
    // Main DTW signals
    input   wire                    clk,
    input   wire                    rst,
    input   wire                    rs,

    input   wire [AXIS_WIDTH-1 : 0] ref_len,
    input   wire                    op_mode,            // Reference mode: 0, query mode: 1
    output  reg                     busy,               // Idle: 0, busy: 1

    // Ref interfacing signals
    input   wire [WIDTH-1 : 0]      ref_data,
    input   wire                    load_done,
    output  wire                    dtw_done,
    output  wire [REFMEM_PTR_WIDTH-1:0] addr_ref,

    // Src FIFO signals
    output  wire                    src_fifo_clear,     // Src FIFO Clear signal
    output  wire                    src_fifo_rden,      // Src FIFO Read enable
    input   wire                    src_fifo_empty,     // Src FIFO Empty
    input   wire [31:0]             src_fifo_data,      // Src FIFO Data

    // Sink FIFO signals
    output  reg                     sink_fifo_wren,     // Sink FIFO Write enable
    input   wire                    sink_fifo_full,     // Sink FIFO Full
    output  reg [31:0]              sink_fifo_data,     // Sink FIFO Data
    output  reg                     sink_fifo_last,     // Sink FIFO Last

    // debug signals
    output  wire [2:0]                              dbg_dtw_state,

    output  wire [31:0]             dbg_cycle_counter,
    output  wire [31:0]             dbg_nquery,
    output  wire [31:0]             dbg_curr_qid
);

/* ===============================
 * local parameters
 * =============================== */
// Operation mode
localparam
    MODE_NORMAL = 1'b0,
    MODE_LOAD_REF = 1'b1;

// FSM states
localparam [2:0] // n states
    DTW_IDLE = 0,
    DTW_REF_LOAD = 1,
    DTW_Q_INIT = 2,
    DTW_Q_RUN = 3,
    DTW_Q_DONE = 4;

/* ===============================
 * registers/wires
 * =============================== */
// DTW datapath signals
reg                 dp_rst;             // dp core reset
reg                 dp_running;         // dp core run enable
wire                dp_done;            // dp core done
reg  [31:0]         curr_qid;           // Current query id
wire [WIDTH-1:0]    curr_minval;        // Current minimum value
wire [31:0]         curr_position;      // Current best match position

// Ref signals
reg [REFMEM_PTR_WIDTH-1:0] addr_ref_reg;
reg dtw_done_reg;

// src fifo signals
reg src_fifo_clear_reg;
reg src_fifo_rden_reg;

// FSM state
reg [2:0] dtw_state;

// Others
reg [1:0] stall_counter;
reg [31:0] r_dbg_nquery;

/* ===============================
 * submodules
 * =============================== */
// DTW datapath
dtw_core_datapath #(
    .width      (WIDTH),
    .SQG_SIZE   (SQG_SIZE)
) inst_dtw_core_datapath (
    .clk            (clk),
    .rst            (dp_rst),
    .running        (dp_running),
    .Input_squiggle (src_fifo_data[15:0]),
    .Rword          (ref_data),
    .ref_len        (ref_len),
    .minval         (curr_minval),
    .position       (curr_position),
    .done           (dp_done),

    // debug
    .dbg_cycle_counter (dbg_cycle_counter)
);

/* ===============================
 * asynchronous logic
 * =============================== */
// assign src_fifo_clear = src_fifo_clear;
assign addr_ref = addr_ref_reg;
assign src_fifo_clear = src_fifo_clear_reg;
assign src_fifo_rden = src_fifo_rden_reg;
assign dtw_done = dtw_done_reg;

// debug
assign dbg_dtw_state = dtw_state;
assign dbg_addr_ref = addr_ref_reg;
assign dbg_nquery = r_dbg_nquery;
assign dbg_curr_qid = curr_qid;

/* ===============================
 * synchronous logic
 * =============================== */
// FSM State change
always @(posedge clk) begin
    if (rst) begin
        dtw_state <= DTW_IDLE;
    end else begin
        case (dtw_state)
        DTW_IDLE: begin
            if (rs) begin
                if (op_mode == MODE_NORMAL && load_done == 1) begin
                    dtw_state <= DTW_Q_INIT;
                end else if (op_mode == MODE_LOAD_REF && load_done == 0) begin
                    dtw_state <= DTW_REF_LOAD;
                end else begin
                    dtw_state <= DTW_IDLE;
                end
            end else begin
                dtw_state <= DTW_IDLE;
            end
        end
        DTW_REF_LOAD: begin
            if (load_done == 0) begin
                dtw_state <= DTW_REF_LOAD;
            end else begin
                dtw_state <= DTW_IDLE;
            end
        end
        DTW_Q_INIT: begin
            if (!src_fifo_empty) begin
                dtw_state <= DTW_Q_RUN;
            end else begin
                dtw_state <= DTW_Q_INIT;
            end
        end
        DTW_Q_RUN: begin
            if (!dp_done) begin
                dtw_state <= DTW_Q_RUN;
            end else begin
                dtw_state <= DTW_Q_DONE;
            end
        end
        DTW_Q_DONE: begin
            if (sink_fifo_full || stall_counter < 2'h3) begin
                dtw_state <= DTW_Q_DONE;
            end else begin
                dtw_state <= DTW_IDLE;
            end
        end

        default: begin
            dtw_state <= DTW_IDLE;
        end
        endcase
    end
end

// DTW FSM output
always @(posedge clk) begin
    case (dtw_state)
    DTW_IDLE: begin
        busy                <= 0;
        src_fifo_rden_reg   <= 0;
        sink_fifo_wren      <= 0;
        addr_ref_reg        <= 0;
        dp_rst              <= 1;
        dp_running          <= 0;
        stall_counter       <= 0;
        src_fifo_clear_reg  <= 1;
        sink_fifo_last      <= 0;
        curr_qid            <= 0;
        dtw_done_reg          <= 0;
    end
    DTW_REF_LOAD: begin
        // same as idle
        src_fifo_rden_reg   <= 0;
        sink_fifo_wren      <= 0;
        addr_ref_reg        <= 0;
        stall_counter       <= 0;
        sink_fifo_last      <= 0;
        curr_qid            <= 0;
        dp_rst              <= 1;
        dp_running          <= 0;
        dtw_done_reg          <= 0;

        // unique to this state
        busy                <= 1;
        src_fifo_clear_reg  <= 0;
    end
    DTW_Q_INIT: begin
        // same as idle
        sink_fifo_wren      <= 0;
        addr_ref_reg        <= 0;
        stall_counter       <= 0;
        sink_fifo_last      <= 0;
        dtw_done_reg          <= 0;

        // unique to this state
        busy                <= 1;
        src_fifo_rden_reg   <= 1;
        src_fifo_clear_reg  <= 0;
        dp_rst              <= 0;

        if (!src_fifo_empty) begin
            curr_qid        <= src_fifo_data;
            dp_running      <= 1;
        end else begin
            dp_running      <= 0;
        end
    end
    DTW_Q_RUN: begin
        // same as idle
        sink_fifo_wren          <= 0;
        stall_counter           <= 0;
        src_fifo_clear_reg      <= 0;
        dtw_done_reg              <= 0;

        // unique to this state
        busy                    <= 1;
        dp_rst                  <= 0;

        if (addr_ref_reg < SQG_SIZE) begin
            // Query loading
            if (!src_fifo_empty) begin
                addr_ref_reg       <= addr_ref_reg + 1;
                src_fifo_rden_reg   <= 1;
                dp_running      <= 1;
            end else begin
                src_fifo_rden_reg   <= 0;
                dp_running      <= 0;
            end
        end else begin
            // Query loaded
            addr_ref_reg           <= addr_ref_reg + 1;
            src_fifo_rden_reg       <= 0;
            dp_running          <= 1;
        end
    end
    DTW_Q_DONE: begin
        busy                <= 1;
        src_fifo_rden_reg   <= 0;
        dp_rst              <= 0;
        dp_running          <= 0;
        dtw_done_reg          <= 1;

        // Serialize output
        if (!sink_fifo_full) begin
            stall_counter <= stall_counter + 1;

            if (stall_counter == 0) begin
                sink_fifo_last  <= 0;
                sink_fifo_wren  <= 1;
                sink_fifo_data  <= curr_qid;
            end else if (stall_counter == 1) begin
                sink_fifo_last  <= 0;
                sink_fifo_wren  <= 1;
                sink_fifo_data  <= curr_position;
            end else if (stall_counter == 2) begin
                sink_fifo_last  <= 0;
                sink_fifo_wren  <= 1;
                sink_fifo_data  <= {16'b0, curr_minval};
            end else begin
                sink_fifo_last  <= 1;
                sink_fifo_wren  <= 0;
                sink_fifo_data  <= 0;
                r_dbg_nquery    <= r_dbg_nquery + 1;
            end
        end 
    end
    endcase
end
endmodule