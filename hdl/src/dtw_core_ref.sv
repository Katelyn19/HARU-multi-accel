
`timescale 1ps / 1ps

`define MAJOR_VERSION       1
`define MINOR_VERSION       0
`define REVISION            0

`define MAJOR_RANGE         31:28
`define MINOR_RANGE         27:20
`define REVISION_RANGE      19:16
`define VERSION_PAD_RANGE   15:0

module dtw_core_ref #(
    parameter DATA_WIDTH                            = 16,   // Data width
    parameter ADDR_WIDTH                            = 32,   // AXI data width
    parameter REF_INIT                              = 0,
    parameter REFMEM_PTR_WIDTH                      = 20
) (
    // Main Module signals
    input   wire                                    clk_in,
    input   wire                                    rst_in,
    input   wire                                    rs_in,                  // Run: 1, Stop: 0
    input   wire                                    op_mode_in,
    input   wire [REFMEM_PTR_WIDTH-1: 0]            ref_len_in,
    output  reg                                     busy_out,               // Idle: 0, busy: 1
    output  wire                                    ref_load_done_out,

    // Src FIFO signals
    output  wire                                    src_fifo_clear_out,     // Src FIFO Clear signal
    output  reg                                     src_fifo_rden_out,      // Src FIFO Read enable
    input   wire                                    src_fifo_empty_in,     // Src FIFO Empty
    input   wire [DATA_WIDTH-1:0]                   src_fifo_data_in,      // Src FIFO Data

    // Ref mem signals
    input   wire [REFMEM_PTR_WIDTH-1: 0]            ref_addr_0_in,
    input   wire [REFMEM_PTR_WIDTH-1: 0]            ref_addr_1_in,
    output  reg  [DATA_WIDTH-1:0]                   ref_data_0_out,
    output  reg  [DATA_WIDTH-1:0]                   ref_data_1_out,

    // Debug signals
    output  wire [1:0]                              dbg_state,
    output  wire [REFMEM_PTR_WIDTH-1: 0]            dbg_addr_ref,
    output  wire                                    dbg_wren_ref
);
/* ===============================
 * local parameters
 * =============================== */
// Operation mode
localparam
    MODE_DTW_READ = 1'b0,
    MODE_LOAD_REF = 1'b1;

// FSM states
localparam [1:0] // n states
    IDLE = 0,
    REF_LOAD = 1,
    DTW_READ = 2;

/* ===============================
 * registers/wires
 * =============================== */
reg                                         ref_load_done;
reg                                         r_src_fifo_clear;
reg                                         wren_ref_node_0;           // Write enable for refmem
reg [REFMEM_PTR_WIDTH-1:0]                  ref_addr_node_0;

// FSM state
reg [1:0] r_state;


/* ===============================
 * submodules
 * =============================== */
// Reference memory
dtw_core_ref_mem #(
    .width      (DATA_WIDTH),
    .initalize  (REF_INIT),
    .ptrWid     (REFMEM_PTR_WIDTH)
) inst_dtw_core_ref_mem (
    .clk            (clk_in),

    .wen_a          (wren_ref_node_0),
    .addr_a         (ref_addr_node_0[REFMEM_PTR_WIDTH-1:0]),
    .din_a          (src_fifo_data_in),
    .dout_a         (ref_data_0_out),

    .wen_b          (1'b0),
    .addr_b         (ref_addr_1_in[REFMEM_PTR_WIDTH-1:0]),
    .din_b          ('d0),
    .dout_b         (ref_data_1_out)
);

/* ===============================
 * asynchronous logic
 * =============================== */
assign dbg_state = r_state;
assign dbg_addr_ref = ref_addr_node_0;
assign dbg_wren_ref = wren_ref_node_0;

assign ref_load_done_out = ref_load_done;
assign src_fifo_clear_out = r_src_fifo_clear;

/* ===============================
 * synchronous logic
 * =============================== */
// FSM State change
always @(posedge clk_in) begin
    if (rst_in) begin
        r_state <= IDLE;
    end else begin
        case (r_state)
        IDLE: begin
            if (rs_in) begin
                if (op_mode_in == MODE_DTW_READ) begin
                    r_state <= DTW_READ;
                end else if (op_mode_in == MODE_LOAD_REF && ref_load_done == 0) begin
                    r_state <= REF_LOAD;
                end else begin
                    r_state <= IDLE;
                end
            end else begin
                r_state <= IDLE;
            end
        end

        REF_LOAD: begin
            if (ref_addr_node_0[REFMEM_PTR_WIDTH-1:0] < ref_len_in[REFMEM_PTR_WIDTH-1:0]) begin
                r_state <= REF_LOAD;
            end else begin
                r_state <= IDLE;
            end
        end

        DTW_READ: begin
            if (op_mode_in == MODE_LOAD_REF) begin
                r_state <= REF_LOAD;
            end
        end
        endcase
    end
end

// FSM output
always @(posedge clk_in) begin
    case (r_state)
    IDLE: begin
        busy_out <= 1'b0;
        src_fifo_rden_out <= 1'b0;
        wren_ref_node_0 <= 1'b0;
        r_src_fifo_clear <= 1'b1;

        if (op_mode_in == MODE_DTW_READ) begin
            ref_addr_node_0 <= ref_addr_0_in;
        end else begin
            ref_addr_node_0 <= 'd0;
        end
    end

    REF_LOAD: begin
        busy_out <= 1'b1;
        src_fifo_rden_out <= 1'b1;
        r_src_fifo_clear <= 1'b0;

        if (!src_fifo_empty_in && src_fifo_rden_out) begin
            ref_addr_node_0 <= ref_addr_node_0 + 1'b1;
            wren_ref_node_0 <= 1'b1;
        end else begin
            wren_ref_node_0 <= 1'b0;
            ref_addr_node_0 <= ref_addr_node_0;
        end

        if (!(src_fifo_empty_in) && (ref_addr_node_0[REFMEM_PTR_WIDTH-1:0] == (ref_len_in[REFMEM_PTR_WIDTH-1:0] - 1'b1))) begin
            ref_load_done <= 1'b1;
        end else begin
            ref_load_done <= 1'b0;
        end
    end

    DTW_READ: begin
        busy_out <= 1'b1;
        src_fifo_rden_out <= 1'b0;
        wren_ref_node_0 <= 1'b0;
        r_src_fifo_clear <= 1'b0;

        if (op_mode_in == MODE_LOAD_REF) begin
            ref_addr_node_0 <= 'd0;
        end else begin
            ref_addr_node_0 <= ref_addr_0_in;
        end
    end
    endcase
end
endmodule;