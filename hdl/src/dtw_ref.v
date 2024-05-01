`timescale 1ns / 1ps

module dtw_ref #(
    parameter WIDTH         = 16,   // Data width
    parameter AXIS_WIDTH    = 32,   // AXI data width
    parameter SQG_SIZE      = 250,  // Squiggle size
    parameter REF_INIT      = 0,
    parameter REFMEM_PTR_WIDTH = 20
)(
    // Main control signals
    input   wire                    clk,
    input   wire                    rst,
    input   wire                    rs,

    input   wire [REFMEM_PTR_WIDTH-1 : 0] ref_len,
    input   wire                    op_mode,            // Reference mode: 0, query mode: 1
    output  reg                     busy,               // Idle: 0, busy: 1

    // dtw core interfacing signals
    input  wire                                 dtw_done,
    input  wire [REFMEM_PTR_WIDTH-1:0]          dtw_read_addr,
    output wire [WIDTH-1:0]                     ref_data_out,
    output wire                                 load_done,

    // Src FIFO signals
    output  wire                    src_fifo_clear_out,     // Src FIFO Clear signal
    output  reg                     src_fifo_rden_out,      // Src FIFO Read enable
    input   wire                    src_fifo_empty,     // Src FIFO Empty
    input   wire [WIDTH-1:0]             src_fifo_data_in,      // Src FIFO Data

    // debug signals
    output  wire [1:0]                              dbg_ref_state,
    output  wire [REFMEM_PTR_WIDTH-1:0]             dbg_addr_ref
);

/* ===============================
 * local parameters
 * =============================== */
// Operation mode
localparam
    MODE_NORMAL = 1'b0,
    MODE_LOAD_REF = 1'b1;

localparam [1:0]
    REF_IDLE = 0,
    REF_LOAD = 1,
    REF_DTW_READ = 2;

/* ===============================
 * registers/wires
 * =============================== */
// state
reg [1:0] ref_state;

// module registers
reg [REFMEM_PTR_WIDTH-1:0] ref_addr_reg;
reg dtw_read_addr_mode_reg;

// ref mem nodes
wire [REFMEM_PTR_WIDTH-1:0] addr_node;
reg                 ref_wren_node;           // Write enable for refmem

/* ===============================
 * submodules
 * =============================== */
// Reference memory
dtw_core_ref_mem #(
    .width      (WIDTH),
    .initalize  (REF_INIT),
    .ptrWid     (REFMEM_PTR_WIDTH)
) inst_dtw_core_ref_mem (
    .clk            (clk),

    .wen          (ref_wren_node),
    .addr         (addr_node[REFMEM_PTR_WIDTH-1:0]),
    .din          (src_fifo_data_in[WIDTH-1:0]),
    .dout         (ref_data_out)
);

/* ===============================
 * asynchronous logic
 * =============================== */
assign addr_node = (dtw_read_addr_mode_reg) ? dtw_read_addr : ref_addr_reg;

// debug
assign dbg_ref_state = ref_state;
assign dbg_addr_ref = addr_node;


/* ===============================
 * synchronous logic
 * =============================== */
// Reference Load FSM
always @(posedge clk) begin
    if (rst) begin
        ref_state <= REF_IDLE;
    end else begin
        case (ref_state)
        REF_IDLE: begin
            if (rs) begin
                if (op_mode == MODE_NORMAL && load_done == 1) begin
                    ref_state <= REF_DTW_READ;
                end else if (op_mode == MODE_LOAD_REF && load_done == 0) begin
                    ref_state <= REF_LOAD;
                end else begin
                    ref_state <= REF_IDLE;
                end
            end else begin
                ref_state <= REF_IDLE;
            end
        end

        REF_LOAD: begin
            if (ref_addr_reg[REFMEM_PTR_WIDTH-1:0] < ref_len[REFMEM_PTR_WIDTH-1:0]) begin
                ref_state <= REF_LOAD;
            end else begin
                ref_state <= REF_IDLE;
            end
        end

        REF_DTW_READ: begin
            if (dtw_done) begin
                ref_state <= REF_IDLE;
            end else begin
                ref_state <= REF_DTW_READ;
            end
        end

        default: begin
            ref_state <= REF_IDLE;
        end
        endcase
    end
end

// Reference Load FSM Outputs
always @(posedge clk) begin
    case (ref_state)
    REF_IDLE: begin
        src_fifo_rden_out   <= 0;
        ref_addr_reg       <= 0;
        ref_wren_node            <= 0;
        src_fifo_clear_out  <= 1;
        busy <= 0;
    end

    REF_LOAD: begin
        src_fifo_rden_out   <= 1;
        src_fifo_clear_out  <= 0;
        busy <= 1;

        // When the output data from the fifo is valid
        if (!src_fifo_empty && src_fifo_rden_out) begin
            ref_addr_reg   <= ref_addr_reg + 1; // next address to read from
            ref_wren_node        <= 1;
        end else begin
            ref_wren_node        <= 0;
        end
    end

    REF_DTW_READ: begin
        src_fifo_rden_out   <= 0;
        ref_wren_node            <= 0;
        src_fifo_clear_out  <= 0;
        ref_addr_reg       <= 0;
        busy <= 1;

    end

    default: begin
        src_fifo_rden_out   <= 0;
        ref_addr_reg       <= 0;
        ref_wren_node            <= 0;
        src_fifo_clear_out  <= 0;
        busy <= 0;
    end
    endcase
end

// Ref Load FSM Output
always @(posedge clk) begin
    if (rst) begin
        load_done <= 1'b0;
    end else begin
        case(ref_state)
            REF_LOAD: begin
                if (ref_addr_reg[REFMEM_PTR_WIDTH-1:0] < ref_len[REFMEM_PTR_WIDTH-1:0]) begin
                    load_done <= 1'b0;
                end else begin
                    load_done <= 1'b1;
                end
            end

            default: begin
                load_done <= load_done;
            end
        endcase
    end
end

// addr mode FSM output
always @(posedge clk) begin
    if (rst) begin
        dtw_read_addr_mode_reg  <= 1;
    end else begin
        case(ref_state)
            REF_IDLE: begin
                if (op_mode == MODE_NORMAL && load_done == 1) begin
                    dtw_read_addr_mode_reg <= 1'b1;
                end else begin
                    dtw_read_addr_mode_reg <= 1'b0;
                end
            end

            REF_LOAD: begin
                if (ref_addr_reg[REFMEM_PTR_WIDTH-1:0] < ref_len[REFMEM_PTR_WIDTH-1:0]) begin
                    dtw_read_addr_mode_reg <= 1'b0;
                end else begin
                    dtw_read_addr_mode_reg <= 1'b1;
                end
            end

            REF_DTW_READ: begin
                dtw_read_addr_mode_reg <= 1'b1;
            end

            default: begin
                dtw_read_addr_mode_reg <= 1'b0;
            end
        endcase
    end
end

endmodule