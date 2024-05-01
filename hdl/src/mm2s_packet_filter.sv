
module mm2s_packet_filter #(
    parameter AXIS_DATA_WIDTH       = 32,
    parameter FIFO_DATA_WIDTH       = 32,
    parameter AXIS_DEST_WIDTH       = 4,
    parameter NUM_FIFOS             = 2
) (
    // Input AXI Stream (from the Master mm2s port of the MCDMA)
    input  wire [AXIS_DATA_WIDTH - 1:0]     SRC_AXIS_tdata_in,
    input  wire [AXIS_DEST_WIDTH - 1:0]     SRC_AXIS_tdest_in,
    input  wire                             SRC_AXIS_tvalid_in,
    output wire                             SRC_AXIS_tready_out,

    // FIFO peripherals
    output  wire [NUM_FIFOS-1:0]                                fifo_wren_out,     // Sink FIFO Write enable
    input   wire [NUM_FIFOS-1:0]                                fifo_full_in,     // Sink FIFO Full
    output  wire [(FIFO_DATA_WIDTH*NUM_FIFOS) - 1:0]            fifo_data_out     // Sink FIFO Data
);

/* ===============================
 * registers / wires
 * =============================== */
wire [NUM_FIFOS-1:0]                        fifo_not_ready;

/* ===============================
 * asynchronous logic
 * =============================== */
assign SRC_AXIS_tready_out = !(|fifo_not_ready);

genvar i;
generate
    for (i = 0; i < NUM_FIFOS; i = i + 1) begin
        assign fifo_wren_out[i] = ((SRC_AXIS_tdest_in == i) && (SRC_AXIS_tvalid_in) && (!fifo_full_in[i])) ? 1'b1 : 1'b0;
        assign fifo_not_ready[i] = ((SRC_AXIS_tdest_in == i) && (SRC_AXIS_tvalid_in) && (fifo_full_in[i])) ? 1'b1 : 1'b0;
        assign fifo_data_out[(i+1)*FIFO_DATA_WIDTH-1:i*FIFO_DATA_WIDTH] = SRC_AXIS_tdata_in[FIFO_DATA_WIDTH-1:0];
    end
endgenerate

endmodule