
module mm2s_packet_filter #(
    parameter AXIS_DATA_WIDTH       = 32,
    parameter FIFO_DATA_WIDTH       = 32,
    parameter AXIS_DEST_WIDTH       = 4,
    parameter NUM_FIFOS             = 2
) (
    // Input AXI Stream (from the Master mm2s port of the MCDMA)
    input  wire [AXIS_DATA_WIDTH - 1:0]     SRC_AXIS_tdata,
    input  wire [AXIS_DEST_WIDTH - 1:0]     SRC_AXIS_tdest,
    input  wire                             SRC_AXIS_tvalid,
    output wire                             SRC_AXIS_tready,

    // FIFO peripherals
    output  wire [NUM_FIFOS-1:0]                    fifo_wren,     // Sink FIFO Write enable
    input   wire [NUM_FIFOS-1:0]                   fifo_full,     // Sink FIFO Full
    output  wire [FIFO_DATA_WIDTH-1:0]              fifo_data     // Sink FIFO Data
);

/* ===============================
 * registers / wires
 * =============================== */
wire [NUM_FIFOS-1:0]                        fifo_not_ready;

/* ===============================
 * asynchronous logic
 * =============================== */
assign SRC_AXIS_tready = !(|fifo_not_ready);

genvar i;
generate
    for (i = 0; i < NUM_FIFOS; i = i + 1) begin
        assign fifo_wren[i] = ((SRC_AXIS_tdest == i) && (SRC_AXIS_tvalid) && (!fifo_full[i])) ? 1'b1 : 1'b0;
        assign fifo_not_ready[i] = ((SRC_AXIS_tdest == i) && (SRC_AXIS_tvalid) && (fifo_full[i])) ? 1'b1 : 1'b0;
        assign fifo_data[i] = SRC_AXIS_tdata[FIFO_DATA_WIDTH-1:0];
    end
endgenerate

endmodule