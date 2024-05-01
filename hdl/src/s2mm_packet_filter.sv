
module s2mm_packet_filter #(
    parameter AXIS_DATA_WIDTH       = 32,
    parameter FIFO_DATA_WIDTH       = 32,
    parameter AXIS_KEEP_WIDTH       = (AXIS_DATA_WIDTH / 8),
    parameter AXIS_DEST_WIDTH       = 4,
    parameter NUM_CHANNELS          = 2
) (
    input  wire                             clk_in,
    input  wire                             rst_in,

    // Output AXI Stream (to the Slave s2mm port of the MCDMA)
    input  wire                             SINK_AXIS_tready_in,
    output wire [AXIS_DATA_WIDTH - 1:0]     SINK_AXIS_tdata_out,
    output wire [AXIS_DEST_WIDTH - 1:0]     SINK_AXIS_tdest_out,
    output wire [AXIS_KEEP_WIDTH - 1:0]     SINK_AXIS_tkeep_out,
    output wire                             SINK_AXIS_tlast_out,
    output wire                             SINK_AXIS_tuser_out,
    output wire                             SINK_AXIS_tvalid_out,

    // FIFO Peripherals
    input  wire [(FIFO_DATA_WIDTH*NUM_CHANNELS) - 1: 0]    fifo_data_in,
    input  wire [NUM_CHANNELS-1:0]          fifo_not_empty_in,
    input  wire [NUM_CHANNELS-1:0]          fifo_last_in,
    output wire [NUM_CHANNELS-1:0]          fifo_r_stb_out,

    output wire  [2:0]                       dbg_state
);
/* ===============================
 * local parameters
 * =============================== */

/* ===============================
 * registers / wires
 * =============================== */
reg  [NUM_CHANNELS-1:0]                     channel_en;
reg  [NUM_CHANNELS-1:0]                     channel_shift_reg;
reg  [AXIS_DATA_WIDTH-1:0]                  tdata_buf;
reg  [AXIS_DEST_WIDTH-1:0]                  tdest_buf;
reg                                         tlast_buf;

reg  [AXIS_DATA_WIDTH-1:0]                  tdata_reg;
reg  [AXIS_DEST_WIDTH-1:0]                  tdest_reg;
reg                                         tlast_reg;
reg                                         tvalid_reg;

/* ===============================
 * asynchronous logic
 * =============================== */
assign SINK_AXIS_tuser_out = 'd0;
assign SINK_AXIS_tkeep_out = {AXIS_KEEP_WIDTH{1'b1}};
assign SINK_AXIS_tdest_out = tdest_reg;
assign SINK_AXIS_tvalid_out = tvalid_reg;
assign SINK_AXIS_tlast_out = tlast_reg;
assign SINK_AXIS_tdata_out = tdata_reg;
assign fifo_r_stb_out[NUM_CHANNELS-1:0] = channel_en[NUM_CHANNELS-1:0];

// data out
always @(*) begin
    tdata_buf = {AXIS_DATA_WIDTH{1'b0}};
    for (integer i = 0; i < NUM_CHANNELS; i = i + 1) begin
        tdata_buf[AXIS_DATA_WIDTH-1:0] = (channel_en[i]) ? fifo_data_in[i*FIFO_DATA_WIDTH+:FIFO_DATA_WIDTH] : tdata_buf[AXIS_DATA_WIDTH-1:0];
    end
end

// tdest
always @(*) begin
    tdest_buf = {AXIS_DEST_WIDTH{1'b0}};
    for (integer i = 0; i < NUM_CHANNELS; i = i + 1) begin
        tdest_buf[AXIS_DEST_WIDTH-1:0] = (channel_en[i]) ? i[AXIS_DEST_WIDTH-1:0] : tdest_buf;
    end
end

// tlast
always @(*) begin
    tlast_buf = 1'b0;
    for (integer i = 0; i < NUM_CHANNELS; i = i + 1) begin
        tlast_buf = tlast_buf | (channel_en[i] & fifo_last_in[i]);
    end
end

/* ===============================
 * synchronous logic
 * =============================== */
// Channel Enable
reg [2:0] dbg_state_reg;
assign dbg_state = dbg_state_reg;
always @(posedge clk_in) begin
    if (rst_in) begin
        channel_en[NUM_CHANNELS-1:0] <= {NUM_CHANNELS{1'b0}};
        dbg_state_reg <= 'h0;
    end
    else begin
        if (!SINK_AXIS_tready_in) begin
            // Don't send anything when the DMA is not ready
            channel_en[NUM_CHANNELS-1:0] <= {NUM_CHANNELS{1'b0}};
            dbg_state_reg <= 'h1;
        end
        else if (^fifo_not_empty_in[NUM_CHANNELS-1:0]) begin
            // Only one fifo is ready for transfer
            channel_en[NUM_CHANNELS-1:0] <= fifo_not_empty_in[NUM_CHANNELS-1:0];
            dbg_state_reg <= 'h2;
        end
        else if (|fifo_not_empty_in[NUM_CHANNELS-1:0]) begin
            // Multiple fifos are ready for transfer
            // Rotate between fifos until only one fifo is left
            channel_en[NUM_CHANNELS-1:0] <= channel_shift_reg[NUM_CHANNELS-1:0];
            dbg_state_reg <= 'h3;
        end
        else begin
            // No fifos ready, disable all channels
            channel_en[NUM_CHANNELS-1:0] <= {NUM_CHANNELS{1'b0}};
            dbg_state_reg <= 'h4;
        end
    end
end

// Channel shift register
always @(posedge clk_in) begin
    if (rst_in) begin
        channel_shift_reg[NUM_CHANNELS-1:0] <= {{(NUM_CHANNELS-1){1'b0}}, 1'b1};
    end
    else begin
        if (SINK_AXIS_tready_in && |fifo_not_empty_in[NUM_CHANNELS-1:0]) begin
            if (channel_shift_reg[NUM_CHANNELS-1:0] == {1'b1, {(NUM_CHANNELS-1){1'b0}}}) begin
                channel_shift_reg[NUM_CHANNELS-1:0] <= {{(NUM_CHANNELS-1){1'b0}}, 1'b1};
            end
            else begin
                channel_shift_reg[NUM_CHANNELS-1:0] <= {{(NUM_CHANNELS-1){1'b0}}, 1'b1};
            end
        end
        else begin
            channel_shift_reg <= channel_shift_reg;
        end
    end
end

// AXIS output
always @(posedge clk_in) begin
    if (rst_in) begin
        tdest_reg <= {AXIS_DEST_WIDTH{1'b0}};
        tlast_reg <= 1'b0;
        tdata_reg <= {AXIS_DATA_WIDTH{1'b0}};
        tvalid_reg <= 1'b0;
    end
    else begin
        tdest_reg <= tdest_buf;
        tlast_reg <= tlast_buf;
        tdata_reg <= tdata_buf;
        tvalid_reg <= ^channel_en;
    end
end
endmodule