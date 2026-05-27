// =============================================================================
// tag_data_array.sv
// Combined tag metadata and data storage for the 4-way set-associative cache.
//
// Tag array : combinational read (all 4 ways in one cycle for parallel compare).
// Data array: 1-cycle registered read for normal CPU access;
//             combinational full-line read for write-back to memory.
// =============================================================================

import cache_pkg::*;

module tag_data_array (
    input  logic clk,
    input  logic rst_n,

    // ---- Tag array read (combinational, all 4 ways) -------------------------
    input  logic [INDEX_BITS-1:0]  ta_rd_index,
    output line_meta_t             ta_rd_out [NUM_WAYS],

    // ---- Tag array write (one way, registered) ------------------------------
    input  logic                   ta_wr_en,
    input  logic [INDEX_BITS-1:0]  ta_wr_index,
    input  logic [WAY_BITS-1:0]    ta_wr_way,
    input  line_meta_t             ta_wr_data,

    // ---- Data array: CPU word read (registered, 1-cycle latency) ------------
    input  logic                        da_rd_en,
    input  logic [INDEX_BITS-1:0]       da_rd_index,
    input  logic [WAY_BITS-1:0]         da_rd_way,
    input  logic [OFFSET_BITS-1:0]      da_rd_offset,
    output logic [DATA_WIDTH-1:0]       da_rd_word,

    // ---- Data array: full-line read (combinational, for write-back) ---------
    input  logic [INDEX_BITS-1:0]       da_line_index,
    input  logic [WAY_BITS-1:0]         da_line_way,
    output logic [DATA_WIDTH*LINE_WORDS-1:0] da_line_out,

    // ---- Data array: CPU word write (byte-enable, registered) ---------------
    input  logic                        da_cpu_wr_en,
    input  logic [INDEX_BITS-1:0]       da_cpu_wr_index,
    input  logic [WAY_BITS-1:0]         da_cpu_wr_way,
    input  logic [OFFSET_BITS-1:0]      da_cpu_wr_offset,
    input  logic [DATA_WIDTH-1:0]       da_cpu_wr_data,
    input  logic [(DATA_WIDTH/8)-1:0]   da_cpu_wr_be,

    // ---- Data array: full-line fill (from memory, registered) ---------------
    input  logic                        da_fill_en,
    input  logic [INDEX_BITS-1:0]       da_fill_index,
    input  logic [WAY_BITS-1:0]         da_fill_way,
    input  logic [DATA_WIDTH*LINE_WORDS-1:0] da_fill_data
);

    // -------------------------------------------------------------------------
    // Storage
    // -------------------------------------------------------------------------
    line_meta_t  tag_mem  [NUM_SETS][NUM_WAYS];
    logic [7:0]  data_mem [NUM_SETS][NUM_WAYS][LINE_WORDS][DATA_WIDTH/8];

    localparam WORD_SEL_BITS = $clog2(LINE_WORDS);
    localparam BYTE_SEL_BITS = $clog2(DATA_WIDTH/8);

    // -------------------------------------------------------------------------
    // Tag array: async read (all ways simultaneously)
    // -------------------------------------------------------------------------
    always_comb
        for (int w = 0; w < NUM_WAYS; w++)
            ta_rd_out[w] = tag_mem[ta_rd_index][w];

    // Tag array: sync write
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int s = 0; s < NUM_SETS; s++)
                for (int w = 0; w < NUM_WAYS; w++) begin
                    tag_mem[s][w].valid <= 1'b0;
                    tag_mem[s][w].dirty <= 1'b0;
                    tag_mem[s][w].tag   <= '0;
                    tag_mem[s][w].lru   <= '0;
                end
        end else if (ta_wr_en) begin
            tag_mem[ta_wr_index][ta_wr_way] <= ta_wr_data;
        end
    end

    // -------------------------------------------------------------------------
    // Data array: registered word read (1-cycle latency)
    // -------------------------------------------------------------------------
    logic [WORD_SEL_BITS-1:0] rd_word_sel;
    assign rd_word_sel = da_rd_offset[OFFSET_BITS-1 : BYTE_SEL_BITS];

    always_ff @(posedge clk) begin
        if (da_rd_en)
            for (int b = 0; b < DATA_WIDTH/8; b++)
                da_rd_word[b*8 +: 8] <= data_mem[da_rd_index][da_rd_way][rd_word_sel][b];
    end

    // -------------------------------------------------------------------------
    // Data array: combinational full-line read (for write-back)
    // -------------------------------------------------------------------------
    always_comb begin
        for (int w = 0; w < LINE_WORDS; w++)
            for (int b = 0; b < DATA_WIDTH/8; b++)
                da_line_out[(w*(DATA_WIDTH) + b*8) +: 8] =
                    data_mem[da_line_index][da_line_way][w][b];
    end

    // -------------------------------------------------------------------------
    // Data array: CPU byte-enable word write
    // -------------------------------------------------------------------------
    logic [WORD_SEL_BITS-1:0] cpu_wr_word_sel;
    assign cpu_wr_word_sel = da_cpu_wr_offset[OFFSET_BITS-1 : BYTE_SEL_BITS];

    always_ff @(posedge clk) begin
        if (da_cpu_wr_en)
            for (int b = 0; b < DATA_WIDTH/8; b++)
                if (da_cpu_wr_be[b])
                    data_mem[da_cpu_wr_index][da_cpu_wr_way][cpu_wr_word_sel][b]
                        <= da_cpu_wr_data[b*8 +: 8];
    end

    // -------------------------------------------------------------------------
    // Data array: full-line fill from memory
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (da_fill_en)
            for (int w = 0; w < LINE_WORDS; w++)
                for (int b = 0; b < DATA_WIDTH/8; b++)
                    data_mem[da_fill_index][da_fill_way][w][b]
                        <= da_fill_data[(w*(DATA_WIDTH) + b*8) +: 8];
    end

endmodule
