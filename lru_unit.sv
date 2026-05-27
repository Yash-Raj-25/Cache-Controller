// =============================================================================
// lru_unit.sv
// 2-bit saturating LRU counter for 4-way set-associative cache.
//
// Policy:
//   Accessed way  → counter set to 3 (MRU).
//   All other ways → decrement by 1, saturating at 0.
//   Victim = lowest-index way with counter == 0.
// =============================================================================

import cache_pkg::*;

module lru_unit (
    input  logic clk,
    input  logic rst_n,

    // Update port
    input  logic                  update_en,
    input  logic [INDEX_BITS-1:0] set_index,
    input  logic [WAY_BITS-1:0]   accessed_way,

    // Query port (combinational)
    input  logic [INDEX_BITS-1:0] query_index,
    output logic [LRU_BITS-1:0]   lru_cnt [NUM_WAYS],
    output logic [WAY_BITS-1:0]   victim_way
);

    logic [LRU_BITS-1:0] cnt [NUM_SETS][NUM_WAYS];

    // Sequential update
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int s = 0; s < NUM_SETS; s++)
                for (int w = 0; w < NUM_WAYS; w++)
                    cnt[s][w] <= '0;
        end else if (update_en) begin
            for (int w = 0; w < NUM_WAYS; w++) begin
                if (w == int'(accessed_way))
                    cnt[set_index][w] <= {LRU_BITS{1'b1}};   // MRU = 3
                else
                    cnt[set_index][w] <= (cnt[set_index][w] == '0)
                                         ? '0
                                         : cnt[set_index][w] - 1'b1;  // saturate at 0
            end
        end
    end

    // Combinational read of counters for query_index
    always_comb begin
        for (int w = 0; w < NUM_WAYS; w++)
            lru_cnt[w] = cnt[query_index][w];
    end

    // Victim: lowest-index way with counter == 0
    always_comb begin
        victim_way = '0;
        for (int w = NUM_WAYS-1; w >= 0; w--)
            if (cnt[query_index][w] == '0)
                victim_way = WAY_BITS'(w);
    end

endmodule
