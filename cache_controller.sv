// =============================================================================
// cache_controller.sv  — FIXED VERSION
// 4-way set-associative cache controller — LRU replacement, no MESI.
//
// DMA coherence handled by:
//   (1) Software flush     : CPU asserts flush_req  before DMA reads memory
//   (2) Software invalidate: CPU asserts inv_req    after  DMA writes memory
//   (3) Non-cacheable regions: MAT bypass — accesses go straight to memory
//
// FSM states (local enum, 3-bit):
//   ST_LC_IDLE        0 — wait for CPU request, flush, or invalidate command
//   ST_LC_TAG_CHECK   1 — parallel 4-way tag comparison
//   ST_LC_WRITE_BACK  2 — flush dirty victim line to memory before eviction
//   ST_LC_ALLOCATE    3 — send read request to memory for missing line
//   ST_LC_MEM_WAIT    4 — wait for memory to return fill data
//   ST_LC_FLUSH       5 — iterate all sets/ways, write back every dirty line
//   ST_LC_INVALIDATE  6 — iterate all sets/ways, clear all valid bits
//   ST_LC_READ_HIT    7 — extra pipeline stage: return registered SRAM word
//
// ---- Bug fixes vs. original -------------------------------------------------
//  FIX-1  req_index/req_tag decoded from latched req_r.addr in multi-cycle
//         states — live cpu_req can change while stall is asserted.
//  FIX-2  ta_rd_index muxed: drive iter_set_r during FLUSH / INVALIDATE so
//         the correct set's tags are read (original always drove req_index).
//  FIX-3  FLUSH counter advance gated: stays on a dirty entry until
//         mem_rsp.valid — original advanced every cycle, skipping entries.
//  FIX-4  ALLOCATE moves to MEM_WAIT unconditionally on the next clock.
//         Original stayed in ALLOCATE until mem_rsp.valid, which is wrong
//         for a multi-cycle memory (the response arrives in MEM_WAIT).
//  FIX-5  nc_wait_r flag: MEM_WAIT distinguishes non-cacheable reads from
//         cache-fill so NC reads don't install spurious tags / dirty bits.
//  FIX-6  Read-hit pipeline: TAG_CHECK issues da_rd_en and moves to
//         ST_LC_READ_HIT; da_rd_word is returned one cycle later once the
//         registered SRAM output has settled.
//  FIX-7  Removed 'logic advance' declaration inside always_ff (illegal in
//         Verilog-2001 / many EDA tools); replaced with an equivalent
//         combinational wire 'iter_advance'.
// =============================================================================

import cache_pkg::*;

module cache_controller (
    input  logic clk,
    input  logic rst_n,

    // CPU interface
    input  cpu_req_t  cpu_req,
    output cpu_rsp_t  cpu_rsp,

    // DMA coherence control (driven by CPU SW before/after DMA)
    input  logic      flush_req,
    input  logic      inv_req,
    output logic      flush_done,
    output logic      inv_done,

    // Memory interface
    output mem_req_t  mem_req,
    input  mem_rsp_t  mem_rsp
);

    // =========================================================================
    // FSM state — local 3-bit enum (adds ST_LC_READ_HIT = 3'b111)
    // =========================================================================
    typedef enum logic [2:0] {
        ST_LC_IDLE       = 3'b000,
        ST_LC_TAG_CHECK  = 3'b001,
        ST_LC_WRITE_BACK = 3'b010,
        ST_LC_ALLOCATE   = 3'b011,
        ST_LC_MEM_WAIT   = 3'b100,
        ST_LC_FLUSH      = 3'b101,
        ST_LC_INVALIDATE = 3'b110,
        ST_LC_READ_HIT   = 3'b111   // FIX-6
    } lc_state_t;

    lc_state_t state_r, state_next;

    // =========================================================================
    // Registered state
    // =========================================================================
    cpu_req_t             req_r;           // latched CPU request
    logic [WAY_BITS-1:0]  victim_way_r;    // latched LRU victim
    logic                 nc_wait_r;       // FIX-5: non-cacheable MEM_WAIT flag
    logic [INDEX_BITS-1:0] iter_set_r;     // FLUSH/INVALIDATE set counter
    logic [WAY_BITS-1:0]  iter_way_r;      // FLUSH/INVALIDATE way counter
    logic [INDEX_BITS-1:0] flush_index_r;  // line-read index (victim or iter)
    logic [WAY_BITS-1:0]  flush_way_r;     // line-read way   (victim or iter)

    // =========================================================================
    // Non-cacheable check (combinational on live cpu_req.addr)
    // =========================================================================
    logic non_cacheable;

    mem_attr_table u_mat (
        .addr          (cpu_req.addr),
        .non_cacheable (non_cacheable)
    );

    // =========================================================================
    // Address decomposition
    // FIX-1: use live cpu_req in IDLE/TAG_CHECK; latched req_r elsewhere.
    // =========================================================================
    cpu_req_t active_req;
    always_comb begin
        if (state_r == ST_LC_IDLE || state_r == ST_LC_TAG_CHECK)
            active_req = cpu_req;
        else
            active_req = req_r;
    end

    logic [TAG_BITS-1:0]    req_tag;
    logic [INDEX_BITS-1:0]  req_index;
    logic [OFFSET_BITS-1:0] req_offset;

    assign req_tag    = active_req.addr[ADDR_WIDTH-1        : INDEX_BITS+OFFSET_BITS];
    assign req_index  = active_req.addr[INDEX_BITS+OFFSET_BITS-1 : OFFSET_BITS];
    assign req_offset = active_req.addr[OFFSET_BITS-1       : 0];

    // =========================================================================
    // Sub-module control wires
    // =========================================================================
    line_meta_t  ta_rd_out [NUM_WAYS];
    logic        ta_wr_en;
    logic [INDEX_BITS-1:0] ta_wr_index;
    logic [WAY_BITS-1:0]   ta_wr_way;
    line_meta_t  ta_wr_data;

    // FIX-2: mux tag-read index so FLUSH/INVALIDATE read iter_set_r
    logic [INDEX_BITS-1:0] ta_rd_index_mux;
    always_comb begin
        if (state_r == ST_LC_FLUSH || state_r == ST_LC_INVALIDATE)
            ta_rd_index_mux = iter_set_r;
        else
            ta_rd_index_mux = req_index;
    end

    logic                              da_rd_en;
    logic [DATA_WIDTH-1:0]             da_rd_word;
    logic [DATA_WIDTH*LINE_WORDS-1:0]  da_line_out;
    logic                              da_cpu_wr_en;
    logic                              da_fill_en;
    logic [DATA_WIDTH*LINE_WORDS-1:0]  da_fill_data;

    // =========================================================================
    // Hit detection (combinational, parallel across 4 ways)
    // =========================================================================
    logic [WAY_BITS-1:0] hit_way;
    logic                hit;

    always_comb begin
        hit     = 1'b0;
        hit_way = 2'b00;
        for (int w = 0; w < NUM_WAYS; w++) begin
            if (ta_rd_out[w].valid && ta_rd_out[w].tag == req_tag) begin
                hit     = 1'b1;
                hit_way = WAY_BITS'(w);
            end
        end
    end

    // =========================================================================
    // LRU unit
    // =========================================================================
    logic [LRU_BITS-1:0] lru_cnt [NUM_WAYS];
    logic [WAY_BITS-1:0] lru_victim;
    logic                lru_update_en;
    logic [WAY_BITS-1:0] lru_accessed_way;

    lru_unit u_lru (
        .clk          (clk),
        .rst_n        (rst_n),
        .update_en    (lru_update_en),
        .set_index    (req_index),
        .accessed_way (lru_accessed_way),
        .query_index  (req_index),
        .lru_cnt      (lru_cnt),
        .victim_way   (lru_victim)
    );

    // =========================================================================
    // Tag / data array
    // =========================================================================
    tag_data_array u_mem (
        .clk              (clk),
        .rst_n            (rst_n),
        .ta_rd_index      (ta_rd_index_mux),          // FIX-2
        .ta_rd_out        (ta_rd_out),
        .ta_wr_en         (ta_wr_en),
        .ta_wr_index      (ta_wr_index),
        .ta_wr_way        (ta_wr_way),
        .ta_wr_data       (ta_wr_data),
        .da_rd_en         (da_rd_en),
        .da_rd_index      (req_index),
        .da_rd_way        (hit_way),
        .da_rd_offset     (req_offset),
        .da_rd_word       (da_rd_word),
        .da_line_index    (flush_index_r),
        .da_line_way      (flush_way_r),
        .da_line_out      (da_line_out),
        .da_cpu_wr_en     (da_cpu_wr_en),
        .da_cpu_wr_index  (req_index),
        .da_cpu_wr_way    (hit_way),
        .da_cpu_wr_offset (req_offset),
        .da_cpu_wr_data   (active_req.wdata),
        .da_cpu_wr_be     (active_req.be),
        .da_fill_en       (da_fill_en),
        .da_fill_index    (req_index),
        .da_fill_way      (victim_way_r),
        .da_fill_data     (da_fill_data)
    );

    // =========================================================================
    // FIX-7: iter_advance computed combinationally (no variable decl in FF)
    // 1 when the FLUSH/INVALIDATE pointer should step to the next entry.
    // =========================================================================
    logic iter_advance;
    always_comb begin
        if (state_r == ST_LC_INVALIDATE)
            iter_advance = 1'b1;   // always safe — no memory wait
        else if (state_r == ST_LC_FLUSH)
            iter_advance = !(ta_rd_out[iter_way_r].valid &&
                             ta_rd_out[iter_way_r].dirty)
                           || mem_rsp.valid;   // FIX-3
        else
            iter_advance = 1'b0;
    end

    // =========================================================================
    // Sequential logic
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r       <= ST_LC_IDLE;
            req_r         <= '0;
            victim_way_r  <= '0;
            nc_wait_r     <= 1'b0;
            flush_index_r <= '0;
            flush_way_r   <= '0;
            iter_set_r    <= '0;
            iter_way_r    <= '0;
        end else begin
            state_r <= state_next;

            // Latch CPU request when entering a cacheable transaction
            if (state_r == ST_LC_IDLE && cpu_req.valid && !flush_req && !inv_req)
                req_r <= cpu_req;

            // Latch victim way on miss
            if (state_r == ST_LC_TAG_CHECK && !hit)
                victim_way_r <= lru_victim;

            // FIX-5: mark NC read
            if (state_r == ST_LC_IDLE && cpu_req.valid && non_cacheable && !cpu_req.we)
                nc_wait_r <= 1'b1;
            else if (state_next == ST_LC_IDLE)
                nc_wait_r <= 1'b0;

            // Line-read pointer: victim during normal ops, iter during flush/inv
            if (state_r == ST_LC_FLUSH || state_r == ST_LC_INVALIDATE) begin
                flush_index_r <= iter_set_r;
                flush_way_r   <= iter_way_r;
            end else begin
                flush_index_r <= req_index;
                flush_way_r   <= victim_way_r;
            end

            // Iteration counters (FIX-3 / FIX-7)
            if (state_next == ST_LC_FLUSH || state_next == ST_LC_INVALIDATE) begin
                if (state_r == ST_LC_IDLE) begin
                    iter_set_r <= '0;
                    iter_way_r <= '0;
                end else if (iter_advance) begin
                    if (iter_way_r == WAY_BITS'(NUM_WAYS-1)) begin
                        iter_way_r <= '0;
                        iter_set_r <= iter_set_r + 1'b1;
                    end else begin
                        iter_way_r <= iter_way_r + 1'b1;
                    end
                end
            end
        end
    end

    // =========================================================================
    // FSM — combinational next-state and output
    // =========================================================================
    always_comb begin
        // ---- Defaults --------------------------------------------------------
        state_next       = state_r;

        cpu_rsp.valid    = 1'b0;
        cpu_rsp.rdata    = '0;
        cpu_rsp.stall    = 1'b0;

        ta_wr_en         = 1'b0;
        ta_wr_index      = req_index;
        ta_wr_way        = victim_way_r;
        ta_wr_data       = '0;

        da_rd_en         = 1'b0;
        da_cpu_wr_en     = 1'b0;
        da_fill_en       = 1'b0;
        da_fill_data     = '0;

        lru_update_en    = 1'b0;
        lru_accessed_way = hit_way;

        mem_req.valid    = 1'b0;
        mem_req.we       = 1'b0;
        mem_req.addr     = '0;
        mem_req.wdata    = '0;

        flush_done       = 1'b0;
        inv_done         = 1'b0;

        // ======================================================================
        unique case (state_r)

        // ----------------------------------------------------------------------
        ST_LC_IDLE: begin
            if (flush_req) begin
                state_next    = ST_LC_FLUSH;
                cpu_rsp.stall = 1'b1;
            end
            else if (inv_req) begin
                state_next    = ST_LC_INVALIDATE;
                cpu_rsp.stall = 1'b1;
            end
            else if (cpu_req.valid) begin
                if (non_cacheable) begin
                    // Non-cacheable bypass
                    mem_req.valid = 1'b1;
                    mem_req.we    = cpu_req.we;
                    mem_req.addr  = cpu_req.addr;
                    if (cpu_req.we) begin
                        mem_req.wdata[0 +: DATA_WIDTH] = cpu_req.wdata;
                        cpu_rsp.valid = 1'b1;    // posted write — ack immediately
                    end else begin
                        cpu_rsp.stall = 1'b1;
                        state_next    = ST_LC_MEM_WAIT;
                    end
                end else begin
                    cpu_rsp.stall = 1'b1;
                    state_next    = ST_LC_TAG_CHECK;
                end
            end
        end

        // ----------------------------------------------------------------------
        ST_LC_TAG_CHECK: begin
            cpu_rsp.stall = 1'b1;
            if (hit) begin
                if (!req_r.we) begin
                    // Read hit — issue SRAM read; return word in READ_HIT (FIX-6)
                    da_rd_en   = 1'b1;
                    state_next = ST_LC_READ_HIT;
                end else begin
                    // Write hit
                    da_cpu_wr_en      = 1'b1;
                    ta_wr_en          = 1'b1;
                    ta_wr_index       = req_index;
                    ta_wr_way         = hit_way;
                    ta_wr_data        = ta_rd_out[hit_way];
                    ta_wr_data.dirty  = 1'b1;
                    cpu_rsp.valid     = 1'b1;
                    cpu_rsp.stall     = 1'b0;
                    lru_update_en     = 1'b1;
                    lru_accessed_way  = hit_way;
                    state_next        = ST_LC_IDLE;
                end
            end else begin
                // Miss
                if (ta_rd_out[lru_victim].valid && ta_rd_out[lru_victim].dirty)
                    state_next = ST_LC_WRITE_BACK;
                else
                    state_next = ST_LC_ALLOCATE;
            end
        end

        // ----------------------------------------------------------------------
        // FIX-6: one extra cycle for registered SRAM output to settle
        // ----------------------------------------------------------------------
        ST_LC_READ_HIT: begin
            cpu_rsp.valid    = 1'b1;
            cpu_rsp.stall    = 1'b0;
            cpu_rsp.rdata    = da_rd_word;
            lru_update_en    = 1'b1;
            lru_accessed_way = hit_way;
            state_next       = ST_LC_IDLE;
        end

        // ----------------------------------------------------------------------
        ST_LC_WRITE_BACK: begin
            cpu_rsp.stall = 1'b1;
            mem_req.valid = 1'b1;
            mem_req.we    = 1'b1;
            mem_req.addr  = {ta_rd_out[victim_way_r].tag,
                             req_index,
                             {OFFSET_BITS{1'b0}}};
            mem_req.wdata = da_line_out;

            if (mem_rsp.valid) begin
                ta_wr_en          = 1'b1;
                ta_wr_index       = req_index;
                ta_wr_way         = victim_way_r;
                ta_wr_data        = ta_rd_out[victim_way_r];
                ta_wr_data.valid  = 1'b0;
                ta_wr_data.dirty  = 1'b0;
                state_next        = ST_LC_ALLOCATE;
            end
        end

        // ----------------------------------------------------------------------
        // FIX-4: always move to MEM_WAIT next cycle (don't wait for mem_rsp here)
        // ----------------------------------------------------------------------
        ST_LC_ALLOCATE: begin
            cpu_rsp.stall = 1'b1;
            mem_req.valid = 1'b1;
            mem_req.we    = 1'b0;
            mem_req.addr  = {req_tag, req_index, {OFFSET_BITS{1'b0}}};
            state_next    = ST_LC_MEM_WAIT;
        end

        // ----------------------------------------------------------------------
        // FIX-5: NC path skips tag install; cache-fill path installs normally
        // ----------------------------------------------------------------------
        ST_LC_MEM_WAIT: begin
            cpu_rsp.stall = 1'b1;

            if (mem_rsp.valid) begin
                if (!nc_wait_r) begin
                    // ---- Cache fill ------------------------------------------
                    da_fill_en            = 1'b1;
                    da_fill_data          = mem_rsp.rdata;

                    ta_wr_en              = 1'b1;
                    ta_wr_index           = req_index;
                    ta_wr_way             = victim_way_r;
                    ta_wr_data.valid      = 1'b1;
                    ta_wr_data.dirty      = req_r.we;
                    ta_wr_data.tag        = req_tag;
                    ta_wr_data.lru        = {LRU_BITS{1'b1}};

                    if (req_r.we) da_cpu_wr_en = 1'b1;

                    lru_update_en         = 1'b1;
                    lru_accessed_way      = victim_way_r;

                    cpu_rsp.valid         = 1'b1;
                    cpu_rsp.stall         = 1'b0;
                    if (!req_r.we)
                        cpu_rsp.rdata = mem_rsp.rdata[0 +: DATA_WIDTH];
                end else begin
                    // ---- Non-cacheable read — forward only, no install --------
                    cpu_rsp.valid = 1'b1;
                    cpu_rsp.stall = 1'b0;
                    cpu_rsp.rdata = mem_rsp.rdata[0 +: DATA_WIDTH];
                end
                state_next = ST_LC_IDLE;
            end
        end

        // ----------------------------------------------------------------------
        // FLUSH: walk every set×way; write back each dirty line.
        // FIX-3: hold on dirty entry until memory acks (iter_advance gates it).
        // ----------------------------------------------------------------------
        ST_LC_FLUSH: begin
            cpu_rsp.stall = 1'b1;

            if (ta_rd_out[iter_way_r].valid && ta_rd_out[iter_way_r].dirty) begin
                mem_req.valid = 1'b1;
                mem_req.we    = 1'b1;
                mem_req.addr  = {ta_rd_out[iter_way_r].tag,
                                 iter_set_r,
                                 {OFFSET_BITS{1'b0}}};
                mem_req.wdata = da_line_out;

                if (mem_rsp.valid) begin
                    ta_wr_en          = 1'b1;
                    ta_wr_index       = iter_set_r;
                    ta_wr_way         = iter_way_r;
                    ta_wr_data        = ta_rd_out[iter_way_r];
                    ta_wr_data.dirty  = 1'b0;
                end
            end

            // Done when last entry processed and not still waiting on a dirty WB
            if (iter_set_r == INDEX_BITS'(NUM_SETS-1) &&
                iter_way_r  == WAY_BITS'(NUM_WAYS-1)  &&
                iter_advance) begin
                flush_done = 1'b1;
                state_next = ST_LC_IDLE;
            end
        end

        // ----------------------------------------------------------------------
        // INVALIDATE: clear valid & dirty for every entry (no write-back).
        // ----------------------------------------------------------------------
        ST_LC_INVALIDATE: begin
            cpu_rsp.stall     = 1'b1;
            ta_wr_en          = 1'b1;
            ta_wr_index       = iter_set_r;
            ta_wr_way         = iter_way_r;
            ta_wr_data        = ta_rd_out[iter_way_r];
            ta_wr_data.valid  = 1'b0;
            ta_wr_data.dirty  = 1'b0;

            if (iter_set_r == INDEX_BITS'(NUM_SETS-1) &&
                iter_way_r  == WAY_BITS'(NUM_WAYS-1)) begin
                inv_done   = 1'b1;
                state_next = ST_LC_IDLE;
            end
        end

        default: state_next = ST_LC_IDLE;
        endcase
    end

endmodule
