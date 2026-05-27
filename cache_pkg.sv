// =============================================================================
// cache_pkg.sv
// Parameters and types for a single 4-way set-associative cache
// with LRU replacement. No MESI — DMA coherence handled via
// software flush/invalidate and non-cacheable memory regions.
// =============================================================================

package cache_pkg;

    // Cache geometry
    parameter int ADDR_WIDTH  = 32;
    parameter int DATA_WIDTH  = 32;
    parameter int NUM_SETS    = 256;
    parameter int NUM_WAYS    = 4;
    parameter int LINE_BYTES  = 64;
    parameter int LINE_WORDS  = LINE_BYTES / (DATA_WIDTH/8);  // 16

    parameter int OFFSET_BITS = $clog2(LINE_BYTES);   // 6
    parameter int INDEX_BITS  = $clog2(NUM_SETS);     // 8
    parameter int TAG_BITS    = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS; // 18
    parameter int WAY_BITS    = $clog2(NUM_WAYS);     // 2
    parameter int LRU_BITS    = 2;   // 2-bit saturating counter per way

    // Non-cacheable region table size
    parameter int NC_REGIONS  = 4;

    // Cache controller FSM states
    typedef enum logic [2:0] {
        ST_IDLE       = 3'b000,
        ST_TAG_CHECK  = 3'b001,
        ST_WRITE_BACK = 3'b010,
        ST_ALLOCATE   = 3'b011,
        ST_MEM_WAIT   = 3'b100,
        ST_FLUSH      = 3'b101,
        ST_INVALIDATE = 3'b110
    } ctrl_state_t;

    // Per-line metadata (valid + dirty + tag + LRU)
    typedef struct packed {
        logic                    valid;
        logic                    dirty;
        logic [TAG_BITS-1:0]     tag;
        logic [LRU_BITS-1:0]     lru;
    } line_meta_t;

    // CPU request / response
    typedef struct packed {
        logic                      valid;
        logic                      we;
        logic [ADDR_WIDTH-1:0]     addr;
        logic [DATA_WIDTH-1:0]     wdata;
        logic [(DATA_WIDTH/8)-1:0] be;
    } cpu_req_t;

    typedef struct packed {
        logic                  valid;
        logic [DATA_WIDTH-1:0] rdata;
        logic                  stall;
    } cpu_rsp_t;

    // Memory interface (to DRAM / memory controller)
    typedef struct packed {
        logic                              valid;
        logic                              we;
        logic [ADDR_WIDTH-1:0]             addr;
        logic [DATA_WIDTH*LINE_WORDS-1:0]  wdata;  // full line for write-back
    } mem_req_t;

    typedef struct packed {
        logic                              valid;
        logic [DATA_WIDTH*LINE_WORDS-1:0]  rdata;
    } mem_rsp_t;

endpackage
