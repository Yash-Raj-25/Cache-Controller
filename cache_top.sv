// =============================================================================
// cache_top.sv
// Top-level integration module.
// Instantiates cache_controller (which internally instantiates
// tag_data_array, lru_unit, and mem_attr_table).
//
// DMA coherence protocol:
//   Before DMA READ  : assert flush_req; wait flush_done → start DMA
//   After  DMA WRITE : assert inv_req;   wait inv_done   → CPU resumes
// =============================================================================

import cache_pkg::*;

module cache_top (
    input  logic clk,
    input  logic rst_n,

    // CPU side
    input  cpu_req_t  cpu_req,
    output cpu_rsp_t  cpu_rsp,

    // DMA coherence control
    input  logic      flush_req,
    input  logic      inv_req,
    output logic      flush_done,
    output logic      inv_done,

    // Memory side
    output mem_req_t  mem_req,
    input  mem_rsp_t  mem_rsp
);

    cache_controller u_ctrl (
        .clk        (clk),
        .rst_n      (rst_n),
        .cpu_req    (cpu_req),
        .cpu_rsp    (cpu_rsp),
        .flush_req  (flush_req),
        .inv_req    (inv_req),
        .flush_done (flush_done),
        .inv_done   (inv_done),
        .mem_req    (mem_req),
        .mem_rsp    (mem_rsp)
    );

endmodule
