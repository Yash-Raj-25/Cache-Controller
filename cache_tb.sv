// =============================================================================
// cache_tb.sv  — FIXED VERSION
// Self-checking testbench for the 4-way set-associative cache.
//
// Tests: read miss, read hit, write hit (dirty), write miss,
//        flush, invalidate, non-cacheable bypass
// =============================================================================

`timescale 1ns/1ps
import cache_pkg::*;

module cache_tb;

    // -------------------------------------------------------------------------
    // Clock / reset
    // -------------------------------------------------------------------------
    logic clk  = 1'b0;
    logic rst_n;
    always #5 clk = ~clk;   // 100 MHz

    // -------------------------------------------------------------------------
    // DUT ports
    // -------------------------------------------------------------------------
    cpu_req_t  cpu_req;
    cpu_rsp_t  cpu_rsp;
    logic      flush_req, inv_req, flush_done, inv_done;
    mem_req_t  mem_req;
    mem_rsp_t  mem_rsp;

    // -------------------------------------------------------------------------
    // DUT
    // -------------------------------------------------------------------------
    cache_top dut (.*);

    // -------------------------------------------------------------------------
    // Simple memory model (64 KB, word-addressed)
    // -------------------------------------------------------------------------
    logic [DATA_WIDTH-1:0] main_mem [0:16383];

    logic [2:0]   mem_lat_r;
    mem_req_t     mem_req_r;

    // the counter is idle.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_rsp.valid <= 1'b0;
            mem_rsp.rdata <= '0;
            mem_lat_r     <= 3'b0;
            mem_req_r     <= '0;
        end else begin
            mem_rsp.valid <= 1'b0;
            mem_rsp.rdata <= '0;

            // Write-back: ack immediately (posted)
            if (mem_req.valid && mem_req.we) begin
                mem_rsp.valid <= 1'b1;
                for (int w = 0; w < LINE_WORDS; w++)
                    main_mem[(mem_req.addr >> 2) + w] <=
                        mem_req.wdata[w*DATA_WIDTH +: DATA_WIDTH];
            end

            // Read: 4-cycle latency; only start when counter idle (FIX-TB-2)
            if (mem_req.valid && !mem_req.we && mem_lat_r == 3'b0) begin
                mem_req_r <= mem_req;
                mem_lat_r <= 3'd3;   // will fire at 3,2,1 -> rsp on count 1
            end

            if (mem_lat_r != 3'b0) begin
                mem_lat_r <= mem_lat_r - 1'b1;
                if (mem_lat_r == 3'd1) begin
                    mem_rsp.valid <= 1'b1;
                    for (int w = 0; w < LINE_WORDS; w++)
                        mem_rsp.rdata[w*DATA_WIDTH +: DATA_WIDTH] <=
                            main_mem[(mem_req_r.addr >> 2) + w];
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Watchdog (FIX-TB-4)
    // -------------------------------------------------------------------------
    initial begin
        #500000;
        $display("[TB] TIMEOUT — simulation did not finish in time");
        $finish;
    end

    // -------------------------------------------------------------------------
    // Task: CPU read — wait for !stall, capture rdata
    // -------------------------------------------------------------------------
    task automatic cpu_read(input logic [31:0] addr, output logic [31:0] data);
        @(negedge clk);
        cpu_req.valid = 1'b1;
        cpu_req.we    = 1'b0;
        cpu_req.addr  = addr;
        cpu_req.wdata = '0;
        cpu_req.be    = 4'hF;
        @(posedge clk);
        while (cpu_rsp.stall) @(posedge clk);
        data = cpu_rsp.rdata;
        @(negedge clk);
        cpu_req.valid = 1'b0;   // FIX-TB-3: deassert cleanly
        @(posedge clk);         // let controller return to IDLE
        $display("[TB] READ  addr=%08h  data=%08h", addr, data);
    endtask

    // -------------------------------------------------------------------------
    // Task: CPU write — wait for !stall
    // -------------------------------------------------------------------------
    task automatic cpu_write(input logic [31:0] addr, input logic [31:0] data);
        @(negedge clk);
        cpu_req.valid = 1'b1;
        cpu_req.we    = 1'b1;
        cpu_req.addr  = addr;
        cpu_req.wdata = data;
        cpu_req.be    = 4'hF;
        @(posedge clk);
        while (cpu_rsp.stall) @(posedge clk);
        @(negedge clk);
        cpu_req.valid = 1'b0;
        @(posedge clk);
        $display("[TB] WRITE addr=%08h  data=%08h", addr, data);
    endtask

    // -------------------------------------------------------------------------
    // Task: flush
    // -------------------------------------------------------------------------
    task automatic do_flush();
        @(negedge clk);
        flush_req = 1'b1;
        @(posedge clk);
        flush_req = 1'b0;
        while (!flush_done) @(posedge clk);
        @(posedge clk);
        $display("[TB] FLUSH complete");
    endtask

    // -------------------------------------------------------------------------
    // Task: invalidate
    // -------------------------------------------------------------------------
    task automatic do_inv();
        @(negedge clk);
        inv_req = 1'b1;
        @(posedge clk);
        inv_req = 1'b0;
        while (!inv_done) @(posedge clk);
        @(posedge clk);
        $display("[TB] INVALIDATE complete");
    endtask

    // -------------------------------------------------------------------------
    // Main test
    // -------------------------------------------------------------------------
    logic [31:0] rdata;

    initial begin
        $timeformat(-9, 1, " ns", 10);
        $dumpfile("cache_tb.vcd");
        $dumpvars(0, cache_tb);

        // Initialise memory
        for (int i = 0; i < 16384; i++) main_mem[i] = 32'hDEAD_0000 | i[31:0];

        // Initialise DUT inputs
        cpu_req   = '0;
        flush_req = 1'b0;
        inv_req   = 1'b0;

        // Reset
        rst_n = 1'b0;
        repeat(4) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        // ------------------------------------------------------------------
        $display("--- Test 1: Read miss (tag 0, set 0) ---");
        cpu_read(32'h0000_0000, rdata);

        // ------------------------------------------------------------------
        $display("--- Test 2: Read hit (same address) ---");
        cpu_read(32'h0000_0000, rdata);

        // ------------------------------------------------------------------
        $display("--- Test 3: Write hit (set 0, marks dirty) ---");
        cpu_write(32'h0000_0000, 32'hCAFE_BABE);

        // ------------------------------------------------------------------
        $display("--- Test 4: Read back written data (hit, dirty line) ---");
        cpu_read(32'h0000_0000, rdata);
        if (rdata !== 32'hCAFE_BABE)
            $error("FAIL Test 4: expected CAFEBABE got %08h", rdata);
        else
            $display("PASS Test 4");

        // ------------------------------------------------------------------
        $display("--- Test 5: Fill all 4 ways of set 1 ---");
        // Addresses 0x0000_0040, 0x0010_0040, 0x0020_0040, 0x0030_0040
        // share INDEX=1, different TAGs
        cpu_read(32'h0000_0040, rdata);
        cpu_read(32'h0010_0040, rdata);
        cpu_read(32'h0020_0040, rdata);
        cpu_read(32'h0030_0040, rdata);

        // ------------------------------------------------------------------
        $display("--- Test 6: Write miss — evicts LRU way of set 1 ---");
        cpu_write(32'h0040_0040, 32'h1234_5678);

        // ------------------------------------------------------------------
        $display("--- Test 7: Non-cacheable read (addr in NC region) ---");
        cpu_read(32'hA000_0010, rdata);

        // ------------------------------------------------------------------
        $display("--- Test 8: Flush (write all dirty lines to memory) ---");
        do_flush();

        // ------------------------------------------------------------------
        $display("--- Test 9: Invalidate all lines ---");
        do_inv();

        // ------------------------------------------------------------------
        $display("--- Test 10: Read after invalidate (should miss, refetch) ---");
        cpu_read(32'h0000_0000, rdata);

        $display("=== ALL TESTS DONE ===");
        #100 $finish;
    end

endmodule
