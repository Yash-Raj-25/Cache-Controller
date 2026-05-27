// =============================================================================
// mem_attr_table.sv
// Memory Attribute Table — marks address ranges as cacheable or non-cacheable.
//
// Non-cacheable regions are typically DMA buffers, MMIO, and framebuffers.
// When an address falls in a non-cacheable region the cache controller
// bypasses all tag lookup and sends the transaction directly to memory.
// =============================================================================

import cache_pkg::*;

module mem_attr_table #(
    parameter logic [ADDR_WIDTH-1:0] NC_BASE [NC_REGIONS] = '{
        32'hA000_0000,   // DMA buffer A
        32'hB000_0000,   // DMA buffer B
        32'hC000_0000,   // MMIO / peripherals
        32'hFFFF_0000    // Boot ROM / special
    },
    parameter logic [ADDR_WIDTH-1:0] NC_SIZE [NC_REGIONS] = '{
        32'h0010_0000,   // 1 MB
        32'h0010_0000,   // 1 MB
        32'h0100_0000,   // 16 MB
        32'h0001_0000    // 64 KB
    }
) (
    input  logic [ADDR_WIDTH-1:0] addr,
    output logic                  non_cacheable
);

    always_comb begin
        non_cacheable = 1'b0;
        for (int i = 0; i < NC_REGIONS; i++) begin
            if (addr >= NC_BASE[i] && addr < (NC_BASE[i] + NC_SIZE[i]))
                non_cacheable = 1'b1;
        end
    end

endmodule
