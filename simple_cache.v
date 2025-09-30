// ============================================================================
// Module 1: A simple, reusable cache
// Design: Direct-mapped, Write-through, No-write-allocate
// Block Size: 1 word (32 bits)
// ============================================================================
module simple_cache #(
    parameter CACHE_SIZE_WORDS = 256 // Cache size in words (256 words = 1KB)
) (
    input clk,
    input rst,

    // --- CPU-facing interface ---
    input [31:0] cpu_address,   // Address from CPU
    input        cpu_read,      // Read enable
    input        cpu_write,     // Write enable
    input [31:0] cpu_write_data,// Data from CPU to write
    output logic [31:0] cpu_read_data, // Data read for CPU
    output logic cpu_stall,     // Stalls the CPU on a miss

    // --- Memory-facing interface ---
    output logic [31:0] mem_address,    // Address to main memory
    output logic        mem_read,       // Read enable to main memory
    output logic        mem_write,      // Write enable to main memory
    output logic [31:0] mem_write_data, // Data to write to main memory
    input [31:0]        mem_read_data,  // Data read from main memory
    input               mem_busy        // Signal if main memory is busy
);

    // Calculate address fields based on cache size.
    // Note: This assumes word-addressing, matching your CPU's memory model.
    localparam INDEX_BITS = $clog2(CACHE_SIZE_WORDS);
    localparam TAG_BITS = 32 - INDEX_BITS;

    // Decompose the incoming CPU address into tag and index
    logic [TAG_BITS-1:0]   tag_from_addr;
    logic [INDEX_BITS-1:0] index_from_addr;
    assign {tag_from_addr, index_from_addr} = cpu_address;

    // Cache internal storage arrays
    logic [TAG_BITS-1:0] tag_array[CACHE_SIZE_WORDS-1:0];
    logic                valid_array[CACHE_SIZE_WORDS-1:0];
    logic [31:0]         data_array[CACHE_SIZE_WORDS-1:0];

    // State machine for handling cache misses
    typedef enum {IDLE, MEM_READ_WAIT} cache_state;
    cache_state current_state, next_state;

    logic hit;

    // Combinational logic for hit detection, state transitions, and outputs
    always_comb begin
        // A hit occurs if the line is valid and the tags match
        hit = valid_array[index_from_addr] && (tag_array[index_from_addr] == tag_from_addr);
        
        // Always output the data from the cache line; it's valid only on a hit
        cpu_read_data = data_array[index_from_addr];
        
        // Stall the CPU if we are waiting for memory OR if it's a read miss
        cpu_stall = (current_state != IDLE) || (cpu_read && !hit);

        // Default outputs to memory
        mem_read = 1'b0;
        mem_write = 1'b0;
        mem_address = cpu_address;
        mem_write_data = cpu_write_data;
        next_state = current_state;

        case(current_state)
            IDLE: begin
                // For a write, we always write-through to main memory
                if (cpu_write) begin
                    mem_write = 1'b1;
                end 
                // For a read miss, request the data from memory
                else if (cpu_read && !hit) begin
                    mem_read = 1'b1;
                    if (!mem_busy) begin // Move to wait state once request is sent
                        next_state = MEM_READ_WAIT;
                    end
                end
            end
            MEM_READ_WAIT: begin
                // Stay in this state until memory is no longer busy
                if (!mem_busy) begin
                    next_state = IDLE;
                end
            end
        endcase
    end

    // Sequential logic for updating state and cache data
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            current_state <= IDLE;
            // Invalidate all cache lines on reset
            for (int i = 0; i < CACHE_SIZE_WORDS; i++) begin
                valid_array[i] <= 1'b0;
            end
        end else begin
            current_state <= next_state;

            // On a write hit, update the cache data (in addition to writing to memory)
            if (current_state == IDLE && cpu_write && hit) begin
                 data_array[index_from_addr] <= cpu_write_data;
            end

            // When memory read is done, update the cache line with the new data
            if (current_state == MEM_READ_WAIT && !mem_busy) begin
                data_array[index_from_addr]  <= mem_read_data;
                valid_array[index_from_addr] <= 1'b1;
                tag_array[index_from_addr]   <= tag_from_addr;
            end
        end
    end
endmodule


