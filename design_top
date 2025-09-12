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


// ============================================================================
// Module 2: Your MIPS CPU, modified to include I-Cache and D-Cache
// ============================================================================
module pipe_MIPS32_with_cache (input clk1, clk2);
	// --- Main Memory & Pipeline Registers (Unchanged) ---
	reg [31:0] PC, IF_ID_IR, IF_ID_NPC;
	reg [31:0] ID_EX_IR, ID_EX_NPC, ID_EX_A, ID_EX_B, ID_EX_Imm;
	reg [2:0] ID_EX_type, EX_MEM_type, MEM_WB_type;
	reg [31:0] EX_MEM_IR, EX_MEM_ALUOut, EX_MEM_B;
	reg EX_MEM_cond;
	reg [31:0] MEM_WB_IR, MEM_WB_ALUOut, MEM_WB_LMD;
	reg [31:0] Reg [0:31];
	reg [31:0] Mem [0:1023]; // Main memory, now accessed by caches
	
	// --- Parameters and Control Signals (Unchanged) ---
	parameter ADD=6'b000000, SUB=6'b000001, AND=6'b000010, OR=6'b000011,
		  SLT=6'b000100, MUL=6'b000101, HLT=6'b111111, LW=6'b001000,
		  SW=6'b001001, ADDI=6'b001010, SUBI=6'b001011, SLTI=6'b001100,
		  BNEQZ=6'b001101, BEQZ=6'b001110;
	parameter RR_ALU=3'b000, RM_ALU=3'b001, LOAD=3'b010,
		  STORE=3'b011, BRANCH=3'b100, HALT=3'b101;
	reg HALTED;
	reg TAKEN_BRANCH;

	// ===================== NEW: CACHE INTEGRATION =========================
	// Wires for cache stall signals and data
	wire i_cache_stall, d_cache_stall;
	wire [31:0] i_cache_read_data;
	wire [31:0] d_cache_read_data;
	
	// Wires for connecting caches to the memory arbiter
	wire [31:0] i_cache_mem_address, d_cache_mem_address;
	wire i_cache_mem_read;
	wire d_cache_mem_read, d_cache_mem_write;
	wire [31:0] d_cache_mem_write_data;
	
	// Main memory busy simulation and data output
	reg mem_busy;
	reg [31:0] mem_read_data_out;

	// Instantiate Instruction Cache (I-Cache)
	simple_cache #(.CACHE_SIZE_WORDS(256)) i_cache (
		.clk(clk1), .rst(HALTED),
		.cpu_address(PC),
		.cpu_read(!HALTED),
		.cpu_write(1'b0), .cpu_write_data(32'b0), // I-cache is read-only
		.cpu_read_data(i_cache_read_data),
		.cpu_stall(i_cache_stall),
		.mem_address(i_cache_mem_address),
		.mem_read(i_cache_mem_read),
		.mem_write(), .mem_write_data(), // Not used
		.mem_read_data(mem_read_data_out),
		.mem_busy(mem_busy)
	);

	// Instantiate Data Cache (D-Cache)
	simple_cache #(.CACHE_SIZE_WORDS(256)) d_cache (
		.clk(clk2), .rst(HALTED),
		.cpu_address(EX_MEM_ALUOut),
		.cpu_read(EX_MEM_type == LOAD),
		.cpu_write(EX_MEM_type == STORE && !TAKEN_BRANCH),
		.cpu_write_data(EX_MEM_B),
		.cpu_read_data(d_cache_read_data),
		.cpu_stall(d_cache_stall),
		.mem_address(d_cache_mem_address),
		.mem_read(d_cache_mem_read),
		.mem_write(d_cache_mem_write),
		.mem_write_data(d_cache_mem_write_data),
		.mem_read_data(mem_read_data_out),
		.mem_busy(mem_busy)
	);

	// Memory Arbiter: D-Cache gets priority over I-Cache to avoid complex stalls
	wire mem_access = (d_cache_mem_read || d_cache_mem_write) || i_cache_mem_read;
	wire [31:0] mem_addr = (d_cache_mem_read || d_cache_mem_write) ? d_cache_mem_address : i_cache_mem_address;
	wire mem_write_en = d_cache_mem_write;

	// Main Memory Access Logic (controlled by arbiter)
	always @(posedge clk1) begin
		mem_busy <= #2 mem_access; // Simulate one cycle memory latency
		if (mem_access) begin
			if(mem_write_en) Mem[mem_addr] <= #2 d_cache_mem_write_data;
			mem_read_data_out <= #2 Mem[mem_addr];
		end
	end
	
	// NEW: Stall signals to control pipeline flow
	wire stall_pc_if = i_cache_stall || d_cache_stall; // Stall PC and IF/ID latch
	wire stall_id_ex = d_cache_stall; // Stall ID/EX latch

	// ================= MODIFIED PIPELINE STAGES =========================
	
	// IF Stage: Fetches from I-Cache. Stalls if cache misses or D-cache needs memory.
	always @ (posedge clk1)
		if (HALTED==0) begin
			if (!stall_pc_if) begin // Only advance if not stalled
				if (((EX_MEM_IR[31:26] == BEQZ) && (EX_MEM_cond == 1)) || ((EX_MEM_IR[31:26] == BNEQZ) && (EX_MEM_cond == 0))) begin
					PC <= #2 EX_MEM_ALUOut + 1;
					IF_ID_NPC <= #2 EX_MEM_ALUOut + 1;
					IF_ID_IR <= #2 32'b0; // Flush IF stage
					TAKEN_BRANCH <= #2 1'b1;
				end else begin
					PC <= #2 PC + 1;
					IF_ID_NPC <= #2 PC + 1;
					IF_ID_IR <= #2 i_cache_read_data; // Instruction now comes from I-Cache
				end
			end
		end

	// ID Stage: Stalls for D-cache misses to prevent data hazards.
	always @ (posedge clk2)
		if (HALTED==0) begin
			if (!stall_id_ex) begin // If not stalled, proceed normally
				if (IF_ID_IR[25:21] == 5'b00000) ID_EX_A <= 0;
				else ID_EX_A <= #2 Reg[IF_ID_IR[25:21]];
				if (IF_ID_IR[20:16] ==5'b00000) ID_EX_B <= 0;
				else ID_EX_B <= #2 Reg[IF_ID_IR[20:16]];
				ID_EX_NPC <= #2 IF_ID_NPC;
				ID_EX_IR <= #2 IF_ID_IR;
				ID_EX_Imm <= #2 {{16{IF_ID_IR[15]}}, {IF_ID_IR[15:0]}};
				
				case (IF_ID_IR[31:26])
					ADD, SUB, AND, OR, SLT, MUL: ID_EX_type <= #2 RR_ALU;
					ADDI, SUBI, SLTI: ID_EX_type <= #2 RM_ALU;
					LW: ID_EX_type <= #2 LOAD;
					SW: ID_EX_type <= #2 STORE;
					BNEQZ, BEQZ: ID_EX_type <= #2 BRANCH;
					HLT: ID_EX_type <= #2 HALT;
					default: ID_EX_type <= #2 HALT;
				endcase
			end else begin // If stalled, inject a NOP (bubble) into the EX stage
				ID_EX_type <= #2 HALT; 
				ID_EX_IR <= #2 32'b0;
			end
		end

	// EX Stage: No changes needed.
	always @ (posedge clk1)
		if (HALTED==0) begin
			EX_MEM_type <= #2 ID_EX_type;
			EX_MEM_IR <= #2 ID_EX_IR;
			TAKEN_BRANCH <= #2 0;
			
			case (ID_EX_type)
				RR_ALU: begin
					case (ID_EX_IR[31:26])
						ADD: EX_MEM_ALUOut <= #2 ID_EX_A + ID_EX_B;
						SUB: EX_MEM_ALUOut <= #2 ID_EX_A - ID_EX_B;
						AND: EX_MEM_ALUOut <= #2 ID_EX_A & ID_EX_B;
						OR:  EX_MEM_ALUOut <= #2 ID_EX_A | ID_EX_B;
						SLT: EX_MEM_ALUOut <= #2 ID_EX_A < ID_EX_B;
						MUL: EX_MEM_ALUOut <= #2 ID_EX_A * ID_EX_B;
						default: EX_MEM_ALUOut <= #2 32'hxxxxxxxx;
					endcase
				end
				RM_ALU: begin
					case (ID_EX_IR[31:26])
						ADDI: EX_MEM_ALUOut <= #2 ID_EX_A + ID_EX_Imm;
						SUBI: EX_MEM_ALUOut <= #2 ID_EX_A - ID_EX_Imm;
						SLTI: EX_MEM_ALUOut <= #2 ID_EX_A < ID_EX_Imm;
						default: EX_MEM_ALUOut <= #2 32'hxxxxxxxx;
					endcase
				end
				LOAD, STORE: begin
					EX_MEM_ALUOut <= #2 ID_EX_A + ID_EX_Imm;
					EX_MEM_B <= #2 ID_EX_B;
				end
				BRANCH: begin
					EX_MEM_ALUOut <= #2 ID_EX_NPC + ID_EX_Imm;
					EX_MEM_cond <= #2 (ID_EX_A==0);
				end
			endcase
		end

	// MEM Stage: Now interacts with the D-Cache.
	always @ (posedge clk2)
		if (HALTED==0) begin
			if(!d_cache_stall) begin // Only advance if D-cache is not stalled
				MEM_WB_type <= EX_MEM_type;
				MEM_WB_IR <= #2 EX_MEM_IR;
				case (EX_MEM_type)
					RR_ALU, RM_ALU: MEM_WB_ALUOut <= #2 EX_MEM_ALUOut;
					// LOAD data now comes from the D-Cache output
					LOAD: MEM_WB_LMD <= #2 d_cache_read_data; 
					// STORE is handled by the D-cache itself, no action needed here
					STORE: ; 
				endcase
			end
		end

	// WB Stage: No changes needed.
	always @ (posedge clk1) begin
		if (TAKEN_BRANCH==0)
			case (MEM_WB_type)
				RR_ALU: Reg[MEM_WB_IR[15:11]] <= #2 MEM_WB_ALUOut;
				RM_ALU: Reg[MEM_WB_IR[20:16]] <= #2 MEM_WB_ALUOut;
				LOAD: Reg[MEM_WB_IR[20:16]] <= #2 MEM_WB_LMD;
				HALT: HALTED <= #2 1'b1;
			endcase
		end
endmodule
