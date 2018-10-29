// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 09.05.2017
// Description: Branch target calculation and comparison

module branch_unit (
    input  logic                      clk_i,
    input  logic                      rst_ni,
    input  logic                      debug_mode_i,

    input  ariane_pkg::fu_data_t      fu_data_i,
    input  logic [63:0]               pc_i,                   // PC of instruction
    input  logic [4:0]                rs1_i,                  // source operand 1 (only for diagnostics)
    input  logic [4:0]                rd_i,                   // destination register (only for diagnostics)
    input  logic                      is_compressed_instr_i,
    input  logic                      fu_valid_i,             // any functional unit is valid, check that there is no accidental mis-predict
    input  logic                      branch_valid_i,
    input  logic                      branch_comp_res_i,      // branch comparison result from ALU
    output logic [63:0]               branch_result_o,

    input  ariane_pkg::branchpredict_sbe_t branch_predict_i,       // this is the address we predicted
    output ariane_pkg::branchpredict_t     resolved_branch_o,      // this is the actual address we are targeting
    output logic                           resolve_branch_o,       // to ID to clear that we resolved the branch and we can
                                                              // accept new entries to the scoreboard
    output ariane_pkg::exception_t         branch_exception_o      // branch exception out
);
    logic [63:0] target_address;
    logic [63:0] next_pc;

    // here we handle the various possibilities of mis-predicts
    always_comb begin : mispredict_handler
        // set the jump base, for JALR we need to look at the register, for all other control flow instructions we can take the current PC
        automatic logic [63:0] jump_base;
        // TODO(zarubaf): The ALU can be used to calculate the branch target
        jump_base = (fu_data_i.operator == ariane_pkg::JALR) ? fu_data_i.operand_a : pc_i;

        target_address                   = 64'b0;
        resolve_branch_o                 = 1'b0;
        resolved_branch_o.target_address = 64'b0;
        resolved_branch_o.is_taken       = 1'b0;
        resolved_branch_o.valid          = branch_valid_i;
        resolved_branch_o.is_mispredict  = 1'b0;
        resolved_branch_o.cf_type        = branch_predict_i.cf;
        // calculate next PC, depending on whether the instruction is compressed or not this may be different
        // TODO(zarubaf): We already calculate this a couple of times, maybe re-use?
        next_pc                          = pc_i + ((is_compressed_instr_i) ? 64'h2 : 64'h4);
        // calculate target address simple 64 bit addition
        target_address                   = $unsigned($signed(jump_base) + $signed(fu_data_i.imm));
        // on a JALR we are supposed to reset the LSB to 0 (according to the specification)
        if (fu_data_i.operator == ariane_pkg::JALR) target_address[0] = 1'b0;

        // we need to put the branch target address into rd, this is the result of this unit
        branch_result_o = next_pc;

        resolved_branch_o.pc = pc_i;

        if (branch_valid_i) begin
            // write target address which goes to PC Gen
            resolved_branch_o.target_address = (branch_comp_res_i) ? target_address : next_pc;
            resolved_branch_o.is_taken = branch_comp_res_i;
            // check the outcome of the branch speculation
            if (fu_data_i.operator == ariane_pkg::BRANCH && branch_comp_res_i != (branch_predict_i.cf == ariane_pkg::Branch)) begin
                // we mis-predicted the outcome
                // if the outcome doesn't match we've got a mis-predict
                resolved_branch_o.is_mispredict  = 1'b1;
            end

            // check if the address of the jump is correct
            if (fu_data_i.operator == ariane_pkg::JALR && target_address != branch_predict_i.predict_address) begin
                resolved_branch_o.is_mispredict  = 1'b1;
            end
            // to resolve the branch in ID
            resolve_branch_o = 1'b1;
        end

        // we placed the prediction on a non branch instruction
        if (!branch_valid_i && fu_valid_i && branch_predict_i.cf != None) begin
            // we should not end here if the front-end is bug free
            $fatal(1, "Mis-predicted on non branch instruction");
        end
    end

    // use ALU exception signal for storing instruction fetch exceptions if
    // the target address is not aligned to a 2 byte boundary
    always_comb begin : exception_handling
        branch_exception_o.cause = riscv::INSTR_ADDR_MISALIGNED;
        branch_exception_o.valid = 1'b0;
        branch_exception_o.tval  = pc_i;
        // only throw exception if this is indeed a branch
        if (branch_valid_i && target_address[0] != 1'b0) begin
            branch_exception_o.valid = 1'b1;
        end
    end

    // Keep a golden model of the predictors
    // pragma translate_off
    `ifndef VERILATOR
        // branch history table (BHT)
        struct packed {
            logic       valid;
            logic [1:0] saturation_counter;
        } bht[ariane_pkg::BTB_ENTRIES-1:0];

        always_ff @(posedge clk_i or negedge rst_ni) begin
            if (~rst_ni) begin
                for (int unsigned i = 0; i < ariane_pkg::BTB_ENTRIES; i++)
                    bht[i] <= '0;
            end else if (!debug_mode_i) begin
                // save new branch decision
                if (resolved_branch_o.valid) begin
                    bht[resolved_branch_o.pc].valid <= 1'b1;

                    case (bht[resolved_branch_o.pc].saturation_counter)
                        2'b00: begin
                            if (resolved_branch_o.is_taken) bht[resolved_branch_o.pc].saturation_counter <= 2'b01;
                        end
                        2'b01: begin
                            if (resolved_branch_o.is_taken) bht[resolved_branch_o.pc].saturation_counter <= 2'b10;
                            else bht[resolved_branch_o.pc].saturation_counter <= 2'b00;
                        end
                        2'b10:
                            if (resolved_branch_o.is_taken) bht[resolved_branch_o.pc].saturation_counter <= 2'b11;
                            else bht[resolved_branch_o.pc].saturation_counter <= 2'b01;
                        2'b11:
                            if (!resolved_branch_o.is_taken) bht[resolved_branch_o.pc].saturation_counter <= 2'b10;
                    endcase
                end
            end
        end

        bht_check_valid : assert property(
            @(posedge clk_i) disable iff (~rst_ni) (bht[pc_i].valid |-> (branch_predict_i.cf != ariane_pkg::None)))
            else $warning("[BHT] We've already seen this branch but did not predict");

        // return address stack
        ras_t [RAS_DEPTH-1:0] ras;

        always_ff @(posedge clk_i or negedge rst_ni) begin
            if (~rst_ni) begin
                ras <= '0;
            end else if (!debug_mode_i) begin
                if (branch_valid_i) begin
                    // its either JALR or JAL
                    if (fu_data_i.operator == ariane_pkg::JALR ||  fu_data_i.operator == ariane_pkg::ADD) begin
                        // destination is x1 or x5 -> this is a call
                        if (rd_i == 5'd1 || rd_i == 5'd5) begin
                            // pop from call stack
                            // assert((branch_predict_i.cf != None) && branch_predict_i.cf_type == RAS) else $warning("Call wasn't detected correctly!");
                        // this is a return
                        end else if (rs1_i == 5'd1 || rs1_i == 5'd5) begin
                            // pop from call stack
                            // assert((branch_predict_i.cf != None) && branch_predict_i.cf_type == RAS) else $warning("Return wasn't detected correctly!");
                        end
                    end

                end
            end
        end

    `endif
    // pragma translate_on
endmodule
