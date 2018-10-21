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
// Date: 14.05.2017
// Description: Emits and re-aligns compressed and unaligned instructions

import ariane_pkg::*;

module instr_realigner (
    input  logic                   clk_i,               // Clock
    input  logic                   rst_ni,              // Asynchronous reset active low
    // control signals
    input  logic                   flush_i,

    input  frontend_fetch_t        fetch_entry_i,
    input  logic                   fetch_entry_valid_i,
    output logic                   fetch_ack_o,

    output fetch_entry_t           fetch_entry_o,
    output logic                   fetch_entry_valid_o,
    input  logic                   fetch_ack_i
);

    logic en;
    logic unaligned;

    logic [INSTR_PER_FETCH-1:0] instr_vld;
    logic [INSTR_PER_FETCH-1:0] instr_vld_bp_masked;

    logic [INSTR_PER_FETCH-1:0][63:0] addr_o;
    logic [INSTR_PER_FETCH-1:0][31:0] instr_o;
    // remember the index of the instruction we are currently serving
    logic [$clog2(INSTR_PER_FETCH)-1:0] idx_d, idx_q;
    logic [INSTR_PER_FETCH-1:0] branch_taken_d, branch_taken_q;
    branchpredict_sbe_t bp_d, bp_q;


    instr_realign i_instr_realign (
        .clk_i       ( clk_i                      ),
        .rst_ni      ( rst_ni                     ),
        .flush_i     ( flush_i                    ),
        .en_i        ( en                         ),
        .valid_i     ( fetch_entry_valid_i        ),
        .address_i   ( fetch_entry_i.address      ),
        .taken_i     ( fetch_entry_i.branch_taken ),
        .data_i      ( fetch_entry_i.instruction  ),
        .valid_o     ( instr_vld                  ),
        .addr_o      ( addr_o                     ),
        .instr_o     ( instr_o                    ),
        .unaligned_o ( unaligned                  )
    );

    // if a branch was taken previously mask later instructions
    assign instr_vld_bp_masked[0] = instr_vld[0];
    assign instr_vld_bp_masked[1] = instr_vld[1] & ~fetch_entry_i.branch_taken[0];

    // we have one valid instruction
    assign fetch_entry_valid_o = |instr_vld_bp_masked;
    // slect the write output
    assign fetch_entry_o.address = addr_o[idx_q];
    assign fetch_entry_o.instruction = instr_o[idx_q];

    always_comb begin
        idx_d = idx_q;
        // etiher the downstream unit is accepting the data or we have fetched an unaligned word were we need to get
        // the next entry to have a valid output
        fetch_ack_o = fetch_ack_i | (~fetch_entry_valid_o & fetch_entry_valid_i);
        // we can advance the re-aligner if there is no remaining instruction
        en = fetch_ack_o & (~instr_vld_bp_masked[1]);
        // the upper 16 bit are also a valid instruction, we need to keep the data around
        if (instr_vld_bp_masked[1] && idx_q == 0) begin
            fetch_ack_o = 1'b0;
            // we can serve the upper part
            if (fetch_ack_i) idx_d = 1;
        end

        // we are serving the upper part of an instruction
        if (idx_q == 1) begin
            // as soon as the instruction fetch was acknowledged we can advance the pointer
            en = fetch_ack_i;
            fetch_ack_o = fetch_ack_i;
            // go back to serving the lower part
            if (fetch_ack_i) idx_d = 0;
        end
        // directly output this instruction. adoptions are made throughout the always comb block
        fetch_entry_o.branch_predict = fetch_entry_i.branch_predict;
        // squas branch prediction if we actually predicted on the upper address
        if (fetch_entry_i.branch_taken[1] && idx_q == 0) begin
            fetch_entry_o.branch_predict.valid = 1'b0;
        end

        // we are serving the upper part of an unaligned instruction and the last branch was taken
        if (unaligned && branch_taken_q[1]) begin
            fetch_entry_o.branch_predict = bp_q;
        end
    end

    // load a new branch prediction entry
    assign bp_d = (fetch_ack_o) ? fetch_entry_i.branch_predict : bp_q;
    assign branch_taken_d = (fetch_ack_o) ? fetch_entry_i.branch_taken : branch_taken_q;

    // assign the correct address for a potentially faulting unaligned instruction
    // we've already done the re-alignment for the instruction word so we
    // can just assign it here to tval
    assign fetch_entry_o.ex.valid = fetch_entry_i.page_fault;
    assign fetch_entry_o.ex.cause = (fetch_entry_i.page_fault) ? riscv::INSTR_PAGE_FAULT : '0;
    assign fetch_entry_o.ex.tval  = fetch_entry_o.address;

    always_ff @(posedge clk_i) begin
        if (~rst_ni) begin
            idx_q <= 0;
            bp_q  <= '0;
            branch_taken_q <= '0;
        end else begin
            idx_q <= idx_d;
            bp_q  <= bp_d;
            branch_taken_q <= branch_taken_d;
            if (flush_i) begin
                idx_q <= 0;
            end
        end
    end
endmodule
