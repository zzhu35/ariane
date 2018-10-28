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
// Date: 26.10.2018
// Description: Instruction Queue, separates instruction front-end from processor
//              back-end.
//
// This is an optimized instruction queue which supports the handling of
// compressed instructions (16 bit instructions). Internally it is organized as
// 4x 32 bit queues which are filled in a consecutive manner. The read port
// is designed so that it will easily allow for multiple issue implementation.
// The input supports arbitrary power of two instruction fetch widths.
//
// The queue supports handling of branch prediction and will take care of
// only saving a valid instruction stream.
//
// Furthermore it contains a replay interface in case the instruction queue
// is already full. As instructions are in general easily replayed this should
// increase the efficiency as I$ misses are potentially hidden. This stands in
// contrast to pessimistic actions (early stalling) or credit based approaches.
//
// TODO(zarubaf): Replaying can be power hungry, provide a mitigation when
// we do not want to replay (e.g.: processor halted, WFI).
//
// TODO(zarubaf): The instruction queues can be reduced to 16 bit. Potentially
// the replay mechanism gets more complicated as it can be that a 32 bit instruction
// can not be pushed at once.

module instr_queue (
    input  logic                                       clk_i,
    input  logic                                       rst_ni,
    input  logic                                       flush_i,
    input  logic [ariane_pkg::INSTR_PER_FETCH:0][31:0] instr_i,
    input  logic [ariane_pkg::INSTR_PER_FETCH:0][64:0] addr_i,
    input  logic [ariane_pkg::INSTR_PER_FETCH:0]       valid_i,
    // branch predict
    input  logic [63:0]                                predict_address_i,
    input  logic [ariane_pkg::INSTR_PER_FETCH:0]       taken_i,
    // we've encountered an exception, at this point the only possible exceptions are page-table faults
    input  logic                                       exception_i,
    // queue is ready to accept more data
    output logic                                       ready_o,
    // replay instruction because one of the FIFO was already full
    output logic                                       replay_o,
    output logic [63:0]                                replay_addr_o, // address at which to replay this instruction
    // to processor backend
    output fetch_entry_t                               fetch_entry_o,
    output logic                                       fetch_entry_valid_o,
    input  logic                                       fetch_entry_ack_i
);

    typedef struct packed {
        logic [31:0] instr; // instruction word
        logic        taken; // branch was taken
        logic        ex;    // exception happened
    } instr_data_t;

    logic [ariane_pkg::INSTR_PER_FETCH*2-1:0][31:0] instr;

    // instruction queues
    instr_data_t [ariane_pkg::INSTR_PER_FETCH:0] instr_data_in, instr_data_out;
    logic [ariane_pkg::INSTR_PER_FETCH:0] push_instr;
    logic [ariane_pkg::INSTR_PER_FETCH:0] pop_instr;
    logic [ariane_pkg::INSTR_PER_FETCH:0] full_instr;
    logic [ariane_pkg::INSTR_PER_FETCH:0] instr_queue_empty;
    logic instr_overflow;

    // address queue
    logic [63:0] address_out;
    logic pop_address;
    logic push_address;
    logic full_address;
    logic address_overflow;

    // input stream counter
    logic [$clog2(ariane_pkg::INSTR_PER_FETCH)-1:0] idx_is_d, idx_is_q;

    // Registers
    // output FIFO select, one-hot
    logic [ariane_pkg::INSTR_PER_FETCH-1:0] idx_ds_d, idx_ds_q;
    logic [63:0] pc_d, pc_q; // current PC
    logic reset_address_d, reset_address_q; // we need to re-set the address because of a flush

    for (genvar i = 0; i < ariane_pkg::INSTR_PER_FETCH; i++) begin : gen_instr_fifo
        fifo_v3 #(
            .DEPTH      ( 2                 ),
            .dtype      ( instr_data_t      )
        ) i_fifo_instr_data (
            .clk_i      ( clk_i             ),
            .rst_ni     ( rst_ni            ),
            .flush_i    ( flush_i           ),
            .testmode_i ( 1'b0              ),
            .full_o     ( full_instr        ),
            .empty_o    ( instr_queue_empty ),
            .usage_o    (                   ),
            .data_i     ( instr_data_in     ),
            .push_i     ( push_instr        ),
            .data_o     ( instr_data_out    ),
            .pop_i      ( pop_instr         )
        );
    end

    // if the branch is taken push the predict/known address
    assign push_address = |(taken_i);

    fifo_v3 #(
        .DEPTH      ( 8                            ),
        .WIDth      ( 64                           ),
    ) i_fifo_address (
        .clk_i      ( clk_i                        ),
        .rst_ni     ( rst_ni                       ),
        .flush_i    ( flush_i                      ),
        .testmode_i ( 1'b0                         ),
        .full_o     ( full_address                 ),
        .empty_o    (                              ),
        .usage_o    (                              ),
        .data_i     ( predict_address_i            ),
        .push_i     ( push_address & ~full_address ),
        .data_o     ( address_out                  ),
        .pop_i      ( pop_address                  )
    );

    logic [$clog2(ariane_pkg::INSTR_PER_FETCH)-1:0] branch_index;
    logic [ariane_pkg::INSTR_PER_FETCH*2-2:0] branch_mask_extended;
    logic [ariane_pkg::INSTR_PER_FETCH-1:0] branch_mask;

    // calculate a branch mask, e.g.: get the first taken branch
    lzc #(
        .WIDTH   ( ariane_pkg::INSTR_PER_FETCH ),
        .MODE    ( 0                           ) // count trailing zeros
    ) i_lzc_branch_mask (
        .in_i    ( taken_i      ), // we want to count trailing zeros
        .cnt_o   ( branch_index ), // first branch on branch_index
        .empty_o (              )
    );

    // the first index is for sure valid
    // for example (64 bit fetch):
    // taken mask: 0 1 1 0
    // leading zero count = 1
    // 0 0 0 1 1 1 1 << 1 = 0 0 1 1 1 1 0
    // take the upper 4 bits: 0 0 1 1
    assign branch_mask_extended = {{{ariane_pkg::INSTR_PER_FETCH-1}{1'b0}}, {{ariane_pkg::INSTR_PER_FETCH}{1'b1}}} << branch_index;
    assign branch_mask = branch_mask_extended[ariane_pkg::INSTR_PER_FETCH*2-2:ariane_pkg::INSTR_PER_FETCH];

    // shift amount, e.g.: instructions we want to retire
    logic [$clog2(ariane_pkg::INSTR_PER_FETCH)-1:0] shamt;
    logic [ariane_pkg::INSTR_PER_FETCH-1:0] valid;

    // mask with taken branches to get the actual amount of instructions we want to push
    // be sure that the instruction FIFO isn't full and that we also have space in the address fifo
    assign valid = valid_i & branch_mask & ({{ariane_pkg::INSTR_PER_FETCH}{~address_overflow}}) & ~full_instr;

    // count the numbers of valid instructions in this package
    lzc #(
        .WIDTH ( ariane_pkg::INSTR_PER_FETCH ),
        .MODE  ( 0                           ) // count trailing zeros
    ) i_lzc_valid (
        .in_i    ( ~valid ), // we want to count trailing ones, e.g.: invert and count trailing zeros
        .cnt_o   ( shamt  ),
        .empty_o (        )
    );

    // save the shift amount for next cycle
    assign idx_is_d = idx_is_q + shamt;

    // ----------------------
    // Input interface
    // ----------------------
    // FIFO mask
    logic [ariane_pkg::INSTR_PER_FETCH*2-1:0] fifo_pos_extended;
    logic [ariane_pkg::INSTR_PER_FETCH-1:0] fifo_pos;

    // rotate left by the current position
    assign fifo_pos_extended = { valid, valid } << idx_is_q;
    // we just care about the upper bits
    assign fifo_pos = fifo_pos_extended[ariane_pkg::INSTR_PER_FETCH*2-1:ariane_pkg::INSTR_PER_FETCH];
    // the fifo_position signal can directly be used to guide the push signal of each FIFO
    assign push_instr = fifo_pos;

    // duplicate the entries for easier selection
    for (genvar i = 0; i < ariane_pkg::INSTR_PER_FETCH; i++) begin : gen_duplicate_instr_input
        assign instr[i] = instr_i[i];
        assign instr[2*i] = instr_i[i];
    end

    // shift the inputs
    for (genvar i = 0; i < ariane_pkg::INSTR_PER_FETCH; i++) begin : gen_fifo_input_select
        assign instr_data_in[i] = instr[i + idx_is_q];
    end

    // ----------------------
    // Replay Logic
    // ----------------------
    // We need to replay a instruction fetch iff:
    // 1. One of the instruction data FIFOs was full and we needed it
    // (e.g.: we pushed and it was full)
    // 2. The address/branch predict FIFO was full
    // if one of the FIFOs was full we need to replay the faulting instruction
    logic [ariane_pkg::INSTR_PER_FETCH:0] instr_overflow_fifo;
    assign instr_overflow_fifo = full_instr & push_instr;
    assign instr_overflow = |instr_overflow_fifo; // at least one instruction overflowed
    assign address_overflow = full_address & push_address;
    assign replay_o = instr_overflow | address_overflow;

    // select the address, in the case of an address fifo overflow just
    // use the base of this package
    // if we successfully pushed some instructions we can output the next instruction
    // which we didn't manage to push
    assign replay_addr_o = (address_overflow) ? addr_i[0] : addr_i[shamt];

    // ----------------------
    // Downstream interface
    // ----------------------
    // as long as there is at least one queue which can take the value we have a valid instruction
    assign fetch_entry_valid_o = ~(&instr_queue_empty);

    always_comb begin
        idx_ds_d = idx_ds_q;

        fetch_entry_o.instruction = '0;
        fetch_entry_o.address = pc_q;

        fetch_entry_o.ex.valid = 1'b0;
        fetch_entry_o.ex.cause = riscv::INSTR_PAGE_FAULT;
        fetch_entry_o.ex.tval = riscv::INSTR_PAGE_FAULT;

        fetch_entry_o.branch_predict.predict_address = address_out;

        // output mux select
        for (int unsigned i = 0; i < ariane_pkg::INSTR_PER_FETCH; i++) begin
            if (idx_ds_q[i]) begin
                fetch_entry_o.instruction = instr_data_out[i].instr;
                fetch_entry_o.instruction.ex.valid = instr_data_out[i].ex;
                fetch_entry_o.branch_predict.valid = instr_data_out[i].taken;
            end
        end

        // rotate the pointer left
        if (fetch_entry_ack_i) begin
            idx_ds_d = {idx_ds_q[ariane_pkg::INSTR_PER_FETCH-2:0], idx_ds_q[ariane_pkg::INSTR_PER_FETCH-1]};
        end
    end

    // TODO(zarubaf): This needs to change for dual-issue
    // if the handshaking is successful and we had a prediction pop one address entry
    assign pop_address = (fetch_entry_o.branch_predict.valid & fetch_entry_ack_i) ? 1'b1 : 1'b0;

    // ----------------------
    // Calculate (Next) PC
    // ----------------------
    always_comb begin
        pc_d = pc_q;

        if (fetch_entry_ack_i) begin
            // TODO(zarubaf): This needs to change for a dual issue implementation
            // advance the PC
            pc_d =  pc_q + ((fetch_entry_o.instruction[1:0] != 2'b11) ? 'd2 : 'd4);
        end

        if (pop_address) pc_d = address_out;

        // we previously flushed so we need to reset the address
        if (reset_address_q) begin
            // this is the base of the first instruction
            pc_d = addr_i[0];
            reset_address_d = 1'b0;
        end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
            idx_ds_q        <= 'b1;
            idx_is_d        <= '0;
            pc_q            <= '0;
            reset_address_q <= 1'b1;
        end else begin
            idx_ds_q        <= idx_ds_d;
            idx_is_d        <= idx_is_d;
            pc_q            <= pc_d;
            reset_address_q <= reset_address_d;
            if (flush_i) begin
                idx_ds_q        <= 'b1;
                idx_is_d        <= '0;
                reset_address_q <= 1'b1;
            end
        end
    end

    // pragma translate_off
    `ifndef VERILATOR
        // assert that cache only hits on one way
        assert property (
          @(posedge clk_i) $onehot0(idx_ds_q)) else begin $error("Output select should be one-hot encoded"); $stop(); end
    `endif
    // pragma translate_on
endmodule
