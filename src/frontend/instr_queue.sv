module instr_queue (
    input  logic clk_i,
    input  logic rst_ni,
    input  logic flush_i,
    input  logic [63:0] base_address_i, // address of first instruction
    input  logic [ariane_pkg::INSTR_PER_FETCH:0][31:0] instr_i,
    input  logic [ariane_pkg::INSTR_PER_FETCH:0]       valid_i,
    // Branch predict
    input  logic [63:0] predict_address_i,
    input  logic [ariane_pkg::INSTR_PER_FETCH:0] taken_i,
    // we've encountered an exception, at this point the only possible exceptions are page-table faults
    input  logic exception_i,
    // queue is ready to accept more data
    output logic ready_o,
    // to processor backend
    output fetch_entry_t fetch_entry_o,
    output logic         fetch_entry_valid_o,
    input  logic         fetch_entry_ack_i
);

    typedef struct packed {
        logic [31:0] instr; // instruction word
        logic        taken; // branch was taken
        logic        ex;    // exception happened
    } instr_data_t;

    instr_data_t [ariane_pkg::INSTR_PER_FETCH:0] instr_data_in, instr_data_out;
    logic [ariane_pkg::INSTR_PER_FETCH:0] push_instr;
    logic [ariane_pkg::INSTR_PER_FETCH:0] pop_instr;
    logic [ariane_pkg::INSTR_PER_FETCH:0] instr_queue_empty;

    logic [63:0] address_out;
    logic push_address;
    logic pop_address;

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
            .full_o     (                   ),
            .empty_o    ( instr_queue_empty ),
            .usage_o    (                   ),
            .data_i     ( instr_data_in     ),
            .push_i     ( push_instr        ),
            .data_o     ( instr_data_out    ),
            .pop_i      ( pop_instr         )
        );
    end

    fifo_v3 #(
        .DEPTH      ( 8                   ),
        .WIDth      ( 64                  ),
    ) i_fifo_address (
        .clk_i      ( clk_i               ),
        .rst_ni     ( rst_ni              ),
        .flush_i    ( flush_i             ),
        .testmode_i ( 1'b0                ),
        .full_o     (                     ),
        .empty_o    (                     ),
        .usage_o    (                     ),
        .data_i     ( predict_address_i   ),
        .push_i     ( |(taken_i)          ), // if the branch is taken push the predict/known address
        .data_o     ( address_out         ),
        .pop_i      ( pop_address         )
    );

    // ----------------------
    // Input interface
    // ----------------------
    always_comb begin

    end

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
            pc_d = base_address_i;
            reset_address_d = 1'b0;
        end
    end

    // TODO(zarubaf): This needs to change for dual-issue
    assign pop_address = (fetch_entry_o.branch_predict.valid & fetch_entry_ack_i) ? 1'b1 : 1'b0;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
            idx_ds_q <= 'b1;
            pc_q <= '0;
            reset_address_q <= 1'b1;
        end else begin
            idx_ds_q <= idx_ds_d;
            pc_q <= pc_d;
            reset_address_q <= reset_address_d;

            if (flush_i) begin
                idx_ds_q <= 'b1;
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