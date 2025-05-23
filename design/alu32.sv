`timescale 1ns / 1ps

module alu32(
        input [3:0] op_code,
        input forced_add,
        input [31: 0] data1,
        input [31: 0] data2,
        input [4:0] shift,
        input carry_in,
        output [31: 0] data_out,
        output zero,
        output reg carry,
        output reg overflow,
        output sign,
        output reg error
    );
    parameter and_op =      4'b0000,
              or_op =       4'b0001,
              nor_op =      4'b0010,
              xor_op =      4'b0011,
              add_op =      4'b0100,
              subtract_op = 4'b0101;
    wire [31:0] data2_inv;
    reg carry_into_msb;
    reg [31:0] alu_intermediate;
    
    cyclic_barrel_shifter cbs(alu_intermediate, shift, data_out);
    assign data2_inv = (~data2) + 1;
    assign zero = ~|(data_out);
    assign sign = data_out[31];
    always_comb begin
        carry = 1'b0;
        overflow = 1'b0;
        alu_intermediate = 32'b0;
        casez({forced_add, op_code})
            5'b1zzzz: begin
                {carry_into_msb, alu_intermediate[30: 0]} = data1[30: 0] + data2[30: 0] + carry_in;
                {carry, alu_intermediate[31]} = data1[31] + data2[31] + carry_into_msb;
                overflow = carry_into_msb ^ carry;
                error = 1'b0;
            end
            {1'b0, and_op}: begin
                alu_intermediate = data1 & data2;
                error = 1'b0;
            end
            {1'b0, or_op}: begin
                alu_intermediate = data1 | data2;
                error = 1'b0;
            end
            {1'b0, nor_op}: begin
                alu_intermediate = ~(data1 | data2);
                error = 1'b0;
            end
            {1'b0, xor_op}: begin
                alu_intermediate = data1 ^ data2;
                error = 1'b0;
            end
            {1'b0, add_op}: begin
                {carry_into_msb, alu_intermediate[30: 0]} = data1[30: 0] + data2[30: 0] + carry_in;
                {carry, alu_intermediate[31]} = data1[31] + data2[31] + carry_into_msb;
                overflow = carry_into_msb ^ carry;
                error = 1'b0;
            end
            {1'b0, subtract_op}: begin
                {carry_into_msb, alu_intermediate[30: 0]} = data1[30: 0] + data2_inv[30: 0];
                {carry, alu_intermediate[31]} = data1[31] + data2_inv[31] + carry_into_msb;
                overflow = carry_into_msb ^ carry ^ (data2 == (1 << 31));
                error = 1'b0;
            end
            default: begin
                error = 1'b1;
            end
        endcase
    
    end
    
endmodule
