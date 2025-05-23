`timescale 1ns / 1ps

module alu_driver(
        input clk, // Global Clock
        input reset_b, // Global asynchronous reset (active low)
        input [31:0] bus_input, // Input from internal bus
        input input_buffer_load, // control signal for loading the input buffer
        input output_buffer_load, // control signal for loading the output buffer, as well the flops storing flags
        input [4:0] shift, // amount by which the result needs to be cyclic left shifted
        input carry_in, // input carry
        input pc_update_control, // select whether the first operand is from the input buffer or fixed value of 1 (for word addressable memory, should change to 4 for byte addressable memory)
                                 // is set to 1 if (pc) is to be updated to (pc)+1
        input [3:0] op_code, // alu opcode
        input forced_add, // control signal that forces the alu to perform addition irrespective of opcode
        output reg [31:0] output_buffer, // Output Buffer stores the result of the two operands
        output reg zero_flag, // zero flag
        output reg carry_flag, // carry flag
        output reg overflow_detect, // overflow detection
        output reg sign_flag, // sign flag
        output reg error_detect // error detection
    );
    reg [31:0] input_buffer; // Input Buffer stores the first operand of the two operands (except when the first operand is 1)
    wire [31:0] alu_result; // holds the result that the alu produces
    wire zero; // holds the result of the zero flag
    wire carry; // holds the result of the carry flag
    wire overflow; // holds the result of the overflow detection
    wire sign; // holds the result of the sign flag
    wire error; // holds the result of the error detection
    wire [31:0] first_operand; // holds the value of the first operand
    
    assign first_operand = (pc_update_control)? 32'b1: input_buffer; 
    alu32 alu(
            .op_code(op_code), // [3:0], input
            .forced_add(forced_add), // 1 bit, input, control signal
            .data1(first_operand), // [31:0], input, first operand
            .data2(bus_input), // [31:0] input, second operand
            .shift(shift), // [4:0] input, cyclic left shift amount
            .carry_in(carry_in), // 1 bit, input, carry in
            .data_out(alu_result), // [31:0], output, result out
            .zero(zero), // 1 bit, output, zero flag
            .carry(carry), // 1 bit, output, carry flag
            .overflow(overflow), // 1 bit, output, overflow detection
            .sign(sign), // 1 bit, output, sign flag
            .error(error) // 1 bit, output, error detection
        );
    always_ff@(posedge clk, negedge reset_b) begin
        if(~reset_b) begin
            input_buffer <= 32'b0;
            output_buffer <= 32'b0;
            zero_flag <= 1'b0;
            carry_flag <= 1'b0;
            overflow_detect <= 1'b0;
            sign_flag <= 1'b0;
            error_detect <= 1'b0;
        end
        else begin
            if(input_buffer_load) begin
                input_buffer <= bus_input;
            end
            if(output_buffer_load) begin
                output_buffer <= alu_result;
                if(~forced_add) begin
                    zero_flag <= zero;
                    carry_flag <= carry;
                    overflow_detect <= overflow;
                    sign_flag <= sign;
                end
                error_detect <= error;
            end
        end
    end
    
endmodule
