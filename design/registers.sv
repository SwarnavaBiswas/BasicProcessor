`timescale 1ns / 1ps

//zero register code: 00000

module registers(
        input clk, // Global clock
        input wr, // control signal for writing into a general purpose register
        input wr_pc, // control signal for writing into program counter
        input reset_b, // Global asynchronous reset (active low)
        input [4:0] reg_code, // code of the register with which data communication should take place
        input [4:0] reg_shift_code, // code of the register for shifting purpose
        input [31:0] data, // data to be written into register of code reg_code
        input [31:0] pc_data, // data to be written into program counter
        output [31:0] register_read, // data stored in register[reg_code]
        output [31:0] register_shift_read, // data stored in register[reg_shift_code]
        output reg [31:0] program_counter, // special purpose program counter register
        output [31:0] stack_pointer, // part of general purpose registers
        output error // check whether programmer is trying to write into the zero register
    );
    reg [31:0] register[0:31];
    parameter zero = 5'b0,
              sp = 5'd2;
              
    assign stack_pointer = register[sp];
    assign register_read = register[reg_code];
    assign register_shift_read = register[reg_shift_code];
    assign error = wr & (reg_code == zero);
    
    always_ff@(posedge clk, negedge reset_b) begin
        if(~reset_b) begin
            for(int i=0; i<32; i++) begin
                register[i] <= 32'b0;
            end
        end
        else begin
            register[zero] <= 32'b0;
            if(wr & (reg_code != zero)) begin
                register[reg_code] <= data;
            end
        end
    end
    
    always_ff@(posedge clk, negedge reset_b) begin
        if(~reset_b) begin
            program_counter <= 32'b0;
        end
        else begin
            if(wr_pc) begin
                program_counter <= pc_data;
            end
        end
    end
    
endmodule
