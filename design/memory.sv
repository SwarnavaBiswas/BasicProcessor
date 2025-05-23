`timescale 1ns / 1ps

module memory(
        input [31: 0] data_in, // data to be stored in the memory
        input [31: 0] addr, // memory address with which data communication should take place
        input wr, // control signal to write data into the memory address
        input rd, // control signal to read data from the memory address
        input clk, // Global clock
        output [31: 0] data_out, // data from the memory location
        output reg write_done, // informs the processor that the data has been successfully written into the memory
        output available // informs the processor that the data is available in the output. 
                        // in this case, its asserted whenever rd is asserted, but for different memories, it may not be the case
    );
    parameter int address_bit_size = 12;
    reg [31: 0] mem[0: ((1 << address_bit_size) - 1)];
    
    always_ff@(posedge clk) begin
        write_done <= wr;
        if(wr) begin
            mem[addr[(address_bit_size-1):0]] <= data_in;
        end
    end
    assign data_out = (rd)? mem[addr[(address_bit_size-1):0]]: 32'b0;
    assign available = rd; // data is available in the same cycle when rd is asserted
endmodule
