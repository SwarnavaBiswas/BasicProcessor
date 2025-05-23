`timescale 1ns / 1ps

module cyclic_barrel_shifter(
        input [31: 0] input_lines, // Input 32 bit number
        input [4: 0] shift, // left cyclic shift amount
        output [31: 0] output_lines // Output 32 bit number
    );
    wire [31: 0] intermediate [0:5];
    assign intermediate[0] = input_lines;
    assign output_lines = intermediate[5];
    
    generate
        for(genvar i=0; i<5; i++) begin
            assign intermediate[i+1] = (shift[i])? {intermediate[i][31 - (1 << i): 0], intermediate[i][31: 32 - (1 << i)]} : intermediate[i];
        end
    endgenerate
    
endmodule
