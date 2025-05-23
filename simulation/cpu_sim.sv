`timescale 1ns / 1ps

module cpu_sim;
    logic clk, reset_b, complete, error;
    logic finish_sim;
    parameter PATH = ""; // Provide the path here
    cpu cpu1(.clk(clk),
             .reset_b(reset_b),
             .complete(complete),
             .error(error));
             
    assign finish_sim = complete | error;
    always #5 clk = ~clk;
    
    localparam instruction_start_address = 2 ** 10;
    localparam result_start_address = 2 ** 11;
    localparam result_end_address = result_start_address + 2 ** 10;
    int fd;
    task writemem(input [1023:0] filename);
        $display("Entered output writing task");
        fd = $fopen(filename);
        $display("Opened file");
        for(int i=result_start_address; i<result_end_address; i++) begin
            $fwrite(fd, "%d: %d\n", i, $signed(cpu.mem.mem[i]));
        end
        $display("Output written successfully");
        $fclose(fd);
    endtask
    
    initial begin
        $display("Reading inputs...");
        $readmemb({PATH, "\\rom.txt"}, cpu.mem.mem, 0);
        $readmemb({PATH, "\\instructions.txt"}, cpu.mem.mem, instruction_start_address);
        $readmemb({PATH, "\\inputs.txt"}, cpu.mem.mem, result_start_address);
        $display("Reading inputs done");
    end
    
    initial begin
        clk = 1'b0;
        reset_b = 1'b1;
        @(posedge clk);
        reset_b = 1'b0;
        @(posedge clk);
        reset_b = 1'b1;
        
        wait(finish_sim);
        @(posedge clk);
        $display("writing output...");
        writemem({PATH, "\\output_raw.txt"});
        $display("finishing...");
        $finish();
    end
    
endmodule
