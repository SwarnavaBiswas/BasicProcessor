This is a basic 32-bit processor that follows the instruction set as attached under documentation folder. <br />
There are 32 internal registers x0 to x31, where x0 is zero register, and x1 is used for jal and jr operations <br />
The content in output_raw are in decimal <br />
The plan of the finite state machine of the cpu is also provided in an excel sheet under documentation folder. <br />
The processor is built assuming that the memory is word addressable. <br />
The processor is not pipelined yet. <br />
<br />
To run an assembly code: <br />
  **Provide the path of the text files in PATH variable in assembler.py and cpu_sim.sv** <br />
  **Write the code in assembly_file.txt** <br />
  **Provide the decimal inputs in inputs_decimal.txt** <br />
  **Run assembler.py** <br />
    The assembler converts the assembly code to processor readable instructions and stores it in instructions.txt <br />
    The assembler converts the content in inputs_decimal.txt to binary and stores it in inputs.txt <br />
  **Run cpu_sim.sv** <br />
    The content in instructions.txt are initially stored in the memory from address 1024 (decimal) <br />
    The content in inputs.txt are initially stored in the memory from address 2048 (decimal) <br />
    The simulation result stores the final outputs starting from memory location 2048 <br />
    The memory locations from 2048 are stored in decimal in output_raw.txt <br />
  **Observe the outputs in output_raw.txt** <br />
<br />
Example code is provided in the assembly_file.txt, and example input in inputs_decimal.txt <br />
output_raw.txt gives output <br />
