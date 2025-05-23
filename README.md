This is a basic processor that follows the instruction set as attached under documentation folder.
The plan of the finite state machine of the cpu is also provided in an excel sheet under documentation folder.
The processor is built assuming that the memory is word addressable.
The processor is not pipelined yet.

To run an assembly code:
  **Write the code in assembly_file.txt**
  **Provide the decimal inputs in inputs_decimal.txt**
  **Run assembler.py**
    The assembler converts the assembly code to processor readable instructions and stores it in instructions.txt
    The assembler converts the content in inputs_decimal.txt to binary and stores it in inputs.txt
  **Run cpu_sim.sv**
    The content in instructions.txt are initially stored in the memory from address 1024 (decimal)
    The content in inputs.txt are initially stored in the memory from address 2048 (decimal)
    The simulation result stores the final outputs starting from memory location 2048
    The memory locations from 2048 are stored in decimal in output_raw.txt
  **Observe the outputs in output_raw.txt**

Example code is provided in the assembly_file.txt, and example input in inputs_decimal.txt
output_raw.txt gives output
