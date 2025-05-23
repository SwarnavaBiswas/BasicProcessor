`timescale 1ns / 1ps

module cpu(
        input clk, // Global Clock
        input reset_b, // Global asynchronous reset (active low)
        output complete, // tells us whether the execution has completed or not, asserted after halt/ encountering error
        output error // tells us whether error has been encountered while executing the instructions
    );
    
    reg [31:0] internal_bus; // internal bus of the processor
    reg [31:0] instruction_buffer; // instruction buffer from the memory
    reg [4:0] shift; // stores the cyclic left shift amount for arithmetic and logical operations
    wire [5:0] cpu_opcode; // stores the opcode of the instruction from the memory 
    wire [5:0] functional_group; // stores the functional group code of the instruction
    reg alu_input_buffer_load, alu_output_buffer_load; // control unit for loading the input and output buffer of the alu driver respectively
    reg carry_in; // stores the carry_in value for alu addition
    reg pc_update_control; // control unit that states whether the alu is used for updating pc by 1
    wire [3:0] alu_opcode; // stores the opcode for the alu
    wire [31:0] alu_output_data; // stores the result of the alu operation
    wire zero_flag; // used to check whether the alu result is zero
    wire carry_flag; // used to check whether there is any carry in the alu operation
    wire overflow_detect; // used to check whether there is overflow in the alu operation
    wire sign_flag; // used to find the sign of the alu result
    wire alu_error_detect; // used to check whether the alu opcode is invalid
    
    reg reg_wr; // control signal for writing into a general purpose register
    reg pc_wr; // control signal for writing into the program counter
    reg [4:0] reg_code; // stores the code of the general purpose register to be accessed
    reg [4:0] reg_shift_code; // stores the code of the general purpose register for the shift purpose
    wire [31:0] reg_data; // stores the value of register[reg_code] when trying to use the value of a general purpose register
    wire [31:0] reg_shift_data; // stores the value of register[reg_shift_code] when general purpose register data is used for shifting
    wire [31:0] program_counter; // program counter
    wire [31:0] stack_pointer; // stack pointer
    wire register_error_detect; // detects whether programmer tries to write into zero register
    
    wire [31:0] address_bus; // bus storing the address of the memory location to be accessed
    reg mem_wr; // control signal to write into the memory address
    reg mem_rd; // control signal to read from the memory address
    wire [31:0] mem_data; // stores the value of memory[address_bus]
    wire mem_data_available; // stores whether the data from the memory is available
    wire mem_write_done; // stores whether the data has been successfully written into the memory
    reg [31:0] mem_buffer_data; // stores the value of memory[address_bus] after passing it through a register buffer
    reg [31:0] mem_address_register; // stores the address of the memory access
    
    wire [31:0] flag; // holds the flags: {28'b0, overflow, sign, carry, zero}
    
    reg [2:0] bus_control; // control signal for controlling which data will take over the internal bus    
    reg [1:0] shift_control; // control signal for controlling which data will take hold of the shift (5 bit) line
    
    wire [4:0] r1; // holds r1 from instruction buffer
    wire [4:0] r2; // holds r2 from instruction buffer
    wire [4:0] r3; // holds r3 from instruction buffer
    wire [4:0] r4; // holds r4 for register shift from instruction buffer
    wire [15:0] immediate; // holds the immediate value in the instruction buffer
    wire [25:0] location; // holds the lsb bits of location for jump and jal instructions
    
    // Some extra control signals:
    reg instruction_buffer_load; // control signal to load the instruction buffer
    reg mem_buffer_load; // control signal to load the mem_buffer_register
    reg mem_address_load; // control signal to load the mem_address_register
    reg forced_add; // control signal that forces the alu to perform addition, irrespective of the alu_opcode
    
    reg control2bus_data; // control signal to put 1 bit data to 32 bit general purpose register (for slt, slte, sgt, sgte, se)
    
    // parameters for bus_control:
    parameter pc2bus_code = 0,
              reg2bus_code = 1, 
              ibuflsb2bus_code = 2, 
              alu2bus_code = 3, 
              membuf2bus_code = 4,
              flag2bus_code = 5,
              control2bus_code = 6,
              jumploc2bus_code = 7;
    
    // parameters for shift_control:
    parameter ibuf2shift_code = 0,
              reg2shift_code = 1,
              zero2shift_code = 2;
              
    // parameters for cpu_opcodes:
    parameter and_op =      6'b000000,
              or_op =       6'b000001,
              nor_op =      6'b000010,
              xor_op =      6'b000011,
              add_op =      6'b000100,
              sub_op =      6'b000101,
              lw_op =       6'b101000,
              sw_op =       6'b101001,
              andi_op =     6'b100000,
              ori_op =      6'b100001,
              xori_op =     6'b100011,
              addi_op =     6'b100100,
              beq_op =      6'b010101,
              bne_op =      6'b110101,
              jump_op =     6'b010000,
              jal_op =      6'b010001,
              jr_op =       6'b010010,
              nop_op =      6'b010011,
              halt_op =     6'b010111,
              flags_op =    6'b111111;
              
    // parameters for functional groups:
    parameter immediate_shift_fn =      6'b000000,
              register_shift_fn =       6'b111111,
              slt_fn =                  6'b000001,
              sgt_fn =                  6'b000010,
              slte_fn =                 6'b000011,
              sgte_fn =                 6'b000100,
              se_fn =                   6'b000101;
    
    parameter link_register_code = 1; // register code for the register that will be used in jal and jr instructions
    
    assign address_bus = mem_address_register;
    assign cpu_opcode = instruction_buffer[31:26];
    assign flag = {28'b0, overflow_detect, sign_flag, carry_flag, zero_flag};
    assign alu_opcode = cpu_opcode[3:0]; // we can observe that the alu opcode is just the last 4 bits of instruction opcode
    assign functional_group = instruction_buffer[5:0]; // getting the functional group
    assign r1 = instruction_buffer[25:21]; // getting r1 code
    assign r2 = instruction_buffer[20:16]; // getting r2 code
    assign r3 = instruction_buffer[15:11]; // getting r3 code
    assign r4 = instruction_buffer[10:6]; // getting r4 code
    assign immediate = instruction_buffer[15:0]; // getting immediate value
    assign location = instruction_buffer[25:0]; // getting location value
    
    alu_driver alu_d(
            .clk(clk), // Global Clock
            .reset_b(reset_b), // Global asynchronous reset (active low)
            .bus_input(internal_bus), // Input from internal bus
            .input_buffer_load(alu_input_buffer_load), // control signal for loading the input buffer
            .output_buffer_load(alu_output_buffer_load), // control signal for loading the output buffer, as well the flops storing flags
            .shift(shift), // amount by which the result needs to be cyclic left shifted
            .carry_in(carry_in), // input carry
            .pc_update_control(pc_update_control), // select whether the first operand is from the input buffer or fixed value of 1 (for word addressable memory, should change to 4 for byte addressable memory)
                                     // is set to 1 if (pc) is to be updated to (pc)+1
            .op_code(alu_opcode), // alu opcode
            .forced_add(forced_add), // control signal to force the alu to perform addition irrespective of alu_opcode
            .output_buffer(alu_output_data), // stores the result of the two operands
            .zero_flag(zero_flag), // zero flag
            .carry_flag(carry_flag), // carry flag
            .overflow_detect(overflow_detect), // overflow detection
            .sign_flag(sign_flag), // sign flag
            .error_detect(alu_error_detect) // error detection
        );
    
    registers regis(
            .clk(clk), // Global clock
            .wr(reg_wr), // control signal for writing into a general purpose register
            .wr_pc(pc_wr), // control signal for writing into program counter
            .reset_b(reset_b), // Global asynchronous reset (active low)
            .reg_code(reg_code), // code of the register with which data communication should take place
            .reg_shift_code(reg_shift_code), // code of the register for shifting purposes
            .data(internal_bus), // data to be written into register of code reg_code
            .pc_data(internal_bus), // data to be written into program counter
            .register_read(reg_data), // data stored in register[reg_code]
            .register_shift_read(reg_shift_data), // data stored in register[reg_shift_code]
            .program_counter(program_counter), // special purpose program counter register
            .stack_pointer(stack_pointer), // part of general purpose registers
            .error(register_error_detect) // check whether programmer is trying to write into the zero register
        );
    
    memory mem(
            .data_in(internal_bus), // data to be stored in the memory
            .addr(address_bus), // memory address with which data communication should take place
            .wr(mem_wr), // control signal to write data into the memory address
            .rd(mem_rd), // control sugnal to read data from the memory address
            .clk(clk), // Global clock
            .data_out(mem_data), // data from the memory location
            .write_done(mem_write_done), // informs the processor whether data has been written into the memory successfully
            .available(mem_data_available) // informs the processor whether the data is available
        );
    
    always_comb begin // multiplexer to choose which signal to provide to the internal bus
        case(bus_control)
            pc2bus_code:        internal_bus = program_counter;
            reg2bus_code:       internal_bus = reg_data;
            ibuflsb2bus_code:   internal_bus = {{16{immediate[15]}}, immediate};
            alu2bus_code:       internal_bus = alu_output_data;
            membuf2bus_code:    internal_bus = mem_buffer_data;
            flag2bus_code:      internal_bus = flag;
            control2bus_code:   internal_bus = {31'b0, control2bus_data};
            jumploc2bus_code:   internal_bus = {program_counter[31:26], location};
            default:            internal_bus = 32'b0;
        endcase
    end
    
    always_comb begin // multiplexer to choose which signal to provide to the shift (5 bits) line
        case(shift_control)
            ibuf2shift_code: shift = instruction_buffer[10:6];
            reg2shift_code:  shift = reg_shift_data[4:0];
            zero2shift_code: shift = 5'b0;
            default:         shift = 5'b0;
        endcase
    end
    
    always_comb begin // multiplexer to choose which value to provide in control2bus_data
        case(functional_group)
            slt_fn:     control2bus_data = overflow_detect ^ sign_flag;
            sgt_fn:     control2bus_data = overflow_detect ^ ((~sign_flag) & (~zero_flag));
            slte_fn:    control2bus_data = overflow_detect ^ (sign_flag | zero_flag);
            sgte_fn:    control2bus_data = overflow_detect ^ (~sign_flag);
            se_fn:      control2bus_data = ~overflow_detect & zero_flag;
            default:    control2bus_data = 1'b0;
        endcase
    end 
    
    always_ff@(posedge clk, negedge reset_b) begin // code for loading the instruction buffer and memory buffer
        if(~reset_b) begin
            instruction_buffer <= 32'b0;
            mem_buffer_data <= 32'b0;
        end
        else begin
            if(instruction_buffer_load) begin
                instruction_buffer <= mem_data;
            end
            if(mem_buffer_load) begin
                mem_buffer_data <= mem_data;
            end
        end
    end
    
    always_ff@(posedge clk, negedge reset_b) begin // code for loading the memory address register
        if(~reset_b) begin
            mem_address_register <= 32'b0;
        end
        else begin
            if(mem_address_load) begin
                mem_address_register <= internal_bus;
            end
        end
    end
    
    
    // FSM Code below:
    
//    States:
    parameter P1 = 0,
              P2 = 1,
              P3 = 2,
              IE1 = 3,
              IE2 = 4,
              ALSI1 = 5,
              ALSI2 = 6,
              ALSR1 = 7,
              ALSR2 = 8,
              ALI1 = 9,
              ALI2 = 10,
              B1 = 11,
              BEQ2 = 12,
              BNE2 = 13,
              BPCU1 = 14,
              BPCU2 = 15,
              MT1 = 16,
              MT2 = 17,
              MTL3 = 18,
              MTL4 = 19,
              MTS3 = 20,
              J1 = 21,
              JAL1 = 22,
              JR1 = 23,
              F1 = 24,
              C = 25,
              E = 26;
    
    reg [4:0] present_state, next_state;
    
    assign complete = (present_state == C);
    assign error = (present_state == E);
    
    // Present state logic:
    always_ff@(posedge clk, negedge reset_b) begin
        if(~reset_b) begin
            present_state <= P1;
        end
        else begin
            present_state <= next_state;
        end
    end
    
    // Next_state logic:
    always_comb begin
        case(present_state)
            P1:     next_state = P2;
            P2:     next_state = (mem_data_available)? P3: P2;
            P3: begin
                    case(cpu_opcode)
                        and_op: begin
                            if(functional_group == immediate_shift_fn) next_state = ALSI1;
                            else if(functional_group == register_shift_fn) next_state = ALSR1;
                            else next_state = E;
                        end
                        or_op: begin
                            if(functional_group == immediate_shift_fn) next_state = ALSI1;
                            else if(functional_group == register_shift_fn) next_state = ALSR1;
                            else next_state = E;
                        end
                        nor_op: begin
                            if(functional_group == immediate_shift_fn) next_state = ALSI1;
                            else if(functional_group == register_shift_fn) next_state = ALSR1;
                            else next_state = E;
                        end
                        xor_op: begin
                            if(functional_group == immediate_shift_fn) next_state = ALSI1;
                            else if(functional_group == register_shift_fn) next_state = ALSR1;
                            else next_state = E;
                        end
                        add_op: begin
                            if(functional_group == immediate_shift_fn) next_state = ALSI1;
                            else if(functional_group == register_shift_fn) next_state = ALSR1;
                            else next_state = E;
                        end
                        sub_op: begin
                            if((functional_group == slt_fn) ||
                               (functional_group == sgt_fn) ||
                               (functional_group == slte_fn)||
                               (functional_group == sgte_fn)||
                               (functional_group == se_fn))
                                    next_state = IE1;
                            else if(functional_group == immediate_shift_fn) next_state = ALSI1;
                            else if(functional_group == register_shift_fn) next_state = ALSR1;
                            else next_state = E;
                        end
                        andi_op:    next_state = ALI1;
                        ori_op:     next_state = ALI1;
                        xori_op:    next_state = ALI1;
                        addi_op:    next_state = ALI1;
                        beq_op:     next_state = B1;
                        bne_op:     next_state = B1;
                        lw_op:      next_state = MT1;
                        sw_op:      next_state = MT1;
                        jump_op:    next_state = J1;
                        jal_op:     next_state = JAL1;
                        jr_op:      next_state = JR1;
                        nop_op:     next_state = P1;
                        halt_op:    next_state = C;
                        flags_op:   next_state = F1;
                        default:    next_state = E;
                    endcase
            end
            IE1:    next_state = IE2;
            IE2:    next_state = (r1 == 0)? E: P1;
            ALSI1:  next_state = ALSI2;
            ALSI2:  next_state = (r1 == 0)? E: P1;
            ALSR1:  next_state = ALSR2;
            ALSR2:  next_state = (r1 == 0)? E: P1;
            ALI1:   next_state = ALI2;
            ALI2:   next_state = (r1 == 0)? E: P1;
            B1: begin
                case(cpu_opcode)
                    beq_op:     next_state = BEQ2;
                    bne_op:     next_state = BNE2;
                    default:    next_state = E;
                endcase
            end
            BEQ2:   next_state = (~overflow_detect & zero_flag)? BPCU1: P1;
            BNE2:   next_state = (~overflow_detect & zero_flag)? P1: BPCU1;
            BPCU1:  next_state = BPCU2;
            BPCU2:  next_state = P1;
            MT1:    next_state = MT2;
            MT2: begin
                case(cpu_opcode)
                    lw_op:      next_state = MTL3;
                    sw_op:      next_state = MTS3;
                    default:    next_state = E;
                endcase
            end
            MTL3:   next_state = (mem_data_available)? MTL4: MTL3;
            MTL4:   next_state = (r1 == 0)? E: P1;
            MTS3:   next_state = (mem_write_done)? P1: MTS3;
            J1:     next_state = P1;
            JAL1:   next_state = J1;
            JR1:    next_state = P1;
            F1:     next_state = (r1 == 0)? E: P1;
            C:      next_state = C;
            E:      next_state = E;
            default:    next_state = E;
        endcase
    end
    
    // Control signals logic:
    always_comb begin
        case(present_state)
            P1: begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b1;
                carry_in = 1'b0;
                pc_update_control = 1'b1;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = 5'b0;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = pc2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b1;
                forced_add = 1'b1;
            end
            P2: begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b1;
                reg_code = 5'b0;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b1;
                bus_control = alu2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b1;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
           end
           P3:  begin
                alu_input_buffer_load = 1'b1;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = r2;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = reg2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
           end
           IE1: begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b1;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = r3;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = reg2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
           end
           IE2: begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b1;
                pc_wr = 1'b0;
                reg_code = r1;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = control2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
           end
           ALSI1: begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b1;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = r3;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = reg2bus_code;
                shift_control = ibuf2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
         ALSI2: begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b1;
                pc_wr = 1'b0;
                reg_code = r1;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = alu2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
           end
         ALSR1: begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b1;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = r3;
                reg_shift_code = r4;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = reg2bus_code;
                shift_control = reg2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
         ALSR2: begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b1;
                pc_wr = 1'b0;
                reg_code = r1;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = alu2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
        ALI1:   begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b1;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = 5'b0;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = ibuflsb2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
        ALI2:   begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b1;
                pc_wr = 1'b0;
                reg_code = r1;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = alu2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
        B1:     begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b1;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = r1;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = reg2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
        BEQ2:   begin
                alu_input_buffer_load = 1'b1;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = 5'b0;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = pc2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
       BNE2:    begin
                alu_input_buffer_load = 1'b1;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = 5'b0;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = pc2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
       BPCU1:   begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b1;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = 5'b0;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = ibuflsb2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b1;
            end
        BPCU2:  begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b1;
                reg_code = 5'b0;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = alu2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
        MT1:    begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b1;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = 5'b0;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = ibuflsb2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b1;
            end
        MT2:    begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = 5'b0;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = alu2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b1;
                forced_add = 1'b0;
            end
        MTL3:   begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = 5'b0;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b1;
                bus_control = reg2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b1;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
        MTL4:   begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b1;
                pc_wr = 1'b0;
                reg_code = r1;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = membuf2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
        MTS3:   begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = r1;
                reg_shift_code = 5'b0;
                mem_wr = 1'b1;
                mem_rd = 1'b0;
                bus_control = reg2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
        J1:     begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b1;
                reg_code = 5'b0;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = jumploc2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
        JAL1:   begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b1;
                pc_wr = 1'b0;
                reg_code = link_register_code;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = pc2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
        JR1:    begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b1;
                reg_code = link_register_code;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = reg2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
        F1:     begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b1;
                pc_wr = 1'b0;
                reg_code = r1;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = flag2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
        C:      begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = 5'b0;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = reg2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
        E:      begin
                alu_input_buffer_load = 1'b0;
                alu_output_buffer_load = 1'b0;
                carry_in = 1'b0;
                pc_update_control = 1'b0;
                reg_wr = 1'b0;
                pc_wr = 1'b0;
                reg_code = 5'b0;
                reg_shift_code = 5'b0;
                mem_wr = 1'b0;
                mem_rd = 1'b0;
                bus_control = reg2bus_code;
                shift_control = zero2shift_code;
                instruction_buffer_load = 1'b0;
                mem_buffer_load = 1'b0;
                mem_address_load = 1'b0;
                forced_add = 1'b0;
            end
        default: begin
            alu_input_buffer_load = 1'b0;
            alu_output_buffer_load = 1'b0;
            carry_in = 1'b0;
            pc_update_control = 1'b0;
            reg_wr = 1'b0;
            pc_wr = 1'b0;
            reg_code = 5'b0;
            reg_shift_code = 5'b0;
            mem_wr = 1'b0;
            mem_rd = 1'b0;
            bus_control = reg2bus_code;
            shift_control = zero2shift_code;
            instruction_buffer_load = 1'b0;
            mem_buffer_load = 1'b0;
            mem_address_load = 1'b0;
            forced_add = 1'b0;
        end
        endcase
    end
endmodule
