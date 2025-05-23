
PATH = "" # Provide the Path here
MAX_INSTRUCTIONS = 1024
INSTRUCTIONS_BASE_ADDRESS = 1024
instructions = [["0"] * 32 for i in range(MAX_INSTRUCTIONS)]
assembly_file = open(f"{PATH}\\assembly_file.txt")


info = {
    "and":      {"opcode": "000000", "func": "000000", "type": "alis"},
    "or":       {"opcode": "000001", "func": "000000", "type": "alis"},
    "nor":      {"opcode": "000010", "func": "000000", "type": "alis"},
    "xor":      {"opcode": "000011", "func": "000000", "type": "alis"},
    "add":      {"opcode": "000100", "func": "000000", "type": "alis"},
    "sub":      {"opcode": "000101", "func": "000000", "type": "alis"},
    "andrs":    {"opcode": "000000", "func": "111111", "type": "alrs"},
    "orrs":     {"opcode": "000001", "func": "111111", "type": "alrs"},
    "norrs":    {"opcode": "000010", "func": "111111", "type": "alrs"},
    "xorrs":    {"opcode": "000011", "func": "111111", "type": "alrs"},
    "addrs":    {"opcode": "000100", "func": "111111", "type": "alrs"},
    "subrs":    {"opcode": "000101", "func": "111111", "type": "alrs"},
    "slt":      {"opcode": "000101", "func": "000001", "type": "alie"},
    "sgt":      {"opcode": "000101", "func": "000010", "type": "alie"},
    "slte":     {"opcode": "000101", "func": "000011", "type": "alie"},
    "sgte":     {"opcode": "000101", "func": "000100", "type": "alie"},
    "se":       {"opcode": "000101", "func": "000101", "type": "alie"},
    "lw":       {"opcode": "101000", "func": "", "type": "mt"},
    "sw":       {"opcode": "101001", "func": "", "type": "mt"},
    "andi":     {"opcode": "100000", "func": "", "type": "ali"},
    "ori":      {"opcode": "100001", "func": "", "type": "ali"},
    "xori":     {"opcode": "100011", "func": "", "type": "ali"},
    "addi":     {"opcode": "100100", "func": "", "type": "ali"},
    "beq":      {"opcode": "010101", "func": "", "type": "br"},
    "bne":      {"opcode": "110101", "func": "", "type": "br"},
    "jump":     {"opcode": "010000", "func": "", "type": "j"},
    "jal":      {"opcode": "010001", "func": "", "type": "j"},
    "jr":       {"opcode": "010010", "func": "", "type": "jr"},
    "nop":      {"opcode": "010011", "func": "", "type": "mc"},
    "halt":     {"opcode": "010111", "func": "", "type": "mc"},
    "flags":    {"opcode": "111111", "func": "", "type": "s"}
}

def get_register_code(st: str):
    if st[0] != "x":
        return ""
    if not st[1:].isdigit():
        return ""
    if len(st[1:]) > 2:
        return ""
    reg_num = int(st[1:])
    if(reg_num >= 32):
        return ""
    reg_code = format(reg_num, '05b')
    return reg_code

def twos_complement(in_bin: str, bit_limit: int):
    bin_temp = ["" for i in range(bit_limit)]
    flip = False
    for i in range(bit_limit-1, -1, -1):
        if (flip and in_bin[i] == "0") or (not flip and in_bin[i] == "1"):
            bin_temp[i] = "1"
        elif (flip and in_bin[i] == "1") or (not flip and in_bin[i] == "0"):
            bin_temp[i] = "0"
        else:
            print("Something went wrong!")
            exit()
        if in_bin[i] == "1":
            flip = True
    out_bin = "".join(bin_temp)
    return out_bin

def get_binary_equivalent(st_num: str, bit_limit: int):
    if len(st_num) == 0:
        return ""
    neg_sign = False
    if(st_num[0] == "-"):
        st_num = st_num[1:]
        neg_sign = True
    if not st_num.isdigit():
        return ""
    num = int(st_num)
    bin = format(num, f'0{bit_limit}b')
    if(neg_sign):
        bin = twos_complement(bin, bit_limit)
    return bin

line_index = INSTRUCTIONS_BASE_ADDRESS
assembly_instructions = []
label_dict = {}
for line in assembly_file:
    if line.count(";") > 0:
        line = line[0: line.index(";")]
    line = line.replace(',', ' ').replace('#', ' ').replace('(', ' ').replace(')', ' ')

    if(line.count(':') > 1):
        print("ERROR")
        exit()
    if(line.count(':') == 1):
        ind = line.index(':')
        label = line[0:ind].split()
        if(len(label) != 1):
            print("ERROR")
            exit()
        label_dict[label[0]] = line_index
        line = line[ind+1:]

    instr = line.split()
    if(len(instr) == 0):
        continue
    assembly_instructions.append(instr)
    line_index += 1

for instr_index, instr_list in enumerate(assembly_instructions):
    op = instr_list[0]
    if op not in info:
        print("ERROR")
        exit()

    if info[op]["type"] == "alis":
        r1_code = get_register_code(instr_list[1])
        r2_code = get_register_code(instr_list[2])
        r3_code = get_register_code(instr_list[3])
        shift_code = get_binary_equivalent(instr_list[4], 5)
        if(r1_code == "" or r2_code == "" or r3_code == "" or shift_code == ""):
            print("ERROR")
            exit()
        instructions[instr_index] = info[op]["opcode"] + r1_code + r2_code + r3_code + shift_code + info[op]["func"]
    elif info[op]["type"] == "alrs":
        r1_code = get_register_code(instr_list[1])
        r2_code = get_register_code(instr_list[2])
        r3_code = get_register_code(instr_list[3])
        r4_code = get_register_code(instr_list[4])
        if(r1_code == "" or r2_code == "" or r3_code == "" or r4_code == ""):
            print("ERROR")
            exit()
        instructions[instr_index] = info[op]["opcode"] + r1_code + r2_code + r3_code + r4_code + info[op]["func"]
    elif info[op]["type"] == "alie":
        r1_code = get_register_code(instr_list[1])
        r2_code = get_register_code(instr_list[2])
        r3_code = get_register_code(instr_list[3])
        if(r1_code == "" or r2_code == "" or r3_code == ""):
            print("ERROR")
            exit()
        instructions[instr_index] = info[op]["opcode"] + r1_code + r2_code + r3_code + "00000" + info[op]["func"]
    elif info[op]["type"] == "mt":
        r1_code = get_register_code(instr_list[1])
        r2_code = get_register_code(instr_list[3])
        val = get_binary_equivalent(instr_list[2], 16)
        if(r1_code == "" or r2_code == "" or val == ""):
            print("ERROR")
            exit()
        instructions[instr_index] = info[op]["opcode"] + r1_code + r2_code + val
    elif info[op]["type"] == "ali":
        r1_code = get_register_code(instr_list[1])
        r2_code = get_register_code(instr_list[2])
        val = get_binary_equivalent(instr_list[3], 16)
        if(r1_code == "" or r2_code == "" or val == ""):
            print("ERROR")
            exit()
        instructions[instr_index] = info[op]["opcode"] + r1_code + r2_code + val
    elif info[op]["type"] == "br":
        r1_code = get_register_code(instr_list[1])
        r2_code = get_register_code(instr_list[2])
        if instr_list[3] in label_dict:
            label_address = label_dict[instr_list[3]]
        elif instr_list[3].isdigit():
            label_address = int(instr_list[3])
        else:
            print("ERROR")
            exit()
        offset = label_address - (instr_index + INSTRUCTIONS_BASE_ADDRESS + 1)
        offset_bin = get_binary_equivalent(str(offset), 16)
        if(r1_code == "" or r2_code == "" or offset_bin == ""):
            print("ERROR")
            exit()
        instructions[instr_index] = info[op]["opcode"] + r1_code + r2_code + offset_bin
    elif info[op]["type"] == "j":
        if instr_list[1] in label_dict:
            label_address = label_dict[instr_list[1]]
        elif instr_list[1].isdigit():
            label_address = int(instr_list[1])
        else:
            print("ERROR")
            exit()
        label_address_bin = get_binary_equivalent(str(label_address), 26)
        instructions[instr_index] = info[op]["opcode"] + label_address_bin
    elif info[op]["type"] == "jr":
        instructions[instr_index] = info[op]["opcode"] + (26 * "0")
    elif info[op]["type"] == "mc":
        instructions[instr_index] = info[op]["opcode"] + (26 * "0")
    elif info[op]["type"] == "s":
        r1_code = get_register_code(instr_list[1])
        if(r1_code == ""):
            print("ERROR")
            exit()
        instructions[instr_index] = info[op]["opcode"] + r1_code + (21 * "0")
    else:
        print("Something went wrong!")
        exit()

joined_instructions = ["".join(instructions[i]) for i in range(MAX_INSTRUCTIONS)]
assembly_file.close()

instructions_file = open(f"{PATH}\\instructions.txt", "w")
for inst in joined_instructions:
    instructions_file.write(inst + "\n")
instructions_file.close()

inputs_decimal = open(f"{PATH}\\inputs_decimal.txt")
inputs = open("inputs.txt", "w")
for dec_line in inputs_decimal:
    for dec_str in dec_line.split():
        dec = get_binary_equivalent(dec_str, 32)
        if(dec == ""):
            print("ERROR")
            exit()
        inputs.write(dec + "\n")

inputs_decimal.close()
inputs.close()