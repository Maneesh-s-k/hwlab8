
//Datapath
module MIPS_datapath(
  input clk,
  input reset,
  
  //From MIPS controller
  input RegDst,          //if its an R or I type instruction and shoul rd be read
  input Jump,           
  input Branch,        
  input MemRead,      
  input MemToReg,    
  input MemWrite,   
  input ALUSrc,    //whether alu's input is 2nd reg or immediate value
  input RegWrite, 
  input Link,    //For jal instruction
  input JR,     //For Jump to reg instruction
  input PC_en, //enable PC to read its input value
  
  //From ALU controller
  input [3:0] ALU_opcode, //4bits, as seen in a prev slide, ig 1 extra bit for binvert, pretty sure
  
  
  output wire [5:0] OpCode, //To controller
  output wire [5:0] funct  //To ALU_controller and controller
);
  
  
  // Instruction Mem and Operand Decode
  wire [31:0] PC_out, instr_out;
  instr_mem Intruction_Memory(.read_address(PC_out), .instruction(instr_out));
  
  assign OpCode = instr_out[31:26];
  
  // Instruction fields
  wire [4:0] rs = instr_out[25:21];
  wire [4:0] rt = instr_out[20:16];
  wire [4:0] rd = instr_out[15:11];
  wire [15:0] immediate = instr_out[15:0];
  wire [25:0] jump_target = instr_out[25:0];
  assign funct = instr_out[5:0];
  
  //------------------------------------------------------------------------
  
  // PC
  wire [31:0] PC_plus4, PC_next;
  wire ALU_zero;
  
  PC Program_Counter(.clk(clk), .reset(reset), .en(PC_en), .d(PC_next), .q(PC_out));
  
  //PC+4
  adder PC_plus4_adder(.a(PC_out), .b(32'd4), .y(PC_plus4));
  
  // Jump address calc
  wire [28:0] jump_target_shifted;
  sll_2_pad #(.INP_WIDTH(26)) Shift_Jump_2(.in(jump_target), .out(jump_target_shifted));
  wire [31:0] jump_addr = {PC_plus4[31:28], jump_target_shifted};
  
  // Immediate address calc
  wire [31:0] sign_ext_imm;
  sign_extend Sign_Extender(.in(immediate), .out(sign_ext_imm));
  
  wire [31:0] shifted_imm;
  sll_2_same Shift_Imm_2(.in(sign_ext_imm), .out(shifted_imm));
  
  wire [31:0] branch_addr;
  adder Branch_Adder(.a(PC_plus4), .b(shifted_imm), .y(branch_addr));
  
  // Branch instr or PC+4
  wire branch_taken = Branch & ALU_zero;
  wire [31:0] PC_branch_mux_out;
  mux2to1 #(.WIDTH(32)) PC_branch_mux(
    .in0(PC_plus4),
    .in1(branch_addr),
    .sel(branch_taken),
    .out(PC_branch_mux_out)
  );
  
  // whether Jump instr
  wire [31:0] PC_jump_mux_out;
  mux2to1 #(.WIDTH(32)) PC_jump_mux(
    .in0(PC_branch_mux_out),
    .in1(jump_addr),
    .sel(Jump),
    .out(PC_jump_mux_out)
  );
  
  //JR instruction
  wire [31:0] reg_read_data1;
  mux2to1 #(.WIDTH(32)) PC_Jump_Reg(
    .in0(PC_jump_mux_out),
    .in1(reg_read_data1),
    .sel(JR),
    .out(PC_next)
  );
 
  //---------------------------------------------------------------------------------------
  
  // Registers
  wire [4:0] write_reg_intermediate, write_reg_final;
  wire [31:0] write_data;
  wire [31:0] reg_read_data2;
  
  // R-type or I-type instr
  mux2to1 #(.WIDTH(5)) write_reg_mux(
    .in0(rt), 
    .in1(rd), 
    .sel(RegDst), 
    .out(write_reg_intermediate)
  );
  // if Jump and Link
  mux2to1 #(.WIDTH(5)) link_reg_mux(
    .in0(write_reg_intermediate), 
    .in1(5'd31), // reg $ra
    .sel(Link), 
    .out(write_reg_final)
  );
  
  reg_file Registers(
    .clk(clk),
    .reset(reset),
    .write_en(RegWrite),
    .read_reg1(rs),
    .read_reg2(rt),
    .write_reg(write_reg_final),
    .write_data(write_data),
    .read_data1(reg_read_data1),
    .read_data2(reg_read_data2)
  );
  
  
  // ------------------------------------------------------------------------
  
  //ALU
  wire [31:0] ALU_in2;
  wire [31:0] ALU_result;
  
  //computing reg w/ immediate or reg w/ reg
  mux2to1 #(.WIDTH(32)) ALU_src_mux(
    .in0(reg_read_data2),
    .in1(sign_ext_imm),
    .sel(ALUSrc),
    .out(ALU_in2)
  );
  
  ALU Arithmetic_Logic_Unit(
    .a(reg_read_data1),
    .b(ALU_in2),
    .op(ALU_opcode),
    .result(ALU_result),
    .zero(ALU_zero)
  );
  
  // ------------------------------------------------------------------------
  
  
  
  // Data Mem and Write Back
  wire [31:0] mem_read_data;
  data_mem Data_Memory(
    .clk(clk),
    .write_en(MemWrite),
    .read_en(MemRead),
    .address(ALU_result),
    .write_data(reg_read_data2),
    .read_data(mem_read_data)
  );
  
  //load instr or addi instr
  wire [31:0] wb_data;
  mux2to1 #(.WIDTH(32)) wb_mux(
    .in0(ALU_result),
    .in1(mem_read_data),
    .sel(MemToReg),
    .out(wb_data)
  );
  
  //whether to write back PC+4 to $ra
  mux2to1 #(.WIDTH(32)) link_data_mux(
    .in0(wb_data),
    .in1(PC_plus4),
    .sel(Link),
    .out(write_data)
  );
  
endmodule

module controller(
    input clk,
    input reset,
    input [5:0] OpCode,

    output RegDst,
    output Jump,
    output Branch,
    output MemRead,
    output MemToReg,
    output MemWrite,
    output ALUSrc,
    output RegWrite,
    output Link,
    output PC_en,
    output JR,
    output [1:0] ALUOp
);
    // Instruction type detection
    wire is_r;
    wire is_lw;
    wire is_sw;
    wire is_beq;
    wire is_jump;
    wire is_jal;
    wire is_jr;
    wire is_addi;

    // One-hot encoding for opcodes
    assign is_r     = (OpCode == 6'b000000);
    assign is_lw    = (OpCode == 6'b100011);
    assign is_sw    = (OpCode == 6'b101011);
    assign is_beq   = (OpCode == 6'b000100);
    assign is_jump  = (OpCode == 6'b000010);
    assign is_jal   = (OpCode == 6'b000011);
    assign is_jr    = (OpCode == 6'b000101);  // Assuming 000101 is used for JR
    assign is_addi  = (OpCode == 6'b001000);

    // Control signal mapping (13 bits)
    assign {RegDst, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch,
            ALUOp[1:0], Jump, Link, JR, PC_en} =

        is_r     ? 13'b100_1000_10_0001 : // R-type
        is_lw    ? 13'b011_1100_00_0001 : // LW // I type
        is_sw    ? 13'b010_0010_00_0001 : // SW // I type 
        is_beq   ? 13'b000_0001_01_0001 : // BEQ
        is_jump  ? 13'b000_0000_00_1001 : // JUMP (to target)
        is_jal   ? 13'b000_1000_00_1101 : // JAL
        is_addi  ? 13'b010_1000_00_0001 : // ADDI // Itype
        is_jr    ? 13'b000_0000_00_0011 : // JR
                   13'b000_0000_11_0000 ; // default (NOP)

endmodule


module alu_control(
    //from dp
    input [5:0] funct,
    //from mips controller
    input [1:0] ALUOp,
    //to ALU ( going via your dp)
    output [3:0] ALU_opcode
);
    wire is_add;
    wire is_sub;
    wire is_and;
    wire is_or;
    wire is_slt;
    //one hot signals
    assign is_add = ((ALUOp==2'b00) || ((ALUOp==2'b10) && (funct==6'b100000))); 
	assign is_sub = ((ALUOp==2'b01) || ((ALUOp==2'b10) && (funct==6'b100010))); 
	assign is_and = ((ALUOp==2'b10) && (funct==6'b100100));
	assign is_or  = ((ALUOp==2'b10) && (funct==6'b100101));
	assign is_slt = ((ALUOp==2'b10) && (funct==6'b101010));

    assign ALU_opcode = is_add ? 4'b0010 : //add
                 is_sub ? 4'b0110 : //sub
                 is_and ? 4'b0000 : //and
                 is_or  ? 4'b0001 : //or
                 is_slt ? 4'b0111 : //slt
                 4'b1111; //default, no operation

endmodule

module ALU (
  input [31:0] a,
  input [31:0] b,
  input [3:0] op,
  output [31:0] result,
  output zero
);
  // Internal wires for operation results
  wire [31:0] and_result;
  wire [31:0] or_result;
  wire [31:0] add_sub_result;
  wire [31:0] slt_result;
  wire [31:0] nor_result;
  
  // Carry chain for addition/subtraction
  wire [32:0] carry;
  wire [31:0] b_input;
  wire Binvert;
  
  // Control signal decoding
  assign Binvert = op[2];  // High for subtraction (op = 0110)
  
  // Generate b_input based on Binvert (invert b for subtraction)
  genvar j;
  generate
    for (j = 0; j < 32; j = j + 1) begin: b_invert_gen
      assign b_input[j] = b[j] ^ Binvert;
    end
  endgenerate
  
  // Set initial carry-in (1 for subtraction to implement 2's complement)
  assign carry[0] = Binvert;
  
  // Basic logic operations
  genvar k;
  generate
    for (k = 0; k < 32; k = k + 1) begin: logic_ops
      assign and_result[k] = a[k] & b[k];
      assign or_result[k] = a[k] | b[k];
      assign nor_result[k] = ~(a[k] | b[k]);
    end
  endgenerate
  
  // Addition/Subtraction with carry chain
  genvar i;
  generate
    for (i = 0; i < 32; i = i + 1) begin: adder_loop
      // Full adder implementation
      assign add_sub_result[i] = a[i] ^ b_input[i] ^ carry[i];
      assign carry[i+1] = (a[i] & b_input[i]) | (a[i] & carry[i]) | (b_input[i] & carry[i]);
    end
  endgenerate
  
  // Set on less than (SLT) - uses the sign bit of subtraction result
  assign slt_result = {31'b0, add_sub_result[31]};
  
  // Operation selection multiplexer
  // Assuming we have a mux module available as mentioned
  mux5to1 result_mux (
    .in0(and_result),
    .in1(or_result),
    .in2(add_sub_result),
    .in3(slt_result),
    .in4(nor_result),
    .sel(op),
    .out(result)
  );
  
  // Zero detection
  assign zero = (result == 32'b0);
endmodule




module instr_mem #(parameter Addr_width = 8) (read_address,instruction);
//asynchronous memory with 256 32-bit locations
//for instruction memory
parameter S=32;
parameter L=(1<<Addr_width);

input [$clog2(L) - 1:0] read_address;
output [(S-1):0] instruction;

reg [S-1:0] memory [L-1:0];
  assign instruction=memory[read_address/4];

//initial begin $readmemh("instr_mem.dat", memory);

initial begin
    memory[0]  = 32'h20080004; // main: addi $t0, $zero, 4
    memory[1]  = 32'h2009000A; // addi $t1, $zero, 10
    memory[2]  = 32'h00005020; // add $t2, $zero, $zero
    memory[3]  = 32'h200B0005; // addi $t3, $zero, 5

    memory[4]  = 32'hAD090000; // store_loop: sw $t1, 0($t0)
    memory[5]  = 32'h21080004; // addi $t0, $t0, 4
    memory[6]  = 32'h214A0001; // addi $t2, $t2, 1
    memory[7]  = 32'h014B702A; // slt $t6, $t2, $t3
    memory[8]  = 32'h11C00001; // beq $t6, $zero, store_done
    memory[9]  = 32'h08000004; // j store_loop

    memory[10] = 32'h20040004; // store_done: addi $a0, $zero, 4
    memory[11] = 32'h200B0005; // addi $a1, $zero, 5
    memory[12] = 32'h0C00000F; // jal sum_array

    memory[13] = 32'hAC020004; // sw $v0, 4($zero)
    memory[14] = 32'hFC000000; // exit (custom opcode 0xFC)

    memory[15] = 32'h00044020; // sum_array: add $t0, $zero, $a0
    memory[16] = 32'h000B4820; // add $t1, $zero, $a1
    memory[17] = 32'h00005020; // add $t2, $zero, $zero
    memory[18] = 32'h00005820; // add $t3, $zero, $zero

    memory[19] = 32'h8D0F0000; // sum_loop: lw $t7, 0($t0)
    memory[20] = 32'h014F5020; // add $t2, $t2, $t7
    memory[21] = 32'h21080004; // addi $t0, $t0, 4
    memory[22] = 32'h216B0001; // addi $t3, $t3, 1
    memory[23] = 32'h0169602A; // slt $t4, $t3, $t1
    memory[24] = 32'h11800001; // beq $t4, $zero, sum_done
    memory[25] = 32'h08000013; // j sum_loop

    memory[26] = 32'h01401020; // sum_done: add $v0, $t2, $zero
    memory[27] = 32'h17E00008; // jr $ra (custom interpretation)
end

endmodule




module mux2to1 #(parameter WIDTH = 32) (
  input [WIDTH-1:0] in0,
  input [WIDTH-1:0] in1,
  input sel,
  output [WIDTH-1:0] out
);
  
  wire sel_n;
  not(sel_n, sel);
  
  genvar i;
  generate
    for(i=0; i<WIDTH; i=i+1) begin: mux_gate
      wire and0_out, and1_out;
      and(and0_out, in0[i], sel_n);
      and(and1_out, in1[i], sel);
      or(out[i], and0_out, and1_out);
    end
  endgenerate
  
endmodule


module adder (
  input [31:0] a,
  input [31:0] b,
  output [31:0] y
);
  wire [31:0] carry;
  
  // First bit addition (using half adder)
  wire ha_and_out;
  xor(y[0], a[0], b[0]);
  and(carry[0], a[0], b[0]);
  
  // Remaining bits (using full adders)
  genvar i;
  generate
    for(i=1; i<32; i=i+1) begin: full_adder_chain
      wire xor1_out, and1_out, and2_out, and3_out;
      
      // Sum calculation
      xor(xor1_out, a[i], b[i]);
      xor(y[i], xor1_out, carry[i-1]);
      
      // Carry calculation
      and(and1_out, a[i], b[i]);
      and(and2_out, a[i], carry[i-1]);
      and(and3_out, b[i], carry[i-1]);
      or(carry[i], and1_out, and2_out, and3_out);
    end
  endgenerate
endmodule



module mux2 (
  input  sel,
  input  in0,
  input  in1,
  output out
);
  assign out = sel ? in1 : in0;
endmodule



module dff (
  input  clk,
  input  reset,  // active‑high synchronous reset
  input  d,      
  output q
);
  wire d_int;       // data after reset‑mux
  wire inv_clk;     // inverted clock
  wire master_q;    // master‑latch output
  wire slave_q;     // slave‑latch output
  mux2 reset_mux (
    .sel  (reset),
    .in0  (d),
    .in1  (1'b0),
    .out  (d_int)
  );
  not inv1 (inv_clk, clk);
  mux2 master_mux (
    .sel  (inv_clk),
    .in0  (master_q),
    .in1  (d_int),
    .out  (master_q)
  );
  mux2 slave_mux (
    .sel  (clk),
    .in0  (slave_q),
    .in1  (master_q),
    .out  (slave_q)
  );
  assign q = slave_q;
endmodule



module decoder5to32 (
  input [4:0] in,
  input en,
  output [31:0] out
);
  assign out = en ? (32'b1 << in) : 32'b0;
endmodule




module mux32_1 (
  input  [1023:0] in_flat,   
  input  [4:0]    sel,
  output [31:0]   out
);
  // Extract the 32‑bit word at index sel
  assign out = in_flat[ sel*32 +: 32 ];
endmodule




module register32 (
  input clk,
  input reset,
  input write_en,
  input [31:0] d,
  output [31:0] q
);
  genvar i;
  generate
    for (i = 0; i < 32; i = i + 1) begin : reg_loop
      dff dff_inst (
        .clk(clk),
        .reset(reset),
        .d(write_en ? d[i] : q[i]),
        .q(q[i])
      );
    end
  endgenerate
endmodule




module sign_extend (
  input [15:0] in,
  output [31:0] out
);
  
  // Sign extension: replicate the most significant bit (in[15]) to fill the upper 16 bits
  assign out = {{16{in[15]}}, in};
  
endmodule





module sll_2_same (
  input [31:0] in,
  output [31:0] out
);
  
  // Shift left logical by 2 bits (multiply by 4)
  // The two least significant bits become 0, and the two most significant bits are discarded
  assign out = {in[29:0], 2'b00};
  
endmodule






module sll_2_pad #(parameter INP_WIDTH=26) (
  input [INP_WIDTH-1:0] in,
  output [INP_WIDTH+1:0] out
);
  
  // Shift left logical by 2 bits and pad with zeros
  // This increases the width by 2 bits
  assign out = {in, 2'b00};
  
endmodule




module mux5to1 (
  input [31:0] in0,      // AND result (000)
  input [31:0] in1,      // OR result (001)
  input [31:0] in2,      // ADD/SUB result (010/110)
  input [31:0] in3,      // SLT result (111)
  input [31:0] in4,      // NOR result (1100)
  input [3:0] sel,       // Operation select (from ALU control)
  output [31:0] out      // Selected result
);
  // Decode the operation select signals using pure gate-level logic
  wire sel_and, sel_or, sel_add, sel_sub, sel_slt, sel_nor;
  
  // AND: 0000
  assign sel_and = ~sel[3] & ~sel[2] & ~sel[1] & ~sel[0];
  
  // OR: 0001
  assign sel_or = ~sel[3] & ~sel[2] & ~sel[1] & sel[0];
  
  // ADD: 0010
  assign sel_add = ~sel[3] & ~sel[2] & sel[1] & ~sel[0];
  
  // SUB: 0110
  assign sel_sub = ~sel[3] & sel[2] & sel[1] & ~sel[0];
  
  // SLT: 0111
  assign sel_slt = ~sel[3] & sel[2] & sel[1] & sel[0];
  
  // NOR: 1100
  assign sel_nor = sel[3] & sel[2] & ~sel[1] & ~sel[0];
  
  // Combine ADD and SUB for the adder/subtractor result
  wire sel_add_sub = sel_add | sel_sub;
  
  // Select the appropriate input for each bit
  genvar i;
  generate
    for (i = 0; i < 32; i = i + 1) begin: mux_bits
      // Gate each input with its select signal
      wire [4:0] gated_inputs;
      
      assign gated_inputs[0] = in0[i] & sel_and;
      assign gated_inputs[1] = in1[i] & sel_or;
      assign gated_inputs[2] = in2[i] & sel_add_sub;  // Combined ADD/SUB
      assign gated_inputs[3] = in3[i] & sel_slt;
      assign gated_inputs[4] = in4[i] & sel_nor;
      
      // OR all gated inputs to produce the output
      assign out[i] = gated_inputs[0] | gated_inputs[1] | gated_inputs[2] | 
                     gated_inputs[3] | gated_inputs[4];
    end
  endgenerate
endmodule

module data_mem #(parameter L = 256)(clk,address,read_data, write_data, read_en, write_en);
//synchronous memory with 256 32-bit locations

parameter S=32; 


input [$clog2(L) - 1:0] address;
input [S-1:0] write_data;
input clk;
input write_en;
input read_en;
output [(S-1):0] read_data;

reg [S-1:0] memory [L-1:0];

  assign read_data=memory[address/4];
  
always @(posedge clk) begin
	if (write_en==1) begin
      memory[address/4]<=write_data;
	end
end

// the current program doesn't require the data memory to be loaded
// initial $readmemh("memdata.dat", memory);

endmodule




module reg_file (
  input         clk,
  input         reset,
  input         write_en,
  input  [4:0]  read_reg1,
  input  [4:0]  read_reg2,
  input  [4:0]  write_reg,
  input  [31:0] write_data,
  output [31:0] read_data1,
  output [31:0] read_data2
);
  wire [31:0]        write_decode; 
  wire [1023:0]      regs_flat;     // 32×32-bit wide

  // one-hot decode
  decoder5to32 dec (
    .in  (write_reg),
    .en  (write_en),
    .out (write_decode)
  );

  // 32× register32 instances, each hooked to a 32‑bit slice
  genvar i;
  generate
    for (i = 0; i < 32; i = i + 1) begin : regs
      register32 r (
        .clk     (clk),
        .reset   (reset),
        .write_en(write_decode[i]),
        .d       (write_data),
        .q       (regs_flat[i*32 +: 32])
      );
    end
  endgenerate

  // two read ports via the same flattened‐bus mux
  mux32_1 m1 (
    .in_flat(regs_flat),
    .sel    (read_reg1),
    .out    (read_data1)
  );
  mux32_1 m2 (
    .in_flat(regs_flat),
    .sel    (read_reg2),
    .out    (read_data2)
  );
endmodule





module PC (
  input clk,
  input reset,  // Active high reset
  input en,
  input [31:0] d,
  output reg [31:0] q
);
  
  // Internal wire for next state logic
  wire [31:0] next_q;
  
  // Next state logic with enable functionality
  assign next_q = en ? d : q;
  
  // D flip-flop implementation with synchronous active-high reset
  always @(posedge clk) begin
    if (reset)
      q <= 32'b0;
    else
      q <= next_q;
  end
  
endmodule

module top_MIPS(
  input reset,
  input clk
);

  // Internal control wires
  wire [5:0] OpCode, funct;
  wire [3:0] ALU_opcode;
  wire [1:0] ALUOp;
  wire RegDst, Jump, Branch, MemRead, MemToReg;
  wire MemWrite, ALUSrc, RegWrite, Link, JR, PC_en;

  // Data path instantiation
  MIPS_datapath datapath (
    .clk(clk),
    .reset(reset),
    .RegDst(RegDst),
    .Jump(Jump),
    .Branch(Branch),
    .MemRead(MemRead),
    .MemToReg(MemToReg),
    .MemWrite(MemWrite),
    .ALUSrc(ALUSrc),
    .RegWrite(RegWrite),
    .Link(Link),
    .JR(JR),
    .PC_en(PC_en),
    .ALU_opcode(ALU_opcode),
    .OpCode(OpCode),
    .funct(funct)
  );

  // Main controller
  controller ctrl (
    .clk(clk),
    .reset(reset),
    .OpCode(OpCode),
    .RegDst(RegDst),
    .Jump(Jump),
    .Branch(Branch),
    .MemRead(MemRead),
    .MemToReg(MemToReg),
    .MemWrite(MemWrite),
    .ALUSrc(ALUSrc),
    .RegWrite(RegWrite),
    .Link(Link),
    .PC_en(PC_en),
    .JR(JR),
    .ALUOp(ALUOp)
  );

  // ALU control unit
  alu_control alu_ctrl (
    .funct(funct),
    .ALUOp(ALUOp),
    .ALU_opcode(ALU_opcode)
  );

endmodule