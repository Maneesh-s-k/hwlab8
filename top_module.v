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