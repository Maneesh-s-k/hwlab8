`timescale 1ns / 1ps


// Code your testbench here
// or browse Examples
`timescale 1ns / 1ps

module tb_top_MIPS();

  reg clk;
  reg reset;
  
  // Instantiate the MIPS processor
  top_MIPS uut(
    .reset(reset),
    .clk(clk)
  );

  // Clock generation (100 MHz)
  always #5 clk = ~clk;

  // Reset and stimulus
  initial begin

    clk = 0;
    reset =1;
    $monitor(
  "Time=%0t | Reset=%b | PC=%0d\n\
R0=%0d R1=%0d R2=%0d R3=%0d R4=%0d R5=%0d R6=%0d R7=%0d\n\
R8=%0d R9=%0d R10=%0d R11=%0d R12=%0d R13=%0d R14=%0d R15=%0d\n\
R16=%0d R17=%0d R18=%0d R19=%0d R20=%0d R21=%0d R22=%0d R23=%0d\n\
R24=%0d R25=%0d R26=%0d R27=%0d R28=%0d R29=%0d R30=%0d R31=%0d",
  $time, reset, uut.datapath.PC_out,
  uut.datapath.Registers.regs_flat[0 +: 32],   uut.datapath.Registers.regs_flat[32 +: 32],
  uut.datapath.Registers.regs_flat[64 +: 32],  uut.datapath.Registers.regs_flat[96 +: 32],
  uut.datapath.Registers.regs_flat[128 +: 32], uut.datapath.Registers.regs_flat[160 +: 32],
  uut.datapath.Registers.regs_flat[192 +: 32], uut.datapath.Registers.regs_flat[224 +: 32],
  uut.datapath.Registers.regs_flat[256 +: 32], uut.datapath.Registers.regs_flat[288 +: 32],
  uut.datapath.Registers.regs_flat[320 +: 32], uut.datapath.Registers.regs_flat[352 +: 32],
  uut.datapath.Registers.regs_flat[384 +: 32], uut.datapath.Registers.regs_flat[416 +: 32],
  uut.datapath.Registers.regs_flat[448 +: 32], uut.datapath.Registers.regs_flat[480 +: 32],
  uut.datapath.Registers.regs_flat[512 +: 32], uut.datapath.Registers.regs_flat[544 +: 32],
  uut.datapath.Registers.regs_flat[576 +: 32], uut.datapath.Registers.regs_flat[608 +: 32],
  uut.datapath.Registers.regs_flat[640 +: 32], uut.datapath.Registers.regs_flat[672 +: 32],
  uut.datapath.Registers.regs_flat[704 +: 32], uut.datapath.Registers.regs_flat[736 +: 32],
  uut.datapath.Registers.regs_flat[768 +: 32], uut.datapath.Registers.regs_flat[800 +: 32],
  uut.datapath.Registers.regs_flat[832 +: 32], uut.datapath.Registers.regs_flat[864 +: 32],
  uut.datapath.Registers.regs_flat[896 +: 32], uut.datapath.Registers.regs_flat[928 +: 32],
  uut.datapath.Registers.regs_flat[960 +: 32], uut.datapath.Registers.regs_flat[992 +: 32]
);
    $dumpfile("mips_waveform.vcd");
    $dumpvars(0, tb_top_MIPS.uut.datapath.Registers);
    
    // Release reset after 20ns
    #20 reset = 0;
    
    // Run simulation for 200ns
    #900 $finish;
  end

endmodule