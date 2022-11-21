`include "lab4_library_input.v"
`timescale 1ns/1ps


module CPU( input clock,reset);  

 reg[31:0] pc; 
 wire signed [31:0] pc_new, pc_step, pc_next;  
 wire [31:0] instr, extended, SrcB;  
 wire RegWrite, RegDst, AluSrc, Branch, MemWrite, MemToReg, MemRead, BranchCond, zero, pcDefiner;  
 wire [3:0] aluControl;
 wire [4:0] ra_registers_A, ra_registers_B, wa;
 wire  [31:0] rd_registers_A, rd_registers_B;
 wire [31:0] ALU_out, writeRegData, writeMemData, readData;
 wire [31:0] instructions;
 wire [4:0] writenReg;

 Memory cpu_IMem(clock, reset, 1'b1, 1'b0, pc >> 2, 32'b0, instructions);

 Memory cpu_DataMem(clock, reset, MemRead , MemWrite, ALU_out, rd_registers_B, readData);

 fsm fsMachine(RegWrite, RegDst, AluSrc, Branch, MemWrite, MemRead, MemToReg, BranchCond, aluControl, instructions[31:26], instructions[5:0]);
 
 RegFile cpu_regs(clock, reset,  instructions[25:21],  instructions[20:16], writenReg, RegWrite, writeRegData, rd_registers_A, rd_registers_B);

 ALU alu(ALU_out, zero, rd_registers_A, SrcB, aluControl);


 //Extension
 assign extended = { {16{instructions[15]}}, instructions[15:0]} ;
 
//Signal Multiplexers

 assign SrcB= (AluSrc == 1'b1) ? extended : rd_registers_B;
 
 assign writenReg = (RegDst == 1'b1) ? instructions[15:11] : instructions[20:16];
 
 assign writeRegData = (MemToReg == 1'b1) ? readData : ALU_out;

 // PC   
 always @(posedge clock or negedge reset)  
 begin   
      if(!reset)  
           pc <= 32'h0;  
      else  
           pc <= pc_next;  
 end


 // Calculate next pc location  
 assign pcDefiner = (BranchCond == 1) ? ~zero : zero;
 assign pc_step =  pc + 32'h4;  
 assign pc_next = (!reset) ? 32'h0 : ((Branch && pcDefiner) ? ((extended << 2)  + pc_step)  : pc_step);

endmodule