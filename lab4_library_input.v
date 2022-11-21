// This file contains library modules to be used in your design. 

`include "constants.h"
`timescale 1ns/1ps

// Small ALU. 
//     Inputs: inA, inB, op. 
//     Output: out, zero
// Operations: bitwise and (op = 0)
//             bitwise or  (op = 1)
//             addition (op = 2)
//             subtraction (op = 6)
//             slt  (op = 7)
//             nor (op = 12)
module ALU (out, zero, inA, inB, op);

parameter AND = 0;
  parameter OR = 1;
  parameter ADD = 2;
  parameter SUB = 6;
  parameter SLT = 7;
  parameter NOR = 12;
  parameter N = 32;
  
  output [N-1:0] out;
  output zero;
  input signed [N-1:0] inA, inB;
  input [3:0] op;

  //super assign = one less register
  assign out = (op == AND) ? (inA & inB) :  ((op == OR) ? (inA | inB) : ((op == ADD) ? (inA + inB) : ((op== SUB) ? (inA - inB) : ((op == SLT) ? ((inA < inB) ? 'b1 : 'b0) : ((op == NOR) ? ~(inA | inB) : 'bx)))));
  assign zero = (out == 0);  // if out equals to 0 then zero = 1

endmodule


// Memory (active 1024 words, from 10 address ).
// Read : enable ren, address addr, data dout
// Write: enable wen, address addr, data din.
module Memory (clock, reset, ren, wen, addr, din, dout);
  input         clock, reset, ren, wen;
  input  [31:0] addr, din;
  wire [31:0] final_addr;
  output [31:0] dout;

  reg [31:0] data[4095:0];
  wire [31:0] dout;
  
  assign  final_addr = addr ;//>> 2;
  assign dout = ((wen==1'b0) && (ren==1'b1)) ? data[final_addr[9:0]] : 32'bx;
  
  always @(ren or wen)   // It does not correspond to hardware. Just for error detection
    if (ren & wen)
      $display ("\nMemory ERROR (time %0d): ren and wen both active!\n", $time);

  always @(posedge ren or posedge wen) begin // It does not correspond to hardware. Just for error detection
    if (final_addr[31:10] != 0)
      $display("Memory WARNING (time %0d): address msbs are not zero\n", $time);
  end     
  
  always @(negedge clock)
   begin
    if (!reset);
    else if((wen == 1'b1) && (ren==1'b0))
        data[final_addr[9:0]] = din;
   end

endmodule


// Register File. Input ports: address raA, data rdA
//                            address raB, data rdB
//                Write port: address wa, data wd, enable wen.
module RegFile (clock, reset, raA, raB, wa, wen, wd, rdA, rdB);

 input clock, reset, wen;
  input [4:0] raA, raB, wa;
  input signed [31:0] wd;
  output signed [31:0] rdA, rdB;
  integer i;
  reg [31:0] data[31:0];


always @(negedge clock, negedge reset) begin
  if(!reset) begin
    for(i = 0; i <= 31; i = i + 1) begin
      data[i] <= 32'b0;  //set to zero all registers simultenusly
    end
  end
  else begin
    if(wen == 1) begin
      data[wa] <= wd;
    end
  end
end

//asynchronous reading from registers (even with reset)
assign rdA = data[raA];
assign rdB = data[raB];

endmodule



// Module to control the data path. 
//                          Input: op, func of the inpstruction
//                          Output: all the control signals needed 
		   
module fsm(output RegWrite, RegDst, AluSrc, Branch, MemWrite, MemRead, MemToReg, BranchCond,  output reg [3:0] alu_Control, input [5:0] opcode, input [5:0] func);
  reg RegWrite, RegDst, AluSrc, Branch, MemWrite, MemToReg, MemRead, BranchCond;
  reg [1:0] ALUOp;

  always @(opcode) begin
    case (opcode)
      `R_FORMAT: begin
        RegWrite = 1'b1;
        RegDst = 1'b1;
        AluSrc = 1'b0;
        Branch = 1'b0;
        BranchCond= 1'b0;
        MemWrite = 1'b0;
        MemRead = 1'b0;
        MemToReg = 1'b0;
        ALUOp = 2'b10;
      end
      
      `LW: begin
        RegWrite = 1'b1;
        RegDst = 1'b0;
        AluSrc = 1'b1;
        Branch = 1'b0;
        BranchCond= 1'b0;
        MemWrite = 1'b0;
        MemRead = 1'b1;
        MemToReg = 1'b1;
        ALUOp = 2'b00;
      end
      
      `SW: begin
        RegWrite = 1'b0;
        RegDst = 1'bx;
        AluSrc = 1'b1;
        Branch = 1'b0;
        BranchCond= 1'b0;
        MemWrite = 1'b1;
        MemRead = 1'b0;
        MemToReg = 1'bx;
        ALUOp = 2'b00;
      end
      
      `BEQ: begin
        RegWrite = 1'b0;
        RegDst = 1'bx;
        AluSrc = 1'b0;
        Branch = 1'b1;
        BranchCond= 1'b0;
        MemWrite = 1'b0;
        MemRead = 1'b0;
        MemToReg = 1'bx;
        ALUOp = 2'b01;
      end
      
      `BNE: begin
        RegWrite = 1'b0;
        RegDst = 1'bx;
        AluSrc = 1'b0;
        Branch = 1'b1;
        BranchCond= 1'b1;
        MemWrite = 1'b0;
        MemRead = 1'b0;
        MemToReg = 1'bx;
        ALUOp = 2'b01;
      end
      
      `ADDI: begin
        RegWrite = 1'b1;
        RegDst = 1'b0;
        AluSrc = 1'b1;
        Branch = 1'b0;
        BranchCond= 1'b0;
        MemWrite = 1'b0;
        MemRead = 1'b0;
        MemToReg = 1'b0;
        ALUOp = 2'b00;
      end
      
      default: begin
        RegWrite = 1'bx;
        RegDst = 1'bx;
        AluSrc = 1'bx;
        Branch = 1'bx;
        BranchCond= 1'bx;
        MemWrite = 1'bx;
        MemRead = 1'b0;
        MemToReg = 1'bx;
        ALUOp = 2'bxx;
      end
    endcase
  end
 
 always @(ALUOp or func) begin
   casez ({ALUOp, func})
    8'b00??????: alu_Control = 4'b0010; //ADD for lw/sw
    8'b01??????: alu_Control = 4'b0110; //SUB for beq/bne
    8'b10100100: alu_Control = 4'b0000; //AND
    8'b10100101: alu_Control = 4'b0001; //OR
    8'b10100000: alu_Control = 4'b0010; //ADD
    8'b10100010: alu_Control = 4'b0110; //SUB
    8'b10101010: alu_Control = 4'b0111; //SLT
    default: alu_Control = 4'bxxxx;
   endcase
 end

endmodule


