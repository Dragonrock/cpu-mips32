`include "constants.h"

/************** Main control in ID pipe stage  *************/
module control_main(output reg RegDst,
                output reg Branch,  
                output reg MemRead,
                output reg MemWrite,  
                output reg MemToReg,  
                output reg ALUSrc,  
                output reg RegWrite,
                output reg BranchCond,  
                output reg [1:0] ALUcntrl,  
                input [5:0] opcode);

  always @(*) 
   begin
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
        ALUcntrl = 2'b10;
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
        ALUcntrl = 2'b00;
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
        ALUcntrl = 2'b00;
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
        ALUcntrl = 2'b01;
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
        ALUcntrl = 2'b01;
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
        ALUcntrl = 2'b00;
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
        ALUcntrl = 2'bxx;
      end
    endcase
    end // always
endmodule


/**************** Module for Bypass Detection in EX pipe stage goes here  *********/
// TO FILL IN: Module details 
          
                       

/**************** Module for Stall Detection in ID pipe stage goes here  *********/
// TO FILL IN: Module details 
module Hazard_Unit(input ID/EX.RegisterRt,
                   input ID/EX.MemRead,
                   input Rs,
                   input Rt,
                   output PCWrite,
                   output selector,   //selector bit for multiplexer
                   output IF/IDWrite);
  always @(*) 
    begin
      if (ID/EX.MemRead == 1 && (ID/EX.RegisterRt == IF/ID.RegisterRs || ID/EX.RegisterRt == IF/ID.RegisterRt))
        begin
          PCWrite = 1'b0; //Write Enable for PC = 0
          IF/IDWrite = 1'b0; //Write Enable for IF/ID = 0
          selector = 1'b0;
        end
    end

endmodule
                       
/************** control for ALU control in EX pipe stage  *************/
module control_alu(output reg [3:0] ALUOp,                  
               input [1:0] ALUcntrl,
               input [5:0] func);

  always @(ALUcntrl or func)  
    begin
      case (ALUcntrl)
        2'b10: 
           begin
             case (func)
              6'b100000: ALUOp  = 4'b0010; // add
              6'b100010: ALUOp = 4'b0110; // sub
              6'b100100: ALUOp = 4'b0000; // and
              6'b100101: ALUOp = 4'b0001; // or
              6'b100111: ALUOp = 4'b1100; // nor
              6'b101010: ALUOp = 4'b0111; // slt
              default: ALUOp = 4'b0000;       
             endcase 
          end   
        2'b00: 
              ALUOp  = 4'b0010; // add
        2'b01: 
              ALUOp = 4'b0110; // sub
        default:
              ALUOp = 4'b0000;
     endcase
    end
endmodule
