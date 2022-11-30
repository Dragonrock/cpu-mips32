
`include "constants.h"
`timescale 1ns/1ps


/************** Main control in ID pipe stage  *************/
module control_main(output reg RegDst,
                output reg Branch,  
                output reg MemRead,
                output reg MemWrite,  
                output reg MemToReg,  
                output reg ALUSrc,  
                output reg RegWrite,
                output reg [1:0] ALUcntrl,  
                input [5:0] opcode);

  always @(*) 
   begin
     case (opcode)
      `R_FORMAT: begin
        RegWrite = 1'b1;
        RegDst = 1'b1;

        ALUSrc = 1'b0;
        Branch = 1'b0;
        MemWrite = 1'b0;
        MemRead = 1'b0;
        MemToReg = 1'b0;
        ALUcntrl = 2'b10;
      end

      `LW: begin
        RegWrite = 1'b1;
        RegDst = 1'b0;
        ALUSrc = 1'b1;
        Branch = 1'b0;
        MemWrite = 1'b0;
        MemRead = 1'b1;
        MemToReg = 1'b1;
        ALUcntrl = 2'b00;
      end

      `SW: begin
        RegWrite = 1'b0;
        RegDst = 1'bx;
        ALUSrc = 1'b1;
        Branch = 1'b0;
        MemWrite = 1'b1;
        MemRead = 1'b0;
        MemToReg = 1'bx;
        ALUcntrl = 2'b00;
      end
      
      `BEQ: begin
        RegWrite = 1'b0;
        RegDst = 1'bx;
        ALUSrc = 1'b0;
        Branch = 1'b1;
        MemWrite = 1'b0;
        MemRead = 1'b0;
        MemToReg = 1'bx;
        ALUcntrl = 2'b01;
      end

      `BNE: begin
        RegWrite = 1'b0;
        RegDst = 1'bx;
        ALUSrc = 1'b0;
        Branch = 1'b1;
        MemWrite = 1'b0;
        MemRead = 1'b0;
        MemToReg = 1'bx;
        ALUcntrl = 2'b01;
      end
      `ADDI: begin
        RegWrite = 1'b1;
        RegDst = 1'b0;
        ALUSrc = 1'b1;
        Branch = 1'b0;
        MemWrite = 1'b0;
        MemRead = 1'b0;
        MemToReg = 1'b0;
        ALUcntrl = 2'b00;
      end
      
      default: begin
        RegWrite = 1'bx;
        RegDst = 1'bx;
        ALUSrc = 1'bx;
        Branch = 1'bx;
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
module Forward_Unit (input [4:0] EX_MEM_RegisterRd,
                     input EX_MEM_RegWrite,
                     input [4:0] MEM_WB_RegisterRd,
                     input MEM_WB_RegWrite,
                     input [4:0] ID_EX_RegisterRt,
                     input [4:0] ID_EX_RegisterRs,
                     output reg [1:0] ForwardA,
                     output reg [1:0] ForwardB);

  always @(*)
    begin

      //Initializing forwarding signals to zero
      {ForwardA, ForwardB} <= 0;

      
      if ((MEM_WB_RegWrite && MEM_WB_RegisterRd) && 
          (MEM_WB_RegisterRd == ID_EX_RegisterRs))
        ForwardA <= 'd1;
        
      else if ((EX_MEM_RegWrite && EX_MEM_RegisterRd) && 
              (EX_MEM_RegisterRd == ID_EX_RegisterRs))
        ForwardA <= 'd2;

      if ((MEM_WB_RegWrite && MEM_WB_RegisterRd) && 
          (MEM_WB_RegisterRd == ID_EX_RegisterRt))
        ForwardB <= 'd1;
      
      else if ((EX_MEM_RegWrite && EX_MEM_RegisterRd) && 
               (EX_MEM_RegisterRd == ID_EX_RegisterRt))
        ForwardB <= 'd2;

    end

endmodule
                       

/**************** Module for Stall Detection in ID pipe stage goes here  *********/
// TO FILL IN: Module details 
module Hazard_Unit(input [4:0] ID_EX_RegisterRt,
                   input ID_EX_MemRead,
                   input [4:0] IF_ID_RegisterRs,
                   input [4:0] IF_ID_RegisterRt,
                   output reg PCWrite,
                   output reg IDEX_Control, //selector bit for multiplexer
                   output reg IFID_Write);
  always @(*) 
    begin
      if ((ID_EX_MemRead) && 
          ((ID_EX_RegisterRt == IF_ID_RegisterRs) || 
          (ID_EX_RegisterRt == IF_ID_RegisterRt)))
        begin
          PCWrite <= 1'b0;   //Write Enable for PC = 0
          IFID_Write <= 1'b0;  //Write Enable for IF/ID = 0
          IDEX_Control <= 1'b0;
        end
      else
        begin
          PCWrite <= 1'b1;
          IFID_Write <= 1'b1;
          IDEX_Control <= 1'b1;
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