/***********************************************************************************************/
/*********************************  MIPS 5-stage pipeline implementation ***********************/
/***********************************************************************************************/
`timescale 1ns/1ps

module cpu(input clock, input reset);
 reg [31:0] PC; 
 reg [31:0] IFID_PCplus4;
 reg [31:0] IFID_instr;
 reg [31:0]  IDEX_signExtend, IDEX_PCplus4;
 reg [4:0]  IDEX_instr_rt, IDEX_instr_rs, IDEX_instr_rd, IDEX_Shamt;                            
 reg        IDEX_RegDst, IDEX_ALUSrc;
 reg [1:0]  IDEX_ALUcntrl;
 reg        IDEX_Branch, IDEX_BranchCond, IDEX_MemRead, IDEX_MemWrite; 
 reg        IDEX_MemToReg, IDEX_RegWrite;                
 reg [4:0]  EXMEM_RegWriteAddr, EXMEM_instr_rd; 
 reg [31:0] EXMEM_ALUOut, EXMEM_PCBranch;
 reg        EXMEM_Zero;
 reg [31:0] EXMEM_MemWriteData;
 reg        EXMEM_Branch, EXMEM_BranchCond, EXMEM_MemRead, EXMEM_MemWrite, EXMEM_RegWrite, EXMEM_MemToReg;
 reg [31:0] MEMWB_DMemOut;
 reg [4:0]  MEMWB_RegWriteAddr, MEMWB_instr_rd; 
 reg [31:0] MEMWB_ALUOut;
 reg        MEMWB_MemToReg, MEMWB_RegWrite;      

 wire [31:0] IDEX_rdA, IDEX_rdB;         
 wire [31:0] instr, ALUInA, ALUInB, ALUOut, signExtend, DMemOut, wRegData, PCIncr, ALUShamt;
 wire Zero, RegDst, MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, Branch;
 wire [5:0] opcode, func;
 wire [4:0] instr_rs, instr_rt, instr_rd, RegWriteAddr;
 wire [3:0] ALUOp;
 wire [1:0] ALUcntrl;
 wire [15:0] imm;

 wire PCWrite, bubble_idex, IFID_Write, BranchCond, PCSrc, Jump;
 wire [1:0] ForwardA, ForwardB;
 wire [31:0] EX_RegB, targetPc;
 
 

/***************** Instruction Fetch Unit (IF)  ****************/
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)     
       PC <= -1;     
    else if (PC == -1)
       PC <= 0;
    else if(PCWrite )begin
      if((Jump &&(IDEX_Branch || IDEX_BranchCond)));
        // case for branch and then jump...stall jump to check for branch
      else if(PCSrc)
        PC <= EXMEM_PCBranch;
      else if(Jump)
        PC <= targetPc << 2;
      else
       PC <= PC + 4;
    end
  end
  



  // IFID pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0 || Jump)  begin
      if ((reset == 1'b0) || ((IDEX_Branch || IDEX_BranchCond) == 1'b0)) begin
        IFID_PCplus4 <= 32'b0;    
        IFID_instr <= 32'b0;
      end
    end 

    else if (IFID_Write)
      begin
       IFID_PCplus4 <= PC + 32'd4;
       IFID_instr <= instr;
    end
  end
  
//Instantiate the Instruction Memory here //done
Memory cpu_IMem(clock, reset, 1'b1, 1'b0, PC >> 2, 32'b0, instr);
  
  

/***************** Instruction Decode Unit (ID)  ****************/
assign opcode = IFID_instr[31:26];
assign func = IFID_instr[5:0];
assign instr_rs = IFID_instr[25:21];
assign instr_rt = IFID_instr[20:16];
assign instr_rd = IFID_instr[15:11];
assign imm = IFID_instr[15:0];
assign signExtend = {{16{imm[15]}}, imm};
assign ALUShamt = IFID_instr[10:6];
assign targetPc = IFID_instr[25:0];

// Register file
RegFile cpu_regs(clock,
                 reset,
                 instr_rs,
                 instr_rt,
                 MEMWB_RegWriteAddr,
                 MEMWB_RegWrite,
                 wRegData,
                 IDEX_rdA,
                 IDEX_rdB);

  // IDEX pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if ((reset == 1'b0) || (bubble_idex))
      begin
       IDEX_signExtend <= 32'b0;
       IDEX_instr_rd <= 5'b0;
       IDEX_instr_rs <= 5'b0;
       IDEX_instr_rt <= 5'b0;
       IDEX_RegDst <= 1'b0;
       IDEX_ALUcntrl <= 2'b0;
       IDEX_ALUSrc <= 1'b0;
       IDEX_Branch <= 1'b0;
       IDEX_MemRead <= 1'b0;
       IDEX_MemWrite <= 1'b0;
       IDEX_MemToReg <= 1'b0;                  
       IDEX_RegWrite <= 1'b0;
       IDEX_Shamt <= 1'b0;
       IDEX_PCplus4 <= 1'b0;
    end 
    else 
      begin
       IDEX_signExtend <= signExtend;
       IDEX_instr_rd <= instr_rd;
       IDEX_instr_rs <= instr_rs;
       IDEX_instr_rt <= instr_rt;
       IDEX_RegDst <= RegDst;
       IDEX_ALUcntrl <= ALUcntrl;
       IDEX_ALUSrc <= ALUSrc;
       IDEX_Branch <= Branch;
       IDEX_BranchCond <= BranchCond;
       IDEX_MemRead <= MemRead;
       IDEX_MemWrite <= MemWrite;
       IDEX_MemToReg <= MemToReg;                  
       IDEX_RegWrite <= RegWrite;
       IDEX_Shamt <= ALUShamt;
       IDEX_PCplus4 <= IFID_PCplus4;
    end
  end

// Main Control Unit 
control_main control_main (RegDst,
                  Branch,
                  MemRead,
                  MemWrite,
                  MemToReg,
                  ALUSrc,
                  RegWrite,
                  BranchCond,
                  ALUcntrl,
                  Jump,
                  opcode);
                  
//Instantiation of Control Unit that generates stalls
Hazard_Unit hazards(PCSrc,
                    IDEX_instr_rt,
                    IDEX_MemRead, 
                    instr_rs,
                    instr_rt,
                    PCWrite,
                    bubble_idex,
                    IFID_Write);


                           
/***************** Execution Unit (EX)  ****************/
                 
assign ALUInA =   (ForwardA == 0) ? IDEX_rdA :
                  (ForwardA == 1) ? wRegData : EXMEM_ALUOut;

assign EX_RegB =  (ForwardB == 0) ? IDEX_rdB :
                  (ForwardB == 1) ? wRegData : EXMEM_ALUOut;

assign ALUInB = (IDEX_ALUSrc == 1'b0) ? EX_RegB : IDEX_signExtend;

assign PCIncr = (IDEX_PCplus4 + (IDEX_signExtend << 2));

assign PCSrc = ((EXMEM_Branch && EXMEM_Zero) || (EXMEM_BranchCond && (!EXMEM_Zero))) ? 1 : 0;

//  ALU
ALU  #(32) cpu_alu(ALUOut, Zero, ALUInA, ALUInB, ALUOp, IDEX_Shamt);

assign RegWriteAddr = (IDEX_RegDst==1'b0) ? IDEX_instr_rt : IDEX_instr_rd;

 // EXMEM pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if ((reset == 1'b0) || (bubble_idex && PCSrc))
      begin
       EXMEM_ALUOut <= 32'b0;    
       EXMEM_RegWriteAddr <= 5'b0;
       EXMEM_MemWriteData <= 32'b0;
       EXMEM_Zero <= 1'b0;
       EXMEM_Branch <= 1'b0;
       EXMEM_MemRead <= 1'b0;
       EXMEM_MemWrite <= 1'b0;
       EXMEM_MemToReg <= 1'b0;                  
       EXMEM_RegWrite <= 1'b0;
       EXMEM_PCBranch <= 1'b0;
       EXMEM_BranchCond <= 1'b0;
      end 
    else 
      begin
       EXMEM_ALUOut <= ALUOut;    
       EXMEM_RegWriteAddr <= RegWriteAddr;
       EXMEM_MemWriteData <= EX_RegB; // was IDEX_rdB
       EXMEM_Zero <= Zero;
       EXMEM_Branch <= IDEX_Branch;
       EXMEM_MemRead <= IDEX_MemRead;
       EXMEM_MemWrite <= IDEX_MemWrite;
       EXMEM_MemToReg <= IDEX_MemToReg;                  
       EXMEM_RegWrite <= IDEX_RegWrite;
       EXMEM_PCBranch <= PCIncr;
       EXMEM_BranchCond <= IDEX_BranchCond;
      end
  end
  
  // ALU control
  control_alu control_alu(ALUOp, IDEX_ALUcntrl, IDEX_signExtend[5:0]);
  
   // Instantiation of control logic for Forwarding goes here
  Forward_Unit forward(EXMEM_RegWriteAddr,
                       EXMEM_RegWrite,
                       MEMWB_RegWriteAddr,
                       MEMWB_RegWrite,
                       IDEX_instr_rt,
                       IDEX_instr_rs,
                       ForwardA,
                       ForwardB);

  
  
  
/***************** Memory Unit (MEM)  ****************/  

// Data memory 1KB
// Instantiate the Data Memory here  
 Memory cpu_DMem(clock,
                 reset,
                 EXMEM_MemRead,
                 EXMEM_MemWrite,
                 EXMEM_ALUOut,EXMEM_MemWriteData,
                 DMemOut);


// MEMWB pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)     
      begin
       MEMWB_DMemOut <= 32'b0;    
       MEMWB_ALUOut <= 32'b0;
       MEMWB_RegWriteAddr <= 5'b0;
       MEMWB_MemToReg <= 1'b0;                  
       MEMWB_RegWrite <= 1'b0;
      end 
    else 
      begin
       MEMWB_DMemOut <= DMemOut;
       MEMWB_ALUOut <= EXMEM_ALUOut;
       MEMWB_RegWriteAddr <= EXMEM_RegWriteAddr;
       MEMWB_MemToReg <= EXMEM_MemToReg;                  
       MEMWB_RegWrite <= EXMEM_RegWrite;
      end
  end

  
  
  

/***************** WriteBack Unit (WB)  ****************/  
assign wRegData = (MEMWB_MemToReg) ? MEMWB_DMemOut : MEMWB_ALUOut;

endmodule
