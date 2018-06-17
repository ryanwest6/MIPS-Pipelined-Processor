// mips_forwarding_pipeline.v
//
// @author: Ryan West
//
// This is an implementation of a simple soft processor using the MIPS instruction
// set architecture. This version implements a 5-stage pipeline with forwarding 
// and hazard detection/correction.

`timescale 1ns / 1ps

module mips_forwarding_pipeline(
input clk,
output[31:0] PC,
output[31:0] Instruction,
output[31:0] ALUResult,
output[31:0] dWriteData,
output[31:0] WriteBackData
    );
    
    /////CONTROL SIGNALS/////

    //Stages: IF (Instruction Fetch), ID (Instruction Decode), EX (Execute),
    // MEM (Memory read-write), WB (Write Back)

    //IF
    reg[31:0] PC_reg = 0;
    

    //ID
    reg RegDst = 0, RegWrite = 0, RegWriteMEM = 0, RegWriteWB = 0, RegWriteWB2 = 0;
    reg[31:0] instruction = 0, instrID = 0, instrEX = 0, PC4ID = 0, PC4EX = 0; 
    
    reg[31:0] extend_result = 0;
    wire[4:0] read1, read2;
    reg[4:0] WriteReg = 0, WriteRegWB = 0, WriteRegWB2 = 0;
    
    reg ALUSrc = 0, ALUSrc2 = 0, ALUSrc3 = 0, ZeroExt = 0;
    
    wire[31:0] WriteData; //data to write
    //wire RegWrite (already an input)
    reg[31:0] Data1 = 0, Data2 = 0; //the register values read
    reg[31:0] RF [31:0]; //32 registers, each 32 bits long
    
        
    //EX
    reg[31:0] ALUOut = 0, ALUOutWB = 0, ALUOutWB2 = 0;
    reg Zero = 0, ZeroMEM = 0, ZeroWB = 0;
    reg[3:0] ALUCtrl = 0; //must be connected.
    
    reg Branch = 0, BranchMEM = 0, BranchMEM2 = 0, BranchMEM3WB = 0;
    reg MemWrite = 0, MemWriteEX = 0, MemWriteMEM = 0;//, MemWriteMEM2 = 0;
    reg MemRead = 0, MemReadMEM = 0;//,MemReadMEM2 = 0;
    reg MemtoReg = 0, MemtoRegMEM = 0, MemtoRegWB = 0, MemtoRegWB2 = 0;
    
    reg[31:0] WriteDMemData = 0, WriteDMemData2 = 0;
    

    //MEM
    reg PCSrc = 0, PCSrcMEM = 0;
    
    reg[31:0] MemOut = 0;
    // memory declaration
    reg [31:0] dmem[1023:0];
    reg [31:0] imem[1023:0];
    
    assign PC = PC_reg;
    assign Instruction = instruction;
    assign ALUResult = ALUOut;
    assign dWriteData = WriteDMemData2;
    assign WriteBackData = WriteData;
    
    // FORWARDING REGS
    wire[4:0] rd_ifid, rs_ifid, rt_ifid;
    reg[4:0] rd_idex = 0, rd_exmem = 0, rd_memwb = 0;
    reg[4:0] rs_idex = 0, rs_exmem = 0, rs_memwb = 0;
    reg[4:0] rt_idex = 0, rt_exmem = 0, rt_memwb = 0;
    reg[4:0] rd_end = 0;

    reg insert_ex_nop = 0, insert_ex_nop_next = 0;
    reg halt_pc = 0, halt_pc_next = 0;
    
    reg[4:0] destReg2 = 0;
    

    //////////INSTRUCTION FETCH (IF)//////////
    
    //Program counter
    always @(posedge clk) begin
    
        //halt_pc implementation:
        if (halt_pc)
            PC_reg <= PC_reg;
        else
            PC_reg <= PCSrcMEM ? (PC4EX + (extend_result << 2)) - 8: PC_reg + 4;
        
        if (halt_pc)
            instruction <= instruction;
        else
            instruction <= imem[PC_reg[11:2]];
    end
    
      initial begin
         $readmemh("forwarding_data_memory.txt", dmem);
         $readmemh("forwarding_instruction_memory.txt", imem);
      end
      
        // Synchronous read or write
      always@(posedge clk)
        // decode address to start at location 0x1000
        if (ALUOut >= 32'h00001000 && ALUOut <= 32'h00001fff) begin
          if (MemReadMEM)
            MemOut <= dmem[(ALUOut-32'h1000)>>2];
          if (MemWriteMEM)
            dmem[(ALUOut-32'h1000)>>2] <= WriteDMemData2;
        end
      
    
    //////////INSTRUCTION DECODE (ID)//////////
    
    wire[31:0] extend_resultWire; 
    
    //sign/zero extend logic
    
    always @(posedge clk)
        extend_result <= extend_resultWire;
    assign extend_resultWire = (ZeroExt ? {16'b0, instruction[15:0]} : 
                                {{16{instruction[15]}}, instruction[15:0]});
    
    /////////////begin register file///////////
        
    wire isImmediate;
    reg isImmediate_idex = 0, isImmediate_exmem = 0;//, isImmediate_memwb = 0;
   
    assign read1 = instruction[25:21];
    assign read2 = instruction[20:16];

    assign rs_ifid = instruction[25:21];
    assign rt_ifid = instruction[20:16];
    assign rd_ifid = isImmediate ? instruction[20:16] : instruction[15:11]; 

    always @(posedge clk) begin
        rd_idex <= rd_ifid;
        rs_idex <= rs_ifid;
        rt_idex <= rt_ifid;    
        
        rd_exmem <= rd_idex;
        rs_exmem <= rs_idex;
        rt_exmem <= rt_idex;

        rd_memwb <= rd_exmem;
        rs_memwb <= rs_exmem;
        rt_memwb <= rt_exmem;
        
        rd_end <= rd_memwb;
        
        isImmediate_idex <= isImmediate;
        isImmediate_exmem <= isImmediate_idex;
        
        halt_pc_next <= halt_pc;
        insert_ex_nop_next <= insert_ex_nop;
    end

     //initializes RF (so it doesn't start with X)
     integer i;
            initial
              for (i = 0; i < 32; i=i+1)
                RF[i] = 0;
        
    always @(posedge clk) begin
        if (read1 == WriteRegWB2 && RegWriteWB2)
            Data1 <= WriteData;
        else
            Data1 <= RF[read1];
        if (read2 == WriteRegWB2 && RegWriteWB2)
            Data2 <= WriteData;
        else    
            Data2 <= RF[read2];
        
        RF[0] <= 0; //hardwire reg 0 to 0
        //RF[6] <= 2;
        //write the register with the new value if Regwrite is high
        if (RegWriteWB2) begin
            RF[WriteRegWB2] <= WriteData;
            end
        //assign WriteData = MemtoReg ? dReadData : ALUOut;
    end
           
           
// CONTROL SECTION

//opcodes
localparam Add = 6'b000000, Sub = 6'b000000, And = 6'b000000, Or = 6'b000000, 
           Nor = 6'b000000, Slt = 6'b000000, Lw = 6'b100011, Sw = 6'b101011, 
           Beq = 6'b000100, J = 6'b000010, Andi = 6'b001100, Ori = 6'b001101, 
           Rtype = 6'b000000, Addi = 6'b001000, Nop = 6'b111111;

//alu control lines
localparam aluAnd = 4'b0000, aluOr = 4'b0001, aluAdd = 4'b0010, 
        aluSub = 4'b0110, aluSlt = 4'b0111, aluNor = 4'b1100;

//funct fields
localparam functAdd = 6'b100000, functSub = 6'b100010, functAnd = 6'b100100,
           functOr = 6'b100101, functSlt = 6'b101010, functNor = 6'b100111;

wire[5:0] opcode = instruction[31:26];
wire[5:0] opcodeEX = instrID[31:26];
wire[5:0] opcodeMEM = instrEX[31:26];
wire[5:0] funct = instrID[5:0]; //SHOULD BE functEX
wire[3:0] arithmetic;
wire[3:0] ALUCtrlWire;
reg[3:0] ALUCtrl2 = 0;


assign isImmediate = (opcode == Addi) || opcode == Andi || opcode == Ori 
        || opcode == Lw || opcode == Sw; 

assign arithmetic = funct == functNor ? aluNor :
                     funct == functSub ? aluSub :
                     funct == functAnd ? aluAnd :
                     funct == functOr ? aluOr : 
                     funct == functSlt ? aluSlt : 
                     aluAdd;
                     
assign ALUCtrlWire = opcodeEX == Lw ? aluAdd :        
                      opcodeEX == Sw ? aluAdd : 
                      opcodeEX == Beq ? aluSub : 
                      opcodeEX == Addi ? aluAdd : 
                      opcodeEX == Andi ? aluAnd : 
                      opcodeEX == Ori ? aluOr : 
                      arithmetic;

always @(*) begin
    //ALU Control
    ALUCtrl <= ALUCtrlWire;

    //Datapath control
    ALUSrc <= opcode == Lw || opcode == Ori || opcode == Andi || opcode == Addi
         || opcode == Sw;
    
    RegWrite <= (opcode == Rtype || opcode == Andi || opcode == Ori
         || opcode == Addi || opcode == Lw);

    RegDst <= opcodeEX == Rtype;// || opcode == Ori || opcode == Andi;
    
    MemtoReg <= (opcode == Lw); //only loads from data memory with LW
    
    ZeroExt <= (opcode == Andi || opcode == Ori);
    MemRead <= (opcode == Lw);
    MemWrite <= opcode == Sw;
    
    Branch <= opcode == Beq;
end


    wire[4:0] destReg;
    assign destReg = ALUSrc3 ? rt_exmem : rd_exmem; //was exmem


// Contains all registers that pipeline to different stages
always @(posedge clk) begin
    
    //Including stalling support if insert_ex_nop is asserted.

    RegWriteMEM <= RegWrite;
    RegWriteWB <= insert_ex_nop ? 0 : RegWriteMEM;
    RegWriteWB2 <= RegWriteWB;

    MemtoRegMEM <= insert_ex_nop ? 0 : MemtoReg;
    MemtoRegWB <= MemtoRegMEM;
    MemtoRegWB2 <= MemtoRegWB;
    
    MemWriteEX <= MemWrite;
    MemWriteMEM <= insert_ex_nop ? 0 : MemWriteEX;
    MemReadMEM <= MemRead;
    
    ALUOutWB <= ALUOut;
    ALUOutWB2 <= ALUOutWB;
    
    WriteRegWB <= insert_ex_nop ? 0 : WriteReg;
    WriteRegWB2 <= WriteRegWB;
    
    PC4ID <= PC_reg + 4;
    PC4EX <= PC4ID;
    
    //THIS ONE IS PROBABLY WRONG , one cycle too early
    ALUSrc2 <= insert_ex_nop ? 0 : ALUSrc;
    ALUSrc3 <= ALUSrc2;
    
    instrID <= instruction;
    instrEX <= instrID;

    WriteDMemData2 <= insert_ex_nop ? 0 : WriteDMemData;
    
    BranchMEM <= insert_ex_nop ? 0 : Branch;
    BranchMEM2 <=  BranchMEM;
    BranchMEM3WB <= BranchMEM2; //maybe delete insert_ex_nop here
    
    Zero <=  (ALUOut == 0);
    ZeroMEM <= Zero;
    ZeroWB <= ZeroMEM;
    
    destReg2 <= destReg;
    
    ALUCtrl2 <= ALUCtrlWire;
end
           

//////////EXECUTE (EX)//////////

//ALU
    //wire aluCtrl already exists
    reg[31:0] A = 0, B = 0; //these are treated like wires
    wire[31:0] A_wire, B_wire;
    wire[4:0] WriteRegWire;
    wire[31:0] ALUOutWire;
    
    assign ALUOutWire = (ALUCtrl == 0) ? A & B:
                        (ALUCtrl == 1) ? A | B:
                        (ALUCtrl == 2) ? A + B:
                        (ALUCtrl == 6) ? A - B:
                        (ALUCtrl == 7) ? ($signed(A) < $signed(B) ? 1 : 0) :
                        (ALUCtrl == 12) ? ~(A | B): //nor
                        0; //DEFAULT VALUE
    
    
    assign WriteRegWire = RegDst ? instrID[15:11] : instrID[20:16];    
    //assign A = Data1[rs];
    assign A_wire = Data1; 
    assign B_wire = ALUSrc2 ? extend_result : Data2;
    
    always @(*) begin
        ALUOut <= ALUOutWire;
        WriteReg <= WriteRegWire;
        
        //Forwarding logic for dWriteData reg (value of data to be written to the DMEM)
        WriteDMemData <= (rt_idex == rd_exmem && isImmediate_idex) ? ALUOutWB : B;
        
//Forwarding Unit for inputs A and B: (in ID stage)
        
        
        if (RegWriteWB && rd_memwb != 0 && !(RegWriteMEM && rd_exmem != 0 
            && rd_exmem == rs_idex) && rd_memwb == rs_idex)
            A <= WriteData;                //MAYBE WRONG        
        else if (RegWriteMEM && rd_exmem != 0 && rd_exmem == rs_idex)
            A <= ALUOutWB;
        else
            A <= A_wire;
        
        if (!isImmediate_idex && RegWriteWB && rd_memwb != 0 
                && !(RegWriteMEM && rd_exmem != 0 && rd_exmem == rt_idex) && 
                rd_memwb == rt_idex && ALUCtrl2 != 4'b0111)
                // equiv output: 01
                B <= WriteData;        
            else if (RegWriteMEM && rd_exmem != 0 && rd_exmem == rt_idex && !isImmediate_idex)
                B <= ALUOutWB;
            else
                B <= B_wire;        
        
        
        
 //Hazard detection unit
        
        //QUITE POSSIBLY just MemRead.   ALSO, I added opcodeEX == Lw, not in book.
        if ((MemReadMEM && (rt_idex == rs_ifid || rt_idex == rt_ifid))  || //Load-use
           (BranchMEM || BranchMEM2 || (BranchMEM3WB && ZeroMEM)) )  //Branches
           insert_ex_nop <= 1;
        else
           insert_ex_nop <= 0;

        if ((opcodeEX == Lw && MemReadMEM && (rt_idex == rs_ifid || rt_idex == rt_ifid)) || 
           (BranchMEM || (Branch && !BranchMEM && !BranchMEM2)) )  //Branches
            halt_pc <= 1;
        else
            halt_pc <= 0;

//////////MEMORY ACCESS (MEM)//////////

//Generate PCSRc (should we branch) if there is a beq instruction and zero from ALU was true
always @(*) //used to be @ posedge clk
    PCSrcMEM <= BranchMEM2 && Zero;         /// MIGHT NEED TO BE ZeroMEM or something
    
//////////WRITE-BACK (WB)//////////

//always @(posedge clk)
assign    WriteData = (MemtoRegWB2 ? MemOut : ALUOutWB2); //If 0, don't write anything.

endmodule
