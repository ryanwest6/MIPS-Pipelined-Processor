// mips_basic_pipeline.v
//
// @author: Ryan West
//
// This is an implementation of a simple soft processor using the MIPS instruction
// set architecture. This version does not use forwarding.

`timescale 1ns / 1ps

module mips_basic_pipeline(
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
    reg[31:0] instruction = 0, instrID = 0, PC4ID = 0, PC4EX = 0;
        
    reg[31:0] extend_result = 0;
    wire[4:0] read1, read2;
    reg[4:0] WriteReg = 0, WriteRegWB = 0, WriteRegWB2 = 0;
    
    reg ALUSrc = 0, ALUSrc2 = 0, ZeroExt = 0;
    
    wire[31:0] WriteData; //data to write
    //wire RegWrite (already an input)
    reg[31:0] Data1 = 0, Data2 = 0; //the register values read
    reg[31:0] RF [31:0]; //32 registers, each 32 bits long
    
        
    //EX
    reg[31:0] ALUOut = 0, ALUOutWB = 0, ALUOutWB2 = 0;
    reg Zero = 0;
    reg[3:0] ALUCtrl = 0;
    
    reg Branch = 0, BranchMEM = 0, BranchMEM2 = 0;
    reg MemWrite = 0, MemWriteEX = 0, MemWriteMEM = 0;//, MemWriteMEM2 = 0;
    reg MemRead = 0, MemReadMEM = 0;//,MemReadMEM2 = 0;
    reg MemtoReg = 0, MemtoRegMEM = 0, MemtoRegWB = 0, MemtoRegWB2 = 0;
    
    reg[31:0] WriteDMemData = 0, WriteDMemData2 = 0;
    
    //MEM
    reg PCSrc = 0, PCSrcMEM = 0;
    
    //register that holds the current ADDRESS of the PC
    reg[31:0] curPC = 0; 
    
    reg[31:0] MemOut = 0;
    // memory declaration
    reg [31:0] dmem[1023:0];
    reg [31:0] imem[1023:0];  
    
    
    //Assign statments for outputs
    assign PC = PC_reg;
    assign Instruction = instruction;
    assign ALUResult = ALUOut;
    assign dWriteData = WriteDMemData2;
    assign WriteBackData = WriteData;


    //////////INSTRUCTION FETCH (IF)//////////
    
    
    //Program counter
    always @(posedge clk) begin
        PC_reg <= PCSrcMEM ? (PC4EX + (extend_result << 2)) - 8: PC_reg + 4;
        instruction <= imem[PC_reg[11:2]];
    end    


    //Instruction/Data memory
    
      // memory initialization from a text file. Must have the file in your project.
      initial begin
         $readmemh("pipe_data_memory.txt", dmem);
         $readmemh("pipe_instruction_memory.txt", imem);
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
    
    always @(posedge clk)
        extend_result <= extend_resultWire;
    
    assign extend_resultWire = (ZeroExt ? {16'b0, instruction[15:0]} : 
                                {{16{instruction[15]}}, instruction[15:0]});
    
    ////////////begin register file///////////
        
    assign read1 = instruction[25:21];
    assign read2 = instruction[20:16];
        
     //initializes RF (so it doesn't start with X in simulation)
     integer i;   // i needs to be declared before it is used
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
            Rtype = 6'b000000, Addi = 6'b001000,
           Nop = 6'b111111;

//alu control lines
localparam aluAnd = 4'b0000, aluOr = 4'b0001, aluAdd = 4'b0010, 
        aluSub = 4'b0110, aluSlt = 4'b0111, aluNor = 4'b1100;

//funct fields
localparam functAdd = 6'b100000, functSub = 6'b100010, functAnd = 6'b100100,
           functOr = 6'b100101, functSlt = 6'b101010, functNor = 6'b100111;

wire[5:0] opcode = instruction[31:26];
wire[5:0] opcodeEX = instrID[31:26];
wire[5:0] funct = instrID[5:0]; //SHOULD BE functEX
wire[3:0] arithmetic;
wire[3:0] ALUCtrlWire;

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
    
    MemtoReg <= (opcode == Lw); //only loads from data memory with LW, otherwise from ALU
    
    ZeroExt <= (opcode == Andi || opcode == Ori);
    MemRead <= (opcode == Lw);
    MemWrite <= opcode == Sw;
    
    Branch <= opcode == Beq;
end

always @(posedge clk) begin
    RegWriteMEM <= RegWrite;
    RegWriteWB <= RegWriteMEM;
    RegWriteWB2 <= RegWriteWB;

    MemtoRegMEM <= MemtoReg;
    MemtoRegWB <= MemtoRegMEM; //passing memToReg through the pipeline    
    MemtoRegWB2 <= MemtoRegWB;
    
    MemWriteEX <= MemWrite;
    MemWriteMEM <= MemWriteEX;
    MemReadMEM <= MemRead;

    ALUOutWB <= ALUOut;
    ALUOutWB2 <= ALUOutWB;
    
    WriteRegWB <= WriteReg;
    WriteRegWB2 <= WriteRegWB;
    
    PC4ID <= PC_reg + 4;
    PC4EX <= PC4ID;
    
    ALUSrc2 <= ALUSrc;
    
    instrID <= instruction;

    WriteDMemData2 <= WriteDMemData;
    
    BranchMEM <= Branch;
    BranchMEM2 <= BranchMEM;
    
    Zero <= (ALUOut == 0);

end
           

//////////EXECUTE (EX)//////////

//ALU
    //wire aluCtrl already exists
    wire[31:0] A,B;
    wire[4:0] WriteRegWire;
    wire[31:0] ALUOutWire;
    
    assign ALUOutWire = (ALUCtrl == 0) ? A & B:
                (ALUCtrl == 1) ? A | B:
                (ALUCtrl == 2) ? A + B:
                (ALUCtrl == 6) ? A - B:
                (ALUCtrl == 7) ? ($signed(A) < $signed(B) ? 1 : 0) : //slt
                (ALUCtrl == 12) ? ~(A | B): //nor
                0; //DEFAULT VALUE
    
    
    assign WriteRegWire = RegDst ? instrID[15:11] : instrID[20:16];    
    //assign A = Data1[rs];
    assign A = Data1; 
    assign B = ALUSrc2 ? extend_result : Data2;
    
    always @(*) begin 
        ALUOut <= ALUOutWire;
        WriteReg <= WriteRegWire;
        WriteDMemData <= Data2;
    end
    

//////////MEMORY ACCESS (MEM)//////////

//Generate PCSRc (should we branch) if beq instruction and zero from ALU was true
always @(*) //used to be @ posedge clk
    PCSrcMEM <= BranchMEM2 && Zero;
    
        //Program counter
    //PCSrc <= ((opcode == Beq) && Zero);


//////////WRITE-BACK (WB)//////////
//always @(posedge clk)
assign    WriteData = (MemtoRegWB2 ? MemOut : ALUOutWB2); //If 0, write nothing
    
endmodule
