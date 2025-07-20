`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Ahmed Said Gulsen and Yavuz Selim Kara
// Project Name: BLG222E Project 1
//////////////////////////////////////////////////////////////////////////////////
module ArithmeticLogicUnitSystem (
    input [2:0] RF_OutASel, RF_OutBSel, RF_FunSel,
    input [3:0] RF_RegSel, RF_ScrSel,
    input [4:0] ALU_FunSel,
    input ALU_WF,
    input [1:0] ARF_OutCSel, ARF_OutDSel, ARF_FunSel,
    input [2:0] ARF_RegSel,
    input IR_LH, IR_Write, Mem_WR, Mem_CS,
    input DR_E, 
    input [1:0] DR_FunSel,
    input [1:0] MuxASel, MuxBSel, MuxCSel,
    input MuxDSel,
    input Clock,
    
    output reg IROut_
);
    wire [31:0] MuxAOut;
    wire [31:0] MuxBOut;
    wire [31:0] MuxDOut;
    wire [7:0] MuxCOut;
    
    wire [31:0] OutA;
    wire [31:0] OutB;
    wire [31:0] ALUOut;
    wire [3:0] FlagsOut;
    
    wire [15:0] OutC;
    wire [15:0] Address;
    wire [7:0] MemOut;
    
    wire [15:0] IROut;
    wire [31:0] DROut;
    
    assign MuxAOut = (MuxASel == 0) ? ALUOut :
                     (MuxASel == 1) ? {16'b0, OutC} :
                     (MuxASel == 2) ? DROut :
                                      {24'b0, IROut[7:0]};

    assign MuxBOut = (MuxBSel == 0) ? ALUOut :
                     (MuxBSel == 1) ? {16'b0, OutC} :
                     (MuxBSel == 2) ? DROut :
                                      {24'b0, IROut[7:0]};

    assign MuxCOut = (MuxCSel == 0) ? ALUOut[7:0] :
                     (MuxCSel == 1) ? ALUOut[15:8] :
                     (MuxCSel == 2) ? ALUOut[23:16] :
                                      ALUOut[31:24] ;
    
    assign MuxDOut = (MuxDSel == 0) ? OutA :
                                      {16'b0, OutC} ;

    always @(*) begin
        IROut_ = IROut[15:8];
    end
    
    RegisterFile RF(
        .I(MuxAOut), 
        .OutASel(RF_OutASel),
        .OutBSel(RF_OutBSel), 
        .FunSel(RF_FunSel),
        .RegSel(RF_RegSel), 
        .ScrSel(RF_ScrSel),
        .Clock(Clock),
        .OutA(OutA), 
        .OutB(OutB)
    );

    ArithmeticLogicUnit ALU( 
        .A(OutA), 
        .B(OutB), 
        .FunSel(ALU_FunSel), 
        .WF(ALU_WF), 
        .Clock(Clock),
        .ALUOut(ALUOut), 
        .FlagsOut(FlagsOut)
    );

    Memory MEM (
        .Address(Address), 
        .Data(MuxCOut),
        .WR(Mem_WR),
        .CS(Mem_CS),
        .Clock(Clock),
        .MemOut(MemOut)
    );

    InstructionRegister IR(
        .I(MemOut),
        .Write(IR_Write),
        .LH(IR_LH),
        .Clock(Clock),
        .IROut(IROut)
    );
    DataRegister DR(
        .I(MemOut), 
        .E(DR_E), 
        .FunSel(DR_FunSel), 
        .Clock(Clock), 
        .DROut(DROut)
    );
    
    AddressRegisterFile ARF(
        .I(MuxBOut),
        .OutCSel(ARF_OutCSel),
        .OutDSel(ARF_OutDSel),
        .FunSel(ARF_FunSel),
        .RegSel(ARF_RegSel),
        .Clock(Clock),
        .OutC(OutC),
        .OutD(Address)
    );
    
endmodule