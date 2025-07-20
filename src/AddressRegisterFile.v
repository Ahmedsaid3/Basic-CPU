`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Student 1 : Ahmed Said Gulsen
// Student 2 : Yavuz Selim Kara
// Project Name: BLG222E Project 1
//////////////////////////////////////////////////////////////////////////////////
module AddressRegisterFile (
    input Clock,
    input [31:0] I,
    input [1:0] FunSel,
    input [2:0] RegSel,
    input [1:0] OutCSel,
    input [1:0] OutDSel,
    output reg [15:0] OutC,
    output reg [15:0] OutD
);

    wire [15:0] QPC, QSP, QAR;
    reg E_PC, E_SP, E_AR;
    
    // Registers
    Register16bit PC(.I(I[15:0]), .E(E_PC), .FunSel(FunSel), .Clock(Clock), .Q(QPC));
    Register16bit SP(.I(I[15:0]), .E(E_SP), .FunSel(FunSel), .Clock(Clock), .Q(QSP));
    Register16bit AR(.I(I[15:0]), .E(E_AR), .FunSel(FunSel), .Clock(Clock), .Q(QAR));
    
    always @(*) begin
        case(RegSel)
            4'b000: begin
                E_PC = 1'b0;
                E_SP = 1'b0;
                E_AR = 1'b0;
            end
            4'b001: begin
                E_PC = 1'b0;
                E_SP = 1'b0;
                E_AR = 1'b1;
            end
           4'b010: begin
                E_PC = 1'b0;
                E_SP = 1'b1;
                E_AR = 1'b0;
            end
            4'b011: begin
                E_PC = 1'b0;
                E_SP = 1'b1;
                E_AR = 1'b1;
            end
           4'b100: begin
                E_PC = 1'b1;
                E_SP = 1'b0;
                E_AR = 1'b0;
            end
            4'b101: begin
                E_PC = 1'b1;
                E_SP = 1'b0;
                E_AR = 1'b1;
            end
           4'b110: begin
                E_PC = 1'b1;
                E_SP = 1'b1;
                E_AR = 1'b0;
            end
            4'b111: begin
                E_PC = 1'b1;
                E_SP = 1'b1;
                E_AR = 1'b1;
            end
        endcase         
    end
    
    // Output selectors
    always @(*) begin
        case (OutCSel)
            2'b00: OutC = QPC;
            2'b01: OutC = QSP;
            2'b10: OutC = QAR;
            2'b11: OutC = QAR;
            default: OutC = 16'b0;
        endcase
    
        case (OutDSel)
            2'b00: OutD = QPC;
            2'b01: OutD = QSP;
            2'b10: OutD = QAR;
            2'b11: OutD = QAR;
            default: OutD = 16'b0;
        endcase
    end

endmodule