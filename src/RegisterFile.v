`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Yavuz Selim Kara and Ahmed Said Gülşen
// Project Name: BLG222E Project 2
//////////////////////////////////////////////////////////////////////////////////
module RegisterFile (
    input Clock,
    input [31:0] I,
    input [3:0] RegSel,
    input [3:0] ScrSel,
    input [2:0] FunSel,
    input [2:0] OutASel,
    input [2:0] OutBSel,
    
    output reg [31:0] OutA,
    output reg [31:0] OutB
);
    // Register outputs
    wire [31:0] QR1, QR2, QR3, QR4;
    wire [31:0] QS1, QS2, QS3, QS4;
    reg ER1, ER2, ER3, ER4, ES1, ES2, ES3, ES4; // Enable signals for registers

    Register32bit R1(.I(I), .E(ER1), .FunSel(FunSel), .Clock(Clock), .Q(QR1));
    Register32bit R2(.I(I), .E(ER2), .FunSel(FunSel), .Clock(Clock), .Q(QR2));
    Register32bit R3(.I(I), .E(ER3), .FunSel(FunSel), .Clock(Clock), .Q(QR3));
    Register32bit R4(.I(I), .E(ER4), .FunSel(FunSel), .Clock(Clock), .Q(QR4));
    
    Register32bit S1(.I(I), .E(ES1), .FunSel(FunSel), .Clock(Clock), .Q(QS1));
    Register32bit S2(.I(I), .E(ES2), .FunSel(FunSel), .Clock(Clock), .Q(QS2));
    Register32bit S3(.I(I), .E(ES3), .FunSel(FunSel), .Clock(Clock), .Q(QS3));
    Register32bit S4(.I(I), .E(ES4), .FunSel(FunSel), .Clock(Clock), .Q(QS4));

    always @(*) begin
        case(RegSel)
            4'b0000: begin
                ER1 = 1'b0;
                ER2 = 1'b0;
                ER3 = 1'b0;
                ER4 = 1'b0;
            end
            4'b0001: begin
		        ER1 = 1'b0;
                ER2 = 1'b0;
                ER3 = 1'b0;
                ER4 = 1'b1;
            end
	       4'b0010: begin
                ER1 = 1'b0;
                ER2 = 1'b0;
                ER3 = 1'b1;
                ER4 = 1'b0;
            end
            4'b0011: begin
                ER1 = 1'b0;
                ER2 = 1'b0;
                ER3 = 1'b1;
                ER4 = 1'b1;
            end
	       4'b0100: begin
                ER1 = 1'b0;
                ER2 = 1'b1;
                ER3 = 1'b0;
                ER4 = 1'b0;
            end
            4'b0101: begin
                ER1 = 1'b0;
                ER2 = 1'b1;
                ER3 = 1'b0;
                ER4 = 1'b1;
            end
	       4'b0110: begin
                ER1 = 1'b0;
                ER2 = 1'b1;
                ER3 = 1'b1;
                ER4 = 1'b0;
            end
            4'b0111: begin
                ER1 = 1'b0;
                ER2 = 1'b1;
                ER3 = 1'b1;
                ER4 = 1'b1;
            end
            4'b1000: begin
                ER1 = 1'b1;
                ER2 = 1'b0;
                ER3 = 1'b0;
                ER4 = 1'b0;
            end
            4'b1001: begin
                ER1 = 1'b1;
                ER2 = 1'b0;
                ER3 = 1'b0;
                ER4 = 1'b1;
            end
	       4'b1010: begin
                ER1 = 1'b1;
                ER2 = 1'b0;
                ER3 = 1'b1;
                ER4 = 1'b0;
            end
            4'b1011: begin
                ER1 = 1'b1;
                ER2 = 1'b0;
                ER3 = 1'b1;
                ER4 = 1'b1;
            end
	       4'b1100: begin
                ER1 = 1'b1;
                ER2 = 1'b1;
                ER3 = 1'b0;
                ER4 = 1'b0;
            end
            4'b1101: begin
                ER1 = 1'b1;
                ER2 = 1'b1;
                ER3 = 1'b0;
                ER4 = 1'b1;
            end
	       4'b1110: begin
                ER1 = 1'b1;
                ER2 = 1'b1;
                ER3 = 1'b1;
                ER4 = 1'b0;
            end
            4'b1111: begin
                ER1 = 1'b1;
                ER2 = 1'b1;
                ER3 = 1'b1;
                ER4 = 1'b1;
            end

            default: begin
                // Default case where no register is enabled
                ER1 = 1'b0;
                ER2 = 1'b0;
                ER3 = 1'b0;
                ER4 = 1'b0;
            end
        endcase

    case(ScrSel)
            4'b0000: begin
                ES1 = 1'b0;
                ES2 = 1'b0;
                ES3 = 1'b0;
                ES4 = 1'b0;
            end
            4'b0001: begin
                ES1 = 1'b0;
                ES2 = 1'b0;
                ES3 = 1'b0;
                ES4 = 1'b1;
            end
            4'b0010: begin
                ES1 = 1'b0;
                ES2 = 1'b0;
                ES3 = 1'b1;
                ES4 = 1'b0;
            end
            4'b0011: begin
                ES1 = 1'b0;
                ES2 = 1'b0;
                ES3 = 1'b1;
                ES4 = 1'b1;
            end
            4'b0100: begin
                ES1 = 1'b0;
                ES2 = 1'b1;
                ES3 = 1'b0;
                ES4 = 1'b0;
            end
            4'b0101: begin
                ES1 = 1'b0;
                ES2 = 1'b1;
                ES3 = 1'b0;
                ES4 = 1'b1;
            end
            4'b0110: begin
                ES1 = 1'b0;
                ES2 = 1'b1;
                ES3 = 1'b1;
                ES4 = 1'b0;
            end
            4'b0111: begin
                ES1 = 1'b0;
                ES2 = 1'b1;
                ES3 = 1'b1;
                ES4 = 1'b1;
            end
            4'b1000: begin
                ES1 = 1'b1;
                ES2 = 1'b0;
                ES3 = 1'b0;
                ES4 = 1'b0;
            end
            4'b1001: begin
                ES1 = 1'b1;
                ES2 = 1'b0;
                ES3 = 1'b0;
                ES4 = 1'b1;
            end
            4'b1010: begin
                ES1 = 1'b1;
                ES2 = 1'b0;
                ES3 = 1'b1;
                ES4 = 1'b0;
            end
            4'b1011: begin
                ES1 = 1'b1;
                ES2 = 1'b0;
                ES3 = 1'b1;
                ES4 = 1'b1;
            end
            4'b1100: begin
                ES1 = 1'b1;
                ES2 = 1'b1;
                ES3 = 1'b0;
                ES4 = 1'b0;
            end
            4'b1101: begin
                ES1 = 1'b1;
                ES2 = 1'b1;
                ES3 = 1'b0;
                ES4 = 1'b1;
            end
            4'b1110: begin
                ES1 = 1'b1;
                ES2 = 1'b1;
                ES3 = 1'b1;
                ES4 = 1'b0;
            end
            4'b1111: begin
                ES1 = 1'b1;
                ES2 = 1'b1;
                ES3 = 1'b1;
                ES4 = 1'b1;
            end
        
            default: begin
                // Default case where no register is enabled
                ES1 = 1'b0;
                ES2 = 1'b0;
                ES3 = 1'b0;
                ES4 = 1'b0;
            end
        endcase
    end

    // Multiplexers
    always @(*) begin
        case (OutASel)
            3'b000: OutA = QR1;
            3'b001: OutA = QR2;
            3'b010: OutA = QR3;
            3'b011: OutA = QR4;
            3'b100: OutA = QS1;
            3'b101: OutA = QS2;
            3'b110: OutA = QS3;
            3'b111: OutA = QS4;
            default: OutA = 32'b0;
        endcase
            
        case (OutBSel)
            3'b000: OutB = QR1;
            3'b001: OutB = QR2;
            3'b010: OutB = QR3;
            3'b011: OutB = QR4;
            3'b100: OutB = QS1;
            3'b101: OutB = QS2;
            3'b110: OutB = QS3;
            3'b111: OutB = QS4;
            default: OutB = 32'b0;
        endcase
    end

endmodule
