`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Student 1 : Ahmed Said Gulsen
// Student 2 : Yavuz Selim Kara
// Project Name: BLG222E Project 1
//////////////////////////////////////////////////////////////////////////////////

module DataRegister (
    input Clock,
    input E,
    input [1:0] FunSel,
    input [7:0] I,
    output reg [31:0] DROut
);

always @(posedge Clock) begin
    if (E)begin
        case (FunSel)
            2'b00: DROut <= {{24{I[7]}}, I};  // Sign extend
            2'b01: DROut <= {24'b0, I};       // Zero extend
            2'b10: DROut <= {DROut[23:0], I}; // Left shift
            2'b11: DROut <= {I, DROut[31:8]}; // Right shift
            default: DROut <= DROut;
        endcase
    end
end

endmodule