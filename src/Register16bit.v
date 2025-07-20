`timescale 1ns / 1ps
module Register16bit (
    input Clock,
    input E,
    input [1:0] FunSel,
    input [15:0] I,
    output reg [15:0] Q
);

always @(posedge Clock) begin
    if (E) begin
        case (FunSel)
            2'b00: Q <= Q - 1;     // Decrement
            2'b01: Q <= Q + 1;     // Increment
            2'b10: Q <= I;         // Load
            2'b11: Q <= 16'b0;     // Clear
        endcase
        // If FunSel is out of range (not likely with 2 bits), Q is unchanged
    end
    // else: retain Q (do nothing)
end

endmodule