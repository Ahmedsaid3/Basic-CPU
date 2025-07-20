`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Yavuz Selim Kara and Ahmed Said Gülşen
// Project Name: BLG222E Project 2
//////////////////////////////////////////////////////////////////////////////////
module ArithmeticLogicUnit (
    input Clock,
    input [31:0] A, B,
    input [4:0] FunSel,
    input WF,
    output reg [3:0] FlagsOut,
    output reg [31:0] ALUOut
);

    reg [32:0] temp_result; // An extra bit for overflow detection
    reg [31:0] A_ext, B_ext;
    reg [31:0] A_sign_ext, B_sign_ext;
    
    always @(*) begin
        // Task dont want us to sign extension for 16-bit inputs - so clear first 16bit
        A_ext = {{16'b0}, A[15:0]}; // clear most significant 16 bits
        B_ext = {{16'b0}, B[15:0]}; // clear most significant 16 bits
        A_sign_ext = {{16{A[15]}}, A[15:0]}; // sign extension
        B_sign_ext = {{16{B[15]}}, B[15:0]}; /// sign extension
        
        case(FunSel)
            // 16-bit inputs
            5'b00000: begin
                ALUOut = A_ext;
                temp_result[31:0] = A_ext; // clear most significant 16 bits
                
            end
            5'b00001: begin
                ALUOut = B_ext;
                temp_result[31:0] = B_ext; // clear most significant 16 bits

            end
            5'b00010: begin
                temp_result[15:0] = ~A[15:0];
                temp_result = {17'b0, temp_result[15:0]}; // clear most significant 16 bits
                ALUOut = temp_result[31:0];

            end
            5'b00011: begin
                temp_result[15:0] = ~B[15:0];
                temp_result = {17'b0, temp_result[15:0]}; // clear most significant 16 bits
                ALUOut = temp_result[31:0];

            end
            5'b00100: begin
                temp_result = A_sign_ext + B_sign_ext; // arithmetic -> sign extension
                ALUOut = temp_result[31:0];

                
            end
            5'b00101: begin
                temp_result = A_sign_ext + B_sign_ext + FlagsOut[2]; // arithmetic -> sign extension
                ALUOut = temp_result[31:0];

                
            end
            5'b00110: begin
                temp_result = A_sign_ext + ~B_sign_ext + 1'b1; // arithmetic -> sign extension
                ALUOut = temp_result[31:0];

                
            end
            5'b00111: begin
                temp_result[15:0] = A[15:0] & B[15:0];
                temp_result = {17'b0, temp_result[15:0]}; // clear most significant 16 bits
                ALUOut = temp_result[31:0];
                

            end
            5'b01000: begin
                temp_result[15:0] = A[15:0] | B[15:0];
                temp_result = {17'b0, temp_result[15:0]}; // clear most significant 16 bits
                ALUOut = temp_result[31:0];

            end
            5'b01001: begin
                temp_result[15:0] = A[15:0] ^ B[15:0];
                temp_result = {17'b0, temp_result[15:0]}; // clear most significant 16 bits
                ALUOut = temp_result[31:0];

            end
            5'b01010: begin
                temp_result[15:0] = ~(A[15:0] & B[15:0]);
                temp_result = {17'b0, temp_result[15:0]}; // clear most significant 16 bits
                ALUOut = temp_result[31:0];

            end
            5'b01011: begin // padded with 0 for 16-bit shift operations
                temp_result[15:0] = {A[14:0], 1'b0};
                temp_result = {17'b0, temp_result[15:0]}; // Logic shift left
                ALUOut = temp_result[31:0];

            end
            5'b01100: begin
                temp_result[15:0] = {1'b0, A[15:1]};
                temp_result = {17'b0, temp_result[15:0]}; // Logic shift right
                ALUOut = temp_result[31:0];
            end
            5'b01101: begin // ASR
                temp_result[15:0] = {A[15], A[15:1]}; // Arithmetic shift right
                temp_result = {17'b0, temp_result[15:0]};
                ALUOut = temp_result[31:0];
             
            end
            5'b01110: begin
                temp_result[15:0] = {A[14:0], FlagsOut[2]};
                temp_result = {17'b0, temp_result[15:0]};
                ALUOut = temp_result[31:0]; // Circular shift left
                // flagsout[2] = a[15] // FLAG ASSIGNS DONE BELOW OF THE CODE
            end
            5'b01111: begin
                temp_result[15:0] = {FlagsOut[2], A[15:1]};
                temp_result = {17'b0, temp_result[15:0]};
                ALUOut = temp_result[31:0]; // Circular shift right
            end
            // 32-bit inputs
            5'b10000: begin
                temp_result[31:0] = A;
                ALUOut = A;

            end
            5'b10001: begin
                temp_result[31:0] = B;
                ALUOut = B;

            end
            5'b10010: begin
                temp_result[31:0] = ~A;
                ALUOut = ~A;

            end
            5'b10011: begin
                temp_result[31:0] = ~B;
                ALUOut = ~B;

            end
            5'b10100: begin
                temp_result = A + B;
                ALUOut = temp_result[31:0];

                
            end
            5'b10101: begin
                temp_result = A + B;
                temp_result = temp_result + FlagsOut[2];
                ALUOut = temp_result[31:0];

                
            end
            5'b10110: begin
                temp_result = A + ~B + 1'b1;
                ALUOut = temp_result[31:0];

                
            end
            5'b10111: begin
                temp_result[31:0] = A & B;
                ALUOut = A & B;
                 
            end
            5'b11000: begin
                temp_result[31:0] = A | B;
                ALUOut = A | B;

            end
            5'b11001: begin
                temp_result[31:0] = A ^ B;
                ALUOut = A ^ B;

            end
            5'b11010: begin
                temp_result[31:0] = ~(A & B);
                ALUOut = ~(A & B);

            end
            5'b11011: begin
                temp_result[31:0] = {A[30:0], 1'b0};
                ALUOut = {A[30:0], 1'b0}; // Logic shift left

      
            end
            5'b11100: begin
                temp_result[31:0] = {1'b0, A[31:1]};
                ALUOut = {1'b0, A[31:1]}; // Logic shift right

            end
            5'b11101: begin
                temp_result[31:0] = {A[31], A[31:1]};
                ALUOut = {A[31], A[31:1]}; // Arithmetic shift right
               
            end
            5'b11110: begin
                temp_result[31:0] = {A[30:0], FlagsOut[2]};
                ALUOut = {A[30:0], FlagsOut[2]}; // Circular shift left

            end
            5'b11111: begin
                temp_result[31:0] = {FlagsOut[2], A[31:1]};
                ALUOut = {FlagsOut[2], A[31:1]}; // Circular shift right
            end
            
            default: begin
                temp_result[31:0] = 32'b0;
                ALUOut = 32'b0; // Default behavior
            end
        endcase
    end
    
    always @(posedge Clock & WF) begin
        
        if (WF == 1'b1) begin
            case (FunSel)
            
                5'b00000: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b00001: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b00010: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b00011: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b00100: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                    FlagsOut[2] = temp_result[32];
                    FlagsOut[0] = (A[15] == B[15] && A[15] != temp_result[15]) ? 1 : 0; // Overflow flag
                end
                5'b00101: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                    FlagsOut[2] = temp_result[32];
                    FlagsOut[0] = (A[15] == B[15] && A[15] != temp_result[15]) ? 1 : 0;
                end
                5'b00110: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                    FlagsOut[2] = temp_result[32];
                    FlagsOut[0] = (A[15] != B[15] && B[15] == temp_result[15]) ? 1 : 0;
                end
                5'b00111: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b01000: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b01001: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b01010: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b01011: begin // DONE - CARRY FLAG CORRECTED
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = A[15];
                end
                5'b01100: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = A[0];
                end
                5'b01101: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                end
                5'b01110: begin // DONE - CARRY FLAG CORRECTED
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = A[15];
                end
                5'b01111: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = A[0];
                end
                5'b10000: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                end
                5'b10001: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                end
                5'b10010: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                end
                5'b10011: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                end
                5'b10100: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                    FlagsOut[2] = temp_result[32];
                    FlagsOut[0] = (A[31] == B[31] && A[31] != temp_result[31]) ? 1 : 0;
                end
                5'b10101: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                    FlagsOut[2] = temp_result[32];
                    FlagsOut[0] = (A[31] == B[31] && A[31] != temp_result[31]) ? 1 : 0;
                end
                5'b10110: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                    FlagsOut[2] = temp_result[32];
                    FlagsOut[0] = (A[31] != B[31] && B[31] == temp_result[31]) ? 1 : 0;
                end
                5'b10111: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                end
                5'b11000: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                end
                5'b11001: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                end
                5'b11010: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                end
                // SHOFT OPERATIONS
                5'b11011: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                    FlagsOut[2] = A[31];
                end
                5'b11100: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                    FlagsOut[2] = A[0];
                end
                5'b11101: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                end
                5'b11110: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                    FlagsOut[2] = A[31];
                end
                5'b11111: begin
                    FlagsOut[3] = (temp_result[31:0] == 32'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[31]; // Negative flag
                    FlagsOut[2] = A[0];
                end
            endcase
        end
    end



endmodule