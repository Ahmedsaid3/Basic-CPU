`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Yavuz Selim Kara and Ahmed Said Gülşen
// Project Name: BLG222E Project 2
//////////////////////////////////////////////////////////////////////////////////
module CPUSystem(
    input wire Clock,
    input wire Reset,
    // input wire T_Reset, 

    output reg [11:0] T
    );
    // Registers for Instruction
    reg [5:0] Opcode; // 6 - bit
    reg [7:0] Address; // 8 - bit address
    reg [2:0] DestReg; // 3 bit destination reg
    reg [2:0] SrcReg1; // 3 bit source1 reg
    reg [2:0] SrcReg2;  // 3 bit source1 reg
    reg [1:0] RegSel;  // 2 bit reg selection
    
    reg T_Reset;
    // Registers (or Wires?) for ALUSystem    
    reg [2:0] RF_OutASel, RF_OutBSel, RF_FunSel;
    reg [3:0] RF_RegSel, RF_ScrSel;
    reg [4:0] ALU_FunSel;
    reg ALU_WF;
    reg [1:0] ARF_OutCSel, ARF_OutDSel, ARF_FunSel;
    reg [2:0] ARF_RegSel;
    reg IR_LH, IR_Write, Mem_WR, Mem_CS;
    reg DR_E;
    reg [1:0] DR_FunSel;
    reg [1:0] MuxASel, MuxBSel, MuxCSel;
    reg MuxDSel;

    
    wire [15:0] IROut;
    
    ArithmeticLogicUnitSystem ALUSys(
            .RF_OutASel(RF_OutASel),   .RF_OutBSel(RF_OutBSel), 
            .RF_FunSel(RF_FunSel),     .RF_RegSel(RF_RegSel),
            .RF_ScrSel(RF_ScrSel),     .ALU_FunSel(ALU_FunSel),
            .ALU_WF(ALU_WF),           .ARF_OutCSel(ARF_OutCSel), 
            .ARF_OutDSel(ARF_OutDSel), .ARF_FunSel(ARF_FunSel),
            .ARF_RegSel(ARF_RegSel),   .IR_LH(IR_LH),
            .IR_Write(IR_Write),       .Mem_WR(Mem_WR),
            .Mem_CS(Mem_CS),           .MuxASel(MuxASel),
            .MuxBSel(MuxBSel),         .MuxCSel(MuxCSel),
            .Clock(Clock),         .DR_FunSel(DR_FunSel),
            .DR_E(DR_E),               .MuxDSel(MuxDSel)
        ); 
    

    
       // ARF Source Selection - - Instruction format 2
      function [1:0] ARF_SourceSel;
        input [2:0] in;
        begin
            ARF_SourceSel = in[1:0];
        end
      endfunction
      
      // ARF Destination Selection
        function [2:0] ARF_DestSel;
          input [2:0] in;
          begin
              case(in[1:0])
                  2'b00: begin
                      ARF_DestSel = 3'b100;         // Only PC enabled
                  end
                  2'b01: begin
                      ARF_DestSel = 3'b010;          // Only SP 
                  end
                  2'b10: begin
                      ARF_DestSel = 3'b001;     // Only AR      
                  end
                  2'b11: begin
                      ARF_DestSel = 3'b001;      // Only AR 
                  end
              endcase
          end
        endfunction
        
        // RF Source Selection
            function [2:0] RF_SourceSel;
              input [2:0] in;
              begin
                    RF_SourceSel = {1'b0,in[1:0]};
                end
            endfunction
                    
        // RF Destination Selection
            function [3:0] RF_DestSel;
              input [2:0] in;
              begin
                  case(in[1:0])
                      2'b00: begin // R1
                          RF_DestSel = 4'b1000;
                      end
                      2'b01: begin // R2
                          RF_DestSel = 4'b0100;                
                      end
                      2'b10: begin // R3 
                          RF_DestSel = 4'b0010;                
                      end
                      2'b11: begin // R4
                          RF_DestSel = 4'b0001;                
                      end
                  endcase
              end
            endfunction

            // RF RegSel - Instruction format 1
            function [3:0] RF_RSel;
              input [1:0] in;
              begin
                  case(in)
                      2'b00: begin // R1
                          RF_RSel = 4'b1000;
                      end
                      2'b01: begin // R2 
                          RF_RSel = 4'b0100;                
                      end
                      2'b10: begin // R3
                          RF_RSel = 4'b0010;                
                      end
                      2'b11: begin // R4
                          RF_RSel = 4'b0001;                
                      end
                  endcase
              end
            endfunction

            function [2:0] RF_OutASel_RSel;
              input [1:0] in;
              begin
                  case(in)
                      2'b00: begin
                          RF_OutASel_RSel = 3'b000;
                      end
                      2'b01: begin
                          RF_OutASel_RSel = 3'b001;                
                      end
                      2'b10: begin
                          RF_OutASel_RSel = 3'b010;                
                      end
                      2'b11: begin
                          RF_OutASel_RSel = 3'b011;                
                      end
                  endcase
              end
            endfunction
      
      // State encoding: One-hot encoding for an 8-state counter
      localparam T0 =  12'b0000_0000_0001;
      localparam T1 =  12'b0000_0000_0010;
      localparam T2 =  12'b0000_0000_0100;
      localparam T3 =  12'b0000_0000_1000;
      localparam T4 =  12'b0000_0001_0000;
      localparam T5 =  12'b0000_0010_0000;
      localparam T6 =  12'b0000_0100_0000;
      localparam T7 =  12'b0000_1000_0000;
      localparam T8 =  12'b0001_0000_0000;
      localparam T9 =  12'b0010_0000_0000;
      localparam T10 = 12'b0100_0000_0000;
      localparam T11 = 12'b1000_0000_0000;
      
      initial begin
           Mem_CS <= 1'b1; // disable memory
           IR_Write <= 1'b0; // disable IR_write
           DR_E <= 1'b0; // disable DR
           RF_RegSel <= 4'b0; // disable Rx
           RF_ScrSel <= 4'b0; // disable Sx
           ARF_RegSel <= 3'b0; // disable arf regs
           ALU_WF <= 1'b0; // disable ALU flags
          T = T0;
      end
      
      always @(negedge Reset or posedge T_Reset) begin
            if (T_Reset) begin
                T = T0;
            end
            if (!Reset) begin
                Mem_CS <= 1'b1; // disable memory
                IR_Write <= 1'b0; // disable IR_write
                DR_E <= 1'b0; // disable DR
                RF_RegSel <= 4'b1111; // Rx
                RF_ScrSel <= 4'b1111; // Sx
                RF_FunSel <= 3'b011; // clear
                ARF_RegSel <= 3'b111; // PC AR SP
                ARF_FunSel <= 2'b11; // Clear
                ALU_WF <= 1'b0; // disable ALU flags
            end
      end
      
      always @(posedge Clock) begin
            if(T == T0) begin
                T = T1;
            end else if (T == T1) begin
                T = T2;
            end else if (T == T2) begin
                T = T3;
            end else if (T == T3) begin
                T = T4;
            end else if (T == T4) begin
                T = T5;
            end else if (T == T5) begin
                T = T6;
            end else if (T == T6) begin
                T = T7;
            end else if (T == T7) begin
                T = T8;
            end else if (T == T8) begin
                T = T9;
            end else if (T == T9) begin
                T = T10;
            end else if (T == T10) begin
                T = T11;
            end else if (T == T11) begin
                T = T0;
            end
        end

    //Mem_CS <= 1'b1; // disable memory
    //IR_Write <= 1'b0; // disable IR_write
    //DR_E <= 1'b0; // disable DR
    //RF_RegSel <= 4'b0; // disable Rx
    //RF_ScrSel <= 4'b0; // disable Sx
    //ARF_RegSel <= 3'b0; // disable arf regs
    //ALU_WF <= 1'b0; // disable ALU flags
        


    
    always @(*) begin
            case(T)
                T0: begin              
                    RF_RegSel <= 4'b0000; // disable Rx
                    RF_ScrSel <= 4'b0000; // disable Sx
                    ALU_WF <= 1'b0; // disable ALU flags
                    DR_E <= 1'b0; // disable DR

                    // PC to Memory
                    ARF_OutDSel <= 2'b00;
                    
                    // Memory read  ---- Memory write and enable flags 0
                    Mem_CS <= 1'b0;
                    Mem_WR <= 1'b0;
                    
                    // load IR[7:0]
                    IR_Write <= 1'b1;
                    IR_LH <= 1'b0; 
                    
                    // PC Increment
                    ARF_RegSel <= 3'b100;
                    ARF_FunSel <= 2'b01;
                end
                T1: begin
                    RF_RegSel <= 4'b0000; // disable Rx
                    RF_ScrSel <= 4'b0000; // disable Sx 
                    ALU_WF <= 1'b0; // disable ALU flags
                    DR_E <= 1'b0; // disable DR

                    // PC to Memory
                    ARF_OutDSel <= 2'b00;
                    
                    // Memory read  ---- Memory write and enable flags 0
                    Mem_CS <= 1'b0;
                    Mem_WR <= 1'b0;
                    
                    // Load IR[15:8]
                    IR_Write <= 1'b1;
                    IR_LH <= 1'b1; 
                    
                    // PC Increment
                    ARF_RegSel <= 3'b100;
                    ARF_FunSel <= 2'b01;
                    
                end
                T2: begin
                    // Disable Memory
                    // Mem_CS <= 1'b1;
                    // Stop PC Increment
                    // ifle ARF_RegSel atanan durumlarda Ã§alÄ±ÅŸmamasÄ± saÄŸlanacak
                    //ARF_RegSel <= 3'b111;
                    // Disable IR
                    // IR_Write <= 1'b0;
                    
                    // Fetch the instruction
                    Opcode <= ALUSys.IR.IROut[15:10];
                    RegSel <= ALUSys.IR.IROut[9:8];
                    Address <= ALUSys.IR.IROut[7:0];
                    DestReg <= ALUSys.IR.IROut[9:7];
                    SrcReg1 <= ALUSys.IR.IROut[6:4];
                    SrcReg2 <= ALUSys.IR.IROut[3:1];
                     
                    case(Opcode)
                        6'b000000: begin // BRA - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0000; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // VALUE TO PC

                            // Choose Output of MUX B
                            MuxBSel <= 2'b11;
                            
                            // Load MuxBOut to PC
                            ARF_RegSel <= 3'b100;
                            ARF_FunSel <= 2'b10;
                        end

                        6'b000001: begin // BNE - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            if (ALUSys.ALU.FlagsOut[3] == 0) begin
                                // VALUE TO PC

                                // Choose Output of MUX B
                                MuxBSel <= 2'b11;
                                
                                // Load MuxBOut to PC
                                ARF_RegSel <= 3'b100;
                                ARF_FunSel <= 2'b10;
                            end else begin
                                ARF_RegSel <= 3'b000; // disable ARF
                                T <= T0;
                            end
                        end
                        
                        6'b000010: begin // BEQ - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags
                            
                            if (ALUSys.ALU.FlagsOut[3] == 1) begin
                                // VALUE TO PC

                                // Choose Output of MUX B
                                MuxASel <= 2'b11;
                                
                                // Load MuxBOut to PC
                                ARF_RegSel <= 3'b100;
                                ARF_FunSel <= 2'b10;
                            end else begin
                                ARF_RegSel <= 3'b000;
                                T <= T0;
                            end
                        end
                        
                        6'b000011: begin // POPL - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0000; // disable Rx
                            RF_ScrSel <= 4'b0000; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags    

                            // Increment SP
                            ARF_RegSel = 3'b010; // SP Enable
                            ARF_FunSel = 2'b01; // FunSel = 001 / Inc
                        end
                        
                        6'b000100: begin // PSHL - done
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0000; // disable Rx
                            RF_ScrSel <= 4'b0000; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
                            
                            // M[SP] <= Rx[7:0] - (OutB = Rx) 
                            RF_OutBSel <= RF_OutASel_RSel(RegSel); // OutA <= Rx
                            ALU_FunSel <= 5'b00001; // ALU_Out <= OutB
                            MuxCSel <= 2'b00;// MuxC_Out <= ALUOut[7:0]
                            ARF_OutDSel <= 2'b01; // OutD <= SP
                            
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;

                            // SP Decrement (-1)
                            ARF_RegSel <= 3'b010; 
                            ARF_FunSel <= 2'b00;
                        end
                        
                        6'b000101: begin // POPH - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0000; // disable Rx
                            RF_ScrSel <= 4'b0000; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags    

                            // Increment SP
                            ARF_RegSel = 3'b110; // SP Enable
                            ARF_FunSel = 2'b01; // FunSel = 001 / Inc
                        end
                        
                        6'b000110: begin // PSHH - done
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0000; // disable Rx
                            RF_ScrSel <= 4'b0000; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
                            
                            // M[SP] <= Rx[7:0] - (OutB = Rx) 
                            RF_OutBSel <= RF_OutASel_RSel(RegSel); // OutA <= Rx
                            ALU_FunSel <= 5'b10001; // ALU_Out <= OutB
                            MuxCSel <= 2'b00;// MuxC_Out <= ALUOut[7:0]
                            ARF_OutDSel <= 2'b01; // OutD <= SP
                            
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;

                            // SP Decrement (-1)
                            ARF_RegSel <= 3'b010; 
                            ARF_FunSel <= 2'b00;
                        end
                        
                        6'b000111: begin // CALL - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags  
                            // S1 <- PC                                        
                            ARF_OutCSel = 2'b00; // PC
                            MuxASel = 2'b01; // ARF OutC
                            RF_ScrSel = 4'b1000; // Select S1
                            RF_FunSel = 3'b010; // Load
                        end
                        6'b001000: begin // RET - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags   

                           // Increment SP
                           ARF_RegSel = 3'b010; // SP Enable
                           ARF_FunSel = 2'b01; // FunSel = 001 / Inc
                        end
                        6'b001001: begin // INC - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                            // Source larÄ± Ã¶nce S1 S2 ye atacaÄŸÄ±z
                            // sonra iÅŸlem gÃ¶rmÃ¼ÅŸ halini uygun yere atacaÄŸÄ±z
                            if (SrcReg1[2] == 0) begin // ARF ----- sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 1000 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin // RF
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 1000 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end                            
                        end                 
                        6'b001010: begin // DEC - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                             
                            // Source larÄ± Ã¶nce S1 S2 ye atacaÄŸÄ±z
                            // sonra iÅŸlem gÃ¶rmÃ¼ÅŸ halini uygun yere atacaÄŸÄ±z
                            if (SrcReg1[2] == 0) begin // ARF ----- sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 1000 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin // RF
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags
                                ALU_FunSel = 5'b10001; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 1000 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end                            
                        end

                        6'b001011: begin // LSL - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                            // In T2 send sreg1 to s1, In T3 send s1 to alu and to the dest
                            // Source larÄ± Ã¶nce S1 S2 ye atacaÄŸÄ±z
                            // sonra iÅŸlem gÃ¶rmÃ¼ÅŸ halini uygun yere atacaÄŸÄ±z
                            if (SrcReg1[2] == 0) begin // ARF -- sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin // RF
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = B - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = B
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b001100: begin // LSR - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                            // In T2 send sreg1 to s1, In T3 send s1 to alu and to the dest
                            // Source larÄ± Ã¶nce S1 S2 ye atacaÄŸÄ±z
                            // sonra iÅŸlem gÃ¶rmÃ¼ÅŸ halini uygun yere atacaÄŸÄ±z
                            if (SrcReg1[2] == 0) begin // ARF -- sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin // RF
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = B - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = B
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b001101: begin // ASR - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable arf regs
                            // In T2 send sreg1 to s1, In T3 send s1 to alu and to the dest
                            // Source larÄ± Ã¶nce S1 S2 ye atacaÄŸÄ±z
                            // sonra iÅŸlem gÃ¶rmÃ¼ÅŸ halini uygun yere atacaÄŸÄ±z
                            if (SrcReg1[2] == 0) begin // ARF -- sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin // RF
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = B - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = B
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel =  - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b001110: begin // CSL - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable arf regs
                            // In T2 send sreg1 to s1, In T3 send s1 to alu and to the dest
                            // Source larÄ± Ã¶nce S1 S2 ye atacaÄŸÄ±z
                            // sonra iÅŸlem gÃ¶rmÃ¼ÅŸ halini uygun yere atacaÄŸÄ±z
                            if (SrcReg1[2] == 0) begin // ARF -- sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin // RF
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = B - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = B
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b001111: begin // CSR - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable arf regs
                            // In T2 send sreg1 to s1, In T3 send s1 to alu and to the dest
                            // Source larÄ± Ã¶nce S1 S2 ye atacaÄŸÄ±z
                            // sonra iÅŸlem gÃ¶rmÃ¼ÅŸ halini uygun yere atacaÄŸÄ±z
                            if (SrcReg1[2] == 0) begin // ARF -- sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel =  - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin // RF
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = B - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = B
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end
                        6'b010000: begin // NOT - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable arf regs
                            // In T2 send sreg1 to s1, In T3 send s1 to alu and to the dest
                            // Source larÄ± Ã¶nce S1 S2 ye atacaÄŸÄ±z
                            // sonra iÅŸlem gÃ¶rmÃ¼ÅŸ halini uygun yere atacaÄŸÄ±z
                            if (SrcReg1[2] == 0) begin // ARF -- sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel =  - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin // RF
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = B - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = B
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end
                        6'b010001: begin // AND - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable arf regs
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SrcReg1[2] == 0) begin // sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end
                        6'b010010: begin // ORR - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable arf regs
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SrcReg1[2] == 0) begin // sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end
                        6'b010011: begin // XOR - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable arf regs
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SrcReg1[2] == 0) begin // sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end
                        6'b010100: begin // NAND - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable arf regs
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SrcReg1[2] == 0) begin // sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end
                        6'b010101: begin // ADD - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable arf regs
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SrcReg1[2] == 0) begin // sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end
                        6'b010110: begin // ADC - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable arf regs
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SrcReg1[2] == 0) begin // sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end
                        6'b010111: begin // SUB - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable arf regs
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SrcReg1[2] == 0) begin // sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end
                        6'b011000: begin // MOV - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0000; // disable Rx 
                            ARF_RegSel <= 3'b000; // disable arf regs
                            // Source larÄ± Ã¶nce S1 S2 ye atacaÄŸÄ±z
                            // sonra iÅŸlem gÃ¶rmÃ¼ÅŸ halini uygun yere atacaÄŸÄ±z
                            if (SrcReg1[2] == 0) begin // ARF ----- sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 1000 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin // RF
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 1000 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end 
                        end
                        6'b011001: begin // MOVL - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                             
                            // Rsel <= IMMEDIATE (8-bit)// immediate = IR[7:0]
                            // LH - choose Least part of IR (IR[7:0])
                            
                            MuxASel <= 2'b11;
                            RF_RegSel <= RF_RSel(RegSel);
                            RF_FunSel <= 3'b100; // Clear others -  Write Low
                        end
                        6'b011010: begin // MOVSH - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Rsel <= IMMEDIATE (8-bit)// immediate = IR[7:0]
                            // LH - choose Least part of IR (IR[7:0])
                            MuxASel <= 2'b11;
                            RF_RegSel <= RF_RSel(RegSel);
                            RF_FunSel <= 3'b110; // Shift Left and Write Low
                        end
                        6'b011011: begin // LDARL (16-bit) - done
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable DR
                            DR_FunSel <= 2'b01; // clear most load 7:0
                            
                            // AR INC
                            ARF_RegSel <= 3'b001; // only AR
                            ARF_FunSel <= 2'b01;
                        end
                        6'b011100: begin // LDARH (32-bit) - done
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable DR
                            DR_FunSel <= 2'b01; // clear most load 7:0
                            
                            // AR INC
                            ARF_RegSel <= 3'b001; // only AR
                            ARF_FunSel <= 2'b01;
                        end
                        6'b011101: begin // STAR   - done       
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF                          
                            
                            if (SrcReg1[2] == 0) begin // ARF ----- sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags 
                                ARF_OutCSel <= ARF_SourceSel(SrcReg1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 1000 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin // RF
                                RF_OutBSel <= RF_SourceSel(SrcReg1);
                                ALU_WF <= 1'b1; // enable ALU flags 
                                ALU_FunSel = 5'b10001; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1000; // write to S1 - RF_ScrSel = 1000 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end
                        6'b011110: begin // LDAL (16-bit) - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags
                            
                            // AR <= IROut[7:0]
                            MuxBSel <= 2'b11;
                            ARF_RegSel <= 3'b001;
                            ARF_FunSel <= 2'b10;
                        end
                        6'b011111: begin // LDAH (32-bit) - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags
                            
                            // AR <= IROut[7:0]
                            MuxBSel <= 2'b11;
                            ARF_RegSel <= 3'b001;
                            ARF_FunSel <= 2'b10;
                        end
                        6'b100000: begin // STA - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags                         
                            
                            MuxBSel <= 2'b11; // IROut
                            ARF_RegSel <= 3'b001; // AR
                            ARF_FunSel <= 2'b10; // Load
                        end
                        6'b100001: begin // LDDRL (16-bit) - done
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags
    
                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable DR
                            DR_FunSel <= 2'b01; // clear most load 7:0
                            
                            // AR INC
                            ARF_RegSel <= 3'b001; // only AR
                            ARF_FunSel <= 2'b01;
                        end
                        6'b100010: begin // LDDRH (32-bit) - done
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable DR
                            DR_FunSel <= 2'b01; // clear most load 7:0
                            
                            // AR INC
                            ARF_RegSel <= 3'b001; // only AR
                            ARF_FunSel <= 2'b01;
                        end
                        6'b100011: begin // STDR - done
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            if (SrcReg1[2] == 0) begin // ARF 
                                RF_RegSel <= 4'b0; // disable Rx
                                MuxBSel <= 2'b10; // MuxBOut = DR
                                ARF_RegSel <= ARF_DestSel(SrcReg1);
                                ARF_FunSel <= 2'b10; // load
                            end else begin // RF
                                ARF_RegSel <= 3'b0; // disable ARF
                                MuxASel = 2'b10; // MuxAOut = DR
                                RF_RegSel <= RF_DestSel(SrcReg1);
                                RF_FunSel <= 3'b010; // load
                            end
                        end
                        6'b100100: begin // STRIM
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags

                            // M[AR+OFFSET] <= Rx (AR is 16-bit register)
                            MuxASel <= 2'b11; // Choose offset (IR)
                            RF_ScrSel <= 4'b1000; // load to S1
                            RF_FunSel <= 3'b010;
                        end
                    endcase
                end
                T3: begin
                    case(Opcode)
                        6'b000000: begin // BRA
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b000001: begin // BNE
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;     
                        end
                        6'b000010: begin // BEQ
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;   
                        end
                        6'b000011: begin // POPL
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Load SP to Memory
                            ARF_OutDSel <= 2'b01; // SP to OutD(Address)

                            // Read memory
                            Mem_WR <= 1'b0;
                            Mem_CS <= 1'b0;

                            // DR[7:0] <= MemOut - clear other bits of DR
                            DR_E <= 1'b1;
                            DR_FunSel <= 2'b01;
                            
                            // Increment SP
                            ARF_RegSel <= 3'b010;
                            ARF_FunSel <= 2'b01;

                            // MemOut to Rx[7:0]
                        end
                        6'b000100: begin // PSHL
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
                            
                            // M[SP] <= Rx[15:8] - (OutB = Rx) 
                            RF_OutBSel <= RF_OutASel_RSel(RegSel); // OutA <= Rx
                            ALU_FunSel <= 5'b00001; // ALU_Out <= OutB
                            MuxCSel <= 2'b01;// MuxC_Out <= ALUOut[15:8]
                            ARF_OutDSel <= 2'b01; // OutD <= SP
                            
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;
    
                            // SP Decrement (-1)
                            ARF_RegSel <= 3'b010; 
                            ARF_FunSel <= 2'b00;
                        end
                        6'b000101: begin // POPH
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Load SP to Memory
                            ARF_OutDSel <= 2'b01; // SP to OutD(Address)

                            // Read memory
                            Mem_WR <= 1'b0;
                            Mem_CS <= 1'b0;

                            // DR[7:0] <= MemOut - clear other bits of DR
                            DR_E <= 1'b0;
                            DR_FunSel <= 2'b01;
                            
                            // Increment SP
                            ARF_RegSel <= 3'b010;
                            ARF_FunSel <= 2'b01;

                            // MemOut to Rx[7:0]
                        end
                        6'b000110: begin // PSHH
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
                            
                            // M[SP] <= Rx[15:8] - (OutB = Rx) 
                            RF_OutBSel <= RF_OutASel_RSel(RegSel); // OutA <= Rx
                            ALU_FunSel <= 5'b10001; // ALU_Out <= OutB
                            MuxCSel <= 2'b01;// MuxC_Out <= ALUOut[15:8]
                            ARF_OutDSel <= 2'b01; // OutD <= SP
                            
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;

                            // SP Decrement (-1)
                            ARF_RegSel <= 3'b010; 
                            ARF_FunSel <= 2'b00;
                        end
                        6'b000111: begin // CALL
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
                            
                            // M[SP] <= S1[7:0] - (OutB = S1) 
                            RF_OutBSel <= 3'b100; // OutB <= S1
                            ALU_FunSel <= 5'b00001; // ALU_Out <= OutB
                            MuxCSel <= 2'b00;// MuxC_Out <= ALUOut[7:0]
                            ARF_OutDSel <= 2'b01; // OutD <= SP
                            
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;

                            // SP Decrement (-1)
                            ARF_RegSel <= 3'b010; 
                            ARF_FunSel <= 2'b00;
                        end
                        6'b001000: begin // RET
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Load SP to Memory
                            ARF_OutDSel <= 2'b01; // SP to OutD(Address)

                            // Read memory
                            Mem_WR <= 1'b0;
                            Mem_CS <= 1'b0;

                            // DR[7:0] <= MemOut - clear other bits of DR
                            DR_E <= 1'b1;
                            DR_FunSel <= 2'b01;
                            
                            // Increment SP
                            ARF_RegSel <= 3'b010;
                            ARF_FunSel <= 2'b01;
                        end
                        6'b001001: begin // INC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // S1 Increment
                            RF_ScrSel <= 4'b1000; // S1 enabled
                            RF_FunSel <= 3'b001; // Rx + 1 / increment        
                        end
                        6'b001010: begin // DEC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // S1 Decrement
                            RF_ScrSel <= 4'b1000; // S1 enabled
                            RF_FunSel <= 3'b000; // Rx - 1 / decrement        
                        end
                        6'b001011: begin // LSL
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags

                            RF_OutASel <= 3'b100; // S1 to OutA
                            MuxDSel <= 1'b0; // RF 
                            ALU_FunSel <= 5'b11011; // LSL(S1) to ALUOut
                            if(DestReg[2] == 0) begin // LSL(S1) to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // LSL(S1) to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // LSL(S1) loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // LSL(S1) to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b001100: begin // LSR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sxs
                            
                            ALU_WF <= 1'b1; // enable ALU flags

                            RF_OutASel <= 3'b100; // S1 to OutA
                            MuxDSel <= 1'b0; // RF 
                            ALU_FunSel <= 5'b11100; // LSR(S1) to ALUOut
                            if(DestReg[2] == 0) begin // LSR(S1) to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // LSR(S1) to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // LSR(S1) loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // LSR(S1) to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b001101: begin // ASR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags

                            RF_OutASel <= 3'b100; // S1 to OutA
                            MuxDSel <= 1'b0; // RF 
                            ALU_FunSel <= 5'b11101; // ASR(S1) to ALUOut
                            if(DestReg[2] == 0) begin // ASR(S1) to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // ASR(S1) to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // ASR(S1) loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // ASR(S1) to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b001110: begin // CSL
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags

                            RF_OutASel <= 3'b100; // S1 to OutA
                            MuxDSel <= 1'b0; // RF 
                            ALU_FunSel <= 5'b11110; // CSL(S1) to ALUOut
                            if(DestReg[2] == 0) begin // LSL(S1) to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // CSL(S1) to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // CSL(S1) loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // CSL(S1) to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b001111: begin // CSR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags

                            RF_OutASel <= 3'b100; // S1 to OutA
                            MuxDSel <= 1'b0; // RF 
                            ALU_FunSel <= 5'b11111; // CSR(S1) to ALUOut
                            if(DestReg[2] == 0) begin // CSR(S1) to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // CSR(S1) to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // CSR(S1) loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // CSR(S1) to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b010000: begin // NOT
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags

                            RF_OutASel <= 3'b100; // S1 to OutA
                            MuxDSel <= 1'b0; // RF 
                            ALU_FunSel <= 5'b10010; // NOT(S1) to ALUOut
                            if(DestReg[2] == 0) begin // NOT(S1) to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // NOT(S1) to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // NOT(S1) loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // NOT(S1) to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b010001: begin // AND
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SrcReg2[2] == 0) begin // sent to S2 !!
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SrcReg2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0100; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutBSel <= RF_SourceSel(SrcReg2);
                                ALU_WF <= 1'b1; // enable ALU flags
                                ALU_FunSel = 5'b10001; // ALUOut = B - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = B
                                RF_ScrSel <= 4'b0100; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b010010: begin // ORR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SrcReg2[2] == 0) begin // sent to S2 !!
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SrcReg2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0100; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutBSel <= RF_SourceSel(SrcReg2);
                                ALU_WF <= 1'b1; // enable ALU flags
                                ALU_FunSel = 5'b10001; // ALUOut = B - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = B
                                RF_ScrSel <= 4'b0100; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        
                        6'b010011: begin // XOR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SrcReg2[2] == 0) begin // sent to S2 !!
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SrcReg2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0100; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutBSel <= RF_SourceSel(SrcReg2);
                                ALU_WF <= 1'b1; // enable ALU flags
                                ALU_FunSel = 5'b10001; // ALUOut = B - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = B
                                RF_ScrSel <= 4'b0100; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b010100: begin // NAND
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SrcReg2[2] == 0) begin // sent to S2 !!
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SrcReg2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0100; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutBSel <= RF_SourceSel(SrcReg2);
                                ALU_WF <= 1'b1; // enable ALU flags
                                ALU_FunSel = 5'b10001; // ALUOut = B - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = B
                                RF_ScrSel <= 4'b0100; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b010101: begin // ADD
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SrcReg2[2] == 0) begin // sent to S2 !!
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SrcReg2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0100; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutBSel <= RF_SourceSel(SrcReg2);
                                ALU_WF <= 1'b1; // enable ALU flags
                                ALU_FunSel = 5'b10001; // ALUOut = B - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = B
                                RF_ScrSel <= 4'b0100; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b010110: begin // ADC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SrcReg2[2] == 0) begin // sent to S2 !!
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SrcReg2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0100; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutBSel <= RF_SourceSel(SrcReg2);
                                ALU_WF <= 1'b1; // enable ALU flags
                                ALU_FunSel = 5'b10001; // ALUOut = B - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = B
                                RF_ScrSel <= 4'b0100; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b010111: begin // SUB
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SrcReg2[2] == 0) begin // sent to S2 !!
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SrcReg2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0100; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutBSel <= RF_SourceSel(SrcReg2);
                                ALU_WF <= 1'b1; // enable ALU flags
                                ALU_FunSel = 5'b10001; // ALUOut = B - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = B
                                RF_ScrSel <= 4'b0100; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b011000: begin // MOV 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags

                            RF_OutBSel <= 3'b100; // S1 to OutB
                            ALU_FunSel <= 5'b10001; // S1 to ALUOut
                            if(DestReg[2] == 0) begin // S1 to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // S1 to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // S1 loaded destination                               
                            end else begin // S1 to RF
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // S1 to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b011001: begin // MOVL 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011010: begin // MOVSH 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011011: begin // LDARL (16-bit) 
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable DR
                            DR_FunSel <= 2'b10; // shift left load 7:0
                            
                            // AR INC
                            ARF_RegSel <= 3'b001; // only AR
                            ARF_FunSel <= 2'b01;
                        end
                        6'b011100: begin // LDARH (32-bit) 
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable DR
                            DR_FunSel <= 2'b10; // shift left load 7:0
                            
                            // AR INC
                            ARF_RegSel <= 3'b001; // only AR
                            ARF_FunSel <= 2'b01;
                        end
                        6'b011101: begin // STAR           
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
    
                            // M[AR] <= S1 (AR is 16-bit register)
                            RF_OutBSel <= 3'b100; // S1
                            ALU_FunSel <= 5'b10001; // S1 to ALUOut
                            MuxCSel <= 2'b11; // first most part [31:24]
                            // AR to Address
                            ARF_OutDSel <= 2'b10; // AR to OutD
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;
                            // AR Increment
                            ARF_RegSel <= 3'b001;
                            ARF_FunSel <= 2'b01;
                        end
                        6'b011110: begin // LDAL (16-bit) 
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags
    
                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable
                            DR_FunSel <= 2'b01; // clear most load [7:0]
                            // AR INC
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 2'b01;
                        end
                        6'b011111: begin // LDAH (32-bit) 
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags
    
                            // Rx <= M[AR] (AR is 16-bit register) 
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable
                            DR_FunSel <= 2'b01; // clear most load [7:0]
                            // AR INC
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 2'b01;
                        end
                        6'b100000: begin // STA
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
    
                            // M[AR] <= Rx (AR is 16-bit register)
                            RF_OutBSel <= RF_OutASel_RSel(RegSel);
                            ALU_FunSel <= 5'b10001; // Rx to ALUOut
                            MuxCSel <= 2'b11; // first write most part
                            // AR to Address
                            ARF_OutDSel <= 2'b10; // AR to OutD
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;
                            // AR Increment
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 2'b01;
                        end
                        6'b100001: begin // LDDRL (16-bit)
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable DR
                            DR_FunSel <= 2'b10; // shift left load 7:0
                            
                            // AR INC
                            ARF_RegSel <= 3'b001; // only AR
                            ARF_FunSel <= 2'b01;
                        end
                        6'b100010: begin // LDDRH (32-bit) 
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable DR
                            DR_FunSel <= 2'b10; // shift left load 7:0
                            
                            // AR INC
                            ARF_RegSel <= 3'b001; // only AR
                            ARF_FunSel <= 2'b01;
                        end
                        6'b100011: begin // STDR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b100100: begin // STRIM
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
    
                            // M[AR+OFFSET] <= Rx (AR is 16-bit register)
                            ARF_OutCSel <= 2'b10; // AR  
                            MuxASel <= 2'b01; // OutC - AR
                            
                            RF_ScrSel <= 4'b0100; // load to S2
                            RF_FunSel <= 3'b010;  // load
                        end
                    endcase
                end
                T4: begin
                    case(Opcode)
                        6'b000011: begin // POPL
                            IR_Write <= 1'b0; // disable IR_write
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
        
                            // Load SP to Memory
                            ARF_OutDSel <= 2'b01; // SP to OutD(Address)
        
                            // Read memory
                            Mem_WR <= 1'b0;
                            Mem_CS <= 1'b0;
        
                            // DR[15:8] <= MemOut 
                            DR_E <= 1'b1;
                            DR_FunSel <= 2'b10;
                            
                            // DR to Rx
                            MuxASel <= 2'b10; // Mux_AOut <= DROut
                            RF_RegSel <= RF_RSel(RegSel); // choose Rx with RSel func
                            RF_FunSel <=  3'b010;
                        end
                        6'b000100: begin // PSHL
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b000101: begin // POPH
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Load SP to Memory
                            ARF_OutDSel <= 2'b01; // SP to OutD(Address)

                            // Read memory
                            Mem_WR <= 1'b0;
                            Mem_CS <= 1'b0;

                            // shift left and DR[7:0] <= MemOut
                            DR_E <= 1'b1;
                            DR_FunSel <= 2'b10;
                            
                            // Increment SP
                            ARF_RegSel <= 3'b010;
                            ARF_FunSel <= 2'b01;

                        end
                        6'b000110: begin // PSHH
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
                            
                            // M[SP] <= Rx[23:16] - (OutB = Rx) 
                            RF_OutBSel <= RF_OutASel_RSel(RegSel); // OutA <= Rx
                            ALU_FunSel <= 5'b10001; // ALU_Out <= OutB
                            MuxCSel <= 2'b10;// MuxC_Out <= ALUOut[23:16]
                            ARF_OutDSel <= 2'b01; // OutD <= SP
                            
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;

                            // SP Decrement (-1)
                            ARF_RegSel <= 3'b010; 
                            ARF_FunSel <= 2'b00;
                        end
                        6'b000111: begin // CALL
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
                            
                            // M[SP] <= S1[15:8] - (OutB = S1) 
                            RF_OutBSel <= 3'b100; // OutB <= S1
                            ALU_FunSel <= 5'b00001; // ALU_Out <= OutB
                            MuxCSel <= 2'b01;// MuxC_Out <= ALUOut[15:8]
                            ARF_OutDSel <= 2'b01; // OutD <= SP
                            
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;

                            // SP Decrement (-1)
                            ARF_RegSel <= 3'b010; 
                            ARF_FunSel <= 2'b00;
                        end
                        6'b001000: begin // RET
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags
            
                            // Load SP to Memory
                            ARF_OutDSel <= 2'b01; // SP to OutD(Address)
            
                            // Read memory
                            Mem_WR <= 1'b0;
                            Mem_CS <= 1'b0;
            
                            // DR[15:8] <= MemOut 
                            DR_E <= 1'b1;
                            DR_FunSel <= 2'b10;
                            
                            // DR to PC
                            MuxBSel <= 2'b10; // Mux_BOut <= DROut
                            ARF_RegSel <= 3'b100;
                            ARF_FunSel <=  2'b10;
                        end
                        6'b001001: begin // INC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags

                            RF_OutBSel <= 3'b100; // S1 to OutB
                            ALU_FunSel <= 5'b10001; // S1 to ALUOut
                            if(DestReg[2] == 0) begin // S1 to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // S1 to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // S1 loaded destination                               
                            end else begin // S1 to RF
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // S1 to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b001010: begin // DEC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags

                            RF_OutBSel <= 3'b100; // S1 to OutB
                            ALU_FunSel <= 5'b10001; // S1 to ALUOut
                            if(DestReg[2] == 0) begin // S1 to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // S1 to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // S1 loaded destination                               
                            end else begin // S1 to RF
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // S1 to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b001011: begin // LSL
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;        
                        end
                        6'b001100: begin // LSR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;        
                        end
                        6'b001101: begin // ASR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;       
                        end
                        6'b001110: begin // CSL
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;       
                        end
                        6'b001111: begin  // CSR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;      
                        end
                        6'b010000: begin // NOT 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;     
                        end
                        6'b010001: begin // AND
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags

                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b10111; // S1 AND S2 to ALUOut
                            if(DestReg[2] == 0) begin // AND to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // AND to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // AND loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // AND to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b010010: begin // ORR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags

                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b11000; // S1 AND S2 to ALUOut
                            if(DestReg[2] == 0) begin // AND to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // AND to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // AND loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // AND to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b010011: begin // XOR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags

                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b11001; // S1 AND S2 to ALUOut
                            if(DestReg[2] == 0) begin // AND to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // AND to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // AND loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // AND to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b010100: begin // NAND
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
    
                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b11010; // S1 AND S2 to ALUOut
                            if(DestReg[2] == 0) begin // AND to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // AND to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // AND loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // AND to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b010101: begin // ADD
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags

                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b10100; // S1 AND S2 to ALUOut
                            if(DestReg[2] == 0) begin // AND to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // AND to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // AND loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // AND to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b010110: begin // ADC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
    
                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b10101; // S1 AND S2 to ALUOut
                            if(DestReg[2] == 0) begin // AND to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // AND to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // AND loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // AND to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b010111: begin // SUB
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags

                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b10110; // S1 AND S2 to ALUOut
                            if(DestReg[2] == 0) begin // AND to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b00; // AND to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // AND loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b00; // AND to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b011000: begin // MOV 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011011: begin // LDARL (16-bit) 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            if(DestReg[2] == 0) begin //  to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b10; // DR to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b10; // DR to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b011100: begin // LDARH (32-bit) 
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable DR
                            DR_FunSel <= 2'b10; // shift left load 7:0
                            
                            // AR INC
                            ARF_RegSel <= 3'b001; // only AR
                            ARF_FunSel <= 2'b01;
                        end
                        6'b011101: begin // STAR           
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
    
                            // M[AR] <= S1 (AR is 16-bit register)
                            RF_OutBSel <= 3'b100; // S1
                            ALU_FunSel <= 5'b10001; // S1 to ALUOut
                            MuxCSel <= 2'b10; // [23:16]
                            // AR to Address
                            ARF_OutDSel <= 2'b10; // AR to OutD
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;
                            // AR Increment
                            if (SrcReg1[2] == 1) begin // RF 
                                ARF_RegSel <= 3'b001;
                                ARF_FunSel <= 2'b01;
                            end else if (SrcReg1[0] == 0) begin // ARF
                                ARF_RegSel <= 3'b0; // disable ARF
                            end
                        end
                        6'b011110: begin // LDAL (16-bit) 
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
    
                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable
                            DR_FunSel <= 2'b10; // shift left load [7:0]
                            // AR INC
                            // ARF_RegSel <= 3'b101;
                            // ARF_FunSel <= 2'b01;
                        end
                        6'b011111: begin // LDAH (32-bit) 
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags
    
                            // Rx <= M[AR] (AR is 16-bit register)
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable
                            DR_FunSel <= 2'b10; // shift left load [7:0]
                            // AR INC
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 2'b01;
                        end
                        6'b100000: begin // STA
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
    
                            // M[AR] <= Rx (AR is 16-bit register)
                            RF_OutBSel <= RF_OutASel_RSel(RegSel);
                            ALU_FunSel <= 5'b10001; // Rx to ALUOut
                            MuxCSel <= 2'b10; // first write most part
                            // AR to Address
                            ARF_OutDSel <= 2'b10; // AR to OutD
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;
                            // AR Increment
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 2'b01;
                        end
                        6'b100001: begin // LDDRL (16-bit)
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b100010: begin // LDDRH (32-bit) 
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable DR
                            DR_FunSel <= 2'b10; // shift left load 7:0
                            
                            // AR INC
                            ARF_RegSel <= 3'b001; // only AR
                            ARF_FunSel <= 2'b01;
                        end
                        6'b100100: begin // STRIM
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
                            
                            // M[AR+OFFSET] <= Rx (AR is 16-bit register)
                            RF_OutASel <= 3'b100; // OutA <= S1 - OFFSET
                            MuxDSel <= 1'b0; // OutA <= OFFSET
                            // ALUOut <= AR+OFFSET
                            RF_OutBSel <= 3'b101; // OutB <= S2 - AR
                            ALU_FunSel <= 5'b10100; // ALUOut <= AR+OFFSET
                            // AR <= ALUOut
                            MuxBSel <= 2'b00; // ALUOut
                            ARF_RegSel <= 3'b001; // AR
                            ARF_FunSel <= 2'b10; // load
                        end
                    endcase
                end
                T5: begin
                    case(Opcode)
                        6'b000011: begin // POPL
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b000101: begin // POPH
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Load SP to Memory
                            ARF_OutDSel <= 2'b01; // SP to OutD(Address)

                            // Read memory
                            Mem_WR <= 1'b0;
                            Mem_CS <= 1'b0;

                            // shift left and DR[7:0] <= MemOut
                            DR_E <= 1'b1;
                            DR_FunSel <= 2'b10;
                            
                            // Increment SP
                            ARF_RegSel <= 3'b010;
                            ARF_FunSel <= 2'b01;

                        end
                        6'b000110: begin // PSHH
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
                            
                            // M[SP] <= Rx[31:24] - (OutB = Rx) 
                            RF_OutBSel <= RF_OutASel_RSel(RegSel); // OutA <= Rx
                            ALU_FunSel <= 5'b10001; // ALU_Out <= OutB
                            MuxCSel <= 2'b11;// MuxC_Out <= ALUOut[31:24]
                            ARF_OutDSel <= 2'b01; // OutD <= SP
                            
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;

                            // SP Decrement (-1)
                            ARF_RegSel <= 3'b010; 
                            ARF_FunSel <= 2'b00;
                        end
                        6'b000111: begin // CALL
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags
        
                            // VALUE TO PC
        
                            // Choose Output of MUX B
                            MuxBSel <= 2'b11;
                            
                            // Load MuxBOut to PC
                            ARF_RegSel <= 3'b100;
                            ARF_FunSel <= 2'b10;
                        end
                        6'b001000: begin // RET
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b001001: begin // INC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b001010: begin // DEC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b010001: begin // AND
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b010010: begin // ORR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b010011: begin // XOR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b010100: begin // NAND
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b010101: begin // ADD
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b010110: begin // ADC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b010111: begin // SUB
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011011: begin // LDARL (16-bit) 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011100: begin // LDARH (32-bit) 
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable DR
                            DR_FunSel <= 2'b10; // shift left load 7:0
                            
                            // AR INC
                            ARF_RegSel <= 3'b001; // only AR
                            ARF_FunSel <= 2'b01;
                        end
                        6'b011101: begin // STAR           
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            if (SrcReg1[2] == 0) begin // ARF
                                Mem_CS <= 1'b1; // disable
                                ARF_RegSel <= 3'b000; // disable
                                ALU_WF <= 1'b0; // disable ALU flags
                                T <= T0; // Reset
                            end else if (SrcReg1[2] == 1) begin // RF
                                ALU_WF <= 1'b1; // enable ALU flags
                                
                                // M[AR] <= S1 (AR is 16-bit register)
                                RF_OutBSel <= 3'b100; // S1
                                ALU_FunSel <= 5'b10001; // S1 to ALUOut
                                MuxCSel <= 2'b01; // first most part [7:0]
                                // AR to Address
                                ARF_OutDSel <= 2'b10; // AR to OutD
                                // Write to memory
                                Mem_WR <= 1'b1;
                                Mem_CS <= 1'b0;
                                // AR Increment
                                ARF_RegSel <= 3'b001;
                                ARF_FunSel <= 2'b01;
                            end
                        end
                        6'b011110: begin // LDAL (16-bit) 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Rx <= DR
                            MuxASel <= 2'b10;
                            RF_RegSel <= RF_RSel(RegSel);
                            RF_FunSel <= 3'b010;
                        end
                        6'b011111: begin // LDAH (32-bit) 
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags
    
                            // Rx <= M[AR] (AR is 16-bit register)
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable
                            DR_FunSel <= 2'b10; // shift left load [7:0]
                            // AR INC
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 2'b01;
                        end
                        6'b100000: begin // STA
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
    
                            // M[AR] <= Rx (AR is 16-bit register)
                            RF_OutBSel <= RF_OutASel_RSel(RegSel);
                            ALU_FunSel <= 5'b10001; // Rx to ALUOut
                            MuxCSel <= 2'b01; // first write most part
                            // AR to Address
                            ARF_OutDSel <= 2'b10; // AR to OutD
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;
                            // AR Increment
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 2'b01;
                        end
                        6'b100010: begin // LDDRH (32-bit) 
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable DR
                            DR_FunSel <= 2'b10; // shift left load 7:0
                            
                            // AR INC
                            ARF_RegSel <= 3'b001; // only AR
                            ARF_FunSel <= 2'b01;
                        end
                        6'b100100: begin // STRIM
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
    
                            // M[AR] <= Rx (AR is 16-bit register)
                            RF_OutBSel <= RF_OutASel_RSel(RegSel);
                            ALU_FunSel <= 5'b10001; // Rx to ALUOut
                            MuxCSel <= 2'b11; // first write most part
                            // AR to Address
                            ARF_OutDSel <= 2'b10; // AR to OutD
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;
                            // AR Increment
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 2'b01;
                        end
                    endcase
                end
                T6: begin
                    case(Opcode)
                        6'b000101: begin // POPH
                            IR_Write <= 1'b0; // disable IR_write
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
        
                            // Load SP to Memory
                            ARF_OutDSel <= 2'b01; // SP to OutD(Address)
        
                            // Read memory
                            Mem_WR <= 1'b0;
                            Mem_CS <= 1'b0;
        
                            // shift left and DR[7:0] <= MemOut 
                            DR_E <= 1'b1;
                            DR_FunSel <= 2'b10;
                            
                            // DR to Rx
                            MuxASel <= 2'b10; // Mux_AOut <= DROut
                            RF_RegSel <= RF_RSel(RegSel); // choose Rx with RSel func
                            RF_FunSel <=  3'b010;
                        end
                        6'b000110: begin // PSHH
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b000111: begin // CALL
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011100: begin // LDARH (32-bit) 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            ALU_WF <= 1'b0; // disable ALU flags

                            if(DestReg[2] == 0) begin //  to ARF
                                RF_RegSel <= 4'b0000; // disable Rx
                                MuxBSel <= 2'b10; // DR to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DestReg);
                                ARF_FunSel <= 2'b10; // loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b000; // disable arf regs
                                MuxASel <= 2'b10; // DR to MuxAOut
                                RF_RegSel <= RF_DestSel(DestReg);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b011101: begin // STAR           
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            
                            ALU_WF <= 1'b1; // enable ALU flags
                            
                            // M[AR] <= S1 (AR is 16-bit register)
                            RF_OutBSel <= 3'b100; // S1
                            ALU_FunSel <= 5'b10001; // S1 to ALUOut
                            MuxCSel <= 2'b00; // first most part [7:0]
                            // AR to Address
                            ARF_OutDSel <= 2'b10; // AR to OutD
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;
                            // AR Increment
                            // ARF_RegSel <= 3'b001;
                            // ARF_FunSel <= 2'b01;
                        end
                        6'b011110: begin // LDAL (16-bit) 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011111: begin // LDAH (32-bit) 
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
    
                            // Rx <= M[AR] (AR is 16-bit register)
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            
                            DR_E <= 1'b1; // enable
                            DR_FunSel <= 2'b10; // shift left load [7:0]
                            // AR INC
                            // ARF_RegSel <= 3'b101;
                            // ARF_FunSel <= 2'b01;
                        end
                        6'b100000: begin // STA
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            
                            ALU_WF <= 1'b1; // enable ALU flags
    
                            // M[AR] <= Rx (AR is 16-bit register)
                            RF_OutBSel <= RF_OutASel_RSel(RegSel);
                            ALU_FunSel <= 5'b10001; // Rx to ALUOut
                            MuxCSel <= 2'b00; // first write most part
                            // AR to Address
                            ARF_OutDSel <= 2'b10; // AR to OutD
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;
                            // AR Increment
                            // ARF_RegSel <= 3'b101;
                            // ARF_FunSel <= 2'b01;
                        end
                        6'b100010: begin // LDDRH (32-bit) 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b100100: begin // STRIM
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
    
                            // M[AR] <= Rx (AR is 16-bit register)
                            RF_OutBSel <= RF_OutASel_RSel(RegSel);
                            ALU_FunSel <= 5'b10001; // Rx to ALUOut
                            MuxCSel <= 2'b10; // first write most part
                            // AR to Address
                            ARF_OutDSel <= 2'b10; // AR to OutD
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;
                            // AR Increment
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 2'b01;
                        end
                    endcase
                end
                T7: begin
                    case(Opcode)
                        6'b000101: begin // POPH
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011100: begin // LDARH (32-bit) 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011101: begin // STAR           
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011111: begin // LDAH (32-bit) 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Rx <= DR
                            MuxASel <= 2'b10;
                            RF_RegSel <= RF_RSel(RegSel);
                            RF_FunSel <= 3'b010;
                        end
                        6'b100000: begin // STA
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b100100: begin // STRIM
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            
                            ALU_WF <= 1'b1; // enable ALU flags
    
                            // M[AR] <= Rx (AR is 16-bit register)
                            RF_OutBSel <= RF_OutASel_RSel(RegSel);
                            ALU_FunSel <= 5'b10001; // Rx to ALUOut
                            MuxCSel <= 2'b01; // first write most part
                            // AR to Address
                            ARF_OutDSel <= 2'b10; // AR to OutD
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;
                            // AR Increment
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 2'b01;
                        end
                    endcase
                end
                T8: begin
                    case(Opcode)
                        6'b011111: begin // LDAH (32-bit) 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b100100: begin // STRIM
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            
                            ALU_WF <= 1'b1; // enable ALU flags
    
                            // M[AR] <= Rx (AR is 16-bit register)
                            RF_OutBSel <= RF_OutASel_RSel(RegSel);
                            ALU_FunSel <= 5'b10001; // Rx to ALUOut
                            MuxCSel <= 2'b00; // first write most part
                            // AR to Address
                            ARF_OutDSel <= 2'b10; // AR to OutD
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;
                            // AR Increment
                            // ARF_RegSel <= 3'b101;
                            // ARF_FunSel <= 2'b01;
                        end
                    endcase
                end
                 T9: begin
                    case(Opcode)
                        6'b100100: begin // STRIM
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            DR_E <= 1'b0; // disable DR
                            RF_RegSel <= 4'b0; // disable Rx
                            RF_ScrSel <= 4'b0; // disable Sx
                            ARF_RegSel <= 3'b0; // disable ARF
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                    endcase
                end                   
                default: begin
                    T <= T0;
                end
            endcase
    end

endmodule