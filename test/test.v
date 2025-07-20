module Register (
input wire Load, 
input wire Clock, 
input wire [7:0] InputData, 

output reg [7:0] OutputData 
);

    always @(posedge Clock) begin 
        if (Load) 
            OutputData <= InputData; 
    end 
          
endmodule 



module TopModule (
    input wire [7:0] InVal, 
    input wire c1,
    input wire c2,
    input wire Clock, 

    output wire [7:0] OutVal 
), 

    wire [7:0] Out1; 

    Register A (
        .Load(c1),
        .Clock(Clock),
        .InputData(InVal),
        .OutputData(Out1)
    );

    Register B (
        .Load(c2),
        .Clock(Clock),
        .InputData(Out1),
        .OutputData(OutVal)
    );


endmodule