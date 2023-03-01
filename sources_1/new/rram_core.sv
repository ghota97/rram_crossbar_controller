
module rram_core(CLK, CLK_ADC, RESET, WR_WL, WR_BL, WE, RE, DATAIN, ADDR, valid_i, ready_i, valid_o, ready_o, ADCSEL, ADCout);
    parameter NUM_ADCs = 32;
    input CLK;
    input CLK_ADC;
    input RESET;
    input WR_WL;
    input WR_BL;
    input [31:0] DATAIN;
    input WE;
    input RE;
    input [9:0] ADDR;
    input valid_i;
    output reg ready_i;
    input [3:0] ADCSEL;
    output reg [3:0] ADCout[NUM_ADCs-1:0];

    input  ready_o;
    output reg valid_o;
    
    reg [1023:0] WL;
    reg [1023:0] BL;
    
    always @(posedge CLK) begin
        if (RESET) begin
            WL <= 1024'b0;
            BL <= 512'b0;
            ready_i <= 1'b0;
            valid_o <= 1'b0;
		end else begin
		      ready_i <= 1'b1;
		    if(WE) WL <= (1<<ADDR);  //Programming the array, select one WL at a time to load weights
            else begin 
                if (valid_i) begin
                    if(WR_WL) WL[ADDR[9:5]] <= DATAIN; //Programming the WL registers for MVM inputs 
                    else if (WR_BL) BL[ADDR[9:5]] <= DATAIN; //Programming the BL for programming weights 
                end
            end
        end
    end
    
    always @(posedge CLK_ADC) begin
        if (RESET) valid_o <= 1'b0;
        else valid_o <= 1'b1;
    end
    
    rram_crossbar rram
    (
      .CLK(CLK), 
      .CLK_ADC(CLK_ADC),
      .RESET(RESET), 
      .WL(WL), 
      .BL(BL), 
      .ADCSEL(ADCSEL),
      .ADCout(ADCout),
      .WREN(WE), 
      .RDEN(RE)
    );
  
  
endmodule
