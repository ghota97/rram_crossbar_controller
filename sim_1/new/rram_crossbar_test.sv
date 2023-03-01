`timescale 1ns / 1ps

// Code your testbench here
// or browse Examples

// Code your testbench here
// or browse Examples

module rram_tb;
  parameter NUM_ADCs = 32;
  logic CLK;
  logic CLK_ADC;
  logic RESET;
  logic [1023:0] WL;
  logic [1023:0] BL;
  logic WREN;
  logic RDEN;
  logic [3:0] ADCout[NUM_ADCs-1:0];
  logic [3:0] ADCSEL;

rram_crossbar rram
    (
      .CLK(CLK), 
      .CLK_ADC(CLK_ADC),
      .RESET(RESET), 
      .WL(WL), 
      .BL(BL), 
      .ADCSEL(ADCSEL),
      .ADCout(ADCout),
      .WREN(WREN), 
      .RDEN(RDEN)
    );
    
    initial begin
      CLK = 1'b0;
      CLK_ADC = 1'b0;
      WL = 1024'b1;
      BL = 1024'b0;
      ADCSEL = 4'b0;
      RDEN = 1'b0;
      WREN = 1'b0;
      RESET = 1'b1;
      #100 RESET = 1'b0; 
      #20 WREN = 1; BL={1024{1'b1}};
      #2000 WREN = 0; 
      #20 RDEN=1'b1;
      #2000;
      #20 RDEN = 1'b0;
    end
    
     always begin
   		 #10 CLK = !CLK;
       	 CLK_ADC = !CLK_ADC;
  	 end
 
endmodule