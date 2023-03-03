`timescale 1ns / 1ps

// Code your testbench here
// or browse Examples

// Code your testbench here
// or browse Examples

module rram_core_tb;
  parameter NUM_ADCs = 32;
  logic CLK;
  logic CLK_ADC;
  logic RESET;
  logic WR_WL;
  logic WR_BL;
  logic [31:0] DATAIN;
  logic [9:0] ADDR;
  logic WE;
  logic RE;
  logic [3:0] ADCout[NUM_ADCs-1:0];
  logic [3:0] ADCSEL;
  logic valid_i;
  logic ready_i;
  logic ready_o;
  logic valid_o;

 rram_core core
    (
    .CLK(CLK), 
    .CLK_ADC(CLK_ADC),
    .RESET(RESET),
    .WR_WL(WR_WL),
    .WR_BL(WR_BL),
    .DATAIN(DATAIN),
    .WE(WE),
    .RE(RE),
    .ADDR(ADDR),
    .valid_i(valid_i),
    .ready_i(ready_i),
    .valid_o(valid_o),
    .ready_o(ready_o),
    .ADCSEL(ADCSEL),
    .ADCout(ADCout)
    );


    initial begin
      CLK = 1'b0;
      CLK_ADC = 1'b0;
      ADCSEL = 4'b0;
      RE = 1'b0;
      WE = 1'b0;
      RESET = 1'b1;
      valid_i = 0;
      WR_WL = 0;
      WR_BL = 0;
      #100 RESET = 1'b0; valid_i = 1;
      #20 WR_WL = 1'b1; ADDR = 10'b0; DATAIN = 32'h0001;
      #20 WR_WL = 1'b0;
      #20 WR_BL = 1'b1; ADDR = 10'b0; DATAIN = 32'hFFFF;
      #20 WR_BL = 1'b0;
      #20 WE = 1; ADDR = 10'b0;
      #20 WE = 0; 
      #20 RE=1'b1; 
      #2000;
      #20 RE = 1'b0;
    end
    
     always begin
   		 #10 CLK = !CLK; 
  	 end
  	 always begin
  	     #11 CLK_ADC = !CLK_ADC;
   		 #9 CLK_ADC = !CLK_ADC;
  	 end
 
endmodule