`timescale 1ns / 1ps

// Code your testbench here
// or browse Examples

// Code your testbench here
// or browse Examples

module rram_controller_tb;
    parameter NUM_ADC = 32;
    parameter NUM_CORE = 4;
    parameter COREID = 1;
    parameter NUM_WL = 1024;
    parameter NUM_BL = 1024;
    parameter NUM_SL = 512;
    parameter INSTR_WIDTH = 4;
    parameter OPCODE_WIDTH = 18;
    parameter DATAIN_WIDTH = 64;
    parameter DATAOUT_WIDTH = 64;
    
    
    logic CLK;
    logic CLK_ADC;
    logic CLK_WL;
    logic CLK_BL;
    logic CLK_ADCOUT;
    logic [$clog2(NUM_CORE)-1:0] CORE_SEL;
    logic [INSTR_WIDTH-1:0] INSTR;
    logic [OPCODE_WIDTH-1:0] OPCODE;
    logic [DATAIN_WIDTH-1:0] DATAIN;
    logic valid_i;
    logic ready_i;
    
    logic  ready_o;
    logic  valid_o;
    
    logic [NUM_WL-1:0] WL_SEL; //Gated WL_SEL fed into the Crossbar
    logic [2:0] BL_SEL [NUM_SL-1:0]; //Gated BL_SEL fed into the Crossbar (3b each), Controls for 2 adjacent BLs (BL+ and BL-) are shared.
    logic [2:0] SL_SEL [NUM_SL-1:0]; //Gated SL_SEL fed into the Crossbar (3b each)
    logic [NUM_SL-1:0] SL_MUX_SEL;   //MUX SEL to select between 16 adjacent SLs and pass one to the shared ADC.

    logic WL_BIAS_SEL; //Selects between WRITE and READ Bias
    logic [2:0] BL_BIAS_SEL; //Selects between WRITE and READ Bias for PLUS, MINUS and REF
    logic SL_BIAS_SEL; //Selects between WRITE and READ for REF

    logic [3:0] ADCOUT[NUM_ADC-1:0]; //32 4-b ADCs
    logic [DATAOUT_WIDTH-1:0] DATAOUT; //Readout requested from ASIC

 rram_controller controller
    (
    .CLK(CLK),
    .CLK_ADC(CLK_ADC),
    .CLK_WL(CLK_WL),
    .CLK_BL(CLK_BL),
    .CLK_ADCOUT(CLK_ADCOUT),
    .CORE_SEL(CORE_SEL),
    .INSTR(INSTR),
    .OPCODE(OPCODE),
    .DATAIN(DATAIN),
    .valid_i(valid_i),
    .ready_i(ready_i),
    
    .ready_o(ready_o),
    .valid_o(valid_o),
    
    .WL_SEL(WL_SEL), //Gated WL_SEL fed into the Crossbar
    .BL_SEL(BL_SEL), //Gated BL_SEL fed into the Crossbar (3b each), Controls for 2 adjacent BLs (BL+ and BL-) are shared.
    .SL_SEL(SL_SEL), //Gated SL_SEL fed into the Crossbar (3b each)
    .SL_MUX_SEL(SL_MUX_SEL),   //MUX SEL to select between 16 adjacent SLs and pass one to the shared ADC.

    .WL_BIAS_SEL(WL_BIAS_SEL), //Selects between WRITE and READ Bias
    .BL_BIAS_SEL(BL_BIAS_SEL), //Selects between WRITE and READ Bias for PLUS, MINUS and REF
    .SL_BIAS_SEL(SL_BIAS_SEL), //Selects between WRITE and READ for REF

    .ADCOUT(ADCOUT), //32 4-b ADCs
    .DATAOUT(DATAOUT)
    );


    initial begin
      CLK = 1'b0;
      CLK_BL = 1'b0;
      CLK_WL = 1'b0;
      CLK_ADC = 1'b0;
      INSTR = 4'd9;  //RESET ALl Regs
      OPCODE = 18'd0;
      DATAIN = 64'd0;
      #100;
      INSTR = 4'd1; //Load weight register at Address 0
      OPCODE = 18'd0; //Address 0
      DATAIN = 64'hABCDEF;
      #100;
      INSTR = 4'd3; //Program Individual Weights, 
      OPCODE = 18'h000; //ROW_POL = 0, COL_POL = 0
      #20;
      OPCODE = 18'h400; //ROW_POL = 0, COL_POL = 1
      #20;
      OPCODE = 18'h800; //ROW_POL = 1, COL_POL = 0
      #20;
      OPCODE = 18'hc00; //ROW_POL = 1, COL_POL = 1
      #100;
      INSTR = 4'd5;  //Load input registers
      OPCODE = 18'd0; //Address 0
      DATAIN = 64'hABABCDEFABCDEF;
      #100;
      INSTR = 4'd6; //MVM Instruction
      OPCODE = 18'h3c00; //Segment Width = 16, Start Address = 0
      #100;
      INSTR = 4'd6; //MVM Instruction
      OPCODE = 18'h3c20; //Segment Width = 16, Start Address = 32
    end
    
     always begin
   		 #10 
   		 CLK = !CLK; 
   		 CLK_BL = !CLK_BL; 
   		 CLK_WL = !CLK_WL; 
  	 end
  	 always begin
  	     #11 CLK_ADC = !CLK_ADC;
   		 #9 CLK_ADC = !CLK_ADC;
  	 end
 
endmodule