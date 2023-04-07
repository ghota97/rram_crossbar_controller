`timescale 1ns / 1ps

// Code your testbench here
// or browse Examples

// Code your testbench here
// or browse Examples

module rram_controller_fsm_tb;
    parameter NUM_ADC = 32;
    parameter NUM_CORE = 4;
    parameter COREID = 1;
    parameter NUM_WL = 1024;
    parameter NUM_BL = 1024;
    parameter NUM_SL = 512;
    parameter INSTR_WIDTH = 4;
    parameter OPCODE_WIDTH = 16;
    parameter LOCAL_OPCODE_WIDTH = 18;
    parameter DATAIN_WIDTH = 64;
    parameter DATAOUT_WIDTH = 64;
    parameter NUM_HD_CLASSES = 32;
    parameter MAX_HD_SEGMENT_COL = 16;
    parameter ADC_WIDTH_THERM = 15;
    parameter ADC_WIDTH = 4;
    parameter PHD_ACC_WIDTH = 16;
    
    
    logic CLK;
    logic reset;
    logic CLK_ADC;
    logic CLK_WL;
    logic CLK_BL;
    logic CLK_ADCOUT;
    logic [$clog2(NUM_CORE)-1:0] CORE_SEL;
    
     //Instruction FIFO 
    logic pop_n_instFIFO;
    logic  empty_instFIFO;
    logic  [INSTR_WIDTH+OPCODE_WIDTH-1:0] dout_instFIFO;
    
    //input data FIFO
    logic pop_n_iFIFO;
    logic  empty_iFIFO;
    logic  [DATAIN_WIDTH-1:0] dout_iFIFO;
    
    //output data FIFO
    logic push_n_oFIFO;
    logic  full_oFIFO;
    logic [DATAIN_WIDTH-1:0] din_oFIFO;
    
    logic [NUM_WL-1:0] WL_SEL; //Gated WL_SEL fed into the Crossbar
    logic [2:0] BL_SEL [NUM_SL-1:0]; //Gated BL_SEL fed into the Crossbar (3b each), Controls for 2 adjacent BLs (BL+ and BL-) are shared.
    logic [2:0] SL_SEL [NUM_SL-1:0]; //Gated SL_SEL fed into the Crossbar (3b each)
    logic [NUM_SL-1:0] SL_MUX_SEL;   //MUX SEL to select between 16 adjacent SLs and pass one to the shared ADC.

    logic WL_BIAS_SEL; //Selects between WRITE and READ Bias
    logic [2:0] BL_BIAS_SEL; //Selects between WRITE and READ Bias for PLUS, MINUS and REF
    logic SL_BIAS_SEL; //Selects between WRITE and READ for REF

    logic [ADC_WIDTH_THERM-1:0] ADCOUT_THERM[NUM_ADC-1:0]; //32 4-b ADCs
    logic [ADC_WIDTH-1:0] ADCOUT[NUM_ADC-1:0]; //32 4-b ADCs

 rram_controller_fsm controller_fsm
    (
    .CLK(CLK),
    .reset(reset),
    .CLK_ADC(CLK_ADC),
    .CLK_WL(CLK_WL),
    .CLK_BL(CLK_BL),
    .CLK_ADCOUT(CLK_ADCOUT),
    .CORE_SEL(CORE_SEL),
    
     //Instruction FIFO 
    .pop_n_instFIFO(pop_n_instFIFO),
    .empty_instFIFO(empty_instFIFO),
    .dout_instFIFO(dout_instFIFO),
    
    //input data FIFO
    .pop_n_iFIFO(pop_n_iFIFO),
    .empty_iFIFO(empty_iFIFO),
    .dout_iFIFO(dout_iFIFO),
    
    //output data FIFO
    .push_n_oFIFO(push_n_oFIFO),
    .full_oFIFO(full_oFIFO),
    .din_oFIFO(din_oFIFO),
    
    
    .WL_SEL(WL_SEL), //Gated WL_SEL fed into the Crossbar
    .BL_SEL(BL_SEL), //Gated BL_SEL fed into the Crossbar (3b each), Controls for 2 adjacent BLs (BL+ and BL-) are shared.
    .SL_SEL(SL_SEL), //Gated SL_SEL fed into the Crossbar (3b each)
    .SL_MUX_SEL(SL_MUX_SEL),   //MUX SEL to select between 16 adjacent SLs and pass one to the shared ADC.

    .WL_BIAS_SEL(WL_BIAS_SEL), //Selects between WRITE and READ Bias
    .BL_BIAS_SEL(BL_BIAS_SEL), //Selects between WRITE and READ Bias for PLUS, MINUS and REF
    .SL_BIAS_SEL(SL_BIAS_SEL), //Selects between WRITE and READ for REF

    .ADCOUT_THERM(ADCOUT_THERM) //32 4-b ADCs
    );


    initial begin
      CLK = 1'b0;
      reset = 1'b1;
      CLK_BL = 1'b0;
      CLK_WL = 1'b0;
      CLK_ADC = 1'b0;
      //Instruction FIFO 
      empty_instFIFO <= 1'b1;
      dout_instFIFO <= 20'b0;
     
      //input data FIFO
      empty_iFIFO <= 1'b1;
      dout_iFIFO <= 64'b0;
  
      full_oFIFO <= 1'b1;
      
      #20 reset = 1'b0;
      #100; //Store instruction
      empty_instFIFO <= 1'b0;
      dout_instFIFO <= 20'h4_0000; //INSTR:-0100, ROW_ADDR=0, COL_ADDR=0, BURST_SIZE = 0
      empty_iFIFO <= 1'b0;
      dout_iFIFO <= 64'hABCD_ABCD;
      full_oFIFO <= 1'b1;
      
      #200;
      dout_instFIFO <= 20'h4_440A; //INSTR:-0100, ROW_ADDR=10, COL_ADDR=64*1, BURST_SIZE = 2
      dout_iFIFO <= 64'hEEEE_CCCC;
      #200;
      dout_iFIFO <= 64'hBABA_CABA;
      
      #35000;
      dout_instFIFO <= 20'h4_C6FF; //INSTR:-0100, ROW_ADDR=2FF(767), COL_ADDR=64*1, BURST_SIZE = 6
      dout_iFIFO <= 64'hEEEE_CCCC;
    
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
