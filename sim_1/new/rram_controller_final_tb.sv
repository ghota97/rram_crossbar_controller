`timescale 1ns / 1ps

// Code your testbench here
// or browse Examples

// Code your testbench here
// or browse Examples

module rram_controller_final_tb;
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
    logic rst_n;
    logic [(NUM_WL/4)-1:0] CLK_WL;
    logic [(NUM_SL/4)-1:0] CLK_BL;
    
    logic [$clog2(NUM_CORE)-1:0] CORE_SEL;
    
     //Instruction FIFO 
    logic  pop_n_instFIFO_ext;
    logic  empty_instFIFO_ext;
    logic  [INSTR_WIDTH+OPCODE_WIDTH-1:0] dout_instFIFO_ext;
    
     //Instruction FIFO 
    logic  pop_n_instFIFO_hd;
    logic  empty_instFIFO_hd;
    logic  [INSTR_WIDTH+OPCODE_WIDTH-1:0] dout_instFIFO_hd;
    
    //input data FIFO
    logic pop_n_iFIFO_ext;
    logic  empty_iFIFO_ext;
    logic  [DATAIN_WIDTH-1:0] dout_iFIFO_ext;
    logic pop_n_iFIFO_hd;
    logic  empty_iFIFO_hd;
    logic  [DATAIN_WIDTH-1:0] dout_iFIFO_hd;
    
    //output data FIFO
    logic push_n_oFIFO_ext;
    logic  full_oFIFO_ext;
    logic [DATAIN_WIDTH-1:0] din_oFIFO_ext; 
    logic  push_n_oFIFO_hd;
    logic  full_oFIFO_hd;
    logic [DATAIN_WIDTH-1:0] din_oFIFO_hd;
    logic [NUM_WL-1:0] WL_SEL; //Gated WL_SEL fed into the Crossbar
    logic [NUM_WL-1:0] WL_SEL; //Gated WL_SEL fed into the Crossbar
    logic [NUM_SL-1:0] BLplus_SEL ; //Gated BL_SEL fed into the Crossbar (3b each), Controls for 2 adjacent BLs (BL+ and BL-) are shared.
    logic [NUM_SL-1:0] BLminus_SEL ;
    logic [NUM_SL-1:0] BLref_SEL ;
    logic [NUM_SL-1:0] SLplus_SEL ;
    logic [NUM_SL-1:0] SLminus_SEL ;
    logic [NUM_SL-1:0] SLref_SEL ;
    logic [NUM_SL-1:0] SL_MUX_SEL; 
    logic [NUM_SL-1:0] SL_MUX_SEL;   //MUX SEL to select between 16 adjacent SLs and pass one to the shared ADC.
    logic WL_BIAS_SEL; //Selects between WRITE and READ Bias
    logic BL_BIAS_SEL; //Selects between WRITE and READ Bias for PLUS, MINUS and REF
    logic SL_BIAS_SEL; //Selects between WRITE and READ for REF

    logic [(NUM_ADC*ADC_WIDTH_THERM)-1:0] ADCOUT_THERM; //32 4-b ADCs
    

 //rram_controller_final rram_controller(
 rram_controller_final  rram_controller(
            .CLK(CLK),
            .rst_n(rst_n),
            .CLK_WL(CLK_WL),
            .CLK_BL(CLK_BL),
            
            .CORE_SEL(CORE_SEL),
            
              //Instruction FIFO
            .pop_n_instFIFO_ext(pop_n_instFIFO_ext),
            .empty_instFIFO_ext(empty_instFIFO_ext),
            .dout_instFIFO_ext(dout_instFIFO_ext),
            .pop_n_instFIFO_hd(pop_n_instFIFO_hd),
            .empty_instFIFO_hd(empty_instFIFO_hd),
            .dout_instFIFO_hd(dout_instFIFO_hd),


            //input data FIFO
            .pop_n_iFIFO_ext(pop_n_iFIFO_ext),
            .empty_iFIFO_ext(empty_iFIFO_ext),
            .dout_iFIFO_ext(dout_iFIFO_ext),
            .pop_n_iFIFO_hd(pop_n_iFIFO_hd),
            .empty_iFIFO_hd(empty_iFIFO_hd),
            .dout_iFIFO_hd(dout_iFIFO_hd),

            //output data FIFO
            .push_n_oFIFO_ext(push_n_oFIFO_ext),
            .full_oFIFO_ext(full_oFIFO_ext),
            .din_oFIFO_ext(din_oFIFO_ext),
            .push_n_oFIFO_hd(push_n_oFIFO_hd),
            .full_oFIFO_hd(full_oFIFO_hd),

            .WL_SEL(WL_SEL), //Gated WL_SEL fed into the Crossbar
            .BLplus_SEL(BLplus_SEL), //Gated BL_SEL fed into the Crossbar (3b each), Controls for 2 adjacent BLs (BL+ and BL-) are shared.
            .BLminus_SEL(BLminus_SEL), //Gated SL_SEL fed into the Crossbar (3b each)
            .BLref_SEL(BLref_SEL), 
            .SLplus_SEL(SLplus_SEL),
            .SLminus_SEL(SLminus_SEL),
            .SLref_SEL(SLref_SEL),
            .SL_MUX_SEL(SL_MUX_SEL),   //MUX SEL to select between 16 adjacent SLs and pass one to the shared ADC.
        
            .WL_BIAS_SEL(WL_BIAS_SEL), //Selects between WRITE and READ Bias
            .BL_BIAS_SEL(BL_BIAS_SEL), //Selects between WRITE and READ Bias for PLUS, MINUS and REF
            .SL_BIAS_SEL(SL_BIAS_SEL), //Selects between WRITE and READ for REF

            .ADC_outp(ADCOUT_THERM) //32 4-b ADCs
    );


    initial begin
      CLK = 1'b0;
      rst_n = 1'b0;
      CLK_BL = 128'b0;
      CLK_WL = 256'b0;
      
      //Instruction FIFO 
      empty_instFIFO_ext = 1'b1;
      dout_instFIFO_ext = 20'b0;
      empty_instFIFO_hd = 1'b1;
      dout_instFIFO_hd = 20'b0;
      //input data FIFO
      empty_iFIFO_ext = 1'b1;
      dout_iFIFO_ext = 64'b0;
      empty_iFIFO_hd = 1'b1;
      dout_iFIFO_hd = 64'b0;
      full_oFIFO_ext = 1'b1;
      full_oFIFO_hd = 1'b1;
      
      //for (int i=0; i < NUM_ADC;i++) begin
      //  ADCOUT_THERM[i] = 15'b010101010101001; //Output from array
      //end
      ADCOUT_THERM = 480'hABCDABCDABCDABCDABCDABCDABCDABCDABCDABCDABCDABCDABCDABCDABCDABCD;
      #200 rst_n = 1'b1;
      #100; 
      /*
      //Store instruction
      empty_instFIFO_ext = 1'b0;
      dout_instFIFO_ext = 20'h4_0000; //INSTR:-0100, ROW_ADDR=0, COL_ADDR=0, BURST_SIZE = 0
      empty_iFIFO_ext = 1'b0;
      dout_iFIFO_ext = 64'hABCD_ABCD;
      full_oFIFO_ext = 1'b1;
      
      #200;
      dout_instFIFO_ext = 20'h4_440A; //INSTR:-0100, ROW_ADDR=10, COL_ADDR=64*1, BURST_SIZE = 2
      dout_iFIFO_ext = 64'hEEEE_CCCC;
      #200;
      dout_iFIFO_ext = 64'hBABA_CABA;
      
      #35000;
      dout_instFIFO_ext = 20'h4_C6FF; //INSTR:-0100, ROW_ADDR=2FF(767), COL_ADDR=64*1, BURST_SIZE = 6
      dout_iFIFO_ext = 64'hEEEE_CCCC;
      */
      
      //Store Hamming Weight instruction
      empty_instFIFO_ext <= 1'b0;
      dout_instFIFO_ext <= 20'h6_1000; //INSTR:-0110, ROW_ADDR=0, COL_ADDR=0, SEGMENT_WIDTH=16
      empty_iFIFO_ext <= 1'b0;
      dout_iFIFO_ext <= 64'hABCD_ABCD_ABCD_ABCD;
      full_oFIFO_ext <= 1'b1;
      #19000;
      empty_instFIFO_ext <= 1'b0;
      dout_instFIFO_ext <= 20'h6_0000; //INSTR:-0110, ROW_ADDR=0, COL_ADDR=0, SEGMENT_WIDTH=8
      empty_iFIFO_ext <= 1'b0;
      dout_iFIFO_ext <= 64'hABCD_ABCD_ABCD_ABCD;
      full_oFIFO_ext <= 1'b1;
      
      #1000;
      //READ instruction
      for (int i=0; i < NUM_ADC;i++) begin
        ADCOUT_THERM[i] = 15'b010101010101001; //Output from array
      end
      empty_instFIFO_ext <= 1'b0;
      dout_instFIFO_ext <= 20'h5_0000; //INSTR:-0101, ROW_ADDR=0, COL_ADDR=0, BURST_SIZE=0
      empty_iFIFO_ext <= 1'b0;
      dout_iFIFO_ext <= 64'hABCD_ABCD_ABCD_ABCD;
      full_oFIFO_ext <= 1'b0;
      
      //HAM_SEGMENT_COMPUTE instruction
      #1000;
      empty_instFIFO_ext <= 1'b0;
      dout_instFIFO_ext <= 20'h7_0600; //INSTR:-0111, rden(1b)=0, reset-acc(1b)=1, segment_size(1b) =1, col_burst(4b)=0, row_burst(2b)=0, row_addr(3b) = 0
      empty_iFIFO_ext <= 1'b0;
      dout_iFIFO_ext <= 64'hABCD_ABCD_ABCD_ABCD;
      full_oFIFO_ext <= 1'b0;
      
      #200;
      empty_instFIFO_ext <= 1'b0;
      dout_instFIFO_ext <= 20'h7_0A00; //INSTR:-0111, rden(1b)=1, reset-acc(1b)=0, segment_size(1b) =1, col_burst(4b)=0, row_burst(2b)=0, row_addr(3b) = 0
      empty_iFIFO_ext <= 1'b0;
      dout_iFIFO_ext <= 64'hABCD_ABCD_ABCD_ABCD;
      full_oFIFO_ext <= 1'b0;
      
    end
    
     always begin
   		 #10 
   		 CLK = ~CLK; 
   		 CLK_BL = ~CLK_BL; 
   		 CLK_WL = ~CLK_WL; 
  	 end
  	 /*always begin
  	     #11 CLK_ADC = !CLK_ADC;
  	     CLK_ADCOUT = !CLK_ADCOUT;
   		 #9 CLK_ADCOUT = !CLK_ADCOUT;
   		 
  	 end */
 
endmodule
