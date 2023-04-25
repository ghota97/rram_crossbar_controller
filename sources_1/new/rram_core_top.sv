`timescale 1ns / 1ps

module rram_core_top(CLK, reset, CLK_WL, CLK_BL, CORE_SEL, pop_n_instFIFO_ext, empty_instFIFO_ext, dout_instFIFO_ext, pop_n_instFIFO_hd, empty_instFIFO_hd, dout_instFIFO_hd, pop_n_iFIFO_ext, empty_iFIFO_ext, dout_iFIFO_ext, pop_n_iFIFO_hd, empty_iFIFO_hd, dout_iFIFO_hd, push_n_oFIFO_ext, full_oFIFO_ext, din_oFIFO_ext, push_n_oFIFO_hd, full_oFIFO_hd, din_oFIFO_hd);
   
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
    
    input CLK;
    input reset;
    input [NUM_WL-1:0] CLK_WL;
    input [NUM_SL-1:0] CLK_BL;
    //input [$clog2(NUM_CORE)-1:0] CORE_SEL;
    input [1:0] CORE_SEL;
    
    //Instruction FIFO 
    output reg pop_n_instFIFO_ext;
    input  empty_instFIFO_ext;
    input  [INSTR_WIDTH+OPCODE_WIDTH-1:0] dout_instFIFO_ext;
    
    output reg pop_n_instFIFO_hd;
    input  empty_instFIFO_hd;
    input  [INSTR_WIDTH+OPCODE_WIDTH-1:0] dout_instFIFO_hd;
    
    
    //input data FIFO
    output reg pop_n_iFIFO_ext;
    input  empty_iFIFO_ext;
    input  [DATAIN_WIDTH-1:0] dout_iFIFO_ext;
    
    output reg pop_n_iFIFO_hd;
    input  empty_iFIFO_hd;
    input  [DATAIN_WIDTH-1:0] dout_iFIFO_hd;
    
    
    //output data FIFO
    output reg push_n_oFIFO_ext;
    input  full_oFIFO_ext;
    output reg [DATAIN_WIDTH-1:0] din_oFIFO_ext;
    
    output reg push_n_oFIFO_hd;
    input  full_oFIFO_hd;
    output reg [DATAIN_WIDTH-1:0] din_oFIFO_hd;
    
    wire [NUM_WL-1:0] WL_SEL; //Gated WL_SEL fed into the Crossbar
    
    wire [NUM_SL-1:0] BLplus_SEL; //Gated BL_SEL fed into the Crossbar (3b each), Controls for 2 adjacent BLs (BL+ and BL-) are shared.
    wire [NUM_SL-1:0] BLminus_SEL;
    wire [NUM_SL-1:0] BLref_SEL ;
    wire [NUM_SL-1:0] SLplus_SEL ; //Gated SL_SEL fed into the Crossbar (3b each)
    wire [NUM_SL-1:0] SLminus_SEL ;
    wire [NUM_SL-1:0] SLref_SEL ;
    
    wire [NUM_SL-1:0] SL_MUX_SEL;   //MUX SEL to select between 16 adjacent SLs and pass one to the shared ADC.
    
    wire WL_BIAS_SEL; //Selects between WRITE and READ Bias
    wire BL_BIAS_SEL; //Selects between WRITE and READ Bias for PLUS, MINUS and REF
    wire SL_BIAS_SEL; //Selects between WRITE and READ for REF
    
    reg [ADC_WIDTH_THERM-1:0] ADCOUT_THERM[NUM_ADC-1:0]; //32 4-b ADCs
    
   
    
    rram_controller_final rram_controller(
            .CLK(CLK),
            .reset(reset),
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
            .BL_SEL(BL_SEL), //Gated BL_SEL fed into the Crossbar (3b each), Controls for 2 adjacent BLs (BL+ and BL-) are shared.
            .SL_SEL(SL_SEL), //Gated SL_SEL fed into the Crossbar (3b each)
            .SL_MUX_SEL(SL_MUX_SEL),   //MUX SEL to select between 16 adjacent SLs and pass one to the shared ADC.
        
            .WL_BIAS_SEL(WL_BIAS_SEL), //Selects between WRITE and READ Bias
            .BL_BIAS_SEL(BL_BIAS_SEL), //Selects between WRITE and READ Bias for PLUS, MINUS and REF
            .SL_BIAS_SEL(SL_BIAS_SEL), //Selects between WRITE and READ for REF

            .ADCOUT_THERM(ADCOUT_THERM) //32 4-b ADCs
    );
    //Power ports in the top_RRAM are VDDH, VDDHA2, VDDL, GND, VBLR+, VBLR-, VBLR_ref, VBLWr+, VBLWr-, VBLWr_ref, VSLR+, VSLR-, VSLR_ref, VSLWr+, VSLWr-, VSLWr_ref, gp, Vgn, COMP_REF+, COMP_REF-, Vref_comp<0:14>
    wire [ADC_WIDTH_THERM*NUM_ADC-1:0] outp;
    wire [ADC_WIDTH_THERM*NUM_ADC-1:0] outn;
    
    reg [NUM_ADC/2-1:0] ADC_CLK;
    reg [NUM_ADC/2-1:0] ADC_CLKb;
    
    top_RRAM anamacro(
            .ADC_CLK(ADC_CLK),   //Dynamic comparator clock
            .ADC_CLKb(ADC_CLKb),
            .BL_Vref_sel09(BLref_SEL),
            .BL_minus_sel09(BLminus_SEL),
            .BL_plus_sel09(BLplus_SEL),
            .SL_MUX_sel09(SL_MUX_SEL),
            .SL_Vref_sel09(SLref_SEL),
            .SL_minus_sel09(SLminus_SEL),
            .SL_plus_sel09(SLplus_SEL),
            .WL_in(WL_SEL),
            .VBLRp_power_sel(~BL_BIAS_SEL) ,  //WhenBL_BIAS_SEL is 0, READ selected, when it's 1, WRITE selected.
            .VBLRp_power_selb(BL_BIAS_SEL) , 
            .VBLRm_power_sel(~BL_BIAS_SEL) ,
            .VBLRm_power_selb(BL_BIAS_SEL) ,
            .VBLR_ref_power_sel(~BL_BIAS_SEL) ,
            .VBLR_ref_power_selb(BL_BIAS_SEL) ,
            .VBLWrp_power_sel(BL_BIAS_SEL) ,
            .VBLWrp_power_selb(~BL_BIAS_SEL) ,
            .VBLWrm_power_sel(BL_BIAS_SEL) ,
            .VBLWrm_power_selb(~BL_BIAS_SEL) ,
            .VBLWr_ref_power_sel(BL_BIAS_SEL) ,
            .VBLWr_ref_power_selb(~BL_BIAS_SEL) ,
            .VSLRp_power_sel(~SL_BIAS_SEL) ,
            .VSLRp_power_selb(SL_BIAS_SEL) ,
            .VSLRm_power_sel(~SL_BIAS_SEL) ,
            .VSLRm_power_selb(SL_BIAS_SEL) ,
            .VSLR_ref_power_sel(~SL_BIAS_SEL) ,
            .VSLR_ref_power_selb(SL_BIAS_SEL) ,
            .VSLWrp_power_sel(SL_BIAS_SEL) ,
            .VSLWrp_power_selb(~SL_BIAS_SEL) ,
            .VSLWrm_power_sel(SL_BIAS_SEL) ,
            .VSLWrm_power_selb(~SL_BIAS_SEL) ,
            .VSLWr_ref_power_sel(SL_BIAS_SEL) ,
            .VSLWr_ref_power_selb(~SL_BIAS_SEL),
            .outp(outp), 
            .outn(outn)
    );
    
    always_comb begin
           for (int i=0;i<NUM_ADC/2;i++) begin
               ADC_CLK[i] = ~CLK_BL[i*NUM_ADC];
               ADC_CLKb[i] = CLK_BL[i*NUM_ADC];
           end
    end
    
    always_comb begin
           for (int i=0;i<NUM_ADC;i++) begin
               ADCOUT_THERM[i] = outp[i*ADC_WIDTH_THERM+:15];
           end
    
    end
    
endmodule


