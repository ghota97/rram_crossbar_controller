`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/24/2023 08:19:13 AM
// Design Name: 
// Module Name: top_RRAM
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
//"set_dont_touch = true"

module top_RRAM (ADC_CLK, ADC_CLKb, BL_Vref_sel09, BL_minus_sel09, BL_plus_sel09, 
                SL_MUX_sel09, SL_Vref_sel09, SL_minus_sel09, SL_plus_sel09, WL_in, VBLRp_power_sel, 
                VBLRp_power_selb, VBLRm_power_sel, VBLRm_power_selb, VBLR_ref_power_sel,
                VBLR_ref_power_selb, VBLWrp_power_sel, VBLWrp_power_selb, VBLWrm_power_sel,
                VBLWrm_power_selb, VBLWr_ref_power_sel, VBLWr_ref_power_selb, VSLRp_power_sel,
                VSLRp_power_selb, VSLRm_power_sel, VSLRm_power_selb, VSLR_ref_power_sel, VSLR_ref_power_selb,
                VSLWrp_power_sel, VSLWrp_power_selb, VSLWrm_power_sel, VSLWrm_power_selb,
                VSLWr_ref_power_sel, VSLWr_ref_power_selb, outp, outn);

            parameter NUM_ADC = 32;
            parameter NUM_WL = 1024;
            parameter NUM_BL = 1024;
            parameter NUM_SL = 512;
            parameter OUT_WIDTH = 15*NUM_ADC;
            parameter ADC_WIDTH = 4;
            
            input [(NUM_ADC/2)-1:0] ADC_CLK;
            input [(NUM_ADC/2)-1:0] ADC_CLKb;
            input [NUM_SL-1:0] BL_Vref_sel09;
            input [NUM_SL-1:0] BL_minus_sel09;
            input [NUM_SL-1:0] BL_plus_sel09;
            input [NUM_SL-1:0] SL_MUX_sel09;
            input [NUM_SL-1:0] SL_Vref_sel09;
            input [NUM_SL-1:0] SL_minus_sel09;
            input [NUM_SL-1:0] SL_plus_sel09;
            input [NUM_WL-1:0] WL_in;
            input VBLRp_power_sel;
            input VBLRp_power_selb;
            input VBLRm_power_sel;
            input VBLRm_power_selb;
            input VBLR_ref_power_sel;
            input VBLR_ref_power_selb;
            input VBLWrp_power_sel;
            input VBLWrp_power_selb;
            input VBLWrm_power_sel;
            input VBLWrm_power_selb;
            input VBLWr_ref_power_sel;
            input VBLWr_ref_power_selb;
            input VSLRp_power_sel;
            input VSLRp_power_selb;
            input VSLRm_power_sel;
            input VSLRm_power_selb;
            input VSLR_ref_power_sel;
            input VSLR_ref_power_selb;
            input VSLWrp_power_sel;
            input VSLWrp_power_selb;
            input VSLWrm_power_sel;
            input VSLWrm_power_selb;
            input VSLWr_ref_power_sel;
            input VSLWr_ref_power_selb;
            
            output [OUT_WIDTH-1:0] outp;
            output [OUT_WIDTH-1:0] outn;
   
endmodule