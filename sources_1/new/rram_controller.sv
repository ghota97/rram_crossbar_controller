`timescale 1ns / 1ps

//4b-instruction, 18-b OpCode, 64-b rram_controller

// INSTR (0000) : No OP
// INSTR (0001) : Write Weights, 9b (START ADDR for Weight Register), We can write 64 registers at once
// INSTR (0010) : Write COLSELb Register to program only the columns we want, OpCode:- 9b (ADDR for COLSEL Register), We can write 64 registers at once 
// INSTR (0011) : Program RRAM, OpCode:- 8b (Number of Cycles to Write) + 1b (ROW_POL) + 1b (COL_POL) + 10b (ADDR to write)
// INSTR (0100) : Read Single Device, OpCode:-  10b (ADDR to read)
// INSTR (0101) : Write Inputs (Load WL Registers),  10b (START ADDR for WL Register)
// INSTR (0110) : Read Multiple Devices i.e do MVM, OpCode:- 6b(Segment Width/Number of Rows to read) + 10b (START ADDR for WL)
// INSTR (0111) : Read ADCoutput, OpCode:- 9b (ADCout Address to Read)
// INSTR (1000):  MUX between SLs to pass to ADC, OpCode:- 4b (MUX_SEL)
// INSTR (1001) : RESET all registers, 

//We need analog biases being bypassed through rram_core_w_controller  

module rram_controller(CLK, CLK_ADC, CLK_WL, CLK_BL, CLK_ADCOUT, CORE_SEL, INSTR, OPCODE, DATAIN, valid_i, ready_i, valid_o, ready_o, WL_SEL, BL_SEL, SL_SEL, WL_BIAS_SEL, BL_BIAS_SEL, SL_BIAS_SEL, SL_MUX_SEL, ADCOUT, DATAOUT);
    
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
    
    
    input CLK;
    input CLK_ADC;
    input CLK_WL;
    input CLK_BL;
    input CLK_ADCOUT;
    //input [$clog2(NUM_CORE)-1:0] CORE_SEL;
    input [1:0] CORE_SEL;
    input [INSTR_WIDTH-1:0] INSTR;
    input [OPCODE_WIDTH-1:0] OPCODE;
    input [DATAIN_WIDTH-1:0] DATAIN;
    input valid_i;
    output reg ready_i;
    
    input  ready_o;
    output reg valid_o;
    
    output reg [NUM_WL-1:0] WL_SEL; //Gated WL_SEL fed into the Crossbar
    output reg [2:0] BL_SEL [NUM_SL-1:0]; //Gated BL_SEL fed into the Crossbar (3b each), Controls for 2 adjacent BLs (BL+ and BL-) are shared.
    output reg [2:0] SL_SEL [NUM_SL-1:0]; //Gated SL_SEL fed into the Crossbar (3b each)
    output reg [NUM_SL-1:0] SL_MUX_SEL;   //MUX SEL to select between 16 adjacent SLs and pass one to the shared ADC.

    output reg WL_BIAS_SEL; //Selects between WRITE (1) and READ (0) Bias
    output reg [2:0] BL_BIAS_SEL; //Selects between WRITE (1) and READ (0) Bias for PLUS, MINUS and REF
    output reg SL_BIAS_SEL; //Selects between WRITE (1) and READ (0) for REF

    input [3:0] ADCOUT[NUM_ADC-1:0]; //32 4-b ADCs
    output reg [DATAOUT_WIDTH-1:0] DATAOUT; //Readout requested from ASIC
    
    
    localparam [INSTR_WIDTH-1:0] CMD_NO_OP            = 4'd0;     // No OP
    localparam [INSTR_WIDTH-1:0] CMD_WRITE_WEIGHTS    = 4'd1;     // Write Weights, 10b (START ADDR for BL/SL Register), We can write 64 egisters at once
    localparam [INSTR_WIDTH-1:0] CMD_WRITE_COLSEL     = 4'd2;     // Write COLSELb Register to program only the columns we want, OpCode:- 10b (ADDR for COLSEL Register), We can write 64 inputs (128 WL Registers) at once 
    localparam [INSTR_WIDTH-1:0] CMD_PROGRAM_DEVICE   = 4'd3;     // Program RRAM, OpCode:- 8b (Number of Cycles to Write) + + 1b (ROW_POL) + 1b (COL_POL) + 10b (ADDR to write)
    localparam [INSTR_WIDTH-1:0] CMD_READ_SINGLE_ADDR = 4'd4;     // Read Single Device, OpCode:-  10b (ADDR to read)
    localparam [INSTR_WIDTH-1:0] CMD_WRITE_INPUTS     = 4'd5;     // Write Inputs (WL Registers),  10b (START ADDR for WL Register)
    localparam [INSTR_WIDTH-1:0] CMD_COMPUTE_MVM      = 4'd6;     // Read Multiple Devices i.e do MVM, OpCode:- 6b(Segment Width/Number of Rows to read, in multiple of 16) + 10b (START ADDR for WL)
    localparam [INSTR_WIDTH-1:0] CMD_READ_ADCOUT      = 4'd7;     // Read ADCoutput, OpCode:- 9b (ADCout Address to Read)
    localparam [INSTR_WIDTH-1:0] CMD_MUX_SEL          = 4'd8;     // MUX between SLs to pass to ADC, OpCode:- 4b (MUX_SEL)
    localparam [INSTR_WIDTH-1:0] CMD_RESET_REGS       = 4'd9;     // Reset All Registers
    
    //Row Controller
    
    //1st stage registering inputs
    reg [NUM_WL-1:0] WL_IN_REG; //Registered WL data coming from asic if the address is selected. NUM_WL/2 as we feed ternary inputs
    //2nd stage just combinational logic
    reg [NUM_WL-1:0] WL_UNGATED; //Ungated WL i.e. WL input before it's gated with CLK_WL.
    
    //Combinational Logic
    always_comb begin
        
        WL_SEL = (INSTR==CMD_COMPUTE_MVM)?(WL_UNGATED & {NUM_WL{CLK_WL}}):WL_UNGATED; //Only gate the Clock when MVM is done, Don't gate it when programming/reading
        
        //This default condition needs to be specified to avoid latches and result in clean synthesis.
        WL_UNGATED = {NUM_WL{1'b0}};  //None of the WL selected
        WL_BIAS_SEL = 1'b0;
        case (INSTR)
            CMD_NO_OP: begin
                WL_UNGATED = {NUM_WL{1'b0}};  //None of the WL selected
            end
            CMD_PROGRAM_DEVICE: begin //Controller needs to spend 4 cycles to program each weight, Row and Column Polarity exposed to ASIC Controller, increment ADDR by +2 after each weight programming
                 ////Written Assuming CMD_PROGRAM_DEVICE is active for OpCode[17;10] number of cycles, If ASIC doesn't time this, we can build a timer internally/ build a FSM
                 //If ROW_POL (OPCODE[11]) is 0, Write to the Positive Input Line (+1), ortherwise write to the negative Input line (-1)
                 WL_UNGATED = (OPCODE[11])?(1<<OPCODE[9:0]+1):(1<<OPCODE[9:0]);  //Programming the array, select one WL at a time to load weights, ROW_POL = OPCODE[11]
                 WL_BIAS_SEL = 1'b1;
            end
            CMD_READ_SINGLE_ADDR: begin
                WL_UNGATED = (1<<OPCODE[9:0]);  //Read the Array
                WL_BIAS_SEL = 1'b0;
            end
            CMD_RESET_REGS: begin
                WL_UNGATED = {NUM_WL{1'b0}}; //Reset WL_UNGATED
            end
            CMD_COMPUTE_MVM: begin
               //OpCode:- 6b(Segment Width/Number of Rows to read) + 10b (START ADDR for WL)
               if(OPCODE[15:10] == 6'd15) WL_UNGATED[OPCODE[9:0]+:32] = WL_IN_REG[OPCODE[9:0]+:32];  //load the inputs for the required HD distance computation
               else if (OPCODE[15:10] == 6'd31) WL_UNGATED[OPCODE[9:0]+:64] = WL_IN_REG[OPCODE[9:0]+:64];  //load the inputs for the required HD distance computation
               else if (OPCODE[15:10] == 6'd63) WL_UNGATED[OPCODE[9:0]+:128] = WL_IN_REG[OPCODE[9:0]+:128];  //load the inputs for the required HD distance computation
               
                WL_BIAS_SEL = 1'b0;
            end
            default:begin
                //Do Nothing
                WL_UNGATED = {NUM_WL{1'b0}};  //None of the WL selected
                WL_BIAS_SEL = 1'b0;
            end
        endcase
    end
    //Sequential Logic
    always @(posedge CLK) begin
        
        case (INSTR)
            
            CMD_WRITE_INPUTS: begin
                //Select the appropriate registers to load inputs, 128 WLs (64 inputs) can be selected at once, thus 7 lsbs are ignored
                for (int i=0; i<DATAIN_WIDTH;i++) begin //Loop, Load DATAIN at even locations, and ~DATAIN at odd locations
					WL_IN_REG[OPCODE[9:7]+2*i] <= DATAIN[i];
                    WL_IN_REG[OPCODE[9:7]+2*i+1] <= ~DATAIN[i];
		        end
            end
            
            CMD_RESET_REGS: begin
                WL_IN_REG  <= {NUM_WL{1'b0}}; //Reset WL_IN_REG
            end
            default:begin
                //Do Nothing
            end
        endcase
    end
    
    
    //Column Controller
    //1st stage registering weights
    reg [NUM_SL-1:0] WEIGHT_REG; //Registered Weights coming from asic if the address is selected.
    reg [NUM_SL-1:0] COLSELb; //Registered Weights coming from asic if the address is selected.
    
    //2nd stage just combinational logic (no registering now)
    reg [2:0] BL_UNGATED [NUM_SL-1:0]; //Gated BL_SEL fed into the Crossbar (3b each)
    reg [2:0] SL_UNGATED [NUM_SL-1:0]; //Gated SL_SEL fed into the Crossbar (3b each)
    
    //Combinational Logic
    always_comb begin
         for (int i=0; i<NUM_SL;i++) begin
                //BL Controls
				BL_SEL[i][0]= (INSTR==CMD_COMPUTE_MVM)?(BL_UNGATED[i][0] & {NUM_SL{CLK_BL}}):BL_UNGATED[i][0]; //BLplus SEL, //Only gate the Clock when MVM is done, Don't gate it when programming/reading
				BL_SEL[i][1]= (INSTR==CMD_COMPUTE_MVM)?(BL_UNGATED[i][1] & {NUM_SL{CLK_BL}}):BL_UNGATED[i][1]; //BLminus SEL, //Only gate the Clock when MVM is done, Don't gate it when programming/reading
				BL_SEL[i][2]= (INSTR==CMD_COMPUTE_MVM)?(BL_UNGATED[i][2] & {NUM_SL{CLK_BL}}):BL_UNGATED[i][2]; //BLref SEL  //Only gate the Clock when MVM is done, Don't gate it when programming/reading
				
				//SL Controls
				SL_SEL[i][0]= (INSTR==CMD_COMPUTE_MVM)?(SL_UNGATED[i][0] & {NUM_SL{CLK_BL}}):SL_UNGATED[i][0]; //SLplus SEL, //Only gate the Clock when MVM is done, Don't gate it when programming/reading
				SL_SEL[i][1]= (INSTR==CMD_COMPUTE_MVM)?(SL_UNGATED[i][1] & {NUM_SL{CLK_BL}}):SL_UNGATED[i][1]; //SLminus SEL, //Only gate the Clock when MVM is done, Don't gate it when programming/reading
				SL_SEL[i][2]= (INSTR==CMD_COMPUTE_MVM)?(SL_UNGATED[i][2] & {NUM_SL{CLK_BL}}):SL_UNGATED[i][2]; //SLref SEL //Only gate the Clock when MVM is done, Don't gate it when programming/reading
		 end
		 
		 //This default condition needs to be specified to avoid latches and result in clean synthesis.
		 for (int i=0; i<NUM_SL;i++) begin
                    BL_UNGATED[i] = 3'b100; //BL selected to reference
                    SL_UNGATED[i] = 3'b100; //SL selected to reference
         end
           
         SL_MUX_SEL = 512'b0; //Default MUXSEL
         DATAOUT = 64'b0; //Dafault DATAOUT
         BL_BIAS_SEL = 3'b000;   //READ     
         SL_BIAS_SEL = 1'b0;  //READ
		 case (INSTR) 
		     CMD_NO_OP: begin
                for (int i=0; i<NUM_SL;i++) begin
                    BL_UNGATED[i] = 3'b100; //BL selected to reference
                    SL_UNGATED[i] = 3'b100; //SL selected to reference
                end
             end
             CMD_PROGRAM_DEVICE: begin 
                     //Controller needs to spend 4 cycles to program each weight, Row and Column Polarity exposed to ASIC Controller, increment ADDR by +2 after each weight programming
                     //Written Assuming CMD_PROGRAM_DEVICE is active for OpCode[17;10] number of cycles, If ASIC doesn't time this, we can build a timer internally/ build a FSM
                     //If ROW_POL is 1, Write to the Positive Input Line (+1), otherwise write to the negative Input line (-1)
                     //When ROW_POL==1 (OPCODE[11]) and COL_POL(OPCODE[10])==1 or ROW_POL==0 and COL_POL==0, We write WEIGHT_REG[i]
                     //When ROW_POL==1 (OPCODE[11]) and COL_POL(OPCODE[10])==0 or ROW_POL==0 and COL_POL==1, We write ~WEIGHT_REG[i]
                     for (int i=0; i<NUM_SL;i++) begin
                        if(~COLSELb[i])
                            if ((OPCODE[11]==1 && OPCODE[10]==1) || (OPCODE[11]==0 & OPCODE[10]==0)) begin
                                BL_UNGATED[i] = {1'b0, ~WEIGHT_REG[i], WEIGHT_REG[i]}; //BL selected to VBL+ if WEIGHT=1, BL selected to VBL- if WEIGHT=0
                                SL_UNGATED[i] = {1'b0, WEIGHT_REG[i], ~WEIGHT_REG[i]}; //SL selected to VBL- if WEIGHT=1, SL selected to VBL+ if WEIGHT=0
                            end else if ((OPCODE[11]==0 && OPCODE[10]==1) || (OPCODE[11]==1 & OPCODE[10]==0)) begin
                                BL_UNGATED[i] = {1'b0, WEIGHT_REG[i], ~WEIGHT_REG[i]}; //BL selected to VBL- if WEIGHT=1, BL selected to VBL+ if WEIGHT=0
                                SL_UNGATED[i] = {1'b0, ~WEIGHT_REG[i], WEIGHT_REG[i]}; //SL selected to VBL+ if WEIGHT=1, SL selected to VBL- if WEIGHT=0
                            end
                        else begin
                            BL_UNGATED[i] = 3'b100; //BL selected to Vref
                            SL_UNGATED[i] = 3'b100; //SL selected to Vref
                        end
                    end
                    BL_BIAS_SEL = 3'b111;   //WRITE    
                    SL_BIAS_SEL = 1'b1;  //WRITE
                end
                CMD_READ_SINGLE_ADDR: begin
                    for (int i=0; i<NUM_SL;i++) begin
                        BL_UNGATED[i] = 3'b001; //BL selected to VBL+, BL selected to VBL-
                        SL_UNGATED[i] = 3'b000; //SL Left floating and the parasitic cap gets charged.
                    end
                    BL_BIAS_SEL = 3'b000;   //READ     
                    SL_BIAS_SEL = 1'b0;  //READ
                end
                CMD_COMPUTE_MVM: begin
                   for (int i=0; i<NUM_SL;i++) begin
                        BL_UNGATED[i] = 3'b001; //BL selected to VBL+, BL selected to VBL-
                        SL_UNGATED[i] = 3'b000; //SL Left floating and the parasitic cap gets charged.
                    end
                    BL_BIAS_SEL = 3'b000;   //READ     
                    SL_BIAS_SEL = 1'b0;  //READ
                end
                CMD_MUX_SEL: begin
                    //Don't add any register for MUX_SEL since we want to MUX it on the same clock edge
                    for (int i=0; i < NUM_ADC;i++) begin
                        SL_MUX_SEL[i*(NUM_SL/NUM_ADC)+:16] = (1<<OPCODE[3:0]); //OPCODE[3:0] is the MUXSEL, (NUM_SL/NUM_ADC)=16
                    end
                end
                CMD_READ_ADCOUT: begin //16 registers can be read at once.
                    for (int i=0; i<16;i++) begin
                        DATAOUT[4*i+:4] = ADCOUT_reg[OPCODE[8:4]+i]; //Column Address = OPCODE[8:4]+i
                    end
                end
                CMD_RESET_REGS: begin
                    for (int i=0; i<NUM_SL;i++) begin
                        BL_UNGATED[i] = 3'b100; //BL to Vref
                        SL_UNGATED[i] = 3'b100; //SL to Vref
                    end
                end
                default:begin
                //Do Nothing
                     for (int i=0; i<NUM_SL;i++) begin
                        BL_UNGATED[i] = 3'b100; //BL selected to reference
                        SL_UNGATED[i] = 3'b100; //SL selected to reference
                    end
                    BL_BIAS_SEL = 3'b000;   //READ     
                    SL_BIAS_SEL = 1'b0;  //READ
                end
          endcase
    end
    
    //Sequential Logic
    always @(posedge CLK) begin
        
        case (INSTR)
            
            CMD_WRITE_WEIGHTS: begin
                //Select the appropriate registers to load inputs, 64 Weights can be selected at once, thus 6 lsbs are ignored
                WEIGHT_REG[OPCODE[8:6]+:DATAIN_WIDTH] <= DATAIN; 
            end
            CMD_WRITE_COLSEL: begin
                //Select the appropriate registers to load inputs, 64 Weights can be selected at once, thus 6 lsbs are ignored
                COLSELb[OPCODE[8:6]+:DATAIN_WIDTH] <= DATAIN; 
            end
            CMD_RESET_REGS: begin
                for (int i=0; i<NUM_SL;i++) begin
                    WEIGHT_REG[i] <= 1'b0;
                    COLSELb[i] <= 1'b0;
                end
            end
            default:begin
                //Do Nothing
            end
        endcase
    end
    
    reg [3:0] ADCOUT_reg [NUM_SL-1:0];
    always @(posedge CLK_ADCOUT) begin
        if (INSTR==CMD_MUX_SEL) begin
            for (int i=0; i < NUM_ADC;i++) begin
                  ADCOUT_reg[i*16+OPCODE[3:0]] <= ADCOUT[i*16+OPCODE[3:0]]; //OPCODE[3:0] is the MUXSEL, (NUM_SL/NUM_ADC) = 16
            end
        end
    end
    
    
endmodule
