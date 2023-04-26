`timescale 1ns / 1ps

//High-level instructions and OpCode coming from instrFIFO or HD Controller 
//These instructions will be used to transition between FSMs and then issue low-level instructions to rram_controller

// INSTR (0000) : STORE RRAM, {burst-size-3b}_{column_addr-3b}_{row_addr-10b}
// INSTR (0001) : READ_RRAM, {burst-size-3b}_{column_addr-3b}_{row_addr-10b}
// INSTR (0010) : STORE_HAM_WEIGHT, {num_write_cycles_3b}_{segment_width-1b}_{col_addr-3b}_{row_addr-9b)
// INSTR (0011) : HAM_SEG_COMPUTE, {rd_en-1b}_{reset_acc-1b}, {segment_width-1b}_{col_burst_size-4b}_{row_burst_sel-2b}_{row_addr-3b}


module rram_controller_final(CLK, reset, CLK_WL, CLK_BL, CORE_SEL, pop_n_instFIFO_ext, empty_instFIFO_ext, dout_instFIFO_ext, pop_n_instFIFO_hd, empty_instFIFO_hd, dout_instFIFO_hd, pop_n_iFIFO_ext, empty_iFIFO_ext, dout_iFIFO_ext, pop_n_iFIFO_hd, empty_iFIFO_hd, dout_iFIFO_hd, push_n_oFIFO_ext, full_oFIFO_ext, din_oFIFO_ext, push_n_oFIFO_hd, full_oFIFO_hd, din_oFIFO_hd, WL_SEL, BLplus_SEL, BLminus_SEL, BLref_SEL, SLplus_SEL, SLminus_SEL, SLref_SEL, SL_MUX_SEL, WL_BIAS_SEL, BL_BIAS_SEL, SL_BIAS_SEL, ADCOUT_THERM);
   
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
    input CLK_WL;
    input CLK_BL;
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
    
    output reg [NUM_WL-1:0] WL_SEL; //Gated WL_SEL fed into the Crossbar
    output reg [NUM_SL-1:0] BLplus_SEL ; //Gated BL_SEL fed into the Crossbar (3b each), Controls for 2 adjacent BLs (BL+ and BL-) are shared.
    output reg [NUM_SL-1:0] BLminus_SEL ;
    output reg [NUM_SL-1:0] BLref_SEL ;
    output reg [NUM_SL-1:0] SLplus_SEL ;
    output reg [NUM_SL-1:0] SLminus_SEL ;
    output reg [NUM_SL-1:0] SLref_SEL ;
    output reg [NUM_SL-1:0] SL_MUX_SEL;   //MUX SEL to select between 16 adjacent SLs and pass one to the shared ADC.
    
    output reg WL_BIAS_SEL; //Selects between WRITE and READ Bias
    output reg BL_BIAS_SEL; //Selects between WRITE and READ Bias for PLUS, MINUS and REF
    output reg SL_BIAS_SEL; //Selects between WRITE and READ for REF
    
    input [ADC_WIDTH_THERM-1:0] ADCOUT_THERM[NUM_ADC-1:0]; //32 4-b ADCs
    reg [ADC_WIDTH-1:0] ADCOUT[NUM_ADC-1:0]; //32 4-b ADCs
    reg [ADC_WIDTH-1:0] ADCOUT_reg [NUM_SL-1:0];
    
    
    wire [INSTR_WIDTH-1:0] INSTR;
    wire [OPCODE_WIDTH-1:0] OPCODE;
    reg [DATAIN_WIDTH-1:0] DATAIN;
    
    reg [NUM_WL-1:0] WL_SEL_prereg; //Gated WL_SEL to be fed into the Crossbar
    reg [2:0] BL_SEL_prereg [NUM_SL-1:0]; //Gated BL_SEL to be fed into the Crossbar (3b each), Controls for 2 adjacent BLs (BL+ and BL-) are shared.
    reg [2:0] SL_SEL_prereg [NUM_SL-1:0]; //Gated SL_SEL to be fed into the Crossbar (3b each)
    reg [NUM_SL-1:0] SL_MUX_SEL_prereg;   //MUX SEL to select between 16 adjacent SLs and pass one to the shared ADC.
    
    reg WL_BIAS_SEL_prereg; //Selects between WRITE and READ Bias
    reg BL_BIAS_SEL_prereg; //Selects between WRITE and READ Bias for PLUS, MINUS and REF
    reg SL_BIAS_SEL_prereg; //Selects between WRITE and READ for REF
    
    //Sequential Logic, Registered WL_SEL, BL_SEL, SL_SEL
    always @(posedge CLK) begin
         WL_BIAS_SEL <= WL_BIAS_SEL_prereg;
         BL_BIAS_SEL <= WL_BIAS_SEL_prereg;
         SL_BIAS_SEL <= SL_BIAS_SEL_prereg;
         for (int i=0; i<NUM_WL;i++) begin
            WL_SEL[i] <= WL_SEL_prereg[i];
         end
         for (int i=0; i<NUM_SL;i++) begin
                //BL Controls
				BLplus_SEL[i] <= BL_SEL_prereg[i][0];
				BLminus_SEL[i] <= BL_SEL_prereg[i][1];
				BLref_SEL[i] <= BL_SEL_prereg[i][2];
				
				//SL Controls
				SLplus_SEL[i] <= SL_SEL_prereg[i][0];
				SLminus_SEL[i] <= SL_SEL_prereg[i][1];
				SLref_SEL[i] <= SL_SEL_prereg[i][2];
				SL_MUX_SEL[i] <= SL_MUX_SEL_prereg[i];
		 end
    end
    
    localparam [INSTR_WIDTH-1:0] INSTR_STORE_RRAM = 4'b0100;
    localparam [INSTR_WIDTH-1:0] INSTR_READ_RRAM = 4'b0101;
    localparam [INSTR_WIDTH-1:0] INSTR_STORE_HAM_WEIGHT = 4'b0110;
    localparam [INSTR_WIDTH-1:0] INSTR_HAM_SEGMENT_COMPUTE = 4'b0111;
    
    assign INSTR = (~empty_instFIFO_ext)?dout_instFIFO_ext[INSTR_WIDTH+OPCODE_WIDTH-1-:INSTR_WIDTH]:dout_instFIFO_hd[INSTR_WIDTH+OPCODE_WIDTH-1-:INSTR_WIDTH];
    assign OPCODE = (~empty_instFIFO_ext)?dout_instFIFO_ext[OPCODE_WIDTH-1:0]:dout_instFIFO_hd[OPCODE_WIDTH-1:0];
                   
    //Instructions for Specific executions 
    
    
    // INSTR (0000) : STORE RRAM, {burst_size-3b, col_addr-3b, row_addr-9b} ##burst_size (x):- how many columns we want to write (64x), col_addr:- base addr for column
    // INSTR (0001) : READ_RRAM, {burst_size-3b, col_addr-3b, row_addr-9b}
    // INSTR (0010) : STORE_HAM_WEIGHT, {segment_width-1b}_{col_addr-3b}_{row_addr-9b}
    // INSTR (0011) : HAM_SEG_COMPUTE, {segment_width-1b}_{col_burst_size-3b}_{row_addr-9b}
    
    localparam [3:0] STATE_RESET = 4'd0;
    localparam [3:0] STATE_IDLE = 4'd1;
    localparam [3:0] STATE_STORE_RRAM = 4'd2;
    localparam [3:0] STATE_READ_RRAM = 4'd3;
    localparam [3:0] STATE_STORE_HAM_WEIGHT = 4'd4; 
    localparam [3:0] STATE_HAM_SEGMENT_COMPUTE = 4'd5;
    
    reg [3:0] curr_state; //FSM STATE
   
    //Counters to keep track of states
    reg [8:0] counter_fsm; //Now 10b, previously 16bit 
    reg [8:0] wr_cycle_ctr; //counter to keep track of NUM_WRITE_CYCLEs in STORE instructions
    
    
    //Separate Counters and done signals needed when multiple FSMs operating in parallel, here we didn't perform any pipelining.
    //reg [9:0] ctr_store_weight, ctr_read_single_addr, ctr_store_ham_weight, ctr_compute_hd, ctr_hd_acc_rdout; 
    //reg done_store_weight, done_read_single_addr, done_store_ham_weight, done_compute_hd, done_hd_acc_rdout;
   
    wire [7:0] NUM_STORE_WEIGHTS_CYCLES; //number of cycles to STORE data into the WEIGHT_REGISTER and COLSELb_REGISTER, and finish writing to RRAM
    wire [6:0] NUM_READ_SINGLE_ADDR_CYCLES; //number of read cycles to readout one row of data from RRAM and quantizing over ADC
    wire [8:0] NUM_STORE_HAM_WEIGHT_CYCLES; //number of cycles to STORE hamming weights into the WEIGHT_REGISTER and COLSELb_REGISTER
    wire [6:0] NUM_COMPUTE_HD_CYCLES; //Number of cycles to compute per instructions, this depends on number of columns to be selected
    wire [6:0] NUM_READOUT_PHD_CYCLES; //Number of cycles to readout partial hamming distance
    
    reg [2:0] COL_BASE_ADDR; //3b Col base address for storing, reading, and storing HD weights
    reg [2:0] COL_BURST_SIZE_WR_RD; //Number of Columns we want to compute over
    reg [4:0] SEGMENT_WIDTH; //Denoting whether we are filling 8 columns for PHD calculation or 16 columns
    reg [9:0] ROW_ADDR_WR_RD; //10b row address for single row write/read
    reg [8:0] ROW_ADDR_HAM_WEIGHT_STORE; //9b row addresss to write hamming distance weights, since each input bit is mapped to 2 differential lines
    reg [8:0] NUM_WRITE_CYCLES; //Number of WRITE cycles needed for RRAM
    reg [2:0] ROW_ADDR_HAM_COMPUTE; //Base Row address for starting the HD Computation
    reg [2:0] ROW_BURST_SIZE_COMPUTE; //Number of rows we want to activate at the same time (16 inputs/32 inputs/64 inputs)
    reg [3:0] COL_BURST_SIZE_COMPUTE; //Number of Columns we want to compute over
    reg RESET_ACC; //reset the accumulator when starting a new PHD calculation
    reg PHD_READOUT_EN; //Whenever HD sends an instruction to readout the accumulated partial hamming distance
   
    //assign NUM_CYCLES based on the instructions
    assign NUM_STORE_WEIGHTS_CYCLES = (COL_BURST_SIZE_WR_RD+1) + 2*(COL_BURST_SIZE_WR_RD+1); //First we load 64b, send 2 256-cycle WRITE each for BL+ and BL- device
    assign NUM_READ_SINGLE_ADDR_CYCLES = (NUM_SL/NUM_ADC) + ADC_WIDTH*(COL_BURST_SIZE_WR_RD+1); //16 cycles to readout all columns, 4b output from each column.
    
    //We always load for 32 classes, takes 8 cycles when segment_width=16, taken 4 otherwise, It takes 4 WR cycles to write one weight (Double-differential encoding) 
    assign NUM_STORE_HAM_WEIGHT_CYCLES = (NUM_HD_CLASSES*SEGMENT_WIDTH)/DATAOUT_WIDTH + 4*((NUM_HD_CLASSES*SEGMENT_WIDTH)/DATAOUT_WIDTH); 
    
    assign NUM_COMPUTE_HD_CYCLES = (COL_BURST_SIZE_COMPUTE+1); //1 Cycle for writing input //If ROW_BURST_SIZE_COMPUTE=0, we compute only 16 rows, ROW_BURST_SIZE_COMPUTE=1, we compute only 32 rows, ROW_BURST_SIZE_COMPUTE=2, we compute 64 rows at once
    reg [3:0] row_burst_ctr; //Counter to count upto (1+2*ROW_BURST_SIZE_COMPUTE)
    assign NUM_READOUT_PHD_CYCLES = PHD_READOUT_EN*(NUM_HD_CLASSES*PHD_ACC_WIDTH)/DATAOUT_WIDTH; //Accumulated result is 16b per class
    
    
    logic [INSTR_WIDTH-1:0] LOCAL_INSTR;
    //logic [LOCAL_OPCODE_WIDTH-1:0] LOCAL_OPCODE;
    //logic [DATAIN_WIDTH-1:0] LOCAL_DATAIN;
    
    //Replacing the OpCodes from rram_controller to local registers/variables
    // LOCAL_INSTR (0001) : Write Weights, 9b (START ADDR for Weight Register), We can write 64 registers at once, only those selected and rest COLSELb HIGH.
    // LOCAL_INSTR (0011) : Program RRAM, OpCode:- 1b (ROW_POL) + 1b (COL_POL) + 10b (ADDR to write)
    // LOCAL_INSTR (0100) : Read Single Device, OpCode:-  10b (ADDR to read)
    // LOCAL_INSTR (0101) : Write Inputs (Load WL Registers),  10b (START ADDR for WL Register)
    // LOCAL_INSTR (0110) : Read Multiple Devices i.e do MVM, OpCode:- 6b(Segment Width/Number of Rows to read) + 10b (START ADDR for WL)
    // LOCAL_INSTR (0111) : Read ADCoutput, OpCode:- 9b (ADCout Address to Read)
    // LOCAL_INSTR (1000) : MUX between SLs to pass to ADC, OpCode:- 4b (MUX_SEL)
    // LOCAL_INSTR (1001) : RESET all registers, 
    
    localparam [INSTR_WIDTH-1:0] CMD_WRITE_WEIGHTS     = 4'd0;     // Write Weights, 10b (START ADDR for BL/SL Register), We can write 64 egisters at once
    reg [8:0] START_ADDR_SL; //Start address for WEIGHTS_REG, LOCAL_OPCODE[8:0]
    localparam [INSTR_WIDTH-1:0] CMD_PROGRAM_DEVICE    = 4'd1;   // Program RRAM, OpCode:- 1b (ROW_POL) + 1b (COL_POL) + 10b (ADDR to write)
    reg [9:0] ROW_ADDR; //LOCAL_OPCODE[9:0]
    reg ROW_POL; //LOCAL_OPCODE[10] // If ROW_POL (OPCODE[11]) is 0, Write to the Positive Input Line (+1), ortherwise write to the negative Input line (-1)
    reg COL_POL; //LOCAL_OPCODE[11] // If ROW_POL (OPCODE[11]) is 0, Write to the BL+ , ortherwise write to BL-
    localparam [INSTR_WIDTH-1:0] CMD_READ_SINGLE_ADDR  = 4'd2;   // Read Single Device, OpCode:-  10b (ADDR to read), Need extra 4b for MUX_SEL 
    reg [3:0] COL_OFFSET; //LOCAL_OPCODE[13:10] //COL OFFSET Address to readout from, could be anywhere in between 1 to 16.
	localparam [INSTR_WIDTH-1:0] CMD_READ_ADC_OUT      = 4'd3;   // Read ADCoutput, OpCode:- 9b (ADCout Address to Read)
	reg [4:0] READOUT_OFFSET_ADDR; //Offset address for ADCout, we can read 16 columns (16x4b) in one cycle
	localparam [INSTR_WIDTH-1:0] CMD_WRITE_HAM_WEIGHTS = 4'd4;   // Write HAM Weights, We can write 64 registers at once but addresses can be interleaved based on col segment size
    localparam [INSTR_WIDTH-1:0] CMD_WRITE_INPUTS      = 4'd5;   // Write Inputs (WL Registers),  10b (START ADDR for WL Register)
    localparam [INSTR_WIDTH-1:0] CMD_COMPUTE_MVM       = 4'd6;   // Read Multiple Devices i.e do MVM, OpCode:- 6b(Segment Width/Number of Rows to read, in multiple of 16) + 10b (START ADDR for WL)
    reg [8:0] ROW_ADDR_MVM;
	reg [5:0]  NUM_ROWS_SEGMENT; //Whether to select between 16 rows, 32 rows and 64 rows in one computation
	localparam [INSTR_WIDTH-1:0] CMD_READ_PHDACC_OUT   = 4'd7;   // Read ADCoutput, OpCode:- 9b (ADCout Address to Read)
	reg [5:0] BASE_CLASS_ADDR;
	localparam [INSTR_WIDTH-1:0] CMD_RESET_REGS        = 4'd8;   // Reset All Registers
	localparam [INSTR_WIDTH-1:0] CMD_NO_OP             = 4'd9;   // Reset All Registers
    
	
	reg [PHD_ACC_WIDTH-1:0] PHD_ACC [0:NUM_HD_CLASSES-1];
    //Row Controller
    
    //1st stage registering inputs
    reg [NUM_WL-1:0] WL_IN_REG; //Registered WL data coming from asic if the address is selected. NUM_WL/2 as we feed ternary inputs
    //2nd stage just combinational logic
    reg [NUM_WL-1:0] WL_UNGATED; //Ungated WL i.e. WL input before it's gated with CLK_WL.
    
    reg ext_hd_sel_fifo; //If ext fifo selected this is 0, for HD it's 1.
    
    always @(posedge CLK) begin
        
        
        //Separate Counters and done signals needed when multiple FSMs operating in parallel, here we didn't perform any pipelining.
        /*
        ctr_store_weight <= 4'b0; 
        ctr_read_single_addr <= 4'b0; 
        ctr_store_ham_weight <= 4'b0;
        ctr_compute_hd <= 4'b0;
        ctr_hd_acc_rdout <= 4'b0;
        
        done_store_weight <= 1'b0; 
        done_read_single_addr <= 1'b0; 
        done_store_ham_weight <= 1'b0; 
        done_compute_hd <= 1'b0;
        done_hd_acc_rdout <= 1'b0;
        */
     if(reset) begin
        
        counter_fsm <= 16'b0;
        
        ext_hd_sel_fifo <= 1'b0;
        pop_n_instFIFO_ext <= 1'b1;
        pop_n_iFIFO_ext <= 1'b1;
        push_n_oFIFO_ext <= 1'b1;
        pop_n_instFIFO_hd <= 1'b1;
        pop_n_iFIFO_hd <= 1'b1;
        push_n_oFIFO_hd <= 1'b1;
        
        COL_BASE_ADDR <= 3'b0;
        COL_BURST_SIZE_WR_RD <= 3'b0; 
        SEGMENT_WIDTH <= 5'b0; 
        ROW_ADDR_WR_RD <= 10'b0; 
        ROW_ADDR_HAM_WEIGHT_STORE <= 9'b0; 
        NUM_WRITE_CYCLES <= 9'b0; 
        ROW_ADDR_HAM_COMPUTE <= 3'b0; 
        ROW_BURST_SIZE_COMPUTE <= 2'b0; 
        COL_BURST_SIZE_COMPUTE <= 4'b0; 
        RESET_ACC <= 1'b0;
        PHD_READOUT_EN <= 1'b0;
        wr_cycle_ctr <= 9'b0;
        row_burst_ctr <= 4'b0;
        LOCAL_INSTR <= CMD_RESET_REGS;
     end else begin
     
        //Assign variables only which doesn't require any latch behaviour
        pop_n_instFIFO_ext <= 1'b1;
        pop_n_iFIFO_ext <= 1'b1;
        push_n_oFIFO_ext <= 1'b1;
        pop_n_instFIFO_hd <= 1'b1;
        pop_n_iFIFO_hd <= 1'b1;
        push_n_oFIFO_hd <= 1'b1;
        RESET_ACC <= 1'b0; 
        
        case (curr_state)
            STATE_RESET: begin
                curr_state <= STATE_IDLE;
            end
            STATE_IDLE: begin
                if(~empty_instFIFO_ext || ~empty_instFIFO_hd) begin
                    if(~empty_instFIFO_ext) begin 
                        pop_n_instFIFO_ext <= 1'b0;
                        ext_hd_sel_fifo <= 1'b0;
                    end else begin 
                        pop_n_instFIFO_hd <= 1'b0;
                        ext_hd_sel_fifo <= 1'b1;
                    end
                    counter_fsm <= 16'b0;
                    case (INSTR)
                        INSTR_STORE_RRAM: begin
                            curr_state <= STATE_STORE_RRAM;
                            ROW_ADDR_WR_RD <= OPCODE[9:0];
                            COL_BASE_ADDR <= OPCODE[12:10]; 
                            COL_BURST_SIZE_WR_RD <= OPCODE[15:13];
                            NUM_WRITE_CYCLES <= 8'hFF;  //Defaults to 255 cycles since we don't have any OpCode left for programming.
                            WL_BIAS_SEL_prereg = 1'b1;
                            BL_BIAS_SEL_prereg = 1'b1;   //WRITE
                            SL_BIAS_SEL_prereg = 1'b1;  // WRITE 
                        end
                        INSTR_READ_RRAM: begin
                            curr_state <= STATE_READ_RRAM;
                            ROW_ADDR_WR_RD <= OPCODE[9:0];
                            COL_BASE_ADDR <= OPCODE[12:10]; 
                            COL_BURST_SIZE_WR_RD <= OPCODE[15:13];
                            WL_BIAS_SEL_prereg = 1'b0;
                            BL_BIAS_SEL_prereg = 1'b0;   // READ 
                            SL_BIAS_SEL_prereg = 1'b0;  // READ 
                        end
                        INSTR_STORE_HAM_WEIGHT: begin
                            curr_state <= STATE_STORE_HAM_WEIGHT;
                            ROW_ADDR_HAM_WEIGHT_STORE <= OPCODE[8:0];
                            COL_BASE_ADDR <= OPCODE[11:9]; 
                            SEGMENT_WIDTH <= (OPCODE[12])?16:8;
                            NUM_WRITE_CYCLES <= 32*(OPCODE[15:13]+1); //If NUM_WRITE_CYCLES is 7, we want to write for 32*8 cycles
                            WL_BIAS_SEL_prereg = 1'b1;
                            BL_BIAS_SEL_prereg = 1'b1;   //WRITE
                            SL_BIAS_SEL_prereg = 1'b1;  // WRITE 
                        end
                        INSTR_HAM_SEGMENT_COMPUTE: begin
                            curr_state <= STATE_HAM_SEGMENT_COMPUTE;
                            ROW_ADDR_HAM_COMPUTE <= OPCODE[2:0];
                            ROW_BURST_SIZE_COMPUTE <= (OPCODE[4:3]==0)?4:((OPCODE[4:3]==1)?2:1);
                            COL_BURST_SIZE_COMPUTE <= OPCODE[8:5]; 
                            SEGMENT_WIDTH <= (OPCODE[9])?16:8;
                            RESET_ACC <= OPCODE[10];
                            PHD_READOUT_EN <= OPCODE[11];
                            WL_BIAS_SEL_prereg = 1'b0;
                            BL_BIAS_SEL_prereg = 1'b0;   // READ 
                            SL_BIAS_SEL_prereg = 1'b0;  // READ 
                        end
                        default: begin
                            curr_state <= STATE_IDLE;
                            counter_fsm <= 16'b0;
                        end
                    endcase
                end
            end
			//1. 10b WRITE_ROW_ADDR is the row address.
			//2. 3b COL_BASE_ADDR is the base addr for column (can be either 0, 64, 128, .....512-64)
			//3. 3b COL_BURST_SIZE is the number of 64b packets we are trying to write. If burst_size=000, we only write 1 packet, if it's 111, we write entire 512 columns
            STATE_STORE_RRAM: begin
                 if (counter_fsm != NUM_STORE_WEIGHTS_CYCLES) begin
                    //Pop dataFIFO, first load the 64b data and then write 64 columns for 2*255 cycles, Pop new data after erevry (1+2*255) cycles
                    
                    if(counter_fsm % 3 == 0) begin
                        //Pop New data from the FIFO
                        //Send in weights_reg_load command 
                        if ((~empty_iFIFO_ext && ~ext_hd_sel_fifo) || (~empty_iFIFO_hd && ~ext_hd_sel_fifo)) begin
                            if (~ext_hd_sel_fifo) pop_n_iFIFO_ext <= 1'b0;
                            else pop_n_iFIFO_hd <= 1'b0;
                            DATAIN <= (ext_hd_sel_fifo)?dout_iFIFO_ext:dout_iFIFO_hd;
							LOCAL_INSTR <= CMD_WRITE_WEIGHTS;
							START_ADDR_SL <= (COL_BASE_ADDR+counter_fsm/3)*DATAIN_WIDTH; //counter_fsm/(1+2*NUM_WRITE_CYCLES) is the counter for burst-size
                            counter_fsm <= counter_fsm+1;
                        end  
                    end else begin //counter_fsm % (1+2*NUM_WRITE_CYCLES) >= 1 && counter_fsm % (1+2*NUM_WRITE_CYCLES) <= 2*NUM_WRITE_CYCLES
                        //Write onto Selected Row and Selected Columns
                        //Send in program_rram command 
						LOCAL_INSTR <= CMD_PROGRAM_DEVICE; 
						ROW_ADDR <= ROW_ADDR_WR_RD; 
						ROW_POL <= 1'b0; //Since We are writing to single row, this will be the 0 throughout 
						COL_POL <= (wr_cycle_ctr % 2 == 0)?1'b0:1'b1; //Alternate every write cycle, start at COl_POL=0, when counter_fsm % (1+2*NUM_WRITE_CYCLES)=1
                        if (wr_cycle_ctr == NUM_WRITE_CYCLES) begin
                            counter_fsm <= counter_fsm+1; 
                            wr_cycle_ctr <= 9'b0;
                        end else wr_cycle_ctr <= wr_cycle_ctr+1;
                        
                    end
                 end else begin //If done, go to STATE_IDLE
                    curr_state <= STATE_IDLE;
                    counter_fsm <= 16'b0;
                    LOCAL_INSTR <= CMD_NO_OP; 
                 end
                 
            end
			//1. {row_addr-10b} is the row address.
			//2. {col_addr-3b} is the base addr for column (can be either 0, 64, 128, .....512-64)
			//3. 4x{burst_size-3b} is the number of 64b packets we are trying to read. If burst_size=000, we only read 64 columns(4bx64) if it's 111, we read entire 512 columns (4bx512). 
			//We read all entire 4b from each column to know the precise analog value of resistance
            STATE_READ_RRAM: begin
                if (counter_fsm != NUM_READ_SINGLE_ADDR_CYCLES) begin
                    if (counter_fsm <= (NUM_SL/NUM_ADC)-1) begin
                        //Select Columns one by one in 16 Cycles, Store ADC output into register
                        counter_fsm <= counter_fsm+1;
						LOCAL_INSTR <= CMD_READ_SINGLE_ADDR;
						ROW_ADDR <= ROW_ADDR_HAM_WEIGHT_STORE;
						COL_OFFSET <= counter_fsm;
                    end else begin
                        if ((~full_oFIFO_ext && ~ext_hd_sel_fifo) || (~full_oFIFO_hd && ext_hd_sel_fifo)) begin
                            //Pack outputs into 64b packets and send to oFIFO if not full.
                            counter_fsm <= counter_fsm+1;
						    LOCAL_INSTR <= CMD_READ_ADC_OUT;
						    READOUT_OFFSET_ADDR <= (counter_fsm>(NUM_SL/NUM_ADC)-1)?counter_fsm-(NUM_SL/NUM_ADC):5'b0; 
                            if (~ext_hd_sel_fifo) push_n_oFIFO_ext <= 1'b0;
                            else push_n_oFIFO_hd <= 1'b0;
                        end
                    end
                end else begin //If done, go to STATE_IDLE
                    curr_state <= STATE_IDLE;
                    counter_fsm <= 16'b0;
                    LOCAL_INSTR <= CMD_NO_OP; 
                end
            
            end
			//1. {row_addr-9b} is the row address.
			//2. {col_addr-3b} is the base addr for column (for segment_width=8b, we can write 32 classes in 4 cycles, base address need to be incremented +128 every cycle and  for segment_width=16b, we can write 32 classes in 8 cycles, base address need to be incremented +64 every cycle)
			//3. {segment_width-1b} denotes whether we are only 8 columns (segment_width=0) or we use all 16 columns (segment_width=1). If the D = 512, our weight memory layout is 64x8b, If D=1024, our layout is 64x16b   
            //4. {NUM_WRITE_CYCLES-3b}: number of cycles to wtite = 32xNUM_WRITE_CYCLES
            STATE_STORE_HAM_WEIGHT: begin
            
                if (counter_fsm != NUM_STORE_HAM_WEIGHT_CYCLES) begin
                    //Pop dataFIFO, first load the 64b data and then write 64 columns for 2*255 cycles, Pop new data after erevry (1+2*NUM_WRITE_CYCLES) cycles
                    
                    if(counter_fsm % 5 == 0) begin
                        //Pop New data from the FIFO
                        //Send in weights_reg_load command 
                        if ((~empty_iFIFO_ext && ~ext_hd_sel_fifo) || (~empty_iFIFO_hd && ~ext_hd_sel_fifo)) begin
                            if (~ext_hd_sel_fifo) pop_n_iFIFO_ext <= 1'b0;
                            else pop_n_iFIFO_hd <= 1'b0;
                            DATAIN <= (ext_hd_sel_fifo)?dout_iFIFO_ext:dout_iFIFO_hd;
							LOCAL_INSTR <= CMD_WRITE_HAM_WEIGHTS;
							START_ADDR_SL <= ((NUM_SL/NUM_ADC)/SEGMENT_WIDTH)*(COL_BASE_ADDR+(counter_fsm/5))*DATAIN_WIDTH; 
							//If segment width is 8, increment address by 64/8 *16 = 128 
							//If segment width is 16, increment address by 64/16 *16 = 64
                            counter_fsm <= counter_fsm+1;
                        end  
                    end else begin //counter_fsm % (1+4*NUM_WRITE_CYCLES) >= 1 && counter_fsm % (1+4*NUM_WRITE_CYCLES) <= 4*NUM_WRITE_CYCLES
                        //Write onto Selected Row and Selected Columns
                        //Send in program_rram command 
						LOCAL_INSTR <= CMD_PROGRAM_DEVICE; 
						ROW_ADDR <= ROW_ADDR_WR_RD; 
						ROW_POL <= ((wr_cycle_ctr % 4 == 0) || (wr_cycle_ctr % 4 == 1))?1'b0:1'b1; //Alternate every 2 cycles
						COL_POL <= (wr_cycle_ctr % 2 == 0)?1'b0:1'b1; //Alternate every write cycle, start at COl_POL=0, when counter_fsm % (1+2*NUM_WRITE_CYCLES)=1
                        if(wr_cycle_ctr == NUM_WRITE_CYCLES) begin
                            counter_fsm <= counter_fsm+1; 
                            wr_cycle_ctr <= 9'b0;
                        end else wr_cycle_ctr <= wr_cycle_ctr+1;
                    end
                    
                 end else begin //If done, go to STATE_IDLE
                    curr_state <= STATE_IDLE;
                    counter_fsm <= 16'b0;
                    LOCAL_INSTR <= CMD_NO_OP; 
                 end
            
            end
            //1. {row_addr-3b} is a 3b address (512/64=8 segments along the row dimension) to indicate the row starting address.
            //2. {row_burst_sel-2b} whether to select 16 rows at once or 32 rows at once or 64 rows at once
            //3. {col_burst_size-4b} is number of columns we need to select. Burst_size=num_columns to read
            //4. {segment_width-1b} If 0, we only need to accumulate over 8 columns, otherwise we accumulate over 16 columns. 
            //5. {reset_acc-1b} Reset the accumulator when this bit is active
            //6. {rd_en-1b} Readout the Accumulated Output and Send it to FIFO.
            
            //TODO:- Merge readout_en instruction with compute instruction
            STATE_HAM_SEGMENT_COMPUTE: begin
              if (counter_fsm != NUM_COMPUTE_HD_CYCLES+NUM_READOUT_PHD_CYCLES) begin //Compute Partial hamming Distance and store in PHD_ACC register
                if(counter_fsm < NUM_COMPUTE_HD_CYCLES) begin
                        //pop iFIFO, fill input REGs, compute HD for ROW_BURST_SIZE_COMPUTE cycles and then again iFIFO
                       
                    if(row_burst_ctr==0) begin
                        //Pop New data from the iFIFO
                        //Send in inputs_reg_load command 
                        if ((~empty_iFIFO_ext && ~ext_hd_sel_fifo) || (~empty_iFIFO_hd && ~ext_hd_sel_fifo)) begin
                            if (~ext_hd_sel_fifo) pop_n_iFIFO_ext <= 1'b0;
                            else pop_n_iFIFO_hd <= 1'b0;
                            DATAIN <= (ext_hd_sel_fifo)?dout_iFIFO_ext:dout_iFIFO_hd;
                            row_burst_ctr <= row_burst_ctr+1;
							LOCAL_INSTR <= CMD_WRITE_INPUTS; 
							ROW_ADDR_MVM <= 64*ROW_ADDR_HAM_COMPUTE;
							NUM_ROWS_SEGMENT <= DATAIN_WIDTH/ROW_BURST_SIZE_COMPUTE; //If burst_size=4, 4 segments
                        end  
                    end else begin //counter_fsm % (1+2*NUM_WRITE_CYCLES) >= 1 && counter_fsm % (1+2*NUM_WRITE_CYCLES) <= 2*NUM_WRITE_CYCLES
                        //Write onto Selected Row and Selected Columns
                        //Send in program_rram command 
                        //Spend one spare cycle for precharge before every compute
                        if (row_burst_ctr % 2 == 1) begin 
                          LOCAL_INSTR <= CMD_NO_OP; //Precharge
                          row_burst_ctr <= row_burst_ctr+1;
                        end else begin 
						  LOCAL_INSTR <= CMD_COMPUTE_MVM;
						  //ROW_ADDR_MVM <= 64*ROW_ADDR_HAM_COMPUTE+NUM_ROWS_SEGMENT*((counter_fsm-1)/(1+COL_BURST_SIZE_COMPUTE)); //Change row base address based on the burst size
						  //COL_OFFSET <= (ROW_BURST_SIZE_COMPUTE==4)?((counter_fsm-1)>>3):((ROW_BURST_SIZE_COMPUTE==2)?((counter_fsm-1)>>2):counter_fsm>>1) ; //Change Column Address
                          ROW_ADDR_MVM <= 64*ROW_ADDR_HAM_COMPUTE+16*((row_burst_ctr-1)/2);
                          COL_OFFSET <= counter_fsm;
                          if(row_burst_ctr == (1+2*ROW_BURST_SIZE_COMPUTE)) begin
                                row_burst_ctr <= 4'b0;
                                counter_fsm <= counter_fsm+1;
                          end else row_burst_ctr <= row_burst_ctr+1;
                        end
                    end
              
                end else begin  //READOUT the outputs from ACC Register
                        if ((~full_oFIFO_ext && ~ext_hd_sel_fifo) || (~full_oFIFO_hd && ext_hd_sel_fifo)) begin
                            counter_fsm <= counter_fsm+1;
                            //Readout 16b PHD ACC value from 32 Classes and push to oFIFO
                            LOCAL_INSTR <= CMD_READ_PHDACC_OUT;
						    //READOUT_OFFSET_ADDR <= (counter_fsm>(NUM_SL/NUM_ADC)-1)?counter_fsm-(NUM_SL/NUM_ADC):5'b0; 
						    BASE_CLASS_ADDR <= (counter_fsm-NUM_COMPUTE_HD_CYCLES)*((DATAOUT_WIDTH/PHD_ACC_WIDTH)) ; // 4 classes can be read each cycle
                            if (~ext_hd_sel_fifo) push_n_oFIFO_ext <= 1'b0;
                            else push_n_oFIFO_hd <= 1'b0;
                        end
                end
            end else begin //If done, go to STATE_IDLE
                        curr_state <= STATE_IDLE;
                        counter_fsm <= 16'b0;
                        LOCAL_INSTR <= CMD_NO_OP; 
            end
            end
            default:begin
                curr_state <= STATE_RESET;
            end
        endcase
     end
    end
    
    
    //Combinational Logic
    always_comb begin
    
        //This default condition needs to be specified to avoid latches and result in clean synthesis.
        WL_UNGATED = {NUM_WL{1'b0}};  //None of the WL selected
        
        case (LOCAL_INSTR)
            CMD_PROGRAM_DEVICE: begin //Controller needs to spend 4 cycles to program each weight, Row and Column Polarity exposed to ASIC Controller, increment ADDR by +2 after each weight programming
                 ////Written Assuming CMD_PROGRAM_DEVICE is active for OpCode[17;10] number of cycles, If ASIC doesn't time this, we can build a timer internally/ build a FSM
                 //If ROW_POL (OPCODE[11]) is 0, Write to the Positive Input Line (+1), ortherwise write to the negative Input line (-1)
                 WL_UNGATED = (ROW_POL)?((1<<ROW_ADDR)+1):(1<<ROW_ADDR);  //Programming the array, select one WL at a time to load weights, ROW_POL = OPCODE[11]
                 
            end
            CMD_READ_SINGLE_ADDR: begin
                WL_UNGATED = (1<<ROW_ADDR);  //Read the Array
               
            end
            CMD_RESET_REGS: begin
                WL_UNGATED = {NUM_WL{1'b0}}; //Reset WL_UNGATED
            end
            CMD_NO_OP: begin
                WL_UNGATED = {NUM_WL{1'b0}}; //Reset WL_UNGATED
            end
            CMD_COMPUTE_MVM: begin
               //OpCode:- 6b(Segment Width/Number of Rows to read) + 10b (START ADDR for WL)
               for (int i=0; i <NUM_WL; i++) begin
                    if (i >= ROW_ADDR_MVM && i < ROW_ADDR_MVM+NUM_ROWS_SEGMENT) WL_UNGATED[i] = WL_IN_REG[i];
                    else WL_UNGATED[i] = 1'b0;
               end
               /*
               if(NUM_ROWS_SEGMENT == 6'd15) WL_UNGATED[ROW_ADDR_MVM+:32] = WL_IN_REG[ROW_ADDR_MVM+:32];  //load the inputs for the required HD distance computation
               else if (NUM_ROWS_SEGMENT == 6'd31) WL_UNGATED[ROW_ADDR_MVM+:64] = WL_IN_REG[ROW_ADDR_MVM+:64];  //load the inputs for the required HD distance computation
               else if (NUM_ROWS_SEGMENT == 6'd63) WL_UNGATED[ROW_ADDR_MVM+:128] = WL_IN_REG[ROW_ADDR_MVM+:128];  //load the inputs for the required HD distance computation
               */
            end
            default:begin
                //Do Nothing
                WL_UNGATED = {NUM_WL{1'b0}};  //None of the WL selected
            end
        endcase
        
        
         for (int i=0; i<NUM_WL;i++) begin
            WL_SEL_prereg[i] = (LOCAL_INSTR==CMD_COMPUTE_MVM)?(WL_UNGATED[i] & CLK_WL):WL_UNGATED[i]; //Only gate the Clock when MVM is done, Don't gate it when programming/reading
         end
    end
    //Sequential Logic
    always @(posedge CLK) begin 
        case (LOCAL_INSTR)
            
            CMD_WRITE_INPUTS: begin
                //Select the appropriate registers to load inputs, 128 WLs (64 inputs) can be selected at once, thus 7 lsbs are ignored
                for (int i=0; i<DATAIN_WIDTH;i++) begin //Loop, Load DATAIN at even locations, and ~DATAIN at odd locations
					WL_IN_REG[2*ROW_ADDR_MVM+2*i] <= DATAIN[i];
                    WL_IN_REG[2*ROW_ADDR_MVM+2*i+1] <= ~DATAIN[i];
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
         
		 
		 //This default condition needs to be specified to avoid latches and result in clean synthesis.
		 for (int i=0; i<NUM_SL;i++) begin
                    BL_UNGATED[i] = 3'b100; //BL selected to reference
                    SL_UNGATED[i] = 3'b100; //SL selected to reference
         end
           
         SL_MUX_SEL_prereg = {NUM_SL{1'b0}}; // Default MUXSEL 
         din_oFIFO_ext = {DATAOUT_WIDTH{1'b0}};
         din_oFIFO_hd = {DATAOUT_WIDTH{1'b0}};
		 case (LOCAL_INSTR) 
				CMD_PROGRAM_DEVICE: begin 
                     //Controller needs to spend 4 cycles to program each weight, Row and Column Polarity exposed to ASIC Controller, increment ADDR by +2 after each weight programming
                     //Written Assuming CMD_PROGRAM_DEVICE is active for OpCode[17;10] number of cycles, If ASIC doesn't time this, we can build a timer internally/ build a FSM
                     //If ROW_POL is 1, Write to the Positive Input Line (+1), otherwise write to the negative Input line (-1)
                     //When ROW_POL==1 (OPCODE[11]) and COL_POL(OPCODE[10])==1 or ROW_POL==0 and COL_POL==0, We write WEIGHT_REG[i]
                     //When ROW_POL==1 (OPCODE[11]) and COL_POL(OPCODE[10])==0 or ROW_POL==0 and COL_POL==1, We write ~WEIGHT_REG[i]
                     for (int i=0; i<NUM_SL;i++) begin
                        if(~COLSELb[i])
                            if ((ROW_POL==1 && COL_POL==1) || (ROW_POL==0 & COL_POL==0)) begin
                                BL_UNGATED[i] = {1'b0, ~WEIGHT_REG[i], WEIGHT_REG[i]}; //BL selected to VBL+ if WEIGHT=1, BL selected to VBL- if WEIGHT=0
                                SL_UNGATED[i] = {1'b0, WEIGHT_REG[i], ~WEIGHT_REG[i]}; //SL selected to VBL- if WEIGHT=1, SL selected to VBL+ if WEIGHT=0
                            end else if ((ROW_POL==0 && COL_POL==1) || (ROW_POL==1 & COL_POL==0)) begin
                                BL_UNGATED[i] = {1'b0, WEIGHT_REG[i], ~WEIGHT_REG[i]}; //BL selected to VBL- if WEIGHT=1, BL selected to VBL+ if WEIGHT=0
                                SL_UNGATED[i] = {1'b0, ~WEIGHT_REG[i], WEIGHT_REG[i]}; //SL selected to VBL+ if WEIGHT=1, SL selected to VBL- if WEIGHT=0
                            end
                        else begin
                            BL_UNGATED[i] = 3'b100; //BL selected to Vref
                            SL_UNGATED[i] = 3'b100; //SL selected to Vref
                        end
                    end
                end
                CMD_READ_SINGLE_ADDR: begin
                    for (int i=0; i<NUM_ADC;i++) begin  //Only read 32 in parallel
                        BL_UNGATED[i*(NUM_SL/NUM_ADC)+COL_OFFSET] = 3'b001; //BL selected to VBL+, BL selected to VBL-
                        SL_UNGATED[i*(NUM_SL/NUM_ADC)+COL_OFFSET] = 3'b000; //SL Left floating and the parasitic cap gets charged.
                    end
                    
                    //Don't add any register for MUX_SEL since we want to MUX it on the same clock edge
                    for (int i=0; i < NUM_ADC;i++) begin
                        SL_MUX_SEL_prereg[i*(NUM_SL/NUM_ADC)+:(NUM_SL/NUM_ADC)] = (1<<COL_OFFSET); //OPCODE[3:0] is the MUXSEL, (NUM_SL/NUM_ADC)=16
                    end
                end
				CMD_READ_ADC_OUT: begin
					 for (int i=0; i<(DATAOUT_WIDTH/ADC_WIDTH);i++) begin
                        din_oFIFO_ext[ADC_WIDTH*i+:ADC_WIDTH] = ADCOUT_reg[DATAOUT_WIDTH*READOUT_OFFSET_ADDR+i]; //DATAOUT_WIDTH(64)*READOUT_OFFSET_ADDR(5b)+i
                        din_oFIFO_hd[ADC_WIDTH*i+:ADC_WIDTH] = ADCOUT_reg[DATAOUT_WIDTH*READOUT_OFFSET_ADDR+i];
                     end
				end
                CMD_COMPUTE_MVM: begin
                    //Don't add any register for MUX_SEL since we want to MUX it on the same clock edge
                    for (int i=0; i < NUM_ADC;i++) begin
						//Only activate 32 BL at the same time
						BL_UNGATED[i*(NUM_SL/NUM_ADC)+COL_OFFSET] = 3'b001; //BL selected to VBL+, BL selected to VBL-
                        SL_UNGATED[i*(NUM_SL/NUM_ADC)+COL_OFFSET] = 3'b000; //SL Left floating and the parasitic cap gets charged.
                        SL_MUX_SEL_prereg[i*(NUM_SL/NUM_ADC)+:(NUM_SL/NUM_ADC)] = (1<<COL_OFFSET); //OPCODE[3:0] is the MUXSEL, (NUM_SL/NUM_ADC)=16
                    end
                end
                
                CMD_READ_PHDACC_OUT: begin //4 classes can be read at once.
                    for (int i=0; i<(DATAOUT_WIDTH/PHD_ACC_WIDTH);i++) begin
                        din_oFIFO_ext[PHD_ACC_WIDTH*i+:PHD_ACC_WIDTH] = PHD_ACC[BASE_CLASS_ADDR+i]; //Column Address = OPCODE[8:4]+i
                        din_oFIFO_hd[PHD_ACC_WIDTH*i+:PHD_ACC_WIDTH] = PHD_ACC[BASE_CLASS_ADDR+i];
                    end
                end
                
                CMD_RESET_REGS: begin
                    for (int i=0; i<NUM_SL;i++) begin
                        BL_UNGATED[i] = 3'b100; //BL to Vref
                        SL_UNGATED[i] = 3'b100; //SL to Vref
                    end
                end
                CMD_NO_OP: begin
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
                end
          endcase
          for (int i=0; i<NUM_SL;i++) begin
                //BL Controls
				BL_SEL_prereg[i][0]= (LOCAL_INSTR==CMD_COMPUTE_MVM)?(BL_UNGATED[i][0] & CLK_BL):BL_UNGATED[i][0]; //BLplus SEL, //Only gate the Clock when MVM is done, Don't gate it when programming/reading
				BL_SEL_prereg[i][1]= (LOCAL_INSTR==CMD_COMPUTE_MVM)?(BL_UNGATED[i][1] & CLK_BL):BL_UNGATED[i][1]; //BLminus SEL, //Only gate the Clock when MVM is done, Don't gate it when programming/reading
				BL_SEL_prereg[i][2]= (LOCAL_INSTR==CMD_COMPUTE_MVM)?(BL_UNGATED[i][2] & CLK_BL):BL_UNGATED[i][2]; //BLref SEL  //Only gate the Clock when MVM is done, Don't gate it when programming/reading
				
				//SL Controls
				SL_SEL_prereg[i][0]= (LOCAL_INSTR==CMD_COMPUTE_MVM)?(SL_UNGATED[i][0] & CLK_BL):SL_UNGATED[i][0]; //SLplus SEL, //Only gate the Clock when MVM is done, Don't gate it when programming/reading
				SL_SEL_prereg[i][1]= (LOCAL_INSTR==CMD_COMPUTE_MVM)?(SL_UNGATED[i][1] & CLK_BL):SL_UNGATED[i][1]; //SLminus SEL, //Only gate the Clock when MVM is done, Don't gate it when programming/reading
				SL_SEL_prereg[i][2]= (LOCAL_INSTR==CMD_COMPUTE_MVM)?(SL_UNGATED[i][2] & CLK_BL):SL_UNGATED[i][2]; //SLref SEL //Only gate the Clock when MVM is done, Don't gate it when programming/reading
		 end
    end
    
    //Sequential Logic
    always @(posedge CLK) begin
        case (LOCAL_INSTR)
            
			CMD_WRITE_WEIGHTS: begin
                //Select the appropriate registers to load inputs, 64 Weights can be selected at once, thus 6 lsbs are ignored
                
				for (int i=0; i<NUM_SL;i++) begin
					if ((i < START_ADDR_SL) || (i > START_ADDR_SL+DATAIN_WIDTH-1)) begin
					   COLSELb[i] <= 1'b1; //Column unselected for programming
					   WEIGHT_REG[i] <= 1'b0; 
					end else begin
					   COLSELb[i] <= 1'b0; //Column selected for programming
					   WEIGHT_REG[i] <= DATAIN[i-START_ADDR_SL]; 
					end
				end
            end
			
            CMD_WRITE_HAM_WEIGHTS: begin
                //If segment width is 8, we write only 8 out of 16 Columns
				//If segment width is 16, we write only 16 out of 16 Columns
				if (SEGMENT_WIDTH == 8) begin
					for (int i=0; i<NUM_SL;i++) begin
						if ((i < START_ADDR_SL) || (i > START_ADDR_SL+2*DATAIN_WIDTH-1)) begin 
							COLSELb[i] <= 1'b1; //Column unselected for programming
							WEIGHT_REG[i] <= 1'b0; 
						end else begin 
							if (i[3:0] < 4'h8) begin
								COLSELb[i] <= 1'b0; //Column selected for programming
								//If START_ADDR=0, Then ADDR 0...7 maps to DATAIN 0..7, ADDR 16...23 maps to DATAIN 8..15 and so on
								WEIGHT_REG[i] <= DATAIN[8*((i-START_ADDR_SL)/16) + ((i-START_ADDR_SL)%16)]; 
							end else begin
								COLSELb[i] <= 1'b1; //Column unselected for programming
								WEIGHT_REG[i] <= 1'b0; 
							end
						end
					end
				end else begin
					for (int i=0; i<NUM_SL;i++) begin
						if ((i < START_ADDR_SL) || (i > START_ADDR_SL+DATAIN_WIDTH-1)) begin
							COLSELb[i] <= 1'b1; //Column unselected for programming
							WEIGHT_REG[i] <= 1'b0; 
						end else begin 
							WEIGHT_REG[i] <= DATAIN[i-START_ADDR_SL]; 
							COLSELb[i] <= 1'b0; //Column selected for programming
						end
					end
				end
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
    
     //Thermometer to binary conversion using priority encoder
    always_comb begin
        for (int i=0; i < NUM_ADC;i++) begin
            //Full adder based encoder
               ADCOUT[i] = ADCOUT_THERM[i][0]+ADCOUT_THERM[i][1]+ADCOUT_THERM[i][2]+ADCOUT_THERM[i][3]+ADCOUT_THERM[i][4]
                           +ADCOUT_THERM[i][5]+ADCOUT_THERM[i][6]+ADCOUT_THERM[i][7]+ADCOUT_THERM[i][8]+ADCOUT_THERM[i][9]
                           +ADCOUT_THERM[i][10]+ADCOUT_THERM[i][11]+ADCOUT_THERM[i][12]+ADCOUT_THERM[i][13]+ADCOUT_THERM[i][14] ;
            
          end         
           /*
            casex(ADCOUT_THERM[i])
                16'b0000000000000000: ADCOUT[i] = 4'b0000;
                16'b000000000000001x: ADCOUT[i] = 4'b0001;
                16'b00000000000001xx: ADCOUT[i] = 4'b0010;
                16'b0000000000001xxx: ADCOUT[i] = 4'b0011;
                16'b000000000001xxxx: ADCOUT[i] = 4'b0100;
                16'b00000000001xxxxx: ADCOUT[i] = 4'b0101;
                16'b0000000001xxxxxx: ADCOUT[i] = 4'b0110;
                16'b000000001xxxxxxx: ADCOUT[i] = 4'b0111;
                16'b00000001xxxxxxxx: ADCOUT[i] = 4'b1000;
                16'b0000001xxxxxxxxx: ADCOUT[i] = 4'b1001;
                16'b000001xxxxxxxxxx: ADCOUT[i] = 4'b1010;
                16'b00001xxxxxxxxxxx: ADCOUT[i] = 4'b1011;
                16'b0001xxxxxxxxxxxx: ADCOUT[i] = 4'b1000;
                16'b001xxxxxxxxxxxxx: ADCOUT[i] = 4'b1001;
                16'b01xxxxxxxxxxxxxx: ADCOUT[i] = 4'b1010;
                16'b1xxxxxxxxxxxxxxx: ADCOUT[i] = 4'b1011;
            endcase 
          */  
        
    end
    
    //2 pipelined delay, 1 from WL_SEL/BL_SEL to feeding WL/BL (1 cycle), another for analog macro operation, latch output on the next edge
    reg [3:0] curr_state_reg;
    reg [3:0] LOCAL_INSTR_reg;
    reg [3:0] COL_OFFSET_reg;
    reg [3:0] curr_state_reg_2;
    reg [3:0] LOCAL_INSTR_reg_2;
    reg [3:0] COL_OFFSET_reg_2;
   
    always @(posedge CLK) begin  //Latch Outputs on the next clock edge of COMPUTE
        curr_state_reg <= curr_state;
        LOCAL_INSTR_reg <= LOCAL_INSTR;
        COL_OFFSET_reg <= COL_OFFSET;
        curr_state_reg_2 <= curr_state_reg;
        LOCAL_INSTR_reg_2 <= LOCAL_INSTR_reg;
        COL_OFFSET_reg_2 <= COL_OFFSET_reg;
        
        if (RESET_ACC) begin 
            for (int i=0; i < NUM_ADC;i++) begin
                PHD_ACC[i] <= {NUM_ADC{1'b0}};
            end
            for (int i=0; i < NUM_SL;i++) begin
                ADCOUT_reg[i] <= {ADC_WIDTH{1'b0}};
            end
        end else if ((curr_state_reg_2==STATE_READ_RRAM && LOCAL_INSTR_reg_2 == CMD_READ_SINGLE_ADDR) || (curr_state_reg_2==STATE_HAM_SEGMENT_COMPUTE && LOCAL_INSTR_reg_2 == CMD_COMPUTE_MVM)) begin
        
            for (int i=0; i < NUM_ADC;i++) begin
				ADCOUT_reg[i*(NUM_SL/NUM_ADC)+COL_OFFSET_reg_2] <= ADCOUT[i]; //OPCODE[3:0] is the MUXSEL, (NUM_SL/NUM_ADC) = 16
				PHD_ACC[i] <= PHD_ACC[i] + ADCOUT[i]; //OPCODE[3:0] is the MUXSEL, (NUM_SL/NUM_ADC) = 16
            end
         end
     end
    
    
endmodule



