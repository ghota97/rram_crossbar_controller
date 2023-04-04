`timescale 1ns / 1ps

//High-level instructions and OpCode coming from instrFIFO or HD Controller 
//These instructions will be used to transition between FSMs and then issue low-level instructions to rram_controller

// INSTR (0000) : STORE RRAM, {burst_size-3b, col_addr-3b, row_addr-9b}
// INSTR (0001) : READ_RRAM, {burst_size-3b, col_addr-3b, row_addr-9b}
// INSTR (0010) : STORE_HAM_WEIGHT, {segment_width-1b}_{col_addr-3b}_{row_addr-9b}
// INSTR (0011) : HAM_SEG_COMPUTE, {segment_width-1b}_{col_burst_size-3b}_{row_addr-9b}


module rram_controller_fsm(CLK, CLK_ADC, CLK_WL, CLK_BL, CLK_ADCOUT, CORE_SEL, pop_n_instFIFO, empty_instFIFO, dout_instFIFO, pop_n_iFIFO, empty_iFIFO, dout_iFIFO, push_n_ofifo, full_oFIFO, din_ofifo, WL_SEL, BL_SEL, SL_SEL, SL_MUX_SEL, WL_BIAS_SEL, BL_BIAS_SEL, SL_BIAS_SEL, ADCOUT);
   
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
    parameter ADC_WIDTH = 4;
    parameter PHD_ACC_WIDTH = 16;

    //Instruction FIFO 
    output reg pop_n_instFIFO;
    input  empty_instFIFO;
    input  [INSTR_WIDTH+OPCODE_WIDTH-1] dout_instFIFO;
    
    //input data FIFO
    output reg pop_n_iFIFO;
    input  empty_iFIFO;
    input  [DATAIN_WIDTH-1-1] dout_iFIFO;
    
    //output data FIFO
    output reg push_n_oFIFO;
    input  full_oFIFO;
    input  [DATAIN_WIDTH-1-1] din_oFIFO;
    
    output [NUM_WL-1:0] WL_SEL; //Gated WL_SEL fed into the Crossbar
    output [2:0] BL_SEL [NUM_SL-1:0]; //Gated BL_SEL fed into the Crossbar (3b each), Controls for 2 adjacent BLs (BL+ and BL-) are shared.
    output [2:0] SL_SEL [NUM_SL-1:0]; //Gated SL_SEL fed into the Crossbar (3b each)
    output [NUM_SL-1:0] SL_MUX_SEL;   //MUX SEL to select between 16 adjacent SLs and pass one to the shared ADC.
    
    output WL_BIAS_SEL; //Selects between WRITE and READ Bias
    output [2:0] BL_BIAS_SEL; //Selects between WRITE and READ Bias for PLUS, MINUS and REF
    output SL_BIAS_SEL; //Selects between WRITE and READ for REF
    
    input [ADC_WIDTH-1:0] ADCOUT[NUM_ADC-1:0]; //32 4-b ADCs
    
    
    wire [INSTR_WIDTH-1:0] INSTR;
    wire [OPCODE_WIDTH-1:0] OPCODE;
    reg [DATAIN_WIDTH-1:0] DATAIN;
    
    localparam [INSTR_WIDTH-1:0] INSTR_STORE_RRAM;
    localparam [INSTR_WIDTH-1:0] INSTR_READ_RRAM;
    localparam [INSTR_WIDTH-1:0] INSTR_STORE_HAM_WEIGHT;
    localparam [INSTR_WIDTH-1:0] INSTR_HAM_SEGMENT_COMPUTE;
    
    assign INSTR = dout_instFIFO[INSTR_WIDTH+OPCODE_WIDTH-1-:INSTR_WIDTH];
    assign OPCODE = dout_instFIFO[OPCODE_WIDTH-1:0];
                   
    //Instructions for Specific executions 
    
    
    // INSTR (0000) : STORE RRAM, {burst_size-3b, col_addr-3b, row_addr-9b} ##burst_size (x):- how many columns we want to write (64x), col_addr:- base addr for column
    // INSTR (0001) : READ_RRAM, {burst_size-3b, col_addr-3b, row_addr-9b}
    // INSTR (0010) : STORE_HAM_WEIGHT, {segment_width-1b}_{col_addr-3b}_{row_addr-9b}
    // INSTR (0011) : HAM_SEG_COMPUTE, {segment_width-1b}_{col_burst_size-3b}_{row_addr-9b}
    
    localparam [2:0] STATE_RESET = 4'd0;
    localparam [2:0] STATE_IDLE = 4'd1;
    localparam [2:0] STATE_STORE_RRAM = 4'd2;
    localparam [2:0] STATE_READ_RRAM = 4'd3;
    localparam [2:0] STATE_STORE_HAM_WEIGHT = 4'd4; 
    localparam [2:0] STATE_HAM_SEGMENT_COMPUTE = 4'd5;
    
    reg [2:0] curr_state; //FSM STATE
   
    //Counters to keep track of states
    reg [15:0] counter_fsm;
    
    //Separate Counters and done signals needed when multiple FSMs operating in parallel, here we didn't perform any pipelining.
    //reg [9:0] ctr_store_weight, ctr_read_single_addr, ctr_store_ham_weight, ctr_compute_hd, ctr_hd_acc_rdout; 
    //reg done_store_weight, done_read_single_addr, done_store_ham_weight, done_compute_hd, done_hd_acc_rdout;
   
    wire [9:0] NUM_STORE_WEIGHTS_CYCLES; //number of cycles to STORE data into the WEIGHT_REGISTER and COLSELb_REGISTER, and finish writing to RRAM
    wire [2:0] NUM_READ_SINGLE_ADDR_CYCLES; //number of read cycles to readout one row of data from RRAM and quantizing over ADC
    wire [9:0] NUM_STORE_HAM_WEIGHT_CYCLES; //number of cycles to STORE hamming weights into the WEIGHT_REGISTER and COLSELb_REGISTER
    wire [3:0] NUM_COMPUTE_HD_CYCLES; //Number of cycles to compute per instructions, this depends on number of columns to be selected
    wire [3:0] NUM_READOUT_PHD_CYCLES; //Number of cycles to readout partial hamming distance
    
    reg [2:0] COL_BASE_ADDR; //3b Col base address for storing, reading, and storing HD weights
    reg [2:0] COL_BURST_SIZE_WR_RD; //Number of Columns we want to compute over
    reg [4:0] SEGMENT_WIDTH; //Denoting whether we are filling 8 columns for PHD calculation or 16 columns
    reg [9:0] ROW_ADDR_WR_RD; //10b row address for single row write/read
    reg [8:0] ROW_ADDR_HAM_WEIGHT_STORE; //9b row addresss to write hamming distance weights, since each input bit is mapped to 2 differential lines
    reg [2:0] NUM_WRITE_CYCLES; //Number of WRITE cycles needed for RRAM
    reg [2:0] ROW_ADDR_HAM_COMPUTE; //Base Row address for starting the HD Computation
    reg [1:0] ROW_BURST_SIZE_COMPUTE; //Number of rows we want to activate at the same time (16 inputs/32 inputs/64 inputs)
    reg [3:0] COL_BURST_SIZE_COMPUTE; //Number of Columns we want to compute over
    reg RESET_ACC; //reset the accumulator when starting a new PHD calculation
    reg PHD_READOUT_EN; //Whenever HD sends an instruction to readout the accumulated partial hamming distance
   
    //assign NUM_CYCLES based on the instructions
    assign NUM_STORE_WEIGHTS_REG_CYCLES = (COL_BURST_SIZE_WR_RD+1) + 2*(COL_BURST_SIZE_WR_RD+1)*NUM_WRITE_CYCLES; //First we load 64b, send 2 256-cycle WRITE each for BL+ and BL- device
    assign NUM_READ_SINGLE_ADDR_CYCLES = (NUM_SL/NUM_ADC) + ADC_WIDTH*(COL_BURST_SIZE_WR_RD+1); //16 cycles to readout all columns, 4b output from each column.
    
    //We always load for 32 classes, takes 8 cycles when segment_width=16, taken 4 otherwise, It takes 4 WR cycles to write one weight (Double-differential encoding) 
    assign NUM_STORE_HAM_WEIGHT_CYCLES = (NUM_HD_CLASSES*SEGMENT_WIDTH/DATAOUT_WIDTH) + 4*(NUM_HD_CLASSES*SEGMENT_WIDTH/DATAOUT_WIDTH)*NUM_WRITE_CYCLES; 
    
    assign NUM_COMPUTE_HD_CYCLES = ROW_BURST_SIZE_COMPUTE*(COL_BURST_SIZE_COMPUTE+1); //If ROW_BURST_SIZE_COMPUTE=0, we compute only 16 rows, ROW_BURST_SIZE_COMPUTE=1, we compute only 32 rows, ROW_BURST_SIZE_COMPUTE=2, we compute 64 rows at once
    assign NUM_READOUT_PHD_CYCLES = (NUM_HD_CLASSES*PHD_ACC_WIDTH)/DATAOUT_WIDTH; //Accumulated result is 16b per class
    
    //Row Controller
    
    //1st stage registering inputs
    reg [NUM_WL-1:0] WL_IN_REG; //Registered WL data coming from asic if the address is selected. NUM_WL/2 as we feed ternary inputs
    //2nd stage just combinational logic
    reg [NUM_WL-1:0] WL_UNGATED; //Ungated WL i.e. WL input before it's gated with CLK_WL.
    
    
    always @(posedge CLK) begin
        
        counter_fsm <= 16'b0;
        
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
        
        COL_BASE_ADDR <= 3'b0;
        COL_BURST_SIZE_WR_RD <= 3'b0;
        SEGMENT_WIDTH <= 1'b0;
        ROW_ADDR_WR_RD <= 10'b0;
        ROW_ADDR_HAM_WEIGHT_STORE <= 9'b0;
        NUM_WRITE_CYCLES <= 3'b0;
        ROW_ADDR_HAM_COMPUTE <= 3'b0;
        NUM_WRITE_CYCLES <= 3'b0;
        ROW_ADDR_HAM_COMPUTE <= 3'b0;
        ROW_BURST_SIZE_COMPUTE <= 2'b0;
        COL_BURST_SIZE_COMPUTE <= 3'b0;
        RESET_ACC <= 1'b0;
        PHD_READOUT_EN <= 1'b0;
        
        
        pop_n_instFIFO <= 1'b1;
        pop_n_iFIFO <= 1'b1;
        push_n_oFIFO <= 1'b1;
        
        case (curr_state)
            STATE_RESET: begin
                curr_state <= STATE_IDLE;
            end
            STATE_IDLE: begin
                if(~empty_instFIFO) begin
                    pop_n_instFIFO <= 1'b0;
                    counter_fsm <= 16'b0;
                    case (INSTR)
                        INSTR_STORE_RRAM: begin
                            curr_state <= STATE_STORE_RRAM;
                            ROW_ADDR_WR_RD <= OPCODE[9:0];
                            COL_BASE_ADDR <= OPCODE[12:10]; 
                            COL_BURST_SIZE_WR_RD <= OPCODE[15:13];
                            NUM_WRITE_CYCLES <= 8'hFF;  //Defaults to 255 cycles since we don't have any OpCode left for programming.
                        end
                        INSTR_READ_RRAM: begin
                            curr_state <= STATE_READ_RRAM;
                            ROW_ADDR_WR_RD <= OPCODE[9:0];
                            COL_BASE_ADDR <= OPCODE[12:10]; 
                            COL_BURST_SIZE_WR_RD <= OPCODE[15:13];
                        end
                        INSTR_STORE_HAM_WEIGHT: begin
                            curr_state <= STATE_STORE_HAM_WEIGHT;
                            ROW_ADDR_HAM_WEIGHT_STORE <= OPCODE[8:0];
                            COL_BASE_ADDR <= OPCODE[11:9]; 
                            SEGMENT_WIDTH <= OPCODE[12];
                            NUM_WRITE_CYCLES <= 32*(OPCODE[15:13]+1); //If NUM_WRITE_CYCLES is 7, we want to write for 32*8 cycles
                        end
                        INSTR_HAM_SEGMENT_COMPUTE: begin
                            curr_state <= STATE_HAM_SEGMENT_COMPUTE;
                            ROW_ADDR_HAM_COMPUTE <= OPCODE[2:0];
                            ROW_BURST_SIZE_COMPUTE <= (OPCODE[4:3]==0)?4:((OPCODE[4:3]==1)?2:1);
                            COL_BURST_SIZE_COMPUTE <= OPCODE[8:5];
                            SEGMENT_WIDTH <= (OPCODE[9])?16:8;
                            RESET_ACC <= OPCODE[10];
                            PHD_READOUT_EN <= OPCODE[11];
                        end
                        default: begin
                            curr_state <= STATE_IDLE;
                            counter_fsm <= 16'b0;
                        end
                    endcase
                end
            end
            STATE_STORE_RRAM: begin
                 if (counter_fsm == NUM_STORE_WEIGHTS_REG_CYCLES-1) begin
                    //Pop dataFIFO, first load the 64b data and then write 64 columns for 2*255 cycles, Pop new data after erevry (1+2*255) cycles
                    
                    if(counter_fsm % (1+2*NUM_WRITE_CYCLES) == 0) begin
                        //Pop New data from the FIFO
                        //Send in weights_reg_load command 
                        if (~empty_iFIFO) begin
                            pop_n_iFIFO <= 1'b0;
                            DATAIN <= dout_iFIFO;
                            counter_fsm <= counter_fsm+1;
                        end  
                    end else begin //counter_fsm % (1+2*NUM_WRITE_CYCLES) >= 1 && counter_fsm % (1+2*NUM_WRITE_CYCLES) <= 2*NUM_WRITE_CYCLES
                        //Write onto Selected Row and Selected Columns
                        //Send in program_rram command 
                        counter_fsm <= counter_fsm+1;
                    end
                    
                 end else begin //If done, go to STATE_IDLE
                    curr_state <= STATE_IDLE;
                    counter_fsm <= 16'b0;
                 end
                 
            end
            STATE_READ_RRAM: begin
                if (counter_fsm == NUM_READ_SINGLE_ADDR_CYCLES-1) begin
                    if (counter_fsm <= (NUM_SL/NUM_ADC)-1) begin
                        //Select Columns one by one in 16 Cycles, Store ADC output into register
                        counter_fsm <= counter_fsm+1;
                    end else begin
                        //Pack outputs into 64b packets and send to oFIFO if not full.
                        counter_fsm <= counter_fsm+1;
                    end
                end else begin //If done, go to STATE_IDLE
                    curr_state <= STATE_IDLE;
                    counter_fsm <= 16'b0;
                end
            
            end
            STATE_STORE_HAM_WEIGHT: begin
            
                if (counter_fsm == NUM_STORE_HAM_WEIGHT_CYCLES-1) begin
                    //Pop dataFIFO, first load the 64b data and then write 64 columns for 2*255 cycles, Pop new data after erevry (1+2*NUM_WRITE_CYCLES) cycles
                    
                    if(counter_fsm % (1 + 2*NUM_WRITE_CYCLES) == 0) begin
                        //Pop New data from the FIFO
                        //Send in weights_reg_load command 
                        if (~empty_iFIFO) begin
                            pop_n_iFIFO <= 1'b0;
                            DATAIN <= dout_iFIFO;
                            counter_fsm <= counter_fsm+1;
                        end  
                    end else begin //counter_fsm % (1+2*NUM_WRITE_CYCLES) >= 1 && counter_fsm % (1+2*NUM_WRITE_CYCLES) <= 2*NUM_WRITE_CYCLES
                        //Write onto Selected Row and Selected Columns
                        //Send in program_rram command 
                        counter_fsm <= counter_fsm+1;
                    end
                    
                 end else begin //If done, go to STATE_IDLE
                    curr_state <= STATE_IDLE;
                    counter_fsm <= 16'b0;
                 end
            
            end
            STATE_HAM_SEGMENT_COMPUTE: begin
                if (~PHD_READOUT_EN) begin //Compute Partial hamming Distance and store in PHD_ACC register
                    if(counter_fsm == NUM_COMPUTE_HD_CYCLES-1) begin
                        //pop iFIFO, fill input REGs, compute HD for ROW_BURST_SIZE_COMPUTE cycles and then again iFIFO
                        
                        if(counter_fsm % (ROW_BURST_SIZE_COMPUTE) == 0) begin
                        //Pop New data from the iFIFO
                        //Send in inputs_reg_load command 
                        if (~empty_iFIFO) begin
                            pop_n_iFIFO <= 1'b0;
                            DATAIN <= dout_iFIFO;
                            counter_fsm <= counter_fsm+1;
                        end  
                    end else begin //counter_fsm % (1+2*NUM_WRITE_CYCLES) >= 1 && counter_fsm % (1+2*NUM_WRITE_CYCLES) <= 2*NUM_WRITE_CYCLES
                        //Write onto Selected Row and Selected Columns
                        //Send in program_rram command 
                        counter_fsm <= counter_fsm+1;
                    end
                    end else begin //If done, go to STATE_IDLE
                        curr_state <= STATE_IDLE;
                        counter_fsm <= 16'b0;
                 end
                end else begin  //READOUT the outputs from ACC Register
                    if(counter_fsm == NUM_READOUT_PHD_CYCLES-1) begin
                        //Readout 16b PHD ACC value from 32 Classes and push to oFIFO
                        if (~full_oFIFO) begin
                            push_n_oFIFO <= 1'b0;
                            //din_oFIFO //prepare din_oFIFO
                        end
                    end else begin //If done, go to STATE_IDLE
                        curr_state <= STATE_IDLE;
                        counter_fsm <= 16'b0;
                    end
                
                end
            end
            default:begin
                curr_state <= STATE_RESET;
            end
        endcase
    end
    
    logic [INSTR_WIDTH-1:0] LOCAL_INSTR;
    logic [LOCAL_OPCODE_WIDTH-1:0] LOCAL_OPCODE;
    logic [DATAIN_WIDTH-1:0] LOCAL_DATAIN;
    
    //Replacing the OpCodes from rram_controller to local registers/variables
    // LOCAL_INSTR (0001) : Write Weights, 9b (START ADDR for Weight Register), We can write 64 registers at once, only those selected and rest COLSELb HIGH.
    // LOCAL_INSTR (0011) : Program RRAM, OpCode:- 1b (ROW_POL) + 1b (COL_POL) + 10b (ADDR to write)
    // LOCAL_INSTR (0100) : Read Single Device, OpCode:-  10b (ADDR to read)
    // LOCAL_INSTR (0101) : Write Inputs (Load WL Registers),  10b (START ADDR for WL Register)
    // LOCAL_INSTR (0110) : Read Multiple Devices i.e do MVM, OpCode:- 6b(Segment Width/Number of Rows to read) + 10b (START ADDR for WL)
    // LOCAL_INSTR (0111) : Read ADCoutput, OpCode:- 9b (ADCout Address to Read)
    // LOCAL_INSTR (1000) : MUX between SLs to pass to ADC, OpCode:- 4b (MUX_SEL)
    // LOCAL_INSTR (1001) : RESET all registers, 
    
    localparam [INSTR_WIDTH-1:0] CMD_WRITE_WEIGHTS    = 4'd0;     // Write Weights, 10b (START ADDR for BL/SL Register), We can write 64 egisters at once
    localparam [INSTR_WIDTH-1:0] CMD_PROGRAM_DEVICE   = 4'd1;     // Program RRAM, OpCode:- 8b (Number of Cycles to Write) + + 1b (ROW_POL) + 1b (COL_POL) + 10b (ADDR to write)
    localparam [INSTR_WIDTH-1:0] CMD_READ_SINGLE_ADDR = 4'd2;     // Read Single Device, OpCode:-  10b (ADDR to read)
    localparam [INSTR_WIDTH-1:0] CMD_WRITE_INPUTS     = 4'd3;     // Write Inputs (WL Registers),  10b (START ADDR for WL Register)
    localparam [INSTR_WIDTH-1:0] CMD_COMPUTE_MVM      = 4'd4;     // Read Multiple Devices i.e do MVM, OpCode:- 6b(Segment Width/Number of Rows to read, in multiple of 16) + 10b (START ADDR for WL)
    localparam [INSTR_WIDTH-1:0] CMD_READ_ADCOUT      = 4'd5;     // Read ADCoutput, OpCode:- 9b (ADCout Address to Read)
    localparam [INSTR_WIDTH-1:0] CMD_RESET_REGS       = 4'd7;     // Reset All Registers
    
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
        case (LOCAL_INSTR)
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
        
        case (LOCAL_INSTR)
            
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
                    
                    //Don't add any register for MUX_SEL since we want to MUX it on the same clock edge
                    for (int i=0; i < NUM_ADC;i++) begin
                        SL_MUX_SEL[i*(NUM_SL/NUM_ADC)+:16] = (1<<OPCODE[3:0]); //OPCODE[3:0] is the MUXSEL, (NUM_SL/NUM_ADC)=16
                    end
                end
                CMD_COMPUTE_MVM: begin
                   for (int i=0; i<NUM_SL;i++) begin
                        BL_UNGATED[i] = 3'b001; //BL selected to VBL+, BL selected to VBL-
                        SL_UNGATED[i] = 3'b000; //SL Left floating and the parasitic cap gets charged.
                    end
                    BL_BIAS_SEL = 3'b000;   //READ     
                    SL_BIAS_SEL = 1'b0;  //READ
                    
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
        
        case (LOCAL_INSTR)
            
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
