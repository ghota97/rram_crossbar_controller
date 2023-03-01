// Code your design here
module rram_crossbar (CLK, CLK_ADC, RESET, WL, BL, ADCSEL, ADCout, WREN, RDEN);
	parameter NUM_ADCs = 32;
	input CLK, RESET;
	input  CLK_ADC;
  
	input  [1023:0] WL;
	input  [1023:0] BL;
    input  WREN;
    input  RDEN;
	
	input  [3:0] ADCSEL; //32 Columns share 0ne ADC
	output reg [3:0] ADCout[NUM_ADCs-1:0];

	logic [3:0] SL_ACC[511:0] ;  //Register for Storing SL outputs, Removed from the input list to keep the RRAM model simple.
	
	logic [1023:0] rram_mem [1023:0];
	  
	logic [9:0] selected_WL;

	//Decoding the row position from the WL vector to write weights or reading out weights
	always_comb begin
         for (int i=0; i<1024;i++) begin
			if(WL[i]) begin
				selected_WL = i;
			end
		 end
	end
	 
	always @(posedge CLK) begin
      if(RESET) begin
        for (int i=0; i<1024;i++) begin
              rram_mem[i] <= 1024'b0;
        end
        for (int i=0; i<512;i++) begin
              SL_ACC[i] <= 4'b0;
        end
      end
      else begin
        if(WREN) rram_mem[selected_WL] <= BL;
		if(RDEN) begin
          for (int i=0; i<512;i++) begin //SL loop
					SL_ACC[i] <= {3'b0, rram_mem[selected_WL][2*i]};  //Reading out single row
		  end
		end
      end
	end
	 
	always @(posedge CLK_ADC) begin
	  if(RESET) begin
	       for (int i=0; i<NUM_ADCs;i++) begin
                ADCout[i] <= 4'b0;
           end
	  end else begin
            for (int i=0; i<NUM_ADCs;i++) begin
			     ADCout[i] <= SL_ACC[16*i+ADCSEL];
			end
	  end
	end
endmodule