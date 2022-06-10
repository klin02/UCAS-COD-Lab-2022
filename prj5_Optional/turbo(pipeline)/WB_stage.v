`include "turbo_macro.v"

module WB_stage (
	input clk,
	input rst,
	
//from MEM
	input MEM_to_WB_valid,
	input [`MEM_TO_WB_BUS_WD-1 : 0] MEM_to_WB_bus,
//to MEM
	output WB_ready,
	
//to RF: pass data through ID
	output [`WB_TO_RF_BUS_WD-1 : 0] WB_to_RF_bus,
	
//by-path forwarding data : to ID
	output [`WB_FW_BUS_WD-1 : 0]	WB_fw_bus,

	output [69:0] inst_retire,
	
//perf cnt
	output reg [31:0] Valid_Inst_cnt
);

//state register
	reg [4:0] WB_cur_state;
	reg [4:0] WB_next_state;
	
//control signals
	reg 	WB_work;
	wire	WB_done;
	
	reg [`MEM_TO_WB_BUS_WD-1 : 0]  MEM_to_WB_bus_reg;
	
//signals related to MEM bus
	wire [31:0]	WB_PC;
	wire 		WB_RF_wen;
	wire [4:0]	RF_waddr;
	wire [31:0]	RF_wdata;
	
        wire            inst_retire_valid;      //the inst retire is valid
	wire 		inst_retire_wen;        //the inst retire is valid and need to WB
	
//control Unit
	always @(posedge clk) begin
		if(rst)
			MEM_to_WB_bus_reg <= 70'b0;
		else begin
			if(MEM_to_WB_valid & WB_ready)
				MEM_to_WB_bus_reg <= MEM_to_WB_bus;
		end
	end
	
	always @(posedge clk) begin
		if(rst)
			WB_work <= 1'b0;
		else begin
			if(WB_ready)
				WB_work <= MEM_to_WB_valid;
		end
	end
	
	assign WB_done = 1'b1;  //always done in one clk
	assign WB_ready = ~WB_work | WB_done;

//Analyse MEM bus
	assign {WB_PC , WB_RF_wen , RF_waddr , RF_wdata} = MEM_to_WB_bus_reg;
	
//to RF: pass data
	assign WB_to_RF_bus = {WB_RF_wen & WB_work , RF_waddr , RF_wdata};
	//CHECK whether WB_work is needed!!
	
//by-path forwarding data: to IF
	assign WB_fw_bus = { WB_RF_wen & WB_work , RF_waddr , RF_wdata};
	
//inst retire
	//hold inst_retire_valid to one cycle to count valid inst
        //hold inst_retire_wen to one cycle to avoid repeated comparison
	assign inst_retire_valid = WB_done & WB_work ;
        assign inst_retire_wen = inst_retire_valid & WB_RF_wen ;
	assign inst_retire = {inst_retire_wen , RF_waddr , RF_wdata,WB_PC};

//performance cnt
	always @(posedge clk) begin
		if(rst) 
			Valid_Inst_cnt <= 32'b0;
		else if(inst_retire_valid)
			Valid_Inst_cnt <= Valid_Inst_cnt +1;
	end
	
endmodule
