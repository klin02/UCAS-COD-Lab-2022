`include "turbo_macro.v"

module MEM_stage(
	input clk,
	input rst,
	
//Memory request channel
	output [31:0] Address,
	output        MemWrite,
	output [31:0] Write_data,
	output [ 3:0] Write_strb,
	output        MemRead,
	input         Mem_Req_Ready,

//Memory data response channel
	input  [31:0] Read_data,
	input         Read_data_Valid,
	output        Read_data_Ready,

//from EX
	input EX_to_MEM_valid,
	input [`EX_TO_MEM_BUS_WD-1 : 0] EX_to_MEM_bus,

//to EX	
	output MEM_ready,

//from WB
	input WB_ready,

//to WB
	output MEM_to_WB_valid,
	output [`MEM_TO_WB_BUS_WD-1 : 0] MEM_to_WB_bus,

//by-path forwarding data : to ID 
	output [`MEM_FW_BUS_WD-1 : 0] MEM_fw_bus,
	
//perf cnt
	output reg [31:0] 	MEM_visit_cnt,
	output reg [31:0] 	MEM_delay_cnt
);

//state register
	reg [4:0]	MEM_cur_state;
	reg [4:0] 	MEM_next_state;

//control signal
	reg 	MEM_work;
	wire 	MEM_done;
	reg  	MEM_to_WB_valid_hold;
	reg [`EX_TO_MEM_BUS_WD-1 : 0]	EX_to_MEM_bus_reg;

//Signal connected to MEM read
	reg [31:0]	Read_data_reg;
	
//other Signal related to EX bus
	wire [31:0] 	MEM_PC;
	wire 		load;
	wire 		store;
	wire [1:0]	load_tag;
	wire [2:0] 	funct3;
	wire 		RF_wen;
	wire [4:0] 	RF_waddr;
	wire 		RF_EX_alter;
	wire [31:0]	RF_EX_alter_data;
	
//Signal for RF
	wire [31:0] RF_wdata;
	
	wire [31:0] load_data;
	wire [31:0] lb_data;  //sign_extend
	wire [31:0] lh_data;
	wire [31:0] lw_data;
	wire [31:0] lbu_data; //zero_extend
	wire [31:0] lhu_data;
	
//state machine : state shift only when work
	always @(posedge clk) begin
		if(rst)
			MEM_cur_state <= `RST;
		else 
			MEM_cur_state <= MEM_next_state;
	end
	
	always @(*) begin
		case(MEM_cur_state)
			`RST: begin
				MEM_next_state = `BSL;
			end
		
			`BSL: begin
				if(MEM_work) begin
					if(load | store)
						MEM_next_state = `SL;
					else 
						MEM_next_state = `SLD;
				end
				else
					MEM_next_state = `BSL;
			end
			
			`SL: begin
				if(MEM_work) begin	
					if(load & Mem_Req_Ready)
						MEM_next_state = `RDW;
					else if(store & Mem_Req_Ready )
						MEM_next_state = `SLD;
					else 
						MEM_next_state = `SL;
				end
				else
					MEM_next_state = `SL;
			end
			
			`RDW: begin
				if(Read_data_Ready & Read_data_Valid)
					MEM_next_state = `SLD;
				else
					MEM_next_state = `RDW;
			end
			
			`SLD: begin
				if(EX_to_MEM_valid) //WB ready
					MEM_next_state = `BSL;
				else
					MEM_next_state = `SLD;
			end
			
			default: begin
				MEM_next_state = `RST;
			end
		endcase
	end

//control Unit
	always @(posedge clk) begin
		if(rst)
			EX_to_MEM_bus_reg <= 146'b0;
		else begin
			if(EX_to_MEM_valid & MEM_ready)
				EX_to_MEM_bus_reg <= EX_to_MEM_bus;
		end
	end
	
	always @(posedge clk) begin
		if(rst)
			MEM_work <= 1'b0;
		else begin
			if(MEM_ready)
				MEM_work <= EX_to_MEM_valid;
		end
	end
	
	always @(posedge clk) begin
		if(rst)
			MEM_to_WB_valid_hold <= 1'b0;
		else begin
			if(EX_to_MEM_valid)
				MEM_to_WB_valid_hold <= 1'b1;
			else if(MEM_to_WB_valid)
				MEM_to_WB_valid_hold <= 1'b0;
		end
	end

	assign MEM_done = MEM_cur_state == `SLD ;
	assign MEM_ready = ~MEM_work | (MEM_done & WB_ready);
	assign MEM_to_WB_valid = MEM_work & MEM_done & WB_ready & MEM_to_WB_valid_hold;
	
//MEM Unit
	assign Read_data_Ready = MEM_cur_state == `RDW | MEM_cur_state == `RST;
	
	always @(posedge clk) begin
		if(Read_data_Ready & Read_data_Valid)
			Read_data_reg <= Read_data;
	end
	
	assign MemRead = load & MEM_cur_state == `SL;
	assign MemWrite = store &  MEM_cur_state == `SL;
	
//Analyse EX bus
	assign {
		MEM_PC,			//145:114
		load,			//113:113
		store,			//112:112
		Address,		//111:80
		load_tag,		//79:78
		funct3,			//77:75
		Write_strb,		//74:71
		Write_data,		//70:39
		RF_wen,			//38:38
		RF_waddr,		//37:33
		RF_EX_alter,		//32:32
		RF_EX_alter_data	//31:0
		} = EX_to_MEM_bus_reg;
	
	assign lb_data 	= ({ 32{~load_tag[1] & ~load_tag[0]} } & { {24{Read_data_reg[ 7]}} , Read_data_reg[ 7: 0] } )	|
			  ({ 32{~load_tag[1] &  load_tag[0]} } & { {24{Read_data_reg[15]}} , Read_data_reg[15: 8] } )	|
			  ({ 32{ load_tag[1] & ~load_tag[0]} } & { {24{Read_data_reg[23]}} , Read_data_reg[23:16] } )	|
			  ({ 32{ load_tag[1] &  load_tag[0]} } & { {24{Read_data_reg[31]}} , Read_data_reg[31:24] } )	;
	assign lh_data 	= ({ 32{~load_tag[1]} } & { {16{Read_data_reg[15]}} , Read_data_reg[15: 0] } )	|
			  ({ 32{ load_tag[1]} } & { {16{Read_data_reg[31]}} , Read_data_reg[31:16] } )   ;
	assign lw_data 	= Read_data_reg ;
	assign lbu_data = {24'b0 , lb_data[ 7:0]} ;
	assign lhu_data = {16'b0 , lh_data[15:0]} ;

	assign load_data = ( {32{funct3 == 3'b000}} &  lb_data )	|
			   ( {32{funct3 == 3'b001}} &  lh_data )	|
			   ( {32{funct3 == 3'b010}} &  lw_data )	|
			   ( {32{funct3 == 3'b100}} & lbu_data )	|
			   ( {32{funct3 == 3'b101}} & lhu_data )	;
					   
	assign RF_wdata = load 		? load_data 		: 
			  RF_EX_alter 	? RF_EX_alter_data	:
			  0;//default
	
//send WB bus
	assign MEM_to_WB_bus = {
				MEM_PC,			//69:38
				RF_wen,			//37:37
				RF_waddr,		//36:32
				RF_wdata		//31:0
				};
//forwarding data: to ID
	//load done valid addr data
	assign MEM_fw_bus = {load , MEM_done , MEM_work & RF_wen , RF_waddr , RF_wdata};

//performance cnt
	always @(posedge clk)begin
		if(rst)
			MEM_visit_cnt <= 32'b0;
		else if(MEM_work & MEM_cur_state == `SL)
			MEM_visit_cnt <= MEM_visit_cnt +1;
	end
	
	always @(posedge clk) begin
		if(rst)
			MEM_delay_cnt <= 32'b0;
		else if((MEM_cur_state == `SL & ~Mem_Req_Ready)|(MEM_cur_state==`RDW & ~Read_data_Valid))
			MEM_delay_cnt <= MEM_delay_cnt +1;
	end
endmodule