`include "turbo_macro.v"
	
module EX_stage(
	input 	clk,
	input 	rst,
	
//overall 
	output cancel,
	
//to IF
	output [31:0] real_PC,
	
//from ID
	input	ID_to_EX_valid,
	input [`ID_TO_EX_BUS_WD-1 : 0] ID_to_EX_bus,
//to ID	
	output	EX_ready,
	
//from MEM
	input 	MEM_ready,
//to MEM
	output 	EX_to_MEM_valid,
	output [`EX_TO_MEM_BUS_WD-1: 0 ] EX_to_MEM_bus,
	
//by-path forwarding data : to ID
	output [`EX_FW_BUS_WD-1 : 0] EX_fw_bus,
	
//perf cnt
	output reg [31:0] BorJ_v_cnt,
	output reg [31:0] Cancel_cnt
);

	wire [31:0]	EX_PC;
	reg 	EX_work;
	wire 	EX_done;
	
	reg [`ID_TO_EX_BUS_WD-1 : 0] ID_to_EX_bus_reg;
	
//Signals connected to ALU
	wire [31:0] 	ALU_A;
	wire [31:0] 	ALU_B;
	wire [2:0]  	ALU_op;
	wire 		ALU_Overflow;
	wire 		ALU_CarryOut;
	wire 		ALU_Zero;
	wire [31:0] 	ALU_Result;

//Signals connected to Shifter
	wire [31:0] 	Shifter_A;
	wire [ 4:0] 	Shifter_B;
	wire [1:0]  	Shifter_op;
	wire [31:0] 	Shifter_Result;
	
//other Signal from ID bus
	wire prdt_go;
	wire [31:0] prdt_tar;
	wire B_type;
	wire R_Icalc;
	wire load;
	wire store;
	wire jump;
	wire [2:0] 	funct3;
	wire RF_wen;
	wire [4:0] 	RF_waddr;
	wire RF_ID_alter;
	wire [31:0]	RF_ID_alter_data;
	wire [31:0] 	rs2_value;
	
//Signal appended for RF
	wire RF_EX_alter;  //last 4 kind, add R_Icalc to RF_ID_alter
	wire [31:0] 	RF_EX_alter_data;
	
//Siganl testing prediction
	wire br_en;
	wire real_go;	//real go =?> cancel
	reg  cancel_hold;

//Signals connected to mem load and write
	wire [31:0] 	Address;
	
	wire [1:0]	load_tag;	//ALU_re[1:0]
	
	wire [3:0]	sb_strb;
	wire [3:0]	sh_strb;
	wire [3:0]	sw_strb;
	wire [3:0]	Write_strb;
	
	wire [31:0] 	sb_data;	//take the least significant 8 bit
	wire [31:0] 	sh_data;
	wire [31:0] 	sw_data;
	wire [31:0] 	Write_data;
	
	
//control Unit
	always @(posedge clk) begin
		if(rst)
			ID_to_EX_bus_reg <= 248'b0;
		else begin
			if(ID_to_EX_valid & EX_ready)
				ID_to_EX_bus_reg <= ID_to_EX_bus;
		end
	end
	
	always @(posedge clk) begin
		if(rst)
			EX_work <= 1'b0;
		else begin
			if(EX_ready)
				EX_work <= ID_to_EX_valid;
		end
	end
	
	assign EX_done = 1'b1; //always done in one cycle
	assign EX_ready = ~EX_work | (EX_done & MEM_ready);
	assign EX_to_MEM_valid = EX_work & EX_done & MEM_ready;
	
//Analyse ID bus
	assign {
			EX_PC,			//247:216
			prdt_go,		//215:215
	 	    	prdt_tar,		//214:183
			ALU_op,			//182:182
			ALU_A,			//181:150
			ALU_B,			//149:118
			Shifter_op,		//117:116
			Shifter_A,		//115:84
			Shifter_B,		//83:79
			B_type,			//78:78
			R_Icalc,		//77:77
			load,			//76:76
			store,			//75:75
			jump,			//74:74
			funct3,			//73:71
			RF_wen,			//70:70
			RF_waddr,		//69:65
			RF_ID_alter,		//64:64
			RF_ID_alter_data, 	//63:32
			rs2_value		//31:0
			} = ID_to_EX_bus_reg;
	
	alu alu_inst(
	   .A(ALU_A),
	   .B(ALU_B),
	   .ALUop(ALU_op),
	   .Overflow(ALU_Overflow),
	   .CarryOut(ALU_CarryOut),
	   .Zero(ALU_Zero),
	   .Result(ALU_Result)
	);
	shifter shifter_inst(
	   .A(Shifter_A),
	   .B(Shifter_B),
	   .Shiftop(Shifter_op),
	   .Result(Shifter_Result)
	);
	
//MEM_related signals:
	assign Address 	= {ALU_Result[31:2], 2'b0};
	assign load_tag = ALU_Result[1:0];
	
	assign sb_strb 	= { ALU_Result[1]& ALU_Result[0] , ALU_Result[1]& ~ALU_Result[0] , ~ALU_Result[1]& ALU_Result[0] , ~ALU_Result[1]& ~ALU_Result[0]};
	assign sh_strb 	= { {2{ALU_Result[1]}} , {2{~ALU_Result[1]}} } ;
	assign sw_strb 	= 4'b1111 ;
	assign Write_strb = 	( {4{funct3 == 3'b000}} & sb_strb )	|
				( {4{funct3 == 3'b001}} & sh_strb )	|
				( {4{funct3 == 3'b010}} & sw_strb )	;
	
	assign sb_data	= { 4{rs2_value[7:0]} } ;
	assign sh_data  = { 2{rs2_value[15:0]}}	;
	assign sw_data	= rs2_value ; 
	assign Write_data = 	( {32{funct3 == 3'b000}} & sb_data )	|
				( {32{funct3 == 3'b001}} & sh_data )	|
				( {32{funct3 == 3'b010}} & sw_data )	;

//Append RF_alter to 4 kind
	assign RF_EX_alter = R_Icalc | RF_ID_alter ;
	assign RF_EX_alter_data = 	R_Icalc ? (funct3==3'b001 | funct3 == 3'b101 ? Shifter_Result : ALU_Result) : 
					RF_ID_alter ? RF_ID_alter_data :
					0;//default

//Test predict result , cancel if fail
	assign br_en = B_type & ( ( ~funct3[2] & ~funct3[0] &  ALU_Zero ) 	| 
				  ( ~funct3[2] &  funct3[0] & ~ALU_Zero) 	|
				  (  funct3[2] & ~funct3[0] &  ALU_Result[0]) 	|
				  (  funct3[2] &  funct3[0] & ~ALU_Result[0])
				);
	assign real_go = br_en | jump;

	always @(posedge clk) begin
		if(ID_to_EX_valid)
			cancel_hold <= 1'b1;
		else 
			cancel_hold <= 1'b0;
	end
	assign cancel = ( real_go ^ prdt_go ) & cancel_hold;	//set high if not equal
	assign real_PC = real_go ? prdt_tar : EX_PC +4;
	
//send MEM bus
	assign EX_to_MEM_bus = {
				EX_PC,			//145:114
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
				};
							
//to ID forwarding data
	//load valid addr data
	//note that when OP is load, IF will block until MEM send valid data
	//no need to care loaddata here
	assign EX_fw_bus = {load, EX_work & RF_wen, RF_waddr, RF_EX_alter_data};

//performance cnt
	always @(posedge clk) begin
		if(rst)
			BorJ_v_cnt <= 32'b0;
		else if(br_en | jump)
			BorJ_v_cnt <= BorJ_v_cnt +1;
	end
	
	always @(posedge clk) begin
		if(rst)
			Cancel_cnt <= 32'b0;
		else begin
			if(cancel)
				Cancel_cnt <= Cancel_cnt + 1;
		end
	end
endmodule
	