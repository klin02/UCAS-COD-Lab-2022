`include "turbo_macro.v"

module ID_stage(
	input 	clk,
	input 	rst,
//overall
	input 	cancel,

//from IF
	input 	IF_to_ID_valid,
	input [`IF_TO_ID_BUS_WD-1 : 0] 	IF_to_ID_bus,
//to IF
	output 	ID_ready,
	output [`PRDT_BUS_WD-1 : 0] prdt_bus,
	
//from EXF
	input 	EX_ready,
//to EX
	output 	ID_to_EX_valid,
	output [`ID_TO_EX_BUS_WD-1 : 0] ID_to_EX_bus,
		
//from WB: pass to reg_file
	input [`WB_TO_RF_BUS_WD-1 : 0] 	WB_to_RF_bus,
	
//from EX,MEM,WB: by-path forwarding data
	input [`EX_FW_BUS_WD-1 : 0] 	EX_fw_bus,
	input [`MEM_FW_BUS_WD-1 : 0]	MEM_fw_bus,
	input [`WB_FW_BUS_WD-1 : 0]	WB_fw_bus,

//perf cnt
	output reg [31:0]	Branch_cnt,
	output reg [31:0]	Jump_cnt,
	output reg [31:0]	Prdt_go_cnt
);
	
	reg  [`IF_TO_ID_BUS_WD-1 : 0]  IF_to_ID_bus_reg;
	wire [31:0] ID_PC;
	wire [31:0] ID_inst;
	
//Signals related to Inst
	wire [ 6:0] opcode;
	wire [ 4:0] rd;
	wire [ 4:0] rs1;
	wire [ 4:0] rs2;
	wire [ 4:0] shamt;
	wire [ 2:0] funct3;
	wire [ 6:0] funct7;
	wire [31:0] I_imm;
	wire [31:0] S_imm;
	wire [31:0] B_imm;
	wire [31:0] U_imm;
	wire [31:0] J_imm;
	wire [31:0] jalr_tar; //sum of rs1 and I_imm

//Signals connected to type_diff
	wire 	R_type;
	wire	I_calc;
	wire 	I_load;
	wire 	I_jalr;
	wire 	I_type;
	wire 	S_type;
	wire	B_type;
	wire 	U_lui;
	wire 	U_auipc;
	wire 	U_type;
	wire 	J_type;

//Signals connected to ALU
	wire [31:0] ALU_A;
	wire [31:0] ALU_B;
	wire [2:0]  ALU_op;

//Signals connected to Shifter
	wire [31:0] Shifter_A;
	wire [ 4:0] Shifter_B;
	wire [1:0]  Shifter_op;
	
//Signals connected to reg_file
	wire [4:0]	RF_raddr1;
	wire [4:0]	RF_raddr2;
	wire [31:0] 	RF_rdata1;
	wire [31:0] 	RF_rdata2;
	wire  		RF_wen;
	wire [4:0] 	RF_waddr;
	wire 	  	WB_RF_wen;
	wire [4:0] 	WB_RF_waddr;
	wire [31:0] 	WB_RF_wdata;
	
//Signal for pipeline changing:  Priority : EX > MEM > WB > ID
	wire [31:0] rs1_value;
	wire [31:0] rs2_value;

//forwarding data from EX,MEM,WB
	//signal relate to block of load
	wire EX_load;
	wire MEM_load;
	wire MEM_done;//block end
	
	wire EX_valid;
	wire MEM_valid;
	wire WB_valid;
	
	wire [4:0]	EX_addr;
	wire [4:0]	MEM_addr;
	wire [4:0]	WB_addr;
	
	wire [31:0]	EX_data;
	wire [31:0]	MEM_data;
	wire [31:0] 	WB_data;
	
//control signal of subsequent modules
	wire R_Icalc;
	wire load;
	wire store;
	wire jump;
	wire RF_ID_alter;  //include last 3 kind of RF_wen
	wire [31:0] RF_ID_alter_data;  
	
//predict signal
	//note that prdt aim to branch
	wire prdt_br;
	wire prdt_go;
	wire[31:0] prdt_tar;	//note that j_tar cannot use alu	
	
//Block signal
	wire block;
	wire EX_related;
	wire MEM_related;	
	wire ID_done;
	reg  ID_work; 	//show whether ID stage really work
	
	
	always @(posedge clk)begin
		if(rst)
			IF_to_ID_bus_reg <= 64'b0;
		else begin
			if(IF_to_ID_valid & ID_ready)
				IF_to_ID_bus_reg <= IF_to_ID_bus;
		end
	end
	
	assign {ID_PC , ID_inst} = IF_to_ID_bus_reg;
	
//Analyse Instruction code
	assign opcode 	= ID_inst[6:0];
	assign rd	= ID_inst[11:7];
	assign rs1	= ID_inst[19:15];
	assign rs2	= ID_inst[24:20];
	assign shamt	= ID_inst[24:20];
	assign funct3	= ID_inst[14:12];
	assign funct7	= ID_inst[31:25];
	assign I_imm[11:0]	= ID_inst[31:20];
	assign I_imm[31:12] 	= funct3 == 3'b011 ? {20'b0} : {20{ID_inst[31]}};
	assign S_imm	= { {20{ID_inst[31]}}, ID_inst[31:25], ID_inst[11:7]};
	assign B_imm	= { {19{ID_inst[31]}}, ID_inst[31], ID_inst[7], ID_inst[30:25], ID_inst[11:8], 1'b0};
	assign U_imm 	= { ID_inst[31:12],12'b0};
	assign J_imm	= { {11{ID_inst[31]}}, ID_inst[31], ID_inst[19:12], ID_inst[20], ID_inst[30:21], 1'b0};
	
//Differ type by one-bit signals	
	assign	R_type	= opcode == 7'b0110011;
	assign 	I_calc	= opcode == 7'b0010011;
	assign 	I_load	= opcode == 7'b0000011;
	assign 	I_jalr	= opcode == 7'b1100111;
	assign 	I_type	= I_calc | I_load | I_jalr;
	assign	S_type	= opcode == 7'b0100011;
	assign 	B_type	= opcode == 7'b1100011;
	assign	U_lui	= opcode == 7'b0110111;
	assign 	U_auipc	= opcode == 7'b0010111;
	assign	U_type	= U_lui | U_auipc;
	assign	J_type	= opcode == 7'b1101111;
	
//Channel to ALU (EX_stage)  : replace RF_rdata with rs_value
	//note that I_jalr and J_type use result in ID, not longer use alu
	assign ALU_op = R_type & ~(funct3==3'b001 | funct3 == 3'b101) ? ( funct3 == 3'b000 	? {funct7[5],2'b10} 	:
									  funct3[2:1] == 2'b01 	? {~funct3[0],2'b11}	:
									  funct3 == 3'b100	? funct3 		:
									  funct3[2:1] == 2'b11	? ~funct3		:
									  0 ):
			I_calc & ~(funct3==3'b001 | funct3 == 3'b101) ? ( funct3 == 3'b000	? 3'b010		:
									  funct3[2:1] == 2'b01 	? {~funct3[0],2'b11}	:
									  funct3 == 3'b100	? funct3 		:
									  funct3[2:1] == 2'b11	? ~funct3		:
									  0 ): 
			I_load | S_type | U_auipc		      ?  3'b010 : 
			B_type 					      ? ( funct3[2:1]==2'b00 ? 3'b110 : {~funct3[1], 2'b11}) :
			0;//default
	assign ALU_A = U_auipc ? ID_PC : rs1_value;
	assign ALU_B =  (I_calc & ~(funct3==3'b001 | funct3 == 3'b101)) | I_load | I_jalr ? I_imm :
			S_type								  ? S_imm : 
			U_auipc								  ? U_imm : 
			J_type								  ? J_imm :
			rs2_value;

//Channel to Shifter (EX_stage)
	assign Shifter_op 	= (R_type | I_calc) & (funct3==3'b001 | funct3 == 3'b101) ? {funct3[2],funct7[5]} : 0;
	assign Shifter_A 	= rs1_value;
	assign Shifter_B	= R_type ? rs2_value[4:0] : shamt;
	
//register value considering pipeline
	assign {EX_load , EX_valid , EX_addr , EX_data} = EX_fw_bus;
	assign {MEM_load , MEM_done , MEM_valid , MEM_addr , MEM_data} = MEM_fw_bus;
	assign {WB_valid , WB_addr , WB_data} = WB_fw_bus;
	
	assign rs1_value =  ~(|rs1) 			? 32'b0 	:
			EX_valid  & (EX_addr  == rs1) 	? EX_data  	:
			MEM_valid & (MEM_addr == rs1) 	? MEM_data 	:
			WB_valid  & (WB_addr  == rs1)	? WB_data  	:
			RF_rdata1;
	assign rs2_value =  ~(|rs2) 			? 32'b0 	:
			EX_valid  & (EX_addr  == rs2) 	? EX_data  	:
			MEM_valid & (MEM_addr == rs2) 	? MEM_data 	:
			WB_valid  & (WB_addr  == rs2)	? WB_data  	:
			RF_rdata2;					
	
//Control Unit
	assign EX_related = (|EX_addr) & EX_valid & (rs1==EX_addr | rs2==EX_addr);
	assign MEM_related = (|MEM_addr) & MEM_valid & (rs1==MEM_addr | rs2==MEM_addr);
	
	assign block 	= ( EX_load & EX_related ) | (MEM_load & MEM_related & ~MEM_done);
	assign ID_done  = ~block & ~cancel;
	
	always @(posedge clk) begin
		if(rst) 
			ID_work <= 1'b0;
		else begin
			if(cancel)
				ID_work <= 1'b0;
			else if(ID_ready)
				ID_work <= IF_to_ID_valid;
		end
	end
	
	assign ID_ready = ~ID_work | (ID_done & EX_ready);
	assign ID_to_EX_valid = ID_work & ID_done & EX_ready;
	
//to EX bus
	//send rs2_value to get Write_data later
	assign R_Icalc  = R_type | I_calc;
	assign load 	= I_load;
	assign store 	= S_type;
	assign jump 	= I_jalr | J_type;
	assign RF_wen 	= R_type | I_type | J_type | U_type;
	assign RF_waddr = rd;
	assign RF_ID_alter = (I_jalr | J_type) | U_lui | U_auipc ;
	assign RF_ID_alter_data = 	I_jalr | J_type ? ID_PC +4 :
					U_lui 		? U_imm		:
					U_auipc		? ID_PC + U_imm :
					0; //default
	assign ID_to_EX_bus = {
				  ID_PC,		//247:216
				  prdt_go,		//215:215
				  prdt_tar,		//214:183
				  ALU_op,		//182:182
				  ALU_A,		//181:150
				  ALU_B,		//149:118
				  Shifter_op,		//117:116
				  Shifter_A,		//115:84
				  Shifter_B,		//83:79
				  B_type,		//78:78
				  R_Icalc,		//77:77
				  load,			//76:76
				  store,		//75:75
				  jump,			//74:74
				  funct3,		//73:71
				  RF_wen,		//70:70
				  RF_waddr,		//69:65
				  RF_ID_alter,		//64:64
				  RF_ID_alter_data,  	//63:32
				  rs2_value		//31:0
				  };

//Channel to regfile
	//read data from ID, write data from WB
	assign RF_raddr1 = rs1;
	assign RF_raddr2 = rs2;
	assign {WB_RF_wen , WB_RF_waddr , WB_RF_wdata} = WB_to_RF_bus;
	
	reg_file reg_file_inst(
		.clk(clk),
		.raddr1(RF_raddr1),
		.raddr2(RF_raddr2),
		.rdata1(RF_rdata1),
		.rdata2(RF_rdata2),
		.waddr(WB_RF_waddr),
		.wdata(WB_RF_wdata),
		.wen(WB_RF_wen)
	);	
	
//predict Unit
	assign prdt_go  = (I_jalr | J_type) | (B_type & prdt_br);
	assign jalr_tar = rs1_value + I_imm;
	assign prdt_tar = 	B_type ? ID_PC + B_imm :
				J_type ? ID_PC + J_imm :
				I_jalr ? {jalr_tar[31:1] ,1'b0} : 
				0; //default
	assign prdt_bus = {prdt_go , prdt_tar};
	predictor prdt_inst (
		.clk		(clk),
		.rst 		(rst),
		.Branch		(B_type),
		.cancel 	(cancel),
		.prdt_br 	(prdt_br)
	);
	
//performance cnt
	always @(posedge clk) begin
		if(rst)
			Branch_cnt <= 32'b0;
		else if (B_type)
			Branch_cnt <= Branch_cnt + 1;
	end
	
	always @(posedge clk) begin
		if(rst)
			Jump_cnt <= 32'b0;
		else if(I_jalr | J_type)
			Jump_cnt <= Jump_cnt +1 ;
	end

	always @(posedge clk) begin
		if(rst)
			Prdt_go_cnt <= 32'b0;
		else begin
			if(prdt_go)
				Prdt_go_cnt <= Prdt_go_cnt + 1;
		end
	end
endmodule