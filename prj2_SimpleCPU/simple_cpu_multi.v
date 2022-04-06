`timescale 10ns / 1ns

module simple_cpu(
	input             clk,
	input             rst,

	output reg [31:0]     PC,
	input  [31:0]     Instruction,

	output [31:0]     Address,
	output            MemWrite,
	output [31:0]     Write_data,
	output [ 3:0]     Write_strb,

	input  [31:0]     Read_data,
	output            MemRead
);

	// TODO: PLEASE ADD YOUR CODE BELOW
//define the state of machine by one-hot
	localparam 	RST	=9'b000000001,
			IF	=9'b000000010,
			ID	=9'b000000100,
			EX	=9'b000001000,
			MEM	=9'b000010000,
			WB	=9'b000100000;
//set reg to store state and single_cycle input
	reg [ 8:0] 	current_state;
	reg [ 8:0] 	next_state;
	reg [31:0] 	Instruction_Reg;
	reg [31:0]	Read_data_Reg;
	reg [31:0]	PC_Reg;
	//PC refresh in EX, set a reg to store origin PC
	
//Signals related to IF
	wire [ 5:0] opcode;
	wire [ 4:0] rs;
	wire [ 4:0] rt;
	wire [ 4:0] rd;
	wire [ 4:0] sa;
	wire [ 5:0] func;
	wire [15:0] imm;
	wire [25:0] index;
	
//Signals related to PC
	//wire [31:0] PC4;
	//wire [31:0] PC8;
	//Using ALU_Result instead
	
//Signals related to extension
	wire [31:0] zero_extension;	//use in ALU
	wire [31:0] sign_extension;	//use in ALU
	wire [31:0] br_extension;	//use in PC refresh
	wire [31:0] j_extension; 	//use in PC refresh
	
//Signals connected to ALU
	wire [31:0] ALU_A;
	wire [31:0] ALU_B;
	wire [ 2:0] ALU_op;
	wire	    ALU_Overflow;
	wire 	    ALU_CarryOut;
	wire 	    ALU_Zero;
	wire [31:0] ALU_Result;

//Signals connected to Shifter
	wire [31:0] Shifter_A;
	wire [ 4:0] Shifter_B;
	wire [ 1:0] Shifter_op;
	wire [31:0] Shifter_Result;

//Signals connected to Reg_file
	wire [ 4:0] 	RF_raddr1;
	wire [ 4:0] 	RF_raddr2;
	wire [31:0] 	RF_rdata1;
	wire [31:0] 	RF_rdata2;
	wire		RF_wen;         //USED IN TESTBENCE , DO NOT MODIFIED
	wire [ 4:0]	RF_waddr;	//USED IN TESTBENCE , DO NOT MODIFIED
	wire [31:0]	RF_wdata;       //USED IN TESTBENCE , DO NOT MODIFIED

//Signals connected to PC refresh
	wire 		br_en;		//branch condition
	wire [31:0]	br_tar;		//branch target
	wire 		j_en;
	wire [31:0]	j_tar;

//Signals connected to load and store
	wire [31:0]	load_data;
	wire [31:0]	lb_data;	//sign_extend
	wire [31:0]	lh_data;
	wire [31:0]	lw_data;
	wire [31:0]	lbu_data;	//zero_extend
	wire [31:0]	lhu_data;
	wire [31:0]	lwl_data;	//left mem, right rt, content depens on vaddr
	wire [31:0]	lwr_data;
	
	wire [ 3:0]	sb_strb;
	wire [ 3:0]	sh_strb;
	wire [ 3:0]	sw_strb;
	wire [ 3:0]	swl_strb;
	wire [ 3:0]	swr_strb;
	
	wire [31:0]	sb_data;	//take the least significant 8 bit
	wire [31:0]	sh_data;
	wire [31:0]	sw_data;
	wire [ 4:0] 	swl_shifter;
	wire [ 4:0] 	swr_shifter;
	//wire [31:0]	swl_data;	//not care undefined place, so can use shift in reverse dir
	//wire [31:0]	swr_data;
	//Using Shifter_Result instead
	
//State Machine
	//Part I: 
    always @ (posedge clk) begin
       if(rst) begin
           current_state <= IF;
       end
       else begin
           current_state <= next_state;
       end
    end

	//Part II: Combinatorial logic
	always @ (*)
	begin
		case(current_state)
			IF: begin
					next_state = ID ;
			end
			//NOP: Instr = 32'b0
			ID: begin
				if(~|Instruction_Reg)
					next_state = IF ;
				else 
					next_state = EX ;
			end
			//EX->IF:
				//REGIMM    2: opcode[5:0]=000001
				//I_branch  4: opcode[5:2]=0001
				//J	    1: opcode[5:0]=000010
			//EX->WB
				//R_Type   18: opcode[5:0]=6'b0 include JALR,JR //JR: rs->GPR[0] 
				//I_calc    7: opcode[5:3]=001 include LUI
				//JAL	    1: opcode[5:0]=000011
			//EX->MEM
				//load and store 12: opcode[5]=1
			EX: begin
				if((opcode[5:0]==6'b000001) | (opcode[5:2]==4'b0001) | (opcode[5:0]==6'b000010) )
					next_state = IF;
				else if((~|opcode)| (opcode[5:3]==3'b001) | (opcode[5:0]==6'b000011))
					next_state = WB;
				else if(opcode[5]==1'b1)
					next_state = MEM;
			end
			//MEM->IF: store: opcode[5:3]=101
			//MEM->WB: load : opcode[5:3]=100
			MEM: begin
				if(opcode[5:3]==3'b101)
					next_state = IF;
				else if(opcode[5:3]==3'b100)
					next_state = WB;
			end
			WB: begin
				next_state = IF;
			end
			default: begin
				next_state = IF;
			end
		endcase
	end
	
	//Part III: deal with output
	//one always module to deal one reg
	always @ (posedge clk) begin
		if(current_state[1]) //IF
			Instruction_Reg <= Instruction ;
	end
	
	always @ (posedge clk) begin
		if(current_state[4]) //MEM
			Read_data_Reg <=Read_data ;
	end
	
	always @ (posedge clk) begin
		if(current_state[1]) //IF
			PC_Reg <= PC;
	end
	
	always @ (posedge clk) begin
		if (rst) begin
			PC <= 32'b0;
		end
		else if(current_state[1] & ~rst) begin 
			PC <= ALU_Result; //all OP refresh in ID, judge by IF
		end
		else if(current_state[3] & ~rst &(|Instruction_Reg))begin 
			PC <= br_en ? br_tar : j_en ? j_tar : PC ; 
			//j or br OP refresh in next IF, judge by EX
			//note that the default result is PC
		end
	end
	
//Analyse Instruction code
	assign opcode	= Instruction_Reg[31:26];
	assign rs	= Instruction_Reg[25:21];
	assign rt	= Instruction_Reg[20:16];
	assign rd	= Instruction_Reg[15:11];
	assign sa	= Instruction_Reg[10:6];
	assign func	= Instruction_Reg[5:0];
	assign imm	= Instruction_Reg[15:0]; //or offset
	assign index	= Instruction_Reg[25:0]; 
	
//PC4 and PC8 prepared for PC refresh
//To save adder, reuse ALU
	//first PC refresh in ID use PC because PC_reg is delay
	//branch or j use PC_reg + 4 or PC because PC is refreshed  
	
	//PC4 = PC +4; 	//use PC because PC4 is delay until ID
	//PC8 = PC_Reg +8; //use PC_Reg because PC is refreshed in MEM
	
//extension prepared for operation
	assign zero_extension 	= {16'b0, imm};
	assign sign_extension 	= {{16{imm[15]}},imm};
	assign br_extension   	= {{14{imm[15]}},imm,2'b0};	//use in Branch
	assign j_extension	= {PC[31:28],index,2'b0};	//use in Jump CHECK PC4 OR PC!!!
	
//signals connected to ALU
	//IF stage: All instruction: PC+4
	//Other stage: To save adder, reuse ALU when PC+8 (swl swr)
	//All type below can differ from others
	//Operation related to ALU list as : Type (op_A op_B) num :   feature
		//R calc (rs rt)  8 : 		   	opcode[5:0]=000000 & func[5]=1
		//I calc (rs imm) 6 : 			opcode[5:3]=001 & (~&opcode[2:0]) diff from lui
		//REGIMM (rs 0)   2 :			opcode[5:0]=000001 BGEZ BLTZ   GPR[rs]?0   using SLT(sign)
		//Branch (rs rt)  2 :			opcode[5:1]=00010  
 		       //(rs 0)   2 : 	 		opcode[5:1]=00011  using SUB and test ZERO or signflag
		//load and store (rs imm) 12 :		opcode[5]=1 using ADD
			//base(rs)+offset(imm)
		//JAL (PC_reg 8) 1:			opcode[5:0]=000011
		//JALR (PC_reg 8) 1:			opcode[5:0]=000000 & func[5:1]=001001

	//Operation not related to ALU list: 
		//shifter 6 : 				opcode[5:0]=000000 & func[5:3]=000
		//jump    2 : J  			opcode[5:0]=000010
			    //JR			opcode[5:0]=000000 & func[5:0]=001000 
		//lui	  1 :				opcode[5:0]=001111
		//move	 (rt 0)  2 :			opcode[5:0]=000000 & func[5:1]=00101
			//move rs to rd, 
			//we can use ALU(ADD/SUB) or no, and we should care the condition about Reg_wen

	assign ALU_op = current_state[1]			? 3'b010 :
			(~|opcode[5:0]) & func[5]==1'b1 	? (func[3:2]==2'b00 ? {func[1],2'b10}		:
						 		  func[3:2]==2'b01 ? {func[1],1'b0,func[0]} 	:
								  func[3:2]==2'b10 ? {~func[0],2'b11} 		:
								  0 ) 	 :
			opcode[5:3]==3'b001 & (~&opcode[2:0])	? (opcode[2:1]==2'b00 ? {opcode[1],2'b10} 	:
								  opcode[2]==1'b1 ? {opcode[1],1'b0,opcode[0]} 	:
								  opcode[2:1]==2'b01 ? {~opcode[0],2'b11} 	:
								  0 ) 	 :
			opcode[5:0]==6'b000001			? 3'b111 :
			opcode[5:2]==4'b0001 			? 3'b110 :
			opcode[5]  ==1'b1			? 3'b010 :
			opcode[5:0]==6'b000011 | ((~|opcode[5:0]) & func[5:0]== 6'b001001)	? 3'b010 :
			0; //default
	assign ALU_A =  current_state[1]						  ? PC	   : 
			opcode[5:0]==6'b000011 | ((~|opcode[5:0]) & func[5:0]==6'b001001) ? PC_Reg : 
			RF_rdata1;
	//ALU_B: 4 / 8/ rt / 0 / sign_extend(imm) /zero_extend(imm)
	//imm_extension: I_calc 6 + load and store 12
		//zero_extension : ANDI ORI XORI 	opcode[2]=1
		//sign_extension : ADDIU SLTI SLTIU	opcode[2]=0    load and store: opcode[5]==1
	assign ALU_B =  current_state[1]			? 32'd4							:
			opcode[5:3]==3'b001 & (~&opcode[2:0]) 	? ( opcode[2] ? zero_extension : sign_extension )	:
			opcode[5]				? sign_extension 					:
			opcode[5:0]==6'b000001 			? 32'b0							:
			opcode[5:0]==6'b000011 | ((~|opcode[5:0]) & func[5:0]== 6'b001001)	? 32'd8			:
			RF_rdata2;
	
	alu alu_inst(
	   .A(ALU_A),
	   .B(ALU_B),
	   .ALUop(ALU_op),
	   .Overflow(ALU_Overflow),
	   .CarryOut(ALU_CarryOut),
	   .Zero(ALU_Zero),
	   .Result(ALU_Result)
	);
	
//signals connected to Shifter
	//Operation related to Shifter list: 6 + 2(swl swr)
	//B width is 5
	//R shift: opcode[5:0]=000000 & funct[5:3]=000
		//xxx  (rt sa) 3:		fun[2]=0
		//xxxv (rt rs) 3:		fun[2]=1
	//Swl/swr: opcode[5:0]=101x10
		//swl/swr (rt swl_shifter)		
	assign Shifter_op = (~|opcode[5:0])&(~|func[5:3])		? func[1:0]		: 
			    {opcode[5:3],opcode[1:0]} == 5'b10110	? {~opcode[2],1'b0}	:
			    0;
	assign Shifter_A = RF_rdata2;
	assign Shifter_B = (~|opcode[5:0])&(~|func[5:3])		? (func[2] ? RF_rdata1[4:0] : sa)	  :
			   {opcode[5:3],opcode[1:0]} == 5'b10110	? (opcode[2] ? swr_shifter : swl_shifter) :
			   0;
	
	shifter shifter_inst(
	   .A(Shifter_A),
	   .B(Shifter_B),
	   .Shiftop(Shifter_op),
	   .Result(Shifter_Result)
	);
	
	
//signals connected to Reg_file
	assign RF_raddr1 = rs;
	assign RF_raddr2 = rt;
	//Operations related to RF_wen 32 
		//Type		Num 	addr 		data		feature
		//R calc 	8	rd 		alu_re		opcode[5:0]=000000 & func[5]=1
		//I calc 	6 	rt 		alu_re		opcode[5:3]=001 & (~&opcode[2:0])
		//load		7 	rt 		loaddata 	opcode[5:3]=100
		//shift 	6 	rd 		shift_re 	opcode[5:0]=000000 & func[5:3]=000
		//JAL 		1 	31 		PC8(alu_re) 	opcode[5:0]=000011
		//JALR		1 	rd 		PC8(alu_re) 	opcode[5:0]=000000 & func[5:0]=001001
		//LUI 		1 	rt 		imm||0^16   	opcode[5:0]=001111
		//MOV 		2 	rd 		rdata1		opcode[5:0]=000000 & func[5:1]=00101
			//mov is only use condition to refresh GPR, br or jump refresh PC, so we should consider whether is 0
	//Operation not related to RF_wen 13 
		//Type 	Num 	feature
		//REGIMM 	2	opcode[5:0]=000001
		//Branch 	4 	opcode[5:2]=0001
		//store 	5 	opcode[5:3]=101
		//J 		1	opcode[5:0]=000010
		//JR 		1 	opcode[5:0]=000000 & func[5:0]=001000
	assign RF_wen = (current_state == WB) & 
			((opcode[5:0]==6'b000000 & func[5]==1'b1) 	 	| 
			(opcode[5:3]==3'b001 & (~&opcode[2:0]))  	 	|
			(opcode[5:3]==3'b100)				 	|
			(opcode[5:0]==6'b000000 & func[5:3]==3'b000)		|
			(opcode[5:0]==6'b000011)				|
			(opcode[5:0]==6'b000000 & func[5:0]==6'b001001)		|
			(opcode[5:0]==6'b001111)				|
			((opcode[5:0]==6'b000000 & func[5:1]==5'b00101) & ((func[0] & (|RF_rdata2))|(~func[0] & (~|RF_rdata2))))) ;	
	assign RF_waddr = (opcode[5:3]==3'b100 | opcode[5:3]==3'b001 ) 	? rt 	:
			  (opcode[5:0]==6'b000011) 		  	? 5'd31 :
			  rd ;
	assign RF_wdata = ((opcode[5:0]==6'b000000 & func[5:1]==5'b00101) & ((func[0] & (|RF_rdata2))|(~func[0] & (~|RF_rdata2))))	? RF_rdata1		:
			   (opcode[5:3]==3'b100)											? load_data		: //define below
			   (opcode[5:0]==6'b000000 & func[5:3]==3'b000) 							  	? Shifter_Result 	:
			   (opcode[5:0]==6'b001111)											? {imm,16'b0} 		:
			   ALU_Result ;
						
	reg_file reg_file_inst(
		.clk(clk),
		.raddr1(RF_raddr1),
		.raddr2(RF_raddr2),
		.rdata1(RF_rdata1),
		.rdata2(RF_rdata2),
		.waddr(RF_waddr),
		.wdata(RF_wdata),
		.wen(RF_wen)
	);
	
//signals connected to PC 
	//branch condition  test zero signflag and overflow:
	//branch target is same
		//BEQ,BNE,BLEZ,BGTZ (using SUB): 	opcode[5:2]=0001 	opcode[1:0]=00,01,10,11
		//BLTZ,BGEZ 	    (using SLT):	opcode[5:0]=000001 	rt[0]=0 1
	//jump has no condition
	//jump target differ:
		//J JAL   : j_extension		opcode[5:1] =00001 
		//JR JALR : rs			opcode[5:0] = 000000 & func[5:1]=00100
	
	assign br_en = ( (opcode[5:2]==4'b0001)  &	( (opcode[1:0]==2'b00 & ALU_Zero) 					|
							(opcode[1:0]==2'b01 & ~ALU_Zero) 					|
							(opcode[1:0]==2'b10 & (ALU_Zero  | (ALU_Overflow ^ ALU_Result[31] )))	|
							(opcode[1:0]==2'b11 & (~ALU_Zero & (ALU_Overflow ~^ ALU_Result[31])))
			 				)
			) | 
			( (opcode[5:0]==6'b000001) & 	( (~rt[0] & ALU_Result[0]) |
							(rt[0]  & ~ALU_Result[0])
							)
			) ;
	assign br_tar = PC + br_extension ; 
	//cannot reuse ALU, because the cond will store until PC refresh is done, so we cannot change input of ALU
	assign j_en = (opcode[5:1]==5'b00001) | ((~|opcode[5:0])&func[5:1]==5'b00100) ;
	assign j_tar  = ({32{(opcode[5:1]==5'b00001)}} & j_extension )			|	
			({32{((~|opcode[5:0])&func[5:1]==5'b00100)}} & RF_rdata1);
	
//signals connected to memory (load and store)
	//load opcode[5:3] = 3'b100 	store opcode[5:3]=101 using Little_endian
	//mem control
	assign MemRead  = current_state[4] & ~opcode[3]; //MEM   //100
	assign MemWrite = current_state[4] &  opcode[3]; //MEM	//101
	assign Address  = {ALU_Result[31:2] , 2'b0};
	
	//load data  mem -> rt
		//byte means vaddr/ALU_result[2:0] 	memword means Read_data
	
	assign lb_data = 	({ 32{~ALU_Result[1] & ~ALU_Result[0]} } & { {24{Read_data_Reg[ 7]}} , Read_data_Reg[ 7: 0] } )	|
				({ 32{~ALU_Result[1] &  ALU_Result[0]} } & { {24{Read_data_Reg[15]}} , Read_data_Reg[15: 8] } )	|
				({ 32{ ALU_Result[1] & ~ALU_Result[0]} } & { {24{Read_data_Reg[23]}} , Read_data_Reg[23:16] } )	|
				({ 32{ ALU_Result[1] &  ALU_Result[0]} } & { {24{Read_data_Reg[31]}} , Read_data_Reg[31:24] } )	;
	assign lh_data = 	({ 32{~ALU_Result[1]} } & { {16{Read_data_Reg[15]}} , Read_data_Reg[15: 0] } )   |
				({ 32{ ALU_Result[1]} } & { {16{Read_data_Reg[31]}} , Read_data_Reg[31:16] } )   ;
	assign lw_data = 	Read_data_Reg ;
	assign lbu_data = 	{24'b0 , lb_data[ 7:0]} ;
	assign lhu_data = 	{16'b0 , lh_data[15:0]} ;
	assign lwl_data = 	({ 32{~ALU_Result[1] & ~ALU_Result[0]} } & { Read_data_Reg[ 7: 0] , RF_rdata2[23: 0] } )	|
				({ 32{~ALU_Result[1] &  ALU_Result[0]} } & { Read_data_Reg[15: 0] , RF_rdata2[15: 0] } )	|
				({ 32{ ALU_Result[1] & ~ALU_Result[0]} } & { Read_data_Reg[23: 0] , RF_rdata2[ 7: 0] } )	|
				({ 32{ ALU_Result[1] &  ALU_Result[0]} } &   Read_data_Reg[31: 0] )				;
	assign lwr_data = 	({ 32{~ALU_Result[1] & ~ALU_Result[0]} } &   Read_data_Reg[31: 0] )				|
				({ 32{~ALU_Result[1] &  ALU_Result[0]} } & { RF_rdata2[31:24] , Read_data_Reg[31: 8] } )	|
				({ 32{ ALU_Result[1] & ~ALU_Result[0]} } & { RF_rdata2[31:16] , Read_data_Reg[31:16] } )	|
				({ 32{ ALU_Result[1] &  ALU_Result[0]} } & { RF_rdata2[31: 8] , Read_data_Reg[31:24] } )	;
	assign load_data = 	( {32{opcode[2:0]==3'b000}} &  lb_data )	|
				( {32{opcode[2:0]==3'b001}} &  lh_data )	|
				( {32{opcode[2:0]==3'b011}} &  lw_data )	|
				( {32{opcode[2:0]==3'b100}} & lbu_data )	|
				( {32{opcode[2:0]==3'b101}} & lhu_data )	|
				( {32{opcode[2:0]==3'b010}} & lwl_data )	|
				( {32{opcode[2:0]==3'b110}} & lwr_data )	;
	
	//store data 	rt->mem 
		//strb is signal showing which bytewrite is valid, code by truthtable //byte means vaddr/ALU_result[2:0]
		//vaddr 	swl_strb 	swl_shifter 	swr_Strb 	swr_shifter
		// 00/0		0001		11000/24	1111		00000/0			
		// 01/1 	0011		10000/16	1110		01000/8
		// 10/2 	0111		01000/8		1100		10000/16
		// 11/3 	1111		00000/0		1000		11000/24
	
	assign sb_strb = { ALU_Result[1]& ALU_Result[0] , ALU_Result[1]& ~ALU_Result[0] , ~ALU_Result[1]& ALU_Result[0] , ~ALU_Result[1]& ~ALU_Result[0]};
	assign sh_strb = { {2{ALU_Result[1]}} , {2{~ALU_Result[1]}} } ;
	assign sw_strb = 4'b1111 ;
	assign swl_strb = { &ALU_Result[1:0] , ALU_Result[1] , |ALU_Result[1:0] , 1'b1};
	assign swr_strb = { 1'b1, ~&ALU_Result[1:0] , ~ALU_Result[1], ~|ALU_Result[1:0]} ;
	assign Write_strb = 	( {4{opcode[2:0]==3'b000}} & sb_strb )	|
				( {4{opcode[2:0]==3'b001}} & sh_strb )	|
				( {4{opcode[2:0]==3'b011}} & sw_strb )	|
				( {4{opcode[2:0]==3'b010}} & swl_strb )	|
				( {4{opcode[2:0]==3'b110}} & swr_strb )	;
	
	assign sb_data	= { 4{RF_rdata2[7:0]} } ;
	assign sh_data	= { 2{RF_rdata2[15:0]}}	;
	assign sw_data	= RF_rdata2 ; 
	assign swl_shifter = {~ALU_Result[1:0] , 3'b0};
	assign swr_shifter = { ALU_Result[1:0] , 3'b0};
	// Reuse shifter module to save two shifter
	// swl_data = RF_rdata2 >> swl_shifter ;
	// swr_data = RF_rdata2 << swr_shifter ; 
	assign Write_data = 	( {32{opcode[2:0]==3'b000}} & sb_data )		|
				( {32{opcode[2:0]==3'b001}} & sh_data )		|
				( {32{opcode[2:0]==3'b011}} & sw_data )		|
				( {32{opcode[1:0]==2'b10}}  & Shifter_Result )	;
endmodule




