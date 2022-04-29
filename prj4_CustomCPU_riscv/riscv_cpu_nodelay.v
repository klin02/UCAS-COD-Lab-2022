`timescale 10ns / 1ns

module custom_cpu(
	input         clk,
	input         rst,

	//Instruction request channel
	output reg [31:0] PC,
	output        Inst_Req_Valid,
	input         Inst_Req_Ready,

	//Instruction response channel
	input  [31:0] Instruction,
	input         Inst_Valid,
	output        Inst_Ready,

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

`ifdef SIM_RETIRED_FIFO
	input         inst_retired_fifo_full,
`endif

	input         intr,

	output [31:0] cpu_perf_cnt_0,
	output [31:0] cpu_perf_cnt_1,
	output [31:0] cpu_perf_cnt_2,
	output [31:0] cpu_perf_cnt_3,
	output [31:0] cpu_perf_cnt_4,
	output [31:0] cpu_perf_cnt_5,
	output [31:0] cpu_perf_cnt_6,
	output [31:0] cpu_perf_cnt_7,
	output [31:0] cpu_perf_cnt_8,
	output [31:0] cpu_perf_cnt_9,
	output [31:0] cpu_perf_cnt_10,
	output [31:0] cpu_perf_cnt_11,
	output [31:0] cpu_perf_cnt_12,
	output [31:0] cpu_perf_cnt_13,
	output [31:0] cpu_perf_cnt_14,
	output [31:0] cpu_perf_cnt_15
);

/* The following signal is leveraged for behavioral simulation, 
* which is delivered to testbench.
*
* STUDENTS MUST CONTROL LOGICAL BEHAVIORS of THIS SIGNAL.
*
* inst_retired (70-bit): detailed information of the retired instruction,
* mainly including (in order) 
* { 
*   reg_file write-back enable  (69:69,  1-bit),
*   reg_file write-back address (68:64,  5-bit), 
*   reg_file write-back data    (63:32, 32-bit),  
*   retired PC                  (31: 0, 32-bit)
* }
*
*/
  wire [69:0] inst_retire;

	// TODO: Please add your custom CPU code here	
//define the state of machine by one-hot
	localparam 	RST		=9'b000000001,
			IF		=9'b000000010,
			IW		=9'b000000100,
			ID		=9'b000001000,				
			EX		=9'b000010000,
			LD		=9'b000100000,
			ST		=9'b001000000,
			RDW		=9'b010000000,
			WB		=9'b100000000;
//set reg to store state and single_cycle input
	reg [ 8:0] 	current_state;
	reg [ 8:0] 	next_state;
	reg [31:0] 	Instruction_Reg;
	reg [31:0]	Read_data_Reg;
	reg [31:0]	PC_Reg;
	//PC refresh in ID, set a reg to store origin PC
	
//Signals related to IF
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
        wire [31:0] ALU_A_origin;
        wire [31:0] ALU_B_origin;
        reg  [31:0] ALU_A_reg;  //not include PC
        reg  [31:0] ALU_B_reg;
	wire [31:0] ALU_A_final;
	wire [31:0] ALU_B_final;
        wire [ 2:0] ALU_op_origin;
        reg  [ 2:0] ALU_op_reg;
	wire [ 2:0] ALU_op_final;
	wire	    ALU_Overflow;
	wire 	    ALU_CarryOut;
	wire 	    ALU_Zero;
	wire [31:0] ALU_Result;
        reg  [31:0] ALU_Result_reg;

//Signals connected to Shifter
	wire [31:0] Shifter_A;
	wire [ 4:0] Shifter_B;
	wire [ 1:0] Shifter_op;
	wire [31:0] Shifter_Result;
        reg  [31:0] Shifter_A_reg;
	reg  [ 4:0] Shifter_B_reg;
	reg  [ 1:0] Shifter_op_reg;
	reg  [31:0] Shifter_Result_reg;

//Signals connected to Reg_file
	wire [ 4:0] 	RF_raddr1;
	wire [ 4:0] 	RF_raddr2;
	wire [31:0] 	RF_rdata1;
	wire [31:0] 	RF_rdata2;
	wire		RF_wen;      
	wire [4:0]	RF_waddr; 	 
	wire [31:0]	RF_wdata;   
	
//Signals connected to PC refresh
	wire 		br_en;
	wire [31:0]	br_tar;
	wire 		j_en;
	wire [31:0]	j_tar;
	
//Signals connected to load and store
	wire [31:0]	load_data;
	wire [31:0]	lb_data;  //sign_extend
	wire [31:0]	lh_data;
	wire [31:0]	lw_data;
	wire [31:0]	lbu_data; //zero_extend
	wire [31:0]	lhu_data;
	
	wire [3:0]	sb_strb;
	wire [3:0]	sh_strb;
	wire [3:0]	sw_strb;
	
	wire [31:0]	sb_data;  //take the least significant 8 bit
	wire [31:0]	sh_data;
	wire [31:0]	sw_data;

//cancel delay, add signal PC_Reg4
        wire [31:0]     PC_Reg4;
        assign PC_Reg4 = PC_Reg +4;

//signals related to retired
	assign inst_retire = {RF_wen , RF_waddr , RF_wdata, PC_Reg};
	
//define signal for State Machine
	//set 1 to Inst_Ready and Read_data_Ready to prevent Error
	assign Inst_Req_Valid	= current_state[1] ;	//IF
	assign Inst_Ready	= current_state[0] | current_state[2] ;//RST IW
	assign Read_data_Ready  = current_state[0] | current_state[7] ;//RST RDW
	
//State Machine
	//Part I: 
    always @ (posedge clk) begin
       if(rst) begin
           current_state <= RST;
       end
       else begin
           current_state <= next_state;
       end
    end

	//Part II: Combinatorial logic
	always @ (*)
	begin
		case(current_state)
			RST: begin
				next_state = IF ;
			end
			IF:	begin
				if (Inst_Req_Ready)
					next_state = IW;
				else 
					next_state = IF;
			end
			IW: begin
				if (Inst_Valid)
					next_state = ID;
				else 
					next_state = IW;
			end
			//NOP: Instr = 32'b0
			ID: begin
				next_state = EX ;  //no need to care NOP
			end
			//EX->IF:
				//B_type
			//EX->WB
				//R_type / I_calc / I_jalr / U_type / J_type 
			//EX->LD
				//I_load
			//EX->ST
				//S_type
			EX: begin
				if(B_type)
					next_state = IF;
				else if(R_type | I_calc | I_jalr | U_type | J_type )
					next_state = WB;
				else if(I_load)
					next_state = LD;
				else if(S_type)
					next_state = ST;
				else
					next_state = RST;
			end
			LD: begin
				if(Mem_Req_Ready)
					next_state = RDW;
				else 
					next_state = LD;
			end
			ST: begin
				if(Mem_Req_Ready)
					next_state = IF;
				else 
					next_state = ST;
			end
			RDW: begin
				if(Read_data_Valid)
					next_state = WB;
				else 
					next_state = RDW;
			end
			WB: begin
				next_state = IF;
			end
			default: begin
				next_state = RST;
			end
		endcase
	end
	
	//Part III: deal with output
	//one always module to deal one reg
	always @ (posedge clk) begin
		if(Inst_Ready & Inst_Valid) //after response
			Instruction_Reg <= Instruction ;
	end
	
	always @ (posedge clk) begin
		if(Read_data_Ready & Read_data_Valid)
			Read_data_Reg <=Read_data ;
	end
	
	always @ (posedge clk) begin
		if(current_state[1])//IF
			PC_Reg <= PC;
	end
	
	always @ (posedge clk) begin
		if (rst) begin
			PC <= 32'b0;
		end
		// else if(current_state[2] & Inst_Valid & ~rst) begin
		// 	PC <= ALU_Result; //PC+4 IW, consider the cycle before ID
		// end
		else if(current_state[4] & ~rst)begin 
			PC <= br_en ? br_tar : j_en ? j_tar : PC_Reg4 ; 
			//j or br OP refresh in next IF, judge by EX
			//note that the default result is PC4
		end
	end
	
	
//Analyse Instruction code
	//imm_extension:
		//I_type: unsigned: SLTIU(011)  signed：other OP  shamt: shift   note that I_jalr: {rs1+signed(offset)}[31:1],0
		//S_type: signed
		//B_type: signed  in multiples of 2
		//U_type: fill low 12 bit with 0
		//J_type: signed  in multiples of 2
	assign opcode 	= Instruction_Reg[6:0];
	assign rd	= Instruction_Reg[11:7];
	assign rs1	= Instruction_Reg[19:15];
	assign rs2	= Instruction_Reg[24:20];
	assign shamt	= Instruction_Reg[24:20];
	assign funct3	= Instruction_Reg[14:12];
	assign funct7	= Instruction_Reg[31:25];
	assign I_imm[11:0]  = Instruction_Reg[31:20];
	assign I_imm[31:12] = funct3 == 3'b011 ? {20'b0} : {20{Instruction_Reg[31]}};
	assign S_imm	= { {20{Instruction_Reg[31]}}, Instruction_Reg[31:25], Instruction_Reg[11:7]};
	assign B_imm	= { {19{Instruction_Reg[31]}}, Instruction_Reg[31], Instruction_Reg[7], Instruction_Reg[30:25], Instruction_Reg[11:8], 1'b0};
	assign U_imm 	= { Instruction_Reg[31:12],12'b0};
	assign J_imm	= { {11{Instruction_Reg[31]}}, Instruction_Reg[31], Instruction_Reg[19:12], Instruction_Reg[20], Instruction_Reg[30:21], 1'b0};
	
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
	
//Channel to ALU
	//IW stage: use add to save adder of PC+4
	//WB stage: use add to save adder of PC_reg+4 by I_jalr and J_type
		//note that ALU is also used in EX_stage to get PC+imm by I_jalr and J_type
	//Operations related to ALU:
	//TYPE				op1	op2	OP
	//R(exclude shift)		rs1	rs2	...
	//I_calc(exclude shift)		rs1 	I_imm 	...
	//I_load I_jalr			rs1 	I_imm 	ADD 				
	//S_type 			rs1 	S_imm	ADD
	//B_type
		//BEQ,BNE		rs1	rs2 	SUB
		//other 		rs1 	rs2 	SLT/SLTU
	//U_auipc 			PC_Reg 	U_imm 	ADD
	//J_type			PC_Reg  J_imm 	ADD
	
        assign ALU_op_origin = R_type & ~(funct3==3'b001 | funct3 == 3'b101) ? ( funct3 == 3'b000	? {funct7[5],2'b10} 	:
                                                                                funct3[2:1] == 2'b01 	? {~funct3[0],2'b11}	:
                                                                                funct3 == 3'b100	? funct3 		:
                                                                                funct3[2:1] == 2'b11	? ~funct3		:
                                                                                0 ):
                                I_calc & ~(funct3==3'b001 | funct3 == 3'b101) ? ( funct3 == 3'b000	? 3'b010		:
                                                                                funct3[2:1] == 2'b01 	? {~funct3[0],2'b11}	:
                                                                                funct3 == 3'b100	? funct3 		:
                                                                                funct3[2:1] == 2'b11	? ~funct3		:
                                                                                0 ): 
                                I_load | I_jalr	| S_type | U_auipc | J_type   ?  3'b010 : 
                                B_type					      ? ( funct3[2:1]==2'b00 ? 3'b110 : {~funct3[1], 2'b11}) :
                                0;//default
        always @(posedge clk) begin
                if(current_state[3]) //ID
                        ALU_op_reg <= ALU_op_origin;
                // else if(current_state[4] & (I_jalr | J_type)) //EX before WB
                //         ALU_op_reg <= 3'b010;
        end
        //assign ALU_op_final = current_state[2]	? 3'b010 : ALU_op_reg;
	assign ALU_op_final = ALU_op_reg;

        assign ALU_A_origin =   U_auipc | J_type ? PC_Reg : 
			        RF_rdata1;
        always @(posedge clk) begin
                if(current_state[3]) //ID
                        ALU_A_reg <= ALU_A_origin;
                // else if(current_state[4] & (I_jalr | J_type)) //EX before WB
                //         ALU_A_reg <= PC_Reg;
        end
	//assign ALU_A_final =  current_state[2]  ? PC : ALU_A_reg;
        assign ALU_A_final = ALU_A_reg;

        assign ALU_B_origin =   I_type & ~(funct3==3'b001 | funct3 == 3'b101)		  ? I_imm :
                                S_type							  ? S_imm : 
                                U_auipc							  ? U_imm : 
                                J_type							  ? J_imm :
                                RF_rdata2;
        always @(posedge clk) begin
                if(current_state[3]) //ID
                        ALU_B_reg <= ALU_B_origin;
                // else if(current_state[4] & (I_jalr | J_type)) //EX before WB
                //         ALU_B_reg <= 32'd4;
        end
	//assign ALU_B_final =  current_state[2] ? 32'd4 : ALU_B_reg;
        assign ALU_B_final = ALU_B_reg;

        always @(posedge clk) begin
                if(current_state[4])//EX
                        ALU_Result_reg <= ALU_Result;
        end

	alu alu_inst(
	   .A(ALU_A_final),
	   .B(ALU_B_final),
	   .ALUop(ALU_op_final),
	   .Overflow(ALU_Overflow),
	   .CarryOut(ALU_CarryOut),
	   .Zero(ALU_Zero),
	   .Result(ALU_Result)
	);
	
//Channel to Shifter
	//Operations related to Shifter
	//R_type shift	rs1 rs2 
	//I_calc shift  rs1 shamt

	assign Shifter_op = (R_type | I_calc) & (funct3==3'b001 | funct3 == 3'b101) ? {funct3[2],funct7[5]} : 0;
	assign Shifter_A  = RF_rdata1;
	assign Shifter_B  = R_type ? RF_rdata2[4:0] : shamt;
	always @(posedge clk)begin
                if(current_state[3]) //ID
                        Shifter_op_reg <= Shifter_op;
                        Shifter_A_reg  <= Shifter_A;
                        Shifter_B_reg  <= Shifter_B;
        end

        always @(posedge clk)begin
                if(current_state[4])//EX
                        Shifter_Result_reg <= Shifter_Result;
        end

	shifter shifter_inst(
	   .A(Shifter_A_reg),
	   .B(Shifter_B_reg),
	   .Shiftop(Shifter_op_reg),
	   .Result(Shifter_Result)
	);

//Channel to Reg_file
	//Operations related to Reg_file:
	//TYPE 			addr 	data
	//R_type 		
		//shift 	rd 	shift_re
		//other 	rd 	alu_re
	//I_calc		
		//shift 	rd 	shift_re
		//other 	rd 	alu_re
	//I_load 		rd 	loaddata
	//I_jalr 		rd 	PC_reg4(alu_re)
	//J_type		rd 	PC_reg4(alu_re)
	//U_lui			rd 	U_imm
	//U_auipc 		rd 	PC + U_imm(alu_re)
	
	assign RF_raddr1 = rs1;
	assign RF_raddr2 = rs2;
	assign RF_wen	 = (current_state[8]) & //WB
			   (R_type | I_type | J_type | U_type);
	assign RF_waddr  = rd;
	assign RF_wdata  = R_type | I_calc ? (funct3==3'b001 | funct3 == 3'b101 ? Shifter_Result_reg : ALU_Result_reg) : 
			   I_load 	   ? load_data  : 
			   I_jalr | J_type ? PC_Reg4 : //A and B just valid in WB
			   U_lui 	   ? U_imm	:
			   U_auipc	   ? ALU_Result_reg : //PC_reg + U_imm
			   0; //default
	
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

//PC refresh Unit
	assign br_en = B_type & ( ( ~funct3[2] & ~funct3[0] &  ALU_Zero ) 	| 
				  ( ~funct3[2] &  funct3[0] & ~ALU_Zero) 	|
				  (  funct3[2] & ~funct3[0] &  ALU_Result[0]) 	|
				  (  funct3[2] &  funct3[0] & ~ALU_Result[0])
				);
	assign br_tar 	= PC_Reg + B_imm ;  //note PC is refreshed in ID, now is PC_reg4
	//cannot reuse ALU here, because br_en and br_tar will work at the same time, and br_en occupy ALU to store
	assign j_en	= J_type | I_jalr ; 
	assign j_tar 	= J_type ? ALU_Result :
			  I_jalr ? { ALU_Result[31:1] , 1'b0 } :
			  0;//default

//Memory visit Unit
	assign MemRead 	= current_state[5]; //LD
	assign MemWrite = current_state[6]; //ST
	assign Address	= {ALU_Result_reg[31:2], 2'b0};
	
	//load data  mem -> rt
		//byte means vaddr/ALU_result[2:0] 	memword means Read_data
	
	assign lb_data 	= ({ 32{~ALU_Result_reg[1] & ~ALU_Result_reg[0]} } & { {24{Read_data_Reg[ 7]}} , Read_data_Reg[ 7: 0] } )	|
			  ({ 32{~ALU_Result_reg[1] &  ALU_Result_reg[0]} } & { {24{Read_data_Reg[15]}} , Read_data_Reg[15: 8] } )	|
			  ({ 32{ ALU_Result_reg[1] & ~ALU_Result_reg[0]} } & { {24{Read_data_Reg[23]}} , Read_data_Reg[23:16] } )	|
			  ({ 32{ ALU_Result_reg[1] &  ALU_Result_reg[0]} } & { {24{Read_data_Reg[31]}} , Read_data_Reg[31:24] } )	;
	assign lh_data 	= ({ 32{~ALU_Result_reg[1]} } & { {16{Read_data_Reg[15]}} , Read_data_Reg[15: 0] } )	|
			  ({ 32{ ALU_Result_reg[1]} } & { {16{Read_data_Reg[31]}} , Read_data_Reg[31:16] } )   ;
	assign lw_data 	= Read_data_Reg ;
	assign lbu_data = {24'b0 , lb_data[ 7:0]} ;
	assign lhu_data = {16'b0 , lh_data[15:0]} ;

	assign load_data = ( {32{funct3 == 3'b000}} &  lb_data )	|
			   ( {32{funct3 == 3'b001}} &  lh_data )	|
			   ( {32{funct3 == 3'b010}} &  lw_data )	|
			   ( {32{funct3 == 3'b100}} & lbu_data )	|
			   ( {32{funct3 == 3'b101}} & lhu_data )	;
	
	//store data 	rt->mem 
		//strb is signal showing which bytewrite is valid, code by truthtable //byte means vaddr/ALU_result[2:0]
		
	
	assign sb_strb 	= { ALU_Result_reg[1]& ALU_Result_reg[0] , ALU_Result_reg[1]& ~ALU_Result_reg[0] , ~ALU_Result_reg[1]& ALU_Result_reg[0] , ~ALU_Result_reg[1]& ~ALU_Result_reg[0]};
	assign sh_strb 	= { {2{ALU_Result_reg[1]}} , {2{~ALU_Result_reg[1]}} } ;
	assign sw_strb 	= 4'b1111 ;
	assign Write_strb = ( {4{funct3 == 3'b000}} & sb_strb )	|
			    ( {4{funct3 == 3'b001}} & sh_strb )	|
			    ( {4{funct3 == 3'b010}} & sw_strb )	;
	
	assign sb_data	= { 4{RF_rdata2[7:0]} } ;
	assign sh_data  = { 2{RF_rdata2[15:0]}}	;
	assign sw_data	= RF_rdata2 ; 
	assign Write_data = ( {32{funct3 == 3'b000}} & sb_data )	|
			    ( {32{funct3 == 3'b001}} & sh_data )	|
			    ( {32{funct3 == 3'b010}} & sw_data )	;

						
//counter
	//set specific name to distinguish: (v means victory, f means failure)
		//cycle, ins, mem(num of visit mem), delay,
		//branch_v, branch_f,jump_v,jump_f
	
	reg [31:0]	cycle_cnt;		//count the num of clock
	reg [31:0]	inst_cnt;		//count the num of obtained instruction
	reg [31:0]	ex_cnt;			//count the num of execution
	reg [31:0]	mem_cnt;		//count the num of mem visit
	reg [31:0]	mem_delay_cnt;		//count the num of delay of mem,including read and write
	reg [31:0]	br_j_v_cnt;		//count the num of PC not refresh for PC4
	reg [31:0]	br_j_f_cnt;		//count the num of PC refresh for PC4, including instr not br or j
	reg [31:0]	branch_v_cnt;		//count the num of successful branch
	reg [31:0]	branch_f_cnt;		//count the num of failed branch
	reg [31:0]	jump_cnt;		//count the num of jump (always success)
	
	assign cpu_perf_cnt_0	= cycle_cnt;
	assign cpu_perf_cnt_1	= inst_cnt;
	assign cpu_perf_cnt_2	= ex_cnt;
	assign cpu_perf_cnt_3	= mem_cnt;
	assign cpu_perf_cnt_4	= mem_delay_cnt;
	assign cpu_perf_cnt_5	= br_j_v_cnt;
	assign cpu_perf_cnt_6	= br_j_f_cnt;
	assign cpu_perf_cnt_7	= branch_v_cnt;
	assign cpu_perf_cnt_8	= branch_f_cnt;
	assign cpu_perf_cnt_9	= jump_cnt;
	
	always @ (posedge clk) begin
		if(rst)
			cycle_cnt <= 32'b0;
		else 
			cycle_cnt <= cycle_cnt + 32'b1;
	end
	
	always @ (posedge clk) begin
		if(rst)
			inst_cnt <= 32'b0;
		else if(current_state[1]) //IF
			inst_cnt <= inst_cnt + 32'b1;
	end
	
	always @ (posedge clk) begin
		if(rst)
			ex_cnt <= 32'b0;
		else if(current_state[4])//EX
			ex_cnt <= ex_cnt + 32'b1;
	end
	
	always @ (posedge clk) begin
		if(rst)
			mem_cnt <= 32'b0;
		else if(current_state[5] | current_state[6]) //ST LD
			mem_cnt <= mem_cnt + 32'b1;
	end
	
	always @ (posedge clk) begin
		if(rst)
			mem_delay_cnt <= 32'b0;
		else if(( (current_state[5]|current_state[6]) & ~Mem_Req_Ready )|( current_state[7] & ~Read_data_Valid )) //LD ST ~Mem_Req_Ready -- RDW ~Read_data_Valid
			mem_delay_cnt <= mem_delay_cnt + 32'b1;
	end		
	//state = EX 4
	//Branch 4 :			opcode[5:2]=0001  
	//REGIMM 2 :			opcode[5:0]=000001
	//jump   4 : J JAL 		opcode[5:1] =00001 
		  //JR JALR 		opcode[5:0] = 000000 & func[5:1]=00100
	always @ (posedge clk) begin
		if(rst)
			br_j_v_cnt <= 32'b0;
		else if(current_state[4] & ( (B_type & br_en) | (J_type | I_jalr) ))
			br_j_v_cnt <= br_j_v_cnt + 32'b1;//branch_suc or jump
	end
	
	always @ (posedge clk) begin
		if(rst)
			br_j_f_cnt <= 32'b0;
		else if(current_state[4] & ~( (B_type & br_en) | (J_type | I_jalr) ))
			br_j_f_cnt <= br_j_f_cnt + 32'b1;//normal refresh
	end
	
	always @ (posedge clk) begin
		if(rst)
			branch_v_cnt <= 32'b0;
		else if(current_state[4] & (B_type & br_en))
			branch_v_cnt <= branch_v_cnt + 32'b1;
	end
	
	always @ (posedge clk) begin
		if(rst)
			branch_f_cnt <= 32'b0;
		else if(current_state[4] & (B_type & ~br_en))
			branch_f_cnt <= branch_f_cnt + 32'b1;
	end
	
	always @ (posedge clk) begin
		if(rst)
			jump_cnt <= 32'b0;
		else if(current_state[4] & (J_type | I_jalr))
			jump_cnt <= jump_cnt + 32'b1;
	end
endmodule
