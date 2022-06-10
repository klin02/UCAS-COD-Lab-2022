`include "turbo_macro.v"

module IF_stage(
    input  clk,
    input  rst,

    output reg [31:0] PC,
    output Inst_Req_Valid,
    input  Inst_Req_Ready,
    input  Inst_Valid,
    output Inst_Ready,

    input [31:0] Instruction,

/* in cpu_to_mem_axi_2x1_arb.v, MemRead and Inst_req_valid, only one can work.*/
    input MemRead,

//overall
	//Signals show prediction fail
    input cancel,

//from ID
    //predict information from ID stage
    input  [`PRDT_BUS_WD - 1 : 0] prdt_bus,
	//Signal show next state ready to accept
    input  ID_ready,
//to ID
    //Signal show this state valid to send
    output IF_to_ID_valid,
    //data IF send to ID
    output [`IF_TO_ID_BUS_WD - 1 : 0] IF_to_ID_bus,	
	
//from EX
    //real pc from EX
    input [31:0] real_PC,
	
    //prediction and fail cnt
	output reg [31:0] Cycle_cnt,
	output reg [31:0] Inst_cnt
);

	reg [4:0] IF_cur_state;
	reg [4:0] IF_next_state;
	
	wire	prdt_go;
	wire [31:0] prdt_tar;
	
	reg 	IF_work; //whether this cycle really work 
	wire 	IF_done; //whether all work are done
	//reg 	renew;	//expand cancel to mark failed IF_pipeline
	
	reg [31:0] Instruction_Reg;
//Temp count the num of inst_req and inst_
	reg [31:0] inst_req_cnt;
	reg [31:0] inst_cnt;

	always @(posedge clk) begin
		if(rst)
			inst_req_cnt <= 0;
		else if (Inst_Req_Valid & Inst_Req_Ready) begin
			inst_req_cnt <= inst_req_cnt + 1;
		end
	end

	always @(posedge clk) begin
		if(rst)
			inst_cnt <= 0;
		else if(Inst_Valid & Inst_Ready) begin
			inst_cnt <= inst_cnt +1 ;
		end
	end
//Inst request and response channel
	assign Inst_Req_Valid = IF_cur_state == `IF & ~cancel & ~MemRead; 
	//assign Inst_Req_Valid = IF_cur_state == `IF & ~cancel; 
	assign Inst_Ready = IF_cur_state == `IW | IF_cur_state == `RST;

//State Machine: IF_pipeline
	always @(posedge clk) begin
		if(rst)
			IF_cur_state <= `RST;
		else
			IF_cur_state <= IF_next_state;
	end
	
	always @(*) begin
		case (IF_cur_state)
			`RST : begin
					IF_next_state = `IF;
			end
			`IF : begin
				if(cancel)
					IF_next_state = `IF;
				else if(Inst_Req_Valid & Inst_Req_Ready)
					IF_next_state = `IW;
				else 
					IF_next_state = `IF;
			end
			`IW : begin
				if(cancel)
					IF_next_state = `IF;
				else if(Inst_Ready & Inst_Valid)
					IF_next_state = `RDS;
				else 
					IF_next_state = `IW;
			end
			`RDS : begin
				if(cancel)
					IF_next_state = `IF;
				else if(ID_ready)
					IF_next_state = `SDD;
				else 
					IF_next_state = `RDS;
			end
			`SDD : begin
				if(ID_ready)
					IF_next_state = `IF ;
				else
					IF_next_state = `SDD;
			end
			default: begin
					IF_next_state = `RST;
			end
		endcase
	end
	
//Signal show whether this IF_pipeline really work
	//MAYBE useless!!! CHECK IT
	always @(posedge clk) begin
		if(rst)
			IF_work <= 1'b0;
		else begin
			if(cancel)
				IF_work <= 1'b0;
			else if(IF_ready)
				IF_work <= 1'b1;
		end
	end
	
//PC refresh Unit
	assign {prdt_go,prdt_tar} = prdt_bus;
	
	// always @(posedge clk) begin
	// 	if(rst)
	// 		PC <= 32'hfffffffc;
	// 	else begin
	// 		if(IF_ready & IF_next_state==`IF) begin
	// 			if(cancel)  //CHECK the COND
	// 				PC <= real_PC;
	// 			else 
	// 				PC <= prdt_go ? prdt_tar : PC +4;
	// 		end
	// 	end
	// end
		
	always @(posedge clk) begin
		if(rst)
			PC <= 32'b0;
		else if(cancel)  //CHECK the COND
			PC <= real_PC;
		else if(IF_done & ID_ready) begin
			PC <= prdt_go ? prdt_tar : PC +4;
		end
	end	

	always @(posedge clk) begin
		if(Inst_Ready & Inst_Valid)
			Instruction_Reg <= Instruction;
	end
	
//Control Unit
	assign IF_ready = ~IF_work | (IF_done & ID_ready);
	assign IF_done  = IF_cur_state ==`SDD & ~cancel;
	// assign IF_done  = (IF_cur_state == `RDS | IF_cur_state ==`SDD) & ~cancel;
	assign IF_to_ID_valid = IF_cur_state ==`RDS & ID_ready & ~cancel & IF_work; 
	assign IF_to_ID_bus ={ 
				PC		,	//63:32			
				Instruction_Reg 	//31:0
				};
						 
//Performance cnt
	always @(posedge clk) begin
		if(rst)
			Cycle_cnt <= 32'b0;
		else 
			Cycle_cnt <= Cycle_cnt +1;
	end
	
	always @(posedge clk) begin
		if(rst)
			Inst_cnt <= 32'b0;
		else if (Inst_Req_Ready & Inst_Req_Valid)
			Inst_cnt <= Inst_cnt +1;
	end
	
endmodule

// 检查pipeline是否需要完整 IF可否连续