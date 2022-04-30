`timescale 10ns / 1ns
`include "turbo_macro.v"

module custom_cpu(
	input         clk,
	input         rst,

	//Instruction request channel
	output [31:0] PC,
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

// TODO: Please add your Turbo CPU code here	
	//tmp signal for BHV_SIM
	//wire        inst_retire_valid_tmp;
	wire [69:0] inst_retire_tmp;
	//wire 	    inst_retired_fifo_full_tmp;
	
	//handshake signals between modules
	wire IF_to_ID_valid;
	wire ID_ready;
	wire ID_to_EX_valid;
	wire EX_ready;
	wire EX_to_MEM_valid;
	wire MEM_ready;
	wire MEM_to_WB_valid;
	wire WB_ready;
	
	//data bus between modules
	wire [`IF_TO_ID_BUS_WD-1 : 0] 	IF_to_ID_bus;
	wire [`ID_TO_EX_BUS_WD-1 : 0] 	ID_to_EX_bus;
	wire [`EX_TO_MEM_BUS_WD-1 : 0]	EX_to_MEM_bus;
	wire [`MEM_TO_WB_BUS_WD-1 : 0]	MEM_to_WB_bus;
	
	//by-path : WB to RF through ID 
	wire [`WB_TO_RF_BUS_WD-1 : 0]	WB_to_RF_bus;
	
	//forwarding data bus
	wire [`EX_FW_BUS_WD-1 : 0]	EX_fw_bus;	
	wire [`MEM_FW_BUS_WD-1 : 0]	MEM_fw_bus;
	wire [`WB_FW_BUS_WD-1 : 0]	WB_fw_bus;	
	
	//prediction data
	wire [`PRDT_BUS_WD-1 : 0] 	prdt_bus;
	wire cancel;
	wire [31:0] real_PC;
	
	//performance cnt
	wire [31:0] Cycle_cnt;
	wire [31:0] Inst_cnt;
	wire [31:0] Branch_cnt;
	wire [31:0] Jump_cnt;
	wire [31:0] Prdt_go_cnt;
	wire [31:0] BorJ_v_cnt;
	wire [31:0] Cancel_cnt;
	wire [31:0] MEM_visit_cnt;
	wire [31:0] MEM_delay_cnt;
	wire [31:0] Valid_Inst_cnt;
	
	//inst retired
	// `ifdef BHV_SIM
	// 	assign inst_retire_valid = inst_retire_valid_tmp;
	// 	assign inst_retire	 = inst_retire_tmp;
	// `endif
                assign inst_retire      = inst_retire_tmp;
	
	// `ifdef SIM_RETIRED_FIFO
	// 	assign inst_retired_fifo_full_tmp = inst_retired_fifo_full;
	// `else
	// 	assign inst_retired_fifo_full_tmp = 1'b0;
	// `endif
	
	//performance cnt
	assign cpu_perf_cnt_0  = Cycle_cnt;
	assign cpu_perf_cnt_1  = Inst_cnt;
	assign cpu_perf_cnt_2  = Branch_cnt;
	assign cpu_perf_cnt_3  = Jump_cnt;
	assign cpu_perf_cnt_4  = Prdt_go_cnt;
	assign cpu_perf_cnt_5  = BorJ_v_cnt;
	assign cpu_perf_cnt_6  = Cancel_cnt;
	assign cpu_perf_cnt_7  = MEM_visit_cnt;
	assign cpu_perf_cnt_8  = MEM_delay_cnt;
	assign cpu_perf_cnt_9  = Valid_Inst_cnt;
	
	//5 stage pipeline: IF-ID-EX-MEM-WB
	
	IF_stage IF_inst(
		.clk		(clk)			,
		.rst		(rst)			,
		.PC		(PC)			,
		.Inst_Req_Valid	(Inst_Req_Valid),
		.Inst_Req_Ready	(Inst_Req_Ready),
		.Inst_Valid	(Inst_Valid)	,
		.Inst_Ready 	(Inst_Ready)	,
		.Instruction 	(Instruction)	,
		.cancel		(cancel)		,
		.prdt_bus	(prdt_bus)		,
		.ID_ready	(ID_ready)		,
		.IF_to_ID_valid (IF_to_ID_valid),
		.IF_to_ID_bus	(IF_to_ID_bus)	,
		.real_PC	(real_PC)		,
		.Cycle_cnt	(Cycle_cnt)		,
		.Inst_cnt	(Inst_cnt)
	);
	
	ID_stage ID_inst(
		.clk	 	(clk)			,
		.rst		(rst)			,
		.cancel		(cancel)		,
		.IF_to_ID_valid	(IF_to_ID_valid),
		.IF_to_ID_bus	(IF_to_ID_bus)	,
		.ID_ready	(ID_ready)		,
		.prdt_bus	(prdt_bus)		,
		.EX_ready	(EX_ready)		,
		.ID_to_EX_valid	(ID_to_EX_valid),
		.ID_to_EX_bus	(ID_to_EX_bus)	,
		.WB_to_RF_bus	(WB_to_RF_bus)	,
		.EX_fw_bus	(EX_fw_bus)		,
		.MEM_fw_bus	(MEM_fw_bus)	,
		.WB_fw_bus	(WB_fw_bus)		,
		.Branch_cnt	(Branch_cnt)	,
		.Jump_cnt	(Jump_cnt)		,
		.Prdt_go_cnt	(Prdt_go_cnt)		
	);
	
	EX_stage EX_inst(
		.clk		(clk)			,
		.rst		(rst)			,
		.cancel		(cancel)		,
		.real_PC	(real_PC)		,
		.ID_to_EX_valid	(ID_to_EX_valid),
		.ID_to_EX_bus	(ID_to_EX_bus)	,
		.EX_ready	(EX_ready)		,
		.MEM_ready	(MEM_ready)		,
		.EX_to_MEM_valid(EX_to_MEM_valid),
		.EX_to_MEM_bus	(EX_to_MEM_bus)	,
		.EX_fw_bus	(EX_fw_bus)		,
		.BorJ_v_cnt	(BorJ_v_cnt)	,
		.Cancel_cnt	(Cancel_cnt)	
	);
	
	MEM_stage MEM_inst(
		.clk		(clk)			,
		.rst		(rst)			,
		.Address	(Address)		,
		.MemWrite	(MemWrite)		,
		.Write_data	(Write_data)	,
		.Write_strb	(Write_strb)	,
		.MemRead	(MemRead)		,
		.Mem_Req_Ready	(Mem_Req_Ready)	,
		.Read_data	(Read_data)		,
		.Read_data_Valid(Read_data_Valid),
		.Read_data_Ready(Read_data_Ready),
		.EX_to_MEM_valid(EX_to_MEM_valid),
		.EX_to_MEM_bus	(EX_to_MEM_bus)	,
		.MEM_ready	(MEM_ready)		,
		.WB_ready	(WB_ready)		,
		.MEM_to_WB_valid(MEM_to_WB_valid),
		.MEM_to_WB_bus	(MEM_to_WB_bus)	,
		.MEM_fw_bus	(MEM_fw_bus)	,
		.MEM_visit_cnt	(MEM_visit_cnt)	,
		.MEM_delay_cnt	(MEM_delay_cnt)
	);
	
	WB_stage WB_inst(
		.clk		(clk)			,
		.rst		(rst)			,
		.MEM_to_WB_valid(MEM_to_WB_valid),
		.MEM_to_WB_bus	(MEM_to_WB_bus)	,
		.WB_ready	(WB_ready)		,
		.WB_to_RF_bus	(WB_to_RF_bus)	,
		.WB_fw_bus	(WB_fw_bus)		,
		//.inst_retired_fifo_full	(inst_retired_fifo_full_tmp),
		//.inst_retire_valid	(inst_retire_valid_tmp)		,
		.inst_retire	(inst_retire_tmp)	,
		.Valid_Inst_cnt	(Valid_Inst_cnt)
	);
endmodule
