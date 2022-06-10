//RST state define for all file
	`define RST 		5'b00001
	
//predictor state  
	`define S_Taken  	5'b00010 //strongly taken
	`define W_Taken  	5'b00100
	`define W_NTaken 	5'b01000
	`define S_NTaken	5'b10000

//IF_stage state
	`define IF	5'b00010
	`define IW 	5'b00100
	`define RDS 	5'b01000 //ready to send
	`define SDD 	5'b10000 //send data

//MEM_stage state
	`define BSL	5'b00010 	//before store or load
	`define SL	5'b00100	//store or load
	`define RDW	5'b01000	//read data wait
	`define SLD	5'b10000	//store or load done
	
	
//BUS width between module
	`define PRDT_BUS_WD 	 33  	// 1 br + 32 tar
	
	`define IF_TO_ID_BUS_WD  64	// 32 PC + 32 Instruction_reg
	`define ID_TO_EX_BUS_WD  248 	// see detail in ID_stage
	`define EX_TO_MEM_BUS_WD 146 	//see detail in EX_stage
	`define MEM_TO_WB_BUS_WD 70	//see detail in MEM_stage
	
	`define WB_TO_RF_BUS_WD	 38  	//1 wen + 5 addr + 32 data
	
	`define EX_FW_BUS_WD	 39 	//1 load + 1 valid + 5 addr +32 data
	`define MEM_FW_BUS_WD	 40 	//1 load + 1 done + 1 valid + 5 addr +32 data
	`define WB_FW_BUS_WD	 38	//1 valid + 5 addr +32 data