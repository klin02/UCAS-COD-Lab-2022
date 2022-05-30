`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Xu Zhang (zhangxu415@mails.ucas.ac.cn)
// 
// Create Date: 06/14/2018 11:39:09 AM
// Design Name: 
// Module Name: dma_core
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module engine_core #(
	parameter integer  DATA_WIDTH       = 32
)
(
	input    clk,
	input    rst,
	
	output reg [31:0]   src_base,
	output reg [31:0]   dest_base,
	output reg [31:0]   tail_ptr,
	output reg [31:0]   head_ptr,
	output reg [31:0]   dma_size,
	output reg [31:0]   ctrl_stat,

	input  [31:0]	    reg_wr_data,
	input  [ 5:0]       reg_wr_en,
  
	output 		    intr,
  
	output reg [31:0]   rd_req_addr,
	output [ 4:0]       rd_req_len,
	output              rd_req_valid,
	
	input               rd_req_ready,
	input  [31:0]       rd_rdata,
	input               rd_last,
	input               rd_valid,
	output              rd_ready,
	
	output reg [31:0]   wr_req_addr,
	output [ 4:0]       wr_req_len,
	output              wr_req_valid,
	input               wr_req_ready,
	output [31:0]       wr_data,
	output              wr_valid,
	input               wr_ready,
	output              wr_last,
	
	output              fifo_rden,
	output [31:0]       fifo_wdata,
	output              fifo_wen,
	
	input  [31:0]       fifo_rdata,
	input               fifo_is_empty,
	input               fifo_is_full
);
	// TODO: Please add your logic design here
//One-Hot code of state
	localparam  	RD_idle	= 3'b001 ,
			RD_req  = 3'b010 ,
			RD_work = 3'b100 ,
			WR_idle = 3'b001 ,
			WR_req  = 3'b010 , 
			WR_work = 3'b100 ;

//Signals of DMA control
	wire INTR;
	wire EN;

//Signals of State Machine
	reg [2:0]	RD_cur_state;
	reg [2:0]	RD_next_state;
	reg [2:0]	WR_cur_state;
	reg [2:0]	WR_next_state;

//Signals of burst time and count
	wire [31:0]	burst_time;
	wire [31:0]	burst_time_A;
	wire [31:0]	burst_time_B;
	reg [31:0] 	RD_burst_cnt;
	reg [31:0]	WR_burst_cnt;

//Signals of len of last burst
	wire [4:0]	last_burst_len;
	wire [4:0]	last_burst_len_A;
	wire [4:0]	last_burst_len_B;

//Signals of last shifter
	reg [8:0]	last_shifter;

//Singasl related to fifo rd
	reg fifo_rden_one_clk_delay;

//Deal with input from CPU
	always @(posedge clk) begin
		if(rst) begin
			src_base  <= 32'b0;
			dest_base <= 32'b0;
			tail_ptr  <= 32'b0;
			head_ptr  <= 32'b0;
			dma_size  <= 32'b0;
			ctrl_stat <= 32'b0;
		end
		else if(EN & WR_cur_state[1] & RD_burst_cnt == burst_time & WR_burst_cnt == burst_time) //check whether state is necessary
		begin
			tail_ptr 	<= tail_ptr +dma_size;
			ctrl_stat[31]  	<= 1'b1; 
		end
		else begin
			if(reg_wr_en[0])	src_base  <= reg_wr_data;
			else if(reg_wr_en[1])	dest_base <= reg_wr_data;
			else if(reg_wr_en[2])	tail_ptr  <= reg_wr_data;
			else if(reg_wr_en[3])	head_ptr  <= reg_wr_data;
			else if(reg_wr_en[4])	dma_size  <= reg_wr_data;
			else if(reg_wr_en[5])	ctrl_stat <= reg_wr_data;
		end
	end

//Singals of DMA enable and intr valid
	assign intr = ctrl_stat[31];
	assign EN   = ctrl_stat[0];

//State Machine of READ
	always @(posedge clk) begin
		if(rst)
			RD_cur_state <= RD_idle;
		else
			RD_cur_state <= RD_next_state;
	end

	always @(*) begin
		case(RD_cur_state)
			RD_idle: begin
				if(EN & WR_cur_state[0] & head_ptr!=tail_ptr)
					RD_next_state = RD_req;
				else
					RD_next_state = RD_idle;
			end
			RD_req: begin
				if(rd_req_ready&rd_req_valid)		//consider RD burst cnt != burst time
					RD_next_state = RD_work;
				else if(RD_burst_cnt == burst_time)
					RD_next_state = RD_idle;
				else
					RD_next_state = RD_req;
			end
			RD_work: begin
				//if(rd_valid & rd_last & ~fifo_is_full)
				if(rd_ready & rd_valid & rd_last)
					RD_next_state = RD_req;
				else
					RD_next_state = RD_work;
			end
			default:
				RD_next_state = RD_idle;
		endcase
	end

//State machine of WRITE
	always @(posedge clk) begin
		if(rst)
			WR_cur_state <= WR_idle;
		else
			WR_cur_state <= WR_next_state;		
	end

	always @(*) begin
		case(WR_cur_state)
			WR_idle: begin
				if(EN & head_ptr!=tail_ptr & !fifo_is_empty)
					WR_next_state = WR_req;
				else	
					WR_next_state = WR_idle;
			end
			WR_req: begin
				if(wr_req_ready & wr_req_valid)
					WR_next_state = WR_work;
				else if(WR_burst_cnt == burst_time)
					WR_next_state = WR_idle;
				else
					WR_next_state = WR_req;
			end
			WR_work: begin
				if(wr_ready & wr_valid & wr_last)
					WR_next_state = WR_req;
				else
					WR_next_state = WR_work;
			end
			default:
				WR_next_state = WR_idle;
		endcase
	end

//burst time and count 
	assign burst_time_A[26:0]  = dma_size[31:5];
	assign burst_time_A[31:27] = 5'b0;
	assign burst_time_B[0]    = |dma_size[4:0];
	assign burst_time_B[31:1] = 31'b0;
	assign burst_time = burst_time_A + burst_time_B;

	always @(posedge clk) begin
		if(rst)
			RD_burst_cnt <= 32'b0;
		else if(RD_cur_state[0] & WR_cur_state[0])
			RD_burst_cnt <= 32'b0;
		else begin
			if(RD_cur_state[1] & rd_req_ready & rd_req_valid)	//the beginning of a burst
				RD_burst_cnt <= RD_burst_cnt +32'b1;
			else
				RD_burst_cnt <= RD_burst_cnt;
		end
	end

	always @(posedge clk) begin
		if(rst)
			WR_burst_cnt <= 32'b0;
		else if(RD_cur_state[0] & WR_cur_state[0])	//zero out after a transfer
			WR_burst_cnt <= 32'b0;
		else begin
			if(WR_cur_state[1] & wr_req_ready & wr_req_valid)	//the beginning of a burst
				WR_burst_cnt <= WR_burst_cnt +32'b1;
			else
				WR_burst_cnt <= WR_burst_cnt;
		end
	end
		
//len of burst, transmission time of 4 byte data
	assign last_burst_len_A[2:0] = dma_size[4:2];
	assign last_burst_len_A[4:3] = 2'b0;
	assign last_burst_len_B[0]   = |dma_size[1:0];
	assign last_burst_len_B[4:1] = 4'b0;
	assign last_burst_len = last_burst_len_A + last_burst_len_B -1 ; //len is 0-7

	assign rd_req_len = RD_burst_cnt == burst_time ? last_burst_len : 5'd7;
	assign wr_req_len = WR_burst_cnt == burst_time ? last_burst_len : 5'd7;

//addr of READ and WRITE
	always @(posedge clk) begin
		if(RD_cur_state[0] & WR_cur_state[0] & head_ptr != tail_ptr)
			rd_req_addr <= src_base + tail_ptr;
		else if(RD_cur_state[2] & rd_ready & rd_valid & rd_last) begin
			if(RD_burst_cnt == burst_time)
				rd_req_addr <= rd_req_addr + dma_size[4:0];
			else
				rd_req_addr <= rd_req_addr + 32'd32;
		end
	end

	always @(posedge clk) begin
		if(RD_cur_state[0] & WR_cur_state[0] & head_ptr != tail_ptr)
			wr_req_addr <= dest_base + tail_ptr;
		else if(WR_cur_state[2] & wr_ready & wr_valid & wr_last) begin
			if(WR_burst_cnt == burst_time)
				wr_req_addr <= wr_req_addr + dma_size[4:0];
			else
				wr_req_addr <= wr_req_addr + 32'd32;
		end
	end

//handshake signal of read and write
	assign rd_req_valid = RD_cur_state[1] & ~fifo_is_full & RD_burst_cnt!=burst_time;
	assign wr_req_valid = WR_cur_state[1] & ~fifo_is_empty & WR_burst_cnt != burst_time;

	assign rd_ready = RD_cur_state[2] & ~fifo_is_full;
	assign wr_valid = fifo_rden_one_clk_delay;

	assign fifo_wen = rd_ready & rd_valid;
	assign fifo_wdata = rd_rdata;

	assign fifo_rden = WR_cur_state[2] & ~fifo_is_empty & wr_ready;
	
	assign wr_data = fifo_rdata;
	//考虑到wr_ready中间突然拉低几个周期，fifo_rden有效时读出来的数据应当进行保存，
	//且应当在之后wr_ready拉高的第一个周期输出，因此此时valid应当为高，不可直接用fifo_rden延后一周期实现
	always @ (posedge clk) begin
		if(~wr_ready)
			fifo_rden_one_clk_delay <= fifo_rden_one_clk_delay;
		else
			fifo_rden_one_clk_delay <= fifo_rden;
	end

//last shifter for wr
	always @(posedge clk) begin
		if(wr_req_ready & wr_req_valid)
			last_shifter <= (9'b1<<(wr_req_len+1));
		else if(fifo_rden)
			last_shifter <= {1'b0,last_shifter[8:1]};
		else
			last_shifter <= last_shifter;
	end

	assign wr_last = last_shifter[0];

// Remove multi driver
// //intr signals
// 	always @(posedge clk) begin
// 		if(EN & WR_cur_state[1] & RD_burst_cnt == burst_time & WR_burst_cnt == burst_time) //check whether state is necessary
// 		begin
// 			tail_ptr <= tail_ptr +dma_size;
// 			ctrl_stat[31]  <= 1'b1; 
// 			intr 	<= 1'b1;
// 		end
// 	end

endmodule

