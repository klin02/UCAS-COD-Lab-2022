`timescale 10ns / 1ns

`define CACHE_SET	8
`define CACHE_SET_WD    3
`define CACHE_WAY	4
`define TAG_LEN		24
`define LINE_LEN	256

//修改set时需要修改:CACHE_SET CACHE_SET_WD TAG_LEN plru初始化
module icache_top (
	input	      clk,
	input	      rst,
	
	//CPU interface
	/** CPU instruction fetch request to Cache: valid signal */
	input         from_cpu_inst_req_valid,
	/** CPU instruction fetch request to Cache: address (4 byte alignment) */
	input  [31:0] from_cpu_inst_req_addr,
	/** Acknowledgement from Cache: ready to receive CPU instruction fetch request */
	output        to_cpu_inst_req_ready,
	
	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit Instruction value */
	output [31:0] to_cpu_cache_rsp_data,
	/** Acknowledgement from CPU: Ready to receive Instruction */
	input	      from_cpu_cache_rsp_ready,

	//Memory interface (32 byte aligned address)
	/** Cache sending memory read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address (32 byte alignment) */
	output [31:0] to_mem_rd_req_addr,
	/** Acknowledgement from memory: ready to receive memory read request */
	input         from_mem_rd_req_ready,

	/** Memory return read data: valid signal of one data beat */
	input         from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data,
	/** Memory return read data: if current data beat is the last in this burst data transmission */
	input         from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready
);

	//TODO: Please add your I-Cache code here
	localparam 	WAIT		= 8'b00000001,
			TAG_RD		= 8'b00000010,
			EVICT		= 8'b00000100,
			MEM_RD		= 8'b00001000,
			RECV		= 8'b00010000,
			CACHE_RD	= 8'b00100000,
			REFILL		= 8'b01000000,
			RESP		= 8'b10000000;
	
	reg [7:0]	cur_state;
	reg [7:0]	next_state;

//4*16 block，每路组中block翻倍，set位数加1，tag位数减1

//signals of storage : 4 way , 8 block each way
	reg [`CACHE_SET-1:0]	valid0 	;
	reg [`CACHE_SET-1:0]	valid1 	;
        reg [`CACHE_SET-1:0]	valid2 	;
        reg [`CACHE_SET-1:0]	valid3 	;
	
//port related to tag_array and data_array, ignore fixed data
        wire TagWen0;
        wire TagWen1;
        wire TagWen2;
        wire TagWen3;
        wire [`TAG_LEN-1 : 0] Tag0;
        wire [`TAG_LEN-1 : 0] Tag1;
        wire [`TAG_LEN-1 : 0] Tag2;
        wire [`TAG_LEN-1 : 0] Tag3;

        wire DataWen0;
        wire DataWen1;
        wire DataWen2;
        wire DataWen3;
        wire [`LINE_LEN-1:0] Array_Wdata;
        wire [`LINE_LEN-1:0] Data0;
        wire [`LINE_LEN-1:0] Data1;
        wire [`LINE_LEN-1:0] Data2;
        wire [`LINE_LEN-1:0] Data3;

//signal of CPU inst analysation
	reg [31:0]	cpu_inst_addr;
	wire [`CACHE_SET_WD-1:0] set;
	wire [`TAG_LEN -1:0]	 tag;
	wire [4:0]	offset;
	
//signals about compare result
	wire Read_Hit;
	reg  Hit_reg; //use in choosing data from cache or mem
	
	wire hit0;
	wire hit1;
	wire hit2;
	wire hit3;
	
	reg  choose0; //hit way0 or replace way0
	reg  choose1;
	reg  choose2;
	reg  choose3;
	wire choose01; //use in PLRU refresh
	
//signals related to PLRU algorithm
	reg [2:0] PLRU[`CACHE_SET-1:0];	//3bit reg: for each block, choose 1 way from 4
	
	wire valid;	//set high if each way is valid
	wire way0;	
	wire way1;
	wire way2;
	wire way3;
	
//signals of data to cpu
	wire [`LINE_LEN-1:0]	cache_block_data;
	wire [ 31:0]	cache_final_data;
	
	reg  [`LINE_LEN-1:0]	mem_block_data;
	wire [ 31:0]	mem_final_data;
	
//state machine
	always @(posedge clk) begin
		if(rst)
			cur_state <= WAIT;
		else 
			cur_state <= next_state;
	end
	
	always @(*) begin
		case (cur_state)
			WAIT: begin
				if(from_cpu_inst_req_valid)
					next_state = TAG_RD;
				else
					next_state = WAIT;
			end
			
			TAG_RD: begin
				if(~Read_Hit)
					next_state = EVICT;
				else if(Read_Hit)
					next_state = CACHE_RD;
				else
					next_state = TAG_RD;
			end
			
			EVICT: begin
				next_state = MEM_RD;
			end
			
			MEM_RD: begin
				if(from_mem_rd_req_ready)
					next_state = RECV;
				else 
					next_state = MEM_RD;
			end
			
			RECV: begin
				if(from_mem_rd_rsp_valid & from_mem_rd_rsp_last)
					next_state = REFILL;
				else 
					next_state = RECV;
			end
			
			CACHE_RD: begin
				next_state = RESP;
			end
			
			REFILL: begin
				next_state = RESP;
			end
			
			RESP: begin
				if(from_cpu_cache_rsp_ready)
					next_state = WAIT;
				else
					next_state = RESP;
			end
			
			default:
				next_state = WAIT;
		endcase
	end
	
//Analyse inst addr and compare first
	always @(posedge clk) begin
		if(cur_state == WAIT)
			cpu_inst_addr <= from_cpu_inst_req_addr;
	end
	
	assign tag	= cpu_inst_addr[31:32-`TAG_LEN];
	assign set	= cpu_inst_addr[4 + `CACHE_SET_WD:5];
	assign offset	= cpu_inst_addr[4:0];
	
	assign hit0	= valid0[set] & Tag0 == tag;
	assign hit1	= valid1[set] & Tag1 == tag;
	assign hit2	= valid2[set] & Tag2 == tag;
	assign hit3	= valid3[set] & Tag3 == tag;
	
	assign Read_Hit = hit0 | hit1 | hit2 | hit3;
	
//EVICT : use PLRU algorithm
	//when some way are invalid, replace them first by order
	//only when all 4 way are valid, use PLRU to choose
	assign valid = valid0[set] & valid1[set] & valid2[set] & valid3[set];
	assign way0 = 	(valid & ~PLRU[set][1] & ~PLRU[set][0]) | 
			(~valid0[set]);
	assign way1 = 	(valid &  PLRU[set][1] & ~PLRU[set][0]) |
			(valid0[set] & ~valid1[set]);
	assign way2 = 	(valid & ~PLRU[set][2] &  PLRU[set][0]) |
			(valid0[set] & valid1[set] & ~valid2[set]);
	assign way3 = 	(valid &  PLRU[set][2] &  PLRU[set][0]) |
			(valid0[set] & valid1[set] & valid2[set] & ~valid3[set]);
	
	//refresh PLRU
	assign choose01 = choose0 | choose1;
	always @(posedge clk) begin
		if(rst) begin //or use generate instead
			PLRU[0] <= 3'b0; PLRU[1] <= 3'b0; PLRU[2] <= 3'b0; PLRU[3] <= 3'b0; 
			PLRU[4] <= 3'b0; PLRU[5] <= 3'b0; PLRU[6] <= 3'b0; PLRU[7] <= 3'b0;
		end
		else if (cur_state == RESP) begin
			PLRU[set][0] <= choose01;
			if(choose01) 
				PLRU[set][1] <= choose0;
			else 
				PLRU[set][2] <= choose2;
		end
	end
	
//choose logic :
	//hit : get cache_data from which way
	//miss: get refresh which way
	//note the init is all 0 because of the valid init
	always @(posedge clk) begin
		if(cur_state == TAG_RD) // if hit, get cache data way
		begin
			Hit_reg <= Read_Hit;
			choose0 <= hit0;
			choose1 <= hit1;
			choose2 <= hit2;
			choose3 <= hit3;
		end
		if(cur_state == EVICT) //if miss, refresh which way, op before will be ignore
		begin
			choose0 <= way0;
			choose1 <= way1;
			choose2 <= way2;
			choose3 <= way3;
		end
	end
	
//valid / tag / data array: initialization or refresh
	//valid array
	always @(posedge clk) begin
		if(rst) begin
			valid0[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
                        valid1[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
                        valid2[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
                        valid3[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
		end
		else if(cur_state == REFILL) begin
			if(choose0) 		valid0[set] <= 1'b1;
			else if(choose1)	valid1[set] <= 1'b1;
			else if(choose2)	valid2[set] <= 1'b1;
			else if(choose3)	valid3[set] <= 1'b1;
		end
	end
	//tag array : when rst, no need to refresh because valid = 0
	assign TagWen0 = cur_state[6] & choose0; //REFILL
        assign TagWen1 = cur_state[6] & choose1;
        assign TagWen2 = cur_state[6] & choose2;
        assign TagWen3 = cur_state[6] & choose3;

	//data array
	assign DataWen0 = cur_state[6] & choose0;
	assign DataWen1 = cur_state[6] & choose1;
	assign DataWen2 = cur_state[6] & choose2;
	assign DataWen3 = cur_state[6] & choose3;

	assign Array_Wdata = mem_block_data;
	
//final data to cpu
	//data from cache
	assign cache_block_data = 	( {`LINE_LEN{choose0}} & Data0 ) |
					( {`LINE_LEN{choose1}} & Data1 ) |
					( {`LINE_LEN{choose2}} & Data2 ) |
					( {`LINE_LEN{choose3}} & Data3 ) ;
	assign cache_final_data = cache_block_data[ {offset,3'b0} +: 32 ];
	
	//data from mem
	assign to_mem_rd_req_addr = {cpu_inst_addr[31:5],5'b0};
	
	always @(posedge clk) begin
		if(cur_state == RECV & from_mem_rd_rsp_valid)
			mem_block_data <= {from_mem_rd_rsp_data , mem_block_data[255:32]};
	end	//result : {data7,...,data0}
	
	assign mem_final_data = mem_block_data[ {offset,3'b0} +: 32 ];
	
	//choose source
	assign to_cpu_cache_rsp_data = 	( {32{ Hit_reg}} & cache_final_data) |
					( {32{~Hit_reg}} & mem_final_data)   ;
	
//handshake signal related to state machine
	assign to_cpu_inst_req_ready 	= cur_state[0]; //WAIT
	assign to_cpu_cache_rsp_valid 	= cur_state[7]; //RESP
	assign to_mem_rd_req_valid 	= cur_state[3]; //MEM_RD
	assign to_mem_rd_rsp_ready 	= cur_state[4] | (cur_state == WAIT); //RECV

//inst of tag_array and data_array
        tag_array tag_way0(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (TagWen0),
                .wdata  (tag),
                .rdata  (Tag0)
        );
        tag_array tag_way1(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (TagWen1),
                .wdata  (tag),
                .rdata  (Tag1)
        );
        tag_array tag_way2(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (TagWen2),
                .wdata  (tag),
                .rdata  (Tag2)
        );
        tag_array tag_way3(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (TagWen3),
                .wdata  (tag),
                .rdata  (Tag3)
        );

        data_array data_way0(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (DataWen0),
                .wdata  (Array_Wdata),
                .rdata  (Data0)
        );
        data_array data_way1(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (DataWen1),
                .wdata  (Array_Wdata),
                .rdata  (Data1)
        );
        data_array data_way2(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (DataWen2),
                .wdata  (Array_Wdata),
                .rdata  (Data2)
        );
        data_array data_way3(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (DataWen3),
                .wdata  (Array_Wdata),
                .rdata  (Data3)
        );
endmodule

