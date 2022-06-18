`timescale 10ns / 1ns

`define CACHE_SET	32
`define CACHE_SET_WD    5
`define CACHE_WAY	8
`define TAG_LEN		22
`define LINE_LEN	256

//修改set时需要修改:CACHE_SET CACHE_SET_WD TAG_LEN plru初始化
module dcache_top (
	input	      clk,
	input	      rst,
  
	//CPU interface
	/** CPU memory/IO access request to Cache: valid signal */
	input         from_cpu_mem_req_valid,
	/** CPU memory/IO access request to Cache: 0 for read; 1 for write (when req_valid is high) */
	input         from_cpu_mem_req,
	/** CPU memory/IO access request to Cache: address (4 byte alignment) */
	input  [31:0] from_cpu_mem_req_addr,
	/** CPU memory/IO access request to Cache: 32-bit write data */
	input  [31:0] from_cpu_mem_req_wdata,
	/** CPU memory/IO access request to Cache: 4-bit write strobe */
	input  [ 3:0] from_cpu_mem_req_wstrb,
	/** Acknowledgement from Cache: ready to receive CPU memory access request */
	output        to_cpu_mem_req_ready,
		
	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit read data */
	output [31:0] to_cpu_cache_rsp_data,
	/** Acknowledgement from CPU: Ready to receive read data */
	input         from_cpu_cache_rsp_ready,
		
	//Memory/IO read interface
	/** Cache sending memory/IO read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address
	  * 4 byte alignment for I/O read 
	  * 32 byte alignment for cache read miss */
	output [31:0] to_mem_rd_req_addr,
        /** Cache sending memory read request: burst length
	  * 0 for I/O read (read only one data beat)
	  * 7 for cache read miss (read eight data beats) */
	output [ 7:0] to_mem_rd_req_len,
        /** Acknowledgement from memory: ready to receive memory read request */
	input	      from_mem_rd_req_ready,

	/** Memory return read data: valid signal of one data beat */
	input	      from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data,
	/** Memory return read data: if current data beat is the last in this burst data transmission */
	input	      from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready,

	//Memory/IO write interface
	/** Cache sending memory/IO write request: valid signal */
	output        to_mem_wr_req_valid,
	/** Cache sending memory write request: address
	  * 4 byte alignment for I/O write 
	  * 4 byte alignment for cache write miss
          * 32 byte alignment for cache write-back */
	output [31:0] to_mem_wr_req_addr,
        /** Cache sending memory write request: burst length
          * 0 for I/O write (write only one data beat)
          * 0 for cache write miss (write only one data beat)
          * 7 for cache write-back (write eight data beats) */
	output [ 7:0] to_mem_wr_req_len,
        /** Acknowledgement from memory: ready to receive memory write request */
	input         from_mem_wr_req_ready,

	/** Cache sending memory/IO write data: valid signal for current data beat */
	output        to_mem_wr_data_valid,
	/** Cache sending memory/IO write data: current data beat */
	output [31:0] to_mem_wr_data,
	/** Cache sending memory/IO write data: write strobe
	  * 4'b1111 for cache write-back 
	  * other values for I/O write and cache write miss according to the original CPU request*/ 
	output [ 3:0] to_mem_wr_data_strb,
	/** Cache sending memory/IO write data: if current data beat is the last in this burst data transmission */
	output        to_mem_wr_data_last,
	/** Acknowledgement from memory/IO: ready to receive current data beat */
	input	      from_mem_wr_data_ready
);

  //TODO: Please add your D-Cache code here
	localparam	WAIT 	= 14'b00000000000001 ,
			TAG_RD	= 14'b00000000000010 , 
			EVICT	= 14'b00000000000100 ,
			MEM_WR	= 14'b00000000001000 ,
			TXD	= 14'b00000000010000 ,	//transmit data
			MEM_RD	= 14'b00000000100000 ,
			RECV	= 14'b00000001000000 ,
			REFILL	= 14'b00000010000000 ,
			CACHE_WR= 14'b00000100000000 ,
			CACHE_RD= 14'b00001000000000 ,
			RESP	= 14'b00010000000000 ,
			BY_REQ	= 14'b00100000000000 ,
			BY_TXD	= 14'b01000000000000 ,
			BY_RECV	= 14'b10000000000000 ;
	
	reg [13:0] cur_state;
	reg [13:0] next_state;
	
//signals for storage //使用位宽而非数组，便于初始化
	reg [`CACHE_SET-1:0]	valid0 	;
	reg [`CACHE_SET-1:0]	valid1 	;
        reg [`CACHE_SET-1:0]	valid2 	;
        reg [`CACHE_SET-1:0]	valid3 	;
	reg [`CACHE_SET-1:0]	valid4 	;
	reg [`CACHE_SET-1:0]	valid5 	;
        reg [`CACHE_SET-1:0]	valid6 	;
        reg [`CACHE_SET-1:0]	valid7 	;

	reg [`CACHE_SET-1:0]	dirty0 	;
	reg [`CACHE_SET-1:0]	dirty1 	;
        reg [`CACHE_SET-1:0]	dirty2 	;
        reg [`CACHE_SET-1:0]	dirty3 	;
	reg [`CACHE_SET-1:0]	dirty4 	;
	reg [`CACHE_SET-1:0]	dirty5 	;
        reg [`CACHE_SET-1:0]	dirty6 	;
        reg [`CACHE_SET-1:0]	dirty7 	;
	
//port related to tag_array and data_array, ignore fixed data
        wire TagWen0;
        wire TagWen1;
        wire TagWen2;
        wire TagWen3;
	wire TagWen4;
        wire TagWen5;
        wire TagWen6;
        wire TagWen7;
        wire [`TAG_LEN-1 : 0] Tag0;
        wire [`TAG_LEN-1 : 0] Tag1;
        wire [`TAG_LEN-1 : 0] Tag2;
        wire [`TAG_LEN-1 : 0] Tag3;
	wire [`TAG_LEN-1 : 0] Tag4;
        wire [`TAG_LEN-1 : 0] Tag5;
        wire [`TAG_LEN-1 : 0] Tag6;
        wire [`TAG_LEN-1 : 0] Tag7;

        wire DataWen0;
        wire DataWen1;
        wire DataWen2;
        wire DataWen3;
	wire DataWen4;
        wire DataWen5;
        wire DataWen6;
        wire DataWen7;
        wire [`LINE_LEN-1:0] Array_Wdata;
        wire [`LINE_LEN-1:0] Data0;
        wire [`LINE_LEN-1:0] Data1;
        wire [`LINE_LEN-1:0] Data2;
        wire [`LINE_LEN-1:0] Data3;
	wire [`LINE_LEN-1:0] Data4;
        wire [`LINE_LEN-1:0] Data5;
        wire [`LINE_LEN-1:0] Data6;
        wire [`LINE_LEN-1:0] Data7;


//signal of CPU inst analysation
	reg 		cpu_mem_rw; //0 read 1 write
	reg [31:0]	cpu_mem_addr;
	reg [31:0]	cpu_mem_wdata;
	reg [3:0]	cpu_mem_wstrb;
	wire [`CACHE_SET_WD-1:0] set;
	wire [`TAG_LEN -1:0]	 tag;
	wire [4:0]	offset; //block保持256bit，32byte，由于CPU所给Address最低二位为0，因此offset只可能为0,4,8,...,28

//signals of path
	wire bypath;
	wire dirty; //judge whether to write back
	
//signals about compare result
	wire Hit;
	reg  Hit_tmp; //use in choosing data from cache or mem
	
	wire hit0;
	wire hit1;
	wire hit2;
	wire hit3;
	wire hit4;
	wire hit5;
	wire hit6;
	wire hit7;
	
	reg  choose0; //hit way0 or replace way0
	reg  choose1;
	reg  choose2;
	reg  choose3;
	reg  choose4;
	reg  choose5;
	reg  choose6;
	reg  choose7;
	wire choose0123; //use in PLRU refresh
	wire choose01;
	wire choose45;

//signals related to PLRU algorithm
	//PLRU bit num is way num -1
	reg [`CACHE_WAY-2:0] PLRU[`CACHE_SET-1:0];	//8 3bit reg: for each block, choose 1 way from 4
	
	wire valid;	//set high if each way is valid
	wire way0;	
	wire way1;
	wire way2;
	wire way3;
	wire way4;	
	wire way5;
	wire way6;
	wire way7;
	
//signals of data to cpu read
	wire [`LINE_LEN-1:0]	cache_block_data;
	wire [ 31:0]	cache_final_data;
	
	wire [ 31:0]	bypath_read_data;
	
	reg  [`LINE_LEN-1:0]	mem_block_data;
	wire [ 31:0]	mem_final_data;
	
//signals of data to cpu write
	//note that CPU write only write in cache or mem(bypath)
	//but write back to mem are dirty_block(8byte) or bypath(1byte)
	wire [`LINE_LEN-1:0]	cache_modified_block;	//modify in CACHE_WR
	wire [ 31:0]	cache_modified_final;
	
	//write back to mem
	reg  [7:0]	last_shifter;	//8 bit, each bit show one byte
	reg  [`LINE_LEN-1:0]	dirty_block_data;
	wire [ 31:0]	bypath_write_data;


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
				if(from_cpu_mem_req_valid)
					next_state = TAG_RD;
				else
					next_state = WAIT;
			end
			
			TAG_RD: begin
				if(bypath)
					next_state = BY_REQ;
				else if(cpu_mem_rw & Hit)  //write hit
					next_state = CACHE_WR;
				else if(~cpu_mem_rw & Hit) //read hit
					next_state = CACHE_RD;
				else 
					next_state = EVICT;
			end
			
			EVICT: begin
				if(dirty)
					next_state = MEM_WR;
				else 
					next_state = MEM_RD;
			end
			
			MEM_WR: begin
				if(from_mem_wr_req_ready)
					next_state = TXD;
				else 
					next_state = MEM_WR;
			end
			
			TXD: begin
				if(from_mem_wr_data_ready & to_mem_wr_data_last)
					next_state = MEM_RD;
				else
					next_state = TXD;
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
			
			REFILL: begin
				if(cpu_mem_rw) //write
					next_state = CACHE_WR;
				else 
					next_state = CACHE_RD; 	//save a cycle than CACHE_RD
			end
			
			CACHE_WR: begin
				next_state = WAIT;
			end
			
			CACHE_RD: begin
				next_state = RESP;
			end
			
			RESP: begin
				if(from_cpu_cache_rsp_ready)
					next_state = WAIT;
				else
					next_state = RESP;
			end
			
			BY_REQ: begin
				if(cpu_mem_rw & from_mem_wr_req_ready)	//write
					next_state = BY_TXD;
				else if(~cpu_mem_rw & from_mem_rd_req_ready) //read
					next_state = BY_RECV;
				else
					next_state = BY_REQ;
			end
			
			BY_TXD: begin
				if(from_mem_wr_data_ready & to_mem_wr_data_last)
					next_state = WAIT;
				else
					next_state = BY_TXD;
			end
			
			BY_RECV: begin
				if(from_mem_rd_rsp_valid & from_mem_rd_rsp_last)
					next_state = RESP;
				else
					next_state = BY_RECV;
			end

			default:
				next_state = WAIT;
		endcase
	end
	
//Analyse mem addr, compare and judge path
	always @(posedge clk) begin
		if(cur_state == WAIT & from_cpu_mem_req_valid)
		begin
			cpu_mem_rw    <= from_cpu_mem_req;
			cpu_mem_addr  <= from_cpu_mem_req_addr;
			cpu_mem_wdata <= from_cpu_mem_req_wdata;
			cpu_mem_wstrb <= from_cpu_mem_req_wstrb;
		end
	end
	
	assign tag	= cpu_mem_addr[31:32-`TAG_LEN];
	assign set	= cpu_mem_addr[4 + `CACHE_SET_WD:5];
	assign offset	= cpu_mem_addr[4:0];
	
	//0x00 ~ 0x1F OR above 0x40000000
	assign bypath 	= (~|cpu_mem_addr[31:5]) | (|cpu_mem_addr[31:30]);
	
	assign hit0	= valid0[set] & Tag0 == tag;
	assign hit1	= valid1[set] & Tag1 == tag;
	assign hit2	= valid2[set] & Tag2 == tag;
	assign hit3	= valid3[set] & Tag3 == tag;
	assign hit4	= valid4[set] & Tag4 == tag;
	assign hit5	= valid5[set] & Tag5 == tag;
	assign hit6	= valid6[set] & Tag6 == tag;
	assign hit7	= valid7[set] & Tag7 == tag;
	
	assign Hit 	= hit0 | hit1 | hit2 | hit3 | hit4 | hit5 | hit6 | hit7;
	
//EVICT : use PLRU algorithm
	//when some way are invalid, replace them first by order
	//only when all 4 way are valid, use PLRU to choose
	assign valid =  valid0[set] & valid1[set] & valid2[set] & valid3[set] & 
			valid4[set] & valid5[set] & valid6[set] & valid7[set];

	assign way0 = 	(valid & ~PLRU[set][0] & ~PLRU[set][1] & ~PLRU[set][3]) | 
			(~valid0[set]);
	assign way1 = 	(valid & ~PLRU[set][0] & ~PLRU[set][1] &  PLRU[set][3]) |
			(valid0[set] & ~valid1[set]);
	assign way2 = 	(valid & ~PLRU[set][0] &  PLRU[set][1] & ~PLRU[set][4]) |
			(valid0[set] & valid1[set] & ~valid2[set]);
	assign way3 = 	(valid & ~PLRU[set][0] &  PLRU[set][1] &  PLRU[set][4]) |
			(valid0[set] & valid1[set] & valid2[set] & ~valid3[set]);
	assign way4 = 	(valid &  PLRU[set][0] & ~PLRU[set][2] & ~PLRU[set][5]) | 
			(valid0[set] & valid1[set] & valid2[set] &  valid3[set] & ~valid4[set]);
	assign way5 = 	(valid &  PLRU[set][0] & ~PLRU[set][2] &  PLRU[set][5]) |
			(valid0[set] & valid1[set] & valid2[set] &  valid3[set] &  valid4[set] & ~valid5[set]);
	assign way6 = 	(valid &  PLRU[set][0] &  PLRU[set][2] & ~PLRU[set][6]) |
			(valid0[set] & valid1[set] & valid2[set] &  valid3[set] &  valid4[set] &  valid5[set] & ~valid6[set]);
	assign way7 = 	(valid &  PLRU[set][0] &  PLRU[set][2] &  PLRU[set][6]) |
			(valid0[set] & valid1[set] & valid2[set] &  valid3[set] &  valid4[set] &  valid5[set] &  valid6[set] & ~valid7[set]);
	
	//refresh PLRU
	assign choose0123 = choose0 | choose1 | choose2 | choose3;
	assign choose01 = choose0 | choose1;
	assign choose45 = choose4 | choose5;

	always @(posedge clk) begin
		if(rst) begin 
			PLRU[0]  <= 7'b0; PLRU[1]  <= 7'b0; PLRU[2]  <= 7'b0; PLRU[3]  <= 7'b0; 
			PLRU[4]  <= 7'b0; PLRU[5]  <= 7'b0; PLRU[6]  <= 7'b0; PLRU[7]  <= 7'b0;
                        PLRU[8]  <= 7'b0; PLRU[9]  <= 7'b0; PLRU[10] <= 7'b0; PLRU[11] <= 7'b0; 
                        PLRU[12] <= 7'b0; PLRU[13] <= 7'b0; PLRU[14] <= 7'b0; PLRU[15] <= 7'b0;  
			PLRU[16] <= 7'b0; PLRU[17] <= 7'b0; PLRU[18] <= 7'b0; PLRU[19] <= 7'b0; 
			PLRU[20] <= 7'b0; PLRU[21] <= 7'b0; PLRU[22] <= 7'b0; PLRU[23] <= 7'b0; 
			PLRU[24] <= 7'b0; PLRU[25] <= 7'b0; PLRU[26] <= 7'b0; PLRU[27] <= 7'b0; 
			PLRU[28] <= 7'b0; PLRU[29] <= 7'b0; PLRU[30] <= 7'b0; PLRU[31] <= 7'b0;   
		end
		else if (cur_state == CACHE_RD | cur_state == CACHE_WR) begin //after read or write visit
			PLRU[set][0] <= choose0123;
			if(choose0123)
			begin
				PLRU[set][1] <= choose01;
				if(choose01)
					PLRU[set][3] <= choose0;
				else
					PLRU[set][4] <= choose2;
			end
			else
			begin
				PLRU[set][2] <= choose45;
				if(choose45)
					PLRU[set][5] <= choose4;
				else
					PLRU[set][6] <= choose6;
			end
		end
	end
	
	//judge whether to write back to MEM
	assign dirty = 	( way0 & valid0[set] & dirty0[set] ) |
			( way1 & valid1[set] & dirty1[set] ) |
			( way2 & valid2[set] & dirty2[set] ) |
			( way3 & valid3[set] & dirty3[set] ) |
			( way4 & valid4[set] & dirty4[set] ) |
			( way5 & valid5[set] & dirty5[set] ) |
			( way6 & valid6[set] & dirty6[set] ) |
			( way7 & valid7[set] & dirty7[set] ) ;
					
//choose logic :
	//hit : get cache_data from which way
	//miss: get refresh which way
	//note the init is all 0 because of the valid init
	always @(posedge clk) begin
		if(cur_state == TAG_RD) // if hit, get cache data way
		begin
			Hit_tmp <= Hit;
			choose0 <= hit0;
			choose1 <= hit1;
			choose2 <= hit2;
			choose3 <= hit3;
			choose4 <= hit4;
			choose5 <= hit5;
			choose6 <= hit6;
			choose7 <= hit7;
		end
		if(cur_state == EVICT) //if miss, refresh which way, op before will be ignore
		begin
			choose0 <= way0;
			choose1 <= way1;
			choose2 <= way2;
			choose3 <= way3;
			choose4 <= way4;
			choose5 <= way5;
			choose6 <= way6;
			choose7 <= way7;
		end
	end

//valid / tag / data / dirty array: initialization or refresh
	//valid array
	always @(posedge clk) begin
		if(rst) begin
			valid0[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
                        valid1[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
                        valid2[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
                        valid3[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
			valid4[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
                        valid5[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
                        valid6[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
                        valid7[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
		end
		if(cur_state == REFILL) begin
			if(choose0) 		valid0[set] <= 1'b1;
			else if(choose1)	valid1[set] <= 1'b1;
			else if(choose2)	valid2[set] <= 1'b1;
			else if(choose3)	valid3[set] <= 1'b1;
			else if(choose4) 	valid4[set] <= 1'b1;
			else if(choose5)	valid5[set] <= 1'b1;
			else if(choose6)	valid6[set] <= 1'b1;
			else if(choose7)	valid7[set] <= 1'b1;
		end
	end
	//tag array : when rst, no need to refresh because valid = 0
                //wdata is tag
        assign TagWen0 = cur_state[7] & choose0; //REFILL
        assign TagWen1 = cur_state[7] & choose1;
        assign TagWen2 = cur_state[7] & choose2;
        assign TagWen3 = cur_state[7] & choose3;
	assign TagWen4 = cur_state[7] & choose4; 
        assign TagWen5 = cur_state[7] & choose5;
        assign TagWen6 = cur_state[7] & choose6;
        assign TagWen7 = cur_state[7] & choose7;

	//data array
        //读写REFILL均需更新，因为CACHE写只会进行部分更新
        assign DataWen0 = (cur_state[7] | cur_state[8]) & choose0; //REFILL or CACHE_WR
        assign DataWen1 = (cur_state[7] | cur_state[8]) & choose1;
        assign DataWen2 = (cur_state[7] | cur_state[8]) & choose2;
        assign DataWen3 = (cur_state[7] | cur_state[8]) & choose3;
	assign DataWen4 = (cur_state[7] | cur_state[8]) & choose4; //REFILL or CACHE_WR
        assign DataWen5 = (cur_state[7] | cur_state[8]) & choose5;
        assign DataWen6 = (cur_state[7] | cur_state[8]) & choose6;
        assign DataWen7 = (cur_state[7] | cur_state[8]) & choose7;

        assign Array_Wdata =    {`LINE_LEN{cur_state[7]}} & mem_block_data | //REFILL
                                {`LINE_LEN{cur_state[8]}} & cache_modified_block ;

	//dirty array
	always @(posedge clk) begin
		if (rst) begin
                        dirty0[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
                        dirty1[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
                        dirty2[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
                        dirty3[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
			dirty4[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
                        dirty5[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
                        dirty6[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
                        dirty7[`CACHE_SET-1:0] <= {`CACHE_SET{1'b0}};
		end
		else if (cur_state == CACHE_WR) begin
			if (choose0) 		dirty0[set] <= 1'b1;
			else if (choose1) 	dirty1[set] <= 1'b1;
			else if (choose2) 	dirty2[set] <= 1'b1;
			else if (choose3) 	dirty3[set] <= 1'b1;
			else if (choose4) 	dirty4[set] <= 1'b1;
			else if (choose5) 	dirty5[set] <= 1'b1;
			else if (choose6) 	dirty6[set] <= 1'b1;
			else if (choose7) 	dirty7[set] <= 1'b1;
		end
		else if (cur_state == REFILL) begin //refill cache with mem data, reset dirty
			if (choose0) 		dirty0[set] <= 1'b0;
			else if (choose1) 	dirty1[set] <= 1'b0;
			else if (choose2) 	dirty2[set] <= 1'b0;
			else if (choose3) 	dirty3[set] <= 1'b0;
			else if (choose4) 	dirty4[set] <= 1'b0;
			else if (choose5) 	dirty5[set] <= 1'b0;
			else if (choose6) 	dirty6[set] <= 1'b0;
			else if (choose7) 	dirty7[set] <= 1'b0;
		end
    	end
   
//READ : final data to cpu  -- source : cache(hit)/mem(miss)/bypath
	//data from cache
	assign cache_block_data = 	( {`LINE_LEN{choose0}} & Data0 ) |
					( {`LINE_LEN{choose1}} & Data1 ) |
					( {`LINE_LEN{choose2}} & Data2 ) |
					( {`LINE_LEN{choose3}} & Data3 ) |
					( {`LINE_LEN{choose4}} & Data4 ) |
					( {`LINE_LEN{choose5}} & Data5 ) |
					( {`LINE_LEN{choose6}} & Data6 ) |
					( {`LINE_LEN{choose7}} & Data7 ) ;
	assign cache_final_data = cache_block_data[ {offset,3'b0} +: 32 ];
	
	//data from mem / bypath
	assign to_mem_rd_req_addr[31:5] = cpu_mem_addr[31:5];
	assign to_mem_rd_req_addr[4:0]	= bypath ? cpu_mem_addr[4:0] : 5'b0;
	
	assign to_mem_rd_req_len = {5'b0 , {3{~bypath}} }; //bypath 0 other 7

	always @(posedge clk) begin
		if((cur_state == RECV | cur_state == BY_RECV) & from_mem_rd_rsp_valid)
			mem_block_data <= {from_mem_rd_rsp_data , mem_block_data[255:32]};
	end	//result : {data7,...,data0} or {data,x,..,x}
	
	assign mem_final_data = mem_block_data[ {offset,3'b0} +: 32 ];
	assign bypath_read_data = mem_block_data [255:224];
	
	//choose source
	assign to_cpu_cache_rsp_data = 	( {32{ bypath}} & bypath_read_data)  	|
					( {32{~bypath}} & cache_final_data) 	;
	
//WRITE : final data from cpu --target : cache(hit or miss) / mem(bypath)
		//note write back to MEM is dirty block or bypath
	//cpu write
	assign cache_modified_final = {
					{ ({8{cpu_mem_wstrb[3]}} & cpu_mem_wdata[31:24]) | ({8{~cpu_mem_wstrb[3]}} & cache_final_data[31:24]) },
					{ ({8{cpu_mem_wstrb[2]}} & cpu_mem_wdata[23:16]) | ({8{~cpu_mem_wstrb[2]}} & cache_final_data[23:16]) },
					{ ({8{cpu_mem_wstrb[1]}} & cpu_mem_wdata[15: 8]) | ({8{~cpu_mem_wstrb[1]}} & cache_final_data[15: 8]) },
					{ ({8{cpu_mem_wstrb[0]}} & cpu_mem_wdata[ 7: 0]) | ({8{~cpu_mem_wstrb[0]}} & cache_final_data[ 7: 0]) }
					 }; 
	// assign cache_modified_block =   ~|offset ? {cache_block_data[`LINE_LEN-1 : 32],cache_modified_final} :   //offset =0
        //                                 &offset  ? {cache_modified_final,cache_block_data[`LINE_LEN -33:0]} : //offset=31
        //                                 {cache_block_data[`LINE_LEN-1 : {offset,3'b0}+8'd32],cache_modified_final,cache_block_data[{offset,3'b0}-1:0]} ;

        //offset最低两位为0，即4字节对齐
        assign cache_modified_block = 
        {`LINE_LEN{offset[4:2]==3'b000 }} & {cache_block_data[`LINE_LEN-1 : 32],cache_modified_final} |
        {`LINE_LEN{offset[4:2]==3'b001 }} & {cache_block_data[`LINE_LEN-1 : 64],cache_modified_final,cache_block_data[31:0]} |
        {`LINE_LEN{offset[4:2]==3'b010 }} & {cache_block_data[`LINE_LEN-1 : 96],cache_modified_final,cache_block_data[63:0]} |
        {`LINE_LEN{offset[4:2]==3'b011 }} & {cache_block_data[`LINE_LEN-1 : 128],cache_modified_final,cache_block_data[95:0]} |
        {`LINE_LEN{offset[4:2]==3'b100 }} & {cache_block_data[`LINE_LEN-1 : 160],cache_modified_final,cache_block_data[127:0]} |
        {`LINE_LEN{offset[4:2]==3'b101 }} & {cache_block_data[`LINE_LEN-1 : 192],cache_modified_final,cache_block_data[159:0]} |
        {`LINE_LEN{offset[4:2]==3'b110 }} & {cache_block_data[`LINE_LEN-1 : 224],cache_modified_final,cache_block_data[191:0]} |
        {`LINE_LEN{offset[4:2]==3'b111 }} & {cache_modified_final,cache_block_data[223:0]} ;
       
	//finally refresh to data array
	
	assign bypath_write_data = cpu_mem_wdata;
	//write back to MEM 
	assign to_mem_wr_req_addr = ( {32{bypath}} & cpu_mem_addr ) 			|
				    ( {32{~bypath & choose0}} & {Tag0,set,5'b0} )	|
				    ( {32{~bypath & choose1}} & {Tag1,set,5'b0} )	|
				    ( {32{~bypath & choose2}} & {Tag2,set,5'b0} )	|
				    ( {32{~bypath & choose3}} & {Tag3,set,5'b0} )	|
				    ( {32{~bypath & choose4}} & {Tag4,set,5'b0} )	|
				    ( {32{~bypath & choose5}} & {Tag5,set,5'b0} )	|
				    ( {32{~bypath & choose6}} & {Tag6,set,5'b0} )	|
				    ( {32{~bypath & choose7}} & {Tag7,set,5'b0} )	;
				    
	assign to_mem_wr_req_len = {5'b0,{3{~bypath}}};
	
	//use last shifter to get last signal
	always @(posedge clk) begin
		if(cur_state == MEM_WR | cur_state == BY_REQ) begin
			if(bypath)
				last_shifter <= 8'b1;
			else
				last_shifter <= {1'b1,7'b0};
		end
		else if(cur_state == TXD & from_mem_wr_data_ready) begin
			last_shifter <= {1'b0,last_shifter[7:1]};
		end
	end
	
	//use dirty block shifter to transmit it 4byte each time
	always @(posedge clk) begin
		if(cur_state == MEM_WR) begin
			dirty_block_data <= cache_block_data;
		end
		else if(cur_state == TXD & from_mem_wr_data_ready) begin
			dirty_block_data <= {32'b0 , dirty_block_data[255:32]};
		end
	end
	
	assign to_mem_wr_data_last = last_shifter[0] & from_mem_wr_data_ready & to_mem_wr_data_valid;
	assign to_mem_wr_data = ( {32{ bypath}} & bypath_write_data) 	  |
				( {32{~bypath}} & dirty_block_data[31:0]) ;
	//for dirty block, strb are always 4'b1111 to ensure complete data transmit
	//for bypath , depends on input port
	assign to_mem_wr_data_strb = {4{~bypath}} | cpu_mem_wstrb;

//handshake signals related to state machine
	assign to_cpu_mem_req_ready   = cur_state == WAIT;
	assign to_cpu_cache_rsp_valid = cur_state == RESP;
	
	assign to_mem_rd_req_valid  = (cur_state == MEM_RD) | (cur_state == BY_REQ & ~cpu_mem_rw);
	assign to_mem_rd_rsp_ready  = (cur_state == RECV) | (cur_state == BY_RECV) | (cur_state == WAIT);
	
	assign to_mem_wr_req_valid  = (cur_state == MEM_WR) | (cur_state == BY_REQ & cpu_mem_rw);
	assign to_mem_wr_data_valid = (cur_state == TXD) | (cur_state == BY_TXD);

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
	tag_array tag_way4(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (TagWen4),
                .wdata  (tag),
                .rdata  (Tag4)
        );
        tag_array tag_way5(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (TagWen5),
                .wdata  (tag),
                .rdata  (Tag5)
        );
        tag_array tag_way6(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (TagWen6),
                .wdata  (tag),
                .rdata  (Tag6)
        );
        tag_array tag_way7(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (TagWen7),
                .wdata  (tag),
                .rdata  (Tag7)
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
        data_array data_way4(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (DataWen4),
                .wdata  (Array_Wdata),
                .rdata  (Data4)
        );
        data_array data_way5(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (DataWen5),
                .wdata  (Array_Wdata),
                .rdata  (Data5)
        );
        data_array data_way6(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (DataWen6),
                .wdata  (Array_Wdata),
                .rdata  (Data6)
        );
        data_array data_way7(
                .clk    (clk),
                .waddr  (set),
                .raddr  (set),
                .wen    (DataWen7),
                .wdata  (Array_Wdata),
                .rdata  (Data7)
        );



endmodule

