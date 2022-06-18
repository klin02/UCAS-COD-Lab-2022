`timescale 10ns / 1ns

`define CACHE_SET	256
`define CACHE_SET_WD    8
`define CACHE_WAY	4
`define TAG_LEN		19
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
                        PLRU[8] <= 3'b0; PLRU[9] <= 3'b0; PLRU[10] <= 3'b0; PLRU[11] <= 3'b0; 
                        PLRU[12] <= 3'b0; PLRU[13] <= 3'b0; PLRU[14] <= 3'b0; PLRU[15] <= 3'b0;  
			PLRU[16] <= 3'b0; PLRU[17] <= 3'b0; PLRU[18] <= 3'b0; PLRU[19] <= 3'b0; 
			PLRU[20] <= 3'b0; PLRU[21] <= 3'b0; PLRU[22] <= 3'b0; PLRU[23] <= 3'b0; 
			PLRU[24] <= 3'b0; PLRU[25] <= 3'b0; PLRU[26] <= 3'b0; PLRU[27] <= 3'b0; 
			PLRU[28] <= 3'b0; PLRU[29] <= 3'b0; PLRU[30] <= 3'b0; PLRU[31] <= 3'b0; 
			PLRU[32] <= 3'b0; PLRU[33] <= 3'b0; PLRU[34] <= 3'b0; PLRU[35] <= 3'b0; 
			PLRU[36] <= 3'b0; PLRU[37] <= 3'b0; PLRU[38] <= 3'b0; PLRU[39] <= 3'b0; 
			PLRU[40] <= 3'b0; PLRU[41] <= 3'b0; PLRU[42] <= 3'b0; PLRU[43] <= 3'b0; 
			PLRU[44] <= 3'b0; PLRU[45] <= 3'b0; PLRU[46] <= 3'b0; PLRU[47] <= 3'b0; 
			PLRU[48] <= 3'b0; PLRU[49] <= 3'b0; PLRU[50] <= 3'b0; PLRU[51] <= 3'b0; 
			PLRU[52] <= 3'b0; PLRU[53] <= 3'b0; PLRU[54] <= 3'b0; PLRU[55] <= 3'b0; 
			PLRU[56] <= 3'b0; PLRU[57] <= 3'b0; PLRU[58] <= 3'b0; PLRU[59] <= 3'b0; 
			PLRU[60] <= 3'b0; PLRU[61] <= 3'b0; PLRU[62] <= 3'b0; PLRU[63] <= 3'b0; 
			PLRU[64] <= 3'b0; PLRU[65] <= 3'b0; PLRU[66] <= 3'b0; PLRU[67] <= 3'b0; 
			PLRU[68] <= 3'b0; PLRU[69] <= 3'b0; PLRU[70] <= 3'b0; PLRU[71] <= 3'b0; 
			PLRU[72] <= 3'b0; PLRU[73] <= 3'b0; PLRU[74] <= 3'b0; PLRU[75] <= 3'b0; 
			PLRU[76] <= 3'b0; PLRU[77] <= 3'b0; PLRU[78] <= 3'b0; PLRU[79] <= 3'b0; 
			PLRU[80] <= 3'b0; PLRU[81] <= 3'b0; PLRU[82] <= 3'b0; PLRU[83] <= 3'b0; 
			PLRU[84] <= 3'b0; PLRU[85] <= 3'b0; PLRU[86] <= 3'b0; PLRU[87] <= 3'b0; 
			PLRU[88] <= 3'b0; PLRU[89] <= 3'b0; PLRU[90] <= 3'b0; PLRU[91] <= 3'b0; 
			PLRU[92] <= 3'b0; PLRU[93] <= 3'b0; PLRU[94] <= 3'b0; PLRU[95] <= 3'b0; 
			PLRU[96] <= 3'b0; PLRU[97] <= 3'b0; PLRU[98] <= 3'b0; PLRU[99] <= 3'b0; 
			PLRU[100] <= 3'b0; PLRU[101] <= 3'b0; PLRU[102] <= 3'b0; PLRU[103] <= 3'b0; 
			PLRU[104] <= 3'b0; PLRU[105] <= 3'b0; PLRU[106] <= 3'b0; PLRU[107] <= 3'b0; 
			PLRU[108] <= 3'b0; PLRU[109] <= 3'b0; PLRU[110] <= 3'b0; PLRU[111] <= 3'b0; 
			PLRU[112] <= 3'b0; PLRU[113] <= 3'b0; PLRU[114] <= 3'b0; PLRU[115] <= 3'b0; 
			PLRU[116] <= 3'b0; PLRU[117] <= 3'b0; PLRU[118] <= 3'b0; PLRU[119] <= 3'b0; 
			PLRU[120] <= 3'b0; PLRU[121] <= 3'b0; PLRU[122] <= 3'b0; PLRU[123] <= 3'b0; 
			PLRU[124] <= 3'b0; PLRU[125] <= 3'b0; PLRU[126] <= 3'b0; PLRU[127] <= 3'b0; 
			PLRU[128] <= 3'b0; PLRU[129] <= 3'b0; PLRU[130] <= 3'b0; PLRU[131] <= 3'b0; 
			PLRU[132] <= 3'b0; PLRU[133] <= 3'b0; PLRU[134] <= 3'b0; PLRU[135] <= 3'b0; 
			PLRU[136] <= 3'b0; PLRU[137] <= 3'b0; PLRU[138] <= 3'b0; PLRU[139] <= 3'b0; 
			PLRU[140] <= 3'b0; PLRU[141] <= 3'b0; PLRU[142] <= 3'b0; PLRU[143] <= 3'b0;
			PLRU[144] <= 3'b0; PLRU[145] <= 3'b0; PLRU[146] <= 3'b0; PLRU[147] <= 3'b0; 
			PLRU[148] <= 3'b0; PLRU[149] <= 3'b0; PLRU[150] <= 3'b0; PLRU[151] <= 3'b0; 
			PLRU[152] <= 3'b0; PLRU[153] <= 3'b0; PLRU[154] <= 3'b0; PLRU[155] <= 3'b0; 
			PLRU[156] <= 3'b0; PLRU[157] <= 3'b0; PLRU[158] <= 3'b0; PLRU[159] <= 3'b0;
			PLRU[160] <= 3'b0; PLRU[161] <= 3'b0; PLRU[162] <= 3'b0; PLRU[163] <= 3'b0; 
			PLRU[164] <= 3'b0; PLRU[165] <= 3'b0; PLRU[166] <= 3'b0; PLRU[167] <= 3'b0; 
			PLRU[168] <= 3'b0; PLRU[169] <= 3'b0; PLRU[170] <= 3'b0; PLRU[171] <= 3'b0; 
			PLRU[172] <= 3'b0; PLRU[173] <= 3'b0; PLRU[174] <= 3'b0; PLRU[175] <= 3'b0;
			PLRU[176] <= 3'b0; PLRU[177] <= 3'b0; PLRU[178] <= 3'b0; PLRU[179] <= 3'b0; 
			PLRU[180] <= 3'b0; PLRU[181] <= 3'b0; PLRU[182] <= 3'b0; PLRU[183] <= 3'b0; 
			PLRU[184] <= 3'b0; PLRU[185] <= 3'b0; PLRU[186] <= 3'b0; PLRU[187] <= 3'b0; 
			PLRU[188] <= 3'b0; PLRU[189] <= 3'b0; PLRU[190] <= 3'b0; PLRU[191] <= 3'b0;
			PLRU[192] <= 3'b0; PLRU[193] <= 3'b0; PLRU[194] <= 3'b0; PLRU[195] <= 3'b0; 
			PLRU[196] <= 3'b0; PLRU[197] <= 3'b0; PLRU[198] <= 3'b0; PLRU[199] <= 3'b0; 
			PLRU[200] <= 3'b0; PLRU[201] <= 3'b0; PLRU[202] <= 3'b0; PLRU[203] <= 3'b0; 
			PLRU[204] <= 3'b0; PLRU[205] <= 3'b0; PLRU[206] <= 3'b0; PLRU[207] <= 3'b0; 
			PLRU[208] <= 3'b0; PLRU[209] <= 3'b0; PLRU[210] <= 3'b0; PLRU[211] <= 3'b0; 
			PLRU[212] <= 3'b0; PLRU[213] <= 3'b0; PLRU[214] <= 3'b0; PLRU[215] <= 3'b0; 
			PLRU[216] <= 3'b0; PLRU[217] <= 3'b0; PLRU[218] <= 3'b0; PLRU[219] <= 3'b0; 
			PLRU[220] <= 3'b0; PLRU[221] <= 3'b0; PLRU[222] <= 3'b0; PLRU[223] <= 3'b0; 
			PLRU[224] <= 3'b0; PLRU[225] <= 3'b0; PLRU[226] <= 3'b0; PLRU[227] <= 3'b0; 
			PLRU[228] <= 3'b0; PLRU[229] <= 3'b0; PLRU[230] <= 3'b0; PLRU[231] <= 3'b0; 
			PLRU[232] <= 3'b0; PLRU[233] <= 3'b0; PLRU[234] <= 3'b0; PLRU[235] <= 3'b0; 
			PLRU[236] <= 3'b0; PLRU[237] <= 3'b0; PLRU[238] <= 3'b0; PLRU[239] <= 3'b0; 
			PLRU[240] <= 3'b0; PLRU[241] <= 3'b0; PLRU[242] <= 3'b0; PLRU[243] <= 3'b0; 
			PLRU[244] <= 3'b0; PLRU[245] <= 3'b0; PLRU[246] <= 3'b0; PLRU[247] <= 3'b0; 
			PLRU[248] <= 3'b0; PLRU[249] <= 3'b0; PLRU[250] <= 3'b0; PLRU[251] <= 3'b0; 
			PLRU[252] <= 3'b0; PLRU[253] <= 3'b0; PLRU[254] <= 3'b0; PLRU[255] <= 3'b0;  
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

