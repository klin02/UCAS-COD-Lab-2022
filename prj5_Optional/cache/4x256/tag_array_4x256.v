`timescale 10 ns / 1 ns

`define TARRAY_DATA_WIDTH 19
`define TARRAY_ADDR_WIDTH 8
`define TARRAY_SET 256
//修改set时需修改，TARRAT_SET TARRT_ADDR_WIDTH
module tag_array(
	input                             clk,
	input  [`TARRAY_ADDR_WIDTH - 1:0] waddr,
	input  [`TARRAY_ADDR_WIDTH - 1:0] raddr,
	input                             wen,
	input  [`TARRAY_DATA_WIDTH - 1:0] wdata,
	output [`TARRAY_DATA_WIDTH - 1:0] rdata
);

	reg [`TARRAY_DATA_WIDTH-1:0] array[ `TARRAY_SET - 1 : 0];
	
	always @(posedge clk)
	begin
		if(wen)
			array[waddr] <= wdata;
	end 

assign rdata = array[raddr];

endmodule
