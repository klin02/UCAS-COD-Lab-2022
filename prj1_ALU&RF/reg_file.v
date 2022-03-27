`timescale 10 ns / 1 ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5
`define REG_NUM 31
`define TOP 32

module reg_file(
	input                       clk,
	input  [`ADDR_WIDTH - 1:0]  waddr,
	input  [`ADDR_WIDTH - 1:0]  raddr1,
	input  [`ADDR_WIDTH - 1:0]  raddr2,
	input                       wen,
	input  [`DATA_WIDTH - 1:0]  wdata,
	output [`DATA_WIDTH - 1:0]  rdata1,
	output [`DATA_WIDTH - 1:0]  rdata2
);

	// TODO: Please add your logic design here
	
	reg [`DATA_WIDTH -1 : 0] RF [`TOP-1 : 0];

	always @ (posedge clk)
	begin
		if(wen & (|waddr)) begin // |waddr is equal to waddr != 5'b0
			RF[waddr] <= wdata;
		end
	end

	assign rdata1 = {`DATA_WIDTH {|raddr1}} & RF [raddr1];
	assign rdata2 = {`DATA_WIDTH {|raddr2}} & RF [raddr2];
endmodule
