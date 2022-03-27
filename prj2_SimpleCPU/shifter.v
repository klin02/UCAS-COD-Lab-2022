`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

//Shiftop
`define	SLL  2'b00
`define	SRL  2'b10 //set the word right logical
`define	SRA  2'b11 //set the word right arithematic

module shifter (
	input  [`DATA_WIDTH - 1:0] A,
	input  [              4:0] B,
	input  [              1:0] Shiftop,
	output [`DATA_WIDTH - 1:0] Result
);
	// TODO: Please add your logic code here

//wire virables for direct results
	//SRA: care space left by SRA,so do not use simple shift
	wire [`DATA_WIDTH - 1:0] R_SLL;
	wire [`DATA_WIDTH - 1:0] R_SRL;
	wire [`DATA_WIDTH - 1:0] R_SRA;
	wire [63:0] SRA_tmp; 

	assign R_SLL = A << B[4:0];
	assign R_SRL = A >> B[4:0];
	assign SRA_tmp = {{32{A[31]}},A} >> B[4:0];
	assign R_SRA = SRA_tmp[31:0];

	assign Result = ({32{Shiftop == `SLL}} & R_SLL) |
         	        ({32{Shiftop == `SRL}} & R_SRL) |
               	({32{Shiftop == `SRA}} & R_SRA) ;
endmodule
