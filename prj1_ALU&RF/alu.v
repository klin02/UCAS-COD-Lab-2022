`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

//ALUop
`define AND  3'b000
`define OR   3'b001
`define ADD  3'b010
`define SUB  3'b110
`define SLT  3'b111
//new ALUop
`define XOR  3'b100
`define NOR  3'b101
`define SLTU 3'b011

module alu(
    input [`DATA_WIDTH - 1:0]  A,
    input [`DATA_WIDTH - 1:0]  B,
    input [              2:0]  ALUop,
    output                     Overflow,
    output                     CarryOut,
    output                     Zero,
    output [`DATA_WIDTH - 1:0] Result
);
	
//wire virables for direct results
	wire [`DATA_WIDTH -1 :0] R_AND;
	wire [`DATA_WIDTH -1 :0] R_OR;
	wire [`DATA_WIDTH -1 :0] R_ADD;
	wire [`DATA_WIDTH -1 :0] R_SUB;
	wire [`DATA_WIDTH -1 :0] R_SLT;
//new wire virables for direct results
 	wire [`DATA_WIDTH -1 :0] R_XOR;
	wire [`DATA_WIDTH -1 :0] R_NOR;
	wire [`DATA_WIDTH -1 :0] R_SLTU;

//wire virable for carryout
	wire flag;
	wire Inverse;

//wire virable for overflow
	wire Compare;
	
//add 1 more bit for completement form
	wire [`DATA_WIDTH :0]	A_tmp;
	wire [`DATA_WIDTH :0]	B_tmp;
	
//result of OP AND / OR / XOR / NOR
	assign R_AND = A & B;
	assign R_OR  = A | B;
	assign R_XOR = A ^ B;
	assign R_NOR = ~R_OR;
	
//completement of A / B    :completement of completement code is equal to itself
	assign Inverse = ALUop[1] & (ALUop[2] | ALUop[0]); //sub or slt or sltu
	assign A_tmp = {1'b0,A};
	assign B_tmp = {1'b0, Inverse ? ~B : B }; //note that the inverse result exclude the highest bit
	
//result of OP ADD / SUB / SLT
	assign {flag , R_ADD} = A_tmp + B_tmp + Inverse;
	assign R_SUB = R_ADD;
	
//CarryOut  -> unsigned num
//ADD: carryout is the same as actual
//SUB: when A=B the result is 0x1 0000 0000
	//when A < B, Completement of B < completement of A, so the highest bit of A-B is 0, however borrow actually happen, vice versa;
	//so carryout = ~flag

//Overflow  -> signed num
//overflow cases:
    // If A > 0 and B > 0 but A + B < 0
    // If A < 0 and B < 0 but A + B >= 0
    // If A > 0 and B < 0 but A - B < 0
    // If A < 0 and B > 0 but A - B >= 0
//Overflow only consider ADD or SUB, but we want to use result for slt, so set compare
	assign CarryOut = (ALUop == `ADD)? flag : ~flag;
	assign Compare = (~A[`DATA_WIDTH-1] & ~B[`DATA_WIDTH-1] &  R_ADD[`DATA_WIDTH-1] & ALUop== `ADD)			|
			  ( A[`DATA_WIDTH-1] & B[`DATA_WIDTH-1]  & ~R_ADD[`DATA_WIDTH-1] & ALUop== `ADD)			|
			  (~A[`DATA_WIDTH-1] & B[`DATA_WIDTH-1]  &  R_SUB[`DATA_WIDTH-1] & (ALUop==`SLT|ALUop== `SUB))	|
			  ( A[`DATA_WIDTH-1] & ~B[`DATA_WIDTH-1] & ~R_SUB[`DATA_WIDTH-1] & (ALUop==`SLT|ALUop== `SUB));
	assign Overflow = (ALUop == `ADD | ALUop==`SUB ) & Compare;

						
//result of OP SLT /SLTU
//Unsigned num: consider SUB, then A < B means carryout happen, we get carryout=~flag above
//Sigend num:   compare means the result is the inverse of expected, so we use xor to get result 

	assign R_SLT[0] = Compare ^ R_SUB[`DATA_WIDTH-1];
	assign R_SLT[`DATA_WIDTH-1 :1] = 31'b0;
	assign R_SLTU[`DATA_WIDTH-1 : 0] = {31'b0, ~flag};
	
//final result and zero check
	assign Result = ({32{ALUop == `AND}} & R_AND ) |
                    	({32{ALUop == `OR }} & R_OR  ) |
                    	({32{ALUop == `ADD}} & R_ADD ) |
                    	({32{ALUop == `SUB}} & R_SUB ) |
                    	({32{ALUop == `SLT}} & R_SLT ) |
			({32{ALUop == `XOR}} & R_XOR ) |
			({32{ALUop == `NOR}} & R_NOR ) |
			({32{ALUop == `SLTU}} & R_SLTU ) ;

    	assign Zero = Result == 32'b0;


endmodule

