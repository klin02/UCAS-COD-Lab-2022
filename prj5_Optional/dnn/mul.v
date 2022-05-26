`timescale 10 ns / 1 ns
module mul(
	input clk,
	input rst,
	input [31:0] a,
	input [31:0] b,
	input run,
	output [63:0] result,
	output done
);
	//assign run=1'b1;
//booth算法 将连续数位1转化为前后位的两个1相减 从而减少相加次数

	reg [2:0] i; //procedure
	reg [64:0] P; 
	reg [31:0] a_tmp;
	reg [31:0] a_rev;// complement code
	reg [5:0] cnt;// count the time of loop
	reg isDone;
	reg inWork;
	
	always @ (posedge clk)
	begin
		if(rst)
		begin
			i<=0;
			P<=0;
			a_tmp <=0;
			a_rev <=0;
			cnt <=0;
			isDone <=0;
			inWork <=0;
		end
		else if(run & ~inWork)//initial work
		begin
			inWork <=1;
			a_tmp <= a;
			a_rev <= ~a +1'b1;
			P <= {32'b0, b, 1'b0 };
			i <= 1;
			cnt <=0;
		end
		else if(run & inWork)
		begin
			case(i)
				//operate
				1:begin 
					if(cnt==32)
					begin
						cnt <=0;
						i <= 3;
					end
					else if( P[1:0] ==2'b00 | P[1:0] == 2'b11)
					begin
						P <= P;
						i <= 2;
					end
					else if( P[1:0] == 2'b01)
					begin
						P <= {P[64:33] + a_tmp , P[32:0]};
						i <= 2;
					end
					else if( P[1:0] == 2'b10)
					begin
						P <= {P[64:33] + a_rev , P[32:0]};
						i <= 2;
					end
				end
				//shift right
				2:begin
					P <= {P[64] ,P[64:1]};
					cnt <= cnt +1'b1;
					i <=1;
				end
				//done flag
				3:begin
					isDone <=1;
					i <= 4;
				end
				//refresh
				4:begin
					isDone <=0;
					inWork <=0;
					i <=0;
				end
				default:
					i <=0;
			endcase
		end
	end
	
	assign done =isDone;
	assign result = {64{done}} & P[64:1];
	
endmodule