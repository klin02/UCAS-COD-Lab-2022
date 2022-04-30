`include "turbo_macro.v"

module predictor(
	input	clk,
	input 	rst,
	
	input 	Branch, //type is branch or jump
	//signal of not taken
	input 	cancel,
	output 	prdt_br
);

	reg [4:0] prdt_cur_state;
	reg [4:0] prdt_next_state;

	always @ (posedge clk) begin
		if(rst) 
			prdt_cur_state <= `RST ; 
		else
			prdt_cur_state <= prdt_next_state ;
	end
	
	//note that state will not change unless Branch
	always @ (*) begin 
		case (prdt_cur_state)
			`RST : begin
				if(rst)
					prdt_next_state = `RST ; 
				else
					prdt_next_state = `S_Taken ; 
			end
			`S_Taken : begin
				if(cancel & Branch) 
					prdt_next_state = `W_Taken ; 
				else
					prdt_next_state = `S_Taken ; 
			end
			`W_Taken : begin
				if(cancel & Branch)
					prdt_next_state = `W_NTaken ; 
				else if (~cancel & Branch)
					prdt_next_state = `S_Taken ;
				else 
					prdt_next_state = `W_Taken ; 
			end
			`W_NTaken : begin
				if(cancel & Branch)
					prdt_next_state = `S_NTaken ; 
				else if (~cancel & Branch)
					prdt_next_state = `W_Taken ; 
				else
					prdt_next_state = `W_NTaken ;
			end
			`S_NTaken : begin
				if(cancel & Branch)
					prdt_next_state = `S_NTaken ;
				else if(~cancel & Branch)
					prdt_next_state = `W_NTaken ; 
				else 
					prdt_next_state = `S_NTaken ;
			end
		endcase
	end
	
	assign	prdt_br = (prdt_cur_state == `S_Taken) | (prdt_cur_state == `W_Taken);

endmodule