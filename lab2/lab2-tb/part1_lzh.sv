/*******************************************************/
/********************Multiplier module********************/
/*****************************************************/
// add additional modules as needs, such as full adder, and others

// multiplier module
module mult_lzh
(
	input [7:0] x,
	input [7:0] y,
	output [15:0] out,   // Result of the multiplication
	output [15:0] pp [9] // for automarker to check partial products of a multiplication 
);
	// Declare a 9-high, 16-deep array of signals holding sums of the partial products.
	// They represent the _input_ partial sums for that row, coming from above.
	// The input for the "ninth row" is actually the final multiplier output.
	// The first row is tied to 0.
	assign pp[0] = '0;
	
	// Make another array to hold the carry signals
	logic [16:0] cin[9];
	assign cin[0] = '0;
	
	// Cin signals for the final (fast adder) row
	logic [8:0] cin_final;
	assign cin_final[0] = '0;
	assign cin_final[8:1] = cin[8][15:8];

	
	// TODO: complete the following digital logic design of a carry save multiplier (unsigned)
	// Note: generate_hw tutorial can help you describe duplicated modules efficiently
	
	// Note: partial product of each row is the result coming out from a full adder at the end of that row
	
	// Note: a "Fast adder" operates on columns 8 through 15 of final row.

	logic [7:0][15:0] input_layer;
  	genvar i, j;
  	generate
    	for(i = 0; i < 8; i ++) begin
      		for(j = 0; j < 16; j ++) begin
        		if(j < i) assign input_layer[i][j] = 1'b0;
        		else if(j < i + 8) assign input_layer[i][j] = x[j - i] & y[i];
        		else assign input_layer[i][j] = 1'b0;
      		end
    	end

    	for(i = 1; i < 9; i ++) begin
    		for(j = 0; j < 16; j ++) begin
    			if(j < i - 1) begin 
    				assign pp[i][j] = pp[i-1][j];
    			end else if(j >= i + 7) begin
    				assign pp[i][j] = 0;
    			end 
    		end
    	end

    	for(i = 0; i < 8; i ++) begin
    		carry_row_lzh #(.N(8)) ins_cr(
    			.A_in(pp[i][i+7:i]),
    			.B_in(input_layer[i][i+7:i]),
    			.carry_in(cin[i][i+7:i]),
    			.S_out(pp[i+1][i+7:i]),
    			.carry_out(cin[i+1][i+8:i+1])
    		);
    	end
  	endgenerate
	
	logic cout_final;
	carry_lookahead_adder_lzh #(.N(8)) ins_cla(
		.x(pp[8][15:8]),
		.y(cin_final[8:1]),
		.c_in({1'b0}),
		.s(out[15:8]),
		.c_out(cout_final)
	);

	assign out[7:0] = pp[8][7:0];
		  
endmodule

// The following code is provided for you to use in your design

module full_adder_lzh
(
	input a,
	input b,
	input c_in,
	output s,
	output c_out
);
	assign s = c_in ^ a ^ b;
	assign c_out = (a & b) | (a & c_in) | (b & c_in);
endmodule

module carry_lookahead_adder_lzh #
(
	parameter N = 8
)
(
	input [N-1:0] x,
	input [N-1:0] y,
	input c_in,
	output [N-1:0] s,
	output c_out
);
	logic [N-1:0] g;
	logic [N-1:0] p;
	logic [N:0] c;
	assign c[0] = c_in;
	assign c_out = c[N];
	assign g = x & y;
	assign p = x | y;
	genvar i;
	generate
		for(i = 0; i < N; i ++) begin
			assign c[i + 1] = g[i] | (p[i] & c[i]);
			assign s[i] = x[i] ^ y[i] ^ c[i];
		end
	endgenerate
endmodule

module carry_row_lzh #
(
	parameter N = 16
)
(
	input [N-1:0] A_in,
	input [N-1:0] B_in,
	input [N-1:0] carry_in,

	output [N-1:0] S_out,
	output [N-1:0] carry_out
);
	// Generate multiplier cells
	genvar i;
	generate
		for(i = 0; i < N; i ++) begin : multiplier_cells_generator
			full_adder_lzh ins_fa(
				.a(A_in[i]),
				.b(B_in[i]),
				.c_in(carry_in[i]),
				.s(S_out[i]),
				.c_out(carry_out[i])
			);
		end
	endgenerate
endmodule
