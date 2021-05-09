/*******************************************************/
/********************Multiplier module********************/
/*****************************************************/
// add additional modules as needs, such as full adder, and others

// multiplier module

module mult
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
	logic [16:8] cin_final;
	assign cin_final[8] = '0;
	
	// TODO: complete the following digital logic design of a carry save multiplier (unsigned)
	// Note: generate_hw tutorial can help you describe duplicated modules efficiently
	
	// Note: partial product of each row is the result coming out from a full adder at the end of that row
	
	// Note: a "Fast adder" operates on columns 8 through 15 of final row.
	
	
	/************* modefied begins ***************/
	
	//an array to save all the input multiplicands needed
	logic [15:0] input_multiplicands[8];
	
	//STEP1: Initialize the input_multiplicands by using generate and for loop
	//one row 8 valid elements, others will be assigned to zero
	//E.g. row[0]: m7q0 ~ m0q0
	genvar row, col;
	generate
		for(row = 0; row < 8; row++)begin
			for(col = 0; col < 16; col++)begin
				if(col < row) assign input_multiplicands[row][col] = 1'b0;
				else if (col - row < 8) assign input_multiplicands[row][col] = x[col - row] & y[row];
				else assign input_multiplicands[row][col] = 1'b0;
			end
		end
	endgenerate
	
	
	
	//STEP2: Initialze the pp array, let the pp array starting  all equals to the values on the rows above if there is no FA above
	//assign the pp be zero if there is no FA after
	genvar i, j;
	generate
		for(i = 1; i < 9; i++)begin
			for(j = 0; j < 16; j++)begin
//				if(j < i - 1) assign pp[i][j] = pp[i-1][j];
//				else assign pp[i][j] = 1'b0;
				if(j < i - 1) assign pp[i][j] = pp[i-1][j];
    			else if(j >= i + 7) assign pp[i][j] = 0; 
			end
		end
	endgenerate
	
	
	
	//STEP3: calculate all the partial multiplicands except for the last row
	genvar m, n;
	generate
		for(m = 0; m < 8; m++)begin
			for(n = m; n < m + 8; n++)begin
				full_adder fa_inst_row
				(
					.a(pp[m][n]),
					.b(input_multiplicands[m][n]),
					.cin(cin[m][n]),
					.cout(cin[m+1][n+1]),
					.s(pp[m+1][n])
				);
			end
		end
	endgenerate
	
	//STEP4: a fast adder at the last row
	genvar p;
	generate
		for(p = 8; p < 16; p++)begin
			full_adder fa_inst_fast
				(
					.a(pp[8][p]),
					.b(cin[8][p]),
					.cin(cin_final[p]),
					.cout(cin_final[p+1]),
					.s(out[p])
				);
		end
	endgenerate
	
	
	//STEP5: assign the lower bits of pp as the lower bits of the final output
	assign out[7:0] = pp[8][7:0];
		  
endmodule


// The following code is provided for you to use in your design
module full_adder(
    input a,
    input b,
    input cin,
    output cout,
    output s
);

assign s = a ^ b ^ cin;
assign cout = a & b | (cin & (a ^ b));
endmodule

