module wallace_mult (
  input [7:0] x,
  input [7:0] y,
  output [15:0] out,
  output [15:0] pp [4]
);

// These signals are created to help you map the wires you need with the diagram provided in the lab document.

	wire [15:0] s_lev01; //the first "save" output of level0's CSA array
	wire [15:0] c_lev01; //the first "carry" output of level0's CSA array
	wire [15:0] s_lev02; //the second "save" output of level0's CSA array
	wire [15:0] c_lev02;
	wire [15:0] s_lev11;
	wire [15:0] c_lev11;
	wire [15:0] s_lev12; //the second "save" output of level1's CSA array
	wire [15:0] c_lev12;
	wire [15:0] s_lev21;
	wire [15:0] c_lev21;
	wire [15:0] s_lev31;
	wire [15:0] c_lev31;

// TODO: complete the hardware design for instantiating the CSA blocks per level.

//STEP1: initialize a input matrix that contains eight operands
	logic [15:0]input_oprands[8];
	genvar row, col;
	generate
	for(row = 0; row < 8; row++)begin
		for(col = 0; col < 16; col++)begin
			if(col < row) assign input_oprands[row][col] = 1'b0;
			else if(col - row < 8)  assign input_oprands[row][col] = x[col-row] & y[row];
			else assign input_oprands[row][col] = 1'b0;
		end
	end
	endgenerate

//STEP2: fill in each layer according to Figure 2
//level 0
	csa csa_level_01
	(
		.op1(input_oprands[0]),
		.op2(input_oprands[1]),
		.op3(input_oprands[2]),
		.S(s_lev01),
		.C(c_lev01)
	);
	
	csa csa_level_02
	(
		.op1(input_oprands[3]),
		.op2(input_oprands[4]),
		.op3(input_oprands[5]),
		.S(s_lev02),
		.C(c_lev02)
	);

//level 1

	csa csa_level_11
	(
		.op1(s_lev01),
		.op2({c_lev01[14:0],{1'b0}}),
		.op3(s_lev02),
		.S(s_lev11),
		.C(c_lev11)
	);
	
	csa csa_level_12
	(
		.op1({c_lev02[14:0], {1'b0}}),
		.op2(input_oprands[6]),
		.op3(input_oprands[7]),
		.S(s_lev12),
		.C(c_lev12)
	);


//level 2, the save and carry output of level 2 will be pp[2] and pp[3]
  
  assign pp[0] = s_lev21;
  assign pp[1] = c_lev21;
  csa csa_level_2
	(
		.op1({c_lev11[14:0], {1'b0}}),
		.op2(s_lev11),
		.op3(s_lev12),
		.S(s_lev21),
		.C(c_lev21)
	);


//level 3, the save and carry output of level 3 will be pp[2] and pp[3]
  
  assign pp[2] = s_lev31;
  assign pp[3] = c_lev31;
  
  csa csa_level_3
	(
		.op1(s_lev21),
		.op2({c_lev21[14:0], {1'b0}}),
		.op3({c_lev12[14:0], {1'b0}}),
		.S(s_lev31),
		.C(c_lev31)
	);

//STEP3: Ripple carry adder to calculate the final output.
	rca rca_level_4
	(
		.op1(s_lev31),
		.op2({c_lev31[14:0], {1'b0}}),
		.cin({1'b0}),
		.sum(out)
	);

endmodule





// These modules are provided for you to use in your designs.
// They also serve as examples of parameterized module instantiation.
module rca #(width=16) (
    input  [width-1:0] op1,
    input  [width-1:0] op2,
    input  cin,
    output [width-1:0] sum,
    output cout
);

wire [width:0] temp;
assign temp[0] = cin;
assign cout = temp[width];

genvar i;
for( i=0; i<width; i=i+1) begin
    full_adder u_full_adder(
        .a      (   op1[i]     ),
        .b      (   op2[i]     ),
        .cin    (   temp[i]    ),
        .cout   (   temp[i+1]  ),
        .s      (   sum[i]     )
    );
end

endmodule


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

module csa #(width=16) (
	input [width-1:0] op1,
	input [width-1:0] op2,
	input [width-1:0] op3,
	output [width-1:0] S,
	output [width-1:0] C
);

genvar i;
generate
	for(i=0; i<width; i++) begin
		full_adder u_full_adder(
			.a      (   op1[i]    ),
			.b      (   op2[i]    ),
			.cin    (   op3[i]    ),
			.cout   (   C[i]	  ),
			.s      (   S[i]      )
		);
	end
endgenerate

endmodule

