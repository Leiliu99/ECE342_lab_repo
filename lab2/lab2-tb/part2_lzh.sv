module wallace_mult_lzh (
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

  logic [7:0][15:0] input_layer;
  genvar i, j;
  generate
    for(i = 0; i < 8; i ++) begin
      for(j = 0; j < 16; j ++) begin
        if(j < i) assign input_layer[i][j] = 1'b0;
        else if(j < i + 8) assign input_layer[i][j] = x[j - i] & y[i];
        //else if(j < i + 8) assign input_layer[i][j] = 1'b1;
        else assign input_layer[i][j] = 1'b0;
      end
    end
  endgenerate

  //level 0
  csa_lzh lev01(
    .op1(input_layer[0]),
    .op2(input_layer[1]),
    .op3(input_layer[2]),
    .S(s_lev01),
    .C(c_lev01)
  );

  csa_lzh lev02(
    .op1(input_layer[3]),
    .op2(input_layer[4]),
    .op3(input_layer[5]),
    .S(s_lev02),
    .C(c_lev02)
  );

  //level 1
  csa_lzh lev11(
    .op1(s_lev01),
    .op2({c_lev01[14:0], {1'b0}}),
    .op3(s_lev02),
    .S(s_lev11),
    .C(c_lev11)
  );

  csa_lzh lev12(
    .op1({c_lev02[14:0], {1'b0}}),
    .op2(input_layer[6]),
    .op3(input_layer[7]),
    .S(s_lev12),
    .C(c_lev12)
  );

  //level 2, the save and carry output of level 2 will be pp[2] and pp[3]
    
  assign pp[0] = s_lev21;
  assign pp[1] = c_lev21;
  csa_lzh lev21(
    .op1(s_lev11),
    .op2(s_lev12),
    .op3({c_lev11[14:0], {1'b0}}),
    .S(s_lev21),
    .C(c_lev21)
  );

  //level 3, the save and carry output of level 3 will be pp[2] and pp[3]
    
  assign pp[2] = s_lev31;
  assign pp[3] = c_lev31;
  csa_lzh lev31(
    .op1(s_lev21),
    .op2({c_lev21[14:0], {1'b0}}),
    .op3({c_lev12[14:0], {1'b0}}),
    .S(s_lev31),
    .C(c_lev31)
  );

  // Ripple carry adder to calculate the final output.

  logic [15:0] sum_result;
  // carry_lookahead_adder #(.N(16)) ins_cla(
  //   .x(s_lev31),
  //   .y({c_lev31[14:0], {1'b0}}),
  //   .c_in({1'b0}),
  //   .s(sum_result)
  // );
  rca_lzh  #(.width(16)) ins_rca(
    .op1(s_lev31),
    .op2({c_lev31[14:0], {1'b0}}),
    .cin({1'b0}),
    .sum(sum_result)
  );
  assign out = sum_result;

endmodule

// These modules are provided for you to use in your designs.
// They also serve as examples of parameterized module instantiation.
module rca_lzh #(width=16) (
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
      full_adder_lzh u_full_adder(
          .a      (   op1[i]     ),
          .b      (   op2[i]     ),
          .cin    (   temp[i]    ),
          .cout   (   temp[i+1]  ),
          .s      (   sum[i]     )
      );
  end

endmodule


module full_adder_lzh(
    input a,
    input b,
    input cin,
    output cout,
    output s
);

  assign s = a ^ b ^ cin;
  assign cout = a & b | (cin & (a ^ b));

endmodule

module csa_lzh #(width=16) (
	input [width-1:0] op1,
	input [width-1:0] op2,
	input [width-1:0] op3,
	output [width-1:0] S,
	output [width-1:0] C
);

  genvar i;
  generate
  	for(i=0; i<width; i++) begin
  		full_adder_lzh u_full_adder(
  			.a      (   op1[i]    ),
  			.b      (   op2[i]    ),
  			.cin    (   op3[i]    ),
  			.cout   (   C[i]	  ),
  			.s      (   S[i]      )
  		);
  	end
  endgenerate

endmodule
