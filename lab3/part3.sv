// This module uses parameterized instantiation. If values are not set in the testbench the default values specified here are used.
// The values for EXP and MAN set here are for a IEEE-754 32-bit Floating point representation.
// TO DO: Edit BITS and BIAS based on the EXP and MAN parameters.

module part3
	#(parameter EXP = 8,			// Number of bits in the Exponent
	  parameter MAN = 23, 			// Number of bits in the Mantissa
	  parameter BITS = EXP + MAN + 1,	// Total number of bits in the floating point number
	  parameter BIAS = 2 ** EXP - 1	// Value of the bias, based on the exponent.
	  )
	(
		input [BITS - 1:0] X,
		input [BITS - 1:0] Y,
		output inf, nan, zero, overflow, underflow,
		output reg[BITS - 1:0] result
);
		// Design your 32-bit Floating Point unit here.
		logic [4:0] special_case;
		logic [EXP:0] potential_exponent;
		logic [EXP:0] potential_bias;
		logic [EXP - 1:0] x_exponent;
    logic [EXP - 1:0] y_exponent;

		assign x_exponent = X[BITS - 2: BITS - EXP - 1];
		assign y_exponent = Y[BITS - 2: BITS - EXP - 1];
		assign potential_exponent = x_exponent + y_exponent;
		assign potential_bias = 2 ** (EXP - 1) - 1;


		logic [2 * MAN + 1:0] mantissa_product_unrounded;
		logic [MAN + 1:0] mantissa_product_cropped;
		logic [EXP - 1:0] exponent_product;
		logic [EXP - 1:0] negative_bias;
		logic sign_product;
		logic [MAN:0] x_mantissa;
		logic [MAN:0] y_mantissa;

		assign x_mantissa = {{1'b1}, X[MAN - 1:0]};
		assign y_mantissa = {{1'b1}, Y[MAN - 1:0]};
		assign mantissa_product_unrounded = x_mantissa * y_mantissa;
		assign sign_product = X[BITS - 1] ^ Y[BITS - 1];
		assign negative_bias = ~potential_bias + 1'b1;

		always_comb begin
				//case 1: check if x or y are zeros
				if(X[BITS - 2:0] == {(BITS - 1){1'b0}} || Y[BITS - 2:0] == {(BITS - 1){1'b0}}) begin
						result = {(BITS){1'b0}};
						special_case = 5'b00100;
				end

				//case 2: check if x or y are nan
				else if((x_exponent == BIAS && X[MAN - 1:0] != {(MAN){1'b0}}) || (y_exponent == BIAS && Y[MAN - 1:0] != {(MAN){1'b0}})) begin
						result = {{1'b0}, {BIAS}, {(MAN){1'b0}}};
						special_case = 5'b01000;
				end

				//case 3: check if x or y are inf
				else if((x_exponent == BIAS && X[MAN - 1:0] == {(MAN){1'b0}}) || (y_exponent == BIAS && Y[MAN - 1:0] == {(MAN){1'b0}})) begin
						result = {{1'b0}, {BIAS}, {(MAN){1'b0}}};
						special_case = 5'b10000;
				end

				//case4: check if output would be underflow
				else if(potential_exponent < potential_bias)begin
						result = {(BITS){1'b0}};
						special_case = 5'b00001;
				end

				//case5: check if output would be overflow
				else if(potential_exponent > (potential_bias + BIAS))begin
						result = {{1'b0}, {BIAS}, {(MAN){1'b0}}};
						special_case = 5'b00010;
				end

				//no specical case
				else begin
						special_case = 5'b00000;

						mantissa_product_cropped = mantissa_product_unrounded[2 * MAN + 1:MAN];
						exponent_product = x_exponent + y_exponent + negative_bias;
						if(mantissa_product_cropped[MAN + 1] == 1'b1) begin
							mantissa_product_cropped = mantissa_product_cropped >> 1;
							exponent_product = exponent_product + 1;
						end

						result = {sign_product, exponent_product, mantissa_product_cropped[MAN - 1:0]};
				end

		end

		assign inf = special_case[4];
		assign nan = special_case[3];
		assign zero = special_case[2];
		assign overflow = special_case[1];
		assign underflow = special_case[0];

endmodule
