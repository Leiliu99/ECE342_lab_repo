module part2
	(
		input [31:0] X,
		input [31:0] Y,
		output inf, nan, zero, overflow, underflow,
		output reg[31:0] result
);

// Design your 32-bit Floating Point unit here
		logic [4:0] special_case;
		logic [8:0] potential_exponent;
    logic [7:0] x_exponent;
    logic [7:0] y_exponent;
		assign x_exponent = X[30:23];
		assign y_exponent = Y[30:23];

		assign potential_exponent = x_exponent + y_exponent;

		logic [47:0] mantissa_product_unrounded;
		logic [24:0] mantissa_product_cropped;
		logic [7:0] exponent_product;
		logic sign_product;
		logic [23:0] x_mantissa;
		logic [23:0] y_mantissa;

		assign x_mantissa = {{1'b1}, X[22:0]};
		assign y_mantissa = {{1'b1}, Y[22:0]};
		assign mantissa_product_unrounded = x_mantissa * y_mantissa;
		assign sign_product = X[31] ^ Y[31];


		always_comb begin
		    //case 1: check if x or y are zeros
				if(X[30:0] == 31'd0 || Y[30:0] == 31'd0) begin
						result = 32'd0;
						special_case = 5'b00100;
				end

				//case 2: check if x or y are nan
				else if((x_exponent == 8'b11111111 && X[22:0] != 23'd0) || (y_exponent == 8'b11111111 && Y[22:0] != 23'd0)) begin
						result = {{1'b0}, {8'b11111111},{23'd0}};
						special_case = 5'b01000;
				end

				//case 3: check if x or y are inf
				else if((x_exponent == 8'b11111111 && X[22:0] == 23'd0) || (y_exponent == 8'b11111111 && Y[22:0] == 23'd0)) begin
						result = {{1'b0}, {8'b11111111},{23'd0}};
						special_case = 5'b10000;
				end

				//case4: check if output would be underflow
				else if(potential_exponent < 9'd127)begin
						result = 32'd0;
						special_case = 5'b00001;
				end

				//case5: check if output would be overflow
				else if(potential_exponent > 9'd382)begin
						result = {{1'b0}, {8'b11111111},{23'd0}};
						special_case = 5'b00010;
				end

				//no specical case
				else begin
						special_case = 5'b00000;

						mantissa_product_cropped = mantissa_product_unrounded[47:23];
						exponent_product = x_exponent + y_exponent + 8'b10000001;
						if(mantissa_product_cropped[24] == 1'b1) begin
							mantissa_product_cropped = mantissa_product_cropped >> 1;
							exponent_product = exponent_product + 1;
						end

						result = {sign_product, exponent_product, mantissa_product_cropped[22:0]};
				end

		end


		assign inf = special_case[4];
		assign nan = special_case[3];
		assign zero = special_case[2];
		assign overflow = special_case[1];
		assign underflow = special_case[0];

endmodule
