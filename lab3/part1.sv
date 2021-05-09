module part1
	(
		input [31:0] X,
		input [31:0] Y,
		output [31:0] result
);
// Design your 32-bit Floating Point unit here.
	logic [47:0] mantissa_product_unrounded;
	logic [24:0] mantissa_product_cropped;
	logic [7:0] exponent_product;
	logic sign_product;

	logic [23:0] x_mantissa;
	logic [23:0] y_mantissa;
	logic [7:0] x_exponent;
	logic [7:0] y_exponent;

	assign x_mantissa = {{1'b1}, X[22:0]};
	assign y_mantissa = {{1'b1}, Y[22:0]};
	assign x_exponent = X[30:23];
	assign y_exponent = Y[30:23];

	assign mantissa_product_unrounded = x_mantissa * y_mantissa;
	assign sign_product = X[31] ^ Y[31];

	always_comb begin
		mantissa_product_cropped = mantissa_product_unrounded[47:23];
		exponent_product = x_exponent + y_exponent + 8'b10000001;
		if(mantissa_product_cropped[24] == 1'b1) begin
			mantissa_product_cropped = mantissa_product_cropped >> 1;
			exponent_product = exponent_product + 1;
		end
	end

	assign result[31] = sign_product;
	assign result[30:23] = exponent_product;
	assign result[22:0] = mantissa_product_cropped[22:0];

endmodule
