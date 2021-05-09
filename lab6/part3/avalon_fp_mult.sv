module avalon_fp_mult
(
	input clk,
	input reset,
	
	// Avalon Slave
	input [2:0] avs_s1_address,
	input avs_s1_read,
	input avs_s1_write,
	input [31:0] avs_s1_writedata,
	output logic [31:0] avs_s1_readdata,
	output logic avs_s1_waitrequest
);
	
	logic nan_sig, ov_sig, un_sig, zero_sig;
	logic [31:0] data_result;
	logic [31:0] data_a, data_b;
	
	avalon_slave avs
	(
		.clk(clk),
		.reset(reset),
		.avs_s1_address(avs_s1_address),
		.avs_s1_read(avs_s1_read),
		.avs_s1_write(avs_s1_write),
		.avs_s1_writedata(avs_s1_writedata),
		.avs_s1_readdata(avs_s1_readdata),
		.avs_s1_waitrequest(avs_s1_waitrequest),

		// FP_MULT
		// STUDENTS TO ADD THESE
		.result(data_result),
	   .flags({ov_sig, un_sig, zero_sig, nan_sig}),
	   .input_one(data_a),
	   .input_two(data_b)
	);

	fp_mult fpm
	(
        .aclr (reset),
		  .clk_en ( 1'b1),
		  .clock ( clk ),
		  .dataa ( data_a ),
		  .datab ( data_b ),
		  .nan ( nan_sig ),
		  .overflow ( ov_sig ),
		  .result ( data_result ),
		  .underflow ( un_sig ),
		  .zero ( zero_sig )
	);

endmodule
