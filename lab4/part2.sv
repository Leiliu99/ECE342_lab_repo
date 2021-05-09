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

module avalon_slave
(
	input clk,
	input reset,

	// Avalon Slave
	input [2:0] avs_s1_address,
	input avs_s1_read,
	input avs_s1_write,
	input [31:0] avs_s1_writedata,
	output logic [31:0] avs_s1_readdata,
	output logic avs_s1_waitrequest,

	// FP_MULT
    // STUDENTS TO ADD THESE
	 input [31:0] result,
	 input [3:0] flags,
	 output logic [31:0] input_one,
	 output logic [31:0] input_two

);

// Mem-mapped regs
// Reg0-32:			A
// Reg1-32:			B
// Reg2-04:			Start/Busy
// Reg3-32:			Result
// Reg4-04:			Status (Flags)
logic [31:0] mul_op_1;
logic [31:0] mul_op_2;
logic [31:0] mul_start;
logic [31:0] mul_result;
logic [31:0] mul_status;

logic[3:0] counter;
logic counter_enable;


enum int unsigned
{
	S_START,
	S_WAIT,
	S_ENDMUL
} state, nextstate;

assign input_one = mul_op_1;
assign input_two = mul_op_2;

// STUDENTS TO ADD THESE
always_ff @ (posedge clk or posedge reset) begin
	if (reset)begin
		state <= S_START;
		//avs_s1_waitrequest <= 1'b0;
	end
	else state <= nextstate;
end


always_ff @ (posedge clk or posedge reset) begin
	if(reset)begin
		counter <= 4'b0;
		mul_op_1 <= 0;
		mul_op_2 <= 0;
		mul_start <= 0;
		mul_result <= 0;
		mul_status <= 0;
	end else begin
		if(counter_enable) counter <= counter + 1;
		
		if(!mul_start[0])begin
			if(avs_s1_read)begin
				case(avs_s1_address)
					3'b000:begin
						avs_s1_readdata <= mul_op_1;
					end
					3'b001:begin
						avs_s1_readdata <= mul_op_2;
					end
					3'b010:begin
						avs_s1_readdata <= mul_start;
					end
					3'b011:begin
						avs_s1_readdata <= mul_result;
					end
					3'b100:begin
						avs_s1_readdata <= mul_status;
					end
					default: avs_s1_readdata <= 0;
				endcase
			end//for if
			
			else if(avs_s1_write)begin
				case(avs_s1_address)
					3'b000:begin
						mul_op_1 <= avs_s1_writedata;
					end
					3'b001:begin
						mul_op_2 <= avs_s1_writedata;
					end
					3'b010:begin
						mul_start <= avs_s1_writedata;
					end
					3'b011:begin
						mul_result <= avs_s1_writedata;
					end
					3'b011:begin
						mul_status <= avs_s1_writedata;
					end
					default: avs_s1_readdata <= 0;
				endcase
			end 
		end
		
		if(state == S_START)begin
			counter <= 4'b0;
		end 
		else if(state == S_ENDMUL)begin
			mul_result <= result;
			mul_status <= {{28'd0},flags};
			mul_start <= 32'b0;
		end
		
		
	end//else
	
end//always ff



always_comb begin
	//nextstate = state;
	case(state)
		S_START: begin
			nextstate = mul_start[0] ? S_WAIT : S_START;
			counter_enable = 1'b0;
			avs_s1_waitrequest = 1'b0;
		end

		S_WAIT: begin
			counter_enable = 1'b1;
			//input_one = mul_op_1;
			//input_two = mul_op_2;
			avs_s1_waitrequest = 1'b1;
			nextstate = counter == 4'b1010 ? S_ENDMUL : S_WAIT;

		end

		S_ENDMUL: begin
			counter_enable = 1'b0;
			avs_s1_waitrequest = 1'b1;
			nextstate = S_START;
		end
	endcase


end


endmodule
