module cpu # (
	parameter IW = 32, // instr width
	parameter REGS = 32 // number of registers
)(
	input clk,
	input reset,

	// read only port
	output logic [IW-1:0] o_pc_addr,
	output logic o_pc_rd,
	input [IW-1:0] i_pc_rddata,
	output [3:0] o_pc_byte_en,

	// read/write port
	output logic [IW-1:0] o_ldst_addr,
	output logic o_ldst_rd,
	output logic o_ldst_wr,
	input [IW-1:0] i_ldst_rddata,
	output logic [IW-1:0] o_ldst_wrdata,
	output logic [3:0] o_ldst_byte_en,

	output logic [IW-1:0] o_tb_regs [0:REGS-1]
);

	//-------------Fetch Instruction Here: Module 1---------
	logic need_branch;
	logic flush_one;
	logic[IW-1:0] branch_addr;

	assign flush_one = need_branch;
	// logic[31:0] branch_addr;

	// assign need_branch = 1'b0; //*********FOR NOW*******
	instruction_fetch IF(.clk(clk), .reset(reset), .need_branch(need_branch), .branch_addr(branch_addr),
		.o_pc_rd(o_pc_rd), .o_pc_addr(o_pc_addr), .o_pc_byte_en(o_pc_byte_en));

	//-------------Instruction Decode Here: Module 2---------
	logic[IW-1:0] instruction_data; //instruction data read from pc
	logic[4:0] rd_id;
	logic start_ALU_id;
	logic[4:0]ALU_operation_code_id;
	logic[IW-1:0] register_a_id, register_b_id;
	logic[IW-1:0] ldst_addr_id, st_data_id;
	logic[4:0] rs_one_id, rs_two_id;
	logic[IW-1:0] temp_b_pc_id;

	logic [IW-1:0] pc_temp_addr_one;
	logic flush_two;

	logic[6:0] op_code_id, op_code_ex;

	assign instruction_data = i_pc_rddata;
	instruction_decode ID(.instruction_data(instruction_data), .o_tb_regs(o_tb_regs), .curr_pc_addr(pc_temp_addr_one),
		.register_a(register_a_id), .register_b(register_b_id), .rd(rd_id),
		.ALU_op(ALU_operation_code_id), .ALU_start(start_ALU_id), .o_ld_st_addr(ldst_addr_id), .o_st_data(st_data_id),
		.rs_one(rs_one_id), .rs_two(rs_two_id), .o_temp_b_pc(temp_b_pc_id), .op_code(op_code_id));

	//-------------Execute Here: Module 3---------
	logic [31:0] st_data_wb;
	logic[4:0] rd_ex, rd_wb;
	logic start_ALU_ex;
	logic[4:0]ALU_operation_code_ex;
	logic[IW-1:0] register_a_ex, register_b_ex, register_z_ex;
	logic[IW-1:0] ldst_addr_ex, st_data_ex;
	logic needwb_ex;
	// logic [IW-1:0] pc_temp_addr_two;
	logic[4:0] rs_one_ex, rs_two_ex;

	logic[IW-1:0] temp_b_pc_ex;
	logic[1:0] need_forward;
	logic[31:0] result_wb;


	execute_instruction EX(.i_op_code(op_code_ex), .Ra(register_a_ex), .Rb(register_b_ex), .i_temp_b_pc(temp_b_pc_ex),
		.i_curr_rsone(rs_one_ex), .i_curr_rtwo(rs_two_ex), .i_ldst_addr(ldst_addr_ex), .i_st_data(st_data_ex), .ALU_op(ALU_operation_code_ex), .start_ALU(start_ALU_ex),
		.Rz(register_z_ex), .need_write_back(needwb_ex), .i_forward_result(result_wb), .o_tb_regs(o_tb_regs), .forward(need_forward),
		.o_ldst_addr(o_ldst_addr), .o_ldst_rd(o_ldst_rd), .o_ldst_wr(o_ldst_wr), .o_st_data(o_ldst_wrdata),
		.o_ldst_byte_en(o_ldst_byte_en), .o_need_b(need_branch), .o_actual_b_pc(branch_addr));

	//-------------Write Back Here: Module 4---------
	logic needwb_wb;
	logic[IW-1:0] register_z_wb;
	logic[4:0]ALU_operation_code_wb;


	// module write_back(input logic[4:0] i_rd,
	// 	input logic needwb, input logic[31:0] i_rz, input logic[4:0]ALU_op,
	// 	input logic[31:0] i_ld_data, output logic[31:0] o_result_wb);

	write_back WB(.i_rd(rd_wb),
		.needwb(needwb_wb), .i_rz(register_z_wb), .ALU_op(ALU_operation_code_wb),
		.i_ld_data(i_ldst_rddata), .o_result_wb(result_wb), .st_data_wb(st_data_wb));

	//--------------pipelined registers-----------
	integer i;
	always @ (posedge reset or negedge clk) begin
		if (reset) begin
			for (i = 0; i < 32; i = i+1) begin
				o_tb_regs[i] <= 32'b0;
			end
		end else begin
			i = 0;
			//write back to the register
			if (needwb_wb) begin
				if (rd_wb != 0) begin
					o_tb_regs[rd_wb] <= result_wb;
				end
			end
		end
	end

	always_ff @ (posedge clk or posedge reset) begin
		if (reset) begin
			//---------pipeline 1----------
			pc_temp_addr_one <= 0;
			//---------pipeline 2--------
			flush_two  <= 0;
			op_code_ex <= 0;
			//if need flush caused by branching
			start_ALU_ex <= 0;
			ALU_operation_code_ex <= 0;//flush
			rd_ex <= 0;
			ldst_addr_ex <= 0;
			st_data_ex <= 0;
			rs_one_ex <= 0;
			rs_two_ex <= 0;
			register_b_ex <= 0;
			register_a_ex <= 0;
			temp_b_pc_ex <= 0;
			st_data_wb <= 0;
			//------------pipeline 3----------
			rd_wb <= 0;
			needwb_wb <= 0;
			register_z_wb <= 0;
			ALU_operation_code_wb <= 0;
		end

		else begin

			//---------pipeline 1----------
			// if(flush_one == 1'b1) pc_temp_addr_one <= 32'd0;
			// else
			pc_temp_addr_one <= o_pc_addr;
	
			//---------pipeline 2--------
			flush_two <= flush_one;
			op_code_ex <= op_code_id;
			//if need flush caused by branching
			if(need_branch == 1'b1 || flush_two)begin//first or second flush
				start_ALU_ex <= 1'd0;
				ALU_operation_code_ex <= 5'b11100;//flush
			end
			else begin
				start_ALU_ex <= start_ALU_id;
				ALU_operation_code_ex <= ALU_operation_code_id;
			end
			//pc_temp_addr_two <= pc_temp_addr_one;
			rd_ex <= rd_id;
			ldst_addr_ex <= ldst_addr_id;
			st_data_ex <= st_data_id;
			rs_one_ex <= rs_one_id;
			rs_two_ex <= rs_two_id;
			register_b_ex <= register_b_id;
			register_a_ex <= register_a_id;
			temp_b_pc_ex <= temp_b_pc_id;
			st_data_wb <= o_ldst_wrdata;
	
			//------------pipeline 3----------
			rd_wb <= rd_ex;
			needwb_wb <= needwb_ex;
			register_z_wb <= register_z_ex;
			ALU_operation_code_wb <= ALU_operation_code_ex;
		end
  end

	localparam  FLUSH = 5'b11100;
	always_comb begin
		//forward prevent data race
		if(rd_wb != 0 && rd_wb == rs_one_ex && rs_two_ex == rs_one_ex && ALU_operation_code_wb != FLUSH) begin
			need_forward = 2'b11;
		end
		else if(rd_wb != 0 && rd_wb == rs_one_ex && ALU_operation_code_wb != FLUSH)begin
			need_forward = 2'b01;
		end
		else if(rd_wb != 0 && rd_wb == rs_two_ex && ALU_operation_code_wb != FLUSH)begin
			need_forward = 2'b10;
		end
		else begin
			need_forward = 2'b00;
		end
	end

endmodule

module instruction_fetch(input clk, input reset, input logic need_branch, input logic[31:0] branch_addr,
	output logic o_pc_rd, output logic[31:0]o_pc_addr, output[3:0] o_pc_byte_en);
	//-----------increment pc, get pc address----------------
	logic [31: 0] increment;
	assign increment = 32'd4;
	// always_ff @(posedge clk) begin
	always_ff@(posedge clk) begin
		if (reset) o_pc_addr <= '0;
		else if(need_branch) o_pc_addr <= branch_addr;
		else o_pc_addr <= o_pc_addr + increment;
	end

	assign o_pc_rd = 1'b1;
	assign o_pc_byte_en = 4'b1111;

endmodule

module instruction_decode(input logic[31:0] instruction_data, input logic [31:0] o_tb_regs [0:31], input logic [31:0] curr_pc_addr,
	output logic[31:0] register_a, output logic[31:0] register_b, output logic[4:0] rd,
	output logic[4:0] ALU_op, output logic ALU_start, output logic[31:0] o_ld_st_addr, output logic[31:0] o_st_data,
	output logic[4:0] rs_one, output logic[4:0] rs_two,
	output logic[31:0] o_temp_b_pc, output logic[6:0] op_code);
	//-------------------instruction format------------------------
	// logic[6:0]op_code;
	//read instruction from
	assign op_code = instruction_data[6:0];
	//parameterized the option code
	localparam  format_R   = 7'b0110011,
              format_I  = 7'b0010011,
              format_I_ld = 7'b0000011,
              format_S = 7'b0100011,
							format_U_lui = 7'b0110111,
							format_U_auipc = 7'b0010111,
							format_B = 7'b1100011,
							format_J_jal = 7'b1101111,
							format_J_jalr = 7'b1100111;

	logic[6:0] func_seven;
	logic[2:0] func_three;
	logic[11:0] imm_I, imm_S;
	logic[19:0] imm_U;
	logic[12:0] imm_B;
	logic[20:0] imm_J;

	logic[31:0] temp_ld_address, temp_st_address;
	assign func_seven = instruction_data[31:25];
	assign rs_two = instruction_data[24:20];
	assign rs_one = instruction_data[19:15];
	assign rd = instruction_data[11:7];
	assign func_three = instruction_data[14:12];
	assign imm_I = instruction_data[31:20];
	assign imm_U = instruction_data[31:12];
	assign imm_B = {instruction_data[31], instruction_data[7], instruction_data[30:25], instruction_data[11:8], 1'd0};
	assign imm_J = {instruction_data[31], instruction_data[19:12], instruction_data[20], instruction_data[30:21], 1'd0};
	assign imm_S = {instruction_data[31:25], instruction_data[11:7]};
	// assign temp_ld_address = o_tb_regs[rs_one] + {{20{imm_I[11]}}, imm_I[11:0]};
  // assign temp_st_address = o_tb_regs[rs_one] + {{20{imm_S[11]}}, imm_S[11:0]};
	assign temp_ld_address =  {{20{imm_I[11]}}, imm_I[11:0]};
  assign temp_st_address =  {{20{imm_S[11]}}, imm_S[11:0]};


	localparam func_seven_zero = 7'b0000000,
						func_seven_one = 7'b0100000;
	//parameterized the ALU op code
  localparam ADD = 5'b00000,
	           SUB = 5'b00001,
						 XOR = 5'b00010,
						 OR = 5'b00011,
	           AND = 5'b00100,
	           SLL = 5'b00101,
	           SRL = 5'b00110,
	           SRA = 5'b00111,
						 SLT = 5'b01000,
						 SLTU = 5'b01001,
						 LUI = 5'b01010,
						 AUIPC = 5'b01011,
						 BEQ = 5'b01100,
						 BNE = 5'b01101,
						 BLT = 5'b01110,
						 BGE = 5'b01111,
						 BLTU = 5'b10000,
						 BGEU = 5'b10001,
						 JAL = 5'b10010,
						 JALR = 5'b10011,
						 LB = 5'b10100,
						 LH  = 5'b10101,
						 LW = 5'b10110,
						 LBU = 5'b10111,
						 LHU = 5'b11000,
						 SB = 5'b11001,
						 SH = 5'b11010,
						 SW = 5'b11011,
						 FLUSH = 5'b11100;

	 always_comb begin
 		case(op_code)
 			format_R:begin
				o_st_data = 0;
				o_temp_b_pc = 32'd0;
				ALU_start = 1'b1;
				register_a = o_tb_regs[rs_one];
				register_b = o_tb_regs[rs_two];
				o_ld_st_addr = 0;
 				case(func_three)
 					3'b000:begin
 						case(func_seven)
 							func_seven_zero: ALU_op = ADD;
 							func_seven_one: ALU_op = SUB;
							default: ALU_op = 0;
 						endcase
 					end
 					3'b001:begin
 						ALU_op = SLL;
 					end
 					3'b010:begin
 						ALU_op = SLT;
 					end
 					3'b011:begin
 						ALU_op = SLTU;
 					end
 					3'b100:begin
 						ALU_op = XOR;
 					end
 					3'b101:begin
 						case(func_seven)
 							func_seven_zero: ALU_op = SRL;
 							func_seven_one: ALU_op = SRA;
							default: ALU_op = 0;
 						endcase
 					end
 					3'b110:begin
 						ALU_op = OR;
 					end
 					3'b111:begin
 						ALU_op = AND;
 					end
					default: ALU_op = 0;
 				endcase
 			end

 			format_I:begin
				o_st_data = 0;
				o_temp_b_pc = 32'd0;
				ALU_start = 1'b1;
				register_a = o_tb_regs[rs_one];
				register_b = {{20{imm_I[11]}}, imm_I[11:0]};
				o_ld_st_addr = 0;
 				case(func_three)
 					3'b000:begin
 						ALU_op = ADD;
 					end
 					3'b001:begin
 						ALU_op = SLL;
 					end
 					3'b010:begin
 						ALU_op = SLT;
 					end
 					3'b011:begin
 						ALU_op = SLTU;
 					end
 					3'b100:begin
 						ALU_op = XOR;
 					end
 					3'b101:begin
 						case(func_seven)
 							func_seven_zero: ALU_op = SRL;
 							func_seven_one: ALU_op = SRA;
							default: ALU_op = 0;
 						endcase
 					end
 					3'b110:begin
 						ALU_op = OR;
 					end
 					3'b111:begin
 						ALU_op = AND;
 					end
					default: ALU_op = 0;
 				endcase
 			end

 			format_U_lui:begin
				o_st_data = 0;
				o_temp_b_pc = 32'd0;
				ALU_start = 1'b1;
				register_a = {{12{imm_U[19]}}, imm_U[19:0]};
				register_b = curr_pc_addr;
				ALU_op = LUI;
				o_ld_st_addr = 0;
			end
 			format_U_auipc:begin
				o_st_data = 0;
				o_temp_b_pc = 32'd0;
				ALU_start = 1'b1;
				register_a = {{12{imm_U[19]}}, imm_U[19:0]};
				register_b = curr_pc_addr;
				ALU_op = AUIPC;
				o_ld_st_addr = 0;
			end
 			format_B:begin
				o_st_data = 0;
				o_temp_b_pc = curr_pc_addr + {{19{imm_B[12]}},imm_B[12:0]};
				ALU_start = 1'b1;
				register_a = o_tb_regs[rs_one];
				register_b = o_tb_regs[rs_two];
				o_ld_st_addr = 0;
 				case(func_three)
 					3'b000: ALU_op = BEQ;
 					3'b001: ALU_op = BNE;
 					3'b100: ALU_op = BLT;
 					3'b101: ALU_op = BGE;
 					3'b110: ALU_op = BLTU;
 					3'b111: ALU_op = BGEU;
					default: ALU_op = 0;
 				endcase
 			end
 			format_J_jal:begin
				o_st_data = 0;
				o_temp_b_pc = curr_pc_addr + {{11{imm_J[20]}}, imm_J[20:0]};
				ALU_start = 1'b1;
				register_a = curr_pc_addr;
				register_b = 32'd4;
				ALU_op = JAL;
				o_ld_st_addr = 0;
			end
			format_J_jalr:begin
				o_st_data = 0;
				o_temp_b_pc = {{20{imm_I[11]}}, imm_I[11:0]};
				ALU_start = 1'b1;
				register_a = curr_pc_addr;
				register_b = 32'd4;
				ALU_op = JALR;
				o_ld_st_addr = 0;
			end
			format_I_ld:begin
				o_st_data = 0;
				o_temp_b_pc = 32'd0;
				ALU_start = 1'b0;
				register_a = o_tb_regs[rs_one];
				register_b = o_tb_regs[rs_two];
				//actually not used for these two above
				o_ld_st_addr = temp_ld_address;
				case(func_three)
					3'b000: ALU_op = LB;
					3'b001: ALU_op = LH;
					3'b010: ALU_op = LW;
					3'b100: ALU_op = LBU;
					3'b101: ALU_op = LHU;
					default: ALU_op = 0;
				endcase
			end
			format_S:begin
				o_temp_b_pc = 32'd0;
				ALU_start = 1'b0;
				register_a = o_tb_regs[rs_one];
				register_b = o_tb_regs[rs_two];
				//actually not used for these two above
				o_ld_st_addr = temp_st_address;
				case(func_three)
					3'b000:begin
						o_st_data = {{24{1'd0}},o_tb_regs[rs_two][7:0]};
						ALU_op = SB;
					end
					3'b001:begin
						o_st_data = {{16{1'd0}},o_tb_regs[rs_two][15:0]};
						ALU_op = SH;
					end
					3'b010:begin
						o_st_data = o_tb_regs[rs_two];
						ALU_op = SW;
					end
					default: begin
						o_st_data = 32'hdeadbeef;
						ALU_op = 0;
					end
				endcase
			end
			
			default: begin
				o_st_data = 32'hdeadbeff;
				ALU_op = 0;
				o_temp_b_pc = 32'd0;
				ALU_start = 1'b0;
				register_a = 0;
				register_b = 0;
				o_ld_st_addr = 0;
			end
 		endcase
 	end

endmodule


module execute_instruction(input logic[6:0] i_op_code, input logic[31:0] Ra, input logic[31:0] Rb, input logic[31:0] i_temp_b_pc,
	input logic[4:0] i_curr_rsone, input logic[4:0] i_curr_rtwo, input logic[31:0] i_ldst_addr, input logic[31:0] i_st_data, input logic[4:0]ALU_op, input logic start_ALU,
	output logic[31:0] Rz, output logic need_write_back, input logic[31:0] i_forward_result, input logic [31:0] o_tb_regs [0:31], input logic[1:0] forward,
	output logic[31:0] o_ldst_addr, output logic o_ldst_rd, output logic o_ldst_wr, output logic[31:0] o_st_data,
	output logic[3:0] o_ldst_byte_en, output logic o_need_b, output logic[31:0] o_actual_b_pc);

	localparam LUI = 5'b01010,
						 AUIPC = 5'b01011,
						 BEQ = 5'b01100,
						 BNE = 5'b01101,
						 BLT = 5'b01110,
						 BGE = 5'b01111,
						 BLTU = 5'b10000,
						 BGEU = 5'b10001,
						 JAL = 5'b10010,
						 JALR = 5'b10011,
						 LB = 5'b10100,
						 LH = 5'b10101,
						 LW = 5'b10110,
	 					 LBU = 5'b10111,
	 					 LHU = 5'b11000,
	 					 SB = 5'b11001,
	 					 SH = 5'b11010,
	 					 SW = 5'b11011,
						 FLUSH = 5'b11100;

	logic[31:0] registerA, registerB;
	alu_operation ALU(ALU_op, registerA, registerB, start_ALU, Rz);

	always_comb begin
		case(forward)
			2'b01:begin
				case(ALU_op)
					JALR, JAL, LUI, AUIPC:begin //not forward data in ALU input, forward it to pc address instead
						registerA = Ra;
						registerB = Rb;
					end

					default:begin
						registerA = i_forward_result;
						registerB = Rb;
					end
				endcase
			end
			2'b10:begin
				if(i_op_code == 7'b0010011 || ALU_op == JALR || ALU_op == JAL)begin
					registerA = Ra;
					registerB = Rb;
				end
				else begin
					registerA = Ra;
					registerB = i_forward_result;
				end
			end
			2'b11:begin
				if(i_op_code == 7'b0010011 || ALU_op == JALR || ALU_op == JAL)begin
					registerA = Ra;
					registerB = Rb;
				end
				else begin
					registerA = i_forward_result;
					registerB = i_forward_result;
				end
			end
			default:begin
				registerA = Ra;
				registerB = Rb;
			end
		endcase

		case(ALU_op)
			LB, LBU:begin
				need_write_back = 1'b1;

				case(forward) 
					2'b01:   o_ldst_addr = i_ldst_addr + i_forward_result;
					2'b11:	 o_ldst_addr = i_ldst_addr + i_forward_result;
					default: o_ldst_addr = i_ldst_addr + o_tb_regs[i_curr_rsone];
				endcase
				
				o_ldst_rd = 1'b1;
				o_ldst_byte_en = 4'b0001;

				o_st_data = 32'd0;
				o_ldst_wr = 1'b0;
				o_need_b = 1'b0;
				o_actual_b_pc = 32'd0;
			end
			LH, LHU:begin
				need_write_back = 1'b1;

				case(forward) 
					2'b01:   o_ldst_addr = i_ldst_addr + i_forward_result;
					2'b11:	 o_ldst_addr = i_ldst_addr + i_forward_result;
					default: o_ldst_addr = i_ldst_addr + o_tb_regs[i_curr_rsone];
				endcase
				o_ldst_rd = 1'b1;
				o_ldst_byte_en = 4'b0011;

				o_st_data = 32'd0;
				o_ldst_wr = 1'b0;
				o_need_b = 1'b0;
				o_actual_b_pc = 32'd0;
			end
			LW:begin
				need_write_back = 1'b1;

				case(forward) 
					2'b01:   o_ldst_addr = i_ldst_addr + i_forward_result;
					2'b11:	 o_ldst_addr = i_ldst_addr + i_forward_result;
					default: o_ldst_addr = i_ldst_addr + o_tb_regs[i_curr_rsone];
				endcase

				o_ldst_rd = 1'b1;
				o_ldst_byte_en = 4'b1111;

				o_st_data = 32'd0;
				o_ldst_wr = 1'b0;
				o_need_b = 1'b0;
				o_actual_b_pc = 32'd0;
			end

			SB:begin
				need_write_back = 1'b0;
				
				o_ldst_wr = 1'b1;
				o_ldst_byte_en = 4'b0001;
				case(forward)
					2'b10: begin
						o_st_data = {{24{1'd0}},i_forward_result[7:0]};
						o_ldst_addr = i_ldst_addr + o_tb_regs[i_curr_rsone];
					end
					2'b01: begin
						o_st_data = i_st_data;
						o_ldst_addr = i_ldst_addr + i_forward_result;
					end
					2'b11: begin
						o_st_data = {{24{1'd0}},i_forward_result[7:0]};
						o_ldst_addr = i_ldst_addr + i_forward_result;
					end
					default: begin
						o_st_data = i_st_data;
						o_ldst_addr = i_ldst_addr + o_tb_regs[i_curr_rsone];
					end
				endcase
				o_ldst_rd = 1'b0;
				o_need_b = 1'b0;
				o_actual_b_pc = 32'd0;
			end
			SH:begin
				need_write_back = 1'b0;
				o_ldst_addr = i_ldst_addr + o_tb_regs[i_curr_rsone];
				o_ldst_wr = 1'b1;
				o_ldst_byte_en = 4'b0011;
				case(forward)
					2'b10: begin
						o_st_data = {{16{1'd0}},i_forward_result[15:0]};
						o_ldst_addr = i_ldst_addr + o_tb_regs[i_curr_rsone];
					end
					2'b01: begin
						o_st_data = i_st_data;
						o_ldst_addr = i_ldst_addr + i_forward_result;
					end
					2'b11: begin
						o_st_data = {{16{1'd0}},i_forward_result[15:0]};
						o_ldst_addr = i_ldst_addr + i_forward_result;
					end
					default: begin
						o_st_data = i_st_data;
						o_ldst_addr = i_ldst_addr + o_tb_regs[i_curr_rsone];
					end
				endcase

				o_ldst_rd = 1'b0;
				o_need_b = 1'b0;
				o_actual_b_pc = 32'd0;
			end
			SW:begin
				need_write_back = 1'b0;
				o_ldst_addr = i_ldst_addr + o_tb_regs[i_curr_rsone];
				o_ldst_wr = 1'b1;
				o_ldst_byte_en = 4'b1111;
				case(forward)
					2'b10: begin
						o_st_data = i_forward_result;
						o_ldst_addr = i_ldst_addr + o_tb_regs[i_curr_rsone];
					end
					2'b01: begin
						o_st_data = i_st_data;
						o_ldst_addr = i_ldst_addr + i_forward_result;
					end
					2'b11: begin
						o_st_data = i_forward_result;
						o_ldst_addr = i_ldst_addr + i_forward_result;
					end
					default: begin
						o_st_data = i_st_data;
						o_ldst_addr = i_ldst_addr + o_tb_regs[i_curr_rsone];
					end
				endcase

				o_ldst_rd = 1'b0;
				o_need_b = 1'b0;
				o_actual_b_pc = 32'd0;
			end
			JAL:begin
				o_need_b = 1'b1;
				o_actual_b_pc = i_temp_b_pc;

				need_write_back = 1'b1;
				o_ldst_addr = 32'd0;
				o_ldst_wr = 1'b0;
				o_ldst_rd = 1'b0;
				o_st_data = 32'd0;
				o_ldst_byte_en = 4'd0;
			end
			JALR:begin
				o_need_b = 1'b1;
				if(forward == 2'b01) o_actual_b_pc = i_temp_b_pc + i_forward_result;
				if(forward == 2'b11) o_actual_b_pc = i_temp_b_pc + i_forward_result;
				else o_actual_b_pc = i_temp_b_pc + o_tb_regs[i_curr_rsone];

				need_write_back = 1'b1;
				o_ldst_addr = 32'd0;
				o_ldst_wr = 1'b0;
				o_ldst_rd = 1'b0;
				o_st_data = 32'd0;
				o_ldst_byte_en = 4'd0;
			end
			BEQ, BNE, BLT, BGE, BLTU, BGEU:begin
				need_write_back = 1'b0;
				o_need_b = Rz[0] ? 1'b1:1'b0;
				o_actual_b_pc = Rz[0] ? i_temp_b_pc:32'd0;

				o_ldst_addr = 32'd0;
				o_ldst_wr = 1'b0;
				o_ldst_rd = 1'b0;
				o_st_data = 32'd0;
				o_ldst_byte_en = 4'd0;

			end
			FLUSH:begin
				need_write_back = 1'b0;
				o_ldst_addr = 32'd0;
				o_ldst_wr = 1'b0;
				o_ldst_rd = 1'b0;
				o_st_data = 32'd0;
				o_ldst_byte_en = 4'd0;
				o_need_b = 1'b0;
				o_actual_b_pc = 32'd0;
			end
			default:begin
				need_write_back = 1'b1;
				o_ldst_addr = 32'd0;
				o_ldst_wr = 1'b0;
				o_ldst_rd = 1'b0;
				o_st_data = 32'hdeadbfff;
				o_ldst_byte_en = 4'd0;
				o_need_b = 1'b0;
				o_actual_b_pc = 32'd0;
			end
		endcase
	end

endmodule

module write_back(input logic[4:0] i_rd,
	input logic needwb, input logic[31:0] i_rz, input logic[4:0]ALU_op,
	input logic[31:0] i_ld_data, output logic[31:0] o_result_wb, input logic [31:0] st_data_wb);

	localparam LB  = 5'b10100,
			   LH  = 5'b10101,
			   LW  = 5'b10110,
			   LBU = 5'b10111,
			   LHU = 5'b11000,
			   SB  = 5'b11001,
	 		   SH  = 5'b11010,
	 		   SW  = 5'b11011;
	integer i;
	always_comb begin
		if(needwb)begin
			case(ALU_op)
				LB:begin
					o_result_wb = {{24{i_ld_data[7]}},i_ld_data[7:0]};
				end
				LH:begin
					o_result_wb = {{16{i_ld_data[15]}},i_ld_data[15:0]};
				end
				LW:begin
					o_result_wb = i_ld_data[31:0];
				end
				LBU:begin
					o_result_wb = {{24{1'b0}},i_ld_data[7:0]};
				end
				LHU:begin
					o_result_wb = {{16{1'b0}},i_ld_data[15:0]};
				end
				default:begin
					o_result_wb = i_rz;
				end
			endcase
		end
		else begin
			case(ALU_op)
				SB: o_result_wb = st_data_wb;
				SH: o_result_wb = st_data_wb;
				SW: o_result_wb = st_data_wb;
				default:  o_result_wb = i_rz;
			endcase
		end

		//o_tb_regs[0] = 32'd0; //ensure register 0 always 0
	end

endmodule

module alu_operation#(
    parameter N = 32
) (
    input [4:0] operation_code,
    input [N-1:0] Operand1,
		input  [N-1:0] Operand2,
		input i_start_ALU,

    output logic signed[N-1: 0] result
);
	localparam ADD = 5'b00000,
						 SUB = 5'b00001,
						 XOR = 5'b00010,
						 OR = 5'b00011,
						 AND = 5'b00100,
						 SLL = 5'b00101,
						 SRL = 5'b00110,
						 SRA = 5'b00111,
						 SLT = 5'b01000,
						 SLTU = 5'b01001,
						 LUI = 5'b01010,
						 AUIPC = 5'b01011,
						 BEQ = 5'b01100,
						 BNE = 5'b01101,
						 BLT = 5'b01110,
						 BGE = 5'b01111,
						 BLTU = 5'b10000,
						 BGEU = 5'b10001,
						 JAL = 5'b10010,
						 JALR = 5'b10011;
   logic flagZ;
   logic flagC;
	logic overflow, negative;

	always_comb begin
		if(i_start_ALU)begin
			case(operation_code)
				ADD, JAL, JALR:begin
					negative = 0;
					overflow = 0;
					{flagC,result} = Operand1 + Operand2;
					flagZ  = (result == 32'd0);
				end
				SUB:begin
					negative = 0;
					overflow = 0;
					{flagC,result} = Operand1 - Operand2;
					flagZ  = (result == 32'd0);
				end
				XOR:begin
					negative = 0;
					overflow = 0;
					result = Operand1 ^ Operand2;
	 				flagZ  = (result == 32'd0);
					flagC  = 0;
				end
				OR:begin
					negative = 0;
					overflow = 0;
					result = Operand1 | Operand2;
					flagZ  = (result == 32'd0);
					flagC  = 0;
				end
				AND:begin
					negative = 0;
					overflow = 0;
					result = Operand1 & Operand2;
					flagZ  = (result == 32'd0);
					flagC  = 0;
				end
				SLL:begin
					negative = 0;
					overflow = 0;
					result = Operand1 << Operand2[4:0];
					flagC  = 0;
					flagZ  = 0;
				end
				SRL:begin
					negative = 0;
					overflow = 0;
					result = Operand1 >> Operand2[4:0];
					flagC  = 0;
					flagZ  = 0;
				end
				SRA:begin
					negative = 0;
					overflow = 0;
					result = $signed(Operand1) >>> Operand2[4:0];
					flagC  = 0;
					flagZ  = 0;
				end
				SLT, BLT:begin
					{flagC,result} = Operand1 - Operand2;
					negative = result[31];
					overflow = Operand1[31]^Operand2[31]^result[31]^flagC;
					result = {31'd0, negative^overflow};
					flagZ  = 0;
				end
				SLTU, BLTU:begin
					negative = 0;
					overflow = 0;
					{flagC,result} = Operand1 - Operand2;
					result = {31'd0, flagC};
					flagZ  = 0;
				end
				LUI:begin
					negative = 0;
					overflow = 0;
					result = Operand1 << 12;
					flagC  = 0;
					flagZ  = 0;
				end
				AUIPC:begin
					negative = 0;
					overflow = 0;
					result = Operand2 + (Operand1 << 12);
					flagC  = 0;
					flagZ  = 0;
				end
				BEQ:begin
					negative = 0;
					overflow = 0;
					{flagC,result} = Operand1 - Operand2;
					flagZ  = (result == 32'd0);
					result = {31'd0, flagZ};
				end
				BNE:begin
					negative = 0;
					overflow = 0;
					{flagC,result} = Operand1 - Operand2;
					flagZ  = (result == 32'd0);
					result = {31'd0, ~flagZ};
				end
				BGE:begin
					{flagC,result} = Operand2 - Operand1;
					negative = result[31];
					overflow = Operand1[31]^Operand2[31]^result[31]^flagC;
					flagZ  = (result == 32'd0);
					result = {31'd0, (negative^overflow) | flagZ};
				end
				BGEU:begin
					negative = 0;
					overflow = 0;
					{flagC,result} = Operand2 - Operand1;
					flagZ  = (result == 32'd0);
					result = {31'd0, flagC | flagZ};
				end
				default: begin
					negative = 0;
					overflow = 0;
					{flagC,result} = 0;
					flagZ  = 0;
					result = 0;
				end
			endcase
		end else begin
			negative = 0;
			overflow = 0;
			{flagC,result} = 0;
			flagZ  = 0;
		end
	end
endmodule
