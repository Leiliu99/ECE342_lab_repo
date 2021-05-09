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

	logic[IW-1:0] instruction_data; //instruction data read from pc
	logic[6:0]op_code;

	//---------read instruction from------------
	//logic instruction_en; //the enable signal to read instruction(read only port)
	//logic instruction_en_ldst; //the enable signal to read instruction(read/write port)
	//reg_n reg_instruction (clk, reset, instruction_en, i_pc_rddata, instruction_data);
	//reg_n reg_instruction_ldst (clk, reset, instruction_en_ldst, i_ldst_rddata, instruction_data);
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
	//-----------instruction format------------------
	logic[6:0] func_seven;
	logic[4:0] rs_one, rs_two, rd;
	logic[2:0] func_three;
	logic[11:0] imm_I, imm_S;
	logic[19:0] imm_U;
	logic[12:0] imm_B;
	logic[20:0] imm_J;
	logic[IW-1:0] register_a, register_b, register_z;
	logic[IW-1:0] temp_ld_address, temp_st_address;
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
	assign temp_ld_address = o_tb_regs[rs_one] + {{20{imm_I[11]}}, imm_I[11:0]};
  assign temp_st_address = o_tb_regs[rs_one] + {{20{imm_S[11]}}, imm_S[11:0]};
	//------------stage 3: S_T3, ALU part----------------
	logic start_ALU;
	logic[4:0]ALU_operation_code;
	logic[6:0]func_seven_I;
	assign func_seven_I = imm_I[11:5];
	localparam func_seven_zero = 7'b0000000,
					func_seven_one = 7'b0100000;
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
	ALU alu_operation(ALU_operation_code, register_a, register_b, start_ALU, register_z);
	always_comb begin
		case(op_code)
			format_R:begin
				case(func_three)
					3'b000:begin
						case(func_seven)
							func_seven_zero: ALU_operation_code = ADD;
							func_seven_one: ALU_operation_code = SUB;
						endcase
					end
					3'b001:begin
						ALU_operation_code = SLL;
					end
					3'b010:begin
						ALU_operation_code = SLT;
					end
					3'b011:begin
						ALU_operation_code = SLTU;
					end
					3'b100:begin
						ALU_operation_code = XOR;
					end
					3'b101:begin
						case(func_seven)
							func_seven_zero: ALU_operation_code = SRL;
							func_seven_one: ALU_operation_code = SRA;
						endcase
					end
					3'b110:begin
						ALU_operation_code = OR;
					end
					3'b111:begin
						ALU_operation_code = AND;
					end
				endcase
			end

			format_I:begin
				case(func_three)
					3'b000:begin
						ALU_operation_code = ADD;
					end
					3'b001:begin
						ALU_operation_code = SLL;
					end
					3'b010:begin
						ALU_operation_code = SLT;
					end
					3'b011:begin
						ALU_operation_code = SLTU;
					end
					3'b100:begin
						ALU_operation_code = XOR;
					end
					3'b101:begin
						case(func_seven)
							func_seven_zero: ALU_operation_code = SRL;
							func_seven_one: ALU_operation_code = SRA;
						endcase
					end
					3'b110:begin
						ALU_operation_code = OR;
					end
					3'b111:begin
						ALU_operation_code = AND;
					end
				endcase
			end

			format_U_lui: ALU_operation_code = LUI;
			format_U_auipc: ALU_operation_code = AUIPC;
			format_B:begin
				case(func_three)
					3'b000: ALU_operation_code = BEQ;
					3'b001: ALU_operation_code = BNE;
					3'b100: ALU_operation_code = BLT;
					3'b101: ALU_operation_code = BGE;
					3'b110: ALU_operation_code = BLTU;
					3'b111: ALU_operation_code = BGEU;
				endcase
			end
			format_J_jal, format_J_jalr: ALU_operation_code = ADD;
		endcase
	end

	always_comb begin
		case(op_code)
			format_R, format_B:begin
				register_a = o_tb_regs[rs_one];
				register_b = o_tb_regs[rs_two];
			end
			format_I:begin
				register_a = o_tb_regs[rs_one];
				register_b = {{20{imm_I[11]}}, imm_I[11:0]};
			end
			format_U_lui, format_U_auipc:begin
				register_a = {{12{imm_U[19]}}, imm_U[19:0]};
				register_b = o_pc_addr;
			end
			format_J_jal, format_J_jalr:begin
				register_a = o_pc_addr;
				register_b = 32'd4;
			end
			default:begin
				register_a = o_tb_regs[rs_one];
				register_b = o_tb_regs[rs_two];
			end
		endcase
	end

	//------------stage 4: S_T4, store to the RY---------
	logic start_store_Y;
	logic[1:0] sel_Y;
	logic[IW-1:0] register_y;
	logic need_branch;
	always_comb begin
		case(op_code)
			format_R: sel_Y = 2'b00;
			format_I: sel_Y = 2'b00;
			format_U_lui: sel_Y = 2'b00;
			format_U_auipc: sel_Y = 2'b00;
			format_J_jal: sel_Y = 2'b00;
			format_J_jalr: sel_Y = 2'b00;
			format_I_ld: sel_Y = 2'b01;
			default: sel_Y = 2'b00;
		endcase
	end

	always_comb begin
		if(start_store_Y)begin
			case(sel_Y)
				2'b00: register_y = register_z;
			endcase
		end

		if(op_code == format_B)begin
			need_branch = register_z[0] ? 1'd1:1'd0;
		end
		else need_branch = 1'd0;
	end

	always_ff @ (posedge clk) begin
		if(start_store_Y)begin
			case(sel_Y)
				2'b01:begin
					case(func_three)
						3'b000: o_tb_regs[rd] <= {{24{i_ldst_rddata[7]}},i_ldst_rddata[7:0]};
						3'b001: o_tb_regs[rd] <= {{16{i_ldst_rddata[15]}},i_ldst_rddata[15:0]};
						3'b010: o_tb_regs[rd] <= i_ldst_rddata[31:0];
						3'b100: o_tb_regs[rd] <= {{24{1'b0}},i_ldst_rddata[7:0]};
						3'b101: o_tb_regs[rd] <= {{16{1'b0}},i_ldst_rddata[15:0]};
					endcase
				end
			endcase
		end
	end

	//------------stage 5: S_T5--------------
	logic start_stage5;
	always_ff @(posedge clk) begin
		if(start_stage5)begin
			o_tb_regs[rd] <= register_y;
			o_tb_regs[0] <= 32'd0;
		end
	end
	//-----------increment pc----------------
	logic PC_increment;
	logic [IW-1: 0] increment;
	assign increment = 32'd4;

	always_ff @(posedge clk) begin
		if (reset) o_pc_addr <= '0;
		else if(PC_increment)begin
			if(need_branch) o_pc_addr <= o_pc_addr + {{19{imm_B[12]}},imm_B[12:0]};
			else if(op_code == format_J_jal) o_pc_addr <= o_pc_addr + {{11{imm_J[20]}}, imm_J[20:0]};
			else if(op_code == format_J_jalr) o_pc_addr <= o_tb_regs[rs_one] + {{20{imm_I[11]}}, imm_I[11:0]};
			else o_pc_addr <= o_pc_addr + increment;
		end
	end

	enum int unsigned {
      S_T1, S_T2, S_T3, S_T4, S_T5
    } state, next_state;

	always_ff @ (posedge clk or posedge reset) begin
        if (reset)begin
					state <= S_T1;
					o_tb_regs[0] <= 32'd0;
					o_tb_regs[1] <= 32'd0;
					o_tb_regs[2] <= 32'd0;
					o_tb_regs[3] <= 32'd0;
					o_tb_regs[4] <= 32'd0;
					o_tb_regs[5] <= 32'd0;
					o_tb_regs[6] <= 32'd0;
					o_tb_regs[7] <= 32'd0;
					o_tb_regs[8] <= 32'd0;
					o_tb_regs[9] <= 32'd0;
					o_tb_regs[10] <= 32'd0;
					o_tb_regs[11] <= 32'd0;
					o_tb_regs[12] <= 32'd0;
					o_tb_regs[13] <= 32'd0;
					o_tb_regs[14] <= 32'd0;
					o_tb_regs[15] <= 32'd0;
					o_tb_regs[16] <= 32'd0;
					o_tb_regs[17] <= 32'd0;
					o_tb_regs[18] <= 32'd0;
					o_tb_regs[19] <= 32'd0;
					o_tb_regs[20] <= 32'd0;
					o_tb_regs[21] <= 32'd0;
					o_tb_regs[22] <= 32'd0;
					o_tb_regs[23] <= 32'd0;
					o_tb_regs[24] <= 32'd0;
					o_tb_regs[25] <= 32'd0;
					o_tb_regs[26] <= 32'd0;
					o_tb_regs[27] <= 32'd0;
					o_tb_regs[28] <= 32'd0;
					o_tb_regs[29] <= 32'd0;
					o_tb_regs[30] <= 32'd0;
					o_tb_regs[31] <= 32'd0;
					//more signal asserts here......
				end
        else state <= next_state;
    end

	always_comb begin
		next_state = state;
		{start_ALU, start_store_Y, start_stage5, PC_increment, o_ldst_rd, o_ldst_wr} = 1'b0;
		case(state)
			S_T1:begin
				o_pc_rd = 1'b1;
				next_state = S_T2;
			end
			S_T2:begin
				instruction_data = i_pc_rddata;
				next_state = S_T3;
			end
			S_T3:begin
				if(op_code == format_I_ld)begin
					start_ALU = 1'b0;
					o_ldst_addr = temp_ld_address;
					o_ldst_rd = 1'b1;
					case(func_three)
						3'b000, 3'b100: o_ldst_byte_en = 4'b0001;
						3'b001, 3'b101: o_ldst_byte_en = 4'b0011;
						3'b010: o_ldst_byte_en = 4'b1111;
					endcase
					next_state = S_T4;
				end
				else if(op_code == format_S)begin
					start_ALU = 1'b0;
					o_ldst_addr = temp_st_address;
					o_ldst_wr = 1'b1;
					case(func_three)
						3'b000:begin
							o_ldst_wrdata = {{24{1'd0}},o_tb_regs[rs_two][7:0]};
							o_ldst_byte_en = 4'b0001;
						end
						3'b001:begin
							o_ldst_wrdata = {{16{1'd0}},o_tb_regs[rs_two][15:0]};
							o_ldst_byte_en = 4'b0011;
						end
						3'b010:begin
							o_ldst_wrdata = o_tb_regs[rs_two];
							o_ldst_byte_en = 4'b1111;
						end
					endcase
					PC_increment = 1'b1; //enable pc increment
					next_state = S_T1;
				end
				else begin
					start_ALU = 1'b1;
					next_state = S_T4;
				end
			end
			S_T4:begin
				start_store_Y = 1'b1;
				next_state = S_T5;
			end
			S_T5:begin
				if(op_code == format_B | op_code == format_I_ld) start_stage5 = 1'b0;
				else start_stage5 = 1'b1;
				next_state = S_T1;
				PC_increment = 1'b1; //enable pc increment
			end
		endcase
	end
endmodule


module ALU#(
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
   logic flagLTU;
	logic overflow, negative;
	logic msb;
	assign msb = Operand1[N-1];
	always_comb begin
			if(i_start_ALU)begin
			case(operation_code)
				ADD, JAL, JALR:begin
					{flagC,result} = Operand1 + Operand2;
					flagZ  = (result == 32'd0);
				end
				SUB:begin
					{flagC,result} = Operand1 - Operand2;
					flagZ  = (result == 32'd0);
				end
				XOR:begin
					result = Operand1 ^ Operand2;
	 				flagZ  = (result == 32'd0);
				end
				OR:begin
					result = Operand1 | Operand2;
					flagZ  = (result == 32'd0);
				end
				AND:begin
					result = Operand1 & Operand2;
					flagZ  = (result == 32'd0);
				end
				SLL:begin
					result = Operand1 << Operand2[4:0];
				end
				SRL:begin
					result = Operand1 >> Operand2[4:0];
				end
				SRA:begin
					result = $signed(Operand1) >>> Operand2[4:0];
				end
				SLT, BLT:begin
					{flagC,result} = Operand1 - Operand2;
					negative = result[31];
					overflow = Operand1[31]^Operand2[31]^result[31]^flagC;
					result = {31'd0, negative^overflow};
				end
				SLTU, BLTU:begin
					{flagC,result} = Operand1 - Operand2;
					result = {31'd0, flagC};
				end
				LUI:begin
					result = Operand1 << 12;
				end
				AUIPC:begin
					result = Operand2 + (Operand1 << 12);
				end
				BEQ:begin
					{flagC,result} = Operand1 - Operand2;
					flagZ  = (result == 32'd0);
					result = {31'd0, flagZ};
				end
				BNE:begin
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
					{flagC,result} = Operand2 - Operand1;
					flagZ  = (result == 32'd0);
					result = {31'd0, flagC | flagZ};
				end
			endcase
		end
	end
endmodule
