module part3
(
    input                       clk,
    input				                reset
);

  /*--------------------logic parameter definition------------------------*/
  //// read only port in processor
  logic [31:0] pc_addr; //pc address output from processor
  logic pc_read_en; //read pc signal ouput from processor
  logic [31:0] pc_read_data;
  logic [3:0] pc_byte_en;

  //// read/write port in processor
  logic [31:0] ldst_addr;
  logic ldst_read_en;
  logic ldst_write_en; //write signal ouput from processor
  logic [31:0] ldst_read_data;
  logic [31:0] ldst_write_data; //write data ouput from processor
  logic [3:0] ldst_byte_en; //byte enable signal ouput processor

  logic [31:0] tb_regs [0:31];

  //// read/write port in memory
  logic [12:0] p1_rw_addr;
  logic [31:0] p1_read_data;
  logic p1_write;
  logic p1_read;

  ////read only port in memory
  logic [12:0] p2_read_addr;
  logic [31:0] p2_read_data;
  logic p2_read;

  //wait request signal
  logic ldst_waitrequest;

  ////input output in avalon
  logic [2:0] avs_address;
  logic avs_read;
  logic avs_write;
  logic [31:0] avs_readdata;
  logic [31:0] avs_writedata;

  assign p1_rw_addr = ldst_addr[14:2];
  assign p2_read_addr = pc_addr[14:2];
  assign avs_writedata = ldst_write_data;


  always_comb begin
    if(ldst_read_en == 1'b1 && ldst_addr == 32'ha00c)begin
      p1_read = 1'b0;
      p1_write = 1'b0;
      p2_read = 1'b0;
      avs_write = 1'b0;
      avs_read = 1'b1;
      avs_address = 3'b011;
    end
    else if(ldst_read_en == 1'b1 && ldst_addr == 32'ha008)begin
      p1_read = 1'b0;
      p1_write = 1'b0;
      p2_read = 1'b0;
      avs_write = 1'b0;
      avs_read = 1'b1;
      avs_address = 3'b010;
    end
    else if(ldst_write_en == 1'b1 && ldst_addr == 32'ha000)begin
      p1_read = 1'b0;
      p1_write = 1'b0;
      p2_read = 1'b0;
      avs_write = 1'b1;
      avs_read = 1'b0;
      avs_address = 3'b000;
      // avs_writedata = ldst_write_data;
    end
    else if(ldst_write_en == 1'b1 && ldst_addr == 32'ha004)begin
      p1_read = 1'b0;
      p1_write = 1'b0;
      p2_read = 1'b0;
      avs_write = 1'b1;
      avs_read = 1'b0;
      avs_address = 3'b001;
      // avs_writedata = ldst_write_data;
    end
    else if(ldst_write_en == 1'b1 && ldst_addr == 32'ha008)begin
      p1_read = 1'b0;
      p1_write = 1'b0;
      p2_read = 1'b0;
      avs_write = 1'b1;
      avs_read = 1'b0;
      avs_address = 3'b010;
      // avs_writedata = ldst_write_data;
    end
    else if(ldst_write_en == 1'b1 && ldst_addr == 32'ha00c)begin
      p1_read = 1'b0;
      p1_write = 1'b0;
      p2_read = 1'b0;
      avs_write = 1'b1;
      avs_read = 1'b0;
      avs_address = 3'b011;
      // avs_writedata = ldst_write_data;
    end
    else begin
      p1_read = ldst_read_en;
      p1_write = ldst_write_en;
      p2_read = pc_read_en;
    end
  end

  always_ff @(posedge clk) begin
    if(ldst_read_en == 1'b1 && ldst_addr == 32'ha00c)begin
      ldst_read_data <= avs_readdata;
    end

    else if(ldst_read_en == 1'b1)begin
      ldst_read_data <= p1_read_data;
    end
    else if(pc_read_en == 1'b1)begin
      pc_read_data <= p2_read_data;
    end
  end

  avalon_fp_mult mult
  (
    .clk(clk),
    .reset(reset),

    // Avalon Slave
    .avs_s1_address(avs_address),
    .avs_s1_read(avs_read),
    .avs_s1_write(avs_write),
    .avs_s1_writedata(avs_writedata),
    .avs_s1_readdata(avs_readdata),
    .avs_s1_waitrequest(ldst_waitrequest)
  );

  cpu cpu_processor(.clk(clk), .reset(reset), .i_ldst_waitrequest(ldst_waitrequest),.o_pc_addr(pc_addr),
  .o_pc_rd(pc_read_en), .i_pc_rddata(pc_read_data),.o_pc_byte_en(pc_byte_en), .o_ldst_addr(ldst_addr),
  .o_ldst_rd(ldst_read_en),.o_ldst_wr(ldst_write_en), .i_ldst_rddata(ldst_read_data), .o_ldst_wrdata(ldst_write_data),
  .o_ldst_byte_en(ldst_byte_en), .o_tb_regs(tb_regs));

  mem mem_inst(clk, reset, p1_rw_addr, p1_write, p1_read, ldst_byte_en, ldst_write_data, p1_read_data,
    p2_read_addr, p2_read, pc_byte_en, p2_read_data);


endmodule


module cpu # (
	parameter IW = 32, // instr width
	parameter REGS = 32 // number of registers
)(
	input clk,
	input reset,

	//load store wait request
	input i_ldst_waitrequest,

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

	assign o_pc_byte_en = 4'b1111;

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

	// always_ff @ (posedge clk) begin
	//
	// end

	//------------stage 5: S_T5--------------
	logic start_stage5;
	// always_ff @(posedge clk) begin
	//
	// end
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

	integer i;
	always_ff @ (posedge clk or posedge reset) begin
        if (reset)begin
					state <= S_T1;
					for (i = 0; i < 32; i++)
                o_tb_regs[i] <= '0;
					//more signal asserts here......
				end
        else begin
					state <= next_state;
					if(start_stage5)begin
						o_tb_regs[rd] <= register_y;
						o_tb_regs[0] <= 32'd0;
					end
					else if(start_store_Y)begin
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
					if(i_ldst_waitrequest == 1'b1)begin
						next_state = S_T3;
					end
					else begin
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
				end
				else if(op_code == format_S)begin
					if(i_ldst_waitrequest == 1'b1)begin
						next_state = S_T3;
					end
					else begin
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

`timescale 1 ps / 1 ps
//synopsys translate_on
module  fp_mult_altfp_mult_her
	(
	aclr,
	clk_en,
	clock,
	dataa,
	datab,
	nan,
	overflow,
	result,
	underflow,
	zero) ;
	input   aclr;
	input   clk_en;
	input   clock;
	input   [31:0]  dataa;
	input   [31:0]  datab;
	output   nan;
	output   overflow;
	output   [31:0]  result;
	output   underflow;
	output   zero;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_off
`endif
	tri0   aclr;
	tri1   clk_en;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_on
`endif

	reg	dataa_exp_all_one_ff_p1;
	reg	dataa_exp_not_zero_ff_p1;
	reg	dataa_man_not_zero_ff_p1;
	reg	dataa_man_not_zero_ff_p2;
	reg	datab_exp_all_one_ff_p1;
	reg	datab_exp_not_zero_ff_p1;
	reg	datab_man_not_zero_ff_p1;
	reg	datab_man_not_zero_ff_p2;
	reg	[9:0]	delay_exp2_bias;
	reg	[9:0]	delay_exp3_bias;
	reg	[9:0]	delay_exp_bias;
	reg	delay_man_product_msb;
	reg	delay_man_product_msb2;
	reg	delay_man_product_msb_p0;
	reg	delay_man_product_msb_p1;
	reg	[23:0]	delay_round;
	reg	[8:0]	exp_add_p1;
	reg	[9:0]	exp_adj_p1;
	reg	[9:0]	exp_adj_p2;
	reg	[8:0]	exp_bias_p1;
	reg	[8:0]	exp_bias_p2;
	reg	[8:0]	exp_bias_p3;
	reg	[7:0]	exp_result_ff;
	reg	input_is_infinity_dffe_0;
	reg	input_is_infinity_dffe_1;
	reg	input_is_infinity_dffe_2;
	reg	input_is_infinity_dffe_3;
	reg	input_is_infinity_ff1;
	reg	input_is_infinity_ff2;
	reg	input_is_infinity_ff3;
	reg	input_is_infinity_ff4;
	reg	input_is_infinity_ff5;
	reg	input_is_nan_dffe_0;
	reg	input_is_nan_dffe_1;
	reg	input_is_nan_dffe_2;
	reg	input_is_nan_dffe_3;
	reg	input_is_nan_ff1;
	reg	input_is_nan_ff2;
	reg	input_is_nan_ff3;
	reg	input_is_nan_ff4;
	reg	input_is_nan_ff5;
	reg	input_not_zero_dffe_0;
	reg	input_not_zero_dffe_1;
	reg	input_not_zero_dffe_2;
	reg	input_not_zero_dffe_3;
	reg	input_not_zero_ff1;
	reg	input_not_zero_ff2;
	reg	input_not_zero_ff3;
	reg	input_not_zero_ff4;
	reg	input_not_zero_ff5;
	reg	lsb_dffe;
	reg	[22:0]	man_result_ff;
	reg	man_round_carry;
	reg	man_round_carry_p0;
	reg	[23:0]	man_round_p;
	reg	[23:0]	man_round_p0;
	reg	[23:0]	man_round_p1;
	reg	[24:0]	man_round_p2;
	reg	nan_ff;
	reg	overflow_ff;
	reg	round_dffe;
	reg	[0:0]	sign_node_ff0;
	reg	[0:0]	sign_node_ff1;
	reg	[0:0]	sign_node_ff2;
	reg	[0:0]	sign_node_ff3;
	reg	[0:0]	sign_node_ff4;
	reg	[0:0]	sign_node_ff5;
	reg	[0:0]	sign_node_ff6;
	reg	[0:0]	sign_node_ff7;
	reg	[0:0]	sign_node_ff8;
	reg	[0:0]	sign_node_ff9;
	reg	[0:0]	sign_node_ff10;
	reg	sticky_dffe;
	reg	underflow_ff;
	reg	zero_ff;
	wire  [8:0]   wire_exp_add_adder_result;
	wire  [9:0]   wire_exp_adj_adder_result;
	wire  [9:0]   wire_exp_bias_subtr_result;
	wire  [24:0]   wire_man_round_adder_result;
	wire  [47:0]   wire_man_product2_mult_result;
	wire  [9:0]  bias;
	wire  [7:0]  dataa_exp_all_one;
	wire  [7:0]  dataa_exp_not_zero;
	wire  [22:0]  dataa_man_not_zero;
	wire  [7:0]  datab_exp_all_one;
	wire  [7:0]  datab_exp_not_zero;
	wire  [22:0]  datab_man_not_zero;
	wire  exp_is_inf;
	wire  exp_is_zero;
	wire  [9:0]  expmod;
	wire  [7:0]  inf_num;
	wire  lsb_bit;
	wire  [24:0]  man_shift_full;
	wire  [7:0]  result_exp_all_one;
	wire  [8:0]  result_exp_not_zero;
	wire  round_bit;
	wire  round_carry;
	wire  [22:0]  sticky_bit;

	// synopsys translate_off
	initial
		dataa_exp_all_one_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_exp_all_one_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_exp_all_one_ff_p1 <= dataa_exp_all_one[7];
	// synopsys translate_off
	initial
		dataa_exp_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_exp_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_exp_not_zero_ff_p1 <= dataa_exp_not_zero[7];
	// synopsys translate_off
	initial
		dataa_man_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_man_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_man_not_zero_ff_p1 <= dataa_man_not_zero[10];
	// synopsys translate_off
	initial
		dataa_man_not_zero_ff_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_man_not_zero_ff_p2 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_man_not_zero_ff_p2 <= dataa_man_not_zero[22];
	// synopsys translate_off
	initial
		datab_exp_all_one_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_exp_all_one_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_exp_all_one_ff_p1 <= datab_exp_all_one[7];
	// synopsys translate_off
	initial
		datab_exp_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_exp_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_exp_not_zero_ff_p1 <= datab_exp_not_zero[7];
	// synopsys translate_off
	initial
		datab_man_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_man_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_man_not_zero_ff_p1 <= datab_man_not_zero[10];
	// synopsys translate_off
	initial
		datab_man_not_zero_ff_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_man_not_zero_ff_p2 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_man_not_zero_ff_p2 <= datab_man_not_zero[22];
	// synopsys translate_off
	initial
		delay_exp2_bias = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_exp2_bias <= 10'b0;
		else if  (clk_en == 1'b1)   delay_exp2_bias <= delay_exp_bias;
	// synopsys translate_off
	initial
		delay_exp3_bias = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_exp3_bias <= 10'b0;
		else if  (clk_en == 1'b1)   delay_exp3_bias <= delay_exp2_bias;
	// synopsys translate_off
	initial
		delay_exp_bias = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_exp_bias <= 10'b0;
		else if  (clk_en == 1'b1)   delay_exp_bias <= wire_exp_bias_subtr_result;
	// synopsys translate_off
	initial
		delay_man_product_msb = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_man_product_msb <= 1'b0;
		else if  (clk_en == 1'b1)   delay_man_product_msb <= delay_man_product_msb_p1;
	// synopsys translate_off
	initial
		delay_man_product_msb2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_man_product_msb2 <= 1'b0;
		else if  (clk_en == 1'b1)   delay_man_product_msb2 <= delay_man_product_msb;
	// synopsys translate_off
	initial
		delay_man_product_msb_p0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_man_product_msb_p0 <= 1'b0;
		else if  (clk_en == 1'b1)   delay_man_product_msb_p0 <= wire_man_product2_mult_result[47];
	// synopsys translate_off
	initial
		delay_man_product_msb_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_man_product_msb_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   delay_man_product_msb_p1 <= delay_man_product_msb_p0;
	// synopsys translate_off
	initial
		delay_round = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_round <= 24'b0;
		else if  (clk_en == 1'b1)   delay_round <= ((man_round_p2[23:0] & {24{(~ man_round_p2[24])}}) | (man_round_p2[24:1] & {24{man_round_p2[24]}}));
	// synopsys translate_off
	initial
		exp_add_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_add_p1 <= 9'b0;
		else if  (clk_en == 1'b1)   exp_add_p1 <= wire_exp_add_adder_result;
	// synopsys translate_off
	initial
		exp_adj_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_adj_p1 <= 10'b0;
		else if  (clk_en == 1'b1)   exp_adj_p1 <= delay_exp3_bias;
	// synopsys translate_off
	initial
		exp_adj_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_adj_p2 <= 10'b0;
		else if  (clk_en == 1'b1)   exp_adj_p2 <= wire_exp_adj_adder_result;
	// synopsys translate_off
	initial
		exp_bias_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_bias_p1 <= 9'b0;
		else if  (clk_en == 1'b1)   exp_bias_p1 <= exp_add_p1[8:0];
	// synopsys translate_off
	initial
		exp_bias_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_bias_p2 <= 9'b0;
		else if  (clk_en == 1'b1)   exp_bias_p2 <= exp_bias_p1;
	// synopsys translate_off
	initial
		exp_bias_p3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_bias_p3 <= 9'b0;
		else if  (clk_en == 1'b1)   exp_bias_p3 <= exp_bias_p2;
	// synopsys translate_off
	initial
		exp_result_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_result_ff <= 8'b0;
		else if  (clk_en == 1'b1)   exp_result_ff <= ((inf_num & {8{((exp_is_inf | input_is_infinity_ff5) | input_is_nan_ff5)}}) | ((exp_adj_p2[7:0] & {8{(~ exp_is_zero)}}) & {8{input_not_zero_ff5}}));
	// synopsys translate_off
	initial
		input_is_infinity_dffe_0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_0 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_0 <= ((dataa_exp_all_one_ff_p1 & (~ (dataa_man_not_zero_ff_p1 | dataa_man_not_zero_ff_p2))) | (datab_exp_all_one_ff_p1 & (~ (datab_man_not_zero_ff_p1 | datab_man_not_zero_ff_p2))));
	// synopsys translate_off
	initial
		input_is_infinity_dffe_1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_1 <= input_is_infinity_dffe_0;
	// synopsys translate_off
	initial
		input_is_infinity_dffe_2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_2 <= input_is_infinity_dffe_1;
	// synopsys translate_off
	initial
		input_is_infinity_dffe_3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_3 <= input_is_infinity_dffe_2;
	// synopsys translate_off
	initial
		input_is_infinity_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff1 <= input_is_infinity_dffe_3;
	// synopsys translate_off
	initial
		input_is_infinity_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff2 <= input_is_infinity_ff1;
	// synopsys translate_off
	initial
		input_is_infinity_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff3 <= input_is_infinity_ff2;
	// synopsys translate_off
	initial
		input_is_infinity_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff4 <= input_is_infinity_ff3;
	// synopsys translate_off
	initial
		input_is_infinity_ff5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff5 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff5 <= input_is_infinity_ff4;
	// synopsys translate_off
	initial
		input_is_nan_dffe_0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_0 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_0 <= ((dataa_exp_all_one_ff_p1 & (dataa_man_not_zero_ff_p1 | dataa_man_not_zero_ff_p2)) | (datab_exp_all_one_ff_p1 & (datab_man_not_zero_ff_p1 | datab_man_not_zero_ff_p2)));
	// synopsys translate_off
	initial
		input_is_nan_dffe_1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_1 <= input_is_nan_dffe_0;
	// synopsys translate_off
	initial
		input_is_nan_dffe_2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_2 <= input_is_nan_dffe_1;
	// synopsys translate_off
	initial
		input_is_nan_dffe_3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_3 <= input_is_nan_dffe_2;
	// synopsys translate_off
	initial
		input_is_nan_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff1 <= input_is_nan_dffe_3;
	// synopsys translate_off
	initial
		input_is_nan_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff2 <= input_is_nan_ff1;
	// synopsys translate_off
	initial
		input_is_nan_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff3 <= input_is_nan_ff2;
	// synopsys translate_off
	initial
		input_is_nan_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff4 <= input_is_nan_ff3;
	// synopsys translate_off
	initial
		input_is_nan_ff5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff5 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff5 <= input_is_nan_ff4;
	// synopsys translate_off
	initial
		input_not_zero_dffe_0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_0 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_0 <= (dataa_exp_not_zero_ff_p1 & datab_exp_not_zero_ff_p1);
	// synopsys translate_off
	initial
		input_not_zero_dffe_1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_1 <= input_not_zero_dffe_0;
	// synopsys translate_off
	initial
		input_not_zero_dffe_2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_2 <= input_not_zero_dffe_1;
	// synopsys translate_off
	initial
		input_not_zero_dffe_3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_3 <= input_not_zero_dffe_2;
	// synopsys translate_off
	initial
		input_not_zero_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff1 <= input_not_zero_dffe_3;
	// synopsys translate_off
	initial
		input_not_zero_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff2 <= input_not_zero_ff1;
	// synopsys translate_off
	initial
		input_not_zero_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff3 <= input_not_zero_ff2;
	// synopsys translate_off
	initial
		input_not_zero_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff4 <= input_not_zero_ff3;
	// synopsys translate_off
	initial
		input_not_zero_ff5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff5 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff5 <= input_not_zero_ff4;
	// synopsys translate_off
	initial
		lsb_dffe = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) lsb_dffe <= 1'b0;
		else if  (clk_en == 1'b1)   lsb_dffe <= lsb_bit;
	// synopsys translate_off
	initial
		man_result_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_result_ff <= 23'b0;
		else if  (clk_en == 1'b1)   man_result_ff <= {((((((delay_round[22] & input_not_zero_ff5) & (~ input_is_infinity_ff5)) & (~ exp_is_inf)) & (~ exp_is_zero)) | (input_is_infinity_ff5 & (~ input_not_zero_ff5))) | input_is_nan_ff5), (((((delay_round[21:0] & {22{input_not_zero_ff5}}) & {22{(~ input_is_infinity_ff5)}}) & {22{(~ exp_is_inf)}}) & {22{(~ exp_is_zero)}}) & {22{(~ input_is_nan_ff5)}})};
	// synopsys translate_off
	initial
		man_round_carry = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_carry <= 1'b0;
		else if  (clk_en == 1'b1)   man_round_carry <= man_round_carry_p0;
	// synopsys translate_off
	initial
		man_round_carry_p0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_carry_p0 <= 1'b0;
		else if  (clk_en == 1'b1)   man_round_carry_p0 <= round_carry;
	// synopsys translate_off
	initial
		man_round_p = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_p <= 24'b0;
		else if  (clk_en == 1'b1)   man_round_p <= man_shift_full[24:1];
	// synopsys translate_off
	initial
		man_round_p0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_p0 <= 24'b0;
		else if  (clk_en == 1'b1)   man_round_p0 <= man_round_p;
	// synopsys translate_off
	initial
		man_round_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_p1 <= 24'b0;
		else if  (clk_en == 1'b1)   man_round_p1 <= man_round_p0;
	// synopsys translate_off
	initial
		man_round_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_p2 <= 25'b0;
		else if  (clk_en == 1'b1)   man_round_p2 <= wire_man_round_adder_result;
	// synopsys translate_off
	initial
		nan_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) nan_ff <= 1'b0;
		else if  (clk_en == 1'b1)   nan_ff <= (input_is_nan_ff5 | (input_is_infinity_ff5 & (~ input_not_zero_ff5)));
	// synopsys translate_off
	initial
		overflow_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) overflow_ff <= 1'b0;
		else if  (clk_en == 1'b1)   overflow_ff <= (((exp_is_inf | input_is_infinity_ff5) & (~ input_is_nan_ff5)) & (~ (input_is_infinity_ff5 & (~ input_not_zero_ff5))));
	// synopsys translate_off
	initial
		round_dffe = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) round_dffe <= 1'b0;
		else if  (clk_en == 1'b1)   round_dffe <= round_bit;
	// synopsys translate_off
	initial
		sign_node_ff0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff0 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff0 <= (dataa[31] ^ datab[31]);
	// synopsys translate_off
	initial
		sign_node_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff1 <= sign_node_ff0[0:0];
	// synopsys translate_off
	initial
		sign_node_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff2 <= sign_node_ff1[0:0];
	// synopsys translate_off
	initial
		sign_node_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff3 <= sign_node_ff2[0:0];
	// synopsys translate_off
	initial
		sign_node_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff4 <= sign_node_ff3[0:0];
	// synopsys translate_off
	initial
		sign_node_ff5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff5 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff5 <= sign_node_ff4[0:0];
	// synopsys translate_off
	initial
		sign_node_ff6 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff6 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff6 <= sign_node_ff5[0:0];
	// synopsys translate_off
	initial
		sign_node_ff7 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff7 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff7 <= sign_node_ff6[0:0];
	// synopsys translate_off
	initial
		sign_node_ff8 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff8 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff8 <= sign_node_ff7[0:0];
	// synopsys translate_off
	initial
		sign_node_ff9 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff9 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff9 <= sign_node_ff8[0:0];
	// synopsys translate_off
	initial
		sign_node_ff10 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff10 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff10 <= sign_node_ff9[0:0];
	// synopsys translate_off
	initial
		sticky_dffe = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sticky_dffe <= 1'b0;
		else if  (clk_en == 1'b1)   sticky_dffe <= sticky_bit[22];
	// synopsys translate_off
	initial
		underflow_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) underflow_ff <= 1'b0;
		else if  (clk_en == 1'b1)   underflow_ff <= (((exp_is_zero & input_not_zero_ff5) & (~ input_is_nan_ff5)) & (~ (input_is_infinity_ff5 & (~ input_not_zero_ff5))));
	// synopsys translate_off
	initial
		zero_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) zero_ff <= 1'b0;
		else if  (clk_en == 1'b1)   zero_ff <= (((exp_is_zero | (~ input_not_zero_ff5)) & (~ input_is_nan_ff5)) & (~ input_is_infinity_ff5));
	lpm_add_sub   exp_add_adder
	(
	.aclr(aclr),
	.cin(1'b0),
	.clken(clk_en),
	.clock(clock),
	.cout(),
	.dataa({1'b0, dataa[30:23]}),
	.datab({1'b0, datab[30:23]}),
	.overflow(),
	.result(wire_exp_add_adder_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.add_sub(1'b1)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		exp_add_adder.lpm_pipeline = 1,
		exp_add_adder.lpm_width = 9,
		exp_add_adder.lpm_type = "lpm_add_sub";
	lpm_add_sub   exp_adj_adder
	(
	.cin(1'b0),
	.cout(),
	.dataa(exp_adj_p1),
	.datab({expmod[9:0]}),
	.overflow(),
	.result(wire_exp_adj_adder_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		exp_adj_adder.lpm_pipeline = 0,
		exp_adj_adder.lpm_width = 10,
		exp_adj_adder.lpm_type = "lpm_add_sub";
	lpm_add_sub   exp_bias_subtr
	(
	.cout(),
	.dataa({1'b0, exp_bias_p3}),
	.datab({bias[9:0]}),
	.overflow(),
	.result(wire_exp_bias_subtr_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.cin(),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		exp_bias_subtr.lpm_direction = "SUB",
		exp_bias_subtr.lpm_pipeline = 0,
		exp_bias_subtr.lpm_representation = "UNSIGNED",
		exp_bias_subtr.lpm_width = 10,
		exp_bias_subtr.lpm_type = "lpm_add_sub";
	lpm_add_sub   man_round_adder
	(
	.cout(),
	.dataa({1'b0, man_round_p1}),
	.datab({{24{1'b0}}, man_round_carry}),
	.overflow(),
	.result(wire_man_round_adder_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.cin(),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		man_round_adder.lpm_pipeline = 0,
		man_round_adder.lpm_width = 25,
		man_round_adder.lpm_type = "lpm_add_sub";
	lpm_mult   man_product2_mult
	(
	.aclr(aclr),
	.clken(clk_en),
	.clock(clock),
	.dataa({1'b1, dataa[22:0]}),
	.datab({1'b1, datab[22:0]}),
	.result(wire_man_product2_mult_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.sclr(1'b0),
	.sum({1{1'b0}})
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		man_product2_mult.lpm_pipeline = 5,
		man_product2_mult.lpm_representation = "UNSIGNED",
		man_product2_mult.lpm_widtha = 24,
		man_product2_mult.lpm_widthb = 24,
		man_product2_mult.lpm_widthp = 48,
		man_product2_mult.lpm_widths = 1,
		man_product2_mult.lpm_type = "lpm_mult",
		man_product2_mult.lpm_hint = "DEDICATED_MULTIPLIER_CIRCUITRY=YES";
	assign
		bias = {{3{1'b0}}, {7{1'b1}}},
		dataa_exp_all_one = {(dataa[30] & dataa_exp_all_one[6]), (dataa[29] & dataa_exp_all_one[5]), (dataa[28] & dataa_exp_all_one[4]), (dataa[27] & dataa_exp_all_one[3]), (dataa[26] & dataa_exp_all_one[2]), (dataa[25] & dataa_exp_all_one[1]), (dataa[24] & dataa_exp_all_one[0]), dataa[23]},
		dataa_exp_not_zero = {(dataa[30] | dataa_exp_not_zero[6]), (dataa[29] | dataa_exp_not_zero[5]), (dataa[28] | dataa_exp_not_zero[4]), (dataa[27] | dataa_exp_not_zero[3]), (dataa[26] | dataa_exp_not_zero[2]), (dataa[25] | dataa_exp_not_zero[1]), (dataa[24] | dataa_exp_not_zero[0]), dataa[23]},
		dataa_man_not_zero = {(dataa[22] | dataa_man_not_zero[21]), (dataa[21] | dataa_man_not_zero[20]), (dataa[20] | dataa_man_not_zero[19]), (dataa[19] | dataa_man_not_zero[18]), (dataa[18] | dataa_man_not_zero[17]), (dataa[17] | dataa_man_not_zero[16]), (dataa[16] | dataa_man_not_zero[15]), (dataa[15] | dataa_man_not_zero[14]), (dataa[14] | dataa_man_not_zero[13]), (dataa[13] | dataa_man_not_zero[12]), (dataa[12] | dataa_man_not_zero[11]), dataa[11], (dataa[10] | dataa_man_not_zero[9]), (dataa[9] | dataa_man_not_zero[8]), (dataa[8] | dataa_man_not_zero[7]), (dataa[7] | dataa_man_not_zero[6]), (dataa[6] | dataa_man_not_zero[5]), (dataa[5] | dataa_man_not_zero[4]), (dataa[4] | dataa_man_not_zero[3]), (dataa[3] | dataa_man_not_zero[2]), (dataa[2] | dataa_man_not_zero[1]), (dataa[1] | dataa_man_not_zero[0]), dataa[0]},
		datab_exp_all_one = {(datab[30] & datab_exp_all_one[6]), (datab[29] & datab_exp_all_one[5]), (datab[28] & datab_exp_all_one[4]), (datab[27] & datab_exp_all_one[3]), (datab[26] & datab_exp_all_one[2]), (datab[25] & datab_exp_all_one[1]), (datab[24] & datab_exp_all_one[0]), datab[23]},
		datab_exp_not_zero = {(datab[30] | datab_exp_not_zero[6]), (datab[29] | datab_exp_not_zero[5]), (datab[28] | datab_exp_not_zero[4]), (datab[27] | datab_exp_not_zero[3]), (datab[26] | datab_exp_not_zero[2]), (datab[25] | datab_exp_not_zero[1]), (datab[24] | datab_exp_not_zero[0]), datab[23]},
		datab_man_not_zero = {(datab[22] | datab_man_not_zero[21]), (datab[21] | datab_man_not_zero[20]), (datab[20] | datab_man_not_zero[19]), (datab[19] | datab_man_not_zero[18]), (datab[18] | datab_man_not_zero[17]), (datab[17] | datab_man_not_zero[16]), (datab[16] | datab_man_not_zero[15]), (datab[15] | datab_man_not_zero[14]), (datab[14] | datab_man_not_zero[13]), (datab[13] | datab_man_not_zero[12]), (datab[12] | datab_man_not_zero[11]), datab[11], (datab[10] | datab_man_not_zero[9]), (datab[9] | datab_man_not_zero[8]), (datab[8] | datab_man_not_zero[7]), (datab[7] | datab_man_not_zero[6]), (datab[6] | datab_man_not_zero[5]), (datab[5] | datab_man_not_zero[4]), (datab[4] | datab_man_not_zero[3]), (datab[3] | datab_man_not_zero[2]), (datab[2] | datab_man_not_zero[1]), (datab[1] | datab_man_not_zero[0]), datab[0]},
		exp_is_inf = (((~ exp_adj_p2[9]) & exp_adj_p2[8]) | ((~ exp_adj_p2[8]) & result_exp_all_one[7])),
		exp_is_zero = (exp_adj_p2[9] | (~ result_exp_not_zero[8])),
		expmod = {{8{1'b0}}, (delay_man_product_msb2 & man_round_p2[24]), (delay_man_product_msb2 ^ man_round_p2[24])},
		inf_num = {8{1'b1}},
		lsb_bit = man_shift_full[1],
		man_shift_full = ((wire_man_product2_mult_result[46:22] & {25{(~ wire_man_product2_mult_result[47])}}) | (wire_man_product2_mult_result[47:23] & {25{wire_man_product2_mult_result[47]}})),
		nan = nan_ff,
		overflow = overflow_ff,
		result = {sign_node_ff10[0:0], exp_result_ff[7:0], man_result_ff[22:0]},
		result_exp_all_one = {(result_exp_all_one[6] & exp_adj_p2[7]), (result_exp_all_one[5] & exp_adj_p2[6]), (result_exp_all_one[4] & exp_adj_p2[5]), (result_exp_all_one[3] & exp_adj_p2[4]), (result_exp_all_one[2] & exp_adj_p2[3]), (result_exp_all_one[1] & exp_adj_p2[2]), (result_exp_all_one[0] & exp_adj_p2[1]), exp_adj_p2[0]},
		result_exp_not_zero = {(result_exp_not_zero[7] | exp_adj_p2[8]), (result_exp_not_zero[6] | exp_adj_p2[7]), (result_exp_not_zero[5] | exp_adj_p2[6]), (result_exp_not_zero[4] | exp_adj_p2[5]), (result_exp_not_zero[3] | exp_adj_p2[4]), (result_exp_not_zero[2] | exp_adj_p2[3]), (result_exp_not_zero[1] | exp_adj_p2[2]), (result_exp_not_zero[0] | exp_adj_p2[1]), exp_adj_p2[0]},
		round_bit = man_shift_full[0],
		round_carry = (round_dffe & (lsb_dffe | sticky_dffe)),
		sticky_bit = {(sticky_bit[21] | (wire_man_product2_mult_result[47] & wire_man_product2_mult_result[22])), (sticky_bit[20] | wire_man_product2_mult_result[21]), (sticky_bit[19] | wire_man_product2_mult_result[20]), (sticky_bit[18] | wire_man_product2_mult_result[19]), (sticky_bit[17] | wire_man_product2_mult_result[18]), (sticky_bit[16] | wire_man_product2_mult_result[17]), (sticky_bit[15] | wire_man_product2_mult_result[16]), (sticky_bit[14] | wire_man_product2_mult_result[15]), (sticky_bit[13] | wire_man_product2_mult_result[14]), (sticky_bit[12] | wire_man_product2_mult_result[13]), (sticky_bit[11] | wire_man_product2_mult_result[12]), (sticky_bit[10] | wire_man_product2_mult_result[11]), (sticky_bit[9] | wire_man_product2_mult_result[10]), (sticky_bit[8] | wire_man_product2_mult_result[9]), (sticky_bit[7] | wire_man_product2_mult_result[8]), (sticky_bit[6] | wire_man_product2_mult_result[7]), (sticky_bit[5] | wire_man_product2_mult_result[6]), (sticky_bit[4] | wire_man_product2_mult_result[5]), (sticky_bit[3] | wire_man_product2_mult_result[4]), (sticky_bit[2] | wire_man_product2_mult_result[3]), (sticky_bit[1] | wire_man_product2_mult_result[2]), (sticky_bit[0] | wire_man_product2_mult_result[1]), wire_man_product2_mult_result[0]},
		underflow = underflow_ff,
		zero = zero_ff;
endmodule //fp_mult_altfp_mult_her
//VALID FILE


// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module fp_mult (
	aclr,
	clk_en,
	clock,
	dataa,
	datab,
	nan,
	overflow,
	result,
	underflow,
	zero);

	input	  aclr;
	input	  clk_en;
	input	  clock;
	input	[31:0]  dataa;
	input	[31:0]  datab;
	output	  nan;
	output	  overflow;
	output	[31:0]  result;
	output	  underflow;
	output	  zero;

	wire  sub_wire0;
	wire  sub_wire1;
	wire [31:0] sub_wire2;
	wire  sub_wire3;
	wire  sub_wire4;
	wire  nan = sub_wire0;
	wire  overflow = sub_wire1;
	wire [31:0] result = sub_wire2[31:0];
	wire  underflow = sub_wire3;
	wire  zero = sub_wire4;

	fp_mult_altfp_mult_her	fp_mult_altfp_mult_her_component (
				.aclr (aclr),
				.clk_en (clk_en),
				.clock (clock),
				.dataa (dataa),
				.datab (datab),
				.nan (sub_wire0),
				.overflow (sub_wire1),
				.result (sub_wire2),
				.underflow (sub_wire3),
				.zero (sub_wire4));

endmodule

// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: LIBRARY: altera_mf altera_mf.altera_mf_components.all
// Retrieval info: PRIVATE: FPM_FORMAT STRING "Single"
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "Cyclone V"
// Retrieval info: CONSTANT: DEDICATED_MULTIPLIER_CIRCUITRY STRING "YES"
// Retrieval info: CONSTANT: DENORMAL_SUPPORT STRING "NO"
// Retrieval info: CONSTANT: EXCEPTION_HANDLING STRING "NO"
// Retrieval info: CONSTANT: INTENDED_DEVICE_FAMILY STRING "UNUSED"
// Retrieval info: CONSTANT: LPM_HINT STRING "UNUSED"
// Retrieval info: CONSTANT: LPM_TYPE STRING "altfp_mult"
// Retrieval info: CONSTANT: PIPELINE NUMERIC "11"
// Retrieval info: CONSTANT: REDUCED_FUNCTIONALITY STRING "NO"
// Retrieval info: CONSTANT: ROUNDING STRING "TO_NEAREST"
// Retrieval info: CONSTANT: WIDTH_EXP NUMERIC "8"
// Retrieval info: CONSTANT: WIDTH_MAN NUMERIC "23"
// Retrieval info: USED_PORT: aclr 0 0 0 0 INPUT NODEFVAL "aclr"
// Retrieval info: CONNECT: @aclr 0 0 0 0 aclr 0 0 0 0
// Retrieval info: USED_PORT: clk_en 0 0 0 0 INPUT NODEFVAL "clk_en"
// Retrieval info: CONNECT: @clk_en 0 0 0 0 clk_en 0 0 0 0
// Retrieval info: USED_PORT: clock 0 0 0 0 INPUT NODEFVAL "clock"
// Retrieval info: CONNECT: @clock 0 0 0 0 clock 0 0 0 0
// Retrieval info: USED_PORT: dataa 0 0 32 0 INPUT NODEFVAL "dataa[31..0]"
// Retrieval info: CONNECT: @dataa 0 0 32 0 dataa 0 0 32 0
// Retrieval info: USED_PORT: datab 0 0 32 0 INPUT NODEFVAL "datab[31..0]"
// Retrieval info: CONNECT: @datab 0 0 32 0 datab 0 0 32 0
// Retrieval info: USED_PORT: nan 0 0 0 0 OUTPUT NODEFVAL "nan"
// Retrieval info: CONNECT: nan 0 0 0 0 @nan 0 0 0 0
// Retrieval info: USED_PORT: overflow 0 0 0 0 OUTPUT NODEFVAL "overflow"
// Retrieval info: CONNECT: overflow 0 0 0 0 @overflow 0 0 0 0
// Retrieval info: USED_PORT: result 0 0 32 0 OUTPUT NODEFVAL "result[31..0]"
// Retrieval info: CONNECT: result 0 0 32 0 @result 0 0 32 0
// Retrieval info: USED_PORT: underflow 0 0 0 0 OUTPUT NODEFVAL "underflow"
// Retrieval info: CONNECT: underflow 0 0 0 0 @underflow 0 0 0 0
// Retrieval info: USED_PORT: zero 0 0 0 0 OUTPUT NODEFVAL "zero"
// Retrieval info: CONNECT: zero 0 0 0 0 @zero 0 0 0 0
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult.v TRUE FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult.qip TRUE FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult.bsf TRUE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult_inst.v TRUE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult_bb.v TRUE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult.inc TRUE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult.cmp TRUE TRUE
// Retrieval info: LIB_FILE: lpm
