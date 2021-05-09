`timescale 1 ps / 1 ps
module tb
();

logic clk;
logic reset;

always begin
	#5 clk = ~clk;
end

initial begin
	clk = 0;
	reset = 1;
	#20
	reset = 0;

	uut.mem_inst.ram[8189] = 32'h40500000;
	uut.mem_inst.ram[8190] = 32'h4101EB85;
	#10000
	if (uut.mem_inst.ram[8191] == 32'h41D31EB8) begin
		$display("Multiplication correct, PASS.");
	end else begin
		$display("Multiplication incorrect, FAIL.");
	end

	uut.mem_inst.ram[8189] = 32'h41D31EB8;
	uut.mem_inst.ram[8190] = 32'h40B1999A;
	#10000
	if (uut.mem_inst.ram[8191] == 32'h431276EA) begin
		$display("Multiplication correct, PASS.");
	end else begin
		$display("Multiplication incorrect, FAIL.");
	end

	uut.mem_inst.ram[8189] = 32'h4C79AED8;
	uut.mem_inst.ram[8190] = 32'h5BED1234;
	#10000
	if (uut.mem_inst.ram[8191] == 32'h68E7389F) begin
		$display("Multiplication correct, PASS.");
	end else begin
		$display("Multiplication incorrect, FAIL.");
	end

	uut.mem_inst.ram[8189] = 32'h0C123456;
	uut.mem_inst.ram[8190] = 32'h000007D0;
	#10000
	if (uut.mem_inst.ram[8191] == 32'h00000000) begin
		$display("Multiplication correct, PASS.");
	end else begin
		$display("Multiplication incorrect, FAIL.");
	end

	uut.mem_inst.ram[8189] = 32'hAA137D12;
	uut.mem_inst.ram[8190] = 32'hBBBBBBBB;
	#10000
	if (uut.mem_inst.ram[8191] == 32'h26585108) begin
		$display("Multiplication correct, PASS.");
	end else begin
		$display("Multiplication incorrect, FAIL.");
	end

	uut.mem_inst.ram[8189] = 32'h13579BDF;
	uut.mem_inst.ram[8190] = 32'hECA86420;
	#10000
	if (uut.mem_inst.ram[8191] == 32'hC08DD29E) begin
		$display("Multiplication correct, PASS.");
	end else begin
		$display("Multiplication incorrect, FAIL.");
	end

	uut.mem_inst.ram[8189] = 32'h528B690A;
	uut.mem_inst.ram[8190] = 32'h45E8BF3F;
	#10000
	if (uut.mem_inst.ram[8191] == 32'h58FD7EAD) begin
		$display("Multiplication correct, PASS.");
	end else begin
		$display("Multiplication incorrect, FAIL.");
	end

	uut.mem_inst.ram[8189] = 32'hBBBBBBBB;
	uut.mem_inst.ram[8190] = 32'hEEFFAA44;
	#10000
	if (uut.mem_inst.ram[8191] == 32'h6B3B7CDC) begin
		$display("Multiplication correct, PASS.");
	end else begin
		$display("Multiplication incorrect, FAIL.");
	end
	$finish;
end

part3 uut
(
	.clk(clk),
    .reset(reset)
);

endmodule
