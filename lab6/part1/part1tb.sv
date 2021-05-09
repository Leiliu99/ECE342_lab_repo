module tb
();

logic clk;
logic reset;
logic [7:0] sw;
logic [7:0] led;

always begin
	#5 clk = ~clk;
end

initial begin
	clk = 0;
	reset = 1;
	#20
	reset = 0;
	sw = 8'b10101111;
	#500
	if (led == sw) begin
		$display("LEDs match the switches, PASS.");
	end else begin
		$display("LEDs don't match the switches, FAIL.");
	end
	sw = 8'b00110101;
	#500
	if (led == sw) begin
		$display("LEDs match the switches, PASS.");
	end else begin
		$display("LEDs don't match the switches, FAIL.");
	end
	$finish;
end

part1 uut
(
	.clk(clk),
    .reset(reset),
    .LEDR(led),
    .SW(sw)
);

endmodule
