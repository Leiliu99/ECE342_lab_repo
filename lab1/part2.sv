//Top module for game
module top 
(
    // Clock pins
    input                     clk,
    input                     reset,
    input                     enter,
    input               [7:0] guess,
    output              [7:0] actual,
    output                    dp_over,
    output                    dp_under,
    output                    dp_equal,
	 output              [3:0] dp_tries
);

	logic over;
	logic under;
	logic equal;
	//logic tries;
	//assign tries = 4'd7;

	// Datapath
	logic dp_inc_actual;
	datapath the_datapath
	(
		.clk(clk),
		.reset(reset),
		.i_guess(guess),
		.i_inc_actual(dp_inc_actual),
		.o_over(over),
		.o_under(under),
		.o_equal(equal),
      .actual(actual)
//		.i_tries(tries),
//		.o_tries(dp_tries)  
	);
	
	// State Machine
	logic ctrl_update_leds;
	control the_control
	(
		.clk(clk),
		.reset(reset),
		.i_enter(enter),
		.o_inc_actual(dp_inc_actual),
		.i_over(over),
		.i_under(under),
		.i_equal(equal),
		.o_update_leds(ctrl_update_leds),
		.o_tries(dp_tries)
//		.i_tries(dp_tries)
	);
	
	// LED controllers
	led_ctrl ledc_under(clk, reset, under, ctrl_update_leds, dp_under);
	led_ctrl ledc_over(clk, reset, over, ctrl_update_leds, dp_over);
	led_ctrl ledc_equal(clk, reset, equal, ctrl_update_leds, dp_equal);
	
endmodule

/*******************************************************/
/********************Control module********************/
/*****************************************************/
module control
(
	input clk,
	input reset,
	
	// Button input
	input i_enter,
	
	// Datapath
	output logic o_inc_actual,
	input i_over,
	input i_under,
	input i_equal,
	
	// LED Control
	output logic o_update_leds,
	
	//initial tries as ouput
	output logic[3:0]o_tries
//	//remaining tries as input
//	input [3:0]i_tries
);

logic [3:0]tries;
// Declare two objects, 'state' and 'nextstate'
// that are of enum type.
enum int unsigned
{
	S_GEN_RAND,
	S_CHECK,
	S_WAIT_NOENTER,
	S_WAIT_ENTER,
	S_END
} state, nextstate;

// Clocked always block for making state registers
always_ff @ (posedge clk or posedge reset) begin
	if (reset) state <= S_GEN_RAND;
	else state <= nextstate;
end

// always_comb replaces always @* and gives compile-time errors instead of warnings
// if you accidentally infer a latch
always_comb begin
	nextstate = state;
	o_inc_actual = 1'b0;
	o_update_leds = 1'b0;
	
	case (state)
		// TODO: complete this
		S_GEN_RAND: begin
			o_inc_actual = 1'b1; //enable incrementing counter
			nextstate = i_enter ? S_WAIT_NOENTER : S_GEN_RAND;	
			tries = 4'd7;
			o_tries = tries;
		end
		
		S_WAIT_NOENTER: begin
			nextstate = i_enter ? S_WAIT_NOENTER : S_CHECK;
		end
		
		S_WAIT_ENTER: begin
			nextstate = i_enter ? S_WAIT_NOENTER : S_WAIT_ENTER;	
		end
			
		S_CHECK: begin
			tries = tries - 4'd1;
			o_tries = tries;
			if (tries == 4'd0) nextstate = S_END;
			else if(i_equal) nextstate = S_END;
			else nextstate = S_WAIT_ENTER; //wait for enter signal
			o_update_leds = 1'b1; //generate ouputs signal high
		end
		
		S_END:begin
			nextstate = S_END;//start generating new random 
		end
		
	endcase
end
endmodule

/*******************************************************/
/********************Datapath module*******************/
/*****************************************************/
module datapath
(
	input clk,
	input reset,
	
	// Number entry
	input [7:0] i_guess,
	
	// Increment actual
	input i_inc_actual,
	
	// Comparison result
	output o_over,
	output o_under,
	output o_equal,
	output logic [7:0] actual
//	
//	input [3:0]i_tries,
//	output [3:0]o_tries
);

// Update the 'actual' register based on control signals
always_ff @ (posedge clk or posedge reset) begin
	if (reset) actual <= '0;
	else begin
		if (i_inc_actual) actual <= actual + 8'd1;
	end
end

// Generate comparisons
assign o_over = i_guess > actual;
assign o_equal = i_guess == actual;
assign o_under = i_guess < actual;
//deduct tries by 1
//assign o_tries = i_tries - 7'd1;
endmodule

/*******************************************************/
/********************LED control module****************/
/*****************************************************/
module led_ctrl
(
	input clk,
	input reset,
	
	input i_val,
	input i_enable,
	output logic o_out
);

always_ff @ (posedge clk or posedge reset) begin
	if (reset) o_out <= '0;
	else if (i_enable) o_out <= i_val;
end

endmodule
