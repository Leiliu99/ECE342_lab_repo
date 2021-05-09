//MARK=18
//TESTCASES=9
module tb();
    logic  clk;
    logic  reset;
    logic  enter;
    logic  [7:0] guess;
    logic  [7:0] actual;
    logic  [7:0] actual_top;
    logic  dp_over;
    logic  dp_under;
    logic  dp_equal;

    logic win;
  
    int time_;
	int num_passed;

    top DUT(.clk, .reset, .enter, .guess, .actual, .dp_over, .dp_under, .dp_equal);

    always
		#0.5 clk = !clk;
    
    initial begin
        clk = 0;
		num_passed = 0;
        reset = 0;
        enter = 0;
        guess = 0;
        actual_top = 0;
        #5.2;
        reset = 1;
        $display("Reset asserts here");
        #5;
        reset = 0;
        time_ = 182;
        guess = 192;
        #time_;
        enter = 1;
        #2;
        enter = 0;
        #10;
        actual_top = actual;
        $display("First guess is entered, actual number = %d", actual);
        #10;
        $display("10 cycles later, check actual number doesn't change after entering the guess");
        $display("Actual number = %d  Your actual = %d", actual_top, actual);
        if(actual_top != actual) begin
          $display("Check fail");
        end else begin
		  num_passed ++;
          $display("Check pass");
        end
        $display("Check guess for over");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        if(dp_over == 1 && dp_under  == 0 && dp_equal == 0) begin
          num_passed ++;
          $display("Check pass");
        end else begin
          $display("Check fail");
        end
        #10;
        guess = actual - 10;
        enter = 1;
        #2;
        enter = 0;
        #5;
        $display("Second guess is entered, check guess for under");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        if(dp_over == 0 && dp_under  == 1 && dp_equal == 0) begin
          num_passed ++;
          $display("Check pass");
        end else begin
          $display("Check fail");
        end
        #10;
        guess = actual;
        enter = 1;
        #2;
        enter = 0;
        #5;
        $display("Third guess is entered, check guess for equal");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        if(dp_over == 0 && dp_under  == 0 && dp_equal == 1) begin
          num_passed ++;
          $display("Check pass");
        end else begin
          $display("Check fail");
        end
        #10;
        guess = actual - 5;
        enter = 1;
        #2;
        enter = 0;
        #5;
        $display("Forth guess is entered, check guess doesn't change after the game ends");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        if(dp_over == 0 && dp_under  == 0 && dp_equal == 1) begin
          num_passed ++;
          $display("Check pass");
        end else begin
          $display("Check fail");
        end
        #5.2;
        reset = 1;
        $display("Reset asserts here");
        #5;
        reset = 0;
        time_ = 18569;
        guess = 18569 % 256 - 50;
        #time_;
        enter = 1;
        #2;
        enter = 0;
        #10;
        actual_top = actual;
        $display("First guess is entered, actual number = %d", actual);
        $display("Check guess for over");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        if(dp_over == 1 && dp_under  == 0 && dp_equal == 0) begin
          num_passed ++;
          $display("Check pass");
        end else begin
          $display("Check fail");
        end
        #10;
        guess = 18569 % 256 - 51;
        enter = 1;
        #2;
        enter = 0;
        #5;
        $display("Second guess is entered, check guess for over");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        if(dp_over == 1 && dp_under  == 0 && dp_equal == 0) begin
          num_passed ++;
          $display("Check pass");
        end else begin
          $display("Check fail");
        end
        #10;
        guess = actual;
        enter = 1;
        #2;
        enter = 0;
        #5;
        $display("Third guess is entered, check guess for equal");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        if(dp_over == 0 && dp_under  == 0 && dp_equal == 1) begin
          num_passed ++;
          $display("Check pass");
        end else begin
          $display("Check fail");
        end
        #10;
        guess = actual - 5;
        enter = 1;
        #2;
        enter = 0;
        #5;
        $display("Forth guess is entered, check guess doesn't change after the game ends");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        if(dp_over == 0 && dp_under  == 0 && dp_equal == 1) begin
          num_passed ++;
          $display("Check pass");
        end else begin
          $display("Check fail");
        end
		$display("Test case passed: %d/9", num_passed);
		$stop();
        //$finish();
    end
endmodule