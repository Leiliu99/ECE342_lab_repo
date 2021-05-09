//MARK=24
//TESTCASES=8
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
    logic [3:0] dp_tries;

    logic win;
    logic under;
    logic over;
    logic equal;
  
    int time_;
	int num_passed;

    top DUT(.clk, .reset, .enter, .guess, .actual, .dp_over, .dp_under, .dp_equal, .dp_tries);

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
        #2;
        $display("Check if trying number starts from 7");
        $display("Your tring number = %d, actual tring number = 7", dp_tries);
        if(dp_tries == 7) begin
          num_passed ++;
          $display("Check pass");
        end else begin
          $display("Check fail");
        end
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
        $display("Check guess for over");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        $display("Your tring number = %d, actual tring number = 6", dp_tries);
        if(dp_over == 1 && dp_under  == 0 && dp_equal == 0 && dp_tries == 6) begin
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
        $display("Second guess is entered, check guess for equal");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        $display("Your tring number = %d, actual tring number = 5", dp_tries);
        if(dp_over == 0 && dp_under  == 0 && dp_equal == 1 && dp_tries == 5) begin
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
        $display("Third guess is entered, check guess doesn't change after the game ends");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        $display("Your tring number = %d, actual tring number = 5", dp_tries);
        if(dp_over == 0 && dp_under  == 0 && dp_equal == 1 && dp_tries == 5) begin
          num_passed ++;
          $display("Check pass");
        end else begin
          $display("Check fail");
        end
        #5.2;
        reset = 1;
        $display("Reset asserts here");
        $display("Check game ends after 7 tries");
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
        $display("guess is %d, your actual is %d", guess, actual);
        #10;
        guess = 18569 % 256 - 51;
        enter = 1;
        #2;
        enter = 0;
        #5;
        $display("Second guess is entered");
        $display("guess is %d, your actual is %d", guess, actual);
        #10;
        guess = actual - 10;
        enter = 1;
        #2;
        enter = 0;
        #5;
        $display("Third guess is entered");
        $display("guess is %d, your actual is %d", guess, actual);
        #10;
        guess = actual - 5;
        enter = 1;
        #2;
        enter = 0;
        #5;
        $display("Forth guess is entered");
        $display("guess is %d, your actual is %d", guess, actual);
        #10;
        guess = actual - 4;
        enter = 1;
        #2;
        enter = 0;
        #5;
        $display("Fifth guess is entered");
        $display("guess is %d, your actual is %d", guess, actual);
        #10;
        guess = actual - 3;
        enter = 1;
        #2;
        enter = 0;
        #5;
        $display("Sixth guess is entered");
        $display("guess is %d, your actual is %d", guess, actual);
        #10;
        guess = actual - 2;
        enter = 1;
        #2;
        enter = 0;
        #5;
        $display("Seventh guess is entered, game ends if the guess is wrong");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Game ends, Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        $display("Your tring number = %d, actual tring number = 0", dp_tries);
        $display("All results should not change after entering new guess");
        under = dp_under;
        over = dp_over;
        equal = dp_equal;
        #10;
        guess = actual - 1;
        enter = 1;
        #2;
        enter = 0;
        #5;
        $display("New guess is entered, check games ends");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        $display("Your tring number = %d, actual tring number = 0", dp_tries);
        if(dp_over == over && dp_under  == under && dp_equal == equal && dp_tries == 0) begin
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
        $display("New guess is entered, check games ends");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        $display("Your tring number = %d, actual tring number = 0", dp_tries);
        if(dp_over == over && dp_under  == under && dp_equal == equal && dp_tries == 0) begin
          num_passed ++;
          $display("Check pass");
        end else begin
          $display("Check fail");
        end
        #10;
        guess = actual + 1;
        enter = 1;
        #2;
        enter = 0;
        #5;
        $display("New guess is entered, check games ends");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        $display("Your tring number = %d, actual tring number = 0", dp_tries);
        if(dp_over == over && dp_under  == under && dp_equal == equal && dp_tries == 0) begin
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
        $display("New guess is entered, check games ends");
        $display("guess is %d, your actual is %d", guess, actual);
        $display("Your over = %d  Your under = %d  Your equal = %d", dp_over, dp_under, dp_equal);
        $display("Your tring number = %d, actual tring number = 0", dp_tries);
        if(dp_over == over && dp_under  == under && dp_equal == equal && dp_tries == 0) begin
          num_passed ++;
          $display("Check pass");
        end else begin
          $display("Check fail");
        end
		$display("Test case passed: %d/8", num_passed);
		$stop();
        //$finish();
    end
endmodule