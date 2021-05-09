`timescale 1ns/1ns
module tb();
  logic [7:0] dut_x, dut_y;
  logic [15:0] dut_out_lzh, dut_out;
    logic [15:0] dut_pp_lzh [4];
  logic [15:0] dut_pp [4];
    
  wallace_mult_lzh DUT1
  (
    .x(dut_x),
    .y(dut_y),
        .out(dut_out_lzh),
        .pp(dut_pp_lzh)
  );

  wallace_mult DUT2
  (
    .x(dut_x),
    .y(dut_y),
    .out(dut_out),
    .pp(dut_pp)
  );
  
  initial begin
      
        // functionality checking
        for (integer x = 0; x < 256; x++) begin
            for (integer y = 0; y < 256; y++) begin
        
        
        dut_x = x[7:0];
        dut_y = y[7:0];
        
        #5;
              
        //$display("Partial Product Checking: 0", dut_partial_product[0]);
        //$display("Partial Product Checking: 1", dut_partial_product[1]);
        //$display("Partial Product Checking: 2", dut_partial_product[2]);
        //$display("Partial Product Checking: 3", dut_partial_product[3]);
        //$display("Partial Product Checking: 4", dut_partial_product[4]);
        //$display("Partial Product Checking: 5", dut_partial_product[5]);
        //$display("Partial Product Checking: 6", dut_partial_product[6]);
        //$display("Partial Product Checking: 7", dut_partial_product[7]);
        //$display("Partial Product Checking: 8", dut_partial_product[8]);
              


            if (dut_out_lzh !== dut_out) begin
                    $display("Functionality Checking: Mismatch %0d * %0d = %0d, expected %0d", x, y, dut_out, dut_out_lzh);
          $stop();
            end
        
        
        for (integer i = 0; i < 4; i++) begin
      
          if (dut_pp_lzh[i] !== dut_pp[i]) begin
            $display("Functionality Checking: Mismatch %0d * %0d = %0d, expected %0d", x, y, dut_out, dut_out_lzh);
            $stop();
          end
        
        end

      end
        end
        $display("Checking Passed");
    $stop();
  end

  

endmodule
