// F1 visual toolchain-proof: a free-running counter ripples the 8 user LEDs — unmistakably
// "programmed + running" regardless of LED polarity. NOT a beacon design; the correlator RTL replaces it.
module blink (input clk, output [7:0] LEDs);
  reg [31:0] cnt = 32'd0;
  always @(posedge clk) cnt <= cnt + 1'b1;
  // upper counter bits → a visible binary ripple at 12 MHz: LED0 ~5.7 Hz ... LED7 ~0.045 Hz
  assign LEDs = cnt[27:20];
endmodule
