// Fabric speed CEILING: a deep pipeline where each stage is ~1 LUT level with NO carry chain
// (rotate-left-1 XOR self = pure XOR). Its Fmax = the fastest this MachXO2 fabric clocks a well-pipelined
// design (FF + 1 LUT + routing) — the target a properly carry-save-pipelined multiplier could approach,
// and the contrast to the ~30 MHz carry-propagation-bound combinational multiply.
module top (input clk, output led);
  localparam N = 40, W = 8;
  reg [W-1:0] s [0:N];
  integer i;
  always @(posedge clk) begin
    s[0] <= s[0] + 8'd1;                                  // churn the head (no long carry: 8-bit)
    for (i = 0; i < N; i = i + 1)
      s[i+1] <= {s[i][W-2:0], s[i][W-1]} ^ s[i];          // rotate1 ^ self : 1 LUT level, no carry chain
  end
  assign led = ^s[N];
endmodule
