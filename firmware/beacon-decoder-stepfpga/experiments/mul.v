// How fast can a TRUE 32x32 multiplier run on this (multiplier-less) MachXO2 if heavily pipelined?
// pmul registers the inputs, computes a*b, then pushes it through STAGES output pipeline registers.
// With Synplify retiming ON (mul_build.tcl), the multiply logic is pulled BACKWARD across those registers,
// distributing it into ~STAGES levels-of-1-3 stages → Fmax rises with STAGES. STAGES is swept by run_mul.sh.
module pmul #(parameter STAGES = 8) (input clk, input [31:0] a, input [31:0] b, output [63:0] p);
  reg [31:0] ar = 0, br = 0;
  reg [63:0] pipe [0:STAGES-1];
  integer i;
  always @(posedge clk) begin
    ar <= a; br <= b;
    pipe[0] <= ar * br;                                   // retiming spreads this across pipe[0..STAGES-1]
    for (i = 1; i < STAGES; i = i + 1) pipe[i] <= pipe[i-1];
  end
  assign p = pipe[STAGES-1];
endmodule

module top (input clk, output led);
  localparam STAGES = 8;                                  // <-- swept by run_mul.sh (sed)
  reg [31:0] a = 32'h0000_0001, b = 32'h0000_0001;
  wire [63:0] p;
  pmul #(.STAGES(STAGES)) M (.clk(clk), .a(a), .b(b), .p(p));
  always @(posedge clk) begin a <= a + 32'h1; b <= b + 32'h3; end    // keep operands live (no const-fold)
  reg led_r = 1'b0;
  always @(posedge clk) led_r <= ^p;                      // XOR-reduce → forces the whole pipeline kept
  assign led = led_r;
endmodule
