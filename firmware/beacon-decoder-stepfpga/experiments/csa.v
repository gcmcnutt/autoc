// Structural CARRY-SAVE pipelined unsigned WxW→2W multiplier — the deterministic way to beat the ~30 MHz
// ripple ceiling. Partial products are reduced by a chain of 3:2 carry-save adders: each bit is one full
// adder (s=x^y^z, carry=maj(x,y,z)), ~1-2 LUT levels, NO carry propagation. A pipeline register every GROUP
// rows trades latency/FF for Fmax. The single carry-propagate add at the very end (S+C) is the one ripple.
// Goal: prove the CSA block clocks at ~100-150 MHz (the fabric ceiling), and see how much of the part one
// WxW multiply costs (the "2-4 correlators adds up fast" question).
module csa_mul #(parameter W = 32, parameter GROUP = 1) (input clk, input [W-1:0] a, input [W-1:0] b, output [2*W-1:0] p);
  localparam PW = 2*W;
  reg [W-1:0] ar = 0, br = 0;
  always @(posedge clk) begin ar <= a; br <= b; end      // registered inputs

  wire [PW-1:0] S [0:W];
  wire [PW-1:0] C [0:W];
  assign S[0] = {PW{1'b0}};
  assign C[0] = {PW{1'b0}};

  genvar k;
  generate
    for (k = 0; k < W; k = k + 1) begin : ROW
      wire [PW-1:0] pp = br[k] ? ({{W{1'b0}}, ar} << k) : {PW{1'b0}};
      wire [PW-1:0] s_ = S[k] ^ C[k] ^ pp;                          // 3:2 CSA sum
      wire [PW-1:0] cm = (S[k] & C[k]) | (C[k] & pp) | (S[k] & pp); // 3:2 CSA carry (majority)
      wire [PW-1:0] c_ = cm << 1;
      if (((k + 1) % GROUP) == 0 || k == W - 1) begin : PIPE        // register S,C every GROUP rows
        reg [PW-1:0] Sr, Cr;
        always @(posedge clk) begin Sr <= s_; Cr <= c_; end
        assign S[k+1] = Sr;
        assign C[k+1] = Cr;
      end else begin : THRU
        assign S[k+1] = s_;
        assign C[k+1] = c_;
      end
    end
  endgenerate

  reg [PW-1:0] prod;
  always @(posedge clk) prod <= S[W] + C[W];                        // final carry-propagate add (the one ripple)
  assign p = prod;
endmodule

module top (input clk, output led);
  localparam W = 32;        // <-- swept by run_csa.sh
  localparam GROUP = 1;     // <-- swept by run_csa.sh
  reg [W-1:0] a = 1, b = 1;
  wire [2*W-1:0] pr;
  always @(posedge clk) begin a <= a + 1'b1; b <= b + 2'd3; end     // keep operands live
  csa_mul #(.W(W), .GROUP(GROUP)) M (.clk(clk), .a(a), .b(b), .p(pr));
  // PIPELINED output reduction (64→32→8→1). A single `^pr` made the wide-XOR ROUTING the critical path
  // (77% route, masking the multiplier's true Fmax) — pipeline it so we measure the DUT, not the harness.
  reg [31:0] r1 = 0; reg [7:0] r2 = 0; reg led_r = 1'b0;
  always @(posedge clk) begin
    r1 <= pr[63:32] ^ pr[31:0];
    r2 <= r1[31:24] ^ r1[23:16] ^ r1[15:8] ^ r1[7:0];
    led_r <= ^r2;
  end
  assign led = led_r;
endmodule
