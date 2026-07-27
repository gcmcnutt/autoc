// AT-SPEED self-test + deploy: PLL the 12 MHz board clock (C1) up to 108 MHz (fast_pll, production-style),
// clock the carry-save 32x32 multiplier at 108 MHz, drive LFSR operand pairs (one per hold-window so a
// combinational golden a*b reference settles), compare, count mismatches. User logic (heartbeat) runs off
// divided counters of the fast clock. Visual:
//   LEDs[0] heartbeat (fast clk alive)  LEDs[1] PLL lock  LEDs[7] STICKY error  LEDs[6:2] error count
// PASS @108 MHz => LEDs[0] blinks, LEDs[1] on, LEDs[7] off, count=0.  Timing failure => LEDs[7] on + count.

module csa_mul #(parameter W = 32) (input clk, input [W-1:0] a, input [W-1:0] b, output [2*W-1:0] p);
  localparam PW = 2*W;
  reg [W-1:0] ar = 0, br = 0;
  always @(posedge clk) begin ar <= a; br <= b; end
  wire [PW-1:0] S [0:W];
  wire [PW-1:0] C [0:W];
  assign S[0] = {PW{1'b0}};
  assign C[0] = {PW{1'b0}};
  genvar k;
  generate for (k = 0; k < W; k = k + 1) begin : ROW
    wire [PW-1:0] pp = br[k] ? ({{W{1'b0}}, ar} << k) : {PW{1'b0}};
    wire [PW-1:0] s_ = S[k] ^ C[k] ^ pp;
    wire [PW-1:0] cm = (S[k] & C[k]) | (C[k] & pp) | (S[k] & pp);
    reg [PW-1:0] Sr, Cr;
    always @(posedge clk) begin Sr <= s_; Cr <= (cm << 1); end
    assign S[k+1] = Sr;
    assign C[k+1] = Cr;
  end endgenerate
  reg [PW-1:0] prod;
  always @(posedge clk) prod <= S[W] + C[W];
  assign p = prod;
endmodule

module selftest (input clk12, output [7:0] LEDs);
  wire clk, lock;
  fast_pll pll (.CLKI(clk12), .CLKOP(clk), .LOCK(lock));   // 12 MHz → 108 MHz

  localparam LAT = 40;
  reg [31:0] lfsr_a = 32'hACE1_2345, lfsr_b = 32'h1234_ABCD;
  reg [31:0] a = 32'd3, b = 32'd5;
  reg [5:0]  hold = 0;
  reg [63:0] ref_r = 0;
  reg        err_sticky = 0;
  reg [23:0] err_count = 0;
  reg [31:0] hb = 0;

  wire [63:0] prod;
  csa_mul #(.W(32)) M (.clk(clk), .a(a), .b(b), .p(prod));

  always @(posedge clk) begin
    hb    <= hb + 1'b1;          // free-running off the 108 MHz clock (divided → LED heartbeat)
    ref_r <= a * b;
    if (!lock) begin             // hold the test in reset until the PLL is locked
      hold <= 0; err_sticky <= 1'b0; err_count <= 0;
    end else if (hold == LAT-1) begin
      if (prod != ref_r) begin err_sticky <= 1'b1; err_count <= err_count + 1'b1; end
      lfsr_a <= {lfsr_a[30:0], lfsr_a[31]^lfsr_a[21]^lfsr_a[1]^lfsr_a[0]};
      lfsr_b <= {lfsr_b[30:0], lfsr_b[31]^lfsr_b[21]^lfsr_b[1]^lfsr_b[0]};
      a <= lfsr_a; b <= lfsr_b;
      hold <= 0;
    end else hold <= hold + 1'b1;
  end

  // Polarity-robust display (the board LEDs are active-low → encode status as MOTION vs BLINK, not levels):
  //   locked + clean  → 8-LED binary RIPPLE (clearly "running at 108 MHz")
  //   error (sticky)  → all 8 blink together ~6 Hz (unmistakable failure, distinct from the ripple)
  //   not locked      → all off
  reg [7:0] disp = 8'h00;
  always @(posedge clk) begin
    if (!lock)           disp <= 8'h00;
    else if (err_sticky) disp <= {8{hb[23]}};   // all together ~6 Hz = ERROR
    else                 disp <= hb[27:20];      // ripple = clean / running
  end
  assign LEDs = disp;
endmodule
