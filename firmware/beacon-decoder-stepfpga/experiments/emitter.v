// S1 — synthetic Gold-code emitter (correlator-sim-plan). Runs on the INTERNAL OSCH (independent of the
// correlator's xtal/PLL; OSCH's ~±5% tolerance models the RC drift). Emits an N=15 Gold code at a 200 Hz
// chip rate; SW1 selects beacon A (code 0) / beacon B (code 1). Scope outputs:
//   code chip → I/O 14 (P8)      epoch sync (HIGH during chip 0) → I/O 15 (N8)
// Codes are the canonical sim.py set (preferred pair, t0..t14 MSB-first), so emitter == sim == correlator.
module emitter (input sw1, output code, output sync);
  wire clk;
  OSCH #(.NOM_FREQ("53.2")) osc (.STDBY(1'b0), .OSC(clk), .SEDSTDBY());   // valid NOM_FREQ (96.77 was not)

  localparam [14:0] CODE0 = 15'b000001101111011;   // beacon A, t0..t14 (MSB=t0)
  localparam [14:0] CODE1 = 15'b110011100000001;   // beacon B
  localparam integer DIV = 266000;                 // 53.2 MHz / 200 Hz (nominal; OSCH tol = the drift)

  reg [18:0] dcnt = 0;                              // 2^19 = 524288 > 266000
  reg [3:0]  chip = 0;                              // 0..14
  always @(posedge clk) begin
    if (dcnt == DIV-1) begin
      dcnt <= 0;
      chip <= (chip == 4'd14) ? 4'd0 : chip + 1'b1;
    end else dcnt <= dcnt + 1'b1;
  end

  wire [14:0] sel = sw1 ? CODE1 : CODE0;
  assign code = sel[14 - chip];                    // chip 0 → t0 ... chip 14 → t14
  assign sync = (chip == 4'd0);                    // epoch marker (75 ms period @200 Hz)
endmodule
