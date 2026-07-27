// Timing + utilization STRESS design (research, not flown). Two knobs drive the two feedback dimensions:
//   DEPTH  — serial 32-bit add/rotate stages between registers → a long combinational path (TIMING).
//   LANES  — replicated copies of that path, XOR-reduced to one reg → LUT/register load (UTILIZATION).
// A tight FREQUENCY constraint in stress.lpf (e.g. 400 MHz) makes trce report large negative slack so we
// can see exactly what the timing feedback looks like near/over the edge. MachXO2 has NO hard multipliers,
// so the add/rotate chain is pure fabric — predictable LUT + carry-chain delay. Nothing is trimmable: every
// lane feeds the single registered output.
module stress (input clk, output reg led = 1'b0);
  // Empirically ~116 LUT4s per stage on this part; 4320 LUTs total. LANES*DEPTH stages.
  // LANES=4, DEPTH=8 → 32 stages ≈ 3700 LUTs (~86%, fits near the edge); DEPTH=8 = a long comb path.
  localparam integer W = 32, LANES = 4, DEPTH = 8;

  reg [W-1:0] seed = 32'hDEAD_BEEF;
  always @(posedge clk) seed <= {seed[W-2:0], seed[31]^seed[21]^seed[1]^seed[0]};  // keep inputs live

  wire [W-1:0] lane [0:LANES-1];
  genvar i, d;
  generate
    for (i = 0; i < LANES; i = i + 1) begin : LANE
      wire [W-1:0] s [0:DEPTH];
      assign s[0] = seed + (i * 32'h9E37_79B1);
      for (d = 0; d < DEPTH; d = d + 1) begin : ST
        // rotate-left-1 + add + xor : a 32-bit carry chain per stage, DEPTH in series → long comb path
        assign s[d+1] = ({s[d][W-2:0], s[d][W-1]} + s[d]) ^ (32'h1111_1111 + d);
      end
      assign lane[i] = s[DEPTH];
    end
  endgenerate

  integer k; reg [W-1:0] acc;
  always @(posedge clk) begin
    acc = {W{1'b0}};
    for (k = 0; k < LANES; k = k + 1) acc = acc ^ lane[k];   // XOR-reduce → forces all lanes to be kept
    led <= ^acc;
  end
endmodule
