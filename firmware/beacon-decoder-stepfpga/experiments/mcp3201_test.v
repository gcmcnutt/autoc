// MCP3201 SPI decode self-test (live hardware, like selftest.v). A 12-bit LFSR injects known values through
// the bit-exact virtual MCP3201 (mcp3201_model); the shared SPI master (spi_mcp3201_reader) reads them back;
// a checker compares decode-vs-injected on every frame, back-to-back, forever. This proves the SPI fetch +
// 16-clock/raw[12:1] decode is MCP3201-correct (and, at S7, that the real chip reads identically).
//
//   LEDs : walking dot  = PASS (zero decode errors so far)
//          fast all-blink = FAIL (a mismatch was latched; never clears -- power-cycle to retest)
//   SPI  : CS=P3, SCLK=M4, DOUT=N4 mirrored for scope.
module mcp3201_test (input clk12, output [7:0] LEDs, output spi_cs, output spi_sclk, output spi_do);
  // --- known-value injector: stable during each frame, advanced on completion ---
  reg [11:0] val  = 12'h555;
  reg [11:0] lfsr = 12'hACE;

  // --- shared SPI reader + virtual MCP3201 ---
  wire cs, sclk, dout, valid; wire [11:0] sample;
  localparam integer FDIV = 25000;        // 480 Hz sample cadence — CS clearly frames each SCLK burst on the
  reg [15:0] fdiv = 0; reg start = 1'b0;  // scope (CS low ~320 us, high ~1.7 ms); full LFSR sweep ~8.5 s.
  always @(posedge clk12)
    if (fdiv == FDIV-1) begin fdiv <= 0; start <= 1'b1; end else begin fdiv <= fdiv+1'b1; start <= 1'b0; end

  mcp3201_model              adc (.clk(clk12), .cs(cs), .sclk(sclk), .value(val), .dout(dout));
  spi_mcp3201_reader #(.SCLK_HALF(120)) rdr (.clk(clk12), .start(start), .miso(dout),
                                             .cs(cs), .sclk(sclk), .sample(sample), .valid(valid));

  // --- checker ---
  reg        err = 1'b0;
  reg [15:0] errc = 0;
  reg [31:0] frames = 0;
  always @(posedge clk12) if (valid) begin
    frames <= frames + 1'b1;
    if (sample != val) begin err <= 1'b1; errc <= errc + 1'b1; end
    lfsr <= {lfsr[10:0], lfsr[11]^lfsr[5]^lfsr[3]^lfsr[0]};  // step the sequence (period 4095)
    val  <= lfsr;                                            // next frame's value (held stable until then)
  end

  // --- pass/fail display (board LEDs active-low) ---
  reg [23:0] hb = 0; always @(posedge clk12) hb <= hb + 1'b1;
  wire [2:0] pos  = hb[23:21];
  wire [7:0] disp = err ? (hb[20] ? 8'hFF : 8'h00)   // FAIL : ~11 Hz all-blink alarm
                        : (8'h01 << pos);            // PASS : walking dot
  assign LEDs     = ~disp;
  assign spi_cs   = cs;
  assign spi_sclk = sclk;
  assign spi_do   = dout;
endmodule
