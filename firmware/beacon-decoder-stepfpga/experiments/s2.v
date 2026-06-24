// S2 -- virtual MCP3201 ADC + soft SPI master (correlator-sim-plan). The S1 emitter (internal OSCH) drives a
// Gold-code chip stream; the RECEIVER domain (12 MHz xtal on C1) models the analog level; a bit-exact virtual
// MCP3201 (mcp3201_model) is read by the shared soft SPI master (spi_mcp3201_reader). Top 8 bits -> 8 LEDs.
// SPI mirrored to real J1 pins P3/M4/N4 for scope.
//
// Sampling = 480 Hz (CAMERA cadence: 2.4 samples per 200 Hz chip -- sample like the frame, NOT oversampled;
// the ADC sample-and-holds on CS down). SCLK = 50 kHz (low EMI; inside the 10 kHz-1.6 MHz validity window)
// just dequeues each 16-bit frame, then idles out to the 480 Hz period. The EFB hard-SPI drops in for the
// reader later (see spi_mcp3201.v).
// Correlator-stress knobs (momentary buttons, active-low w/ pull-ups), in prep for the S3 correlator:
//   K1 (L14) = flip 1 RANDOM chip / code period   K3 (M14) = drop the high to 1/3 (weak signal)
//   K2 (M13) = flip 2 RANDOM chips / code period   K4 (N14) = raise the low to ~1/4 FS (high noise floor)
// Independent (stackable); SW1 still selects code A/B.
module s2_top (input clk12, input sw1, input k1, input k2, input k3, input k4,
               output code_pin, output sync_pin, output [7:0] LEDs,
               output spi_cs, output spi_sclk, output spi_do);
  // ---- emitter (internal OSCH, independent of the xtal) ----
  wire oclk; OSCH #(.NOM_FREQ("53.2")) osc (.STDBY(1'b0), .OSC(oclk), .SEDSTDBY());
  localparam [14:0] CODE0 = 15'b000001101111011, CODE1 = 15'b110011100000001;
  localparam integer EDIV = 266000;                       // 53.2 MHz / 200 Hz chip
  reg [18:0] edc = 0; reg [3:0] echip = 0;
  wire wrap = (edc == EDIV-1);
  always @(posedge oclk)
    if (wrap) begin edc <= 0; echip <= (echip==4'd14)?4'd0:echip+1'b1; end else edc <= edc+1'b1;

  // Random bit-error injection (channel model). A free-running LFSR draws fresh random chip target(s) each
  // code period; K1 flips 1 chip/period, K2 flips 2 chips/period (independent → stackable). Random position,
  // not a fixed notch. PRNG runs in the emitter domain (glitch-free chip index); P8 emits the ACTUAL corrupted
  // code (and the epoch on N8 stays clean so the scope/correlator can still time-align).
  reg [15:0] lfsr = 16'hACE1;
  always @(posedge oclk) lfsr <= {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
  reg [3:0] tgtA=0, tgtB=0, tgtC=0;                       // 0..14 (map the all-ones nibble 15 -> 0)
  wire eop = wrap & (echip==4'd14);                       // end of code period -> latch new targets
  always @(posedge oclk) if (eop) begin
    tgtA <= (lfsr[3:0] ==4'd15)?4'd0:lfsr[3:0];
    tgtB <= (lfsr[7:4] ==4'd15)?4'd0:lfsr[7:4];
    tgtC <= (lfsr[11:8]==4'd15)?4'd0:lfsr[11:8];
  end
  reg [1:0] k1s=0, k2s=0; always @(posedge oclk) begin k1s<={k1s[0],~k1}; k2s<={k2s[0],~k2}; end
  wire flip = (k1s[1] & (echip==tgtA))                                 // K1: 1 random chip
            | (k2s[1] & ((echip==tgtB) | (echip==tgtC)));             // K2: 2 random chips

  wire [14:0] esel = sw1 ? CODE1 : CODE0;
  wire ecode = esel[14-echip] ^ flip;                    // actual code w/ injected errors (clean if no buttons)
  assign code_pin = ecode;                                // I/O 14 (P8): the real corrupted code the ADC sees
  assign sync_pin = (echip == 4'd0);                      // I/O 15 (N8): clean epoch / index pulse

  // ---- receiver (12 MHz xtal domain): analog model + virtual MCP3201 + shared SPI reader ----
  reg [1:0] cdc = 0; always @(posedge clk12) cdc <= {cdc[0], ecode};   // sync the (corrupted) code into this domain
  wire code_rx = cdc[1];
  // signal/noise knobs (rx domain): K3 drops the high to 1/3 (weak signal); K4 raises the low to ~1/4 FS
  // (high noise floor). Together they squeeze the correlator's high/low margin.
  reg [1:0] k3s=0, k4s=0; always @(posedge clk12) begin k3s<={k3s[0],~k3}; k4s<={k4s[0],~k4}; end
  wire [11:0] hi = k3s[1] ? 12'h550 : 12'hFF0;            // 1/3 of 0xFF0
  wire [11:0] lo = k4s[1] ? 12'h400 : 12'h010;            // ~1/4 full-scale
  wire [11:0] adc_level = code_rx ? hi : lo;              // analog model: chip hi/lo (12-bit soft sample)

  localparam integer FDIV = 25000;        // sample = clk12 / 25000 = 480 Hz (camera cadence)
  reg [15:0] fdiv = 0; reg fetch = 1'b0;
  always @(posedge clk12)
    if (fdiv == FDIV-1) begin fdiv <= 0; fetch <= 1'b1; end else begin fdiv <= fdiv+1'b1; fetch <= 1'b0; end

  wire cs, sclk, dout, valid; wire [11:0] sample;
  mcp3201_model              adc (.clk(clk12), .cs(cs), .sclk(sclk), .value(adc_level), .dout(dout));
  spi_mcp3201_reader #(.SCLK_HALF(120)) rdr (.clk(clk12), .start(fetch), .miso(dout),
                                             .cs(cs), .sclk(sclk), .sample(sample), .valid(valid));
  reg [11:0] s_l = 0; always @(posedge clk12) if (valid) s_l <= sample;

  assign spi_cs   = cs;
  assign spi_sclk = sclk;
  assign spi_do   = dout;            // virtual ADC DOUT (scope; becomes the real chip's input pin later)
  assign LEDs = s_l[11:4];           // top 8 bits (board active-low): code1 -> 0xFF, code0 -> 0x01
endmodule
