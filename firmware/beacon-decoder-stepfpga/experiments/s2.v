// S2a — virtual MCP3201 ADC + soft SPI master (correlator-sim-plan). The S1 emitter (internal OSCH) drives a
// Gold-code chip stream; the RECEIVER domain (12 MHz xtal on C1) models the analog level, a virtual MCP3201
// SPI *slave* serializes the 12-bit sample, and a soft SPI *master* reads it back over an internal SPI link
// (CS/SCLK/DOUT — protocol-identical to what'll read the real MCP3201). Top 8 bits → 8 LEDs.
//
// Sample rate ≈ clk12/(16·SCLK_HALF) ≈ 47 kS/s here (MCP3201 ~100 kS/s class; oversamples the 200 Hz chip).
// NOT 480 fps — that's the camera. S2b: route SPI to the real J1 pins for scope + swap in the EFB hard-SPI.
module s2_top (input clk12, input sw1, input k1, output code_pin, output sync_pin, output [7:0] LEDs,
               output spi_cs, output spi_sclk, output spi_do);   // SPI mirrored to real J1 pins (scope)
  // ---------------- emitter (internal OSCH, independent of the xtal) ----------------
  wire oclk; OSCH #(.NOM_FREQ("53.2")) osc (.STDBY(1'b0), .OSC(oclk), .SEDSTDBY());
  localparam [14:0] CODE0 = 15'b000001101111011, CODE1 = 15'b110011100000001;
  localparam integer EDIV = 266000;                       // 53.2 MHz / 200 Hz chip
  // K1 (L14, active-low w/ pull-up): hold to slow the emitter 100× (chip 200 Hz → 2 Hz) so the 15-chip
  // code steps visibly across the LEDs (0.5 s/chip) — a human-speed view of the whole emitter→ADC→SPI chain.
  reg [1:0] k1s = 0; always @(posedge oclk) k1s <= {k1s[0], ~k1};
  wire slow = k1s[1];
  reg [24:0] edc = 0; reg [3:0] echip = 0;
  wire [24:0] ethr = slow ? (25'd26600000 - 1) : (25'd266000 - 1);   // ×100 when K1 pressed
  always @(posedge oclk)
    if (edc >= ethr) begin edc <= 0; echip <= (echip==4'd14)?4'd0:echip+1'b1; end else edc <= edc+1'b1;
  wire [14:0] esel = sw1 ? CODE1 : CODE0;
  wire ecode = esel[14-echip];
  assign code_pin = ecode;
  assign sync_pin = (echip == 4'd0);

  // ---------------- receiver (12 MHz xtal domain) ----------------
  reg [1:0] cdc = 0; always @(posedge clk12) cdc <= {cdc[0], ecode};   // sync the code bit into this domain
  wire code_rx = cdc[1];
  wire [11:0] adc_level = code_rx ? 12'hFF0 : 12'h010;     // analog model: chip hi/lo (12-bit soft sample)

  // virtual MCP3201 SPI slave + soft SPI master, continuous back-to-back reads.
  // SCLK = clk12 / (2·SCLK_HALF) = 12/16 = 750 kHz. Frame = 16 SCLK: {lead0, null0, B11..B0, x, x}, MSB first.
  localparam integer SCLK_HALF = 8;       // SCLK = clk12/16 = 750 kHz (≤ MCP3201 1.6 MHz max)
  localparam integer FDIV = 500;          // fetcher cadence = clk12/500 = 24 kHz ("fetcher clock" — the
                                          // sample-fetch rate; tunable ≤100 kHz ADC max; oversamples the 200 Hz chip)
  reg [15:0] fdiv = 0; reg fetch = 1'b0;
  always @(posedge clk12)
    if (fdiv == FDIV-1) begin fdiv <= 0; fetch <= 1'b1; end else begin fdiv <= fdiv+1'b1; fetch <= 1'b0; end

  reg [3:0]  ph = 0;
  reg        sclk = 0, cs = 1'b1, st = 1'b0;
  reg [4:0]  nbit = 0;
  reg [15:0] sh_slave = 0, sh_master = 0;
  reg [11:0] sample = 0;
  always @(posedge clk12) begin
    if (!st) begin                                        // idle (CS high); start a frame on the fetcher tick
      cs <= 1'b1;
      if (fetch) begin
        cs <= 1'b0; sclk <= 1'b0; nbit <= 0; ph <= 0;
        sh_slave <= {2'b00, adc_level, 2'b00}; sh_master <= 0; st <= 1'b1;   // [15]lead [14]null [13:2]B11..B0
      end
    end else if (ph == SCLK_HALF-1) begin
      ph <= 0; sclk <= ~sclk;
      if (!sclk) sh_master <= {sh_master[14:0], sh_slave[15]};   // rising → master samples DOUT
      else begin                                                // falling → slave advances
        sh_slave <= {sh_slave[14:0], 1'b0};
        nbit <= nbit + 1'b1;
        if (nbit == 5'd15) begin cs <= 1'b1; sample <= sh_master[13:2]; st <= 1'b0; end
      end
    end else ph <= ph + 1'b1;
  end

  assign spi_cs   = cs;
  assign spi_sclk = sclk;
  assign spi_do   = sh_slave[15];   // virtual ADC DOUT (scope mirror; becomes an INPUT when the real MCP3201 lands)
  assign LEDs = sample[11:4];       // top 8 bits (board active-low): code1→0xFF, code0→0x01 (hold K1 to see chips)
endmodule
