// S2 -- virtual MCP3201 ADC + soft SPI master (correlator-sim-plan). The S1 emitter (internal OSCH) drives a
// Gold-code chip stream; the RECEIVER domain (12 MHz xtal on C1) models the analog level; a bit-exact virtual
// MCP3201 (mcp3201_model) is read by the shared soft SPI master (spi_mcp3201_reader). Top 8 bits -> 8 LEDs.
// SPI mirrored to real J1 pins P3/M4/N4 for scope.
//
// Sampling = 480 Hz (CAMERA cadence: 2.4 samples per 200 Hz chip -- sample like the frame, NOT oversampled;
// the ADC sample-and-holds on CS down). SCLK = 50 kHz (low EMI; inside the 10 kHz-1.6 MHz validity window)
// just dequeues each 16-bit frame, then idles out to the 480 Hz period. The EFB hard-SPI drops in for the
// reader later (see spi_mcp3201.v).
module s2_top (input clk12, input sw1, output code_pin, output sync_pin, output [7:0] LEDs,
               output spi_cs, output spi_sclk, output spi_do);
  // ---- emitter (internal OSCH, independent of the xtal) ----
  wire oclk; OSCH #(.NOM_FREQ("53.2")) osc (.STDBY(1'b0), .OSC(oclk), .SEDSTDBY());
  localparam [14:0] CODE0 = 15'b000001101111011, CODE1 = 15'b110011100000001;
  localparam integer EDIV = 266000;                       // 53.2 MHz / 200 Hz chip
  reg [18:0] edc = 0; reg [3:0] echip = 0;
  always @(posedge oclk)
    if (edc == EDIV-1) begin edc <= 0; echip <= (echip==4'd14)?4'd0:echip+1'b1; end else edc <= edc+1'b1;
  wire [14:0] esel = sw1 ? CODE1 : CODE0;
  wire ecode = esel[14-echip];
  assign code_pin = ecode;
  assign sync_pin = (echip == 4'd0);

  // ---- receiver (12 MHz xtal domain): analog model + virtual MCP3201 + shared SPI reader ----
  reg [1:0] cdc = 0; always @(posedge clk12) cdc <= {cdc[0], ecode};   // sync the code bit into this domain
  wire code_rx = cdc[1];
  wire [11:0] adc_level = code_rx ? 12'hFF0 : 12'h010;     // analog model: chip hi/lo (12-bit soft sample)

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
