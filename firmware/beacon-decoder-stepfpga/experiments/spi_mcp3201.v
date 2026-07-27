// Shared MCP3201 SPI blocks for the correlator sim — used by s2 (virtual-ADC display) and the decode
// self-test (mcp3201_test). Both run in the receiver (clk) domain.
//
// MCP3201 frame (datasheet 21290F): CS↓ = sample-and-hold; ~1.5-clk sample window; a NULL bit; then
// B11..B0 MSB-first; if CS stays low it repeats LSB-first. DOUT changes on SCLK falling, master reads on
// rising. SCLK must be 10 kHz–1.6 MHz (the held charge droops below 10 kHz). Reading 16 rising edges
// MSB-first gives raw[15:0]; the 12-bit code = raw[12:1] (raw[13]=null) — the common 2-byte-driver
// alignment ((byte0 & 0x1F) << 7 | byte1 >> 1).
//
// 8-bit hard-IP note (MCP3201 datasheet p21): an 8-bit SPI master (e.g. the MachXO2 EFB) reads the 12-bit
// result as two 8-bit transfers with CS held low across both and SCLK PAUSED (idle high or low per the EFB
// CPOL) between them -- the device holds state on a stalled clock. spi_mcp3201_reader below does one
// continuous 16-clock read (simplest, charge-valid); the EFB IP may instead use the paused two-byte scheme.
// Either way mcp3201_model validates it: the model is edge-driven, so a paused SCLK just holds DOUT.

// Soft SPI master / MCP3201 reader. SCLK = clk / (2·SCLK_HALF); 120 → 50 kHz @ 12 MHz (low-EMI, charge-valid).
module spi_mcp3201_reader #(parameter integer SCLK_HALF = 120) (
  input  wire        clk,
  input  wire        start,        // 1-clk pulse: begin one frame (the fetcher tick)
  input  wire        miso,         // DOUT from the (virtual or real) MCP3201
  output reg         cs,
  output reg         sclk,
  output reg  [11:0] sample,
  output reg         valid         // 1-clk pulse when `sample` updates
);
  reg [7:0]  ph;
  reg        busy;
  reg [4:0]  nr;                    // rising-edge count
  reg [15:0] raw;
  initial begin cs=1'b1; sclk=1'b0; sample=12'h000; valid=1'b0; busy=1'b0; ph=8'd0; nr=5'd0; raw=16'd0; end
  always @(posedge clk) begin
    valid <= 1'b0;
    if (!busy) begin
      cs <= 1'b1; sclk <= 1'b0;
      if (start) begin cs <= 1'b0; ph <= 8'd0; nr <= 5'd0; raw <= 16'd0; busy <= 1'b1; end
    end else if (ph == SCLK_HALF-1) begin
      ph <= 8'd0; sclk <= ~sclk;
      if (!sclk) begin                          // rising edge → master samples DOUT
        raw <= {raw[14:0], miso};
        nr  <= nr + 1'b1;
      end else if (nr == 5'd16) begin            // falling after the 16th rising → frame complete
        cs <= 1'b1; busy <= 1'b0;
        sample <= raw[12:1]; valid <= 1'b1;      // raw[13]=null, raw[12:1]=B11..B0
      end
    end else ph <= ph + 1'b1;
  end
endmodule

// Bit-exact virtual MCP3201 slave (the test injector; swapped for the real chip's DOUT pin at S7). Clocked by
// `clk`, it watches CS/SCLK and emits the datasheet DOUT sequence for the 12-bit `value` held at CS↓.
module mcp3201_model (
  input  wire        clk,
  input  wire        cs,
  input  wire        sclk,
  input  wire [11:0] value,         // analog sample present at CS↓ (sample-and-hold)
  output reg         dout
);
  reg        cs_d, sclk_d;
  reg [11:0] held;
  reg [4:0]  fcnt;                  // SCLK falling edges since CS↓
  initial begin dout=1'b1; cs_d=1'b1; sclk_d=1'b0; held=12'd0; fcnt=5'd0; end
  always @(posedge clk) begin
    cs_d <= cs; sclk_d <= sclk;
    if (cs & ~cs_d) dout <= 1'b1;                                   // CS↑ : idle high (real chip tri-states)
    else if (~cs & cs_d) begin held <= value; fcnt <= 5'd0; dout <= 1'b1; end   // CS↓ : sample-and-hold + lead
    else if (~cs & sclk_d & ~sclk) begin                           // SCLK falling → present next bit
      fcnt <= fcnt + 1'b1;                                          // (bit is sampled at the next rising edge)
      case (fcnt)
        5'd0:    dout <= 1'b1;                                      // → rising-2 : lead
        5'd1:    dout <= 1'b0;                                      // → rising-3 : NULL
        default: dout <= (fcnt <= 5'd13) ? held[11 - (fcnt - 2)]    // → rising-4..15 : B11..B0
                                         : held[0];                 // over-clock region (masked out)
      endcase
    end
  end
endmodule
