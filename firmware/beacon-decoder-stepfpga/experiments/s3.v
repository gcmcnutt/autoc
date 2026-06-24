// S3a -- correlator (DUT) on ONE code, fed by the S2 sim front end (emitter -> injection -> virtual MCP3201
// -> soft SPI reader). The decoded 480 Hz / 12-bit sample stream drives a soft matched filter: a 36-sample
// sliding window (15 chips x 2.4 samples/chip) correlated against the SW1-selected code upsampled to a +/-1
// template. The code is +/-1, so the matched filter is signed-accumulate -- no multiplier yet (that arrives
// with soft-weighted templates). Per-period peak -> lock FSM.
//
//   8 LEDs  : correlation bar-graph (thermometer) -- watch the peak rise on lock, collapse under errors/noise
//   LEDl    : beacon-A lock -- red=no-lock, yellow=tentative, green=confirmed   (LEDr reserved for B / S6)
//   P8 / N8 : emitter code (post-corruption) / clean epoch -- scope    SPI mirror on P3/M4/N4
//   Knobs   : SW1 code A/B; K1/K2 inject 1/2 random chip flips; K3 weak signal; K4 high noise floor (= S2)
//   7-seg margin number is S3b.
module s3_top (input clk12, input sw1, input k1, input k2, input k3, input k4,
               output code_pin, output sync_pin,
               output spi_cs, output spi_sclk, output spi_do,
               output [7:0] LEDs, output [2:0] LEDl, output [2:0] LEDr);

  // =================== front end (identical to s2) ===================
  wire oclk; OSCH #(.NOM_FREQ("53.2")) osc (.STDBY(1'b0), .OSC(oclk), .SEDSTDBY());
  localparam [14:0] CODE0 = 15'b000001101111011, CODE1 = 15'b110011100000001;
  localparam integer EDIV = 266000;
  reg [18:0] edc = 0; reg [3:0] echip = 0;
  wire wrap = (edc == EDIV-1);
  always @(posedge oclk)
    if (wrap) begin edc <= 0; echip <= (echip==4'd14)?4'd0:echip+1'b1; end else edc <= edc+1'b1;
  reg [15:0] lfsr = 16'hACE1;
  always @(posedge oclk) lfsr <= {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
  reg [3:0] tgtA=0, tgtB=0, tgtC=0;
  wire eop = wrap & (echip==4'd14);
  always @(posedge oclk) if (eop) begin
    tgtA <= (lfsr[3:0] ==4'd15)?4'd0:lfsr[3:0];
    tgtB <= (lfsr[7:4] ==4'd15)?4'd0:lfsr[7:4];
    tgtC <= (lfsr[11:8]==4'd15)?4'd0:lfsr[11:8];
  end
  reg [1:0] k1s=0, k2s=0; always @(posedge oclk) begin k1s<={k1s[0],~k1}; k2s<={k2s[0],~k2}; end
  wire flip = (k1s[1] & (echip==tgtA)) | (k2s[1] & ((echip==tgtB) | (echip==tgtC)));
  wire [14:0] esel = sw1 ? CODE1 : CODE0;
  wire ecode = esel[14-echip] ^ flip;
  assign code_pin = ecode;
  assign sync_pin = (echip == 4'd0);

  reg [1:0] cdc = 0; always @(posedge clk12) cdc <= {cdc[0], ecode};
  wire code_rx = cdc[1];
  reg [1:0] k3s=0, k4s=0; always @(posedge clk12) begin k3s<={k3s[0],~k3}; k4s<={k4s[0],~k4}; end
  wire [11:0] hi = k3s[1] ? 12'h550 : 12'hFF0;
  wire [11:0] lo = k4s[1] ? 12'h400 : 12'h010;
  wire [11:0] adc_level = code_rx ? hi : lo;

  localparam integer FDIV = 25000;        // 480 Hz sample cadence
  reg [15:0] fdiv = 0; reg fetch = 1'b0;
  always @(posedge clk12)
    if (fdiv == FDIV-1) begin fdiv <= 0; fetch <= 1'b1; end else begin fdiv <= fdiv+1'b1; fetch <= 1'b0; end
  wire cs, sclk, dout, valid; wire [11:0] sample;
  mcp3201_model              adc (.clk(clk12), .cs(cs), .sclk(sclk), .value(adc_level), .dout(dout));
  spi_mcp3201_reader #(.SCLK_HALF(120)) rdr (.clk(clk12), .start(fetch), .miso(dout),
                                             .cs(cs), .sclk(sclk), .sample(sample), .valid(valid));
  assign spi_cs = cs; assign spi_sclk = sclk; assign spi_do = dout;

  // =================== correlator (DUT) ===================
  // 36-slot sample window (2.4 samples/chip x 15). slot->chip map (each chip spans 2 or 3 slots, avg 2.4):
  function [3:0] mapchip(input [5:0] s);
    case (s)
      6'd0,6'd1,6'd2:    mapchip=4'd0;  6'd3,6'd4:         mapchip=4'd1;
      6'd5,6'd6,6'd7:    mapchip=4'd2;  6'd8,6'd9:         mapchip=4'd3;
      6'd10,6'd11:       mapchip=4'd4;  6'd12,6'd13,6'd14: mapchip=4'd5;
      6'd15,6'd16:       mapchip=4'd6;  6'd17,6'd18,6'd19: mapchip=4'd7;
      6'd20,6'd21:       mapchip=4'd8;  6'd22,6'd23:       mapchip=4'd9;
      6'd24,6'd25,6'd26: mapchip=4'd10; 6'd27,6'd28:       mapchip=4'd11;
      6'd29,6'd30,6'd31: mapchip=4'd12; 6'd32,6'd33:       mapchip=4'd13;
      default:           mapchip=4'd14;
    endcase
  endfunction

  integer i;
  reg [11:0] win [0:35];
  reg [5:0]  ai = 0;                       // accumulate slot index
  reg        busy = 0;
  reg signed [20:0] acc = 0, corr = 0;
  reg        corr_rdy = 0;
  wire       tmpl_ai = esel[14 - mapchip(ai)];           // +1 if code bit=1, else -1
  always @(posedge clk12) begin
    corr_rdy <= 1'b0;
    if (valid) begin                                      // shift window, kick the accumulate
      for (i=35; i>0; i=i-1) win[i] <= win[i-1];
      win[0] <= sample;
      ai <= 0; acc <= 0; busy <= 1'b1;
    end else if (busy) begin                              // one slot/cycle (36 << 25000 cycles available)
      acc <= acc + (tmpl_ai ? $signed({9'd0, win[ai]}) : -$signed({9'd0, win[ai]}));
      if (ai == 6'd35) begin corr <= acc; corr_rdy <= 1'b1; busy <= 1'b0; end
      else ai <= ai + 1'b1;
    end
  end

  // per-period peak (max positive correlation over a 36-sample window)
  localparam signed [20:0] THRESH = 21'sd35000;           // tune by watching the bar-graph
  reg signed [20:0] peakacc = 0, peak = 0;
  reg [5:0] pcnt = 0;
  always @(posedge clk12) if (corr_rdy) begin
    if (corr > peakacc) peakacc <= corr;
    if (pcnt == 6'd35) begin peak <= (corr > peakacc) ? corr : peakacc; peakacc <= 0; pcnt <= 0; end
    else pcnt <= pcnt + 1'b1;
  end

  // lock FSM: leaky hit counter over periods
  wire period_end = corr_rdy & (pcnt == 6'd35);
  reg [3:0] hitc = 0;
  always @(posedge clk12) if (period_end) begin
    if (peak > THRESH) begin if (hitc != 4'd8) hitc <= hitc + 1'b1; end
    else                     if (hitc != 4'd0) hitc <= hitc - 1'b1;
  end
  wire green = (hitc >= 4'd4), tent = (hitc >= 4'd1) & ~green;

  // =================== displays ===================
  // bar-graph: thermometer of peak (active-low). ~8192 corr/LED -> full bar near the clean peak (~77k).
  wire [3:0] lvl = (peak[20]) ? 4'd0 : (peak[19:13] > 7'd8 ? 4'd8 : peak[19:13]);
  wire [7:0] bar = (8'hFF >> (4'd8 - lvl));               // lvl ones from LSB up
  assign LEDs = ~bar;

  // RGB[0] lock (assume LEDl[0]=R,[1]=G,[2]=B, active-low; remap if HW differs)
  wire [2:0] rgb_on = green ? 3'b010 : tent ? 3'b011 : 3'b001;   // green / yellow(R+G) / red
  assign LEDl = ~rgb_on;
  assign LEDr = 3'b111;                                   // off (reserved for beacon B, S6)
endmodule
