// S3c -- DUAL correlator: check BOTH Gold codes at once while emitting ONE. The S2 sim front end (emitter ->
// injection -> virtual MCP3201 -> soft SPI) feeds two matched filters (CODE0 & CODE1 templates) sharing one
// 36-sample window, DC estimate and energy; each has DC-removal + AGC (|corr|/energy match ratio) and its own
// min-lock / limited-hold FSM. Only the emitted code's correlator should lock -> a live code-discrimination
// test (the CDMA foundation for S6).
//
//   7-seg d1/d2 : signal quality 0-9 for CODE0 / CODE1 (AGC match ratio)
//   LEDl / LEDr : lock of CODE0 / CODE1  (red=search, yellow=acq, green=lock, green-blink=hold)
//   8 LEDs      : q0 bar (left nibble) | q1 bar (right nibble)
//   P8/N8       : emitted code (post-corruption) / clean epoch -- scope;  SPI mirror P3/M4/N4
//   Knobs       : SW1 selects which code is EMITTED; K1/K2 1/2 random flips; K3 weak signal; K4 high floor
//   Lock timing : MINLOCK=2 periods (~150 ms) to confirm, HOLDMAX=2 (~150 ms) before re-acquire
module s3_top (input clk12, input sw1, input k1, input k2, input k3, input k4,
               output code_pin, output sync_pin,
               output spi_cs, output spi_sclk, output spi_do,
               output [7:0] LEDs, output [2:0] LEDl, output [2:0] LEDr,
               output [6:0] d1, output [6:0] d2, output enableLd1, output enableLd2,
               output txd,                               // BCN telemetry UART -> pad A2 -> STEPLink -> COM3
               input  rxd);                              // command UART     <- pad A3 <- STEPLink <- COM3

  // =================== front end (identical to s2) ===================
  wire oclk; OSCH #(.NOM_FREQ("53.2")) osc (.STDBY(1'b0), .OSC(oclk), .SEDSTDBY());
  reg [4:0] cmd_reg = 0;                                  // remote knobs over UART RX (latched in clk12, below)
  reg [2:0] cmd_o0=0, cmd_o1=0;                           // sync code/inj1/inj2 into the emitter (oclk) domain
  always @(posedge oclk) begin cmd_o0 <= cmd_reg[2:0]; cmd_o1 <= cmd_o0; end
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
  wire flip = ((k1s[1]|cmd_o1[1]) & (echip==tgtA)) | ((k2s[1]|cmd_o1[2]) & ((echip==tgtB) | (echip==tgtC)));
  wire [14:0] esel = (sw1 | cmd_o1[0]) ? CODE1 : CODE0;
  wire ecode = esel[14-echip] ^ flip;
  assign code_pin = ecode;
  assign sync_pin = (echip == 4'd0);

  reg [1:0] cdc = 0; always @(posedge clk12) cdc <= {cdc[0], ecode};
  wire code_rx = cdc[1];
  reg [1:0] k3s=0, k4s=0; always @(posedge clk12) begin k3s<={k3s[0],~k3}; k4s<={k4s[0],~k4}; end
  wire [11:0] hi = (k3s[1]|cmd_reg[3]) ? 12'h550 : 12'hFF0;
  wire [11:0] lo = (k4s[1]|cmd_reg[4]) ? 12'h400 : 12'h010;
  // analog-bandwidth model: low-pass the chip waveform so edges RAMP (band-limited PD/TIA), not ideal squares
  // -> edge samples land mid-ramp. IIR at clk12/8 (~1.5 MHz "analog" rate); LPF_SH sets the slew (~0.34 ms, a
  // mild pass — tunable knob for the A4d study, NOT a heavy smear).
  localparam integer LPF_SH = 9;
  wire [19:0] tgt8 = {(code_rx ? hi : lo), 8'b0};        // 12.8 fixed-point chip target
  reg  [19:0] aflt = 0; reg [2:0] lpf_div = 0;
  always @(posedge clk12) begin
    lpf_div <= lpf_div + 1'b1;
    if (lpf_div == 3'd7)
      aflt <= (tgt8 > aflt) ? aflt + ((tgt8 - aflt) >> LPF_SH) : aflt - ((aflt - tgt8) >> LPF_SH);
  end
  wire [11:0] adc_level = aflt[19:8];                     // band-limited (ramped) analog sample

  localparam integer FDIV = 25000;
  reg [15:0] fdiv = 0; reg fetch = 1'b0;
  always @(posedge clk12)
    if (fdiv == FDIV-1) begin fdiv <= 0; fetch <= 1'b1; end else begin fdiv <= fdiv+1'b1; fetch <= 1'b0; end
  wire cs, sclk, dout, valid; wire [11:0] sample;
  mcp3201_model              adc (.clk(clk12), .cs(cs), .sclk(sclk), .value(adc_level), .dout(dout));
  spi_mcp3201_reader #(.SCLK_HALF(120)) rdr (.clk(clk12), .start(fetch), .miso(dout),
                                             .cs(cs), .sclk(sclk), .sample(sample), .valid(valid));
  assign spi_cs = cs; assign spi_sclk = sclk; assign spi_do = dout;

  // =================== dual correlator (DUT) ===================
  function [3:0] mapchip(input [5:0] s);                  // 36 slots (2.4/chip) -> 15 chips
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
  function [6:0] seg7(input [3:0] q);                     // threeN1 map: {a,b,c,d,e,f,g} bit6..0, active-high
    case (q)
      4'd0: seg7=7'b1111110; 4'd1: seg7=7'b0110000; 4'd2: seg7=7'b1101101; 4'd3: seg7=7'b1111001;
      4'd4: seg7=7'b0110011; 4'd5: seg7=7'b1011011; 4'd6: seg7=7'b1011111; 4'd7: seg7=7'b1110000;
      4'd8: seg7=7'b1111111; default: seg7=7'b1111011;   // 9
    endcase
  endfunction
  function [3:0] bar4(input [3:0] q);                     // coarse 4-LED thermometer of a 0-9 quality
    bar4 = (q>=4'd8)?4'hF : (q>=4'd6)?4'h7 : (q>=4'd4)?4'h3 : (q>=4'd2)?4'h1 : 4'h0;
  endfunction

  reg [19:0] dc_acc = 0; wire [11:0] dc = dc_acc[19:8];
  integer i; reg [11:0] win [0:35];
  reg [5:0] ai = 0; reg busy = 0, rdy = 0;
  reg signed [21:0] acc_c0=0, acc_c1=0, corr0=0, corr1=0;
  reg        [21:0] acc_e =0, energy=0;
  wire t0 = CODE0[14 - mapchip(ai)];
  wire t1 = CODE1[14 - mapchip(ai)];
  wire signed [12:0] dev  = $signed({1'b0, win[ai]}) - $signed({1'b0, dc});
  wire signed [21:0] devx = dev;
  wire        [11:0] dabs = dev[12] ? (~dev[11:0] + 1'b1) : dev[11:0];
  always @(posedge clk12) begin
    rdy <= 1'b0;
    if (valid) begin
      for (i=35; i>0; i=i-1) win[i] <= win[i-1];
      win[0] <= sample;
      dc_acc <= dc_acc - {8'b0, dc} + {8'b0, sample};
      ai <= 0; acc_c0 <= 0; acc_c1 <= 0; acc_e <= 0; busy <= 1'b1;
    end else if (busy) begin
      acc_c0 <= acc_c0 + (t0 ? devx : -devx);
      acc_c1 <= acc_c1 + (t1 ? devx : -devx);
      acc_e  <= acc_e + {10'b0, dabs};
      if (ai == 6'd35) begin corr0<=acc_c0; corr1<=acc_c1; energy<=acc_e; rdy<=1'b1; busy<=1'b0; end
      else ai <= ai + 1'b1;
    end
  end

  // per-period peak |corr| for each code (order/polarity-invariant); shared peak energy
  wire signed [21:0] a0 = corr0[21] ? -corr0 : corr0;
  wire signed [21:0] a1 = corr1[21] ? -corr1 : corr1;
  reg [21:0] pk0=0, pk1=0, pke=1, best0=0, best1=0, beste=1;
  reg [5:0]  pcnt=0; reg pend=0;
  always @(posedge clk12) begin
    pend <= 1'b0;
    if (rdy) begin
      if (a0 > pk0) pk0 <= a0;
      if (a1 > pk1) pk1 <= a1;
      if (energy > pke) pke <= energy;
      if (pcnt == 6'd35) begin
        best0 <= (a0>pk0)?a0:pk0; best1 <= (a1>pk1)?a1:pk1;
        beste <= ((energy>pke)?energy:pke) | 22'd1;
        pk0<=0; pk1<=0; pke<=0; pcnt<=0; pend<=1'b1;
      end else pcnt <= pcnt + 1'b1;
    end
  end

  // quality q in 0-9 = min(9, 9*|corr|/energy) -- the ACTUAL match level for each code (like the thermometer):
  // the matching code reads ~9, the other reads its true cross-corr/noise level (~5) as a diagnostic hint.
  // The lock FSM (GOOD), not the digit, does the binary discrimination. Sequential per code.
  wire [24:0] n0_raw = (best0<<3)+best0;           // 9*best0
  wire [24:0] n1_raw = (best1<<3)+best1;           // 9*best1
  reg [24:0] num=0; reg [21:0] den=1; reg [3:0] cnt=0; reg [1:0] dstate=0;
  reg [3:0] q0=0, q1=0; reg q0_rdy=0, q1_rdy=0;
  always @(posedge clk12) begin
    q0_rdy <= 1'b0; q1_rdy <= 1'b0;
    case (dstate)
      2'd0: if (pend) begin num <= n0_raw; den <= beste; cnt <= 0; dstate <= 2'd1; end
      2'd1: if ((num >= {3'b0,den}) && (cnt < 4'd9)) begin num <= num - {3'b0,den}; cnt <= cnt + 1'b1; end
            else begin q0 <= cnt; q0_rdy <= 1'b1; num <= n1_raw; den <= beste; cnt <= 0; dstate <= 2'd2; end
      2'd2: if ((num >= {3'b0,den}) && (cnt < 4'd9)) begin num <= num - {3'b0,den}; cnt <= cnt + 1'b1; end
            else begin q1 <= cnt; q1_rdy <= 1'b1; dstate <= 2'd0; end
    endcase
  end

  // per-code lock FSM: min-lock-time to confirm, limited hold across short dropouts, then re-acquire
  localparam [3:0] GOOD = 4'd6;        // q>=6 of 9: one notch above the Gold cross-corr floor (~5) so the
                                       // wrong code never locks, but below a 1-bit-error level (~7) so a single
                                       // flip rides through. (N=15 has a thin 5->7 margin; longer codes widen it.)
  localparam [2:0] MINLOCK = 3'd2, HOLDMAX = 3'd2;
  localparam [1:0] SEARCH=2'd0, ACQ=2'd1, LOCK=2'd2, HOLD=2'd3;
  reg [1:0] st0=SEARCH, st1=SEARCH; reg [2:0] g0=0,b0=0,g1=0,b1=0;
  always @(posedge clk12) if (q0_rdy) begin
    if (q0 >= GOOD) case (st0)
      SEARCH: begin st0<=ACQ; g0<=3'd1; end
      ACQ:    if (g0>=MINLOCK-1) st0<=LOCK; else g0<=g0+1'b1;
      LOCK:   b0<=0;
      HOLD:   begin st0<=LOCK; b0<=0; end
    endcase
    else case (st0)
      SEARCH: ; ACQ: begin st0<=SEARCH; g0<=0; end
      LOCK:   begin st0<=HOLD; b0<=3'd1; end
      HOLD:   if (b0>=HOLDMAX) begin st0<=SEARCH; g0<=0; b0<=0; end else b0<=b0+1'b1;
    endcase
  end
  always @(posedge clk12) if (q1_rdy) begin
    if (q1 >= GOOD) case (st1)
      SEARCH: begin st1<=ACQ; g1<=3'd1; end
      ACQ:    if (g1>=MINLOCK-1) st1<=LOCK; else g1<=g1+1'b1;
      LOCK:   b1<=0;
      HOLD:   begin st1<=LOCK; b1<=0; end
    endcase
    else case (st1)
      SEARCH: ; ACQ: begin st1<=SEARCH; g1<=0; end
      LOCK:   begin st1<=HOLD; b1<=3'd1; end
      HOLD:   if (b1>=HOLDMAX) begin st1<=SEARCH; g1<=0; b1<=0; end else b1<=b1+1'b1;
    endcase
  end

  // =================== displays ===================
  reg [22:0] blink = 0; always @(posedge clk12) blink <= blink + 1'b1;
  function [2:0] rgb(input [1:0] st, input bk);          // R=[0] G=[1] B=[2], active-high "on" mask
    rgb = (st==LOCK) ? 3'b010 : (st==HOLD) ? (bk?3'b010:3'b000) : (st==ACQ) ? 3'b011 : 3'b001;
  endfunction
  assign LEDl = ~rgb(st0, blink[21]);                    // CODE0 lock (active-low LEDs)
  assign LEDr = ~rgb(st1, blink[21]);                    // CODE1 lock
  assign LEDs = ~{bar4(q0), bar4(q1)};                   // left nibble=q0 bar, right nibble=q1 bar
  assign d1   = seg7(q0);                                // 7-seg: CODE0 quality 0-9
  assign d2   = seg7(q1);                                // 7-seg: CODE1 quality 0-9
  assign enableLd1 = 1'b0;                               // active-low digit enable (threeN1: enableL=0 = on)
  assign enableLd2 = 1'b0;

  // =================== command RX (UART on pad A3): latch knob byte 0x40-0x5F -> cmd_reg[4:0] ===================
  // mask bits: [0]=code-select(B), [1]=inject-1-bit, [2]=inject-2-bit, [3]=weak-signal, [4]=high-floor.
  // host sends chr(0x40 | mask); physical SW1/K1-K4 still OR in. 0x40-0x5F only -> newlines/noise ignored.
  wire [7:0] rxb; wire rxv;
  uart_rx #(.DIV(104)) u_rx (.clk(clk12), .rxd(rxd), .data(rxb), .valid(rxv));
  always @(posedge clk12) if (rxv && (rxb[7:5]==3'b010)) cmd_reg <= rxb[4:0];

  // =================== BCN telemetry (UART TX on pad A2 -> STEPLink -> COM3; host/ reads it) ===================
  reg [13:0] seq = 0; reg [11:0] adc_l = 0;
  always @(posedge clk12) if (valid) adc_l <= sample;
  reg [18:0] tdiv = 0; reg tick = 0;
  always @(posedge clk12)                                // emit ~40 Hz (12 MHz / 300000) — control-loop family
    if (tdiv == 19'd299999) begin tdiv<=0; tick<=1'b1; seq<=(seq==14'd9999)?14'd0:seq+1'b1; end
    else begin tdiv<=tdiv+1'b1; tick<=1'b0; end
  wire [1:0] lka = (st0==SEARCH)?2'd0:(st0==ACQ)?2'd1:2'd2;   // frame lock: 0 no_lock / 1 tentative / 2 confirmed
  wire [1:0] lkb = (st1==SEARCH)?2'd0:(st1==ACQ)?2'd1:2'd2;
  bcn_tx u_bcn (.clk(clk12), .tick(tick), .seq(seq), .adc(adc_l),
                .corrA(best0[19:0]), .lockA(lka), .marginA(q0),
                .corrB(best1[19:0]), .lockB(lkb), .marginB(q1), .txd(txd));
endmodule
