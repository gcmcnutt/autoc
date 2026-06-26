// S6 -- TWO emitters (codes A & B on independent skewed OSCH divisors) + a random-noise source, SUMMED into
// the one virtual ADC; the dual correlator checks both codes -> live CDMA + noise-floor test. Sources enable
// from the DIP switches (local demo) OR over USB (richer control as we grow).
//   DIP1 M7=enA   DIP2 M8=enB   DIP3 M9=enN   DIP4 M10=code-B clock skew (-3%/+3%)
//   USB: '+'=REMOTE (USB owns) / '-'=LOCAL (switches own) / 0x80|mask = 7 knobs
//        mask [0]enA [1]enB [2]enN [3]inj-1bit(A&B) [4]inj-2bit(A&B) [5]weak-signal [6]DC-floor
//   K1..K4 momentary = local inj-1bit / inj-2bit / weak / floor.  (Command mode will grow richer -- per-source
//   magnitude, skew, etc.; the switches stay a simple local-demo subset.)
//   7-seg d1/d2 = per-code quality 0-9; LEDl/LEDr = per-code lock; 8 LEDs = q bars; P8/N8 = code A / epoch.
//   UART: TX telemetry on A2, RX commands on A3.
module s3_top (input clk12,
               input sw1, input dip2, input dip3, input dip4,   // DIP1(M7) DIP2(M8) DIP3(M9) DIP4(M10)
               input k1, input k2, input k3, input k4,
               output code_pin, output sync_pin,
               output spi_cs, output spi_sclk, output spi_do,
               output [7:0] LEDs, output [2:0] LEDl, output [2:0] LEDr,
               output [6:0] d1, output [6:0] d2, output enableLd1, output enableLd2,
               output txd, input rxd);

  // ============ control: physical switches/buttons (active-low, pulled up) + USB override ============
  reg [6:0] cmd_reg = 0; reg remote = 0;
  reg [1:0] dsw1=0,dsw2=0,dsw3=0,dsw4=0, kb1=0,kb2=0,kb3=0,kb4=0;
  always @(posedge clk12) begin
    dsw1<={dsw1[0],sw1}; dsw2<={dsw2[0],dip2}; dsw3<={dsw3[0],dip3}; dsw4<={dsw4[0],dip4};
    kb1<={kb1[0],~k1}; kb2<={kb2[0],~k2}; kb3<={kb3[0],~k3}; kb4<={kb4[0],~k4};
  end
  wire enA    = remote ? cmd_reg[0] : dsw1[1];    // DIP "ON" label = DISABLE (mute) -> source on when switch OFF
  wire enB    = remote ? cmd_reg[1] : dsw2[1];
  wire enN    = remote ? cmd_reg[2] : dsw3[1];
  wire einj1  = remote ? cmd_reg[3] : kb1[1];
  wire einj2  = remote ? cmd_reg[4] : kb2[1];
  wire eweak  = remote ? cmd_reg[5] : kb3[1];
  wire efloor = remote ? cmd_reg[6] : kb4[1];
  reg signed [19:0] ediva_cmd = 20'sd266000;       // emitter-A divider via USB 'E' command (skew A; LOCAL=nominal)
  wire [18:0] ediva_eff = remote ? ediva_cmd[18:0] : 19'd266000;   // LOCAL: A is the nominal 200 Hz reference
  reg signed [19:0] edivb_cmd = 20'sd266000;       // emitter-B divider set by the USB 'F' command (rate test)
  wire [18:0] edivb_eff = remote ? edivb_cmd[18:0] // remote: 'F'-commanded rate
                                 : (dsw4[1] ? 19'd262000 : 19'd257000);  // B ALWAYS skewed vs A (real beacons
                                 // never share a clock): DIP4 off ~+1.5% / on ~+3.5% -> they slip -> averaged
  reg [7:0] burst_cmd = 0;                          // USB 'K': dropout-span in CHIPS (periodic occlusion of A&B)
  wire [7:0] burst_eff = remote ? burst_cmd : 8'd0; // LOCAL = no burst (power-on clean)

  // ============ emitters: internal OSCH; A & B at independent divisors -> real inter-beacon slip ============
  wire oclk; OSCH #(.NOM_FREQ("53.2")) osc (.STDBY(1'b0), .OSC(oclk), .SEDSTDBY());
  localparam integer N = 31, L = 74;                       // code length (chips); window = round(2.4·N) samples
  localparam [30:0] CODE0 = 31'b0000000100011011000011001110011,   // N=31 Gold preferred pair (xcorr {-9,-1,7})
                    CODE1 = 31'b0100011001100111100101001011110;   // (N=63 codes in git @ 1bd3b4a if needed)
  reg [1:0] inj1o=0, inj2o=0;                     // inject bits synced into the emitter (oclk) domain
  reg [18:0] ediva_o1=19'd266000, ediva_o2=19'd266000, edivb_o1=19'd266000, edivb_o2=19'd266000;  // dividers -> oclk
  reg [7:0] burst_o1=0, burst_o2=0;              // burst span synced into oclk
  always @(posedge oclk) begin
    inj1o<={inj1o[0],einj1}; inj2o<={inj2o[0],einj2};
    ediva_o1<=ediva_eff; ediva_o2<=ediva_o1; edivb_o1<=edivb_eff; edivb_o2<=edivb_o1;
    burst_o1<=burst_eff; burst_o2<=burst_o1;
  end

  // emitter A @ OSCH/EDIV_A (nominal 200 Hz, or skewed via USB 'E'), with random bit-error injection
  wire [18:0] EDIV_A = ediva_o2;
  reg [18:0] edcA=0; reg [5:0] echA=0; wire wrapA=(edcA>=EDIV_A-1);
  always @(posedge oclk) if (wrapA) begin edcA<=0; echA<=(echA==N-1)?6'd0:echA+1'b1; end else edcA<=edcA+1'b1;
  reg [17:0] lfsr=18'h1ACE1; always @(posedge oclk) lfsr<={lfsr[16:0],lfsr[17]^lfsr[10]^lfsr[7]^lfsr[0]};
  reg [5:0] et0=0,et1=0,et2=0; wire eopA=wrapA&(echA==N-1);     // random error chips (mod N), 6-bit positions
  always @(posedge oclk) if (eopA) begin
    et0<=(lfsr[5:0]>=N)?6'd0:lfsr[5:0]; et1<=(lfsr[11:6]>=N)?6'd0:lfsr[11:6]; et2<=(lfsr[17:12]>=N)?6'd0:lfsr[17:12];
  end
  wire flipA = (inj1o[1]&(echA==et0)) | (inj2o[1]&((echA==et1)|(echA==et2)));
  wire codeA = CODE0[N-1-echA] ^ flipA;
  assign code_pin = codeA;                         // I/O14 P8: code A (post-corruption)
  assign sync_pin = (echA==6'd0);                  // I/O15 N8: code-A epoch

  // emitter B @ OSCH/EDIV_B (skewed -> steady slip vs A and vs the receiver)
  wire [18:0] EDIV_B = edivb_o2;                             // emitter-B rate: DIP4 skew (local) or 'F' cmd (remote)
  reg [18:0] edcB=0; reg [5:0] echB=0; wire wrapB=(edcB>=EDIV_B-1);
  always @(posedge oclk) if (wrapB) begin edcB<=0; echB<=(echB==N-1)?6'd0:echB+1'b1; end else edcB<=edcB+1'b1;
  reg [5:0] et0B=0,et1B=0,et2B=0; wire eopB=wrapB&(echB==N-1);   // B's own random error positions (mod N)
  always @(posedge oclk) if (eopB) begin
    et0B<=(lfsr[5:0]>=N)?6'd0:lfsr[5:0]; et1B<=(lfsr[11:6]>=N)?6'd0:lfsr[11:6]; et2B<=(lfsr[17:12]>=N)?6'd0:lfsr[17:12];
  end
  wire flipB = (inj1o[1]&(echB==et0B)) | (inj2o[1]&((echB==et1B)|(echB==et2B)));
  wire codeB = CODE1[N-1-echB] ^ flipB;                       // inj corrupts BOTH codes (independent positions)

  // ===== burst dropout (A4d-3): blank `burst` CONSECUTIVE chips once per BURST_WINDOW -> isolated occlusion =====
  // (distinct from the spread 1/2-bit K1/K2 injectors). Models a beam-block / sun glint of a measured span; the
  // FSM HOLD + flywheel must coast it. Both beacons squelched together (whole-scene occlusion). 'K' value = chips.
  localparam integer BURST_WINDOW = 8*N;                      // ≈2.5 s @ N=31: one isolated burst per window
  reg [9:0] dropcnt=0;
  always @(posedge oclk) if (wrapA) dropcnt <= (dropcnt>=BURST_WINDOW-1) ? 10'd0 : dropcnt+1'b1;
  wire blanked = (dropcnt < {2'b0, burst_o2});               // first `burst` chips of each window: signal gone

  // ============ analog front end: sum enabled sources, band-limit (ramp), add noise, clamp to 12-bit ============
  reg [1:0] cAs=0,cBs=0,blk_s=0; always @(posedge clk12) begin cAs<={cAs[0],codeA}; cBs<={cBs[0],codeB}; blk_s<={blk_s[0],blanked}; end
  wire codeA_rx=cAs[1], codeB_rx=cBs[1], blank_rx=blk_s[1];   // blank_rx = burst occlusion active (both beacons)
  // ANALOG per-source magnitudes (settable over USB: 'A'/'B'/'G' + value -> ×6). Graded + headroom so the sum
  // is LINEAR in normal use (the 12-bit clamp is just ADC saturation at extremes), not a worst-case all-rails OR.
  localparam signed [15:0] PED=16'sd1800;
  localparam [11:0] AMPA_DEF=12'd750, AMPB_DEF=12'd750, AMPN_DEF=12'd400;   // reset/LOCAL defaults (symmetric)
  reg [11:0] ampA_r=AMPA_DEF, ampB_r=AMPB_DEF, ampN_r=AMPN_DEF;  // 'A'/'B'/'G' command targets (REMOTE only)
  // LOCAL mode uses the reset defaults so manual operation == power-on (the 'A'/'B'/'G' trims apply only in REMOTE)
  wire [11:0] ampA_e = remote ? ampA_r : AMPA_DEF, ampB_e = remote ? ampB_r : AMPB_DEF, ampN_e = remote ? ampN_r : AMPN_DEF;
  wire [11:0] ampA = eweak ? {2'b0,ampA_e[11:2]} : ampA_e;       // weak (K3) -> ~1/4 amplitude (range/aspect)
  wire [11:0] ampB = eweak ? {2'b0,ampB_e[11:2]} : ampB_e;
  wire signed [15:0] sigA = (enA & ~blank_rx) ? (codeA_rx ? $signed({4'b0,ampA}) : -$signed({4'b0,ampA})) : 16'sd0;
  wire signed [15:0] sigB = (enB & ~blank_rx) ? (codeB_rx ? $signed({4'b0,ampB}) : -$signed({4'b0,ampB})) : 16'sd0;
  wire signed [16:0] sigsum = PED + sigA + sigB;                 // pedestal + analog signals
  // band-limit (analog LPF -> ramped chip edges); IIR at clk12/8, 17.8 fixed-point
  localparam integer LPF_SH = 9;
  reg signed [25:0] aflt=0; reg [2:0] lpf_div=0;
  wire signed [25:0] tgt = {sigsum, 8'b0};
  always @(posedge clk12) begin lpf_div<=lpf_div+1'b1; if (lpf_div==3'd7) aflt<=aflt+((tgt-aflt)>>>LPF_SH); end
  wire signed [16:0] sig_bl = aflt[25:8];
  // white noise per sample + optional DC floor
  reg [15:0] nlfsr=16'h1234; always @(posedge clk12) nlfsr<={nlfsr[14:0],nlfsr[15]^nlfsr[13]^nlfsr[12]^nlfsr[10]};
  wire signed [11:0] nctr = $signed({1'b0,nlfsr[10:0]}) - 12'sd1024;                 // ±1023 continuous random
  wire signed [24:0] nscaled = nctr * $signed({1'b0,ampN_e});                        // scale to ±ampN (analog level)
  wire signed [16:0] nz = enN ? (nscaled >>> 10) : 17'sd0;
  wire signed [16:0] fl = efloor ? 17'sd400 : 17'sd0;
  wire signed [18:0] adc_s = sig_bl + nz + fl;
  wire [11:0] adc_level = (adc_s < 0) ? 12'd0 : (adc_s > 19'sd4095) ? 12'd4095 : adc_s[11:0];

  // ============ virtual MCP3201 + soft SPI reader (480 Hz sample) ============
  localparam integer FDIV = 25000;
  reg [15:0] fdiv = 0; reg fetch = 1'b0;
  always @(posedge clk12)
    if (fdiv == FDIV-1) begin fdiv <= 0; fetch <= 1'b1; end else begin fdiv <= fdiv+1'b1; fetch <= 1'b0; end
  wire cs, sclk, dout, valid; wire [11:0] sample;
  mcp3201_model              adc (.clk(clk12), .cs(cs), .sclk(sclk), .value(adc_level), .dout(dout));
  spi_mcp3201_reader #(.SCLK_HALF(120)) rdr (.clk(clk12), .start(fetch), .miso(dout),
                                             .cs(cs), .sclk(sclk), .sample(sample), .valid(valid));
  assign spi_cs = cs; assign spi_sclk = sclk; assign spi_do = dout;

  // ============ dual correlator (DUT) ============
  function [6:0] seg7(input [3:0] q);                     // threeN1 map: {a,b,c,d,e,f,g} bit6..0, active-high
    case (q)
      4'd0: seg7=7'b1111110; 4'd1: seg7=7'b0110000; 4'd2: seg7=7'b1101101; 4'd3: seg7=7'b1111001;
      4'd4: seg7=7'b0110011; 4'd5: seg7=7'b1011011; 4'd6: seg7=7'b1011111; 4'd7: seg7=7'b1110000;
      4'd8: seg7=7'b1111111; default: seg7=7'b1111011;
    endcase
  endfunction
  function [3:0] bar4(input [3:0] q);
    bar4 = (q>=4'd8)?4'hF : (q>=4'd6)?4'h7 : (q>=4'd4)?4'h3 : (q>=4'd2)?4'h1 : 4'h0;
  endfunction

  reg [19:0] dc_acc = 0; wire [11:0] dc = dc_acc[19:8];
  integer i; reg [11:0] win [0:L-1];
  // CLOSED-LOOP DPLL: per-code effective period Leff = L + (measured slip) -> the template's slot->chip advance
  // (Bresenham) tracks the emitter's actual rate, keeping the long-window matched filter coherent under skew.
  // Two passes (code 0 then code 1), each with its own Leff; energy (template-independent) computed in pass 0.
  reg [7:0] ai = 0; reg busy = 0, rdy = 0, pass = 0;       // ai 0..L-1; 8-bit sized for N≤63 (L≤151)
  reg [5:0] cchip = 0; reg [8:0] cacc = 0;                 // cchip 0..N-1; 6-bit sized for N≤63
  reg signed [21:0] acc_c=0, corr0=0, corr1=0;
  reg        [21:0] acc_e=0, energy=0;
  // [A4d-2 partial/progressive candidate INVESTIGATED & DEFERRED — see DESIGN.md §5: a ½-window candidate cannot
  // be both early AND low-false-alarm at N=31 (½-window 3 dB gain deficit + a partial Gold code loses the full
  // code's cross-corr bound → 50–70 % false-candidate on noise/wrong-code). The robust 3-level ladder ships;
  // revisit the candidate at N=63 where the half is a full 31-chip code.]
  wire signed [17:0] sr0 = slip0 >>> SLIP_SH, sr1 = slip1 >>> SLIP_SH;          // mean slip (samples/period)
  wire signed [5:0]  sc0 = (sr0 > 18'sd10) ? 6'sd10 : (sr0 < -18'sd10) ? -6'sd10 : sr0[5:0];   // clamp ±10 (N=31)
  wire signed [5:0]  sc1 = (sr1 > 18'sd10) ? 6'sd10 : (sr1 < -18'sd10) ? -6'sd10 : sr1[5:0];
  wire signed [9:0]  Leff0 = 10'sd74 + sc0, Leff1 = 10'sd74 + sc1;   // base L=74 (N=31)
  wire        [8:0]  Leff  = pass ? Leff1[8:0] : Leff0[8:0];
  wire tcode = pass ? CODE1[cchip] : CODE0[cchip];                              // time-reversed template
  wire signed [12:0] dev  = $signed({1'b0, win[ai]}) - $signed({1'b0, dc});
  wire signed [21:0] devx = dev;
  wire        [11:0] dabs = dev[12] ? (~dev[11:0] + 1'b1) : dev[11:0];
  always @(posedge clk12) begin
    rdy <= 1'b0;
    if (valid) begin
      for (i=L-1; i>0; i=i-1) win[i] <= win[i-1];
      win[0] <= sample;
      dc_acc <= dc_acc - {8'b0, dc} + {8'b0, sample};
      ai<=0; cchip<=0; cacc<=0; acc_c<=0; acc_e<=0; pass<=1'b0; busy<=1'b1;
    end else if (busy) begin
      acc_c <= acc_c + (tcode ? devx : -devx);
      if (~pass) acc_e <= acc_e + {10'b0, dabs};                               // energy in pass 0 only
      if (cacc + N >= Leff) begin cacc <= cacc + N - Leff; if (cchip < N-1) cchip <= cchip + 1'b1; end
      else cacc <= cacc + N;
      if (ai == L-1) begin
        if (~pass) begin corr0<=acc_c; energy<=acc_e; pass<=1'b1; ai<=0; cchip<=0; cacc<=0; acc_c<=0; end
        else       begin corr1<=acc_c; rdy<=1'b1; busy<=1'b0; end
      end else ai <= ai + 1'b1;
    end
  end

  wire signed [21:0] a0 = corr0[21] ? -corr0 : corr0;
  wire signed [21:0] a1 = corr1[21] ? -corr1 : corr1;
  reg [21:0] pk0=0, pk1=0, pke=1, best0=0, best1=0, beste=1;
  reg [7:0]  pkph0=0, pkph1=0, bestph0=0, bestph1=0;     // peak phase 0..L-1; 8-bit sized for N≤63
  reg [7:0]  pcnt=0; reg pend=0;                          // phase counter 0..L-1; 8-bit sized for N≤63
  always @(posedge clk12) begin
    pend <= 1'b0;
    if (rdy) begin
      if (a0 > pk0) begin pk0 <= a0; pkph0 <= pcnt; end
      if (a1 > pk1) begin pk1 <= a1; pkph1 <= pcnt; end
      if (energy > pke) pke <= energy;
      if (pcnt == L-1) begin
        best0 <= (a0>pk0)?a0:pk0; best1 <= (a1>pk1)?a1:pk1;
        bestph0 <= (a0>pk0)?pcnt:pkph0; bestph1 <= (a1>pk1)?pcnt:pkph1;
        beste <= ((energy>pke)?energy:pke) | 22'd1;
        pk0<=0; pk1<=0; pke<=0; pkph0<=0; pkph1<=0; pcnt<=0; pend<=1'b1;
      end else pcnt <= pcnt + 1'b1;
    end
  end

  // quality q in 0-9 = min(9, 9*|corr|/energy) -- ACTUAL match level per code (matched ~9, other ~cross/noise)
  wire [24:0] n0_raw = (best0<<3)+(best0<<1);       // 10*best (realistic peak ratio ~0.9 -> full-scale 9)
  wire [24:0] n1_raw = (best1<<3)+(best1<<1);
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

  // per-code lock FSM: min-lock to confirm, limited hold across short dropouts, then re-acquire
  localparam [3:0] GOOD = 4'd5;        // N=31: cross-corr floor drops to q~3, so a lower threshold holds A+B
  localparam [2:0] MINLOCK = 3'd2, HOLDMAX = 3'd2;
  localparam [1:0] SEARCH=2'd0, ACQ=2'd1, LOCK=2'd2, HOLD=2'd3;
  reg [1:0] st0=SEARCH, st1=SEARCH; reg [2:0] g0=0,b0=0,g1=0,b1=0;
  // frequency flywheel: coast counts periods since lock; while < COASTMAX the rate is still held -> WARM
  // re-acquire (confirm on the FIRST good period instead of MINLOCK). Init stale so the first lock is COLD.
  // Coast window is WALLCLOCK-driven (set by emitter↔rx osc stability, NOT code length): hold the rate only as
  // long as drift stays < ~½ chip. ~10 s with the OSCH RC -> 65 periods @ ~154 ms for N=31 (32 @ ~315 ms at N=63).
  // A stable xtal (20–50 ppm) on both ends would extend this dramatically -> far longer fast-re-acquire.
  localparam [7:0] COASTMAX = 8'd65;
  reg [7:0] coast0=8'd133, coast1=8'd133;
  wire warm0 = (coast0 < COASTMAX), warm1 = (coast1 < COASTMAX);
  always @(posedge clk12) if (flush) begin st0<=SEARCH; coast0<=COASTMAX; g0<=0; b0<=0; end
  else if (q0_rdy) begin
    if (st0==LOCK) coast0<=8'd0; else if (coast0<COASTMAX) coast0<=coast0+1'b1;
    if (q0 >= GOOD) case (st0)
      SEARCH: if (warm0) st0<=LOCK; else begin st0<=ACQ; g0<=3'd1; end   // warm: 1-period re-lock (held rate)
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
  always @(posedge clk12) if (flush) begin st1<=SEARCH; coast1<=COASTMAX; g1<=0; b1<=0; end
  else if (q1_rdy) begin
    if (st1==LOCK) coast1<=8'd0; else if (coast1<COASTMAX) coast1<=coast1+1'b1;
    if (q1 >= GOOD) case (st1)
      SEARCH: if (warm1) st1<=LOCK; else begin st1<=ACQ; g1<=3'd1; end   // warm: 1-period re-lock (held rate)
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

  // ============ displays ============
  reg [22:0] blink = 0; always @(posedge clk12) blink <= blink + 1'b1;
  localparam [3:0] GREEN = 4'd8;                          // q>=GREEN = strong lock (green); GOOD..GREEN-1 = yellow
  function [2:0] rgb(input [1:0] st, input [3:0] q, input bk);   // color = lock HEALTH
    rgb = (st==LOCK) ? (q>=GREEN ? 3'b010 : 3'b011)             // locked: green if strong, yellow if marginal
        : (st==HOLD) ? (bk ? (q>=GREEN?3'b010:3'b011) : 3'b000) // hold: blink the lock color (coasting)
        : (st==ACQ)  ? 3'b011                                   // acquiring (brief)
        :              3'b001;                                  // search = red
  endfunction
  wire [2:0] rmt = remote ? 3'b100 : 3'b000;             // blue tint while REMOTE (USB override active)
  // PWM the RGB channels to balance color (green/blue out-shine red, so R+G read green) + dim for the eyes.
  // Tunable: lower duty = dimmer channel. ~47 kHz (flicker-free).
  reg [7:0] pwm = 0; always @(posedge clk12) pwm <= pwm + 1'b1;
  localparam [7:0] DUTY_R = 8'd255, DUTY_G = 8'd128, DUTY_B = 8'd128;   // R full, G/B half-intensity (tweakable)
  wire [2:0] pgate = {pwm < DUTY_B, pwm < DUTY_G, pwm < DUTY_R};      // {B,G,R} brightness windows
  assign LEDl = ~((rgb(st0, q0, blink[21]) | rmt) & pgate);
  assign LEDr = ~((rgb(st1, q1, blink[21]) | rmt) & pgate);
  assign LEDs = ~{bar4(q0), bar4(q1)};
  assign d1   = seg7(q1);            // right digit = code B (threeN1: d1 is the RIGHT digit)
  assign d2   = seg7(q0);            // left  digit = code A (q0) — matches LEDl (left = code A)
  assign enableLd1 = 1'b0;
  assign enableLd2 = 1'b0;

  // ============ command RX (UART on pad A3): '+'=remote on, '-'=off, 0x80|mask -> 7-bit knobs ============
  wire [7:0] rxb; wire rxv;
  uart_rx #(.DIV(104)) u_rx (.clk(clk12), .rxd(rxd), .data(rxb), .valid(rxv));
  reg [7:0] pend_op = 8'h00;                       // opcode awaiting its value byte
  wire signed [19:0] fval = 20'sd266000 - ($signed({12'b0, rxb}) - 20'sd128) * 20'sd200;  // 'F' value -> rate
  wire [11:0] amp6 = (rxb << 2) + (rxb << 1);      // value -> amplitude ×6 (0..1530)
  always @(posedge clk12) if (rxv) begin
    if (pend_op != 8'h00) begin
      case (pend_op)
        8'h45: ediva_cmd <= fval;                  // 'E' emitter-A frequency (skew A, rate test)
        8'h46: edivb_cmd <= fval;                  // 'F' emitter-B frequency (rate test)
        8'h41: ampA_r   <= amp6;                   // 'A' code-A analog magnitude
        8'h42: ampB_r   <= amp6;                   // 'B' code-B analog magnitude
        8'h47: ampN_r   <= amp6;                   // 'G' noise analog magnitude
        8'h4B: burst_cmd <= rxb;                   // 'K' dropout-burst span (chips)
      endcase
      pend_op <= 8'h00;
    end
    else if (rxb == 8'h2B) remote <= 1'b1;         // '+'  REMOTE (USB owns)
    else if (rxb == 8'h2D) remote <= 1'b0;         // '-'  LOCAL (switches own)
    else if (rxb==8'h45||rxb==8'h46||rxb==8'h41||rxb==8'h42||rxb==8'h47||rxb==8'h4B) pend_op <= rxb;  // value-taking opcodes
    else if (rxb[7])       cmd_reg <= rxb[6:0];    // 0x80-0xFF -> 7-bit knob mask
  end
  // 'Z' = flush flywheel (true cold). Gated on pend_op==0 so a VALUE byte that happens to be 0x5A (e.g. 'F'/'A'/
  // 'B'/'G' arg = 90) is consumed as the arg above and does NOT spuriously flush.
  reg flush = 1'b0; always @(posedge clk12) flush <= (rxv && rxb == 8'h5A && pend_op == 8'h00);

  // ============ per-code DPLL: rate estimate from peak-phase slip (the frequency flywheel) ============
  // IIR mean of the per-period peak-phase delta (samples/period); updates only while LOCKED -> FROZEN (held)
  // through outages = the flywheel. Reported offset-binary: rate = 32768 + 32·(mean slip). Host recovers:
  //   slip = (rate-32768)/32 ;  chip_rate_Hz = N·480 / (L + slip)   [N=31, L=74; faster emitter -> peak earlier
  //   -> negative slip -> higher chip_rate]. (See host/beacon_telemetry/frame.py chip_rate_hz().)
  localparam integer SLIP_SH = 5;                        // IIR ~32 periods (~2.4 s) of averaging
  reg pend_d = 0; always @(posedge clk12) pend_d <= pend;
  reg [7:0] prev0=0, prev1=0;                            // peak phase 0..L-1; 8-bit sized for N≤63
  reg signed [17:0] slip0=0, slip1=0;
  wire signed [9:0] raw0 = $signed({2'b0,bestph0}) - $signed({2'b0,prev0});   // ±(L-1) -> 10-bit signed
  wire signed [9:0] raw1 = $signed({2'b0,bestph1}) - $signed({2'b0,prev1});
  wire signed [9:0] dlt0 = (raw0 > L/2) ? raw0-L : (raw0 < -(L/2)) ? raw0+L : raw0;   // unwrap to +/- L/2
  wire signed [9:0] dlt1 = (raw1 > L/2) ? raw1-L : (raw1 < -(L/2)) ? raw1+L : raw1;
  wire lockedA = (st0==LOCK)||(st0==HOLD);
  wire lockedB = (st1==LOCK)||(st1==HOLD);
  wire signed [17:0] dlt0x = dlt0, dlt1x = dlt1;          // sign-extend (signed) — keep the IIR fully signed
  // FAST-ACQUIRE (snap-to-estimate): the slow IIR (~32 periods) crawls the rate estimate across the emitter↔rx
  // skew, so cold full-quality lags ~10 s at N=63 under RC-osc offset. On a COLD lock edge, SNAP slip to the
  // steady-state estimate (slip_ss ≈ dlt·2^SLIP_SH — HW-verified) so it lands at the rate in ~1 period; the IIR
  // then fine-tracks. WARM re-locks keep the flywheel's held slip (don't snap → no estimate thrown away).
  reg lockedA_d=0, lockedB_d=0;
  always @(posedge clk12) if (flush) begin slip0<=0; slip1<=0; lockedA_d<=0; lockedB_d<=0; end
  else if (pend_d) begin
    prev0 <= bestph0; prev1 <= bestph1; lockedA_d <= lockedA; lockedB_d <= lockedB;
    if      (lockedA & ~lockedA_d & ~warm0) slip0 <= dlt0x <<< SLIP_SH;          // cold lock edge -> snap
    else if (lockedA)                       slip0 <= slip0 - (slip0>>>SLIP_SH) + dlt0x;  // track (flywheel when unlocked)
    if      (lockedB & ~lockedB_d & ~warm1) slip1 <= dlt1x <<< SLIP_SH;
    else if (lockedB)                       slip1 <= slip1 - (slip1>>>SLIP_SH) + dlt1x;
  end
  wire signed [18:0] r0raw = 19'sd32768 + slip0;
  wire signed [18:0] r1raw = 19'sd32768 + slip1;
  wire [15:0] rateA = (r0raw<0)?16'd0:(r0raw>19'sd65535)?16'd65535:r0raw[15:0];
  wire [15:0] rateB = (r1raw<0)?16'd0:(r1raw>19'sd65535)?16'd65535:r1raw[15:0];

  // ============ recovery counters: SAMPLES from signal-return to confirmed lock (precise, measured on-chip
  //              since USB can't run fast). q>=SIG_TH = "signal present" (above the q~3 floor). reclat holds the
  //              latency of the LAST (re)acquisition; host: ms = rec*1000/480, chips = rec/2.4 ============
  localparam [3:0] SIG_TH = 4'd4;
  reg [15:0] reccntA=0, reclatA=0, reccntB=0, reclatB=0; reg recingA=0, recingB=0;
  always @(posedge clk12) begin
    if (valid & recingA) reccntA <= reccntA + 1'b1;
    if (q0_rdy) begin
      if (st0==LOCK) begin if (recingA) begin reclatA<=reccntA; recingA<=1'b0; end end   // locked -> latch
      else if (q0 >= SIG_TH) begin if (~recingA) begin recingA<=1'b1; reccntA<=16'd0; end end  // signal present -> start
      else recingA <= 1'b0;                                                              // no signal -> idle
    end
  end
  always @(posedge clk12) begin
    if (valid & recingB) reccntB <= reccntB + 1'b1;
    if (q1_rdy) begin
      if (st1==LOCK) begin if (recingB) begin reclatB<=reccntB; recingB<=1'b0; end end
      else if (q1 >= SIG_TH) begin if (~recingB) begin recingB<=1'b1; reccntB<=16'd0; end end
      else recingB <= 1'b0;
    end
  end

  // ============ BCN telemetry (UART TX on pad A2 -> STEPLink -> COM3) ============
  reg [13:0] seq = 0; reg [11:0] adc_l = 0;
  always @(posedge clk12) if (valid) adc_l <= sample;
  reg [18:0] tdiv = 0; reg tick = 0;
  always @(posedge clk12)                                // ~40 Hz (control-loop family)
    if (tdiv == 19'd299999) begin tdiv<=0; tick<=1'b1; seq<=(seq==14'd9999)?14'd0:seq+1'b1; end
    else begin tdiv<=tdiv+1'b1; tick<=1'b0; end
  // confidence ladder: 0=no_lock, 1=tentative (ACQ), 2=confirmed (LOCK/HOLD)
  wire [1:0] lka = (st0==SEARCH)?2'd0:(st0==ACQ)?2'd1:2'd2;
  wire [1:0] lkb = (st1==SEARCH)?2'd0:(st1==ACQ)?2'd1:2'd2;
  bcn_tx u_bcn (.clk(clk12), .tick(tick), .seq(seq), .adc(adc_l),
                .corrA(best0[19:0]), .lockA(lka), .marginA(q0),
                .corrB(best1[19:0]), .lockB(lkb), .marginB(q1),
                .rateA(rateA), .rateB(rateB), .recA(reclatA), .recB(reclatB), .txd(txd));
endmodule
