// S6 -- TWO-STREAM correlator, code A now DIGITALLY INJECTED. Code A is a fully-synthetic reference on its own
// independent OSCH clock, but the fake-ADC round-trip is gone: instead of (analog model -> virtual MCP3201 -> soft
// SPI reader), the code-A chip is mapped straight to a 12-bit level and shoved into winA at the sample tick. Code B
// is unchanged -- the REAL wired MCP3201 read over SPI, its impairments coming from the physical link.
//   SPI (real ADC): SPI_CLK=M4 (out)  SPI_DO=N4 (IN)  SPI_CS=P3 (out)
// Split of concerns: code A = controlled/known test channel (impairments injected over USB); code B = real world
// (off-axis, obstruction, range -> real errors). The 7-seg digits still show code B's raw ADC upper byte as hex.
//
// I/O inventory:
//   INPUTS  : DIP1(sw1,M7)=enA (enable code A)   DIP2(M8)=enB (enable code-B/ADC decode)   [OFF=enabled]
//   OUTPUTS : LEDl/LEDr(RGB)=code A/B lock   8 LEDs=q bars (L nibble=A, R nibble=B)   d2:d1=real ADC hi-byte hex
//             sync_pin(N8)=code-A epoch   txd(A2)=BCN telemetry
//   BUTTONS : K1(L14)/K2(M13) = inject 1-/2-bit errors into code A (LOCAL); USB mask owns them in REMOTE
//   USB testability (RX on A3): '+'/'-' = remote/local (REMOTE = knobs live)
//             'E' <v> = code-A clock rate (skew -> exercise the DPLL/flywheel)
//             'A' <v> = code-A amplitude (HI/LO half-separation -> weak-signal margin sweep)
//             'K' <v> = code-A burst dropout span in chips (-> exercise HOLD + flywheel coast)
//             0x80|mask 7-bit knobs: [0]enA [1]enB [3]inj-1bit(A) [4]inj-2bit(A) [5]weak(A)
module s6_top (input clk12,
               input sw1, input dip2, input k1, input k2,       // DIP1=enA, DIP2=enB, K1/K2=inject 1-/2-bit errors (code A)
               output sync_pin,                                 // N8 = synthetic code-A epoch
               output spi_cs, output spi_sclk, input spi_do,    // SPI to REAL MCP3201: CS/SCLK out, DO in (code B)
               output [7:0] LEDs, output [2:0] LEDl, output [2:0] LEDr,
               output [6:0] d1, output [6:0] d2, output enableLd1, output enableLd2,
               output txd, input rxd);

  // ============ timing constants — SIM scales the clock dividers ÷100 (keeping their ratio, so samples/chip and
  // the whole decode datapath are bit-identical) so a lock lands in ~200k cycles instead of ~15M. SCLK_HALF scales
  // too so the SPI frame still fits inside one sample tick (frame ≈ 16·2·SCLK_HALF must be < FDIV). Synth (`ifndef
  // SIM`) uses the real hardware values. Define +define+SIM on the iverilog cmd line to select the sim set. ========
`ifdef SIM
  localparam integer EDIV_NOM = 2660, FDIV = 250, SCLK_HALF = 2;    // ÷100: chip 20 kHz, sample 48 kHz, SCLK 3 MHz
`else
  localparam integer EDIV_NOM = 266000, FDIV = 25000, SCLK_HALF = 120;  // real: chip 200 Hz, sample 480 Hz, SCLK 50 kHz
`endif

  // ============ control: physical switches/buttons (active-low, pulled up) + USB override ============
  reg [6:0] cmd_reg = 0; reg remote = 0;
  reg [1:0] dsw1=0, dsw2=0, kb1=0, kb2=0;           // switches: DIP1=enA, DIP2=enB; buttons: K1/K2=inject (active-low)
  always @(posedge clk12) begin
    dsw1<={dsw1[0],sw1}; dsw2<={dsw2[0],dip2}; kb1<={kb1[0],~k1}; kb2<={kb2[0],~k2};
  end
  // DIP "ON" label = DISABLE (mute) -> source enabled when the switch is OFF (pulled up = 1). Same for both.
  wire enA    = remote ? cmd_reg[0] : dsw1[1];      // DIP1: enable code A (synthetic reference)
  wire enB    = remote ? cmd_reg[1] : dsw2[1];      // DIP2: enable code B correlator (real-ADC decode)
  // code-A error injectors: LOCAL = K1/K2 pushbuttons; REMOTE = USB mask bits. weak = USB-only.
  wire einj1  = remote ? cmd_reg[3] : kb1[1];       // K1 -> inject 1 random bit-flip / period into code A
  wire einj2  = remote ? cmd_reg[4] : kb2[1];       // K2 -> inject 2 random bit-flips / period into code A
  wire eweak  = remote & cmd_reg[5];                // weak code A (smaller HI/LO separation) — USB-only
  reg signed [19:0] ediva_cmd = EDIV_NOM;           // 'E' emitter-A divider (skew A; LOCAL = nominal)
  wire [18:0] ediva_eff = remote ? ediva_cmd[18:0] : EDIV_NOM;   // 19-bit wire truncates the constant
  reg [7:0] burst_cmd = 0;                           // 'K' code-A burst-dropout span in chips (LOCAL = none)
  wire [7:0] burst_eff = remote ? burst_cmd : 8'd0;

  // ============ emitters: internal OSCH; A & B at independent divisors -> real inter-beacon slip ============
  wire oclk; OSCH #(.NOM_FREQ("53.2")) osc (.STDBY(1'b0), .OSC(oclk), .SEDSTDBY());
  localparam integer N = 31, L = 74;                       // code length (chips); window = round(2.4·N) samples
  localparam [30:0] CODE0 = 31'b0000000100011011000011001110011,   // N=31 Gold preferred pair (xcorr {-9,-1,7})
                    CODE1 = 31'b0100011001100111100101001011110;   // (N=63 codes in git @ 1bd3b4a if needed)
  // sync the USB knobs into the emitter (oclk) domain
  reg [1:0] inj1o=0, inj2o=0; reg [7:0] burst_o1=0, burst_o2=0;
  reg [18:0] ediva_o1=EDIV_NOM, ediva_o2=EDIV_NOM;
  always @(posedge oclk) begin
    inj1o<={inj1o[0],einj1}; inj2o<={inj2o[0],einj2};
    ediva_o1<=ediva_eff; ediva_o2<=ediva_o1; burst_o1<=burst_eff; burst_o2<=burst_o1;
  end

  // emitter A @ OSCH/EDIV_A (nominal 200 Hz, or skewed via USB 'E'), with USB-controlled random bit-error injection
  wire [18:0] EDIV_A = ediva_o2;
  reg [18:0] edcA=0; reg [5:0] echA=0; wire wrapA=(edcA>=EDIV_A-1);
  always @(posedge oclk) if (wrapA) begin edcA<=0; echA<=(echA==N-1)?6'd0:echA+1'b1; end else edcA<=edcA+1'b1;
  reg [17:0] lfsr=18'h1ACE1; always @(posedge oclk) lfsr<={lfsr[16:0],lfsr[17]^lfsr[10]^lfsr[7]^lfsr[0]};
  reg [5:0] et0=0,et1=0,et2=0; wire eopA=wrapA&(echA==N-1);     // pick random error chips (mod N) once per period
  always @(posedge oclk) if (eopA) begin
    et0<=(lfsr[5:0]>=N)?6'd0:lfsr[5:0]; et1<=(lfsr[11:6]>=N)?6'd0:lfsr[11:6]; et2<=(lfsr[17:12]>=N)?6'd0:lfsr[17:12];
  end
  wire flipA = (inj1o[1]&(echA==et0)) | (inj2o[1]&((echA==et1)|(echA==et2)));
  wire codeA = CODE0[N-1-echA] ^ flipA;                        // code A with injected bit errors ('1'/'2' knobs)
  assign sync_pin = (echA==6'd0);                  // I/O15 N8: synthetic code-A epoch (reference)
  // burst dropout: blank `burst` consecutive chips once per window -> occlusion test (HOLD + flywheel must coast)
  localparam integer BURST_WINDOW = 8*N;
  reg [9:0] dropcnt=0;
  always @(posedge oclk) if (wrapA) dropcnt <= (dropcnt>=BURST_WINDOW-1) ? 10'd0 : dropcnt+1'b1;
  wire blankedA = (dropcnt < {2'b0, burst_o2});

  // ============ code-A digital inject: chip -> 12-bit level straight into winA (no analog model / virtual ADC) ============
  reg [1:0] cAs=0, blkAs=0; always @(posedge clk12) begin cAs<={cAs[0],codeA}; blkAs<={blkAs[0],blankedA}; end
  wire codeA_rx=cAs[1], blankA_rx=blkAs[1];
  // amplitude = HI/LO half-separation about mid-scale; 'A' trim (REMOTE), 'weak' knob halves it.
  localparam [11:0] MID=12'd2048, AMPA_DEF=12'd1024;
  reg [11:0] ampA_r=AMPA_DEF;                                    // 'A' code-A amplitude target (REMOTE only)
  wire [11:0] ampA_e = remote ? ampA_r : AMPA_DEF;
  wire [11:0] ampA = eweak ? {1'b0,ampA_e[11:1]} : ampA_e;      // weak -> half separation
  wire [11:0] hiA = MID + ampA, loA = MID - ampA;               // clean 2-level code (matched filter loves sharp edges)
  wire [11:0] injA = (enA & ~blankA_rx) ? (codeA_rx ? hiA : loA) : MID;   // muted/occluded -> mid-scale (no signal)

  // ============ code-B MCP3201 front end (480 Hz sample) — the ONLY SPI reader now ============
  // Code A no longer round-trips through a virtual ADC (it injects digitally, above). Code B = the REAL wired
  // MCP3201 over SPI (CS=P3, SCLK=M4 drive out; DOUT=N4 spi_do IN). Its `valid` is the shared sample tick.
  reg [15:0] fdiv = 0; reg fetch = 1'b0;             // FDIV defined at top (SIM-scaled)
  always @(posedge clk12)
    if (fdiv == FDIV-1) begin fdiv <= 0; fetch <= 1'b1; end else begin fdiv <= fdiv+1'b1; fetch <= 1'b0; end
  wire csB, sclkB, validB; wire [11:0] sampleB;
  spi_mcp3201_reader #(.SCLK_HALF(SCLK_HALF)) rdrB (.clk(clk12), .start(fetch), .miso(spi_do),
                                              .cs(csB), .sclk(sclkB), .sample(sampleB), .valid(validB));
  assign spi_cs = csB; assign spi_sclk = sclkB;   // FPGA drives CS + SCLK to the real chip
  wire valid = validB;                            // the code-B ADC read is the sample tick for both streams

  // ============ dual correlator (DUT) ============
  function [6:0] seg7(input [3:0] h);                    // HEX glyph 0-F; threeN1 map {a,b,c,d,e,f,g} bit6..0, active-high
    case (h)
      4'h0: seg7=7'b1111110; 4'h1: seg7=7'b0110000; 4'h2: seg7=7'b1101101; 4'h3: seg7=7'b1111001;
      4'h4: seg7=7'b0110011; 4'h5: seg7=7'b1011011; 4'h6: seg7=7'b1011111; 4'h7: seg7=7'b1110000;
      4'h8: seg7=7'b1111111; 4'h9: seg7=7'b1111011; 4'hA: seg7=7'b1110111; 4'hB: seg7=7'b0011111;
      4'hC: seg7=7'b1001110; 4'hD: seg7=7'b0111101; 4'hE: seg7=7'b1001111; 4'hF: seg7=7'b1000111;
    endcase
  endfunction
  function [3:0] bar4(input [3:0] q);
    bar4 = (q>=4'd8)?4'hF : (q>=4'd6)?4'h7 : (q>=4'd4)?4'h3 : (q>=4'd2)?4'h1 : 4'h0;
  endfunction

  reg [19:0] dcA_acc = 0; wire [11:0] dcA = dcA_acc[19:8];   // code-A running DC baseline
  reg [19:0] dcB_acc = 0; wire [11:0] dcB = dcB_acc[19:8];   // code-B (real ADC) running DC baseline
  // sample windows as CIRCULAR-BUFFER distributed RAM (async read) — replaces the two L-deep FF shift registers
  // + their 74:1 read muxes. wp = write ptr; rp = read ptr (walks newest->oldest, mirroring ai); rp0 = newest.
  // Async read keeps the accumulation FSM's timing byte-identical to the shift-reg version (curr = win[rp] == old win[ai]).
  reg [11:0] winA [0:L-1]; reg [11:0] winB [0:L-1];
  reg [6:0] wp=0, rp=0, rp0=0;                               // 0..L-1 pointers (L=74 -> 7-bit)
  // CLOSED-LOOP DPLL: per-code effective period Leff = L + (measured slip) -> the template's slot->chip advance
  // (Bresenham) tracks the emitter's actual rate, keeping the long-window matched filter coherent under skew.
  // Two passes (code 0 then code 1), each with its own Leff; energy (template-independent) computed in pass 0.
  reg [7:0] ai = 0; reg busy = 0, rdy = 0, pass = 0;       // ai 0..L-1; 8-bit sized for N≤63 (L≤151)
  reg [5:0] cchip = 0; reg [8:0] cacc = 0;                 // cchip 0..N-1; 6-bit sized for N≤63
  reg signed [21:0] acc_c=0, corr0=0, corr1=0;
  reg        [21:0] acc_e=0, energyA=0, energyB=0;    // per-stream energy: A from winA (pass 0), B from winB (pass 1)
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
  wire        [11:0] dc   = pass ? dcB : dcA;                                   // per-stream DC baseline
  wire        [11:0] cur  = pass ? winB[rp] : winA[rp];                         // async RAM read; rp mirrors ai (newest->oldest)
  wire signed [12:0] dev  = $signed({1'b0, cur}) - $signed({1'b0, dc});
  wire signed [21:0] devx = dev;
  wire        [11:0] dabs = dev[12] ? (~dev[11:0] + 1'b1) : dev[11:0];
  wire        [11:0] sampB_g = enB ? sampleB : 12'd0;                          // DIP2 off -> code-B decode muted (raw ADC still displayed)
  always @(posedge clk12) begin
    rdy <= 1'b0;
    if (valid) begin
      winA[wp] <= injA; winB[wp] <= sampB_g;                                   // circular write (no shift): A=inject, B=real ADC
      rp <= wp; rp0 <= wp;                                                      // start reads at the just-written newest
      wp <= (wp==L-1) ? 7'd0 : wp+1'b1;
      dcA_acc <= dcA_acc - {8'b0, dcA} + {8'b0, injA};
      dcB_acc <= dcB_acc - {8'b0, dcB} + {8'b0, sampB_g};
      ai<=0; cchip<=0; cacc<=0; acc_c<=0; acc_e<=0; pass<=1'b0; busy<=1'b1;
    end else if (busy) begin
      acc_c <= acc_c + (tcode ? devx : -devx);
      acc_e <= acc_e + {10'b0, dabs};                                          // energy accumulates each pass (latched below)
      rp <= (rp==7'd0) ? (L-1) : rp-1'b1;                                      // walk newest -> oldest (mirrors ai)
      if (cacc + N >= Leff) begin cacc <= cacc + N - Leff; if (cchip < N-1) cchip <= cchip + 1'b1; end
      else cacc <= cacc + N;
      if (ai == L-1) begin
        if (~pass) begin corr0<=acc_c; energyA<=acc_e; pass<=1'b1; ai<=0; cchip<=0; cacc<=0; acc_c<=0; acc_e<=0; rp<=rp0; end
        else       begin corr1<=acc_c; energyB<=acc_e; rdy<=1'b1; busy<=1'b0; end
      end else ai <= ai + 1'b1;
    end
  end

  wire signed [21:0] a0 = corr0[21] ? -corr0 : corr0;
  wire signed [21:0] a1 = corr1[21] ? -corr1 : corr1;
  reg [21:0] pk0=0, pk1=0, pkeA=1, pkeB=1, best0=0, best1=0, besteA=1, besteB=1;
  reg [7:0]  pkph0=0, pkph1=0, bestph0=0, bestph1=0;     // peak phase 0..L-1; 8-bit sized for N≤63
  reg [7:0]  pcnt=0; reg pend=0;                          // phase counter 0..L-1; 8-bit sized for N≤63
  always @(posedge clk12) begin
    pend <= 1'b0;
    if (rdy) begin
      if (a0 > pk0) begin pk0 <= a0; pkph0 <= pcnt; end
      if (a1 > pk1) begin pk1 <= a1; pkph1 <= pcnt; end
      if (energyA > pkeA) pkeA <= energyA;
      if (energyB > pkeB) pkeB <= energyB;
      if (pcnt == L-1) begin
        best0 <= (a0>pk0)?a0:pk0; best1 <= (a1>pk1)?a1:pk1;
        bestph0 <= (a0>pk0)?pcnt:pkph0; bestph1 <= (a1>pk1)?pcnt:pkph1;
        besteA <= ((energyA>pkeA)?energyA:pkeA) | 22'd1;
        besteB <= ((energyB>pkeB)?energyB:pkeB) | 22'd1;
        pk0<=0; pk1<=0; pkeA<=0; pkeB<=0; pkph0<=0; pkph1<=0; pcnt<=0; pend<=1'b1;
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
      2'd0: if (pend) begin num <= n0_raw; den <= besteA; cnt <= 0; dstate <= 2'd1; end
      2'd1: if ((num >= {3'b0,den}) && (cnt < 4'd9)) begin num <= num - {3'b0,den}; cnt <= cnt + 1'b1; end
            else begin q0 <= cnt; q0_rdy <= 1'b1; num <= n1_raw; den <= besteB; cnt <= 0; dstate <= 2'd2; end
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
  // 7-seg digits repurposed for bench bring-up: HEX readout of the REAL ADC upper byte (adc[11:4]).
  // d2:d1 = 0x00..0xFF as the ADC input sweeps 0..Vref.  (Per-code quality still on the 8 bar LEDs + RGB lock.)
  assign d1   = seg7(adc_l[7:4]);    // right digit = real ADC bits[7:4]  (low  nibble of the upper byte)
  assign d2   = seg7(adc_l[11:8]);   // left  digit = real ADC bits[11:8] (high nibble of the upper byte)
  assign enableLd1 = 1'b0;
  assign enableLd2 = 1'b0;

  // ============ command RX (UART on pad A3): '+'=remote on, '-'=off, 0x80|mask -> 7-bit knobs ============
  wire [7:0] rxb; wire rxv;
  uart_rx #(.DIV(104)) u_rx (.clk(clk12), .rxd(rxd), .data(rxb), .valid(rxv));
  reg [7:0] pend_op = 8'h00;                       // opcode awaiting its value byte
  wire signed [19:0] fval = 20'sd266000 - ($signed({12'b0, rxb}) - 20'sd128) * 20'sd200;  // 'E' value -> code-A rate
  wire [11:0] amp6 = (rxb << 2) + (rxb << 1);      // value -> code-A amplitude (half-sep) ×6 (0..1530)
  always @(posedge clk12) if (rxv) begin
    if (pend_op != 8'h00) begin
      case (pend_op)                               // s6 USB value-opcodes (code-A testability): 'E' rate, 'A' amp, 'K' burst
        8'h45: ediva_cmd <= fval;                  // 'E' code-A clock rate (skew -> DPLL test)
        8'h41: ampA_r    <= amp6;                  // 'A' code-A amplitude (HI/LO half-separation)
        8'h4B: burst_cmd <= rxb;                   // 'K' code-A burst-dropout span (chips)
      endcase
      pend_op <= 8'h00;
    end
    else if (rxb == 8'h2B) remote <= 1'b1;         // '+'  REMOTE (USB owns: enA/enB + inj/weak via 0x80|mask)
    else if (rxb == 8'h2D) remote <= 1'b0;         // '-'  LOCAL (DIP1/DIP2 own; no impairments)
    else if (rxb==8'h45||rxb==8'h41||rxb==8'h4B) pend_op <= rxb;  // value-taking opcodes
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

  // s5: per-channel recovery-latency counters REMOVED (area reclaim for the dual-stream windows). They were a
  // telemetry nicety (samples from signal-return to confirmed lock); recA/recB now report 0. Restore at N=63 or
  // when the code-B correlator drives real light.

  // ============ BCN telemetry (UART TX on pad A2 -> STEPLink -> COM3) ============
  reg [13:0] seq = 0; reg [11:0] adc_l = 0;
  always @(posedge clk12) if (valid) adc_l <= sampleB;   // BCN 'adc' + 7-seg = the RAW real ADC (watch 0..4095 / 0x00..0xFF)
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
                .rateA(rateA), .rateB(rateB), .recA(16'd0), .recB(16'd0), .txd(txd));
endmodule
