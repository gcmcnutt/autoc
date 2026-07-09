`timescale 1ns/1ps
// s6 correlator testbench (iverilog). Verifies the DUT decodes in SIMULATION — in particular that the new
// CIRCULAR-BUFFER RAM windows (wp/rp/rp0) reproduce the shift-register behavior, i.e. code A still LOCKS.
//   - code A: fully internal (OSCH emitter -> digital inject -> winA) — the primary canary.
//   - code B: a free-running CODE1 stimulus mapped to a 2-level analog value, fed through the bit-exact
//             mcp3201_model on the SPI bus (emulates the REAL wired MCP3201) -> winB. Bonus check.
// Build with +define+SIM so s6.v uses the ÷100 sim-scaled clock dividers (lock in ms of sim time, not seconds).
module tb_s6;
  // ---- 12 MHz correlator clock (period 83.333 ns) ----
  reg clk12 = 1'b0;
  always #41.667 clk12 = ~clk12;

  // ---- DUT control inputs. Switches pulled UP = 1 => ENABLED (OFF=enabled); buttons released = 1 (active-low) ----
  reg sw1 = 1'b1;    // DIP1 OFF -> enA = 1 (code A enabled)
  reg dip2 = 1'b1;   // DIP2 OFF -> enB = 1 (code B enabled)
  reg k1 = 1'b1, k2 = 1'b1;   // K1/K2 injectors released (no errors)
  reg rxd = 1'b1;    // UART idle high -> stays LOCAL (no USB override)

  // ---- DUT outputs ----
  wire spi_cs, spi_sclk, spi_do;
  wire [7:0] LEDs; wire [2:0] LEDl, LEDr; wire [6:0] d1, d2;
  wire enableLd1, enableLd2, sync_pin, txd;

  // ---- emulated REAL MCP3201 on the SPI bus, fed by the code-B stimulus level ----
  reg [11:0] adcB_level = 12'd2048;
  mcp3201_model u_adcB (.clk(clk12), .cs(spi_cs), .sclk(spi_sclk), .value(adcB_level), .dout(spi_do));

  // ---- DUT ----
  s6_top dut (
    .clk12(clk12), .sw1(sw1), .dip2(dip2), .k1(k1), .k2(k2),
    .sync_pin(sync_pin), .spi_cs(spi_cs), .spi_sclk(spi_sclk), .spi_do(spi_do),
    .LEDs(LEDs), .LEDl(LEDl), .LEDr(LEDr), .d1(d1), .d2(d2),
    .enableLd1(enableLd1), .enableLd2(enableLd2), .txd(txd), .rxd(rxd)
  );

  // ---- code-B stimulus: free-running CODE1 chip clock (~20 kHz sim-scaled), mapped to a 2-level ADC value.
  // Independent of the DUT's OSCH -> the code-B DPLL has to track it (realistic). Enabled after code A warms up.
  localparam [30:0] CODE1 = 31'b0100011001100111100101001011110;
  localparam integer NB = 31;
  reg [5:0] chipB = 0;
  reg codeB_en = 1'b0;
  real tchip = 50000.0;   // ns/chip (~20 kHz nominal); adjustable mid-run to skew the emitter clock
  real ampB  = 1.0;       // code-B amplitude scale (1.0 = full, <1 = weak/attenuated through the analog chain)
  initial forever begin #(tchip); chipB = (chipB==NB-1) ? 6'd0 : chipB + 1'b1; end
  // 2-level analog value about mid-scale; ampB scales the swing (models attenuation through the amp/range)
  always @* adcB_level = codeB_en ? (CODE1[30-chipB] ? 12'd2048 + $rtoi(1152.0*ampB)
                                                      : 12'd2048 - $rtoi(1152.0*ampB)) : 12'd2048;

  // ---- lock watchers (hierarchical refs). st: SEARCH=0 ACQ=1 LOCK=2 HOLD=3 ----
  reg aLocked = 0, bLocked = 0; integer tA = 0, tB = 0;
  initial begin wait (dut.st0 == 2'd2); aLocked = 1; tA = $time;
                $display("[%0t ns] code A LOCK  (q0=%0d)", $time, dut.q0); end
  initial begin wait (dut.st1 == 2'd2); bLocked = 1; tB = $time;
                $display("[%0t ns] code B LOCK  (q1=%0d)", $time, dut.q1); end

  // ---- progress strobe ----
  initial forever begin #1000000;
    $display("[%0t ns] A: q0=%0d st0=%0d | B: q1=%0d st1=%0d | codeB_en=%0d", $time, dut.q0, dut.st0, dut.q1, dut.st1, codeB_en);
  end

  // ---- track code-A margin while the injectors are pressed (K1/K2 active-low: pressed = 0) ----
  reg [3:0] q0_min_inj = 4'd15; reg injected = 0;
  reg bLock_skew = 0, bLock_weak = 0; reg [15:0] rateB_skew = 0;
  always @(posedge clk12) if (!k1 || !k2) begin injected <= 1; if (dut.q0 < q0_min_inj) q0_min_inj <= dut.q0; end

  // ---- run: A acquire -> bring up B -> inject errors on A -> recover; summarize ----
  initial begin
    $dumpfile("s6.vcd"); $dumpvars(0, tb_s6);
    #3000000  codeB_en = 1'b1;              // 3 ms: bring up code-B stimulus after A is acquiring
    #9000000;                               // ->12 ms: both locked strong
    $display("[%0t ns] --- inject: press K1+K2 (3 bit-flips/period into code A) ---", $time);
    k1 = 1'b0; k2 = 1'b0;                   // press both injectors
    #4000000;                               // ->16 ms under injection (margin should dip, lock should hold)
    k1 = 1'b1; k2 = 1'b1;                   // release
    $display("[%0t ns] --- inject off (release K1/K2) ---", $time);
    #4000000;                               // ->20 ms recover

    // ---- code-B clock SKEW: emitter RC-osc runs ~+5% fast (chip period -5%). DPLL must re-track + hold lock. ----
    $display("[%0t ns] --- skew: code-B chip period -5%% (emitter fast) ---", $time);
    tchip = 47500.0;                        // -5%
    #6000000;                               // ->26 ms: let the per-code DPLL re-track
    rateB_skew = dut.rateB; bLock_skew = (dut.st1 == 2'd2);
    $display("[%0t ns] after +5%% skew: B q1=%0d st1=%0d rateB=%0d (nominal 32768 -> shifts with rate)", $time, dut.q1, dut.st1, dut.rateB);

    // ---- code-B WEAK: attenuate the analog swing to 1/4 (long range). AGC-normalized quality must hold lock. ----
    $display("[%0t ns] --- weak: code-B amplitude -> 1/4 (attenuated through the chain) ---", $time);
    ampB = 0.25;
    #5000000;                               // ->31 ms
    bLock_weak = (dut.st1 == 2'd2);
    $display("[%0t ns] weak code B: q1=%0d st1=%0d corrB=(see telem)", $time, dut.q1, dut.st1);

    $display("======== s6 sim summary ========");
    $display("code A : %s  (q0=%0d st0=%0d)  lock@ %0t ns", aLocked?"LOCK   ":"NO-LOCK", dut.q0, dut.st0, tA);
    $display("code B : %s  (q1=%0d st1=%0d)  lock@ %0t ns", bLocked?"LOCK   ":"NO-LOCK", dut.q1, dut.st1, tB);
    $display("inject : K1/K2 drove code-A margin down to q0=%0d (from 9) — injector path %s",
             q0_min_inj, (injected && q0_min_inj < 4'd9) ? "WORKS" : "NO EFFECT");
    $display("skew   : code B held lock across +5%% emitter skew: %s (rateB %0d != nominal 32768)",
             bLock_skew ? "YES" : "NO", rateB_skew);
    $display("weak   : code B held lock at 1/4 amplitude (AGC): %s", bLock_weak ? "YES" : "NO");
    if (aLocked && bLocked && (q0_min_inj < 4'd9) && bLock_skew && bLock_weak)
         $display("RESULT: PASS  (RAM decode A+B; inject/skew/weak all handled)");
    else if (aLocked && bLocked)
         $display("RESULT: PARTIAL (A+B lock; check inject/skew/weak lines above)");
    else $display("RESULT: FAIL  (code A/B did not lock)");
    $finish;
  end
endmodule
