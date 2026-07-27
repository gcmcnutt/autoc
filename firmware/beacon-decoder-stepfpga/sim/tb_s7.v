`timescale 1ns/1ps
// s7 testbench: verifies the two s7 gateware changes against s6 behavior (build with +define+SIM):
//   A4d-8 gear-shifted DC tracker  -> a PEDESTAL+amplitude step on code B must re-lock FAST (< 4 ms sim
//                                     = 0.4 s real-equivalent; the s6 alpha=1/256 tracker needs ~16 ms sim).
//   A4d-7 min-energy lock gate     -> a long DARK window must produce ZERO lock=2 flashes after HOLD expires
//                                     (s6 spuriously flashed on energy-starved q).
// Plus the standing canaries: code A and code B both acquire and lock.
module tb_s7;
  reg clk12 = 1'b0;
  always #41.667 clk12 = ~clk12;

  reg sw1 = 1'b1, dip2 = 1'b1, k1 = 1'b1, k2 = 1'b1, rxd = 1'b1;
  wire spi_cs, spi_sclk, spi_do;
  wire [7:0] LEDs; wire [2:0] LEDl, LEDr; wire [6:0] d1, d2;
  wire enableLd1, enableLd2, sync_pin, txd;

  reg [11:0] adcB_level = 12'd2048;
  mcp3201_model u_adcB (.clk(clk12), .cs(spi_cs), .sclk(spi_sclk), .value(adcB_level), .dout(spi_do));

  s7_top dut (
    .clk12(clk12), .sw1(sw1), .dip2(dip2), .k1(k1), .k2(k2),
    .sync_pin(sync_pin), .spi_cs(spi_cs), .spi_sclk(spi_sclk), .spi_do(spi_do),
    .LEDs(LEDs), .LEDl(LEDl), .LEDr(LEDr), .d1(d1), .d2(d2),
    .enableLd1(enableLd1), .enableLd2(enableLd2), .txd(txd), .rxd(rxd)
  );

  // code-B stimulus: CODE1 at ~20 kHz sim chips; amplitude scale ampB; pedestal offset pedB; enable codeB_en
  localparam [30:0] CODE1 = 31'b0100011001100111100101001011110;
  localparam integer NB = 31;
  reg [5:0] chipB = 0; reg codeB_en = 1'b0;
  real ampB = 1.0; integer pedB = 0;
  initial forever begin #50000; chipB = (chipB==NB-1) ? 6'd0 : chipB + 1'b1; end
  always @* begin : lvl
    integer v;
    v = codeB_en ? (2048 + pedB + (CODE1[30-chipB] ? $rtoi(1152.0*ampB) : -$rtoi(1152.0*ampB)))
                 : 12'd2048;
    adcB_level = (v < 0) ? 12'd0 : (v > 4095) ? 12'd4095 : v[11:0];
  end

  // watchers
  reg aLocked = 0, bLocked = 0;
  initial begin wait (dut.st0 == 2'd2); aLocked = 1; $display("[%0t] code A LOCK (q0=%0d)", $time, dut.q0); end
  initial begin wait (dut.st1 == 2'd2); bLocked = 1; $display("[%0t] code B LOCK (q1=%0d)", $time, dut.q1); end
  initial forever begin #1000000;
    $display("[%0t] A q0=%0d st0=%0d | B q1=%0d st1=%0d amp=%0.2f ped=%0d en=%0d",
             $time, dut.q0, dut.st0, dut.q1, dut.st1, ampB, pedB, codeB_en);
  end

  // ---- A4d-8 measurement: time from step to q1>=GOOD sustained ----
  realtime t_step = 0, t_rec = 0;
  reg meas_arm = 0;
  always @(posedge clk12)
    if (meas_arm && dut.q1_rdy && dut.q1 >= 5) begin t_rec = $realtime; meas_arm = 0; end

  // ---- A4d-7 measurement: lock=2 flashes during the dark window (after HOLD expiry) ----
  integer dark_flashes = 0; reg dark_arm = 0; reg was_unlocked = 0;
  always @(posedge clk12)
    if (dark_arm) begin
      if (dut.st1 != 2'd2) was_unlocked <= 1;
      else if (was_unlocked) dark_flashes <= dark_flashes + 1;   // 2 again AFTER having dropped = false flash
    end

  initial begin
    $dumpfile("s7.vcd"); $dumpvars(0, tb_s7);
    #3000000  codeB_en = 1'b1;                 // 3 ms: code-B up
    #7000000;                                  // 10 ms: both should be locked
    // === A4d-8: pedestal (+500) + amplitude (x0.25) step ===
    $display("[%0t] --- STEP: pedestal +500, amplitude x0.25 ---", $time);
    t_step = $realtime; meas_arm = 1; ampB = 0.25; pedB = 500;
    #6000000;                                  // 16 ms: settle window
    if (t_rec > t_step)
      $display("A4d-8 recovery: %0.2f ms sim (%0.1f s real-equiv)  [target < 4 ms]",
               (t_rec-t_step)/1e6, (t_rec-t_step)/1e6*0.1);
    else $display("A4d-8 recovery: NOT RECOVERED in window");
    ampB = 1.0; pedB = 0;                      // restore
    #4000000;                                  // 20 ms
    // === A4d-7: long dark window ===
    $display("[%0t] --- DARK window 12 ms (1.2 s real-equiv) ---", $time);
    codeB_en = 0;
    #2000000 dark_arm = 1;                     // arm after HOLD has had time to expire
    #10000000 dark_arm = 0;
    codeB_en = 1;                              // 32 ms: light returns
    #6000000;                                  // 38 ms: relock check
    $display("======== s7 sim summary ========");
    $display("locks       : A=%s B=%s (end st1=%0d q1=%0d)", aLocked?"Y":"N", bLocked?"Y":"N", dut.st1, dut.q1);
    $display("A4d-8 settle: %s (%0.2f ms sim)", (t_rec>t_step && (t_rec-t_step)<4e6)?"PASS":"FAIL",
             (t_rec>t_step)?(t_rec-t_step)/1e6:-1.0);
    $display("A4d-7 dark  : %s (false lock flashes=%0d)", (dark_flashes==0)?"PASS":"FAIL", dark_flashes);
    $display("relock      : %s", (dut.st1==2'd2)?"PASS":"FAIL");
    if (aLocked && bLocked && dark_flashes==0 && dut.st1==2'd2 && t_rec>t_step && (t_rec-t_step)<4e6)
         $display("RESULT: PASS");
    else $display("RESULT: FAIL");
    $finish;
  end
endmodule
