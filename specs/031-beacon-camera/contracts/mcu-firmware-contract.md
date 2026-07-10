# Contract — ATtiny412 Beacon-Pod Firmware

**Spec ref**: FR-1.3 (Gold-code generation), FR-1.2 (driver topology), FR-1.7 (UVLO contract)
**Reference impl**: `firmware/beacon-pod/src/`

## Single-line description

> When V_BAT is present, the MCU clocks out one bit of a 15-chip Gold-code LUT to a single open-drain DIM-pin GPIO every 10 ms. The supervisor open-drain output is wired to the same DIM node — it pulls DIM LOW at UVLO regardless of MCU state, forcing the LM3410X into ~80 nA shutdown. Nothing else.

## Boot sequence

| Step | Action | Bound |
|---|---|---|
| 1 | MCU resets on V_BAT rise above ~1.8 V | <10 ms after JST-PH mate |
| 2 | Configure internal 20 MHz RC oscillator; set CCP-protected CPU clock to 20 MHz (no prescaler) | <5 ms |
| 3 | Configure GPIO directions: **DIM-out = open-drain emulated** (DIR=input by default = high-Z; PORT.OUT=0 prearmed so any DIR=output flips it low); 2× code-select = input-pull-up; diagnostic-LED = output-low (push-pull) | <1 ms |
| 4 | Sample the 2-bit code-select once; choose one of 4 N=15 Gold-code LUTs (compiled into flash) | <1 ms |
| 5 | Configure TCA0 in single-shot 16-bit mode: period = 20 MHz / 100 Hz = 200 000 counts → 100 Hz tick rate. Enable TCA0 overflow interrupt | <1 ms |
| 6 | Set `chip_idx = 0`. Write `LUT[selected_code][0]` to DIM (DIRCLR for chip=1 release; DIRSET for chip=0 drive LOW) and diagnostic-LED GPIO. Start TCA0 | T = 0 |
| 7 | Enter `sei()` + `while(1) { sleep_cpu(); }` idle loop. Wake-from-sleep on every TCA0 interrupt | continuous |

**Total boot time**: <20 ms from V_BAT rise → first chip emission. **Spec gate FR-1.7 #1: ≤100 ms.** Comfortable margin.

## Steady-state per-tick behavior (TCA0 ISR)

```c
ISR(TCA0_OVF_vect) {
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;  // clear flag
    chip_idx = (chip_idx + 1) % 15;
    uint8_t bit = LUT[selected_code][chip_idx];

    // Code-epoch SYNC marker on the spare GPIO (PA7) — bench scope aid only:
    // HIGH for chip 0, LOW otherwise → one pulse per code period to trigger the
    // scope on and time-align the Gold-code epoch (and the receiver capture).
    if (chip_idx == 0) SYNC_PORT.OUTSET = (1 << SYNC_PIN);
    else               SYNC_PORT.OUTCLR = (1 << SYNC_PIN);

    // DIM is open-drain emulated: chip=1 releases (DIR=input → high-Z, pull-up
    // holds DIM HIGH); chip=0 drives LOW (DIR=output + PORT.OUT was prearmed
    // LOW at boot). This is the wired-AND topology with the supervisor's
    // open-drain output — no DC contention path.
    if (bit) {
        DIM_PORT.DIRCLR = (1 << DIM_PIN);     // release DIM → pull-up takes it HIGH
        DIAG_PORT.OUTSET = (1 << DIAG_LED_PIN); // push-pull: LED on
    } else {
        DIM_PORT.DIRSET = (1 << DIM_PIN);     // drive DIM LOW (OUT was prearmed = 0)
        DIAG_PORT.OUTCLR = (1 << DIAG_LED_PIN); // push-pull: LED off
    }
    // total ISR cost: ~10 cycles at 20 MHz = 500 ns. Negligible vs 10 ms tick.
}
```

No PWM peripheral. No analog. No floating-point. No malloc. No watchdog reset path needed (firmware is stateless across reset). DIM uses open-drain emulation via the DIRCLR/DIRSET pair (a 1-cycle pin-mode flip per chip transition); the diagnostic LED is push-pull on a separate pin.

## DIM-pin signal contract

| Property | Value |
|---|---|
| Drive mode | Open-drain emulated (DIRCLR=release / DIRSET=drive-LOW with PORT.OUT prearmed to 0) — joins supervisor + pull-up in wired-AND on DIM node |
| Logic level | 0 V (chip=0, drive-LOW) or V_BAT-pulled-HIGH ≈ 3.7 V (chip=1, release) |
| Drive strength (LOW) | ATtiny412 GPIO default (~20 mA sink) — far more than the LM3410X DIM pin's ~100 nA input current |
| Slew rate | Pull-up RC (10 kΩ × ~10 pF DIM parasitic + supervisor leakage ≈ 100 ns rise) on chip=1; native GPIO edge (~10 ns) on chip=0 — both negligible vs 10 ms chip period |
| Chip period | 10 ms ± internal-RC drift (see NFR-4) |
| Duty per code period | 7–8 ON chips of 15 (Gold-code-derived; specific value per selected LUT) |

## Code-select-pin contract

| Property | Value |
|---|---|
| Pull-direction | Internal pull-up enabled on MCU side |
| External | Solder jumper to GND for "0", float for "1" (or a 2-bit DSM-02 DIP switch) |
| Sample time | Once at boot only (no runtime re-read) |
| Encoding | LSB at one pin, MSB at the other → 4 codes (0..3); Phase 1 pods are 0 + 1 |

## Scope-sync pin (bench-monitoring aid)

| Property | Value |
|---|---|
| Pin | The remaining free 412 GPIO — **PA7** in the pod pin map (the pin not used by DIM / 2× code-select / diagnostic-LED / UPDI). *(On the XNANO eval, PA7 is currently code-select; free a pin by putting code-select on jumpers/PA1-PA2 or dropping the UART, or just probe whichever GPIO is spare in your build.)* |
| Drive | Push-pull output; in the TCA0 ISR, **HIGH during chip 0, LOW otherwise** → one pulse per 15-chip code period, aligned to the code epoch (at 200 Hz: a 5 ms pulse every 75 ms). |
| Purpose | **Oscilloscope trigger** — trigger on the SYNC edge for a stable, epoch-aligned view of the Gold code on DIM, and to time-align the receiver/ADC capture to the code start. Pure bench aid; **not part of the optical link** and not populated on a flight pod. |
| Cost | ~2 ISR cycles; no extra hardware (route to a test pad / header pin). |

## Diagnostic-LED contract

| Property | Value |
|---|---|
| Drive | Separate push-pull GPIO (LED + 1 kΩ to GND), toggled in the same ISR as DIM with the same chip bit — guaranteed to blink at chip rate when the MCU is alive and emitting. (LED is on its own pin so its drive mode is independent of DIM's open-drain emulation.) |
| Visibility | Green 0603 (or 3 mm through-hole for hand-prototype) on the inboard face of the half-cube, through a light-pipe slot per FR-1.1 |
| Operator interpretation | Blinking → pod is alive AND MCU is servicing the chip-rate ISR (does NOT prove LEDs are emitting — only proves MCU is alive; if supervisor pulled DIM LOW, the diagnostic LED still blinks but the IR LEDs are dark). Solid off → MCU dead. Solid on → MCU hung between ISR ticks (hard-fail-to-fix). For end-to-end "IR-emission-confirmed" use the smartphone-IR check from `eye-safety-measurements.md` §3 or the photodiode rig from Cart §F of `verified-bom.md` (T012b). |

## What the MCU does NOT do

- Does NOT participate in UVLO (FR-1.7 #4 — supervisor is the sole gate; MCU may continue running any LUT state when supervisor pulls DIM LOW because the wired-AND topology gives supervisor's open-drain pull-down dominance regardless of MCU output).
- Does NOT use ADC for battery voltage (no diagnostic path needed; supervisor handles cutoff).
- Does NOT have a serial port enabled at runtime (UPDI is for programming only; held in reset during normal op).
- Does NOT support OTA firmware update — flashing requires the UPDI cable.

## Firmware test contract

| Test | Mechanism |
|---|---|
| Chip rate ≈ 100 Hz | Capture DIM-pin waveform on oscilloscope (10 µs/div); count edges per second. Pass: 100 Hz ± 5% (NFR-4 bound). |
| Correct LUT bit sequence | Capture 200 ms of DIM waveform; export CSV; run `firmware/beacon-pod/tests/scope-trace-decode.py` against the captured CSV. Asserts: 15-chip period, expected bits per chip per LUT, no extra/missing transitions. |
| Boot latency | Scope trigger on V_BAT rise; measure time-to-first DIM-edge. Pass: ≤100 ms. |
| Power-off behavior | Pull V_BAT mid-emission; measure time-to-last DIM-edge. Pass: ≤50 ms (FR-1.7 #2). |
| Code-select | Build 4 pod variants with each 2-bit setting; verify the corresponding LUT is selected (decode tool again). |

These are the FR-1.5 + FR-1.7 hardware bench-verifications; the scope-trace-decode Python script is the executable test.
