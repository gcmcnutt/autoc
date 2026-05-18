# Contract — ATtiny412 Beacon-Pod Firmware

**Spec ref**: FR-1.3 (Gold-code generation), FR-1.2 (driver topology), FR-1.7 (UVLO contract)
**Reference impl**: `firmware/beacon-pod/src/`

## Single-line description

> When V_BAT is present and supervisor releases EN, the MCU clocks out one bit of a 15-chip Gold-code LUT to a single DIM-pin GPIO every 10 ms. Nothing else.

## Boot sequence

| Step | Action | Bound |
|---|---|---|
| 1 | MCU resets on V_BAT rise above ~1.8 V | <10 ms after JST-PH mate |
| 2 | Configure internal 20 MHz RC oscillator; set CCP-protected CPU clock to 20 MHz (no prescaler) | <5 ms |
| 3 | Configure GPIO directions: DIM-out = output-low, 2× code-select = input-pull-up, diagnostic-LED = output-low | <1 ms |
| 4 | Sample the 2-bit code-select once; choose one of 4 N=15 Gold-code LUTs (compiled into flash) | <1 ms |
| 5 | Configure TCA0 in single-shot 16-bit mode: period = 20 MHz / 100 Hz = 200 000 counts → 100 Hz tick rate. Enable TCA0 overflow interrupt | <1 ms |
| 6 | Set `chip_idx = 0`. Write `LUT[selected_code][0]` to DIM and diagnostic-LED GPIOs simultaneously. Start TCA0 | T = 0 |
| 7 | Enter `sei()` + `while(1) { sleep_cpu(); }` idle loop. Wake-from-sleep on every TCA0 interrupt | continuous |

**Total boot time**: <20 ms from V_BAT rise → first chip emission. **Spec gate FR-1.7 #1: ≤100 ms.** Comfortable margin.

## Steady-state per-tick behavior (TCA0 ISR)

```c
ISR(TCA0_OVF_vect) {
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;  // clear flag
    chip_idx = (chip_idx + 1) % 15;
    uint8_t bit = LUT[selected_code][chip_idx];
    if (bit) {
        DIM_PORT.OUTSET = (1 << DIM_PIN) | (1 << DIAG_LED_PIN);
    } else {
        DIM_PORT.OUTCLR = (1 << DIM_PIN) | (1 << DIAG_LED_PIN);
    }
    // total ISR cost: ~10 cycles at 20 MHz = 500 ns. Negligible vs 10 ms tick.
}
```

No PWM peripheral. No analog. No floating-point. No malloc. No watchdog reset path needed (firmware is stateless across reset).

## DIM-pin signal contract

| Property | Value |
|---|---|
| Logic level | 0 V / V_BAT (3.0–4.2 V LiPo swing) |
| Drive strength | ATtiny412 GPIO default (~20 mA source/sink) — far more than the LM3410-Y DIM pin's ~1 µA input current |
| Slew rate | Native GPIO edge (~10 ns) — DIM is a digital input, doesn't need slewing |
| Chip period | 10 ms ± internal-RC drift (see NFR-4) |
| Duty per code period | 7–8 ON chips of 15 (Gold-code-derived; specific value per selected LUT) |

## Code-select-pin contract

| Property | Value |
|---|---|
| Pull-direction | Internal pull-up enabled on MCU side |
| External | Solder jumper to GND for "0", float for "1" (or a 2-bit DSM-02 DIP switch) |
| Sample time | Once at boot only (no runtime re-read) |
| Encoding | LSB at one pin, MSB at the other → 4 codes (0..3); Phase 1 pods are 0 + 1 |

## Diagnostic-LED contract

| Property | Value |
|---|---|
| Drive | Same GPIO write as DIM-pin (LED + 1 kΩ to GND) — guaranteed to blink at chip rate if and only if the LED-driver DIM signal is being asserted |
| Visibility | Green 0603 (or 3 mm through-hole for hand-prototype) on the inboard face of the half-cube, through a light-pipe slot per FR-1.1 |
| Operator interpretation | Blinking → pod is alive. Solid off → MCU dead or supervisor tripped. Solid on → MCU hung in some non-ISR state (call out as a hard-fail-to-fix) |

## What the MCU does NOT do

- Does NOT participate in UVLO (FR-1.7 #4 — supervisor is the sole gate; MCU may run any LUT state when supervisor pulls EN).
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
