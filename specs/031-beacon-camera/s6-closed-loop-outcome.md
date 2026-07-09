# s6 outcome — real-emitter closed loop: decode, perturb, and recovery characterization

**Status**: MEASURED, 2026-07-08. **Scope**: the s6 gateware milestone (digital-inject code A +
circular-RAM windows + real-ADC code B) and the first full closed-loop characterization of the *real*
signal chain: ATtiny416 emitter (DIM, 3.3 V rail) → MCP3201 ADC → SPI → StepFPGA correlator, with the
emitter host-perturbable over its USART0 command link.

Prior state: [correlator-sim-plan.md](../../firmware/beacon-decoder-stepfpga/correlator-sim-plan.md)
S1–S6 on-FPGA milestones (synthetic stimulus); s5 = first real-ADC read (static DC sweep, hex display).
This doc records what s6 changed and what the closed loop measured.

## Gateware changes (s6 vs s5)

| Change | Why | Result |
|---|---|---|
| Code A **digitally injected** (chip → 2-level 12-bit value straight into `winA`) — the analog model / virtual MCP3201 / second soft-SPI reader are gone | code B is now the real ADC; the fake-ADC round-trip was pure ceremony | code A keeps its **independent OSCH clock** (still exercises the DPLL) as a clean reference channel |
| Sample windows → **circular-buffer distributed RAM** (`wp`/`rp` pointers, async read) replacing two 74-deep FF shift registers + 74:1 mux trees | the shift regs were ~1776 FF + ~240 MUX81 — the device was at 98% | **2108 → 1338 slices (98% → 62%)**; correlator timing byte-identical by construction (async read) |
| Testability restored: K1/K2 = 1-/2-bit error inject (LOCAL), USB `'E'` rate / `'A'` amp / `'K'` burst + mask knobs (REMOTE) | controlled impairments on the reference channel; the real channel gets *real* impairments | knob cost ≈ 75 slices — absorbed by the RAM win |
| `` `ifdef SIM `` clock scaling (÷100, ratio-preserving: `EDIV_NOM`/`FDIV`/`SCLK_HALF`) | HDL simulation in ms instead of minutes | synthesized bitstream verified unchanged |

Budget: **1338/2160 slices (62 %), 1100 FF (24 %), timing met** — headroom for N=63 work; EBR (0/10) still
untouched (the next capacity lever if deeper windows are needed; requires a 1-cycle-read pipeline change).

## New validation surface: WSL-native HDL sim

[`firmware/beacon-decoder-stepfpga/sim/`](../../firmware/beacon-decoder-stepfpga/sim/README.md) — iverilog
harness compiling the real RTL (OSCH stubbed). Stood up specifically to verify the circular-RAM window
rewrite **before** any flash (no board was in hand): code A and code B both lock, K1/K2 modulate margin,
+5 % code-B skew tracked, 1/4-amplitude AGC hold. All PASS, later confirmed identical on hardware.

## Closed-loop measurements (real emitter → real ADC)

Setup: emitter DIM wired directly to ADC IN+ (rail-to-rail 0–3.3 V ≈ full-scale chips — the max-contrast
case; the TIA/amp comes next). Emitter perturbed over its mEDBG CDC (38400; `'F'`/`'C'`/`'D'`/`'R'`);
decoder observed via BCN telemetry on COM3. **Methodology: clean-state** — every trial from a verified
locked baseline at nominal; one perturbation per trial. (A ±10 % compound slew between skew states falsely
drops lock — that early mistake is why the methodology note exists.)

### Lock + frequency tracking

- **First real-signal code-B lock**: `corrB≈145k` (≈2.3× the synthetic code A at ±1024 swing — as expected
  for full-scale), `marginB` 7–9, dual lock with code A simultaneously (live CDMA with the RAM windows).
- **±5 % emitter frequency** (settled steps from nominal): lock held both directions, 0 unlocked frames in
  a 163-frame watch; `rateB` slip tracks direction correctly. Corruption (`'C' 10/31` chips) and word-scale
  dropout (`'D' 15/31`) degrade margin → unlock → `'R'` recovers, every time.
- The closed-loop DPLL (`Leff = L + slip`) absorbs a steady rate offset into `rateB`≈nominal in steady
  state — *held lock* is the pass criterion, not the rate readout.

### Dropout → recovery ladder (the occlusion characterization)

[`host/recovery_sweep.py`](../../firmware/beacon-decoder-stepfpga/host/recovery_sweep.py): signal stopped
dead (`'D' 0x1F` = all 31 chips blanked → DIM low), durations 5 ms → 8 s, **×3 repeats each**, 27/27 trials
valid, recovery measured from telemetry seq (25 ms ticks), onset-anchored at the margin collapse (±~1 code
period ≈ 155 ms resolution).

| loss duration | outcome (3/3 unless noted) | detail |
|---|---|---|
| 5–25 ms (1–5 chips) | **NO-IMPACT** | margin dips to 6–7; lock threshold never touched |
| 78 ms (½ word) | boundary | phase-dependent: 1× no-impact, 2× ride-through (margin 3) |
| 155 ms (1 word) | **RODE-THROUGH** | HOLD carries it; lock never drops; full margin ≤ ~150 ms after return |
| 0.5 – 8 s | **RECOVERED** | HOLD absorbs the first **300–325 ms** (=HOLDMAX 2 periods — metronome-consistent across all 15 trials), then unlocked for the remainder; **warm re-lock ≲ 150 ms (~1 period) after signal return** |

**No ratchet**: recovery flat across repeats (scatter ±1–2 quanta of tick×period phase, no growth trend);
`hold` = 300–325 ms on every trial — no state accumulates across repeated outages. All rungs ≤ 8 s sit
inside the ~10 s flywheel coast window → all re-locks were WARM. The cold cliff (>10 s outage) was not
probed; one extra rung (e.g. 12 s) would measure it if needed.

## Implications for the next step (TIA/amp, then emitter-as-light)

1. Direct-wire is the max-contrast case; the amp changes *contrast and pedestal*, and both tolerances are
   already measured: AGC holds margin at 1/4 amplitude, and the pseudo-diff ADC cancels the bias.
2. Occlusions up to a code word are invisible; multi-second occlusions cost the outage + ~1 period. This is
   the recovery budget the airframe sees for beam blocks / glints.
3. Re-running `recovery_sweep.py` after the amp is installed gives a direct before/after diff on the same
   ladder — the harness is the instrument.
4. Watch items for the amp bring-up: keep the swing inside 0–Vref (no railing), pedestal mid-range for
   symmetric headroom.

## Artifacts

- Gateware: `firmware/beacon-decoder-stepfpga/experiments/s6.v` (+ pins/tcl/deploy/open scripts)
- HDL sim: `firmware/beacon-decoder-stepfpga/sim/` (tb, OSCH stub, runner, README)
- Host: `host/recovery_sweep.py` (+ README §closed-loop); raw CSVs land in `host/results/` (gitignored)
- Emitter link details: `firmware/beacon-pod` USART0 commands, 38400, DTR required (pyserial)
