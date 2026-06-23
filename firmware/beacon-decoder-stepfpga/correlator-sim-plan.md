# Correlator on-FPGA simulation harness — plan (soup-to-nuts, wrapping the real correlator)

**Status**: PLAN, 2026-06-23. **Goal**: develop + validate the *actual* beacon correlator gateware entirely
on the STEP-MXO2 eval board — a **synthetic emitter** feeds a **simulated ADC** into the **real correlator**
(the DUT), with the board's rich I/O for control + observation. This exercises the hard questions —
acquisition time, dropout/erasure tolerance, **per-beacon independent clock drift (±5%, NFR-4)**, and
**two-code CDMA separation (§5)** — *before* the `cad/beacon-receiver` analog front end exists. The ONLY
change to go live is swapping the simulated ADC for the real MCP3201 (S7).

Builds on the proven pieces: WSL→Diamond build/flash/monitor flow ([fpga-toolchain-plan.md](../../specs/031-beacon-camera/fpga-toolchain-plan.md)),
PLL/OSCH clocking + at-speed run ([experiments/](experiments/)), the `BCN` UART telemetry contract +
host reader ([host/](host/README.md)). Correlator requirements: acquisition-research-plan
[§5](../../specs/031-beacon-camera/acquisition-research-plan.md) (receiver chain, **per-beacon self-syncing
DPLL — do NOT assume a shared clock**), §7 (200 Hz chip, N=15 Gold, soft-decision + erasure-aware,
early/partial ~70% acquisition, oversampling), NFR-4 (±5% drift).

## Block diagram (all in-FPGA)

```
            ── emitter clock domain (free-running, ±5%-skewable) ──        ── correlator/DSP clock (PLL, stable) ──
 OSCH/PLL ─► chip-rate divider (nominal 200 Hz, ±5% via DIP) ─► Gold-code   │
            LFSR/LUT (N=15, code A) ─┬─► raw code  ───────────────────────► GPIO  (scope: the chip stream)
                                     ├─► epoch/index pulse (HIGH @ chip 0) ► GPIO  (scope: trigger / time-align)
                                     └─► "optical+analog model" ─► simulated ADC ─► soft samples @~100 kS/s, 12-bit
                                          (attenuation, DC pedestal, noise,            (same word format as MCP3201)
                                           bit-flip / erasure injection)                       │
                                                                                               ▼
   error/skew/code knobs (DIP + momentary)                                         ┌───────────────────────────┐
   lock/margin/acq/errors (8 LED + 2 RGB + 2×7-seg + UART)  ◄──────────────────────┤  REAL correlator (DUT):   │
                                                                                   │  per-beacon DPLL + sliding │
                                                                                   │  soft matched filter +     │
                                                                                   │  lock FSM + margin/CEP     │
                                                                                   └───────────────────────────┘
```

The emitter is **intentionally unsynchronized to the correlator** (free-running phase + an offset/selectable
chip-rate divisor → models the real ±5% RC-oscillator independence). Higher-fidelity option: drive the
emitter from a *second* PLL output at a slightly different frequency. The simulated ADC emits the **same
soft-sample stream + word format the MCP3201 will** (acquisition-research-plan §5), so the correlator is
bit-identical between sim and real — S7 just re-points its sample input.

## Staged milestones

| # | Build | Proves | Board shows |
|---|---|---|---|
| **S1** | **Synthetic emitter**: Gold code (N=15) @200 Hz chip on the emitter clock; epoch/index pulse + raw code on two GPIO | the code generator + timing (cf. acquisition Stage 0 "hello gold code", but in-FPGA) | scope on the two GPIOs: 15-chip code + epoch trigger |
| **S2** | **Simulated ADC**: code → soft 12-bit samples @~100 kS/s (oversampled ~500×/chip) with DC pedestal + selectable noise | the sample source the correlator consumes; MCP3201 word format | 7-seg: live sample value / level |
| **S3** | **Correlator (DUT)** on ONE code: chip integrator → sliding soft matched filter → tentative/confirmed lock FSM + correlation margin | acquisition works end-to-end in silicon (F4) | RGB[0]: red=no-lock / yellow=tentative / green=confirmed; 7-seg: margin (SNR proxy) |
| **S4** | **Knobs**: DIP=code select; momentary=inject 1/2/3/4-bit errors per code period; DIP=noise level; UART `BCN` telemetry of {seq, sample, corr, lock, margin} | erasure/flip tolerance (§7), soft-decision gain; matches the host reader | 8 LEDs: per-chip hits; UART log via `host/monitor.sh` |
| **S5** | **Clock drift (NFR-4)**: skew the emitter chip rate ±5% (DIP-selected divisor); the correlator's **DPLL must lock to the emitter's own rate** | the per-beacon self-syncing DPLL under drift — the §5 load-bearing requirement | RGB[0]: still reaches green across ±5%; 7-seg: acquired-rate or acq-time |
| **S6** | **Second emitter + code B** on an *independent* skewed clock, summed into the simulated ADC; **two independent correlators/DPLLs** | honest **two-code CDMA separation under real clock slip** (§5, Stage 1) — in-FPGA | RGB[0]=A lock, RGB[1]=B lock; both green = separated; 8 LEDs split A/B activity |
| **S7** | **Bridge to real HW**: replace the simulated ADC with the **MCP3201 SPI** front end (`cad/beacon-receiver`); correlator unchanged | the sim→real swap; feeds acquisition-research-plan Stages 0–2 | same display, now off real photons |

S1–S6 need **no analog hardware** — pure FPGA, flashed/observed over the proven loop. S3+ is the actual
shippable correlator RTL (the A4a–c / F4–F5 work), just exercised by the synthetic stimulus.

## Eval-board I/O map (STEP-MXO2)

Output sites known from the threeN1 prior art; **input (switch) sites are TBD — look up in the StepFPGA
board doc** (`docs/step-mxo2-lpc.pdf` / EIM docs) at S1.

| Resource | Pins (known) | Proposed function |
|---|---|---|
| 8 LEDs | N13,M12,P12,M11,P11,N10,N9,P9 (active-low) | per-chip correlation activity / state bar |
| 2 RGB LEDs | `LEDl[2:0]`, `LEDr[2:0]` (threeN1 lpf) | beacon A / B lock: red=no, yellow=tentative, green=confirmed |
| 2× 7-seg | `d1[6:0]`,`d2[6:0]` + `enableLd1/2` (threeN1 lpf) | margin / SNR, acq-time, error count, or acquired chip-rate |
| 4 DIP | **TBD** | [1:0] code select · [2] noise level · [3] clock-skew (−5/+5%) |
| 4 momentary | **TBD** | [0] inject N-bit error (cycles 1→4) · [1] reset · [2] cycle display mode · [3] freeze/step |
| USB-UART (COM3) | STEPLink CDC | `BCN` telemetry logs → `host/monitor.sh` (Windows-side read) |

## Notes / decisions to settle at build time

- **Clocks**: correlator on a stable PLL clock (modest — data rate is kHz; the multiplier headroom is proven).
  Emitter on a free-running, skewable chip-rate divider (model ±5%); optionally a 2nd PLL output for true
  domain independence. Mind clock-domain crossing on the sample handoff (the sim ADC is the clean boundary).
- **Reuse the multiplier**: correlation = MAC; one **time-multiplexed** carry-save multiplier serves the taps
  (experiments showed ~kHz data vs ~125 MHz mult → hundreds of slots/sample). Don't replicate per tap.
- **Resource budget**: keep the whole harness (2 emitters + 2 correlators + sim-ADC + I/O) within the part —
  watch LUT/carry (the self-test already hit 81% with *two* multipliers; the correlator should share one).
- **Telemetry**: emit the existing `BCN,seq,adc,corrA,lockA,marginA,corrB,lockB,marginB` frame so the host
  `frames_from_lines` parser works unchanged.
- **Active-low LEDs**: encode status as motion/color, not bare levels (lesson from the self-test).

## Relationship to the program plan

This is the **execution vehicle for the correlator** (acquisition-research-plan F4/F5 / tasks A4a–c): it
validates the soft-decision matched filter + per-beacon DPLL + lock FSM in silicon, under injected
errors/drift/CDMA, before the analog front end. S7 is the hand-off to the real `cad/beacon-receiver` and
acquisition-research-plan Stages 0–2 (bench/ground). The synthetic emitter also doubles as a permanent
**known-good stimulus** for regression after the analog path lands.
