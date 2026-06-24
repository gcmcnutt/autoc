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
          ── emitter clock: internal OSCH (its ~±5% tol = the drift) ──   ── correlator clock: 12 MHz xtal → PLL ──
 OSCH ─────► chip-rate divider (200 Hz; DIP can also force ±5% offset) ─► Gold-code   │
            LFSR/LUT (N=15, code A) ─┬─► raw code  ───────────────────────► I/O 14  (scope: the chip stream)
                                     ├─► epoch/index pulse (HIGH @ chip 0) ► I/O 15  (scope: trigger / time-align)
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

The emitter runs on the **internal OSCH oscillator** and the correlator on the **12 MHz crystal → PLL** —
**two independent physical oscillators**, so the emitter is genuinely async to the correlator (the real
condition). OSCH's own frequency tolerance (~±5%) **directly models the RC-oscillator drift (NFR-4)** — no
contrived skew needed; a DIP can additionally force a controlled ±5% chip-rate offset for repeatable sweeps.
The **simulated ADC generates the soft-sample stream directly from the code in-FPGA** (no physical
square-wave emission/re-sampling) in the **same 12-bit word format the MCP3201 will** (acquisition-research-plan
§5), so the correlator is bit-identical sim↔real — S7 just re-points its sample input. The raw code + epoch
are still driven to **I/O 14 / I/O 15** for scope observability.

## Staged milestones

| # | Build | Proves | Board shows |
|---|---|---|---|
| **S1** | **Synthetic emitter** on internal OSCH: Gold code (N=15) @200 Hz chip; raw code → **I/O 14**, epoch/index pulse → **I/O 15** | the code generator + timing (cf. acquisition Stage 0 "hello gold code", but in-FPGA) | scope on I/O 14 (15-chip code) triggered off I/O 15 (epoch) |
| **S2 ✓** | **Simulated ADC + SPI** (done, HW-verified): code → 12-bit analog model → bit-exact virtual **MCP3201** read by a soft SPI master @ **480 Hz** sample (camera cadence, 2.4 samples/chip — *not* oversampled), SCLK 50 kHz. Shared `spi_mcp3201.v`; LFSR decode self-test (`mcp3201_test`) proves bit-exact fetch | the sample source + MCP3201 word-format/decode | s2: sample top byte on 8 LEDs; test: PASS walking-dot; scope CS=P3, SCLK=M4, DOUT=N4 |
| **S3 ✓** | **Correlator (DUT)**: soft matched filter + DC-removal/**AGC** (signal-level-independent match ratio) + **min-lock / limited-hold FSM**; **dual correlator** checks BOTH codes at once (code discrimination, one signal emitted) | acquisition + AGC + discrimination in silicon (F4); HW-verified | 7-seg: per-code quality 0-9; RGB L/R: per-code lock (R/Y/G/green-blink-hold); 8 LEDs: q bars |
| **S4** | **Knobs**: DIP=code select; momentary=inject 1/2/3/4-bit errors per code period; DIP=noise level; UART `BCN` telemetry of {seq, sample, corr, lock, margin} | erasure/flip tolerance (§7), soft-decision gain; matches the host reader | 8 LEDs: per-chip hits; UART log via `host/monitor.sh` |
| **S5** | **Clock drift (NFR-4)**: skew the emitter chip rate ±5% (DIP-selected divisor); the correlator's **DPLL must lock to the emitter's own rate** | the per-beacon self-syncing DPLL under drift — the §5 load-bearing requirement | RGB[0]: still reaches green across ±5%; 7-seg: acquired-rate or acq-time |
| **S6** | **Second emitter + code B** on an *independent* skewed clock, summed into the simulated ADC; **two independent correlators/DPLLs** | honest **two-code CDMA separation under real clock slip** (§5, Stage 1) — in-FPGA | RGB[0]=A lock, RGB[1]=B lock; both green = separated; 8 LEDs split A/B activity |
| **S7** | **Bridge to real HW**: the soft MCP3201 SPI master is already real (S2) — just swap the virtual `mcp3201_model` for the **real MCP3201** on `cad/beacon-receiver` (DOUT → an input pad); correlator unchanged | the sim→real swap; feeds acquisition-research-plan Stages 0–2 | same display, now off real photons |

S1–S6 need **no analog hardware** — pure FPGA, flashed/observed over the proven loop. S3+ is the actual
shippable correlator RTL (the A4a–c / F4–F5 work), just exercised by the synthetic stimulus.

## Hardening study (→ tasks **A4d**)

S3 surfaced that **N=15 is likely under-spec** (thin cross-corr↔1-bit-error margin: floor q≈5 vs 1-bit q≈7),
so this harness becomes the vehicle for the code-length / latency / dropout study (see
[`specs/031-beacon-camera/tasks.md` §A4d](../../specs/031-beacon-camera/tasks.md)). Design target = a **soft
confidence ramp, not a binary lock**, in three latency tiers (the camera-mode model — source first seen as
event-camera-like pulses with apparent screen motion):

- **Candidate** — low latency, **~½ code word** (partial correlation; e.g. an N=63 code after ~20 chips) → an
  image predictor starts watching a screen region.
- **Hard lock** — medium, **~1–2 code words**; rides through varied dropout lengths.
- **Re-acquire** — fast, **a few bits** — drive through occlusion / sun / ground clutter / noise / **two
  beacons on one pixel** / apparent image motion.

**Key output:** minimum code length (+ chip/oversample) for confident acquisition + dropout hardening — the
coding standard for 040's camera CEP. Corollary: with hard lock + motion predictors, **chip-rate need not be
rigidly tied to camera frame rate**. Needs: partial/progressive correlator, a burst-error knob, and the S6
second emitter — all on this harness.

## Eval-board I/O map (STEP-MXO2)

Output sites known from the threeN1 prior art. **Switch (DIP/momentary) input sites + the I/O-14/15 ball
mapping are TBD** — from `docs/step-mxo2-lpc.pdf` (schematic; image-only, no text layer to extract) + the EIM
pinout / pin-allocation drawings at <https://support.eimtechnology.com/documentation/stepfpga>. Resolve at S1.

| Resource | Pins (known) | Proposed function |
|---|---|---|
| 8 LEDs | N13,M12,P12,M11,P11,N10,N9,P9 (active-low) | per-chip correlation activity / state bar |
| 2 RGB LEDs | `LEDl[2:0]`, `LEDr[2:0]` (threeN1 lpf) | beacon A / B lock: red=no, yellow=tentative, green=confirmed |
| 2× 7-seg | `d1[6:0]`,`d2[6:0]` + `enableLd1/2` (threeN1 lpf) | margin / SNR, acq-time, error count, or acquired chip-rate |
| 4 DIP | **TBD** | [1:0] code select · [2] noise level · [3] clock-skew (−5/+5%) |
| 4 momentary | **TBD** | [0] inject N-bit error (cycles 1→4) · [1] reset · [2] cycle display mode · [3] freeze/step |
| USB-UART (COM3) | STEPLink CDC | `BCN` telemetry logs → `host/monitor.sh` (Windows-side read) |
| Emitter epoch sync | board **I/O 15** (ball TBD) | scope trigger / receiver time-align |
| Emitter gold code | board **I/O 14** (ball TBD) | scope: raw 200 Hz chip stream |

## Notes / decisions to settle at build time

- **Clocks**: correlator on the **12 MHz crystal → PLL** (stable; modest rate OK — data is kHz, multiplier
  headroom proven). Emitter on the **internal OSCH** (a separate physical oscillator; its ~±5% tolerance *is*
  the drift — NFR-4). The **simulated ADC is the clean clock-domain-crossing boundary** into the correlator.
  (NB: pick a valid OSCH `NOM_FREQ` — "96.77" was rejected earlier; confirm the legal value list at S1.)
- **Reuse the multiplier**: correlation = MAC; one **time-multiplexed** carry-save multiplier serves the taps
  (experiments showed ~kHz data vs ~125 MHz mult → hundreds of slots/sample). Don't replicate per tap.
- **Resource budget**: keep the whole harness (2 emitters + 2 correlators + sim-ADC + I/O) within the part —
  watch LUT/carry (the self-test already hit 81% with *two* multipliers; the correlator should share one).
- **Telemetry**: emit the existing `BCN,seq,adc,corrA,lockA,marginA,corrB,lockB,marginB` frame so the host
  `frames_from_lines` parser works unchanged.
- **Active-low LEDs**: encode status as motion/color, not bare levels (lesson from the self-test).
- **EFB hard-SPI — deferred to backlog** (decided 2026-06-23): the soft SPI master (`spi_mcp3201.v`) is the
  design — HW-verified and ample for the kHz correlator. The MachXO2 EFB hardened SPI (IPexpress →
  Architecture_Modules → EFB; WISHBONE-driven, see TN1205) is *not* needed here and is more likely repurposed
  later for **buffered SD-card writes in the 1-diode flight receiver**. Revisit only if a hardware SPI offload
  is actually required; the `mcp3201_model` is edge-driven so it can re-validate a hard block's paused-clock
  read if we ever do.

## Relationship to the program plan

This is the **execution vehicle for the correlator** (acquisition-research-plan F4/F5 / tasks A4a–c): it
validates the soft-decision matched filter + per-beacon DPLL + lock FSM in silicon, under injected
errors/drift/CDMA, before the analog front end. S7 is the hand-off to the real `cad/beacon-receiver` and
acquisition-research-plan Stages 0–2 (bench/ground). The synthetic emitter also doubles as a permanent
**known-good stimulus** for regression after the analog path lands.
