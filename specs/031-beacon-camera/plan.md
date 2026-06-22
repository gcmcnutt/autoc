# Implementation Plan: 031 — 1-bit Beacon Acquisition (active phase)

> **This is the ACTIVE 031 plan** (the 1-bit single-IR-sensor acquisition phase). It is an **index /
> overview that references the detailed docs** rather than restating them — ask it "what is the plan,
> the milestones, the acceptance criteria" and follow the links for depth. The original camera-pipeline
> `plan.md` moved to [`../040-camera-redo/plan.md`](../040-camera-redo/plan.md) (parked, likely stale).
>
> ⚠️ **Spec-kit note**: do **not** run the stock `/speckit.plan` or `/speckit.tasks` generators on this
> branch — `FEATURE_SPEC` resolves to the shared mega-`spec.md` (not this phase) and the generators would
> clobber the curated `research.md`/`tasks.md`. Plan + tasks are hand-maintained.

**Branch**: `031-beacon-camera` | **Updated**: 2026-06-22
**Spec / plan-of-record**: [`acquisition-research-plan.md`](acquisition-research-plan.md) (scope, experiments, staged arc, sim)
**Design / toolchain**: [`fpga-toolchain-plan.md`](fpga-toolchain-plan.md) · [`emitter-toolchain-plan.md`](emitter-toolchain-plan.md)
**Tasks**: [`tasks.md`](tasks.md) **Phase A** (hand-maintained)
**Emitter requirements**: shared mega-spec [`spec.md`](spec.md) FR-1.x (LED / driver / Gold-code / eye-safety) + [`contracts/mcu-firmware-contract.md`](contracts/mcu-firmware-contract.md)

---

## Summary

031 field-proves the **coded-beacon temporal/code channel** — can we acquire a 15-bit Gold code through
real air, separate two beacons on **one photodiode** (CDMA), and characterize dropout — using a
single-sensor ADC-soft-decision receiver, **before any camera investment**. It also de-risks the
emitter (ATtiny412/UPDI) and FPGA (Diamond/MachXO2) toolchains the camera phase (040) reuses.

**Definition of Done = Stage 2 ground-field gate** (bench + ground only; breadboard hardware). Flying the
1-bit receiver (Stage 3) is a separate end-of-roadmap "pre-camera flight" follow-on. Camera localization →
040. Baseline: **20 Hz control loop / 480 fps / 200 Hz chip / 75 ms (15-chip) code / 15-bit**.

## Technical Context

- **Emitter firmware**: C, **bare-metal (no Arduino/OS)** — avr-gcc + serialUPDI on ATtiny412 (build of record); PlatformIO is an optional bare-C wrapper, not adopted. See [emitter-toolchain-plan.md](emitter-toolchain-plan.md).
- **FPGA gateware**: Verilog on **Lattice MachXO2-4000HE (STEP-MXO2)**, built on the Windows-host **Diamond** flow via WSL interop; HDL co-sim in `iverilog`/`vvp` against `acquisition-sim/sim.py` golden vectors. See [fpga-toolchain-plan.md](fpga-toolchain-plan.md).
- **Receiver chain**: `PD → TIA (MCP6022/OPA381) → [AC/DC-couple] → ADC (MCP3201) → StepFPGA correlator → UART telemetry`. **ADC soft-decision only — no comparator** (acquisition-research-plan §5).
- **Two beacons = two independent timing domains** — per-beacon self-syncing chip-rate/phase loop (DPLL); do not assume shared clock/phase (acquisition-research-plan §5).
- **Python 3.11**: `acquisition-sim/` (predict), `nfr4-clockdrift-sim/`, UART capture/analysis.
- **Logging**: UART → laptop for all 031 stages (no SD; airborne SD is the deferred follow-on).
- **Hardware homes**: emitter [`cad/beacon-eval/`](../../cad/beacon-eval/verified-bom-eval.md), receiver [`cad/beacon-receiver/`](../../cad/beacon-receiver/).

## Constitution Check

| Principle | Status |
|---|---|
| **I — Testing-First** | ✅ HDL co-sim golden vectors from `sim.py` (FPGA F2); scope-trace decode of the emitter code (emitter E2 / `make decode`); pytest where Python is involved. Sim **predicts**, bench **measures against** it. |
| **IV — Unified Build** | ⚠️ Justified exception: emitter (avr-gcc/UPDI) and FPGA (Diamond/MachXO2) are inherently separate toolchains — cannot unify under the C++ CMake build. Each documented in its toolchain plan. |
| **V — Fail-loud** | ✅ No silent fallbacks in decode/lock; lock ladder + correlation-margin are explicit outputs. |
| **VII — No silent fallback defaults** | ✅ Defaults (chip rate, oversampling, thresholds) are explicit + swept, not hidden. |

## Milestones

Strategy: **prove the tools before the logic; get a hello-world up early to gauge the hill.** Emitter and
FPGA proceed in parallel; the two meet at the bench (Stage 0).

| # | Milestone | Bundles | Done when (acceptance) |
|---|---|---|---|
| **M0** | **Toolchain verify** (de-risk first) | FPGA **F1** (Diamond→blink STEP-MXO2), Emitter **E1** (UPDI→blink ATtiny412) | Both flash loops confirmed on real hardware — a `.jed` builds from WSL and blinks; a `.hex` flashes via serialUPDI and blinks. Tooling proven before any logic. |
| **M1** | **Firmware + gateware bring-up** | Emitter **E2** (200 Hz Gold-code ISR), **E3** (UVLO+WDT); FPGA **F2** (sim harness), **F3** (MCP3201 SPI master + UART), **F4** (soft correlator + lock FSM) | Emitter emits the scope-verified 15-chip / 5 ms-chip code; FPGA streams live ADC envelope over UART and locks to a single live emitter; margin/telemetry sane vs `sim.py`. |
| **M2** | **Bench Stage 0 — hello gold code** | FPGA F4 vs a real single emitter, ~1 m, ND-attenuated | Scope + ADC show the 15-chip code; correlation peak clears noise; chip-rate / code / oversampling sweepable. |
| **M3** | **Bench Stage 1 — two codes, one detector** | FPGA **F5** (two-code CDMA + early/partial acquisition) | Both codes (A/B) resolved from the summed signal, no cross-leak; cross-corr floor + acquisition time measured vs `sim.py`; partial-code (~70%) lock characterized. |
| **▣ CHECKPOINT** | **Code checkpoint — ahead of the full plan** | — | Hello-world (Stage 0, ideally Stage 1) working; checkpoint firmware/gateware and **re-assess the hill** — is 031's full scope right-sized, or split? |
| **M4** | **Ground field — Stage 2 (031 GATE)** | beacons on craft (static/taxi/hand-held), PD on ground + collection lens + filter | **Acquire-and-agree**: two-code acquisition across the planned range/aspect grid AND measured acquisition-time + SNR-vs-range agree with the [§9 sim](acquisition-research-plan.md) within **~few dB** (predict-then-test). Exploratory — newly-surfaced issues are research output, **not** automatic gate failures. |

Milestone detail: FPGA F0–F5 in [fpga-toolchain-plan.md §4](fpga-toolchain-plan.md); emitter E1–E3 in [emitter-toolchain-plan.md §4](emitter-toolchain-plan.md); staged arc + pass criteria in [acquisition-research-plan.md §4](acquisition-research-plan.md). Executable tasks: [tasks.md](tasks.md) Phase A.

## Acceptance criteria (the gate)

**031 passes at M4 (Stage 2).** Concretely: across the range/aspect grid the receiver acquires both codes,
and the measured **acquisition-time** and **post-correlation SNR-vs-range** track the §9 sim prediction
within ~few dB — closing the predict-then-test loop and proving the Gold-code design + basic FPGA
correlator architecture in real air. Not a contractual probability number (breadboard/exploratory phase).
Each earlier milestone's "Done when" is its own gate (above).

## Out of scope / deferred

- **Stage 3 (flying the 1-bit receiver)** → end-of-roadmap "pre-camera flight" follow-on (small-form-factor
  receiver + onboard record-to-SD; airborne-logging options in acquisition-research-plan §6). Sequencing
  condition-dependent; number it when specced.
- **Localization (x, y) / camera pipeline** → feature **040** ([`../040-camera-redo/README.md`](../040-camera-redo/README.md)).

## Open / plan-phase research items

(Beyond the §10 empirical bench measurements — LED center wavelength, solar background, dropout envelope.)

- **Emitter toolchain** — settled: bare-C avr-gcc/UPDI (Arduino/OS rejected); PlatformIO optional fallback.
- **FPGA fit** — does two-beacon **independent-DPLL** decode fit the MachXO2-4000HE (4320 LUTs), or force
  time-multiplexing / a bigger part? (The "~1000× headroom" note is about oversampling, not two correlators.)
- **FPGA IP** — only the MachXO2 PLL is library IP (Diamond); the rest is custom RTL (fpga §3).
- **Analog front end** — TIA + AC/DC-coupling + MCP3201 soft-sample quality (FPGA F3; fpga §5 open decisions).
</content>
