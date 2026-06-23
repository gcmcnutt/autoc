# 031 — Beacon Acquisition Research Plan (1-bit / single-sensor phase)

**Status**: DRAFT — research plan, 2026-06-18
**Scope**: 031 is re-scoped to a **1-bit (single-IR-sensor) acquisition-research phase**. We characterize the *coded-beacon temporal/code channel* — acquisition time, signal quality, code separation, dropout tolerance — through real outdoor atmosphere (ground/taxi/hand-held, **not airborne in this phase**), with a single photodiode receiver. **Localization (x, y) and the full camera pipeline are deferred** (a single detector has no spatial resolution; see §2). This plan governs the bench + field work the [emitter](../../cad/beacon-eval/verified-bom-eval.md) + [receiver](../../cad/beacon-receiver/eval-loop-bom.md) BOMs were ordered for.

**Definition of Done (clarified 2026-06-22): Stage 2 ground-field acquisition is the 031 gate** — proving implementation, toolchain, basic firmware, and valid ground field tests on breadboard hardware. **Airborne flight of the 1-bit receiver (Stage 3) is deferred to a separate follow-on "pre-camera flight" feature** — a major addition (small-form-factor receiver + onboard record-to-SD-card). See §4 + §6.

Hardware home: emitter `cad/beacon-eval/`, receiver `cad/beacon-receiver/`. Spec/research home: here. The deferred **camera pipeline is its own feature, `040` (camera redo)** — *emitters shared with 031*, single-sensor front end replaced by a camera + bigger FPGA; the old camera `spec.md`/`plan.md` are 040's reference. ("camera phase" throughout this doc = feature 040.) **040 home (parked placeholder):** [`../040-camera-redo/README.md`](../040-camera-redo/README.md).

---

## Clarifications

### Session 2026-06-22

- Q: Phase-level Definition of Done for 031 — what proves the Gold-code design + basic FPGA architecture? → A: **Stage 2 (ground-field acquisition across ranges/aspects) is the 031 gate** — proving implementation, toolchain, basic firmware, and valid ground field tests on breadboard hardware. **Stage 3 (flying the 1-bit receiver) is deferred to a separate follow-on "pre-camera flight" feature** — a major addition needing a small-form-factor receiver + onboard record-to-SD-card.
- Q: Does "1-bit" mean single-sensor or 1-bit quantization, and is the hard-1-bit comparator a deliverable? → A: **"1-bit" = single sensor** (one detector; analog → multi-bit **ADC soft-decision**). **ADC soft-decision is the only receiver path — drop the comparator / hard-1-bit baseline entirely** (the camera-era front end runs AGC + soft-decision anyway, so a hard-1-bit path wouldn't represent the target architecture; the comparator can come off the receiver BOM).
- Q: What is the Stage 2 quantitative pass bar (now the 031 gate)? → A: **Acquire-and-agree** — two-code acquisition succeeds across the planned range/aspect grid AND measured acquisition-time + SNR-vs-range agree with the §9 sim prediction within a stated margin (default **≤3 dB SNR-vs-range / ±30% acquisition-time**, first-bench-calibratable), closing the predict-then-test loop. Not a contractual probability number (breadboard/exploratory phase). **As research, newly-surfaced issues are an expected output, not automatic gate failures.**
- Q (operator direction): How is the "pre-camera flight" follow-on positioned, and how should the work-order read? → A: The pre-camera-flight feature sits **toward the end of the program roadmap** (not the next sequential number) — sequencing is condition-dependent (e.g. fly emitters against a ground receiver first). The 031 work-order is **research → bench HW design → toolchain verify → coding + test → code checkpoint ahead of `/speckit.plan`** (see Development arc below); plan-phase research items beyond §10 (emitter 412 toolchain/PlatformIO, two-correlator FPGA fit, analog front-end, FPGA IP) are enumerated there. Getting a hello-world up early is the point — it gauges the steepness of the hill before committing to the full plan.

---

## Development arc (work order) — and the checkpoint before the formal plan

031 is a steep, many-piece climb; the strategy is **get a hello-world up early to gauge the slope** before committing to a full plan/tasks for the Stage-2 gate. Work proceeds in this order, emitter and receiver in parallel:

1. **Research** ✅ — link budget, Gold-code/matched-filter sim ([`acquisition-sim/`](acquisition-sim/)), clock-drift sim ([`nfr4-clockdrift-sim/`](nfr4-clockdrift-sim/)), and this plan. Predicts the curves the bench measures (§9).
2. **Bench hardware design** ✅ — emitter ([`cad/beacon-eval/`](../../cad/beacon-eval/verified-bom-eval.md)) + receiver ([`cad/beacon-receiver/`](../../cad/beacon-receiver/)) BOMs designed/ordered.
3. **Toolchain verification** — prove the tools *before* the logic, so a tooling break never masquerades as a logic bug:
   - **FPGA**: Diamond-via-WSL-interop loop confirmed ([fpga-toolchain-plan.md §6](fpga-toolchain-plan.md)); hardware-confirmed by milestone **F1** (blink an LED off the 12 MHz clock — full build → flash on the STEP-MXO2).
   - **Emitter**: ATtiny412 flash loop — **avr-gcc + serialUPDI** (1-wire UPDI) on the breadboard 412, committed + working ([`firmware/beacon-pod/`](../../firmware/beacon-pod/SETUP.md)); the **PlatformIO "later option"** (xiao-style) is detailed in [emitter-toolchain-plan.md](emitter-toolchain-plan.md). Hardware-confirmed by milestone **E1** (blink before Gold-code logic), mirroring the FPGA "prove the loop before logic" discipline.
4. **Coding + test**:
   - **Emitter** firmware: Gold-code LUT @ 200 Hz on the DIM GPIO + diagnostic blink.
   - **FPGA** gateware: correlator blocks + lock FSM ([fpga-toolchain-plan.md §3](fpga-toolchain-plan.md)), HDL co-sim golden vectors from `sim.py` (**F2**), then on hardware (**F3–F5**).
   - Bench **Stage 0** (hello gold code) → **Stage 1** (two codes, one detector).
5. **Code checkpoint — ahead of `/speckit.plan`.** With hello-world working (Stage 0, ideally Stage 1), checkpoint the firmware/gateware and **re-assess the hill** before writing the formal plan/tasks for the Stage-2 field gate. This is the decision point on whether 031's full scope is the right size or should be split.

**Open research / de-risk items that feed the formal plan** (beyond the §10 empirical bench measurements):
- **Emitter toolchain** — avr-gcc + serialUPDI vs PlatformIO-for-412 (step 3); confirm UPDI flashing on the breadboard.
- **FPGA fit** — does **two-beacon independent-DPLL** decode (two self-syncing correlators under the ~10% inter-beacon drift of §5) fit the MachXO2-4000HC (4320 LUTs), or force time-multiplexing / a bigger part? (The "~1000× headroom" note in fpga §3 is about sample-rate oversampling, *not* two full correlators + DPLLs.)
- **FPGA IP** — only the MachXO2 **PLL** is library IP (Diamond); the SPI master / DC-AGC / chip integrator / soft correlator / lock-FSM / UART are custom RTL ([fpga §3](fpga-toolchain-plan.md)).
- **Analog front end** — TIA (MCP6022 / OPA381) + AC/DC-coupling + MCP3201 soft-sample quality (FPGA milestone F3; coupling default / sample rate / telemetry framing are [fpga §5 open decisions](fpga-toolchain-plan.md)).

---

## 1. Why single-sensor first

A single photodiode strips the problem to its core: **photons → code**, no optics/spatial confound, receiver sample rate decoupled from any camera frame rate. It answers the make-or-break questions cheaply, before any camera investment:

- Can we **acquire** a 15-bit Gold code through real air at flight ranges, and how fast?
- Can **two beacons be separated on one detector** (CDMA), and does that survive real atmosphere + maneuvering?
- What are the **real dropout statistics** that set code length and soft-decision thresholds?

It also de-risks the toolchains (MCU UPDI flashing, Lattice/StepFPGA flow, ADC capture) that the camera phase reuses.

## 2. What this phase can and cannot answer

| Question | Single PD? | Why |
|---|---|---|
| Acquire/decode the code at range | ✅ | temporal correlation, no spatial info needed |
| Separate two beacons (codes A/B) on one detector | ✅ | CDMA — orthogonal Gold codes, correlate the *summed* signal |
| Dropout/erasure rate through attitudes | ✅ | the ADC envelope captures aspect-driven fades |
| Acquisition time vs SNR vs oversampling | ✅ | FPGA samples fast, sweeps freely |
| **Position (x, y) / bearing of a beacon** | ❌ | one detector = zero spatial resolution → camera phase |

**Key framing — the single detector IS the model for the hardest camera case.** With a wide (~120°) capture FOV, two wingtip beacons ~0.9 m apart at 100 m subtend only ~1.4 camera pixels — they **share a pixel**. That merged pixel's time-series is the *sum* of both codes, separated only by code-division — identical to the single-photodiode problem. So validating two-code separation on one diode directly validates the camera's worst case. (When merged you recover code-identity + intensity but a single blended position; two distinct positions need spatial separation — the camera phase.)

## 3. The two experiments (different optics — don't force one rig to do both)

The étendue of one small detector forbids wide-FOV *and* large-aperture at once, so range and FOV trade. Split accordingly:

### Exp A — All-attitude lock maintenance (~40° FOV, moderate range)
- **Goal**: does the **beacon's 270° half-cube coverage** keep enough light on the detector through rolls / banks / pitches, and does the **code survive the aspect-driven intensity fades + brief LED-occultations** as the target tumbles through all relative attitudes.
- **Optics**: **large-area PD (BPW34, 7.5 mm²)** + short lens → ~40° FOV; **~30–50 m** range (where a 40° FOV single PD has adequate signal). 40° matches a realistic chase that keeps the beacon in the forward cone.
- **The real payload is dropout statistics**: how deep/long the signal drops as the beacon rotates through its pattern = the real-world **erasure rate** → sets code length + soft-decision/erasure thresholds. (camera_considerations pegs N=15 at ~8-erasure tolerance; this measures whether maneuvering stays under it.)
- AGC studies are secondary here (acknowledged) — but **log the ADC envelope**; the fades *are* the data.

### Exp B — Max-range acquisition (narrow-aimed, 100 m)
- **Goal**: validate the link budget + acquisition time at design range.
- **Optics**: **collection lens (Ø25–50 mm) + bandpass filter + small PD (BPV10NF)**, **aimed** (narrow ~couple° FOV) at the craft on a tripod/pan-head. The lens — not the filter — supplies the signal (bare 0.8 mm² PD at 100 m ≈ pW; a 25–50 mm lens ≈ 100–1000× more). The filter rejects solar background.
- Narrow-aimed geometry means light hits the filter near-normal → the **narrow 10 nm filter becomes viable + best** (no angle-shift), *if* the LED center is confirmed ≈850 nm on the bench. Else the 30–40 nm MidOpt.

## 4. Staged arc

| Stage | Setup | Measures | Pass |
|---|---|---|---|
| **0 — Bench: hello gold code** | 1 emitter → 1 PD, ~1 m, ND-attenuated, laptop capture | code recovers; chip rate / code / oversampling sweepable | scope + ADC show the 15-chip code; correlation peak clears noise |
| **1 — Bench: two codes, one detector** | 2 emitters (codes A/B) → 1 PD | **CDMA separation** at design SNR; cross-corr floor; acquisition time | both codes resolved from the summed signal; A/B don't leak into each other |
| **2 — Ground field** _(031 GATE)_ | beacons on craft (static/taxi/hand-held), PD on ground + cone + filter | range, aspect, sun, scintillation; real solar noise floor | two-code acquire across the range/aspect grid **+ measured SNR-vs-range within ≤3 dB of the §9 sim + acquisition-time within ±30%** (predict-then-test; first-bench-calibratable default). Exploratory — newly-surfaced issues are research output, not automatic gate failures |
| **3 — Air field** _(DEFERRED → follow-on "pre-camera flight" feature, NOT 031)_ | target craft flying; PD on ground (or on tracker) | real range/aspect/body-rate; **two-code separation through real air**; dropout stats | maintain lock through maneuvers (Exp A); acquire at 100 m (Exp B) |

**031 gate = Stage 2** (ground-field acquisition across ranges/aspects). Stage 0/1 stream to a laptop over UART (no SD); Stage 2 likewise (PD on ground, laptop capture). **Stage 3 (flying the 1-bit receiver) is out of 031 scope** — it becomes a follow-on "pre-camera flight" feature whose major additions are a small-form-factor receiver + onboard record-to-SD-card (the airborne-logging options in §6). It sits **toward the end of the program roadmap, not next** — sequencing is condition-dependent (e.g. we may fly the *emitters* against a *ground* receiver first, depending on how the bench/ground work goes). Number it whenever it's actually specced, not now.

## 5. Receiver chain + FPGA roles

`PD → TIA (MCP6022 / OPA381) → [AC-couple] → ADC (MCP3201) → StepFPGA (MachXO2) correlator → lock + telemetry`

- **ADC soft-decision is the only receiver path** (soft-decision + AGC + erasure). **No comparator / hard-1-bit baseline** — the camera-era front end runs AGC + soft-decision anyway, so a hard-1-bit path wouldn't represent the target architecture (the comparator can come off the receiver BOM). "1-bit" here names the **single-sensor** (one-detector) phase, not 1-bit quantization. The MachXO2 has **no analog input**, so the external ADC is mandatory.
- **StepFPGA = the correlator/acquisition engine** (hard-SPI to the ADC, sliding matched filter, tentative/confirmed lock, UART telemetry). Good practice for the eventual Lattice camera FPGA.
- **Two beacons = two INDEPENDENT timing domains — decode each separately** (operator 2026-06-08, ex-handoff §6.8). The two wingtip emitters run off **separate ±5% internal-RC oscillators**, so their chip rates can differ by up to **~10%** and drift independently (of each other *and* of the receiver clock). The codes may be generated as a Gold pair, but at the **receiver** the summed signal must be decoded with a **separate, self-syncing chip-rate/phase loop per beacon** — each correlator searches and locks to *its own* actual chip rate, with its own DPLL and its own locked-rate value. Do **not** assume the two share a clock, phase, or integer ratio. This revises the "single LFSR pair" framing of [spec.md](spec.md) §5: the per-beacon independent lock is what makes the two-code CDMA separation (Stage 1) honest under real clock slip, and it is the receiver-side requirement the StepFPGA correlator must implement.
- **Data rate is low** (~0.2 MB/s at 100 kS/s) — *not* a high-bandwidth-SD problem. That's a camera-phase issue (18–46 MB/s → CrossLink-NX + HyperRAM), deliberately out of scope here.

## 6. Logging

- **Bench + ground (031 — all stages in scope)**: stream correlator output / raw ADC over **USB-UART** to a laptop. This covers Stage 0/1/2 — no onboard storage needed for the 031 gate. Host reader + frame contract framed in [`firmware/beacon-decoder-stepfpga/host/`](../../firmware/beacon-decoder-stepfpga/host/README.md). The StepFPGA UART (LPC CDC = Windows **COM3**) is read **on the Windows side via interop** (`host/monitor.sh` → PowerShell `SerialPort`), like the build — the board stays on Windows so `D:` stays flashable *and* COM3 readable simultaneously. **Proven end-to-end 2026-06-23.** (usbipd-attach-into-WSL was rejected: it pulls `D:` off Windows and the WSL CDC read hangs.)
- **Airborne (DEFERRED to the follow-on "pre-camera flight" feature, NOT 031)**: onboard storage. Options to settle in that feature: (a) StepFPGA → microSD via **raw-sector SPI writes** (good Lattice practice; ~11 KB on-chip buffer is fine at 0.2 MB/s); or (b) a cheap **RP2040/Teensy** logging the ADC / FPGA lock output via existing SD libraries. The microSD breakout + card are in the receiver BOM as a deferred add — exercised there, not in this phase.

## 7. Coding / acquisition knobs to sweep (Stage 0/1)

- **Chip rate**: baseline **200 Hz** (480 fps / 2.4 frames-per-chip → 5 ms/chip, 75 ms / 15-chip period). Supersedes the spec's 100 Hz (which was 240 fps × 2.4 fpc). Sweep 100–240 Hz (Nyquist cap = fps/2 = 240 Hz at 480 fps). The single-PD bench oversamples ~1000× so chip rate is free to explore here; 200 Hz is the value representative of the camera era.
- **Code length**: N=15 baseline (1-bit error tolerance, ~8-erasure track tolerance); 31-bit is the documented upgrade.
- **Early / partial-code acquisition**: declare tentative lock at ~70 % of the code (~55 ms) — the lever that keeps re-acquisition inside the 20 Hz control budget (75 ms full code = 1.5 ticks; ~70 % ≈ 1.1 ticks @ 200 Hz). Measure detection-vs-false-alarm at partial integration.
- **Soft-decision + erasure-aware correlation**: flips cost 2, erasures cost 1 — mark saturated/faded chips as erasures. Measure the gain.
- **Oversampling**: samples-per-chip vs acquisition reliability.

These are exactly what the sim (§9) predicts and the bench validates.

## 8. Deferred constraints (camera phase — keep visible)

- **20 Hz control loop (037)**: acquisition latency in *control ticks* — at the 200 Hz chip rate the full 15-chip code = 75 ms = **1.5 ticks** @ 20 Hz; early/partial acquisition trims it toward ~1 tick.
- **480 fps camera baseline**: drives the **200 Hz chip-rate choice** (2.4 fpc); Nyquist caps chip rate ≤240 Hz. The single-PD bench oversamples far beyond, so it validates the choice rather than being bound by it.
- **CEP / 2-D apparent motion**: the real CEP driver (blob-crossing-rate over the ~75 ms decode window) — only exercisable with the pixel array. Feeds [`BACKLOG.md` "CEP realism"](BACKLOG.md) + `038-accurate-m2` sensor grounding. **Note**: the single-PD bench *does* produce the *decode-confidence* half of CEP (correlation margin + dropout + lock ladder) — the two-component physical CEP model is captured in [`BACKLOG.md` "CEP — physical model"](BACKLOG.md) for a cleaner tracker-mode first pass (038). Deferred half = the spatial/apparent-motion term.
- **Localization / camera pipeline**: deferred; the stale full-camera BOM is [`verified-bom.md`](verified-bom.md) (bannered).

## 9. Sim support (predict, then test)

**Built** — [`acquisition-sim/sim.py`](acquisition-sim/sim.py) (031 characterization one-off; lives in this spec dir per the tooling convention). Reuses the Gold-code + matched-filter primitives and models what the bench should show — results in [`acquisition-sim/acquisition-results.md`](acquisition-sim/acquisition-results.md):
- **Time to positive lock vs fraction of code received** (full / 50% / 70%) → chips / ms / 20 Hz-ticks to lock.
- **Missing-bit (erasure) response** — single + multi, random vs consecutive.
- **Bit-flip response** — a wrong chip costs ~2× a missing one → mark fades as erasures.
- **Two codes on one detector (CDMA)** — acquire A with B summed in.
- Solar background is folded into the per-chip-SNR axis (lower SNR = more sky).

Outputs are the predicted curves Stage 0–3 measure against, and the calibration target for the eventual sim noise model. (The clock-drift one-off now lives alongside, in [`nfr4-clockdrift-sim/`](nfr4-clockdrift-sim/).)

## 10. Open questions

- LED actual center wavelength (unbinned) — measured on the bench (filter dip) → decides narrow vs wide filter for Exp B.
- Real solar/sky NIR background at the chosen FOV/filter → the true SNR margin vs the ~40 dB / ~10 dB-derated link-budget prediction.
- Dropout depth/duration envelope through real maneuvers (Exp A) → final code-length + soft-threshold choice.
- Acquisition-time knee vs chip rate / partial-code threshold under real SNR.
