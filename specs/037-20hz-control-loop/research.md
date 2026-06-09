# Phase 0 Research: Faster Control Loop (037)

**Date**: 2026-06-09 | **Status**: research-to-run (deliverables defined; several decisions deferred to
bench/datasheet/sim execution). Each item below is either **DECIDED** (settled in spec/grounding) or
**OPEN** (a research task with a defined output artifact). Phase A may not start until R1–R3, R5, R6 are
DECIDED; R4 runs in parallel; R7 (031 co-resolve) runs alongside R1–R3.

Format per item: **Decision / Rationale / Alternatives considered / Output**.

---

## R0. Grounding facts (DECIDED — from the 2026-06-09 code scan)

- **Decision**: The sim control cadence is governed by two constants that must move together —
  `gEvalUpdateIntervalMsec` (crrcsim `inputdev_autoc.h:53`, default 100 ms, env-overridable
  `AUTOC_EVAL_INTERVAL_MSEC`) and `SIM_TIME_STEP_MSEC` (autoc `aircraft_state.h:39`, 100 ms). FDM dt is
  `autoc_config.xml:157` (`dt=0.005`=200 Hz); outer fps `:3` (20 Hz). A startup assertion already enforces
  `cycleLengthMs | evalIntervalMsec`.
- **Rationale**: Establishes the exact knobs Phase A turns and the invariant they must satisfy.
- **Alternatives**: n/a (factual).
- **Output**: the touchpoint table in [plan.md](./plan.md#resolved-cadence-touchpoints-from-phase-0-grounding-scan-2026-06-09).

## RT. Smoothing theory — smooth vs relocate bang-bang (OPEN — gates the whole bake)

- **Decision (to produce)**: a defended **per-axis prediction** of whether a faster cadence smooths the
  command or just relocates bang-bang to the new Nyquist — *before* any retrain. See spec "Smoothing
  theory" section.
- **Rationale**: Faster ≠ smoother. A relay/sliding-mode-optimal controller will keep saturating-and-
  flipping at the new rate (the airframe low-passes a faster dither *better*). We need a first-principles
  reason to expect (A) aliasing-dither (smooths) over (B) relay-optimal (won't), so the bake is a
  confirmation, not a fishing trip.
- **Discriminators (from data we already have)**: (1) airframe roll time constant τ_roll from the FDM
  step-response trace + `Cl_p=-0.47`, compared to candidate tick T; (2) **raw pre-tanh roll drive** in t6
  traces — near the saturation knee ⇒ (A), deep ⇒ (B); (3) servo slew (low-passes command→motion) →
  define whether we optimize **command** or **motion** smoothness.
- **Alternatives considered**: empirical "try 20 Hz and look" (rejected — wastes bakes, and a null result
  wouldn't distinguish A from B); analytic-only without sim confirm (rejected — pair theory with the
  retrain that tests it).
- **Output**: the prediction (A or B per axis with the numbers) + the de-alias gate it implies. If (B),
  cheap no-go before any bake (lever is objective/airframe — out of scope).

## R1. Projected cadence from candidate hardware (OPEN — gates the rate)

- **Decision (to produce)**: a **projected NN loop rate** (the cadence we'd actually fly) + a per-part
  clock table. Not pre-picked to 20/40/48 — **projected from what candidate/similar hardware can sustain**
  (camera grid R2, transport ceiling R3, servo/IMU responsiveness), avoiding equal/near-equal unsynced
  collisions, and guided by RT's prediction of where (A) starts to win. Output is a small **selectable-rate
  shortlist** + the chosen projected rate, not a single guessed number.
- **Bundle note**: the projected rate is not trained alone — it derives a config **bundle** (cadence +
  latency/jitter model + history-bucket sizing + FDM rate) per spec "Smoothing theory §projected-cadence
  flow". R1 feeds the cadence; R5 the buckets; the latency model from R3+020; FDM from the cadence-config
  contract.
- **Rationale**: Achievable rates are set by real part tolerances, not nominal harmonics; the harmonic
  family only *bounds* beats, and equal-rate unsynced clocks are the worst case (spec correction
  2026-06-09). The real robustness is dt-aware control + per-source self-syncing correlators.
- **Alternatives considered**: (a) pick 20 Hz outright — rejected: doesn't use the camera-grid/transport
  facts and risks an unsynced collision; (b) full 10/20/50 sweep first — rejected: prove the lever on one
  researched rate first (spec).
- **Output**: table `{part → clock source (RC/crystal) → tolerance ±% → programmable-rate grid →
  implication}` for camera, Xiao nRF52840 time-base (HFXTAL vs RC), Lattice FPGA candidates; INAV I/O
  noted non-binding for rate (binds the MSP budget only).

## R2. Camera frame-rate grid + AGC/exposure budget (OPEN — co-resolved with 031)

- **Decision (to produce)**: the camera's chosen reachable frame rate (+ register recipe) and a fixed
  exposure/gain operating point for beacon detection. Candidate parts: **ST VD55G1** (primary), **OV9281**
  (backup) — both 300+ fps at ~320×240.
- **Rationale**: Frame rate is **near-continuous** via `pixel_clock/(HTS×VTS)` + PLL — 320/410/450/460/
  465/470/480 are all reachable, not a coarse menu. Bounds: max pixel-clock/PLL (fps ceiling per window),
  min HTS/VTS, line-time granularity. **AGC coupling**: `t_exp ≤ frame_period − readout`; higher fps →
  less exposure → more gain → less SNR. AEC/AGC must be **fixed/disabled** (auto-hunting aliases the
  pulsed beacon); pick fixed ~1 ms exposure (matches 1 kHz chip) + fixed gain at max range.
- **Alternatives considered**: free-running 480 assumption (rejected — quantized + tolerance-bound);
  auto-exposure (rejected — aliases beacon).
- **Output**: fps-vs-window curve for the chosen part; register recipe (PLL/HTS/VTS) for the selected
  rate; exposure/gain/SNR at candidate rates with beacon link-margin check.

## R3. Serial transport ceiling — read AND write (OPEN — gates the rate ceiling)

- **Decision (to produce)**: the transport decision (baud bump vs SPI) and the resulting read/write
  budget at the target rate.
- **Rationale**: 115200 is a **soft config limit** (both `inav-hb1.cfg` serial ports and
  `xiao msplink.cpp:342 Serial1.begin(115200)`), not hardware. Read (12.6 ms) + write (9.2 ms) share the
  pipe; raising baud to 921600/1M (≈8×) cuts these to ~1.5/~1.2 ms — likely removing the link as the
  binding limiter before any local-IMU work. SPI (nRF52840 SPIM ~32 MHz / STM32F7 ~25–50 MHz) is the
  ceiling option. **We already send at ~20 Hz with duplicate contents** (`MSP_NN_EVAL_DIVISOR=2`), so the
  send rate is partly proven; the open part is fresh read+write together at a higher rate.
- **Alternatives considered**: keep 115200 (rejected — caps rate well below 50 Hz); jump straight to SPI
  (deferred — try baud bump first, cheapest).
- **Output**: minimum command-frame cost at 115k and at the bumped baud; max sustainable read+write rate;
  baud-vs-SPI recommendation with the post-change budget. Cross-check/update `project_sim_latency`.

## R4. NN-eval cycle-count harness (OPEN — parallel; informs Phase C scope)

- **Decision (to produce)**: measured cycles/µs per (shape × tanh-impl × gather) on the real nRF52840,
  and the defended eval budget.
- **Rationale**: Don't trust the ~0.1 ms theoretical floor; measured eval is 2.7 ms avg / 7.4 ms max
  today (library `nn_forward_recurrent`, looped, `nn_program_generated.cpp`; timed with `micros()` not
  DWT). The harness decides whether unroll+fast-tanh (Phase C) is even needed at the chosen rate.
- **Alternatives considered**: FLOP-count estimate only (rejected — misses cache/flash-wait/preemption);
  on-target only (kept, plus an off-target op-counter for fast iteration).
- **Output**: on-target `DWT->CYCCNT` numbers for (a) MACs, (b) +tanh (`expf` vs poly/LUT), (c) +gather,
  for M1 (1923 w) and M2 (2595 w); off-target op-counter cross-check; fp32 kept for parity.

## R5. History time-basis (OPEN — settle in the Phase-A retrain)

- **Decision (lean, to confirm in retrain)**: move history to **time-based / log-spaced lags**, choosing
  N on a *time* basis — not "just add slots." Current: M1 6 slots (`nn_inputs.h:36-58`), M2 `span[6]` +
  beacon `[6]` (`evaluator.h:112-127`), uniform 100 ms past-only (`HIST_PAST[]`), error ring
  `HISTORY_SIZE=10` hardcoded "1 s @ 10 Hz".
- **Rationale**: At a faster cadence, fixed-N-ticks shrinks the trend window (6 ticks: 600 ms→300 ms at
  20 Hz), and it's already 15× too short vs lost-sight (`project_032_phase1_setup`). Log/Fibonacci-spaced
  lags hold a much longer window with few slots and are ~rate-invariant in time.
- **Alternatives considered**: (a) more uniform slots — rejected: grows NN input linearly, still
  tick-bound; (c) full time-resampling — viable, heavier; log-spaced is the lean middle.
- **Output**: chosen lag set (e.g. t−1,−2,−3,−5,−8,−13…), the new `NN_INPUT_COUNT` for M1/M2, and the
  Principle-V layout note (see contracts/nn-input-layout.md). **This changes the dmp/NN layout.**

## R6. Cadence config representation (OPEN — small, do with Phase A)

- **Decision (to confirm)**: unify the control-rate cadence into a single source of truth read from
  `.ini`/CLI (not the `AUTOC_EVAL_INTERVAL_MSEC` env var), keeping `gEvalUpdateIntervalMsec` and
  `SIM_TIME_STEP_MSEC` coherent.
- **Rationale**: `feedback_cli_over_env_vars` (tooling configures via CLI/ini, not env); two constants for
  one cadence is a drift hazard. Per Principle VII, the new key carries no in-class default.
- **Alternatives considered**: keep env var (rejected — violates the convention, and the rate is now a
  first-class experiment knob); fully merge the two constants into one (preferred if low-risk; confirm the
  crrcsim/autoc boundary allows it).
- **Output**: the cadence config contract (contracts/cadence-config.md) + the chosen representation.

## R7. 031 co-resolve (OPEN — runs with R1–R3)

- **Decision (to produce)**: a jointly-feasible {control rate, camera part, decoder FPGA} point.
- **Rationale**: Two-way loop — 037→031 the rate sets 031's acquisition/frame/chip budget; 031→037 the
  candidate parts' clocks/fps/PLL bound the rate (`docs/aircraft_tracker_handoff.md` §4). Neither is
  upstream; co-resolve from the options at both ends.
- **Alternatives considered**: treat 031 as purely downstream (rejected — its part clocks bound R1/R2).
- **Output**: write the chosen rate back into `aircraft_tracker_handoff.md` as the acquisition-budget
  input; pull 031's part shortlist forward as R1/R2's parts-to-characterize; carry the per-beacon
  independent-correlator requirement.

---

## Tick-rescale audit (DECIDED scope — feeds Phase-A tasks)

The cadence change is **not just the interval constant**. Confirmed per-tick/cadence-coupled terms:

| Term | Site | Action |
|---|---|---|
| streak ramp | `fitness_decomposition.cc:33-34` | none — already `fitStreakRampSec/(SIM_TIME_STEP_MSEC/1000)` ✅ |
| stability accum | `fitness_decomposition.cc:179` | normalize per second (Σ → Σ·dt) or rescale — **was unnormalized** |
| energy accum | `fitness_decomposition.cc:180` | same — normalize/rescale |
| closing_rate | `evaluator.cc:333` `/0.1f` | replace with actual dt |
| closure_rate | `fitness_decomposition.cc:212` | uses `SIM_TIME_STEP_MSEC/1000` ✅ but verify magnitude semantics |
| thrash_rate | `fitness_decomposition.cc:233-244,301-305` | per-second already; raw count granularity changes — document |
| error ring | `aircraft_state.h:330` `HISTORY_SIZE=10` | resize on time basis (R5) |
| engage delay | `023 contract` (not in code) | implement `ceil(ms/step)` rate-independent |
| SIM_TOTAL_TIME_MSEC | `aircraft_state.h:40` | unchanged (time-based) — tick count doubles, fine |

**Comparison discipline**: per `project_late_run_fitness_interpretation`, compare rates with fixed-eval
and the variation-stable per-axis dctrl/sign-flip comparators, not raw training fitness.
