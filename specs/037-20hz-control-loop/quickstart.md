# Quickstart: 037 Faster Control Loop

How to execute the gating work. Phase 0 + A are the near-term path; B/C are conditioned on the Phase-A
gate.

## Phase 0 — theory + research (do first; gates the rate AND the bake)

1. **Smoothing theory (RT)** — predict, from first principles, whether faster cadence smooths the roll
   command or just relocates bang-bang to the new Nyquist: extract τ_roll from the FDM step-response
   trace + `Cl_p=-0.47`; inspect raw pre-tanh roll drive in t6 traces (near-knee ⇒ smooths; deep ⇒
   relay); decide command-vs-motion smoothness target. A (B)-relay verdict = cheap no-go before any bake.
2. **Clock/transport survey → projected cadence (R1/R3/R7)** — datasheet + bench, co-resolved with 031's
   part shortlist (`docs/aircraft_tracker_handoff.md` §4). Produce the part clock table, the camera fps
   grid + AGC budget (R2), the transport decision (try baud bump 115200→921600/1M first), and the
   **selectable-rate shortlist → projected NN loop rate** we'd actually fly. → research.md.
3. **Cycle-count harness (R4)** — build the `xiao/` bench target; measure DWT cycles for M1/M2 ×
   {macs,+tanh,+gather}. Runs in parallel. → contracts/eval-cycle-harness.md.
4. **Output**: the smoothing prediction + the **projected rate and its derived training-config bundle**
   (cadence + latency/jitter model + history-bucket sizing + FDM rate) + transport + eval budget. Phase A
   cannot start until the rate is projected and the bundle defined.

## Phase A — sim retrain + de-alias gate (the go/no-go)

Prework landed first (its own early phase — see tasks.md Phase 2):
- **P3 short-source fix** (revert `c95887e` erase; keep 294 1:1, neutralize in place) — gates clean M2.
- **P1 entry-envelope** sigmas: `config.h` `entryConeSigma 30→18`, `entrySpeedSigma 0.10→0.05–0.06`
  (clamp stays 2.5σ). Clean step.
- **P2 logfile slim** (after dmp-reconstructability check) and **P4 renderer focus-mode single-arena**
  (needed to *use* the renderer at ≥40 Hz).

Steps:
1. Set the cadence config to the researched rate (contracts/cadence-config.md). Keep
   `gEvalUpdateIntervalMsec` ≡ `SIM_TIME_STEP_MSEC`; raise `autoc_config.xml` `fdmDt` (≥500 Hz, or 2 kHz)
   if the arm is ≥50 Hz.
2. Apply the **tick-rescale** edits (research.md audit): normalize stability/energy, fix `closing_rate`
   `/0.1f` → dt, implement rate-independent engage-delay. Add the cadence-invariance unit tests.
3. Apply the **history time-basis** (R5): log/time-spaced lags; update `NN_INPUT_COUNT`/topology;
   fail-loud loader (contracts/nn-input-layout.md). Run the layout round-trip + fail-loud tests.
4. **Build**: `bash scripts/rebuild.sh` (correctness) and the operator-driven `rebuild-perf.sh`
   coherence/regression build if CMakeLists changed.
5. **Retrain M1** at the researched rate via `scripts/train.sh <ini> <logfile>` (Principle IX — never the
   agent's `run_in_background`). Tag dumps `retain=expire`; name `autoc-037-t<N>-<rate>...`
   (`feedback_artifact_naming_convention`).
6. **Measure the gate** (fixed-eval, per `project_late_run_fitness_interpretation`): roll lag-1 autocorr
   negative→≥0 AND sign-flip −≥20 pts (t6 56%→≤36%) at tracking within noise of **historical t6 10 Hz**
   (Q3). Use dmp-dump-based analytics (new script, not historical scripts —
   `feedback_historical_scripts_immutable`).
7. **Milestone close**: per-milestone type-domain audit on touched eval/nn + operator regression gate
   (bit-replay at unchanged 10 Hz) — Principle VI/the gate run here, not only at final polish.
8. **Decision**: clears → **US1b M2 retrain** (if signal) and/or Phase B at that rate; persists →
   Phase C; not a clear win → cheap no-go, stop before firmware. Write the outcome doc; cross-link the
   chosen rate into `aircraft_tracker_handoff.md` (R7).

## Phase A (M2) — tracker retrain, gated on the M1 signal (US1b)

If Phase A (M1) shows a sensible higher-rate signal, retrain M2 (tracker) at the same projected cadence
on the already-implemented M2 layout/rescale; requires the P3 short-source fix for a clean M2 bake.
Measure M2 de-alias + tracking vs the M2 baseline; the result decides whether the flown controller is
M1, M2, or both.

## Phase B — embedded ~20 Hz (only if A clears at ~20 Hz)

- Set `xiao MSP_NN_EVAL_DIVISOR=1` (20 Hz NN) once read+write fits the link (transport-link.md).
- Bring up `xiao/src/imu_local.*` (LSM6DS3 TWIM-DMA + complementary fusion, INAV slow sync, auto-cal
  alignment). **Prereq: execute the 021 convention cross-check.**
- P5 packed logging (now in-scope). `activate → capture → confirm → promote → flight → sim↔real`.

## Phase C — 50 Hz stretch (only if A shows 20 Hz insufficient)

- Unroll + fast-tanh (guided by R4 budget); real-time slot scheduling; tail-bounding (async flash,
  NVIC priorities, DMA completion ISRs). (renderer focus-mode is now Phase-A prework.)

## Guardrails

- Operator drives the bitwise regression gate (`feedback_operator_runs_regression_gate`).
- Never rebuild autoc while a training run is live (`feedback_no_rebuild_during_training`).
- Incremental builds for iteration; full `rebuild-perf.sh` only for the gate (Constitution IV).
- Type-domain grep audit on touched `src/eval` `src/nn` files before closing the milestone (Principle VI).
