# Implementation Plan: 038 Accurate M2 — Architecture for Tracking Depth

**Branch**: `038-accurate-m2` | **Date**: 2026-06-27 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/038-accurate-m2/spec.md`

## Summary

038 attacks the M2 (optical tracker) **tracking-depth ceiling** at the architecture level, after 037
proved the depth is *architecture-capped, not reward-limited* (every reward knob across t11–t15 only
relocated trade-offs; close-tracking stayed ~11–13 %, median error ~17 m, in-FOV ~70 %, reacquire 8–10 s).

Scope settled at clarify (2026-06-27):
1. **Phase 0 — prework / tech-debt** (P0-A…P0-G): 033 PRNG validation, renderer-reads-dmp config-hygiene,
   standardized report script, the **one clean-slate dmp break** (simTimeMsec stamping + self-describing
   dmp + wind_velocity recording), 037 carry-forward housekeeping, the t15 streak-threshold revert
   (0.3→0.5), and the M1-re-bake expectation.
2. **Generic RNN architecture studies** (US1 deeper/non-uniform history, US2 two-timescale recurrence,
   US3 auxiliary target-predictor head) — run as **parallel independent ablations** off the same baseline,
   then combine winners. **Mixed M1-first**: US1 gated on an M1 ablation; US2/US3 may go straight to M2.
3. **US4 visibility-maintenance reward** — the one FOV-specific deliverable; a new lexicase scenario axis.

**Deferred to a follow-on**: US5 (camera variations + prediction-through-blindness), blocked on US3's
predictor winning + 031's incoming CEP rules. **Success gate**: any one reward-invariant 037 ceiling moves.

The technical approach is grounded in a full integration map (see [research.md](research.md)): the NN
topology is compile-time `constexpr` in `topology.h`, history is `kNNHistoryLagsMsec` in `nn_inputs.h`,
fitness flows through `FitnessComputer`/`computeScenarioScores`/`lexicase_select`, and the dmp contract is
`EvalResults`/`AircraftState` cereal. Every architecture change is **format-breaking** (retrain from
scratch, xiao contract update, no cereal version bump, fail-loud read) and is bundled into P0-D's single
dmp break so there is exactly one determinism break for the whole feature.

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim), C++ (xiao / PlatformIO arduino-mbed), Python 3.11 (analytics)
**Primary Dependencies**: Eigen (vec3/quat math), cereal (NN01 + EvalResults + dmp serialization), inih
(ini parsing), GoogleTest (unit/contract), CRRCSim LaRCSim FDM (sim physics); analytics: numpy, matplotlib
**Storage**: file-based — `data.dat` (per-tick trace), `data.stc` (per-gen aggregate), S3 per-mode buckets
`autoc-m1` / `autoc-m2` / `autoc-eval` (`.dmp[.zst]`, cereal `EvalResults`); local `.ini` config
**Testing**: GoogleTest (`run_autoc_tests`); contract tests for PRNG cascade + eval-mode replay; the
**eval-vs-training bitwise regression gate** (`rebuild-perf.sh`, single-threaded FP-deterministic, compares
per-scenario `ScenarioScore` vectors — NOT whole-dmp bytes)
**Target Platform**: Linux desktop (train: autoc + crrcsim FDM workers); Seeed XIAO BLE Sense (deploy)
**Project Type**: Single C++ project (evolution engine + FDM submodule + embedded target) + Python analytics
**Performance Goals**: 20 Hz control loop (50 ms cadence, `SIM_TIME_STEP_MSEC=50`); FP-determinism for the
bitwise gate; xiao NN forward-pass within the real-time slot (codegen via `nn2cpp`)
**Constraints**: absolute per-scenario determinism + eval-vs-training bitwise parity (FR-030); optical-only
M2 (no physical target-range input, unitless closure); no cereal version bump (greenfield, fail-loud read);
xiao firmware contract must track any NN input/output/topology change
**Scale/Scope**: pop ~8000, 294 scenarios/eval (M2), ~800 gen runs; NN tracker topology 54→32→16r→3
(2595 weights); 6-slot history window; detached multi-day bakes via `scripts/train.sh`

**NEEDS CLARIFICATION (deferred to plan-phase research.md, per spec's intentional `[NEEDS RESEARCH]`):**
- US1 history layout: depth (→1.6 s?), step count (>6?), spacing scheme (log / Fibonacci / exp / geometric)
- US2 two-timescale recurrence structure: fixed-leak vs evolved time-constant; new layer vs split channel
- US3 predictor head: head shape, prediction target + horizon, lexicase prediction-accuracy objective
  design (must NOT use scalar compositing), pure-evolve vs gradient-pretrain-then-evolve
- US4 visibility-maintenance reward shape (CEP-gate vs continuous in-FOV term)

These are research outputs of the study phase, NOT pre-committed here (spec Out-of-Scope). research.md
records the candidate approaches + decision criteria so each ablation has a concrete starting point.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Constitution v1.8.0 (10 principles). Assessment for 038:

| Principle | Status | Notes |
|---|---|---|
| **I. Testing-First** | ✅ PLAN | Each architecture change ships GoogleTest coverage (topology/layout/serialization/replay tests already exist and must be regenerated). P0-A is itself a determinism-test deliverable. US4 reward needs a fitness unit test. Ablations are research spikes (Exemption) but any mainlined code carries tests. |
| **II. Build Stability** | ✅ PLAN | Every commit compiles + passes tests; xiao build (`pio run`) must stay green across NN-contract changes. Format-breaking changes land coherently (no half-broken intermediate states). |
| **III. No Compatibility Shims** | ✅ PLAN | Clean-cut: old dmps/genomes orphaned, no migration shims. P0-F reverts the streak threshold by editing inis directly. Update all call sites (FR carried: M2-era no-fallback-param). |
| **IV. Unified Build** | ✅ PLAN | New tests register via top-level CMakeLists.txt; format/determinism-affecting changes use `rebuild-perf.sh` (operator-driven), source-only edits incremental. crrcsim submodule change (P0-D simTimeMsec) → pointer-bump-first per submodule merge order. |
| **V. Versioned Persistence** | ⚠️ GATE | dmp (`EvalResults`/`AircraftState`) + NN01 are the load-bearing artifacts. P0-D bundles ALL format breaks into ONE. Per spec + project practice: **NO cereal version bump** (greenfield); readers **fail loud** on mismatch (the safety net). This is the deliberate tension with V's "bump directly" — resolved by the project's no-cereal-versioning practice (greenfield, fully-committed transition, fail-loud read). Documented in research.md. |
| **VI. Type-Domain Discipline** | ⚠️ GATE | New eval/nn code (history layout, predictor head, visibility reward) MUST use `gp_scalar`/`gp_fitness`/`gp_vec3`/`gp_quat`. P0-E includes the `src/eval/ src/nn/` grep audit. Per-milestone grep at `/speckit.implement` close. NN-byte-format buffers stay `// raw-ok:`. |
| **VII. No Silent Fallback Defaults** | ✅ PLAN | New constructor-supplied members (predictor config, visibility-reward params from config/WorkerInit) MUST NOT carry in-class defaults — single assignment site. The 032 `cepGateThreshold` bug is the cautionary case; applies directly to new fitness/NN config. |
| **VIII. Artifact Lifecycle** | ✅ PLAN | New bakes tag `retain=expire`; pin only milestones (a winning architecture baseline) and record the S3 prefix in the outcome doc. Uniform run-id naming `autoc-038-tN-<details>` per artifact-naming convention. |
| **IX. Detached Training Launch** | ✅ PLAN | All bakes via `scripts/train.sh` (never `run_in_background`). **Pre-run build gate**: clean build + tests before each larger run (`make` ordinary; `rebuild-perf.sh` for determinism-affecting). Operator drives launches + the regression gate. |
| **X. Single Ordered Backlog** | ✅ PLAN | Deferred items (US5, SQLite analytics store, `EvalVariationScaleOverride`) stay in `specs/BACKLOG.md`, not new memory files. |

**Gate verdict**: PASS. The two ⚠️ items (V no-version-bump, VI type-domain) are not violations — they are
explicit, practice-sanctioned handling, recorded in research.md and enforced at implement-time. No entry
in Complexity Tracking required.

## Project Structure

### Documentation (this feature)

```text
specs/038-accurate-m2/
├── plan.md              # This file
├── research.md          # Phase 0 — resolves the [NEEDS RESEARCH] markers (history/recurrence/predictor/reward)
├── data-model.md        # Phase 1 — schema deltas (history layout, NN topology, EvalResults config block, ScenarioScore axis)
├── quickstart.md        # Phase 1 — how to run the ablations, the gate, the report wrapper
├── contracts/           # Phase 1 — dmp/NN01/config contract specs + the new visibility-reward fitness contract
└── tasks.md             # Phase 2 — /speckit.tasks output (NOT created here)
```

### Source Code (repository root)

```text
include/autoc/
├── nn/
│   ├── nn_inputs.h          # kNNHistoryLagsMsec (US1), kNNHistoryLayoutVersion, NNInputs/TrackerInputs layout (US3)
│   ├── topology.h           # NN_TOPOLOGY / TRACKER_NN_TOPOLOGY, NN_RECURRENT, weight/state counts (US2, US3)
│   ├── evaluator.h          # NNGenome, NNControllerBackend, TrackerObservationRing::kDepth (US1)
│   └── mode.h               # ModeStrategy bundle (pathgen vs tracker)
├── eval/
│   ├── aircraft_state.h     # history buffer, HISTORY_SIZE, cereal serialize, simTimeMsec, wind_velocity (P0-D)
│   ├── fitness_computer.h   # decomposeStepScore, applyStreak, cone config
│   ├── fitness_decomposition.h  # ScenarioScore (+visibility axis US4), TrackerDiag, computeScenarioScores
│   ├── selection.h          # lexicase pool axes (US4 new axis)
│   └── scenario_meta_apply.h    # applyVariationScale (reward ramp)
├── rpc/
│   ├── protocol.h           # EvalResults (P0-D self-describing config block), BeaconObservation/CameraViewSample
│   └── scenario_metadata.h  # scenarioSeed, cameraSeed insertion point (deferred US5)
└── util/
    ├── config.h             # AUTOC_CONFIG_FIELDS X-macro (new knobs), fit* params, crash-penalty knobs
    └── scenario_prng.h      # MasterPRNG/ScenarioRootPRNG/ClassPRNG cascade (P0-A validation)

src/
├── nn/        evaluator.cc (forward pass, recurrence US2, predictor head US3), serialization.cc (NN01), mode.cc, population.cc
├── eval/      fitness_computer.cc, fitness_decomposition.cc (US4 reward), selection.cc, tracker_stepper.cc, camera_projection.cc
├── util/      config.cc (X-macro parse)
├── autoc.cc   computeVariationScale, applyCrashPenalty, computeScenarioScores call, startup config print
└── analytics/ dmp-fed plotters (P0-C maintained home) + requirements.txt → pyproject.toml

crrcsim/ (submodule)
└── src/
    ├── SimStateHandler.cpp                          # getSimulationTimeSinceReset truncation (P0-D-1)
    └── mod_inputdev/inputdev_autoc/
        ├── inputdev_autoc.cpp                       # setSimTimeMsec; wire setWindVelocity (P0-D-3)
        └── crrcsim_tracker_helper.cpp               # source-spacing check → revert to strict gap

tools/      renderer.cc + dmp_dump.cc (P0-B read-from-dmp), nn2cpp.cc (xiao codegen), nnextractor.cc
xiao/       src/generated/nn_program_generated.cpp (regen on NN-contract change), include/nn_program.h
tests/      nn_layout_tests, nn_evaluator_tests, nn_serialization_tests, scenario_prng_tests, eval_mode_replay_tests, tick_rescale_tests
scripts/    train.sh, rebuild.sh, rebuild-perf.sh, generate_pngs.sh (P0-C wrapper)
*.ini       autoc.ini (M1), autoc-tracker.ini (M2), autoc-eval*.ini (gate), autoc-basic-m1.ini (smoke)
```

**Structure Decision**: Single mature C++17 project with a crrcsim submodule, an embedded xiao target, and
a Python analytics package — no new top-level structure. 038 edits existing modules in place; the only new
*files* are spec artifacts, possibly a `src/analytics/pyproject.toml` (P0-C), and regenerated test fixtures.

## Execution model — gated milestones with escape (operator 2026-06-27)

038's substantive research is **empirical** — which history layout / recurrence structure / predictor head
actually moves a ceiling is answered only by ablation bakes (days each, operator-in-the-loop), not by a
plan-time document. So `research.md` is the **launchpad** (desk-resolvable facts: the map, the P0-D break,
each ablation's starting config + decision criteria, the protocol), and the real research lives in the
phases.

Tasks are therefore structured as a **series of internal checkpoints / milestones, each with a distinct
acceptance gate** rather than a linear build-it list:

- Each milestone (P0-A…P0-G; each ablation US1/US2/US3; US4) has its own acceptance criterion tied to a
  spec SC / FR.
- **Escape is a first-class outcome at every gate.** If an ablation does not clear its gate (no ceiling
  moves per SC-001, or cost outweighs lift), it **escapes to `specs/BACKLOG.md` or a future feature** —
  the milestone closes as "ruled out / deferred," it does not block the feature.
- A genuine **branch/decision node** sits after the parallel ablations: judge winners against the SC-001
  ceilings (fixed-eval comparator), then **combine winners** into the final architecture. Which winners
  combine cannot be pre-sequenced — it is decided at that gate from the bake results.
- Phase 0 (P0-A…P0-G) is the **deterministic** part (normal implement work + the one P0-D break); the
  ablations are the **experiment-loop** part (bake → judge → keep/escape).

This keeps the feature honest about its empirical nature: the plan's job is to set up the milestones and
gates correctly; the depth-win is discovered in execution, and dead ends route to backlog cleanly.

## Phase 0 — Outline & Research

research.md resolves the spec's intentional `[NEEDS RESEARCH]` markers into concrete, decision-criteria'd
starting points for the ablations (it does NOT pre-commit final forms — those emerge from the bakes):

1. **US1 history layout candidates** — depth/count/spacing options (current `{800,400,200,100,50,0}` 6-slot
   0.8 s log-spaced vs deeper 1.6 s, more steps, alternative spacings) + the format-break surface
   (`kNNHistoryLayoutVersion`, `HISTORY_SIZE`, `TrackerObservationRing::kDepth`, `HIST_PAST[]`, xiao codegen).
2. **US2 two-timescale recurrence** — structural options (parallel slow channel widening hidden-2 vs an
   inserted second recurrent layer vs evolved per-unit time-constants) + weight/state-count + NN01 impact.
3. **US3 predictor head** — output-head shape, prediction target (next-tick beacon NDC / span), horizon,
   and the lexicase prediction-accuracy objective as a SEPARATE axis (never scalar-composited — sidesteps
   the 033 Pareto collapse); pure-evolve vs pretrain-then-evolve.
4. **US4 visibility reward** — CEP-gate vs continuous in-FOV term, as a new `ScenarioScore` lexicase axis,
   ramp behavior, M1=0 invariant.
5. **P0-D contract-break design** — simTimeMsec stamp-by-step-count, self-describing `EvalResults` config
   block, wind_velocity wiring; one break, no version bump, fail-loud read.
6. **Cross-cutting** — the no-cereal-version-bump V/practice tension resolution; the mixed-M1-first ablation
   protocol; the regression-gate procedure for each format change.

**Output**: research.md with each marker resolved to Decision / Rationale / Alternatives.

## Phase 1 — Design & Contracts

1. **data-model.md** — the entity/schema deltas: revised history layout (constants + version), NN topology
   (US2 layer/state, US3 output count), `EvalResults` self-describing config block (P0-D), `AircraftState`
   simTimeMsec/wind_velocity (P0-D), new `ScenarioScore.visibility_score` axis (US4), and the (recorded,
   deferred) `cameraSeed` slot.
2. **contracts/** — the format contracts that downstream consumers depend on: NN01 (topology/recurrent/
   weight-count), dmp `EvalResults` v-schema (the one P0-D break), the lexicase-axis contract for the new
   visibility reward, and the xiao firmware NN-contract (input/output/topology/cadence sync points).
3. **quickstart.md** — operator runbook: how to launch each ablation via `train.sh`, run the pre-run build
   gate, the eval-vs-training bitwise gate, and regenerate the report set via `scripts/generate_pngs.sh m2`.
4. **Agent context update** — run `.specify/scripts/bash/update-agent-context.sh claude`.

**Output**: data-model.md, contracts/*, quickstart.md, updated agent context.

## Complexity Tracking

> No constitution violations requiring justification. The format-breaking NN/dmp changes are bundled into
> P0-D's single agreed dmp break (operator-sanctioned, no version bump, fail-loud read) — this is the
> *simpler* alternative to per-change migration shims (Principle III), not a complexity addition.
