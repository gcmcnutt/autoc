# Implementation Plan: 023 — OOD Coverage, Engage Transient, Throttle Discipline

**Branch**: `023-ood-and-engage-fixes` | **Date**: 2026-04-08 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/home/gmcnutt/autoc/specs/023-ood-and-engage-fixes/spec.md`
**Predecessor**: [022-tracking-cone-fitness](../022-tracking-cone-fitness/spec.md) (shipped, flown 2026-04-07)

## Summary

Feature 023 addresses the three root causes of the failed 2026-04-07 flight and hardens the
training pipeline against a class of silent bugs that has bitten us repeatedly. The primary
technical lever is a **structural elimination of the dPhi/dTheta atan2 wrap discontinuities**
(40x higher rate in real flight than in sim training) by replacing the two angles with a
3-axis direction cosine representation that cannot wrap by construction. Ancillary work:
engage transient fix, INAV engage delay modeling in sim, throttle lexicase dimension,
discontinuity-forcing training paths, craft parameter variations, and iterative authority
limiting.

Two **blocking prerequisites** land first:

1. **Type-safe NN sensor interface refactor** (`NNInputs` struct-of-floats with `static_assert`
   on layout) — compiler-enforced topology consistency. Required before the 27→33 input
   migration to prevent a repeat of the 021 silent serialization corruption bug.
2. **Train/eval scenario construction de-duplication** — the BACKLOG "Bug 1/2/3" family is
   really one root cause: `runNNEvaluation()` has drifted from the training path. Refactor
   extracts a shared helper. Acceptance: eval on saved betterz2 weights reproduces training
   fitness within FP rounding (determinism cross-check).

Execution discipline: a **3-rung validation ladder** — minisim smoke test (plumbing gate,
not tracking quality) → CRRCSim incremental training milestones (observational layering,
mirrors 022 betterz1/betterz2) → xiao field-test prep. Minisim does not support variations;
its role is narrowly "prove the new code path can train *anything*."

**Flight validation gate**: the next flight's blackbox analysis must show **zero** projection
discontinuities in the bearing inputs. This is a structural check on Change 6, not a
statistical one — the 3-cosine representation cannot wrap, so any discontinuity in real
flight means the pipeline is bugged somewhere.

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim, xiao), Python 3.11 (analysis scripts, data.dat parsers)
**Primary Dependencies**: Eigen (vec3/dot), cereal (serialization), inih (config), GoogleTest, CRRCSim LaRCSim FDM, INAV MSP protocol, PlatformIO (xiao target)
**Storage**: File-based — `autoc.ini`, `data.dat`, `data.stc`, xiao flash logs, S3 for training artifacts
**Testing**: GoogleTest for autoc unit + integration tests; `bash scripts/rebuild.sh` as the canonical build + test driver; deliberate-break verification for type-safe NN interface
**Target Platform**: Linux (aarch64 training host + x86 workstation); Seeed XIAO BLE Sense nRF52840 (embedded target via PlatformIO `xiaoblesense_arduinocore_mbed`)
**Project Type**: Multi-component — autoc (evolution engine, C++17), crrcsim (C++ FDM, git submodule), xiao (PlatformIO embedded), plus Python analysis tools. Unified top-level CMake per Constitution IV.
**Performance Goals**:
- xiao NN inference: ~280 μs per forward pass at `{33, 32, 16, 3}` on nRF52840 FPU (negligible vs 100 ms tick budget)
- CRRCSim training: no regression from 022 baseline (~16K sims/sec)
- Expected evolution walltime: 1.5–2x 022's 400-gen run due to 2.7x weight count
**Constraints**:
- Zero projection discontinuities in real-flight blackbox (structural, not statistical)
- Compile-time topology consistency: changing `NN_INPUT_COUNT` must propagate or produce compile errors at every dependent site
- `data.dat` and xiao flash logs must emit one column per NN input field AND one column per NN output field, every tick, with no coverage regression
- Eval-mode fitness on saved weights must reproduce training-time fitness within FP rounding
- No new trig calls on xiao (direction cosines eliminate atan2 without introducing cos/sin)
- 10 Hz NN tick rate (100 ms SimTimeStepMs) — 20 Hz is Out of Scope for 023
**Scale/Scope**:
- NN: 33 inputs → {32, 16} hidden → 3 outputs, 1667 weights (vs 022's 611)
- Training: population 3000, ~400 generations, ~245 scenarios per individual at full curriculum
- Real flight: 10 Hz NN cycle, ~300–600 samples per autoc span, blackbox at 1/32 rate
- Affected files: ~15 autoc/xiao files for the type-safe refactor, 1 CRRCSim file (`inputdev_autoc.cpp`) for engage delay modeling, 1 durable research doc in `~/autoc/docs/` (INAV audit), 1 one-off research doc in the feature dir (train/eval dedup survey), 3 new test suites

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The constitution defines 4 principles: **Testing-First**, **Build Stability**, **No Compatibility Shims**, **Unified Build**.

### I. Testing-First — PASS

023 follows test-driven workflow:

- **Type-safe NN interface (Phase 0a)**: verification test runs BEFORE the 27→33 migration — refactor the struct at 27 inputs with all existing tests still passing, then deliberately break `NN_INPUT_COUNT` and confirm compile error. This is the "test meaningful failure first" step.
- **Train/eval dedup (Phase 0b)**: acceptance test is "eval on betterz2 weights reproduces training fitness within FP rounding." Concrete, runnable, fails before refactor (that's the bug) and passes after.
- **Change 6 (direction cosines)**: new unit tests for `computeTargetDir()` including the singularity fallback at `dist < 1e-4`. Existing `fitness_decomposition_tests.cc`, `fitness_computer_tests.cc`, `nn_evaluator_tests.cc`, `contract_evaluator_tests.cc` all updated to the new struct.
- **Change 1 (engage transient)**: new unit test on `resetHistory()` covering first-engage and re-engage scenarios with a synthetic `AircraftState`.
- **Change 1b (engage delay modeling)**: CRRCSim-side change requires a minisim integration test that verifies NN outputs are ignored during the delay window but inputs update normally.
- **Success criterion #9 (logging coverage)**: explicit diff-test comparing `data.dat` column count and per-tick values pre- and post-refactor at identical seed+weights.

### II. Build Stability — PASS

The plan's three-rung ladder is explicitly designed around build stability:

- Phase 0a.1 refactors the struct without changing behavior. All tests must pass. Only then does Phase 0a.2 apply the 27→33 migration.
- Phase 2 (minisim smoke test) requires a clean `bash scripts/rebuild.sh` + all unit tests passing + 50+ gens of successful minisim training as a precondition to CRRCSim promotion.
- Xiao build verification (`pio run -e xiaoblesense_arduinocore_mbed`) required before any field-test prep.

No known build-stability risks. CRRCSim submodule pointer bumps follow the 022 lesson (submodule-first, then parent — captured in memory from the 022 merge session).

### III. No Compatibility Shims — PASS

The plan is explicit about clean-cut migrations:

- Type-safe NN struct refactor: all ~15 callers updated in one pass. No `float[]` alias.
- 27→33 migration: one commit, compiler catches misses.
- Train/eval dedup: the old duplicated scenario setup is deleted, not kept as an "old path" wrapper.
- Change 6: no `getVirtualPosition()`-style deprecated alias. `dPhi`/`dTheta` are removed entirely from the NN input path in one commit.
- Change 1b engage delay: adds new code; the old "NN commands start immediately" path is deleted, not kept under a feature flag.

The one place we DO ship a config knob is `NNAuthorityLimit` (Change 8), which defaults to 1.0 (no change from pre-023 behavior) and is a tuning knob, not a compatibility shim. Similarly `EngageDelayMs` is a runtime config, not a compatibility alias.

### IV. Unified Build — PASS

No changes to the unified CMake structure. FetchContent dependencies unchanged. CRRCSim remains `add_subdirectory(crrcsim)`. New test suites land under the existing `tests/` directory and are picked up automatically.

### Complexity Tracking

No constitutional violations requiring justification. The plan is well within the established patterns.

## Project Structure

### Documentation (this feature)

```text
specs/023-ood-and-engage-fixes/
├── plan.md                      # This file
├── research.md                  # Phase 0 output — index into research artifacts + decisions
├── train-eval-code-dedup.md     # Phase 0 output — one-off survey, obsolete after Phase 0b refactor lands
├── data-model.md                # Phase 1 output — NNInputs struct, NN topology, data.dat columns
├── quickstart.md                # Phase 1 output — minisim smoke test recipe
├── contracts/                   # Phase 1 output
│   ├── nn_interface.md          # NNInputs struct contract, topology invariants
│   ├── engage_delay.md          # CRRCSim engage delay window behavior
│   └── history_reset.md         # resetHistory() semantics
├── spec.md                      # (exists — feature specification)
└── tasks.md                     # Phase 2 output — generated by /speckit.tasks

docs/                            # Durable cross-feature reference material (NOT feature-specific)
└── inav-signal-path-audit.md    # Phase 0 output — long-lived, may inform 024+ architectural decisions
```

### Source Code (repository root)

023 affects files across all three components: autoc (training engine), crrcsim (FDM submodule),
and xiao (embedded target). No structural reorganization — additions slot into the existing layout.

```text
# autoc (training engine, top-level repo)
include/autoc/
├── nn/
│   ├── nn_inputs.h          # NEW — canonical NNInputs struct (Phase 0a.1)
│   ├── topology.h           # UPDATED — derive NN_INPUT_COUNT from sizeof(NNInputs)
│   └── evaluator.h          # UPDATED — forward() takes NNInputs const&
├── eval/
│   ├── aircraft_state.h     # UPDATED — store NNInputs, update cereal serialization
│   ├── fitness_computer.h   # (unchanged — 022 conical surface still canonical)
│   └── fitness_decomposition.h
├── util/
│   └── config.h             # UPDATED — add EngageDelayMs, NNAuthorityLimit
└── autoc.h                  # UPDATED — remove duplicate topology defines

src/
├── nn/
│   └── evaluator.cc         # UPDATED — nn_gather_inputs populates fields by name
├── eval/
│   ├── fitness_decomposition.cc  # UPDATED — computeTargetDir() for 3-cosine representation
│   └── sensor_math.cc       # UPDATED — computeTargetDir() helper, path-tangent-in-body-frame
├── util/
│   └── config.cc            # UPDATED — parse new config keys
└── autoc.cc                 # UPDATED — data.dat format, Change 8 authority limit, Phase 0b dedup refactor

tests/
├── nn_inputs_tests.cc       # NEW — static_assert tests, field access tests
├── nn_evaluator_tests.cc    # UPDATED — input layout assertions by name
├── contract_evaluator_tests.cc  # UPDATED — topology assertions
├── fitness_decomposition_tests.cc  # UPDATED — computeTargetDir tests, singularity tests
├── fitness_computer_tests.cc    # (unchanged — 022 conical surface preserved)
├── engage_reset_tests.cc    # NEW — resetHistory() unit tests
├── engage_delay_tests.cc    # NEW — CRRCSim-side integration (may live under crrcsim/tests/)
└── eval_determinism_tests.cc  # NEW — Phase 0b acceptance test

tools/
├── nn2cpp/
│   └── *                     # UPDATED — codegen emits struct field access
└── minisim.cc                # UPDATED — type-safe NNInputs consumer

# crrcsim submodule (git submodule at crrcsim/)
crrcsim/src/mod_inputdev/inputdev_autoc/
├── inputdev_autoc.cpp       # UPDATED — engage delay window (Change 1b), computeTargetDir()
└── inputdev_autoc.h         # UPDATED — config for delay window

# xiao (embedded target, xiao/ subdir)
xiao/src/
├── msplink.cpp              # UPDATED — populate NNInputs fields by name, computeTargetDir(), resetHistory()
└── generated/               # UPDATED — nn2cpp output regenerated with struct field access

# Python analysis tools
specs/019-improved-crrcsim/
└── sim_response.py          # UPDATED — parse data.dat columns by name, not position

# Durable cross-feature research (`docs/` is for long-lived reference material)
docs/
└── inav-signal-path-audit.md   # may inform 024+ architectural decisions

# One-off feature-scoped research (stays in feature dir, obsolete after refactor lands)
specs/023-ood-and-engage-fixes/
└── train-eval-code-dedup.md    # survey feeds Phase 0b, discarded afterward
```

**Structure Decision**: Multi-component with no reorganization. The existing autoc + crrcsim
submodule + xiao PlatformIO layout already cleanly separates concerns. 023 adds new files
under the existing directory structure (new headers, new tests, feature-scoped research in
the feature dir, durable research in `~/autoc/docs/`) and updates existing files in place.
The type-safe NN refactor touches ~15 files but the directory structure is unchanged. The
INAV signal path audit lives in the global `docs/` directory because its findings are
long-lived — it may inform 024+ architectural escape-hatch decisions beyond 023's scope.
The train/eval code deduplication survey lives inside the feature directory because it
is a one-shot survey that becomes obsolete once the Phase 0b refactor lands.

## Phase 0: Outline & Research

**Status**: In progress (background research agents dispatched 2026-04-08).

### Research tasks dispatched

1. **INAV signal path audit** (deep source read of `~/inav`, MSP extension review, MSP2
   latency measurement from `flight-results/flight-20260407/flight_log_*.txt`, escape-hatch
   architectures) → `~/autoc/docs/inav-signal-path-audit.md` (durable — stays in `docs/`
   because it may inform 024+ architectural decisions beyond 023's scope)
2. **Train/eval code duplication survey** (side-by-side read of `src/autoc.cc`, divergence
   enumeration, refactor proposal) → `specs/023-ood-and-engage-fixes/train-eval-code-dedup.md`
   (one-off — becomes obsolete once Phase 0b refactor lands; stays inside the feature dir)

### Unknowns / NEEDS CLARIFICATION (resolved or pending research)

- **INAV filter inventory** — resolved by dispatched research agent
- **MSP2 round-trip latency bound** — resolved by dispatched research agent (parsing flight logs)
- **Train/eval divergence set** — resolved by dispatched research agent
- **Refactor shape for Phase 0b** — resolved by dispatched research agent (proposed helper signature)
- **Authority-limit trigger criterion (Change 8)** — DEFERRED to implementation time per
  Q4 clarify decision. No research needed; criterion is set empirically after observing
  Phase 3 Milestone A baseline behavior.
- **Variation ramp rate for bigger sigmas, throttle lexicase epsilon** — plan-phase tuning
  decisions, resolvable during implementation. Low-impact.

### Decisions captured outside research.md

The spec's Clarifications section already locks these decisions (no re-research needed):

- Q1: Engage transient fix → pre-fill with current in-autoc geometry on every transition
- Q2: INAV engage delay → simulate in sim pipeline with centered-stick during ~0.75s window
- Q3: Type-safe NN interface → struct-of-floats with named fields + `static_assert` layout
- Q4: Authority-limit trigger → empirical, set during implementation
- Q5: Streak threshold ramp → stays in BACKLOG

### Phase 0 output

research.md will be written after both background agents complete, consolidating their
findings plus the above decisions into a single index document with pointers to the deep
research: `~/autoc/docs/inav-signal-path-audit.md` (durable) and
`specs/023-ood-and-engage-fixes/train-eval-code-dedup.md` (feature-scoped).

## Phase 1: Design & Contracts

**Prerequisites**: research.md complete (Phase 0), both background research agents
returned with findings written to their respective locations (INAV audit in
`~/autoc/docs/`, train/eval dedup survey in the feature directory).

### Data model extraction (`data-model.md`)

Core entities to document:

1. **NNInputs** (struct-of-floats, `include/autoc/nn/nn_inputs.h`)
   - Field list, types, ranges, units
   - `static_assert` invariants
   - Cereal serialization contract (field order = binary format order)
   - Relationship to existing `AircraftState.nnInputs_` storage
   - Relationship to `data.dat` column layout (1 column per field)

2. **NN Topology** (`include/autoc/nn/topology.h`)
   - Layer sizes {33, 32, 16, 3}
   - Weight count (1667) derivation
   - `NN_TOPOLOGY_STRING` constant
   - Validation rules: topology string must match `sizeof(NNInputs) / sizeof(float)` input count

3. **EngageState** (within `AircraftState` or adjacent)
   - Delay window active flag
   - Delay window start tick
   - Last-known pre-engage stick position (for the "centered stick" sim behavior)
   - Interaction with `resetHistory()`

4. **ScenarioSetup** (the shared helper from Phase 0b refactor — exact shape from research agent)
   - Inputs it consumes (config globals, path, wind, entry variations)
   - Outputs it populates (`EvalData` or equivalent)
   - Invariants: training and eval paths produce bitwise-identical setups given identical inputs

5. **data.dat column schema**
   - One column per `NNInputs` field (33 bearing+dist+state + 3 outputs + meta)
   - Column header names derived from struct field names
   - Parser contract: read by name, fail loud on missing columns

### Contracts (`contracts/`)

1. **`contracts/nn_interface.md`** — NNInputs struct as the compile-time API between sensor
   gathering (xiao msplink.cpp, CRRCSim inputdev_autoc.cpp, minisim.cc) and NN forward pass
   (evaluator.cc). Field list, ownership, invariants, serialization contract.

2. **`contracts/engage_delay.md`** — CRRCSim engage delay window behavior: timeline, input
   update semantics, output suppression semantics, interaction with Change 1 history reset,
   config knob (`EngageDelayMs`), test coverage requirements.

3. **`contracts/history_reset.md`** — `AircraftState::resetHistory()` semantics: when it
   runs, what it populates, how it derives values from current geometry, how xiao and
   CRRCSim share the implementation.

### Quickstart recipe (`quickstart.md`)

The minisim smoke test for Phase 2 of the Implementation Order. Step-by-step runnable recipe:

1. Build: `bash scripts/rebuild.sh`
2. Smoke test: `./build/minisim --config autoc-minisim-smoke.ini` (with a smoke-test-specific
   config pinning 1 path, zero variations)
3. Verify: fitness decreases over 50 gens, no NaN, no hang, `data.dat` has all 33+3+meta columns
4. Verify eval determinism: `./build/autoc --eval nn_weights.dat` reproduces the training-time
   fitness within FP rounding
5. Promote to CRRCSim: same config, swap minisim backend for CRRCSim

### Agent context update

Run `.specify/scripts/bash/update-agent-context.sh claude` to update CLAUDE.md with the
023-specific technology additions (type-safe NNInputs pattern, 3-cosine bearing representation,
engage delay modeling). The script preserves manual additions between markers per its
documented behavior.

### Phase 1 outputs

- `data-model.md` — entity definitions and contracts
- `contracts/nn_interface.md`
- `contracts/engage_delay.md`
- `contracts/history_reset.md`
- `quickstart.md` — minisim smoke test recipe
- Updated `CLAUDE.md` with 023 technology mentions

### Post-design Constitution Re-Check

Will re-evaluate after Phase 1 artifacts are written. No anticipated violations — the
design faithfully implements the spec's Changes 1–8 + prerequisites, all within the
established C++17/Eigen/cereal/GoogleTest stack and the unified CMake structure.

## Complexity Tracking

*No constitutional violations require justification.* The 023 plan is well within established
patterns:

- Multi-component changes use the same autoc/crrcsim/xiao coordination pattern as 022
- Type-safe NN interface is a refactor of existing code, not a new subsystem
- Research artifacts are documentation, not code
- CRRCSim engage delay is a single-file addition with config knob
- Train/eval dedup is a pure refactor — no new behavior

The only thing that feels "bigger than it looks" is the type-safe NN refactor blast radius
(~15 files), but that blast radius is deliberate: it's the entire point of the refactor.
Paying the migration cost once buys permanent protection against the silent serialization
corruption bug class that bit 021.
