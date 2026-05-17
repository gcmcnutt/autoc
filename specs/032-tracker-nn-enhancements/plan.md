# Implementation Plan: 032 Tracker NN Enhancements — Derived Pose Features (Phase 1)

**Branch**: `032-tracker-nn-enhancements` | **Date**: 2026-05-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/032-tracker-nn-enhancements/spec.md`

## Summary

Phase 1 of 032 extends the tracker-mode NN input vector from **45 → 54** with derived perceptual features (Feature B) and locks in port/starboard identity-stable beacon ordering as a contract (Feature A). Bake at the empirically-best 16r recurrent topology (T-102 closed 32r as worse), then evaluate plateau-avgInRamp via the 030 reference protocol (≥322 gens, last-50 average). Phase 1 ships **minisim + crrcsim only**; xiao tracker-mode port is NOT in scope. Conventions docs (`docs/COORDINATE_CONVENTIONS.md`, `docs/sensor-pipeline.md`) get updated in phase 1 to keep the future xiao port wire-equivalent without archaeology. Outcome decision rule (Q7): ≥0.15 = success, 0.10–0.15 = run A-only + B-only attribution bakes, <0.10 = clean miss → phase 2 / M3.

Per the operator's 2026-05-16 reinforcement: M2 schema version numbers are NOT revised during 032 experiments — backward compat in the M2 realm is dropped wholesale (greenfield), while M1 (pathgen mode) MUST remain functional.

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim), Python 3.11 (analysis/inspection scripts)
**Primary Dependencies**: Eigen (vec3/quat math), cereal (NN serialization, EvalData wire-protocol), inih (autoc-tracker.ini parsing), GoogleTest (unit + contract tests), CRRCSim LaRCSim FDM (crrcsim bake path)
**Storage**: file-based — `data.dat` (per-tick training trace, must record all 54 inputs + 3 outputs), `data.stc` (genome stats), S3 `.dmp` (cereal-serialized `EvalResults` — schema grows but version field is NOT bumped per M2 policy)
**Testing**: GoogleTest (unit + contract); end-to-end via `scripts/rebuild-perf.sh` regression gate (FP-deterministic) and bake-comparison runs
**Target Platform**: Linux desktop (autoc evolution engine + minisim or crrcsim FDM); xiao firmware is OUT of phase 1 scope
**Project Type**: C++17 application (training engine + flight-dynamics simulator + future embedded controller)
**Performance Goals**: Bake throughput parity with 030 postdiag2 (pop=5000, ~322+ gens to plateau-read); per-tick NN forward-pass overhead negligible (9 added inputs against 32→16r→3 hidden topology = ~20% weight count increase)
**Constraints**:
- Determinism preserved (rebuild-perf.sh bitwise-equal gate must still pass after refactor; per [project_variation_design_principles](../../.claude/projects/-home-gmcnutt-autoc/memory/project_variation_design_principles.md))
- M1 pathgen mode UNCHANGED — `PathgenInput` enum and `NNInputs` struct untouched, M1 dmps remain readable, M1 bakes remain bitwise-equal to pre-032 baseline
- M2 schema bumps allowed without cereal version field changes (greenfield in the M2 realm)
- `data.dat` and dmp must BOTH carry all 54 NN inputs + 3 outputs (dmp-honesty invariant; 032's schema bump is the audit boundary per [feedback_honest_dmp_recording](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_honest_dmp_recording.md))
- Type-domain discipline (Constitution VI): new derived-feature scratch math uses `gp_scalar` (input slots stay `float[N]` raw-ok by NN-byte-format whitelist)
**Scale/Scope**: ~3 source files touched in autoc (`include/autoc/nn/nn_inputs.h`, `src/nn/evaluator.cc`, `src/eval/tracker_stepper.cc`), ~1 in crrcsim (`crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp`), ini schema in `autoc-tracker.ini`, 2 docs updates, ~3 new test files; estimated <1.5k LOC delta

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Justification |
|---|---|---|
| **I. Testing-First** | PASS | Phase 1 includes contract tests for the extended `TrackerInput` enum (round-trip name↔index, size assertions), unit tests for the derived-feature math (span, span-rate, tilt sin/cos) including the CEP-gated neutral-substitution path, and end-to-end regression check via `rebuild-perf.sh` bitwise-equal gate. TDD: tests-first per milestone. |
| **II. Build Stability** | PASS | autoc + crrcsim build is the gate; xiao firmware is NOT touched in phase 1 so its build is trivially unaffected. Pre-merge rebuild-perf.sh must pass with M1 fitness unchanged. |
| **III. No Compatibility Shims** | PASS | Greenfield M2 schema growth. No `TrackerInputsV1` shim, no parallel old-shape readers, no version-bump migration code. Old M2 dmps simply become unreadable post-032 (acknowledged); M1 dmps untouched and remain readable via the `PathgenInputs` path. |
| **IV. Unified Build** | PASS | No new dependencies. Reuses cereal / inih / GoogleTest from top-level FetchContent. |
| **V. Versioned Persistence Artifacts** | PASS WITH NOTE | Phase 1 deliberately does NOT bump the dmp version field per the M2 experimental policy. Constitution V's read-side fail-loud is therefore the safety net: a post-032 reader hitting a pre-032 M2 dmp will read the new-shape struct off old bytes — cereal's binary-archive length mismatch fails loudly (no silent truncation). M1 dmps continue to deserialize cleanly through the `PathgenInputs` path. Documented in research.md R5. |
| **VI. Type-Domain Discipline** | PASS | NN input slots stay `float[N]` (NN-byte-format whitelist, `// raw-ok:` annotated). Derived-feature scratch arithmetic in `gather_tracker_inputs` uses `gp_scalar` / `gp_vec3` for span / tilt computation before the final cast into NN-byte-format slots. Closing per-milestone grep audit at /speckit.implement time per Constitution VI cadence. |

No violations to record in Complexity Tracking.

**Post-design re-check (after Phase 1 artifacts generated)**: PASS. The design surfaces (data-model.md, contracts/, quickstart.md) introduce no new violations. Notable:
- Constitution VI: `gather_tracker_inputs` extension uses `gp_scalar` for trig intermediates with explicit `raw-ok`-annotated casts at NN-byte-format slot writes (matches the 030 `dist_raw` pattern in `evaluator.cc:475-479`). See [contracts/gather_tracker_inputs_v54.md](./contracts/gather_tracker_inputs_v54.md) §3.
- Constitution V: dmp schema grows without version bump per the M2 policy; fail-loud safety net (cereal archive length mismatch) is the read-side defense. See [research.md R5](./research.md#r5--dmp-schema-growth-without-version-bump-m2-policy-reinforcement).
- Constitution III: Feature A is a sim no-op per [research.md R1](./research.md#r1--feature-a-in-sim-is-already-satisfied-the-only-change-is-the-contract); no shim, no parallel struct, no version-gated reader. Feature B is greenfield with no migration path.

## Project Structure

### Documentation (this feature)

```text
specs/032-tracker-nn-enhancements/
├── plan.md              # This file (/speckit.plan command output)
├── research.md          # Phase 0 output (/speckit.plan command)
├── data-model.md        # Phase 1 output (/speckit.plan command)
├── quickstart.md        # Phase 1 output (/speckit.plan command)
├── contracts/           # Phase 1 output (/speckit.plan command)
│   ├── nn_sensor_interface_v54.md
│   ├── gather_tracker_inputs_v54.md
│   ├── ini_schema.md
│   └── identity_invariant.md
└── tasks.md             # Phase 2 output (/speckit.tasks command - NOT created by /speckit.plan)
```

### Source Code (repository root)

```text
include/autoc/nn/
├── nn_inputs.h                   # MODIFIED — TrackerInput enum + kTrackerInputMeta + TrackerInputs struct grow 45 → 54
└── evaluator.h                   # MODIFIED — gather_tracker_inputs signature stable (out-param), header doc update only

src/nn/
└── evaluator.cc                  # MODIFIED — gather_tracker_inputs computes span/span-rate/tilt, applies CEP-gating, populates new slots

src/eval/
└── tracker_stepper.cc            # MODIFIED — projectAndShiftHistory extended to track span history (span[6] mirrors the NDC history-shift convention)

include/autoc/eval/
└── tracker_stepper.h             # MODIFIED — TrackerHistoryWindow gains span[6] (or new field) per Phase 1 design

crrcsim/src/mod_inputdev/inputdev_autoc/
└── crrcsim_tracker_helper.cpp    # MODIFIED — same history-shift extension as autoc minisim path (wire-equivalent)

tests/
├── nn_sensor_interface_tests.cc      # EXTENDED — round-trip + count==54 for tracker mode
├── gather_tracker_inputs_tests.cc    # EXTENDED — covers new derived features + CEP-gating substitution
└── derived_features_tests.cc          # NEW — unit tests for span / span-rate / tilt sin-cos math

autoc-tracker.ini                  # MODIFIED — new [DerivedFeatures] section: CEP threshold knob

docs/
├── COORDINATE_CONVENTIONS.md      # MODIFIED — port/starboard identity-stable ordering invariant; tilt convention
└── sensor-pipeline.md             # MODIFIED — CEP-gating rule + neutral-substitution values; xiao-port-prep note
```

**Structure Decision**: This feature lives in the existing repo layout (autoc/crrcsim mono-tree with shared `include/autoc/` headers). No new top-level directories. The minisim path (autoc-internal evaluator) and the crrcsim path (crrcsim_tracker_helper) MUST stay wire-equivalent for the derived features — both consume the same `gather_tracker_inputs` helper after the refactor, with the helper extended in `src/nn/evaluator.cc`. This is a tighter integration than the current 030 split (where each path duplicates the history-shift loop), and is itself a contract worth recording.

## Complexity Tracking

> No Constitution violations require justification. Section intentionally empty.
