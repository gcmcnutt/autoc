# Implementation Plan: 033 — M1 smoothness + replay-friendly variation PRNGs

**Branch**: `033-m1-smooth-plus-variations` | **Date**: 2026-05-20 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/033-m1-smooth-plus-variations/spec.md`

## Summary

033 phase 1 ships two co-dependent worker-side changes plus a dmp schema extension:

1. **Master-seed → per-scenario sub-PRNG architecture** (§2.A) — decouples NN-evolution PRNG from per-scenario variation PRNGs; sub-PRNGs organized by variation CLASS (wind, rabbit, entry, craft-future, camera-future); each class re-seeds at scenario start from a scenario-derived seed. Bit-deterministic per `(NN, scenarioSeed[K])`.
2. **Multiplicative smoothness penalty on per-step fitness** (§2.B) — worker-side `applyStreak()` integration point. `smoothness_factor ∈ [0.5, 1.0]` computed from Pythagorean `sqrt(Δpt² + Δrl² + Δth²)`; floor 0.5 (YOLO start). Penalty applied before streak multiplier so smooth controllers compound more.
3. **`scenarioSeed[K]` recorded in `EvalResults` dmp** alongside the existing per-scenario fields. Scenario seed alone is sufficient for full replay; per-class sub-seeds derive deterministically from it.

Phase 1 bake target: M1 only. Success gates (per Clarifications Q1): continued learning, streak% ≥ 2/3 of 029 pastonly3 baseline, best-fitness in same range, materially-better per-axis aggressiveness. Mezzanine real-flight test is the qualifier to phase 2 (M2 inheritance + kamikaze, scope §2.C/§2.D).

**Significant intentional break**: post-033 M1 fitness numbers will NOT be bitwise-equal to pre-033 because the PRNG-stream decoupling re-orders draws. We're not preserving back-compat; the 033 closeout SETS a new M1 baseline. Old dmps fail-loud on read (cereal length mismatch from added `scenarioSeed[K]` field) — per project no-cereal-versioning policy.

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim), Python 3.11 (analysis scripts)
**Primary Dependencies**: Eigen (vec3/quat math), cereal (NN serialization, EvalResults wire-protocol + dmp), inih (ini parsing), GoogleTest (unit + contract tests), CRRCSim LaRCSim FDM
**Storage**: file-based — `data.dat` (per-tick training trace), `data.stc` (per-gen aggregate), S3 `.dmp` (cereal-serialized `EvalResults` — schema grows with new `scenarioSeed[K]` field; NO cereal version bump per [feedback_no_cereal_versioning](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md))
**Testing**: GoogleTest (unit + contract); end-to-end via small bake + `rebuild-perf.sh` (NEW M1 baseline post-033, not the pre-033 one)
**Target Platform**: Linux desktop (autoc engine + crrcsim FDM); no xiao changes
**Project Type**: C++17 application (training engine + flight-dynamics simulator)
**Performance Goals**: PRNG architecture overhead = N draws per scenario init (negligible — one-time per scenario); smoothness factor = per-tick sqrt + multiply + bounded clamp (negligible vs NN forward-pass cost)
**Constraints**:
- Bit-deterministic per `(NN, scenarioSeed[K])` — explicit goal; eval-mode regression test enforces
- Determinism preserved across worker counts (existing concern per [project_v15_determinism_candidates](../../.claude/projects/-home-gmcnutt-autoc/memory/project_v15_determinism_candidates.md); 033 doesn't address the intra-run worker-completion-order issue, only the cross-run / cross-mode PRNG-state-drift issue)
- M1 pathgen and M2 tracker modes BOTH consume the same scenario-sub-PRNG architecture; identical wind/entry/rabbit draws across modes (only what's used differs)
- Type-domain discipline (Constitution VI): smoothness factor + per-tick Δ math use `gp_scalar`; raw NN output access uses raw-ok per existing convention
**Scale/Scope**: ~5 source files touched in autoc (`include/autoc/util/rng.h`, NEW `include/autoc/util/scenario_prng.h`, `src/eval/fitness_computer.cc`, `src/util/joint_prng.cc` or equivalent variation-seeding site, `include/autoc/rpc/protocol.h`), 3 in crrcsim (`inputdev_autoc.cpp` + tracker/pathgen helpers — just pass scenarioSeed[K] to SimStateHandler::reset which already accepts it), ini schema in all 3 ini files, ~3 new test files; estimated <1k LOC delta

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Justification |
|---|---|---|
| **I. Testing-First** | PASS | TDD per phase 1 milestone: pure-math smoothness tests (extend `derived_features_tests.cc`), PRNG architecture tests (round-trip determinism, append-only contract), worker-side fitness-decomposition test (smoothness × streak ordering), end-to-end reproducibility test (same `(NN, masterSeed)` → bit-identical per-scenario `#GenDiag` output) |
| **II. Build Stability** | PASS WITH NOTE | M1 pathgen fitness numbers WILL change vs pre-033 because PRNG-stream decoupling re-orders draws. **This is intentional and accepted** — 033's whole point is to change M1 behavior. Closeout SETS new M1 baseline. Bitwise gate post-033 is "M1 with `SmoothnessPenaltyFloor=1.0` (no-op penalty) under new PRNG architecture == 033 closeout M1 baseline" — verifies the PRNG refactor + the no-penalty path internal consistency, NOT pre-033 equivalence. Documented in research.md R1 + tasks.md US1 |
| **III. No Compatibility Shims** | PASS | Old dmps fail-loud on cereal length mismatch (added `scenarioSeed[K]` vector). No parallel old-shape reader. PRNG architecture is greenfield — no legacy single-stream fallback flag |
| **IV. Unified Build** | PASS | No new dependencies. Reuses cereal / inih / GoogleTest |
| **V. Versioned Persistence Artifacts** | PASS WITH NOTE | Same exemption as 032: no cereal version bump for dmp schema growth (`scenarioSeed[K]` add). Fail-loud cereal-archive-length-mismatch is the safety net. Both M1 and M2 dmps grow if scenarioSeed is persisted from a shared write site — research.md R4 confirms placement |
| **VI. Type-Domain Discipline** | PASS | Smoothness factor + per-tick Δ aggregation use `gp_scalar`; only crosses into raw `float` at the per-tick NN-output read (existing raw-ok-annotated site). New `ScenarioPRNG` class internals use standard `uint32_t` / `uint64_t` for PRNG state (not eval-pipeline scalars; PRNG bits are integer-domain). Closing per-milestone grep at /speckit.implement time |

No violations to record in Complexity Tracking.

## Project Structure

### Documentation (this feature)

```text
specs/033-m1-smooth-plus-variations/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output — PRNG entity + scenarioSeed + smoothness factor
├── quickstart.md        # Phase 1 output — operator bake protocol
├── contracts/           # Phase 1 output
│   ├── scenario_prng_chain.md     # master → scenario → sub-PRNG init contract
│   ├── smoothness_factor.md       # per-tick math + integration with applyStreak
│   ├── dmp_schema_extension.md    # scenarioSeed[K] add point in EvalResults
│   └── ini_schema.md              # SmoothnessPenaltyFloor + SmoothnessMotionMode
└── tasks.md             # Phase 2 output (/speckit.tasks)
```

### Source Code (repository root)

```text
include/autoc/util/
├── rng.h                          # MODIFIED — global helpers stay (back-compat for evolution-side draws) BUT seeded from explicit autoc-NN-PRNG seed at startup (not from the same value that the scenario seeds derive from)
└── scenario_prng.h                # NEW — ScenarioPRNG + per-class sub-PRNG types; init chain helpers

include/autoc/eval/
├── derived_features.h             # MODIFIED — add compute_smoothness_factor() pure helper alongside compute_pair_span / compute_tilt
└── fitness_computer.h             # MODIFIED — applyStreak signature gains smoothness_factor param (or new applyStreakSmooth variant — research.md R2 decides)

include/autoc/rpc/
├── scenario_metadata.h            # MODIFIED — scenarioSeed field added (uint64_t), legacy windSeed field REMOVED in same PR per spec Clarifications 2026-05-21 (Constitution III: no coexistence)
└── protocol.h                     # MODIFIED — EvalResults adds scenarioSeedList vector; serialize() walks it (no version bump)

include/autoc/util/
└── config.h                       # MODIFIED — AutocConfig gains smoothnessPenaltyFloor (double, default 0.5), smoothnessMotionMode (string, default "pythagorean")

src/util/
├── config.cc                      # MODIFIED — parse [Smoothness] section knobs; loud-fail on out-of-range
├── joint_prng.cc (or similar)     # MODIFIED — refactor to use new ScenarioPRNG chain instead of joint single-stream draws
└── scenario_prng.cc               # NEW — implementation for scenario_prng.h

src/eval/
├── fitness_computer.cc            # MODIFIED — applyStreak() honors smoothness_factor input
├── fitness_decomposition.cc       # MODIFIED — call site computes smoothness_factor from per-tick NN-output Δ + passes through
├── pathgen_stepper.cc             # MODIFIED — per-tick loop: track previous NN output, compute Δ, pass to fitness path
└── tracker_stepper.cc             # MODIFIED — same per-tick treatment as pathgen

src/autoc.cc                       # MODIFIED — init chain: masterPRNG → autoc-side NN stream seed + scenarioSeed[K] table; record scenarioSeed in EvalResults

crrcsim/src/mod_inputdev/inputdev_autoc/
├── inputdev_autoc.cpp             # MODIFIED — when scenario starts, pass scenarioSeed[K] through to the proper wind PRNG via SimStateHandler::reset(windSubSeed) — the wind sub-PRNG seed derived from scenarioSeed[K] per the chain
├── crrcsim_tracker_helper.cpp     # MODIFIED — same scenario-init treatment as pathgen path
└── crrcsim_pathgen_helper.cpp     # MODIFIED — same

tests/
├── scenario_prng_tests.cc         # NEW — init-chain determinism, append-only contract, cross-mode equivalence
├── derived_features_tests.cc      # EXTENDED — pure-math smoothness factor: floor=1.0 ⇒ factor=1.0 always (no-op); floor=0.5 + zero motion ⇒ factor=1.0; floor=0.5 + max motion ⇒ factor=0.5; Pythagorean vs sum vs max correctness
├── fitness_computer_tests.cc      # EXTENDED — applyStreak honors smoothness_factor before multiplier
├── tracker_dmp_roundtrip_tests.cc # EXTENDED — scenarioSeed[K] round-trips through EvalResults serialize
└── contract_config_tests.cc       # EXTENDED — [Smoothness] section parse + range-check loud-fail

autoc.ini                          # MODIFIED — new [Smoothness] section
autoc-tracker.ini                  # MODIFIED — same
autoc-tracker-minisim.ini          # MODIFIED — same

docs/
└── variation-prng.md              # NEW — operator-facing doc explaining master-seed contract + class structure + reproducibility recipe
```

**Structure Decision**: Same mono-tree layout used by 032. Smoothness math lives alongside the existing `compute_pair_span` / `compute_tilt` helpers in `include/autoc/eval/derived_features.h` (single home for pure-math fitness/derivation primitives). PRNG chain gets its own header `include/autoc/util/scenario_prng.h` because it's an independent subsystem (PRNG mechanics, not eval scalars). Both autoc-minisim and crrcsim FDM paths consume the same headers — wire-equivalent.

## Complexity Tracking

> No Constitution violations require justification. The M1-bitwise-break (Principle II note) is documented as INTENTIONAL — 033's purpose IS to change M1 fitness numbers via reward-shape + PRNG-stream changes. New M1 baseline established at closeout.

### Post-design re-check (2026-05-20)

After generating research.md, data-model.md, contracts/ (4 files), and quickstart.md, re-evaluating Constitution gates:

| Principle | Re-check | Notes |
|---|---|---|
| **I. Testing-First** | PASS | Each contract enumerates its validation tests:
- `scenario_prng_chain.md` → `tests/scenario_prng_tests.cc` (NEW) + `tests/wind_replay_tests.cc` (NEW)
- `smoothness_factor.md` → `tests/derived_features_tests.cc` (EXTENDED) + `tests/fitness_computer_tests.cc` (EXTENDED) + `tests/stepper_smoothness_tests.cc` (NEW)
- `dmp_schema_extension.md` → `tests/tracker_dmp_roundtrip_tests.cc` (EXTENDED) + `tests/eval_mode_replay_tests.cc` (NEW or EXTENDED)
- `ini_schema.md` → `tests/contract_config_tests.cc` (EXTENDED)
TDD ordering preserved (pure-math + parsing tests precede integration tests precede end-to-end bake regression). |
| **II. Build Stability** | PASS WITH NOTE (unchanged) | Pre-033 M1 bitwise gate intentionally broken; new M1 baseline set at closeout. Quickstart documents the eval-mode bit-identical regression check as the within-033 substitute gate. |
| **III. No Compatibility Shims** | PASS | All four contracts encode the fail-loud-on-old-shape behavior (cereal length mismatch on old dmps; no `SmoothnessPenaltyFloor=legacy-mode` toggle; no parallel old single-PRNG fallback path). |
| **IV. Unified Build** | PASS (unchanged) | No new deps introduced by the design pass. |
| **V. Versioned Persistence Artifacts** | PASS WITH NOTE (unchanged) | `dmp_schema_extension.md` documents the NO-version-bump policy explicitly and the fail-loud safety net. Same exemption as 032. |
| **VI. Type-Domain Discipline** | PASS | `smoothness_factor.md` has a dedicated type-domain-compliance section confirming `gp_scalar` discipline at the pure-math + applyStreak + per-tick boundary. `ini_schema.md` documents raw-ok at the ini boundary, cast at consumer. `scenario_prng_chain.md` uses `uint32_t`/`uint64_t` for PRNG state (integer domain, not eval scalars — appropriate). |

No new Constitution violations introduced by Phase 1 design.
