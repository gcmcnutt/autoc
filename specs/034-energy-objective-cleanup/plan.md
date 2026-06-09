# Implementation Plan: M1/M2 Cleanup + Craft Variations → Flight Test

**Branch**: `034-energy-objective-cleanup` (name historical — energy split to 035) | **Date**: 2026-05-29 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/034-energy-objective-cleanup/spec.md`

## Summary

034 resets all training and uses that reset to (1) tear out the already-inert minisim worker, (2) clean-cut the dead (already-bypassed) smoothness apparatus across 12 transport boundaries, (3) clear cross-cutting tech-debt fold-ins, and (4) add deterministic per-scenario craft variations (CG/drag/servo/power/trim) to the CRRCSim-simulated chase craft — culminating in an operator-run M1→M2 bake + flight test. Energy-as-lexicase moved to 035.

Research (Phase 0) confirmed the cleanups are lower-risk than feared (smoothness already bypassed → removal is behavior-neutral; minisim already routes to crrcsim) but surfaced **two operator decisions** that shape task scope:

- **D1 (craft-param scope) — DECIDED**: craft set = **CG, drag, trim, thrust-scale, pitch-effectiveness, roll-effectiveness** (pitch/roll independent → slight asymmetry). Servo *lag* (control-rate time-constant) **deferred** to a follow-on (new dynamical element, pt3-filter training-stunt risk). Control *effectiveness* substitutes — it's settable control-authority aero coefficients (`Cm_de`/`CL_de` pitch, aileron→roll coeff), same easy tier as CG/drag/trim. All 6 apply at the existing per-scenario FDM reset hook.
- **D2 (already-done fold-ins) — DECIDED**: FR-013 (crash-hull PRNG) and FR-014 (mod_inputdev→autoc_common) **verified already satisfied** in-tree → **dropped**. Crash-hull already seeds from `deriveClassSubSeeds(scenarioSeed).rabbit` (`crrcsim_tracker_helper.cpp:54-56`); mod_inputdev already links `autoc_common`. Net US3 = **5 live items** (config X-macro, seed-width, **per-(path,wind) variation-table FR-012**, S3 prefix, EvalResults trim). (Separate out-of-scope non-determinism: wall-clock `multiloop`/`simTimeMsec`, per crrcsim/CLAUDE.md — NOT 034.)

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim), Python 3.11 (analysis scripts)
**Primary Dependencies**: Eigen (vec3/quat), cereal (NN + EvalResults + dmp serialization), inih (ini parsing), GoogleTest (unit/contract), CRRCSim LaRCSim FDM
**Storage**: file-based — `data.dat` (per-tick trace), `data.stc` (per-gen aggregate), S3 `.dmp` (cereal `EvalResults`); greenfield schema growth, **NO version revision for transport or .dmp during the M2 initiative** (operator directive; old dmps orphaned by training reset, not migrated)
**Testing**: GoogleTest (`scripts/rebuild.sh` builds + runs); xiao `pio run -e xiaoblesense_arduinocore_mbed`
**Target Platform**: Linux (training host); xiao SAMD/nRF (deploy, not touched by 034 code)
**Project Type**: Single multi-component repo (autoc evolution + crrcsim FDM submodule + xiao firmware)
**Performance Goals**: No regression to per-gen eval throughput; EvalResults trim (US3 #6) should reduce autoc-side working set (memory, not latency)
**Constraints**: Bit-exact M1→M1 replay determinism gate (validated in 033) MUST survive seed-width + craft-seed-cascade changes; craft variation must be a true no-op at σ=0
**Scale/Scope**: pop=8000 / wind=36 baseline; ~123 smoothness refs to remove; ~82 config fields for X-macro; 5 craft params (3-5 depending on D1)

**Unknowns resolved in research.md** — remaining implement-time verifications: (a) `derived_features.h` retains 032 span features (not smoothness-only); (b) `PathgenStepper` orphaned by minisim removal?; (c) roll-authority coefficient name in `fdm_larcsim`. *(No dmp/transport version revision — M2-era policy; not a verification item.)*

## Constitution Check

*GATE: evaluated against constitution v1.3.0.*

| Principle | Status | Notes |
|---|---|---|
| I. Testing-First | **PASS** | Smoothness tests deleted (not disabled); craft-variation determinism + no-op tests added before impl; before/after fixed-seed eval for removal behavior-neutrality |
| II. Build Stability | **PASS** | `scripts/rebuild.sh` (autoc+crrcsim) + xiao build green at each milestone close |
| III. No Compatibility Shims | **PASS** | Clean smoothness cut; `minisimProgram→workerProgram` rename with NO alias; greenfield craft fields (no default-zero back-compat); orphaned `PathgenStepper` deleted if confirmed unused |
| IV. Unified Build | **PASS** | FR-014 (mod_inputdev→autoc_common) already satisfies this; no duplicate dep decls introduced |
| V. Versioned Persistence | **PASS (M2-era policy)** | **No schema version revision for transport OR .dmp during the M2 initiative** (operator directive 2026-05-29; `feedback_no_cereal_versioning`). Greenfield growth: old dmps are orphaned by the training reset, not migrated — no version-field bump, no migration shim, no bespoke fail-loud contract test. Rely on existing cereal behavior; if a stale dmp is ever loaded, the struct-layout mismatch surfaces through normal cereal error paths. No added versioning ceremony. |
| VI. Type-Domain Discipline | **PASS** | Craft sampling/delta math = `gp_scalar`; milestone-close audit grep on touched `src/eval/`,`src/nn/` |
| VII. No Silent Fallback Defaults | **PASS** | Craft-param members from `ScenarioMetadata`/`WorkerInit`/ini carry no in-class defaults; constructor initializer-list is sole assignment site |

No violations requiring Complexity Tracking. The Principle V no-version-bump + fail-loud reliance is the single item to verify per-schema-change at implement time (recorded, not a gate failure).

## Project Structure

### Documentation (this feature)

```text
specs/034-energy-objective-cleanup/
├── plan.md              # This file
├── research.md          # Phase 0 — 5-sweep consolidation (DONE)
├── data-model.md        # Phase 1 — schema deltas (this run)
├── quickstart.md        # Phase 1 — operator bake/flight runbook (this run)
├── contracts/           # Phase 1 — craft-variation + schema contracts (this run)
├── spec.md              # Feature spec (clarified)
└── tasks.md             # Phase 2 — /speckit.tasks (NOT created here)
```

### Source Code (repository root)

Existing multi-component layout; 034 touches these real paths:

```text
src/
├── autoc.cc                      # data.dat writers, config print, run-id prefix, buildEvalData/WorkerInit, seed cascade
├── eval/
│   ├── fitness_computer.cc        # applyStreak (drop smoothness param)
│   ├── fitness_decomposition.cc   # commented smoothness block (remove)
│   ├── pathgen_stepper.cc         # smoothness state/compute (remove); verify orphaning
│   ├── tracker_stepper.cc         # smoothness state/compute (remove)
│   └── (new) craft_variation.*    # craft-delta sampling (gp_scalar, deriveClassSubSeeds)
├── util/
│   └── config.cc                  # parse (X-macro target); seed parse; workerProgram rename
include/autoc/
├── eval/{derived_features,fitness_computer,pathgen_stepper,tracker_stepper,aircraft_state}.h
├── rpc/protocol.h                 # WorkerInit + ScenarioMetadata (drop smoothness; add craft fields)
└── util/{config.h, scenario_prng.h, rng.h}
crrcsim/src/
├── mod_inputdev/inputdev_autoc/   # per-scenario reset hook (craft apply); smoothness mirror (remove)
├── mod_fdm/fdm_larcsim/           # CD_prof/Cm_0/CG_arm set at initAirplaneState; engine() thrust-scale
├── crrc_main.cpp / SimStateHandler.cpp  # initialize_flight_model reset path
└── global.h                       # Global:: craft-delta carriers (mirror entry-offset pattern)
tools/
└── minisim.cc                     # DELETE
tests/                             # delete smoothness tests; craft-variation tests; update stepper-init test
specs/029-no-future-arch/audit_shift_register.py   # refactor hard-offset → header lookup (data.dat fragility)
```

**Structure Decision**: No new components; in-place edits across autoc + crrcsim. One new autoc source pair (`craft_variation.{h,cc}`) for the sampling logic, mirroring the existing variation-generator pattern.

## Implementation Phasing (for /speckit.tasks)

Sequenced so each phase is independently buildable + testable, US1 first as the single-path force-multiplier:

1. **US1 — Minisim teardown** (first): delete `minisim.cc` + CMake lines; rename `minisimProgram→workerProgram`; verify/delete orphaned `PathgenStepper`; comment/test/doc updates. Gate: build green, crrcsim eval bit-identical.
2. **US2 — Smoothness removal**: strip all 12 categories; refactor `audit_shift_register.py` to header lookup FIRST; delete dead `sim_response.py`; delete smoothness tests; update format-contract doc. Gate: zero live refs, fixed-seed eval byte-identical, fail-loud on old dmp.
3. **US3 — Fold-ins** (5 live, after D2 verify): config X-macro auto-print; seed-width right-size (preserve determinism gate); per-(path,wind) variation-table (FR-012 — resolve `gScenarioVariations` indexing or prove no-gap); S3 run-id `tracker-` prefix; EvalResults non-elite trim. (FR-013/FR-014 dropped — already done.)
4. **US4 — Craft variations** (6 params per D1): `ScenarioMetadata` craft fields + craft seed; `craft_variation.{h,cc}` sampling (`gp_scalar`, fractional-σ Gaussian, non-ramping, no-op at σ=0); `Global::` carriers + per-scenario apply at init (`CG_arm`, `CD_prof`, `Cm_0`, `Cm_de`/`CL_de` pitch-eff, aileron→roll-coeff roll-eff) + `engine()` thrust-scale; determinism + no-op tests. Gate: same-seed bit-identical airframe, σ=0 no-op, replay gate intact. *(Implement note: confirm the roll-authority coefficient name in `fdm_larcsim`.)*
5. **US5 — Bake + flight (operator-run)**: acceptance milestones, tracked separately from implementation tasks (per `feedback_operator_runs_regression_gate`).

## Complexity Tracking

No constitution violations requiring justification. (D1/D2 are scope decisions, not complexity exceptions.)
