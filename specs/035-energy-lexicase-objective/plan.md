# Implementation Plan: Energy as a Lexicase Secondary Objective

**Branch**: `035-energy-lexicase-objective` | **Date**: 2026-06-04 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/035-energy-lexicase-objective/spec.md`

## Summary

035 has two halves, sequenced behind a **hard verification gate**:

1. **Pre-work foundation (Phase A)** — make the training factory cheaper and the 035
   measurement honest *before* touching fitness: retire `data.dat` (S3 dmp becomes the sole
   per-run trace) and add zstd compression in the same cereal pass; build a `dmp-dump` CLI so
   Python tooling reads dmps; normalize the S3 run-selector and collapse run-id naming to one
   uniform scheme; move to per-mode buckets (`autoc-m1` / `autoc-m2` / `autoc-eval`) with
   tag-driven 30-day lifecycle; repoint the dangling eval-tracker source; verify the
   already-fixed eval bugs. **Gate:** basic M1 train works, basic eval is bit-equivalent
   (per-scenario-score replay), basic M2 works, and `dmp-dump` reads the dmps — all on the new
   buckets, zstd-compressed, `retain=expire`-tagged. Energy work does not start until this is green.

2. **Energy investigation (Phase B)** — re-enable `energy_score` as a lexicase axis (not a
   scalar penalty), replace the linear throttle proxy with a **convex throttle-command integral**,
   apply it **from gen 0** (no ramp), make lexicase epsilon **MAD-relative** (constant-ε behind
   an ini switch), re-enable the 4 Selection027 tests, and bake on **both M1 and M2** (same
   objective: on-point tracking + minimum energy). Classify each outcome
   (energy-works / tracking-collapses / energy-unmoved) against its mode's tracking-only
   baseline, budgeting 2–3 bakes per mode for the basin lottery, and answer the
   total-energy (PE/KE) go/no-go.

Hull-crash-cost and stability are explicitly out (deferred to a future feature / off for v1 —
see spec Clarifications).

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim), Python 3.11 (analysis/plot scripts)
**Primary Dependencies**: Eigen (vec3/quat), cereal (NN + EvalResults + dmp serialization), inih
(ini), GoogleTest, AWS SDK C++ (S3), CRRCSim LaRCSim FDM — **NEW: libzstd** (system lib present:
`/usr/bin/zstd`, `libzstd.so`)
**Storage**: S3 per-mode buckets `autoc-m1` / `autoc-m2` / `autoc-eval` (run-id naming uniform
across all: `autoc-<id>/gen<10000−N>.dmp.zst`); `data.dat` retired; `data.stc` per-gen aggregate
retained
**Testing**: GoogleTest (unit + contract); operator-run rebuild-perf bit-replay gate (per-scenario-
score equality, NOT whole-dmp — dmps carry non-deterministic provenance timestamps); `dmp-dump`
round-trip
**Target Platform**: Linux aarch64 training box (promaxgb10-4331) + x86 dev; cereal binary
portability assumed (not re-validated here)
**Project Type**: single project — evolutionary training engine + CLI tools + crrcsim worker
**Performance Goals**: zstd must not bottleneck training (level ~10–19, chosen by spike against a
real 40 MB dmp); per-eval RPC footprint unchanged; perf build stays FP-deterministic
**Constraints**: absolute determinism / bit-replay within a build
([project_variation_design_principles]); no cereal version bump (greenfield, fail-loud read);
basin lottery ~1:3 → budget 2–3 bakes per mode
**Scale/Scope**: M1 bake pop=8000 / wind=36 (~216 scenarios); M2 tracker bake ~294 source
scenarios; energy axis adds 1 lexicase test-case per scenario

**Resolved unknowns (were NEEDS CLARIFICATION; see research.md):** convex `f(out_th)` form;
MAD-relative epsilon computation set; rebuild-perf gate basis; zstd level + container layout;
`dmp-dump` output format; bucket-migration sequencing; FR-002 axis-grouping applicability with
stability off.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|---|---|---|
| I. Testing-First | ✅ | Re-enable 4 Selection027 tests (FR-004); new tests for MAD-epsilon, convex energy metric, `dmp-dump` round-trip, shared selector, run-id uniformity. Tests precede impl per task ordering. |
| II. Build Stability | ✅ | Each task leaves build green; libzstd added to top-level CMake before any include. |
| III. No Compatibility Shims | ✅ | `data.dat` clean-cut (no dual-write, FR-P05); `runIdPrefixForMode` `tracker-` branch removed (FR-P07b). Accepting legacy plain `.dmp` on read is a **Principle-V backward-compatible read**, not a III shim. |
| IV. Unified Build | ✅ | libzstd declared once at top-level CMake; crrcsim mod_inputdev gets it transitively via autoc_common (also closes the cherry-pick fragility). |
| V. Versioned Persistence | ✅ | dmp schema grows (energy/derived); greenfield, no cereal version bump per practice; reader fails loud on mismatch; `.zst` container read path accepts legacy `.dmp` (documented migration). |
| VI. Type-Domain Discipline | ✅ | `energy_score` is `gp_fitness`; convex `f()` + MAD math use `gp_fitness`/`gp_scalar`; run the grep gate on `src/eval/` touched paths at close. |
| VII. No Silent Fallback Defaults | ✅ | MAD-epsilon selectable via ini field (no in-class masking default); dmp loader fails loud on missing `TrackerSourceRun` (FR-P13). |
| VIII. Training-Artifact Lifecycle & Retention | ✅ | This feature **implements** Principle VIII (added this session): tag-on-upload, lifecycle, uniform naming, zstd, fail-loud loader. |

**No violations.** Complexity Tracking left empty.

## Project Structure

### Documentation (this feature)

```text
specs/035-energy-lexicase-objective/
├── plan.md              # This file
├── research.md          # Phase 0 — resolved decisions
├── data-model.md        # Phase 1 — schema/struct changes
├── quickstart.md        # Phase 1 — the verification-gate runbook
├── contracts/           # Phase 1 — CLI / config / lifecycle contracts
│   ├── dmp-dump-cli.md
│   ├── s3-selector.md
│   ├── ini-config.md
│   └── lifecycle-policy.json
└── tasks.md             # Phase 2 (/speckit.tasks — NOT created here)
```

### Source Code (repository root)

```text
src/
├── autoc.cc                       # data.dat tearout (FR-P05); upload tagging (FR-P10); run-id uniform (FR-P07b)
├── eval/
│   ├── fitness_decomposition.cc   # energy_score → convex integral (FR-001b); ScenarioScore
│   ├── selection.cc               # re-enable energy axis (FR-001); MAD epsilon (FR-003); gen-0 (timing)
│   └── source_dmp_loader.cc       # zstd inflate on read (FR-P09); fail-loud (FR-P13)
├── util/
│   ├── config.{h,cc}              # S3Bucket per mode; MAD-epsilon switch; (X-macro field list)
│   ├── run_id.h                   # collapse runIdPrefixForMode → uniform "autoc-" (FR-P07b)
│   └── s3_run_selector.{h,cc}     # NEW — shared findLatestRun(bucket)/extractGenNumber (FR-P07)
include/autoc/
├── eval/fitness_decomposition.h   # ScenarioScore (energy_score doc; crash_cost NOT added — deferred)
└── eval/aircraft_state.h          # (source of per-tick throttleCommand for energy integral)
tools/
├── dmp_dump.cc                    # NEW — S3/local .dmp[.zst] → CSV (per-tick) + YAML (meta) (FR-P02)
├── nnextractor.cc                 # route through shared selector; .zst aware
└── renderer.cc                    # route through shared selector; fix extractGenNumber invert bug; .zst
tests/
├── selection_tests.cc            # re-enable 4 DISABLED_ Selection027 (FR-004); MAD-epsilon tests
├── s3_run_selector_tests.cc      # NEW — shared selector + uniform naming
├── run_id_prefix_tests.cc        # update for uniform prefix (FR-P07b)
├── energy_metric_tests.cc        # NEW — convex integral correctness
└── dmp_dump_tests.cc             # NEW — round-trip / column parity
crrcsim/src/mod_inputdev/         # link autoc_common (libzstd transitive); zstd on source dmp read
CMakeLists.txt                     # find/ link libzstd at top level (Principle IV)
specs/03[2-5]*/*.py               # repoint from data.dat → dmp-dump CSV/YAML (FR-P03)
scripts/                           # rebuild-perf gate doc update → per-scenario-score equality (FR-P04)
autoc.ini, autoc-tracker.ini, autoc-eval*.ini, autoc-basic-m1.ini   # S3Bucket per mode; MAD switch; FR-P13 repoint
```

**Structure Decision**: Single-project layout (Constitution Architecture). Phase A touches
serialization / S3 / tooling surfaces; Phase B is concentrated in `fitness_decomposition.{h,cc}`
+ `selection.cc` + tests. The only **new** modules are `s3_run_selector` (extracted shared logic)
and `tools/dmp_dump.cc`; everything else is in-place modification.

## Phasing & the verification gate

```
Phase A (pre-work) ──► [VERIFICATION GATE] ──► Phase B (energy) ──► [BAKES + verdict]
```

**Phase A — foundation (no fitness changes):**
- A1. Shared S3 selector + uniform run-id naming (FR-P07/P07b) + renderer invert-bug fix.
- A2. data.dat retirement + dmp schema additions + zstd (FR-P01–P06, P09) — one cereal pass;
  `dmp-dump` CLI (FR-P02) + Python repoint (FR-P03).
- A3. Per-mode buckets + tag-on-upload + lifecycle + ini migration (FR-P08, P10–P12). Admin-side
  bucket creation / IAM / lifecycle are operator prerequisites (tracked, not code).
- A4. FR-P13 repoint dangling eval-tracker source; FR-P14 verify eval bugs already fixed;
  FR-P15 constitution principle (DONE this session).

**VERIFICATION GATE (operator-run, the user's explicit ask):**
1. **basic M1 train works** — `autoc-basic-m1.ini` (pathgen, pop 3000): produces `.dmp.zst` under
   `autoc-m1`, `retain=expire`-tagged, **zero `data.dat`** written.
2. **basic eval equivalent** — rebuild-perf bit-replay: eval reproduces training per-scenario
   scores byte-for-byte (FR-P04; per-scenario-score equality, dmps non-deterministic).
3. **basic M2 works** — tracker smoke (`autoc-tracker.ini`): `.dmp.zst` under `autoc-m2`; source
   dmp resolves (FR-P13).
4. **we can read the dmp files** — `dmp-dump <s3-uri>` emits CSV+YAML that the existing plot
   scripts consume.

Only when 1–4 are green does Phase B begin.

**Phase B — energy investigation:**
- B1. `energy_score` → convex throttle-command integral (FR-001b).
- B2. Re-enable energy lexicase axis, gen-0, physical grouping note (FR-001/002, timing).
- B3. MAD-relative epsilon + constant-ε ini switch (FR-003).
- B4. Re-enable 4 Selection027 tests (FR-004).
- B5. M1 energy bake (FR-005) + M2 energy bake (FR-005b); 2–3 bakes/mode.
- B6. Outcome doc + classification (SC-001/002) + total-energy go/no-go (FR-007).

## Complexity Tracking

> No constitution violations — section intentionally empty.
