# Implementation Plan: 044 — M1 craft-sensor history + roll plant

**Branch**: `044-m1-craft-history` | **Date**: 2026-10-19 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `specs/044-m1-craft-history/spec.md` (clarified 2026-10-19)

## Summary

Give the M1 policy its own recent body rates and normal acceleration (gyro p/q/r + accel z at 100/50 ms, +8 inputs, layout cleaned so
history sits beside its current value, shared with M2), make the roll airframe partly honest (a `craftClP` variation class centred on a
3 Hz roll corner with aileron authority scaled to keep the steady roll rate), widen pitch damping to the streamer, model the FC sensor
filter delays, fix three baseline-boundary defects (independent variation streams, wind-draw logging, explicit eval strength), profile
the training cost, prove attribution with two smokes, bake 800 generations, pick by equal environment and fly. 045 M2 waits until the
roll and pitch flight criteria pass (≥ 15 % below the 2026-10-18 reference).

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim, xiao firmware), Python 3.11 (analytics)
**Primary Dependencies**: Eigen, cereal, inih, GoogleTest, CRRCSim LaRCSim FDM, INAV MSP (unchanged), PlatformIO (xiao)
**Storage**: files — dmp (S3 `autoc-m1` / `autoc-eval`), data.dat, xiao flight log, blackbox CSV
**Testing**: GoogleTest via ctest; tier-0 eval-vs-training bitwise gate; flight-log round trip; step-test harness
**Target Platform**: Linux training box (DGX Spark, 20 cores); Seeed XIAO BLE Sense (companion); INAV 8.0 fork on MATEKF722MINI (unchanged)
**Project Type**: evolution engine + flight-dynamics simulator + embedded controller
**Performance Goals**: training results bitwise unchanged by any adopted speed-up; the 800-gen bake fits in ~5 days (t6 took 4.76)
**Constraints**: FP determinism (`rebuild-perf.sh` basis); no rebuild during training; the xiao forward pass stays within its tick budget (+~1 %)
**Scale/Scope**: 294 scenarios × 5000 population × 800 generations; 39 layout-touching files (README code scan)

No NEEDS CLARIFICATION remain (spec clarified; plan-level unknowns resolved in [research.md](research.md)).

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-checked after Phase 1 design.*

| principle | how 044 complies | status |
|---|---|---|
| I Testing-first | tests before code for: the index-free check and layout-signature refusals, history ring + pre-fill, sim/xiao vector equality, layout asserts, stream independence, `craftClP` draw/clamp + `Cl_da` scaling, eval-scale key validation, flight-log v6 round trip | ✅ |
| II Build stability | `rebuild.sh` + ctest + xiao `pio run` at every commit | ✅ |
| III No compatibility shims | one clean format break; all callers updated; `recordErrorHistory` signature change forces both call sites | ✅ |
| IV Unified build | header-only additions expected; any CMakeLists change → clean `rebuild-perf.sh` (operator) | ✅ |
| V Versioned artifacts | dmp schema 4 → 5, flight log 5 → 6, layout marker v3 → v4; old readers fail loud | ✅ |
| VI Type-domain discipline | new eval/nn code uses `gp_scalar`/`gp_vec3`; raw floats only in NN byte-format structs with `// raw-ok:`; grep audit at close | ✅ |
| VII No silent fallback defaults | `craftClP` follows `craftCmQ` (wire-struct default, NaN-unset in crrcsim, never a silent value); `EvalVariationScale` explicit −1 sentinel, validated | ✅ |
| VIII Artifact retention | bake + flown genome + 043 eval dumps pinned `retain=keep`, prefixes in the MANIFEST / outcome | ✅ |
| IX Detached launch + pre-run gate | smokes and bake via `scripts/train.sh`; clean build + tests before each | ✅ |
| X Single ordered backlog | deferrals (non-bitwise speed-ups, accel x/y revisit) go to `specs/BACKLOG.md` | ✅ |

Post-design re-check (after data-model / contracts): no new violations; Complexity Tracking empty.

## Project Structure

### Documentation (this feature)

```text
specs/044-m1-craft-history/
├── README.md            # derivation of record + code scan
├── spec.md              # governs (clarified 2026-10-19)
├── plan.md              # this file
├── research.md          # Phase 0 (R1–R14)
├── data-model.md        # Phase 1
├── quickstart.md        # Phase 1 operator runbook
├── contracts/           # nn-input-layout-v4, flight-log-v6, nn01-genome, config-keys, scenario-table
├── checklists/requirements.md
└── tasks.md             # Phase 2 (/speckit.tasks — not created here)
```

### Source Code (repository root)

```text
include/autoc/nn/          nn_inputs.h (layout v4, craft history lags), topology.h (counts), input_mask.h, mode.h
include/autoc/eval/        aircraft_state.h (craft ring, recordErrorHistory), craft_variation.h (craftClP, CmQ re-centre),
                           sensor_lpf.h (filters), craft_observations.h, envelope_state.h, tracker_stepper.h
include/autoc/util/        scenario_prng.h (fmix32 sub-seeds), config.h (CraftClPSigma, EvalVariationScale)
include/autoc/rpc/         scenario_metadata.h (craftClP), protocol.h (schema 5)
src/                       autoc.cc (draw/log/eval scale), nn/evaluator.cc (gather), nn/mode.cc, util/scenario_prng.cc
crrcsim/src/               global.{h,cpp} (craftClP), mod_fdm/fdm_larcsim (Cl_p/Cl_da), mod_inputdev/inputdev_autoc (record,
                           sensor filters at sub-step, tracker helper)
tools/                     dmp_dump.cc, nn2cpp.cc (layout signature assert), nnextractor.cc (signature/count refusal), renderer.cc
src/nn/serialization.cc    NN01 version bump + layout signature
xiao/                      include/flight_log_format.h (v6), src/msplink.cpp (record), web/flight_logger.html (decoder)
src/analytics/             flightlog_decode.py (INPUT_NAMES v4), input_investment.py, pitch_cycle_progress.py (+roll)
specs/043-acro-dual-loop/  analysis scripts ported to read columns by name
tests/                     the 14 layout tests + new tests listed under Principle I
```

**Structure Decision**: existing single-repo layout (autoc + crrcsim submodule + xiao); no new projects or targets.

## Execution order (gates)

1. **Preserve** — pin 043 eval dumps; tag `044-last-45in` at step 4's end (FR-005).
2. **Profile** — per-tick A/B, parent profile → `profile.md`; bitwise-neutral speed-ups only (FR-015).
3. **Plant** on the 45-input build — `craftClP` + `Cl_da` scaling, CmQ re-centre, sensor filters; **roll honesty ladder passes**
   (FR-006…008, research R11).
4. **Baseline fixes** — stream independence, wind-draw columns, `EvalVariationScale`; new tier-0 baselines (FR-009…011). → tag.
5. **Smoke A** (plant only, 45 inputs) (FR-012).
6. **Format break** — **index-free gate first** (check added to ctest, existing literal hits fixed, flight-log header name table,
   layout signature in NN01 / nn2cpp / logs — FR-016), then layout v4, recording on both targets, versions, `nnextractor` / `nn2cpp` /
   xiao regenerate and build, decoders, tests, script ports (FR-001…004); the gate passes again after the break.
7. **Smoke B** (plant + history) → SC-006 recorded.
8. **Bake** `autoc-044-t1-…`, 800 gens, MANIFEST first (FR-013).
9. **Pick → fly → judge** (FR-014, SC-001…005); miss on SC-001/002 → another cycle; 045 waits.

## Complexity Tracking

None — no constitution violations to justify.
