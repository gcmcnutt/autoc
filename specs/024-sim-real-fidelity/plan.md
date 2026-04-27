# Implementation Plan: 024 — Sim/Real Fidelity

**Branch**: `024-sim-real-fidelity` | **Date**: 2026-04-19 | **Spec**: [spec.md](./spec.md)
**Input**: `/home/gmcnutt/autoc/specs/024-sim-real-fidelity/spec.md`

## Summary

Close the sim-to-real dynamics gap by methodically verifying every NN-visible
sensor signal is self-consistent, conforms to
[COORDINATE_CONVENTIONS.md](../../docs/COORDINATE_CONVENTIONS.md) on both sim
and real sides, and is delivered on a uniform 100 ms sample cadence. Feature
closes after: research → corrections → retrain → eval → flight test of
retrained (unchanged-topology) NN showing sim-parity dynamics.

Technical approach: Phase 0 research re-derives exact column conventions for
INAV blackbox, blackbox-tools decode, CRRCSim headless inner loop, and Xiao
timer source from code. Phase 1 produces a sensor-data dictionary (contract),
plan out the analysis tool for both flight CSV and sim `data.dat`, a
compound-attitude bench plan, and a cadence-fix design choice.

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim, xiao), Python 3.11 (analysis scripts)
**Primary Dependencies**: Eigen (vec3/quat math), cereal, inih, GoogleTest,
CRRCSim LaRCSim FDM, INAV MSP protocol, numpy + matplotlib
**Storage**: File-based — `data.dat` (sim), blackbox CSV (flight), xiao flash logs
**Testing**: GoogleTest (C++); Python scripts are diagnostic (no test framework)
**Target Platform**: Linux desktop (training + analysis), Seeed XIAO BLE Sense
**Project Type**: Multi-component — evolution engine + FDM + embedded target + analysis
**Performance Goals**: 100 ms (10 Hz) NN eval cadence sim and real; 50 Hz path-ready
**Constraints**: Zero NN topology changes; eval-suite determinism preserved;
build stability on main per constitution
**Scale/Scope**: 15 work items across sensor audit, fixes, cadence, bench, retrain, flight

**Resolved clarifications** (from [spec.md Clarifications](./spec.md#clarifications)):

- Retrain + eval + flight-test all in 024 scope; NN topology unchanged
- Use flight-20260417 for WI1 first; supplementary flight only as fallback
- Pass/fail: sign-inversion gates fail; correlation reviewed not gated
- History buffer: current-sim-tick at `recordErrorHistory` call site
- Cadence approach deferred to Phase 0 (both CRRCSim + Xiao timing)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Testing-First — PASS

- WI15 adds contract tests (engage reset, engage delay, NN input invariants).
- WI7 (minisim q_EB) closes with a locking unit test.
- Each WI3 fix pairs with a regression check (cross-check failure→pass).

### II. Build Stability — PASS

- WI3/WI4 changes land as small commits with `scripts/rebuild.sh` + `pio run`
  verified before push.
- Retrain (Validation #5) requires xiao build success with new weights.

### III. No Compatibility Shims — PASS

- WI3 fixes canonicalize conventions at producers (msplink, minisim), not
  consumer-side shims.
- WI10 rewrites rotted analysis scripts rather than shimming old assumptions.
- WI9 schema version fails loud on drift — not a compat layer.

### IV. Unified Build — PASS

- No new top-level CMake targets. Fixes touch existing sources.
- Python analysis stays diagnostic, outside CMake.

**No violations. Proceed to Phase 0 research.**

## Project Structure

### Documentation (this feature)

```text
specs/024-sim-real-fidelity/
├── spec.md                                      # Feature spec
├── plan.md                                      # This file
├── research.md                                  # Phase 0 — research output
├── data-model.md                                # Phase 1 — sensor data dictionary
├── contracts/
│   ├── blackbox_csv_contract.md                 # INAV blackbox CSV columns
│   ├── xiao_log_contract.md                     # xiao flight log format
│   └── sim_data_dat_contract.md                 # sim data.dat columns
├── quickstart.md                                # Phase 1 — reproduce WI1 audit
├── tasks.md                                     # Phase 2 output (not here)
├── gyro_vs_quat_sim.py / .png                   # Existing (sim rate scatter)
├── cmd_response_scatter_sim_lagged.py / .png    # Existing (sim lag scatter)
└── sim_polar_viz.py                             # Existing (sim polar viz)
```

### Source Code (repository root)

Touches five existing code areas. No new top-level directories.

```text
autoc/
├── tools/renderer.cc                     # WI11 legacy path decision
├── tests/                                # WI15 new contract tests
│   ├── engage_reset_tests.cc
│   ├── engage_delay_tests.cc
│   └── nn_inputs_tests.cc
├── crrcsim/src/mod_inputdev/inputdev_autoc/
│   └── inputdev_autoc.cpp                # WI4 cadence fix (submodule)
├── xiao/src/msplink.cpp                  # WI3 pitch axis, WI8 rabbit logging
├── include/autoc/eval/aircraft_state.h   # WI7 minisim q_EB
├── include/autoc/nn/nn_inputs.h          # WI9 schema version
└── specs/018-flight-analysis/...         # WI10 script rewrites

flight-results/flight-20260417/
├── sensor_self_check.py                  # WI1 new — main audit
└── sensor_self_check.png                 # WI1 report output

docs/
├── COORDINATE_CONVENTIONS.md             # Update per WI1/WI3/WI5 findings
└── INAV_BLACKBOX.md                      # NEW — Phase 0 reference
```

**Structure Decision**: No layout changes. Each WI lands as a narrow
commit against the existing tree. Phase 0 research lives both in
`research.md` (this feature) and in a new permanent `docs/INAV_BLACKBOX.md`.

## Complexity Tracking

No constitution violations. No complexity to justify.
