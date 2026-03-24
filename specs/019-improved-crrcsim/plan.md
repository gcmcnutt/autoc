# Implementation Plan: 019 Improved CRRCSim Fidelity

**Branch**: `019-improved-crrcsim` | **Date**: 2026-03-24 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/019-improved-crrcsim/spec.md`

## Summary

CRRCSim's hb1_streamer.xml doesn't match the real aircraft closely enough — the NN
converges to degenerate strategies (full pitch + full throttle) that exploit sim physics.
This feature tunes the sim to approximate reality by comparing response curves
(throttle→speed, roll→rate, pitch→rate) between flight blackbox data and data.dat,
iterating until they look similar. Then reduces MSP pipeline latency and retrains.

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim), C (INAV autoc branch), C++ (xiao/PlatformIO), Python 3.11 (analysis scripts)
**Primary Dependencies**: Eigen (math), cereal (serialization), CRRCSim LaRCSim FDM, INAV MSP protocol
**Storage**: File-based — hb1_streamer.xml (model), data.dat (training output), blackbox CSV (flight data)
**Testing**: GoogleTest (autoc), bench testing on flight hardware (xiao/INAV), response curve comparison (manual/visual)
**Target Platform**: Linux desktop (training), Seeed XIAO BLE Sense nRF52840 (embedded), INAV on STM32F405 (flight controller)
**Project Type**: Simulation tuning + embedded firmware
**Performance Goals**: Pipeline latency 49ms → ~27ms, sim response curves within 2× of flight across speed envelope
**Constraints**: No dedicated step-function flights available — use existing data. INAV stays on 8.0.0 autoc branch (no major upgrade).
**Scale/Scope**: ~5 files modified in crrcsim, ~3 files in INAV, ~2 files in xiao, ~3 new Python analysis scripts

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Testing-First | PASS | Aero tuning validated by response curve comparison (visual test). Pipeline changes bench-tested. INAV servo fix bench-verified. |
| II. Build Stability | PASS | All changes compile-checked: `scripts/rebuild.sh` (autoc+crrcsim), `pio run` (xiao). INAV builds separately on autoc branch. |
| III. No Compatibility Shims | PASS | Direct parameter changes to hb1_streamer.xml. Custom MSP command is additive (new handler, no shims). |
| IV. Unified Build | PASS | crrcsim remains `add_subdirectory(crrcsim)`. No new dependencies. Analysis scripts are standalone Python (stdlib only). |

**Post-Phase 1 re-check**: All gates still pass. No new dependencies introduced.
hb1_streamer.xml changes are data-only (no code structure changes). MSP2_AUTOC_STATE
is additive to INAV's existing MSP handler dispatch.

## Project Structure

### Documentation (this feature)

```text
specs/019-improved-crrcsim/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0: FDM structure, parameters, MSP feasibility
├── data-model.md        # Phase 1: Entities, tuning parameters, MSP payload
├── quickstart.md        # Phase 1: Tuning workflow commands
└── tasks.md             # Phase 2 output (/speckit.tasks)
```

### Source Code (repository root)

```text
# CRRCSim (submodule: crrcsim/)
crrcsim/models/hb1_streamer.xml                              # Tuning target (Phases 1-3)
crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h     # COMPUTE_LATENCY update (Phase 6)
crrcsim/src/mod_fdm/fdm_larcsim/fdm_larcsim.cpp             # FDM reference (read-only for understanding)

# Analysis scripts (new)
scripts/flight_response.py          # Extract response curves from flight blackbox CSV
scripts/sim_response.py             # Extract response curves from data.dat
scripts/compare_response.py         # Side-by-side comparison (ascii art)

# INAV (external: ~/inav, autoc branch)
src/main/fc/fc_msp.c               # MSP2_AUTOC_STATE handler (Phase 6)
src/main/blackbox/blackbox.c        # Servo logging fix (Phase 6)

# Xiao firmware (xiao/)
xiao/src/msplink.cpp                # MSP2_AUTOC_STATE parser + flight mode override (Phase 6)
xiao/src/MSP.cpp                    # MSP2 command support (if not already present)

# Reference (read-only)
docs/COORDINATE_CONVENTIONS.md      # Units and frames
docs/sensor-pipeline.md             # Full pipeline documentation (018)
```

**Structure Decision**: No new directories. Analysis scripts go in existing `scripts/`.
hb1_streamer.xml is data-only modification. INAV and xiao changes are minimal local
patches to existing files.

## Complexity Tracking

No constitution violations to justify.
