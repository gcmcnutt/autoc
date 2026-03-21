# Implementation Plan: Flight Analysis & Sim-to-Real Verification

**Branch**: `018-flight-analysis` | **Date**: 2026-03-20 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/018-flight-analysis/spec.md`

## Summary

Build tooling to diagnose why the first NN flight test (2026-03-20) produced poor flight
behavior despite a working end-to-end pipeline. The core approach is: prove the sensor
pipeline is correct, visualize what the NN sees vs reality, verify output mapping, then
characterize real aircraft dynamics to calibrate the simulator. All analysis operates on
two data sources: INAV blackbox CSV and xiao flight logs, correlated by timestamp.

## Technical Context

**Language/Version**: C++17 (renderer, shared code), Python 3.11 (analysis scripts)
**Primary Dependencies**: Eigen (math), VTK (renderer), cereal (data.dat parsing), blackbox-tools (INAV decode)
**Storage**: File-based — flight logs, blackbox CSVs, eval-results/ directories, S3 for training artifacts
**Testing**: GoogleTest (C++), pytest or script-based verification (Python analysis tools)
**Target Platform**: Linux aarch64 (DGX Spark GB10), cross-reference with Cortex-M4F (xiao) and x86 (WSL2)
**Project Type**: Analysis tooling + firmware updates + visualization
**Performance Goals**: Analysis scripts process a 3-minute flight (~12K INAV frames, ~2K xiao ticks) in < 10s
**Constraints**: Must not require flight hardware for development — all analysis works on captured log files
**Scale/Scope**: One flight session (5 test spans) as primary dataset. Tooling must generalize to future flights.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Gate | Status | Notes |
|------|--------|-------|
| I. Testing-First | PASS | Analysis scripts have verifiable outputs (correlation errors, projection overlays). Renderer changes testable via known-good sim data. Cross-ISA comparison is inherently a test. |
| II. Build Stability | PASS | Changes span autoc (renderer, analysis scripts), xiao (rabbit logging), crrcsim (no changes yet). Each builds independently. |
| III. No Compatibility Shims | PASS | New log fields (rabbit position) are additive. Renderer updated to parse new format — no backward compat for old GP format. |
| IV. Unified Build | PASS | Analysis scripts are standalone Python. Renderer is already in the autoc CMake tree. No new build dependencies. |

## Project Structure

### Documentation (this feature)

```text
specs/018-flight-analysis/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Phase 0: coordinate convention research
├── data-model.md        # Phase 1: data entities and relationships
├── tasks.md             # Task breakdown (already exists, will be updated by /speckit.tasks)
└── checklists/
    └── requirements.md  # Spec quality checklist
```

### Source Code (repository root)

```text
# Analysis tools
scripts/
├── verify_flight_log.py        # Existing: bench test verification
├── eval_suite.sh               # Existing: automated regression
├── correlate_flight.py         # NEW: INAV/xiao join tool (T205)
└── flight_charts.py            # NEW: standard flight analysis charts (T264)

# Documentation
docs/
├── sensor-pipeline.md          # NEW: full sensor chain documentation (T208)
└── coordinate-conventions.md   # Existing: coordinate reference

# Renderer
tools/
└── renderer.cc                 # MODIFIED: parse new log format, rabbit overlay, projection

# Embedded
xiao/src/
└── msplink.cpp                 # MODIFIED: add rabbit position to NN log line (T222)

# Flight data (not tracked in git)
eval-results/
└── flight-YYYYMMDD/
    ├── blackbox_*.TXT           # Raw INAV blackbox
    ├── blackbox_*.01.csv        # Decoded CSV
    ├── blackbox_*.01.gps.csv    # GPS data
    ├── blackbox_*.01.gps.gpx    # GPX track
    ├── flight_log_*.txt         # Xiao flight log
    └── analysis/                # Generated charts and reports
```

**Structure Decision**: Analysis tools go in `scripts/` (Python, standalone). Documentation
in `docs/`. Renderer and firmware changes in existing locations. No new build targets needed.
Flight data stays in `eval-results/` (gitignored).
