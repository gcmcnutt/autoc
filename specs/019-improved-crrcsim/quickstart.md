# Quickstart: 019 Improved CRRCSim Fidelity

**Date**: 2026-03-24 | **Branch**: `019-improved-crrcsim`

## Prerequisites

- autoc builds: `bash scripts/rebuild.sh`
- crrcsim builds (part of unified build)
- Flight blackbox CSVs: `eval-results/bench-20260322/blackbox_log_2026-03-22_212133.01.csv`
- BIG3 training data: `20260323-data.dat`, `20260323-data.stc`
- Python 3.11 with csv, math (stdlib only for analysis scripts)

## Key files

| File | Purpose |
|------|---------|
| `crrcsim/models/hb1_streamer.xml` | Aircraft model — tuning target |
| `crrcsim/src/mod_fdm/fdm_larcsim/fdm_larcsim.cpp` | FDM aero computation |
| `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` | NN→FDM control mapping |
| `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h` | Latency constants |
| `include/autoc/eval/aircraft_state.h` | AircraftState + PhysicsTraceEntry |
| `docs/COORDINATE_CONVENTIONS.md` | Units and frame reference |
| `docs/sensor-pipeline.md` | Full pipeline documentation (018) |

## Tuning workflow (per axis)

### 1. Extract flight response curve

```bash
# Analyze blackbox CSV for throttle/roll/pitch response
python3 scripts/flight_response.py \
  --axis throttle \
  --csv eval-results/bench-20260322/blackbox_log_2026-03-22_212133.01.csv
```

### 2. Extract sim response curve

```bash
# Analyze data.dat for same axis
python3 scripts/sim_response.py \
  --axis throttle \
  --data 20260323-data.dat \
  --gen-range 1-5
```

### 3. Compare and tune

Side-by-side comparison (ascii art). Adjust parameters in hb1_streamer.xml.

### 4. Mini training run (10-20 gens)

```bash
# Quick training to regenerate sim curves
# (use autoc.ini with reduced generations)
./build/autoc --config autoc-mini.ini
```

### 5. Re-extract and compare

Repeat steps 2-3 with new data.dat until curves converge.

## Pipeline latency update

After aero tuning is complete:

```bash
# Bench test with custom MSP command (after INAV + xiao firmware update)
# Measure new pipeline latency
# Update COMPUTE_LATENCY in inputdev_autoc.h:
#   #define COMPUTE_LATENCY_MSEC_DEFAULT <measured_value>
```

## Full training run

```bash
# BIG training with calibrated model
nohup ./build/autoc --config autoc.ini > logs/autoc-019-BIG.log 2>&1 &

# Monitor progress
tail -f logs/autoc-019-BIG.log
# Check data.stc for fitness trajectory
tail -20 data.stc
```

## Validation

Compare trained controller's data.dat response curves to flight blackbox.
Response curves should visually resemble each other across the operating envelope.
