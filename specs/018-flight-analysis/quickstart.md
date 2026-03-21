# Quickstart: Flight Analysis

## Prerequisites

- Flight data in `eval-results/flight-YYYYMMDD/`:
  - INAV blackbox file (`blackbox_*.TXT`)
  - Xiao flight log (`flight_log_*.txt`)
- blackbox-tools built: `~/blackbox-tools/obj/blackbox_decode`
- Python 3.11+ with standard library (no pip dependencies)
- autoc renderer built: `build/renderer`

## Workflow

### 1. Decode INAV blackbox

```bash
~/blackbox-tools/obj/blackbox_decode eval-results/flight-YYYYMMDD/blackbox_*.TXT
```

Produces `.01.csv`, `.01.gps.csv`, `.01.gps.gpx`, `.01.event` alongside the source file.

### 2. Correlate sensor data

```bash
python3 scripts/correlate_flight.py \
  eval-results/flight-YYYYMMDD/blackbox_*.01.csv \
  eval-results/flight-YYYYMMDD/flight_log_*.txt
```

Outputs: per-tick correlation with position/velocity/quaternion deltas, test span identification, timing analysis.

### 3. Visualize in renderer

```bash
build/renderer -x eval-results/flight-YYYYMMDD/flight_log_*.txt
```

Shows flight tape with rabbit path overlay. Use keyboard to step through test spans.

### 4. Generate analysis charts

```bash
python3 scripts/flight_charts.py eval-results/flight-YYYYMMDD/
```

Produces standard plots: heading, attitude, position, velocity, timing histograms.

### 5. Verify sim projection

```bash
build/renderer -s <S3_KEY>  # Load a known-good sim eval
# Enable projection overlay to verify math against actual rabbit path
```

## Key Files

| File | Purpose |
|------|---------|
| `scripts/correlate_flight.py` | INAV/xiao join tool |
| `scripts/flight_charts.py` | Standard analysis charts |
| `scripts/verify_flight_log.py` | Bench test verification (existing) |
| `scripts/eval_suite.sh` | Automated regression (existing) |
| `tools/renderer.cc` | 3D visualization with flight replay |
| `docs/sensor-pipeline.md` | Pipeline documentation |
| `docs/coordinate-conventions.md` | Coordinate reference |
