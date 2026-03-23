# Flight Analysis: 2026-03-22

## Flight Configuration
- **Weights**: BIG-3 gen 400, fitness 2724 (autoc-9223370262877190993-2026-03-20T03:33:04.814Z/gen9600.dmp)
- **Flight mode**: MANUAL (not ACRO — confirmed via INAV config)
- **Firmware**: 20Hz loop refactor (T221c), no arm delay (T221a), rabbit logging (T222)
- **Paths tested**: 0=StraightAndLevel, 2=HorizontalFigureEight, 1=SpiralClimb

## Test Spans

| Span | Path | Duration | Termination | NN Ticks |
|------|------|----------|-------------|----------|
| 1 | StraightAndLevel | 12.8s | Path complete | 126 |
| 2 | HorizontalFigureEight | 14.0s | Pilot switch | 136 |
| 3 | SpiralClimb | 15.6s | Pilot switch | 153 |

## Key Findings

### 1. Sensor Pipeline: CONFIRMED CORRECT
- Position X/Y correlation: <0.01m error (2013 samples matched)
- Z negation (NEU→NED): consistent across all spans
- Rabbit position logging: working, shows correct path geometry
- Renderer projection: racetrack, figure-eight, spiral all reconstruct correctly

### 2. Control Response: SIGNS CORRECT, GAINS MISMATCHED
- Roll command → roll rate: r=+0.45 (strong positive, correct direction)
- Pitch command → pitch rate: r=+0.26 (positive, correct direction)
- Cross-axis coupling: weak (clean separation)
- Throttle → speed: positive correlation confirmed
- **Improvement over flight-20260320**: r=+0.02 (hilarious) → r=+0.45 (less hilarious)
- MANUAL mode eliminates PID interference seen in ACRO mode

### 3. AHRS Accuracy: PLAUSIBLE
- GPS heading vs AHRS heading: 7-13° offset (within normal EKF range)
- Turn rate at 35° bank, 17 m/s: 23°/s measured = 23°/s expected
- Accelerometer at rest: [0.12g, -0.09g, 1.00g] — correct
- Quaternion continuity: smooth, no tumble or flips through large angles
- No IMU tumble detected in any span

### 4. Tracking Quality: INITIAL TRACKING WORKS, THEN DEGENERATES

| Span | Tracking (<10m) | Intercept (10-30m) | Struggling (30-60m) | Degenerate (>60m) | Saturated (≥2/3) |
|------|----------------|-------------------|--------------------|--------------------|-----------------|
| 1 | 17% | 11% | 30% | 41% | 14% |
| 2 | 21% | 33% | 24% | 22% | 46% |
| 3 | 16% | 12% | 8% | 63% | 61% |

**Pattern**: First 2-3 seconds show real tracking (dist <10m, proportional commands).
Then divergence begins (dist grows, NN saturates). By 5-8s, the NN locks into a fixed
degenerate strategy: `out=[0.995, 0.573, 0.999]` — full pitch + moderate roll + full
throttle. This is the same spiral attractor from training, re-emerging because the real
craft dynamics don't match the sim.

**Root cause**: The NN was trained with entry variations of ±20m/±3m. Once the real craft
drifts beyond ~30m (due to gain mismatch), the NN is outside its training distribution
and defaults to a saturated fixed-point strategy.

### 5. Sim vs Flight Dynamics

| Metric | Sim | Flight | Ratio |
|--------|-----|--------|-------|
| Speed range (m/s) | 7-24 | 10-23 | ~1.0× |
| Figure-8 turn rate (°/s) | peak 40 | 18-25 | 0.6-0.8× |
| Spiral climb rate (m/s) | 3.0 sustained | 4.7 peak / 0.7 avg | peak 1.6×, sustained 0.2× |
| NN output magnitude | [-0.6, +1.0] | full [-1.0, +1.0] | flight saturates more |
| Pitch rate per RC unit | ~0.5 °/s/unit (est) | ~0.36-0.46 °/s/unit | ~0.8× |

**Interpretation**: Speed ranges match (~1.0×). Turn rates are slower in flight (0.6-0.8×).
Climb rate peaks higher but can't sustain. The NN outputs are more extreme in flight —
meaning the real craft needs bigger commands for the same effect, consistent with lower
control authority than the sim model.

### 6. MSP Timing
- Pipeline latency (NN eval → send): 9.1-9.5ms avg (consistent across spans)
- Send interval: 50.4-51.4ms avg (bimodal: ~4ms send-only, ~96ms NN ticks)
- Late sends (>70ms): 125-127 out of 252-256 (exactly half — expected bimodal pattern)
- Not a concern: all within INAV's 200ms timeout

## Action Items for Next Iteration

### Immediate (before next flight)
1. **Calibrate hb1.xml** — reduce control effectiveness to ~0.7× current, adjust thrust curve
2. **Expand training envelope** — increase entry position sigma to ±50m to cover recovery scenarios
3. **Consider recovery phase** — explicit training on "return from far away" scenarios

### Investigation (post-flight analysis scripts)
4. **Per-axis rate gain measurement** — quantify °/s per unit RC command for pitch/roll/throttle
5. **INAV MANUAL mode scaling** — verify no rate/expo is applied in MANUAL mode
6. **hb1.xml parameter audit** — list key parameters, compare to known aircraft specs

## Methodology

### Data Sources
- INAV blackbox: `blackbox_log_2026-03-22_103857.TXT` → decoded with `~/blackbox-tools/obj/blackbox_decode`
- Xiao flight log: `flight_log_2026-03-22T18-02-06.txt`
- Sim reference: `~/autoc/eval-data.dat` (BIG-3 eval run)

### Analysis Scripts (in `specs/018-flight-analysis/`)
- `correlate_flight.py` — INAV/xiao timestamp join, position/velocity correlation
- `latency_analysis.py` — per-span steady-state pipeline latency
- `span_timeline.py` — activation sequence tracing

### Renderer
- `build/renderer -x <xiao_log>` — flight replay with projected rabbit path
- Fixed: quaternion body→world rotation (removed incorrect conjugation)
- Fixed: duplicate span detection (controlDisableRe double-match)
