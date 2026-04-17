# Sim-to-Real Dynamics Analysis

Comparative analysis of sim training dynamics vs real flight blackbox data.
Run this analysis after each training run or flight to track sim fidelity.

## Units Reference

| Source | Gyro rates | Attitude | RC commands | Position |
|--------|-----------|----------|-------------|----------|
| data.dat (sim) | **rad/s** (gyrP/Q/R) | quaternion (qw/qx/qy/qz) | [-1, +1] (outPt/Rl/Th) | meters (NEU) |
| INAV blackbox | **deg/s** (gyroADC) | decidegrees (attitude) | [-500, +500] (rcCommand) | cm (BaroAlt) |

Normalize for comparison: sim rad/s * 57.296 = deg/s; flight rcCommand / 500 = [-1, +1].

## Analysis Method

1. Extract last-generation data from data.dat (best individual, all path/wind combos)
2. Extract full-flight blackbox CSV (manual flight preferred, no PID interference)
3. Compare: rate magnitudes, control authority (rate per unit command), output distributions, attitude envelope, tracking distances

---

## 2026-04-15 — hb1-adjust4 gen 249 vs flight-20260322 manual

### Config
- **Training**: hb1-adjust4 (yaw coupling), lexicase selection, pop=3500, 5 paths x 49 winds
- **Sim model**: hb1_streamer.xml with adjust4 tuning
- **Fitness**: conical surface (behind=7m, ahead=2m, cone=45°), streak 1-5x over 5s
- **Training progress**: gen 249/400, best fitness=-24588 (best run yet; test4=-16k, test7=-21k, adjust3=-17k)
- **Flight**: 2026-03-22 manual mode (no PID), 3 test spans, BIG-3 gen400 weights

### Rate Magnitudes (std, deg/s)

| Axis | Sim | Flight | Sim/Flight |
|------|-----|--------|------------|
| Roll | 155 | 86 | 1.80x |
| Pitch | 120 | 68 | 1.77x |
| Yaw | 86 | 35 | 2.48x |

Sim is ~1.8x more agile than real aircraft. Yaw particularly overdone at 2.5x.
This **reverses** the March 22 finding (flight had lower authority); the adjust4
model over-corrected.

### Control Authority (deg/s per full-deflection command)

| Axis | Sim | Flight | Sim/Flight |
|------|-----|--------|------------|
| Roll | 139 | 226 | 0.62x |
| Pitch | 152 | 118 | 1.29x |

Roll authority lower in sim per unit command, but NN compensates with saturation.
Pitch authority slightly higher in sim. Note: different sample rates/lag make this
an approximate comparison (sim 1-step lag at 117ms; flight instantaneous at ~10ms).

### Output Distributions

**Saturation (|val| > 0.95):**

| Channel | Sim | Flight (pilot) |
|---------|-----|----------------|
| Roll | 55% | 0% |
| Pitch | 3% | 15% |
| Throttle | 42% | n/a |

**Proportional (|val| < 0.3):**

| Channel | Sim | Flight (pilot) |
|---------|-----|----------------|
| Roll | 32% | 80% |
| Pitch | 7% | 71% |

Roll is heavily bang-bang in sim (bimodal U-shape: 27% at -1.0, 28% at +1.0).
Pitch is "bang-and-hold" biased positive (mean +0.36, nose-up for altitude).
Throttle is binary (42% full, 13% idle).

### Bank Angle Distribution

| Range | Sim | Flight |
|-------|-----|--------|
| <10° (level) | 14% | 22% |
| 10-30° (moderate) | 25% | 37% |
| 30-60° (steep) | 29% | 19% |
| >60° (extreme) | 32% | 22% |

Sim spends 60% in steep/extreme bank vs 41% in flight.

### Tracking Quality (Sim Only)

| Metric | Value |
|--------|-------|
| Mean distance | 6.5m |
| p50 distance | 5.1m |
| p95 distance | 16.0m |
| p99 distance | 25.1m |
| Mean closing rate | -0.35 m/s (slightly opening) |
| avgMaxStreak | 25.9 |
| pctInStreak | 34.4% |

### Fitness Progression

| Gen | Best Fitness | Sigma | avgMaxStreak | pctInStreak |
|-----|-------------|-------|--------------|-------------|
| 1 | -1989 | 0.200 | 1.6 | 1.9% |
| 50 | -5981 | 0.186 | 2.4 | 2.1% |
| 100 | -13671 | 0.161 | 11.0 | 15.6% |
| 150 | -18941 | 0.140 | 18.4 | 26.0% |
| 200 | -22924 | 0.123 | 23.4 | 33.1% |
| 249 | -24588 | 0.105 | 25.9 | 34.4% |

### Assessment

1. **Rates in the right ballpark** — 1.8x gap is the closest match yet
2. **Roll bang-bang is the biggest transferability risk** — 55% saturated, 4.6 sign-changes/s
3. **Yaw coupling over-tuned** — 2.5x real rates; may cause spiral/degenerate behavior in flight
4. **Throttle binary** — no energy management learned
5. **Still improving** — 150 gens remaining, fitness curve not plateaued
