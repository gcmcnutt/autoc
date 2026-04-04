# Flight Report — 2026-04-03

**Location**: Sunnyvale Baylands
**Wind**: ~330° at 10 kts (~5 m/s)
**Aircraft**: hb1 with streamer, Master Airscrew GF 5.5×4 prop (new)
**Firmware**: INAV MATEKF722MINI with MSP gyro extension, xiao gen 9743 (27 inputs, fitness 508K)
**Blackbox**: 1/32 rate — **0 Z jumps on both flights** (decode fix confirmed)

## Flight 1 (log .01): Autoc Test + Manual Dynamics (195s)

### Autoc Spans
Two MSPRCOVERRIDE spans:
- **Span 1** (t=26.9-34.9s, 8.0s): Roll ±654°/s, pitch ±269°/s, altitude 49-90m.
  Servos hitting full travel (1000-2000). Aggressive oscillation observed.
  Speed 5.6-21.7 m/s (mean 15.6). Motor 1000-1998 (full range).
- **Span 2** (t=57.4-65.1s, 7.7s): Calmer, roll ±456°/s, pitch ±148°/s.
  Speed higher (11.5-30.2 m/s, mean 22.4). Some oscillation.

**Assessment**: NN immediately commands full-authority roll (±0.87-0.99) from
the first sample. In sim this produces ~100°/s; on real hardware ~500°/s.
The aircraft tumbles within 0.8s of NN taking control. The 3.2/s roll reversal
is not subtle oscillation — it's full bang-bang limit cycling between ±1.0.

The AHRS appears to be reporting accurately — the aircraft really is tumbling
because the NN's first command kicks it to 500°/s roll rate. The renderer
shows the NN's perspective correctly; it matches the physical tumble.

Root cause: 5x roll gain mismatch between sim (106°/s at full stick) and
real aircraft (500°/s). The NN trained where full-stick is a moderate
maneuver; on real hardware it's catastrophic.

Velocity heading vs yaw heading differ by ~25° — consistent with 10kt
crosswind effect, not an AHRS error. Initial attitudes are sane (±14° roll,
+13° pitch) and match expected cruise at the time of engagement.

**Hypothesis**: AHRS is trustworthy. The rendered trajectory IS what happened.
The problem is purely the sim-to-real gain mismatch driving immediate tumble.

### Manual Characterization (t=74-135s, MANUAL mode)
See Dynamics section below.

## Flight 2 (log .02): ACRO + Manual Dynamics (225s)

### ACRO Segment (t=0-146s)
Full characterization maneuvers in ACRO mode. INAV rate PID active.

### MANUAL Segment (t=146-184s)
Same maneuvers repeated in MANUAL mode for comparison.

## Dynamics Summary

### Roll Rates (full stick, rc=±500)

| Direction | Power | Peak Rate | Speed | Mode | Notes |
|-----------|-------|-----------|-------|------|-------|
| RIGHT | 47% | +522 °/s | 10.8 | ACRO | |
| RIGHT | 51% | +564 °/s | 22.5 | ACRO | |
| RIGHT | 56% | +612 °/s | 18.5 | ACRO | |
| RIGHT | 67% | +556 °/s | 11.1 | MANUAL | |
| RIGHT | 100% | +636 °/s | 14.8 | MANUAL | |
| RIGHT | 100% | +646 °/s | 17.0 | ACRO | Fastest roll |
| LEFT | 47% | -454 °/s | 14.9 | ACRO | |
| LEFT | 51% | -366 °/s | 25.1 | ACRO | High speed |
| LEFT | 56% | -448 °/s | 12.3 | ACRO | |
| LEFT | 67% | -432 °/s | 10.6 | MANUAL | |
| LEFT | 100% | -507 °/s | 15.8 | ACRO | |
| LEFT | 100% | -554 °/s | 15.3 | MANUAL | Fastest left |

**Roll asymmetry**: Right ~550-650°/s, Left ~430-550°/s.
Prop spins CCW (viewed from front) → torque aids right roll, resists left.
Asymmetry ~17-25%, decreasing at higher power (more dynamic pressure).

**Sim comparison**: CRRCSim with Cl_da=0.12 produces ~340°/s at cruise.
Real aircraft is 1.5-1.9x faster. Need Cl_da increase or asymmetric model.

### Pitch Rates (full stick, rc=±500)

| Direction | Power | Peak Rate | Speed | Mode | Notes |
|-----------|-------|-----------|-------|------|-------|
| INSIDE (up) | 51% | -173 °/s | 13.2 | ACRO | |
| INSIDE (up) | 51% | -192 °/s | 25.1 | ACRO | Combined roll+pitch |
| INSIDE (up) | 100% | -189 °/s | 18.1 | ACRO | |
| INSIDE (up) | 100% | -218 °/s | 17.4 | MANUAL | Sustained pull |
| INSIDE (up) | 100% | -236 °/s | 25.5 | MANUAL | Best at high speed |
| OUTSIDE (dn) | 51% | +200 °/s | 7.7 | ACRO | Low speed |
| OUTSIDE (dn) | 100% | +231 °/s | 12.0 | ACRO | |
| OUTSIDE (dn) | 100% | +174 °/s | 15.3 | MANUAL | |

**Pitch observations**:
- Inside loop (nose up): 170-236°/s. Barely completes loop at ~10 m/s.
  Speed bleeds to <10 m/s at top, pitch authority drops.
- Outside loop (nose down): 170-230°/s. Fails at ~180° (inverted).
  Asymmetric wing camber: less lift inverted → less pitch authority.
- Pitch is ~3x slower than roll — limits maneuverability.

**Sim comparison**: Cm_de=-0.24 — need to verify what rate this produces.

### Throttle/Speed (wind-corrected estimates)

Wind from 330° at ~10 kts (5.1 m/s).

| Power | Groundspeed | Est. Airspeed | Heading | Notes |
|-------|-------------|--------------|---------|-------|
| 16% | 19.8 m/s | ~15.9 m/s | 114° (downwind) | Gliding/descending |
| 51% | 12.0 m/s | ~14.5 m/s | 261° (crosswind) | |
| 64% | 25.2 m/s | ~21.3 m/s | 113° (downwind) | |
| 87% | 9.5 m/s | ~11.7 m/s | 253° (crosswind) | Climbing? |

Wind makes speed comparison difficult. Downwind groundspeed inflated by ~5 m/s.
Into-wind segments needed for cleaner airspeed data.

### ACRO vs MANUAL Comparison

Roll and pitch rates are similar between ACRO and MANUAL modes. INAV's rate
PID is not significantly limiting rates — the aircraft's aerodynamic limits
dominate. This suggests the ACRO PID gains are well-matched or the rates
requested are within the PID's authority.

### New Prop (Master Airscrew GF 5.5×4)

Replacing APC 5.25×4.25. eCalc predicted equivalent performance.
Flight confirms: similar power-to-speed relationship. No obvious
differences from prior flights with APC prop.

## Issues Identified

1. **Autoc roll bang-bang limit cycle**: Xiao log confirms NN alternates
   between full left (-0.879) and full right (+0.991) roll at 3.2 reversals/s.
   Gyro rate feedback creates classic limit cycle at 10Hz NN sample rate:
   NN sees high rate → commands opposite → 100ms+30ms delay → overshoots →
   sees opposite rate → repeat. Pitch is stable (2 reversals in 8.4s).
   Throttle locked at full power.
2. **INAV servo LPF active in MANUAL mode**: `servo_lpf_hz = 20` applies
   unconditionally via `filterServos()` in servos.c — no flight mode check.
   This attenuates and delays the NN's 3.2Hz roll commands. The servo[0]
   shows only 0.1 direction reversals/s despite 3.2/s in NN commands.
   **TODO for next flight**: set `servo_lpf_hz = 0` to remove this filter
   and see the true NN→servo response. The filter may be masking or
   worsening the oscillation by adding phase lag.
3. **Roll asymmetry**: 17-25% L/R difference from prop torque (CCW prop,
   viewed from front). Right ~550-650°/s, Left ~430-550°/s. Sim model
   is symmetric — will cause tracking bias.
4. **Pitch authority**: Marginal for loops. ~200°/s both inside/outside.
   Inside loop barely completes at 10 m/s. Outside loop fails at inverted.
   Asymmetric wing camber contributes.
5. **Worn aileron hinge**: Right elevon paper hinge loosening. Affects
   roll consistency. Repair or replace before next flight.

## Data Products

- Blackbox: 2 flights, 0 Z decode errors ✓
- Both flights have GYRO_RAW + ACC + QUAT + SERVOS + RC_COMMAND + MOTORS
- Xiao logs: downloading (pending)
- GPS tracks available for trajectory analysis

## Root Cause Analysis

The NN immediately commands full-authority roll (±0.87-0.99) from the first
sample because it trained in sim where full-stick produces only ~100°/s.
On the real aircraft, full-stick produces ~500°/s. The aircraft tumbles
within 0.8s of NN taking control — not from AHRS error or oscillation,
but from a **5x roll gain mismatch** between sim and reality.

Evidence:
- Initial attitude at autoc enable is sane (roll=-14°, pitch=13°, heading
  matches velocity within 25° of wind correction)
- NN commands full roll from the very first sample (no gradual buildup)
- Gyro rates spike to 1683 deci-deg/s (168°/s) immediately, reaching
  3600 deci-deg/s (360°/s) within 1s
- Roll attitude goes -14° → +80° → +116° → +163° → -172° in ~1.5s (full tumble)
- AHRS quaternion norm stays 1.0, no discontinuities — accurate reporting

The servo LPF (20Hz) adds phase lag but is not the primary cause — the NN
is already bang-banging at full authority. The LPF masks the severity by
smoothing the servo response.

## Corrections Applied

1. **hb1_streamer.xml**: Cl_da 0.12→0.18 (roll authority 1.5x, targeting
   ~500°/s from ~106°/s). Cm_de -0.24→-0.36 (pitch authority 1.5x,
   targeting ~200°/s from ~71°/s). Both conservative given nonlinear
   amplification history.
2. **Renderer**: Fixed -x mode to parse 27-input NN log lines (was
   hardcoded to expect 29 inputs, skipping rabbit reconstruction).
3. **Retraining**: New run with updated sim dynamics. NN should learn
   proportional control when full-stick is violent in sim too.

## Next Steps

1. ~~Analyze xiao logs — verify NN gyro inputs during autoc~~ DONE
2. ~~Investigate oscillation root cause~~ DONE: 5x gain mismatch, not AHRS
3. ~~Update hb1_streamer.xml with measured rates~~ DONE: Cl_da=0.18, Cm_de=-0.36
4. Retrain with updated sim — verify convergence and proportional control
5. Set `servo_lpf_hz = 0` in INAV config for next flight
6. Repair right elevon hinge
7. Fly with retrained NN + corrected sim dynamics
8. Flight 2 (separate day): CG/trim without streamer
