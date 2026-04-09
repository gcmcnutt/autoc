# Flight Report — 2026-04-07

**Location**: Sunnyvale Baylands
**Wind**: ~10 kts from 330° (empirical circle fit from groundspeed scatter: **7.6 kts from 294°**, 15.8 m/s avg airspeed)
**Aircraft**: hb1 with streamer, Master Airscrew GF 5.5×4 prop
**Firmware**: INAV MATEKF722MINI with MSP gyro extension
**NN**: gen 400 from `autoc-9223370261317929669-2026-04-07T04:40:46.138Z`, fitness -34771, topology 27→16→8→3
**Training**: 022 point-accumulation fitness, V4 conical surface (behind=7m, ahead=2m, cone=45°), streak 1-5x over 5s
**Blackbox**: 1/32 rate, 0 Z decode errors

## Summary

**Two flights, two tests each** (per xiao logs):
- Flight 1: path 0 (StraightAndLevel) → path 2 (HorizontalFigureEight)
- Flight 2: path 2 → path 0

**First real evidence of control intent** — the aircraft shows lateral (XY) motion that looks directional, unlike the March 27 bang-bang limit cycle. Roll/pitch are mostly proportional (not saturated). First confirmed inverted flight recovery observed on Flight 2 Test 2.

**But training transferred poorly** — every span descends significantly, oscillates between upright and inverted, and ultimately requires manual recovery. The Z axis is the weakest dimension. No actual path tracking was achieved.

## Autoc Spans

| Flight | Span | Path | Duration | Entry (t) | Notes |
|--------|------|------|----------|-----------|-------|
| 1 | 1 | StraightAndLevel (0) | 13.4s | 206.9s | 27.5% inverted, descent 36m |
| 1 | 2 | HorizontalFigureEight (2) | 12.9s | 240.7s | 33.1% inverted, descent 53m |
| 2 | 1 | HorizontalFigureEight (2) | 20.3s | 71.1s | Some pitch control, recovery loops |
| 2 | 2 | StraightAndLevel (0) | 15.5s | 117.4s | **Sustained ~level flight 30m below arm at t=10-15s**, first confirmed inverted control segment |

## Dynamics

### Attitude Excursions
- Roll extends full ±180° on all spans — full attitude envelope exercised
- Pitch peaks to ±85° regularly
- **27-33% of span time inverted (|roll|>90°)** — well outside training regime
- Recovery mostly accidental equilibrium, not active policy

### Control Outputs (from blackbox rcCommand)

| Span | Roll sat | Pitch sat | Throttle | Peak roll rate | Peak pitch rate |
|------|----------|-----------|----------|----------------|-----------------|
| F1-S1 | 1.4% | 0.9% | **100% full** | ±478°/s | ±251°/s |
| F1-S2 | 1.6% | 0% | **100% full** | ±391°/s | ±291°/s |
| F2-S1 | 0% | 0% | **100% full** | ±481°/s | ±272°/s |
| F2-S2 | 0% | 0% | **100% full** | ±403°/s | ±365°/s |

- **Throttle at 100% saturation across all four spans** — never modulated
- Roll and pitch are mostly proportional (low saturation)
- Control reversals 0.9-2.5/s — not bang-bang (previous flight was 3.2/s)
- Peak pitch rates (365°/s) exceed measured real-aircraft limits (~236°/s) — consistent with high-speed regime

### Velocity
- Entry groundspeeds: 17-24 m/s
- Peak groundspeeds during spans: 23-26 m/s
- Wind-corrected airspeeds: ~15-20 m/s average (groundspeed ± ~3 m/s wind component)
- **Sim training typical airspeed: 12-13 m/s** — real is ~25% faster at entry, up to 50% faster at peaks

### Altitude Descent
- Every span loses 20-55m of altitude
- Flight 2 Span 2 found an accidental steady-state at ~38m below arm point for ~5 seconds
- No active altitude maintenance policy observed

## Wind Analysis (Empirical)

Circle fit to 167 stable cruise ground-velocity samples:
- **Wind: 3.9 m/s (7.6 kts) from 294°**
- **Constant airspeed estimate: 15.8 m/s**

Takeoff/landing crab angles of +4° to +22° visible in pre-autoc cruise samples, consistent with crosswind.

Compare to sim: `autoc_config.xml` line 92 sets `<wind velocity="12" direction="330" turbulence="1" />` = 3.66 m/s from 330°. **Sim wind matches real wind within error.** Wind is NOT the transferability issue.

## Coordinate Conventions (Verified)

Audit of the sim→xiao→NN pipeline confirmed:
- NEU→NED conversion correct (Z negated in `neuVectorToNedMeters`)
- Quaternion conjugate applied at MSP boundary (`neuQuaternionToNed`)
- Virtual position = raw - origin_offset computed correctly
- Quaternion norm stable (1.0000 ± 0.0003) — AHRS healthy
- Position jumps small (max 3.5m/step during autoc) except one GPS dropout at end of Flight 2 Span 2 (24m jump)
- Velocity is groundspeed (consistent between sim `getVelRelGround()` and xiao `state.autoc_state.vel` via GPS)
- NN input[23] labeled "airspeed" is actually groundspeed — but training is on same convention, so consistent

**No coordinate bugs found in the runtime path.** Transforms are clean.

## Root Cause Assessment (REVISED after sim vs real comparison)

### **Critical Bug: dPhi/dTheta Discontinuities (40x sim rate)**

The NN bearing inputs use `atan2` for angle computation, which wraps at
±π. This causes ~2π jumps when the target crosses the axis boundary
(behind aircraft for dTheta, below for dPhi). Statistics:

```
              Sim gen400   Real flight
dPhi jumps/s    0.02          ~0.8
dTheta jumps/s  0.03          ~1.0
```

Real flight sees **40x** the rate of discontinuity events compared to
training distribution. **137 of 245 sim scenarios (56%) have ZERO bearing
jumps > π radians.** The NN has essentially no training on how to respond
to a bearing discontinuity — the closed-loop training kept sim trajectories
in smooth regions.

On real flight, once the aircraft overshoots the rabbit or goes inverted,
the atan2 arguments cross zero and dTheta/dPhi jump ~2π in one sample. NN
output becomes garbage. Cascade failure.

**Fix**: replace dPhi/dTheta with a smooth bearing representation
(direction cosines: 3-vector of target direction in body frame). No
wrapping, no singularities. Details in [023 spec](../../specs/023-ood-and-engage-fixes/spec.md).

### **Critical Bug: Stale History on Re-engage (not first engage)**

Checking all 4 autoc engages across both flights:

```
Flight 1 Engage 1 (t=205.6s): dist-9=1.3  dist0=1.3 dd/dt=+0.0   (CLEAN)
Flight 1 Engage 2 (t=239.5s): dist-9=104.9 dist0=1.3 dd/dt=+1097 *** STALE ***
Flight 2 Engage 1 (t=69.7s):  dist-9=1.3  dist0=1.3 dd/dt=+0.0   (CLEAN)
Flight 2 Engage 2 (t=116.0s): dist-9=122.6 dist0=1.3 dd/dt=+1142 *** STALE ***
```

**Pattern**: History IS reset on the **first** autoc engage per flight.
History is NOT reset on **subsequent** engages — it retains stale values
from the previous disengage.

The `now` slot is always computed fresh (1.3m, reflecting the newly-generated
path origin in front of the aircraft). But the history slots [-0.9s, -0.3s,
-0.1s] still hold distance values from whatever the aircraft-to-rabbit
geometry was when the previous autoc span ended — typically 100+m after
the aircraft flew away.

Closing rate input `dd/dt = (prev_dist - now_dist) / 0.1s` computes to
**+1100 m/s** garbage when prev=115m and now=1.3m.

**Impact**: affects 2 of 4 autoc spans in this flight. The stale history
persists for ~9 NN cycles (900ms) until the buffer cycles through. By
coincidence this 900ms matches the INAV engage delay (see below) — so
on re-engages, the aircraft gets garbage inputs throughout the critical
transition.

**Fix**: on autoc engage transition (not just first-ever), reset the
`dPhi/dTheta/dist` history buffers in xiao. Either fill with current
in-autoc values OR zero out. Both work.

### **Related: INAV Engage Delay (~750ms)**

Even with history properly initialized, INAV has a ~750ms delay between
"NN Control switch enabled" and "MANUAL mode active":

```
xiao 205.611s: Path armed, NN switch enabled (NN starts computing)
inav 206.14s:  mode='ARM|MSPRCOVERRIDE' (MSP override on, still in ACRO)
inav 206.90s:  mode='ARM|MANUAL|MSPRCOVERRIDE' (NN commands effective)
               → 750ms after NN started computing
```

During those 750ms:
- NN is computing outputs every 100ms (7-8 samples)
- History buffer is being filled with real in-flight data
- But NN outputs are NOT reaching the servos (INAV still in ACRO/stabilized)
- Aircraft follows whatever the stabilized mode + current stick inputs dictate

**On FIRST engage**: by the time MANUAL mode kicks in, history has 7-8
samples of real data (not from the reset value). That's almost a full
buffer (10 slots) of clean data. First MANUAL-mode NN output is on
solid ground.

**On RE-engage (with stale-history bug)**: the first 7-8 samples during
the delay are being pushed into the history buffer, but the slots they
replace are stale. Effective history window includes both stale (initial)
and clean (in-flight) values. Closing rate is spurious.

**Implication**: training should probably account for the INAV engage
delay. Either:
- Train with non-zero entry position (real-flight conditions where the
  NN has seen 750ms of aircraft motion before commands matter)
- OR add an explicit 750ms delay during training evaluation
- Most importantly: the entry position variation we train with doesn't
  actually match real conditions — the real aircraft has been "in autoc
  context" for 750ms when commands first take effect, with the aircraft
  having moved wherever it was heading.

### Primary: Out-of-Distribution Inputs After Failed Intercept

NN input distribution comparison (sim gen 400 vs real flight combined):

| Input | Sim median | Real median | Real OOR vs sim p1-p99 |
|-------|-----------|-------------|-------------------------|
| **dist (m)** | **4.4** | **56.5** | **67.9%** |
| **dd/dt (m/s)** | +0.4 | **-7.2** | **15.4%** |
| **dPhi (rad)** | 0.055 | 0.093 | **38.6%** |
| **dTheta (rad)** | 0.407 | 0.444 | **41.1%** |
| vel (m/s) | 14.7 | 16.8 | 14.4% |
| gyrQ (rad/s) | 1.06 | 0.94 | 10.5% |

**The real aircraft lives in a regime the NN never saw during training.** Distance
is 12x the sim median. Closing rate is NEGATIVE (diverging) vs sim's slightly-positive
(converging). 39-41% of bearing angles are outside sim's p1-p99 range. When NN inputs
go out-of-distribution, outputs become unreliable — including throttle pegging to full.

**Why this happens**:
- Sim training with `EntryPositionRadiusSigma=15` puts the aircraft within ~15m of
  the rabbit at start. Combined with the variation ramp, initial conditions are mild.
- The NN trains heavily in the 0-20m distance regime. p99 of sim distance is only 27m.
- On real flight, the aircraft enters at 17 m/s groundspeed (faster than nominal),
  overshoots the first turn, and distance grows beyond 30m within a few seconds.
- Once distance > 30m, EVERY subsequent NN call is out-of-distribution on dist,
  dd/dt, and bearing inputs.
- Output garbage includes pegged throttle (no energy management), erratic attitude
  commands, and no recovery behavior.

**Critical implication**: the NN has no "recover from far away" policy because it
never had to learn one. Training is a closed-loop system — if the initial policy
tracks well, the aircraft stays near the rabbit, and the NN never sees far-rabbit
states in training data.

### Secondary: Throttle Saturation (even in-distribution)

Even within the velocity distribution sim saw, throttle is biased high:
- Sim median throttle output: +0.976 (near full)
- Sim % > 0.9: 55.5%
- Real % > 0.9: 97-99%

The gap between 55% and 97% is the OOD effect. But even 55% at-full in sim is
concerning — the point-accumulation fitness rewards fast pursuit, and sim drag
caps speed at ~18 m/s regardless of throttle, so the NN has no reason to modulate.

### Real Aircraft Dynamics Match Sim Reasonably Well

Wind-corrected analysis shows:
- Real airspeed: ~16 m/s avg (sim median 14.7)
- Real peak: ~22-24 m/s (sim max 26.6)
- Real aircraft IS within sim velocity range

So it's not "real is too fast for training." The speed match is OK. The issue is
**geometry**: real aircraft starts further from the rabbit than training, and once
it falls behind, it can't recover.

### OLD (now-revised) Hypothesis: Throttle Exploit

The NN learned **"full throttle always"** in sim because:
1. Sim drag caps speed at ~18 m/s regardless of throttle (CD_prof=0.18 is aggressive)
2. The 022 fitness rewards path closure and streak multiplier — faster closure = more points
3. No penalty for high throttle (mean throttle was a deferred lexicase dimension)

On real hardware:
- Real Vmax at full throttle is ~25 m/s (vs sim ~18 m/s) — less drag OR more thrust
- Full throttle pushes real aircraft well past sim training regime
- Control effectiveness and damping scale with V² — dynamic pressure 2x higher than training
- NN commands roll rate that was "normal" in sim → violent response on real

### Secondary: Sim-to-Real Speed Gap

Wind-corrected airspeed comparison:
- Sim training median: ~12-13 m/s (confirmed via gen 400 data.dat analysis)
- Real average: ~16 m/s (25% faster)
- Real peak: ~22-24 m/s airspeed (50% faster than sim median, ~25% faster than sim Vmax)

Not catastrophic (old analysis thought 50% faster; wind correction reduced the gap), but still outside where the NN practiced. The control policy trained at 12-13 m/s doesn't have deep experience at 20+ m/s dynamics.

### Tertiary: Rabbit Speed Mismatch

- **Xiao rabbit**: fixed 13.0 m/s
- **Sim rabbit**: variable 10-18 m/s (nominal 13, sigma 2, clamped)
- Real aircraft airspeed: 15-24 m/s
- Real aircraft is **faster than the rabbit by 15-85%**
- Consequence: aircraft over-runs the rabbit, NN sees "rabbit behind me", pulls hard turn, repeats

In sim training, the rabbit can be as fast as 18 m/s. When combined with the full-throttle exploit, the NN learned to chase aggressively. But in sim, the aircraft speed is capped at ~18 m/s so the relative geometry is bounded. On real, aircraft can exceed rabbit by much more than trained.

### Not Root Cause

- **Wind**: sim and real have similar wind (both ~3.6-5 m/s, 294°-330° direction). Not a significant factor.
- **AHRS**: quaternion norm stable, position smooth (except one GPS dropout), transforms clean. AHRS is healthy even in dynamic regimes.
- **Coordinate bugs**: audit found none in the runtime path.
- **Control surface authority**: peak real rates (478°/s roll, 365°/s pitch) match the sim tuning. Aircraft can DO what the NN commands.

## Recommendations (REVISED — focus on OOD recovery training)

### Critical Priority

1. **Increase training distance coverage dramatically**
   - Current training has p99 distance = 27m. Real flight median distance = 56m.
   - **Increase `EntryPositionRadiusSigma` from 15m to 30-50m**
   - NN must see "far from rabbit" states during training so it develops intercept
     and recovery behaviors.
   - With the current scoring surface (behind_scale=7m, ahead_scale=2m), scores
     drop rapidly past ~15m. Evolution must learn to close distance from far
     entry points to gather points at all.
   - Pair with a longer variation ramp — give evolution time to discover intercept.

2. **Add longer paths or random path type** (6th path)
   - 5 deterministic paths give ~1200 steps total per scenario
   - Add `SimNumPathsPerGeneration=6` to include SeededRandomB — long, varied
   - Random turns break streaks, forcing recovery policy
   - More OOD coverage during training

### High Priority

3. **Add mean throttle as second lexicase dimension** (from deferred 022 clarification)
   - Sim median throttle is already near full (0.976) — the NN has a weak throttle
     modulation policy.
   - Adding `mean_throttle` to lexicase pushes evolution toward efficient energy use.
   - Required because the primary fitness has no energy/efficiency term.

4. **Rabbit speed: consider fixing to 13 m/s**
   - Current sim: variable 10-18 m/s. Real: fixed 13 m/s.
   - Introduces needless variation during debug phase.
   - **Recommended**: set `RabbitSpeedSigma=0` in sim for next run.
   - Can reintroduce variation after basic tracking works.

### Medium Priority

5. **Sim drag investigation**
   - Real Vmax ~25 m/s vs sim ~18 m/s suggests sim drag is too high OR thrust too low
   - BUT wind-corrected airspeed comparison shows real is within sim range
   - Lower priority than OOD coverage — fix the training distribution first
   - Worth re-checking after next run

6. **Training regime: larger entry speed variation**
   - Current `EntrySpeedSigma = 0.1` (10% around nominal)
   - Real flight enters at 17 m/s (30% faster than 13 nominal)
   - Increase to 0.25-0.3 so NN sees 9-17 m/s entry airspeeds

### Medium Priority

4. **Rabbit speed reconsideration**
   - Current sim: `RabbitSpeedSigma=2`, range 10-18 m/s
   - Current xiao: fixed 13 m/s
   - Options:
     - **A**: Keep sim variable but reduce sigma (e.g. 1.0 m/s) — tighter training distribution near nominal
     - **B**: Fixed 13 m/s in both sim and xiao — eliminate variation as a training variable entirely
     - **C**: Increase sim sigma AND increase xiao rabbit speed to match real aircraft airspeed — but this requires xiao path generation changes and is least clean
   - **Recommended: B** — fix rabbit at 13 m/s in both for now. Remove one variable while we fix the bigger issues. Reintroduce variation later.

5. **Training regime: more inverted exposure**
   - Current `EntryRollSigma = 30°` (±30° roll variations)
   - Real flight showed 27-33% inverted time
   - Either increase EntryRollSigma to 60-90°, or keep at 30° and accept that training isn't designed for full-attitude
   - Full-attitude is a V2 goal, not V1

### Lower Priority

6. **Altitude-aware scoring (V2)**
   - Asymmetric cross-track: below > above penalty
   - Documented in spec but deferred

7. **Throttle penalty direct term**
   - Alternative to lexicase throttle dimension
   - Subtract `alpha * max(0, throttle - 0.5)` from step score
   - Would encourage throttle < 50% as default
   - More aggressive than lexicase — might over-constrain

### Control Rate and Authority (Future Investigation)

Once the training regime fix is in, consider:

8. **Higher NN sample rate**
   - Currently 10 Hz (100ms step). A control loop update at 10 Hz may be too slow
     for the real aircraft dynamics we're seeing — 480°/s roll rates cover 48°
     between NN updates.
   - Options: 20 Hz (50ms) or 50 Hz (20ms). Higher rate means:
     - Smaller delta between NN updates → smoother control
     - Larger training data volume → more generation time
     - Potentially more stable closed-loop response to disturbances
   - Cost: must retrain entirely (different temporal dynamics, history buffer sizing).

9. **Reduce NN command authority at the boundary**
   - Currently NN output [-1, +1] maps directly to RC [1000, 2000] full range.
   - Option: scale output by 0.5 at the xiao→INAV boundary → NN sees full range
     in training, but real aircraft only gets half authority.
   - Would prevent aggressive commands from inducing immediate tumble.
   - Simple A/B test: flash xiao with a scale factor, compare spans.
   - Does NOT require retraining — pure boundary transform.
   - Risk: if the NN's control strategy relies on full authority, reducing it
     means the aircraft can't turn fast enough to track.

10. **Authority ramp on autoc engage**
    - Start with 20% authority at engage, ramp to 100% over 3 seconds
    - Gives the NN a chance to settle before aggressive commands are allowed
    - Compensates for "hot engage" at mid-maneuver speeds
    - Simple xiao-side change, no retraining

## Data Products

- Blackbox: 2 flights, GYRO_RAW+ACC+QUAT+SERVOS+RC_COMMAND+MOTORS+NAV_POS
- Xiao logs: 2 flight logs with Nav State records (pos_raw, pos, vel, quat, gyro)
- GPS tracks: 2 GPX files
- No decode errors, no AHRS glitches (except one GPS dropout at F2S2 end)

## Next Steps

1. ~~Fix coordinate bug~~ DONE (test3 run)
2. ~~Fix V4 scoring surface (conical, ahead gradient)~~ DONE (current run)
3. Add mean throttle lexicase dimension
4. Tighten sim drag to match real Vmax ~25 m/s
5. Fix rabbit speed in sim/xiao (eliminate variable during debug phase)
6. Retrain → re-fly
7. Consider full-attitude training regime (EntryRollSigma 30→60°) once basic control is stable
