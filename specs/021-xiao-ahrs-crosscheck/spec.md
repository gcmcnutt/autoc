# 021 — AHRS Cross-Check + NN Input Redesign

**Status**: Draft
**Priority**: P0 — blocker for flight trust and next training cycle
**Created**: 2026-03-28

## Problem

Mar 27 flight showed the NN commanding reasonable inputs (roll ±0.6, pitch ±0.7)
but the aircraft entering uncontrolled 360° rolls and tumbles. INAV's AHRS
quaternion was self-consistent (norm=1.0, no discontinuities), but we cannot
verify whether it accurately reflects physical attitude during aggressive
maneuvering.

Additionally, analysis of the NN's current 29 inputs revealed fundamental issues:
- **alpha/beta** (inputs 24-25) are computed from GPS groundspeed + AHRS quaternion.
  In flight these swing ±70° — physically meaningless. GPS is 5-10Hz, stale vs
  30Hz NN tick, and measures groundspeed not airspeed. No wind correction.
- **No rate gyros** — the NN sees quaternion snapshots 30ms apart with no
  angular rate information. It cannot damp oscillations or anticipate overshoot.
- **Absolute quaternion** — encodes heading, which is irrelevant to control.
  Path-relative sensors already capture heading error.
- **Previous commands as inputs** (26-28) — a hack for learned integration.

The NN is trying to learn both inner-loop stabilization AND outer-loop path
tracking from high-level inputs, with no rate feedback. This works in sim
where dynamics match perfectly but fails on real hardware.

## Solution

### Approach: Sim-First

All sim-side changes (CRRCSim rate PID, NN input redesign, training validation)
are implemented and proven BEFORE touching flight hardware. INAV/xiao changes
are plumbing to replicate what the sim already provides.

### Phase 1: Bench Verification + CRRCSim Rate PID (MVP)

**Bench verification:**
- Confirm gyro/accel polarity on bench hardware (no props needed)
- Verify INAV ACRO PID config matches documented values

**CRRCSim ACRO rate PID (sim-side inner loop):**
- Implement first in minisim (fast iteration, simplified physics)
- Port to CRRCSim with full LaRCSim FDM
- NN output [-1,1] → desired rate via INAV's rate curve (expo=0, linear)
- PID compares desired vs actual body rate → surface deflection
- Match INAV gains: roll FF=50 P=5 I=7, pitch FF=50 P=5 I=7
- Rate limits: roll 560°/s, pitch 400°/s, yaw 240°/s

### Phase 2: NN Input Redesign

Based on Phase 1 data, redesign the sensor inputs for the next training cycle.

**Research items:**

#### Accelerometer / gravity vector (cross-check only, NOT NN input)
- Accelerometer measures gravity + inertial acceleration — NOT pure gravity
- At aggressive attitudes (>45° bank, pullouts, inverted): centripetal forces
  dominate and the accel vector does not indicate "down"
- Quaternion retained as NN attitude input for all-attitude flight
- Accelerometer value for AHRS cross-check: compare accel direction vs
  quaternion-derived gravity to detect AHRS divergence under load
- **INAV source**: filtered accel available via MSP (extend MSP2_AUTOC_STATE
  with accel[3] in Phase 2, for cross-check logging)
- **Xiao local LSM6DS3**: independent cross-check source (Phase 1 diagnostic)

#### Proposed input set
```
Path tracking (existing, validated):
  dPhi history + forecast      (6 inputs — heading bearing to rabbit)
  dTheta history + forecast    (6 inputs — elevation bearing to rabbit)
  dist history + forecast      (6 inputs — range to rabbit)
  closing rate                 (1 input)

Attitude (keep):
  quaternion (w,x,y,z)         (4 inputs — earth-relative attitude, needed for
                                all-attitude flight: inverted, knife-edge, etc.
                                Gravity vector from accel is NOT a reliable
                                attitude proxy under centripetal loads.)

Dynamics (new):
  rate gyros p, q, r           (3 inputs — INAV filtered gyro via MSP, post-LPF.
                                Provides damping feedback the NN previously lacked.
                                Critical for ACRO mode: NN commands desired rates,
                                needs to see current rates to modulate.)
  airspeed                     (1 input — keep, GPS-derived groundspeed for now)

Removed:
  alpha/beta (2)               — computed from GPS groundspeed + quaternion,
                                measured ±70° in flight — physically meaningless
                                without pitot tube
  previous commands (3)        — unnecessary with ACRO rate feedback. The ACRO
                                PID provides the integration/damping these were
                                hacking around.

Total: 27 inputs (down from 29)
```

Note: gyro filter params (LPF cutoff, dynamic notch) need research.
Current filter config may be tuned for PID stability, not NN input
fidelity. Phase 1 characterization flight should log both GYRO_RAW
and filtered gyroADC to compare and determine if filter adjustment needed.

#### Control architecture — two-loop separation

```
Outer loop (NN, 10Hz on xiao):
  Inputs: path sensors + quaternion + rate gyros + airspeed
  Outputs: desired roll rate, desired pitch rate, throttle
  Job: intercept and tracking — "where do I need to go"

Inner loop (INAV ACRO PID, 1kHz on FC):
  Inputs: gyro rates (raw, 1kHz)
  Setpoints: NN desired rates (held between 10Hz updates)
  Outputs: servo deflections (elevon mixer)
  Job: stabilization and rate control — "hold this rate"
  At center stick (NN output = 0): rates damp to zero, aircraft
  holds current attitude vector. This is the key property — the
  NN doesn't need to actively stabilize, only steer.

Benefits:
  - Sim-to-real gap shrinks: NN only needs rate dynamics to match,
    not full aerodynamic response to servo deflection
  - Coupling handled: INAV PID at 1kHz resolves roll/pitch/yaw
    coupling that 10Hz NN cannot react to
  - Speed regime robustness: INAV PID adapts to dynamic pressure
    changes (same stick = same rate at any speed)
  - Future: NN tick rate can increase to 20-50Hz for tighter
    tracking without changing the architecture
```

#### CRRCSim / minisim presentation
- Rate gyros: available from FDM (p, q, r body rates in `v_R_omega_total`)
- LaRCSim already computes them internally, need to wire through autoc RPC to evaluator
- Quaternion: already wired (unchanged)

#### ACRO mode in sim
- CRRCSim currently applies NN output directly as surface deflection
- With ACRO, NN output is interpreted as desired angular rate via INAV's rate curve
- INAV mapping: NN [-1,1] → INAV rate curve → [-max_rate, +max_rate] deg/s
- INAV expo set to 0 (linear mapping): max_rate = rate_param * 10 + 200
- Current hb1 config: roll 560°/s, pitch 400°/s, yaw 240°/s
- CRRCSim needs matching rate PID in autoc input device:
  - NN output [-1,1] → desired rate via same linear curve
  - PID compares desired rate vs FDM body rate (p,q,r) → surface deflection
  - PID gains tuned to match INAV's measured response from Phase 1

### Phase 3: Training + Flight Validation

- Train with redesigned inputs + ACRO inner loop
- Sim ACRO PID tuned to match INAV's measured rates from Phase 1
- Validate with flight test

## Phase 1 Design Details

### Bench verification
- Ground polarity check: gyroADC[0] positive for right-wing-down, gyroADC[1]
  positive for nose-up, accSmooth[2] positive when level
- Per `docs/COORDINATE_CONVENTIONS.md` — verified against INAV source

### CRRCSim rate PID
- Implement in minisim first (fast iteration), then port to CRRCSim
- PID per axis: `output = FF*cmd + P*(cmd_rate - actual_rate) + I*integral`
- Gains from INAV config: FF=50, P=5, I=7, D=0 (roll/pitch), FF=60, P=6, I=10 (yaw)
- Rate limits: roll 560°/s, pitch 400°/s, yaw 240°/s
- Env var overrides for tuning without rebuild

### ACRO mode switch (Phase 5, xiao deployment)
- Change xiao CH6 override to select ACRO instead of MANUAL
- Check INAV config: `aux` mode assignments for ACRO vs MANUAL
- Set `rc_expo = 0` for linear rate mapping
- On bench: confirm servo response to NN commands through ACRO PID

### Pilot characterization protocol (parallel, when props arrive)
See `flight-choreography.md` for full sequence. Key maneuvers:
1. Roll steps at cruise/slow/fast speeds
2. Pitch doublets at multiple attitudes
3. Throttle steps from cruise
4. ACRO mode segments (if comfortable)

## INAV ACRO Configuration Check

Before flight, verify these INAV settings match expected behavior:
```
# Check current values:
get rate_accel_limit_roll_yaw
get rate_accel_limit_pitch
get max_rate_inclination_roll
get max_rate_inclination_pitch
get fw_p_roll
get fw_i_roll
get fw_d_roll
get fw_p_pitch
get fw_i_pitch
get fw_d_pitch
```

## Dependencies
- Phase 1: CRRCSim + autoc changes only (no INAV/xiao code changes)
- Phase 2: autoc evaluator.cc, CRRCSim autoc input device
- Phase 3: INAV fc_msp.c (gyro extension), xiao msplink.cpp (consumer + ACRO)
- Phase 4: full retraining + flight hardware deployment

## Success Criteria

### Phase 1: Bench + CRRCSim Rate PID (MVP)
1. Gyro/accel polarity verified on bench hardware
2. Rate PID working in minisim — NN converges with rate commands
3. Rate PID ported to CRRCSim — convergence comparable to minisim

### Phase 2: NN Input Redesign
4. 27-input NN (removed alpha/beta + prev cmds, added rate gyros) trains successfully
5. Rate gyros wired through CRRCSim → evaluator
6. Convergence comparable or better than BIG3 baseline

### Phase 3: Training + Hardware Deployment
7. BIG production run eval suite passes (>95% aeroStandard completion)
8. INAV MSP extended with gyro[3], xiao consumer working on bench
9. ACRO mode active on bench — NN commands rates, INAV PID stabilizes

### Phase 4: Flight Test
10. Flight test with redesigned controller — sustained path tracking >10s
11. No uncontrolled rotation

### Parallel: Characterization Flight (when props arrive)
12. Clean blackbox data (Z not corrupted)
13. Response curves collected for sim PID tuning

## Clarifications

### Session 2026-03-28
- Q: How should the NN-to-ACRO command mapping work? → A: Use INAV's rate curve with expo=0 (linear). NN [-1,1] maps through INAV's standard rate formula. CRRCSim must implement matching rate PID with same curve.
- Q: Which source for NN gravity vector input during flight? → A: REVISED — gravity vector is NOT a reliable attitude proxy under centripetal loads. Quaternion retained as NN input. Accelerometer used for AHRS cross-check only, not NN input.
- Q: Raw or filtered gyro rates for NN input? → A: Filtered (post-LPF, pre-PID). Matches what INAV's own PID acts on. Research task: document current gyro filter params and assess if they need adjustment for NN use.
- Q: Safety overrides in Phase 1 or 2? → A: Phase 2. Implement as distance-from-autoc-origin sphere check. Refine later with altitude floor, energy bounds. Foundation for future upper-level planner.
- Q: MSP2_AUTOC_STATE extension timing? → A: Wait for Phase 2. Phase 1 gets gyro/accel from blackbox, no MSP change needed.
- Note: gyroADC[0-2] (filtered) is always logged in blackbox base fields. gyroRaw[0-2] logged when GYRO_RAW enabled. Both available for Phase 1 characterization.
- Note: PEAKS_R/P/Y (dynamic notch filter frequencies, 9 fields) stay OFF — PID tuning tool, not relevant for NN or characterization.

## Out of Scope (021)
- **Xiao onboard AHRS** (LSM6DS3 + Madgwick) — deferred to 022
- **Board alignment investigation** (T501) — deferred to 022 (needs AHRS cross-check)
- **Accelerometer in MSP** — deferred (only needed for cross-check, not NN input)
- Pitot tube / airspeed sensor (would fix alpha, but hardware change)
- Magnetometer fusion (yaw drift acceptable for ~10s spans)
- Real-time AHRS switching or voting
- GPS-dependent inputs (unreliable, intermittent)
