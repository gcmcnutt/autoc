# Phase 0 Research: 024 — Sim/Real Fidelity

**Date**: 2026-04-19
**Feature**: [024-sim-real-fidelity](./spec.md)

## Scope

Four independent research streams to eliminate convention and cadence guesswork
before WI1–WI6 begin:

1. **INAV blackbox emission**: what each CSV field actually means, source-cited
2. **blackbox-tools decoder**: whether CSV export applies any transformation
3. **CRRCSim headless inner loop**: why sim data.dat steps 117 ms
4. **Xiao timer source**: is the 10 Hz loop drift-free or software-accumulated?

Consolidated research below. Full permanent reference in
[docs/INAV_BLACKBOX.md](../../docs/INAV_BLACKBOX.md).

## 1. INAV blackbox emission (custom fork at ~/inav)

### The custom-fork context
The flight FC runs `gcmcnutt/inav`, NOT mainline. Commit `3b9715954` (Jul 2025)
added `quaternion[0..3]` to the blackbox main frame. Mainline INAV does not log
quaternions; any analysis script expecting mainline-INAV blackbox columns will
not find them.

### Position and velocity
- **navPos[0..2]** (blackbox.c:1776): int32 centimeters, **NEU frame**
  (North-East-**Up**). Source is `navLatestActualPosition[i]`, which the
  navigation subsystem maintains in NEU per imu.c:220–226 HACK comment.
  **Must negate Z and /100 to convert to our NED meters**. This matches what
  msplink.cpp already does in `neuVectorToNedMeters`.
- **navRealVel[0..2]** (blackbox.c:1777): int16 cm/s, **NEU frame**, same
  conversion.
- **navTgtPos/navTgtVel**: same frames and units.

### Quaternion (the big one)
- **quaternion[0..3]** (blackbox.c:1733–1736): int16 each, `×10000` scaling.
  Order: `[w, x, y, z]` scalar-first.
- **Direction**: **body→earth** in NEU. Source is `orientation.q[0..3]`, used
  via `quaternionRotateVectorInv` at imu.c:218 to transform body vectors to
  earth — confirming body→earth storage direction.
- **Pitch convention**: INAV's internal attitude uses **nose-down = positive
  pitch** (non-aerospace). The attitude extraction at imu.c:567 yields
  `attitude.values.pitch` that is negative for nose up (matches the INAV
  configurator UI display).
- **Source filter**: Mahony AHRS (gyro + accel + mag fusion), NOT pure gyro
  integration. Therefore the logged quat is an estimate subject to complementary-
  filter lag / drift correction.
- **Conversion to our stack (q_EB, Hamilton, aerospace RHR, NED)**:
  1. Scale: `q_float = raw / 10000`
  2. Convert NEU→NED: this requires a composition of sign flips; **simple
     conjugate is NOT correct in general** (confirmed earlier — flight pitch
     inversion in WI5 hypothesis).
  3. Correct INAV's pitch sign to aerospace RHR.

  **Derivation is the subject of WI3 fix** once bench verification (WI5)
  characterizes the actual rotation. Until then, treat the flight pitch-axis
  anomaly as **expected** given a non-aerospace pitch in the source quat.

### Angular rate
- **gyroADC[0..2]** (blackbox.c:1674): int16 deg/s, `[roll, pitch, yaw]`
  (body X, Y, Z). Raw deg/s — NOT `× 16` (we had this hypothesized wrong).
- **Body convention**: **INAV non-aerospace**. Roll matches aerospace
  (right-roll-positive), **pitch is nose-DOWN-positive**, **yaw is
  nose-LEFT-positive** (CCW from above). Msplink already negates gyroADC[1]
  and gyroADC[2] to get aerospace RHR — this is correct.

### Accelerometer
- **accADC[0..2]** (blackbox.c:1675): int16 in units of `acc_1G / 256 ≈ 1/256 G`.
  For typical ICM-series IMUs, `acc_1G = 256` LSB/g. Body frame, `[fwd, right, down]`.
  At level rest: `accADC ≈ [0, 0, +256]` (gravity in body +Z = down). To convert:
  `accel_ms2 = accADC × (1/256) × 9.81`.

### Euler angles
- **attitude[0..2]** (blackbox.c:1729–1731): int16 decidegrees, order `[roll,
  pitch, yaw]`. Pitch column carries INAV's sign (nose-down-positive =
  negative for nose up). Derived from quaternion via imu.c:566–568.
- **Cross-check utility**: our own Euler extraction from the corrected (earth→body
  aerospace RHR) quat should match `attitude[]` after sign correction on pitch.

### Magnetometer
- **magADC[0..2]** (blackbox.c:1687): raw ADC, body frame, `[X, Y, Z]` same
  as gyro/accel body axes. Not scaled to gauss or tesla; direction vector
  only for our purposes.

### Barometric altitude
- **BaroAlt** (blackbox.c:1751): int32 cm **above ground** (calibrated at
  powerup). Positive = above.

### RC and actuators
- **rcData[0..3]**: µs PWM pulse width, `[roll, pitch, throttle, yaw]` AETR
  order, 1000–2000 µs range, 1500 µs center.
- **rcCommand[0..3]**: processed (post-expo, smoothing, deadband). `[roll,
  pitch, yaw, throttle]` — **note throttle moves to index 3** in rcCommand
  (but stays at index 2 in rcData!). Easy to get wrong.
- **servo**: µs PWM.
- **motor**: µs PWM, first motor logged with throttle-idle offset.

### Timestamp
- **time (us)** (blackbox.c:1664): system monotonic µs. Not sim-aware; just
  the FC's local clock.

### Sample rates
- Main frame (IMU / nav / RC / servo / motor) logs at the blackbox loop rate,
  default ~200 Hz (every Nth flight controller iteration).
- Slow state (flight mode flags, nav state, mspOverrideFlags) logs at ~10 Hz
  or on change.

### Key conversion gotchas
1. **navPos Z is up-positive** — flip sign when converting to NED.
2. **Quaternion is body→earth with INAV's pitch convention** — simple conjugate
   is insufficient. WI3 must characterize the correct transform.
3. **Gyro is deg/s raw**, pitch and yaw need sign flip to aerospace RHR.
4. **accADC 1G = 256 LSB** (not the `acc_1G_const` of other firmwares).
5. **rcCommand element order [roll, pitch, yaw, throttle]** differs from
   rcData [roll, pitch, throttle, yaw].

## 2. blackbox-tools CSV decoder (~/blackbox-tools)

**Passthrough decoder — zero transformations by default.** CSV contains raw
INAV internal values. All scaling is the consumer's responsibility.

- Quaternion: written as int64 from the raw int16; **`/10000` NOT applied**
  (blackbox_decode.c:921, UNIT_RAW).
- Gyro: written as raw deg/s int16 (blackbox_decode.c:910, UNIT_RAW).
  `flightlogGyroToRadiansPerSecond()` exists but only runs with `--unit-rotation`
  flag.
- Position/velocity: raw cm / cm/s (blackbox_decode.c:269–275, UNIT_RAW).
- Attitude: raw decidegrees (blackbox_decode.c:913–914, UNIT_RAW).
- Mag: raw ADC.
- **No frame rotation** unless `--simulate-imu` is passed (which produces
  separate derived columns, doesn't transform the logged ones).

**Decision**: The contract for our analysis tool is: read CSV as raw INAV
internal units, apply all conversions consistently at the read boundary using
the table in §1.

## 3. CRRCSim headless inner loop

### Architecture (from `CTime.cpp`, `SimStateHandler.cpp`, `Simulator.cpp`)
- **`gameSpeed = 25` fps** from `autoc_config.xml:3`.
- **`dt = 0.002777 seconds`** (physics substep) — crrc_main.cpp:492. Fixed at
  startup; not run-time variable.
- **`cycleLength ≈ 39.378 ms`** (one outer "frame") — CTime.cpp:71. Computed
  as `(int)(1000/gameSpeed/dt)*dt + rounding`.
- Each frame advances **`multiloop ≈ 14`** physics substeps internally.
- **`simTimeMsec = sim_steps × dt × 1000`** — SimStateHandler.cpp:390–393.
  Deterministic, step-count-based, not wall-clock.

### Why data.dat logs at 117 ms
`inputdev_autoc.cpp:341`:
```cpp
bool shouldEval = (simTimeMsec > lastUpdateTimeMsec + gEvalUpdateIntervalMsec) || ...
```
With 39 ms frames:
- After frame 1: simTimeMsec ≈ 38.9 ms (< 100, skip)
- After frame 2: simTimeMsec ≈ 77.8 ms (< 100, skip)
- After frame 3: simTimeMsec ≈ 116.6 ms (> 100, **fire**)
- Delta between fires: 3 × 39 ms = **117 ms**. Observation matches.

### The `SIM_FPS = 25.0` mystery
inputdev_autoc.h:53. Used only in overflow-bucket calculation
(inputdev_autoc.cpp:99–102). Value matches `gameSpeed` not physics rate; the
comment "~40 Hz physics tick" is **stale and incorrect** — physics is
2.777 ms substeps, not 40 Hz. **Fix as part of WI4**: replace value-vs-comment
mismatch with accurate comment citing `gameSpeed`.

### Determinism
Headless mode (`video.enabled=0`) uses fixed `cycleLength`. `multiloop` is
deterministic, `sim_steps` is deterministic, `simTimeMsec` is deterministic.
Any cadence fix MUST preserve this.

### Recommended cadence fix (WI4)
**Accumulator-based time budget** at `inputdev_autoc.cpp:341`:

```cpp
static double evalAccum = 0.0;
evalAccum += (simTimeMsec - lastUpdateTimeMsec);
bool shouldEval = (evalAccum >= gEvalUpdateIntervalMsec) || (++cycleCounter > overflowLimit);
if (shouldEval) {
  evalAccum -= gEvalUpdateIntervalMsec;  // preserve the fractional remainder
}
```

- Long-run average: exactly 100 ms.
- Per-tick: ≤ 39 ms jitter (one frame boundary). Over many ticks, averaged out.
- Fully deterministic (depends only on `simTimeMsec` and a deterministic
  accumulator state).
- Trivially extends to 50 ms (20 Hz) by changing the interval constant.

**Rejected alternatives**:
- `>` → `>=`: cosmetic; frame-boundary sampling still lands at 117 ms.
- Change physics `dt`: invasive, affects FDM stability.
- Separate timer thread: unnecessary complexity; determinism risk.

### Decision log
- **Cadence fix**: accumulator-based time budget at inputdev_autoc.cpp:341.
- **SIM_FPS comment**: correct the stale comment in same commit.
- **20 Hz readiness**: accumulator works unchanged — just change the interval
  constant in config. No structural work needed for WI13.

## 4. Xiao timer source

### Architecture (from `controller.cpp`, `msplink.cpp`, `main.cpp`)
- **No hardware timer.** The old Ticker ISR was explicitly removed
  (`controller.cpp:17`, `msplink.cpp:43, 757`).
- **Free-running Arduino `loop()` with `millis()` threshold check**
  (controller.cpp:21–40):
  ```cpp
  if (now - lastLoopTime >= MSP_LOOP_INTERVAL_MSEC) {
    lastLoopTime = now;
    if ((loopCounter % MSP_NN_EVAL_DIVISOR) == 0) { mspUpdateState(); }
    mspSetControls();
    loopCounter++;
  }
  ```
- `MSP_LOOP_INTERVAL_MSEC = 50` (main.h:24), `MSP_NN_EVAL_DIVISOR = 2` →
  NN fires every 100 ms. MSP send fires every 50 ms.
- `MSP_REPLY_TIMEOUT_MSEC = 50` (main.h:26): blocking MSP request wait.

### Observed timing on flight-20260417
From flight_log samples:
- Median interval: 101 ms
- Range: 92–122 ms
- No long-run drift observed over ~300 s flight
- Jitter source: millis() 1 ms granularity + MSP latency + flash-log I/O

### Blocking call profile (typical 100 ms tick)
- MSP fetch: 20–30 ms (blocking UART round-trip to INAV)
- NN forward pass: 1–2 ms
- MSP send: ~1 ms
- Flash log write: usually < 5 ms
- **Total worked: ~25–35 ms out of 100 ms budget.**

### 20 Hz feasibility
Total work per 50 ms: ~25–35 ms. Margin is tight but feasible if MSP latency
stays predictable. **Soft-polling loop is marginal for 20 Hz** — a USB or
flash stall could blow a 50 ms deadline. For 20 Hz we likely need a hybrid
approach: hardware timer sets a flag, main loop services it.

### Decision log
- **10 Hz today**: current soft-polling is acceptable. Observed ±10–20 ms
  jitter around 100 ms target is within training-data noise tolerance.
- **`xiao_ms` vs `inav_ms` are not synchronized**. `xiao_ms` = `millis()` at
  `mspUpdateState()` call; `inav_ms` = INAV's `timestamp_us / 1000` in the
  MSP packet. Two independent clocks, one packet transit time apart. Relevant
  for WI1 cross-checks when correlating by inav_ms across sources.
- **20 Hz requires a hybrid timer** (WI13 design target).

## Consolidated Decisions Going Into Phase 1

| Area | Decision | WI |
|---|---|---|
| Blackbox CSV read semantics | Raw INAV values, all scaling at read boundary | WI1 tool |
| Quaternion convention | Recharacterize INAV→aerospace at bench (WI5), not via simple conjugate | WI3 + WI5 |
| navPos/navVel sign | NEU→NED: `/100`, negate Z | WI1 tool |
| Gyro sign | `× π/180`, negate pitch and yaw for aerospace RHR | WI1 tool |
| Accel scale | `× 1/256 × g` for m/s² | WI1 tool |
| Sim cadence fix | Accumulator-based time budget, inputdev_autoc.cpp:341 | WI4 |
| Xiao cadence at 10 Hz | No change — current soft-polling accepted | WI4 |
| Xiao cadence at 20 Hz | Hybrid hardware-timer design (future) | WI13 |
| `xiao_ms` vs `inav_ms` | Not sync'd; use `inav_ms` for cross-source joins | WI1 tool |
| SIM_FPS constant | Fix stale comment in WI4 commit | WI4 |

## Open Items

1. WI3 quaternion transform — definitive derivation deferred to WI5 compound-
   attitude bench. Plan assumes WI5 reveals the specific per-axis signs
   needed.
2. If WI5 reveals that INAV's quat is fundamentally body→earth-NEU (not just
   "body→earth with INAV pitch convention"), the transform may involve more
   than a conjugate + sign flip. WI3 prose acknowledges this possibility.
