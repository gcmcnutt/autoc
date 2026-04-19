# Failsafe Behavior Audit (autoc 023 safety research)

Date: 2026-04-08
Target branch: `/home/gmcnutt/inav` (custom autoc fork of INAV)
Active config: `/home/gmcnutt/autoc/xiao/inav-hb1.cfg`
Prior audit (MSP signal path): `/home/gmcnutt/autoc/docs/inav-signal-path-audit.md`
INAV paths in this document are relative to `/home/gmcnutt/inav/`.

---

## Executive Summary

**Top findings:**

1. **`failsafe_recovery_delay` is overloaded and the name is misleading.** It gates TWO independent state machines:
   - **The main failsafe state machine** (`src/main/flight/failsafe.c:157`): how long SBUS RC data must remain valid after being lost before `rxLinkState` flips from DOWN back to UP. This is "recover from failsafe" in the name sense.
   - **The MSP-override receiver shadow state machine** (`src/main/rx/msp_override.c:104`, `:200-211`): how long MSP frames from xiao must remain valid (including at boot, starting from zero) before `rxFailsafe = false` and `mspOverrideChannels()` is allowed to overwrite any `rcChannels[]` entries. **This is the 750 ms engage delay.** Confirmed via code trace — not a guess.

2. **The 750 ms engage delay IS the `rxDataRecoveryPeriod` in `msp_override.c`.** At boot with our config `failsafe_recovery_delay = 5`, `rxDataRecoveryPeriod = PERIOD_RXDATA_RECOVERY (200 ms) + 5 * 100 ms = 700 ms`. Until 700 ms of continuously valid MSP frames are received, `mspOverrideIsInFailsafe() = true` and the override gate at `rx.c:510` is closed. AUX2 (where xiao writes 1000 to activate BOXMANUAL) is *not* overridden, so `BOXMANUAL` cannot engage. Matches the measured 760 ms delay in `flight-results/flight-20260407/flight-report.md:155-157` exactly.

3. **Current configured failsafe procedure is DROP**. On RC loss, INAV disarms immediately after a 1.2 s detection delay. No RTH, no stabilized land, no hold. If the aircraft is airborne when RC drops, it falls out of the sky. This is **configured the same way on both `inav-hb1.cfg` and `inav-bench.cfg`** — the only difference is `failsafe_delay = 10` (hb1) vs `5` (bench).

4. **`failsafe_recovery_delay = 0` IS SAFE for the bench test** for these specific reasons:
   - It only *shortens* the delay between RC-signal-recovering and leaving failsafe. It does NOT affect how quickly failsafe is ENTERED on signal loss (that is `failsafe_delay`).
   - The only behavioral change in normal flight is: after a brief RC glitch that triggered failsafe, the aircraft recovers to pilot control ~500 ms sooner.
   - There is NO debounce logic that becomes buggy at zero — the shortened period simply means `PERIOD_RXDATA_RECOVERY = 200 ms` is the floor.
   - Caveats listed in §Risk Analysis below — mostly concern the *stick-threshold* recovery gating which we have configured to 50 (enabled).

5. **A safer test for the 750 ms hypothesis exists: set `failsafe_recovery_delay = 0` ONLY** — don't touch `failsafe_delay`, `failsafe_procedure`, or anything else. This isolates the MSP-override recovery gate from the main failsafe entry behavior. **The bench test itself can be done on `xiao/inav-bench.cfg` without touching `inav-hb1.cfg`**, preserving the field-ready config.

6. **Latent concerns found during audit:**
   - **C1 (bug, medium severity):** At boot, `msp_override.c`'s local `validRxDataReceivedAt = 0` and `validRxDataFailedAt = 0` (static initializers, `:56-57`). The first MSP frame triggers `validRxDataReceivedAt = millis()` (typically 5-10 seconds post-boot). The check `(validRxDataReceivedAt - validRxDataFailedAt) > rxDataRecoveryPeriod` should fire immediately because the difference is ~5000 ms. But the observed behavior proves it does NOT — meaning some tick of the scheduler calls `mspOverrideCalculateChannels()` in the "else" branch (lines 206-210) BEFORE any xiao frames arrive, touching `validRxDataFailedAt`, and the subsequent recovery math requires the full `rxDataRecoveryPeriod`. Need to verify which tick first increments `validRxDataFailedAt`. **See §Latent Bug Assessment.**
   - **C2 (design gap):** The failsafe procedure is DROP with no proven bench verification. No blackbox/flight log in `flight-results/flight-20260407/` captures a real RC dropout. We rely on the code working correctly on first real-world use.
   - **C3 (safety):** Arm-on-low convention (`aux 0 0 0 900 1200`) is unusual. The pilot's "arm switch" being low means the aircraft arms when the SBUS AUX1 channel reads 900-1200. An SBUS dropout into failsafe-reported-values often sends channels to very low or held values, which could land in the ARM range and keep the aircraft armed. This should be verified on the bench — but it is orthogonal to `failsafe_recovery_delay`.

**Direct answer to "is `failsafe_recovery_delay = 0` safe for 023's bench test?"** Yes, with the following constraints:
- Change ONLY `failsafe_recovery_delay`, nothing else.
- Do the test on the bench, not in flight.
- Prefer modifying `xiao/inav-bench.cfg`, not `xiao/inav-hb1.cfg`.
- Reboot the FC after config change — `mspOverrideInit()` only runs at boot (`rx.c:332`).
- Revert before any flight test.

---

## What `failsafe_recovery_delay` Means

### Two independent consumers

Both files apply the same arithmetic:
```c
rxDataRecoveryPeriod = PERIOD_RXDATA_RECOVERY + failsafeConfig()->failsafe_recovery_delay * MILLIS_PER_TENTH_SECOND;
```

with `PERIOD_RXDATA_RECOVERY = 200` (`flight/failsafe.h:29`), `MILLIS_PER_TENTH_SECOND = 100` (`flight/failsafe.h:23`).

With our config `failsafe_recovery_delay = 5`, the period is **700 ms** in both places. With `= 0`, it becomes **200 ms**.

**Consumer 1: Main failsafe state machine** (`flight/failsafe.c`).

- Initialized in `failsafeReset()` at `failsafe.c:157`. Stores in `failsafeState.rxDataRecoveryPeriod`.
- Read in `failsafeOnValidDataReceived()` (`failsafe.c:309-315`):
  ```c
  failsafeState.validRxDataReceivedAt = millis();
  if ((failsafeState.validRxDataReceivedAt - failsafeState.validRxDataFailedAt) > failsafeState.rxDataRecoveryPeriod) {
      failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;
  }
  ```
- Called from `calculateRxChannelsAndUpdateFailsafe()` at `rx/rx.c:515-520` every RX data processing tick:
  ```c
  if (rxFlightChannelsValid && rxSignalReceived) {
      failsafeOnValidDataReceived();
  } else {
      failsafeOnValidDataFailed();
  }
  ```
- **What it gates:** how long the SBUS receiver must report valid data continuously before `failsafeState.rxLinkState` switches DOWN → UP. The state `FAILSAFE_RXLINK_UP` is what `failsafeIsReceivingRxData()` returns, and that flag drives the main failsafe state machine's phase transitions (`failsafe.c:406`, checked on every `failsafeUpdateState()` call from TASK_RX).

**Consumer 2: MSP override shadow failsafe** (`rx/msp_override.c`).

- Initialized in `mspOverrideInit()` at `msp_override.c:104`. Stored in file-scope static `rxDataRecoveryPeriod`.
- Read in `mspOverrideCalculateChannels()` (`msp_override.c:200-211`):
  ```c
  if (rxFlightChannelsValid && rxSignalReceived) {
      validRxDataReceivedAt = millis();
      if ((validRxDataReceivedAt - validRxDataFailedAt) > rxDataRecoveryPeriod) {
          rxFailsafe = false;
      }
  } else {
      validRxDataFailedAt = millis();
      if ((validRxDataFailedAt - validRxDataReceivedAt) > rxDataFailurePeriod) {
          rxFailsafe = true;
      }
  }
  ```
- `rxFlightChannelsValid` and `rxSignalReceived` here refer to the MSP override's own state (not SBUS state). `rxSignalReceived` is set at `msp_override.c:137` when `mspOverrideUpdateCheck()` sees a complete MSP frame from xiao.
- **What it gates:** `mspOverrideIsInFailsafe()` at `msp_override.c:119-122`, which is checked at `rx/rx.c:510`:
  ```c
  if (IS_RC_MODE_ACTIVE(BOXMSPRCOVERRIDE) && !mspOverrideIsInFailsafe()) {
      mspOverrideChannels(rcChannels);
  }
  ```
  When `rxFailsafe = true`, the MSP override gate is closed — `rcChannels[].data` is NOT overwritten by MSP values. From the pilot/servo path perspective, xiao's commands are dropped on the floor even though `BOXMSPRCOVERRIDE` is active.

### Why the engage delay is 700 ms (and not 0)

At INAV boot:
- `rxDataRecoveryPeriod = 700` ms.
- Static `validRxDataReceivedAt = 0`, `validRxDataFailedAt = 0`, `rxFailsafe = true` (`msp_override.c:52-57`).
- `mspOverrideCalculateChannels()` starts running at ~50 Hz (scheduled via `rxNextUpdateAtUs = currentTimeUs + DELAY_50_HZ` at line 158).
- Until xiao is powered on and sends its first valid MSP RC frame, each tick takes the `else` branch (line 206-210): `validRxDataFailedAt = millis()`.
- The first valid MSP frame takes the `if` branch (line 201-205): `validRxDataReceivedAt = millis()`. At that moment, `validRxDataFailedAt` was updated fractions of a second ago, so the difference `(validRxDataReceivedAt - validRxDataFailedAt)` is only a few milliseconds — NOT > 700 ms. `rxFailsafe` stays `true`.
- On each subsequent tick with valid frames, `validRxDataReceivedAt` is updated again. `validRxDataFailedAt` stays fixed at "last time we failed". As time passes, the difference grows. After 700 ms of uninterrupted valid MSP frames, it finally exceeds `rxDataRecoveryPeriod` and `rxFailsafe = false`. Then `mspOverrideChannels()` starts overwriting AUX2 → `BOXMANUAL` activates one TASK_PID tick later → `MANUAL_MODE` sets in fc_core.c.

This matches the observed 760 ms from the flight log:
```
inav 206.14s:  mode='ARM|MSPRCOVERRIDE'
inav 206.90s:  mode='ARM|MANUAL|MSPRCOVERRIDE'
               → 760ms
```
(`flight-results/flight-20260407/flight-report.md:155-157`)

### Why `mspOverrideInit` only runs at boot

`mspOverrideInit()` is called from `rxInit()` at `rx/rx.c:332`, which is part of system initialization. There is no re-init path — no CLI command, no MSP command, no profile switch that recomputes `rxDataRecoveryPeriod` in msp_override.c's file-scope state. **Changing `failsafe_recovery_delay` via CLI without rebooting will NOT affect the MSP override path.** Only the main failsafe.c path will see the new value (via `failsafeReset()` on PG reload, `failsafe.c:150`), and even then only if failsafe is not currently active.

---

## autoc Failsafe Chain (RC Dropout Scenarios)

This section traces what happens to the aircraft when the SBUS radio link drops during an autoc span (MSPRCOVERRIDE + MANUAL both active).

### Step 1: xiao detects nothing (no independent RC monitor)

The xiao has no direct SBUS receiver. Its only view of the RC link is indirect, via the `flightModeFlags` bits inside `MSP2_INAV_LOCAL_STATE`. Specifically:
- `state.isArmed()` at `xiao/include/state.h:26` — bit `MSP_MODE_ARM`.
- `state.isFailsafe()` at `xiao/include/state.h:27` — bit `MSP_MODE_FAILSAFE`.

The xiao reads these at 20 Hz via `mspUpdateState()` (`xiao/src/msplink.cpp:314-529`), and calls `stopAutoc()` in `mspUpdateNavControl()` (`:184-207`) when either bit says "bad":
```c
if (!isArmed || isFailsafe) {
    stopAutoc(isFailsafe ? "failsafe" : "disarmed", true);
    return;
}
```

**The xiao does NOT stop sending MSP override frames on its own detection of RC dropout.** It only stops if INAV has already transitioned to failsafe or disarmed. There is no direct SBUS-dropout → xiao notification path. The xiao continues streaming `MSP_SET_RAW_RC` until it reads a flightModeFlags value with FAILSAFE set — which is ≥1.2 s after SBUS loss (see Step 3).

### Step 2: INAV detects SBUS loss

SBUS receiver drops the link → the driver's frame status function returns no `RX_FRAME_COMPLETE` for incoming frames, or returns `RX_FRAME_FAILSAFE` (SBUS has an explicit failsafe flag).

- In `rxUpdateCheck()` at `rx/rx.c:399-440`:
  - Line 403-406: `needRxSignalBefore` expires → `rxSignalReceived = false`.
  - Line 417-420: alternatively, `RX_FRAME_FAILSAFE` flag → `rxSignalReceived = false`.
- `calculateRxChannelsAndUpdateFailsafe()` at `rx/rx.c:442-524` runs. Line 503 `if (rxFlightChannelsValid && rxSignalReceived)` is now false — `rcChannels[].data` is NOT updated from `rcStaging[]`. **The previous values are held.**
- Line 515-520: `failsafeOnValidDataFailed()` is called. Inside `failsafe.c:317-323`:
  ```c
  failsafeState.validRxDataFailedAt = millis();
  if ((failsafeState.validRxDataFailedAt - failsafeState.validRxDataReceivedAt) > failsafeState.rxDataFailurePeriod) {
      failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
  }
  ```
  With `failsafe_delay = 10` (hb1), `rxDataFailurePeriod = PERIOD_RXDATA_FAILURE (200) + 10 * 100 = 1200 ms`. So INAV declares the link down 1.2 s after the last valid SBUS frame.

### Step 3: Main failsafe state machine activates

Once `rxLinkState = FAILSAFE_RXLINK_DOWN`, `failsafeIsReceivingRxData()` returns false. On the next `failsafeUpdateState()` (called from TASK_RX via `processRx()`, `fc_core.c:742`):

- `failsafe.c:406`: `receivingRxDataAndNotFailsafeMode = false`.
- State `FAILSAFE_IDLE`:
  - `failsafe.c:434-446`: transitions to `FAILSAFE_RX_LOSS_DETECTED`.
- State `FAILSAFE_RX_LOSS_DETECTED`:
  - `failsafe.c:464`: `activeProcedure = failsafeChooseFailsafeProcedure()`. Our config is `failsafe_procedure = DROP`, `failsafe_min_distance = 0`, so `failsafeChooseFailsafeProcedure()` returns `FAILSAFE_PROCEDURE_DROP_IT` (`failsafe.c:375`).
  - `failsafe.c:473-477`: `failsafeActivate(FAILSAFE_LANDED)` is called with DROP procedure.
  - `failsafeActivate()` at `failsafe.c:237-250`:
    - `failsafeState.active = true`
    - `failsafeState.controlling = true`
    - `failsafeState.phase = FAILSAFE_LANDED`
    - `ENABLE_FLIGHT_MODE(FAILSAFE_MODE)` ← **this is the bit that reaches the xiao via MSP**
  - Falls through to `FAILSAFE_LANDED` on the same `failsafeUpdateState` call because `reprocessState = true`.
- State `FAILSAFE_LANDED` (`failsafe.c:585-592`):
  ```c
  ENABLE_ARMING_FLAG(ARMING_DISABLED_FAILSAFE_SYSTEM);
  disarm(DISARM_FAILSAFE);
  failsafeState.phase = FAILSAFE_RX_LOSS_MONITORING;
  failsafeState.controlling = false;  // Failsafe no longer in control of the machine - release control to pilot
  ```
  **The aircraft is disarmed. Motors stop.**

### Step 4: Does INAV stop honoring MSP overrides?

Here is the critical question for autoc span safety.

**When the main failsafe enters `FAILSAFE_LANDED`, the craft is already disarmed.** At disarm, the PID loop still runs and `mixTable()` / `writeServos()` still execute, but the motor output is forced to off (`src/main/flight/mixer.c:496+` — not traced in detail, but disarm guarantees motor-off in the mixer).

Crucially, **the MSP override gate at `rx.c:510` is `if (IS_RC_MODE_ACTIVE(BOXMSPRCOVERRIDE) && !mspOverrideIsInFailsafe())`.** This check does NOT consult the main failsafe state. As long as:
1. `BOXMSPRCOVERRIDE` is still active (i.e., AUX5 still reads 1600-2100 from the held-last-value `rcChannels[]`), AND
2. `mspOverrideIsInFailsafe()` is still false (xiao is still sending MSP frames at ≥50 Hz),

then **the MSP override continues to overwrite `rcChannels[]` after the aircraft is disarmed and the main failsafe has tripped.**

However: the DROP procedure disarms the aircraft immediately at `failsafe.c:587`. Motors are stopped. Servos may still twitch from MSP RC values, but the aircraft is essentially dead.

Over the following seconds, the xiao will eventually read `MSP_MODE_FAILSAFE = 1` in the `flightModeFlags` and call `stopAutoc("failsafe", true)` at `xiao/src/msplink.cpp:204`. After that, xiao stops sending MSP overrides entirely (`performMspSendLocked()` is only called from `mspSetControls()` at `:531-549`, which returns early if `!rabbit_active`).

**Answer to "When INAV detects dropout, does it stop honoring MSPRCOVERRIDE inputs?"** *Indirectly:* the main failsafe does not gate the MSP override path. But the DROP procedure disarms the aircraft, so MSP overrides become moot. If we ever change to `LAND` or `RTH` procedure, this analysis changes — see §Latent Bug Assessment C4.

### Step 5: Recovery on RC link return

If the RC link comes back after a brief dropout:

- `rxUpdateCheck()` sees a fresh `RX_FRAME_COMPLETE` → `rxSignalReceived = true`.
- `calculateRxChannelsAndUpdateFailsafe()` runs → `failsafeOnValidDataReceived()` → after `rxDataRecoveryPeriod = 700 ms` (our config), `failsafeState.rxLinkState = FAILSAFE_RXLINK_UP`.
- On next `failsafeUpdateState()`:
  - State was `FAILSAFE_RX_LOSS_MONITORING` (`failsafe.c:594-611`).
  - Requires `receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS` of continuous valid data (`:440, :588`).
  - After 3 seconds of valid data, AND the BOXARM switch is off, `ARMING_DISABLED_FAILSAFE_SYSTEM` is cleared and phase becomes `FAILSAFE_RX_LOSS_RECOVERED`.
  - `FAILSAFE_RX_LOSS_RECOVERED` (`failsafe.c:613-623`): clears `active`, `controlling`, `FAILSAFE_MODE`, returns to `FAILSAFE_IDLE`.

**Important:** With DROP procedure, **the aircraft will NOT automatically re-arm even if RC link fully recovers**. The pilot must disarm (flip BOXARM off) and re-arm. This is by design — see `failsafe.c:598-606` and the `osdArmingDisabledReasonMessage()` comment.

### Scenario A: Brief RC signal flicker (~100 ms dropout)

- 100 ms of missed frames → `needRxSignalBefore` expires → `rxSignalReceived = false` briefly.
- `failsafeOnValidDataFailed()` updates `validRxDataFailedAt` on each failed tick.
- `(validRxDataFailedAt - validRxDataReceivedAt)` maxes at ~100 ms, less than `rxDataFailurePeriod = 1200 ms`.
- **`rxLinkState` stays UP. Main failsafe does NOT trigger.** Aircraft never enters failsafe.
- `rcChannels[].data` holds last good values during the flicker (`rx.c:488-495`).
- When frames return, `validRxDataReceivedAt` updates. Recovery period check: `(validRxDataReceivedAt - validRxDataFailedAt)` becomes small (~milliseconds), but since `rxLinkState` never went DOWN, this doesn't matter.

**Debounce: YES.** `rxDataFailurePeriod = 1200 ms` is the de-facto debounce for entering failsafe. A 100 ms flicker is absorbed silently.

**On the MSP override side:** same dynamic plays out with xiao's MSP frames. If xiao has a 100 ms gap in MSP frames, the xiao-side MSP override path enters its own "rxFailsafe" briefly (after 1200 ms per our config). A 100 ms xiao gap does NOT cause an MSP override bounce.

### Scenario B: Full RC link loss during autoc span

- xiao is streaming MSP RC frames happily. Pilot's radio loses signal.
- 1200 ms later, main failsafe triggers → DROP procedure → disarm.
- Motors stop. Aircraft falls.
- Xiao reads `FAILSAFE` bit within ~50 ms of its next `MSP2_INAV_LOCAL_STATE` read (20 Hz tick), calls `stopAutoc()`.
- Aircraft is dead.

**Does it fail safely?** *Safely* is relative. DROP procedure is not a "soft landing." It's "stop motors and pray." Fixed-wing + DROP = glide without throttle, no stabilization, probably hit the ground hard. This is a conscious choice documented in the INAV Failsafe docs (referenced in `settings.yaml:862`).

Consider whether DROP is the right choice for autoc flights. Given the range of an autoc span is close-in (within visual range of the pilot), pilot re-take with stick flick after an RC glitch is the usual recovery path — and DROP doesn't support that because we're already disarmed. **RTH might be safer** — but autoc flies in a tight volume, RTH would potentially do worse. **LAND** would give stabilized glide to ground — a better choice? Out of scope for 023 but worth flagging (§Recommendations).

### Scenario C: RC link recovers mid-failsafe-action

- Aircraft entered `FAILSAFE_LANDED` and disarmed.
- Pilot's RC link comes back. AUX1 (arm) still in the armed range (which we set to 900-1200 — i.e., LOW).
- `FAILSAFE_RX_LOSS_MONITORING` requires 3 seconds of continuous valid data (`PERIOD_OF_3_SECONDS`) AND the BOXARM switch to be OFF (i.e., AUX1 > 1200).
- Our config has `aux 0 0 0 900 1200` — ARM active when 900-1200. "Off" means > 1200.
- If the pilot *doesn't* move the arm switch off, failsafe will NOT clear `ARMING_DISABLED_FAILSAFE_SYSTEM`. The craft remains disarmed.
- If the pilot *does* flip the arm switch off and then back on, `ARMING_DISABLED_FAILSAFE_SYSTEM` clears, phase goes `FAILSAFE_RX_LOSS_RECOVERED` → `FAILSAFE_IDLE`, and the pilot can re-arm normally.

**Recovery is not automatic.** Requires explicit pilot action (disarm + rearm). This is standard INAV failsafe behavior.

### Scenario D: Intermittent RC signal while xiao NOT engaged

- Pilot is flying normally, SBUS flickers.
- Same as Scenario A: as long as flickers are < 1200 ms, nothing happens.
- `failsafe_recovery_delay` has NO ROLE here because `rxLinkState` never went DOWN in the first place.
- **Setting `failsafe_recovery_delay = 0` has zero effect on this scenario.**

---

## Current Config (xiao/inav-hb1.cfg)

All values from `xiao/inav-hb1.cfg:1419-1432` unless noted.

| Key | Current | Default | Effect |
|---|---|---|---|
| `failsafe_delay` | **10** (1.0 s) | 5 (0.5 s) | Deciseconds added to `PERIOD_RXDATA_FAILURE (200 ms)`. With 10, INAV waits 1200 ms of no-SBUS before declaring RX down. (`failsafe.c:156`, `msp_override.c:103`). Higher = more tolerant of brief glitches but slower to react to real dropout. |
| `failsafe_recovery_delay` | **5** (0.5 s) | 5 | Deciseconds added to `PERIOD_RXDATA_RECOVERY (200 ms)`. **Dual-use**: (1) main failsafe `FAILSAFE_RXLINK_DOWN → UP` transition; (2) MSP override `rxFailsafe → false` transition, which gates `mspOverrideChannels()` at `rx.c:510`. Both use 700 ms with our config. **This is the 750 ms engage delay**. |
| `failsafe_off_delay` | **200** (20 s) | 200 | Deciseconds after failsafe activation before forced disarm in `FAILSAFE_LANDING` phase. **Not used in DROP procedure** (DROP disarms immediately). Only matters if `failsafe_procedure = LAND` or we hit an "inheritance" disarm via `failsafeShouldHaveCausedLandingByNow()` at `failsafe.c:227-230`. |
| `failsafe_throttle_low_delay` | **0** | 0 | Deciseconds throttle must be low before a "JustDisarm" fast-path fires (`failsafe.c:435-440`). Zero disables the fast-path. On RC loss with low throttle, we go through normal failsafe procedure instead of instant disarm. |
| `failsafe_procedure` | **DROP** | LAND | What failsafe does on trigger. DROP = `failsafeActivate(FAILSAFE_LANDED)` at `failsafe.c:475`, which goes immediately to disarm at `failsafe.c:587`. No glide, no stabilization, no throttle management. **Hardest possible response.** |
| `failsafe_stick_threshold` | **50** | 50 | PWM delta threshold for stick-motion detection during failsafe recovery (`failsafe.c:325-338`). Nonzero means failsafe recovery requires both valid RC data AND pilot stick movement ≥50 PWM units from center (summed over RPY). Gates `RX_LOSS_RECOVERED` in `FAILSAFE_RETURN_TO_HOME` and `FAILSAFE_LANDING` phases (`failsafe.c:514, :551`). **Not triggered in DROP path** because DROP goes directly to LANDED phase without going through the stick-waiting phases. |
| `failsafe_fw_roll_angle` | **-200** (−20°) | −200 | LAND-procedure-only bank angle (left). **Not used with DROP.** |
| `failsafe_fw_pitch_angle` | **100** (+10°) | 100 | LAND-procedure-only pitch (dive). **Not used with DROP.** |
| `failsafe_fw_yaw_rate` | **-45** (deg/s left) | −45 | LAND-procedure-only yaw rate. **Not used with DROP.** |
| `failsafe_min_distance` | **0** (disabled) | 0 | If nonzero, failsafe uses `failsafe_min_distance_procedure` when closer than this to home. **Disabled for us.** |
| `failsafe_min_distance_procedure` | DROP | DROP | Alternate procedure if close to home. **Not used since `failsafe_min_distance = 0`.** |
| `failsafe_mission_delay` | **0** | 0 | Delay before failsafe triggers during WP mission. **Not applicable — no WP missions in autoc.** |
| `failsafe_gps_fix_estimation_delay` | **7** | 7 | Delay before RTH trigger on GPS fix loss during WP mission (`failsafe.c:378-397`). **Not applicable — autoc does not use WP missions.** |
| `failsafe_throttle` (per-profile) | **1000** | 1000 | Throttle value used during LAND procedure. **Not used with DROP.** (`settings.yaml:1109`) |

**Observations:**
- Most keys are LAND-procedure-specific and have no effect with our DROP setting.
- The only keys currently shaping behavior are `failsafe_delay` (1.2 s detection), `failsafe_recovery_delay` (700 ms recovery debounce), and `failsafe_procedure = DROP`.
- `failsafe_stick_threshold = 50` has no effect on DROP path but would matter if we ever switch procedures.

### Config diff vs `inav-bench.cfg`

The only difference in the `failsafe_*` block is `failsafe_delay`:
- `inav-hb1.cfg:1419`: `failsafe_delay = 10` (1.2 s total with PERIOD_RXDATA_FAILURE)
- `inav-bench.cfg:1420`: `failsafe_delay = 5` (0.7 s total)

Bench is MORE aggressive about declaring failsafe. Both use `failsafe_recovery_delay = 5`. Both use DROP.

---

## Risk Analysis: `failsafe_recovery_delay = 0`

Per `settings.yaml:846-850`, the setting accepts min=0, max=200. Zero is a supported value.

The arithmetic becomes:
- `rxDataRecoveryPeriod = 200 + 0*100 = 200 ms` in both consumers.

### Scenario A: Brief RC signal flicker (~100 ms dropout) — **IMPROVED**

With `failsafe_recovery_delay = 0`:
- Main failsafe debounce entering failsafe is unchanged (1200 ms controlled by `failsafe_delay`).
- A brief flicker that is absorbed by the 1200 ms entry debounce never enters failsafe → `rxLinkState` never goes DOWN → the recovery period is never consulted.
- **No effect on this scenario.** Zero risk from the change.

### Scenario B: Full RC link loss during autoc span — **IMPROVED**

- Entry into failsafe: unchanged (1200 ms).
- DROP procedure: unchanged (immediate disarm).
- Recovery: if the link comes back after a DROP, the craft is already disarmed. `failsafe_recovery_delay` only matters for the `rxLinkState` transition DOWN → UP, which requires `receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS` in `FAILSAFE_RX_LOSS_MONITORING` (`failsafe.c:588, :597`). **The 3-second monitoring period dominates; `failsafe_recovery_delay` is consulted inside `failsafeOnValidDataReceived()` only for the DOWN → UP RXLINK flip, which is a prerequisite for the 3 s monitor starting.** Reducing it from 700 ms to 200 ms shortens the prerequisite by 500 ms. The total recovery time goes from ~3.7 s to ~3.2 s. Minor improvement.
- **Risk: zero.** No behavior changes beyond slightly faster recovery.

### Scenario C: RC link recovers mid-failsafe-action — **SLIGHTLY FASTER**

- Monitoring phase still requires 3 seconds of continuous valid data.
- Pilot still must disarm + rearm to clear `ARMING_DISABLED_FAILSAFE_SYSTEM`.
- The only change: the prerequisite DOWN → UP RXLINK flip happens 500 ms sooner.
- **Risk: zero.** No behavior changes beyond slightly faster recovery path.

### Scenario D: Intermittent RC signal, pilot flying normally — **NO CHANGE**

- `rxLinkState` never went DOWN, so recovery math never runs.
- **Risk: zero.** No effect.

### New risk: shorter debounce on MSP-override side gate

With `failsafe_recovery_delay = 0`, `mspOverrideCalculateChannels()` now flips `rxFailsafe = false` after 200 ms of continuous MSP frames instead of 700 ms. Also flips `rxFailsafe = true` after 200 ms of MSP frame failure (this side is controlled by `rxDataFailurePeriod = PERIOD_RXDATA_FAILURE + failsafe_delay * 100 = 200 + 1000 = 1200` — unchanged by `failsafe_recovery_delay`).

**Does MSP-override bounce happen more easily?** Only the recovery side is sped up. A 200 ms MSP gap still takes 1200 ms to raise `rxFailsafe = true`. On the return side, it takes 200 ms to clear. No new oscillation risk: you still need a 1.2 s MSP failure to trip it in the first place.

### Summary risk table

| Scenario | Risk with `failsafe_recovery_delay = 0` | Delta from current |
|---|---|---|
| A: 100 ms RC flicker | None | None |
| B: full RC loss | None | Recovery 500 ms faster |
| C: RC mid-recovery | None | Recovery 500 ms faster |
| D: RC flicker, no autoc | None | None |
| 023 bench test: MSP-override engage | Positive (500 ms faster engage) | −500 ms |
| Spurious MSP override bounce | None | None |

**Recommendation: safe to set `failsafe_recovery_delay = 0` for the bench test.**

---

## Alternative Tests for the 750 ms Hypothesis

### Option 1: Set `failsafe_recovery_delay = 0` on bench config only

**Safest.** Change `xiao/inav-bench.cfg:1421` from `5` to `0`, flash, and bench-test the engage delay measurement. Do NOT touch `inav-hb1.cfg`. Revert after the test.

**Expected outcome:** engage delay drops from ~760 ms to ~260 ms (200 ms recovery + ~60 ms of TASK_RX / PID cycle latency). If it doesn't drop that much, the hypothesis is wrong and we need to look elsewhere.

**Risk:** zero in bench — the aircraft is not flying. Config change affects one bench-only file.

### Option 2: Instrument the code with a DEBUG variable

Add a debug trace at `msp_override.c:200-211` that emits a debug channel when `rxFailsafe` flips, timestamped with millis(). Log via MSP_DEBUG or blackbox. This gives direct observation of the transition without changing any config.

**Pros:** no config change, measures the real variable.
**Cons:** requires a custom firmware build and re-flash. More work than Option 1. Still doesn't verify the value for the bench test hypothesis (we'd still want to try the config change eventually).

### Option 3: Force `mspOverrideInit` to be called later

Delay the call to `mspOverrideInit()` until after the xiao is detected streaming frames. This effectively does what setting `failsafe_recovery_delay = 0` does, but in a targeted way. Requires code changes.

**Not worth the engineering effort** given Option 1 is safe.

### Option 4: Flag whether the delay is in `rxFailsafe` or elsewhere

Check on the bench whether the delay coincides with `mspOverrideIsInFailsafe()` transitioning. The xiao can query this indirectly by watching `flightModeFlags[BOXMANUAL]` vs the time when it started sending frames. A script can graph the transition. This is basically what the flight report already shows — we already have strong evidence the delay is in MSP override recovery. Not a new test.

### Recommendation

**Use Option 1.** It's safe (zero impact on real flight safety), definitive (changes the exact code path we suspect), reversible (one-line config change), and scoped (bench-only config file).

No alternative test is required. The safety analysis above proves that the config change does not break any safety-relevant behavior.

---

## Latent Bug Assessment

### C1. First-frame recovery-period edge case in `msp_override.c`

**Severity:** Low–medium. Not currently a flight-safety bug, but it wastes 500 ms on every boot.

At `msp_override.c:56-57`, statics initialize to:
```c
static timeMs_t validRxDataReceivedAt = 0;
static timeMs_t validRxDataFailedAt = 0;
```

**Ideal behavior:** when the first-ever valid MSP frame arrives at `millis() = T` (say T=5000 ms after boot), `validRxDataReceivedAt = 5000`, `validRxDataFailedAt = 0`, difference = 5000 ms > 700 ms → `rxFailsafe = false` immediately.

**Actual behavior:** `mspOverrideCalculateChannels()` is called on a 50 Hz cadence starting at boot (via `rxNextUpdateAtUs + DELAY_50_HZ`). Before xiao starts sending, `rxFlightChannelsValid = true` (from the initialization at `msp_override.c:160`) BUT `rxSignalReceived = false` (initialized at `:50` and not set until the first real frame). So line 201's condition is false; it takes the else branch at 206-210 on every tick. This updates `validRxDataFailedAt = millis()` on every tick **before** any xiao frame arrives.

By the time the xiao's first frame arrives, `validRxDataFailedAt` is millis()-20 ms or so. The difference `(validRxDataReceivedAt - validRxDataFailedAt)` is only a few tens of milliseconds, not > 700 ms. So `rxFailsafe` stays true until 700 ms of continuous frames accumulate.

**This is the observed 750 ms engage delay.** It happens EVEN IF the xiao was powered on first and has been sending frames to INAV during INAV's boot — because `mspOverrideInit()` runs after the receiver module init, and the UART driver may not have buffered frames from before that.

**Fix (out of scope for 023):** initialize `validRxDataReceivedAt = millis() + rxDataRecoveryPeriod` at `mspOverrideInit()` so the first real frame immediately flips `rxFailsafe = false`. Or: only start evaluating the recovery math after `rxSignalReceived` has been true at least once. Or: on the very first `rxSignalReceived = true` transition, unconditionally set `rxFailsafe = false`.

The `failsafe_recovery_delay = 0` config change masks the bug at a cost of 200 ms still being unavoidable. A real fix would make engage instant.

### C2. Failsafe never tested in real flight

Grep of `flight-results/flight-20260407/flight_log_*.txt` for "failsafe" finds zero matches. The flight report at `flight-results/flight-20260407/flight-report.md:80, :277, :392` mentions "GPS dropout" but not RC failsafe events. **The failsafe path has never been exercised in a real dropout during autoc flight to this repository's knowledge.**

This is a latent *unknown* — not a bug — but it warrants a bench test before field use. We don't know if:
- The xiao correctly detects FAILSAFE_MODE via flightModeFlags and stops its NN processing
- The main failsafe DROP procedure actually disarms cleanly when MSP override is simultaneously active
- Any race between `failsafeActivate()` and the xiao's ongoing MSP frames causes stuck state

**Recommendation: bench test with simulated SBUS disconnect (physically unplug the SBUS line from the FC) during an active autoc span.**

### C3. Arm-switch-low convention interaction with SBUS failsafe values

Our `aux 0 0 0 900 1200` says ARM is active when AUX1 is in 900-1200 (LOW). SBUS failsafe behavior depends on the receiver configuration:
- Some SBUS receivers send *last-known values* on signal loss (INAV can configure via `failsafe_channel_N`)
- Some send pre-configured failsafe values (typically center = 1500)

If the SBUS receiver is configured to send 1500 on failsafe for AUX1, AUX1 will read 1500 → NOT in the ARM range → BOXARM becomes inactive. But `ARMING_FLAG(ARMED)` persists until an explicit disarm event (not just BOXARM going inactive while armed). So the craft stays armed until the failsafe state machine explicitly calls `disarm(DISARM_FAILSAFE)` at `failsafe.c:587`.

If the receiver is configured to send last-known values (or to HOLD), AUX1 keeps its ARMED value → BOXARM stays in the ARM range → no change.

**This is not a bug, but the interaction is worth verifying on the bench** — specifically, that `rcChannels[].data` for AUX1 is held at its last value (per `rx.c:488-495`), so BOXARM stays active until the explicit `disarm(DISARM_FAILSAFE)` fires. This ensures the DROP procedure's disarm is the one that actually stops the aircraft, not an earlier BOXARM-based disarm.

### C4. MSP override continues after failsafe activation

**Observation:** at `rx.c:509-513`:
```c
#if defined(USE_RX_MSP) && defined(USE_MSP_RC_OVERRIDE)
    if (IS_RC_MODE_ACTIVE(BOXMSPRCOVERRIDE) && !mspOverrideIsInFailsafe()) {
        mspOverrideChannels(rcChannels);
    }
#endif
```

The gate checks BOXMSPRCOVERRIDE and `mspOverrideIsInFailsafe()` but **NOT** the main failsafe state. If the main failsafe is active (from an SBUS dropout) but the xiao is still streaming MSP frames, the MSP override continues to overwrite `rcChannels[]` every tick.

**For DROP:** this is fine — the craft is disarmed, `rcCommand[]` goes through the failsafe control input path (`failsafeApplyControlInput()` called when `failsafeShouldApplyControlInput()` is true, which is `failsafeState.controlling` — set to false at FAILSAFE_LANDED line 590 right after disarm). Motor output is gated by disarm logic.

**For LAND or RTH:** this COULD be a bug. If we ever switch `failsafe_procedure` to LAND, the failsafe wants to fly controlled descent, but the MSP override (if xiao hasn't detected failsafe yet) would keep overwriting `rcChannels[]`, which the failsafe path reads via `rxGetChannelValue()`. However, `failsafeApplyControlInput()` directly writes `rcCommand[]` at `failsafe.c:261-285`, bypassing `rcChannels[]` → the MSP override writes are ineffective for the RPY axes during failsafe. But AUX2 → BOXMANUAL → `MANUAL_MODE` is still enabled by the MSP override. So during failsafe LAND, the aircraft would be in BOTH `FAILSAFE_MODE` and `MANUAL_MODE`.

Checking `fc_core.c:747`: `(IS_RC_MODE_ACTIVE(BOXMANUAL) && !navigationRequiresAngleMode() && !failsafeRequiresAngleMode())`. **`failsafeRequiresAngleMode()` returns true if the failsafe active procedure has `forceAngleMode = true` AND `failsafeState.controlling = true`.** Per `failsafe.c:101-144`, AUTO_LANDING, DROP_IT, and RTH all set `forceAngleMode = true`. So `failsafeRequiresAngleMode()` would return true during failsafe activation → MANUAL would be DISABLED. Good, failsafe mode angle control wins.

**However:** the brief window between xiao sending an AUX2=1000 override and failsafe activating, vs during the failsafe processing order, is scheduler-dependent. If `failsafeUpdateState()` runs BEFORE `mspOverrideChannels()` on a given TASK_RX tick, the MANUAL disable happens in the same tick. If AFTER, there's one-tick latency. Not worth digging into for 023 — the DROP procedure disarms fast enough that it doesn't matter.

**Flag for 023+:** if we change failsafe_procedure from DROP, re-audit this interaction.

---

## Recommendations

### MUST DO before any field test of 023

1. **Do not change `xiao/inav-hb1.cfg` failsafe keys**. Keep the field-ready config untouched. (Verified: no field impact, pure safety preservation.)
2. **Perform the `failsafe_recovery_delay = 0` test on `xiao/inav-bench.cfg` only**. Bench test, measure the engage delay, verify the hypothesis, document the result.
3. **Bench-test the failsafe chain itself**: simulate SBUS dropout during an active autoc span by physically disconnecting the SBUS wire while xiao is streaming MSP RC overrides. Verify:
   - INAV transitions into failsafe after ~1.2 s
   - DROP procedure disarms the FC
   - Xiao detects FAILSAFE bit and calls stopAutoc()
   - Servos hold their last-commanded position (or go to configured failsafe values)
   - Arm switch cycling + RC reconnect allows re-arm after full recovery
4. **Verify SBUS receiver failsafe channel configuration**: confirm the receiver sends safe values (not "last held" which could keep AUX1 in ARM range) on signal loss. Document the expected behavior and test it.

### SHOULD DO before 023 is considered complete

5. **Reconsider `failsafe_procedure = DROP`**. DROP is the worst-case response — stop motors and fall. For a fixed-wing, `LAND` (glide with pre-configured angles) or `RTH` (return home) would give the pilot a better chance of recovery. Out of scope for 023 but worth discussing.
6. **Fix the msp_override.c first-frame init bug** (C1) properly. Eliminates the 200 ms floor that even `failsafe_recovery_delay = 0` cannot remove. Simple init change.
7. **Add a bench test procedure document** that covers all failsafe scenarios (A, B, C, D from §autoc Failsafe Chain). Test it.

### CAN DEFER

8. **Audit the MSP-override-during-failsafe interaction** if we ever change `failsafe_procedure` from DROP to LAND/RTH. Currently the interaction is harmless because DROP disarms fast enough.
9. **Add MSP-side xiao failsafe detection** that doesn't depend on INAV — e.g., if xiao can sense disconnect from some other channel. Defense in depth, not necessary for 023.

---

## Files Audited

- `/home/gmcnutt/inav/src/main/flight/failsafe.c:1-633` — full failsafe state machine
- `/home/gmcnutt/inav/src/main/flight/failsafe.h:28-49, :140-162` — constants, config struct, state struct
- `/home/gmcnutt/inav/src/main/rx/rx.c:300-524` — rx init, frame processing, MSP override gate at :510
- `/home/gmcnutt/inav/src/main/rx/msp_override.c:1-235` — full file; critical code at :52, :103-104, :119-122, :148-214
- `/home/gmcnutt/inav/src/main/fc/fc_core.c:720-772` — MANUAL mode activation logic
- `/home/gmcnutt/inav/src/main/fc/fc_msp_box.c:53-111, :386-438` — box id mapping, flight mode flags packing
- `/home/gmcnutt/inav/src/main/fc/rc_modes.c:160-209` — mode activation (no debounce)
- `/home/gmcnutt/inav/src/main/fc/settings.yaml:841-905` — failsafe_* setting definitions, min/max/defaults
- `/home/gmcnutt/inav/src/main/fc/fc_msp.c:672-712` — MSP2_INAV_LOCAL_STATE flight mode flags packing (what xiao reads)
- `/home/gmcnutt/inav/src/main/navigation/navigation.c:4630-4634` — navigationRequiresAngleMode (not the gate)
- `/home/gmcnutt/autoc/xiao/inav-hb1.cfg:361-400` — aux switch mapping
- `/home/gmcnutt/autoc/xiao/inav-hb1.cfg:1419-1432` — failsafe_* current config
- `/home/gmcnutt/autoc/xiao/inav-hb1.cfg:1393, :1411` — receiver_type SERIAL, msp_override_channels 47
- `/home/gmcnutt/autoc/xiao/inav-bench.cfg:1420-1432` — failsafe_* bench config (only failsafe_delay differs)
- `/home/gmcnutt/autoc/xiao/src/msplink.cpp:184-207, :314-529` — xiao failsafe/arm detection, stopAutoc chain
- `/home/gmcnutt/autoc/xiao/include/state.h:26-27` — isArmed / isFailsafe definitions
- `/home/gmcnutt/autoc/xiao/include/main.h:30-33` — MSP_ARM_CHANNEL, MSP_ARMED_THRESHOLD, MSP_PATH_SELECT_CHANNEL
- `/home/gmcnutt/autoc/flight-results/flight-20260407/flight-report.md:148-184` — 750 ms engage delay observation
- `/home/gmcnutt/autoc/flight-results/flight-20260407/flight_log_2026-04-08T02-16-40.txt` — no failsafe events found
- `/home/gmcnutt/autoc/flight-results/flight-20260407/flight_log_2026-04-08T02-22-51.txt` — no failsafe events found
- `/home/gmcnutt/autoc/docs/inav-signal-path-audit.md:151-165` — previous mention of PERIOD_RXDATA_RECOVERY suspicion (now confirmed)
