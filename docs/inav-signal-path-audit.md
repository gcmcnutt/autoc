# INAV Signal Path Audit (autoc 023 research)

Date: 2026-04-08
Target branch: `/home/gmcnutt/inav` (custom autoc fork of INAV)
Active config: `/home/gmcnutt/autoc/xiao/inav-hb1.cfg`
Flight logs used: `/home/gmcnutt/autoc/flight-results/flight-20260407/flight_log_2026-04-08T02-16-40.txt`, `flight_log_2026-04-08T02-22-51.txt`

All INAV file references are relative to `/home/gmcnutt/inav/`.

---

## Executive Summary

**Top findings:**

1. **Gyro main LPF is aggressively low.** `gyro_main_lpf_hz = 25` in `inav-hb1.cfg:1330` feeds a PT1 applied every PID looptime (`src/main/sensors/gyro.c:485`). Group delay ≈ 1/(2π·25) ≈ 6.4 ms. This delay is baked into `gyro.gyroADCf`, which is what the xiao sees via `MSP2_INAV_LOCAL_STATE` (`src/main/fc/fc_msp.c:709-711`). The MSP field labeled "0.1 deg/s gyro" is NOT raw gyro — it is fully filtered, notched, and (with our config) Kalman-post-processed.

2. **"Setpoint Kalman" is actually a gyro measurement Kalman and is ON.** Despite the CLI/OSD name `setpoint_kalman_enabled` (`src/main/fc/settings.yaml:320`), the `kalmanEnabled` flag is consumed in `src/main/sensors/gyro.c:506-510` where `gyroKalmanUpdate()` is applied to `gyroADCf`. Our config sets `setpoint_kalman_enabled = ON, setpoint_kalman_q = 100` (`inav-hb1.cfg:1341-1342`). This is an input-adaptive IIR with state-dependent (non-constant) group delay on top of the 25 Hz PT1. **This was almost certainly misconfigured.** It is NOT required for flight.

3. **RC command smoothing (PT3 @ 250 Hz) is in the MANUAL-mode path.** `rc_filter_lpf_hz = 250` (`inav-hb1.cfg:1401`) is non-zero, so `rcInterpolationApply()` runs every PID tick (`src/main/fc/fc_core.c:929-931`). It applies a PT3 to `rcCommand[ROLL|PITCH|YAW|THROTTLE]` BEFORE `mixTable`/`servoMixer`. There is no MANUAL-mode gating (`src/main/fc/rc_smoothing.c:143-145`). Group delay at 250 Hz cutoff ≈ 1.9 ms — small, but the filter exists and is not modeled in CRRCSim sim. Setting to 0 would eliminate it.

4. **Servo LPF is OFF (already fixed).** `servo_lpf_hz = 0` in our config (`inav-hb1.cfg:1452`); default was 20 Hz (`src/main/fc/settings.yaml:1304`). Confirmed disabled in `src/main/flight/servos.c:232`. Good.

5. **MSP serial task is LOW priority at 100 Hz.** `TASK_SERIAL` in `src/main/fc/fc_tasks.c:488-493` — period 10 ms, priority `TASK_PRIORITY_LOW`. Under scheduler load the age-weighted priority climbs, but in the worst case a just-arrived `MSP_SET_RAW_RC` can wait up to ~10 ms for `taskHandleSerial` to drain it, then one more `TASK_RX` event-driven tick, then the next `TASK_PID` (500 µs). Measured xiao-side round-trip (fetch→eval→send) median ~23 ms, max ~42 ms (see §MSP2 Round-Trip Latency Measurement below). INAV-internal post-receive latency is an additional 0-15 ms on top of that.

**Is the delay reducible?**

- **Yes, materially:** Turning off `gyro_main_lpf_hz` (or raising to ≥100 Hz), turning off `setpoint_kalman_enabled`, and zeroing `rc_filter_lpf_hz` removes 6-10+ ms of phase delay from the NN input path and the servo-command path without requiring architectural changes. These are all config-only.
- **Partially reducible:** The 100 Hz TASK_SERIAL cadence is an architectural ceiling for xiao-to-INAV MSP. Raising TASK_SERIAL to 500-1000 Hz is possible but requires a code change and may starve TASK_RX/TASK_PID.
- **Not reducible without architecture:** The reported 750 ms "engage delay" between `MSPRCOVERRIDE` activation and `MANUAL` mode activation is **NOT located in the INAV source I audited**. See §MSP Scheduling and Latency → "750 ms engage delay" below — marked **unknown** and likely xiao-side.

**MSP2 round-trip latency bound (measured):** 15.5–42 ms (min–max), median ~23 ms. From `flight_log_2026-04-08T02-16-40.txt:544` and `:1009`, `flight_log_2026-04-08T02-22-51.txt:718` and `:1289`. These are xiao-reported values; INAV-internal post-receive latency adds 0–15 ms on top.

**Escape-hatch required?** No — not for 023. Config changes alone should remove most reducible delay. Architectural alternatives (§Escape-Hatch Architectures) are documented as 024+ candidates if post-fix measurements still show problematic latency.

---

## NN-Output → Servo Path (MANUAL mode)

Entry point: `MSP_SET_RAW_RC` received from xiao on UART20 (MSP) at 115200 baud (`inav-hb1.cfg:158`, serial function 1).

Stage-by-stage (stateful filters bolded if in MANUAL path):

1. **UART RX → serial port driver buffer.** Interrupt-driven (platform/HAL level). No rate limit. ~87 µs/byte at 115200. A 24-byte `MSP_SET_RAW_RC` payload → ~2 ms wire time.
2. **TASK_SERIAL poll (100 Hz, priority LOW).** `taskHandleSerial` at `src/main/fc/fc_tasks.c:104-125` runs `mspSerialProcess` which calls `mspSerialProcessOnePort` (`src/main/msp/msp_serial.c:463`). **It processes one MSP command per task invocation** (line 484: `break; // process one command at a time so as not to block.`). So if two MSP commands arrive back-to-back, the second waits for the next 10 ms tick — **up to 10 ms starvation under normal load**. Under high PID/gyro load, age-scaled priority climbs (see scheduler in `src/main/scheduler/scheduler.c:208-297`).
3. **`MSP_SET_RAW_RC` handler.** `src/main/fc/fc_msp.c:2068-2080`. Reads up to `MAX_SUPPORTED_RC_CHANNEL_COUNT` uint16 values into a local `frame[]`, calls `rxMspFrameReceive(frame, channelCount)` (`src/main/rx/msp.c:41-53`). This copies into `mspFrame[]` and sets `rxMspFrameDone = true`. **No rate limit on the handler side.** Synchronous; finished by the time `mspSerialProcessOnePort` returns.
4. **TASK_RX event-driven via `taskUpdateRxCheck`.** `src/main/fc/fc_tasks.c:527-533` — `checkFunc = taskUpdateRxCheck`, `staticPriority = TASK_PRIORITY_HIGH`, fallback period 10 Hz. The checkFunc calls `rxUpdateCheck` → `mspOverrideUpdateCheck` (`src/main/rx/msp_override.c:124-146`). That function calls `rxMspFrameStatus` via `rxRuntimeConfigMSP.rcFrameStatusFn`, which returns `RX_FRAME_COMPLETE` if `rxMspFrameDone` was set → `rxDataProcessingRequired = true`. When true, the scheduler elevates TASK_RX and runs it next cycle.
5. **`taskUpdateRxMain` → `processRx` → `calculateRxChannelsAndUpdateFailsafe` → `mspOverrideCalculateChannels`.** `src/main/rx/rx.c:442-524`, `src/main/rx/msp_override.c:148-214`.
   - **`rxNextUpdateAtUs = currentTimeUs + DELAY_50_HZ`** at `msp_override.c:158`. This sets the NEXT fallback tick 20 ms ahead but **does not block** event-driven updates — `rxDataProcessingRequired` is also set whenever a new frame arrives (line 135-138). So frames don't queue behind the 50 Hz timer; they drive TASK_RX directly.
   - Channels from `mspRcChannels[]` are copied into `rcStaging[]`, range-checked via `isRxPulseValid`, then `rcChannels[].data`.
   - `mspOverrideChannels(rcChannels)` (`src/main/rx/rx.c:509-513`, `src/main/rx/msp_override.c:216-223`) overwrites `rcChannels[].raw` and `.data` for every channel in `mspOverrideChannels` mask **only if** `BOXMSPRCOVERRIDE` is active and not in failsafe. `msp_override_channels = 47` (bits 0,1,2,3,5) → ROLL, PITCH, YAW, THROTTLE, AUX1.
6. **TASK_PID tick (`taskMainPidLoop`, `src/main/fc/fc_core.c:885-989`).** Runs at configured looptime. `inav-hb1.cfg:1326` sets `looptime = 500` µs (2 kHz).
7. **`processPilotAndFailSafeActions(dT)` (`src/main/fc/fc_core.c:389-436`).** In MANUAL mode:
   - `rcCommand[ROLL]   = getAxisRcCommand(rxGetChannelValue(ROLL),   manual.rcExpo8, deadband)` (line 397).
   - `rcCommand[PITCH]  = getAxisRcCommand(rxGetChannelValue(PITCH),  manual.rcExpo8, deadband)`.
   - `rcCommand[YAW]    = -getAxisRcCommand(rxGetChannelValue(YAW),   manual.rcYawExpo8, yaw_deadband)`.
   - **`getAxisRcCommand`** (`src/main/fc/fc_core.c:179-198`) applies deadband then a **static expo lookup** via `rcLookup(stickDeflection, rate)`. Nonlinear but memoryless — no phase delay, but it is a gain curve. In our config: `manual_rc_expo` = **0** in profile 0 (`inav-hb1.cfg:1920`) but **35** in profiles 1 and 2 (`:2047, :2174`). Active control-rate profile is `controlrate_profile = 0` (`:2257`) → expo = 0 → `rcLookup` becomes linear. Good. Yaw same (0 in profile 0).
   - **Manual rate scaling** (`fc_core.c:402-405`): `rcCommand[axis] = rcCommand[axis] * manual.rates[axis] / 100`. Our config: `manual_roll_rate = 100, manual_pitch_rate = 100, manual_yaw_rate = 100` → unity scaling. Good.
   - **`applyRateDynamics` is NOT called in MANUAL mode** (line 402 else branch skips it, line 406-417). Good.
   - Throttle: `rcCommand[THROTTLE] = throttleStickMixedValue()` (line 422) → `src/main/fc/rc_controls.c:134-141`: range remap + `rcLookupThrottle` (expo curve). Also memoryless.
8. **`rcInterpolationApply(isRXDataNew, currentTimeUs)`** (`src/main/fc/fc_core.c:929-931`, `src/main/fc/rc_smoothing.c:78-146`). **NOT gated by MANUAL mode — always active when `rxConfig()->rcFilterFrequency != 0`.** Our config: `rc_filter_lpf_hz = 250` (`inav-hb1.cfg:1401`). Applies a **PT3 filter** (three cascaded PT1s) at 250 Hz cutoff, running at looptime (2 kHz) to `rcCommand[0..3]` (`rc_smoothing.c:143-145`). Group delay of a PT3 at 250 Hz ≈ 3 / (2π·250) ≈ 1.9 ms. Default is `50 Hz` (would be ~9.5 ms). Setting to 0 disables entirely.
9. **`pidController(dT)` (`src/main/fc/fc_core.c:949`).** In MANUAL mode, `axisPID[]` is still computed but its output is ignored by mixer (step 10 below). I-term wind-up is reset when in MANUAL (`fc_core.c:768-771`).
10. **`mixTable()` (`src/main/fc/fc_core.c:951`, body at `src/main/flight/mixer.c:496-650`). In MANUAL mode on a fixed-wing:** `src/main/flight/mixer.c:520-525`:
    ```c
    if (STATE(FIXED_WING_LEGACY) && FLIGHT_MODE(MANUAL_MODE)) {
        input[ROLL]  = rcCommand[ROLL];
        input[PITCH] = rcCommand[PITCH];
        input[YAW]   = rcCommand[YAW];
    }
    ```
    Direct passthrough from `rcCommand` to motor mixer inputs. Throttle path: `mixerThrottleCommand = rcCommand[THROTTLE]` scaled by battery profile `throttleScale` (ours = 1.0, no-op, `inav-hb1.cfg:2258`). Battery compensation `FEATURE_THR_VBAT_COMP` — need to verify enabled; if so, another scalar multiply. No filter.
11. **`servoMixer(dT)` (`src/main/fc/fc_core.c:953-956`, body at `src/main/flight/servos.c:278-476`).**
    - In MANUAL mode (`servos.c:282-285`): `input[INPUT_STABILIZED_ROLL/PITCH/YAW] = rcCommand[ROLL/PITCH/YAW]` (direct).
    - **Servo rate limiter** at `servos.c:427`: `rateLimitFilterApply4(&servoSpeedLimitFilter[i], inputRaw, currentServoMixer[i].speed * 10, dT)`. This IS stateful — it caps slew rate. Speed is per-rule. **Our config has `smix 0 1 0 -100 0 -1`, `smix 1 1 1 100 0 -1`, etc.** (`inav-hb1.cfg:2219-2222`). The 5th column is `speed`. All four rules have `speed = 0`. Per `servos.c:421-425` comment, `0 = no limiting`. Verified no-op for our config. **Note:** this is a rate limiter — if set, it would be a per-cycle slew cap, not a filter. Group delay is unbounded in the saturating regime, zero otherwise.
    - Per-servo output scaling and midpoint addition (`servos.c:437-460`). Memoryless.
12. **`writeServos()` (`src/main/fc/fc_core.c:971-973`, body at `src/main/flight/servos.c:253-276`).**
    - Calls `filterServos()` first (`servos.c:230-251`). **Servo lowpass biquad** at `servo_lowpass_freq` Hz runs at looptime. Our config: `servo_lpf_hz = 0` → `if (servoConfig()->servo_lowpass_freq)` is false → filter skipped. Good. **Default would have been 20 Hz biquad → ~8 ms group delay. This was a previous footgun that was fixed.**
    - Then `pwmWriteServo` at servo PWM rate (`servo_pwm_rate = 50` Hz, `inav-hb1.cfg:1451`). **This introduces up to 20 ms output quantization** (new value latched into DMA buffer, emitted at next 50 Hz PWM edge). Relevant but unavoidable for standard analog servos.

**Filters in MANUAL-mode path, summary:**

| Stage | Filter | Enabled in our config? | Group delay |
|---|---|---|---|
| rcInterpolation | PT3 @ rc_filter_lpf_hz | **YES @ 250 Hz** | ~1.9 ms |
| applyRateDynamics | p/i/d mutation | not called in MANUAL | 0 |
| servoSpeedLimit | rate limiter | **per rule; all zero (off)** | 0 |
| filterServos | biquad @ servo_lpf_hz | **OFF (0)** | 0 |
| PWM output | 50 Hz cadence | n/a (physical) | up to 20 ms |

---

## Raw Gyro → MSP2_INAV_LOCAL_STATE Path

**Note:** the user's spec referred to this as `MSP2_GYRO`, but inspection shows no such MSP2 code exists in this fork. Gyro rates are piggybacked on `MSP2_INAV_LOCAL_STATE` (cmd `0x210E`, `src/main/msp/msp_protocol_v2_inav.h:38`). The handler is at `src/main/fc/fc_msp.c:672-712`. Extended field at lines 705-711:
```c
sbufWriteU16(dst, (uint16_t)(int16_t)lrintf(gyro.gyroADCf[0] * 10.0f));  // roll
sbufWriteU16(dst, (uint16_t)(int16_t)lrintf(gyro.gyroADCf[1] * 10.0f));  // pitch
sbufWriteU16(dst, (uint16_t)(int16_t)lrintf(gyro.gyroADCf[2] * 10.0f));  // yaw
```
The source is `gyro.gyroADCf[]`, populated by `gyroUpdate()` and `gyroFilter()`. The legacy `MSP_RAW_IMU` also uses `gyroRateDps(i)` which returns `lrintf(gyro.gyroADCf[axis])` (`src/main/sensors/gyro.c:595-601`). Both paths read the **fully filtered** value.

**Pipeline from sensor ADC → `gyro.gyroADCf`:**

1. **Hardware DLPF (gyro chip register).** `gyroDev[0].lpf = GYRO_LPF_256HZ` (`src/main/sensors/gyro.c:318`). MPU6000 internal low-pass at 256 Hz. Group delay ≈ 0.6 ms. Always on.
2. **Gyro sampling and DMA into driver buffer.** Runs at `TASK_GYRO_LOOPTIME` (default 125 µs = 8 kHz on STM32F7; our `looptime = 500` µs applies to PID loop, not gyro sampling). Zero-latency beyond sample period.
3. **TASK_GYRO → `gyroUpdate()` (`src/main/sensors/gyro.c:538-569`).** Runs at REALTIME priority (`fc_tasks.c:482-487`). Calls `gyroUpdateAndCalibrate` to pull raw into `gyroADCf` (unfiltered at this stage, `gyro.c:556`). Stores to `gyro.gyroRaw[]` for blackbox.
4. **Stage 1: Anti-aliasing LPF.** `gyroLpfApplyFn` at full gyro rate (`gyro.c:565`). Configured at `gyro_anti_aliasing_lpf_hz = 250` Hz (`inav-hb1.cfg:1327`) — **default is 250 Hz**. PT1 filter (`initGyroFilter` at `gyro.c:248-257`). Group delay ≈ 0.64 ms.
5. **TASK_PID → `gyroFilter()` (`src/main/fc/fc_core.c:916`, body at `src/main/sensors/gyro.c:461-536`).** Runs at PID looptime (2 kHz). This is the main filter stack.
6. **Stage 2a: RPM filter.** `rpmFilterGyroApply` (`gyro.c:471`). Our config: `rpm_gyro_filter_enabled = OFF` (`inav-hb1.cfg:1472`). **Skipped.**
7. **Stage 2b: LULU filter.** `gyroLuluApplyFn` (`gyro.c:477`). Our config: `gyro_lulu_enabled = OFF` (`inav-hb1.cfg:1328`). **Skipped** (`nullFilterApply`).
8. **Stage 3: Main LPF ("gyro LPF2").** `gyroLpf2ApplyFn` (`gyro.c:485`). Configured at **`gyro_main_lpf_hz = 25`** (`inav-hb1.cfg:1330`) with `gyro_filter_mode = STATIC` (=1, so enabled per `sensors/gyro.c:274-278`). PT1 at 25 Hz. **Group delay ≈ 1 / (2π · 25) ≈ 6.4 ms.** Default is 60 Hz (delay ≈ 2.65 ms). **This is the dominant gyro-path delay in our config.**
9. **Stage 4: Adaptive filter push (no filtering, diagnostic).** `adaptiveFilterPush(axis, gyroADCf)` at `gyro.c:488`. `gyro_filter_mode = STATIC`, not ADAPTIVE, so no adaptive update occurs in the filter itself — just the accumulator push. No delay.
10. **Stage 5: Dynamic gyro notch.** `dynamicGyroNotchFiltersApply` (`gyro.c:491-495`). Our config: `dynamic_gyro_notch_enabled = ON` (`inav-hb1.cfg:1335`), `dynamic_gyro_notch_q = 250`, `dynamic_gyro_notch_mode = 2D`. Narrow biquad notches centered on detected noise peaks. Phase delay is narrow-band (peaks near notch frequency, ~few ms, falls off). Less impact on 25 Hz signal content than the main LPF.
11. **Stage 6: Secondary dynamic gyro notch.** `secondaryDynamicGyroNotchFiltersApply` (`gyro.c:502`). **Always called** — no enable check. Another cascade of biquads.
12. **Stage 7: Gyro Kalman.** `gyroKalmanUpdate` (`gyro.c:506-510`) gated by `kalmanEnabled`. **Our config: `setpoint_kalman_enabled = ON, setpoint_kalman_q = 100`** (`inav-hb1.cfg:1341-1342`). **This is a misleading name — it IS a gyro measurement filter, not a setpoint filter.** The implementation at `src/main/flight/kalman.c:53-74` is a 1-D Kalman that treats the incoming gyro sample as a measurement, uses a process model `x += (x - lastX)` (first-order predictor), and updates an adaptive R from a sliding variance window of the input (`kalman.c:76-106`). Group delay is state-dependent (varies with observed variance). In steady-state low-noise operation it behaves like a heavy LPF. **This filter is ON top of the 25 Hz PT1 — double filtering.**
13. **Stored in `gyro.gyroADCf[axis]`.** `gyro.c:512`. This is the value the xiao reads via `MSP2_INAV_LOCAL_STATE`.

**Estimated gyro-path delay to MSP:**
- Dominant: stage 3 PT1 @ 25 Hz ≈ **6.4 ms**.
- Plus anti-alias PT1 @ 250 Hz ≈ 0.6 ms.
- Plus hardware DLPF 256 Hz ≈ 0.6 ms.
- Plus Kalman (state-dependent, **unknown but non-zero**).
- Plus notches (narrow-band, ~1-3 ms near notch center).
- **Total conservative: 8-12 ms** of phase delay on `gyro.gyroADCf` before it ever leaves INAV.
- Plus MSP serial response latency to reach xiao.

**Reducibility:** raising `gyro_main_lpf_hz` to 60 (default) cuts stage 3 to ~2.65 ms. Setting it higher (e.g., 100-150 Hz for fixed-wing) brings it to 1-1.6 ms. Disabling Kalman (`setpoint_kalman_enabled = OFF`) removes stage 7 entirely. These are config-only changes.

---

## MSP Scheduling and Latency

**Task priorities and periods** (`src/main/fc/fc_tasks.c:476-533`):

| Task | Priority | Period / trigger | File:line |
|---|---|---|---|
| TASK_GYRO | REALTIME | 125 µs (8 kHz, gyro sample) | fc_tasks.c:482-487 |
| TASK_PID | REALTIME | 500 µs (our looptime, 2 kHz) | fc_tasks.c:476-481 |
| TASK_RX | HIGH | event-driven via `taskUpdateRxCheck`; fallback 10 Hz | fc_tasks.c:527-533 |
| TASK_SERIAL | **LOW** | **100 Hz (10 ms)** | fc_tasks.c:488-493 |
| TASK_GPS | MEDIUM | 50 Hz | fc_tasks.c:535-541 |

The scheduler (`src/main/scheduler/scheduler.c:208-297`) is cooperative and runs the highest dynamic-priority task per pass. Dynamic priority = `staticPriority * taskAgeCycles`. TASK_REALTIME tasks (GYRO, PID) get forced execution if overdue beyond `desiredPeriod`. TASK_SERIAL's LOW priority means it is preempted by any higher-priority waiting task and only runs in gaps.

**MSP command processing path:**
- UART interrupt → platform serial ring buffer (no rate limit, ~2 ms wire time for 24-byte `MSP_SET_RAW_RC` at 115200).
- `taskHandleSerial` polls `serialRxBytesWaiting()` every time it runs, drains available bytes, invokes `mspSerialProcessReceivedData` per byte, and **processes at most one command per task invocation** (`src/main/msp/msp_serial.c:482-485`: `break; // process one command at a time so as not to block.`). Any second command waits until next TASK_SERIAL tick.
- Post-command sync: `waitForSerialPortToFinishTransmitting(mspPort->port)` before `mspPostProcessFn` (`msp_serial.c:488-491`). Blocks the TASK_SERIAL slot until TX drain completes.
- **No explicit queue. No explicit buffering beyond the UART driver ring buffer.** The 10 ms cadence of TASK_SERIAL is the main source of ingress jitter.

**"750 ms engage delay" — UNKNOWN source.**

The flight report (`flight-results/flight-20260407/flight-report.md:148-164`) documents that xiao enables `MSPRCOVERRIDE` at time T and INAV logs `MANUAL` mode active at T+750 ms. I searched the INAV fork for:

- `BOXMANUAL` activation delays: none found. `updateActivatedModes` (`src/main/fc/rc_modes.c:168-213`) is synchronous — a range-active AUX channel activates the box immediately.
- Mode-activation operator condition timers: none found.
- Any hold-off in `processRx` between `MSPRCOVERRIDE` enabled and `MANUAL_MODE` enabled: the check at `fc_core.c:745-755` is a direct boolean — if `IS_RC_MODE_ACTIVE(BOXMANUAL)` and not navigation/failsafe requiring angle, `ENABLE_FLIGHT_MODE(MANUAL_MODE)` immediately.
- `mspOverrideCalculateChannels` (`src/main/rx/msp_override.c:148-214`) has a data-driven path that marks `rxDataProcessingRequired = true` as soon as a frame arrives. The `rxNextUpdateAtUs = currentTimeUs + DELAY_50_HZ` at line 158 only sets the NEXT periodic fallback; it does not block event-driven updates. So MSP-driven AUX1 value changes should propagate within ~10-20 ms.
- `rxDataRecoveryPeriod` / `rxDataFailurePeriod` timers in `msp_override.c:103-104` are derived from `failsafeConfig->failsafe_recovery_delay * 100 ms` (`inav-hb1.cfg:1420: failsafe_recovery_delay = 5`) + `PERIOD_RXDATA_RECOVERY`. **This could plausibly add 500 ms+** via `rxFailsafe = false` only being cleared after `validRxDataReceivedAt - validRxDataFailedAt > rxDataRecoveryPeriod`. Need to check `PERIOD_RXDATA_RECOVERY`:

<!-- follow-up: PERIOD_RXDATA_RECOVERY / PERIOD_RXDATA_FAILURE constants and failsafe_recovery_delay arithmetic -->

`PERIOD_RXDATA_RECOVERY` is defined in `src/main/rx/rx.h` — I did not exhaustively trace it. If its value + `failsafe_recovery_delay * 100 ms` = 750 ms, that is likely the cause. **Mark as PROBABLE but unconfirmed.** A reliable test: set `failsafe_recovery_delay = 0` and remeasure the engage delay. If it drops, the recovery timer is the culprit.

**If 750 ms delay is NOT in the recovery timer:** the delay is almost certainly on the xiao side — the xiao may intentionally wait for sensor/NN warmup before driving the MANUAL aux channel. That code lives in `/home/gmcnutt/autoc/xiao/` and is out of scope for this audit.

---

## MANUAL Mode Code Walk

This traces the execution actually taken, tick-by-tick, when `FLIGHT_MODE(MANUAL_MODE)` is true on a fixed-wing.

```
Scheduler dispatches TASK_PID (src/main/scheduler/scheduler.c:272-285)
  ↓
taskMainPidLoop (src/main/fc/fc_core.c:885-989)
  ↓
gyroFilter() [src/main/fc/fc_core.c:916]
  ↳ reads gyro.gyroADCf[] (already has hardware DLPF, stage-1 anti-alias PT1)
  ↳ stage-3 PT1 @ 25 Hz  ← delay
  ↳ dynamic notch + secondary notch
  ↳ Kalman  ← delay (OUR CONFIG HAS THIS ON)
  ↳ writes gyro.gyroADCf[]
  ↓
imuUpdateAccelerometer() / imuUpdateAttitude() [fc_core.c:918-919]
  ↳ orientation quat updated; this does NOT feed the MANUAL servo path,
    but IS sent in MSP2_INAV_LOCAL_STATE.
  ↓
processPilotAndFailSafeActions(dT) [fc_core.c:925 → fc_core.c:389-436]
  ↓
  rxGetChannelValue(ROLL)  ← comes from mspOverrideChannels via rcChannels[]
  getAxisRcCommand(raw, manual.rcExpo8, deadband) [fc_core.c:179-198]
    ↳ constrain(raw - 1500, -500, 500)
    ↳ applyDeadbandRescaled(val, deadband, -500, 500)
    ↳ rcLookup(val, rate)  — static expo curve; our expo=0 → linear
  rcCommand[ROLL]  *= manual.rates[ROLL] / 100  — ours = 100 → unity
  (pitch, yaw identical; yaw gets unary minus — INAV positive-yaw-left convention)
  rcCommand[THROTTLE] = throttleStickMixedValue() [rc_controls.c:134-141]
    ↳ (raw - mincheck) rescale
    ↳ rcLookupThrottle(scaled) — static throttle expo curve
  ↓
if (rxConfig()->rcFilterFrequency) rcInterpolationApply(...) [fc_core.c:929-931]
  ↳ PT3 @ 250 Hz on rcCommand[0..3]  ← delay (~1.9 ms), NOT gated by MANUAL
  ↓
pidController(dT)  — runs but axisPID output is unused in MANUAL mixer path
  ↓
mixTable() [fc_core.c:951 → src/main/flight/mixer.c:496-650]
  ↳ if (STATE(FIXED_WING_LEGACY) && FLIGHT_MODE(MANUAL_MODE)):
      input[ROLL]  = rcCommand[ROLL]        ← DIRECT passthrough (filtered by step above)
      input[PITCH] = rcCommand[PITCH]
      input[YAW]   = rcCommand[YAW]
  ↳ mixerThrottleCommand = rcCommand[THROTTLE] * throttleScale (=1.0, no-op)
  ↳ (throttle vbat compensation gated by FEATURE_THR_VBAT_COMP — verify config)
  ↳ motor[i] = rpyMix + mixerThrottleCommand * mix rule  (our mmix is 1x single motor)
  ↓
servoMixer(dT) [fc_core.c:954 → src/main/flight/servos.c:278-476]
  ↳ if FLIGHT_MODE(MANUAL_MODE): input[INPUT_STABILIZED_*] = rcCommand[*]
  ↳ loop over servo rules:
      rateLimitFilterApply4(..., speed*10, dT)  ← speed=0 for all → passthrough
      servo[target] += inputLimited * rate/100
  ↳ per-servo scaleMin/scaleMax multiply, + middle, constrain min/max
  ↓
processServoAutotrim(dT)  — skipped unless BOXAUTOTRIM active
  ↓
writeServos() [fc_core.c:971-973 → src/main/flight/servos.c:253-276]
  ↳ filterServos():
      if servo_lowpass_freq != 0 → biquad apply  ← ours = 0, SKIPPED
      constrain(servo[i], min, max)
  ↳ pwmWriteServo(i, servo[i])  — writes to PWM DMA buffer
  ↓
HW PWM output at servo_pwm_rate = 50 Hz (inav-hb1.cfg:1451)
  ↳ quantizes outgoing command to 20 ms PWM frame cadence
```

**Memoryless-in-MANUAL transforms:**
- `getAxisRcCommand` deadband + expo (our expo = 0, linear)
- `rcLookup` / `rcLookupThrottle`
- `manual.rates[]` gain scaling (ours = 100 / 100%, unity)
- `getThrottleScale` battery profile (ours = 1.0, unity)
- smix rate, scaleMin/scaleMax, mid, min/max constraint
- Yaw unary minus in `getAxisRcCommand` (INAV convention)

**Stateful transforms in MANUAL path:**
- `rcInterpolationApply` PT3 @ 250 Hz (active, ~1.9 ms delay)
- `servoSpeedLimit` rate limit (config = 0, inactive)
- `filterServos` biquad (config = 0, inactive)

---

## MSP2 Round-Trip Latency Measurement

The xiao emits periodic "MSP pipeline" summary events to its flash log containing min/mean/max statistics over a sample window. Format:
```
MSP pipeline: samples=N fetch=min/mean/max ms eval=min/mean/max ms send=min/mean/max ms total=min/mean/max ms
```
- `fetch` = time from dispatching `MSP2_INAV_LOCAL_STATE` request to receiving the response.
- `eval` = NN inference time.
- `send` = time from dispatching `MSP_SET_RAW_RC` to send complete.
- `total` = full xiao round-trip (fetch + eval + send, or a superset).

**Measured samples from `/home/gmcnutt/autoc/flight-results/flight-20260407/`:**

| Log file | Line | samples | fetch (min/mean/max ms) | eval (min/mean/max ms) | send (min/mean/max ms) | **total (min/mean/max ms)** |
|---|---|---|---|---|---|---|
| `flight_log_2026-04-08T02-16-40.txt` | 544 | 138 | 6.9 / 12.0 / 24.0 | 1.9 / 2.2 / 6.9 | 6.6 / 9.0 / 15.0 | **15.5 / 23.3 / 35.9** |
| `flight_log_2026-04-08T02-16-40.txt` | 1009 | 133 | 6.8 / 12.3 / 20.8 | 1.8 / 2.2 / 7.0 | 6.6 / 9.0 / 16.6 | **15.9 / 23.4 / 33.9** |
| `flight_log_2026-04-08T02-22-51.txt` | 718 | 202 | 6.8 / 12.5 / 21.7 | 1.9 / 2.2 / 6.9 | 6.7 / 8.7 / 16.8 | **15.5 / 23.4 / 35.9** |
| `flight_log_2026-04-08T02-22-51.txt` | 1289 | 157 | 6.9 / 12.9 / 29.1 | 1.8 / 2.1 / 6.9 | 6.7 / 8.9 / 15.9 | **15.6 / 23.9 / 41.7** |

These are end-of-span snapshots (the xiao emits them after an autoc span completes). Four spans, two flights, all clean samples.

**Round-trip latency bound (xiao-observed, span-median):** **23-24 ms**.
**Round-trip latency max observed:** **41.7 ms**.

**What these numbers do NOT include:**
- INAV internal latency between `MSP_SET_RAW_RC` receipt and servo command taking effect: up to 10 ms TASK_SERIAL drain + one TASK_RX event-driven pass + one TASK_PID tick. Typical: 1-5 ms. Worst: ~15 ms.
- Servo PWM output quantization at 50 Hz: 0-20 ms to reach the servo wire.
- Plant response (control surface inertia, aerodynamic lag).

**Therefore end-to-end xiao-sensor-to-servo-wire latency is ~25 ms typical, ~55 ms worst-case.** This bounds the "control phase delay" that autoc 023's CRRCSim sim needs to model.

---

## Current autoc INAV Config vs Defaults

Filters and rate-limiting stages relevant to the NN signal path.

| Filter / setting | Default | autoc config (inav-hb1.cfg) | In MANUAL path? | Modeled in CRRCSim? |
|---|---|---|---|---|
| `gyro_anti_aliasing_lpf_hz` (PT1, full gyro rate) | 250 Hz | **250 Hz** (`:1327`) | yes (feeds MSP) | unknown |
| `gyro_main_lpf_hz` (PT1, looptime) | 60 Hz | **25 Hz** (`:1330`) — **MORE FILTERING** | yes (feeds MSP) | no |
| `gyro_filter_mode` | STATIC | STATIC (`:1331`) | — | n/a |
| `gyro_lulu_enabled` | OFF | OFF (`:1328`) | — | n/a |
| `dynamic_gyro_notch_enabled` | ON | ON (`:1335`) | yes | no |
| `dynamic_gyro_notch_q` | 120 | **250** (`:1336`) — narrower | yes | no |
| `dynamic_gyro_notch_mode` | 2D | 2D (`:1338`) | yes | no |
| `setpoint_kalman_enabled` (actually gyro measurement Kalman) | OFF (defined in settings.yaml:322) | **ON** (`:1341`) — **EXTRA STAGE** | yes (feeds MSP) | no |
| `setpoint_kalman_q` | varies | 100 (`:1342`) | yes | no |
| `rc_filter_lpf_hz` (PT3, looptime) | 50 Hz | **250 Hz** — LESS FILTERING than default (`:1401`) | yes | no |
| `rc_filter_auto` | OFF | OFF (`:1402`) | — | n/a |
| `rc_filter_smoothing_factor` | 30 | 30 (`:1403`) | only if auto ON | n/a |
| `servo_lpf_hz` (biquad, looptime) | 20 Hz | **0 (OFF)** — **FIXED** (`:1452`) | — | n/a |
| `servo_pwm_rate` | 50 Hz | 50 Hz (`:1451`) | yes (hw output) | partial |
| `smix` speed (rate limiter, per rule) | 0 | 0 for all rules (`:2219-2222`) | yes (no-op) | no |
| `manual_rc_expo` (profile 0, active) | 0 | 0 (`:1920`) — linear | yes | yes |
| `manual_roll_rate` / pitch / yaw | 100 | 100 / 100 / 100 (`:1922-1924`) — unity | yes | yes |
| `looptime` | varies | 500 µs (2 kHz) (`:1326`) | yes | n/a |
| `msp_override_channels` | 0 | 47 (bits 0,1,2,3,5) (`:1411`) | yes | n/a |
| `throttle_scale` (battery profile) | 1.0 | 1.0 (`:2258`) | yes (unity) | yes |
| `rate_dynamics_*` | (see below) | 100/100/10/10/0/0 (`:1926-1931`) — no-op | not in MANUAL | n/a |

**Recommended config changes for 023 (all config-only, no code changes):**

1. **`set gyro_main_lpf_hz = 60`** (restore default) or higher. Cuts stage-3 gyro delay from 6.4 ms to 2.65 ms or less. Critical.
2. **`set setpoint_kalman_enabled = OFF`**. Removes an entire adaptive Kalman stage from the gyro output. The name was misleading; it is NOT a setpoint filter. Critical.
3. **`set rc_filter_lpf_hz = 0`** (disable entirely) or leave at 250. Setting to 0 removes the PT3 from the MANUAL-mode servo command path (~1.9 ms).
4. Consider **`set dynamic_gyro_notch_enabled = OFF`** for the NN input path. Dynamic notches are primarily for multirotor vibration rejection. Our airframe is fixed-wing and our NN input is not sensitive to vibration spectra at 100 Hz+. Would also remove the state-dependent secondary dynamic notch (which is ALWAYS called regardless of enable flag).
5. Investigate **`set failsafe_recovery_delay = 0`** (from 5) to test whether it's the source of the 750 ms engage delay.

Each of these should be applied one at a time with bench measurement to confirm the hypothesis and quantify the improvement.

---

## Escape-Hatch Architectures (024+ candidates)

**NOT a 023 deliverable.** Only relevant if post-fix measurements still show problematic latency or if a future constraint requires bypassing the INAV control loop.

### Option A: Direct xiao → servo via PWM/PPM

Use the xiao's native PWM output to drive servos directly during autoc spans, bypassing INAV's flight-mode dispatch / mixer / servo-output path entirely. INAV becomes a pure sensor bus — provide attitude/position via MSP, nothing more.

**Pros:** deterministic, <1 ms latency, no MSP scheduling jitter, no INAV filters in the output path at all.
**Cons:** requires a hardware servo multiplexer (or xiao outputting alongside INAV with failover logic); safety-critical mode switching; loses INAV failsafe for autoc spans; needs new boot-time configuration so servos default to INAV on boot. Non-trivial wiring and configuration.

### Option B: Drop GPS from NN input path during autoc spans

GPS updates at 10 Hz (`gps_ublox_nav_hz = 10` in `inav-hb1.cfg:1486`) — the jitteriest and slowest input. During a short autoc span (1-3 s), GPS provides only 10-30 samples. Consider:
- Freeze GPS position at span start, dead-reckon with IMU for the span duration.
- Or feed velocity-only (no position) to the NN during spans.

**Pros:** removes a 100 ms-period jitter source from the NN input.
**Cons:** requires retraining the NN with GPS-frozen inputs; may degrade long-span tracking; conflicts with the existing "rabbit vs aircraft" geometry that depends on position.

### Option C: Higher-rate NN on xiao with IMU-only inputs, GPS as slow correction

Run the NN at 200-500 Hz using the xiao's onboard IMU (LSM6DS3) directly — bypassing the INAV gyro pipeline entirely. Fetch position/velocity via MSP at 10-50 Hz for slow correction.

**Pros:** removes the entire INAV gyro filter stack from the NN input path; enables much higher NN update rate; xiao IMU is physically on the same board so no extra bus latency.
**Cons:** needs IMU calibration on xiao; needs coordinate alignment verification; IMU bias drift over span; doubled NN training complexity (two input cadences). Relates to existing memory project `project_xiao_imu_crosscheck` — already on backlog.

---

## Files Audited

Source files read or grep'd (paths relative to `/home/gmcnutt/inav/`):

- `src/main/msp/msp_protocol_v2_inav.h` (MSP2_INAV_LOCAL_STATE = 0x210E)
- `src/main/msp/msp_serial.c` (mspSerialProcess, mspSerialProcessOnePort, 100 Hz polling, one-command-per-tick rule at :484)
- `src/main/fc/fc_msp.c` (MSP2_INAV_LOCAL_STATE handler at :672-712, MSP_SET_RAW_RC handler at :2068-2080, MSP_RAW_IMU handler at :475-489)
- `src/main/fc/fc_tasks.c` (task table at :476-533, taskHandleSerial at :104-125, priorities/periods)
- `src/main/fc/fc_core.c` (taskMainPidLoop at :885-989, processRx at :631-844, processPilotAndFailSafeActions at :389-436, getAxisRcCommand at :179-198, rcInterpolationApply dispatch at :929-931, MANUAL mode enable at :745-755)
- `src/main/fc/rc_smoothing.c` (rcInterpolationApply body at :78-146, PT3 filter stack, not gated by MANUAL)
- `src/main/fc/rc_controls.c` (throttleStickMixedValue at :134-141)
- `src/main/fc/rc_modes.c` (updateActivatedModes at :168-213, no activation delay)
- `src/main/fc/fc_msp_box.c` (BOX permanentId mapping: BOXMANUAL = 12, BOXMSPRCOVERRIDE = 50)
- `src/main/rx/rx.c` (calculateRxChannelsAndUpdateFailsafe at :442-524, taskUpdateRxCheck dispatch, rxNextUpdateAtUs timer logic)
- `src/main/rx/rx.h` (DELAY_50_HZ / DELAY_10_HZ / DELAY_5_HZ macros at :48-50, NON_AUX_CHANNEL_COUNT = 4 at :89)
- `src/main/rx/msp.c` (rxMspFrameReceive at :41-53, rxMspFrameStatus at :55-65)
- `src/main/rx/msp_override.c` (mspOverrideUpdateCheck at :124-146, mspOverrideCalculateChannels at :148-214, mspOverrideChannels at :216-223, 50 Hz fallback timer at :158, failsafe_recovery_delay arithmetic at :103-104)
- `src/main/sensors/gyro.c` (gyroInitFilters at :259-291, gyroFilter at :461-536, gyroUpdate at :538-569, gyroRateDps at :595-601, filter chain stages)
- `src/main/sensors/gyro.h` (gyroFilterMode_e enum, kalman_q/kalmanEnabled fields)
- `src/main/flight/kalman.c` (gyroKalmanUpdate at :108-112, kalman_process at :53-74, adaptive R computation)
- `src/main/flight/mixer.c` (mixTable at :496-650, MANUAL passthrough at :520-525, throttle scaling at :593-608)
- `src/main/flight/servos.c` (filterServos at :230-251, writeServos at :253-276, servoMixer at :278-476, MANUAL branch at :282-285, rateLimitFilterApply4 at :427)
- `src/main/flight/rate_dynamics.c` (applyRateDynamics at :49-84, no-op with our 100/100/0/0 config)
- `src/main/scheduler/scheduler.c` (scheduler main at :208-297, event-driven checkFunc dispatch)
- `src/main/fc/settings.yaml` (defaults: gyro_main_lpf_hz=60, rc_filter_lpf_hz=50, servo_lpf_hz=20, mspOverrideChannels=0)

Config files:
- `/home/gmcnutt/autoc/xiao/inav-hb1.cfg` (active flight config)

Flight logs:
- `/home/gmcnutt/autoc/flight-results/flight-20260407/flight_log_2026-04-08T02-16-40.txt` (lines 544, 1009)
- `/home/gmcnutt/autoc/flight-results/flight-20260407/flight_log_2026-04-08T02-22-51.txt` (lines 718, 1289)
- `/home/gmcnutt/autoc/flight-results/flight-20260407/flight-report.md` (context: 750 ms engage delay, OOD analysis)

---

## Sections marked "unknown"

1. **Root cause of the 750 ms MSPRCOVERRIDE → MANUAL transition delay.** Likely candidates:
   (a) `failsafe_recovery_delay = 5` × 100 ms + `PERIOD_RXDATA_RECOVERY` in `mspOverrideCalculateChannels` (`src/main/rx/msp_override.c:103-104, 201-211`) — PROBABLE but unconfirmed without tracing `PERIOD_RXDATA_RECOVERY`.
   (b) Xiao-side delay between enabling `MSPRCOVERRIDE` AUX channel and enabling `MANUAL` AUX channel — out of scope for this audit.
   Recommended test: set `failsafe_recovery_delay = 0` and re-measure on bench.

2. **`FEATURE_THR_VBAT_COMP` active or not.** The throttle compensation multiplier at `src/main/flight/mixer.c:605-606` is in the MANUAL path if enabled. Not audited whether our feature bitmask enables it.

3. **Gyro Kalman (setpoint_kalman) group delay.** Implementation is input-adaptive (`src/main/flight/kalman.c:76-106`) — R is updated from observed input variance. Group delay is state-dependent and cannot be characterized by a single number without either simulation or bench measurement with known sinusoidal input.

4. **Dynamic notch group delay at typical in-flight frequencies.** Depends on `dynamicGyroNotchFiltersApply` implementation details not fully traced. Narrow-band (high Q = 250), so impact on 1-10 Hz NN-relevant signal content is expected to be small (<1 ms) — but not measured.

5. **Exact INAV-internal post-receive delay from `MSP_SET_RAW_RC` to servo wire.** Bounded to 1-15 ms from task scheduling analysis, but not directly measured. Would require oscilloscope between UART RX and servo PWM output, or a build with tight logging on both ends.

6. **Whether `gyroLpfApplyFn` (stage 1 anti-alias) actually runs at 8 kHz or the 2 kHz PID rate.** Code at `src/main/sensors/gyro.c:259-262` says `initGyroFilter(..., getGyroLooptime())` — but `getGyroLooptime()` returns `gyro.targetLooptime` set from the actual gyro driver sample rate (`src/main/sensors/gyro.c:324`). On MPU6000 with `GYRO_LPF_256HZ` this should be 8 kHz. Not verified on the specific hardware in our airframe.

7. **MSP2_GYRO as a distinct message.** User spec referenced this; none exists in the fork. Gyro is piggybacked on `MSP2_INAV_LOCAL_STATE`. Confirmed by exhaustive grep for `MSP2_GYRO` — zero matches.
