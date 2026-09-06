# INAV signal-path audit — **ACRO** path (2026-09-05)

⚠️ **This supplements, and in places supersedes, [`inav-signal-path-audit.md`](inav-signal-path-audit.md),
which traced the MANUAL / passthrough path.** 043 moved the NN from MANUAL to ACRO, and INAV's command
chain is **mode-dependent in more places than the PID loop**. Every finding below is against
`xiao/inav-hb1.cfg` (flight article, 2026-09-04 capture) and the flown fork build `38ff0d29e`.

Verified where possible against the 2026-09-05 flight's own blackbox, not just by reading source.

---

## 1. ⭐⭐ `rc_expo = 20` applies in ACRO and NOT in MANUAL — the action space is non-linear

`src/main/fc/fc_core.c:397-399` selects the expo **by flight mode**:

```c
rcCommand[ROLL] = getAxisRcCommand(rxGetChannelValue(ROLL),
    FLIGHT_MODE(MANUAL_MODE) ? currentControlRateProfile->manual.rcExpo8    // manual_rc_expo = 0
                             : currentControlRateProfile->stabilized.rcExpo8, // rc_expo      = 20
    rcControlsConfig()->deadband);
```

Curve (`rc_curves.c:56`): `rcLookup(sd, expo) = (2500 + expo·(t²−25))·t/25`, `t = sd/100`.

⭐ **Confirmed empirically from the flight**, fitting measured `rcData → rcCommand`:

| mode | best-fit expo | mean residual (counts) |
|---|---:|---|
| **ACRO** (n=3051) | **20** | 4.4 roll / 2.8 pitch — vs 27.5 / 24.0 at expo 0 |
| **MANUAL** (n=969) | **0** | 0.6 roll / 0.9 pitch |

Attenuation vs linear:

| stick | 10% | 20% | 40% | 50% | 70% | 90% | 100% |
|---|---:|---:|---:|---:|---:|---:|---:|
| ACRO / MANUAL | 0.802 | 0.808 | 0.832 | **0.850** | 0.898 | 0.962 | 1.000 |

⛔ **The sim models `commandScale × maxRate` — LINEAR.** So for every command short of full deflection the
real aircraft commands **~80–88%** of the rate the sim commands for the same NN output. Full stick is
unaffected, which is why the T056 bench arm-C check (full pitch ⇒ ~100% elevator) passed and hid this.

⭐ **The arithmetic closes on the flight measurement**: sim rate-loop slope **0.74** × expo attenuation
**~0.82** = **0.61**, against a measured real roll slope of **0.51–0.61** (§10 of `flight-analysis.md`).
Expo plausibly accounts for most of the authority gap that was previously attributed to the airframe.

### ⭐ Recommended fix: set `rc_expo = 0` on the flight article

Expo exists so a **human thumb** gets fine resolution near centre. The NN is not a thumb — it emits a
calibrated setpoint, and a static non-linearity between its output and the commanded rate is pure
distortion of an action space it trained on as linear. Setting `rc_expo = 0`:

- makes the real aircraft match **what the genome was actually trained against**, rather than changing the
  sim to chase a knob that only exists for pilots;
- is a single CLI line, no firmware change;
- ⚠️ affects **only** ACRO — the pilot's MANUAL feel is governed by `manual_rc_expo`, already 0.

⛔ If instead expo is kept, then `rcLookup` must be implemented in `cntrl_inavfwrate` and the genome
retrained. Changing the FC is far cheaper and strictly closer to the trained policy.

## 2. `applyRateDynamics()` is ACRO-only — but INERT here

`fc_core.c:405-417` runs it in the non-MANUAL branch only. ✅ **No effect with this config**: the guard
(`rate_dynamics.c`) requires `sensitivityCenter != 100 || sensitivityEnd != 100 || weightCenter > 0 ||
weightEnd > 0`, and the profile is **100 / 100 / 0 / 0**.

⚠️ Record it as a **latent** ACRO-only divergence: if anyone ever touches those six knobs, an
un-modelled setpoint filter appears in the flown path and in no sim.

## 3. Throttle in ACRO — answering the direct question

| aspect | finding |
|---|---|
| Mode dependence | **None.** `throttleStickMixedValue()` (`rc_controls.c:134`) has no MANUAL branch. |
| Expo | **`thr_expo = 0`** ⇒ `rcLookupThrottle` reduces to a **linear** map (`rc_curves.c:51`). `thr_mid = 50`. |
| TPA | **`tpa_rate = 0`** ⇒ throttle-PID attenuation inert. |
| **Filtered?** | ⛔ **YES.** `rcInterpolationApply` (`fc_core.c:930`) runs `for (int stick = 0; stick < 4; stick++)` — **index 3 is THROTTLE** — applying a **PT3** at `rc_filter_lpf_hz = 250`. No mode gate. |
| Magnitude | PT3 @ 250 Hz ⇒ **~1.9 ms** group delay. Against a 50 ms command step this is edge-rounding, not a dynamic element. **Not modelled in sim; not worth modelling.** |

⇒ Throttle is filtered, but negligibly. It is **not** expo'd and **not** TPA'd. The 3.96 Hz throttle
chatter in the flight is **not** a filtering artifact.

## 4. ⛔ LANDMINE: never enable `rc_filter_auto` with a 20 Hz MSP override

`rc_filter_auto = OFF` today, so the fixed 250 Hz is used. If it were **ON**, the cutoff is derived from
the RC update rate (`rc_smoothing.c:120-126`): `scaleRange(autoSmoothFactor, 1, 100, nyquist,
rcUpdateFrequency/10)`. At our **20 Hz** MSP override with `rc_filter_smoothing_factor = 30`:

| setting | cutoff | PT3 group delay |
|---|---:|---:|
| `rc_filter_auto = OFF` (**current**) | 250 Hz | **1.9 ms** |
| `rc_filter_auto = ON` | **~7.7 Hz** | **~62 ms** |

⭐ 62 ms would be comparable to the *entire* 81.6 ms phase budget 043 exists to attack — silently, on all
four channels. The auto-smoothing heuristic assumes a human RX link, not a 20 Hz machine setpoint.
**Leave it OFF and note why.**

## 5. ⛔ The gyro Kalman is ON, is in BOTH paths, and is un-modelled

`setpoint_kalman_enabled = ON`, `setpoint_kalman_q = 100`. ⚠️ Despite the name it is **not** a setpoint
filter — `src/main/sensors/gyro.c:506-510` applies `gyroKalmanUpdate()` to **`gyroADCf`**, and that value
becomes `gyro.gyroADCf[axis]`, which is:

1. what the **PID rate loop** reads as its measurement, **and**
2. what `MSP2_AUTOC_STATE` ships to the xiao as the **NN's gyro input** (`fc_msp.c`).

So it sits in the control loop *and* in the policy's observation. It is an **input-adaptive IIR with
state-dependent (non-constant) group delay**, on top of the 25 Hz PT1 and the dynamic notch.

⛔ `inav-signal-path-audit.md:18` already concluded *"This was almost certainly misconfigured. It is NOT
required for flight."* — **and it is still ON.** The sim models `gyroLpfHz = 25` only: neither the Kalman
nor the dynamic notch exists in `cntrl_inavfwrate`.

⇒ **Recommend `setpoint_kalman_enabled = OFF`** for the same reason as expo: it removes an un-modelled,
non-constant-delay element from both the loop and the NN's observation, rather than requiring the sim to
replicate an adaptive filter.

## 6. Consolidated: ACRO path vs sim

| element | config | in ACRO? | in MANUAL? | modelled in sim? |
|---|---|---|---|---|
| `rc_expo` | **20** | ✅ | ❌ (uses `manual_rc_expo = 0`) | ⛔ **NO — linear assumed** |
| `applyRateDynamics` | 100/100/0/0 | ✅ (inert) | ❌ | n/a (inert) |
| `deadband` | 2 | ✅ | ✅ | ⛔ no (0.4%, negligible) |
| RC PT3 smoothing | 250 Hz, auto OFF | ✅ all 4 sticks | ✅ | ⛔ no (~1.9 ms) |
| `thr_expo` / `thr_mid` | 0 / 50 | ✅ linear | ✅ | ✅ effectively |
| `tpa_rate` | 0 | inert | inert | n/a |
| PID gains P/I/D/FF | 15/3/7/50, 15/5/5/70 | ✅ | ❌ | ✅ **exact transcription** |
| `pidSumLimit` | 500 | ✅ | ❌ | ✅ |
| `gyro_main_lpf_hz` | 25 | ✅ | ✅ | ✅ (loop side only) |
| **gyro Kalman** | **ON, q=100** | ✅ | ✅ | ⛔ **NO** |
| **dynamic gyro notch** | ON, Q250, 2D, ≥30 Hz | ✅ | ✅ | ⛔ **NO** |
| `dterm_lpf_hz` | 10, PT2 | ✅ | ❌ | ✅ |
| **`acc_lpf_hz`** (NN input) | **15, BIQUAD** | ✅ | ✅ | ⛔ **NO (~21 ms)** |
| `servo_lpf_hz` | 0 (OFF) | ✅ | ✅ | n/a |
| `servo_pwm_rate` | 50 | ✅ | ✅ | ✅ (0–20 ms latch) |
| `rate_accel_limit_roll_pitch` | 0 | inert | inert | n/a |
| `fw_reference_airspeed` | 1500 | TURN_ASSIST only — not active | — | n/a |

## 7. Recommended actions, in order

1. ⭐ **`set rc_expo = 0`** — the single largest un-modelled divergence, empirically confirmed, and the fix
   moves the *aircraft* toward the trained action space rather than the sim toward a pilot-comfort knob.
2. ⭐ **`set setpoint_kalman_enabled = OFF`** — removes an adaptive, non-constant-delay filter from both
   the rate loop and the NN's gyro observation. The 2026-era audit already recommended this.
3. **Model `acc_lpf_hz = 15` (~21 ms) and `gyro_main_lpf_hz = 25` (~6.4 ms) on the NN input path in sim** —
   these are real and stay even after (2), because they are not pilot-comfort knobs.
4. **Leave `rc_filter_auto = OFF`**, and add a comment in `inav-hb1.cfg` saying why.
5. Re-verify all of the above on the bench, then **re-fly before any retrain** — (1) and (2) change the
   plant the genome sees, so they belong in the same flight as the prefill fix.

⚠️ **(1) and (2) are FC config changes, so FR-012a discipline applies**: change identically in sim where
the sim models the term, bench-verify, and fold into the config of record before any bake.
