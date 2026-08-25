# Contract — the modelled INAV fixed-wing rate loop

**Consumer**: `crrcsim/src/mod_cntrl/cntrl_inavfwrate/` · **Source of truth**: `~/inav`
`src/main/flight/pid.c` on the config of record `xiao/inav-hb1.cfg` (`platform_type = AIRPLANE`).
**Derivation**: [../research.md](../research.md) R1.

⛔ **This is ACRO — rate control.** No attitude term appears anywhere in this contract. An implementation
that reads attitude has built ANGLE (spec FR-019a).

## Per-substep computation, per axis (roll, pitch)

```
rateTarget = command × maxRate                      // command ∈ [-1,+1]; see action-space.md
rateTarget = clamp(rateTarget, ±GYRO_SATURATION_LIMIT)
rateError  = rateTarget − gyroRate

σ          = (maxRate × itermLockRateThresholdPct/100) / 2.35482
damping    = exp(−rateTarget² / (2σ²))              // Gaussian in the SETPOINT
aP = aD    = damping
aI         = min(damping, iLockActive ? 0 : 1)

P  = lpf_P(rateError × kP) × aP
D  = lpf_D(prevGyroRate − gyroRate) × kD / dt × aD  // derivative ON MEASUREMENT
FF = rateTarget × kFF                                // unfiltered
I  = clamp(I + rateError × kI × dt × aI, ±pidSumLimit × itermLimitPct/100)

out = clamp(P + FF + I + D, ±pidSumLimit)            // ±500, feeds the mixer unchanged
```

`iLockActive` ⇔ `|rateError| > maxRate × engageThresholdPct/100` **and** `|rateTarget|` exceeded
`0.2 × maxRate` within the last `lockTimeMaxMs`.

## Constants — all from the model XML (FR-014), values as on the flying config

| XML key | roll | pitch | source |
|---|---:|---:|---|
| `kP` | 0.484 | 0.484 | `fw_p_* / 31` |
| `kI` | 0.750 | 1.250 | `fw_i_* / 4` |
| `kD` | 0.003675 | 0.002625 | `fw_d_* / 1905` |
| `kFF` | **1.613** | **2.258** | `fw_ff_* / 31` |
| `maxRate` (°/s) | 360 | 120 | `roll_rate`/`pitch_rate` × 10 |
| `dtermLpfHz` / type | 10 / PT2 | 10 / PT2 | `dterm_lpf_hz` |
| `itermLockRateThresholdPct` | 40 | 40 | `fw_iterm_lock_rate_threshold` |
| `engageThresholdPct` | 10 | 10 | `fw_iterm_lock_engage_threshold` |
| `lockTimeMaxMs` | 500 | 500 | `fw_iterm_lock_time_max_ms` |
| `itermLimitPct` | 33 | 33 | `pid_iterm_limit_percent` |
| `pidSumLimit` | 500 | 500 | `getPidSumLimit`, fixed-wing |

## Deliberately NOT modelled — with reasons

| omitted | why |
|---|---|
| **TPA** | `tpa_rate = 0` ⇒ `tpaFactor = 1.0`. Modelling it would add a mechanism that is off. |
| **D-boost** | `d_boost_min = d_boost_max = 1.000` on this profile ⇒ identity. |
| **setpoint acceleration limit** | `rate_accel_limit_roll_pitch = 0` ⇒ off. Yaw's 10000 is irrelevant (no rudder). |
| **`pidLevel` / self-levelling** | ⛔ ANGLE-only. Including it is the FR-019a defect. |
| **yaw output** | computed by INAV, reaches no surface — no rudder (spec FR-018). |
| **soaring-mode deadband, autotune** | modes not in use. |

## Tests

1. **Steady state** — constant setpoint ⇒ achieved rate converges to it; rise time consistent with the gains.
2. ⭐ **Zero command, mis-trimmed craft** — non-zero `craftTrimDelta` ⇒ rate settles to zero and stays (SC-012).
3. ⛔ **No self-levelling** — displaced to bank with zero command, bank is approximately held over ~1 s and then drifts (expected: no attitude reference, drift on the order of seconds). ⭐ Run from **+30° and −30°**: the failure signature is drift **correlated with bank sign** (ANGLE restoring toward zero), not drift as such (FR-019a, SC-012 converse).
4. **Attenuation curve** — measured `aP` matches `exp(−r²/2σ²)` at r ∈ {0, σ, 2σ}; σ = 61.2 roll, 20.4 pitch.
5. **FF dominance** — at 88 °/s roll with zero error, output ≈ 142 ± tolerance (FF alone).
6. **I-lock** — a large setpoint step with large error freezes I accumulation for ≤ `lockTimeMaxMs`.
7. **Determinism** — identical seed/config reproduces bit-identical trajectories; eval-vs-training gate holds.
8. **Trainability (SC-004)** — a known-good genome still improves under the model.
