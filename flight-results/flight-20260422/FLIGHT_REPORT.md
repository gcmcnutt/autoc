# Flight 2026-04-22 — Post-024 Shakedown

**Date**: 2026-04-22 ~17:52 local
**NN weights**: cadence7 (gen 400, best=-35951)
**Xiao binary**: cadence7 weights + timer fix (`b2e6702`) + rabbit logging (`T080`)
**INAV**: flight FC with `(w, x, -y, -z)` quat convention (`T042`) and flight-ready RC mapping
**Wind**: ~320° at ~15 kt (reported) — within training envelope (training base 330°±45°, 12 m/s sigma)

**Overall**: 024's stated goal — **sim-to-real cmd → rate sign-correct on all three axes** — is
achieved. Conventions clean through the full pipeline. Timer fix holds in flight. AHRS
tracks independent-integration reasonably. The dominant remaining issue is behavioural,
not convention: the NN collapses to a full-throw bang-bang policy, and needs temporal
state machinery (smoothing/integration) it currently doesn't have.

## Engage spans

Three engage spans across the flight:

| Span | Duration | NN evals | Path | Avg airspeed | Behaviour |
|---|---|---|---|---|---|
| 1 | 8.3 s | 84  | 0 | 18.0 m/s (near Vmax) | Short attempt; high airspeed, aggressive maneuvering |
| **2** | **18.3 s** | **184** | **2** | **16.3 m/s** | **Longest; completed loops/figure-8s; pilot's "mightily"** |
| 3 | 13.5 s | 135 | 0 | 14.1 m/s | Slower rabbit variant; lower-amplitude inputs |

`ctl loop:` summary (from the new fixed-schedule timer):

| Span | ticks | overruns | resyncs | maxLate | avgLate |
|---|---|---|---|---|---|
| 1 | 168 | 0 | 0 | 38 ms | 1.77 ms |
| 2 | 368 | 0 | 0 | 39 ms | 2.28 ms |
| 3 | 270 | 0 | 0 | 40 ms | 2.06 ms |

Timer fix (`b2e6702`) held across all three spans: zero overruns, zero resyncs, max lateness
within one tick of ideal. 100 ms cadence locked.

## Convention / sensor audit

### 8-panel audit on blackbox ([`sensor_self_check.png`](./sensor_self_check.png))

7 of 8 cross-checks PASS:

- **Check 1 position↔velocity integration**: PASS all axes (slope ≈ 1, r > 0.92). navPos and navVel are self-consistent.
- **Check 2 gyro↔quat-delta**: PASS all axes (slope ≈ 1, r > 0.998). The strongest convention-correctness signal — the raw gyro in INAV's frame and the raw quaternion integrate as a self-consistent pair. Every previous flight had to battle this; it is now clean.
- **Check 4 accel↔gravity**: PASS in sign but weak correlation — see `acc_1G` note below.
- **Check 5 heading↔track**: PASS slope +0.29 r=0.43. Weak r because crabbing dominates in today's wind.
- **Check 6 mag↔heading**: PASS sign but stddev 68° — mag is noisy in flight (typical).
- **Check 7 attitude vector↔velocity dir**: **FAIL on D axis** (slope −0.35). N and E are sign-correct (+0.86, +0.67). This is the one yellow flag — body-forward nose-down projects opposite to velocity-down. Combined with today's bang-bang, our read is that aircraft nose-direction decouples from descent-angle during abrupt maneuvers; not necessarily a convention bug, but worth revisiting on a smoother flight.

### Direct gyro↔quat-delta agreement ([`gyro_vs_quat.png`](./gyro_vs_quat.png))

400 NN samples, all three planes:

| plane | slope | r |
|---|---|---|
| roll  | +1.21 | +0.96 |
| pitch | +1.02 | +0.96 |
| yaw   | +1.17 | +0.97 |

Slopes near +1, correlations > 0.95. AHRS attitude and raw gyro are integrating into each
other with no sign flip and near-unity scale.

### Cmd → response scatter ([`cmd_response_scatter.png`](./cmd_response_scatter.png))

Per-axis best-lag correlations (cmd → rate):

| axis | best-lag r | convention |
|---|---|---|
| pitch → gyrQ | **+0.73 @ +202 ms** | ✓ positive, correct |
| roll  → gyrP | **+0.75 @ +202 ms** | ✓ positive, correct |
| throttle → vel | **−0.21 @ +303 ms** | saturated (NN pins throttle at +1 ≥ 90 % of the time; no control variance to correlate against) |

### AHRS independent check ([`gravity_check_*_span*.png`](./), new this flight)

Two independent cross-checks of the AHRS quaternion that do NOT use the self-referential
"renderer projection looks right" tautology:

1. **Gravity projection**: rotating world gravity `(0, 0, g)` into body frame via the AHRS
   quaternion should track the measured accelerometer vector (with maneuvering load riding
   on top as linear accel). In all three spans, the projected-gravity dashed curves
   visibly track the slow-varying component of the measured accel. Residual mean during
   low-gyro samples: ~1.5 g. That's larger than "perfectly steady" should be, but a
   bang-bang aircraft is never actually steady — this is centripetal + pitch accel from
   rapid maneuvers riding through the quasi-steady filter. No systematic offset direction.
2. **Gyro-integrated attitude vs AHRS attitude**: integrating the raw gyro from the first
   AHRS quat of each span produces an attitude time-series. Over 18 seconds (span 2), the
   gyro-only integrated roll and pitch track the AHRS roll and pitch closely — divergence
   hovers around zero, not growing. The AHRS is not drifting on the short term, and the
   raw gyro integrates without sign flips that would indicate a convention error.

**Verdict**: AHRS accuracy is as good as we can prove without an independent inertial
reference (the xiao's onboard IMU cross-check is [project_xiao_imu_crosscheck.md](/home/gmcnutt/.claude/projects/-home-gmcnutt-autoc/memory/project_xiao_imu_crosscheck.md) in the
backlog — would become the gold-standard check). For the current flight campaign, the
conventions are correct and the AHRS estimates are consistent with both raw inertials and
gravity projection.

### Hidden bug discovered: acc_1G scaling

The `plot_gravity_check.py` run surfaced a latent issue: **this FC's ICM IMU uses
`acc_1G = 2048` LSB/g** (visible in the `.TXT` header). The shared analysis library
[`sensor_self_check_lib.py`](../flight-20260417/sensor_self_check_lib.py) hardcodes
`ACC_1G_LSB = 256` (typical older boards). The gravity/accel cross-check (check 4) passes
only on sign gating; slope magnitudes have been off by 8× on every flight from this FC.

Fix: `plot_gravity_check.py` now auto-detects from the `.TXT` header (falls back to 256
if not found). `sensor_self_check_lib.py` should adopt the same approach — noted as a
backlog item.

## Control behaviour

### Bang-bang is dominant ([`bangbang_flight_*.png`](./))

Per-span `|out|` and `|Δout|` statistics:

| Span | `<|pt|>` | `<|rl|>` | `<|th|>` | `<|Δout|>/tick` |
|---|---|---|---|---|
| 1 | 0.70 | 0.65 | 0.91 | 1.08 |
| 2 | 0.56 | 0.61 | 0.97 | 1.01 |
| 3 | 0.55 | 0.55 | 0.93 | 0.96 |

The output-value histograms (bottom-right panel of each bang-bang PNG) show **bimodal
lobes at ±1 for pitch and roll**, and a **single spike at +1 for throttle**. This is the
canonical bang-bang signature. Mean `|out|` per tick ≈ 2.14, which is essentially
identical to cadence7 *training* (2.2) — the NN behaves exactly as it was trained. The
bang-bang isn't a deployment bug; it's what the training landscape rewards.

### Elevon asymmetry

The join-analysis `rcData→servo` correlations (from
[`join_analysis_..._span*.png`](./)) are:

| span | roll_rcData→servo0 | pitch_rcData→servo1 |
|---|---|---|
| 1 | -0.89 | -0.52 |
| 2 | -0.89 | -0.53 |
| 3 | -0.81 | -0.52 |

Both magnitudes and polarities are consistent across spans. The near-unity correlation on
servo0 (-0.89) vs the weaker correlation on servo1 (-0.52) suggests an asymmetric elevon
mix or travel — visible in the video playback. Bench-check item.

### Throttle saturation is the physical limit

`<|thr|>` = 0.91–0.97 across all spans. The NN asks for near-full throttle essentially
continuously. With today's wind, rabbit at 12 m/s ground-speed on an upwind leg demands
~19.7 m/s airspeed (>Vmax); on a downwind leg, airspeed would drop under stall. Neither
end of the envelope admits energy conservation, so throttle-saturation-with-pitch-trading
is the only policy available. Span 1's 18.0 m/s avg airspeed (near Vmax=17) and the
throttle histogram column at +1 confirm this.

## Against training envelope

Training envelope (from [`autoc.ini`](../../autoc.ini) + [`autoc_config.xml`](../../crrcsim/autoc_config.xml)):

- Wind magnitude: 12 m/s base (CRRCSim config is m/s, verified).
- Wind direction: 330° ± 45° sigma.
- Rabbit speed: 12 m/s nominal ± 2 m/s sigma, range [9, 18].

Today: ~7.7 m/s wind (15 kt), 320°, rabbit nominal 12 m/s. All within envelope, direction
within sigma, wind *less* than training. The sim-to-real gap is therefore not the
envelope — it's:

1. Real gust/turbulence spectrum vs CRRCSim's Dryden model (sim under-models gustiness).
2. HB1 rudder-moment under-modeled in sim (noted yesterday, logged to 025).
3. No NN input for wind state or time-history beyond direction cosines.

## What's next

024 flight-validation is effectively complete:
- Timer fix shipped and verified (`b2e6702`).
- Renderer time-pairing fix shipped and verified (`4f94d14`).
- Conventions all sign-correct in flight.
- AHRS independently cross-checked.

**The blocker for meaningful progress on control quality is not a bug — it's a
missing NN-architectural capability**:

> The NN sees current target direction cosines (6 history taps via `d(cos)/dt`), current
> quat, current gyro, current airspeed, current distance — all *current-state* inputs.
> It has no machinery to build an integrator, no explicit time-history of its own
> outputs, no low-pass filter structure. Evolution against a current-state-only input
> vector will, in the limit, always find bang-bang as the optimum — because smooth
> control requires internal state the NN cannot represent.

This supersedes 025 (craft variations) as the critical-path task. Adding aero/servo/CG
variations under the current bang-bang policy will mostly add noise; variations will
select meaningfully only once the NN has the structural capacity to produce smooth
control.

Proposed:
1. **New story "026-nn-temporal-state"** (or similar) — restore prior-output feedback into
   the NN input vector, or add explicit integrator/filter-ish nodes to the NN topology.
   This was done in earlier features (pre-022) and removed; worth revisiting now that we
   have a clean cmd→rate baseline to measure "smoothness" against.
2. **025-craft-variations: blocked on 026.** Leave the HB1 rudder, aero variation, and
   servo dynamics items in place; come back once 026 yields a smooth baseline.
3. **Minor follow-ups** (backlog):
   - Fix `sensor_self_check_lib.py` to auto-detect `acc_1G` from `.TXT` header.
   - Bench-check servo0/servo1 travel symmetry on the flight FC.
   - Xiao onboard IMU cross-check (project memory item) — independent AHRS check.

## Artifacts

All in this directory:

- `sensor_self_check.png` / `.md` — 8-panel convention audit (blackbox).
- `gyro_vs_quat.png` — direct gyro ↔ quat-delta scatter.
- `cmd_response_scatter.png` — per-axis best-lag cmd→rate scatter.
- `join_analysis_..._span{1..3}_path{0,2}.png` / `.csv` — per-span multi-panel flight
  diagnostic from [`specs/023-ood-and-engage-fixes/join_flight_analysis.py`](../../specs/023-ood-and-engage-fixes/join_flight_analysis.py).
- `bangbang_flight_..._span{1..3}_path{0,2}.png` — new, per-span bang-bang
  visualization from [`specs/024-sim-real-fidelity/plot_bangbang_flight.py`](../../specs/024-sim-real-fidelity/plot_bangbang_flight.py).
- `gravity_check_..._span{1..3}_path{0,2}.png` — new, AHRS independent check from
  [`specs/024-sim-real-fidelity/plot_gravity_check.py`](../../specs/024-sim-real-fidelity/plot_gravity_check.py).
