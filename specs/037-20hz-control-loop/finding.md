# 037 Finding: actuator dynamics, servo sluggishness, and the 10 Hz ceiling

Date: 2026-06-10. Refer to this when starting the **20 Hz M1**.

## TL;DR

A realistic in-FDM servo (slew-limited) lowers the achievable tracking fitness, and
at **10 Hz the NN responds by brute-forcing path completion with bang-bang surfaces +
high throttle rather than learning smooth control** — the servo low-passes the
bang-bang into sluggish motion, so it completes paths but tracks loosely. This is
largely *intended* servo realism, and it is the core argument for 20 Hz (a faster loop
can finesse the servo instead of banging it). **Operator is skeptical of the servo
time constant / slew params** (they are flight-data *estimates* with real uncertainty)
— validate them before trusting the 20 Hz de-alias verdict.

## Servo composition bug (found + fixed, crrcsim 36a94f3)

basic-m1 (t1) stalled. The servo filter **compounded slew x lag** (slew-clamped the
error THEN multiplied by the lag blend), making the effective rate ~4x too slow
(~1.5 vs the intended 6 full-throw/s). Fixed: exact dt-invariant lag `1 - exp(-dt/tau)`
with the slew cap on the lag STEP (independent, no compounding). Verified
(`verify_037_metrics.py`): small input -> t63% = 20 ms = tau; full throw -> ~6.5
full-throw/s. The servo lag is real (NOT a no-op, unlike the thrust lag).

## Overnight baseline (t3 — craft-RAMPED, now SUPERSEDED)

- `autoc-037-t3-m1-baseline`, seed **1781066211**, S3
  `autoc-m1/autoc-9223370255788564066-2026-06-10T04:36:51.741Z/`.
- 8 h, gen 368, **bit-deterministic (368/368 elite SAME, 0 diverged)**, no errors.
- best climbed -2400 -> -6750; completions 0 -> **259/294**.
- This run RAMPED craft variations -> superseded by the env-only-ramp decision below;
  re-run the baseline once env-only-ramp is built.

## KEY FINDING: servo-limited sluggish tracking + 10 Hz brute-force

Tracking quality across t3 (elite per gen):

| gen | complete/294 | avgMaxStreak | pctInStreak | stability | energy |
|----:|----:|----:|----:|----:|----:|
| 1   | 0   | 1.5 | 3.0% | -15k | 3.8k |
| 161 | 169 | 1.1 | 0.6% | -52k | 49k  |
| 361 | 259 | 1.6 | 0.9% | -50k | 62k  |

The NN learned to **complete** the paths, but tracking quality went the **wrong way**:
`pctInStreak` dropped 3% -> <1%, the streak stayed ~1.5, while it drove the surfaces
**3x harder into saturation** (stability -15k -> -50k) and burned **17x the throttle**
(energy 3.8k -> 62k).

Mechanism: the NN brute-forces completion with **bang-bang surfaces + high throttle**,
but the **slew-limited servo low-passes the bang-bang** -> the *actual* surface motion
is sluggish -> it can't stay tight on the maneuvering rabbit. So: **sluggish, yes** —
the servo behaves like a real servo (you can't track infinitely tight), which lowers
the realistic fitness ceiling vs the pre-037 idealized infinite-bandwidth servo.

The important part: **the NN's response to the slow servo is to bang HARDER, not smooth
out** — and that is the **10 Hz limitation**. At 10 Hz the loop is too slow to finesse a
realistic servo, so it muscles. This is essentially the core argument *for* 20 Hz, and
it means the energy / de-alias gates may only really move at 20 Hz (at 10 Hz the servo
and the bang-bang fight each other).

## OPEN QUESTION: servo time-constant skepticism (operator)

The operator is **skeptical of the servo time constant (tau_servo = 20 ms)** and
possibly the slew (6 full-throw/s). These are **estimates** with real uncertainty:

- `tau_servo = 20 ms` is an **upper bound** — the ~60 Hz blackbox log floor can't
  resolve a faster servo.
- `slew ~ 6 full-throw/s` was **inferred from servo datasheets**, not measured (the
  pilot never commands fast enough to hit the limit in the logs).
- The ~90 ms cmd->rate flight lag is **servo + airframe**, not the isolated servo.

TODO before the servo gates the 20 Hz de-alias verdict: **validate the params** — a
bench servo step-response measurement, or an A/B (basic-m1 with the servo vs a
near-instant servo via `kCraftServoSlewCenter` / `kCraftServoTauCenter`): if tracking
tightens dramatically without the servo, the servo is the (realistic) limiter and the
params matter; if even a fast servo stays loose, the limiter is elsewhere (fitness /
cone / NN capacity).

## Variation ramp: env-only (craft NOT ramped)

Decision (operator 2026-06-10): **only ENV variations ramp** (entry pose / wind /
position — a difficulty curriculum). **Craft variations are NOT ramped** — they are
present at full drawn magnitude from gen 0 (`scenario_meta_apply.h`). Rationale: the
ramp protects a fresh population from *unflyable* difficulty (env), not diversity
(craft). A varied craft is still flyable; ramping it only delays fleet-robustness
training and (for the dynamics) physically-odd interpolation of a time constant.
**Camera variations (future tracker mode) are the same category — do NOT ramp them.**

## Thrust lag (separate, near-inert)

The thrust lag lags `craftThrustScale` (~1.0 multiplier), NOT the throttle->thrust path
-> near-inert. Spool-up fidelity is NOT modeled. Separate rework when wanted.

## For the 20 Hz M1

Hypothesis to test: at 20 Hz the NN **finesses** the realistic servo (smoother control,
tighter tracking, command de-alias) where 10 Hz **brute-forces** it. Before trusting
the de-alias gate, resolve the servo-param skepticism above. The env-only-ramp baseline
(to be re-run) is the 10 Hz reference for the comparison.

## 20 Hz smoke runs (t4 FAIL / t5 PASS, 2026-06-10)

**t4 (`autoc-037-t4-basic-m1-20hz`) — FAILED, found the second flip bug.** All 3000
individuals scored an IDENTICAL -96.508202 (best=avg=worst, flat across gens, all-crash):
NN commands never reached the surfaces. Root cause: the pending-command apply sat at the
END of `getInputData`, AFTER eval staging — fine at `framesPerEval=2` (the eval frame
staged, the in-between frame applied) but at `framesPerEval=1` (20 Hz) every frame
re-staged `gPendingCommand` (pushing `readyTimeMsec` forward) before the apply could
fire. **Fixed crrcsim `f81fd31`**: apply moved to frame top — behavior-identical at
10 Hz, correct at any framesPerEval. (Pattern echo of the 36a94f3 servo bug: cadence
plumbing that only ever ran at one rate hides order-of-operations bugs; the basic-m1
smoke run caught both.)

**t5 (`autoc-037-t5-basic-m1-20hz`) — PASS, 20 Hz learner validated.** pop 3000 /
16 winds, perf build (autoc `c4767ed` + crrcsim `f81fd31`):

| gen | t2 best (10 Hz) | t5 best (20 Hz) |
|----:|----:|----:|
| 1   | -187 | -201 |
| 80  | ~-206 | -256 |
| 120 | ~-175 | -280 |
| 160 | -246  | **-403** |

- Gen-1 magnitudes land on the historical scale (tick-rescale anchoring works:
  best/avg/worst -201/-86/-66 vs t2's -187/-81/-62).
- The climb starts EARLIER and goes FARTHER than 10 Hz — by gen 160 t5 is past where
  t2 finished at gen 280 (-277). Consistent with the RT case-A prediction (50 ms tick
  samples the 192 ms roll pole instead of aliasing it). ~9.5 s/gen (2000-tick scenarios
  did not hurt wall-clock at this pop).
- Killed at gen 173 (best -392.2) — diagnostic complete.

**Open for the bake comparison**: the env-only-ramp 10 Hz reference re-run (above) never
happened — we flipped to 20 Hz instead, and HEAD can no longer run 10 Hz without a
checkout. De-alias gate (T027) uses the recorded 035-t6 metrics per Q1/Q3; if a
matched-config 10 Hz reference is wanted, run it from stage-1 commit `9592dea`.
