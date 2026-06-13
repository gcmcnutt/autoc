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

**DATASHEET CHECK (operator 2026-06-11)**: the actual craft servo is a **Power HD DSM-44
digital micro** — transit spec **0.07 s / 60°** (no-load), controller maps the NN span
linearly onto 1000–2000 µs. In sim units (full-throw/s) by mechanical span over that
pulse range: 60°→14.3, **90° (typical)→9.5**, 120°→7.1; under aero load (~1.5× derate)
the 90° case lands ≈6.3 ≈ the modeled center. Verdict on the params: the **3–9 draw
bracket is reasonable but shaded SLOW** — no-load hardware sits at/above the bracket
max, so t6 likely overstated the servo tracking cap somewhat. The transit spec is a
slew figure; tau (small-signal lag) remains an upper-bound estimate at 20 ms (a digital
micro's small-step response is likely well under). Span for 1000–2000 µs **CONFIRMED
90° (operator 2026-06-11)** — servo v2's `kServoMechSpanDeg = 90` stands (slew center
≈12.1 full-throw/s, clamp 8–16). Still open: a loaded bench step test. (Does NOT move
the t7 cadence verdict — the bang-bang returns with NO servo at all.)

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

## t6 bake (20 Hz + servo) — STOPPED at gen 267 for the t7 servo A/B (2026-06-10)

`autoc-037-t6-m1-20hz`, S3 `autoc-m1/autoc-9223370255738048515-2026-06-10T18:38:47.292Z/`,
gens 1-267, ~7.7 h, zero ELITE_DIVERGED. Final: best **-8,926**, 271/294 complete,
avgMaxStreak 4.4, **pctInStreak 1.6%**, roll ac +0.65 / flips ~24% / sat 0%, throttle sat ~99%.

- **vs the servo-matched 10 Hz arm (t3): clearly better** — passed t3's final best (-6,750 @368)
  by gen ~200, 3.6× t3's completions at gen 125, roll coherent throughout (t3-style bang-bang
  never formed), stability pressure lower (-44k vs t3's -52k trend).
- **vs the no-servo 10 Hz reference (035-t6): much slower and tracking-limited** — 035-t6 hit
  pctInStreak 7.1% by gen 200 / 14.3% by 300; t6 managed 1.6% at 267. Regime occupancy
  (dynamics_progress.png): tracking band only ~2%, intercept plateaued ~45-48%, distance
  floor ~19-20 m. The system **intercepts but cannot hold the tracking cone** — operator read:
  limited tracking ability, likely a basin/halt pattern.
- Confound unresolved by t6 alone: is the tracking cap the CADENCE (insufficient even at 20 Hz)
  or the SERVO DAMPING (lag+slew making tight cone-holding physically hard)? t3 vs t6 says
  20 Hz > 10 Hz *given* the servo; neither isolates the servo itself at 20 Hz.

## t7 arm (overnight 2026-06-10): 20 Hz, servo lag+slew DISABLED

Operator decision: comment out the in-FDM servo damping (`fdm_larcsim.cpp` `#if 0` block —
applies to BOTH aileron/roll and elevator/pitch; throttle was never servo-filtered) to restore
035-era idealized instant surfaces while keeping the full 20 Hz bundle (cadence, history basis,
entry cone, env-only ramp, energy^2.5). Run name `autoc-037-t7-m1-20hz-noservo`.

This is the finding.md "A/B (basic-m1 with the servo vs a near-instant servo)" TODO, run at
full bake scale: **t7 vs 035-t6 isolates the cadence at matched (ideal) servo physics; t7 vs t6
isolates the servo at matched 20 Hz.** Expected reads:
- t7 tracks like 035-t6 (streak takeoff) WITH coherent roll → cadence helps, servo damping is
  the tracking limiter → revisit servo params (the operator's standing skepticism above).
- t7 tracks like 035-t6 but with 035-style bang-bang roll → 20 Hz alone doesn't de-alias;
  the t6 coherence was servo-deadening (operator's caution) → re-think.
- t7 still tracking-limited → the limiter is elsewhere (objective/cone/NN capacity), not servo.

PRNG note: the per-scenario servoTau/servoSlew draws still execute (order preserved); the FDM
just ignores them. Determinism structure identical to t6.

## t7 verdict (gen ~463, 2026-06-11) + population-health note

**Case B confirmed at convergence depth.** t7's roll reconverged to the 10 Hz controller as
tracking deepened: gen 463 elite **lag1_ac −0.165, sign-flips 59.3%, dctrl 0.944** ≈ the 035-t6
baseline (−0.24 / 56% / ~1.0). Both T027 gate legs FAIL in the cadence-isolated arm, at
sub-parity tracking (pctInStreak 14.4 vs 22.0 at matched gen; streak holds 1.46 s vs 1.97 s
wall-clock — NOTE: avgMaxStreak is tick-denominated, divide by 2 at 20 Hz before comparing).
**Neither intervention (cadence ×2, servo realism) materially changed the craft's behavior —
still bang-bang.** The relay strategy is objective/architecture-optimal, not a sampling artifact.

**Population health (operator observation)**: at matched elite (best ≈ −22.4k), t7's population
mass is much thinner than 035-t6's — avg −3.9k vs −4.6k (and ~2× gap at matched gens: −3.9k vs
−7.1k at gen ~460), shorter gens despite +20% tick overhead, best-slope past gen 300 ≈ 60% of
035-t6's. Two candidate mechanisms: (1) mutational brittleness at 20 Hz — recurrent/relay
dynamics execute 2× per wall-clock second, so perturbations compound faster and offspring die
early; (2) chopped streak holds deny the mid-tier the streak-multiplier amplification that
deepens 035-t6's avg. A 20 Hz robustness tax on the evolution engine itself.

## NEXT EXPERIMENT (t8 candidate, operator 2026-06-11): 10 Hz at HEAD config — the anchor arm

Design: **back to 10 Hz, servo filter still OFF**, full bake, rough comparison to 035-t6 (and
029/pastonly3 lineage, which the operator is re-reading):
- `SIM_TIME_STEP_MSEC` 100 + all inis `ControlIntervalMsec=100` (cadence triple holds:
  fps=20/dt=0.005 → framesPerEval=2, the pre-037 arrangement). Servo block stays `#if 0`.
- Keeps the 037-era non-cadence config: **env-only ramp (craft NOT scaled, full magnitude from
  gen 0)** including the new (inert, with filter off) actuator-dynamics draws; tightened entry
  cone; energy^2.5; ms-based history lags (at 100 ms ticks = {16,8,4,2,1,0} — window 1.6 s, a
  KNOWN diff vs 035's uniform 0.5 s window that can't be removed without another layout change).
- **Question: does evolution look roughly like 035-t6?** t8 vs 035-t6 isolates the accumulated
  non-cadence config drift at matched cadence; t8 vs t7 isolates cadence at matched config —
  closing the comparison square (t6/t7/t8/035-t6).
- Caveat: single runs carry the basin lottery (specs/BACKLOG "M1 basin-landscape": ~1/3 fresh
  seeds stall; seeds don't transfer across builds) — "roughly the same" is the honest bar, and
  a stall may need a reroll before reading config drift into it.

Follow-on angle if t8 anchors cleanly: re-enable servo filtering (`#if 1`, possibly with the
DSM-44-corrected slew center ~7–8) and look closer at the servo arm with trustworthy params.

## t8 final (gens 1–625, stopped 2026-06-12): servo v2 @ 10 Hz still deadens — but the model is 2× too slow

**NOTE: the run executed as t8 is the P-O6 servo-ON arm** (`ServoModelEnabled=1`, auto seed
1781238882), NOT the P-O5 servo-OFF anchor designed above — that anchor arm has still never run.
S3: `autoc-m1/autoc-9223370255615893343-2026-06-12T04:34:42.464Z/`. PNGs (final, gen 625) in this dir.

**Result: deadened, flat from gen 1.** pctInStreak never exceeded **2.9%** across all 626 gens
(final 2.8%, avgMaxStreak 3.8 ticks = 0.38 s) vs 24.7% (035-t6 @610), 20.4% (t7 @610), 1.6%
(t6/servo-v1 @267). Fitness ground to −11.8k on non-tracking components (proximity/energy/
stability — occupancy t/i/p = 3/51/46%, mean dist 15 m: the craft *closes* but never holds the
band). Per-axis @625: roll dctrl 0.36 / |out| 0.42 (mid-amplitude, plant-muted), throttle
**pinned full** (|out| 0.996, sat 100%, dctrl 0.012), pitch 0.11/0.73. Roll lag1-ac +0.30 /
flips 42% — partially plant-smoothed relay, not learned finesse. Crash floor 21/294 = 7.1%,
same ballpark as t7's ~13–23/294 → floor tracks config (craft-full/history), not cadence.
Unlike v1's hard cap, v2 grinds upward slowly — but 10× below the no-servo arms throughout.

### Audit finding (2026-06-12): factor-2 slew unit mismatch — the v2 servo is twice the datasheet lag

`include/autoc/eval/craft_variation.h:44-48` derives the slew center in **full-span/s**:
`(60°/0.055 s)/90° ≈ 12.1` — i.e. 12.1 *90°-spans* per second, full-span transit 82.5 ms
(the DSM-44 measurement). But the FDM consumes it in **half-span units**:
`fdm_larcsim.cpp` `slewCap = 0.5 * servoSlew * dt` with surfaces on [−0.5, +0.5] (full span =
1.0 surface units). Effective surface rate = 0.5×12.1 = **6.05 u/s → 165 ms full-span transit,
2× slower than the datasheet servo** the constants file documents. v1 had the same mismatch
(center 6.0 intended as derated span/s → effective 3.0 u/s → 333 ms transit), which is why t6's
"pessimistic" servo was a hard cap — it was 2× more pessimistic than its own pessimistic params.

Operating-point impact at 10 Hz relay control: a full command reversal needs 1.0 u; the bugged
servo travels max 0.605 u per 100 ms tick → the surface **can never complete a reversal within a
tick** and oscillates inside a ±0.30 envelope (~60% authority loss at relay frequency). With
correct units it completes in 82.5 ms + ≤20 ms PWM latch ≈ within the tick — full authority,
honest delay. The bug is qualitative exactly at the operating point P-O6 was asking about
("does a little honest filtering coexist with tracking?") — **t8 did not answer that question;
it tested a servo half the datasheet speed.**

Fix (operator convention 2026-06-12): the slew number lives in **autoc command units/s** — the
[-1,1] NN/INAV span, full range = 2.0 units = the mechanical span — and platform code translates
at its boundary (crrcsim keeps its ×0.5 as the [-1,1]→[-0.5,0.5] surface conversion; INAV/xiao
would consume [-1,1] natively). So the autoc side changes: `kCraftServoSlewCenter` ≈ **24.2**
units/s (= 82.5 ms span transit), clamp **16–32** (125–62.5 ms transit, heavy-load to no-load),
ini `CraftServoSlewSigma` 2.0→**4.0** (doubled with the units, same physical spread). FDM keeps
the 0.5 but re-commented as the platform translation. Then **t9 = rerun this arm with the
corrected servo** — that is the honest P-O6 experiment.

Secondary fidelity notes (not t8 blockers, relevant once servos get honest): the throttle path
has NO actuator model — `thrustTau` lags only the scenario-static `craftThrustScale`
(`fdm_larcsim.cpp` engine(): a one-time ~0.5 s spool at scenario start, decorative thereafter);
the NN throttle command is neither PWM-latched nor lagged. t6 showed relay muscle migrating to
the un-lagged actuator — with corrected servos, an honest ESC/motor path (latch + spool on the
*command*) is the next mask to close.
