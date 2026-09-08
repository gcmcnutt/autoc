# What changes for the next M1 bake, versus 043-t2

**Baseline**: `043-t2` = `autoc-m1/autoc-9223370248704297747-2026-08-31T04:27:58.060Z/`, gen 800,
fitness −88,013.84, flown 2026-09-05 and 2026-09-06.
See [artifacts/MANIFEST.md](artifacts/MANIFEST.md) for its full pinning.

⛔ **Every item below is determinism-affecting or plant-affecting.** The next run is **not** comparable
tick-for-tick with t2, and is not meant to be — these are fidelity corrections, not tuning. Judge it on
fixed-eval and per-axis measures, per [project_late_run_fitness_interpretation](../../.claude/projects/-home-gmcnutt-autoc/memory/project_late_run_fitness_interpretation.md).

---

## A. Sim plant — DONE, in the tree now

| # | change | was | now | evidence |
|---|---|---|---|---|
| A1 | `COMPUTE_LATENCY_MSEC_DEFAULT` | **30 ms** | **10 ms** | flight-measured 2026-09-05: fetch 2.9 + eval 1.6 + send 5.4 = **9.9 ms**. The old value predated current firmware. |
| A2 | NN **gyro** input | raw FDM | ⛔ **still raw — see below** | `gyro_main_lpf_hz = 25` (~6.4 ms) |
| A3 | NN **accel** input | raw | ⛔ **still raw — see below** | `acc_lpf_hz = 15` (~21.2 ms, 42% of a tick) |

⛔ **A2/A3 were implemented and then BACKED OUT on 2026-09-07, before any bake.** The sensor gather runs
only on the **20 Hz** eval cadence, so 25 Hz and 15 Hz filters sit far above the 10 Hz Nyquist — measured
k = 0.887 / 0.825, i.e. near pass-through adding a few ms by accident rather than by design. The real
chain is *filter at 2 kHz, then sample at 20 Hz*, whose effect on the sampled value is a **group delay**,
not a filter that can be re-run at the sample rate.

⇒ The filter classes are kept in `include/autoc/eval/sensor_lpf.h` with both correct implementations
written up: (a) filter at FDM substep rate (5 ms) and let the gather read the filtered state — a
`Controller` runs every FDM step, which is how `Cntrl_StepTest` works; or (b) a per-channel sub-tick delay
folded into the existing staged-command latency path. ⚠️ **A2/A3 are therefore OPEN, not done.**

⭐ A1 alone still pushes the right way — the sim now **acts sooner** than it used to, matching §6.

## A′. ⭐ OBJECTIVE — M1 now pays for leaving the arena (T084)

⛔ **This is the biggest change in the list, and the only one that touches the objective.**

| setting | was | now |
|---|---:|---:|
| `EnableHullCrashPenalty` | 0 | **1** |
| `HullCrashPenaltyFactor` | 0.5 | **0.75** |
| `OobCrashPenaltyWeight` | 0.0 | **2.0** |

On t2, an arena egress cost M1 **only** the points forgone to the end of that scenario, while breaking a
streak cost the whole 5 s climb back from 1× to 5× — so late in a scenario, **busting the floor was
cheaper than backing off**. Measured: crash 4–7% vs 041-t7's 0.7%, **100% `egFloor`/`egRadius`**,
`hullStrike = 0`.

⛔ **Two-line change, not one**: `applyCrashPenalty()` returns early on `!enableHullCrashPenalty`
(`autoc.cc:276`) and that gate covers the **OOB branch too** — the weight alone is a silent no-op.

⇒ ⛔ **Raw fitness is NOT comparable to t2's −88,013.84.** Judge on crash rate, `pctInStreak`, and the
per-axis measures.

## B. Aircraft config — APPLIED to the FC, ⚠️ NOT yet in the config of record

| # | change | status |
|---|---|---|
| B1 | `rc_expo` **20 → 0** | ✅ applied; 09-05 bench fits expo **0** (residual 0.98/1.24 vs 25.9/13.7). Makes the *aircraft* linear, matching what the sim always assumed. |
| B2 | `setpoint_kalman_enabled` **ON → OFF** | ✅ applied; removed an adaptive filter from both the rate loop and the NN's gyro. ⭐ Also bought **10.6 ms** of loop latency, which is what A1 records. |

⛔ **`xiao/inav-hb1.cfg` still reads `rc_expo = 20` and Kalman `ON`.** The changeset was applied to the FC
but **the fresh dump was never pulled**, so the config of record is stale — the exact failure T051a fixed
two weeks ago. ⇒ **Pull a dump before the bake**, or the run's manifest will describe an aircraft that
does not exist.

## C. Firmware — DONE

| # | change | note |
|---|---|---|
| C1 | engage-prefill frame fix | the 2026-09-05 root cause: 5 of 6 history slots seeded with ‖origin‖ (162–210 m) for 800 ms into a fresh recurrent state |
| C2 | flight log **v4 → v5** | cone constants in header + `step_score` per tick; all three readers updated |

⚠️ C1 changes what the *policy* sees at engage but **not** what training sees — the sim never had the bug.
So it improves flight, not the bake.

---

## D. ⛔ Open, and gating in my view

| # | item | why it gates |
|---|---|---|
| D1 | **Pitch peak timing**: sim peaks at **85 ms**, real at **133–166 ms** | The one *measured* plant defect still unfixed. Candidates: `Cmq`, `Cm_alpha`, pitch inertia. |
| D2 | **Second airframe** | Everything rests on `n = 1` with a **known-asymmetric wing and nose-heavy CG**. The stall cycle is exactly the kind of thing that is an *article* property. A bake tuned to this article may be tuned to its defects. |
| D3 | Fresh `inav-hb1.cfg` dump | see B — cheap, and the manifest depends on it |
| D4 | **A2/A3 sensor-path delay** | backed out as un-modelled-at-20-Hz; the accel term is 21 ms on the channel 041 P5-1 added. Needs the FDM-rate or sub-tick-delay implementation. |

⚠️ **My read**: D1 and D2 are the difference between a bake that closes 043 and one that has to be
re-run. A1–A3 are unambiguous and worth having regardless, but the *pitch* channel — the one 043 exists
to fix — still has a measured, unexplained 50–80 ms timing error, and we have one aircraft's word for
what "correct" even looks like. ⭐ **The second wing is cheaper than an 800-generation bake** (~43 h), and
it is the only thing that can tell us whether the stall cycle is the design or the article.

⇒ Recommendation: **fly the second article first**, then decide D1 with two data points, then bake.
If the schedule does not allow that, bake with A+B+C and treat the pitch result as provisional — but
record in the manifest that it was baked against a plant with a known 50–80 ms pitch-timing error.

## E. Not changing, and why

- **Rate-loop gains, `pidSumLimit`, filters, `servo_pwm_rate`, rates 36/24** — the config audit found the
  sim is a **parameter-exact replica** of the FC rate loop. Nothing to fix.
- **Roll model** — within ~17% gain and ~10% rise. Leave it.
- **`craftServoSlew` / clamp `[16,32]`** — T046's bench step-response never ran, so the 0.055-vs-0.070
  s/60° conflict is still open. ⛔ Do **not** move these on an in-flight guess; per `2c691aa` this is a
  fine-tune wanting the second article.
- **`rc_filter_auto`** — stays OFF. At 20 Hz MSP it would compute ~7.7 Hz / 62 ms.
