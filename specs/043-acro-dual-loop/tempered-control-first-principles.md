# Tempered control — first principles, the history, and the grease for t4 (2026-10-05)

Operator framing that this answers: *"with all the rest of t4 planned, we STILL need to develop a system
that resists bang-bang … the whole point of dual loop was to offload acro control to inav … and then our
autoc 20 Hz loop can focus on patrol/intercept/tracking … the controls need to be a bit more tempered — not
dull, just smooth and perhaps thinking a little further ahead. Back to first principles."*

[`vibration-analysis-20261004.md`](vibration-analysis-20261004.md) § 6.4 is the new fact that forces this:
the 2–3 Hz pitch cycle is a **±7 g loading event** (filtered `|accSmooth[2]|` p99 5.2–7.5 g, max 8.4–11.4 g)
that wrecks the static source, pushes the accelerometer toward its rail, and is the common parent of
most of the Z trouble. And t3 § 5.3 already showed it is **commanded** — `out_pitch` peaks at 2.19 Hz.

---

## 1. The chain, end to end, as it exists today

Everything between the network and the elevon, with what each stage does to a rough command:

| stage | what it does to roughness | evidence |
|---|---|---|
| **NN output** (20 Hz, ∈ [−1, 1], = rate setpoint × 240 °/s pitch / 360 roll) | whatever the policy emits | — |
| **xiao → `MSP_SET_RAW_RC`** | **nothing** — no slew, no filter | `grep` of `xiao/src`: no smoothing anywhere |
| **sim command apply** | **nothing** — *"no slew rate limiting — matches hardware path"* | `inputdev_autoc.cpp:393` |
| **INAV `applyRateDynamics`** (ACRO path, `fc_core.c:408`) | identity today: `rate_dynamics_*` at neutral (100/100/10/10/0) | `inav-hb1.cfg:1926` |
| **INAV rc interpolation LPF** | effectively off: `rc_filter_lpf_hz = 250` against a 20 Hz input | `inav-hb1.cfg:1401` |
| **INAV fixed-wing rate loop** | **reproduces it faithfully**: `kFF` 2.26 pitch ≫ `kP` 0.484, and P/D are Gaussian-attenuated toward zero as commanded rate rises (σ 20.4 °/s at `pitch_rate 12`, ~41 at 24) — a setpoint step becomes a surface step at FF gain | `research.md` R1 / Finding 2 |
| **servo** (honest slew model since 037) | the only tempering in the chain, ~82.5 ms transit | 037 |
| **airframe** | real one rings at 3.0–3.8 Hz (short period) — **the sim's does not** (D1) | `actuator-pin.md` § 7 |

⭐ **The dual loop did what it was asked.** Rate *tracking* is offloaded and works: delivered fraction
0.51–0.70, no inner-loop instability, pitch rail 13% → 0.0–0.3% across the 09-05 → 09-13 flights.
What it was never asked to do is make the **setpoint** smooth. A 2 kHz feed-forward loop fed a 20 Hz
staircase produces a 20 Hz staircase on the surface — at `kFF` gain. The offload is complete for
tracking and **absent for shaping**.

## 2. The objective never asks for it

The lexicase pool today (`src/eval/selection.cc:76-90`), per scenario:

| case | quantity | status |
|---|---|---|
| `score` | tracking (cone / streak) | ON |
| `energy_score` | convex throttle power | ON since 035 |
| `stability_score` | `Σ (|out_pt|−1)+(|out_rl|−1)` — pitch/roll **amplitude** | **OFF since 035 FR-008** |
| `prediction_score` | M2 predictor head | M2 only |

So **no case in the pool measures how fast the pitch/roll command changes.** The commented-out axis was
the wrong quantity anyway — amplitude, not rate. The project's own per-tick thrash counters
(`THRASH_THRESHOLD = 0.5`, `fitness_decomposition.cc:392`) exist but feed a tracker diagnostic, not
selection. The crash price (t3) showed what a single priced constraint does: it moved the behaviour it
named, exactly as intended, and nothing else. Command rate has never been named.

035's bet (FR-008/FR-009) was that smoothness would *emerge* from the energy axis via induced drag. The
outcome was honest about how far that went: pitch `dctrl` 0.524 → 0.506, throttle −20%, and **roll
concentrated at 0.95–1.25** — the energetically cheapest axis absorbed the roughness. Roll was then
attributed to the 10 Hz loop and handed to 037. 037's 20 Hz + honest servo collapsed saturation
(roll 46% → 1%, pitch 46% → 6%, throttle |out| 0.99 → 0.70) — in sim, and with the caveat the wrap
itself records: per-tick `dctrl`/flip metrics are **rate-confounded**, flattered at 20 Hz. 041's input
scaling cut airframe load (p50/p95 1.92/5.21 g → 1.45/4.26 g; ≥ 8 g ticks 5.9× lower) as a side effect
of better gradients, not of any smoothness term. **No run has ever trained with a smoothness axis under
lexicase.** The only smoothness *objective* ever tried was 033's scalar penalty, which collapsed
(banned since; `project_scalar_multiobjective_collapse`).

### 2a. The older history, from git (operator: "we did have control smoothness way back")

| when | form | fate |
|---|---|---|
| **2024-07** (`b015982`, GP era) | `control_smoothness_sum`: Euclidean norm of per-tick Δ(roll, pitch, throttle), clamped, ^`FITNESS_CONTROL_WEIGHT`, normalised by distance completed — **scalar** term | pre-spec-kit; superseded |
| **2025-11-28** (`6e226d0`, last pre-spec-kit `autoc.cc`) | `CONTROL_RATE_PENALTY 2.0` on `roll_change + pitch_change` **together with** `CONTROL_SATURATION_PENALTY 5.0` above 0.9, plus "control excess relative to path requirements" — all **scalar** | superseded by 015/022 fitness |
| **2026-03-16 → 17** (015, `f8eb9c3` → `92edaa6`) | `Σ|Δu|` as a **lexicase** dimension | ⛔ **reverted next day**: *"rewards saturation — a pegged output has Δ=0 and looks perfectly smooth, reinforcing the spiral exploit (pitch=max, throttle=max, roll-only)."* Revisit note: path-relative smoothness (Δu ÷ path curvature). |
| **2026-05** (033) | multiplicative per-tick smoothness factor — scalar | ⛔ collapsed (banned) |
| **2026-06** (035 FR-008) | amplitude axis `Σ(|out|−1)` | switched OFF, bet on emergence from energy |

⭐ Two things the archaeology settles. First, **the only time command rate was a lexicase axis it was
exploited within a day** — and notice that the 2025-11 scalar had carried a *saturation* penalty next to the
rate penalty, which is exactly the companion the 015 axis lacked. Second, every form so far measured the
**command**, which is an indirect measure of what we care about. The operator's reframing is the right one:
the candidates are the **direct** measures — command smoothness, total energy, **craft body-z acceleration**.

### 2b. Total energy has its own exploit on record

041 P2-5 replaced the throttle-power axis with *"metres of Es destroyed"* (total specific energy lost).
t4 of 041 **re-pegged throttle at 1.000 on 100% of 129,732 ticks** (`fitness_decomposition.cc:292-330`):
full throttle *raises* Es, so the policy hid energy loss behind power. P2-7 reverted to charging power
spent. The useful residue: with throttle unable to hide it, *"the only remaining way to lose energy is
drag, measured at **corr(load, destroyed) = +0.72**"* — the project has already measured that manoeuvre
**load is the drag-energy signal**, without the throttle loophole.

## 3. Sim vs flight — the command stream itself

Same metric, same 20 Hz, same units, t3 genome. Sim = its own training table (294 scenarios, median
[p10..p90]); flight = 09-13 spans 1/2/3.

| axis | | `<|Δout|>` | sign-flip % | lag-1 autocorr | rail % |
|---|---|---:|---:|---:|---:|
| pitch | sim | 0.102 [0.08..0.15] | 40 | 0.90 | 0.2 |
| | **flight** | **0.19–0.23** | **52–59** | 0.77–0.79 | 0.0–0.3 |
| roll | sim | 0.190 [0.14..0.25] | 46 | 0.70 | 0.7 |
| | **flight** | **0.23–0.38** | **62–71** | 0.20–0.51 | 0–1.2 |
| throttle | sim | 0.158 | 47 | 0.90 | 18 |
| | **flight** | 0.12–0.14 | 62–64 | 0.50–0.64 | **87–91** |

Two readings, both true:

1. **In flight the pitch command is ~2× rougher than in sim**, roll flips sign on 2 of every 3 ticks,
   and 56–81% of pitch-rate power sits at 2–3 Hz against sim's 10.7% (t3 ADDENDUM A1). That excess is
   **closed-loop with the real plant** — the resonance the sim lacks (D1). The policy is not emitting it in
   sim because nothing in sim rings back.
2. **The sim baseline is itself far from smooth.** A sign-flip rate of 40–47% is a random walk (50%),
   and roll's lag-1 autocorrelation of 0.70 at 20 Hz means the roll command is substantially
   re-decided every 50 ms. The sim already produces |n_z| p99 **4.3 g** median per scenario (p90 6.0 g,
   max 10 g) — large loads, un-priced.

⇒ D1 explains the *excess* in flight. It does not explain the *floor*, and the floor is what a 2 kHz loop
faithfully reproduces. Fixing D1 lets the sim charge for exciting the resonance; it does not, on its own,
make the policy prefer a smooth setpoint. **Both are needed, and only one is in t4 today.**

## 4. Why the obvious fix was already tried and failed — and what is different now

Setpoint filtering was tried: **023 Phase 9a** replicated INAV's pt3 RC-smoothing filter in crrcsim at a
40 Hz cutoff. Training **stunted** — best −2225 vs −4410 at gen 55, `pctInStreak` 3% vs 12%; 20 Hz was
worse. Conclusion of record: *"the filter mechanically prevents the NN from making the quick corrections
it needs … The NN must learn that smooth commands are better through its own fitness signal, not be
mechanically constrained. A fitness-based smoothness incentive (lexicase or per-step penalty) is the
better path."* 043's spec acknowledged it and reasoned ACRO would differ because it is a controller, not
a smoother — which turned out to be exactly right: ACRO fixed tracking, and left shaping untouched.

What has changed since that experiment: 10 Hz → 20 Hz; direct surfaces → rate setpoints; an ideal
servo → the honest one; and the operator's constraint is now explicit — **tempered, not dull**. What has
not changed is the evidence that a hard low-pass in the command path fights the policy. A *slew limit*
(bounding |Δsetpoint| per tick, no phase lag below the bound) is a different object from a 3rd-order LPF,
but it is still a mechanical constraint and the 023 result is a standing warning. The fitness route comes
first.

## 5. "Thinking a little further ahead" — lookahead is REJECTED (operator 2026-10-05: "that's cheating")

The 20 Hz loop's job in the dual-loop design is patrol / intercept / tracking — planning. Today it is a
**reactor**: past-only inputs with a 0.8 s history window (029 removed future inputs so the M1 architecture
would carry to M2, where the real target's future is unknown). The project's own record of what lookahead
did (`project_evolved_strategy_vs_airframe`):

- **with-future** controllers (cadence7-redux, more-rnn1/2/3, +0.1/+0.5 s) flew **smooth path-following**
  and **generalized better off-distribution** (more-rnn3 passed the tier1/tier2-random/tier3-stress tiers
  that the past-only controller failed);
- **past-only** controllers converged on **reactive tight-spiral** strategies — fitness-equivalent on the
  training envelope, more brittle off it.

⛔ **Off the table.** Ancient versions had +0.1/+0.5 s lookahead and it flew smoother — but 029 removed it
for the reason that still holds: *"Tracker mode has no such oracle. A target craft's future trajectory is
not known to the controller, so any future-lookahead input is fictional."* Giving M1 a path oracle would
teach a crutch M2 can never have, and the whole point of the architecture is that M1 carries to M2.
"Thinking further ahead" therefore has to come from the recurrent state (the internal predictor that
past-only training builds — `project_no_future_curve_shape`) and from the formal predictor line for M2,
not from an input. Recorded so it is not re-proposed.

## 6. The measure: excess over what the path demands (operator 2026-10-05)

> *"Param tuning is something to avoid. Suppose one of these bang-bang controllers was driving the target?
> The chase would have to track. A smooth line in any attitude should fly smooth. An abrupt turn like the
> random paths should turn hard. So the measure is more about wandering around paths."*

That rules out every fixed-threshold form (an `n_cap`, a flip-rate ceiling, a Δu budget): the **baseline is the
path**, and the quantity is the **excess** the craft adds to it. Two parameter-free realisations, both
computable in the scorer's existing per-tick loop — it already derives the path **tangent** per tick
(`fitness_decomposition.cc:236-248`, from consecutive path points in pathgen mode and from
`target.velocity` in tracker mode) and the record carries `getGyroRates()` (rad/s, body) and
`getSpecificForceG()`:

| form | per scenario (lower = better) | what it says |
|---|---|---|
| ⭐ **excess rotation** | `R_craft − R_path`, `R_craft = Σ_t (|p|+|q|)·dt`, `R_path = Σ_t ∠(tangent_t, tangent_{t−1})` | rotate as much as the path turns — no more. A straight path demands ~0; a hard turn demands a lot and is free. |
| **excess load** | `Σ_t max(0, |n_z| − n_req)·dt`, `n_req = √(1 + (v²κ/g)²)` from path curvature κ and rabbit speed | pull as much g as the coordinated turn the path requires — no more. |

Neither has a tunable number. Neither is throttle-gameable. Neither has 015's Δ=0 exploit (a pegged
elevator rotates and loads the airframe). And because tracker mode takes the demand from the **target's
own motion**, a bang-bang target simply raises the demand — the chase is charged only for what it adds.

Measured on t3's 294 training scenarios:

| | median | by path (Straight / Spiral / Fig-8 / 45° loop / HighPerch / RandomB) |
|---|---:|---|
| craft rotation `R_craft` | 47.8 rad | |
| path demand `R_path` | 9.5 rad | |
| **ratio `R_craft / R_path`** | **4.9×** | **5.4× / 4.5× / 3.9× / 3.1× / 6.7× / 5.5×** |
| excess rotation | 37.7 rad, MAD 40%, p10 13 – p90 99 | |
| excess load | 10.3 g·s, MAD 41% | 6.6 / 8.7 / 10.4 / 4.6 / 13.6 / 38.1 |
| corr with `|n_z|` p99 | rotation **+0.69**, load **+0.75** | |

⭐ **The signature is exactly the one described**: the craft does ~5× the rotation the path asks for, and
the *straight-and-level* path is among the worst (5.4×) while the hard 45° loop is the best (3.1×) —
wandering on a line, not working on a turn. Spread is wide (MAD 40%), so lexicase has plenty to select on,
and it exists in **today's** sim (the sim-visibility caveat bites the jerk term, not this).

⭐ **As implemented (2026-10-06, `excess_rotation.h`, ‖ω‖ over all three body axes)** the real code path puts the
t3 genome under the t4 regiment at excess median **47.8 rad**, **rotRatio median 5.10×** — Straight **6.09×** /
Spiral 4.57× / Fig-8 4.03× / RandomA 5.10× / HighPerch 6.92× / RandomB 4.87×. Same signature, same ordering;
these are the numbers of record (the table above was the offline |p|+|q| estimate).

⚠️ Honest baseline: a coordinated turn needs roll-in and roll-out beyond the path's direction change, so a
perfect pilot would not score 1.0× either — the measure is a comparison across genomes, not an absolute
zero. That is fine under lexicase (relative, per case). Of the two forms, **excess rotation** is the primary:
it is literally "wandering around the path", it includes roll (the 029 spiral measurement was this ratio at
10–60×), and it carries no g/v² modelling. Excess load is the sibling in acceleration space if t4 shows
rotation down but loads not.

## 7. The grease — ranked, with what each costs

| # | lever | first-principles role | history | cost | t4? |
|---|---|---|---|---|---|
| **A** | ⭐ **Excess-rotation lexicase axis** (§ 6) — `R_craft − R_path` per scenario, third co-equal case, from gen 0, MAD epsilon, `EnableExcessRotationAxis` default OFF. **No tunable parameter.** | charges what the craft adds over the path's demand — smooth line ⇒ smooth, hard turn ⇒ hard, bang-bang target ⇒ tracked | the 029 spiral diagnosis used this ratio (10–60×); never an axis | small: one accumulator in the existing per-tick loop (`tangent`/`prevTangent` already there; `getGyroRates()` on the record), one `ScenarioScore` field, one `pool.push_back`, tests | ⭐ **yes** |
| **B** | **Command-rate axis with a saturation companion** (the 2025-11 pairing) or **path-relative** Δu (015's revisit note; backlog "Path-Relative Smoothness") | prices the proximate cause | 015 exploit on record for the bare form | small–moderate | t5, if A leaves a command floor |
| **C** | **Setpoint slew limit in the action space**, identical function on xiao and in sim | bounds what a staircase can be | 023 pt3 LPF stunted training | moderate; INAV's `rate_dynamics` is the native analogue, unmodelled in sim | t5 only if needed |
| **D** | ~~Path lookahead~~ | — | — | — | ⛔ **rejected — cheating (§ 5)** |
| **E** | **D1** (T093–T096) | lets the sim ring, so A also charges for exciting the resonance | — | in plan | yes if the fit lands |
| **F** | **Throttle +rail** (handoff § 3 item 3) | the existing energy axis already charges power; t3 railed 85–90% in a headwind | 035 energy axis is this | none — the wind envelope changes the pressure; re-read after t4 | observe |

⭐ **Recommendation for t4: A, excess rotation over the path's demand.** No parameter to tune, no exploit
on record (not throttle-gameable, not Δ=0-gameable), the measured 4.9× has the wandering signature and wide
spread in today's sim, it carries to M2 unchanged (demand from `target.velocity`), and it is lexicase-native so
it cannot collapse the way 033 did. Keep the throttle power axis exactly as it is (it works). Excess load is
the sibling if loads stay high while rotation falls; command-rate (B) stays in reserve, with its saturation
companion.

## 8. "Not dull" — the acceptance, stated before the bake

Lexicase keeps the tracking cases co-equal, so dulling is not built in — but it must be *measured*, and
with rate-fair signals (the 037 caveat):

- **Sim, non-regression (SC-003/SC-005)**: `pctInStreak` and avg target distance within noise of t3 on the
  calm-wind bins; crash rate ≤ t3.
- **Sim, the thing bought**: `R_craft / R_path` median down from **4.9×** (and the straight-and-level path no
  longer among the worst) with `pctInStreak` held; `|n_z|` p99 down from 4.30 g and ticks > 3 g from 12.6%
  as consequences; command flip % falling (watch, don't gate); pitch 2–3 Hz share stays ~10%.
- **Flight, the thing that matters**: pitch-rate power in 2–3 Hz well below 56–81%; `|accSmooth[2]|` p99
  below the 5.2–7.5 g band; pitch command flip % below 52–59%; `pctInStreak` not below t3's flown 54.6%.
- **Servo-demand refs** (027 gate, not physical): slew ≤ 0.27, amplitude ≤ 0.67 — report, don't gate.

⚠️ With the axis on, the t3 genome's fitness under the new pool is a *different number*. Score the t3 genome
under the t4 objective once, record it in the t4 MANIFEST as the baseline, and compare t4 to that — not
to −88,026.62.

## 9. Revisiting the other objectives — "the concern is trades for worse tracking but closer in general"

What the pool now says, per scenario: **`score`** (be in the cone, hold the streak — this *is* "closer in
general"), **`energy_score`** (don't spend power), **`excess_rotation`** (don't rotate more than the path
asks). Crash price multiplies all of it. Nothing else changed. Why this should not trade tracking away:

1. **The precedent is on record.** 035 added `energy` as a co-equal case to a tracking-only pool and tracking
   did **not** collapse — `pctInStreak` went on to the 41.5% record; what moved was throttle. Lexicase keeps
   the tracking cases co-equal; each selection event still demands a winner on tracking a third of the time,
   and the shuffled case order means no genome survives by being smooth and distant. The property is pinned
   by `Selection043.RotationTradeoffBothSurvive`.
2. **The axis cannot be satisfied by lagging.** A craft that falls behind to avoid rotating loses the
   tracking case on that scenario; a craft that holds the cone *without* excess rotation wins both. The only
   genomes the new case removes are the ones that were rotating without buying closeness — which is exactly
   the 5–6× on the straight-and-level path.
3. **The demand is the path, so hard tracking stays free.** Random courses and the 45° loop raise
   `path_rotation`; the measure cannot punish following them. In tracker mode the demand is the target's own
   motion — a bang-bang target is tracked, not fled.
4. **Where a real trade would show, and what we would NOT do.** If t4's calm-wind `pctInStreak` falls below
   t3's while `rotRatio` falls, that is a trade and T106 fails. The response is **not** a weight or an
   epsilon — there are none to turn — but to look at the *demand side*: is the path tangent too jagged on
   some generator (Catmull-Rom at 0.02 steps is smooth; the analytic helpers are piecewise and may put a
   corner where a pilot would blend), or is `‖ω‖` charging yaw a coordinated turn needs that the tangent does
   not count? Both are measurement questions with no knob.
5. **Energy and crash price are untouched**, and the throttle `+rail` (handoff § 3 item 3) is re-read after
   t4: the wind envelope changes the pressure, and the energy axis already charges power.

⚠️ With wind now 0–8 m/s, scenario difficulty varies far more than in t3; the per-case MAD epsilon absorbs
that per scenario, which is what it is for. Report t4 binned by realized wind, not pooled.
