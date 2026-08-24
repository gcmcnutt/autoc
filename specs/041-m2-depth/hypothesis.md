# 041 — hypothesis + approach (pre-spec)

**Status**: PRE-SPEC design memo, written 2026-08-06 against [README.md](README.md) (the 040-wrap seed)
and [BACKLOG.md](../BACKLOG.md). Fed `/speckit.specify`. Filename deliberately avoids the speckit
output names (`spec.md`, `plan.md`, `data-model.md`, `quickstart.md`, `contracts/`) — 040 paid for that
collision once.

> ## ⚠️ SUPERSEDED WHERE IT CONFLICTS — `spec.md` Clarifications govern
>
> This memo remains the **derivation of record**: it is the only place the *reasoning*, and especially the
> **retractions**, are written down. Read it for **why**. Do not read it for **what** — two clarify sessions
> (2026-08-07, 2026-08-10) reversed several conclusions reached here, and where the two disagree,
> [spec.md](spec.md) § Clarifications and [research.md](research.md) win.
>
> Reversals since this memo was drafted, with the section each affects:
>
> | this memo argued | actually decided | see |
> |---|---|---|
> | B2 carries the streak input **plus a load axis** (§7) | **No load axis in 041.** One behavioural lever only | spec FR-019/FR-020 |
> | A3 gates the bundle — "decides whether a load axis is needed" (§5) | **A3 is report-only**; it feeds the follow-on feature and gates nothing | research R4 |
> | Predictor target = **reappearance geometry** across a gap (§4, §5 C2a) | **Continuous current-bearing estimate** — horizon-free state estimator; the reappearance framing still hid an unknown-gap-duration horizon | spec FR-025 |
> | Head scored passively | **Actuated** — prediction *error* fed back as an input | spec FR-025c–f |
> | *(silent on)* camera field | **V 90° → 75°** to match ordered hardware, M2-only | spec FR-029 |
> | `streak_proxy` (§3) | canonical names are **`IN_ENVELOPE`** and **`ENVELOPE_SECS`** | data-model §1.1 |
>
> Stale passages below carry inline `⚠️ SUPERSEDED` markers pointing at the governing requirement.

---

## 0. Two corrections to the seed, both checked in code

### 0.1 M1 does NOT carry the one-tick defect. The fix is tracker-only.

The seed says *"M1 has been optimizing the same wrong target since 030."* It has not.

`68f64ab` touches exactly one branch — `if (is_tracker)` in
[fitness_decomposition.cc:203-231](../../src/eval/fitness_decomposition.cc#L203-L231), where the rabbit
comes from the **parallel array** `targetTrajectoryList[i][stepIndex]`. That is the array with 367
entries against 368 states, and that is the whole bug.

M1 takes the `else` branch ([:241-244](../../src/eval/fitness_decomposition.cc#L241-L244)) and reads
`stepState.getThisPathIndex()` — an index **carried on the state object itself**, stamped in the tick
loop at [inputdev_autoc.cpp:909](../../crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp#L909)
immediately after the rabbit odometer advances, into the same `AircraftState` that is pushed at the end
of that tick. There is no second index to be off by. The pre-loop initial push carries `pathIndex = 0`
and the objective's `while (++stepIndex …)` skips index 0 entirely.

**Consequence for the spec**: "the one-tick fix invalidated the pinned M1" is not a reason to rebake M1.
The real reasons are in §1 and they are good ones — but the justification has to change, because a spec
built on a false premise will be judged against the wrong success criterion.

### 0.2 E1 (ablate the predictor head) cannot run on M1.

The backlog entry says *"Run on M1, which climbs fast and reliably at pop 3000."* The predictor head is
**tracker-only** — `kNumSpanAuxOutputs` lands in `TRACKER_NN_OUTPUT_COUNT` only
([topology.h:104-110](../../include/autoc/nn/topology.h#L104-L110)), and `autoc.cc:1668` says it
outright: *"harmless in pathgen where prediction_score is 0 for all candidates."* An axis that is
identically zero for every individual is inert, not ablatable — an M1 E1 measures nothing.

**E1 is a cheap M2 run**, not an M1 run. It stays first (it is still the cheapest thing that can kill
the branch) but it must be budgeted as tracker-mode wall-clock at reduced scale, not as hours.

---

## 1. What actually justifies redoing M1

Three reasons, none of which is the one-tick fix:

1. **Bundling.** 041 wants at least one M1 **format-breaking** change (the streak accumulator, §3).
   `NNInputs` is a serialization contract — declaration order *is* on-disk byte order for data.dat,
   cereal, `nn2cpp`, and `sim_response.py` ([nn_inputs.h:93-96](../../include/autoc/nn/nn_inputs.h#L93-L96)).
   Any input change forces a rebake and re-flashes xiao. If M1 is being rebaked anyway, **every pending
   M1-incompatible change lands in one commit, once.** This is the operator's "now is the time" and it is
   the strongest argument on the list.
2. **The comparator is already expired.** The 038-t9 elite cannot be loaded by a current binary
   (040 deferral). A pinned dmp preserves *numbers*, not a *controller*. Whatever 041 pins must archive
   `nn_weights*.dat` (NN01, stable, ~11 KB) beside the dmp — option 2 of that deferral, adopted here.
3. **Regime honesty.** [[project_m1_basin_lottery_actual_rate]] — M1 climbs reliably at pop 3000 /
   single longSequential / 16 winds and unreliably at pop 8000 / 49 winds (0/5 recently). The spec must
   *state which regime each M1 run uses and why*, rather than inheriting 8000/49 by habit.

---

## 2. The thesis

> **Every remaining M2 lever is a variant of one gap: the controller cannot perceive the reward
> machine it is being scored by.**

Two independent eliminations already point here. 037 says depth is **reward-invariant** — it is not the
objective's scale ([[project_m2_tracking_ceiling]]). 040 says it is **not perception fidelity** — a much
harder, much more honest camera moved competence not at all. What is left is the coupling *between* the
two: what the controller can observe about how well it is doing.

The gap is concrete and measurable, not a metaphor:

- **The streak multiplier is the dominant fitness shaping** — `FitStreakMultiplierMax = 5.0`, ramping
  over `FitStreakRampSec = 5.0 s` ([fitness_computer.cc:59-67](../../src/eval/fitness_computer.cc#L59-L67)).
- **The NN's entire perceptual history window is 0.8 s** (`kNNHistoryLagsMsec = {800,400,200,100,50,0}`).
- So the reward is conditioned on **6× more history than the policy can observe.** From the NN's seat
  this is a Markov violation: it cannot represent "how deep into the pocket am I", and that is precisely
  what 80% of the available fitness pays for.

Note what the NN *can* already do. The instantaneous in-envelope test is a function of `dist` and the
`target_*` bearings — inputs it already has. **The flag is computable; the integral is not.**

> ⚠️ **SUPERSEDED 2026-08-07 (research R2)** — this paragraph concluded the *accumulator* was therefore the
> missing quantity. Correct about **information**, wrong about **decision relevance**: the multiplier is
> monotone in duration, so with no competing cost in 041's objective there is no state where leaving the
> envelope is preferable. The optimal policy is "stay in", which needs the **flag**, not the depth.
> **The flag is the primary input**; the accumulator is secondary and becomes decision-relevant only once a
> load or energy cost exists. Both slots still ship (one format break), and ablation separates them.

### H1 — the pocket hypothesis (this is the falsifiable claim)

**Control aggressiveness is not a smoothness deficiency. It is the rational policy for a controller that
cannot perceive that it is already winning.** A net with no representation of "I am deep in a scoring
streak, hold still" has no way to distinguish *hold* from *hunt*, so it hunts always, everywhere. Calm is
not currently a *representable* behavior, which is why every attempt to penalize aggressiveness into
existence has failed — 033 phase 1 died in a Pareto corner doing exactly that
([[project_scalar_multiobjective_collapse]]).

This makes aggressiveness a **symptom**, and predicts it falls out for free once the state exists —
no smoothness axis, no penalty coefficient. That is an objective-**presence** change, which is the only
kind [[feedback_clear_objectives_not_tuning]] permits.

**Predictions (H1 is wrong if these fail).** The instrument column matters — see §5 for why only the
ablation row is a controlled measurement:

| observable | H1 predicts | instrument |
|---|---|---|
| zeroing the streak inputs on the trained elite | **degrades tracking / restores bang-bang** | input ablation, within-build (**rigorous**) |
| `pctInStreak`, `avgMaxStreak` | **up** | ballpark vs historical band |
| per-axis `dCtrl`, `⟨\|u\|⟩` (`per_axis_aggressiveness`) | **down**, with no smoothness objective added | ballpark vs historical band |
| bang-bang axis | may migrate rather than vanish ([[project_bangbang_axis_migration]]) — migration still counts as movement | ballpark |
| effect size M1 vs M2 | **larger on M1** (in-envelope far more often ⇒ the accumulator is informative more of the time) | ballpark, both modes |

**Partial outcomes are informative, and the spec should name them in advance:** ablation bites but
aggressiveness stays flat ⇒ the input is used, for tracking rather than for calm (H1 half-right, and calm
needs its own lever). Ablation does nothing ⇒ the net ignored the accumulator, H1 is dead, and thread 2
becomes the whole feature.

### H2 — the drag-tax hypothesis (operator 2026-08-06)

**Throttle saturation is not an independent bang-bang axis. It is the energy cost of bank and pitch
thrash.** Operator's chain: aggressive control gets the craft *hard back on track*, at the price of a lot
of drag, which is paid in throttle. And the throttle is not "mostly dead" for a pathological reason —
**the craft is behind the target, so "go for it" is the correct command.**

If that holds, then penalizing throttle treats a symptom and penalizing bank/pitch *rate* treats the
cause — and the two produce opposite outcomes on a chase that is legitimately behind.

**H2 is testable offline, for free, before any axis is designed.** `dmp-dump`'s per-tick CSV already
carries `out_pt, out_rl, out_th, stpPt, dist, mult` for both modes, so on any pinned run (040 t4, the
pinned M1) we can ask directly: *within a run, per regime, does `dCtrl` on pitch/roll predict throttle
level?* Strong within-regime correlation supports H2 and makes bank-rate the axis to build. Weak
correlation says throttle is its own phenomenon and the energy question stands on its own. No bake, no new
tooling — this is Phase A work (§5 A3).

### The per-axis reality — and why "smoothness" is the wrong frame (operator 2026-08-06)

Standing verdict on the whole class first: **smoothness factors have been tried for a while. They do
deaden the paths — and they are generally beaten by aggressive manoeuvres.** That is not a
parameterisation failure, it is a competitive one: in a fitness contest, aggressive wins. So the answer
is not a better-shaped smoothness penalty, and this memo's earlier "redefine it as a rate axis" is
still a smoothness penalty wearing different units.

What changes the picture is the per-axis physics, because the three axes are *not* the same phenomenon:

| axis | what is actually going on | implication |
|---|---|---|
| **throttle** | **always tends high because the chase is the same aircraft model as the target.** A twin-airframe tail chase needs max performance to close. | **Structural to the experiment, not a defect.** The fix is different craft (already a backlog item), never a penalty. This kills the energy axis as an aggressiveness lever outright. |
| **roll** | smooths out **only because it is a well-damped axis on a planar-wing craft** | Roll calmness is **plant, not policy** — it is not evidence of good behaviour and not a lever. |
| **pitch** | **bang-bang, and rough on the whole airframe — ±11 g** | **This is the axis that matters**, and its cost is *structural load*, not roughness. |

That reframes the objective from an aesthetic one to a physical one: **the problem is not that pitch looks
rough, it is that pitch is loading the airframe to ±11 g.** Flight data agrees and is trending the wrong
way — routine ±7–8 g, a **+11.2 / −8.4 g record on 2026-07-20 on an identical policy**, loads creeping up
flight-over-flight with "inspect airframe between flights" already in the record.

A load objective has the property every smoothness penalty lacked: **a real limit instead of a tuned
coefficient.** An airframe has a g-limit; that is a clear, achievable objective in the sense
[[feedback_clear_objectives_not_tuning]] requires, not a knob traded against tracking. It is also the one
version of this that a maximally-aggressive controller cannot simply out-compete, because exceeding it is
a physical failure rather than a lower score.

**And the measurement is already recorded, unread.** `PhysicsTraceEntry` carries per-tick `acc[3]`,
`omegaDotBody[3]`, `alpha`, `vRelWind`, populated for **every elite reeval**
(`inputdev_autoc.cpp:1047`) and serialized into every gen dmp — and **nothing in `tools/` or `src/`
reads it.** Body-frame normal acceleration follows from `acc[]` + `quat[]`. So per-regime, per-axis load
is answerable **offline, today, on any pinned run**, with no recording change and no bake.

#### The load objective needs accelerometer INPUTS — same gap, second instance (operator 2026-08-07)

Checked: **neither input struct carries linear acceleration.** Both `PathgenInput` and `TrackerInput` have
`GYRO_P/Q/R` — *angular rate* — and nothing else in that family
(`nn_inputs.h:172`, `:227`). So a load objective without an accelerometer input would repeat H1's mistake
exactly: **penalize a quantity the policy cannot perceive.** The NN would be selected against ±11 g while
having no way to tell 1 g from 11 g.

So the pairing is structural, and it is the same structural insight applied twice:

| | objective side | observation side |
|---|---|---|
| tracking | streak multiplier (exists) | **streak/regime input (missing)** |
| load | ~~load axis (proposed)~~ → **deferred to the follow-on feature** (spec FR-019) | **accelerometer input (missing)** — still ships in 041 (spec FR-017a) |

- **Deployable, not a sim privilege.** Accelerometers are the one sensor this airframe unambiguously has —
  INAV provides it, and the xiao carries an LSM6DS3 ([[project_xiao_imu_crosscheck]]). This is a *more*
  deployable input than most of what the NN already reads.
- **Feed specific force, not kinematic acceleration.** A real accelerometer measures specific force —
  including gravity — so the sim must feed `acc − g` rotated into body frame to match what hardware
  reports. Feeding FDM kinematic acceleration would train against a signal no accelerometer produces, and
  the sim-to-real gap would be a *constant 1 g offset in the most load-relevant axis*. This is the kind of
  detail that has cost this project flights before; it belongs in the spec, not the plan.
- **Cost**: +3 slots body-frame (or +1 if normal load alone proves sufficient — a design call, and
  A3 can inform it from the recorded `acc[]`). `PathgenInput` 37→40, `TrackerInput` 58→61, both
  in the same A1 break as the streak input. Noise/bias modelling is a separate question the spec should at
  least name, since a raw sim accelerometer is unrealistically clean.

**Total energy is the adjacent currency**, and the operator's instinct that we may be back to it looks
right: "aggressive = more energy" makes energy the natural cost unit, and the deferred *Total Energy
Management + Altitude-Aware Distance* backlog item (energy as NN input or lexicase objective, above-target
better than below) was written for exactly this question. It moves from parked to plan-phase adjacent —
but note that energy-as-*cost* and throttle-as-*penalty* are different things, and only the first survives
the twin-airframe point above.

### H3 — aggressiveness is regime-conditional, not globally good or bad

Operator's read, which is sharper than "reduce aggressiveness":

| regime | aggressiveness | why |
|---|---|---|
| **patrol** | **good** | nothing to be efficient about yet; cover volume, reacquire |
| **intercept** | **bad** | this is the long closing phase where drag is actually paid |
| **tracking** | **possibly necessary** | following a manoeuvring target needs authority |

So the objective wants *"some weighted mode of flight considerations"* — a penalty that knows which regime
it is in, rather than one global smoothness term. A global term is wrong in two of the three regimes,
which is a decent explanation for why every attempt at one has failed.

**The regime partition already exists, and it is the same partition as the streak machinery.**
`src/analytics/dynamics_progress.py:74-80` has carried it since 2026-06-10:

- `tracking` = `stpPt ≥ TRACK_THRESHOLD` where **`TRACK_THRESHOLD = 0.5` — the identical value as
  `FitStreakThreshold`**, so *in-tracking and streak-maintaining are the same condition by construction*
- `intercept` = below threshold **and closing** (smoothed `d(dist)/dt < 0`)
- `patrol` = below threshold, not closing

Which means the NN can already compute the *instantaneous* regime: `stpPt` is a function of the
along/lateral offset it perceives, and `closing_rate` is **already an input**. What it cannot compute is
**how long it has been in the regime** — the same integral H1 identifies. One quantity, two uses: the
regime signal is the NN's missing **input** (H1) *and* the objective's missing **gate** (H3).

### The threads are sides of the same gap

- **Streak accumulator** = the reward machine's **observed past**, hand-computed. (Input side.)
- **Value head** = the reward machine's **predicted future**, learned. (Output side.)

They are a ladder, not two experiments. The accumulator is the cheapest possible hand-built fragment of a
value function; if hand-building it helps, learning the rest is the natural next step, and if hand-building
it does nothing, the value head is a much worse bet. Sequence accordingly.

---

## 3. Thread 1 — the streak accumulator (M1 and M2)

Design is already in the backlog ("Streak/in-envelope input", filed 2026-07-09); this memo adopts it and
sharpens the M1 side, which that entry left as tracker-only.

**Do not feed the fitness machinery's true streak state** — that is a sim-only privilege and violates the
direct-observable principle. Derive it from what a real airframe can compute:

- `in_envelope(t)` — the observable geometric test (M1: rabbit distance + bearing; M2: both beacons
  CEP-visible AND span in `[lo, hi]` AND centroid within a centered radius).
- `ENVELOPE_SECS(t)` = `min(consecutive-in-envelope seconds / FitStreakRampSec, 1)`, reset on dropout.
  *(Drafted here as `streak_proxy`; the canonical names are `IN_ENVELOPE` and `ENVELOPE_SECS` — data-model §1.1.)*

Two new inputs. **ms-based, so cadence-invariant** — the same discipline as `kNNHistoryLagsMsec`. The proxy
need not match `stpPt` exactly; it needs to *correlate*, and the net learns the mapping.

#### M1 gets the exact signal; M2 needs an estimator (operator 2026-08-06)

This is the cleanest staging argument in the feature, and it is why M1 carries this thread:

- **M1 can use the real thing.** Distance to the rabbit and in-track status are *directly available* — the
  rabbit is a computed waypoint, so `stpPt ≥ FitStreakThreshold` is exact, and it stays exact on real
  hardware where the rabbit comes from GPS. No estimator, no proxy error. **M1 tests the hypothesis
  cleanly**: if an exact in-streak input does not change behaviour, no estimator will.
- **M2 needs an estimator**, because in-envelope has to be inferred from perception — and 040 made
  perception materially harder in six directions, camera mount misalignment among them. So the M2 port
  carries estimator error *on top of* whatever H1 is worth, and a flat M2 result would be ambiguous
  between "the signal doesn't help" and "the estimate is too noisy to use."

**Consequence for sequencing**: M1 is not a convenience proving ground here, it is the *only* place the
hypothesis can be tested without an estimator confound. **The M2 estimator design is explicit plan-phase
research** (operator), not an implementation detail — and it should be scoped only after M1 says whether
the exact signal is worth estimating.

**H3 widens this slightly, at no extra cost.** Since the regime partition is
`{tracking, intercept, patrol}` and the NN already carries `closing_rate`, the honest signal is
*regime occupancy* rather than streak alone: which regime, and how long in it. `in_envelope` already
distinguishes tracking; the intercept/patrol split is `closing_rate`'s sign, which the net has. So the
input set stays **two slots** — the envelope flag and the normalized duration — and the duration is what
carries the new information in every regime, not just the tracking one. Whether the duration accumulator
should reset on regime *change* rather than only on envelope dropout is a design detail for the spec.

**Cost and blast radius:**

| | M1 (`NNInputs`) | M2 (`TrackerInput`) |
|---|---|---|
| count | **37 → 39** | 58 → 60 |
| rebake | **yes — this is the format break** | yes, but no M1 source rebake (T023 split `AircraftState` serialize was built for this) |
| xiao | `nn2cpp` regen + reflash | n/a |

---

## 4. Thread 2 — the predictor, recast

### The purpose fixes the target (operator 2026-08-07)

> *"Predictor is mostly to help re-acquire best we can from target out of camera FOV."*

That settles what the head should predict, and it is **not** span at +50/100/150 ms. Those horizons were
chosen for actuation lag; the *purpose* is bridging blindness. So the target becomes **where the target
will reappear** — reappearance bearing (and span) after a blind gap — at **blindness-scale horizons
(≈0.5–2 s)** rather than actuation-scale ones.

> ⚠️ **SUPERSEDED (spec FR-025)** — this target still hides an unknown gap duration: at prediction time the
> net does not know *when* reacquisition happens. The decided target is a **continuous current-bearing
> estimate**, horizon-free. The three reasons given below for why this beats span are sound and unchanged;
> only the target definition moved.

This is the 038 re-target design, and the operator's statement selects it over the alternatives. Three
reasons it is the right target where span was not:

1. **Persistence collapses across a blind gap.** That was the whole problem with span: the no-change
   baseline was already right to within 15%, so a perfect head added nothing. Across seconds of blindness,
   "assume unchanged" is badly wrong, and there is real information to carry.
2. **The information genuinely exists.** A target leaving frame at a known bearing rate, banking at a known
   attitude, is *predictive* of where it re-enters. This is a learnable quantity, unlike Δspan whose
   measured `r(Δ)` was ≈ 0 at every horizon.
3. **It aims at the documented bottleneck** — reacquire-through-blindness, with worst-case blind windows
   near 8 s ([[project_m2_tracking_ceiling]]). The current objective **CEP-gates both endpoints**, so blind
   gaps are *excluded* from scoring: the head is scored only in the regime where prediction is
   information-free and skipped in the one where it would pay.

⚠️ Note that the current scoring excludes exactly the regime the purpose cares about. The re-target is
therefore not a tuning change but a change in *which ticks are scored at all* — visible→reacquisition
pairs, not visible→visible.

#### The payoff is an optics budget, and that is what makes this worth doing

> *"If we can get this, fantastic — we will consider FOV down to 90° for example. Options."*

This reframes the predictor from a control aid into a **perception-budget lever**, which is a far stronger
justification than anti-overrun lead. Narrowing the field trades coverage for range and resolution, and the
trade is quadratic in SNR (∝ 1/FOV²):

| FOV | vs 120° SNR | angular resolution @ 320 px |
|---|---|---|
| 120° (today) | — | 2.67 px/deg |
| **90°** | **+2.5 dB** | **3.56 px/deg (+33%)** |

Against the standing ~24 dB shortfall at 100 m (040 `optics-record.md`), +2.5 dB is modest but real — and
the resolution gain is immediate. The point is the *direction*: better reacquisition is what buys
permission to narrow the field, and narrowing is one of the few levers on a range budget that is currently
short. The 120°-vs-range tension is already documented as the central optics conflict (BACKLOG Question 2c:
*you cannot have 120° FOV and 100 m range on a 320×240 sensor*).

**FOV reduction is an option 041 enables, not 041 scope.** It is a downstream feature, and it needs the
reacquisition result first.

### The alternative if reacquisition does not clear — a value head

Retained as the fallback rather than the lead. If C2 says the current-bearing estimate is not learnable either, the
remaining idea is to stop predicting the world and predict *value*: discounted future accumulated
`stepPoints`.

- **The horizon becomes a discount γ, not a hyperparameter to pick.**
- **Prediction accuracy IS objective accuracy** — no proxy gap, which is what killed the span head.
- **The Monte-Carlo target costs zero extra simulation** — `fitness_decomposition.cc` already computes
  `stepPoints` per tick; the target is a backward pass over a trace we already record.
- **It ports to M1 unchanged**, where a span head cannot go at all (no beacons). That matters: it makes
  thread 2 testable on the fast, reliable learner instead of only on 27 h M2 bakes.

**Three traps to design around, all already paid for:**

1. **Score Δ against a no-information baseline, never a level.** The span head's `|e|` metric conflated
   offset error with information content; persistence was already right to within 15% and the head's
   `r(Δspan) ≈ 0` at every horizon. The value head's equivalent baseline is **constant-mean V**, and the
   reported statistic is **r²**, not mean error.
2. **Normalize into the output's domain.** 040-t1's head was raw bounded `tanh` against a 0.049-rad
   target — ~95% of the range wasted, and the error curve became a *saturation readout*, not a skill
   readout. Discounted `stepPoints` is bounded by `max_points/(1−γ)`; normalize by it so the target
   occupies the range.
3. **Do not make it a lexicase axis on faith.** The span axis was one of three × 294 scenarios — a third
   of the pool selecting on `r² ≈ 0`, which is *actively harmful*, not inert. A new head earns an axis by
   clearing §5 A3 first.

---

## 5. Approach — four phases (operator scoping, 2026-08-06)

**Thread split, per the operator**: *M1 tests aggressiveness* (the rerun is owed anyway, so it carries a
lever), *M2 tests the predictor*. Production M1 regime is **pop 8000 / 49 winds**.

### Phase A — backlog

| | what | gate |
|---|---|---|
| **A0** | **Research phase — the index-parallel scan.** Operator-mandated, own output: an inventory of index-coupled contracts and structs with two lifetimes, plus a decision on the grouped-record (`tickList[i][k] = {state, cameraView, targetSample}`) refactor. | Before any objective change lands. |
| **A1** | **The format-break bundle, one commit**: streak accumulator inputs (§3), grouped-record refactor if A0 recommends it, `wind_velocity` recording, self-describing dmp — whichever of those A0 promotes from candidate to commitment. Plus the `nn_weights*.dat` archival practice. | Clean build + full test suite (Constitution IX); nothing lands while a bake is live. |
| **A3** | **Offline regime / load study — no bake.** On the pinned M1 and 040 t4: (a) split ticks into `{tracking, intercept, patrol}` on the existing `stpPt ≥ 0.5` + closing rule; (b) **read the load out of `PhysicsTraceEntry`** — per-tick `acc[]` + `quat[]` → body normal acceleration, per axis, per regime. It is recorded for every elite and has **no consumer today**, so this needs a small reader (extend `dmp-dump` with physics columns), not a recording change; (c) test H2 — within regime, does pitch/roll `dCtrl` predict throttle level and load; (d) report per-regime `dCtrl` / `⟨\|u\|⟩` / peak-g so H3 is a measurement rather than an intuition. ⚠️ **SUPERSEDED — A3 is REPORT-ONLY** (research R4): 041 builds no load axis, so A3 gates nothing. Its findings are input to the follow-on aggressiveness feature, sequenced after the M2 predictor go/no-go. | Runs today; gates nothing. |
| **A2** | **Genome ablation tool, minimal scope** — `--zero-input <named columns>` + Δ fitness / per-axis Δ`dCtrl` / Δ`⟨\|u\|⟩` reporting ([BACKLOG.md](../BACKLOG.md) entry, currently unbuilt). **This is a dependency, not a nicety**: it is the only rigorous instrument in the feature (§5). Leave `--zero-whh` and the rest of the mask vocabulary unbuilt. | Must exist before B2's elite is read. |

### Phase B — M1 experiment bake (aggressiveness)

- **B1 — smoke tests, not comparison arms** (operator 2026-08-06). The topology change is **generic** —
  it lands in both `NNInputs` and `TrackerInput` — so the smoke set must prove it runs in **both modes**:
  a pop-3000 M1 run and a short tracker run. What is being checked is plumbing and ballpark sanity (does
  it build, run, climb at all, produce sane inputs), *not* a delta against anything. Recent runs have
  shown jumping straight to a prod build is fine, so keep these cheap and add more only if something
  looks off.
- **B2 — production bake at pop 8000 / 49 winds.** The artifact: the new pinned M1 and the M2 source.
  Archive its weights.
- **B2′ — repeat if it does not climb.** 8000/49 has a documented ~0/5 recent climb rate
  ([[project_m1_basin_lottery_actual_rate]]). Judge climb by `pctInStreak`/`avgMaxStreak`, not completions
  ([[project_servo_era_progress_metrics]] — completions are a red herring in the servo era). The repeat is
  a lottery re-draw, not statistical power.

### The M2 constraint (operator 2026-08-07)

**M2's job in 041 is one question: does the predictor produce any signal — predictor or not.** Everything
else about M2 is expected to arrive for free:

- **M2 is already a good learner.** Every M2 run to date climbs fast and settles at about the same
  competence regardless of what was changed — that is the standing observation behind
  [[project_m2_tracking_ceiling]], and it means M2 does not need help *learning*, it needs something worth
  learning from.
- **Fixing aggressiveness on M1 should carry most of the way into M2 by itself**, through two paths at
  once: the streak input and any load objective are **generic** (both `NNInputs` and `TrackerInput`), so
  M2 trains with them too; and M2 chases M1's *recorded trajectories*, so a calmer, less
  load-spiking source is a materially easier and more physical thing to chase.

So M2's aggressiveness read is a **free observation** — did it inherit the M1 gain — not a designed
experiment, and Phase D is a **predictor** experiment. If the answer is "no signal", then dropping the head
(topology 7→3, reclaiming 119 output weights and a third of the lexicase pool) is a legitimate and
complete deliverable.

### Phase C — M2 predictor work (no bake)

- **C1 — E1: ablate the span head.** `EnablePredictorHead` 0 vs 1. Cheap tracker run, **not** an M1 run
  (§0.2). Prior is that tracking *improves*, since it returns a third of the lexicase pool from a channel
  with `r² ≈ 0` to axes that mean something. If E1 wins, that is a finding on its own.
- **C2 — offline feasibility, minutes, no simulation.** Regressions on recorded per-tick traces, each
  scored as **r² against a no-information baseline**, never mean error:
  - *(a)* ⚠️ **SUPERSEDED (spec FR-025)** — drafted as *"reappearance bearing across a blind gap"*, which
    still carried an unknown-gap-duration horizon. The decided target is a **continuous current-bearing
    estimate**: every tick the head emits "where I believe the target is *now*", scored against truth wherever
    truth exists, dead-reckoning through gaps, **horizon-free**. Scored as r² vs **hold-last-seen**, binned by
    blind-gap age. This is the head's actual purpose and the cheapest possible test of it.
  - *(b)* Δspan at the old horizons (E2) — run only as the control that confirms *why* the old head
    failed. ⚠️ Confounded until the `prediction_score` one-tick pairing is fixed; that fix is Phase A work.
  - *(c)* discounted future `stepPoints` — the value-head fallback, vs constant-mean.
  **Free bonus from the same study**: the blind-gap distribution itself — how often, how long, and at what
  bearing offset the target re-enters frame. That is the input to any future FOV-narrowing decision, and
  nobody has measured it.
- **C3 — design the head that C2 endorses**, or retire the head entirely (topology 7→3) if neither clears.

### Phase D — M2 bake (the predictor question)

One ~27 h tracker bake off the new B2 source, carrying the C3 predictor design — **or carrying no head at
all**, if C1/C2 say the branch is dead. Scope it to the predictor question; do not add an M2-side
aggressiveness lever (see the M2 constraint above).

Reads, in priority order:
1. **Predictor signal, or its absence** — the deliverable. Judged on r² against a no-information baseline
   (§4), never on mean error.
2. **Did the M1 aggressiveness fix carry through** — free observation, per-axis reports + peak load.
3. **Novel-geometry generalization** — the T085 gap (15.3% → 8.4% inside 5 m) is the number 041 exists to
   move. ⚠️ With T085's source left to expire (§6.3), this now rests on the single pinned 038-t10 set, so a
   difference cannot be cross-checked against a second independent 49-scenario sample.

### How H1 is actually tested — there is no control bake

**Operator correction 2026-08-06, and it is the right one**: there is nothing at the micro level to A/B.
PRNG, model and fitness all change between runs, so a cross-run delta is not a measurement of the lever —
it is a measurement of everything at once. Bit-replay is a within-build property
([[feedback_replay_scope_within_build]]). The bar 040 actually used is the honest one: **is it in the
ballpark** — 040 showed no significant regression from 039/038, and that was the finding.

So H1 splits into two claims with two different instruments, and only one of them is rigorously testable:

**H1a — mechanism: does the learned policy USE the streak state?** Rigorously testable, *within build*.
Take B2's trained elite and **zero the streak input columns**, then re-eval on the identical scenario set
with identical seeds. This is the [genome ablation tool](../BACKLOG.md) (`--zero-input`), already designed
to report exactly what H1 needs — Δ fitness, per-axis Δ`dCtrl`/Δ`⟨|u|⟩`, per-scenario Δ histogram. Same
build, same weights, one variable. If zeroing the accumulator degrades tracking or *restores* the
bang-bang signature, the state is load-bearing and H1a is confirmed. If nothing changes, the net ignored
the input and H1 is dead on the spot — for the cost of an eval, not a bake.

**H1b — outcome: does aggressiveness fall while tracking holds?** Only answerable in the ballpark sense.
Read B2's per-axis reports against the **historical band** (038/039/040-era `dCtrl`/`⟨|u|⟩`) and require
*no significant tracking regression* plus movement of the aggressiveness numbers in the predicted
direction. Not a controlled delta, and the spec should not pretend otherwise.

Worth stating plainly, because it is the limit of the whole design: ablation tells you whether the trained
policy *depends* on the input. It cannot tell you what training would have produced *without* it. That
question needs a control bake, and we have decided not to buy one.

**This makes the ablation tool a Phase-A dependency**, not a nice-to-have — it is the only rigorous
instrument in the feature. Scope it minimally: `--zero-input` on named columns is enough; `--zero-whh`
and the rest of the mask vocabulary can wait for the research question that needs them.

**Ordering constraint from 040's trap 2** — *land objective changes BETWEEN baselines, never between a run
and its comparator*: the entire A1 bundle lands in one commit, before B1 starts, and nothing lands during
a live bake ([[feedback_no_rebuild_during_training]]).

---

## 6. Prerequisites — the backlog sweep

Full read of [BACKLOG.md](../BACKLOG.md), the in-code `TODO`s, and 040's outstanding-at-wrap list. Items
are grouped by *why they are prerequisites*, not by where they came from.

### 6.0 The find: the aggressiveness objective is already in the tree, commented out

Two lexicase axes were plumbed for 027 v3/v4 and disabled behind `CADENCE7-REDUX` markers. They are not
proposals — they are **restore-by-uncomment**:

| axis | term | plumbing | restore |
|---|---|---|---|
| **C2 stability** | `Σ_t (\|out_pt\|−1) + (\|out_rl\|−1)` per scenario | `ScenarioScore::stability_score`, `fitness_decomposition.cc` + `selection.cc` | uncomment `pool.push_back({s, &ScenarioScore::stability_score, 0.5})` |
| **C2 energy** | `Σ_t (out_th−1)/2` per scenario | `ScenarioScore::energy_score`, same pattern | same pattern |

What is valuable here is the **plumbing** — `ScenarioScore` fields, the per-scenario accumulation in
`fitness_decomposition.cc`, the lexicase pool wiring in `selection.cc`, and the 4 `DISABLED_` tests in
`tests/selection_tests.cc:152+` that gate exactly this code. Two further points in their favour:

1. **The grouping is already right.** Stability is `pitch + roll` in one term, energy is throttle alone —
   *exactly* the physical grouping [[project_smoothness_axis_grouping]] concluded was correct.
2. **They are lexicase axes, not scalar penalties**, so they sit on the right side of
   [[project_scalar_multiobjective_collapse]] — the trap that killed 033 phase 1 on this problem.

They also compose with thread 2: **E1 frees a third of the lexicase pool from a channel with r² ≈ 0, and a
working aggressiveness axis is what should fill it.**

#### ⚠️ But the TERMS are wrong, and the earlier draft of this memo got it wrong too

Both axes penalize **amplitude**: `Σ(out_th − 1)/2` is throttle *level*, and
`Σ(|out_pt|−1) + (|out_rl|−1)` is bank/pitch *magnitude*. Under H2 and H3 that is the wrong statistic in
two independent ways:

- **It punishes correct behaviour.** A chase that is behind the target *should* go to full throttle
  (H2). A chase following a manoeuvring target *should* use bank authority (H3). An amplitude penalty
  taxes both.
- **It cannot see the actual pathology.** The stuck-basin signature is throttle **lock** — exactly
  1.000 with **σ = 0.000** — and the backlog's own r1/r2/r3 read is explicit that *saturation is not the
  tell*: r1 and r2 both passed **through** mean ≈ 0.85 with σ > 0 and escaped; only r3 locked. A level
  penalty cannot distinguish them, and it penalizes the climbers *harder*, because they spend productive
  time at high throttle.

So: **I retract the earlier recommendation that the energy axis ride along with B2 as basin insurance.**
As written it attacks mean level, which is neither the pathology nor a defect. If basin insurance is
wanted, the discriminating statistic is **variance / rate, not level** — an axis that rewards throttle
*modulation* fails σ = 0.000 instantly and leaves "go for it" alone. That is a small redefinition of the
existing term, using the same plumbing.

This also lines up with two standing findings: 037 migrated bang-bang to throttle *because the servo got
real* — throttle is the un-lagged actuator, so throttle misbehaviour is a **rate** phenomenon
([[project_bangbang_axis_migration]]); and the ABANDONED pt3-filter experiment showed mechanically
constraining command *amplitude* stunted training, with the conclusion that the NN must learn smoothness
through fitness.

#### What to build instead — and it is not a smoothness axis

Given the standing verdict in §2 (smoothness factors lose to aggressive manoeuvres) and the per-axis
physics, the two candidates that are **not** smoothness penalties are:

1. **The streak / regime input — the primary lever, and not a penalty at all.** Operator's framing:
   fitness uses streak, but nothing tells the RNN *"good job, keep in there"*. The signal's job is to
   **indicate when to change phase** in tracking mode. Adding an observation does not compete with
   aggression on the fitness landscape; it makes a different policy *representable*. That is why it is
   the lever to run first.
2. **Load as a physical cost, on pitch.** Not "pitch is rough" but "pitch is at ±11 g". Real limit,
   no tuned coefficient, and not out-competable — see §2.

The existing `ScenarioScore` / `selection.cc` plumbing is still the right vehicle for (2) if it is built;
what changes is the **term** — a load/energy quantity rather than a control-amplitude or control-rate one.
Keep the physical grouping. Do **not** build a throttle axis: throttle-high is a consequence of the twin
airframe, and penalizing it would fight the experiment's own setup.

**Sequencing — resolved 2026-08-07, see §7.** B2 carries **both** the streak/regime input and whatever load
axis A3 justifies. The earlier one-lever-per-bake recommendation is withdrawn: the input is ablatable
post-hoc and the axis is not ablatable by any means, so splitting the bakes buys no attribution the
ablation does not already give, and costs an extra 8000/49 lottery ticket. If A3 says load is not the
problem, no axis gets built.

### 6.1 Must land in the A1 bundle (fitness- or schema-affecting → one break, one rebake)

Everything here changes what a run means, so per 040's trap 2 it lands **before** B2 and **never** during
a live bake ([[feedback_no_rebuild_during_training]]).

| item | why it is a 041 prerequisite |
|---|---|
| **Streak accumulator inputs** (§3) | The headline. `NNInputs` 37→39, `TrackerInput` 58→60. |
| **Energy lexicase axis** (§6.0) | Basin insurance for B2. |
| **`prediction_score` one-tick pairing fix** | `cams[j]` ↔ `states[j+1]`. **C2a is confounded until this lands** — the head has never been scored against the target it was meant to predict. Fitness-affecting. |
| **Grouped-record refactor** (`tickList[i][k] = {state, cameraView, targetSample}`) | *If A0 promotes it.* Makes the whole index-parallel class unrepresentable rather than tested-for. A rebake is the only cheap moment to take the schema hit. |
| **`wind_velocity` recording** | `AircraftState::wind_velocity` is serialized but **never set** — zero in every dmp, M1 and M2. Honest-recording violation ([[feedback_honest_dmp_recording]]) for an input the controller demonstrably experiences. Getter, setter and the `dmp_dump` columns all already exist. |
| **Self-describing dmp** (config block per gen) | Readers (`dmp_dump`, renderer, analytics) currently take fitness/cadence params from *whatever ini is on disk now*. 041 will have several inis pointed at several sources; this is the structural fix, and it wants the same break. |
| **Integer-ms `simTimeMsec` stamping** | ±1 ms jitter today (200 Hz step clock truncated). It feeds **the ms-based NN history-lag selection and the `span_rate` gap denominator** — i.e. it adds noise to the exact rate inputs 041 is reasoning about. ⚠️ **CRRCSim submodule change, determinism-affecting** → pointer-bump first ([[feedback_submodule_merge_order]]), clean-slate only. |

### 6.2 Tooling that must exist before the reads

| item | why |
|---|---|
| **Genome ablation tool, `--zero-input`** | The only rigorous instrument in the feature (§5). |
| ~~Named NN input columns~~ | ✅ **Already exists** — `kPathgenInputMeta` / `kTrackerInputMeta` (`nn_inputs.h:180`, `:246`) carry `{"GYRO_P", "gyrP", 7}`-style name + short-name + width per slot, `static_assert`-ed against `COUNT`. So `--zero-input GYRO_P` is directly supportable and the Type-Safe-Sensor-Interface dependency **is not needed**. New inputs must add their meta rows, which the assert enforces. |
| **INI robustness: raise `INI_MAX_LINE`, surface `ParseError()` line number** | The 200-char cap **already bit this exact work once**: `EnablePredictorHead = 1` plus its explanatory comment hit 216 bytes and aborted startup with a bare *"Cannot parse configuration file"* — no line, no reason, and it cost a bisect. 041 adds several knobs with comments. Cheap, and it fails loud in the right way instead of the wrong one. |
| **Decouple `contract_tracker_config_tests.cc` from mutable production values** | It pins `FitStreakThreshold == 0.5` and friends as drift gates against the **live** ini. 041 touches streak config, so this suite will fail for a reason that has nothing to do with correctness. Keep a structural "production ini parses clean, required keys present" guard; drop the tunable-value pins. |
| **Time-denominate the streak metrics** (037 P-O11) | `pctInStreak` / `avgMaxStreak` are 041's *primary* progress signal, and raw tick-denominated streak counts read 2× at 20 Hz. Surface in seconds consistently before they are used to judge a bake. |

### 6.3 Comparator hygiene — cheap, and 040 already paid for the lesson

- ✅ **040 T087 — DONE for the two runs that matter (2026-08-07, verified object-by-object).**
  - `autoc-m1/autoc-9223370253553029228-2026-07-06T01:35:46.579Z/` (the M2 training source, `gen9200` =
    gen 800) — 800/800 `retain=keep`, already pinned; that pin is why it outlived its 30-day window.
  - `autoc-m2/autoc-9223370251039771221-2026-08-04T03:43:24.586Z/` (t4, 041's comparator) — was
    **`retain=expire` on all 800**, i.e. due to delete ≈2026-09-02, mid-041. Now 800/800 `retain=keep`.
- ⚠️ **The T085 novel-geometry source is deliberately NOT pinned** (operator 2026-08-07: "m1 source,
  m2 t4 — retain, that's it"). `autoc-eval/autoc-9223370250819561192-.../gen9999.dmp.zst` stays
  `retain=expire` and deletes ≈2026-09-05. **Consequence to carry into Phase D**: the two-pinned-novel-sets
  plan in §5 reduces to one (038-t10), so a novel-geometry result can no longer be cross-checked against a
  second independent 49-scenario sample. Regenerating produces a *different* 49 scenarios, so this is not
  recoverable after expiry — only re-derivable as a new, non-comparable set.
- **Archive `nn_weights*.dat` beside every pinned dmp** (040 deferral, option 2). A pinned dmp preserves
  numbers; only the NN01 weights preserve a controller you can re-fly.
- **The eval-ini shape hazard.** `autoc-eval-tracker.ini` is one file doing two incompatible jobs
  (training-repro vs novel-geometry), and 040 T021 needed **four** hand-aligned fields — including the
  scenario *shape* — to reproduce a run. 041 repoints it at ≥3 sources (new M1, 040-t5, 038-t10). The 1:1
  seed-table guard catches count mismatches only; a config differing in sigmas or enables would produce a
  plausible wrong number. Split the file or add a pre-flight assertion.
- **`data.dat` retention.** Confirmed 2026-08-06: **no `data.dat` or `data.stc` exists in the repo root** —
  every prior run's per-tick trace is already gone, including 040's. Not fatal: `dmp-dump`'s per-tick CSV
  carries `out_pt, out_rl, out_th, stpPt, dist, mult` for both modes, so per-tick analysis of any pinned S3
  run (including A3's study) is fully available from the dmps. But the file is overwritten at the start of
  every run and B2 + repeats means several, so **snapshot `data.dat` for the final M1 and M2 runs**
  (operator 2026-08-06). Trivial, and it is the difference between an attempt being analyzable later or
  not.

### 6.4 Checked and requires no action

- **Eval fitness Bugs 2 and 3** (stale S3 fitness, missing `rabbitSpeedConfig`) — verified still fixed at
  035 T024. Named here only so the spec does not re-litigate them.
- **In-code `TODO`s** — swept. Nothing on the 041 path: the live ones are an eval-logger extraction note,
  a `math_utils` rng-migration note, a renderer camera-centering nit, and two stale `autoc.cc` markers
  (T044 scenario iteration, an M6e crash-filter note). None are prerequisites.

### 6.5 Adjacent, deliberately OUT (named so the spec cannot absorb them by drift)

- **Stability lexicase axis in B2** — held out on purpose (§6.0), not forgotten.
- **Trainer write-through analytics cache** — nominally due ("pre-work for the feature after 039"), but it
  touches the trainer hot path and wants its own `rebuild-perf` pass. In only if S3 re-fetch actually
  bites; `GENERATE_PNGS_CACHE=<persistent path>` is the interim.
- **`EvalVariationScaleOverride`** — only if B2 exits before the ramp reaches 1.0.
- **Streak-threshold ramp** (022 T024) — actively unwanted: curriculum stays OFF for weak-signal work.
- **Control-amplitude / control-rate smoothness axes of any shape** — ruled out on the standing record,
  not merely deprioritized: they deaden the paths and lose to aggressive manoeuvres (§2). Kept out so the
  spec cannot quietly reintroduce one as "the aggressiveness axis".
- **A throttle penalty** — throttle-high is a consequence of chase == target airframe. Fix the airframe
  asymmetry (existing backlog item) or leave it alone.
- **Total energy as NN input / lexicase objective** — *promoted from parked to plan-phase adjacent*, since
  "aggressive = more energy" makes it the natural cost unit for the load thread. Still out of B2: one lever
  per bake.
- **Spherical projection, camera-variation delta (t1′), camera PRNG slot, detection-envelope realism, CEP
  physical model** — 040 closed perception.
- **Pitch marginal-stability levers, FDM propeller** — gated on **n>1 flight articles**.
- **US2 two-timescale recurrence** — parked; unpark trigger unchanged.
- **Hull-crash lexicase dimension, negative-ahead reward, two-sim co-evaluation, demetic islands,
  chase-vs-target different craft, online craft identification** — all downstream of a working M2.
- **Airframe self-occlusion re-enable, xiao tracker-mode port, GPU eval** — unrelated triggers.

---

## 7. Decisions taken, and what is still open

**Settled by the operator 2026-08-06:**

- **Production M1 regime = pop 8000 / 49 winds** (Phase B2), repeated as a lottery re-draw if it does not
  climb. **Jump straight to the prod build** — recent runs have shown that is fine.
- **Pop 3000 is a SMOKE TEST, not a comparison arm.** The topology change is generic (M1 *and* M2), so the
  smoke set proves it runs in both modes; more rapid smokes only if something looks off. There is **no
  control bake** — nothing at the micro level is A/B-able when PRNG, model and fitness all move between
  runs. The bar is *in the ballpark*, exactly as 040 read no-significant-regression against 039/038.
- **H1 is therefore tested by input ablation on the trained elite** (within-build, one variable) for the
  mechanism claim, and by a ballpark read against the historical per-axis band for the outcome claim. This
  promotes the genome ablation tool to a Phase-A dependency (§5).
- **Scope = backlog → M1 experiment bake (aggressiveness) → potentially repeat → M2 predictor work →
  M2 bake.** Threads split by mode: M1 carries aggressiveness, M2 carries the predictor.
- **The value head moves to M2** (Phase C2b) rather than being an M1 rung. Its M1-portability stays a
  design *advantage* to note, not a phase to run.

- **Smoothness axes are out as a class** (operator 2026-08-06): tried for a while, they deaden paths and
  lose to aggressive manoeuvres. The aggressiveness thread is now (a) the streak/regime **input**, and
  (b) **load on pitch** as a physical cost — not roughness as an aesthetic one.
- **M1 uses the exact in-streak signal; the M2 estimator is plan-phase research.**

- **Success bar for B2 = non-regression** (operator 2026-08-06: *"in reality the gate is no worse than
  now"*). 041's M1 phase is mostly backlog plus a closer look at aggressiveness; **disproving H1, H2 or
  H3 is an acceptable and expected outcome**, not a failure of the feature. The spec writes the bar down
  this way so a flat result reads as information rather than as a miss.
- **M2 is constrained to the predictor question** (§5) — aggressiveness inherits from M1 rather than being
  re-experimented on.

### A0 and B2, resolved on the clean-slate license (operator 2026-08-07)

> *"We are in the 'don't have to be backward compatible' phase of the entire M2 initiative."*

This is broader than [[feedback_no_cereal_versioning]] — that policy says don't *version* the schema; this
says the whole M2 initiative is currently unconstrained by compatibility. There is therefore **no compat
tax to weigh against the clean option**, which resolves both open items in the same direction: take the
structural fix, not the instrumented one.

**A0 — promote the whole §6.1 bundle, and take the structural version.** The grouped-record refactor
(`tickList[i][k] = {state, cameraView, targetSample}`) was the only genuinely open call, and it was open
*only* because of the schema cost. With no compat tax, prefer it over asserting/testing the parallel-index
invariant: it makes the failure class **unrepresentable** rather than caught, which retires four known
instances and any future ones at once. A0's output becomes the *inventory* (what else shares the shape) and
the migration list, not a go/no-go.

**B2 — carry both levers**, reversing the one-lever-per-bake recommendation. The reasoning is an asymmetry
in what is recoverable after the fact, not a change of appetite.

> ⚠️ **SUPERSEDED 2026-08-07 — there is no second lever.** The load axis left 041 entirely (spec FR-019/020),
> so B2 carries the envelope/streak input alone. The asymmetry argument below is still *correct* and is worth
> keeping, because it is why bundling **inputs** is safe: inputs are ablatable, fitness axes are not. With no
> axis in the bake, **every new thing in B2 is an input, so nothing in it is un-attributable** (spec FR-020a).

- The **streak/regime input is ablatable post-hoc** — zero the columns on the trained elite, re-eval
  within build (§5). H1a survives regardless of what else was in the bake.
- A **load axis is not ablatable at all.** It shaped the weights during evolution; zeroing anything at eval
  time cannot recover its contribution. So a separate bake buys attribution *only* against a
  combined-minus-input cross-run delta — exactly the micro-comparison the operator has ruled meaningless
  when PRNG, model and fitness all move.

Since separating the levers buys no attribution the ablation does not already provide, and costs an extra
8000/49 lottery ticket, bundling was the right call — and the 2026-08-07 clarification then removed the axis
altogether, so **B2 carries the envelope/streak input and nothing else behavioural.**

**Still open, needed before `/speckit.specify` closes:**
2. **What "in the ballpark" means numerically for B2** — which historical runs form the comparison band
   and how much per-axis movement counts as no-regression. 040 made this call qualitatively; naming the
   band before the bake is cheap and keeps the read honest either way.
3. **Confirm the reading of "calibration efforts"** as **re-establishing the comparator set** — new M1
   baseline, archived weights, both novel-geometry eval sources pinned and re-run — rather than
   re-calibrating 040's 15 assumed perception values (which SC-012 proved need no code change, and which
   041 has no new measurements to inform).
