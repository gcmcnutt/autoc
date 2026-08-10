# 041 Phase 0 — Research

**Purpose**: resolve every `NEEDS CLARIFICATION` from [plan.md](plan.md) Technical Context, or assign it
explicitly to a later phase / to the operator. Each entry states the **decision**, the **rationale**, and
the **alternatives rejected**.

Evidence base: [hypothesis.md](hypothesis.md) (derivation), [spec.md](spec.md) (requirements),
`specs/BACKLOG.md`, and direct code reads cited inline.

---

## R1 — Accelerometer input: how many slots, and framed how?

**Decision**: **3 slots, body-frame specific force**, normalized by g, named **`ACCEL_X/Y/Z`** in both
`PathgenInput` and `TrackerInput`, placed adjacent to `GYRO_P/Q/R`. Instantaneous only — **no history
window** in this feature (but see the open sub-question below).

### The framing: craft state is 6-DOF, and today it is half-complete (operator 2026-08-07)

Current craft state is **identical in both modes — 8 slots**: `QUAT_W/X/Y/Z` (attitude), `AIRSPEED`,
`GYRO_P/Q/R` (angular rate). The inertial block is therefore **3 rate, 0 accel**: half an IMU.

This is the primary rationale, and it is stronger than the coordination argument an earlier draft used:
**3 accel + 3 rate is one coherent sensor**, not two features. It also maps exactly onto hardware — the
xiao carries an **LSM6DS3, a 6-DOF IMU: 3 accel + 3 gyro** — so this is completing the sensor the airframe
already has, not inventing an input. After the change, craft state is 11 slots of which 6 are the inertial
block.

**Naming**: `ACCEL_X/Y/Z` rather than `SPEC_FORCE_X/Y/Z`. A real accelerometer *outputs* specific force, so
`ACCEL_*` is the honest hardware name, and adjacency to `GYRO_*` makes the 6-DOF block visible in the enum,
the metadata table, and every `data.dat` header. The specific-force semantics stay documented at the
declaration and pinned by test.

**Rationale (mechanics)**:
- *Specific force, not kinematic acceleration.* A real accelerometer measures `a − g` in body frame.
  Feeding FDM kinematic acceleration would train against a signal no accelerometer produces, leaving a
  constant ~1 g offset in the most load-relevant axis — invisible in sim, wrong in flight. This is a
  Principle VII failure shape (a silently wrong value that looks correct).
- *Three axes, not one.* Normal load alone (body z) is the damage-relevant scalar, but a partial IMU is a
  strange thing to ship, and the other two axes distinguish a coordinated pull from a skidding one for 2
  slots. Withholding them makes that question unanswerable in the bake.
- *Normalized by g* so the input sits in a tanh-friendly range: ±11 g → ±11 raw is far outside every other
  input's scale, and 040-t1 already demonstrated what happens when a quantity sits at the wrong scale
  against a bounded activation. Divide by a `kAccelScale_g` constant in the same spirit as
  `kCruiseSpeed_mps` and `kDistToBoundaryScale_m`.

**Alternatives rejected**: 1 slot (normal load only) — a partial IMU, and it forecloses the coordination
question; raw m/s² unnormalized — scale mismatch against a bounded activation; kinematic acceleration —
sim-only quantity, breaks sim-to-real.

### Temporal depth on the 6-DOF block — DECIDED: instantaneous, with a recorded fallback ladder

**Decision (operator 2026-08-07): ship instantaneous, 3 slots.** The fallbacks below are *recorded, not
deferred-by-omission* — if load turns out to need temporal depth, the escalation is known and neither step
requires rethinking the design.

| step | what | slot cost | when to reach for it |
|---|---|---:|---|
| **0 — chosen** | `ACCEL_X/Y/Z` instantaneous | **+3** | now |
| **1 — fallback A** | **Differentiation**: add jerk channels, `d(accel)/dt` per axis | **+3** | the net responds to load but cannot *anticipate* it — e.g. a load axis lands and peak load does not fall |
| **2 — fallback B** | Full 6-lag history on the accel channels | **+18** | a rate channel is still insufficient |

**Why differentiation is the right first fallback rather than history**: it captures the rate-of-onset
argument — load is a *building* phenomenon and one sample cannot express "this is climbing" — at
**one-sixth the slot cost** of a lag buffer. It also has direct precedent: the project has already made
exactly this trade twice, and both are in the current input vector. `SPAN_RATE` is a true per-second rate
over the NOW↔TM1 gap (`kNNHistoryRecentGapSec`), and `CLOSING_RATE` is the same shape for distance. So
"derivative instead of history" is an established in-tree pattern with an established cadence-invariant
formulation to copy.

**Why not history first**: the standing asymmetry is deliberate — the NN gets 6-slot history for the
TARGET and **none for SELF** (`quat`, `airspeed`, `gyro` all instantaneous). Giving accel history would be
the first self-history in the input vector, at 18 slots, and the standing read is that hand-built lag
buffers are *"a fudge for weak internal memory"* — doubling down is off-thesis when the recurrent layer is
the intended memory. The 038 +128-input enrichment also delayed a run's takeoff by ~100 generations, so
slot growth is not free.

**Cheap evidence available before escalating**: Study A can report the autocorrelation of normal load at
the history lags. Strong autocorrelation at 50–100 ms ⇒ one sample already carries most of the
information, and neither fallback is warranted.

**Open sub-question for the plan phase**: noise and bias modelling. A sim accelerometer is
unrealistically clean. Decision: **ship clean in 041**, and record "accelerometer noise/bias as a craft- or
camera-class variation axis" as a backlog item at wrap. Rationale: adding an unmeasured noise model is
tuning against a guess, and the deployed IMU's characteristics are available from the xiao bench when
somebody measures them.

---

## R2 — Envelope-occupancy accumulator: what resets it?

**Decision**: **two slots** — `IN_ENVELOPE` (0/1) and `ENVELOPE_SECS` (normalized
`min(duration / FitStreakRampSec, 1)`, **linear** — no log, no tanh). The accumulator resets **on envelope
exit only**, not on regime change. It is an **external counter** in the stepper/firmware, not a learned
quantity, computed identically in both modes.

### ⚠️ Correction (operator 2026-08-07): the FLAG is the key input, not the accumulator

An earlier draft of this memo reasoned that the *integral* was the missing quantity, since the flag is
computable from existing inputs and the duration is not. That is true about **information** but wrong about
**decision relevance**, and the operator's framing is the correct one: *"in tracking is probably the key NN
input — the fitness maintains the multiplier, not the controller."*

The first-principles argument: the streak multiplier is **monotone** in duration. With **no competing cost
in 041's objective** — the load axis was deferred (R4) — there is no state in which leaving the envelope is
preferable, so the optimal policy is simply "stay in". That needs the flag; it does not need the depth.
Knowing "how deep am I" only changes behaviour once something competes with staying, i.e. once a load or
energy cost exists.

**What the flag still buys, even though it is information-computable**: it is a *derived feature* that
saves the net from learning a nonlinear cone boundary itself — the same rationale as the hand-derived
`beacon_pair_span` and `target_tilt` channels. In M2 it is not even computable from the inputs (it needs
CEP-visibility + separation-range + centroid logic), so there it adds information as well.

**Why both slots still ship**: the format break happens once, and the follow-on load feature is exactly the
context in which depth becomes decision-relevant. Shipping the accumulator now avoids a second M1 rebake
later — the same reasoning that ships `ACCEL_*` before any load objective exists (R1). Ablation separates
the two channels (FR-014b), so carrying the secondary slot costs one input and no attribution.

**Rationale**:
- The quantity's job is to mirror what the *reward* pays for, and the reward's streak counter resets on
  exactly one condition: `stepPoints < FitStreakThreshold` (`fitness_computer.cc:60-64`). Matching that
  keeps the observation a faithful proxy of the multiplier. Resetting on regime change would make the
  input mean something the fitness does not.
- Normalizing by `FitStreakRampSec` makes 1.0 mean "multiplier saturated", which is the decision-relevant
  boundary for a phase change.
- Milliseconds-based per FR-016, so a cadence change re-derives rather than silently rescales — the same
  discipline as `kNNHistoryLagsMsec`.

**Alternatives rejected**: reset on any regime change — decouples observation from reward; unnormalized
seconds — scale mismatch and cadence-fragile; a single combined slot (duration only, with 0 meaning
not-in-envelope) — conflates "just entered" with "not in", and costs the net a discontinuity to learn for
no slot saving worth having.

**Note on regime**: `{tracking, intercept, patrol}` needs **no dedicated input**. `IN_ENVELOPE`
distinguishes tracking, and `CLOSING_RATE` — already an input — splits intercept from patrol. Confirmed
against `dynamics_progress.py:74-80`, where `TRACK_THRESHOLD = 0.5` is the *same value* as
`FitStreakThreshold`. The regime partition is therefore already computable by the policy; only the
integral was missing.

---

## R3 — M2 in-envelope estimator

**Decision**: **defer the design to the plan phase of the M2 work (Phase C/D), and do not build it in the
A1 bundle.** M1 uses the exact geometric condition (FR-018). The tracker input *slots* are reserved and
populated in A1 so the layout breaks once; the estimator behind them starts as the direct analogue —
both beacons CEP-visible, separation within `[lo, hi]`, pair centroid within a centred radius — with
accuracy characterised, not assumed.

**Rationale**:
- Sequencing: M1 answers whether an *exact* signal is worth anything. If it is not, no estimator is needed
  and the M2 slots carry a cheap approximation at no cost. If it is, the estimator's required fidelity is
  known from the M1 effect size — which is the right order to design in.
- The estimator is the one genuinely research-shaped part of the feature (operator: "yes, some research
  during plan phase"), and 040 made perception materially harder in six directions including per-scenario
  mount misalignment, so its error budget is not guessable from first principles.
- Reserving the slots in A1 avoids a second format break, which is the whole point of FR-005.

**Alternatives rejected**: building a full estimator now — designs against an unknown fidelity target;
leaving the tracker slots out — forces a second break; feeding the fitness machinery's true streak state
to M2 — forbidden by FR-015 and not deployable.

---

## R4 — Load objective: does it exist in 041, and in what form?

**Decision (operator 2026-08-07): NO load axis in 041.** Study A **reports only**. The M1 bake carries one
behavioural lever — the envelope/streak input — and the question it answers is *does streak in the RNN help
or hurt*. Load and per-regime aggressiveness findings feed a **follow-on feature**, deliberately sequenced
**after the M2 predictor go/no-go** so it is designed intentionally rather than bundled speculatively.

The constraints below are recorded so the follow-on inherits them rather than rediscovering them. Any
future load objective: a **selection axis** (not a scalar penalty), on **peak/exceedance against a physical
limit** (not mean amplitude), grouped **physically** (bank vs throttle), and **pitch-first**.

**Consequence worth stating — 041's bake now has zero un-attributable changes.** With no axis, every new
thing in the bundle is an *input*, and inputs are ablatable post-hoc (contracts/ablation-cli.md). The
earlier worry about a fitness axis being un-attributable dissolves: attribution is fully recoverable for
everything in the bake. The only residual cost is 5 extra input slots' worth of search space — negligible
against the +128-input enrichment that cost ~100 generations in 038.

**Rationale**:
- The whole class of amplitude/rate smoothness penalties is ruled out on the standing record — they deaden
  paths and lose to aggressive manoeuvres (spec Non-Goals). What survives is a *physical limit*, because
  exceeding it is a failure rather than a lower score, so it cannot simply be out-competed.
- Scalar composition is ruled out by [[project_scalar_multiobjective_collapse]] — 033 phase 1 died in a
  Pareto corner on precisely this problem.
- Physical grouping per [[project_smoothness_axis_grouping]]: bank (pitch+roll) trades geometrically
  through the bank vector; throttle is decoupled.
- Pitch-first because that is where the damage is: bang-bang on pitch, ±11 g. Roll appears calm only
  because it is a well-damped axis on a planar wing — plant, not policy. Throttle tends high because the
  chase is the same airframe as the target, which is structural to the experiment.
- **No throttle axis** (FR-020). Retracted from an earlier draft of this memo: the plumbed 027-era energy
  term penalizes throttle *level*, and the r1/r2/r3 record is explicit that saturation is not the
  pathology — climbers pass **through** mean ≈0.85 with σ>0; only the stuck basin *locks* at exactly 1.000
  with σ=0.000. A level penalty cannot tell them apart and penalizes the climbers harder.

**Reusable plumbing if a load axis lands**: `ScenarioScore` already carries `stability_score` and
`energy_score` fields with per-scenario accumulation in `fitness_decomposition.cc` and pool wiring in
`selection.cc`, disabled behind `CADENCE7-REDUX` markers, plus 4 `DISABLED_` tests in
`selection_tests.cc:152+`. Reuse the **plumbing**, replace the **term**.

**Alternatives rejected**: re-enabling the 027 terms as written (wrong statistic, see above); a scalar
penalty (known collapse); a global regime-blind term (wrong in two of three regimes).

---

## R5 — Grouped per-tick record: what happens to the pre-loop initial state?

**Decision**: the initial state is **stored once, beside the tick list, as a named field** — not as
`tickList[0]` with null camera/target members.

**Rationale**:
- The bug being retired is precisely that this extra state was pushed *into* a list that other lists did
  not have an entry for. Making it a distinct, named field makes the asymmetry **explicit and typed**
  instead of accidental — which is the stated intent ("that choice becomes explicit instead of
  accidental").
- Putting it in the list with empty co-members recreates the hazard in a new form: consumers would need to
  know slot 0 is special, which is the same "invariant in a comment" failure.
- Consumers that iterate ticks then need no offset arithmetic at all — the objective's `stepIndex - 1`
  clamp disappears rather than being relocated.

**Alternatives rejected**: `tickList[0]` with sentinel members — recreates the class; a per-sample tick
index — rejected by the operator on 2026-08-02 ("stick with proper indexing, not additional storage"), and
it pays permanent bytes per tick to paper over a structural problem.

---

## R6 — Cereal version bump: constitution or habit?

**Decision**: **bump the version field.** No migration paths, no shims; readers of an older artifact fail
loudly naming both versions.

**Rationale**: Constitution V's write-side contract requires a committed transition to bump the version
directly *while not maintaining compatibility shims*, and Principle IV establishes the constitution
outranks agent memory. The recorded habit ([[feedback_no_cereal_versioning]]) is about **not maintaining
compatibility** — which 041 honours completely — not about refusing to **declare identity**. Separating
those two concerns dissolves the apparent conflict.

Practical payoff: the 040 wrap recorded that the 038-t9 baseline "cannot be loaded by a current binary,
dies with `vector::_M_default_append`". That is a schema mismatch presenting as a memory error — the exact
outcome fail-loud-with-versions exists to prevent. A reader saying "artifact v=N, reader v=N+1" would have
turned an afternoon of confusion into one line.

**Alternatives rejected**: no bump (status quo — reproduces the `_M_default_append` diagnosis problem);
bump *and* migrate (violates III and the clean-slate license).

**Follow-up**: update the memory entry to reflect the resolved practice, since it currently reads as an
unqualified "never bump".

---

## R7 — What does "no tracking regression" mean numerically?

**Proposed default, for operator confirmation**: the comparison band is the **039/040-era per-axis reports
plus the 040-t4 M2 numbers**, and non-regression means:

- primary: `pctInStreak` and `avgMaxStreak` within noise of, or above, the band;
- crash/OOB scenarios not worse in count;
- per-axis `dCtrl` and `⟨|u|⟩` reported per regime — movement *down* is the hypothesis, but flat is
  non-regression, not failure;
- peak load reported and not higher.

**Rationale**: 040 made this call qualitatively ("no significant regression from 039/038") and it worked
as a bar. Writing it down before the bake keeps the read honest either way, which matters more than the
exact thresholds when a refuted hypothesis is an accepted outcome (SC-012).

⚠️ **Absolute fitness sums are not comparable across runs** with different scenario counts — per-scenario
or per-step rates are the only valid currency, and streak-fraction metrics are already per-scenario.
Raw per-scenario score is additionally length-confounded when path classes differ.

**Assigned to**: operator (plan.md open item 2).

---

## R8 — M1 bake retry budget and abort criterion

**Proposed default, for operator confirmation**: **up to 3 attempts**; abort an attempt early on the
**stuck-basin signature**, which is throttle **lock** — amplitude exactly 1.000 with **σ = 0.000** and
`dCtrl` 0.000 — together with `avgMaxStreak` frozen and best-sigma not annealing past ~0.14 by gen 200.

**Rationale**: the documented rate at this population/wind count is roughly 1 in 3, so 3 attempts is the
budget that makes a climber likely rather than hoped-for. The abort signal is chosen from the direct
r1/r2/r3 evidence: **saturation is not the tell** (r1 and r2 both passed through mean ≈0.85 with σ>0 and
escaped; only r3 locked), and the gen-150 inflection heuristic is unreliable (r2 did not inflect until gen
280 yet finished strong). Judge climb on streak-fraction metrics, not completions —
[[project_servo_era_progress_metrics]].

**Assigned to**: operator (plan.md open item 3).

---

## R9 — Where does the offline study get its data?

**Decision**: from **S3 dmps via a `dmp-dump` physics-column extension**, not from `data.dat`.

**Rationale**: verified 2026-08-07 — **no `data.dat` or `data.stc` exists in the repo root**; every prior
run's per-tick trace is already gone, 040's included. But `PhysicsTraceEntry` carries per-tick `acc[3]`,
`omegaDotBody[3]`, `alpha`, `vRelWind`, is populated for **every elite reeval**
(`inputdev_autoc.cpp:1047`), serialized into every gen dmp — and **has no consumer in `tools/` or
`src/`**. So load has been recorded all along and never read. `dmp-dump` already emits
`out_pt,out_rl,out_th,stpPt,dist,mult` per tick for both modes; adding physics columns is a reader change.

This also settles FR-022 forward: snapshot the final M1/M2 `data.dat`, but the *analysis* path does not
depend on it.

**Alternatives rejected**: `data.dat` (gone, and overwritten each run); new recording (unnecessary — the
data exists); the per-axis-from-dmp backlog item in full (larger than needed; the physics columns are the
slice 041 requires).

---

## R10 — Predictor target: what exactly gets predicted?

**Decision (refined by clarification 2026-08-07): a continuous estimate of the target's CURRENT bearing** —
emitted every tick including blind ticks, scored against truth wherever truth exists, dead-reckoning through
gaps. **Horizon-free: a state estimator, not a forecast.** Feasibility decided offline before any bake
(FR-023).

**Why the refinement was needed**: the earlier "reappearance bearing at gap-scale horizons" formulation
still carried the operator's original objection — *at prediction time we do not know when reacquisition
happens*, so the horizon is unknown, merely relocated from an arbitrary constant into an unpredictable gap
duration. Making the target "where is it *now*" removes the horizon entirely: the target is always
well-defined, every tick is a training sample, and persistence still collapses as the gap ages.

**Operator's framing of the use case**: when you lose sight of the other aircraft you estimate which way
they will go *and* which way you should go to get behind them. 041 builds only the **first half** — the
estimate. The second half is a control behaviour, which is what *consuming* the estimate would enable, and
combined offense/defense is explicitly a later thread. Expectation is deliberately modest: *"we can perhaps
get some sort of signal by starting with C."*

⚠️ **Metric consequence**: a pooled r² on this target is misleading by construction — on visible ticks the
truth is an input, so both head and baseline are near-perfect. The statistic MUST be **binned by blind-gap
age**, and the bins that matter are the long ones (worst-case blind windows reach ~8 s).

**Rationale**: the purpose fixes the target — the head exists "mostly to help re-acquire best we can from
target out of camera FOV" (operator 2026-08-07). The old target failed for reasons that do not apply here:
span moves ~0.0075 rad per 150 ms against a ~0.049 rad level, so **persistence was already right to within
15%** and a perfect head added nothing; measured `r(Δspan) ≈ 0` at every horizon, best error at
*generation 1*. Across a multi-second gap, persistence collapses and the information content is real.

⚠️ The current objective **CEP-gates both endpoints**, so blind gaps are *excluded from scoring* — the head
is scored only where prediction is information-free and skipped where it would pay. The re-target therefore
changes **which ticks are scored at all**, not a coefficient.

**Payoff framing**: this makes the predictor a *perception-budget* lever, not a control aid. Better
reacquisition is what buys permission to narrow the field of view; 120°→90° is ≈+2.5 dB SNR (∝ 1/FOV²) and
+33% angular resolution (3.56 vs 2.67 px/deg at 320 px) against a standing ~24 dB shortfall at 100 m.
**FOV reduction is out of 041 scope** — an option this enables.

**Alternatives rejected**: span at 50/100/150 ms (measured worthless, and those horizons were chosen for
actuation lag, not the purpose); rescaling the existing head (a coefficient fix on an objective the
persistence bar already calls worthless); a value head (retained as the *fallback* if reappearance
geometry does not clear offline — its virtue is that prediction accuracy *is* objective accuracy and it
ports to M1, but nothing has validated it).

---

## R11 — Does the M2 bake need an aggressiveness lever?

**Decision**: **no** (FR-028). M2 inherits.

**Rationale**: two independent paths carry it. The new inputs are **generic** — they land in both
`PathgenInput` and `TrackerInput` — so M2 trains with them. And M2 chases M1's *recorded trajectories*, so
a calmer, less load-spiking source is a materially easier and more physical thing to chase. Meanwhile M2
is already a good learner: every run to date climbs fast and settles at about the same competence
regardless of what changed, which is the observation behind [[project_m2_tracking_ceiling]] — M2 does not
need help learning, it needs something worth learning from.

**Risk accepted**: if inheritance does not happen, an M2-side lever becomes a follow-on feature rather
than a mid-feature addition (spec Dependencies & Assumptions).

---

## R12 — Tooling prerequisites: what is actually missing?

Swept, with two items removed from the prerequisite list on inspection:

| item | status |
|---|---|
| Named input columns for the ablation mask | ✅ **already exists** — `kPathgenInputMeta` / `kTrackerInputMeta` (`nn_inputs.h:180`, `:246`) carry `{"GYRO_P","gyrP",7}`-style name + short name + width per slot, `static_assert`-ed against `COUNT`. `--zero-input GYRO_P` is directly supportable; the Type-Safe-Sensor-Interface backlog item is **not** a dependency. New inputs must add meta rows, which the assert enforces. |
| Genome ablation tool | ❌ **must build** — `tools/nn_ablate.cc`, `--zero-input` only |
| `dmp-dump` physics columns | ❌ **must build** — R9 |
| INI robustness (raise `INI_MAX_LINE`, surface `ParseError()` line number) | ❌ **must build** — the 200-char cap already aborted startup once on `EnablePredictorHead = 1` plus its comment (216 bytes), with no line number, costing a bisect. 041 adds knobs with comments. |
| Decouple `contract_tracker_config_tests.cc` from mutable production values | ❌ **must fix** — it pins `FitStreakThreshold == 0.5` against the live ini as a drift gate; 041 touches streak config, so it fails for a reason unrelated to correctness |
| Time-denominate streak metrics | ❌ **should fix** — `pctInStreak`/`avgMaxStreak` are 041's primary progress signal and raw tick-denominated counts read 2× at 20 Hz |

---

## R13 — Hardware accel source (added by clarification, 2026-08-07)

**Decision**: **INAV over MSP**, via an extension to the custom `MSP2_AUTOC_STATE` consolidated command so
accel arrives in the same single round trip.

**Rationale**: every other craft-state channel (`quat`, `gyro`, position, velocity) already arrives on this
path, so accel belongs there — one clock, one alignment convention, no second body frame to calibrate. INAV
also already applies board alignment to its sensors, so this inherits that handling instead of duplicating
it. And INAV's accelerometer output *is* body-frame specific force, which is exactly the sim semantics
(research.md R1).

**The precedent is exact**: fork commit `63cffaf4f` — *"feat(021): extend MSP2_AUTOC_STATE with filtered
gyro rates"* — did this same thing one field earlier. Copy its shape: append at payload end, fixed integer
scale stated at the write site (gyro used deci-deg/s ×10; accel would use milli-g ×1000, giving ±32 g in
int16 against an observed ±11 g), and document the axis/sign convention for the consumer. Site:
`~/inav/src/main/fc/fc_msp.c`, `MSP2_INAV_LOCAL_STATE` case.

**Consequences**:
- **A third build surface enters the feature** — autoc/crrcsim, xiao, and now INAV, the last in **two target
  variants**: bench = **`MAMBAF722_2022A`** (STM32F722, per `xiao/inav-bench.cfg`) and flight =
  **`MATEKF722MINI`** (per `xiao/inav-hb1.cfg`), both at `63cffaf4`. Both were built together in **021 T041**
  (closed), with commands in `specs/020-pre-flight-pipeline/plan.md`. Deploy to bench first.
- **Not a risk** (corrected 2026-08-07): INAV builds routinely on this machine — vendored toolchain at
  `~/inav/tools/arm-gnu-toolchain-13.2.rel1`, `build/inav_8.0.0_MATEKF722MINI.hex` already present. This is
  ordinary scope.
- ⚠️ **Procedure**: **remove the GPS before flashing an INAV controller.** Standing hardware quirk; belongs
  in the runbook, not in memory.
- **Sign convention is a known trap** — the gyro extension required the consumer to negate pitch and yaw,
  and a board-alignment discrepancy has already cost ~10° of pitch bias in flight data. Resolve against
  `COORDINATE_CONVENTIONS.md` and pin with a test.
- **Payload/latency**: 039 measured zero overruns at 115200 with the current payload; +6 bytes is small, and
  the unexercised 460800 baud-raise lever is the documented fallback if headroom is marginal.
- **Bundling**: since INAV is being built and flashed anyway, the queued `mspOverrideInit` first-frame patch
  (backlog C1) can ride along nearly free — the same "now is the time" logic that motivated the format
  break. A deliberate decision, not an automatic inclusion.

**Alternatives rejected**: the xiao's own LSM6DS3 — introduces a second body frame needing its own airframe
calibration plus a second clock, for a sensor that has never been in the control path; INAV-primary with
LSM6DS3 logged as cross-check — defensible and closes the 021 thread, but adds log plumbing to a feature
already carrying three build targets; zero-filling in firmware — would make the flight test not a test of
the trained policy.

---

## R14 — Camera model alignment with ordered hardware (added 2026-08-10)

Source: [031 handoff](../031-beacon-camera/handoff-041-camera-model.md) +
[camera-era-knobs.md](../031-beacon-camera/camera-era-knobs.md).

**Two of the three asks were already satisfied**, which the 031-side instance had no way to know:

| ask | status |
|---|---|
| pinhole → equidistant (f·θ) | ✅ **already done** — `camera_projection.cc:158-184`, since 038 t9 / 040 T031. Bearings are θ in radians; `radPerPx` is uniform, so "constant °/px everywhere" is already exploited |
| FOV/projection as parameters, not constants | ✅ **already done** — 040 retired the FOV keys; field is *derived* from grid × °/px (FR-003), and SC-012 proved 15 assumed values substitute with no code change |
| **H = 120° / V = 75°** | ⚠️ **real delta** — H already 120°, **V is 90°** |

**Decision**: adopt V = 75° via `CameraPixelsV 240 → 200` (200 × 0.375 = 75° exactly). Also makes the grid
aspect **1.6**, matching the real OV9281 (1280×800); the prior 240 px was a 4:3 invention that left the vertical
tracking envelope optimistic by 15°. `CameraDegPerPixel` untouched ⇒ `radPerPx`, quantisation and CEP unchanged.
M2-only; M1 has no camera and is unaffected.

⚠️ **Do not re-open the projection casually.** t9's switch to equidistant **near-froze evolution** (elite
unreplaced ~57 gens, pop average climbing ~5× slower) because the rectilinear tan-stretch had been an
*accidental training aid* — edge NDC gradient ~2.3/rad vs a flat 0.955/rad, and keep-in-frame is an edge
phenomenon. 040 T031 went angular anyway and it held, but the path cost a run.

**Two physical constraints imported as predictor contract** (FR-030, FR-031):
- Drift budget Δθ ≈ ½·a_target·t²/r + 1–2° IMU error. A 3 g target exits a ±36° half-field in ~1.1 s @50 m /
  ~1.5 s @100 m ⇒ **the predictor's timescale is order 1 s, not sub-second**, and this is an independent
  cross-check on Study B's measured distribution.
- **Warm code relock floors at ≈155 ms** (N/f_chip, N = 31 @ ~189 Hz). A perfect predictor still waits that
  long ⇒ **the predictor's value is pointing, not latency**; sub-155 ms reacquire claims are unphysical.

**One value corroborated, not changed**: `CameraDetectionRangeM = 100.0` was ASSERTED (040 FR-033a) and the 031
photon budget independently lands 100–110 m class bright-day at 4×4 defocus (SNR ≈22 @100 m, ÷4, vs ×4.5
threshold). Does not contradict 040's single-PD pessimism — small-pixel sky patch is a different budget.

**Deferred with triggers** (both now in `specs/BACKLOG.md`): the birded two-camera pair — the operator adds the
cost the handoff omitted, **inter-camera alignment/calibration variation**, a new variation class rather than a
doubled one; and on-the-fly mode switching, whose interesting form is target-state-driven (**close and bright ⇒
centre-bore**), gated on the A8-2 register measurement.

**Feedback loop worth naming**: **041's predictor verdict is an input to the lens purchase** — 1.8 mm vs 2.x mm
"depends on how useful predictors are here in 041" (operator). So the go/no-go and the measured blind-gap
distribution are hardware-purchase-relevant, not merely training-relevant.

---

## Resolved / assigned summary

| # | Unknown | Outcome |
|---|---|---|
| R1 | Accelerometer slots + framing | **Decided**: 3 slots `ACCEL_X/Y/Z`, body specific force, g-normalized, adjacent to `GYRO_*` (completes the 6-DOF block), instantaneous — with a recorded fallback ladder: differentiation (+3) before history (+18) |
| R2 | Envelope accumulator reset | **Decided**: 2 slots, reset on envelope exit only; no regime input needed |
| R3 | M2 estimator | **Deferred to Phase C/D by design**; slots reserved in A1 |
| R4 | Load axis | **Conditional on A3**; form constrained (axis, limit, physical grouping, pitch-first) |
| R5 | Initial state in grouped record | **Decided**: named field beside the list |
| R6 | Cereal version bump | **Decided**: bump, no migration, fail loud — constitution over habit |
| R7 | Non-regression band | **Proposed** → operator |
| R8 | Retry budget / abort signal | **Proposed** (3 attempts; σ=0.000 throttle lock) → operator |
| R9 | Offline study data source | **Decided**: dmp physics columns; `data.dat` already gone |
| R10 | Predictor target | **Decided**: **continuous current-bearing estimate** — horizon-free state estimator, scored vs hold-last-seen and binned by blind-gap age; the earlier reappearance-geometry framing is superseded |
| R11 | M2 aggressiveness lever | **Decided**: none; inherit |
| R12 | Tooling prerequisites | **Swept**; one prerequisite deleted (naming already exists) |
| R14 | Camera model vs ordered hardware | **Decided**: projection already equidistant (no action); adopt **V = 75°** via `CameraPixelsV = 200`, M2-only; import drift budget + 155 ms relock floor as predictor contract; 100 m corroborated; dual-camera and mode-switching deferred with triggers |
| R13 | Hardware accel source | **Decided**: INAV over MSP via `MSP2_AUTOC_STATE` extension — adds INAV as a third build surface in **two target variants** (bench = `MAMBAF722_2022A`, flight = `MATEKF722MINI`, bench first). Routine and precedented (021 T041). Disconnect GPS before flashing |

No `NEEDS CLARIFICATION` remains that blocks `/speckit.tasks`. R7 and R8 are operator confirmations that
set acceptance thresholds, not design.
