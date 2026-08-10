# Feature Specification: 041 — M2 Depth (observation-side objectives)

**Feature Branch**: `041-m2-depth`
**Created**: 2026-08-07
**Status**: Draft
**Input**: Operator scoping 2026-08-06/07. Handle the backlog first (especially anything with an M1
model incompatibility, since a rebake is owed anyway); then calibration; then two primary attempts —
(1) what balance can be struck between control aggressiveness and tracking, given that the streak
mechanism drives fitness but is never shown to the NN; (2) can the M2 predictor be made to produce
useful signal, given that at any step we do not know what to predict *to*. Full derivation, including
the corrections and retractions that produced it, is in [hypothesis.md](hypothesis.md).

---

## Why this feature exists

Two independent eliminations have converged. 037 established M2 depth is **reward-invariant** — not the
objective's scale ([[project_m2_tracking_ceiling]]). 040 established it is **not perception fidelity** — a
materially harder, much more honest camera moved competence not at all. What is left is the coupling
between them: **what the controller can observe about how well it is doing.**

The gap is concrete, not a metaphor. `FitStreakMultiplierMax = 5.0` ramping over `FitStreakRampSec = 5.0 s`
is the dominant fitness shaping, while the NN's entire perceptual window is **0.8 s**
(`kNNHistoryLagsMsec`). The reward is conditioned on **6× more history than the policy can observe** — a
Markov violation from the NN's seat. The same shape would appear a second time the moment a load objective
existed — it would penalize g-loading that no input reports — which is why 041 ships the accelerometer now
(FR-017a) and defers the objective itself (FR-019).

041 therefore adds **observations to match an objective that already exists**, rather than adding objectives.

---

## Clarifications

### Session 2026-08-07

- Q: How is `IN_ENVELOPE` computed for M1? → A: **Exact, and the computation moves into the per-tick eval
  path as the single source of truth for both the NN input and fitness.** Today the step score / streak is
  computed post-hoc in `computeScenarioScores` over recorded states; instead compute it during eval, record
  it from source, and have fitness consume that same result — so the input the policy sees and the reward
  it is scored by are by construction the same number. M1 is exact in **both sim and flight**: flight M1 has
  the same signals (a virtual target location and the chase's own location), so there is no sim-to-real gap
  in this input. **M2 differs** — its envelope is a camera-derived *estimate*, expected to get reasonably
  close in sim but never exact.
- Q: What must Study A show for a load axis to enter the A1 bundle? → A: **Nothing — no load axis in 041.**
  Study A **reports only**. The M1 bake carries exactly one behavioural lever: the envelope/streak input,
  answering "does streak in the RNN help or hurt". The load and per-regime aggressiveness findings feed the
  **next feature**, which is sequenced *after* the M2 predictor go/no-go so it can be designed
  intentionally rather than bundled speculatively. Accelerometer inputs still land in A1 (see FR-017a) —
  they are observations, not an objective.
- Q: What makes the H1a ablation answer "decisive"? → A: **B + C.** A pass requires *both* a measurable
  fitness drop **and** a behavioural signature (`pctInStreak` down or per-axis `dCtrl` up); fitness-only or
  behaviour-only is recorded as a weaker partial result, not a pass. And it must be **benchmarked against
  control-input ablations** of similar slot count, so "what a meaningful ablation looks like" has a
  within-run reference rather than an assumed threshold. Additionally: compare the new M1 against the prior
  M1 **per regime** (patrol / intercept / track-streak) to see whether the new controller shows
  *differentiated intent* across regimes where the old one did not.
- Q: Is xiao a compile gate only, or a deliverable? → A: **Compile + bench + flight test (option C)** —
  conditional: the flight test proceeds **unless the M1 result is an utter fail/reject**. So 041 carries a
  real firmware deliverable, not just a `nn2cpp` regeneration: the xiao must compute `IN_ENVELOPE` exactly
  (step-score cone geometry + thresholds in firmware) and maintain the `ENVELOPE_SECS` accumulator with a
  per-engage reset, and the accelerometer channels must be sourced on hardware.
- Q: Where do the accelerometer channels come from on hardware? → A: **INAV over MSP**, same path as
  `quat`/`gyro`. This requires **extending the custom `MSP2_AUTOC_STATE` consolidated command** so accel
  arrives in the *same single round trip* — so 041 carries an **INAV fork component**. *(Corrected
  2026-08-07: INAV builds routinely on this machine — vendored toolchain, existing
  `inav_8.0.0_MATEKF722MINI.hex` — so this is ordinary scope, not a toolchain risk. It does need **two
  target variants**: bench = `MAMBAF722_2022A`, flight = `MATEKF722MINI`, bench deployed first — both built
  together already in 021 T041. And the GPS module must be disconnected before flashing.)*
- Q: What exactly does the predictor output? → A: **A continuous current-bearing estimate (option C)** —
  every tick the head emits "where I believe the target is *now*", scored against truth whenever truth is
  available (visible ticks, and at reacquisition after a gap). During blindness it must dead-reckon.
  **Horizon-free**: this is a state estimator, not a forecast, which dissolves the "we don't know what to
  predict to" problem rather than relocating it into an unknown gap duration. Real-world analogue
  (operator): when you lose sight of the other aircraft you estimate which way they will go *and* which way
  you should go to get behind them — 041 builds only the first half (the estimate); the second half is what
  *consuming* it would enable. Combined offense/defense is explicitly a later thread. Expectation is
  calibrated: *"we can perhaps get some sort of signal by starting with C"* — not a breakthrough.
- Q: Is the head passive, or is its output consumed? → A: **Consumed — the prediction ERROR is fed back as
  an input (option C)**, not the estimate itself. When truth arrives the net receives "how wrong was I",
  which is non-redundant information (the estimate itself is derivable from the hidden state that produced
  it; the error is not). The head is therefore **no longer passive** — which is the documented reason the
  previous attempts returned nothing: t6 established that passive tie-break scoring does not couple to
  control. Operator notes this is *"fairly close to RL for each step"* and is the step already contemplated.
  ⚠️ **But it is not RL**: this is a GA, so the error is an **observation** the policy can condition on, not
  a learning signal that updates weights. Evolution still does all the weight updating.
- Q: Does the M2 bake use a real envelope estimator or zero-filled slots? → A: **A real direct-perception
  estimator (option B)**, populated and used in the D bake. M2 infers in-streak by examining perception and
  **accumulates the same way**, so the RNN receives *the same input channels* in both modes — only the flag's
  *source* differs (exact geometry in M1, perception in M2). The accumulator is an **external counter**
  maintained by the stepper/firmware, not a learned quantity. Rejected: deriving the envelope from the
  predictor's own bearing estimate, which would make one unproven thing depend on another.
- Q: Should the duration accumulator be log-scaled? → A: **Question withdrawn — and the framing corrects a
  mistake in this spec's earlier reasoning.** Operator: *"in tracking is probably the key NN input — the
  fitness maintains the multiplier, not the controller."* The **binary in-envelope flag is the primary
  input**; duration is secondary. First-principles support: the streak multiplier is *monotone* in duration,
  so with **no competing cost in 041's objective** (the load axis was deferred) there is never a moment where
  leaving the envelope is preferable — the optimal policy is simply "stay in", which needs the flag and not
  the depth. Depth becomes decision-relevant only once a competing cost exists, i.e. in the follow-on load
  feature. **Both slots still ship** (the format break happens once, and the follow-on will want the
  accumulator), with linear normalization — no log, no tanh remap — and ablation separates their
  contributions.
- Q: Should the in-cone condition also require the target to be visible? → A: **Raised, then deferred to the
  backlog — NOT in 041.** The defect is real (position-only scoring credits a chase that is inside the cone
  but pointed away, the head-on crossing case) and small (~one tick per crossing). But asking *"can you be in
  streak and not visible?"* exposed a design tree rather than a one-line fix: gate the score or only the
  streak, hard gate or grace window, hysteresis at the field-of-view boundary, and a geometric proxy for M1
  which has no perception. A hard gate would also punish every short excursion — harsh precisely in the
  regime a narrower field of view creates more of. It is an **objective** change, so it wants a baseline
  moment and a deliberate design, not a late addition. Filed with full nuance in
  [BACKLOG.md](../BACKLOG.md).
- Q: What result at C2 authorizes building the head into the D bake? → A: **It must beat the hold-last-seen
  baseline in the gap-age bins that actually occur (option B)** — a win confined to trivially-short bins does
  not authorize a 27 h bake. **Refinement**: the qualifying bins are set from the **measured blind-gap
  distribution** that Study B produces as a by-product, not from a guessed constant. Operator's rationale is
  why this matters: *"especially if FOV goes to 90° and we have short excursions out of view and turn
  correctly"* — narrowing the field makes excursions **more frequent and possibly short**, so the decisive
  bins may be **sub-second**, and an arbitrary ≥1 s threshold would set the bar in the wrong place. Let the
  distribution say where the mass is, then require the win there.

---

### Session 2026-08-10 — camera model alignment with ordered hardware

Source: [031 handoff](../031-beacon-camera/handoff-041-camera-model.md) +
[camera-era-knobs.md](../031-beacon-camera/camera-era-knobs.md). Camera-era hardware is **ordered** (InnoMaker
OV9281 ×2, 1.8 mm M12 fisheye, Pi 3A+) but weeks out; the ask was to nudge the sim optic closer to the real
lens before the next long run.

- Q: Switch the sim projection from pinhole to equidistant (f·θ)? → A: **Already done — no action.** 038 t9
  established the angular mapping and 040 T031 kept it while dropping per-axis normalization
  (`camera_projection.cc:158-184`): bearings are `θ` in **radians**, and `radPerPx` is a single uniform value,
  so the "constant angular resolution everywhere" property the handoff wanted is already exploited. ⚠️ Historical
  note worth carrying: t9's switch to equidistant **near-froze evolution** (elite unreplaced ~57 gens) because
  the rectilinear tan-stretch had been an accidental training aid at the frame edge. We are where the handoff
  wants us, but the path cost a run — do not re-open the projection casually.
- Q: Make FOV and projection parameters rather than constants? → A: **Already done — no action.** 040 retired
  `CameraFOVHorizontalDeg`/`VerticalDeg`; FOV is **derived** from `CameraPixelsH/V × CameraDegPerPixel` so field
  and resolution cannot disagree, and SC-012 proved all 15 assumed values substitute with no code change.
- Q: Adopt the handoff's H = 120° / V = 75° field? → A: **Yes, for the next M2 run.** H is already 120°
  (320 px × 0.375). **V is 90° and optimistic by 15°** — the concern the handoff raises. Fix is
  `CameraPixelsV 240 → 200`: 200 × 0.375 = **75° exactly**, and 320:200 = **1.6**, the real sensor's aspect
  (OV9281 is 1280×800), so the sim grid becomes a clean 4× downsample instead of a 4:3 invention. Operator:
  *"sure it adds another variable but it is likely more realistic."*
- Q: Does this affect M1? → A: **No — no change there.** M1 has no camera; it keeps its global view exactly as
  before. This is an M2-only change.
- Q: Single camera or the birded pair? → A: **Single camera stays.** The dual-camera fork goes to the backlog —
  and the operator adds a cost the handoff did not: a two-camera rig brings **inter-camera alignment and
  calibration variation**, compounding the per-scenario mount misalignment 040 already models. The handoff also
  softened its own case (single-fisheye penalty narrowed ×6.5 → **×2.2**), and it was conditioned on a marginal
  link budget, which no longer reads marginal.
- Q: Model on-the-fly camera-mode switching (the 453 fps crop-vs-bin two-regime structure)? → A: **Not now —
  future research**, but with a sharper framing than the handoff's: in tracker mode, **when the target is close
  and bright, go centre-bore** (narrow field for angular resolution) and stay wide otherwise. That is a
  target-state-driven mode switch, not merely acquisition-vs-tracked. Filed to backlog; gated on the A8-2
  register measurement that decides whether the fast mode is a crop or a bin.
- Q: Which lens gets bought? → A: **041's predictor verdict is an input to that decision** (operator): whether
  a 1.8 mm or a 2.x mm lens makes sense "depends on how useful predictors are here in 041." A working predictor
  tolerates a narrower field; a dead one demands a wider one. This makes the predictor go/no-go and the measured
  blind-gap distribution **hardware-purchase-relevant**, not just training-relevant.

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Retire the index-coupled failure class (Priority: P1)

A developer changing how per-tick data is recorded or consumed cannot silently introduce an off-by-one
between two collections that are related only by a comment. Before any objective change lands, the
codebase is swept for this shape and the structural fix is taken.

**Why this priority**: Four instances were found in one session; three were live bugs, one of them **the
M2 objective itself**, wrong since 030 and invisible in recorded data. One cross-cutting question found
what four features of feature-shaped work missed. Everything downstream in 041 changes recorded data, so
this precedes it.

**Independent Test**: The inventory exists and every entry is either fixed, structurally eliminated, or
covered by a zero-answer test. Fully valuable even if 041 stops here.

**Acceptance Scenarios**:

1. **Given** the codebase, **When** the scan completes, **Then** an inventory exists of every
   index-parallel collection pair crossing a producer/consumer boundary, every struct serving two
   lifetimes, and every value duplicated in two places.
2. **Given** per-tick recording, **When** state, camera view and target sample are grouped into one
   record, **Then** there is no second index available to be off by, and the invariant is
   unrepresentable-to-violate rather than documented.
3. **Given** a fitness term that pairs two recorded series, **When** a test constructs data whose correct
   answer is exactly zero, **Then** the term scores exactly zero, and a deliberately shifted input scores
   visibly worse.

---

### User Story 2 - One clean-slate contract break (Priority: P1)

Every pending change that is incompatible with the current M1 model or dmp schema lands in a single
commit, so exactly one rebake is owed rather than several.

**Why this priority**: The M1 model must be rebaked for 041's new inputs regardless. The whole M2
initiative is currently in a no-backward-compatibility phase (operator 2026-08-07), so there is no compat
tax to weigh against the clean option. Deferring any of these items means paying a second rebake later.

**Independent Test**: A fresh build records a dmp that is self-describing, carries realized wind, has
exact tick stamps, and is read by every consumer; the input count assertions and metadata tables agree.

**Acceptance Scenarios**:

1. **Given** the new input layout, **When** the build compiles, **Then** input-count assertions, the
   per-slot metadata tables, and the serialization contract all agree, and a mismatch fails at compile
   time rather than producing a corrupt run.
2. **Given** a recorded dmp, **When** a reader needs a fitness or cadence parameter, **Then** it comes
   from the dmp's own config block, not from whatever ini happens to be on disk.
3. **Given** a 20 Hz run, **When** tick timestamps are inspected, **Then** gaps are exactly 50 ms with no
   ±1 ms truncation jitter.
4. **Given** any recorded tick, **When** the realized wind is inspected, **Then** it carries the wind the
   FDM actually applied rather than zero.

---

### User Story 3 - Instruments that can answer the questions (Priority: P1)

Before spending a training bake, the questions 041 asks are made answerable: one instrument that measures
whether a trained policy *uses* an input, and one offline study that measures load and regime from data
already recorded.

**Why this priority**: There is **no control bake** in 041 (see Non-Goals), so the input-ablation tool is
the only rigorous instrument in the feature. The offline study decides whether a load objective is
justified *before* it is designed, and can disprove a hypothesis for the cost of an afternoon.

**Independent Test**: Ablating a known-load-bearing input on an existing elite produces a measurable
degradation; the offline study produces per-regime load and control statistics from a pinned run with no
new recording.

**Acceptance Scenarios**:

1. **Given** a trained elite and a named input column, **When** that column is zeroed and the elite is
   re-evaluated on the identical scenario set and seeds, **Then** the report gives Δfitness, per-axis
   Δ control-rate and Δ mean-magnitude, and a per-scenario Δ distribution.
2. **Given** a pinned run's recorded physics trace, **When** the offline study runs, **Then** it reports
   body-frame normal acceleration per axis and per regime, without any recording change.
3. **Given** the same study, **When** regimes are classified, **Then** every tick is assigned to
   tracking, intercept or patrol using the already-established rule, and control statistics are reported
   per regime rather than pooled.

---

### User Story 4 - Give the controller the tracking state it is paid for (Priority: P1)

The NN receives an observation of the reward regime it is being scored by — whether it is in the scoring
envelope and for how long — plus an accelerometer reading of the load it is generating. A new M1 model is
baked with these inputs and becomes the M2 source.

**Why this priority**: This is the feature's central hypothesis, and M1 is the only place it can be
tested without an estimator confound: M1's rabbit is a computed waypoint, so in-envelope is exact and
stays exact on real hardware via GPS.

**Independent Test**: The bake completes and clears the non-regression bar; ablating the new inputs on
the resulting elite says whether the learned policy depends on them.

**Acceptance Scenarios**:

1. **Given** a tick, **When** the NN gathers inputs, **Then** it receives an in-envelope indication and a
   normalized duration-in-envelope, both derived only from quantities available on real hardware, and both
   defined in milliseconds so a cadence change re-derives them.
2. **Given** a tick, **When** the NN gathers inputs, **Then** it receives specific force in body frame —
   what an accelerometer measures, gravity included — not FDM kinematic acceleration.
3. **Given** the completed bake, **When** its per-axis reports and peak load are compared against the
   historical band, **Then** tracking has not regressed.
4. **Given** the completed bake, **When** the new inputs are zeroed by ablation, **Then** the result
   distinguishes "the policy uses this state" from "the policy ignored it".

---

### User Story 5 - Decide the predictor's fate on evidence, not design (Priority: P2)

Whether the auxiliary predictor head can produce useful signal is settled by cheap offline measurement
and one ablation, before any bake is committed to it.

**Why this priority**: The current head is measured to carry no information — `r(Δspan) ≈ 0` at every
horizon, best error at generation 1 — while occupying a third of the lexicase pool, so it is actively
harmful rather than inert. Two experiments here can kill or redirect the whole branch for hours of work.

**Independent Test**: The offline study returns a verdict on whether the target's current bearing is
estimable through a blind gap — beating hold-last-seen, binned by gap age; the ablation returns whether the
dead head is taxing the control search.

**Acceptance Scenarios**:

1. **Given** recorded runs, **When** visible→blind→reacquire triples are extracted, **Then** the study
   reports whether pre-blindness history predicts re-entry bearing better than persistence, scored as
   variance explained against that baseline rather than as mean error.
2. **Given** the same study, **When** it completes, **Then** it also reports the blind-gap distribution:
   frequency, duration, and re-entry bearing offset.
3. **Given** the predictor head disabled, **When** a tracker run is compared against the head enabled,
   **Then** the result says whether returning a third of the selection pressure to the remaining axes
   helps.
4. **Given** no offline target clears its baseline, **When** the design decision is made, **Then**
   retiring the head is an accepted outcome, and the topology and lexicase pool shrink accordingly.

---

### User Story 6 - One M2 bake, scoped to the predictor question (Priority: P2)

A single tracker bake off the new M1 source carries whichever predictor design survived, or no head at
all, and reports whether prediction produced signal.

**Why this priority**: M2 already learns well; it does not need help learning. The aggressiveness work is
expected to reach M2 for free, both because the new inputs are generic across modes and because a calmer
source is an easier thing to chase. So the bake exists to answer the predictor question.

**Independent Test**: The bake completes and the predictor verdict is reportable independent of any
aggressiveness outcome.

**Acceptance Scenarios**:

1. **Given** the completed bake, **When** the predictor is judged, **Then** the verdict is stated as
   variance explained against a no-information baseline, at blindness-scale horizons.
2. **Given** the completed bake, **When** aggressiveness is examined, **Then** it is reported as an
   inherited observation rather than as a designed M2 experiment.
3. **Given** the completed bake, **When** generalization is measured, **Then** it is measured on a pinned
   novel-geometry source as well as the training set, and the single-pinned-set limitation is stated.

---

### Edge Cases

- **The M1 bake draws the stuck basin.** Documented rate is roughly 1 in 3 at this population and wind
  count. The tell is throttle *lock* — exactly 1.000 with zero variance — not throttle saturation, which
  climbers pass through legitimately. Repeat is a lottery re-draw, and the retry count and abort criterion
  are declared before the first attempt.
- **The bake exits before the variation ramp completes**, saving an elite at partial scale, so its
  evaluation can never test full variation.
- **A new input is added without its metadata row**, silently misaligning names against slots.
- **Specific force versus kinematic acceleration** — feeding the wrong one puts a constant 1 g offset in
  the most load-relevant axis and would train against a signal no accelerometer produces.
- **Head-on crossing** — the chase passes through the scoring cone with the target outside any field of
  view, and position-only scoring credits it. **Known and deliberately unaddressed in 041** (deferred to
  backlog); the envelope flag will therefore assert "in envelope" for ~one tick per crossing while the
  controller cannot see the target. Small, and now written down rather than latent.
- **A sim accelerometer is unrealistically clean** compared to the real sensor's noise and bias.
- **The in-envelope accumulator's reset rule** — on envelope dropout only, or on any regime change.
- **The eval configuration file is left pointed at a previous exercise**, so a run reproduces the wrong
  scenario shape. The existing 1:1 seed-table guard catches count mismatches only.
- **A recorded run's per-tick trace is overwritten** by the next bake's launch.
- **A pinned baseline's weights cannot be loaded** by a later binary once the schema moves.

---

## Requirements *(mandatory)*

### Functional Requirements — research and structural (US1)

- **FR-001**: The feature MUST produce an inventory of index-coupled contracts: every collection pair
  related by a shared index across a producer/consumer boundary, every struct serving both an RPC and a
  persisted lifetime, and every value duplicated across two definitions.
- **FR-002**: Per-tick recording MUST group co-temporal data into a single record so that no second index
  exists to be misaligned. The pre-loop initial state MUST be handled by an explicit, stated choice rather
  than by accident.
- **FR-003**: Every fitness term that pairs two recorded series MUST have a test whose correct answer is
  exactly zero, plus a companion test proving a shifted input scores visibly worse.
- **FR-004**: The auxiliary prediction score MUST be paired with the camera view and state from the same
  tick. *(This changes fitness and MUST NOT land while a bake is live.)*

### Functional Requirements — contract bundle (US2)

- **FR-005**: All **M1-affecting** model- and schema-incompatible changes MUST land in one commit, before the
  first bake of the feature and never during a live bake.
- **FR-005a**: One **M2-only** layout change is permitted *after* that commit: the tracker innovation
  channels (FR-025d), which land in the M2 phase once T088 fixes the estimate's dimensionality. This does not
  violate FR-005, and the reason is load-bearing rather than incidental:
  - `TrackerInputs` and `NNInputs` are **separate structs with separate genomes**, so growing the tracker
    vector cannot invalidate the M1 model.
  - 040's **T023 split of `AircraftState::serialize`** was built precisely so tracker-input growth needs **no
    M1 source rebake** — the M1 source dmp stays readable by the M2 binary across the change.
  - Consequently the final tracker count is **63 + N**, N = the estimate's dimensionality, resolved at T088;
    every artifact quoting "63" is quoting the *post-A1, pre-M2* value.
  ⚠️ Any *other* post-A1 layout change is a violation. This exemption is specific and does not generalize.
- **FR-006**: Each per-gen dump MUST carry the fitness and cadence configuration used to produce it;
  readers MUST prefer the recorded configuration over any on-disk configuration file.
- **FR-007**: Tick timestamps MUST be stamped so that a run at the configured cadence records exact
  intervals, with no truncation jitter. *(Determinism-affecting; submodule pointer bump first.)*
- **FR-008**: Each recorded tick MUST carry the realized wind the flight model applied.
- **FR-009**: Schema changes MUST NOT introduce version negotiation or compatibility shims. Per
  Constitution V's write-side contract, the artifact's **version field MUST be bumped** for a transition
  the project is committed to, and readers of an older artifact MUST **fail loudly** identifying both
  versions — never silently truncate or default-initialize. *(This resolves in favour of the constitution
  over the informal "never bump" habit; see plan.md Constitution Check.)*
- **FR-010**: Every pinned dump MUST have the corresponding NN weight file archived alongside it, because
  a dump preserves numbers while only the weight format preserves a controller that can be re-flown.

### Functional Requirements — instruments (US3)

- **FR-011**: A genome ablation capability MUST accept a named input mask and an existing genome, evaluate
  it on the identical scenario set and seeds, and report Δfitness, per-axis Δcontrol-rate,
  Δmean-magnitude, and a per-scenario Δ distribution. Named columns MUST come from the existing per-slot
  metadata tables. Weight-block masking beyond input columns is out of scope.
- **FR-011a**: The ablation report MUST break results down **per regime** (patrol / intercept /
  track-streak), not pooled — a signal that only appears in one regime is the expected shape of the
  hypothesis, and pooling would hide it.
- **FR-011b**: The H1a read MUST include **control-input ablations** of comparable slot count to calibrate
  the scale, spanning known-critical to known-marginal inputs (e.g. a target-bearing channel, the
  arena-inward vector, the rate gyros). A verdict on the envelope inputs MUST be stated relative to that
  spectrum, never against an assumed absolute threshold.
- **FR-011c**: The prior M1's per-regime behavioural profile MUST be derived **offline from its recorded
  dmp**, not by re-evaluating it. ⚠️ The prior M1 genome has 37 inputs and **cannot be loaded by a 041
  binary** — the input-count change makes it unrunnable, which is the "baseline weights expire" trap in a
  new form. Profiling from recorded per-tick data is the mitigation and it requires no re-flight.
- **FR-012**: An offline analysis MUST classify every recorded tick into tracking, intercept or patrol
  using the established threshold-and-closing rule, and report control and load statistics per regime.
- **FR-013**: The offline analysis MUST derive body-frame normal acceleration from the already-recorded
  physics trace, requiring a reader rather than a recording change.

### Functional Requirements — aggressiveness (US4)

- **FR-014**: The NN input vector MUST include an in-envelope indication and a normalized
  duration-in-envelope, in both pathgen and tracker modes. **The binary flag is the primary input**; duration
  is secondary — the fitness machinery maintains the multiplier, the controller does not (operator
  2026-08-07). Duration normalizes **linearly** against the ramp duration; no log or tanh remap.
- **FR-014a**: The accumulator MUST be an **external counter** maintained by the stepper (and by firmware
  on-target), computed identically in both modes. Only the flag's *source* differs — exact geometry in M1,
  perception in M2 — so the RNN's input channels are mode-independent.
- **FR-014b**: The H1a ablation MUST separate the two channels: flag alone, duration alone, and both. The
  expectation is that the **flag** carries the effect; a duration-only effect would be a distinct and more
  surprising finding.
- **FR-015**: Those inputs MUST be derived only from quantities obtainable on real hardware. The fitness
  machinery's internal streak state MUST NOT be fed to the NN.
- **FR-016**: Duration MUST be expressed in milliseconds against the ramp duration, so a cadence change
  re-derives it rather than silently rescaling it.
- **FR-017**: The NN input vector MUST include specific force in body frame — gravity included, as an
  accelerometer measures — in both modes.
- **FR-018**: In M1, in-envelope MUST use the **exact** condition — the same step-score/threshold geometry
  the objective uses — in **both sim and flight**. Flight M1 carries the same signals (virtual target
  location + chase location), so this input has no sim-to-real gap. Any estimator is M2-only.
- **FR-018a**: The step-score / streak computation MUST be performed **once per tick in the eval path** and
  consumed by **both** the NN input gather and the fitness accumulation — a single source of truth. It MUST
  NOT be computed independently in two places, and the per-tick result MUST be recorded so the source dmp
  carries it.
  *Rationale*: the feature exists because the reward is invisible to the policy; computing the same quantity
  twice would reintroduce the possibility of the input and the reward disagreeing. This supersedes the
  current post-hoc-over-recorded-states arrangement in `computeScenarioScores`.
  ⚠️ **Fitness-affecting** → belongs in the A1 bundle, never mid-bake. Moving the computation from recorded
  states to live states MUST be verified numerically identical, or the change is a silent objective change.
- **FR-017a**: The accelerometer inputs land in A1 **even though 041 adds no load objective**. Rationale:
  the format break is being taken once, and the aggressiveness feature that follows would otherwise need
  its own break plus a second M1 rebake. They complete the 6-DOF craft-state block that already carries
  `GYRO_P/Q/R`.
- **FR-019**: 041 MUST NOT add a load objective of any kind. Study A **reports** load and per-regime
  control statistics; those findings are input to a **follow-on feature**, sequenced after the M2 predictor
  go/no-go. Any future load objective MUST be expressed against a physical limit rather than a tuned
  coefficient, MUST be a population-diverse selection axis rather than a scalar penalty, and MUST group
  axes physically — recorded here so the follow-on inherits the constraints.
- **FR-020**: The feature MUST NOT add a control-amplitude or control-rate smoothness objective, MUST NOT
  add a throttle penalty, and MUST NOT add a load axis. *(Rationale in Non-Goals.)*
- **FR-020a**: The M1 bake MUST carry exactly **one behavioural lever** — the envelope/streak input. Every
  other change in the bundle is either an observation (accelerometer), a structural fix, or a recording
  change. Consequently **every new input is ablatable**, so per-input attribution is fully recoverable
  post-hoc and no un-attributable change enters the bake.
- **FR-021**: The production M1 bake MUST run at the production population and wind count, with retry
  count and abort criterion declared beforehand, and climb judged on streak-fraction metrics rather than
  completions.
- **FR-022**: The final M1 and M2 runs' per-tick traces MUST be preserved rather than overwritten by a
  subsequent launch.

### Functional Requirements — hardware deployment (US4, conditional)

- **FR-022a**: The xiao firmware MUST compute `IN_ENVELOPE` and `ENVELOPE_SECS` on-target with the same
  semantics as sim: the step-score cone geometry (`FitDistScaleBehind`/`Ahead`, `FitConeAngleDeg`) thresholded
  at `FitStreakThreshold`, and a duration accumulator that **resets on engage** as well as on envelope exit.
  This is firmware work, not a codegen regeneration.
- **FR-022b**: The accelerometer channels MUST be sourced from **INAV over MSP**, on the same path as
  `quat`/`gyro`, and MUST match the sim's semantics — body-frame specific force including gravity.
- **FR-022e**: The custom **`MSP2_AUTOC_STATE`** consolidated command MUST be extended so accel arrives in
  the **same single round trip** as the existing fields. The 021 gyro extension is the template to copy:
  append at the end of the payload, fixed integer scaling with the factor stated at the write site, and the
  INAV axis/sign convention documented for the consumer. *(Site:
  `~/inav/src/main/fc/fc_msp.c`, the `MSP2_INAV_LOCAL_STATE` case; most recent fork commit `63cffaf4f`
  "extend MSP2_AUTOC_STATE with filtered gyro rates" is the precedent.)*
- **FR-022f**: The INAV axis and sign convention for accel MUST be resolved against
  `COORDINATE_CONVENTIONS.md` and pinned by a test, exactly as the gyro extension required the consumer to
  negate pitch and yaw. ⚠️ This is a known trap class here — a board-alignment discrepancy already put a
  ~10° pitch bias into flight data once.
- **FR-022h**: The accel MUST be sourced from INAV's **transformed** state — `acc.accADCf`, which is
  post-`applySensorAlignment`, post-`applyBoardAlignment`, and normalized to g — and MUST NOT come from any
  pre-alignment raw sensor value (`accADC`, `acc.dev.ADCRaw`). This mirrors `gyro.gyroADCf`, which the
  existing extension already uses correctly. *Rationale*: a raw read would embed each controller's own
  board misalignment into the NN input, differently on the bench board (roll = −16) than on the flight
  board — invisible in sim, wrong in flight, and not detectable from recorded data.
- **FR-022g**: The added payload MUST NOT push the MSP cycle past its loop budget. 039 measured zero
  overruns at 115200 with the current payload; if headroom proves marginal, the unexercised 460800
  baud-raise lever is the documented next step rather than dropping the field.
- **FR-022c**: Desktop/firmware parity MUST be demonstrated on the bench before flight: the generated
  forward pass reproduces the desktop forward pass on a fixed input vector, and the new inputs report sane
  values against a known geometry.
- **FR-022d**: A flight test of the new M1 IS in scope and proceeds **unless the M1 result is an utter
  fail/reject** — i.e. the bake fails the non-regression bar, or H1a returns a clear fail with no
  behavioural change. The go/no-go MUST be recorded with its reason either way.

### Functional Requirements — predictor (US5, US6)

- **FR-023**: Predictor feasibility MUST be established offline, on recorded data, before any bake is
  committed to a predictor design.
- **FR-024**: Predictor targets MUST be scored as variance explained against an explicit no-information
  baseline, never as mean absolute error against the quantity's level.
- **FR-024a**: The C2 → D go/no-go rule MUST be: the head beats hold-last-seen **in the gap-age bins where
  real excursions occur**, by a margin stated in advance, with adequate per-bin sample counts. Those
  qualifying bins MUST be derived from the **measured blind-gap distribution** rather than a guessed
  threshold — narrowing the field of view is expected to make excursions more frequent and shorter, so the
  decisive bins may be sub-second.
- **FR-024b**: The measured blind-gap distribution MUST be produced and recorded **before** the go/no-go is
  taken, since it defines the criterion. It is also the input to any future field-of-view decision, so it is
  a deliverable in its own right regardless of the predictor verdict.
- **FR-025**: If a predictor head is retained, its target MUST be a **continuous estimate of the target's
  current bearing** — emitted every tick, including blind ticks — **not** a fixed-horizon forecast and not a
  reappearance event. It is a state estimator: the horizon is always "now".
- **FR-025a**: Scoring MUST use truth wherever truth exists — every visible tick, plus the reacquisition
  tick that terminates a blind gap. Blind ticks MUST NOT be excluded by visibility gating; the gap is the
  regime the head exists for. *(The current objective CEP-gates both endpoints, scoring only where
  prediction is information-free — that exclusion is the defect being corrected.)*
- **FR-025b**: The no-information baseline for this target is **hold-last-seen bearing** (dead-reckoning
  with zero rate). Persistence is trivially strong on visible ticks and collapses across a gap, so the
  reported statistic MUST be broken out **by blind-gap age** — a single pooled number would be dominated by
  the easy visible ticks and would hide the only regime that matters.
- **FR-025c**: The head MUST be **actuated, not passive** — the **prediction error** (innovation) MUST be
  fed back into the tracker input vector on the following tick. The estimate itself MUST NOT be fed back:
  it is derivable from the hidden state that produced it, whereas the error is not.
  *Rationale*: t6 established that passive tie-break scoring does not couple to control. A re-targeted but
  still-passive head would risk reproducing that null result with a better target.
- **FR-025d**: Error input channels MUST mirror the estimate's dimensionality and carry **signed** values
  per axis — the direction of the error is what says which way the model is biased, and a magnitude alone
  discards it. `TrackerInput` grows accordingly (tracker-only; `PathgenInput` is unaffected, as the
  predictor has always been M2-only).
- **FR-025e**: During blindness no new truth exists, so the error channels MUST **hold their last computed
  value** rather than being zeroed — zero would falsely assert "my model is correct". Staleness is already
  observable via the existing `TIME_SINCE_SEEN` input, so the pair (held error, time since seen) lets the
  policy represent "my last error was large and it has been N seconds" **without any new staleness slot**.
- **FR-025f**: The feedback path MUST be implemented in the stepper — compute the error when truth arrives,
  write it into the next tick's input vector. It MUST NOT require output-layer recurrence; the output layer
  stays non-recurrent.
- **FR-026**: A predictor output MUST be scaled into the target quantity's domain so the usable output
  range is not a small fraction of one unit.
- **FR-027**: Retiring the head entirely — shrinking the output topology and the selection pool — MUST be
  an accepted outcome.
- **FR-028**: The M2 bake MUST NOT introduce an M2-side aggressiveness lever; aggressiveness is inherited
  and observed.

### Functional Requirements — camera model (US6, M2-only)

- **FR-029**: The M2 camera's vertical field MUST be **75°**, achieved by setting `CameraPixelsV = 200` against
  the existing `CameraDegPerPixel = 0.375` (200 × 0.375 = 75°). Horizontal stays **120°** (320 px). `radPerPx`
  MUST NOT change, so per-pixel quantisation and CEP are unaffected.
  *Rationale*: the ordered 1.8 mm fisheye on an OV9281 estimates to ~124° × 78° equidistant; 120 × 75 is the
  conservative split, and 320:200 = 1.6 matches the real 1280×800 aspect. The prior 240 px (90°) was a 4:3
  invention that made the vertical tracking envelope optimistic by 15°.
- **FR-029a**: This is **M2-only**. M1 has no camera and keeps its global view unchanged.
- **FR-029b**: ⚠️ **Fitness-affecting** (more sentinel ticks, more and longer blind gaps) → A1 bundle, never
  mid-bake. The derived-FOV assertion comment in `camera_projection.h` (currently citing ±0.785 rad / 45° for
  V) MUST be updated so the documented half-angles match the grid.
- **FR-030**: The predictor's contract MUST record the physical drift budget from
  [camera-era-knobs.md](../031-beacon-camera/camera-era-knobs.md) §3: blind-interval bearing growth
  Δθ ≈ ½·a_target·t²/r + 1–2° IMU feed-forward error, giving field-exit times for a 3 g target of ~1.1 s @50 m
  and ~1.5 s @100 m at a ±36° half-field. This sets the **timescale** the predictor operates on — order 1 s,
  not sub-second — and is an independent cross-check on Study B's measured gap distribution.
- **FR-031**: The predictor's contract MUST record that **warm reacquisition has a floor of ≈155 ms**
  (N/f_chip, N = 31 Gold at ~189 Hz chips). A perfect predictor pointing perfectly still waits that long for
  code relock. Consequence: **the predictor's value is in pointing, not in latency** — any claim that it reduces
  time-to-reacquire below the code-lock floor is unphysical.
- **FR-032**: `CameraDetectionRangeM = 100.0` MUST remain as-is. It was an assertion (FR-033a in 040) and is now
  **independently corroborated** by the 031 photon budget: bright-day post-correlation SNR ≈22 @100 m, ÷4 at
  4×4 defocus → ≈5.5 against a ×4.5 lock threshold ⇒ 100–110 m class. Recorded as a strengthened assumption, not
  a changed value. *(This does not contradict 040's gloomy single-PD figure — the small pixel's tiny sky patch
  is a different budget.)*

### Key Entities

- **Per-tick record**: the grouped unit replacing today's parallel per-tick collections — chase state,
  camera view, target sample — with one index.
- **Regime**: a per-tick classification into tracking, intercept or patrol, derived from the in-cone
  location metric and closing rate. Already the analytics vocabulary; becomes an NN observation and an
  objective condition.
- **Envelope occupancy**: the in-envelope flag plus normalized duration; the observation-side counterpart
  of the streak multiplier.
- **Specific force**: body-frame accelerometer reading including gravity; the observation-side counterpart
  of a load objective.
- **Load**: body-frame normal acceleration per axis, derived from the recorded physics trace.
- **Reappearance geometry**: target bearing and separation at re-entry after a visibility gap; the
  candidate predictor target.
- **Pinned comparator**: a run whose dump objects are retained and whose weights are archived.

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The index-coupled inventory covers the codebase, and every entry is fixed, structurally
  eliminated, or covered by a zero-answer test. No known instance remains asserted only by comment.
- **SC-002**: All model- and schema-incompatible changes land in exactly one commit, and exactly one M1
  rebake is owed for the feature.
- **SC-003**: The full test suite passes on the bundle commit, with tick-stamp exactness, realized-wind
  recording, and self-describing dumps each covered by a test.
- **SC-004**: Input ablation on an existing elite reproduces the unablated fitness exactly when the mask
  is empty, establishing the instrument is sound before it is trusted.
- **SC-005**: The offline study reports load and control statistics for all three regimes on at least two
  pinned runs, from recorded data only.
- **SC-006**: The M1 bake completes at production scale and shows **no tracking regression** against the
  historical band. This is the bar — improvement is welcome, non-regression is success.
- **SC-007**: Ablating the envelope-occupancy inputs on the M1 elite yields a **pass** (fitness drop **and**
  behavioural shift, larger than the marginal end of the control-ablation spectrum), a **partial** (one but
  not both), or a **fail** (neither) — stated per regime and relative to the control ablations. Any of the
  three closes the hypothesis; only an unclassifiable result fails this criterion.
- **SC-007a**: A per-regime behavioural comparison between the new and prior M1 exists, answering whether
  the new controller differentiates intent across patrol / intercept / track-streak where the prior one did
  not. Derived from recorded data on both sides.
- **SC-008**: Per-axis aggressiveness and peak load are reported per regime for the new M1 elite, making
  the "aggressive in patrol, costly in intercept" claim a measurement.
- **SC-009**: The predictor question is answered — a target that clears its no-information baseline
  offline, or a decision to retire the head — before the M2 bake starts.
- **SC-010**: The M2 bake completes and reports the predictor verdict, the inherited aggressiveness
  observation, and novel-geometry generalization against the training set.
- **SC-011**: Both comparator runs remain retrievable for the duration of the feature, with weights
  archived.
- **SC-011a**: Desktop/firmware forward-pass parity is demonstrated on the bench, and the new inputs read
  sane values on-target, before any flight.
- **SC-011b**: Either a flight test of the new M1 is flown and reported, or the no-go is recorded with the
  M1 result that caused it. Silence on this point is not an acceptable outcome.
- **SC-012**: Every hypothesis the feature tested is recorded as supported or refuted with its evidence.
  A refuted hypothesis is a successful outcome.

---

## Non-Goals

Named explicitly, because each has a standing rationale that the spec must not quietly reverse.

- **Smoothness objectives of any shape** — control-amplitude or control-rate penalties. Tried repeatedly:
  they deaden the paths and are **generally beaten by aggressive manoeuvres** (operator 2026-08-06). This
  is a competitive verdict on the class, not a parameterisation problem. The replacement is an
  *observation* plus a *physical* limit, not a better-shaped penalty.
- **A throttle penalty** — throttle tends high because the chase is the same aircraft model as the target,
  so a twin-airframe tail chase needs the performance. Structural to the experiment; the fix is airframe
  asymmetry, which is a separate backlog item.
- **A control bake / micro A/B** — with PRNG, model and fitness all moving between runs, a cross-run delta
  measures everything at once. Small-population runs are **smoke tests** for plumbing and ballpark
  sanity, not comparison arms. Attribution comes from within-build ablation instead.
- **Choosing the camera field of view or topology** — out of scope, but 041 is a *load-bearing input* to that
  choice, so the frame matters. The live outcome space (operator 2026-08-10) spans **~90° single camera** (wins
  if 120° is marginal on photon budget *and* the predictor carries the extra blind time), **120° single**
  (status quo), and **~270° from several cameras** (*"if we just can't get there without, so be it"*).
  Narrowing buys range quadratically (SNR ∝ 1/FOV²) and costs blind time ∝ √(half-field) — so **a working
  predictor is what makes a narrow field affordable, and a dead one forces the field wider.** That is why the
  go/no-go and the measured blind-gap distribution must be recorded with their evidence: they feed a hardware
  purchase (1.8 mm vs 2.x mm), not just a training config. Full frame in [BACKLOG.md](../BACKLOG.md).
  The V = 75° change (FR-029) is a *fidelity correction to match ordered hardware*, not an exploration of this
  trade.
- **The birded two-camera pair** — deferred to backlog. Beyond the input-vector cost (the beacon block roughly
  doubles) it brings **inter-camera alignment and calibration variation** on top of the per-scenario mount
  misalignment already modelled. Single camera stays; the single-fisheye signal penalty is only ×2.2, and the
  link budget that would have forced the pair does not read marginal.
- **On-the-fly camera-mode switching** — future research, but its shape is decided: **an NN output knob**
  (operator 2026-08-10), target-state-driven — *close and bright ⇒ go centre-bore*, wide otherwise. The two
  hard parts are consequences of actuating it: **transition lag** (a crop change plus code relock is ≥155 ms,
  so the policy must *anticipate* rather than react) and **mode state fed back as an input**. That feedback is
  the **same wiring 041 builds for the predictor innovation** (FR-025c–f), so this item inherits a working
  template if 041 lands it — a reason to sequence it after. Gated on the A8-2 measurement of whether the
  453 fps mode is a centre crop or a bin.
- **Learned cross-camera registration** — if the dual-camera path is ever taken, the auto-correlation and the
  shifts/warps that bring two uncalibrated views into one frame are plausibly **a learned function rather than
  a calibration constant** (operator 2026-08-10). Notably this *inverts* the alignment-variation objection —
  a registration net trained against misalignment makes uncalibrated mounting the thing it is robust to — at
  the cost of a second trained stage inside the perception front end. "Something for later."
- **Perception changes** — projection model, camera variation deltas, detection-envelope realism, CEP
  physical model. 040 closed perception.
- **Flight-model re-tune items** — pitch marginal-stability levers, propeller correction. Gated on more
  than one flight article.
- **Two-timescale recurrence**, **hull-crash selection dimension**, **negative-ahead reward**,
  **two-sim co-evaluation**, **island-model selection**, **differing chase and target craft**, **online
  craft identification** — all retain their existing triggers.
- **Total energy as an input or objective** — promoted to adjacent, since "aggressive = more energy" makes
  it the natural cost unit, but out of this feature's bake: one lever at a time.
- **A load objective / axis** — 041 measures load and reports it; it does not act on it. The aggressiveness
  feature is deliberately sequenced **after** the M2 predictor go/no-go so it can be designed against the
  Study A findings *and* a settled predictor decision, rather than guessed at now. Accelerometer inputs
  ship anyway (FR-017a) so that feature needs no second format break.

---

## Dependencies & Assumptions

- **Assumption**: the whole M2 initiative is in a no-backward-compatibility phase (operator 2026-08-07),
  so structural fixes are preferred over compatibility-preserving instrumentation.
- **Assumption**: fixing aggressiveness on M1 carries most of the way into M2 unaided, via generic inputs
  and an easier source trajectory. FR-028 depends on this; if it proves false, an M2-side lever becomes a
  follow-on feature rather than a mid-feature addition.
- **Dependency**: both comparator runs pinned and retained. **Satisfied 2026-08-07** — the M1 source and
  the M2 comparator are each verified retained object-by-object.
- **Known limitation**: the second novel-geometry evaluation source is deliberately not retained, so a
  novel-geometry result cannot be cross-checked against an independent second sample. Regenerating
  produces a different, non-comparable set.
- **Open**: confirmation that "calibration" here means re-establishing the comparator set — new M1
  baseline, archived weights, novel source re-run — rather than re-calibrating the perception model's
  assumed physical values, which have no new measurements to inform them.

### New dependency introduced by the hardware decision (2026-08-07)

- **An INAV fork change is now in scope** — `MSP2_AUTOC_STATE` gains accel fields. Three build surfaces in
  this feature, not two: autoc/crrcsim, xiao, **and INAV** — the last in **two established target variants**:
  bench = **`MAMBAF722_2022A`**, flight = **`MATEKF722MINI`**. Bench is built and deployed first; a change
  validated only on the bench target is not validated for flight.
- **INAV builds routinely on this machine** (corrected 2026-08-07): vendored ARM toolchain in-tree,
  `build/inav_8.0.0_MATEKF722MINI.hex` already present. Ordinary scope, not a toolchain risk.
- ⚠️ **Hardware procedure: remove the GPS before flashing an INAV controller.** Standing quirk; it belongs in
  the runbook because forgetting it costs a debugging session.
- **Bundling opportunity, same "now is the time" logic as the format break**: if INAV is being built and
  flashed anyway, the queued `mspOverrideInit` first-frame patch (backlog C1 — MSPRCOVERRIDE engage pays a
  spurious 200 ms floor) can ride along at near-zero marginal cost. Decide deliberately rather than
  discovering later that the opportunity passed.
- **Flight deliverable is conditional** (FR-022d) — so the INAV work is only wasted if the M1 result is an
  utter reject, and even then the MSP extension keeps its value for every later flight.
