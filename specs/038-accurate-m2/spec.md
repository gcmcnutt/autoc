# Feature Specification: 038 Accurate M2 — Hull-Crash Penalty + Camera Variations

**Feature Branch**: TBD (likely `038-accurate-m2` when unparked; currently drafted inside the
`037-20hz-control-loop` working branch alongside the 037 wrap, mirroring how 031 was drafted
in-branch).
**Created**: 2026-06-16
**Status**: DRAFT — spec + clarify only, no plan/tasks yet.
**Input**:
- Operator direction 2026-06-16: open 038 to make M2 (tracker) *robust and flight-safe*. Headline
  feature items: **hull-crash penalty** (ahead of camera) and **camera variations**. Preceded by a
  short **front-matter cleanup** the operator wants done first. "We could start over from M1 if
  needed" — settled by a spike inside the camera story.
- Backlog source items (`specs/BACKLOG.md`):
  - "Hull-crash-cost as a lexicase fitness dimension (M1/M2)" (FUTURE FEATURE; operator-sequenced
    *after 037*; constraints from 2026-06-04).
  - "Forward loop — craft + camera variations into M1" (basin-landscape entry, 2026-05-27); the
    `cameraSeed` insertion point reserved in `include/autoc/rpc/scenario_metadata.h:98` (FR-020).
  - "[BACKLOG 038] Standardize training reporting" (one `scripts/` wrapper, logfile → all PNGs).
  - "M1 basin-landscape" entry → the 033 PRNG validation/bug-hunt sub-thread.
  - "Self-describing dmp" → the renderer-reads-less-from-.ini sub-thread.
  - "Lexicase epsilon = constant 0.5 doesn't scale" (`project_lexicase_mad_epsilon`).

## Program context — three parallel tracks (operator, 2026-06-16)

038 is one of three tracks running concurrently. Keeping them distinct keeps 038's scope honest:

| Track | Owner feature | Goal | Relationship to 038 |
|---|---|---|---|
| **xiao keeps up for M1 flights** | **037 closeout** | xiao runs the 20 Hz control loop in real flight — resolve the **local-IMU** question, **fully unroll** the NN forward pass, land a **defined, tweakable latency**. T009/T010 (camera-fps / transport-ceiling research) are effectively selected by the 20 Hz GO. | **Ahead of 038.** A baseline M1 flight needs this; it is *not* 038 work. 038 must not absorb it. |
| **M2 robustness** | **038 (this spec)** | hull-crash penalty + camera variations → an M2 that tracks closely **without colliding** and **generalizes across the camera envelope**. | This feature. |
| **Camera design** | **031** | real beacon/camera/recording hardware; eventually a field-calibrated noise model. | 038 camera variations run on **parameterized σ now**, recalibrated from 031 field data later. Not gated on 031. |

## Clarifications

### Session 2026-06-16

- Q: What pillars should 038 cover? → A: **Hull-crash penalty + camera variations.** Full M2
  *environment parity* (chase re-simulated in the source's own air mass) is **dropped** — "not needed
  at this level of accuracy" (operator). It stays a backlog item, not 038 scope.
- Q: Ordering of the two feature items? → A: **Hull-crash is ahead of camera.** A colliding controller
  is unflyable regardless of camera robustness — safety first, then generalization.
- Q: Is the "source-side vs M2-side camera variation" decision a standalone first user story? → A:
  **No — fold it into the camera story as its opening spike step.** (The earlier US0/US2 split was
  confusing.) The spike still settles the "re-bake M1?" question, but as step 1 of US2, not a P1 of
  its own.
- Q: Anything ahead of the feature work? → A: **Yes — a Phase-0 front-matter cleanup** the operator
  wants first: (1) **033 PRNG validation**, (2) **renderer reads less from `.ini`**, (3) the
  **standardized report script** (one wrapper, logfile → all PNG reports). Usual front-of-feature
  tech-debt.
- Q: How should 038 camera-variation values relate to 031? → A: **Parameterized σ now, calibrate
  later.** Sensible σ defaults immediately (same play as 034 craft variations); recalibrate from 031's
  first paired-craft field recordings when they exist.
- Q: MAD-relative lexicase epsilon? → A: **Conditional (P3).** May be needed once camera+craft push to
  5–6 variation dims; t11-m2 so far looks fine, so include only if the added dimensionality degrades
  selection signal.

### Session 2026-06-16 (hull-crash mechanism — operator)

- Q: Do we still want explicit "early crash worse than late"? → A: **No — reframe.** Today's
  score-stop **already** encodes timing economically: an early crash banks little (short track), a
  **late crash after a long good track banks a huge score**. The real gap is the *late* banked crash,
  not the early one. So the penalty does **not** need a tick-weighting term; it needs to claw back the
  late crash regardless of accumulated score.
- Q: OOB vs hull crash? → A: **Keep today's score-stop for OOB** (out-of-bounds; M1 works fine).
  **Hull crash** — infrequent but costly — gets a stronger, member-level penalty.
- Q: What mechanism? → A: **Start simple and member-level**, two candidates: **(A) throw out the
  entire member** on any hull crash (death-penalty / a crashing member is dominated by any cleaner
  member), or **(B) multiply the member's whole fitness by 0.5 per hull crash** (gentler, preserves
  genes, still a ~50% hit for one rare strike). This is a **constraint / aggregate multiplier**, NOT a
  per-scenario lexicase axis and NOT scalar-objective compositing — so it sidesteps the 033 Pareto
  collapse (`project_scalar_multiobjective_collapse`). Rationale: "99 out of 100 [clean] is still
  pretty bad" — a ~1 % crash rate must be **decisively** worse than 0 %.
- Q: Do the M2 sensors carry enough info to *avoid* the hull (know the rabbit / optimal point /
  imminent contact)? → A: **Partially — [NEEDS RESEARCH].** The NN senses beacon-pair **span**
  (distance proxy) + **span-rate** (closure) → it can perceive *frontal* closing. But it has **no
  explicit target-hull-proximity input** (the `dist_to_boundary` input is the arena cylinder = OOB,
  not the target), the beacon signal **degrades as the target fills/exceeds the FOV** (out-of-frame /
  CEP-gated right when a strike is imminent), and a **from-behind overshoot** ("pass the rabbit and it
  runs over us") is largely unperceivable with a forward camera. The penalty selects *against*
  dangerous geometries; whether the NN can *actively avoid* a strike may require an explicit
  target-range / hull-proximity input — open research, not assumed.

### Session 2026-06-16 (optical-only inputs + chosen mechanism — operator)

- Q: Add a physical target-range / hull-proximity input to help avoidance? → A: **No. M2 is
  optical-only by design** — the NN infers from the two beacon points + **RNN situational awareness**
  (the substrate already yielding emergent patrol / intercept / track, so the evolutionary approach is
  working). **Avoid direct physical-unit conversions**; closure is already carried **unitless**
  (span-rate), and the 20 Hz loop gives finer closure data. Any new collision feature, if ever needed,
  stays optically-derived and unitless — supersedes the FR-008 "evaluate a physical range input"
  option below.
- Q: Penalty strength + mechanism? → A: **Start with a big penalty: halve the member's total fitness
  per hull strike** (mechanism B). Leaves *some* signal (not a hard wall) — "we want finesse, not
  crash into the rabbit." Death-penalty (A) is the escalation if halving proves too weak.

## Overview

M2 (the tracker / chase controller) trains against a recorded M1 source trajectory through the
analytic beacon-projection front-end, with **craft** variations (034) but **no camera variation** and
**no crash-cost incentive**. Two gaps block a trustworthy, flyable M2:

1. **Safety.** Hull-strikes against the target grow *monotonically with tracking skill* — the chase
   learns to fly *into* the target as it sharpens (030: ~1→11/gen; 032: ~3× faster; 035-t7 logged the
   escalation live). A controller that tracks beautifully and collides is not flyable. M2 needs a
   crash-cost pressure that is **selection-decisive even for a rare strike**, without re-creating the
   033 scalar-penalty Pareto collapse.

2. **Camera generalization.** M2 trains against one nominal camera. Real hardware (031) will have
   per-build mount tolerances, focal/FOV spread, and a noise/CEP envelope. Without camera variation in
   training, M2 overfits one virtual camera and won't survive the sim-to-real camera gap — the same
   lesson craft variations addressed for the airframe.

038 first clears a small **Phase-0 cleanup** (PRNG validation, renderer config-hygiene, report
tooling), then adds (US1) a **hull-crash penalty** as a lexicase/dominance fitness dimension and (US2)
**camera variations** as a parameterized per-scenario PRNG class (opening with a spike that decides
whether camera variation forces an M1 source re-bake). Out of scope: M2 environment parity, the
eval-playback determinism harness, and all 031 hardware/firmware and 037 xiao (M1-flight) work.

## Sensor inventory & sim→real grounding (operator review 2026-06-16)

All 54 M2 NN inputs reviewed against the principle *achieve true sim→real, don't calibrate to it*.
Verdict: the suite is sim→real-grounded — ~83 % is unitless optical, and every physical input is
genuinely available on the aircraft. CEP is the one 031-fed model; no input is a sim-only liability.

| Inputs (count) | Derivation | Units | Real-flight source |
|---|---|---|---|
| beacon L/R x,y (24) | world→NDC beacon centroid | NDC, unitless [−1,1] | camera (031) — **position quality** |
| beacon L/R CEP (12) | **classic CEP** — DSP position-uncertainty estimate; magnitude driven by decode/reacquisition confidence + intermittency + crossing-rate over the multi-frame code-acquisition window (per-frame blur low @480 fps global shutter; the ~150 ms code period is the smear window, not the fps) | unitless [0,1] + sentinel | 031 decode pipeline (sim = off-axis placeholder → evolve to decode-time/intermittency/crossing-rate) |
| beacon_pair_span (6) | NDC `sqrt(dx²+dy²)` | NDC, unitless | derived from beacon positions |
| span_rate (1) | NDC/s closure (037 T022) | ~unitless (NDC/s) | derived; finer at 20 Hz |
| tilt sin/cos (2) | `atan2` over NDC | unitless [−1,1] | derived |
| quat w/x/y/z (4) | chase attitude | unitless [−1,1] | AHRS (GPS-aided) |
| airspeed (1) | relVel / 13 m/s cruise | dimensionless ≈[0,2] | **AHRS/GPS velocity** — available |
| gyro p/q/r (3) | body angular rates | rad/s (physical) | **onboard IMU** — real, native units |
| dist_to_boundary (1) | `tanh(d/20m)` to arena cylinder, along velocity | dimensionless [0,1) | **deliberate crutch** (see below) |

**Resolutions (operator 2026-06-16):**
- **CEP = classic CEP (position uncertainty), magnitude driven by signal quality + crossing-rate.**
  Real-flight CEP is a circular-error-probable estimate the camera DSP produces; its size tracks
  Gold-code decode/reacquisition confidence (partial decode ⇒ high CEP) + intermittency (occlusion) +
  **apparent crossing-rate over the multi-frame code-acquisition window** (the code spans ~2.4
  frames/bit × ~15–16 bits ≈ ~150 ms, so a fast cross smears the decode). Per-frame exposure blur is
  low (global shutter @~480 fps), but the high fps does **not** remove the acquisition-window smear —
  the code period sets it. Stays an NN input + the visibility gate. The sim's off-axis placeholder
  should evolve toward a **decode-time + intermittency + crossing-rate** model (031-fed).
- **dist_to_boundary STAYS** — a deliberate crutch: prevents flyaway and induces patrol mode. Real
  flight has AHRS+GPS position, but only the relative distance-to-boundary-**along-velocity** matters
  during these phases; the cylinder's world placement is irrelevant (velocity from AHRS).
- **gyro stays rad/s** — the aircraft has an IMU; native units transfer.
- **airspeed stays** — AHRS/GPS velocity is available; the 13 m/s cruise normalization is a scaling
  anchor, not a calibration dependency.

**Implication for US2:** camera variations vary **position quality** (centroid x/y, intrinsics, mount
extrinsics) and **signal quality** (CEP = intermittency / reacquisition / occlusion) as **distinct**
phenomena — CEP variation is dropout/confidence, *not* position jitter.

## Phase 0 — Front-matter cleanup & tech debt (prerequisite, ahead of US1)

The operator wants these landed at the front of 038, before the fitness/variation work. Each maps to
an existing backlog thread; none is novel feature scope, but they de-risk the bakes that follow.

- **P0-A — 033 PRNG validation.** Confirm the 033 PRNG-cascade rework (`acf732f`) faithfully and
  deterministically regenerates a scenario from its seeds (the prerequisite for any honest variation
  work). Either clear it or find the suspected single-SHA bug. Reference: BACKLOG "M1 basin-landscape"
  → "033 PRNG single-SHA bug hunt"; `project_v15_determinism_candidates`.
- **P0-B — Renderer reads less from `.ini`.** Reduce the renderer's (and replay tooling's) dependence
  on the *current* `.ini` for fitness/cadence params — they should come from the run/dmp, so a drifted
  ini can't misrender an old run. A step toward (not the whole of) the "self-describing dmp" backlog
  item; scope to what the renderer needs now.
- **P0-C — Standardized report script.** One `scripts/` wrapper whose param is a logfile name →
  emits the full PNG report set (derive run-id/bucket/gen/mode from the log head). Lands the
  "[BACKLOG 038] Standardize training reporting" item; folds the dmp-fed plotters into a maintained
  analytics home with `pyproject`/`requirements`.

**Phase-0 acceptance**: P0-A produces a written verdict (clear / bug-found-and-fixed) with a
determinism check; P0-B lets the renderer replay a pinned run without reading fitness params from the
live `.ini`; P0-C reproduces today's 6-PNG M2 set from a single `scripts/<wrapper> <logfile>` call.

## Stakeholders

- **Operator (single-developer project)** — runs the bakes, judges fitness signal, owns σ defaults and
  the flight-safety bar. Building 031 test articles + driving the 037 xiao M1-flight track in parallel.
- **M1 source library (upstream)** — t10 today; possibly a re-baked craft+camera-varied library if
  US2's spike says camera variation must live source-side.
- **031 hardware track (sibling)** — provides the eventual camera/beacon noise calibration that
  replaces 038's placeholder σ. 038 keeps the camera-variation params in a shape 031 can drive.
- **Real-flight M2 (downstream)** — the consumer the hull-crash penalty unblocks.

## User Scenarios & Testing

### US1 — Hull-crash penalty as a lexicase fitness dimension (Priority: P1)

As the operator, I want a chase controller whose fitness *punishes hull-strikes against the target* so
the evolved policy tracks closely **without colliding**, making M2 safe to fly.

**Why this priority**: the flight-gating safety dimension and the operator-sequenced "after 037"
headline. Ahead of camera variations — a colliding controller is unflyable regardless of camera
robustness. The danger is *late*: a competent controller that intercepts and tracks, then strikes the
hull near the end of a long course, banks a near-full score under today's score-stop.

**Independent Test**: a bake (M1 or M2) with the hull penalty on vs the same seed without it; the
penalized run reaches comparable tracking depth (pctInStreak / distance) at a **materially lower
hull-strike rate at matched tracking depth**, and a member that strikes the hull in even ~1/300
scenarios ranks **decisively** below an otherwise-equal clean member ("99/100 clean is still bad").

**Acceptance Scenarios**:

1. **Given** two members with near-identical tracking scores where one strikes the hull in one
   scenario and the other never strikes, **When** selection runs, **Then** the clean member wins
   decisively (rare strike is member-deciding) — via the ×0.5-per-strike multiplier (B), escalating to
   death-penalty (A) if too weak.
2. **Given** a member that tracks well for a long course and **then** strikes the hull near the end,
   **When** fitness is computed, **Then** the late strike does **not** let it bank a near-full score —
   the member-level penalty claws it back regardless of accumulated tracking points.
3. **Given** an **OOB** (out-of-bounds) exit rather than a hull strike, **When** the scenario ends,
   **Then** the existing **score-stop** behavior is unchanged (M1-compatible; only hull strikes get
   the stronger penalty).
4. **Given** a long bake with the penalty, **When** the hull-strike-vs-generation curve is plotted,
   **Then** strikes do **not** climb monotonically with tracking skill the way 030/032/035-t7 did.

---

### US2 — Camera variations (parameterized per-scenario PRNG class) (Priority: P2)

As the operator, I want each training scenario to draw a slightly different camera (intrinsics /
extrinsics / CEP-noise within σ) so the evolved M2 generalizes across the real camera build-tolerance
envelope instead of overfitting one virtual camera.

**Why this priority**: generalization for the sim-to-real camera gap; the airframe analog (craft
variations, 034) is proven to help. Below US1 (safety first).

**Step 1 — scoping spike (settles "re-bake M1?")**: before building the machinery, decide whether
camera variation must be baked into the **M1 source trajectories** (forcing a new M1 library bake) or
can live entirely **M2-side** (varying only the chase's optical path against an unchanged source). The
M1 source is an M1 craft tracking a pathgen rabbit — blind to the M2 chase camera — so the expectation
is **M2-side**, but the spike confirms it and, if it says "re-bake", adds that re-bake as an explicit
scoped line with its own compute budget. Deliverable: a written verdict, not code.

**Independent Test**: a bake with `EnableCameraVariations=1` vs off at the same seed; per-scenario the
camera params differ deterministically from `cameraSeed`, replay reproduces them bitwise, and the
varied-camera controller tracks a held-out *novel* camera draw at least as well as the fixed-camera
controller tracks the nominal camera (the generalization lift, mirroring craft variations).

**Acceptance Scenarios**:

1. **Given** the Step-1 spike verdict, **When** US2 is planned, **Then** the source-side-vs-M2-side
   question and the M1-re-bake decision are settled and recorded.
2. **Given** `EnableCameraVariations=1`, **When** two scenarios draw different `cameraSeed`s, **Then**
   their camera intrinsics/extrinsics/CEP-noise differ per the configured σ and re-running the same
   seed reproduces them exactly (determinism per `project_variation_design_principles`).
3. **Given** `EnableCameraVariations=0`, **When** a scenario runs, **Then** the nominal camera is used
   and results are bit-identical to the pre-038 path (clean macro-disable).
4. **Given** the variation ramp, **When** a late-run scenario is evaluated, **Then** camera variation
   is applied at the ramped magnitude via the shared `applyVariationScale` (same machinery as craft).
5. **Given** 031 later publishes a measured camera noise/CEP envelope, **When** the operator updates
   the σ defaults, **Then** only config/default values change — no structural code change.

---

### US3 — Lexicase epsilon that scales with per-scenario magnitude (Priority: P3, conditional)

As the operator, I want lexicase epsilon to scale with each test case's magnitude (MAD-relative)
rather than a constant 0.5, because once craft+camera push to a 5–6-dim regime the per-scenario spread
ranges from ~0.3 % (deep long-track) to ~100 % (early crash) and a fixed absolute epsilon mis-weights
both ends.

**Why this priority**: a known follow-on (`project_lexicase_mad_epsilon`) that becomes load-bearing
*specifically when camera variations land* (US2). **Conditional**: t11-m2 looks fine so far; include
only if US2's added dimensionality degrades selection signal. Keep an ini switch for historical
reproducibility.

**Independent Test**: with MAD-relative epsilon, selection on a mixed test set (tiny-spread long-track
+ huge-spread early-crash scenarios) preserves discrimination at both ends vs the constant-0.5 baseline
collapsing one of them.

**Acceptance Scenarios**:

1. **Given** a generation with both ~0.3 %- and ~100 %-spread scenarios, **When** MAD-relative epsilon
   is used, **Then** each scenario's epsilon tracks its own median-absolute-deviation.
2. **Given** the ini switch set to constant-0.5, **When** an old run is replayed, **Then** historical
   behavior is reproduced.

### US4 — M2 history buffer depth & spacing (decoupled from M1) (Priority: P3, research)

As the operator, I want to investigate whether the M2 tracker NN benefits from a **different temporal
history buffer than M1** — deeper (e.g. back to the 1.6 s window), more steps, and/or a different lag
spacing (log / Fibonacci / exponential / geometric) — because M2's patrol → intercept → track behavior
over complex/maneuvering paths may need longer or denser situational-awareness context than M1's
known-path tracking does.

**Why this priority**: research, not load-bearing for safety (US1) or generalization (US2). But cheap
to motivate now: M1 and M2 currently **share** `kNNHistoryLagsMsec` ([−0.8, −0.4, −0.2, −0.1, −0.05,
now], 6 slots, t10 0.8 s) — and there's no reason they must. M2 sees the harder temporal problem (a
maneuvering target, not a fixed path); the RNN's situational awareness is exactly what a deeper/denser
buffer would feed.

**Independent Test**: a bake with an M2-specific history layout (deeper window / more steps / alt
spacing) vs the M1-matched layout at the same seed; the alt layout shows equal-or-better tracking
(pctInStreak / distance) and/or cleaner regime transitions, determinism + replay preserved.

**Acceptance Scenarios**:

1. **Given** an M2-specific history layout distinct from M1's, **When** a tracker bake runs, **Then**
   it is honored deterministically and replays bitwise.
2. **Given** a layout change, **When** it lands, **Then** M2 genomes retrain from scratch (format-
   breaking NN-input change; no cereal version bump; xiao firmware contract updated).

---

### Edge Cases

- **Hull penalty vs existing crash handling**: how does the member-level hull penalty compose with the
  current `crashReason` handling (`fitness_decomposition.cc`)? OOB stays score-stop; the hull path must
  apply the member-level penalty (A or B) without double-counting, and must cleanly distinguish a hull
  strike from an OOB exit.
- **Rare-crash decisiveness**: a single strike in ~1/300 scenarios must still be member-deciding — the
  member-level death-penalty/multiplier achieves this directly, without relying on a lone lexicase test
  case happening to become the decider.
- **Sensor blind spots**: a from-behind overshoot strike (chase passes the rabbit, target overruns it)
  is unperceivable with a forward camera; the penalty can only select against the *geometry*, not teach
  avoidance, unless an explicit target-range input is added (FR-008).
- **Camera variation and beacon visibility**: an extrinsic mount offset or wider-FOV draw can change
  which beacons are CEP-gated visible; variation must not produce degenerate all-blind scenarios.
- **Determinism with new PRNG draws**: adding `cameraPRNG` (and any crash-cost state) must keep the
  joint per-scenario PRNG sample reproducible and eval-mode replay bitwise.
- **Cross-run fitness comparability**: the crash penalty changes the fitness scale; ablation/baseline
  comparisons must use a fixed-eval comparator, not raw late-run training fitness.

## Requirements

### Phase 0 — Cleanup (prerequisite)

- **FR-P0A**: 033 PRNG cascade MUST be validated to deterministically regenerate a scenario from its
  seeds; the outcome (clear / fixed) is recorded with a determinism check.
- **FR-P0B**: The renderer/replay path MUST source fitness/cadence params from the run/dmp rather than
  the current `.ini` for replaying a pinned run (scope: what the renderer needs now).
- **FR-P0C**: A single `scripts/` wrapper taking a logfile name MUST regenerate the full PNG report set
  (derives run-id/bucket/gen/mode from the log); the dmp-fed plotters move to a maintained analytics
  home with `pyproject`/`requirements`.

### Hull-crash penalty (US1)

- **FR-001**: OOB (out-of-bounds) handling MUST be **unchanged** — keep today's score-stop. Only
  **hull strikes** get the new penalty.
- **FR-002**: The hull penalty MUST be **member-level** (operates on the member's whole fitness, not a
  single scenario's tracking score), so a late strike after a long good track is clawed back
  regardless of accumulated points. No tick-weighting term — the score-stop already encodes early-vs-
  late economics.
- **FR-003**: The mechanism is a **member-level aggregate multiplier**, NOT scalar-objective
  compositing and NOT a per-scenario lexicase axis (sidesteps the 033 Pareto collapse). **Start with
  (B): member total fitness × 0.5 per hull strike** — a big penalty that still leaves *some* signal
  (preserves gradient/genes; the goal is finesse, not a hard wall). **(A) death-penalty** (any strike →
  member dominated by any cleaner member) is the escalation if (B) proves too weak. [NEEDS RESEARCH:
  how the multiplier composes with the current M2 lexicase selection — applied to the aggregate
  before/within selection — settled at the plan phase.]
- **FR-004**: A **rare** hull-strike (≈1/300 scenarios) MUST be **decisively** worse than zero strikes
  ("99/100 clean is still bad").
- **FR-005**: Per-scenario hull-strike facts (occurred? at which tick? accumulated score at strike)
  MUST be recorded in the dmp (honest-recording) for `dmp-dump`/analytics without log-parsing.
- **FR-006**: The penalty MUST apply to **both M1 and M2**, behind a clean enable knob.
- **FR-007**: The hull-strike-rate-vs-generation curve MUST be analyzable from the dmp/log to confirm
  strikes do not climb monotonically with tracking skill.
- **FR-008**: M2 is **optical-only by design** — the NN MUST infer hull danger from the two beacon
  points (span + span-rate, **unitless**) plus RNN situational awareness, the same substrate already
  producing emergent patrol / intercept / track. **Do NOT add physical-unit inputs** (no metric
  target-range / hull-proximity sensor), and **avoid direct physical-unit conversions** generally —
  closure stays unitless (span-rate), finer at 20 Hz. Any new collision-relevant feature, if ever
  needed, MUST stay optically-derived and unitless. **Accepted consequence**: a from-behind overshoot
  strike is unperceivable from a forward camera — the penalty selects against that geometry rather than
  teaching avoidance.

### Camera variations (US2)

- **FR-010**: Add `cameraSeed` to `ScenarioMetadata` at the reserved insertion point (after
  `craftSeed`, `scenario_metadata.h:98`) and a `cameraPRNG` cascade slot, following the craft pattern.
- **FR-011**: Per scenario, the camera MUST be drawn deterministically from `cameraSeed` across two
  *distinct* quality axes: **position quality** (centroid x/y, intrinsics focal/FOV, extrinsics mount
  pose) and **signal quality** (CEP = the classic position-uncertainty estimate whose magnitude tracks
  decode/reacquisition confidence + intermittency + crossing-rate over the multi-frame code-acquisition
  window; *not* static position jitter; per-frame blur low @480 fps global shutter but the ~150 ms code
  period is the smear window). [NEEDS CLARIFICATION: exact list + σ defaults — seed from 031
  `camera_considerations.md`; the sim CEP model evolves from the off-axis placeholder toward a
  decode-time/intermittency/crossing-rate model as 031 lands.]
- **FR-012**: Camera variation MUST be ramped via `applyVariationScale` and gated by an
  `EnableCameraVariations` macro-disable; off ⇒ bit-identical to pre-038.
- **FR-013**: Camera variation MUST be eval-replayable bitwise (per-scenario override path, as craft).
- **FR-014**: σ defaults MUST live in config so 031's later calibration updates values without
  structural code change.
- **FR-015**: Draws MUST be clamped/validated so no scenario degenerates to all-beacons-blind purely
  from a camera draw.
- **FR-016**: US2 Step-1 spike MUST be resolved before US2 implementation; a "re-bake M1" verdict adds
  an explicit scoped line with its own compute budget.

### Selection scaling (US3, conditional)

- **FR-020**: IF camera variation degrades lexicase discrimination at 5–6 dims, offer **MAD-relative
  epsilon** behind an ini switch defaulting to the historical constant-0.5.

### M2 history buffer (US4, research)

- **FR-050**: The M2 tracker history layout (window depth, step count, lag spacing) MAY differ from
  M1's — a separate `kNNHistoryLagsMsec`-class constant for the tracker. [NEEDS RESEARCH: depth
  (→1.6 s?), count (>6?), spacing scheme (log / Fibonacci / exponential / geometric) — judged by M2
  tracking + regime-transition quality.]
- **FR-051**: A history-layout change is a **format-breaking NN-input change** — TrackerInputs layout
  + xiao firmware contract update, no cereal version bump, M2 genomes retrain from scratch,
  determinism + bitwise replay preserved.

### Cross-cutting

- **FR-030**: All new state (crash-cost, `cameraPRNG`) MUST preserve absolute per-scenario determinism
  and eval-vs-training bitwise parity (the regression gate).
- **FR-031**: Schema changes follow no-cereal-versioning / fail-loud practice (greenfield, no version
  bump, readers fail loud).

### Key Entities

- **Hull-strike record** (new) — per-scenario flag + tick + accumulated-score-at-strike, recorded in
  the dmp; feeds the **member-level** penalty (death-penalty or ×0.5/strike), not a per-scenario
  lexicase axis. OOB is tracked separately and keeps the score-stop.
- **cameraSeed / cameraPRNG** (new) — per-scenario camera-variation PRNG root + cascade slot, parallel
  to `craftSeed`/craftPRNG.
- **Camera-variation parameter block** — intrinsics/extrinsics/CEP-noise σ set, config-resident,
  031-calibratable.
- **Camera (existing)** — `camera_projection.cc` analytic beacon projection; the thing being varied.

## Success Criteria

### Measurable Outcomes

- **SC-000**: Phase 0 done — 033 PRNG verdict recorded with a determinism check; renderer replays a
  pinned run without the live `.ini`'s fitness params; the full M2 PNG set regenerates from one
  `scripts/<wrapper> <logfile>` call.
- **SC-001**: At matched tracking depth, an M2 bake with the hull-crash penalty shows a materially
  lower hull-strike rate than the no-penalty baseline at the same seed/generation, and the strike-vs-
  gen curve does **not** climb monotonically with tracking skill.
- **SC-002**: In a constructed selection test, a member striking ≥1 of ~300 scenarios ranks
  **decisively** below an otherwise-equal clean member.
- **SC-003**: A late hull strike after a long good track does NOT bank a near-full score — the
  member-level penalty claws it back (confirmable from the dmp's per-scenario strike record); OOB exits
  remain score-stop, unchanged.
- **SC-004**: `EnableCameraVariations=0` ⇒ bit-identical to pre-038; on ⇒ per-scenario camera params
  deterministic from `cameraSeed`, bitwise on replay.
- **SC-005**: A varied-camera M2 controller tracks a held-out *novel* camera draw at least as well as a
  fixed-camera controller tracks the nominal camera (generalization lift).
- **SC-006**: The eval-vs-training bitwise regression gate passes with all 038 schema/PRNG changes.

## Out of Scope (explicit)

- **M2 environment parity** (chase re-simulated in the source's own air mass) — **dropped** (operator:
  not needed at this accuracy level). Remains the BACKLOG "M2 sim playback parity" item.
- **Eval-playback / determinism-witness harness** (`EvaluateMode=playback`) — deferred; not required
  for the generalization goal.
- **Deep self-describing-dmp schema work** — P0-B does only the renderer-side config-hygiene slice; the
  full `EvalResults` config-block serialization stays its own backlog item.
- **031 hardware/firmware/optics** and the camera noise *calibration* — 038 consumes calibration when
  it exists but does not produce it.
- **037 xiao M1-flight track** (local IMU, full loop unroll, 20 Hz on-target, defined latency) — a
  **parallel 037-closeout** effort, ahead of 038, NOT 038 scope.
- The crash-cost *final metric form* (FR-004) — a plan-phase research output.

## Dependencies & Sequencing

1. **Phase 0** (P0-A PRNG validation, P0-B renderer config-hygiene, P0-C report script) — front-matter,
   ahead of the feature work.
2. **US1 hull-crash penalty** — P1, independent of US2; the safety gate for any real-flight M2.
3. **US2 camera variations** — opens with the source-vs-M2-side spike; σ placeholder-then-031-
   calibrated.
4. **US3 MAD-epsilon** — conditional on US2's dimensionality hurting selection.
4b. **US4 M2 history buffer** — research, independent of US1–US3; can run as an ablation anytime
   against the M2 baseline (decoupled `kNNHistoryLagsMsec`, retrain from scratch).
5. **Parallel, not 038**: 037 xiao M1-flight track; 031 camera design.
6. Upstream baseline: 037 t10 source + t11-m2 retrain. Downstream: real-flight M2 (gated by US1).
