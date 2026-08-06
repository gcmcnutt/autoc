# Feature Specification: 040 Camera Redo — Perception Fidelity Refinement for M2

**Feature Branch**: `040-camera-redo`
**Created**: 2026-07-28
**Status**: Draft
**Input**: Operator scoping conversation 2026-07-28. All measured hardware values, findings, and open
items are recorded in [`input-data-checklist.md`](input-data-checklist.md) — **that file is the input of
record for this spec; consume it rather than re-deriving.**

## Overview

040 makes the simulated beacon camera resemble hardware we can actually build, and retrains M2 against
it. It is a **research and design-refinement** feature, not a build feature: no FPGA pipeline, no final
part selection. The output is a *refined camera scheme grounded in deliverable hardware*, a simulator
that reflects it, and an M2 controller that is more robust because it trained against honest perception.

031 proves field hardware in parallel. **M3 (optical + time-of-flight) is the real destination**, so this
feature deliberately favours what generalises to M3 — camera geometry, occlusion, photon budgeting,
acquisition latency — over what is specific to coded-beacon CDMA. Time-of-flight will eventually supply
range directly, so the span→range inference channel is transitional and is corrected here, not extended.

**Why now**: 039 showed decent M2 performance against the *simulated* camera. That camera is currently
unphysical in ways that matter — beacon separation is 17% wrong, range does not affect visibility at all,
acquisition is instantaneous, the airframe and propeller are transparent, and the nominal field of view
cannot be built on the assumed sensor. Any M2 competence measured against it carries unknown transfer
risk to hardware.

**Plumbing first, calibration later** (operator direction, 2026-07-28). The deliverable is a
higher-fidelity simulator with the *right structure and the right knobs* — not final numbers. Field
hardware is still being proven, and the measurements that would pin the signal model do not yet exist.
So every physical quantity in the perception chain is a configured value with a stated default and a
recorded basis, and the feature succeeds if a later real-to-simulation calibration can be absorbed by
changing configuration rather than by redesign. Findings that cannot be settled now — the wide-field
link-budget shortfall, the sensor-format problem, the emitter beam-width discrepancy — are **recorded as
calibration targets, not treated as blockers**.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Airframe fidelity verdict, before anything else (Priority: P1)

As the operator, I need to know whether the simulator's flight model still matches the real article
closely enough to keep training against it — **before** any perception work begins — because if the plant
must change, the M1 source trajectories must be regenerated first, and every downstream activity in this
feature consumes them.

**Why this priority**: This is a gate, not a task. Discovering a required plant change *after* the
perception work would invalidate it. Running it first either clears the path (expected) or reorders the
entire feature. It also has standing value on its own: a documented, quantified comparison of simulated
versus real airframe parameters.

**Independent Test**: Produce a written comparison of every simulator airframe/propulsion parameter
against the measured article data, each marked agree / disagree / unknown with magnitude, ending in an
explicit recorded decision to regenerate or defer. Delivers value even if nothing else in the feature
ships.

**Acceptance Scenarios**:

1. **Given** the measured article data (mass, span, wing area, propeller, station stack) and the
   simulator's flight model, **When** the comparison is run, **Then** every parameter is classified with
   its magnitude of disagreement, and known discrepancies are individually accounted for.
2. **Given** a discrepancy that would change the simulated plant, **When** the verdict is recorded,
   **Then** it states either "regenerate M1 source now, before perception work" or "defer, with the
   discrepancy filed for the flight-test feature", with reasoning.
3. **Given** the verdict is "defer", **When** the remaining stories proceed, **Then** no M1 source
   regeneration occurs anywhere in this feature.

---

### User Story 2 - Honest camera geometry (Priority: P1)

As the operator, I need the simulated camera's output to carry the geometry of a real sensor — a genuine
pixel grid, direction measured the same way on both axes, and the correct physical separation between
beacons — so that what the controller learns from is dimensionally truthful.

**Why this priority**: The largest single correctness defect in the current perception chain lives here.
Beacon separation is the entire basis for inferring range, and it is ~17% wrong. Separately, the existing
coarse encoding of bearing is *less* precise than the sensor it claims to model on one axis and more
precise on the other, and the two axes are scaled differently, which distorts every derived quantity that
mixes them. Everything else in this feature builds on top of this representation.

**Independent Test**: Project known geometries through the camera and confirm that a fixed angular
separation reads the same regardless of where it sits in frame and regardless of its orientation; that
reported bearings resolve no finer and no coarser than the sensor's pixel grid permits; and that inferred
range matches truth for a target at known distance.

**Acceptance Scenarios**:

1. **Given** two points separated by a fixed angle, **When** the pair is placed at frame centre and again
   near the frame edge, **Then** the reported separation agrees between the two placements to within the
   documented residual.
2. **Given** the same pair rotated from horizontal to vertical, **When** separation is reported, **Then**
   it is unchanged — the two axes carry the same scale.
3. **Given** a target at a known distance, **When** range is inferred from the reported separation and the
   physical beacon spacing, **Then** it matches truth within the resolution the pixel grid allows.
4. **Given** any beacon position, **When** its bearing is reported, **Then** the value is quantised to the
   sensor's pixel grid — neither finer nor coarser.
5. **Given** the perception change, **When** the controller's input vector is inspected, **Then** its size
   is unchanged and no regeneration of M1 source data is required.

---

### User Story 3 - Prove the mount choice clears the aircraft's own obstructions (Priority: P2)

As the operator, I need to confirm that the chosen camera position genuinely keeps the aircraft's own wing,
nose and propeller out of the useful field — **across the whole mounting-error envelope, not merely at
nominal** — and to retain the ability to price a different mount if this one proves impractical to build.

**Why this priority**: The mount moved. Scoping analysis showed a leading-edge position ~8 inches outboard
eliminates wing obstruction entirely (nothing sits ahead of the leading edge) and pushes the propeller
shadow from directly below the boresight — where a chased target lives — out past 41° inboard, where it
is reached only in a hard inboard turn. So obstruction stops being a defect to model faithfully and
becomes a **design choice to validate**. It still needs validating, because a camera glued 20° off — the
worst case in the variation envelope — brings that shadow back to roughly 21° from its own boresight.
Design intent and build reality have to be checked together.

**Independent Test**: Sweep a target through the full field and produce a visibility map at the nominal
mount, then repeat across the mounting-error envelope; report the resulting distribution of obstruction
onset angle — median, 95th percentile and clipped extreme — and observe where it lands.

**Acceptance Scenarios**:

1. **Given** the baseline mount at nominal alignment, **When** a target is swept through the field,
   **Then** the obstruction onset angle is reported and the wing contributes no obstruction at all.
2. **Given** the mounting-error envelope, **When** the sweep is repeated across it, **Then** the
   **distribution** of obstruction onset is reported — median, 95th percentile, and clipped extreme — so
   the typical case is visible alongside the tail rather than being hidden behind it.
3. **Given** an alternative mount position, **When** it is configured, **Then** the model reports its
   obstruction consequences without code changes, so mount options can be compared before committing.
4. **Given** the propeller intersects the field at any mount, **When** blockage is modelled, **Then** it is
   a periodic partial reduction in received signal consistent with a blade partially covering the
   aperture, not a hard cutoff.
5. **Given** the obstruction model, **When** the usable field is reported, **Then** it is published as the
   *effective* field, not the nominal rectangle.
6. **Given** a brief obstruction, **When** it clears, **Then** signal recovery follows the measured
   ride-through behaviour rather than an instantaneous restoration.

---

### User Story 4 - Signal quality that means something (Priority: P2)

As the operator, I need the per-beacon confidence value fed to the controller to represent actual signal
quality — degrading with distance, aspect and interference, and taking real time to establish after a loss
— instead of the current placeholder that varies only with position in frame and gates nothing.

**Why this priority**: Today this value is decorative: it is computed from screen position alone, and
visibility is binary. Range does not affect it, so a beacon at 5 m and at 500 m are perceptually
identical. Making it honest introduces the graded degradation and reacquisition delay that the documented
M2 bottleneck — recovering the target after losing it — actually consists of.

**Independent Test**: Hold a beacon at a series of ranges and aspects and confirm confidence degrades
monotonically and matches the measured bench relationship; interrupt the signal and confirm the recorded
time to re-establish tracking matches measurements.

**Acceptance Scenarios**:

1. **Given** a beacon receding from the camera, **When** confidence is reported, **Then** it degrades
   consistent with the measured relationship between distance and signal strength, and eventually the
   beacon is reported as not visible.
2. **Given** both beacons converging toward the same detector region at long range, **When** confidence is
   reported, **Then** it degrades by the measured penalty for two codes sharing one detector.
3. **Given** a beacon appearing after a period of absence, **When** tracking re-establishes, **Then** the
   elapsed time matches the measured acquisition behaviour, including the partial-code early-lock case.
4. **Given** a brief signal interruption shorter than the measured hold interval, **When** it ends,
   **Then** tracking was never lost.
5. **Given** identical inputs, **When** perception is evaluated repeatedly, **Then** results are
   bit-identical — the model introduces no randomness.

---

### User Story 5 - A more robust M2 (Priority: P2)

As the operator, I want a retrained M2 controller that performs credibly against the corrected perception
model, so that measured competence reflects what hardware can deliver rather than an idealised camera.

**Why this priority**: This is the feature's outcome — the reason the preceding stories exist. It cannot
precede them, but without it the feature delivers a better simulator and no evidence it matters.

**Independent Test**: Train against the corrected perception model and evaluate on the novel-path harness,
comparing competence against the previous baseline on the established comparators.

**Acceptance Scenarios**:

1. **Given** the corrected perception model, **When** M2 is trained, **Then** it reaches a competence
   plateau and the run completes without systemic failure.
2. **Given** the trained controller, **When** it is evaluated on paths it never trained against, **Then**
   competence is reported on the established comparators alongside the prior baseline.
3. **Given** the comparison, **When** results are recorded, **Then** the report states plainly whether
   perception honesty cost, preserved, or improved measured competence — as an aggregate delta, with a cost
   being an acceptable and expected outcome rather than a failure. Per-term attribution is not required.

---

### User Story 6 - Per-scenario camera variation (Priority: P3)

As the operator, I want each training scenario to draw its own small camera imperfections — mounting
misalignment, airframe variation, ambient light level — so the controller cannot depend on a perfectly
known and perfectly repeatable camera.

**Why this priority**: Robustness insurance rather than correctness. Real precedent exists: a mount
misalignment on a related component put a large systematic bias into recorded flight data. The
infrastructure is already reserved and unused, so the cost is low — but the feature stands without it.

**Independent Test**: Confirm scenarios draw distinct camera parameters, that the same scenario identifier
always reproduces the same draw, and that disabling variation exactly reproduces prior behaviour.

**Acceptance Scenarios**:

1. **Given** variation enabled, **When** two scenarios are compared, **Then** their camera parameters
   differ within the configured bounds.
2. **Given** a scenario identifier, **When** it is evaluated on separate occasions, **Then** the camera
   parameters drawn are identical.
3. **Given** variation configured to zero, **When** results are compared against the no-variation
   baseline, **Then** they are bit-identical.
4. **Given** the chase and target draw from shared environment settings, **When** camera parameters are
   drawn, **Then** they remain specific to the chase, since perception belongs to the chase alone.

---

### User Story 7 - Optics findings preserved for the hardware decision (Priority: P3)

As the operator, I want the optics analysis produced during scoping captured as a durable artefact — with
its assumptions and the measurements that would confirm or overturn it — so the eventual lens and sensor
decision inherits the reasoning instead of rediscovering it.

**Why this priority**: The field of view is now decided for this feature (120°, per the clarification
below), so no comparison study is needed. But the analysis behind that decision found two things that
outlive it: a wide field sits well below the link budget that justified emitter drive level, and 120°
cannot be built on the originally assumed sensor. Losing those findings would be expensive. This is
record-keeping and feed-forward, not new investigation — hence lowest priority.

**Independent Test**: A reviewer who was not part of the scoping conversation can read the artefact and
correctly state why 120° was retained, what it costs, and which measurements would change the answer.

**Acceptance Scenarios**:

1. **Given** the optics analysis, **When** it is recorded, **Then** it states the signal shortfall of a
   wide field against the existing link budget, the sensor format a real 120° build requires, and the
   assumptions each rests on.
2. **Given** the record, **When** a future lens decision is taken up, **Then** it identifies the specific
   measurements that would confirm or overturn the analysis.
3. **Given** the record, **When** it is filed, **Then** the deferred optics directions it bears on —
   narrower fields, dual field-of-view, a second camera — are linked to it rather than described afresh.

---

### Edge Cases

- **Target directly ahead, behind the spinner.** Measured geometry leaves only a few degrees of clearance
  at the planned camera height, and the target sits near boresight in a tail chase. The model must handle
  a target that is geometrically visible but propeller-obstructed for a sustained period.
- **Engine speed sweeping through an interference condition.** Engine speed follows the controller's own
  throttle output, so it crosses interference conditions continuously during flight. Behaviour must be
  continuous across the transition, with no discontinuity as blade timing passes through alignment.
- **Both beacons merging into one detector region.** At long range the two beacons become
  indistinguishable. Separation-derived quantities must degrade gracefully to "unavailable" rather than
  reporting noise as signal.
- **Camera obstructed and target lost simultaneously.** Obstruction and genuine loss of signal must not be
  conflated; recovery timing differs and diagnostics must distinguish them.
- **Beacon identity unresolved before decode.** A centroid exists before the code identifies which beacon
  produced it. Separation survives this (it is a magnitude), but **tilt sign does not** — a swapped pair
  flips tilt by 180°, and tilt drives the roll command. The trap is that positional confidence can be high
  while identity is unknown, so quality must carry the identity uncertainty too (FR-017d) — it is the only
  channel available to flag it.
- **Reduced field of view with unchanged obstruction geometry.** Obstructions sit at fixed angles, so
  narrowing the field changes what fraction is lost. The effective field must be recomputed, never assumed.
- **A scenario drawing an extreme camera misalignment.** Variation must remain within physically plausible
  build tolerance and must never place the camera inside the airframe or behind its own propeller hub.
- **Mount configured back behind the propeller.** The obstruction model must remain correct for a mount
  that puts the propeller near the boresight, since that configuration is the trigger for the deferred
  engine-speed work and must be evaluable before it is adopted.

## Requirements *(mandatory)*

### Functional Requirements

#### Camera geometry (US2)

- **FR-001**: Perception MUST quantise beacon bearing onto a discrete sensor grid of defined pixel
  dimensions, so that reported precision matches sensor capability rather than an unrelated encoding.
- **FR-002**: Perception MUST report bearing on a common angular scale for both axes, so a given angular
  separation reads identically regardless of orientation.
- **FR-003**: The field of view MUST be derived from sensor dimensions and angular pixel size rather than
  specified as independent per-axis values, so that field of view and resolution cannot disagree.
- **FR-004**: Physical beacon separation MUST match the measured article (0.772 m, from a 30″ span plus
  the beacon enclosure offset at each tip), replacing the current 0.9 m.
- **FR-005**: Quantities that wrap through a full turn MUST continue to be represented in a form free of
  discontinuity; bounded bearings MUST NOT be, since they cannot wrap.
- **FR-006**: The controller's input vector size MUST remain unchanged, and no regeneration of M1 source
  trajectories may be required.

#### Obstruction (US3)

- **FR-007**: Perception MUST model obstruction by the chase aircraft's wing, nose, and propeller disc,
  positioned from the measured station geometry, for **any** configured mount — so that a mount which
  eliminates an obstruction demonstrates that rather than assuming it.
- **FR-007a**: The baseline mount is the **leading edge, ~8 inches outboard of centreline, ~1 inch above
  the thrust line** (the thrust line passing through the centre of the leading edge), **boresight parallel
  to the thrust line**. The vertical offset is near-irrelevant to clearance — it moves the camera's radial
  distance from the thrust axis only from 8.00 to 8.06 inches — but it tilts the propeller shadow slightly
  below horizontal rather than leaving it purely inboard.
- **FR-007b**: **No fixed obstruction threshold is imposed.** Instead the model MUST report the
  **distribution** of obstruction onset angle across the mounting-error envelope — median, 95th percentile,
  and clipped extreme — and the outcome is observed rather than gated. Reporting only the worst case would
  discard most of the information, since the bulk of the distribution sits within 10° of alignment where
  the shadow stays past roughly 31° off boresight.
- **FR-008**: The wing MUST be represented with its true thickness rather than as a solid enclosing box,
  and the camera MUST NOT be positioned on the boundary of any obstruction volume.
- **FR-009**: The propeller MUST be modelled as a **static angular obstruction region** derived from the
  mount geometry, applying a fixed representative attenuation for a blade partially covering the aperture —
  not a binary cutoff, and **not** an engine-speed-dependent or blade-phase-resolved model. At the baseline
  mount the propeller sits well off the boresight, so phase fidelity buys nothing; this keeps the shadow
  honest at no data cost.
- **FR-010**: *(Withdrawn — engine-speed modelling is out of scope; see Out of Scope. Numbering retained so
  earlier references remain resolvable.)*
- **FR-011**: The interaction between blade-passage timing, beacon code timing and frame rate MUST be
  **recorded as a research question** with the envelope arithmetic that motivates it — not resolved here.
  It is a decoder-design concern rather than a controller-training one, and it is moot at the baseline
  mount.
- **FR-011a**: Camera mount position and orientation MUST be configurable rather than fixed, so candidate
  mounting schemes can be compared on obstruction before hardware is committed. At minimum the model must
  be able to evaluate a wing-top mount and leading-edge mounts at varying outboard offset.
- **FR-012**: The effective field of view — nominal field minus all obstructions — MUST be reported and
  used, rather than the nominal rectangle.
- **FR-013**: Recovery from brief obstruction MUST follow the measured ride-through and re-establishment
  behaviour rather than restoring instantaneously.

#### Signal quality (US4)

- **FR-014**: Per-beacon confidence MUST derive from modelled signal quality — distance, emission aspect,
  obstruction, and ambient level — replacing the current position-only placeholder.
- **FR-015**: Signal strength MUST fall with distance according to the measured relationship, calibrated
  against the bench measurements recorded in 031.
- **FR-016**: When both beacons occupy the same detector region — which occurs at long range — detection
  and code identification MUST remain available, degraded only by the measured interference penalty. This
  configuration is **field-proven**, not a failure mode: the 031 single-detector rig is exactly this case
  and decodes both codes reliably. What merging removes is **spatial separation**, hence separation-derived
  range (FR-033) — not tracking.
- **FR-017**: Establishing tracking MUST take modelled time consistent with measured acquisition
  behaviour, including early lock on a partial code.
- **FR-017a**: A **tentative lock MUST report a bearing**, carrying a signal-quality value indicating large
  positional variance, which improves as the code integrates toward confirmed lock. The controller is not
  starved during the acquisition window — it receives an early, explicitly untrusted fix. Quality therefore
  spans three regimes: small (confident), large (tentative), and the distinct not-visible indication.
- **FR-017b**: The tracking state machine MUST remain **internal to perception and MUST NOT be exposed as a
  controller input**. The perception interface stays bearing plus a quality signal; the controller infers
  tracking state from how quality behaves across its history window rather than being handed a category.
  Tracking state is recorded for diagnostics only (FR-028).
- **FR-017c**: Separation-derived quantities — beacon separation, range, target tilt — MUST be computed
  from **tentative-lock bearings** rather than suppressed. The blob centroid exists before the code
  decodes: detection and identification are separate operations, so a bearing is available pre-lock and the
  centre of the detected blob *is* the coordinate. Quality is presented alongside, and the controller is
  expected to learn the appropriate discount rather than being handed a gap.
- **FR-017d**: **Identity uncertainty MUST be reflected in the quality value**, which is the interface's
  *only* confidence channel — separation and tilt are bare values carrying no confidence of their own, and
  the input vector is fixed at 58 (FR-006), so no channel can be added. Tilt sign depends on beacon
  identity while separation does not: a swapped pair flips tilt by 180°, and tilt drives the roll command.
  Pre-decode, two cleanly-detected blobs could otherwise carry *low* quality values (confident positions)
  while identity is unknown — confidently wrong tilt with nothing flagging it. Unresolved identity MUST
  therefore inflate quality on the affected beacons. This is coherent because quality is redefined as
  **signal quality** (FR-014), not strictly positional uncertainty.
- **FR-018**: Loss of signal MUST pass through a hold interval before tracking is reported lost, matching
  measured behaviour.
- **FR-019**: The emission pattern MUST replace the current hard angular cutoff with a **flat-top profile
  with shoulders** — near-constant to roughly ±45° from each emitter axis, falling through the half-power
  points at ±75° — matching the datasheet beam width and the observed flat region. A cosine power law MUST
  NOT be used, as it misrepresents both the flat region and the skirt.
- **FR-020**: The perception model MUST be fully deterministic — identical inputs produce bit-identical
  outputs, with no randomness anywhere in the signal-quality path.
- **FR-020a**: All per-beacon state carried across ticks — integration progress, tracking state, hold
  timers — MUST be reset at every scenario boundary. Unreset state would leak between scenarios and break
  determinism; the same trap already applies to the existing carried-forward perception state, and both
  execution paths must reset identically.

#### Camera variation (US6)

- **FR-021**: Each scenario MUST draw camera parameters — mount position and orientation error, wing
  thickness, ambient light level — from the reserved per-scenario variation channel.
- **FR-022**: Camera variation MUST be reproducible from the scenario identifier alone, and MUST remain
  specific to the chase aircraft even when other variation is shared with the target.
- **FR-023**: With variation magnitudes set to zero, results MUST be bit-identical to the no-variation
  baseline.

#### Airframe fidelity check (US1)

- **FR-024**: The system MUST produce a documented comparison of simulated versus measured airframe and
  propulsion parameters, each classified with its magnitude of disagreement.
- **FR-025**: The comparison MUST conclude with an explicit recorded decision to regenerate M1 source
  trajectories or to defer, with reasoning, and MUST be completed before any perception work begins.
- **FR-026**: Any deferred discrepancy MUST be filed against the feature that will address it, rather than
  left unrecorded.

#### Optics record (US7)

- **FR-027**: The optics analysis MUST be recorded as a durable artefact stating the wide-field signal
  shortfall against the existing link budget, the sensor format a real 120° build requires, the
  assumptions each rests on, and the measurements that would confirm or overturn them.

#### Calibratability — the plumbing-first contract

- **FR-033**: Perception MUST model detection and range-inference as **separate envelopes**: bearing
  remains available to the detection range, while separation-derived range degrades to unavailable once the
  beacon pair falls below the sensor's resolving limit. Neither may be reported as usable outside its own
  envelope.
- **FR-033a**: The detection envelope is **asserted, not emergent** — the sensor is taken as good to the
  configured detection range (default ~100 m), and the signal budget shapes **quality within it** rather
  than cutting visibility short. Rationale: the budget is not yet calibrated well enough to be trusted as a
  *limit*, but is good enough to shape a *gradient*, and the degradation modes that would set a real limit
  (sun angle, glint, dust, ambient level, sensor variation) are deliberately out of scope. Signal-to-noise
  is therefore **proxied into the quality value**, which is what carries range dependence to the controller.
- **FR-034**: Every physical quantity in the perception chain — emitter drive and beam pattern, ambient
  level, detection threshold, acquisition and hold timing, code-interference penalty, exposure, aperture,
  angular pixel size — MUST be an externally configured value with a stated default and a recorded basis,
  never a value fixed at build time.
- **FR-035**: Each configured physical quantity MUST be classified in documentation as **measured**
  (traceable to a recorded measurement), **derived** (computed from measured values), or **assumed**
  (a placeholder awaiting calibration), so a later calibration pass knows exactly what it may overwrite.
- **FR-036**: The perception model MUST absorb a later real-to-simulation calibration through
  configuration changes alone, without structural redesign — this is the feature's central success
  condition.

#### Instrumentation and diagnostics

- **FR-028**: Recorded perception data MUST capture the internal state needed to diagnose the new model —
  at minimum the signal-quality measure and the tracking state — so behaviour can be reviewed after a run.
- **FR-029**: Recorded data changes MUST be limited to diagnostics; the controller's input vector and the
  M1 source format MUST remain unchanged.
- **FR-030**: The visualisation tools MUST render the corrected geometry and the effective field of view,
  including obstructed regions.

#### Performance

- **FR-037**: The perception model's cost MUST be measured as its effect on **total evaluation
  throughput**, benchmarked directly against the **prior M2 run** rather than an abstract baseline, since
  that is the run whose generation count is being traded against.
- **FR-038**: Total evaluation throughput MUST NOT regress by more than **10%**. A breach MUST be raised as
  an explicit accept-or-optimise decision, not absorbed silently — throughput converts directly into
  generations reached, which is the scarce resource on this project.

#### Prerequisites

- **FR-031**: The per-tick perception update rule MUST be single-sourced before the signal-quality model is
  introduced, since it is currently duplicated across two execution paths and would otherwise diverge.
- **FR-032**: Configuration values describing the camera MUST be externally settable rather than fixed at
  build time, and values that no longer reflect the operating configuration MUST be corrected or removed.

### Key Entities

- **Camera** — the chase aircraft's sensor: mount position and orientation, sensor grid dimensions,
  angular pixel size, aperture, exposure, and the derived nominal and effective fields of view.
- **Beacon** — an emitter at a target wingtip: position, enclosure geometry, emission pattern, drive
  level. Two per target, and their angular separation is the sole basis for inferring range.
- **Beacon observation** — what perception reports per beacon per tick: bearing on the sensor grid, a
  signal-quality measure, tracking state, and a not-visible indication.
- **Obstruction set** — what blocks the camera: wing, nose, and the propeller disc. The wing and nose are
  opaque solids; the propeller disc is a static angular region applying partial attenuation, with no
  engine-speed or blade-phase dependence (FR-009).
- **Signal budget** — the modelled chain from emitter drive through distance, aspect, obstruction, ambient
  level and code interference to a per-beacon signal quality.
- **Tracking state** — per beacon: searching, acquiring, tracking, or holding through a brief loss, with
  the modelled transition timing between them. **Internal to perception** — never a controller input
  (FR-017b); recorded only for diagnostics. Its effect reaches the controller solely through the
  signal-quality value.
- **Camera variation draw** — the per-scenario camera imperfections, reproducible from the scenario
  identifier.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: A fixed angular separation between beacons is reported within 2% of the same value wherever
  it appears in frame and at any orientation, versus the position- and orientation-dependent variation
  present today.
- **SC-002**: Range inferred from beacon separation matches truth within the resolution the sensor grid
  permits, across the operational range band, with no systematic bias — correcting the ~17% error caused
  by the wrong physical separation.
- **SC-003**: The published effective field of view accounts for all obstructions and differs from the
  nominal field by a stated, geometrically justified amount.
- **SC-004**: Signal quality degrades monotonically with distance across the detection envelope, following
  the *shape* of the measured bench relationship. The envelope itself is asserted (FR-033a), so the test is
  the **gradient**, not the cutoff point — quality at the far edge is materially worse than at close range,
  with no discontinuity.
- **SC-005**: Time to re-establish tracking after a loss matches measured acquisition behaviour within one
  control tick, and interruptions shorter than the measured hold interval never lose tracking.
- **SC-006**: Repeated evaluation of an identical scenario produces bit-identical results, and with
  variation disabled results are bit-identical to the pre-change baseline.
- **SC-007**: The obstruction model reports the distribution of propeller-shadow onset angle across the
  mounting-error envelope — median, 95th percentile and clipped extreme — at the baseline mount, with the
  wing contributing no obstruction.
- **SC-008**: A retrained M2 completes without systemic failure and its competence on unseen paths is
  reported against the prior baseline on the established comparators as an **aggregate delta**, trained
  from the **same M1 source the baseline used** so the delta isolates the perception change. If that source
  is unavailable, the substitution MUST be stated and the delta reported as confounded rather than
  presented as a clean comparison. The
  question this answers is deliberately coarse — *are we in the right room, and is this more honest?* — not
  which individual term cost what. No competence floor gates the feature; a drop attributable to more
  honest perception is a valid outcome. Per-term attribution is available later if wanted, since every
  physical quantity is a configured value (FR-034) and a fixed elite can be re-evaluated under alternative
  configurations without retraining — but it is **not required here**.
- **SC-009**: No M1 source trajectories are regenerated, and the controller input vector is unchanged —
  confirming this feature stayed parallel to flight-test work.
- **SC-010**: Every physical quantity the model relies on is classified measured / derived / assumed, with
  a recorded basis, and none is fixed at build time — so a calibration pass knows exactly what it may
  overwrite.
- **SC-011**: Bearing remains reported out to the asserted detection range (FR-033a) while separation-derived range
  degrades to explicitly unavailable below the resolving limit, with a documented crossover — rather than
  range being reported as usable where the sensor cannot resolve it.
- **SC-012**: A calibration rehearsal — substituting a plausible alternative value for each assumed
  quantity — changes results without requiring any structural change, demonstrating that a later
  real-to-simulation pass is absorbable through configuration alone.
- **SC-013**: Total evaluation throughput, benchmarked against the prior M2 run, regresses by no more than
  10%; the measured figure is recorded whatever it turns out to be.

## Assumptions

1. **Measured geometry is authoritative.** The station stack (propeller at station 0, wing leading edge
   6″, camera 8″, wing trailing edge 13″), 30″ span, 7″ chord, 1″ thickness, and the 5.5×4 two-blade
   propeller come from operator measurement on 2026-07-28 and are treated as ground truth.
2. **Camera mount.** 2.5–3″ above the thrust line at the wing's high point, aligned with the thrust line
   with no deliberate tilt. Analysis recommends roughly 1″ of standoff above the wing surface — required
   for any downward visibility and simultaneously lifting propeller clearance from under 2° to about 9°.
   The spec assumes this standoff is adopted; if not, obstruction is substantially worse.
3. **Emitter geometry.** A 1 cm enclosure at each wingtip with one face against the tip and five emitters
   on the exposed faces, matching the arrangement already modelled in the 031 analysis script.
4. **Propulsion data.** Engine speed derives from throttle using the published propulsion calculation.
   Its airspeed axis is not trusted — its wing area entry disagrees with measured geometry by about 28%,
   while the flight model's value matches measurement — so throttle is used as the independent variable.
5. **Airframe verdict.** The comparison is expected to conclude "defer regeneration": a standing decision
   from the previous feature declines recalibration from a single article, and the known propeller
   discrepancy is a thrust-model issue better batched with other flight-model work. The spec does not
   presume this outcome; it requires the check to run first.
6. **Exposure.** Exposure duration is not yet known, since no camera is selected. It is treated as a
   configurable parameter with a stated default, and its influence on propeller interference is reported
   as a sensitivity rather than a fixed result.
7. **Photon budget deferred.** Establishing actual photon counts requires the first camera article and raw
   uncompressed capture. This feature calibrates against 031 bench measurements and states the resulting
   uncertainty.
8. **Bench measurements transfer with stated caveats.** The 031 measurements were taken with five
   co-aimed emitters at bench drive level; the flight enclosure aims emitters in five directions, so only
   one or two face the chase. Calibration accounts for this rather than scaling bench numbers by drive
   level alone.
9. **The reserved variation channel is available.** The fifth per-scenario variation channel is reserved
   and unused, and its ordering is fixed, so using it does not disturb existing scenario generation.
10. **Retraining is expected.** Any perception change invalidates existing M2 controllers. Retraining is
    the planned outcome, not a cost to be avoided.
11. **Field of view is 120°, decided not studied** (operator, 2026-07-28). Retained deliberately: it keeps
    the field of view and the perception model from changing in the same step, which preserves
    interpretability against prior baselines.
12. **Detection range is ~100 m, asserted rather than derived** (FR-033a) — the sensor is taken as good to
    that range and the budget shapes quality within it. Separation-derived range remains usable only within
    roughly a quarter of that. The two envelopes are modelled separately (FR-033). The real limit depends
    on degradation modes deliberately out of scope here; asserting the envelope avoids an under-calibrated
    budget producing a confidently wrong cutoff.
13. **Numeric calibration is provisional by design.** Values are seeded from 031 bench measurements and
    datasheets and will be wrong in detail. The feature is judged on whether the structure absorbs
    correction (FR-036), not on the accuracy of the seed values.
13a. **The M1 source is known-mediocre, and that caps M2's absolute numbers** (operator, 2026-07-28: *"the
    current best M1 is so-so… at some point we go back to improved fidelity there (less pitch
    aggressiveness), but for now we are refining camera track"*). The chase can only track as well as the
    target flies, so **absolute M2 competence here is bounded by target flight quality, not by perception**.
    Three consequences: (a) low absolute figures MUST NOT be read as a perception failure; (b) only the
    **aggregate delta against the same-source baseline** is interpretable (SC-008), which is why the source
    is pinned rather than refreshed; (c) when M1 fidelity is later improved, **every existing M2 baseline
    becomes incomparable** — a future feature's numbers will jump for reasons unrelated to this one, and
    that discontinuity should be expected rather than investigated.
14. **Emitter beam width is 150° FWHM with a flat top to ±45°**, per the manufacturer datasheet (DS190
    Table 1) plus operator observation of the flat region. This is measured, not assumed. The 031 analysis
    script's 130° figure is a defect to be corrected at the source.
15. **Merged beacons are a proven configuration, not a failure mode.** The 031 field rig is a single
    detector with both beacons illuminating it, and it decodes both codes reliably — so the long-range
    merge case costs spatial separation, not tracking. Modelling must not treat it as a detection cliff.
16. **Camera variation magnitudes** (operator, 2026-07-28): boresight pointing error and roll about the
    optical axis each 10° standard deviation, **hard-clipped at 20°** — clipped rather than sampled from
    the tail, so training never sees an implausibly misaligned camera. Position varies within a 1 cm box.
    Position is negligible for bearing (±5 mm is 0.03° at 10 m) but material for obstruction, since it
    swings propeller clearance by roughly 15%; it therefore feeds the obstruction path only.
17. **Baseline mount: leading edge, ~8 inches outboard, boresight parallel to the thrust line** (operator,
    2026-07-28). Chosen because a leading-edge position removes wing obstruction entirely and 8 inches
    outboard pushes the propeller shadow past 41° inboard — reached only in a hard inboard turn — while
    full elimination would need roughly 13 inches on a 15-inch semi-span, i.e. a wingtip mount with much
    worse crash exposure on a combat aircraft for a modest additional gain. The position remains a
    parameter (FR-011a): it is a working baseline, not a frozen decision, and the feature is expected to
    validate it rather than assume it.
18. **A single camera plus a small processing module fits the available space** and delivers the perception
    output directly (operator). A second camera on the opposite side is anticipated but deferred; if built
    symmetrically at ∓8 inches it would also cancel any asymmetric drag from the first.

## Dependencies

- **031 bench measurements** supply the signal-quality calibration: acquisition timing, hold and
  re-establishment behaviour, code interference penalty, error tolerance, and range measurements in dark
  and daylight. 040 consumes these; it does not produce them.
- **Measured article geometry** — partially delivered; remaining gaps are listed in the input checklist and
  are values rather than structure.
- **Existing M1 source trajectories** are consumed unchanged, contingent on the US1 verdict.
- **Novel-path evaluation harness** — established standing practice, used to judge the retrained
  controller.

## Out of Scope

Deferred to later features, each with a recorded trigger:

- **Photon budget at full frame rate** — requires the first camera article and raw uncompressed capture.
- **Second camera** — anticipated on the opposite wing, deferred to after M2. The likely eventual form is a
  **raptor-style binocular arrangement**: two identical wide cameras on the leading edge, splayed outward,
  giving wide lateral coverage with a binocular overlap straight ahead. Splaying beyond about 19° also
  moves the propeller shadow out of frame entirely, so the arrangement chosen for coverage happens to
  eliminate obstruction as well. As a *ranging* upgrade it would be roughly twice as coarse as using the
  target's own beacon separation, so its value is robustness rather than precision: **range from a single
  visible beacon** (which separation cannot provide, and which is exactly the banking case), and
  **geometric rejection of false detections** such as ground reflections. It grows the controller's input
  vector, so it is a feature in its own right. Full analysis recorded in the backlog under perception
  representation. A rear-facing camera is a separate idea again, aimed at the same blindness bottleneck.
- **Dual field-of-view or multi-range optics** — the eventual answer to patrol-versus-engagement range,
  deferred until the field-of-view study and a photon budget exist.
- **Detection pipeline hardware** — remains gated on 031 field results.
- **Reflected-light, glint and sun modelling** — requires sun position and time-of-day in the simulator,
  plus flight data on sensor response to strong ambient light.
- **Target obstructing its own beacons** — needs the target's wing shadowing pattern; the emission
  roll-off is an adequate approximation for now.
- **Motion-blur contribution to signal quality** — awaits real camera data at high angular rates.
- **Streamer as a distributed, variable-length target** — a reward-modelling question, not a perception
  one; the real streamer is roughly 25 ft and shortens as it is cut, while the current model treats it as
  a fixed point.
- **Flight-model recalibration** — including the propeller discrepancy found here; batched for the
  flight-test feature when more than one article exists.
- **Attitude-reference (IMU) misalignment as a variation axis** — the flight controller carries a board
  alignment setting for pitch/roll/yaw offsets, and a real misalignment has already put a ~10° pitch bias
  into recorded flight data. This is a **distinct** axis from camera misalignment and does not overlap it:
  camera error biases where the target *appears*, attitude error biases where the craft *believes it is
  pointing*, and the controller receives the two independently, so robustness to one does not confer
  robustness to the other. Natural follow-on to this feature's camera-variation work. **Trigger**: after
  camera variation is exercised, or any flight where attitude bias is suspected.
- **Engine-speed-dependent propeller modelling** — blade phase, a throttle-to-engine-speed relationship,
  and the periodic attenuation that follows. **Moot at the baseline mount**, where the propeller sits
  41–61° off the boresight; the static shadow of FR-009 is sufficient there and costs no propulsion data.
  **Trigger**: a mount that places the camera behind the propeller disc — a return to a wing-top or
  centreline position, or any build where the propeller re-enters the useful field.
- **Propeller interference analysis** — whether blade passage beats against the beacon code timing or the
  frame rate, and with what consequence for decoding. The envelope arithmetic is recorded (FR-011) because
  it is cheap to preserve, but resolving it is a decoder-design research project. **Trigger**: as above,
  or a bench measurement showing margin ripple synchronised to engine speed.

## Clarifications

### Session 2026-07-28 (post-plan pass)

- Q: During a tentative lock, are separation-derived quantities computed or suppressed? → A: **Computed**
  (FR-017c). The blob centroid exists before the code decodes — detection and identification are separate,
  so the centre of the detected blob *is* the coordinate even pre-lock. Quality is presented alongside;
  let the controller learn the discount rather than handing it a gap.
- Q: Is quality (CEP) the only confidence channel in the interface? → A: **Yes** — per beacon the interface
  is `x`, `y`, `quality`; separation and tilt are bare values, and the 58-input vector admits no additions.
  Consequence (FR-017d): **identity uncertainty must be folded into quality**, because tilt sign depends on
  beacon identity while separation does not, and positional confidence can be high while identity is
  unknown. Coherent because quality is redefined as *signal* quality (FR-014), not strictly positional.
- Q: Is the detection range configured or emergent from the signal budget? → A: **Asserted — the sensor is
  good to ~100 m, with signal-to-noise proxied into the quality value** (FR-033a). The budget shapes the
  *gradient* within the envelope rather than setting a *cutoff*, because it is not calibrated well enough
  to be trusted as a limit and the modes that would set a real limit (sun angle, glint, dust, ambient,
  sensor variation) are out of scope. SC-004 revised accordingly: the test is the gradient, not the cutoff.
  Detection-quality variation filed to the backlog.
- Q: Does the retrain use the same M1 source as the prior M2 baseline? → A: **Yes if available** — it is
  the only way the aggregate delta isolates perception rather than mixing in source differences (SC-008).
  ⚠️ **Time-sensitive**: dumps are tagged `retain=expire` and auto-delete after 30 days unless pinned
  (Principle VIII), so the baseline's source must be **verified and pinned early**, not at retrain time.
  If it has expired, the substitution is stated and the delta reported as confounded.

### Session 2026-07-28

- Q: What compute budget applies to the new perception model? → A: Measure the effect on **total**
  evaluation throughput, ceiling **≤10% regression measured against the prior M2 run**; a breach escalates
  to an explicit accept-or-optimise decision rather than being absorbed silently. Operator expectation:
  against the per-step physics evaluation this should land well under 10%.
- Q: What obstruction threshold gates the mount choice? → A: **None — let it ride.** Most of the
  mounting-error distribution sits within 10° with a hard limit at 20°, so rather than gate on a fixed
  clear cone, report the **distribution** of obstruction onset (median / 95th / clipped extreme) and
  observe the outcome. Baseline mount refined to **leading edge, 8″ outboard, ~1″ above the thrust line**
  (which runs through the centre of the leading edge).
- Q: What does the controller see during acquisition? → A: **A bearing with large positional variance in
  the quality signal** (tentative lock), improving as the code integrates. Critically, the **state machine
  is derived by the controller, not exposed**: the camera presents bearing plus a quality signal, and the
  quality value alone encodes tentative-lock and hold conditions. No categorical state reaches the
  controller.
- Q: How is the M2 competence change attributed? → A: **Aggregate delta only, one retrain.** The question
  at this stage is coarse — *are we in the right room, and is this more honest?* — and further refinement is
  inevitable anyway, including craft changes. No competence floor gates the feature. Per-term attribution
  stays *possible* later (every quantity is a knob, so a fixed elite can be re-evaluated under alternative
  configurations) but is not required here.

**Q1 — Field of view and operational range → RESOLVED: keep 120°, design to ~100 m detection.**

Operator decision: retain the 120° horizontal field and build the signal-quality model around an expected
detection range of ~100 m. The intent is explicitly **to get the plumbing in place** while field hardware
is being proven — a higher-fidelity simulator with the right structure and the right knobs, whose numeric
calibration arrives later when real-to-simulation measurement is available.

This resolves the tension raised during scoping rather than dismissing it. The analysis stands — a wide
field is roughly 40 dB down on the narrow-camera link budget, and 120° needs a physically larger sensor
than assumed — but those are **calibration and hardware-selection findings, not blockers**, because 040's
deliverable is the structure. Keeping 120° also avoids perturbing the field of view and the perception
model in the same change, which keeps the retrained controller interpretable against prior baselines.

Recorded consequences (see FR-033):

- **Detection and range-inference envelopes are different, and both must be modelled.** At 120° over 320
  pixels, the beacon pair subtends roughly one pixel at 100 m — unresolvable — so separation-derived range
  is at its floor long before detection fails. Separation becomes usable somewhere around 25 m.
- The intended behaviour is therefore **bearing available to ~100 m, range only inside ~25 m**, degrading
  smoothly between. This is physically honest and mission-shaped: patrol acquires on bearing alone;
  range emerges as the engagement closes.

**Q2 — Emitter beam width → RESOLVED from the datasheet: 150° FWHM with a flat top to ±45°.**

The manufacturer datasheet (DS190 Table 1) gives a typical FWHM beam angle of **150°** for the emitter in
use, alongside 286 mW/sr radiant intensity and 1250 mW radiometric power at rated current. The 031
optical-outcome document was correct; **the 031 analysis script is wrong** — it applies a 130° half-power
width and attributes it to this datasheet. That defect should be corrected at the source.

Operator observation adds the shape the single FWHM number hides: relative intensity is **flat out to
roughly ±45°**, then rolls off through the 50% points at ±75°. So the pattern is a **flat top with
shoulders**, not a cosine power law — a `cos^m` fit under-reads the flat region by around 16% and
over-reads the skirt.

Consequence for the beacon enclosure: with each emitter flat over ±45° and the enclosure faces 90° apart,
**the flat regions tile the outboard hemisphere** — every outboard direction lies within 45° of some
emitter axis. Illumination toward the chase is therefore near-uniform, which confirms the operator's
earlier judgement that off-axis emission modelling is low priority, and makes the emission term close to
constant over the emitting hemisphere with roll-off only near the enclosure's mounting shadow.
