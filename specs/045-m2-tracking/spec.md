# Feature Specification: 045 — M2 tracking: navigation integrity, a re-fly, then a realistic-signal M2 estimate

**Feature Branch**: `045-m2-tracking`
**Created**: 2026-11-10
**Status**: Draft
**Input**: Operator scoping 2026-11-08 → 2026-11-10 and the seed in [README.md](README.md). Read README for *why*, for the evidence
(the 11-08 and 10-18 flight analyses) and for the INAV setting inventory. This document is *what*, and governs where they differ.
Reasoning and research for every phase, including Phase 0, belong in `plan.md` / `research.md`, not here.

---

## READ-FIRST — document order

| document | role |
|---|---|
| **spec.md** (this) | **governs.** Scope, execution order, requirements, acceptance. |
| [README.md](README.md) | derivation of record: the front tasks, the N1–N6 / B1 pre-tasks, the INAV setting inventory by class, the M2 carry-over (camera gates, input-scale audit, 041 hand-over). |
| [`../044-m1-craft-history/outcome.md`](../044-m1-craft-history/outcome.md) | what 044 established; the flight baseline of record (044-t1 g800). |
| [`../044-m1-craft-history/flight-analysis-20261108.md`](../044-m1-craft-history/flight-analysis-20261108.md) | § 6–7: the faulted nav estimate this feature responds to. |
| [`../042-camera-physics/README.md`](../042-camera-physics/README.md), [`../031-beacon-camera/bench-journal.md`](../031-beacon-camera/bench-journal.md) | where the camera and signal-lock measurements come from. |

---

## Why this feature exists

The policy flies a path, or in M2 phase 1 a target, that is defined in the navigation frame fixed at engage. **INAV's navigation
estimate has failed under the policy on every ACRO-era flight**: vertical divergence of 58–101 m, and on 11-08 a 2.3 s horizontal
freeze followed by a 105 m reset. **Attitude is off 10–25° in hard manoeuvres.** The simulator feeds the policy perfect state, so
training never sees any of this. A self-referential path survives smooth drift. It does not survive resets, freezes or tilt. Until
the frame is trustworthy, a flight measures the frame, not the policy.

The M2 goal for this feature is the first answer in a long while to **"does M2 work at all"** on the current camera architecture.
That means a simulation-only training run with the field-measured camera geometry and a **perfect signal** (an always-locked
receiver). The last M2 runs were 040/041, on a sensor model that is now out of date. Realistic signal (acquisition latency, lock and
dropout, occlusion variations) comes in later runs, once M2 is closer to flying. The camera field data is recorded now so those runs
start from measurement. A simulation result is useful before any M2 flight, but only if the M1 foundation underneath it has been
checked in the air.

⭐ **The long pole is the real camera** (operator 2026-11-10): first *modelling* real-world camera performance honestly (lens
aberrations, signal quality, mounting), then *improving* the real camera. These form a loop: measure → model → find which gap
matters → improve the hardware → re-measure. This feature turns that loop once on the model side. It reviews every camera
variation and lens assumption built in 040 against the field data, and records which gaps remain for the hardware side (042/031).

### Decisions already made (not reopened)

- **The 044 SC-002 pitch 2–3 Hz miss is backlog, not a 045 gate** (operator 2026-11-10). The result is close, and M2 fidelity work
  comes before fine-tuning basic control. Filed in [BACKLOG](../BACKLOG.md) § 044 deferrals.
- **Estimator fault detection is advisory**: flagged and logged, never a new disengage, no change to operations. The intended
  direction is the companion's own IMU during autoc, not a stricter INAV gate.
- **INAV keeps the rate loop and the safety fallback** (failsafe, pilot override, return). 044 said INAV stays the bridge "for
  attitude/position estimation" through 045; that part **is revisited here** (US6, US4's INAV-role study).
- **The 045 M2 training run is simulation-only and will not fly.** It runs on a **perfect signal**: it asks whether M2 works at all,
  not what the hardware's signal costs (operator 2026-11-10). Realistic signal is a later run (phase 5).
- **The first implementation pass ends at the re-fly gate** (G2). Phases 3–5 are specified now and planned and implemented after it.
- Article 2 flies only if it is ready and the weather allows. Its absence does not block the gate.

---

## Execution order *(governs task ordering — not the story priorities)*

| phase | content | stories | exits through |
|---|---|---|---|
| **0a — front tasks** | write-through dmp cache; fast, resumable BLE log download | US1, US2 | — (independent; may merge alone) |
| **0b — pre-tasks** | measure nav/attitude drift offline; review INAV configuration for all-attitude flight and define INAV's role during autoc; advisory fault detection plus additional blackbox fields; companion inertial frame and simulator parity (adopt or defer) | US3, US4, US5, US6 | **G1 — re-fly readiness** |
| **1 — re-fly** | fly 044-t1 g800 on the Phase 0 configuration; article 2 in the same window if possible | US7 | **G2 — re-fly gate** (end of first implementation pass) |
| **2 — camera update** | bring the field-measured camera geometry into the simulator; review every 040 camera variation and the lens model against the field data; record the signal measurements for later; list the remaining model gaps for the hardware side | US8 | **G3 — camera parameters landed** |
| **3 — M2 sim + training config** | current camera architecture; perfect-signal mode; quick input-domain check (all inputs), rescale only where forced | US9 | **G4 — M2 smoke green** |
| **4 — M2 training run** | simulation-only, perfect signal: does M2 work at all | US10 | feature outcome |
| *5 — realistic signal (later)* | the 040 signal model on measured values: acquisition latency, lock and dropout, occlusion variations | US11 | after phase 4 shows M2 works; may become its own feature |

- **G1 (re-fly readiness):** US3 reported, with the attitude-stability budget; US4's configuration change set (including the
  calibration policy and compass alignment) recorded, read back and on the aircraft; INAV's reduced-role design written (built after
  G2); accelerometer calibration, board alignment and outdoor compass calibration done and recorded per article (FR-007b); US5
  verified on the bench; US6 adopted (shadow on the re-fly) or explicitly deferred with US3's numbers.
- **G2 (re-fly gate):** the re-fly is flown and analysed (SC-006, SC-007). Nothing in phases 2–4 starts before G2.
- **G3:** no M2 training starts against the pre-field-work camera model (the README's Gate 1).
- **G4:** the M2 smoke passes on the updated simulator before the full run.

---

## Clarifications

### Session 2026-11-10 (at specify)

- Q: Is the tracker input-scale audit (README Gate 2) in scope? → A: Yes, but light, and the M1 inputs stay roughly as they are
  (refined in clarify, below; FR-022).
- Q: What does "signal" mean in FR-024? → A: The beacon optical link (detection, quality, lock, acquire/hold/reacquire timing), as
  distinct from field-of-view and occlusion geometry. Defined in FR-021.
- Q: How is the cost of realistic signal separated? → A: Not in this run. The 045 M2 run is a perfect-signal training run that asks
  "does M2 work at all" on the field-measured camera (FR-020a, US10). Realistic signal (acquisition, lock, dropout, occlusion
  variations) comes later (phase 5, US11), with the phase 4 run as its comparison point. The camera field data is recorded now.
- Operator note: the 040 camera variations (lens aberration, signal quality, mounting and so on) need re-evaluating against the
  field data. Modelling real camera performance, then improving the real camera, is the hardest part of getting to a real target
  (FR-020b, US8).

### Session 2026-11-10 (clarify)

- Q: If the companion inertial frame (US6) is adopted at G1, does the policy fly on it during the re-fly? → A: **Shadow.** It runs
  and is logged next to INAV's estimate, and g800 keeps flying on INAV. Plan/research also estimates the attitude drift of the
  candidate sources under the measured flight load, as the attitude-stability budget (target < 1° over a span) (FR-006a).
- Q: Formal normalization changes M1's shared inputs — re-bake M1, or not? → A: Keep M1 inputs roughly the same. Do a quick
  domain-stats check of every input from the latest training and the latest flight; aim for roughly the same range as a secondary
  effort; change a constant only if forced (saturated or dead input). The tracker's camera inputs replace direction-to-target and
  are small integers anyway (FR-022). This supersedes "formal normalization pulled in".
- Q: If article 1's receiver turns out GPS-only, what then? → A: It is believed to be a Matek **M8Q-5883** (u-blox M8 class), which
  should support the other constellations. Confirm on the bench, update its firmware if that is possible and needed, and if it
  still cannot, swap in one of the other receivers on hand before the re-fly (FR-008).
- Q: Is navigation state perfect in the phase 4 M2 run? → A: **Yes.** Perfect navigation state, like the perfect signal. Estimator
  errors (N5) come in with phase 5 realism (FR-020a).
- Operator addition: the investigation should consider turning off INAV's automatic accelerometer calibration in favour of a bench
  calibration (FR-007a).
- Q: Does the re-fly fly with INAV in its reduced role (N6)? → A: **No, B.** The re-fly flies the tuned configuration for this use
  case: estimator error bounds, sample rates, the questionable barometer, more satellites, the calibration policy, plus
  fallback/gate settings that don't touch the rate loop or what the policy observes. The reduced-role design is written before G1
  and built after G2 (FR-010).
- Operator note: every board (controller, companion, receiver/compass) is mounted on double-sided foam tape: decent damping, but
  each board has its own resonances and its alignment is approximate. The re-fly prep takes a more serious pass at rough
  accelerometer calibration, board alignment and outdoor compass calibration (FR-007b).

---

## User Scenarios & Testing *(mandatory)*

The actor throughout is **the operator**: the person who trains, flies, downloads logs and judges results.

### User Story 1 — Training-box reports stop paying egress (Priority: P1, phase 0a)

The operator runs reports on the training box as often as they like. Each training artefact leaves the archive at most once, and
other hosts can still read any generation from the archive as soon as it is written.

**Why this priority**: data egress is the largest item on the cloud bill (Oct 2026: 378 GB, $25 of $32), and M2 reporting reads
more than M1. Independent of everything else.

**Independent Test**: one full report cycle on a run trained with the cache in place; the archive's access logs show the result.

**Acceptance Scenarios**:

1. **Given** a run trained on the training box with the cache configured, **When** a full report cycle runs there, **Then** the
   archive shows zero downloads by that host of artefacts it wrote itself.
2. **Given** the cache is configured but unwritable, or a cached entry is corrupt, **When** a reader touches it, **Then** the tool
   stops with the path named and does not silently fall back to the archive.
3. **Given** the cache is not configured, **When** any tool starts, **Then** it says so once and behaves exactly as today.

---

### User Story 2 — Field log download is fast and survives a drop (Priority: P1, phase 0a)

At the field, the operator downloads a full flight log from the companion over Bluetooth in about a minute. If the link drops,
the download resumes where it stopped.

**Why this priority**: the re-fly (US7) and every later flight depend on it. Today 300 kB takes ~10 minutes and restarts from
zero on a drop.

**Independent Test**: on the bench, download a full-size log, interrupt it mid-transfer, resume, and compare the bytes with a
wired read.

**Acceptance Scenarios**:

1. **Given** a ~300 kB flight log, **When** the operator downloads it with the companion and phone at field distance, **Then** it
   completes in under 60 s.
2. **Given** a download interrupted at any point, **When** the operator reconnects, **Then** it continues from the last confirmed
   position and the final file matches the source byte for byte.
3. **Given** the transfer is slow, **When** the operator asks why, **Then** the measured cause of the old slowness is on record.

---

### User Story 3 — We know how wrong the navigation and attitude frame is (Priority: P1, phase 0b)

From the logs we already have (11-08, 10-18), the operator gets a report of how far attitude, velocity and position drift over an
engaged span when propagated from the inertial sensors alone, compared with INAV's estimate including its faults.

**Why this priority**: it sizes everything else in Phase 0. It decides whether a companion-owned frame (US6) is worth building, and
which attitude source is better in manoeuvres. Offline; no flight needed.

**Independent Test**: the report reproduces from the committed tools and the existing logs.

**Acceptance Scenarios**:

1. **Given** the 11-08 and 10-18 logs, **When** the analysis runs, **Then** it reports, per engaged span: attitude drift at 17 s
   (sensor-only vs INAV), velocity and position drift for a span-local propagator, and each INAV freeze, reset or step.
2. **Given** that report, **When** the operator compares attitude sources, **Then** the report states which source is more accurate
   in sustained high-load manoeuvres, and by how much, with the reference's own uncertainty stated.

---

### User Story 4 — INAV is configured for all-attitude flight, and its role during autoc is defined (Priority: P1, phase 0b)

The operator has a reviewed, recorded flight-controller configuration for all-attitude, high-load flight. It covers sensor fusion
and estimator margins, satellite constellations and receiver dynamics, altitude source and its weighting, filters, fallback
behaviour, logging and hardware hygiene. Each change is classified by what it touches. The role INAV plays while the policy flies
is written down: a rate loop and simple gates, with its full estimator kept for the fallback.

**Why this priority**: the 11-08 and 10-18 analyses show the configuration of record (mostly cruise-airplane defaults) does not hold
up under the policy. The re-fly is worthless without this.

**Independent Test**: on the bench, the recorded configuration loads and its status read-back confirms each setting took effect.
The change set is reviewable against the configuration of record.

**Acceptance Scenarios**:

1. **Given** the configuration of record, **When** the review completes, **Then** every changed setting is recorded with its class
   (inside the rate loop / what the policy observes / navigation estimate / fallback / logging / hardware), the evidence for it, and
   whether it changes what the policy was trained on.
2. **Given** article 1's satellite receiver, **When** it is inspected on the bench, **Then** its model and supported constellations
   are recorded. If it cannot use more than one constellation, that is recorded as a hardware finding, not a configuration change.
3. **Given** a constellation or receiver setting is changed, **When** the controller reboots, **Then** a status read-back confirms
   the receiver accepted it. The controller is known to revert a rejected setting silently.
4. **Given** the INAV-role study, **When** it completes, **Then** it records, for each thing autoc depends on INAV for (rate loop,
   state to the companion, fallback), whether it depends on INAV's navigation estimate, and the gates INAV applies during autoc.

---

### User Story 5 — Estimator faults are visible in every log, and nothing else changes (Priority: P1, phase 0b)

Every companion log tick says whether INAV's position and altitude estimates were valid, how uncertain they were, and whether the
estimate was frozen or stepped on that tick. The blackbox carries the estimator fields the drift analysis currently has to infer.
Flight behaviour is unchanged.

**Why this priority**: the 11-08 fault went unnoticed until analysis days later. The re-fly must carry this.

**Independent Test**: on the bench, block the satellite receiver mid-run. The ticks are flagged and the companion keeps flying the
policy exactly as before.

**Acceptance Scenarios**:

1. **Given** a bench run, **When** satellite reception is blocked, **Then** the affected ticks are flagged invalid or frozen, and on
   restore a step is flagged.
2. **Given** a flagged tick, **When** the companion runs the policy, **Then** engagement, disengagement and outputs are unaffected.
3. **Given** a flight log with flagged spans, **When** the flight gates and bench-join tools run, **Then** they list those spans
   automatically.

---

### User Story 6 — A span-local inertial frame owned by the companion, and the simulator matches it (Priority: P2, phase 0b — adopt or defer)

At engage, the companion takes INAV's state once, then carries attitude, velocity and position forward on its own inertial sensor
at sensor rate. It is continuous and gravity-correct, and it never resets inside a span. Satellite position is a soft cross-check.
Whatever estimator the policy flies on, the simulator gives the policy that estimator's errors, not perfect state.

**Why this priority**: it is the architecture step the operator wants, but its size and value come from US3. The spec requires a
decision with numbers, not the build.

**Independent Test**: replayed against a recorded flight's sensor stream, the frame's drift over 17 s is within US3's measured
budget, with no discontinuities.

**Acceptance Scenarios**:

1. **Given** US3's report and the attitude-stability budget, **When** G1 is assessed, **Then** this story is either adopted or
   deferred, with the numbers that justify the choice.
2. **Given** it is adopted, **When** the re-fly flies, **Then** the frame runs in shadow: logged next to INAV's estimate for the
   same spans, with the policy still flying on INAV.
3. **Given** it is adopted, **When** a span is propagated, **Then** the frame has no discontinuity anywhere in the span.
4. **Given** the policy flies on an estimator other than perfect state, **When** it is trained or evaluated, **Then** the simulator
   applies that estimator's measured error characteristics.

---

### User Story 7 — The re-fly: clean navigation and attitude data on the current model (Priority: P1, phase 1, gate G2)

The operator flies 044-t1 g800 on the Phase 0 configuration and gets a flight log in which the navigation frame stayed continuous
through engaged spans. The faults that remain are flagged and measured. If article 2 is ready and the weather allows, it flies in
the same window.

**Why this priority**: it is the gate. It shows Phase 0 worked in the air, and it gives a clean baseline before M2.

**Independent Test**: the flight's own logs, analysed with the 11-08 tools plus the US5 flags.

**Acceptance Scenarios**:

1. **Given** the re-fly, **When** it is analysed, **Then** every engaged span reports its estimator faults, and spans with faults
   are excluded from the M1 comparison and listed.
2. **Given** the same policy as 11-08, **When** tracking and control metrics are compared as flown, **Then** the differences can be
   attributed to configuration, because nothing that changes what the policy was trained on was changed (or the simulator changed
   with it and g800 was re-evaluated first).
3. **Given** article 2 flew, **When** its data is analysed, **Then** it is reported separately from article 1's, with its own
   configuration of record.

---

### User Story 8 — The camera model and its variations reflect the field measurements (Priority: P2, phase 2)

The simulator's camera (lens, field of view, angular resolution, range and response) uses the values measured in the 031/042 field
and bench work. Each value traces to its measurement. The per-scenario **camera variations built in 040** are reviewed against the
same data: boresight and roll misalignment, mount translation, wing thickness, the deferred ambient term, and the lens model itself
(today an ideal projection with no aberration, distortion residual or centroid noise). Each one is kept, updated, added or deferred,
with the reason. The gaps the simulator still cannot represent are listed for the hardware side.

**Why this priority**: every M2 number depends on it, and real-world camera performance is the hardest part of getting to a real
target. A run on the old model is spent.

**Independent Test**: each camera parameter and variation in the tree cites its measurement or is marked provisional. A replay of a
recorded field geometry through the simulator matches the recorded detections within the measurement's stated uncertainty.

**Acceptance Scenarios**:

1. **Given** the field measurements, **When** the camera parameters land, **Then** none of the previous lens's values remain.
2. **Given** each 040 camera variation, **When** it is reviewed, **Then** its magnitude is re-derived from measurement or a stated
   build tolerance (the 040 alignment values were flagged as doubtful), and its fate (kept / updated / deferred) is recorded.
3. **Given** the field data shows an imperfection the model lacks (for example lens aberration or bearing noise from blob
   centroiding), **When** the review completes, **Then** it is either added as a variation or recorded as a known gap.
4. **Given** a measurement is not yet available, **When** the parameter is needed, **Then** it is listed as provisional with its
   source estimate, and the M2 result names it as a caveat.

---

### User Story 9 — M2 simulation runs on the current camera with a perfect signal and usable inputs (Priority: P2, phase 3)

M2 training sees the target through the field-measured camera geometry (field of view, angular resolution, airframe occlusion). The
receiver is always locked: every beacon in view is reported at full quality with no acquisition delay. A quick check of every
input's domain confirms that none is saturated or dead. M1's shared inputs stay roughly as they are.

**Why this priority**: it isolates the question "does M2 work at all on this camera?" from the signal-realism question, after a
long gap with no M2 runs.

**Independent Test**: the M2 smoke on the updated simulator. The startup log states the signal mode, and the input-investment
report shows the tracker inputs in use.

**Acceptance Scenarios**:

1. **Given** the updated simulator, **When** the M2 smoke runs, **Then** it trains without crash or schema failure, its log states
   that the signal is perfect, and the policy visibly uses the tracker inputs.
2. **Given** a beacon outside the field of view or behind the airframe, **When** it is simulated, **Then** it is not seen. Geometry
   is never perfect, only the signal.

---

### User Story 10 — A simulation-only M2 run: does M2 work at all (Priority: P2, phase 4)

The operator gets the first M2 result on the current camera architecture: does the policy acquire and track a target, and how well.
It comes from a training run on a perfect signal that is never flown, and it is comparable on definitions with the earlier M2
results.

**Why this priority**: the feature's outcome. It decides whether the realistic-signal work (US11) is worth doing next.

**Independent Test**: the run's fixed-evaluation report.

**Acceptance Scenarios**:

1. **Given** the completed run, **When** it is evaluated on the fixed evaluation set, **Then** the report gives tracking occupancy,
   per-step error, reacquire time after the target leaves view, and crash rate, on the same definitions as the 040/041 tracker
   results.
2. **Given** a comparison with 040/041, **When** it is reported, **Then** it states that those runs used a realistic-signal model and
   this one does not, so a better number is not attributed to the camera alone.

---

### User Story 11 — Realistic signal: acquisition, lock and dropout (Priority: P3, phase 5 — later, after US10)

Once M2 is shown to work, training adds the signal as the hardware delivers it. Acquisition takes a realistic time from entering
view. Lock is held, degraded or lost the way the measured receiver chain does it (rotation rate, range, sun, occlusion variations),
and reacquisition takes a realistic time.

**Why this priority**: it matters closer to flying M2. The measurements are recorded now (US8) so this starts from data.

**Independent Test**: on a scripted geometry set in simulation, acquisition time, lock duration and dropout match the measured
values.

**Acceptance Scenarios**:

1. **Given** a scripted pass through the field of view, **When** it is simulated with realistic signal, **Then** acquisition, lock
   and loss match the measured behaviour for that geometry and rotation rate.

---

### Edge Cases

- **Article 1's receiver supports only one constellation**: firmware update if possible, otherwise swap in a multi-constellation
  receiver from stock before the re-fly (FR-008). Article 2's receiver (M10Q-5883) supports several.
- **A board's foam-tape mount resonates in the band the estimator or policy uses, or its alignment shifts under load**: recorded
  per board with its frequency and axis. A mount change is hardware hygiene, A/B-able on the next flight. It is not a reason to
  hold the re-fly unless it saturates a sensor.
- **Article 1's compass is mis-aligned through the board-alignment path**: corrected and recorded in the configuration change set
  before mag calibration. Calibrating a mis-aligned compass bakes the error in.
- **The controller reverts a rejected setting silently**: no setting counts as applied without a read-back after reboot.
- **US3 shows INAV's attitude is better than the companion's sensor in manoeuvres**: US6 is deferred, or narrowed (for example
  INAV attitude with companion position), on those numbers.
- **A change to what the policy observes is unavoidable**: a change that makes attitude *more accurate* (fusion weights,
  magnetometer calibration) moves flight closer to the simulator's perfect state and is allowed. A change that alters *delay* or the
  rate loop (filters, rate gains) is held, unless the simulator changes first and g800 is re-evaluated.
- **Rain or wind cancels the window**: the re-fly waits. Article 2 is dropped from the window before article 1.
- **The re-fly still shows estimator faults**: the gate passes if the faults are flagged and measured. Faults that persist are
  evidence for US6, not a reason to hold the gate.
- **A camera measurement is not ready by phase 2**: provisional values with a stated source, named in the M2 result.
- **The M2 smoke shows the policy ignores the tracker inputs**: the input-domain check (FR-022) is looked at first, before the
  objective or the training ramp is blamed (041 t4–t6).
- **A shared M1 input is found saturated**: it is the one case where a shared-block constant changes in 045. The M1 source and g800
  are then re-evaluated before either is used as an M2 source or comparator.

---

## Requirements *(mandatory)*

### Functional Requirements — phase 0a (front tasks)

- **FR-001**: Every training-artefact read and write on the training box MUST go through one local cache that is written when the
  archive is written, and read before the archive is asked.
- **FR-002**: The archive MUST remain the record and MUST be written immediately, so other hosts can read any generation as soon as
  it exists.
- **FR-003**: The cache MUST be enabled only by explicit configuration. An unwritable cache or a corrupt entry MUST stop the tool
  with the path named. An eviction sweep MUST follow the archive's retention classes.
- **FR-004**: The flight-log download MUST resume from the last confirmed position after a drop, and MUST verify the completed
  file against the source.
- **FR-005**: The cause of the current download slowness MUST be measured and recorded before it is fixed.

### Functional Requirements — phase 0b (pre-tasks)

- **FR-006**: An offline analysis MUST report, per engaged span of the 11-08 and 10-18 flights: attitude drift at 17 s from
  sensor-only propagation vs INAV; velocity and position drift for a span-local propagator; and INAV's freezes, resets and steps.
- **FR-006a**: Planning MUST estimate an **attitude-stability budget**: the attitude drift over a 17 s span under the load measured
  in flight (sustained pulls of ~4.4 g, peaks 5–7.5 g), for each candidate attitude source. The candidates are INAV's AHRS, fusion
  on the companion's own inertial sensor at sensor rate, and any inertial part with on-chip fusion worth considering. The target is
  **< 1° over a span**. The estimate MUST state its error sources (gyro bias and its stability, scale error, vibration, rate and
  range limits, read rate) and MUST be checked against FR-006's measured drift.
- **FR-007**: The flight-controller configuration MUST be reviewed for all-attitude, high-load flight. The record MUST classify
  every changed setting as inside the rate loop, observation path, navigation estimate, fallback, logging or hardware, and state its
  evidence and whether it changes what the policy was trained on.
- **FR-007a**: The review MUST decide the **inertial calibration policy**. Specifically: whether to disable the controller's automatic
  calibrations in favour of a recorded bench calibration. These are the in-flight accelerometer bias learning (fed by satellite and
  barometer residuals, so it learns from faulted data) and the gyro-zero and 1 g gravity calibration taken at every boot. The
  stored 1 g reference, currently 3.8 % below standard gravity, MUST be explained on the bench (scale error, or a calibration taken
  in motion). The companion's own inertial sensor MUST be bench-calibrated to the same standard (zero, gain, gyro bias, temperature
  sensitivity) if US6 is adopted.
- **FR-007b**: **Sensor mounting** MUST be part of the review. The flight controller, the companion and the receiver/compass are
  each held on with double-sided foam tape (operator 2026-11-10). That gives decent damping, but each board then has its own
  resonances in different directions, and its alignment can be off or shift under load. For each board, the review MUST estimate
  its mount resonances from the pre-filter inertial logs and check its alignment (level-boot gravity, and in-flight fit against
  satellite-derived acceleration). Before the re-fly, the operator does a rough accelerometer calibration, a board-alignment check
  and an **outdoor** compass calibration on each article. Each result is recorded in the configuration change set.
- **FR-008**: Article 1's satellite receiver (believed: Matek M8Q-5883) MUST be identified, with its firmware and supported
  constellations, before any constellation change is made. If it cannot run more than GPS, even after a firmware update where one
  is possible, it MUST be replaced with a multi-constellation receiver before the re-fly. Its compass is then re-aligned and
  re-calibrated.
- **FR-008a**: Each article's compass alignment MUST be verified against its physical mounting. With no explicit compass
  alignment set, the flight controller applies its own board alignment to the external compass. On article 1 that is a 170° roll
  (10° off flat) and a 90° yaw. Any resulting tilt error MUST be checked as a contributor to the attitude-dependent heading error
  seen in manoeuvres.
- **FR-009**: Every applied controller setting MUST be confirmed by a read-back after reboot.
- **FR-010**: INAV's role during autoc MUST be recorded: what the rate loop, the state stream to the companion and the fallback each
  depend on, and which estimator-independent gates INAV applies while the policy flies. The **reduced-role design** (rate loop plus
  simple gates, full estimator only for the fallback) MUST be written before G1 and is **built after G2**. For the re-fly, INAV
  flies its existing role on the FR-007 configuration tuned for this use case: estimator error bounds, sample rates, the barometer's
  role, satellite constellations and the calibration policy, plus fallback and gate settings that change neither the rate loop nor
  what the policy observes.
- **FR-011**: The state stream from the controller MUST carry the estimator's position and altitude validity and uncertainty. The
  companion MUST flag every tick that is invalid, frozen or stepped.
- **FR-012**: Fault flags MUST NOT change engagement, disengagement or policy outputs.
- **FR-013**: The flight-analysis tools MUST list flagged spans automatically and MUST be able to exclude them.
- **FR-014**: The blackbox MUST record the estimator fields the drift analysis needs: at least the navigation position and velocity
  estimate and its validity, uncertainty, fusion weight and satellite count.
- **FR-015**: A decision MUST be recorded at G1 on the companion-owned span-local inertial frame (adopt, narrow or defer), citing
  FR-006's and FR-006a's numbers. If adopted, the frame MUST be continuous through a span with no internal resets, and the simulator
  MUST give the policy that estimator's measured errors before any training run that relies on it. **On the re-fly it runs in
  shadow**: computed and logged next to INAV's estimate, never fed to the policy.

### Functional Requirements — phase 1 (re-fly)

- **FR-016**: The re-fly MUST fly 044-t1 g800 on the configuration recorded under FR-007, with the policy on INAV's estimate. Any
  companion frame (FR-015) runs in shadow only.
- **FR-017**: For the re-fly, settings in the rate loop, and settings that change observation delay, MUST be held at the
  configuration of record. The exception is a setting whose change the simulator models first, with g800 re-evaluated on it.
  Changes that only improve estimate accuracy are allowed.
- **FR-018**: The re-fly analysis MUST report estimator faults per engaged span, and MUST compare tracking and control with the
  11-08 flight as flown, excluding faulted spans and listing them.
- **FR-019**: If article 2 flies, its data MUST be analysed and reported separately, with its own configuration of record.

### Functional Requirements — phases 2–5 (M2; specified now, implemented after G2)

- **FR-020**: Every camera parameter the simulator uses MUST trace to a field or bench measurement, or be marked provisional with
  its source. Signal measurements from the field work MUST be recorded with their sources in phase 2, even though phase 4 does not
  use them.
- **FR-020b**: Every per-scenario camera variation from 040, and the lens model, MUST be reviewed against the field data, with each
  one's fate (kept / updated / added / deferred) and the reason recorded. Variations that are camera **geometry or imperfection**
  (alignment, mounting, lens distortion or aberration, bearing noise) apply in phase 4. Variations that are **signal** (ambient,
  link quality) belong to phase 5. Model gaps that remain MUST be listed as inputs to the camera-hardware work (042/031).
- **FR-020a**: The simulator MUST have a **perfect-signal mode**: always locked, every beacon in view reported at full quality with
  no acquisition delay. Field of view and airframe occlusion still apply. The mode MUST be set explicitly in configuration and
  stated in the run log. Phase 4 runs in this mode, with **perfect navigation state** (no estimator error model).
- **FR-021** *(phase 5, later)*: M2 simulation's **signal model** MUST reflect the measured receiver chain. *Signal* means the beacon optical link: whether
  each beacon is detected, its reported quality, its lock state, and the time to acquire, hold and reacquire lock. Field-of-view
  limits and airframe occlusion are geometry, not signal. 040 already models the link budget and the acquisition sequence, with the
  timings measured on the shipped receiver. Phase 5 MUST (a) replace every remaining *assumed* link value with a field or bench
  measurement, or list it as provisional; and (b) add any loss mechanism the field work found that the model lacks (for example,
  lock loss when the aircraft's rotation sweeps a beacon through a narrow lensed receiver field faster than the receiver can hold
  it); and (c) add occlusion variations across scenarios.
- **FR-022**: A **quick input-domain check** MUST cover every input (M1 shared block and tracker-specific): the spread, bounds and
  fraction of ticks at a bound, from the most recent training (044-t1) and the most recent flight (11-08), and for the tracker
  inputs from the M2 smoke. The goal is all inputs in roughly the same range, as a **secondary** effort. Constants change only
  where forced: an input that saturates, or one so quiet the policy cannot select on it (the 041 failure mode). **M1's shared
  inputs stay roughly as they are.** The tracker's camera inputs replace the M1 direction-to-target inputs and are already small
  quantized integers. The README's Gate 2 audit is this check. The backlog's formal input-normalization item stays in the backlog.
- **FR-023**: The M2 run MUST be simulation-only and MUST NOT be flown.
- **FR-024**: The phase 4 result MUST be reported on the same tracking definitions as the 040/041 tracker results, and MUST state
  the signal difference: 040/041 used a realistic-signal model, phase 4 uses a perfect signal. When phase 5 runs on realistic
  signal, the phase 4 run becomes its perfect-signal comparison point. The difference between them is the cost of the signal
  (FR-021's meaning).

### Functional Requirements — process

- **FR-025**: Changes that affect training determinism MUST pass the operator-driven regression gate before any run.
- **FR-026**: Every schema change (state stream, flight log, training artefact) MUST be greenfield, with readers that fail loud on a
  mismatch. Retained baselines MUST be extracted before a change orphans them.
- **FR-027**: The configuration of record MUST be updated in the repository with each change set, so every flight names the exact
  configuration it flew.

### Key Entities

- **Configuration change set**: an ordered set of controller settings, each with its class, evidence, read-back and effect on what
  the policy was trained on. One per flight.
- **Estimator fault flag**: per tick: position and altitude validity, uncertainty, and frozen or stepped. Advisory only.
- **Drift report**: per span and per source: attitude, velocity and position drift over the span.
- **Span-local inertial frame**: the companion's own state estimate, from engage to disengage. Continuous, with no internal resets.
- **Camera parameter record**: each simulator camera value and per-scenario variation with its measurement source (or a
  provisional flag), its fate in the review, and whether it is geometry (phase 4) or signal (phase 5).
- **Camera model gap list**: real-camera behaviour the simulator cannot yet represent. The hand-off to the hardware-improvement
  side.
- **Signal mode**: perfect (always locked; phase 4) or realistic (the measured signal model; phase 5). Explicit in configuration,
  stated in every run log.
- **Signal model** *(phase 5)*: acquisition latency, lock retention and loss, reacquisition and occlusion variation, each traced to
  measurement.
- **M2 performance result**: the fixed-evaluation tracking report on 040/041 definitions, labelled with its signal mode.

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Phase 0 (to G1)**

- **SC-001**: For one full report cycle on a run trained with the cache, the training box downloads zero artefacts it wrote itself,
  and at most one copy of anything else. The month's egress stays inside the free tier (100 GB).
- **SC-002**: A ~300 kB flight log downloads in under 60 s at field distance. An interrupted download resumes and completes
  byte-identical.
- **SC-003**: The drift report exists for every engaged span of 11-08 and 10-18. It reproduces from committed tools and names the
  better attitude source in high-load manoeuvres. The attitude-stability budget states, for each candidate source, whether < 1°
  over a span is reachable under the measured load.
- **SC-003a** *(if US6 is adopted)*: On the re-fly, the shadow frame is logged for 100 % of engaged ticks, and its attitude drift per
  span is reported against the budget and against INAV's.
- **SC-004**: 100 % of changed controller settings have a class, evidence and a post-reboot read-back on record. Article 1's
  receiver model and constellations are recorded.
- **SC-005**: In a bench run with satellite reception blocked, every affected tick is flagged and no flight behaviour changes.

**Re-fly (G2)**

- **SC-006**: The re-fly log carries the fault flags and the added blackbox fields for 100 % of engaged ticks.
- **SC-007**: Engaged spans with a navigation freeze, or a reset larger than 10 m, fall from 11-08's rate (1 of 4 spans) to zero. If
  not, every such fault is flagged and measured and fed into the US6 decision. Either way, the comparison with 11-08 is reported
  with faulted spans excluded.

**M2 (after G2)**

- **SC-008**: Every simulator camera parameter and every camera variation cites a measurement or is listed as provisional, each
  040 variation has a recorded fate, and the remaining model gaps are listed for the hardware side. The M2 result names each
  provisional value.
- **SC-009**: The M2 smoke trains on the updated simulator without crash or schema failure, its log states the perfect-signal mode,
  and every tracker input shows non-trivial investment.
- **SC-010**: The phase 4 report answers "does M2 work at all". It gives tracking occupancy, per-step error, reacquire time and crash
  rate on the 040/041 definitions, states the signal difference from those runs, and is produced without any flight.
- *(phase 5, later)* **SC-011**: Simulated acquisition latency, lock duration and dropout on the scripted geometry set match the
  measured values within the measurement's stated uncertainty.

## Assumptions

- The flight baseline of record is 044-t1 g800 (`a5ed3521ea1e70b7`), flown 11-08 on article 1.
- The existing 11-08 and 10-18 logs carry enough 250 Hz inertial data for the drift analysis. Where they do not, the gap is
  recorded and the added blackbox fields close it for the re-fly.
- A 60 s download target is a reasonable field expectation for a ~300 kB log. The real bound is set once the cause of the slowness
  is measured.
- Re-fly conditions are recorded and the flight is compared as flown, as in 044.
- The camera and receiver measurements from the 031/042 field and bench work are available, or can be made, by phase 2.
- The M2 run reuses the current tracker training regime except where phase 3 changes it.

## Out of scope

The 044 SC-002 pitch fix (backlog); any new disengage reason or change to operations based on estimator faults; flying an M2
policy; companion-hosted inner (rate) loop (backlog: dedicated fast inner loop); replacing INAV's fallback; camera-proxy score
gradient (M2 phase 2 study per README); building camera hardware improvements (042/031 own those; this feature hands them the gap
list).
