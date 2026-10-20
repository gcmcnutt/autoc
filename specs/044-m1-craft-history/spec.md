# Feature Specification: 044 — M1 craft-sensor history + roll plant

**Feature Branch**: `044-m1-craft-history`
**Created**: 2026-10-19
**Status**: Draft
**Input**: Operator scoping at the 043 wrap (2026-10-18) and the requirements draft in [README.md](README.md) — read it for *why*
and for the evidence and code scan behind each requirement; this document is *what*, and governs where they differ.

---

## READ-FIRST — document order

| document | role |
|---|---|
| **spec.md** (this) | **governs.** Scope, requirements, acceptance. |
| [README.md](README.md) | derivation of record: the evidence for each choice, the 2026-10-18 code scan of the input layout (39 files), operator answers. |
| [`../043-acro-dual-loop/outcome.md`](../043-acro-dual-loop/outcome.md) | what 043 established and the baseline this feature must beat. |
| [`../043-acro-dual-loop/flight-analysis-20261018.md`](../043-acro-dual-loop/flight-analysis-20261018.md) | the reference flight (t6 gen 752) and the airframe identification behind the roll requirement. |
| [`../043-acro-dual-loop/policy-temporal-view.md`](../043-acro-dual-loop/policy-temporal-view.md) | why craft-sensor history, which signals, which lags (§§ 4–8). |

---

## Why this feature exists

The 043 exit flight (2026-10-18, t6 gen 752, full streamer) halved the 2 Hz pitch cycle once the simulator's pitch airframe was
honest, and confirmed the measured pitch inertia within ~10 %. It left two things:

1. The remaining pitch motion is **3–5 Hz command dither** — 3–5 Hz pitch-rate RMS rose 29 → 56 °/s while the 2–3 Hz cycle fell
   103 → 51 °/s.
2. **Roll oscillation rose** — 3–5 Hz roll-rate RMS 58 → 97 °/s — because the simulator's roll mode responds ~4.5× more slowly than
   the aircraft's, so training never paid for roll dither (the same mechanism that hid the pitch cycle before 043 fixed it).

And the policy has **no history of any of its own sensors**: it remembers its previous command, not what the airframe did, so the
damping term is largely out of its reach. 044 gives the policy that history, makes the roll airframe partly honest, and proves the
result in one flight. It is deliberately **lean — one bake, one flight** — so M2 ([045](../045-m2-tracking/README.md)) can start on the
updated M1 stack; M2 is the ultimate goal and inherits everything here.

### Decisions already made (not reopened)

- **INAV remains the bridge** through 044 and 045: failsafe, attitude/position estimation, pilot modes, logging and the 2 kHz rate
  loop stay on the flight controller. Embedding the IMU / GPS / fast loop on the companion computer is a later hardware step.
- **Objective, selection and training regime are unchanged from t6**, so the 044 flight compares cleanly with the t6 flight.
- **The flight-controller configuration is frozen** at the 2026-10-18 configuration of record.

---

## User Scenarios & Testing *(mandatory)*

The actor throughout is the **operator** (designs, bakes, flies, and judges the controller). The flown aircraft is article 1.

### User Story 1 — The policy sees its own recent rotation (Priority: P1)

The operator bakes a controller whose inputs include the aircraft's own recent body rates and normal acceleration, so it can learn
to damp the airframe rather than dither against it.

**Why this priority**: it is the feature's main lever against the remaining 3–5 Hz pitch and roll dither, and it changes the input
layout that M2 inherits.

**Independent Test**: a smoke run with the history inputs against a smoke run without them, on the same plant, reaches the same
pitch / roll smoothness faster (the falsifier: if not, the input change failed).

**Acceptance Scenarios**:

1. **Given** a scenario start or an engage, **When** the first decision tick runs, **Then** every history slot holds the current
   reading (no zeros, no stale values from a previous span).
2. **Given** identical aircraft state in the simulator and on the companion computer, **When** both compute the inputs, **Then** the
   two input vectors are identical, history included.
3. **Given** a flight log recorded with the new layout, **When** an old decoder reads it, **Then** it refuses the file with a version
   error rather than mis-reading columns.

---

### User Story 2 — The roll airframe is partly honest (Priority: P1)

The operator trains on a plant whose roll response sits between the old simulator and the measured aircraft, and whose pitch damping
covers the towed streamer, so roll dither costs something in training.

**Why this priority**: without it, a history-equipped policy can still learn roll dither the simulator filters out — the 043 lesson.

**Independent Test**: the flown t6 gen-752 genome, evaluated on the new plant, shows more 3–5 Hz roll than on the t6 plant — moving
toward what it did in flight — before any bake is started.

**Acceptance Scenarios**:

1. **Given** the new roll class at its centre, **When** a roll step is applied, **Then** the roll response corner sits near 3 Hz at
   15 m/s and the steady roll rate per aileron equals the t6 plant's.
2. **Given** the pitch damping class, **When** sampled across the training regiment, **Then** it spans the bare aircraft through the
   streamer-on value measured in flight.

---

### User Story 3 — One controller, chosen fairly, flown and judged (Priority: P1)

The operator runs one full bake, picks the flight candidate by comparing finalists in one identical, full-strength environment, flies
it with the 043 procedure, and judges it against the 2026-10-18 flight.

**Why this priority**: the feature exists to produce a flight-proven improvement; everything else serves this.

**Independent Test**: the flight is analyzed with the same scripts as the reference flight and each success criterion is reported as
met / not met with its number.

**Acceptance Scenarios**:

1. **Given** the finished bake, **When** finalists are evaluated, **Then** every finalist flies the same never-trained scenarios at
   full variation strength, and the pick is made on smoothness at matched tracking, not on training fitness.
2. **Given** the flight, **When** it is analyzed, **Then** streamer state per span, wind, and the frozen flight-controller
   configuration are recorded with the results.

---

### User Story 4 — Training runs are trustworthy and affordable (Priority: P2)

Before the bake, the operator removes known defects that would otherwise be frozen into the new baselines, and finds where training
time has gone.

**Why this priority**: 044 starts new baselines anyway; defects fixed later would force another reset. Compute has crept up (310 →
~560 s per generation) and the bake is 800 generations.

**Independent Test**: each fix has its own unit or determinism check; the profile is recorded and any adopted speed-up leaves results
bit-for-bit identical.

**Acceptance Scenarios**:

1. **Given** any scenario, **When** its variation streams are drawn, **Then** no variation class reuses another class's random draws.
2. **Given** a training log, **When** read, **Then** each scenario's wind speed, turbulence, gust length and thermal strength are
   listed alongside its direction.
3. **Given** two genomes from different generations, **When** evaluated, **Then** the operator can set one variation strength for both
   without editing their weights files.

---

### Edge Cases

- **Warm-up**: the first ticks after an engage or a companion-computer reboot mid-flight — history must be pre-filled from the current
  reading, never left at zero (a zero gyro history reads as a violent rate step).
- **Departed flights**: a tumbling scenario produces body rates far outside the training range; analyses that fit models to flight or
  simulator data exclude them, as 043's tools do.
- **Old artifacts**: 45-input genomes and data cannot be evaluated by the new build; they must stay readable from an archive or a tagged
  build.
- **Roll class extremes**: at the fast end of the class, scaling aileron authority with roll damping must not leave the inner loop
  saturating; the step test covers both ends.
- **Variation off**: with variations disabled (quiet tier), the new independent streams must still give a fixed, reproducible scenario.
- **Streamer cut mid-flight**: a span after a cut is analyzed separately (bare vs streamer-on).

---

## Requirements *(mandatory)*

### Functional Requirements — inputs

- **FR-001 Craft-sensor history.** The policy's inputs MUST include body pitch, roll and yaw rate and body normal acceleration at
  **100 ms and 50 ms** ago, alongside their current values (8 added inputs; M1 45 → 53, M2 66 → 74). The set of signals and the set of
  lags MUST each be defined in one place, so changing either is a single edit plus a format break. Longitudinal and lateral
  acceleration get **no** history (they alias heavily in flight and predict nothing); no explicit command history (the recurrent state
  already holds the previous command).
- **FR-002 One recording point, both sides.** History MUST be recorded once per decision tick, at the point where the target history
  is recorded, by the same logic in the simulator and on the companion computer, from the same values the inputs use; at scenario
  start and at engage it MUST be pre-filled with the current reading.
- **FR-003 Adjacent layout, shared with M2.** Each sensor's history MUST sit next to its current value, oldest first (as the target
  history already does), in the craft block shared by the M1 and M2 input vectors. M2 keeps the same craft history; the block is
  designed now so 045 needs no second break of it.
- **FR-004 Complete format break.** Every consumer of the input layout MUST move in one change: simulator, companion firmware, flight
  log (with a format-version bump so old decoders refuse new logs), training and dump formats, the three hand-maintained decoders /
  name lists, the tests, and 043's analysis scripts (ported to read by name). Historical scripts of earlier features stay unchanged.
- **FR-005 Preserve the past before breaking it.** Before the break, the last 45-input build MUST be tagged and the data and evaluation
  results of the flown genomes (t5 gen 488, t6 gen 752) and the t6 final elite archived.

### Functional Requirements — plant

- **FR-006 Roll mode, partway to the aircraft, as a variation class.** The roll damping MUST become a per-scenario variation class
  centred on a roll-response corner of **~3 Hz** at 15 m/s (operator: *"part way to 5 Hz"*), spanning ~1.5–5 Hz (the aircraft measured
  ~5 Hz; the t6 simulator 1.1 Hz). Aileron authority MUST scale with roll damping so the steady roll rate per aileron is unchanged.
  A roll honesty check (step response; closed-loop roll response against the 2026-10-18 flight; gen 752 showing more 3–5 Hz roll on
  the new plant) MUST pass before any bake.
- **FR-007 Pitch damping spans the streamer.** The pitch damping class MUST cover the bare aircraft through the streamer-on damping
  measured in the 2026-10-18 flight (about 3.5× the bare value), weighted toward how the aircraft is flown.
- **FR-008 Flight-controller sensor delays.** The simulator MUST reproduce the flight controller's gyro and accelerometer filter delays
  (~6 ms and ~21 ms) on the signals the policy reads.

### Functional Requirements — baseline-boundary fixes

- **FR-009 Independent variation streams.** Each variation class (wind, rabbit, entry, craft, camera) MUST draw from a stream that does
  not reproduce any other class's draws; a test MUST check this.
- **FR-010 Wind draws logged.** The training log's scenario table MUST list each scenario's wind speed, turbulence, gust length and
  thermal strength, as it lists direction today.
- **FR-011 Explicit evaluation strength.** Evaluation MUST accept an operator-set variation strength; by default it keeps today's
  behavior (the strength stored with the genome).

### Functional Requirements — process

- **FR-012 Attribution smokes.** Two short training runs on the new plant MUST precede the bake: plant only (45 inputs) and plant plus
  history (53). They decide whether the history inputs earn their place (User Story 1's falsifier).
- **FR-013 One bake.** One full **800-generation** bake on the t6 regiment with the new plant and inputs; its run record (manifest) is
  written before launch, and the run report gains roll smoothness panels beside pitch.
- **FR-014 Fair pick, frozen procedure.** The flight candidate MUST be chosen in one identical full-strength environment (never-trained
  scenarios), then flown with the 043 procedure: frozen flight-controller configuration, servo and attitude logging, streamer state per
  span, level and still boot, logs erased beforehand.
- **FR-015 Profile before the bake.** The per-tick cost increase between 043-t2 and 043-t3 (+47 % at equal flight length) and the
  coordinating process's overhead MUST be measured and recorded before the bake; only speed-ups that leave training results
  bit-for-bit identical are adopted in 044.

### Key Entities

- **Craft history window**: for each of four craft signals, the readings at 100 ms, 50 ms and now; filled at engage; recorded per tick.
- **Craft input block**: the 28 craft inputs shared by M1 and M2, ordered so each signal's history sits beside its current value.
- **Roll damping class** (new) and **pitch damping class** (re-centred): per-scenario plant variations, each with a centre, a range
  and a documented measured basis.
- **Variation streams**: one independent random stream per variation class per scenario.
- **Reference flight**: the 2026-10-18 t6 gen-752 flight (two flights, seven engaged spans, full streamer) and its measured values.

---

## Success Criteria *(mandatory)*

Measured on the 044 flight against the 2026-10-18 reference flight, same aircraft, streamer configuration and flight-controller
configuration, with the reference flight's analysis scripts.

### Measurable Outcomes

- **SC-001**: 3–5 Hz roll-rate RMS **below 97 °/s**.
- **SC-002**: 3–5 Hz pitch-rate RMS **below 56 °/s**, and 2–3 Hz pitch-rate RMS **no higher than 51 °/s**.
- **SC-003**: Tracking does not regress — median distance to the target within the reference flight's 9–14 m — and no new crash or
  failsafe mode appears.
- **SC-004**: The roll response corner identified from the flight lies **inside the roll damping class range** (the plant change is
  honest in the air).
- **SC-005**: Oscillation-band normal load (1.5–5 Hz RMS) **below 1.88 g**. Total load p99 (reference 7.0 g) is reported but not
  gated — 043 showed it measures tracking turns more than oscillation.
- **SC-006**: In simulation, before the flight, the history-equipped smoke reaches the plant-only smoke's best smoothness in fewer
  generations (the attribution result, recorded either way).

## Assumptions

- Article 1 flies, with the full streamer, at ~15 m/s median airspeed, in calm-to-moderate wind like the reference flight.
- The roll inertia (0.0069 kg·m²) stays provisional; the roll requirement fixes the measured *ratio* of roll damping to inertia, not
  the inertia.
- The t6 training regiment (population, 294 scenarios, wind envelope, variation ramp) is reused unchanged.
- The flight-controller configuration of record does not change; any change would also have to be made in the simulator first.

## Out of scope

M2 tracking and the camera work (→ 045); any INAV change; objective or selection changes; article 2 (flown only if ready with the same
frozen gains); replacing INAV with companion-hosted estimation or a fast loop.
