# Feature Specification: 039 Xiao 20 Hz Flight — embedded control-loop catch-up

**Feature Branch**: `039-xiao-20hz-flight` (pre-created 2026-07-10 off merged-038 main `af6318e`; clean
`rebuild-perf.sh` gate GREEN)
**Created**: 2026-06-19 (stub, re-homed from 037) · **Specified**: 2026-07-10
**Status**: Draft
**Input**: User description: "039 Xiao 20 Hz Flight — embedded control-loop catch-up … Outcome: a
flight test flying 038's M1 elite (37-input/2051-weight, t5 rebake) at 20 Hz on the xiao, exhibiting
the control smoothing we see in sim — the primary motivator for 20 Hz. Scope: (1) firmware regen from
029-vintage to the 038 contract; (2) NN forward-pass unroll pulled forward; (3) LATENCY RESEARCH as a
first-class story — examine old flight logs and what crrcsim simulates, then decide a plan of action
(possibly amend the sim latency model and rerun an M1 bake; also decides the deferred local IMU);
(4) COMPRESSION RESEARCH as a first-class story — packed log format AND open research on compression
techniques to lower write bandwidth to flash; (5) the flight test itself with sim-vs-real smoothing
comparison as the acceptance read. Primary research (latency + compression) structured to run during
the planning phase."

## Overview

038 closed with the best-validated M1 controller to date (t5 rebake elite: enriched 37-input
situational-awareness contract, ≈ parity with the best-ever 037 t10, novel-geometry generalization
proven at wrap). The xiao still flies **029-vintage firmware** (last flight 2026-05-17) — two input-
schema generations behind, at 10 Hz. 039 is the bridge: **fly 038's M1 at 20 Hz on real hardware and
see the sim's control smoothing in the real flight logs** — that smoothing is the primary motivator
for 20 Hz (037's finding). Two open questions are promoted to first-class research, run during the
planning phase: **latency** (is the sim's compute-latency + servo-response model honest enough, or
does M1 need a retrain on an amended model?) and **log compression** (20 Hz ≈ 2× log volume against a
fixed flash budget and write-bandwidth ceiling).

## Clarifications

### Session 2026-07-10

- Q: How should the baked arena be placed for real flight (defines what the new dist-to-boundary /
  inward-vector NN inputs read)? → A: Centered at the span-engage point (re-centered each engage),
  training-size geometry — mirrors training entry semantics, where the craft is at the arena center
  at scenario start.
- Q: What makes the flight's smoothing read a PASS for SC-005? → A: Per-axis band vs sim — flight
  per-axis dCtrl ⟨|Δu|⟩ and amplitude ⟨|out|⟩ within ±25% of the same controller's sim values, each
  axis, plus qualitatively less bang-bang than the 2026-05-17 (10 Hz-era) flight.
- Q: What parity tolerance gates the bench comparison between xiao and desktop NN evaluation? → A:
  No numeric replay-parity harness — bench verification is observational: the firmware runs on the
  stationary bench, a span engages, and the recorded logs show the generated path moving around the
  craft with the NN inputs/outputs evolving sensibly (no NaN / lockup / implausible saturation).
- Q: What measured-vs-modeled latency gap triggers amending the sim model + an M1 retrain? → A: No
  pre-set rule — the operator decides at the research review with the measured numbers in hand; the
  research deliverable is the numbers + a recommendation, not an automatic trigger.
- Q: What flight duration must a full 20 Hz log capture within the flash budget? → A: ~2 flights of
  3–4 minutes each between flash clears (packs run short — the model drives the throttle hard), and
  high-bandwidth logging runs only during autoc engagement (spans), not the whole flight.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Firmware catch-up to the 038 controller contract (Priority: P1)

The operator regenerates the xiao NN program from the pinned 038 M1 elite (37-input/2051-weight),
brings the firmware's input gathering up to the 038 contract (including the new situational-awareness
inputs), and pulls the NN forward-pass unroll forward from the 50 Hz stretch so evaluation cost is a
non-issue at 20 Hz. Bench verification proves the embedded evaluation matches the desktop reference
before anything flies.

**Why this priority**: Nothing else in 039 can happen without it — the flight candidate cannot run on
029-vintage firmware, and every downstream measurement (latency bench, tick budget, flight) uses this
firmware.

**Independent Test**: On the stationary bench (no flight), the firmware runs a full engaged span and
the recorded logs show the generated path moving around the craft with NN inputs/outputs evolving
sensibly; per-tick evaluation time is measured and within budget.

**Acceptance Scenarios**:

1. **Given** the pinned 038 M1 weight file, **When** the firmware NN program is regenerated and built,
   **Then** the xiao build compiles with the 038 input contract (situational-awareness inputs
   included) and boots with the correct topology reported.
2. **Given** a stationary bench with the firmware live, **When** a span is engaged and run to
   completion, **Then** the recorded logs show the path moving around the craft and all 37 inputs +
   3 outputs evolving plausibly — correct ranges, no NaN, no lockup, arena inputs reading
   center-of-arena values at engage (per the re-centering clarification).
3. **Given** the unrolled forward pass, **When** per-tick cost is measured on target, **Then** the
   gather + evaluate + output path fits the 20 Hz tick with margin, and the measured cost is recorded
   for the latency model.

---

### User Story 2 - Latency ground truth → plan of action (Priority: P1)

The operator determines what the real control pipeline latency actually is and whether the simulator's
latency + servo-response model is honest enough for the trained M1 to transfer. The research examines
(a) existing/old flight logs for measured pipeline timing, (b) what crrcsim currently simulates
(compute latency and the 037 servo v2 response model), and (c) fresh bench measurements from the
regenerated 20 Hz firmware. The deliverable is a **decision**: either the sim model stands (fly the
existing elite), or the model is amended and an M1 retrain is run on the updated model before the
flight. The same numbers decide whether the deferred high-bandwidth local IMU is needed at all.

**Why this priority**: This is the critical deferred ask. Flying a controller trained against a wrong
latency model can mask or fake the smoothing result the whole feature exists to demonstrate — and a
retrain is the longest-lead item in 039, so the decision must come early (planning-phase research).

**Independent Test**: The decision memo exists with numbers: measured real pipeline latency
(component-by-component), the sim's current modeled values, the delta, and the go/no-go on amending +
retraining and on the local IMU. Testable without any flight.

**Acceptance Scenarios**:

1. **Given** old flight logs with pipeline timing evidence, **When** analyzed, **Then** the real
   sensor→command latency is quantified with its components (state fetch, evaluation, command send,
   actuation), including the tail behavior, at the 20 Hz cadence.
2. **Given** the sim's current latency + servo-response parameters, **When** compared against the
   measured values, **Then** the gap is stated numerically and a documented decision follows: model
   stands / model amended.
3. **Given** the decision is "amended", **When** the sim model is updated, **Then** an M1 bake on the
   updated model is planned and run, and its elite passes the same parity gates as the t5 elite before
   becoming the flight candidate.
4. **Given** the measured latency budget, **When** evaluated against control-loop needs, **Then** the
   local-IMU question is answered on the record (stays deferred / becomes necessary).

---

### User Story 3 - Flight logging that sustains 20 Hz (compression research + new format) (Priority: P2)

The operator can log full flights at 20 Hz within the existing flash budget and write-bandwidth
ceiling. Open research (planning phase) evaluates compression techniques and alternative log formats —
compact/differential encodings and/or lightweight general-purpose compression — against real recorded
log content, then the chosen format is implemented with a single authoritative writer/reader pair so
ground tooling decodes it. Field retrieval stays over BLE (no cable in the field).

**Why this priority**: 20 Hz roughly doubles log volume; the current verbose text format was already
identified (023) as the real 20 Hz blocker — not NN compute. Without this, the flight in US5 cannot
capture the full-flight evidence the smoothing comparison needs.

**Independent Test**: Bench-log a sustained 20 Hz session; verify no control-tick interference from
the write path, full-session capture within the flash budget, and lossless ground-side decode of the
downloaded log.

**Acceptance Scenarios**:

1. **Given** representative recorded log content, **When** candidate encodings are evaluated, **Then**
   measured compression ratio and write-bandwidth per candidate are documented and a format is chosen
   on evidence.
2. **Given** the implemented format, **When** logging a sustained 20 Hz bench session, **Then** the
   write path (including metadata maintenance) never delays a control tick, and the flash budget
   holds for two flights' engaged spans (2 × 3–4 min) without an intermediate clear.
3. **Given** a completed session, **When** downloaded over BLE and decoded on the ground, **Then** the
   decoded stream is complete and field-for-field faithful, and the sim-to-real analysis tooling can
   consume it.
4. **Given** the pre-flight flash-clear flow (ground-triggered erase), **When** preparing for a
   flight, **Then** the existing clear/initialize behavior is preserved (logging remains disabled
   until initialized).

---

### User Story 4 - The 20 Hz control loop on real hardware (Priority: P2)

The operator enables the full 20 Hz control tick on the xiao–INAV system: the NN evaluates every tick
(no divisor), the link budget supports state-read + command-write at 20 Hz, and the loop holds its
cadence under real conditions (bench, then flight-shaped bench with logging active).

**Why this priority**: This is the cadence the controller was trained for; without it US5's smoothing
read is confounded. It depends on US1 (firmware) and is informed by US2 (latency numbers) and US3
(logging active during the loop).

**Independent Test**: Bench session with the full stack active (state fetch, NN eval, command write,
logging): measured tick cadence holds 20 Hz with bounded jitter and zero missed/overrun ticks over an
engagement-length session (several consecutive 3–4 min spans).

**Acceptance Scenarios**:

1. **Given** the regenerated firmware, **When** the NN evaluation divisor is set to every-tick and the
   link rate is provisioned for 20 Hz, **Then** a flight-length bench run shows 20 Hz cadence held
   with bounded jitter and no overruns.
2. **Given** the 20 Hz loop with logging active, **When** running flight-shaped scenarios on the
   bench, **Then** control-tick timing is unaffected by log writes (verified from the timing log
   itself).

---

### User Story 5 - The M1 flight test: sim smoothing, for real (Priority: P1 — capstone, gated on US1–US4)

The operator flies the 038 M1 (or its retrained successor if US2 amends the model) at 20 Hz, downloads
the logs, and runs the sim-vs-real comparison. The acceptance read is **control smoothing**: the
per-axis control character seen in the sim (the 037/038 aggressiveness envelope) shows up in the real
flight — the primary motivator for 20 Hz.

**Why this priority**: This is the 039 outcome; everything else exists to enable it. It is last only
by dependency, not by value.

**Independent Test**: A completed flight with full 20 Hz logs, plus a produced sim-vs-real per-axis
comparison report against the same controller's sim baseline.

**Acceptance Scenarios**:

1. **Given** the flight candidate and the pre-flight checklist, **When** the flight is flown across
   the standard test paths (including the OOD random-intercept path), **Then** the full flight is
   captured at 20 Hz and retrieved in the field over BLE.
2. **Given** the flight logs and the sim baseline for the same controller, **When** the per-axis
   comparison is produced, **Then** real per-axis control activity (step-to-step change and amplitude
   character) is consistent with the sim envelope, and qualitatively smoother than the 10 Hz-era
   flights.
3. **Given** the flight evidence, **When** the 039 outcome is judged, **Then** a documented verdict
   states whether 20 Hz + the 038 controller exhibits sim-grade smoothing in reality, with the resid-
   ual gaps named (feeding the next feature).

---

### Edge Cases

- **Evaluation overrun at 20 Hz**: if a tick's gather+eval+send exceeds the tick under fault
  conditions, the system must degrade predictably (skip/coalesce, never emit stale-mislabeled
  commands) and the event must be visible in the log.
- **Arena inputs in real flight**: the 038 contract includes arena-relative inputs
  (distance-to-boundary, inward vector) that had no 029 equivalent. Real flight has no cylinder —
  per clarification, the training-size arena is **re-centered at the span-engage point on each
  engage** (mirroring training, where the craft starts at the arena center). The engage-time
  re-centering must be verified on the bench (inputs read center-of-arena values at engage) and the
  recorded arena origin must land in the log for post-flight analysis.
- **Recurrent warm-up at engage**: hidden state resets on span activation; the first ticks after
  engage are warm-up. The bench span (US1) must include the engage/reset transient in its logs, and
  the flight procedure should expect it at each engage.
- **Latency research contradicts the sim badly**: if the measured latency invalidates the current
  elite, the retrain becomes 039's long pole — the plan must sequence the flight after the retrain
  rather than flying a known-mistrained controller (operator decision point).
- **Compression underdelivers**: if no candidate format sustains full-rate dual-content logging within
  the write budget, the log must rate-tier (full-rate control-critical fields, decimated telemetry)
  rather than silently dropping frames.
- **BLE retrieval of larger logs**: 20 Hz logs are ~2× larger; field download time over BLE must stay
  practical for between-flight turnaround (compression helps here too — download is of compressed
  content).

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The embedded NN program MUST be regenerated from the pinned 038 M1 elite weight file
  (37-input/2051-weight contract) with input gathering semantically identical to the desktop
  reference, including the situational-awareness inputs with training-size arena geometry
  **re-centered at the span-engage point on each engage** (arena origin recorded in the flight log).
- **FR-002**: Embedded evaluation MUST be verified on the stationary bench before flight: a full
  engaged span whose recorded logs show the generated path moving around the craft with all NN
  inputs/outputs evolving plausibly (correct ranges, no NaN/lockup, arena inputs reading
  center-of-arena at engage) — observational verification, not a numeric replay harness.
- **FR-003**: The NN forward pass MUST be restructured (unrolled) so that measured per-tick evaluation
  cost on target leaves positive margin in the 20 Hz tick alongside gather, command send, and logging.
- **FR-004**: The latency research MUST quantify real pipeline latency (components and tail) from
  existing flight logs plus fresh bench measurements on the regenerated firmware, and state the sim's
  currently modeled equivalents side-by-side.
- **FR-005**: A documented latency decision MUST result: sim model stands, or sim model is amended.
  There is no pre-set numeric trigger — the research delivers the measured-vs-modeled numbers plus a
  recommendation, and the operator decides at the research review. If amended, an M1 retrain on the
  amended model MUST be run and its elite passes the same verification gates (FR-002) to become the
  flight candidate.
- **FR-006**: The latency research MUST explicitly answer whether the deferred high-bandwidth local
  IMU is required for the 20 Hz loop, on the record.
- **FR-007**: The compression research MUST evaluate candidate log encodings (compact/differential
  and/or lightweight general-purpose compression) against real recorded log content, with measured
  ratio and write-bandwidth per candidate, and select a format on that evidence.
- **FR-008**: The selected log format MUST sustain full-rate 20 Hz logging during autoc-engaged
  spans (high-bandwidth logging is engagement-scoped, not whole-flight) and fit **two flights of
  3–4 minutes each** in flash between ground clears, with a write path (including metadata
  maintenance) that never delays a control tick.
- **FR-009**: The log format MUST have a single authoritative writer/reader pair; ground tooling
  (including the sim-to-real analysis flow) MUST decode downloaded logs losslessly.
- **FR-010**: Field log retrieval MUST remain over BLE; the existing ground-triggered flash
  clear/initialize flow MUST be preserved.
- **FR-011**: The control loop MUST run the NN every tick at 20 Hz (no evaluation divisor) with the
  link provisioned for per-tick state read + command write; cadence and jitter MUST be measured and
  bounded over a bench session covering several consecutive engagement-length (3–4 min) spans.
- **FR-012**: The flight test MUST capture full 20 Hz logs sufficient to produce a per-axis
  sim-vs-real control-character comparison against the same controller's sim baseline, and that
  comparison MUST be produced as the feature's acceptance read.
- **FR-013**: Pre-flight verification MUST confirm the safety envelope is unchanged (arming/mode-flip
  semantics, failsafe behavior) with the new firmware before the flight.

### Key Entities

- **Flight candidate**: the pinned 038 M1 elite weight set (or its retrained successor per FR-005);
  identified by training run + generation; verified by bench parity before flash.
- **Latency decision memo**: measured real latency components + sim-modeled values + the
  stands/amended decision + the local-IMU verdict; the planning-phase research deliverable.
- **Flight log (new format)**: the compressed/packed per-tick record (control inputs/outputs, state,
  timing marks) with writer/reader pair; the evidence base for the smoothing comparison.
- **Sim-vs-real comparison report**: per-axis control-character comparison (sim baseline vs flight)
  for the flown controller; the 039 acceptance artifact.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The xiao runs the 038-contract M1 through a full engaged span on the stationary bench,
  with recorded logs showing the path moving around the craft and all inputs/outputs plausible
  (ranges, no NaN/lockup, engage-time arena re-centering visible).
- **SC-002**: Engagement-length sessions (bench, then the real flights' engaged spans) hold 20 Hz
  control cadence with zero logging-induced tick overruns.
- **SC-003**: Two 3–4 minute flights' engaged spans at 20 Hz are captured end-to-end within the
  flash budget (no ground clear between them) and retrieved in the field over BLE with lossless
  decode — at a stored size at least 2× smaller per tick than the current text format (so 20 Hz
  fits where 10 Hz text did).
- **SC-004**: The latency decision memo exists with quantified real-vs-sim numbers and explicit
  verdicts on (a) amend+retrain and (b) the local IMU — delivered during the planning phase, before
  implementation locks the flight candidate.
- **SC-005**: The flown flight's per-axis control activity matches the same controller's sim values
  within a ±25% band, per axis, on both step-to-step change (dCtrl ⟨|Δu|⟩) and amplitude (⟨|out|⟩) —
  and is qualitatively less bang-bang than the 2026-05-17 (10 Hz-era) baseline flight.
- **SC-006**: A documented 039 verdict states whether sim-grade smoothing was exhibited in real
  flight, with residual gaps named.

## Assumptions

- The flight candidate defaults to the 038 t5 rebake elite; it is replaced only by the FR-005 retrain
  path if the latency research amends the sim model.
- 20 Hz (50 ms tick) is the operating configuration (037 decision); 50 Hz remains a gated stretch
  goal outside this feature's success criteria.
- The arena-relative inputs use the training-matching arena geometry (R=80 m, 5–100 m AGL),
  re-centered at the span-engage point on each engage (per clarification; verified on the bench and
  logged per the edge case above).
- The slaved high-bandwidth local IMU is **deferred by default** (operator direction 2026-07-10);
  only the FR-006 verdict can pull it back in.
- BLE is the only field retrieval path (no cable in the field); USB download stays on the backlog.
- Existing flash pre-erase behavior is the baseline: the data region is erased only by the
  ground-triggered clear command, and logging is disabled until initialized (verified in code
  2026-07-10) — the compression work targets density and write-path tail, not erase avoidance.
- The primary research items (latency: US2; compression candidates: US3 scenario 1) are executed
  during the planning phase (`/speckit.plan` research), so implementation starts with both decisions
  made.

## Out of Scope

- The M2 / tracker controller work (038 closed; 040 owns M2 depth).
- Camera/beacon hardware + optics (031/040).
- 50 Hz on-target operation (gated stretch; only the eval-cost derisk (unroll) is pulled forward).
- USB log download (BLE stays for field use).
- The slaved high-bandwidth local IMU (unless FR-006 concludes it is required).
- Any controller-cadence / fitness-function changes beyond the FR-005 latency-amended M1 rebake.
