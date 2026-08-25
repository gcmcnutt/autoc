# Specification Quality Checklist: 043 — ACRO dual-loop

**Purpose**: Validate specification completeness and quality before proceeding to planning
**Created**: 2026-08-23
**Feature**: [spec.md](../spec.md)

## Content Quality

- [x] No implementation details (languages, frameworks, APIs)
      *Deviation, deliberate and documented*: this spec cites file paths, config keys and measured
      constants. In this project a control-loop spec that does not name `looptime`, `gyro_lpf_hz` or
      `FlightStateRecord` is not verifiable — the constants ARE the requirement (see § Clarifications,
      "modelling the config RATES is key"). Requirements remain outcome-shaped; the citations are
      evidence, not designs. Consistent with 038/040/041 specs.
- [x] Focused on user value and business needs
- [x] Written for non-technical stakeholders
      *Within project norms* — the "stakeholder" here is the operator/engineer.
- [x] All mandatory sections completed

## Requirement Completeness

- [x] No [NEEDS CLARIFICATION] markers remain
      Resolved with the operator 2026-08-23 (SC-001 → subjective-first, per 041's AC-1 pattern).
- [x] Requirements are testable and unambiguous
- [x] Success criteria are measurable
      SC-001 is deliberately subjective at first — an explicit operator decision, with the 041-t7
      reference spectra and 039's rate-RMS-by-regime table stated so the judgement has a fixed
      comparator. SC-002…SC-011 measurable as written.
- [x] Success criteria are technology-agnostic (no implementation details)
      Stated as flight/sim outcomes (band power, occupancy, determinism), not as code properties.
- [x] All acceptance scenarios are defined
- [x] Edge cases are identified
      8 recorded, including the two permanent-loss hazards (pinned-dmp orphaning, flash budget).
- [x] Scope is clearly bounded
      IN / OUT tables, with the transport deferral argued rather than merely listed.
- [x] Dependencies and assumptions identified
      9 assumptions, 6 risks, 3 build surfaces.

## Feature Readiness

- [x] All functional requirements have clear acceptance criteria
- [x] User scenarios cover primary flows
      6 stories, each independently testable; US1/US2 are P1 and deliver value alone.
- [x] Feature meets measurable outcomes defined in Success Criteria
- [x] No implementation details leak into specification
      Per the deviation noted above.

## Notes

**All checks pass. Ready for `/speckit.plan`** (or `/speckit.clarify` if more decisions are wanted
before planning).

Operator decisions taken 2026-08-23, encoded in the spec:

- **SC-001 bar** → subjective first, quantified only after ACRO shows signal (041 AC-1 pattern).
- **Yaw** → question dissolved: **the airframe has no rudder.** Verified in `xiao/inav-hb1.cfg`
  (`smix` routes ROLL + PITCH to two elevon servos; yaw appears in no rule) and in the FDM servo
  model (aileron + elevator only). FR-018 now records and verifies this rather than deciding it.
- **Bake plan** → **one bake with everything**; attribution on regression is post-hoc ablation
  against 041-t7. Recorded as assumption 10 with its accepted cost.

**Priority vs order** (raised 2026-08-23): the P1/P2/P3 labels are speckit *priority*, not phase, and
under the one-bake decision they read as a misleading schedule — variations (US5) and housekeeping
(US6) both land before the bake (US4). Spec now carries an explicit **§ Execution order** table, and
US5 was raised P3 → P2 to reflect that it is operator-scoped pre-work carrying a format change.
⛔ `/speckit.tasks` MUST take its phase order from that section, not from the priority labels.

**Architectural attribute added 2026-08-23** (operator): inputs and outputs are unchanged in count,
magnitude and polarity — only the outputs' interpretation changes, so a zero command now means **zero
commanded rotation RATE** rather than *neutral surfaces*. The inner loop's I-term therefore **buries the
craft-balance variations**, which resurface through the energy channel and at physical limits. Encoded as
FR-019 (the model MUST have an I-term; a mis-trimmed craft must settle to zero rate), FR-059/FR-059a
(per-axis observability audit + "does the regiment reach the limits"), SC-012, SC-013, R10, and a polarity
verification clause in FR-016. ⛔ This is the attribute to hold on to while modifying crrcsim: a P/PD-only
inner loop would look correct and mis-model the entire variation regime.

### Post-plan reconciliation 2026-08-25

`/speckit.analyze` aborted — no `tasks.md` yet. `/speckit.clarify` **not** run, and the reason is that the
problem was **drift, not underspecification**: research.md and the conversation had run ahead of spec.md
(the inner-loop authority axis existed only in research; the static-margin axis was "candidate" in spec and
"best-motivated" in research; FR-005 was left "droppable" and plan.md never addressed it; plan.md predated
FR-012a / FR-019a-b / FR-052a / SC-001a). Clarify writes generated questions into spec.md — it would not
have touched plan.md drift, and would most likely have targeted the items deliberately routed to
measurement (R2/R6/R10). Reconciliation + three named forks was the right instrument.

**Three forks resolved by the operator:**

| fork | decision |
|---|---|
| `craftGyroScale` vs an inner-loop authority scale (same uncertainty) | ⭐ **gyro scale only** — no inner-loop gain variation. research.md's recommendation **DECLINED** and marked as such in place |
| static-margin / pitch-stability axis | ✅ **ships as FR-052b `craftCmQ`** — ⛔ but split to avoid the double-count: `craftCGDelta` already carries the *static* (mass) side, so the new axis is the *dynamic* (damping) side only. ⭐ Range pre-derived in `hb1_streamer.xml` (−3.6 … −5.0, centre −4.2) |
| FR-005 outbound sequence number | ⛔ **cut with US1** ⇒ ⭐ **043 makes no xiao log-format change at all** |

**Propagated to**: spec.md (session-3 clarifications, FR-052b, section A deleted, SC-006 clause, risk row,
FR-056 resolved), plan.md (phase table + a new "added since first draft" coverage table), data-model.md,
contracts/craft-imu-axes.md, research.md (recommendation marked DECLINED).

⭐ **Same discipline applied twice**: the anti-double-count reasoning that settled the gyro question was
then used unprompted to split `craftCmQ` from `craftCGDelta`.

### Scope changes 2026-08-24 (post-cleanup)

**1. US1 dropped — xiao logging outside autoc spans.** Operator: *"isn't worth dragging all that in
through msplink. No need for now."* Removed: the US1 story, FR-001…FR-004, SC-007, the flash-budget edge
case, the `FlightStateRecord` key entity, and execution-order step 1 (steps renumbered 1–9). US2…US6 keep
their numbers so every downstream reference stays valid; a two-line DROPPED stub marks the gap.
- ⚠️ **Accepted cost, recorded**: the pilot-flown-vs-NN-flown comparison again needs a **blackbox
  clock-join**. That path is proven (041: −970 ppm, cross-validated to 0.5%), so it is known-working, not
  a gap. Re-open trigger stated: a flight where the join fails or is unavailable. Backlog item *"Export RC
  Commands to Xiao Log"* remains filed.
- ⚠️ **FR-005 kept** (retitled section A, "Command-path observability"). It is a locally-generated
  sequence counter in the existing engaged-tick record — it drags nothing new through msplink, and it
  matters *more* under ACRO, where a stale frame is a stale rate setpoint the 2 kHz loop chases. Flagged
  as droppable on the same reasoning if the plan disagrees.

**2. INAV phase-delay parameters moved IN scope (FR-012a, new).** Operator: *"there might be a param or
two to tune in INAV, especially for phase delay impacts."* ⛔ Gains (`fw_*` P/I/D/FF) stay fixed — this is
a separate question, and the reframing is what makes it live: **6.4 ms of gyro PT1 is 11° at 5 Hz but 115°
at 50 Hz.** Negligible against the 81.6 ms outer-loop budget the README computed; a real phase-margin term
for a 2 kHz inner loop. Five candidates tabled (`gyro_main_lpf_hz`, `dterm_lpf_hz`, `acc_lpf_hz`, dynamic
notch Q, `servo_pwm_rate` — the last moved from flat-OUT to gated candidate), none pre-approved, with four
mandatory disciplines: computed phase contribution first; **mirrored in the sim** (changing one side only
widens the gap); bench-verified into the config of record; and *"a param or two"*, because each one costs
attribution in a one-bake feature. New **R11**.

### Cleanup pass 2026-08-24

Run at the operator's request after five rounds of refinement. **Focus: ACRO as INAV defines it — and
therefore implicitly NOT ANGLE.** `/speckit.clarify` deliberately **not** run: zero open markers, and every
remaining unknown (R1–R10) is routed to *measurement* in the plan phase, so clarify would manufacture
premature decisions about exactly what the spec says to measure.

| change | why |
|---|---|
| ⭐ **New § "What ACRO is"** hoisted to the front, before IN scope | the definition is the spine; every consequence (zero command, no self-levelling, no recovery, IMU axes, buried balance error) is now **derived there once** and referenced, instead of re-argued in six places |
| Clarifications **split into two sessions** — operator scoping (from README) / architecture (after reading `pid.c`) | 153 undifferentiated lines; the INAV findings were buried among the scoping answers |
| Drafting retractions **removed from requirement bodies** (FR-052, FR-054, FR-059 row, R1, the IMU entry) | requirements were reading as an argument with an earlier draft. Conclusions kept inline; provenance lives here |
| FR-019 / FR-019a / SC-014 / the envelope edge case **thinned to pointers** | each re-derived the ACRO definition |
| READ-FIRST table | now directs the reader to § What ACRO is first |

**Not done, deliberately**: letter-suffixed FRs (FR-019a/b, FR-052a, FR-059a) left as-is — house style
(041 used FR-017a/FR-022a), and renumbering would break built-up references for no gain.

Net: 950 → 960 lines. The definitional section added length; the de-duplication gave most of it back.
In range for this repo (039=382, 038=601, 040=728, 041=1083).

---

**Correction history** (kept here, out of the requirement bodies):

⛔ **CORRECTION 2026-08-23 — ACRO vs ANGLE conflation, found and fixed.** An earlier draft of this spec
described ACRO as "zero command = attitude hold", treating attitude as a controlled variable. **Wrong, and
the difference is load-bearing** (operator: *"I mistook INAV acro for attitude. Huge difference."*).
Verified in `src/main/flight/pid.c`: INAV always derives a **rate** setpoint from the stick (Step 2), and
only under ANGLE/HORIZON/ANGLEHOLD does `pidLevel()` overwrite it with `angleError × PID_LEVEL.P` — the
source's own *"self-leveling strength"* (Step 3). **ACRO never reads `attitude.raw[]`.** Corrections made:

| where | change |
|---|---|
| § Clarifications | replaced the wrong entry with an ACRO-vs-ANGLE comparison table + why ACRO (ANGLE "only works when zero input means straight and level") |
| FR-019 | acceptance restated on **rate**, not attitude; attitude retention noted as emergent, not the objective |
| **FR-019a (new)** | ⛔ the model MUST NOT implement self-levelling / attitude feedback — building it produces ANGLE mode and a policy trained on a safety net the aircraft lacks. `Cntrl_Omega` is safe by construction |
| **FR-019b (new)** | setpoint acceleration limit — `rate_accel_limit_roll_pitch = 0` (off) on this config; the rate setpoint can step |
| SC-012 | restated on rate; **converse added** — displaced to bank with zero command, the craft must NOT return to level |
| SC-014 | *"recover to patrol"* split: the inner loop gives controllability only; **recovery stays the POLICY's job, unchanged from MANUAL** |
| Edge cases | ⛔ new: beyond the commandable envelope there is **no recovery behaviour — none** |
| R1 | marked partly answered by this finding |

⛔ **CORRECTION 2026-08-23 (second) — IMU alignment was downgraded too far.** The prior entry called the
effect "an OBSERVATION defect, not a control one" and "second-order". **Both wrong.** At ±5° the
cross-axis coupling is `sin 5° ≈ 9%` on every command — first-order — because **INAV's roll and pitch ARE
the IMU's axes**. What survives from the downgrade is narrower and still true: it does not break
zero-rate nulling, since the misalignment rotation is invertible. Net position now recorded as three
distinct mechanisms (nulling: unaffected; commanded axes: ~9% coupling; the policy's self-view and hence
its target geometry: consistently rotated).

**Scoping settled with it:**

- ⭐ Reframed from *"a new mount/IMU variation class"* to **"a few more craft axes"** (operator: *"craft
  variations are many and we add a few more, to be closer to reality"*). **FR-054 reversed** — default is
  now **append to the existing craft class**, which `craft_variation.h` already documents a recipe for; a
  separate sixth PRNG class only if independent toggling is wanted. The earlier draft over-built this.
- **FR-052** now lists the axes with magnitudes: mount attitude ±5° (σ 2.0°), gyro scale ~5% (σ 2%),
  accel scale ~5% (σ 2%), accel bias set from the **measured** 041-t7 1.70σ `accel_y` gap.
- **FR-052a (new)** — ⛔ gross misconfiguration OUT: no upside-down AHRS, no swapped axis, no 90°/180°
  board-alignment error. Configuration faults caught at the bench, not build tolerances; the backlog's
  170°-vs-180° item stays out.
- **Purpose stated in SC-009**: the policy must fly the task from its **error view of its goal** — the
  axes exist so the controller pursues the goal through its own measurement error, not so a report can
  say they exist.

**Refinements 2026-08-23 (operator), after the first pass:**

- **FR-019** — "holds attitude" is bounded to **within physical limits**, and the zero-command sweep
  across all attitudes runs **before autoc is connected**, so a hold failure is attributable to the
  model rather than surfacing later as odd policy behaviour.
- **FR-059a** — ⛔ framing corrected. This is an **underpowered** craft, so aero/control limits are hit
  easily, and the dominant mechanism is **hold-attitude-and-descend**, not commanded-rate saturation. A
  rate controller holds rotation, not trajectory. The buried variation therefore resurfaces through the
  **energy channel** (`SPECIFIC_ENERGY` / `AIRSPEED` — inputs the policy already has), i.e. it is
  transformed, not deleted. The trim→surface-deflection→drag path (`CD_ELsq = 0.05` in the model) is
  filed as a **research item, not a claim**.
- **FR-059 mount/IMU row** — downgraded. Alignment does **not** meaningfully affect attitude hold in
  normal commandable regimes (the misalignment rotation is invertible); it is an **observation** defect
  — biased attitude estimate and accel channels, which is where the measured 1.70σ `accel_y` gap lives.
  The class stays, justified on observation grounds only.
- **FR-014a (new)** — ⭐ CRRCSim **already has** `Cntrl_Omega`, a rate PID with XML-loaded gains, and
  `fdm_larcsim` already calls the controller hook in the correct place (before the servo model). Four
  gaps recorded — missing feed-forward, ~333 Hz vs 2 kHz, different anti-windup, expo. R9 asks
  reuse-or-rewrite and is flagged as the largest single swing in the sim work's size.
- **Fidelity bar set 2026-08-23** — ⭐ *"take a decent stab at emulating INAV and measure and go from
  there."* ⛔ **Not a precision-matching exercise.** Recorded with its three reasons: this was
  deliberately avoided until now ("a lot of knobs and loop rate and phasing errors"); the craft operates
  at its aerodynamic limit, where any FDM is least trustworthy, so divergence there is expected rather
  than a defect; and acceptance is therefore **behavioural** — *"the craft can recover to patrol"*
  (new SC-014). SC-002 was rewritten from "the sim predicts the aircraft" to "structurally faithful, and
  its divergence is MEASURED". What remains required is structural: loop topology, the ratios between
  cascaded rates, and the ordering.
- **Tension surfaced, not smoothed over**: FR-059's observability audit measures buried axes at the
  limits, which is the regime the plant model resolves worst. Its verdicts are labelled **sim verdicts,
  provisional on the flight** — an axis is not permanently retired on a sim-only reading there.
- **Schedule risk relocated**: "measure and go from there" is a loop of unknown length that must converge
  *before* the single bake commits. Added to the risk table, with the stop signal named — tuning
  constants to close an aero-limit gap means stop and record the divergence.
- **Hard sequencing constraint for `/speckit.tasks`**: FR-057 (extract from the pinned 041-t7
  dmps) MUST precede any change to the persisted per-scenario metadata format. Missing it is
  permanent.
