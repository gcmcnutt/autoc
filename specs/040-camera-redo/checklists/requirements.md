# Specification Quality Checklist: 040 Camera Redo — Perception Fidelity Refinement for M2

**Purpose**: Validate specification completeness and quality before proceeding to planning
**Created**: 2026-07-28
**Feature**: [spec.md](../spec.md)

## Content Quality

- [x] No implementation details (languages, frameworks, APIs)
- [x] Focused on user value and business needs
- [x] Written for non-technical stakeholders *(see Notes — domain-technical by necessity)*
- [x] All mandatory sections completed

## Requirement Completeness

- [x] No [NEEDS CLARIFICATION] markers remain — **both resolved 2026-07-28 (see Iteration 2)**
- [x] Requirements are testable and unambiguous
- [x] Success criteria are measurable
- [x] Success criteria are technology-agnostic (no implementation details)
- [x] All acceptance scenarios are defined
- [x] Edge cases are identified
- [x] Scope is clearly bounded
- [x] Dependencies and assumptions identified

## Feature Readiness

- [x] All functional requirements have clear acceptance criteria
- [x] User scenarios cover primary flows
- [x] Feature meets measurable outcomes defined in Success Criteria
- [x] No implementation details leak into specification

## Validation Record

### Iteration 1 — 2026-07-28

**Issues found and fixed:**

1. *Implementation leakage* — the first draft named source files, struct fields, config keys, and
   numeric encodings (int8/int16, `BeaconObservation`, `CepGateThreshold`, PRNG slot 5) throughout the
   requirements. All were rewritten as capability statements. Example: "replace the int8 quantization
   with int16 pixel coordinates" became FR-001, "quantise beacon bearing onto a discrete sensor grid of
   defined pixel dimensions". The concrete identifiers remain available to planning via
   [`input-data-checklist.md`](../input-data-checklist.md), which is linked as the input of record.
2. *Success criteria expressed as internal mechanics* — early drafts phrased outcomes as "CEP derives
   from correlation margin". Rewritten as observable outcomes ("signal quality degrades monotonically
   with distance and reproduces the measured bench relationship").
3. *Unbounded clarifications* — five candidate clarification items were identified during scoping. Three
   had defensible defaults and were moved to Assumptions with their basis recorded (exposure duration,
   camera standoff, bench-measurement transfer). Two remain, per the limit.
4. *Priority inversion* — the airframe-fidelity check was initially P3. Promoted to P1: it gates
   everything, because a plant change would require regenerating the M1 source trajectories that every
   other story consumes.

**Result**: all Content Quality and Feature Readiness items pass. Requirement Completeness passes except
for the deliberate clarification markers below.

### Iteration 2 — 2026-07-28 (operator clarification session)

Both clarification markers resolved, and the feature's character sharpened.

1. **Q1 resolved**: keep 120° field of view; design the signal model to ~100 m detection. Stated intent is
   *plumbing first* — structure and knobs now, numeric calibration when real-to-simulation measurement
   exists.
2. **Q2 resolved**: emitter beam width demoted from blocker to calibratable assumption, seeded at the 130°
   half-power figure the existing 031 analysis implements, flagged as a bench-measurement target.
3. **Overview reframed** around plumbing-first, making explicit that unresolvable findings (wide-field
   link-budget shortfall, sensor-format problem, beam-width discrepancy) are recorded as calibration
   targets rather than blockers.
4. **US7 rescoped** from a field-of-view comparison study to preserving the optics findings as a durable
   artefact — the comparison is moot once 120° is decided, but the reasoning behind it outlives the
   decision and would be costly to lose.
5. **Added FR-033 through FR-036** for the calibratability contract: separate detection and
   range-inference envelopes; every physical quantity externally configured; each classified
   measured/derived/assumed; and the central condition that a later calibration is absorbable through
   configuration alone.
6. **Added SC-010 (revised), SC-011, SC-012** to make the plumbing-first contract measurable — including a
   calibration rehearsal that substitutes alternative values for assumed quantities and confirms no
   structural change is needed.
7. **Added Assumptions 11–14** recording the decisions and their reasoning.

**Result**: all checklist items pass.

## Notes

### Resolution of the original clarification markers

Both were explicitly identified during scoping as decisions the operator must make, with an instruction
not to invent values:

- **Q1 — field of view / operational range.** Two independent analyses (photon budget and buildability)
  indicate the inherited 120° field is neither achievable on the assumed sensor nor consistent with the
  100 m range the existing link budget was written for. Guessing would silently fix the feature's most
  consequential parameter, and every signal-quality number depends on it.
- **Q2 — emitter beam width.** The 031 documents contradict each other (130° HPBW vs 150° FWHM). No
  default is defensible when the two sources disagree; the exponent underlies the whole emission model.

Both were resolved in Iteration 2. The plumbing-first reframing is what unblocked them: once the
deliverable is *structure plus knobs* rather than final numbers, a contested physical constant becomes a
configured default with a recorded basis instead of a gate. Neither finding was discarded — both are
carried as calibration targets (FR-035) and preserved in the optics record (US7).

### On "written for non-technical stakeholders"

Marked passing with a caveat. This is a simulator-fidelity feature in a research codebase whose audience
is the operator; the surrounding specs (030–039) are comparably domain-technical. The spec has been
written to avoid *implementation* detail — no file paths, type names, function names, or config keys — but
it necessarily uses domain vocabulary (bearing, field of view, code word, lock, beacon separation). That
is the right register for this project, and matching the sibling specs matters more than a generic
readability standard.

### Numeric values carried into the spec

Measured values (0.772 m separation, the station stack, the 5.5×4 propeller) appear because they are
*requirements*, not implementation choices — FR-004 is meaningless without the number. Each traces to the
input checklist, per SC-010.
