# Feature Specification: Energy as a Lexicase Secondary Objective

**Feature Branch**: `035-energy-lexicase-objective` (to be created when this feature is picked up)
**Created**: 2026-05-29
**Status**: Draft (split out of 034 on 2026-05-29 — operator routing)
**Input**: Split from `034` US4. 034 ("M1 cleanup + craft variations") delivers a clean, honest M1/M2 codebase with airframe-parameter diversity, up to and including a flight test. 035 then takes up the substantive multi-objective question 034 deliberately left alone: **does energy work as a real lexicase secondary objective?**

## Context

033 wrapped with the finding that a scalar-aggregated multi-objective collapses the controller into one Pareto corner (`project_scalar_multiobjective_collapse`). The intended secondary objective was always **energy minimization** (lower throttle / better energy management), and energy is already plumbed: `ScenarioScore::energy_score` is computed + emitted, and its lexicase test-case pool entry is one commented line at `src/eval/selection.cc:69` (with the sibling `stability_score` axis at `:68`).

034 was originally going to carry this, but the operator split it out so 034 can ship variations + cleanup + a flight test without entangling them with the open multi-objective investigation. 035 is **genuine investigation, not a one-line uncomment** — energy-as-lexicase-axis underperformed in 027/028, so the deliverable is a *determination* (it works, with evidence / it collapses tracking, with the failure mode / it's unmoved), not a guaranteed win.

035 also inherits the **MAD-relative lexicase epsilon** change (`project_lexicase_mad_epsilon`): constant ε=0.5 doesn't scale with per-scenario magnitude, and both the energy axis and 034's craft-variation dimensions widen that magnitude spread, so epsilon semantics are part of making any lexicase multi-axis result honest.

**Dependency:** 035 builds on the 034-delivered baseline (minisim gone, smoothness gone, craft variations in, tech-debt fold-ins done). The energy bake is compared against the 033/034 tracking-only baseline.

## User Scenarios & Testing *(mandatory)*

### User Story 1 — Determine whether energy works as a lexicase secondary objective (Priority: P1)

The project wants controllers that minimize energy in addition to tracking. This story re-enables energy as a lexicase test-case axis (not a scalar penalty), runs bakes, and determines whether the GA produces a controller that is materially more energy-efficient *without* collapsing tracking — and if it doesn't, documents why.

**Why this priority:** This is the actual goal the 033 smoothness detour was a wrong turn toward. Side benefit: an energy axis directly penalizes the throttle-pegged dead-neuron stuck basin, potentially lowering the intrinsic basin-lottery fail rate (the cheap first attack from the basin-landscape backlog entry).

**Independent Test:** Run an energy-lexicase bake to convergence and compare against the tracking-only baseline (pop=8000/wind=36, plus 034's craft variations) on two axes: (a) tracking quality (per-scenario score, avgMaxStreak) must not materially regress; (b) energy_score must materially improve. A clear yes (both hold) or a clear no (with documented failure mode) both satisfy the story.

**Acceptance Scenarios:**

1. **Given** the energy lexicase axis re-enabled and axes grouped physically (bank = pitch+roll, throttle/energy separate), **When** a bake runs to convergence, **Then** the outcome is classified as energy-improved-without-tracking-collapse, tracking-collapsed, or energy-unmoved — with per-scenario evidence in an outcome doc.
2. **Given** the 4 previously-`DISABLED_` Selection027 multi-objective tests, **When** the energy axis is active, **Then** those tests are re-enabled and pass.
3. **Given** lexicase active with craft variations widening the per-scenario magnitude spread, **When** epsilon is applied, **Then** it is MAD-relative (not constant 0.5), with the constant-epsilon path retained behind an ini switch for historical reproducibility.
4. **Given** the investigation outcome, **When** energy-as-throttle-proxy is found insufficient, **Then** the open question on richer total-energy (altitude+airspeed) is answered with a go/no-go for a follow-on, rather than silently dropped.

### Edge Cases

- **Energy axis re-collapses tracking** the way scalar smoothness did. The physical-axis grouping + MAD-relative epsilon are the hypothesized mitigations; if it still collapses, that's a documented outcome, not a silent failure.
- **Energy improves but only by flying slower/lower** (degenerate "minimize energy by minimizing flying"). Tracking-non-regression is the guardrail.
- **MAD-epsilon change perturbs determinism / historical reproducibility** — the constant-epsilon path must remain selectable so prior runs can be reproduced bit-for-bit.
- **Basin lottery confounds the comparison** — a stuck (throttle-pegged) bake tells us nothing about the energy axis; per the basin protocol, budget 2–3 bakes to clear the ~1:3 lottery before drawing conclusions.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Energy MUST be re-enabled as a lexicase test-case axis (per-scenario `energy_score`), NOT a scalar penalty composed into the tracking score (uncomment `src/eval/selection.cc:69`).
- **FR-002**: Lexicase axes MUST be grouped physically — bank (pitch+roll) together, throttle/energy separate — per `project_smoothness_axis_grouping`.
- **FR-003**: Lexicase epsilon MUST be MAD-relative (scales with per-scenario magnitude) rather than constant 0.5, with the constant-epsilon path retained behind an ini switch for historical reproducibility (`project_lexicase_mad_epsilon`).
- **FR-004**: The 4 `DISABLED_` Selection027 multi-objective tests MUST be re-enabled and pass with the energy axis active.
- **FR-005**: An energy-lexicase bake MUST be run and its outcome classified against the tracking-only baseline (pop=8000/wind=36 + 034 craft variations) on both tracking-quality and energy-efficiency, with per-scenario evidence captured in an outcome doc.
- **FR-006**: The investigation MUST guard against degenerate energy wins (energy improved only by abandoning tracking) — tracking-quality non-regression is a required condition for an "energy works" verdict.
- **FR-007**: The open question (throttle-proxy energy vs richer total-energy = altitude+airspeed) MUST be resolved with a go/no-go recommendation for a follow-on, informed by the bake outcome.
- **FR-008**: Whether the `stability_score` axis (`selection.cc:68`) is also re-enabled MUST be an explicit decision recorded with rationale (not left ambiguous).

### Key Entities

- **Secondary objective**: a per-scenario score axis (energy_score, optionally stability_score) used as a lexicase test case alongside the tracking score — distinct from a scalar penalty folded into one number.
- **energy_score**: per-scenario `Σ(out_th − 1)/2`, already computed + emitted to data.stc; lower = better (less throttle).
- **MAD-relative epsilon**: lexicase pass/fail threshold scaled by the median-absolute-deviation of per-scenario scores, replacing the constant 0.5.
- **Baseline**: the 034-delivered tracking-only run (pop=8000/wind=36 + craft variations) that energy bakes are compared against.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The energy investigation produces a clear verdict (energy-works / tracking-collapses / energy-unmoved) backed by a baked comparison against the baseline, captured in an outcome doc — no ambiguous "maybe."
- **SC-002**: If energy works, the resulting controller shows materially improved energy_score with tracking quality (per-scenario score + avgMaxStreak) within noise of the baseline; if it doesn't, the failure mode is documented well enough to inform the total-energy go/no-go.
- **SC-003**: The MAD-relative epsilon change preserves bit-exact reproducibility of prior runs when the constant-epsilon ini switch is set.
- **SC-004**: The Selection027 multi-objective tests are re-enabled and green.

## Assumptions

- 034 is delivered first: minisim removed, smoothness removed, craft variations landed, tech-debt fold-ins done, and a craft-variation-trained controller flight-tested. 035 starts from that baseline.
- The investigation may conclude energy-as-lexicase doesn't work; that is a valid, valuable outcome (it closes a question the project has circled since 027).
- "Materially improved/regressed" thresholds are set against the baseline's per-scenario metrics at investigation time (per-scenario currency).
- The basin lottery (~1:3 stuck) applies; budget 2–3 bakes to land a non-stuck climber worth analyzing.

## Out of Scope

- Total-energy (altitude+airspeed) implementation — 035 only produces the go/no-go recommendation for it (FR-007).
- Demetic / island-model GA for basin escape — separate BACKLOG research entry (the energy axis is the cheap first attack on the same stuck basin).
- Craft/camera variation work — owned by 034 (craft) and a later iteration (camera).
- Changing NN topology or the tracking fitness surface (conical scoring).
