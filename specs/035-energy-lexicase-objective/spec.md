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

## Prerequisites (pre-035, before US1)

### Retire data.dat — S3 dmps become the sole per-run trace

**Rationale:** Today every training run writes a multi-GB `data.dat` (text per-tick trace of the elite) alongside an S3 dmp per gen. The dmp is a cereal-serialized `EvalResults` containing essentially the same elite trajectory (`aircraftStateList` per-tick, `pathList`, `scenarioMetaList`, `crashReasonList`, tracker-mode camera/target lists). S3 dmps are already unique per run and persist longer than the workspace `data.dat`. Maintaining the text duplicate adds I/O cost, disk pressure, and a divergence surface (every schema bump must update both writers).

The 034-era trade ("keep data.dat for Python analysis tools, it streams cheaply") expires now that 035 introduces new per-scenario axes (energy_score, MAD-relative epsilon stats) that would otherwise need to be added to *both* writers. Single source of truth wins.

**Approach:** drop the data.dat writer entirely; add any data.dat-only columns to the dmp schema (greenfield, no cereal version bump per project policy); build a `dmp-dump` CLI that reads a `.dmp` (from local file or S3 stdin) and emits the same text-column format the Python plot scripts already consume. Python tools invoke the CLI (`dmp-dump < x.dmp | python3 plot_…py`) instead of streaming data.dat directly.

**Functional Requirements (pre-035):**

- **FR-P01**: `EvalResults` MUST carry every data.dat field that isn't trivially recomputable from `aircraftStateList[scenario][tick]` + `pathList`. Audit columns: NN inputs/outputs/position/quat/body-velocity/PID-internals/rabbit-speed are already in `AircraftState`; the **derived group** (dhome, dist, along, stepPoints, mult, rampSc) is NOT — decide per-column whether to (a) add to AircraftState/ScenarioMetadata as stored state or (b) recompute in the dumper from existing dmp content. Default: recompute unless the path-following math is too coupled to autoc internals.
- **FR-P02**: A new CLI tool MUST fetch a `.dmp` directly from S3 (given an S3 URI or bucket+key) and stream a semi-human-readable, easily-parseable representation to stdout. Format SHOULD be CSV for per-tick time-series rows (one row per scenario × tick, columns named for `aircraftStateList` fields + derived dhome/dist/along/etc.) and YAML for the per-run/per-scenario metadata block (`scenarioMetaList`, `crashReasonList`, gp hash, provenance), with a header that segregates the two. Local-file input MAY be supported as a developer convenience but S3 is the primary mode (since S3 is the authoritative per-run trace post-data.dat retirement).
- **FR-P03**: The Python analysis scripts in `specs/03[2-5]*/*.py` MUST be updated to consume the dumper's CSV/YAML output (subprocess invocation or piped stdin) — no direct data.dat dependency remaining, and no byte-compat with the legacy `data.dat` format is required (the scripts get rewritten to use the new column names directly).
- **FR-P04**: The `rebuild-perf.sh` M1→M1 replay regression gate MUST swap from `data.dat`-byte equality to dmp-byte equality (or per-scenario-score byte equality if dmps carry non-deterministic metadata like upload timestamps). The byte-exact replay property itself is non-negotiable.
- **FR-P05**: All `data.dat` plumbing MUST be removed from `src/autoc.cc` (the `fout` ofstream, `logEvalResults` writer, `strOutFile` open, pathgen + tracker header emission). Constitution III: clean cut, no dual-write.
- **FR-P06**: The `.gitignore` `*.dat` rule, `include/autoc/eval/eval_logger.h` comment, and any spec/doc reference to `data.dat` as a live artifact MUST be updated.

**Acceptance:** a short bake produces zero `data.dat` files; `dmp-dump` invoked against a gen dmp produces output the existing plot scripts consume without modification (beyond input-source plumbing); `rebuild-perf.sh` gate passes via dmp byte-equality.

**Out of scope (deferred to a later iteration):**
- Self-describing dmp format / cross-version compatibility — current cereal is positional; OK for now, document the limitation.
- A Python-native cereal reader — the CLI dumper as text bridge is sufficient; native Python deserialization is nice-to-have, not gating.

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
- **FR-008b**: Candidate lexicase axes for this multi-objective framework are **energy** (primary, this feature), **stability** (FR-008), and **M2 hull-crash-cost** (the stranded "033 phase-2" kamikaze penalty — see BACKLOG "[035-family] M2 hull-crash-cost lexicase axis" + `project_hull_escalation_finding`). Crash-cost is a *fitness dimension* (035-family), NOT a 036/island concern; it must be a lexicase axis, not a scalar penalty (same collapse risk). The M2 run will quantify the hull-escalation rate that motivates it — record whether to fold crash-cost into 035 now or as an immediate 035 follow-on.
- **FR-009**: 035 MUST optimize the **energy dimension, NOT smoothness**. Do not add a control-rate / bang-bang penalty. Smoothness is an emergent *consequence* of minimizing energy (aggressive maneuvering → induced drag → energy cost), not a target in its own right. The 034 origm1-5000×49 run already self-smoothed (roll-rate halved in the back half) under a pure tracking objective with no smoothness term — evidence that the search drifts toward smooth on its own, so the lever to add is energy, not a smoothness whip. (Prior scalar smoothness/penalty attempts *dulled* the system — see discussion below.)

### Design discussion — measuring total energy input (operator, 2026-06-03)

The energy metric is the central design question of 035 (supersedes the throttle-proxy placeholder in FR-007 / Key Entities). Direction:

- **Quantity to minimize = total energy *input*, summed over time** — `Σ_t energy_consumed(t)`. Energy *consumed*, not a control-amplitude proxy. Start with the consumed-energy integral as the v1 metric.
- **Energy input is probably a non-linear function of throttle**, not the linear `Σ(out_th − 1)` placeholder. Motor/prop power draw rises super-linearly with throttle; the metric should reflect that (e.g. a convex function of throttle command, or the FDM's actual electrical/shaft power if exposed).
- **Induced drag is part of the cost.** Aggressive maneuvering (tight turns, high-AoA) burns energy via induced drag even at constant throttle — and the LaRCSim FDM already simulates it. So a *measured* energy term naturally charges for bang-bang on every axis, which is exactly why no separate smoothness penalty is needed (FR-009).
- **Optionally fold in mechanical-energy state** — potential energy (altitude) and kinetic (airspeed). A craft that trades altitude/speed for tracking is spending stored energy; a full accounting is `Δ(PE+KE) + work_in`. **But start simple**: consumed-energy-over-time first; add the PE/KE state terms only if the throttle-integral metric proves insufficient (this is the richer-total-energy go/no-go of FR-007).
- **Why lexicase, and the "dulling" risk.** Several prior approaches that made "more energy = worse" *dulled the whole system* — the scalar 033 smoothness-penalty floor collapsed the controller into a Pareto corner ([project_scalar_multiobjective_collapse](../../.claude/projects/-home-gmcnutt-autoc/memory/project_scalar_multiobjective_collapse.md)), and energy-as-penalty underperformed in 027/028. The bet for 035 is that **lexicase keeps energy as a separate selection dimension** (some test cases select on energy, others on tracking) rather than a scalar discount on tracking — so it can pull energy down without dulling the tracking drive. If lexicase *also* dulls it, that's a key negative result.
- **Timing (consider for design):** bang-bang may be useful *early* (aggressive basin-finding) and only wasteful *late* — the 034 run found its basin rough then smoothed. So energy pressure may want to **ramp in late** (like the variation ramp) rather than apply from gen 0, to avoid suppressing early exploration.
- **M2 motivation:** tracking mode is intrinsically high-energy (a chase craft maneuvers continuously to follow an erratic target), and real-flight battery is a hard constraint — so the energy dimension matters *more* for M2/deployment than for smooth pathgen courses. This is the deployment reason 035 is load-bearing.

### Key Entities

- **Secondary objective**: a per-scenario score axis (energy_score, optionally stability_score) used as a lexicase test case alongside the tracking score — distinct from a scalar penalty folded into one number.
- **energy_score**: the per-scenario energy metric. The existing `Σ(out_th − 1)/2` (linear throttle proxy, already computed/emitted) is a **placeholder starting point** — see the "measuring total energy input" discussion above: v1 should move toward a consumed-energy-over-time integral with a non-linear throttle term (and induced drag captured by the FDM), with PE/KE state terms as a later option. Lower = better (less energy).
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
