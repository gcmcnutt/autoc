# 038 Accurate M2 — Tasks

> Stub. The full dependency-ordered breakdown comes from `/speckit.tasks` once the spec is clarified.
> Captured here first: the **hull-crash-penalty experiment we want to run NOW on the 037 setup**,
> before the P0-D clean-slate re-bake (operator 2026-06-16).

## T001 [US1] Hull-crash penalty — first-attempt rule (try on the live 037 setup)

**Goal**: see whether a hull-strike fitness penalty makes the chase track *without colliding*, using
the **current sensors / source (t10) / 20 Hz config** — no contract break, M2-only — so we learn
before invalidating everything for P0-D.

**The rule (first attempt, 2026-06-16)**: per member, let K = number of its scenarios that ended in
`HullStrike`. Multiply **all 294 of that member's per-scenario tracking `score` values by `0.5^K`**
(member-wide), before lexicase selection.
- `score` is negative-with-lower-better, so ×0.5 pulls toward 0 = **worse** → the crasher loses the
  `score` test cases on its strong scenarios. Correct direction.
- **Only the `score` (tracking) axis** — NOT `energy_score` (different sign/meaning; scaling it would
  move energy the wrong way).
- **Aggressive on purpose**: today's elite has ~11–12 strikes → `0.5^12 ≈ 0.0002` → effectively
  removed from selection. Slightly-worse-but-cleaner controllers then emerge. "That will leave a mark."
- **Tunable** (revisit from observed behavior): the per-strike factor (0.5), per-strike vs flat
  (halve-once-if-any-strike), a cap on K, or a softer ramp. Start with `0.5^K`.

**Interaction notes**:
- Hull strikes only *fire* via the existing annealed `p_crash` Bernoulli curriculum (per-scenario
  rabbit-class-seeded), so the penalty bites more as the curriculum tightens — expected.
- This is a fitness/selection change, **not** a dmp-format change → no P0-D contract break needed.
  (Recording `crash_cost` per FR-005 is additive/optional for this experiment; HullStrike is already
  in `crashReason`.)

**Implementation hook**: parent-side, where each member's per-scenario `ScenarioScore` vector is
finalized before `lexicase_select` (the `computeScenarioScores` path; see
`project_fitness_to_worker_backlog`). Count `HullStrike` scenarios → multiply that member's `.score`
fields by `0.5^K`. Behind an `EnableHullCrashPenalty` knob (default off; on for the experiment).

**t11-safe build/run**: t11 is live on `build/autoc` — do NOT rebuild it (no-rebuild-during-training).
Build the variant in a **separate worktree / build dir**; operator launches **t12** from that binary
via `scripts/train.sh` (Constitution IX). t11 keeps running as the penalty-OFF baseline.

**Acceptance / what we're looking for**:
- t12 (penalty ON) vs t11 (penalty OFF), same source/sensors/config: hull-strike rate **drops at
  matched tracking depth** (pctInStreak / mean target distance), and the strike-vs-gen curve does not
  climb monotonically with tracking skill.
- Read with `scripts/generate_pngs.sh m2` + a per-gen `#GenCrash hullStrike` trend; compare the two
  runs (the evolution `--compare` overlay).

**Status**: PLANNED — awaiting go to implement (worktree build). Rule is the operator-confirmed first
attempt; expect to iterate.

---

## (Deferred to /speckit.tasks) the rest of 038

US1 full (crash_cost recording, M1+M2, dmp surfacing), US2 camera variations, US3 MAD-epsilon,
US4 history buffer, and Phase 0 (P0-A PRNG validation, P0-B renderer config-hygiene, P0-C reporting
[partly landed], P0-D clean-slate one-dmp-break: simTimeMsec stamping + self-describing dmp +
wind_velocity recording). See `spec.md`.
