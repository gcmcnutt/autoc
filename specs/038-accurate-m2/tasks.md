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

**Status**: IN PROGRESS (impl). Rule is the operator-confirmed first attempt; expect to iterate.
(t11 stopped at gen 604, so `build/autoc` is free — no worktree needed.)

### T001 also doubles as the RNN-memory-capacity probe (whh_xh_ratio reasoning, 2026-06-17)

Observed on t11 (M2): `whh_xh_ratio` (recurrent-path / input-path hidden activation magnitude,
`compute_synthetic_activation_ratio`) climbed **0.39 → ~0.90 and was still rising at gen 600**, in
lockstep with pctInStreak — whereas the **M1 source (t10) stayed flat ~0.45–0.49**. Reason: M2's
target is FOV-limited (27% of ticks fully blind, **43% blind in the final 5 ticks before a hull
strike** — see intercept F/G visibility panels), so the RNN must **predict through the blind gaps**;
M1's rabbit is always "visible" so it rides live input. `w_hh_cv` diversified once early (~0.16) then
stayed flat while the ratio kept rising → the 16-dim memory is being **scaled harder, not
restructured**. Current recurrent capacity: **hidden-2 16-wide only** (16-dim state, 256 W_hh);
hidden-1 (32-wide) is feedforward.

**T001 is the cheapest discriminator for "do we need more RNN?":**
- If the penalty **sustains** whh_xh_ratio AND holds tracking → the 16-dim memory can both track and
  hedge-when-blind; no more RNN needed yet.
- If it forces a trade-off (tracking tanks, or the ratio collapses/saturates because 16 units can't
  predict-and-avoid at once) → that's the **bottleneck signal** → widen next.

**Gated follow-on (only if T001 shows the squeeze): T00x — wider RNN ("32r").** hidden-2 16→32
(32×32 W_hh) or make hidden-1 recurrent too — the 028-deeper-rnn lever; precedent in 030-postdiag3 /
032. Cost: bigger search space (basin-lottery/convergence) + xiao forward-pass time — pay only once
the 16-dim limit bites. Do NOT widen blindly ahead of T001.

> **Direct weight evidence DOWN-WEIGHTS 32r (2026-06-18, `src/analytics/rnn_capacity.py`)**: SVD of the
> elite W_hh (16×16) on both t11 (g604) and t12 (g493) shows **effective rank ~10.7/16** (90% of the
> dynamics in **8** modes), i.e. **~5–6 recurrent dims sit idle — the layer is NOT saturated**. The
> rising whh_xh_ratio is *reliance*, not capacity exhaustion (a saturated layer would show eff-rank≈16,
> flat spectrum). The penalty barely changed the structure (t11→t12 eff-rank 10.69→10.76, ρ 5.33→5.46,
> ‖W_hh‖ 10.1→10.8) — it leans harder on the **same** modes, doesn't need new ones. So a bigger
> recurrent layer would just add idle modes; the bottleneck (if any) is upstream (optical input /
> policy), not recurrent width. **rnn_capacity is now part of the M2 report mix** (generate_pngs.sh,
> M2-only) to watch eff-rank/spectral-radius over time — widen only if eff-rank climbs toward 16.

---

## (Deferred to /speckit.tasks) the rest of 038

US1 full (crash_cost recording, M1+M2, dmp surfacing), US2 camera variations, US3 MAD-epsilon,
US4 history buffer, and Phase 0 (P0-A PRNG validation, P0-B renderer config-hygiene, P0-C reporting
[partly landed], P0-D clean-slate one-dmp-break: simTimeMsec stamping + self-describing dmp +
wind_velocity recording). See `spec.md`.
