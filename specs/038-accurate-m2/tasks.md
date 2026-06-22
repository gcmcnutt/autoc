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

**Status**: FIRST RESULT IN — t12 ran gen 1→774, stopped 2026-06-19 (plateau). Rule worked on its
target but **displaced** the failure mode; t13 will iterate the weights + the signal gradient. See
the t12 outcome below.

### T001 RESULT — t12 (penalty ON) vs t11 (penalty OFF), 2026-06-19

Same source (t10 M1, `gen9200.dmp`), 20 Hz + 0.8 s, 294 scenarios; t12 = t11 + `EnableHullCrashPenalty=1`,
`HullCrashPenaltyFactor=0.5` (member-level `0.5^K_hull` on all 294 tracking `.score`). t12 stopped at
**gen 774, best −13836.3**; t11 stopped gen 604.

**The penalty works decisively on its target:**
- `hullStrike = 0` held the **entire run** (gen 1→774), vs t11 climbing to **23–24** as it sharpened.
  A rare strike really was made member-deciding (SC-001 / SC-002 direction confirmed).
- Tracking depth was **not** sacrificed: `rabbitComplete` 272–273/294 (t11 260–263), pctInStreak ~2 pts
  behind t11. The chase still tracks; it just doesn't collide.

**But it displaced rather than eliminated the failure (the headline finding):**
- The failure mode converted **hull → OOB (`Eval`, arena egress)**. At matched gens (= matched
  variation scale): gen 500 t11 `eval=2`/`hull=23` vs t12 `eval=42`/`hull=0` — a 20× OOB swing with
  hull zeroed. The controller *actively chooses flyaway over collision* because OOB was left as the only
  cheap exit (it just stop-accumulates; only `HullStrike` got the `0.5^K` hammer).
- OOB was **non-monotonic**: spiked to 41 @gen 602, relaxed to ~21–22 by gen 774 as the elite locked in.
- Tactic is **preventative, not reactive**: cut throttle when close, hold a large standoff (median
  min-dist 3.42→3.63 m vs t11 ~2.1 m), not learned evasion. `tactics.png` Panel B (throttle-vs-distance
  policy) is distinctly different from t11; the encounter histogram barely changed.

**RNN capacity (rnn_capacity.py, elite W_hh SVD) — no widening indicated:**
- Effective rank steady **~10.4–10.8 / 16** across the run (10.76 g493 → 10.41 g602 → 10.68 g746),
  **not** climbing toward 16; ρ ~6.3–6.4, ‖W_hh‖ ~11.4. The rising `whh_xh_ratio` is *reliance* on the
  same ~8–10 modes, not capacity exhaustion. **32r stays down-weighted.**

**Convergence character — subtle signals (operator, "looking closer", 2026-06-19):**
- Best-fitness **jumps more in later gens** than other runs (volatile elite late).
- Population **avg getting worse** (this is *why the run converged quick / short*).
- Selection **sigma flattening** — not moving down as fast as a clean run.
- **Avg-max-streak worsening** (the standoff tactic trades tracking-streak for safety).
- Flight-dynamics chart **held**; per-axis aggressiveness **held** similar cross-gen improvements.
- *Interpretation*: the `0.5^K` multiplier on a **fluctuating** crash count (elite OOB swings 12↔42
  gen-to-gen) injects fitness **variance** — the whipsaw flagged in the t13 discussion. Pop-avg-down +
  sigma-flat reads as selection fighting that variance rather than refining tracking.

### T001 → t13 design notes (operator, 2026-06-19) — "make the signal gradient slightly different"

Three levers, to try together or bisected:
1. **Relax + split the penalty**: hull crash `0.5 → 0.75`, **add** exit-arena (OOB) `→ 0.9`
   (`mult = 0.75^K_hull × 0.9^K_oob`). Less hull-avoidance pressure (reduce the displacement) + a real
   (but milder) OOB cost. Exchange rate ≈ **1 collision ≈ 2.7 flyaways** — matches "both severe, slight
   hull bias." **Caveat** (from the t13 discussion): `0.9^K_oob` is still steep on a *common* count
   (`0.9^21 ≈ 0.11`); the relaxed hull factor should drop OOB counts, but watch for population-crush /
   the need for fraction-based OOB or MAD-epsilon (US3) if the signal washes out.
2. **Wind parity**: M2 does **not** replay the on-course wind M1 saw — consider replaying the *same*
   wind the source experienced, so the chase's air mass matches the path it's chasing. (Relates to the
   two-sim co-eval backlog + `wind_study.py`: M1/M2 winds differ ~38° but were uncorrelated with score —
   parity may still matter for *path feasibility* under the harder t10 source.)
3. **Reshape the tracking cone / ramp**: the t10 source paths are more complex (best-ever M1 =
   aggressive maneuvers), so the cone (`FitConeAngleDeg=45`) and/or variation ramp may want retuning to
   give a gradient suited to the harder paths.

Related study (backlogged 2026-06-19): **negative reward when the chase gets *ahead* of the target** —
reward-shaping sibling to the crash penalty; may shape the same safety behavior through the gradient
instead of (or with) the multiplier. See `specs/BACKLOG.md` → "Negative reward when the chase gets ahead".

(t11 stopped at gen 604, t12 at gen 774 — `build/autoc` is free for the t13 build.)

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
