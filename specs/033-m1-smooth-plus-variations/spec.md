# Feature Specification: 033 — M1 smoothness + replay-friendly variation PRNGs (+ kamikaze inherit)

**Feature Branch**: `033-m1-smooth-plus-variations` (proposed; created 2026-05-19)
**Created**: 2026-05-19
**Status**: DRAFT — operator-initiated during 032 phase-1 bake monitoring

## Boundary lines

033 is a **reward-shape + variation-determinism** feature that addresses three problems exposed by 029/030/032 bakes + the 0517 real flight:

1. **Bang-bang controllers in M1** — the 0517 pastonly3 flight ([FLIGHT_REPORT](../../flight-results/flight-20260517/FLIGHT_REPORT.md)) confirmed the bang-bang signature documented in [project_bangbang_axis_migration](../../.claude/projects/-home-gmcnutt-autoc/memory/project_bangbang_axis_migration.md) is materially rough on the airframe and limits real-flight performance. Same root cause as M2's kamikaze pattern (032 §1.8 reframe 2026-05-18): aggressive evolution against airframe limits with no penalty for the aggression.
2. **Kamikaze in M2** — 032's phase-1 bake showed hull-strike escalation grows monotonically with controller skill; current objective IS kamikaze (operator quote 2026-05-18). 032 spec.md §1.8 introduces US3 (multiplicative ½-score penalty); 033 carries that work, possibly with stronger penalty form.
3. **Wind/variation non-replayability** — comparing 030 vs 032 bakes (and any two M2 bakes) is confounded because per-scenario variation (wind direction, entry cone, speeds) draws from PRNG state that drifts across runs. Even at the same gen index, two bakes face different "weather". This blurs ablation experiments and any "did the new sensors help" question.

| Feature | Scope | Status |
|---|---|---|
| **028 (recurrent NN, lexicase smoothness)** | First attempt at smoothness via lexicase test case | shipped; bang-bang persisted post-031/032 |
| **029 (no-future arch)** | Past-only sensor input; bang-bang continued in real flight | shipped; 0517 flight evidence |
| **030 (tracker mode)** | M2 hull-strike escalation discovered | shipping v1 |
| **031 (beacon-camera hardware)** | LED pyramid + camera + lens; physical optical front end | parked pre-hardware-order |
| **032 (derived perceptual features)** | M2 sensors for closing/wingspan; sensors are sufficient, reward is the problem | in flight (gen ~600 of 800) |
| **033 (this spec)** | **(a) Replay-friendly variation PRNG architecture, (b) M1 smoothness penalty, (c) M2 inherits, (d) kamikaze penalty** | DRAFT |
| **034+ (future)** | Craft variations, camera variations, etc. — separate stories | not started |

---

## 1. What we learned that motivates 033

### 1.1 M1 bang-bang persists in real flight

029 pastonly3 converged-elite NN flew on 2026-05-17 ([FLIGHT_REPORT](../../flight-results/flight-20260517/FLIGHT_REPORT.md)). Pilot qualitative: "the bang-bang on roll is still present and rough on the airframe." Span 2 (path 2 figure-eight) tracked cleanly but the bang-bang roll signature limits how aggressively the controller can be flown in real wind. 028's lexicase smoothness path improved the symptom but didn't eliminate it; 029 (no-future arch) made the controller "shape" different but bang-bang stayed.

### 1.2 M2 kamikaze is the same root cause

Confirmed during 032 phase-1 monitoring (2026-05-18). Per-gen hull-strike count grows monotonically with controller skill in both 030 (1→11/gen over 500 gens) and 032 (~3× faster than 030). The new 032 sensors are sufficient (k/dist fit is clean; NN weakly uses span for throttle modulation already) — what's missing is reward gradient pushing toward "don't crash." With current objective, kamikaze IS the global optimum.

**The general principle** (spec.md 032 §1.8 Reframe): anywhere reward is "be aggressive at any cost," evolution converges to aggressive-as-possible solutions that are unsafe to fly. M1 bang-bang and M2 kamikaze are two instances of the same reward-shape pathology.

### 1.3 M1 and M2 baselines show the SAME control-aggressiveness signature (2026-05-20)

Compared `per_axis_aggressiveness.py` output across the 029 pastonly3 converged-elite run (M1) and the 032 phase-1 closeout (M2). **Same room** — pitch/roll/throttle aggressiveness distributions and dCtrl / ⟨|out|⟩ metrics are in the same range on both modes, with the same characteristic bang-bang signature (roll usually dominant).

**Why this matters**: it confirms the reward-shape root cause crosses modes — M1's pathgen objective and M2's tracker-cone objective converge to the same control-aggressiveness regime when neither has a smoothness term. They differ in tracking strategy (M1 follows synthesized rabbit, M2 chases the trail rabbit derived from source dmp) but their per-axis control discipline is identically un-disciplined.

**Implication for 033 success criterion**: we now have a **definitive paired baseline** for the smoothness penalty experiment. The "without smoothness" reference isn't an estimate or a target — it's two concrete runs (M1 pastonly3 + M2 032 phase-1) with measured aggressiveness signatures we can compare against. Phase-1 success criterion (§2.B) becomes concrete: **"reasonably tracking AND materially better per-axis aggressiveness numbers vs the M1/M2 baseline pair."** Tighter than a generic "bang-bang reduced" target.

### 1.4 Wind/variation drift across bakes blurs ablations

032 phase-1 bake's avgInRamp is being compared against 030 postdiag2's plateau (~0.07 vs ~0.12-0.15 in 032). But these two bakes faced **different per-scenario wind realizations** — even though both used the same `WindScenarios=49 × paths=6 = 294` scenario count, the joint-PRNG state (`scenarioSequence`, `windSeed`, entry-cone draws) drifted across the two runs. So "0.12 vs 0.07" mixes:
- Real signal: the 9 new sensors helped
- Confound: 032 happened to see easier weather than 030 (or vice versa)

Per [project_v15_determinism_candidates](../../.claude/projects/-home-gmcnutt-autoc/memory/project_v15_determinism_candidates.md): "Non-determinism hurts evolution signal, not just elite-reeval." This is the same concern, but for cross-run ablation rather than intra-run elite-reeval.

The fix: a **master-seed architecture** where the same master seed produces the same variation table across runs, regardless of which downstream consumers are active.

---

## 2. Proposed 033 scope

Four work items, all controller-side (no hardware, no FPGA, no perception changes):

### 2.A Replay-friendly variation PRNG architecture

**Problem**: today's per-scenario variation is computed by sequentially consuming a joint-PRNG state. If feature X is disabled (e.g., M2 doesn't use rabbit-speed variations), its PRNG slot is **not consumed** — so the downstream consumers (wind, entry cone) get different values than they would if X were enabled. This makes M1 ↔ M2 ablation impossible without re-running both ends.

**Solution**: master-seed + sub-consumer architecture.

- **Master seed** (per-run): seeds a `MasterPRNG` instance at run init.
- **Sub-consumer slots** (fixed enumeration, frozen contract):
  | Slot | Purpose | Consumed by |
  |---|---|---|
  | 0 | Wind direction | M1 + M2 |
  | 1 | Wind strength | M1 + M2 |
  | 2 | Entry cone sigma | M1 + M2 |
  | 3 | Entry roll sigma | M1 + M2 |
  | 4 | Entry speed sigma | M1 + M2 |
  | 5 | Entry position radius | M1 + M2 |
  | 6 | Entry position alt | M1 + M2 |
  | 7 | Rabbit speed (target speed in pathgen) | M1 only — but slot is consumed in M2 too (to keep downstream slot states aligned) |
  | 8 | Crash hull PRNG seed | M2 only (and consumed in M1 even if hull is off) |
  | 9+ | reserved for future variation features (craft variations, camera variations, etc.) |

- **Each sub-consumer** is a separate PRNG seeded deterministically from `MasterPRNG.next()` — so its sequence is independent of whether other sub-consumers are active.

- **Per-scenario variation generation**: scenario N draws from sub-consumer K's Nth output, regardless of which other sub-consumers exist or are active.

- **Result**: M1 and M2 bakes with the same master seed see the **same wind, entry cone, speed, position variations** across scenarios — only the controller-relevant draws differ. Cross-feature ablation becomes a clean A/B.

- **Wind replay within a scenario**: the per-scenario wind realization (gust profile over time, thermal vortex placement) should also be deterministic per (scenario, master-seed) pair. Today CRRCSim's wind is per-frame stochastic from a separate PRNG — needs to be reseeded per-scenario from sub-consumer 0 (wind direction) so that replays of the same scenario produce identical wind.

### 2.B M1 smoothness fitness penalty — multiplicative on stepPoints

**Goal**: penalize bang-bang within the M1 objective function so evolution selects for smooth controllers — without throwing away the streak-multiplier dynamics that drive convergence.

**Operator-preferred form (2026-05-19)**: a multiplicative penalty factor on per-tick `stepPoints` BEFORE the streak multiplier is applied. Smooth controllers get full credit (×1); bang-bang controllers get half credit (×0.5).

```text
per_tick_motion = aggregate(|Δoutput[pt]|, |Δoutput[rl]|, |Δoutput[th]|)
                  # Pythagorean: sqrt(Δpt² + Δrl² + Δth²)
                  # OR sum:       |Δpt| + |Δrl| + |Δth|

smoothness_factor = 1.0 − 0.5 × min(per_tick_motion / motion_max, 1.0)
                  # = 1.0 when no axis moved between ticks (zero output Δ)
                  # = 0.5 when all 3 axes did a max-throw simultaneously
                  # bounded in [0.5, 1.0]

final_step_score = step_score × smoothness_factor × streak_multiplier
```

Where `motion_max` is:
- Pythagorean: `sqrt(2² + 2² + 2²) = sqrt(12) ≈ 3.46` (each axis output ∈ [−1, +1] → max Δ = 2 per axis)
- Sum: `2 + 2 + 2 = 6`

**Why multiplicative-on-stepPoints (vs additive subtract or vs lexicase)**:
- **Preserves streak dynamics**: the streak multiplier still compounds as normal; smooth controllers get more streak gain than bang-bang ones at the same in-cone progress. Doesn't disrupt the fitness-shape evolution has been optimizing against.
- **Bounded penalty (floor at 0.5)**: avoids the "controller learns to do nothing" failure mode that an unbounded subtract risks. The penalty is informative (lose up to half) but not annihilating.
- **Penalty is zero when controller is at rest**: matches physical intuition — no control input = no airframe wear = no penalty. Aggressive controllers that need to be aggressive (high-roll turns inside the cone) still get rewarded, just less per tick.
- **Tunable form**: ini knob `SmoothnessPenaltyFloor` (default 0.5; raise to 0.7 for mild penalty, lower to 0.3 for harsh) + `SmoothnessMotionMode` (`"pythagorean"` vs `"sum"`).
- **Single knob**, axis-agnostic — no per-axis tuning needed. If roll dominates bang-bang historically, that's where most of the penalty lands automatically.

**Test design**: instrument the existing 030/032 bake data.dat per-tick to compute what `smoothness_factor` distribution looks like with `floor=0.5`. Expectation: late-gen elites with bang-bang signature will see floors of 0.5-0.7 per tick on bang-bang axes, costing significant fraction of streak compound. That's the signal evolution should follow.

**Alternative (deferred)** Path B — lexicase smoothness test case (the [027 path](../027-recurrent-nn/spec.md)). 027/028 tried this; results were mixed. Path A is preferred per simplicity + the BACKLOG endorsement.

**Success criterion**: the per-axis aggressiveness chart ([per_axis_aggressiveness.py](../030-tracker-mode/per_axis_aggressiveness.py)) shows dCtrl ≤ 0.27 per-axis with sum-over-axes ≤ 0.80 (the existing 028 spec-gate budget). Phase-1 readiness gate: those budgets met at training plateau AND held through eval-reeval AND replayed cleanly in the mezzanine real flight.

**Concrete baseline for A/B**: §1.3 establishes that M1 pastonly3 (029) and M2 032-phase-1 closeout produce **the same control-aggressiveness signature** (same "room" of the per-axis chart — both bang-bang, both well outside the 028 spec-gate budget). That paired baseline is what 033's smoothness experiment is compared against — not an estimate, not a target — concrete measured numbers from two existing runs. The 033 phase-1 NN must show:
- **Reasonably tracking** (avgInRamp within some delta of M1 or M2 baseline depending on which mode is bake; exact delta TBD via /clarify, but on the order of "not catastrophic collapse" — e.g., ≥ 0.05 if baseline was 0.07-0.16)
- **Materially better per-axis numbers** vs the M1/M2 baseline pair — dCtrl ≤ 0.27 budget being the tight target, but even partial movement (e.g., dropping from sum 1.4 toward 0.8) is informative phase-1 signal
- Tracking-vs-smoothness frontier is acceptable: a slight tracking degradation in exchange for major smoothness gain is the desired tradeoff for real-flight deployment, since the M2 0517-flight evidence was the bang-bang signature was the actual airframe-flyability blocker, not the tracking number

**Mezzanine test — real flight**: per operator routing 2026-05-19, the real-flight validation is the gate between 033 phase 1 (smoothness alone in M1) and 033 phase 2 (M2 work). A successful real flight with reduced bang-bang signature on the airframe is the qualifier to move on.

### 2.C M2 inherits the smoothness objective

Same `SmoothnessPenaltyK` term added to M2 fitness. No code change beyond what 2B introduces — the per-tick `Δoutput` penalty is mode-agnostic (just reads the NN's outputs, identical for M1 and M2).

**Test**: M2 trained with the smoothness penalty should show reduced bang-bang in pitch/roll/throttle time series + reduced hull-strike rate (because aggressive overshoot now costs fitness in addition to the kamikaze penalty from 2D).

### 2.D Kamikaze penalty (carried from 032 US3) — **033 phase 2, AFTER smoothness mezzanine flight**

Per 032 spec.md §1.8 + US3 task list. Multiplicative ½-of-accumulated-score penalty on hull crash. Operator note 2026-05-19: "perhaps a crash is far more significant cost than just stop accumulating" — suggesting the ½ factor may be too gentle.

**Refined option in 033**: try `HullCrashScoreFactor = 0` (crash → zero out scenario score) first. Total loss of in-scenario credit. Strongest possible signal. If that over-deters and the controller never gets close, back off to 0.25 / 0.5.

**Phasing — kamikaze is 033 phase 2, not phase 1** (operator routing 2026-05-19): "we will prob work smoothness a LOT before we get to m2 so kamakaze can happen then." Phase 1 of 033 focuses on §2.A + §2.B (smoothness in M1) and lands a real-flight validation. Phase 2 of 033 adds kamikaze + §2.C (M2 inheriting smoothness) once the M1 smoothness signature is stable in flight. This sequencing prevents conflating "did smoothness help?" with "did kamikaze help?" and keeps the M1 bang-bang fix as the immediate gate.

Cross-reference: 032 §1.8 US3 had the original framing. 033 elevates it from 032 phase-1b contingency to 033 first-class — but **runs in 033 phase 2, after smoothness validates in flight**.

### 2.E Variation enablement matrix updates

With 2A in place, the existing `Enable<X>Variations` knobs gain new semantics:
- `Enable...= 1` → sub-consumer K is active, contributes to per-scenario variation
- `Enable...= 0` → sub-consumer K still draws (slot is consumed for replay-stability) but the drawn value is **discarded** before applying to the scenario. Downstream consumers still see the same draws.

Update doc: `docs/COORDINATE_CONVENTIONS.md` or a new `docs/variation-prng.md` spelling out the master-seed contract.

---

## 3. Out of scope (deferred to later features)

- **Craft variations** (mass, wingspan, prop, drag perturbations) — earmarked for 034+ (separate story per operator routing 2026-05-19)
- **Camera variations** (FOV, mount, lens distortion perturbations) — earmarked for 034+ (separate story)
- **Per-tick simulator timing jitter** — [BACKLOG] "Simulator Sampling Time Variation" entry. Worth doing alongside 2A but not bundled
- **Total Energy Management / altitude-aware distance** — [BACKLOG DEFERRED]
- **NN topology / architecture change** — 033 stays at the current 32→16r→3 / 33→32→16r→3
- **Hardware** — no 031 (beacon-camera) work
- **Different sensor inputs** — no 032-style additions

## 4. Open questions (operator triage before /clarify)

### Q1 — Smoothness penalty form: per-step Δ² or jerk (Δ²·Δ)?

A: per-step `|Δoutput|²` (Path A from §2.B). Jerk-based is more "smoothness-aware" but expensive to interpret; squared-Δ is the standard control-theory term.

### Q2 — Smoothness K-magnitude: a priori or via gen-0 calibration?

Need to set K such that the per-tick smoothness penalty is **comparable in magnitude** to the in-cone fitness reward — too small and it's noise, too large and the NN learns to do nothing. Calibration approach: instrument the existing 030 bake to compute what a "smoothness K=1.0" penalty would add up to per scenario, then set K such that smoothness contribution is ~10-20% of typical positive fitness.

### Q3 — Replay PRNG contract — does it apply per-scenario or per-run?

A: per-scenario. Each scenario `i` draws from sub-consumer K's `i`th output. Run-init reseeds all sub-consumers from master. Two runs with same master seed see identical variation table.

### Q4 — Wind sub-stream re-seeding within scenario

A: CRRCSim's wind currently uses a per-frame stochastic process. Make the per-scenario wind seed deterministic from sub-consumer 0 (wind direction). Wind details (gust frequency, vortex placement, etc.) become deterministic conditional on (master_seed, scenario_index).

### Q5 — Kamikaze penalty: 0 / 0.25 / 0.5?

A: try 0 first per 2D operator preference. If too strong (controller never gets close), back off.

### Q6 — When to attempt 033?

Two readiness signals:
1. 032 phase-1 closeout: outcome.md + plateau-avgInRamp readout in hand. Lets us measure 033's M2 contribution against 032's known baseline (with replay PRNG making the comparison sharp).
2. Operator decides "the bang-bang signature on real flight is a blocker for further M1 development" — gated on next flight outcome.

Both signals expected within ~weeks. 033 likely starts soon after 032 closes.

---

## 5. Items from earlier specs / backlog that fold into 033

Items originally filed elsewhere that 033 absorbs:

- **[BACKLOG DEFERRED] Path-Relative Smoothness** (`specs/BACKLOG.md` §"Path-Relative Smoothness"): "Normalize Δu by path curvature: penalize excess control, not turns. On hold — slew limiting already killed bang-bang, lexicase hasn't plateaued." → un-deferred under 033 §2.B (the slew-limiting claim turned out to be premature; bang-bang persists in flight). 033 may adopt the path-curvature normalization or keep raw Δ² simpler.
- **[BACKLOG ABANDONED] INAV pt3 RC Smoothing Filter in CRRCSim**: the abandonment conclusion explicitly endorsed "A fitness-based smoothness incentive (lexicase or per-step penalty) is the better path." 033 §2.B is exactly that path.
- **[BACKLOG DEFERRED] Simulator Sampling Time Variation**: out of scope for 033 (separate concern) but worth noting alongside 2A's replay architecture — if jitter is added later, it needs its own master-seed sub-consumer slot.
- **[032 US3] Kamikaze multiplicative penalty**: promoted from 032 phase-1b contingency to 033 §2.D first-class objective.
- **[project_v15_determinism_candidates]** + **[project_tracker_fitness_nondeterminism]**: the cross-run/elite-reeval non-determinism concerns. 033 §2.A addresses the *cross-run* half cleanly; the intra-run elite-reeval determinism (workers + FP non-associativity) is a separate concern not addressed here.
- **[project_bangbang_axis_migration]**: the cross-controller bang-bang signature. 033 §2.B targets the root cause; the memory entry remains valid for tracking how the new objective changes which axis dominates (if any does still dominate).

---

## 6. Status

- **Draft created**: 2026-05-19, during 032 phase-1 bake at gen ~600 (avgInRamp parked at 0.147, within 0.003 of phase-1 success threshold)
- **Branch**: not yet (created after 032 closeout)
- **/clarify**: not started
- **/plan**: not started
- **Dependencies**: 032 phase-1 closeout (US1 outcome.md), 029 0517 flight evidence (already in hand)
- **Phased plan** (operator routing 2026-05-20, finalized after 032 closeout):
  - **033 phase 1 — smoothness penalty (YOLO 0.5 start)**:
    1. §2.A replay-friendly PRNG architecture (foundation; sub-streams seeded from a master PRNG so cross-run / cross-mode ablation sees identical weather)
    2. §2.B M1 smoothness penalty implementation (multiplicative-on-stepPoints, axis-agnostic motion aggregate, **YOLO `SmoothnessPenaltyFloor = 0.5` to start** per operator routing 2026-05-20 — no separate research sub-phase; the M1/M2 baseline pair from §1.3 is the reference, so we just bake against the locked-in 0.5 floor and read the result)
    3. Full-scale M1 bake with floor=0.5 → produces new M1 baseline NN
    4. **Mezzanine real-flight test**: fly the new M1 NN, validate reduced bang-bang signature on the airframe. Operator sign-off on flight outcome
    5. **If 0.5 was wrong** (e.g., controller never learns to track, or smoothness barely budges), iterate on floor / motion-mode then. Otherwise carry forward to phase 2
  - **033 phase 2 — M2 inheritance + kamikaze** (post M1 mezzanine flight pass):
    6. §2.C apply same smoothness penalty to M2 (the M2 sensor suite from 032 is believed-good; smoothness should drop in transparently)
    7. §2.D kamikaze penalty (`HullCrashScoreFactor` — start aggressive per 2D guidance, back off if over-deters)
    8. M2 bake + hull-strike rate comparison vs 032 baseline (use replay PRNG from §2.A for clean A/B)
    9. M2 real-flight validation
  - **Post-033 follow-ons (separate stories)**:
    10. **Craft variations** (mass, prop, drag perturbations) — already specced as 025; addon after 033 lands
    11. **Camera variations** (FOV, mount, lens distortion perturbations) — NO existing spec; 034/035 candidate
    12. Repeat the smoothness + flight + kamikaze cycle with the wider training distribution

**Key reorder vs prior draft**: penalty-floor research (step 3) is its own sub-phase BEFORE committing to a full-scale bake. Avoids 70+ hours of compute on the wrong floor value.

**Belief carried into 033**: the 032 M2 sensor suite is good as-is — phase 3's M2 work doesn't require new sensors, just the new objective. If smoothness alone (phase 3 step 7, no kamikaze yet) materially reduces hull-strike rate, kamikaze (step 8) may be unnecessary or can be set milder than originally planned.
