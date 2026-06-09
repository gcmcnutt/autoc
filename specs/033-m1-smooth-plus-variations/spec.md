# Feature Specification: 033 — M1 smoothness + replay-friendly variation PRNGs (+ kamikaze inherit)

**Feature Branch**: `033-m1-smooth-plus-variations` (proposed; created 2026-05-19)
**Created**: 2026-05-19
**Status**: DRAFT — operator-initiated during 032 phase-1 bake monitoring

## Clarifications

### Session 2026-05-21
- Q: Legacy `windSeed` field disposition after 033 phase 1? → A: **Remove `windSeed` in 033 PR**. Delete the field from `ScenarioMetadata` entirely. `windPRNG.next()` (derived from `scenarioSeed`) becomes the sole wind seed source. Per Constitution III (no shims, no vestigial coexistence). Cereal length-mismatch loud-fail on old dmps is already expected for the `scenarioSeedList` add; the `windSeed` removal piggybacks on the same break.
- Q: Enable-flag scope for inert classes (craft / camera) in 033 phase 1? → A: **Only wire the 3 currently-active classes** (wind / rabbit / entry) with `Enable<X>Variations=0` draw-and-discard semantics. Do NOT add `EnableCraftVariations` / `EnableCameraVariations` knobs now. `craftPRNG` and `cameraPRNG` are still SEEDED at scenario init (slots 3 and 4 of the `scenarioRoot.next()` chain per T020) but have no consumer; the append-only contract is preserved because the SEED derivation order is fixed in 033. When 025 (craft) and 034 (camera) un-park, their PRs add the Enable knob + draw-and-discard + consumer atomically — wind/rabbit/entry seeds and replays of pre-025/034 bakes are unaffected.

### Session 2026-05-20
- Q: Phase-1 success criterion for "reasonably tracking" — what's the M1 floor? → A: M1-specific (not M2). Three concrete gates: (i) **continued learning** (best-fitness still trending UP gen-over-gen at the smoothness-on plateau — no "do nothing" stall), (ii) **streak% ≥ 2/3 of 029 pastonly3 baseline streak%** (the smoothness penalty discounts stepPoints up to 50% but streak compound still applies; if controller is too sluggish to maintain streaks, this drops; 2/3 is the iterate-or-advance gate), (iii) **best-fitness in same range as 029 baseline** (not catastrophically lower). Materially better per-axis aggressiveness numbers complete the picture.
- Q: PRNG sub-consumer reset boundary — per-gen / streaming / per-scenario? → A: **Per-scenario** — every scenario, ALL sub-PRNGs reseed from a scenario-derived seed (tied to master). Inside a scenario, sub-PRNGs advance normally. Scenario K is bit-identical every time it runs across gens, modes, and variation-enable states. M2's `rabbitPRNG` (rabbit speed in M1, crash-hull in M2) and `windPRNG` draw the same values as M1 — only what's USED differs, not what's DRAWN. Eval pipeline becomes bit-deterministic on (NN, scenario-seed). NN-evolution PRNG is a separate stream, unaffected by scenario sub-PRNGs.
- Q: Master-seed default + init chain? → A: **Keep `Seed = -1` (time-based)** as today's default; `Seed = N` reproduces a prior run by copying the printed seed. Init chain: `MasterPRNG.init(masterSeed)` → first `.next()` seeds the autoc-side NN-evolution PRNG → next M draws produce `scenarioSeed[0..M-1]`. Per scenario K: scenarioSeed[K] seeds a local scenario-master-PRNG → consume N values to seed the N variation sub-PRNGs (slot-by-slot). Sub-PRNGs then advance normally during the scenario. Adding a new variation slot doesn't disturb existing seeds.
- Q: Smoothness motion aggregate default (Pythagorean / sum / max)? → A: **Pythagorean** `sqrt(Δpt² + Δrl² + Δth²)` (L2 norm). `motion_max = sqrt(12) ≈ 3.46`. Single-axis bang-bang gets factor ≈ 0.71; all-3 simultaneous bang-bang gets the full 0.5 penalty. Standard L2 "magnitude-of-motion" interpretation; smoother gradient than L1 sum or L∞ max. `SmoothnessMotionMode` ini knob retained for later experimentation but default is Pythagorean.
- Q: Three operator-volunteered refinements (placement, sub-PRNG structure, dmp recording)? → A: **(a) Penalty placement = worker-side** at the per-step fitness computation site (where `stepPoints` is computed, before `applyStreak`). No autoc-side state needed; per-tick local computation from NN output Δ. **(b) Sub-PRNG structure organized by VARIATION CLASS not per-variable slot**: `master → {autoc PRNG, scenarioSeed[K] → {wind PRNG, rabbit PRNG, entry PRNG, craft PRNG (future), camera PRNG (future)}}`. Multiple variables within a class share that class's PRNG; adding a new variable in an existing class doesn't disturb existing seeds; adding a new CLASS appends to the end. **(c) Record `scenarioSeed[K]` in the .dmp** per-scenario, alongside the existing variation-ramp scale field. Scenario seed alone is sufficient for full replay (workers re-derive class PRNGs deterministically from it); no need to record per-class sub-seeds. **(d) M2 phase-2 success criteria** intentionally deferred — phase 1 may not work and phase-2 design lands on phase-1 data.

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

**Solution**: master-seed → per-scenario sub-PRNG architecture (clarified 2026-05-20).

**Core determinism contract**: scenario K must be **bit-identical** every time it runs, regardless of gen, mode, or which variation features are enabled. Same wind, same rabbit profile, same future craft/camera variations. This is the gate for full reproducibility — "same NN + same scenario seed → bit-deterministic output."

**Init chain** (clarified 2026-05-20 — keeps existing `Seed = -1 | N` ini convention; sub-PRNGs organized by VARIATION CLASS, not per-variable slot):

```
Seed (autoc.ini / autoc-tracker.ini)
  ├─ -1 → use wall clock; effective seed value is printed at run start (operator copies into a later eval-mode ini to reproduce)
  └─ N  → use N directly (typically copied from a prior bake's startup line)

masterSeed = effective Seed value
  → MasterPRNG.init(masterSeed)
    → MasterPRNG.next() → seed for autoc-side NN-evolution PRNG (population init, mutation, crossover, selection — all unrelated to scenario variation)
    → MasterPRNG.next() repeated M times → scenarioSeed[0..M-1] (one seed per scenario in the table; M = num_scenarios)

per scenario K (on the worker side, every time scenario K runs):
  scenarioSeed[K] → seed local "scenario root PRNG" → derive one sub-PRNG seed per VARIATION CLASS:
    windPRNG.seed   = scenarioRootPRNG.next()    # wind class — direction, strength, gust profile, vortex placement all consume from this single stream
    rabbitPRNG.seed = scenarioRootPRNG.next()    # rabbit class — speed, hull (M2), entry-rabbit-state details
    craftPRNG.seed  = scenarioRootPRNG.next()    # craft class (future 025 work) — mass, drag, prop perturbations
    cameraPRNG.seed = scenarioRootPRNG.next()    # camera class (future 034/035 work) — FOV jitter, mount offset, lens distortion
    # entry-pose draws (entry cone, entry roll, entry speed, entry position) consume from a dedicated "entry" PRNG OR are grouped with one of the above — plan-phase decides
  Each class PRNG advances normally as its variation type consumes draws during the scenario.
```

**Key change vs earlier slot-per-variable framing**: sub-PRNGs are organized by VARIATION CLASS (wind, rabbit, craft, camera, ...). Multiple variables within a class share one sub-PRNG (e.g., wind direction + wind strength + per-tick gust profile all consume from windPRNG). Adding a new variable in an existing class adds another draw inside that class's PRNG — doesn't add a new sub-PRNG seed slot. Adding a new CLASS adds a slot.

Key properties guaranteed by this chain:
- **`Seed = -1` ↔ `Seed = <printed-value>` round-trips**: copy the value from training run's first log line into eval mode's ini → bit-identical replay (today's contract; 033 preserves it)
- **NN-evolution PRNG and variation PRNG never cross streams**: separate first-draw and separate downstream consumers
- **Adding a new variation slot (N+1)** just consumes one more `scenarioMasterPRNG.next()` per scenario; doesn't disturb earlier slots; doesn't change existing scenario seeds. Future craft/camera additions slot in cleanly
- **Cross-mode replay**: M1 and M2 of scenario K both initialize subPRNG[K] from the same `scenarioSeed[K]` → same wind, etc. Only the SLOT-CONSUMER differs (M1 uses `rabbitPRNG` for rabbit-speed segments; M2 uses `rabbitPRNG` for crash-hull. Both modes seed it identically; only downstream consumption diverges per-mode)
- **Scenario seed table**: 033 introduces `scenarioSeed[K]` as a `uint64_t` field on `ScenarioMetadata`. The legacy `windSeed` field (the pre-033 per-scenario seed source for CRRCSim wind) is **removed in the same PR** — `windPRNG.next()` (derived from `scenarioSeed[K]`) becomes the sole wind seed source. Per Constitution III, no coexistence / shim path.
- **Sub-PRNG classes** (variation classes per scenario; each class is its own deterministic stream, seeded from `scenarioRootPRNG.next()`):
  | Class | What it draws | Active in M1 | Active in M2 |
  |---|---|---|---|
  | wind   | Wind direction + strength + per-tick gust profile + thermal vortex placement | ✓ | ✓ |
  | rabbit | Rabbit/target speed segments + crash-hull PRNG (when hull is on) | ✓ | ✓ (hull-only effective) |
  | entry  | Entry cone sigma + entry roll + entry speed + entry position | ✓ | ✓ |
  | craft  | Mass / drag / prop perturbations (currently inert; future 025 work) | future | future |
  | camera | FOV / mount offset / lens distortion perturbations (currently inert; future 034/035 work) | n/a (no camera) | future |
  | reserved | Future variation classes — sim-tick jitter, etc. — append-only | | |

  **Append-only contract**: adding a new variation CLASS appends a new sub-PRNG seed draw at the END of the per-scenario init sequence. Existing classes' seeds (and therefore their per-scenario values) don't change. Reproducibility of old runs preserved.

- **Reset boundary** (the architecture-defining decision): at the **start of each scenario**, ALL sub-PRNGs are reseeded from the scenario seed (e.g., `subPRNG[K].seed = derive(scenarioSeed, K)`). **NOT per-gen, NOT streaming across gens** — strictly per-scenario.

- **Within scenario**: each sub-PRNG advances normally as its variation type consumes draws. Wind sub-PRNG (slot 0) might be tapped every sim tick for gust profile; rabbit-speed sub-PRNG (slot 7) might be tapped a few times per scenario for piecewise speed segments. Variable per-scenario consumption is fine — the sub-PRNG just advances naturally.

- **Result — full per-scenario determinism**:
  - Scenario K's wind/rabbit/entry/etc. are bit-identical at every gen
  - Scenario K seen by M2 has the **same wind** as scenario K seen by M1 (because `windPRNG` is seeded identically regardless of mode; only `rabbitPRNG` consumers differ in what's USED, not what's DRAWN)
  - Two runs with same master seed produce identical training trajectories (modulo NN-evolution PRNG noise — see below)

- **Wind replay within a scenario**: CRRCSim's wind currently uses a per-frame stochastic process from a separate PRNG. Under 033, CRRCSim's wind PRNG is reseeded **at scenario start** from `windPRNG` (which itself was just reset from scenario seed). Replays of the same scenario produce identical wind gust profiles tick-by-tick.

- **NN-evolution PRNG is separate**: autoc-side population creation, mutation, crossover, and selection consume their own PRNG stream (unrelated to scenario variation). It's seeded from master at autoc startup but is otherwise unaffected by 033's scenario sub-PRNG architecture. Wire-protocol-wise, autoc sends (NN bytes, scenario seed) to the worker; the worker reconstructs all scenario PRNGs from that seed alone. NN PRNG and variation PRNG never cross streams.

- **Eval determinism corollary**: same NN + same scenario seed through the eval pipeline → bit-deterministic output. This is the regression-test contract — `eval_results(NN, scenarioK) === eval_results(NN, scenarioK)` always.

- **Per-scenario seed recorded in .dmp** (clarified 2026-05-20): every scenario's record in the .dmp gets its `scenarioSeed[K]` value persisted, **alongside the existing per-scenario variation-ramp scale field** (`computeVariationScale()` result, currently recorded per spec [019] / [030] convention). The scenario seed is sufficient for full replay — given (NN bytes, scenarioSeed[K]), the worker can deterministically re-init all class PRNGs and reproduce the scenario bit-for-bit. We don't separately record per-class sub-seeds because they're trivially derivable from the scenario seed.

### 2.B M1 smoothness fitness penalty — multiplicative on stepPoints

**Goal**: penalize bang-bang within the M1 objective function so evolution selects for smooth controllers — without throwing away the streak-multiplier dynamics that drive convergence.

**Operator-preferred form (2026-05-19)**: a multiplicative penalty factor on per-tick `stepPoints` BEFORE the streak multiplier is applied. Smooth controllers get full credit (×1); bang-bang controllers get half credit (×0.5).

**Placement (clarified 2026-05-20)**: **worker-side**, at the per-step fitness computation site (where `stepPoints` is computed and before `applyStreak` runs). The penalty derivation is local to the per-tick NN output Δ — no autoc-side state needed. Worker computes `final_step_score = step_score × smoothness_factor × streak_multiplier` per tick; autoc-side fitness aggregation receives the already-discounted scores. Implementation site: `FitnessComputer::applyStreak()` in [src/eval/fitness_computer.cc](../../src/eval/fitness_computer.cc) (or its per-tick caller) — plan-phase confirms exact insertion point.

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
- **Tunable form**: ini knob `SmoothnessPenaltyFloor` (default 0.5; raise to 0.7 for mild penalty, lower to 0.3 for harsh) + `SmoothnessMotionMode` (default `"pythagorean"` per /clarify 2026-05-20; alternatives `"sum"` / `"max"` available as knobs but not phase-1 default).
- **Single knob**, axis-agnostic — no per-axis tuning needed. If roll dominates bang-bang historically, that's where most of the penalty lands automatically.

**Test design**: instrument the existing 030/032 bake data.dat per-tick to compute what `smoothness_factor` distribution looks like with `floor=0.5`. Expectation: late-gen elites with bang-bang signature will see floors of 0.5-0.7 per tick on bang-bang axes, costing significant fraction of streak compound. That's the signal evolution should follow.

**Alternative (deferred)** Path B — lexicase smoothness test case (the [027 path](../027-recurrent-nn/spec.md)). 027/028 tried this; results were mixed. Path A is preferred per simplicity + the BACKLOG endorsement.

**Success criterion**: the per-axis aggressiveness chart ([per_axis_aggressiveness.py](../030-tracker-mode/per_axis_aggressiveness.py)) shows dCtrl ≤ 0.27 per-axis with sum-over-axes ≤ 0.80 (the existing 028 spec-gate budget). Phase-1 readiness gate: those budgets met at training plateau AND held through eval-reeval AND replayed cleanly in the mezzanine real flight.

**Concrete baseline for A/B (M1-flavored, since phase 1 bakes M1)**: §1.3 establishes that M1 pastonly3 (029) and M2 032-phase-1 closeout produce **the same control-aggressiveness signature** (same "room" of the per-axis chart — both bang-bang, both well outside the 028 spec-gate budget). That paired baseline is what 033's smoothness experiment is compared against — not an estimate, not a target — concrete measured numbers from two existing runs.

Phase-1 success criteria (clarified 2026-05-20 — **M1-specific, not M2**):
- **Continued learning** — best-fitness curve still trends UP gen-over-gen at the smoothness-on plateau. No "controller learns to do nothing" failure mode where evolution stalls or regresses.
- **Streak% ≥ 2/3 of 029 pastonly3 baseline streak%** — the smoothness penalty discounts per-tick stepPoints by up to 50%, but streak-multiplier dynamics still apply. If the controller becomes too sluggish to maintain streaks (because it's not banking quickly enough to chase rabbit turns), streak% drops. The 2/3-of-baseline floor is the "still tracking acceptably" gate. Iterate on floor / motion-mode if streak% falls below.
- **Best-fitness in the same range as 029 pastonly3 baseline** — not catastrophically lower. The smoothness term reshapes the fitness landscape but shouldn't lower the achievable maximum by an order of magnitude.
- **Materially better per-axis aggressiveness numbers** vs the 029 pastonly3 baseline — dCtrl ≤ 0.27 budget being the tight target, but even partial movement (e.g., dropping from sum ≈1.4 toward 0.8) is informative phase-1 signal.
- Tracking-vs-smoothness frontier is acceptable: a slight tracking degradation in exchange for major smoothness gain is the desired tradeoff for real-flight deployment, since the 0517-flight evidence was that the bang-bang signature is the actual airframe-flyability blocker, not the tracking number.

**M2 phase-2 criteria** (deferred to /clarify before phase 2 starts): expected to be avgInRamp-based against the 032 baseline (0.158), but precise threshold set after phase 1 lands and we know whether smoothness shifted M2 baselines too.

**Mezzanine test — real flight**: per operator routing 2026-05-19, the real-flight validation is the gate between 033 phase 1 (smoothness alone in M1) and 033 phase 2 (M2 work). A successful real flight with reduced bang-bang signature on the airframe is the qualifier to move on.

### 2.C M2 inherits the smoothness objective

Same `SmoothnessPenaltyFloor` knob (per §2.B) applies to M2 fitness. No code change beyond what 2B introduces — the per-tick `Δoutput` penalty is mode-agnostic (just reads the NN's outputs, identical for M1 and M2).

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
