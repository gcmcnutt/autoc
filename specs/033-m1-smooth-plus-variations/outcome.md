# 033 — Outcome / Phase-1 Wrap Summary

**Result**: ✅ **WRAP** — US1 (replay PRNG architecture) shipped and validated; US2 (M1 smoothness penalty) **scalar approach failed and is stubbed** (plumbing intact, configured no-op); US3/US4 remain deferred. The genuinely-hard part (a working secondary objective) is **re-scoped to a follow-on feature** — and the objective is now believed to be **energy minimization, not smoothness** (see §4).

**Closeout date**: 2026-05-29. **Branch**: `033-m1-smooth-plus-variations` @ `5465cb7` (merge of 032 into 033). **Decision**: do NOT abandon 033 → continue-from-032 (that would discard the validated PRNG rework). Wrap 033, keep the PRNG work as the new baseline, defer the multi-objective challenge.

---

## 1. Bake summary — the 033 closeout run

`pop=8000 / wind=36 / paths=6` on 033 code, fresh seed `1779919143`, smoothness off (floor=1.0). This run doubled as (a) PRNG-rework validation on 033 code and (b) the improved-baseline confirmation.

| gen | Best | per-scenario | avgMaxStreak | pctInStreak | Sigma |
|----:|-----:|-------------:|-------------:|------------:|------:|
| 150 | −12,535 | −58 | 8.3 | 7.4% | 0.174 |
| 280 | −22,808 | −106 | 23.7 | 24.4% | 0.121 |
| 437 | −37,066 | −172 | 36.2 | 38.1% | 0.075 |
| 567 | −39,948 | −185 | 37.9 | 39.4% | 0.053 (floor) |

Reference — pastonly3 (029, pop5000/wind49) at gen 600: Best −55,937 → **−190/scenario**, avgMaxStreak 38.2, pctInStreak 40.7%. The 033 run is **within ~3% per-scenario** and matching streak metrics — a strong, pastonly2/3-class climber. (Absolute Best differs because Best is a sum over scenarios: 216 here vs 294 for pastonly3 — always compare per-scenario.)

## 2. US1 — Replay-friendly variation PRNG architecture (P1) — ✅ SHIPPED + VALIDATED

The master-seed → `ScenarioRootPRNG(scenarioSeed)` → 5 class sub-PRNGs (wind/rabbit/entry/craft/camera via `deriveClassSubSeeds()`) rework (commit `acf732f`).

- **Climbing-basin validation**: the closeout run above is a strong climber on 033's reworked cascade — the phases 1–4 stalls were the **basin lottery** (unlucky seeds + pop=5000), NOT a PRNG bug. See BACKLOG "M1 basin-landscape" entry.
- **Bit-exact replay validated 2026-05-28**: M1→M1 eval of a gen-437 weight reproduced the stored fitness to the cent (−37065.799573), concurrent with training, no file collision (eval writes `eval-data.*`). Confirmed the variation-ramp scale travels **with the recorded scenario** — replay is gen-agnostic, works mid-bake at any gen.
- **Decision (2026-05-29)**: keep the multi-PRNG. It doesn't hurt (replay worked pre-033 with a single seed too) and it **adds debuggability** (per-class sub-seed replay/inspection). Retained as the new M1 baseline.

## 3. US2 — M1 multiplicative smoothness penalty (P1) — ❌ SCALAR APPROACH FAILED, STUBBED

Per-tick smoothness factor (Pythagorean motion default) multiplied onto `stepPoints` inside `FitnessComputer::applyStreak()` before the streak multiplier.

- **Failure**: phases 1–3 (floor 0.5 / 1.0) stalled hard (gen-457 kill at best −13,979 vs −50,918 baseline; streak% 1/11 of baseline). Phase-4 bypassed the term entirely and *still* stalled until the basin-lottery diagnosis. Root cause is **not a bug** — it's [project_scalar_multiobjective_collapse]: a scalar-aggregated multi-objective (smoothness × tracking jammed into one number) drives the controller to one Pareto corner ("a-or-b, not a-mix") and evolution stalls.
- **Current state**: plumbing is **fully wired across every transport** but configured as a **no-op** (`SmoothnessPenaltyFloor=1.0`):
  - `WorkerInit` + `EvalData`: `smoothnessPenaltyFloor` + `smoothnessMotionMode` ([protocol.h:203,399](../../include/autoc/rpc/protocol.h#L203))
  - `AircraftState.smoothnessFactor_` → serialized into the dmp + data.dat `smooth` column ([aircraft_state.h:587](../../include/autoc/eval/aircraft_state.h#L587))
  - crrcsim worker computes per-tick in **both** pathgen and tracker branches ([inputdev_autoc.cpp:1086](../../crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp#L1086))
  - `compute_smoothness_factor` in `derived_features.h`; `applyStreak` smoothness_factor param
- **Verdict**: the scalar-penalty *mechanism* is a dead end. See §4 for why the *plumbing itself* may also be the wrong scaffolding.

## 4. KEY REFRAME — the secondary objective is energy, not smoothness (2026-05-29)

The 033 premise was "penalize bang-bang via a smoothness/total-motion factor." On reflection the real objective is **minimize energy**, and that is **already plumbed independently of the smoothness work**:

- `ScenarioScore::energy_score` = `Σ(out_th − 1)/2` per tick, "already negative; lower = better" ([fitness_decomposition.cc:192,287](../../src/eval/fitness_decomposition.cc#L192)). It's the `energy=` column in every data.stc `#NNGen` line.
- `ScenarioScore::stability_score` = `Σ(|out_pt|−1)+(|out_rl|−1)` similarly plumbed.
- Both have lexicase-pool entries **commented out one line each** at [selection.cc:68-69](../../src/eval/selection.cc#L68-L69) (CADENCE7-REDUX diagnostic disabled them). Re-enabling energy as an objective is a one-line uncomment — and it attacks the throttle-peg stuck basin directly (a throttle-pegged individual fails the energy test cases).

**Implication**: if energy minimization (throttle proxy, or richer total-energy = altitude+airspeed) is the objective, **the entire 033 smoothness apparatus is unused** — fully wired but never the thing being optimized. It becomes removal-candidate dead code, not reusable scaffolding. This flips the earlier "keep the plumbing for future lexicase smoothness" recommendation.

## 5. Improved baseline — pop=8000 / wind=36

Adopted as the new pathgen default (committed to `autoc.ini`). Climber rate **2/3 (r1 strong, r2 slow-then-strong, r3 stuck)** vs the recent **1/4** at pop=5000/wind=49. Improves but does NOT eliminate the basin lottery (~1:3 stuck rate is intrinsic to current topology+evolution settings). Full detail + the multi-seed protocol in BACKLOG "M1 basin-landscape protocol" entry. Reference `.stc` files: `specs/032-tracker-nn-enhancements/pop8000-wind36-r{1,2,3}-data.stc`.

## 6. What ships in the 033 wrap

- **US1 PRNG architecture** — retained, new M1 baseline.
- **US2 smoothness plumbing** — present, stubbed no-op (floor=1.0). NOT removed in this wrap; removal vs keep-inert is a 034 decision (see §7).
- **pop=8000/wind=36 baseline** — committed to `autoc.ini`.
- **US3 (M2 inherits smoothness) + US4 (kamikaze penalty)** — remain deferred (were always "033 phase 2").

## 7. Deferred to 034 (cleanup + the real multi-objective)

See the dedicated proposal below (§8). Headline: decide smoothness-plumbing fate, re-enable energy as the actual secondary objective via lexicase (not scalar penalty), and clear the Phase-7 housekeeping (T056–T058).

## 8. Open task routing

**Closed/superseded in 033:**
- T053 (this outcome doc) — done.
- US2 smoothness as a scalar penalty — superseded; do not pursue scalar composition again ([project_scalar_multiobjective_collapse]).

**Deferred to 034 (proposed):**
- **Smoothness-plumbing decision**: remove (if energy-only direction confirmed) vs leave inert. Lean: remove, since energy is the objective and dead no-op plumbing across 5 transport boundaries is maintenance drag. If kept, document as "inert, reserved."
- **Energy as the secondary objective via lexicase**: uncomment [selection.cc:69](../../src/eval/selection.cc#L69) (energy) and optionally :68 (stability); group axes physically per [project_smoothness_axis_grouping] (bank=pitch+roll, throttle/energy separate); NOT scalar penalty. Re-enable the 4 `DISABLED_` Selection027 tests.
- **Total-energy richer objective** (altitude+airspeed) vs throttle-proxy — see BACKLOG "[DEFERRED] Total Energy Management."
- **T056** — `gScenarioVariations` per-(path,wind) instead of per-wind.
- **T057** — startup banner prints every active `AutocConfig` key (the "we'd see it if it were logged" gap bit us repeatedly this cycle).
- **T058** — right-size seed widths 64→32-bit to match Park-Miller entropy.
- **T054** — `gp_scalar` type audit on the US2 surface (moot if US2 smoothness removed).
- **T051/T052** — CLAUDE.md + doc cross-links.

**Remains in 033 phase 2 (gated on M2 mezzanine flight pass):**
- US3 (T043–T045), US4 (T046–T050).

## 9. Headline takeaways for next-feature planning

1. **Basin lottery is intrinsic** (~1:3 stuck) at current topology+evolution settings — a factory throughput tax re-paid on every M1/M2 iteration. Cheap first attack = energy lexicase axis (kills throttle-peg basin directly); bigger lift = demetic/island GA (BACKLOG research entry).
2. **Scalar multi-objective composition is banned** — use lexicase/Pareto.
3. **The secondary objective is energy, and it's already plumbed** — the 033 smoothness detour built the wrong scaffolding.
4. **Multi-PRNG replay is a factory tool** (regression/eval/debug), validated bit-exact — NOT a training-realism mechanism. M2 training should use random/independent variations, not source-environment replay (see BACKLOG M2 bullet).
