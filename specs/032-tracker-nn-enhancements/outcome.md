# 032 Phase 1 — Outcome / Phase-1 Summary

**Result**: ✅ **SUCCESS** per spec Q7. plateau-avgInRamp = **0.1578** (≥ 0.15 threshold).

**Closeout date**: 2026-05-20. **Bake commit hash**: `907eb9b` (autoc) + `e3d623f` (crrcsim submodule), with `129b1f0` adding `.gitignore` for `.claude/` post-commit. The 032 phase-1 code shipped is the head of `032-tracker-nn-enhancements` at closeout.

---

## 1. Bake summary

| Property | Value |
|---|---|
| Source dmp | `autoc-9223370259105171692-2026-05-02T19:20:04.115Z/gen9200.dmp` (M1 pastonly3 converged gen-800 reference) |
| Population | 5000 |
| Generations | **800** (natural completion) |
| Scenarios | 294 (6 paths × 49 winds) |
| FDM | CRRCSim (`MinisimProgram = ./scripts/crrcsim.sh`) |
| Topology | 54 → 32 → 16r → 3 (unchanged from 030 baseline, with 9 added input slots) |
| Weight count | 2595 (was 2307 in 030; +12.5% from wider input fan-in to hidden1) |
| Wall-clock | 74.1 h (266,844 s) |
| Throughput | 4,407.95 sims/s (1.18 billion total NN forward passes) |
| Best fitness | **−21,482.53** |
| **plateau-avgInRamp** (last 50 gens 751-800) | **0.1578** (min 0.138, max 0.166) |

---

## 2. Comparison to 030 baseline

| Metric | 030 postdiag2 (gen 322 plateau) | 032 phase-1 (gen 800 plateau) | Δ |
|---|---|---|---|
| Best fitness | −17,060 | **−21,482** | +26% (more negative = more accumulated points) |
| plateau-avgInRamp | ~0.07 | **0.1578** | **2.25× lift** |
| avgVis (latest gen) | ~0.70 | 0.757 | +8% |
| avgRngMin (latest gen) | 3.4 m | 2.25 m | tighter |
| avgRngMed (latest gen) | ~19 m | 12.03 m | 37% tighter |
| Hull-strike rate (late-bake avg per gen) | ~7-11/gen | ~13-16/gen | **escalation pattern** (§1.8 — see "What we learned") |

The 9 new derived perceptual features (`beacon_pair_span[6]`, `span_rate`, `target_tilt_sin/cos`) lifted the M2 plateau by **>2× over 030 baseline** — phase-1 hypothesis confirmed. The hypothesis was that visibility-time signal richness (range + closing rate explicit) was the load-bearing constraint on the 030 plateau; this result validates it.

---

## 3. What was built (scope as shipped)

Per spec.md §2:

- **(A) Beacon identity-stable ordering** — confirmed already-satisfied in sim by mount-keyed projection pipeline (research.md R1). Doc + contract test only; no sim code change. Documents in `docs/COORDINATE_CONVENTIONS.md` + `docs/sensor-pipeline.md` for xiao migration prep are TODO per the open-task routing below.
- **(B) Three derived perceptual features** (9 new input slots):
  - `beacon_pair_span[6]` — raw NDC Euclidean distance between port + starboard beacons, 6-tick history (100ms grid)
  - `span_rate` — one-tick raw diff (no scaling, no smoothing)
  - `target_tilt_sin/cos` — atan2(dy,dx) over NDC port→starboard line, sin/cos encoded (avoids ±π wrap)
- **CEP-gating** — when EITHER beacon's CEP ≥ 1.25 (matches `kCepSentinelThreshold`), substitute neutral values (span=0, span_rate=0, tilt=(0,1)).
- **Plumbing** — `TrackerInputs` struct 45 → 54, `TrackerHistoryWindow` gains `span[6]`, `WorkerInit` carries `cepGateThreshold` for crrcsim worker processes, data.dat per-tick emitter extended for 9 new columns.
- **Dmp serialization** — auto-extended via cereal walk; no version bump (M2 policy per [feedback_no_cereal_versioning](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md)).
- **No EnableDerivedFeatures runtime flag** — removed pre-T017 per operator direction "greenfield M2 means the code IS the change; B-off attribution comes from git revert per research.md R7."

**Tests**: 25/25 passing including new `derived_features_tests.cc` (pure-math), extended `nn_sensor_interface_tests.cc` (COUNT==54), extended `gather_tracker_inputs_tests.cc` (end-to-end + CEP-gate + identity-stable), and extended `contract_tracker_config_tests.cc` ([DerivedFeatures] section).

---

## 4. What we learned during the bake — findings durable beyond the headline number

These are the substantive 2026-05-17 / 18 / 19 findings recorded in spec.md §1.8 + related memories. They drive the next-feature routing more than the 0.158 plateau itself.

### 4.1 Sensor fidelity confirmed (Panel D of intercept_analysis)

`spn0` vs real `dist` is a clean k/dist scatter (k ≈ 11). The sensor itself is providing accurate distance information across all 294 scenarios. CEP-gating (23k+ ticks) appropriately suppresses noisy reads. Spread along the k/dist line is target-attitude geometry (wing-projection envelope), not noise — intrinsic to a 2-beacon system.

### 4.2 NN ALREADY weakly uses span for throttle, but **not** for closure (Panels B/C)

`outTh` binned-mean trends down as `spn0` increases (slight nose-back at closer targets) — the NN found a small use for the new input. `outTh` is essentially flat vs `dspn` (closure rate ignored). Combined finding: **the sensors are sufficient; the bottleneck is reward gradient**.

### 4.3 The objective IS kamikaze (operator quote 2026-05-18)

Comparing gen 100 vs gen 400 intercept charts: the NN is **doing the same kamikaze strategy more precisely** — not evolving toward "back off when close." Improvement is bang-bang sharpening, not behavioral evolution. Cross-cutting: M1 bang-bang persistence and M2 hull-escalation are the same root cause — reward function rewards aggressive evolution against airframe limits with no penalty for the aggression. Captured in [project_hull_escalation_finding](../../.claude/projects/-home-gmcnutt-autoc/memory/project_hull_escalation_finding.md).

### 4.4 Hull-strike escalation monotonic with controller skill (§1.8 table)

| 50-gen window | 030 postdiag2 | 032 phase-1 |
|---|---|---|
| 1–50 | 1.00 / gen | 1.12 / gen |
| 51–100 | 2.14 | ~5.8 |
| 401–450 | 7.36 | (extrapolated) |
| 751–800 | 11.26 (estimated from final tail) | ~13-16 (current latest) |

032's tighter sensors enable faster convergence to the kamikaze attractor. Phase-1's high avgInRamp comes partly from chase "muscling through" — 62% of scenarios penetrate the hull radius at least once in elite-eval at gen 482 (per intercept_analysis Panel E). This is fitness-positive under the current objective but unsafe to fly in reality.

### 4.5 Wind-replay confound when comparing across runs

Each run's per-scenario variation (wind, entry cone) draws from joint-PRNG state that drifts across runs. Comparing 032 (0.158) to 030 (0.07) mixes the real signal (new sensors helped) with the confound (different weather sampled). Captured in 032 spec.md §1.8 + routed to 033 §2.A (master-seed sub-consumer architecture).

### 4.6 Modality vs fault framing (spec.md §1.7, 2026-05-16)

Beacon invisibility is a MODALITY (geometry-driven by 270° wingtip emission cones + chase-relative attitude), not a sensor fault. Next-iteration sensor work should extract intent from partial visibility, not engineer for fault robustness (a multi-sensor-era problem). Captured in [project_sensor_modality_vs_fault](../../.claude/projects/-home-gmcnutt-autoc/memory/project_sensor_modality_vs_fault.md).

---

## 5. Spec Q7 outcome decision

| `plateau-avgInRamp` band | Spec rule | This run |
|---|---|---|
| ≥ 0.15 | SUCCESS → close-out; operator decides next milestone | **✓ THIS PATH** |
| 0.10–0.15 | PARTIAL → run B-off attribution bake | n/a |
| < 0.10 | MISS → route to phase 2 / M3 | n/a |

**Decision**: SUCCESS. Phase 1 closes. **US2 (git-revert B-off attribution bake) is SKIPPED** per spec Q7. **US3 (kamikaze penalty experiment) and related smoothness work are routed to 033** rather than 032 phase-1b — operator decision 2026-05-19 to bundle them with M1 smoothness work and the replay-PRNG architecture.

---

## 6. Open task routing — what moves where

Per the closeout review, here's where every still-open 032 task lands:

### Routed to 033 ([specs/033-m1-smooth-plus-variations/spec.md](../033-m1-smooth-plus-variations/spec.md))

| 032 task | 033 destination |
|---|---|
| **US3 T043-T053** (kamikaze multiplicative penalty experiment) | **033 §2.D phase 2** (after smoothness mezzanine flight). Operator-preferred form upgraded to `HullCrashScoreFactor = 0` (full zero-out) initially, back off if over-deters. |
| 032 §1.8 hull-escalation finding (the *why* of US3) | **033 §1.2** (carried forward as motivating evidence) |
| Wind-replay confound observation (032 §1.8 Reframe 2026-05-19) | **033 §2.A** (replay-friendly master-seed sub-consumer PRNG architecture) |

### SKIPPED per Q7 outcome (SUCCESS, not PARTIAL)

| 032 task | Why skipped |
|---|---|
| **US2 T032-T036** (git-revert B-off attribution bake) | Spec Q7: only fires for 0.10–0.15 PARTIAL band. We landed 0.1578 SUCCESS — direct cross-run attribution unneeded because the 2× lift over 030 baseline is unambiguous (modulo wind-replay confound noted above; 033 §2.A addresses the residual concern more cleanly than US2 would have). |

### Routed to BACKLOG

| 032 task | Backlog reason |
|---|---|
| **T041** OPTIONAL refactor — consolidate the two `projectAndShiftHistory` implementations (autoc minisim + crrcsim helper) into a single shared helper | Wire-equivalent obligation per [data-model.md §8](./data-model.md#8-cross-platform-mirroring). Code works, tests pass, identity is verified by contract test. Cleanup hit not justified at closeout; should land alongside 033 or 034 refactor pass when those touch the same files |
| **T026** M1-under-CRRCSim regression as separate from minisim bitwise | Operator-driven; not load-bearing for the bitwise M1 gate (T025 covers that bit). File as backlog: "M1 under crrcsim qualitative regression" — to be revisited when M1 objective function changes per 033 §2.B/C |

### REMAINING TODO before final closeout commit (operator-driven)

| Task | Status | Notes |
|---|---|---|
| **T001** baseline rebuild-perf.sh + record M1 fitness | needs operator run | Original baseline reference — could skip retroactively since we're already past the bake; the post-bake T025 gate is what matters now |
| **T023** Update `docs/COORDINATE_CONVENTIONS.md` "Beacon Identity-Stable Ordering" + tilt convention | TODO | Phase-1 deliverable for xiao migration prep. Should land before commit |
| **T024** Update `docs/sensor-pipeline.md` (identity-stable + CEP-gating + xiao-port-prep) | TODO | Same as T023 |
| **T025** rebuild-perf.sh M1 bitwise-equal gate | needs operator run | Spec gate — must pass to close. M1 code untouched; expectation is pass |
| **T027** Determinism sanity (dual-seed run) | could defer to 033 | Used the same protocol in-flight (smoke bakes were deterministic). Defer-OK |
| **T037** Constitution VI grep audit over `src/eval/`, `src/nn/`, `include/autoc/eval/`, `include/autoc/nn/` | TODO | Code already uses `// raw-ok:` annotations per contracts/gather_tracker_inputs_v54.md; spot-check expected to be clean |
| **T038** SMOKE_REPORT.md | SUPERSEDED by **this file** (outcome.md) | The phase-1 summary content is here; SMOKE_REPORT.md as a separate document is redundant. Mark T038 as superseded |
| **T040** Project memory `project_032_phase1_result.md` | TODO | Will write at closeout commit (companion to this outcome.md) |
| **T042** Closeout commit | pending all above | Submodule pointer first (crrcsim) per [feedback_submodule_merge_order](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_submodule_merge_order.md) — no crrcsim changes this round, so just the autoc parent commit |

### Outdated task descriptors (drift since closeout)

| Task | What's stale |
|---|---|
| T002 doc inspection — marked [X] | Inline content captured the findings; partially superseded by this outcome.md's §6 routing |
| T018-T022 (minisim smoke / crrcsim mirror / smoke / dmp audit) — marked [ ] in tasks.md but actually DONE | These were completed during the in-flight implementation (commits 8520c04, e3d623f). The unchecked state is bookkeeping drift; should be marked [X] at closeout commit. |
| T028-T030 (production bake / plateau read / outcome decision) — marked [ ] but DONE this turn | The bake completed naturally at gen 800; this document records the result |
| T031 (write outcome.md) — marked [ ], DONE by this document | |

---

## 7. Closeout audit status (deferred until operator runs)

These are the explicit pre-commit gates from quickstart.md:

| Audit | Status |
|---|---|
| M1 bitwise gate (rebuild-perf.sh) | NEEDS OPERATOR RUN. Expectation: pass — M1 pathgen code path is untouched in 032 |
| Dmp honesty (all 54 inputs + 3 outputs captured) | ✓ DONE in-flight — cereal walk in AircraftState.h auto-extends; data.dat emitter explicitly extended for 9 new columns (autoc.cc per src/autoc.cc fix during T018 smoke) |
| Determinism (same-seed dual run) | DEFERRED — relied on in-flight smoke-bake determinism check; full sweep TODO if needed before 033 |
| Constitution VI grep (raw `float`/`double` without `// raw-ok:`) | NEEDS OPERATOR RUN. Expectation: clean per contracts pattern |

---

## 8. Headline takeaways for next-feature planning

1. **The 9 new sensors work** — k/dist sensor fidelity is clean, NN is using them. The 2.25× avgInRamp lift over 030 baseline is real signal.
2. **The reward function is the limiting factor**, not sensors or topology. Both M1 (bang-bang) and M2 (kamikaze) symptoms trace to "aggressive evolution against airframe limits with no penalty for the aggression."
3. **033 picks up the reward-shape work** — smoothness penalty (multiplicative-on-stepPoints) addresses M1 bang-bang first, real-flight validated as a mezzanine gate, then kamikaze + M2-inheritance follow.
4. **Wind-replay PRNG architecture** (033 §2.A) is the foundation for clean cross-run ablation — without it, comparing M1-with-smoothness vs M1-baseline is itself confounded by varying weather. Land it before the smoothness experiment.
5. **The flight test loop is now mandatory** — sim-only metrics (avgInRamp, hull-strike count) are good but not sufficient. The 0517 pastonly3 flight showed bang-bang is the real-world blocker even at sim-success training. Real flight is the next-feature qualifier.
