# 040 Camera Redo — Outcome (T086)

**Date**: 2026-08-06 | **Spec**: [spec.md](spec.md) | **Tasks**: [tasks.md](tasks.md)

---

## Verdict — operator, 2026-08-06

> **040 gave us a better camera model, much closer to real. Training results compared to the prior
> camera are more or less the same — and that is the going concern.**

Both halves matter and neither is a disappointment.

The perception model got materially harder in every direction the feature touched — acquisition costs
3–6 ticks instead of zero, quality decays with range, obstruction is ON for the first time,
separation-derived range goes explicitly unavailable past ~28 m, the beacon separation shrank 17%, and
the camera now carries a per-scenario mechanical error envelope. **Competence did not fall.** The
controller came out closer to the target and holding it at least as well as the model that flattered
it. The honest reading is the one T084 reached: *the prior model's optimism was not buying the
controller anything real.*

The going concern is the flip side. If a large honesty change to perception moves the competence
numbers this little, then perception fidelity is **not** what is capping M2 — which is what
[[project_m2_tracking_ceiling]] said in 037 and what 040 has now tested from the other direction. The
cap is somewhere else, and 041 goes looking for it.

The novel-path eval run at wrap (T085, below) sharpens this rather than softening it. On 49 scenarios
neither controller had seen, the ±10°-camera-variation run and the no-variation run are
**indistinguishable** — and the same eval shows where the real gap is: going from trained geometry to
novel geometry leaves the standoff distribution almost untouched while **halving the fraction of time
inside 5 m**. The limiting skill is not perception fidelity and not mechanical robustness; it is
converting a held target into a close chase on geometry the controller has not memorized.

---

## SC-008 — the aggregate delta

Prior baseline is **038 t9** (the last M2 before this feature), all runs trained from the **same pinned
M1 source** (`autoc-m1 · autoc-9223370253553029228-2026-07-06T01:35:46.579Z/gen9200.dmp.zst`,
`retain=keep`), so the source is not a moving part. Each run at its own final generation.

| metric | 038-t9 g430 | **040-t1 g800** | **040-t4 g800** | t1 vs t9 | t4 vs t9 |
|---|---:|---:|---:|---:|---:|
| avgMaxStreak (ticks) | 21.10 | **23.60** | 18.60 | +11.8% | −11.8% |
| pctInStreak | 7.50% | **8.00%** | 7.40% | +6.7% | −1.3% |
| avgInRamp | 0.090 | **0.096** | 0.087 | +6.7% | −3.3% |
| avgRngMed | 16.86 m | 15.14 m | **14.73 m** | −10.2% | **−12.6%** |
| avgRngMin | 3.60 m | 3.14 m | **3.03 m** | −12.8% | **−15.8%** |
| avgVis | 0.795 | 0.789 | **0.817** | −0.8% | +2.8% |
| loss_total | 832 | 876 | 979 | +5.3% | +17.7% |

**Read**: on range the honest model is unambiguously better — t4 sits **2.1 m closer at the median**
and **0.57 m closer at the minimum** than the baseline it replaces, while seeing the beacon more of the
time. On streak occupancy it is a wash: t1 up ~7%, t4 down ~1%, both inside the run-to-run spread the
project has seen on this objective. Nothing here is a systemic failure, and no competence floor gates
this feature (spec §SC-008).

Per-path detail at t4 g800 (`score_by_path`): median tracking error 10.1 m (path 3) to 15.4 m
(path 5). Path 5 — the random-intercept geometry — is the floor at 15.4 m / 8% of ticks inside 5 m,
and it did **not** move between gen 585 and gen 800. The hardest geometry is where the remaining
distance is.

### The confound, stated rather than buried

**t1 and t4 did not optimize the same objective.** `68f64ab` landed between them and corrected a
one-tick target offset that had been in the M2 objective since 030 — the chase was being scored
against where the target would be at tick *k+1*, ~0.85 m against a 3.048 m intended trail, a **28%
error in the definition of the task**. A run against the corrected objective *should* sit closer and
*should* score streak occupancy differently, which is exactly the shape of the t1→t4 difference.

Therefore: **the t1↔t4 delta is not attributable to camera variation**, and no number in this document
should be read as "camera variation costs X". The mid-run read at gen ~330 (`e932e28`, "camera
variation costs ~nothing") was taken before the run converged and before this confound was fully
weighed; it does not survive as a quantitative claim. What survives is the qualitative one: **a run
carrying up to 10° of camera misalignment in every scenario reached the same competence room as the
runs that did not**, and it did so while the perception model got harder in five other ways.

The clean experiment — t1′ with variation off on current code — is **deferred to the backlog**. It
answers a question 040's success criteria never asked, at ~53 hours of bake.

---

## T085 — the novel-path eval (run 2026-08-06)

Two stages, both against the **pinned M1** every 040 M2 bake trained on
(`autoc-m1 · autoc-9223370253553029228-…/gen9200.dmp.zst`, gen 800, fitness −41083.52):

1. **M1 flies novel geometry.** `./build/autoc -i autoc-eval.ini` → 7 **random** paths × 7 winds =
   **49 novel scenarios** with every variation class live (entry cone/roll/speed, wind direction,
   craft CG/drag/trim/thrust/effectiveness/servo/τ, rabbit speed). **49/49 RabbitComplete, no crashes.**
   Log `logs/autoc-040-t5-m1-novel-eval.log`, master seed **1786035214**, source
   `autoc-eval · autoc-9223370250819561192-2026-08-06T16:53:34.615Z/gen9999.dmp.zst`.
2. **M2 chases that recording.** Same binary, same objective, same camera-variation envelope, both
   elites scored on the *identical* 49 scenarios — `logs/autoc-040-t5-m2-novel-eval-{t4,t1}.log`.

### Result — t4 vs t1 on unseen geometry

| metric (49 novel scenarios) | **t4** (±10° camera var) | **t1** (no var) | Δ |
|---|---:|---:|---:|
| eval fitness | **−2034.19** | −2020.23 | **+0.7%** (t4 better) |
| median scenario score | 40.65 | 40.71 | −0.1% |
| crashes | 3 / 49 | 3 / 49 | — |
| per-tick error, median | 15.00 m | 14.93 m | +0.5% |
| per-tick error, mean | **16.55 m** | 17.02 m | **−2.8%** |
| per-tick error, p90 | **29.15 m** | 30.96 m | **−5.8%** |
| ticks inside 5 m | **8.4%** | 7.8% | **+8.1%** |
| avgVis | **0.786** | 0.760 | +3.4% |
| worst blind streak (mean, ticks) | **47.6** | 50.1 | −5.0% |
| avgInRamp | 0.038 | 0.041 | −8.0% |
| mean maxStreak | 15.7 | 17.5 | −10.4% |

**They are the same controller to within noise.** Every difference is ≤10% and they point in both
directions — t4 wins the error distribution (mean, p90, close-in occupancy, visibility), t1 wins streak
length. Total score differs by **0.7%** across 42,000 ticks. Both crash on **the same two scenarios**
(17 `Eval`, 19 `HullStrike`) plus one each of their own, which says the failures are geometry-driven,
not controller-driven.

This is the cleanest statement 040 can make about camera variation, and it is much cleaner than the
training curves: **identical scenarios, identical binary, identical objective, weights the only
variable.** Carrying up to 10° of boresight/roll error and a per-axis mount offset in every scenario
costs nothing measurable on geometry the controller has never seen.

### Generalization gap — the number SC-008 actually wanted

Same elite (t4), same computation, training set vs novel set:

| | ticks | median err | mean err | p90 | inside 5 m |
|---|---:|---:|---:|---:|---:|
| t4 on its **294 training** scenarios | 131,802 | 13.03 m | 15.07 m | 28.82 m | **15.3%** |
| t4 on **49 novel** scenarios | 42,285 | 15.00 m | 16.55 m | 29.15 m | **8.4%** |
| gap | | +15% | +10% | +1% | **−45%** |

**The standoff distribution barely moves; the tight-tracking fraction halves.** The controller keeps
the target at roughly the same distance on unseen geometry — the p90 tail is within 1% — but the share
of time it converts that into a close chase drops from 15.3% to 8.4%. Whatever the tight-tracking
skill is built on, it is partly memorized geometry. That is a generalization finding about the
*controller*, not about perception, and it is a 041 lead rather than a 040 defect.

### ⚠️ The 038-t9 baseline elite is not recoverable

The intended three-way (t4 / t1 / t9 on the same novel scenarios) could not be completed:
`nnextractor` on `autoc-m2 · autoc-9223370253232842213-…/gen9570.dmp.zst` fails with
`vector::_M_default_append` — a **cereal schema mismatch**. 040 grew the dmp schema and the project
does not version it ([[feedback_no_cereal_versioning]]), so 038-era dumps cannot be read by a 040
binary. **The consequence, stated plainly**: the prior baseline survives only as the numbers in its
log; its weights can no longer be re-flown under current code by any tool in the tree. Every
"vs 038-t9" figure in this document is therefore a comparison of *logged training metrics*, not a
head-to-head. t1-vs-t4 above is the only true head-to-head 040 has.

### What IS established about camera variation

- **SC-006 holds by test, not by argument**: `ZeroSigmaProducesExactlyTheNominalCamera` (200 seeds) +
  `NominalDrawLeavesTheTickConfigExactlyUntouched` — a variation-off run is bit-identical to baseline.
- The draw is reproducible from scenario id, truncated (not resampled) at 2.5σ, and independent of the
  other variation classes — T066–T070.
- The **shipped envelope** is 10° (σ 4.0) boresight and roll, per-axis translation ±5 mm x / ±10 mm y
  / ±3 mm z, wing thickness σ 0.8 mm. Emitter and ambient stay nominal by scope decision (Phase 7).

---

## Runs of record

| run | what | gens | prefix (`autoc-m2`) | master seed |
|---|---|---:|---|---:|
| **040-t1** | no camera variation, pre-objective-fix | 800 | `autoc-9223370251379769240-2026-07-31T05:16:46.567Z/` | 1785475004 |
| 040-t2 | 20° envelope — **VOID** (mount-inside-wing bug) | 369 | — | — |
| 040-t3 | 2° envelope — **VOID** (same bug) | 22 | — | — |
| **040-t4** | 10° envelope, post-fix — **the feature's result** | 800 | `autoc-9223370251039771221-2026-08-04T03:43:24.586Z/` | 1785815000 |

t2 and t3 are void for the reason recorded in `7ba6b11`: the baseline mount sits 2 mm proud of the
wing leading edge with its y and z already inside the slab, so an unclamped translation draw buried the
aperture **inside** the wing in ~16% of scenarios, and a ray origin inside a box hits in every
direction. Both runs capped identically at ~44 s worst blind streak from generation 1 — a tenfold
envelope change moving nothing was the tell. Fixed generally (a primitive containing the aperture does
not block it — the leading edge is notched for the camera), not by clamping the mount.

t4 converged: **61 elite changes, the last at gen 660**, 140 generations flat, run completed clean with
no worker deaths.

**Throughput (SC-013)**: t4 ran 1,176,235,200 evaluations in 189,781 s = **6,198 sims/s**, against t1's
6,678 and the t9 baseline's ≈5,600 late-run. The ≤10% regression ceiling is met with margin; camera
variation costs ~7% against t1, which is the per-scenario draw plus the obstruction path now being live
in every scenario rather than skipped.

**Retention (T087, Principle VIII)**: t1 and t4 uploaded tagged `retain=expire`. **t4 is the
perception-era M2 baseline** and its prefix above is what 041 compares against, so it must be pinned
`retain=keep`. ⚠️ **The tag has not been applied** — see outstanding items.

---

## Success criteria roll-up

| SC | verdict | evidence |
|---|---|---|
| SC-001 separation isotropy ≤2% | ✅ | `beacon_projection_tests.cc` |
| SC-002 range-from-separation vs truth | ✅ | `beacon_projection_tests.cc` |
| SC-003 effective FOV ≠ nominal | ✅ | T088 POV panel draws both; `airframe_occlusion` |
| SC-004 monotonic quality falloff | ✅ | `signal_model_tests.cc` |
| SC-005 reacquisition within one measured interval | ✅ | `acquisition_state_tests.cc` |
| SC-006 bit-identity; zero-variation ≡ baseline | ✅ | T068, T067 |
| SC-007 propeller-shadow onset distribution | ✅ | research.md §R6 |
| **SC-008 aggregate delta on unseen paths** | ✅ | delta reported on the training set **and** on 49 novel scenarios (T085); the 038-t9 comparison is from logged metrics, since its elite is no longer loadable — stated, not glossed |
| SC-009 no M1 regeneration, input vector unchanged | ✅ | source pinned; 58 inputs |
| SC-010 every quantity classified M/D/A | ✅ | [contracts/config-surface.md](contracts/config-surface.md) |
| SC-011 bearing to range, separation-range unavailable below limit | ✅ | `gather_tracker_inputs_tests.cc` |
| SC-012 calibration rehearsal, no structural change | ✅ | [calibration-rehearsal.md](calibration-rehearsal.md) |
| SC-013 throughput ≤10% regression | ✅ | 6,198 sims/s vs ≈5,600 baseline |

---

## Outstanding at wrap

Carried into the PR rather than silently dropped:

1. **T087 — S3 retention tags not applied.** t4 needs `retain=keep` (041's comparator); t1 may stay
   `retain=expire`. The novel-geometry source generated for T085
   (`autoc-eval · autoc-9223370250819561192-2026-08-06T16:53:34.615Z/`) should **also** be pinned
   `retain=keep` — it is now the reference novel scenario set and re-generating it produces different
   scenarios. This mutates S3 and is left to the operator.
2. **t1′ — the attributable camera-variation delta.** Deferred to [BACKLOG.md](../BACKLOG.md). T085
   weakens the case for it considerably: t4 and t1 are indistinguishable on identical novel scenarios
   under identical code, which is the comparison t1′ was meant to supply.
3. **The 038-era dmp is unreadable by current binaries** (see T085 above). Not a 040 defect — it is
   the standing no-cereal-versioning policy behaving as designed — but the *consequence*, that a
   baseline's weights expire when the schema moves, was not previously written down anywhere.

None of these blocks 041, which starts from the t4 prefix either way.

---

## What 040 hands to 041

The perception model is now **plumbing-first**: every physical quantity is a configured value with a
measured/derived/assumed classification, and SC-012 proved substituting all 15 assumed values requires
no code change. When the lens and bandpass field data arrives, calibration is an ini edit.

The standing hardware gap is unchanged and recorded in [optics-record.md](optics-record.md): modelled
receive at 100 m is **0.039 nA against a ≤10 nA measured decode floor, ≈24 dB short**. The 100 m
envelope remains an *assertion* (FR-033a). That is camera-perf / emitter-power work, not controller
work.

And the going concern above is 041's brief: perception fidelity was not the cap, so the next places to
look are the objective's control-quality terms and the predictor head that
[041 CANDIDATE](../BACKLOG.md) has already measured as carrying no signal.
