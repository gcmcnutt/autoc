# 041-t5 wrap — the energy axis was not the binding constraint

**Run**: `autoc-041-t5-m1-throttleaxis`, seed 1787170949, stopped by operator at **gen 475**.
**One change vs t4**: `energy_score` reverted from Es-destroyed to the 035 convex throttle-power integral.
Everything else byte-identical — same ini, same pop 5000, same ramp-off, same arena, same 45 inputs.

---

## 1. Result

| | t5 | t4 | ratio |
|---|---:|---:|---:|
| best fitness | **−15,985** | −14,855 | **1.08×** |
| flat since gen | **357** | 376 | — |
| final sat_throttle | 99.6% | 100.0% | — |

⛔ **The P2-7 objective change bought ~8%, and did not move the failure mode at all.** Both runs freeze at
~−15k around gen 360–376 with throttle pinned. From gen 381 to 455 every sampled metric is *byte-identical*
— `sat_th 99.6 / sat_pitch 49.5 / sat_roll 2.4`, sixteen consecutive samples. The elite did not turn over
for ~100 generations. That is a stopped search, not a slow one.

⚠️ **[objective-amendment.md](objective-amendment.md) is now itself partly wrong, and this is the correction.**
It argued t4 failed *because* Es-destroyed rewarded pinned throttle. t5 removed that term and pinned throttle
anyway, at the same level, at the same generation. The amendment's *recommendation* (restore the power axis)
was harmless and mildly positive; its *causal story* was not supported. ⭐ The generalisable lesson is the
operator's, stated before the data came in: **small fitness tweaks do not move total fitness.** Cf.
[feedback_clear_objectives_not_tuning](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_clear_objectives_not_tuning.md).

## 2. What DOES separate the runs: the variation ramp

| run | `VariationRampStep` | last gen | best | shape |
|---|---:|---:|---:|---|
| 041-t1 | **40** | 608 | −23,651 | still improving at 592 |
| 041-t4 | **0** | 511 | −14,855 | flat since 376 |
| 041-t5 | **0** | 475 | −15,985 | flat since 357 |
| 038-t5 | **40** | 800 | −41,084 | improving to 634 |

2/2 ramp-off runs freeze at gen ~360–376. 2/2 ramp-on runs still improving past gen 590. No other single
variable splits the four this cleanly — the energy axis certainly does not.

⭐ **This also retires the "t1 regression."** t1 was ramp-ON and *still climbing* when it ended at gen 608.
Comparing it to a completed 800-gen run was comparing an unfinished race. There was never an anomaly.

⚠️ **CONFOUND — do not treat §2 as proven.** `computeVariationScale()` returns `stepIndex/(numSteps−1)`, so
038-t5 at gen 455 trained at **58% variation** while t5 ran 100% from gen 1. Easier scenarios survive longer
and raw score is length-confounded. Two live readings:

* **(a) curriculum** — the ramp finds a basin on easy cases then perturbs outward, genuinely reaching better
  policies. Operator's reading: *"variations can also sometimes break up the local minima."*
* **(b) scoreboard** — ramp-off runs hit the true full-variation ceiling at ~−15k, and ramp-on runs only
  *look* better because they are graded on an easier exam.

⛔ Raw training fitness **cannot** separate these — this is exactly what
[project_late_run_fitness_interpretation](../../.claude/projects/-home-gmcnutt-autoc/memory/project_late_run_fitness_interpretation.md)
warns about, and the running commentary during this bake drifted from that rule. **Fixed-eval settles it**:
score the t5 elite and the 038-t5 elite at the same variation scale.

## 3. ⭐ The craft's ability to track is NOT broken

The operator's discriminator: *"if we busted the craft's ability to track then no objective tweak will help."*
Measured, it did not:

| | gen 1 | gen 455 |
|---|---:|---:|
| mean target distance | 39.32 m | **11.25 m** |
| p95 target distance | 85.91 m | **25.98 m** |
| intercept-regime occupancy | 25.3% | **50.2%** |
| mean streak | 3.13 | **9.49** |

The 45-input / recurrent architecture learns to close from 39 m to 11 m and doubles intercept occupancy. The
machinery works. It stops *searching*, which is a landscape/selection problem, not a capability one.
⭐ **That is the evidence for the local-minima reading over the broken-architecture reading.**

## 4. ⭐ t5 flies SMOOTHER than the prior M1 — the metric the milestone actually cares about

Per-axis change rate `⟨|Δctrl|⟩` per tick, budget **0.27**:

| | pitch | roll | throttle |
|---|---:|---:|---:|
| **041-t5** @455 | **0.062** ✅ | **0.234** ✅ | **0.003** ✅ |
| 038-t5 @800 | 0.281 ❌ | 0.288 ❌ | 0.199 ✅ |

t5 is **under budget on all three axes**; the prior M1 was over on two. Pitch is 4.5× smoother.

⚠️ **But read the throttle number honestly.** `dctrl_throttle = 0.003` because throttle is *frozen at 0.998*
— a constant has no change rate. That is degenerate smoothness, not control quality. The pitch improvement
(0.062 vs 0.281 at comparable `mag_pitch` 0.892 vs 0.662) is real; the throttle one is an artifact.

⭐ Still, the honest headline for the milestone is: **041 produced a smoother-flying controller that scores
worse.** Operator's framing — *"the reason we are messing with m1 right now is to see smoother flying"* —
means t5 is closer to the actual goal than its fitness suggests. Fitness and the milestone objective have
come apart, and that gap is itself a finding.

## 5. Next step

⭐ **t6 = t5's exact config with `VariationRampStep = 40`. One variable.** It is the operator's own proposed
test and the four-run table supports it. Predictions, stated in advance so the read is honest:

* **improves past gen ~400** → reading (a), local minima; the ramp is a real search mechanism and belongs on
  for M1 regardless of the M2 practice that took it off.
* **freezes at ~−15k again** → reading (b); the ceiling is architectural/landscape, the ramp was a scoreboard
  effect, and the next lever is the NN arch + inputs, not any objective term.

⛔ **Run the fixed-eval comparison first** (t5 elite vs 038-t5 elite at equal variation scale). It is cheap,
and without it a good t6 cannot be distinguished from an easier exam.

⚠️ **Do not bundle any other change into t6.** t4→t5 was a clean single-variable read and that is the only
reason this wrap can say anything definite. Two changes at once is what made t4 unattributable.
