# Feature Specification: 032 Tracker NN Enhancements — Derived Pose Features

**Feature Branch**: `032-tracker-nn-enhancements` (active 2026-05-15)
**Created**: 2026-05-11
**Status**: ACTIVE — /clarify in progress 2026-05-15.

## Clarifications

### Session 2026-05-15
- Q: Is Feature A (beacon identity-stable ordering) in phase 1, or deferred? → A: IN phase 1 — single bake tests identity-stable ordering AND the 3 new derived features together (A + B).
- Q: Quantified success threshold for phase 1? → A: avgInRamp ≥ 0.15 at plateau (vs current ~0.07). Underlying story: overrun reduction — the new sensors expose range/closing/tilt explicitly so the chase stops shooting past target.
- Q: Span-rate formula? → A: one-tick diff `span[now] − span[now-1]`. Low-latency, raw signal. Smoothed trend already lives in the span[6] history slot.
- Q: Missing-beacon handling for new derived features? → A: **CEP-threshold gated** — if EITHER beacon's CEP exceeds a threshold (treat as missing/untrusted), substitute the **neutral reference values** representing "target straight ahead, in formation, no action needed": tilt `(sin θ, cos θ) = (0, 1)` (θ=0, wings-level relative); span = 0; span-rate = 0. Apply this convention uniformly across minisim / crrcsim impls (phase 1 scope). Document the rule in `docs/COORDINATE_CONVENTIONS.md` and `docs/sensor-pipeline.md` as part of phase 1 deliverables — **purpose of the doc updates is xiao-migration prep**: phase 1 ships minisim + crrcsim only; capturing the convention now keeps the future xiao tracker-mode port wire-equivalent without archaeology. (CEP threshold value itself is a config knob — set during /plan or implementation.) Note: existing 030 NDC slot handling also gets re-evaluated under this convention as part of Feature A's identity-stable ordering work, since both must be consistent.
- Q: History init at scenario start (`beacon_pair_span[6]` before tick 0)? → A: **Follow the existing point-history convention** — derived feature histories mirror whatever the existing 030 NDC point histories do at scenario start (currently replicate-first-valid in tracker_stepper.cc). Single convention covers both old and new history buffers. CEP-gating from Q4 layers on top during normal-operation blind ticks; scenario-start init is purely about pre-tick-0 buffer fill.
- Q: How is "plateau" defined for the phase 1 success criterion `avgInRamp ≥ 0.15 at plateau`? → A: **Match 030 reference protocol** — bake to ≥ 322 gens (smoke15 reference length), read avgInRamp averaged over the last 50 gens. Same protocol used to call the 030 -17k plateau; keeps phase-1 cross-comparable with the reference run without inventing new stability infrastructure.
- Q: Borderline-result decision rule (avgInRamp between 0.10 and 0.15)? → A: **Partial-band attribution bakes.** Three outcome bands: (i) plateau-avgInRamp ≥ 0.15 → phase 1 success, move on. (ii) 0.10 ≤ plateau-avgInRamp < 0.15 → partial signal; run two follow-on bakes (A-only: identity-stable ordering without derived features; B-only: derived features without identity-stable ordering) using the same 322-gen / last-50 plateau protocol, to attribute which sub-feature carried the lift. THEN decide phase-2 routing based on which sub-feature looks load-bearing. (iii) plateau-avgInRamp < 0.10 → clean miss; route to phase 2 (FOV-blindness / lost-sight patrol) or M3-grade perception without intermediate isolation bakes.
- Q: Phase 1 per-feature observability requirements? → A: **Defer to existing convention** — data.dat already records all NN inputs + outputs, and the 9 new derived-feature input slots must be captured under that convention. The 032 schema bump (45 → 54) is a dmp-honesty audit boundary per [feedback_honest_dmp_recording](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_honest_dmp_recording.md): the dmp format must also capture all inputs + outputs, including the 9 new slots — any gap (or rationale for one) gets reconciled as part of phase 1. Additional troubleshooting telemetry beyond inputs/outputs is NOT pre-committed at spec time; added during implementation/operation if a specific diagnostic need surfaces.

> **Why this exists**: 030 M2 tracker training (smoke14b, smoke15, postdiag1, postdiag2) reaches fitness around **-17,000** and plateaus. M1 pathgen reached -50,000+ on similar source trajectories. The gap is NOT NN topology, training compute, or determinism (all confirmed clean). The gap is **information content of the NN input vector**. This spec captures what we learned during 030 and outlines the derived-input changes that 032 would attack.

## Boundary lines

032 is a **controller-side feature** that adds derived perceptual features to the existing tracker NN input vector. It does NOT change hardware, does NOT change the optical front end, does NOT add new *physical* sensors (no new beacons, no new camera, no new IMU channels). It re-derives information from the SAME 2-beacon-pair-NDC + history input shape that 030 ships with — the new input slots are *computed* features over the existing optical front end.

| Feature | Scope | Status |
|---|---|---|
| **030 (tracker-mode)** | NN architecture + simulator + crrcsim integration + minimum-viable beacon perception | shipping v1 (this work) |
| **031 (beacon-camera hardware)** | LED pyramid + camera + lens + filter + raw-frame recording — physical optical front end | drafted spec, parked pre-hardware-order |
| **032 (this spec)** | NN-input-side derived features to break front/back / pose ambiguity from current beacon pair | **placeholder — letter to ourselves** |
| **M3 (future)** | Full target identification + pose estimation via dedicated perception loop (CNN/transformer + temporal prior) — supersedes 032's hand-crafted features | research-track placeholder |

Note: "M1 / M2 / M3" are training-mode labels (pathgen / tracker-from-beacons / tracker-from-pose). Spec numbers (030 / 031 / 032) are feature numbers. M2 ≠ 030; M2 is the training mode that 030 implements.

---

## 1. What we learned from 030 v1 — the plateau diagnosis

### 1.1 Where M2 lands today

Across four production-scale runs through 2026-05-11:

| run | bake | pop | gens | best fitness | notes |
|---|---|---|---|---|---|
| smoke14b | crrcsim, hull-off | 6000 | 98 | -14,315 | clean determinism, no plateau yet at gen 98 |
| smoke15 | crrcsim, hull-on | 6000 | 322 | -17,300 | plateau evident gens 200-322 |
| postdiag1 | crrcsim, hull-on, post-polish | 6000 | (running) | climbing | mid-bake |
| postdiag2 | crrcsim, hull-on, pop=5000 | 5000 | (running) | TBD | overnight bake |

M1 pathgen reached **-50,000+** at 800 gens with the SAME NN topology (33→32→16r→3), so the architecture has capacity. The bottleneck is upstream of the controller.

### 1.2 Behavior shape (from #GenDiag + playback inspection)

The chase **intercepts but doesn't track**: it gets to trail-distance (avgRngMin = 3.4m ≈ TrailDistance = 3.048m) routinely, but spends **93% of all ticks OUT of the fitness ramp** (avgInRamp ≈ 0.07). Streaks build briefly when chase passes through the cone, then break.

Streak-break reasons (post-polish, geometric-only taxonomy):
- **angle dominates** (≈55% of breaks) — chase is laterally off the trail line
- **far rising** (≈40%) — chase falls behind during reacquisition
- **over rare** (<1%) — overshoot heavily punished by evolution
- **hull rare** (<1%) — chase barely enters the 1m kill sphere

avgFlips = 17 / scenario, avgSpiral = 0.28 — chase uses banking aggressively to chase angular cues. Roll-axis aggressiveness (avgThrRl = 7.25) is 2× pitch-axis (avgThrPt = 4.00). Visible in playback as repeated transits through the cone without locking.

### 1.3 First-principles cause — open hypothesis list (2026-05-13, updated 2026-05-15)

**Original framing** (this spec v1, 2026-05-11): 2-point projection front-back ambiguity was the binding constraint.

**Operator challenge** (2026-05-13): M1 pathgen reached -50K WITHOUT pose info either. M1's inputs were body-frame bearing unit vec + range scalar + closing-rate scalar — NO target heading, NO target attitude, NO target velocity vector. Pose ambiguity therefore cannot be THE dominant constraint, because M1 hit -50K without it.

**T-102 result (2026-05-15)**: 030 baseline tuning bumped recurrent topology 16r → 32r (4× W_hh capacity, 256 → 1024 weights). Run baked to gen 540+ against the same source dmp; **best fitness at 32r is consistently 700-1500 points WORSE than 16r at every comparable gen**. State capacity is NOT the binding constraint either. See [postdiag3_report.md](../030-tracker-mode/postdiag3_report.md). The 030 baseline reverts to 16r; 032 starts from there.

**Operator playback observations on postdiag3 (2026-05-15)** — two patterns that directly motivate 032's phase 1 feature set:

1. **Overruns persist regardless of generation.** Chase passes through the cone and shoots past. Diagnosis: chase has no absolute-distance signal — only apparent point separation between the 4 NDC beacons. With known target geometry, angular separation IS a direct range proxy, but the NN has to learn that mapping implicitly from 4 NDC pairs across 6 ticks of history. **032's `span` + `span-rate` features expose this signal explicitly.**
2. **Chaos shuffles instead of refining.** Small best-fitness increments (~100 points) come paired with radical per-scenario behavior changes — overruns happen in different places, chaos pattern reshuffles, but the *amount* of chaos is conserved. Classic GA-stuck-in-local-optima signature: the gradient surface is too ambiguous for selection to reward incremental tightening, so it trades failure modes rather than refining within a strategy. **A cleaner "closer/farther" gradient (span-rate) should smooth the landscape enough for incremental refinement.**

Both observations point at the same upstream cause: input-vector information content, not architecture or capacity. This is the strongest evidence yet that **phase 1 (visibility-time signal richness) is the right place to start**.

| candidate | hypothesis | first test in 032 | status |
|---|---|---|---|
| Visibility-time signal richness | NN sees raw NDC + history but doesn't see derived geometric primitives (angular width = range proxy; beacon-pair tilt = roll proxy). M1 had bearing/range as scalars; M2 must learn them from int8-quantized NDC pairs across 6-tick history. | **Phase 1**: add span (width) + span-rate + tilt (angle) as new NN inputs. Same 16r topology. | **strongest candidate after T-102 + postdiag3 playback obs (2026-05-15)** |
| FOV-induced blindness | M2 is blind 30% of ticks (`avgVis ≈ 0.7`); M1 had infinite-FOV oracle bearing. Long blind runs (maxLost ~90 ticks) require dead-reckoning that the perception loop can't help with. | **Phase 2** (only if Phase 1 doesn't lift the ceiling): wider camera FOV, or richer dead-reckoning state inside the NN. | not falsified, but not yet primary suspect |
| Front-back pose ambiguity | 2 beacons project the same way for target heading-toward vs heading-away. Cannot lead target without disambiguation. | **Deferred**: M3 perception loop with temporal pose prior, or hardware addition of a 3rd beacon (operator-rejected for 030/031/032). | dropped from M3 conviction by M1's -50K-without-pose result; phase 4 / M3 only if span/tilt + FOV widening both miss |
| ~~Recurrent state capacity~~ | ~~16r W_hh insufficient to maintain target prior across visibility cycles~~ | ~~030 baseline tuning (M11.preA.5 T-102)~~ | **falsified 2026-05-15 — 32r is *worse* than 16r at every gen** |

030 NN topology stays at 16r for v1 baseline (32r reverted post-T-102). 032 phase 1 layers derived perceptual inputs on top of the 16r topology.

This was previewed in the 030 spec opening: see [specs/030-tracker-mode/spec.md](../030-tracker-mode/spec.md) pose-estimation framing section.

### 1.4 What does NOT help (testing the failure hypothesis)

Eliminate the candidates that DON'T attack the actual problem:

| candidate | does it help? | why |
|---|---|---|
| Wider FOV (e.g., 180° fisheye) | NO | same projection geometry, just wider field. Doesn't break front-back. |
| Narrower FOV (e.g., 90°) | NO | same projection geometry, narrower field. Worse for acquisition. |
| Non-linear lens (log-polar / dual-FOV) | NO | spatial pixel redistribution. Geometry unchanged. |
| More NN topology / parameters | NO | M1 with same topology reached -50k. Capacity isn't the bottleneck. |
| Softer cone-fitness shape | NO | tuning, not first-principles. M1 used same cone, hit -50k. |
| Shorter streak ramp | NO | tuning, addresses symptom not cause. |
| More training compute / longer gens | NO | smoke15 plateaued in <200 gens; more gens don't shift the ceiling. |

### 1.5 What DOES help (the 032 design space)

032 ships ONLY controller-side input/feature changes (no hardware, no NN topology — topology decisions belong to 030 baseline tuning). Two complementary moves, smallest-bet-first per the 1.3 hypothesis ranking:

#### A. Beacon code identity propagation (cheap, partial visibility-time win)

Today the NN labels beacons "left" and "right" by NDC ordering — that's not what the FPGA actually emits. The Gold-code correlator (per [docs/aircraft_tracker_handoff.md](../../docs/aircraft_tracker_handoff.md) §5.5) reports beacon IDENTITY (code A vs code B), which maps to physical port vs starboard wing on the target. Propagating that identity through to the NN gives the chase "I see target's port wing at (x, y)" instead of "the leftmost dot is at (x, y)" — that breaks front-back ambiguity directly.

Today's tracker beacons are labeled by mount position (left wingtip = port = code A, right wingtip = starboard = code B), but the NN input pipeline doesn't preserve identity through the projection — it just sorts by image-plane x. **Fix**: in `gather_tracker_inputs`, deliver beacon_L NDC + beacon_R NDC with IDENTITY-stable ordering (always port first, regardless of which is on which image side).

This is mostly a contract change in the gather pipeline; the FPGA-side code-correlation already does the work. The simulator's `projectBeacon` knows which beacon is which (it has the body-mount Y sign). The chase NN inputs just need to preserve that ordering through the camera transform instead of re-sorting by NDC x.

#### C. Derived perceptual features as new NN input slots — NO CALIBRATION

Compute these in `gather_tracker_inputs` from the beacon pair, feed as new input slots alongside the existing 45. **Key constraint: no physical-unit calibration. Everything is fraction-of-full-scale or angle-of-screen-line, so the same input transform works in sim AND in real flight without per-craft wingspan-calibration overhead.**

| feature | value | range | history? | input slots |
|---|---|---|---|---|
| **Beacon-pair span** (unitless, screen-fraction) | distance between port and starboard beacon centroids on the NDC image plane — `‖(x_r,y_r) − (x_l,y_l)‖` | ~0 (far) → ~2 (beacons at opposite screen edges) | yes, 6-tick | 6 |
| **Span-rate** (unitless, span Δ per chase tick) | one-tick diff: `span[now] − span[now-1]` (one chase tick = 100ms @ 10Hz). Low-latency raw signal; smoothed trend lives in span[6] history. | small, signed (positive = approaching, negative = receding) | "now" only | 1 |
| **Target tilt** (sin θ, cos θ pair) | image-plane angle θ of the line from port (red) → starboard (green) beacon. **Convention: θ = 0 when chase + target wings are level relative to each other** (port→starboard line projects horizontally with port on chase-image-left). Encoded as `(sin θ, cos θ)` to remove the ±π wraparound discontinuity. | each component [-1, 1] | "now" only | 2 |

**Notes on the design:**

- **Why fraction-of-screen, not metric range**: in real flight you don't have a wingspan-calibration step before each session. The NN learns "beacon-pair span = 1.0 means I'm close enough to see them at half the FOV" without ever needing to know the target's wingspan in meters. Same transform works in sim. M1's `dist[6]` was metric because the simulator could oracle-emit it; we're explicitly NOT depending on that here.

- **Why span-rate is "now" only**: the 6-tick beacon-pair span history already encodes the rate implicitly. Adding span-rate as an explicit single scalar gives the NN a "right now is target approaching or receding" cheap-to-consume input without forcing it to backprop a finite difference through six raw values. History on span is for trend; rate is for immediate.

- **Why tilt is encoded as (sin θ, cos θ) and NOT raw radians**: a raw-radian tilt wraps at ±π — roll the chase aircraft through inverted relative to the target and the input jumps from near +π to near -π in a single tick. NN activations don't like that discontinuity: gradients across the wrap are meaningless and the network has to learn two disjoint regimes for what is physically the same maneuver. The (sin θ, cos θ) pair is the standard NN angle encoding — continuous through the wrap (sin ±π = 0, cos ±π = -1; both smooth), bounded in [-1, 1], and the network can trivially recover θ via `atan2(sin, cos)` if it wants the linear form. Cost: 2 input slots instead of 1; worth it.

- **Why tilt convention is "wings-level relative to target = 0"**: gives the chase NN a clean reference point. When chase is rolled the same as target (e.g., both wings-level in formation), tilt = 0 — no input pressure on the controller. When chase is rolled relative to target, tilt is non-zero proportional to relative roll. This complements the existing absolute-world chase-attitude inputs (`quat_w/x/y/z`): those say "I'm rolled in the world"; tilt says "I'm rolled relative to target". The NN can use the combination to separate the two regimes.

- **Why tilt is "now" only**: image-plane tilt history would be redundant with the 6-tick beacon-pair NDC history (which already records the geometry that determines tilt). If tilt-rate becomes load-bearing as a target-maneuver leading indicator, add it as a single "now" scalar — but not in this iteration.

- **Tilt does NOT disambiguate target front-back**. A target heading toward chase banking left looks the same as a target heading away banking right in image-plane tilt alone — that ambiguity is acknowledged and accepted for 032 (deferred to M3 perception loop).

- **Beacon size is NOT a separate input**. The CEP value already encodes beacon-detection-quality / uncertainty. We treat beacons as point sources + CEP, matching the renderer's mini-panel splat representation. No major-axis-orientation, no blob area, no σ_xx/σ_yy. (If FPGA later emits those, they could feed a richer 032 variant — but not in this iteration.)

- **NOT in this iteration**: bearing-rate vector (dMidpoint/dt), explicit closing-rate scalar, tilt-rate scalar, beacon area/major-axis. The bearing-rate signal is implicit in the 6-tick beacon position history. We're keeping the 032 feature set tight to test whether **span + span-rate + tilt** alone moves the avgInRamp ceiling — if so we get a clean result attribution; if not, that argues for the next-level addition.

- **Missing-beacon convention** (locked 2026-05-15 via /clarify Q4): each tick, `gather_tracker_inputs` checks each beacon's CEP. If EITHER beacon's CEP exceeds a config-tunable threshold (treat as untrusted detection), the derived features substitute their **neutral reference values** representing "target straight ahead, in formation":
  - `target_tilt`: `(sin θ, cos θ) = (0, 1)` — θ = 0, wings-level relative, no roll pressure
  - `beacon_pair_span`: 0 — "no closing-distance signal"
  - `span_rate`: 0 — "no relative motion"

  These are the inputs that produce no controller-side action pressure. Phase 1 includes propagating this CEP-gating convention through the existing 18 NDC slots too (consistent treatment with Feature A's identity-stable ordering work). Cross-impl invariant: minisim, crrcsim, and any future xiao tracker-mode path apply the SAME CEP threshold + SAME substitution values. Documented in `docs/COORDINATE_CONVENTIONS.md` (sensor pipeline section) and `docs/sensor-pipeline.md` as a phase 1 deliverable.

  *Implication for phase-2 lost-sight patrol*: this convention deliberately gives the NN "everything's fine" inputs during blind ticks — it cannot distinguish "actually fine" from "blind, fine-looking inputs are masking it." Phase 2 will need explicit `vis_now` / `ticks_since_seen` signals on top to enable patrol behavior. Acknowledged tradeoff: phase 1 treats blind ticks as no-action zones; phase 2 unlocks patrol learning.

- **History init at scenario start** (locked 2026-05-15 via /clarify Q5): `beacon_pair_span[6]` follows the SAME convention as the existing 030 NDC point histories at scenario start. The current pattern is replicate-first-valid (`tracker_stepper.cc` `projectAndShiftHistory()` and the scenario-init pre-fill); span[6] mirrors it. If the existing convention is later revised (e.g., to switch to CEP-gated neutral fill at scenario start too), the derived-feature histories follow. The intent: a single load-bearing convention spans all history buffers — old and new — so that minisim / crrcsim / xiao remain wire-equivalent.

Topology impact: NN input layer grows from **45 → 54** (6 span + 1 span-rate + 2 tilt sin/cos). Weight count grows ~20%; modest.

### 1.6 What's acknowledged + accepted in 032 scope

- **Front-back ambiguity is acknowledged and accepted** for 032. Resolving it cleanly requires either (a) a 3rd target beacon (operator-rejected — adds hardware, doesn't match real-flight ambition) or (b) a perception loop that integrates pose over time (M3-track research, deferred). 032 tests whether richer image-plane features + the existing 6-tick history give the NN enough signal to *predict* (not just react) within the ambiguity envelope. Result is either "ambiguity wasn't the load-bearing constraint" or "we hit a different ceiling and now know we need M3-grade perception."
- **Simulator has oracle access to BOTH chase AND target attitude.** The chase NN's `quat_w/x/y/z` inputs already see chase attitude; target attitude is in the source dmp and the simulator's projection. We can use both oracles **for training-time analysis / ground-truth comparison** (e.g., "what target tilt does the simulator's beacon-pair-on-image-plane reproduce vs the ground-truth target.q_EB roll component?") but the NN itself sees ONLY the derived perceptual inputs. No oracle leakage into the controller.
- **The trajectory we're on**: 2 coded beacons (030 / 032) → eventually non-broadcast visible-light identification + pose recovery (M3). 032 is the last stop before perception becomes its own feature.

### 1.8 Hull-strike escalation pattern — reward-shape experiment candidate (2026-05-17)

Discovered during 032 phase-1 bake monitoring. **Hull-strike rate grows monotonically as the controller improves**, in both 030 and 032 — and 032 escalates ~3× faster than 030 thanks to the new sensors enabling tighter close-in tracking.

Per-gen `reason=HullStrike` count, 50-gen window averages, comparing the two bakes side-by-side:

| 50-gen window | 030 postdiag2 | 032 killed-at-173 | ratio |
|---|---|---|---|
| 1–50 | 1.00 | 1.12 | ~equal |
| 51–100 | 2.14 | **5.78** | 2.7× |
| 101–150 | 3.34 | **9.70** | 2.9× |
| 151–200 | 2.60 | **9.97** (n=31) | 3.8× |
| 201–250 | 2.98 | — | — |
| 251–300 | 4.62 | — | — |
| 301–350 | 7.56 | — | — |
| 351–400 | 6.66 | — | — |
| 401–450 | 7.36 | — | — |
| 451–500 | 9.20 | — | — |
| 501–550 | 11.26 (n=42) | — | — |

**Reading**: 030 alone shows the trend (1.0 → 11.3 over 500 gens, ~11×). 032 hits the same trajectory ~3× faster because the added closing-rate + wingspan inputs let the NN tighten standoff distance more aggressively earlier.

**Why this happens (fitness math)**:
- Reward is in-cone ticks × streak multiplier (up to 5×). Compounding upside.
- Hull-crash cost: scenario terminates → forfeit remaining points in *that* scenario only. Bounded loss (one scenario out of ~294).
- Net: aggressive close-tracking with occasional hull crashes is fitness-positive at scale. Evolution finds it cheaper to crash sometimes than to back off everywhere.

The `CrashHullProbability=0.10` per-tick Bernoulli (≈50% chance of crash within 7 ticks inside hull) is already non-trivial — but evolution still routes around it as the streak-fitness upside grows.

**Phase-1b candidate experiment** (parallel to Q7 attribution bake — also contingent on phase-1 plateau outcome):

| Option | Cost | Likely effect |
|---|---|---|
| **A. Kamikaze multiplicative penalty** on hull crash — scenario's accumulated score × 0.5 (or other configurable α ∈ [0, 1])  | trivial — one-line in scenario aggregator, plus an ini knob (e.g. `HullCrashScoreFactor`) | "You got close and died" → half the work doesn't count. Multiplicative scales with how much was already earned, so high-skill aggressive close-trackers feel it more than incidental brush-by crashes. Preserves partial credit for early-scenario progress |
| **B. Raise `CrashHullProbability`** 0.10 → 0.30 | one ini value | stronger probabilistic enforcement |
| **C. Reduce `FitStreakMultiplierMax`** 5.0 → 3.0 | one ini value | lowers in-ramp upside relative to hull cost; may also slow convergence |
| **D. Expand `CrashHullRadius`** 1.0m → 1.5m | one ini value | larger no-fly zone, forces more standoff |

Operator preference (2026-05-17): **Option A with α = 0.5** as cleanest first try — the kamikaze framing is intuitive (a crash that comes after a long good tracking run loses half its value; a crash near scenario start loses very little because little was earned yet — appropriately mild), orthogonal to cone shape and streak math, single ini knob. **Option B** as strong alternative if A doesn't bite enough.

**Decision protocol**: defer to post-phase-1 closeout. Whichever phase-1 outcome routes (success / partial / miss per Q7), the hull-penalty experiment is a candidate variant. If phase-1 plateau-avgInRamp ≥ 0.15 (success), the hull-penalty experiment becomes "can we get the same or better avgInRamp with fewer hull strikes" (efficiency / safety win). If partial / miss, it joins the attribution-bake matrix.

**Forward implication for real-flight deployment**: a controller trained with high hull-strike tolerance will collide with the target in real flight. The hull-penalty knob is also the gate between "good sim numbers" and "safe enough to flight-test." Worth doing before xiao deployment regardless of phase-1 outcome.

### 1.7 Sensor modality vs sensor fault — distinction for the next iteration (2026-05-16)

A critical framing note added during 032 phase 1 implementation: **beacon invisibility in this design is a MODALITY, not a SIGNAL LOSS**.

Beacons aren't omni-directional. With 270° emission cones mounted on wingtips, certain target attitudes / chase-relative geometries naturally render one or both beacons out of the chase camera's view (target heading directly away, target rolled past inverted, target outside FOV laterally, etc.). When the chase sees only the starboard beacon for several ticks, that's not a sensor failure — it's information: target's port wing is occluded from chase's viewpoint, telling us something about relative attitude.

**Phase 1 (current) treats all blindness uniformly** — both-beacon-visible triggers full derived-feature computation; either-beacon-gated substitutes neutral values across all derived slots. That's the simplest correct-when-both-visible behavior, accepted as a phase-1 tradeoff.

**The NEXT iteration of sensors / synthetic sensors should distinguish modality from loss.** Goals:
- **Better turn prediction**: extract intent from partial-visibility patterns (e.g., "we're seeing only the starboard wing → target is rolling left, anticipate the turn")
- **Better intercept**: maintain meaningful lead computation using single-beacon bearing + history when only one beacon is visible
- **Continued operation with one sensor visible**: degrade gracefully from "full pose info" → "half pose info, fall back to bearing + last-known span/tilt" → "no info, patrol"
- **Soft-fail semantics**: substitute degraded but useful inputs (last-known with staleness flag) instead of jumping to neutral
- **Per-beacon timers + history**: `ticks_since_seen_left`, `ticks_since_seen_right` as independent counters; last-known per-beacon NDC retained across short gaps for interpolation

**NOT in scope yet**: sensor-fault robustness (cross-checking independent sensors against each other). That's a later milestone, predicated on having multiple independent sensor modalities to cross-check. For now we assume the optical front end is honest; the only "failure mode" is geometry-driven invisibility.

This note is forward-looking — captured here so the next sensor-architecture spec (032 phase 2, or a successor) has the design constraints + goals already articulated. The phase-1 implementation lives with the uniform-blindness simplification; the design space for richer modality handling is documented and queued.

---

## 2. Proposed 032 scope (controller-side only)

**030 baseline tuning is done** (M11.preA.5 T-102 closed 2026-05-15). The 16r → 32r experiment falsified the state-capacity hypothesis (32r is worse, not better — see [postdiag3_report.md](../030-tracker-mode/postdiag3_report.md)). **032 starts from postdiag2's 16r baseline.** Phase 1 features layer on top of the 16r topology — no architecture change.

032 includes ONLY changes that can ship without hardware or FPGA work:

- **(A) Beacon identity propagation** — gather pipeline change. Port-beacon NDC always in the L slots, starboard always in R, regardless of which is on which image side. Zero input-vector size change.
- **(B) Three derived perceptual features** (9 new input slots total — all must be recorded in both data.dat and dmp per the inputs+outputs honesty invariant, locked 2026-05-15 via /clarify Q8; 032's schema bump is the audit boundary):
  - `beacon_pair_span[6]` — unitless screen-fraction, 6-tick history at 10Hz = 600ms trend window (target's apparent width on screen — encodes range without wingspan calibration)
  - `span_rate` — "now" only, single scalar (target's apparent expansion/contraction rate per chase tick — encodes closing rate)
  - `target_tilt` — encoded as `(sin θ, cos θ)` pair, "now" only, 2 scalars. Convention: θ = 0 when chase + target wings are level relative to each other. Sin/cos encoding avoids the ±π wraparound discontinuity that would otherwise break NN gradients when chase rolls past inverted-relative-to-target.
- **(C)** Update sim noise / occasion-of-error models so the perceptual features see realistic real-flight error envelopes (beacon detection jitter, occasional sentinel under maneuver, etc.) — gated on bench evidence from 031 phase-1 recordings when they land.

**Phase 1 (locked 2026-05-15)** ships (A) + (B) together in a single bake:
- (A) **Beacon identity-stable ordering**: existing 6-tick beacon-NDC history slots change semantics — port-side beacon always in L slots, starboard always in R slots, regardless of which is on which image side at any given tick. Zero input-vector size change; reinterprets existing 18 input slots. (Old 030 weights are not portable across this reorder — fine, we're greenfield-training anyway.)
- (B) **Three derived perceptual features** appended to the input vector (45 → 54).

Single bake then tests whether the combined feature set lifts the ceiling. **Phase 1 success threshold (locked 2026-05-15 via /clarify Q2): avgInRamp ≥ 0.15 at plateau** (vs current ~0.07 ceiling — see Appendix A). **Plateau read protocol (locked 2026-05-15 via /clarify Q6): bake to ≥ 322 gens (matches smoke15 reference length), then average avgInRamp over the last 50 gens — that average is the success number.** avgInRamp is the symptom-level metric: chase currently spends 93% of ticks out of the cone; doubling time-in-cone is the bar for "phase 1 worked." Underlying story is overrun reduction — the new sensors expose range/closing/tilt explicitly so the chase stops shooting past target. Best-fitness number is a secondary signal (variation-ramp pressure makes cross-run fitness comparisons noisy per [project_late_run_fitness_interpretation](../../.claude/projects/-home-gmcnutt-autoc/memory/project_late_run_fitness_interpretation.md)).

**Outcome decision rule (locked 2026-05-15 via /clarify Q7)**:
- **≥ 0.15** → phase 1 success; advance to closeout, then operator decides next milestone (031 hardware, M3 perception, or 030-related follow-ups).
- **0.10 ≤ avgInRamp < 0.15** → partial signal. Run TWO follow-on bakes using the same 322-gen / last-50-gen plateau protocol: (i) **A-only** = identity-stable ordering enabled, derived features disabled; (ii) **B-only** = identity-stable ordering reverted to NDC-x-sort, derived features enabled. Attribution decides whether to refine A, refine B, or escalate to phase 2.
- **< 0.10** → clean miss; route to phase 2 (FOV-blindness / lost-sight patrol) or M3-grade perception without intermediate isolation bakes.

032 explicitly **DOES NOT** include:
- **Calibration of any input to physical units** — everything is fraction-of-screen or angle-on-screen. Same transform sim ↔ real flight.
- **Hardware changes** (031 scope: LED pyramid, optical filter, camera)
- **FPGA wire-format changes** (031-fpga sub-spec; the chase consumes whatever the FPGA emits today: x/y/CEP per beacon — point sources)
- **Beacon size / blob shape / major-axis as inputs** (CEP already encodes detection quality / spread)
- **Bearing-rate vector or explicit closing-rate scalar** (implicit in 6-tick history of beacon positions + the new span[6])
- **3rd beacon on target** (operator decision 2026-05-11: rejected; pose recovery is M3 territory via a richer perception loop, not via more beacons)
- **NN topology / architecture change** (use 030's existing 32→16r→3 hidden layers; just wider input layer 45→54). Note: T-102 (M11.preA.5) falsified the state-capacity hypothesis, so 16r is the empirically-best baseline. A 16r-vs-32r A/B with phase-1 features installed is a candidate side-experiment if features alone don't lift the ceiling enough.
- **Front-back ambiguity resolution** (acknowledged + accepted for 032; deferred to M3 — see §1.6 below)

## 3. Open questions (operator triage before /clarify)

### Q1 — Beacon identity propagation: stable ordering or explicit identity field?

Option A: re-order beacon_L / beacon_R inputs by code identity (always port first). 0 input-vector size change.
Option B: add an explicit `beacon_L_is_port` boolean flag per input slot. +12 input slots (×6 history).
Option C: ignore identity, just use code-ID-derived ordering and call it good.

Default: A (no input-vector size change, identity baked into ordering convention).

### Q2 — Which derived features get 6-tick history vs "now"-only? (locked 2026-05-11, refined 2026-05-15)

- `beacon_pair_span`: **6-tick history** (trend matters; span shrinks as target accelerates away). 6 ticks × 100ms = 600ms window at 10Hz chase rate.
- `span_rate`: **"now" only**, single scalar (the trend lives in span[6]; rate is the immediate-action input)
- `target_tilt`: **"now" only**, encoded as `(sin θ, cos θ)` pair (2 scalars). Convention: θ = 0 when chase + target are wings-level relative to each other. Sin/cos encoding chosen over raw radians to avoid the ±π wraparound discontinuity at chase-relative-inverted attitudes; the NN can recover θ via `atan2(sin, cos)` internally if it wants the linear angle. Tilt history would be redundant with the raw beacon-pair NDC history.

### Q3 — Calibration / unit conversion concerns? (locked 2026-05-11 — NONE)

Everything is fraction-of-screen or angle-on-screen. No wingspan input, no per-craft scale factor, no per-camera calibration. Same input transform in sim AND real flight. The NN learns the empirical mapping between "span = 0.5 unitless" and "target is at meaningful tracking distance" without ever seeing meters. If real-flight cameras have different FOV than sim's 120°, the absolute span values DO shift — but the NN can be retrained with the deployment FOV using the same code, and the transform itself stays unitless.

### Q4 — Does 032 break the 030 wire contract?

030's TrackerInputs is float[45] — a load-bearing serialization contract per the 030 spec FR-006 + FR-019. 032 grows it to **float[54]** (45 + 6 span-history + 1 span-rate + 2 tilt-sin-cos). That's a struct extension; needs a CEREAL_CLASS_VERSION bump or a parallel TrackerInputsV2 struct. **Per the project policy** ([feedback_no_cereal_versioning](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md)) we don't bump CEREAL — so 032 is a greenfield change. Old genomes from 030 are not portable, which matches the "input-transform changes break weight portability" pattern established in M11.preA.2.

### Q5 — When to attempt 032?

Two readiness signals:
1. 030 v1 sealed: SMOKE_REPORT.md + outcome.md + closeout commits landed
2. Operator decides "M2 plateau is real enough that we want to try input-space changes before moving to 031 hardware work"

Both signals expected within days. 032 likely is the most efficient next step (controller-side change, no hardware dependency) — but 031 hardware bring-up can start in parallel.

## 4. Items moved to 032 from elsewhere

This section lists items originally filed in other specs/backlogs that fit better in 032:

- *(none yet — first pass at 032 scoping. Add items as they surface during v1 closeout or post-bake analysis.)*

### 4.1 Open follow-ups during 032

- **Project-memory location decision** (2026-05-17): currently `~/.claude/projects/-home-gmcnutt-autoc/memory/` is per-user, per-host, git-ignored. Operator preference is to move toward Option B (in-repo, e.g., `docs/project_memory/`) or Option C (hybrid: shared in-repo + personal local) so memory propagates to other hosts via git pull. Leaning B (single operator, no privacy issue) or C (future-proof if a second collaborator joins). Decide before phase-1 closeout so the memory created during the bake (post-mortem findings, follow-ons) lands in the right place from the start. See gitignore commit `129b1f0` discussion.
- **Hull-strike escalation + kamikaze penalty experiment** (2026-05-17): see §1.8. Phase-1b candidate; run after phase-1 closeout. Operator-preferred form is multiplicative ½-of-accumulated-score penalty on hull crash (kamikaze framing). Gates safe-enough-for-real-flight deployment regardless of phase-1 plateau outcome. Tasks: US3 in tasks.md.

## 5. Status

- **Draft created**: 2026-05-11 alongside postdiag2 overnight bake
- **Branch active**: 2026-05-15 (`032-tracker-nn-enhancements`, after 030 T-102 closeout commit 8bfad02)
- **/clarify**: completed 2026-05-15 (8 questions answered total — initial pass Q1-Q5, second pass Q6-Q8; see Clarifications section at top)
- **/plan**: not started — next step
- **First experiment** (locked 2026-05-15 via /clarify Q1): phase 1 ships (A) + (B) together — beacon identity-stable ordering PLUS the 3 new derived features in a single bake. Earlier draft framing ("start with A alone") superseded; rationale was avoiding confounded attribution, but the operator decision is that the combined surface is the right phase-1 test.
- **Phase-1 implementation surface** (confirmed 2026-05-15): minisim + crrcsim only; xiao tracker-mode port is NOT phase 1, but the conventions docs (`docs/COORDINATE_CONVENTIONS.md`, `docs/sensor-pipeline.md`) get updated in phase 1 specifically to make that future xiao port wire-equivalent.
- **Topology**: phase 1 holds 16r (T-102 result stands). 16r → 32r resizing is a contingency, NOT a phase-1 step — only revisited if (a) phase-1 plateau-avgInRamp falls in the 0.10–0.15 partial band AND attribution bakes (per Q7 rule) point to under-capacity rather than under-signal, or (b) a future phase explicitly chooses to retest state capacity with the richer input vector installed.

---

## Appendix A — Why the plateau is real (data backing the diagnosis)

Quick numbers (smoke15 final, gens 1→322):

| metric | gen 1 | gen 100 | gen 322 | reading |
|---|---|---|---|---|
| best fitness | -4,474 | -15,092 | -17,300 | 3.9× improvement, then saturated |
| avgVis | 0.36 | 0.69 | ~0.70 | target visible 70% of ticks; converged |
| avgInRamp | 0.05 | 0.07 | ~0.07 | **never gets above 7%** ← the symptom |
| avgRngMin | 3.77 | 3.4 | 3.4 | reaches trail distance routinely |
| avgRngMed | 43 | 19 | 19 | sits ~6× trail distance most of the time |
| avgFlips | 8 | 17 | 17 | converged, twitchy |
| angle (streak-break count) | 255 | 380 | 380 | converged |
| far (streak-break count) | 66 | 280 | 280 | converged |

The "converged" trajectory of every metric past gen ~100 is the signature: more compute doesn't move it because the controller has already extracted everything it can from the input space. The 7% in-ramp rate is the bound the input information content imposes.

**Locked 2026-05-15 via /clarify Q2**: phase 1 success threshold is **avgInRamp ≥ 0.15 at plateau** (~2× current ceiling). **Plateau read (locked 2026-05-15 via /clarify Q6): bake ≥ 322 gens, average avgInRamp over last 50 gens.** **Outcome rule (locked 2026-05-15 via /clarify Q7): ≥0.15 = success; 0.10–0.15 = partial → A-only + B-only attribution bakes; <0.10 = clean miss → route to phase 2 / M3.** Earlier framing of "Feature A alone" superseded by Q1 (phase 1 = A + B together).
