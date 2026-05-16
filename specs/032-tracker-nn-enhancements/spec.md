# Feature Specification: 032 Tracker NN Enhancements — Derived Pose Features

**Feature Branch**: TBD (currently parked; drafted in 030 branch during M11.wrap)
**Created**: 2026-05-11
**Status**: DRAFT — letter-to-ourselves capturing 030 v1 outcome + first-principles diagnosis of why M2 plateaus. No /clarify, /plan, or /tasks yet.

> **Why this exists**: 030 M2 tracker training (smoke14b, smoke15, postdiag1, postdiag2) reaches fitness around **-17,000** and plateaus. M1 pathgen reached -50,000+ on similar source trajectories. The gap is NOT NN topology, training compute, or determinism (all confirmed clean). The gap is **information content of the NN input vector**. This spec captures what we learned during 030 and outlines the derived-input changes that 032 would attack.

## Boundary lines

032 is a **controller-side feature** that adds derived perceptual features to the existing tracker NN input vector. It does NOT change hardware, does NOT change the optical front end, does NOT add new sensors. It re-derives information from the SAME 2-beacon-pair-NDC + history input shape that 030 ships with.

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
| **Span-rate** (unitless, span Δ per chase tick) | tick-to-tick change in beacon-pair span (one chase tick = 100ms @ 10Hz) | small, signed (positive = approaching, negative = receding) | "now" only | 1 |
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

Topology impact: NN input layer grows from **45 → 54** (6 span + 1 span-rate + 2 tilt sin/cos). Weight count grows ~20%; modest.

### 1.6 What's acknowledged + accepted in 032 scope

- **Front-back ambiguity is acknowledged and accepted** for 032. Resolving it cleanly requires either (a) a 3rd target beacon (operator-rejected — adds hardware, doesn't match real-flight ambition) or (b) a perception loop that integrates pose over time (M3-track research, deferred). 032 tests whether richer image-plane features + the existing 6-tick history give the NN enough signal to *predict* (not just react) within the ambiguity envelope. Result is either "ambiguity wasn't the load-bearing constraint" or "we hit a different ceiling and now know we need M3-grade perception."
- **Simulator has oracle access to BOTH chase AND target attitude.** The chase NN's `quat_w/x/y/z` inputs already see chase attitude; target attitude is in the source dmp and the simulator's projection. We can use both oracles **for training-time analysis / ground-truth comparison** (e.g., "what target tilt does the simulator's beacon-pair-on-image-plane reproduce vs the ground-truth target.q_EB roll component?") but the NN itself sees ONLY the derived perceptual inputs. No oracle leakage into the controller.
- **The trajectory we're on**: 2 coded beacons (030 / 032) → eventually non-broadcast visible-light identification + pose recovery (M3). 032 is the last stop before perception becomes its own feature.

---

## 2. Proposed 032 scope (controller-side only)

**030 baseline tuning is done** (M11.preA.5 T-102 closed 2026-05-15). The 16r → 32r experiment falsified the state-capacity hypothesis (32r is worse, not better — see [postdiag3_report.md](../030-tracker-mode/postdiag3_report.md)). **032 starts from postdiag2's 16r baseline.** Phase 1 features layer on top of the 16r topology — no architecture change.

032 includes ONLY changes that can ship without hardware or FPGA work:

- **(A) Beacon identity propagation** — gather pipeline change. Port-beacon NDC always in the L slots, starboard always in R, regardless of which is on which image side. Zero input-vector size change.
- **(B) Three derived perceptual features** (9 new input slots total):
  - `beacon_pair_span[6]` — unitless screen-fraction, 6-tick history at 10Hz = 600ms trend window (target's apparent width on screen — encodes range without wingspan calibration)
  - `span_rate` — "now" only, single scalar (target's apparent expansion/contraction rate per chase tick — encodes closing rate)
  - `target_tilt` — encoded as `(sin θ, cos θ)` pair, "now" only, 2 scalars. Convention: θ = 0 when chase + target wings are level relative to each other. Sin/cos encoding avoids the ±π wraparound discontinuity that would otherwise break NN gradients when chase rolls past inverted-relative-to-target.
- **(C)** Update sim noise / occasion-of-error models so the perceptual features see realistic real-flight error envelopes (beacon detection jitter, occasional sentinel under maneuver, etc.) — gated on bench evidence from 031 phase-1 recordings when they land.

If 032 phase (A) + (B) lifts the fitness ceiling meaningfully → hypothesis "visibility-time signal richness was binding" confirmed; ship. If ceiling stays flat → FOV-blindness or pose-ambiguity hypotheses earn their place; design a follow-on then.

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

## 5. Status as of draft

- **Draft created**: 2026-05-11 alongside postdiag2 overnight bake
- **/clarify**: not run; gated on 030 v1 closeout completion
- **/plan**: not started
- **First experiment**: when 032 unparks, start with feature (A) alone — beacon identity propagation, ZERO input-vector size change. Measure whether front-back ambiguity reduction alone shifts the plateau. Add (B) only if (A) doesn't move the needle.

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

If 032 (A) alone gets avgInRamp meaningfully above 0.07 — even to 0.15 — that's evidence the front-back disambiguation was the binding constraint. If it doesn't move, that argues for (B) or deferring to M3 / richer perception.
