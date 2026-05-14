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

### 1.3 First-principles cause — open hypothesis list (2026-05-13 update)

**Original framing** (this spec v1, 2026-05-11): 2-point projection front-back ambiguity was the binding constraint.

**Operator challenge** (2026-05-13): M1 pathgen reached -50K WITHOUT pose info either. M1's inputs were body-frame bearing unit vec + range scalar + closing-rate scalar — NO target heading, NO target attitude, NO target velocity vector. Pose ambiguity therefore cannot be THE dominant constraint, because M1 hit -50K without it.

This spec no longer asserts a single locked diagnosis. The plateau has multiple plausible binding constraints; **032's job is to test them in order**, smallest-bet-first. Candidate ordering (operator-routed 2026-05-13):

| candidate | hypothesis | first test in 032 |
|---|---|---|
| Visibility-time signal richness | NN sees raw NDC + history but doesn't see derived geometric primitives (angular width = range proxy; beacon-pair tilt = roll proxy). M1 had bearing/range as scalars; M2 must learn them from int8-quantized NDC pairs across 6-tick history. | **Phase 1**: add span (width) + span-rate + tilt (angle) as new NN inputs. Same NN topology. |
| FOV-induced blindness | M2 is blind 30% of ticks (`avgVis ≈ 0.7`); M1 had infinite-FOV oracle bearing. Long blind runs (maxLost ~90 ticks) require dead-reckoning that the perception loop can't help with. | **Phase 2** (only if Phase 1 doesn't lift the ceiling): wider camera FOV, or richer dead-reckoning state inside the NN. |
| Front-back pose ambiguity | 2 beacons project the same way for target heading-toward vs heading-away. Cannot lead target without disambiguation. | **Deferred**: M3 perception loop with temporal pose prior, or hardware addition of a 3rd beacon (operator-rejected for 030/031/032). |

The 030 NN topology bump 16r → 32r is **030 baseline tuning** (NOT a 032 feature) and runs ahead of this spec; if 32r alone meaningfully closes the M1↔M2 gap, this spec's first-experiment scope shrinks accordingly.

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

| feature | value | range | history? |
|---|---|---|---|
| **Beacon-pair span** (unitless, screen-fraction) | distance between port and starboard beacon centroids on the NDC image plane — `‖(x_r,y_r) − (x_l,y_l)‖` | ~0 (far) → ~2 (beacons at opposite screen edges) | yes, 6-tick |
| **Span-rate** (unitless, span Δ per tick) | tick-to-tick change in beacon-pair span | small, signed (positive = approaching, negative = receding) | "now" only |
| **Target tilt** (radians) | angle of the line from red (port) → green (starboard) beacon, measured CCW from screen +x axis. **Independent of chase attitude** (it's a pure image-plane measurement). | ±π | "now" only |

**Notes on the design:**

- **Why fraction-of-screen, not metric range**: in real flight you don't have a wingspan-calibration step before each session. The NN learns "beacon-pair span = 1.0 means I'm close enough to see them at half the FOV" without ever needing to know the target's wingspan in meters. Same transform works in sim. M1's `dist[6]` was metric because the simulator could oracle-emit it; we're explicitly NOT depending on that here.

- **Why span-rate is "now" only**: the 6-tick beacon-pair span history already encodes the rate implicitly. Adding span-rate as an explicit single scalar gives the NN a "right now is target approaching or receding" cheap-to-consume input without forcing it to backprop a finite difference through six raw values. History on span is for trend; rate is for immediate.

- **Why target tilt is "now" only and image-plane-absolute, not chase-attitude-corrected**: the chase NN already has its own attitude (quat_w/x/y/z) as inputs. If it wants chase-corrected target tilt, it can derive it. Feeding raw image-plane tilt keeps the input simple and matches what an FPGA-side blob major-axis would emit naturally. **Tilt does NOT disambiguate target front-back** (a target heading toward chase banking left looks the same as a target heading away banking right in image-plane tilt alone) — that ambiguity is acknowledged and accepted for 032.

- **Beacon size is NOT a separate input**. The CEP value already encodes beacon-detection-quality / uncertainty. We treat beacons as point sources + CEP, matching the renderer's mini-panel splat representation. No major-axis-orientation, no blob area, no σ_xx/σ_yy. (If FPGA later emits those, they could feed a richer 032 variant — but not in this iteration.)

- **NOT in this iteration**: bearing-rate vector (dMidpoint/dt), explicit closing-rate scalar, beacon area/major-axis. The bearing-rate signal is implicit in the 6-tick beacon position history. We're keeping the 032 feature set tight to test whether **span + span-rate + tilt** alone moves the avgInRamp ceiling — if so we get a clean result attribution; if not, that argues for the next-level addition.

Topology impact: NN input layer grows from **45 → 53** (6 span + 1 span-rate + 1 tilt). Weight count grows ~16%; modest.

### 1.6 What's acknowledged + accepted in 032 scope

- **Front-back ambiguity is acknowledged and accepted** for 032. Resolving it cleanly requires either (a) a 3rd target beacon (operator-rejected — adds hardware, doesn't match real-flight ambition) or (b) a perception loop that integrates pose over time (M3-track research, deferred). 032 tests whether richer image-plane features + the existing 6-tick history give the NN enough signal to *predict* (not just react) within the ambiguity envelope. Result is either "ambiguity wasn't the load-bearing constraint" or "we hit a different ceiling and now know we need M3-grade perception."
- **Simulator has oracle access to BOTH chase AND target attitude.** The chase NN's `quat_w/x/y/z` inputs already see chase attitude; target attitude is in the source dmp and the simulator's projection. We can use both oracles **for training-time analysis / ground-truth comparison** (e.g., "what target tilt does the simulator's beacon-pair-on-image-plane reproduce vs the ground-truth target.q_EB roll component?") but the NN itself sees ONLY the derived perceptual inputs. No oracle leakage into the controller.
- **The trajectory we're on**: 2 coded beacons (030 / 032) → eventually non-broadcast visible-light identification + pose recovery (M3). 032 is the last stop before perception becomes its own feature.

---

## 2. Proposed 032 scope (controller-side only)

**030 baseline tuning runs ahead of 032** and may shift the starting point: the 16r → 32r recurrent topology bump is sequenced as 030 work (M11.preA.5). 032 starts against whichever 030 baseline is current.

032 includes ONLY changes that can ship without hardware or FPGA work:

- **(A) Beacon identity propagation** — gather pipeline change. Port-beacon NDC always in the L slots, starboard always in R, regardless of which is on which image side. Zero input-vector size change.
- **(B) Three derived perceptual features**:
  - `beacon_pair_span[6]` — unitless screen-fraction, 6-tick history (target's apparent width on screen — encodes range without wingspan calibration)
  - `span_rate` — "now" only (target's apparent expansion/contraction rate — encodes closing rate)
  - `target_tilt` — radians on the image plane, "now" only (line angle from red→green beacon, NOT chase-attitude-corrected)
- **(C)** Update sim noise / occasion-of-error models so the perceptual features see realistic real-flight error envelopes (beacon detection jitter, occasional sentinel under maneuver, etc.) — gated on bench evidence from 031 phase-1 recordings when they land.

If 032 phase (A) + (B) lifts the fitness ceiling meaningfully → hypothesis "visibility-time signal richness was binding" confirmed; ship. If ceiling stays flat → FOV-blindness or pose-ambiguity hypotheses earn their place; design a follow-on then.

032 explicitly **DOES NOT** include:
- **Calibration of any input to physical units** — everything is fraction-of-screen or angle-on-screen. Same transform sim ↔ real flight.
- **Hardware changes** (031 scope: LED pyramid, optical filter, camera)
- **FPGA wire-format changes** (031-fpga sub-spec; the chase consumes whatever the FPGA emits today: x/y/CEP per beacon — point sources)
- **Beacon size / blob shape / major-axis as inputs** (CEP already encodes detection quality / spread)
- **Bearing-rate vector or explicit closing-rate scalar** (implicit in 6-tick history of beacon positions + the new span[6])
- **3rd beacon on target** (operator decision 2026-05-11: rejected; pose recovery is M3 territory via a richer perception loop, not via more beacons)
- **NN topology / architecture change** (use 030's existing 32→16r→3 hidden layers; just wider input layer 45→53)
- **Front-back ambiguity resolution** (acknowledged + accepted for 032; deferred to M3 — see §1.6 below)

## 3. Open questions (operator triage before /clarify)

### Q1 — Beacon identity propagation: stable ordering or explicit identity field?

Option A: re-order beacon_L / beacon_R inputs by code identity (always port first). 0 input-vector size change.
Option B: add an explicit `beacon_L_is_port` boolean flag per input slot. +12 input slots (×6 history).
Option C: ignore identity, just use code-ID-derived ordering and call it good.

Default: A (no input-vector size change, identity baked into ordering convention).

### Q2 — Which derived features get 6-tick history vs "now"-only? (locked 2026-05-11)

- `beacon_pair_span`: **6-tick history** (trend matters; span shrinks as target accelerates away)
- `span_rate`: **"now" only** (the trend lives in span[6]; rate is the immediate-action input)
- `target_tilt`: **"now" only** (image-plane tilt at the current instant; tilt-history would be redundant with raw beacon position history)

### Q3 — Calibration / unit conversion concerns? (locked 2026-05-11 — NONE)

Everything is fraction-of-screen or angle-on-screen. No wingspan input, no per-craft scale factor, no per-camera calibration. Same input transform in sim AND real flight. The NN learns the empirical mapping between "span = 0.5 unitless" and "target is at meaningful tracking distance" without ever seeing meters. If real-flight cameras have different FOV than sim's 120°, the absolute span values DO shift — but the NN can be retrained with the deployment FOV using the same code, and the transform itself stays unitless.

### Q4 — Does 032 break the 030 wire contract?

030's TrackerInputs is float[45] — a load-bearing serialization contract per the 030 spec FR-006 + FR-019. 032 grows it to **float[53]** (45 + 6 span-history + 1 span-rate + 1 tilt). That's a struct extension; needs a CEREAL_CLASS_VERSION bump or a parallel TrackerInputsV2 struct. **Per the project policy** ([feedback_no_cereal_versioning](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md)) we don't bump CEREAL — so 032 is a greenfield change. Old genomes from 030 are not portable, which matches the "input-transform changes break weight portability" pattern established in M11.preA.2.

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
