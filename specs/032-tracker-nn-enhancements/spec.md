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

### 1.3 First-principles cause: 2-point projection ambiguity

What M1's pathgen NN had as inputs vs what M2's tracker NN has:

| info | M1 (pathgen) | M2 (tracker, 030) | recoverable from M2? |
|---|---|---|---|
| Body-frame bearing to target (3D unit vec) | `target_x/y/z[6]` oracle | derivable from beacon-pair midpoint | YES — straightforward |
| Range to target | `dist[6]` oracle | derivable from beacon-pair angular separation × known wingspan | YES — straightforward |
| Closing rate (dRange/dt) | `closing_rate` oracle | derivable from beacon-pair separation history | YES — straightforward |
| Target's heading direction | implicit via DPHI history | **FRONT-BACK AMBIGUOUS** from 2-point projection alone | NO — fundamental |
| Target's bank/roll attitude | from target.q_EB oracle | partial — beacon-pair line tilt on image plane gives roll modulo the front-back flip | PARTIAL |
| Target's pitch attitude | oracle | invisible (2 beacons on a wing don't constrain pitch) | NO |
| Target velocity vector | oracle | derivable from history but lives in the front-back-aliased space | PARTIAL |

The killer is **front-back ambiguity**: a target heading toward chase at 45° produces near-identical beacon-pair geometry to one heading away at 45°. The chase therefore cannot LEAD the target — it can only react to current bearing. That's why the behavior shape is "intercept-not-track."

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

The information that breaks the front-back ambiguity, ranked by leverage-per-effort:

#### A. Beacon code identity propagation (cheap, high-leverage)

Today the NN labels beacons "left" and "right" by NDC ordering — that's not what the FPGA actually emits. The Gold-code correlator (per [docs/aircraft_tracker_handoff.md](../../docs/aircraft_tracker_handoff.md) §5.5) reports beacon IDENTITY (code A vs code B), which maps to physical port vs starboard wing on the target. Propagating that identity through to the NN gives the chase "I see target's port wing at (x, y)" instead of "the leftmost dot is at (x, y)" — that breaks front-back ambiguity directly.

Today's tracker beacons are labeled by mount position (left wingtip = port = code A, right wingtip = starboard = code B), but the NN input pipeline doesn't preserve identity through the projection — it just sorts by image-plane x. **Fix**: in `gather_tracker_inputs`, deliver beacon_L NDC + beacon_R NDC with IDENTITY-stable ordering (always port first, regardless of which is on which image side).

This is mostly a contract change in the gather pipeline; the FPGA-side code-correlation already does the work. The simulator's `projectBeacon` knows which beacon is which (it has the body-mount Y sign). The chase NN inputs just need to preserve that ordering through the camera transform instead of re-sorting by NDC x.

#### B. Derived pose features as new NN input slots

Compute these in `gather_tracker_inputs` from the beacon pair + chase state, feed as new input slots alongside the existing 45:

| feature | value | history? | replaces? |
|---|---|---|---|
| **Range estimate** (m) | inter-beacon angular separation × known wingspan / 2·tan(fov/2) | yes, 6-tick | nothing — pure addition |
| **Closing rate** (m/s, signed) | dRange/dt over recent history slots | "now" only | nothing |
| **Bearing-rate vector** (rad/s × 2) | dMidpoint/dt in NDC | "now" only | nothing |
| **Target line-tilt** (rad) | angle of beacon-pair vector on image plane | yes, 6-tick | nothing |
| **Beacon-pair separation** (NDC) | raw angular separation between left and right beacons | yes, 6-tick | nothing — useful even if range derivation is also fed |

These don't fully break front-back ambiguity — only beacon-identity-propagation (A) does. But they give the NN direct access to the geometric primitives it currently has to LEARN to infer from 6 history slots of (x_l, y_l, cep_l, x_r, y_r, cep_r). M1 had `dist[6]` and `closing_rate` directly — that's the analog 032 restores.

Topology impact: NN input layer grows from 45 → ~75 (if all features get 6-tick history) or 45 → ~55 (if range gets history but bearing-rate / line-tilt are "now" only). Weight count grows ~30%; manageable.

#### C. Beacon shape / orientation from FPGA (deferred to 031-fpga + 032 jointly)

If the FPGA emits blob major-axis orientation (from CCA stage's σ_xx / σ_yy / σ_xy moments per handoff §6.4), the chase sees beacon-pair line tilt directly without computing from centroids. Bigger lift; needs FPGA implementation. Captures roll attitude.

#### D. Beacon apparent size (deferred, joint with 031-fpga)

Blob area (`bbox_size`, `N` per handoff §6.4) gives range directly without the wingspan-assumption. Helps when target is at oblique angle (wingspan projection foreshortens).

(C) and (D) are joint with 031-fpga because they require new FPGA outputs in the wire format.

---

## 2. Proposed 032 scope (controller-side only)

032 includes ONLY changes that can ship without hardware or FPGA work:

- **(A) Beacon identity propagation** — gather pipeline change
- **(B) Derived pose features** — new input slots: range[6] + closing_rate + bearing_rate × 2 + line_tilt[6] + beacon_pair_sep[6]
- **(NEW)** Update training scenarios to test pose-derived features under front-back-ambiguous geometries (head-on vs tail-on at same off-line angle)

032 explicitly DOES NOT include:
- Hardware changes (031 scope)
- FPGA wire-format changes (031-fpga + 032 joint work — deferred to follow-on)
- 3rd beacon on target (operator decision 2026-05-11: rejected; M3 perception loop addresses pose via different path)
- New NN topology / architecture (use 030's existing 33→32→16r→3, just wider input layer)

## 3. Open questions (operator triage before /clarify)

### Q1 — Beacon identity propagation: stable ordering or explicit identity field?

Option A: re-order beacon_L / beacon_R inputs by code identity (always port first). 0 input-vector size change.
Option B: add an explicit `beacon_L_is_port` boolean flag per input slot. +12 input slots (×6 history).
Option C: ignore identity, just use code-ID-derived ordering and call it good.

Default: A (no input-vector size change, identity baked into ordering convention).

### Q2 — Which derived features get 6-tick history vs "now"-only?

History costs input-vector slots (×6). "Now"-only is cheap but loses temporal context. Defaults per feature in B above; revisit after seeing which features bias the NN's behavior in early bake.

### Q3 — Will derived features make the NN "cheat" on simulator-derived oracles?

Beacon-pair range from wingspan assumes known wingspan. In sim, wingspan is exact. In real flight, target's wingspan is known a priori (it's the operator's other airplane) but with some calibration error. **Check**: derived range error grows with target-aspect angle (oblique view ⇒ apparent wingspan foreshortens). Need a sim-side noise/error model on derived range that matches the real-flight envelope before declaring 032 ships.

### Q4 — Does 032 break the 030 wire contract?

030's TrackerInputs is float[45] — a load-bearing serialization contract per the 030 spec FR-006 + FR-019. 032 grows it to ~55-75 floats. That's a struct extension; needs a CEREAL_CLASS_VERSION bump or a parallel TrackerInputsV2 struct. **Per the project policy** ([feedback_no_cereal_versioning](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md)) we don't bump CEREAL — so 032 is a greenfield change. Old genomes from 030 are not portable, which matches the "input-transform changes break weight portability" pattern established in M11.preA.2.

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
