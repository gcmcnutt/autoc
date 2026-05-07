# Feature Specification: 030 Tracker Mode — beacon-camera target tracking via flight playback

**Feature Branch**: TBD (likely `030-tracker-mode` when this kicks off; currently parked inside the `029-tracker-mode` working branch)
**Created**: 2026-04-29 (originally numbered 029; renumbered 030 in the 2026-04-30 pivot)
**Status**: Parked — gated on [`specs/029-no-future-arch/`](../029-no-future-arch/) clearing
**Input**: User description: tracker mode — train a NN to track another aircraft via simulated wingtip-beacon camera, replacing pathgen with playback of prior training-run flights as targets.

> **PIVOT NOTE (2026-04-30)**: This spec was originally 029. After kicking off the past-only-input baseline experiment (US1), the insight surfaced that no-future-input training is its own architectural problem (controller can only react, not predict — needs a predictor inside the NN). That problem became the new 029 ([`specs/029-no-future-arch/`](../029-no-future-arch/)). Tracker mode (this spec) was bumped to 030 and parked until 029 clears with a working past-only architecture.
>
> **Internal references in this document still say "029"** in many places (e.g., FR-001, US numbers, schema names). Those are accurate as historical text but should be read as "030 future work." A deeper rewrite happens when 030 is unparked.
>
> **What's preserved from the original 029 work**:
> - US1 past-only experiment → moved to [`specs/029-no-future-arch/`](../029-no-future-arch/) as 029's only active US
> - All other content (US2-US6: substrate, camera, training, renderer, review) → stays here as 030
>
> **Prerequisite**: 029-no-future-arch must demonstrate a recurrent NN architecture that trains effectively from past-only inputs. That architecture becomes 030's controller baseline.
>
> **Pose-estimation framing (2026-04-30)**: from two beacon dots `(screen_x, screen_y, visible)` per beacon × history slots, the only physically grounded predictor is one that recovers target **pose** (bearing, bank angle, range, attitude rates) and propagates pose forward via banked-aircraft kinematics. A naive position-extrapolator is fundamentally wrong on a banking target — target follows an arc, extrapolator predicts a straight line. This framing reshapes the 030 controller architecture choice:
> - **Pose-derived geometric features** (inter-beacon midpoint / angle / distance / rates) are first-class inputs alongside the raw `(x, y, visible)` triples — they expose the physical state directly rather than asking the NN to infer it from raw points.
> - **Auxiliary supervised pose head** (per 029 task T062) — supervised against ground-truth target pose during training (pathgen has known rabbit pose; tracker mode has known playback-aircraft pose) — shapes the trunk representation toward pose-awareness. The head structure transfers between 029 and 030 unchanged.
> - **Banked-aircraft kinematics in any predictor stage** — turn radius from bank × airspeed, pitch rate from elevator-equivalent — give the predictor physical truth to lean on, not just function-fitting on past samples.
>
> When this spec is unparked, the controller-architecture section should bake in the pose-estimation framing as the primary perception-to-control bridge, with the raw `(x, y, visible)` triples treated as the *minimal* interface (matching the deployed FPGA centroid extractor) but with pose-derived features computed inside the NN trunk.
>
> **Two-loop architecture (2026-04-30)** — looking past 030 to the eventual pixel-mode perception:
>
> The front/back-aliasing problem in pose estimation (a target heading away at 45° has near-identical instantaneous beacon geometry to one heading toward at 45°) is fundamentally only resolvable from *trajectory*, not from a single observation. This drives a clean perception-control decomposition:
>
> ```
> camera frames → [Pose Perception Loop] → pose + pose_history → [Control Loop] → pitch/roll/throttle
>                  temporal smoothing,                            existing NN-evolved
>                  prediction-aided                                recurrent controller
>                  estimator (CNN/                                 (consumes pose history
>                  transformer + recurrent)                        instead of raw beacon coords)
> ```
>
> | Loop | Update rate | Training | Deploy | Notes |
> |---|---|---|---|---|
> | Perception | camera-fast (30-60 Hz) | supervised against ground-truth pose | perception MCU / FPGA + small NN | prediction-aided; future pose-prior disambiguates ambiguous observations |
> | Control | NN-tick (10 Hz) | GA-evolved (existing pipeline) | small embedded MCU | unchanged contract apart from input-feature semantics |
>
> **Implications for the spec arc**:
>
> - **Pose history can be faked now.** In pathgen / playback modes the target pose is known exactly. Synthesize pose-history with optional noise / dropout (curriculum: anneal noise from 0 to realistic perception-error magnitude over training). Control loop can develop against this interface *independently of any perception-loop work*. This is the highest-leverage decoupling.
> - **Pose history interface = natural successor to current input-history pattern.** Today: `(target_x, target_y, target_z, dist) × 6 slots`. Tomorrow: `(bearing, bank, range, pitch_attitude, plus rates) × 6 slots`. Same shape, different semantics. Today's no-future work (029) carries forward unchanged at the layout level.
> - **Pixel-mode perception is its own feature, not part of 030.** Background declutter, object detection, pose estimation with prediction prior — that's a full feature spec (likely 031 or 032). 030 stays at "FPGA-extracted (x, y, visible) interface that produces beacon-derived pose features," and the pose-history-fed control loop is what 030 trains. The 031 perception loop slots in later, replacing the FPGA centroid extractor with a richer pose stream — control loop unchanged.
> - **Pose history is the contract** between perception and control. Define it crisply when 030 unparks; it'll govern at least three features (030, 031-perception, and any future re-flight against real cameras).
>
> **Staged path for the spec arc** (operator-confirmed 2026-04-30 — also captured in [memory: project_perception_control_two_loop.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_perception_control_two_loop.md)):
>
> | Milestone | Perception | Controller training | What's tested | Controller weights |
> |---|---|---|---|---|
> | 029 (current) — no-future arch | rabbit oracle (none) | GA on past-only inputs | architecture trains predictively | new |
> | **030 — this spec** — sim-beacons learnable | analytic beacon projection | GA on beacon-derived features | perception→control bridge in sim | new |
> | Real-flight-with-beacons | FPGA centroid extractor on real hw | reuses 030 weights | sim-to-real + minimal-calibration (FR-003f) | unchanged |
> | 031+ — pixel-mode | camera→CNN/transformer pose loop | reuses controller (interface = pose history) | full vision stack | unchanged |
>
> The load-bearing property: **controller weights ship unchanged across the bottom three rows**, because the pose-history interface is identical. 030's controller-training output deploys directly to real-flight-with-beacons; once 031 lands, the same controller continues to work behind a richer perception loop. This justifies measured pace — each row uniquely tests one property; rushing any row collapses the test of the next one.

## Overview

Today, autoc trains a controller to track a synthetic *rabbit point* moving along a `pathgen`-generated geometric path. 029 introduces a **distinct training mode** in which the target is **another aircraft** flying a *recorded* trajectory (from a prior pathgen-mode training run), and the controller's only signal about the target is the **2D screen position of two wingtip beacons** projected through a simulated forward-facing camera on the training craft.

The control NN's output remains pitch / roll / throttle. Its inputs change fundamentally: instead of a 3D target position handed to it directly, it receives sparse 2D visual signals subject to occlusion, FOV cutoff, and behind-the-craft loss. The fitness criterion is tracking quality against the recorded target trajectory.

This is the first concrete instance of the **library-based training** strategic direction (operator framing 2026-04-29) and the foundation for the eventual real-target-tracking milestone — the same NN architecture later replaces sim beacons with real-world target-pose inference, with no change to the controller's training framework.

## Clarifications

### Session 2026-05-07

- Q: Arena-awareness NN inputs (FR-016) + trail-rabbit degenerate-velocity fallback (FR-008a) — are the Session 2026-05-04 shapes (4 home inputs; nose-trail fallback) the right v1 shape, or can they be simplified for the smoke run? → A: **Simplify both for v1 smoke; promote complexity later if smoke signal indicates it's needed.**
  - **Arena inputs**: replace `HOME_X / HOME_Y / HOME_Z` (body-frame unit vector toward origin) + `HOME_DIST` (3D Euclidean distance) — 4 inputs — with a single scalar `DIST_TO_BOUNDARY_ALONG_VEL` (meters of safe forward flight before chase intersects cylinder wall, floor, or ceiling, projected along current velocity vector). Single source of truth: the same `arena.h::distanceToBoundary()` function feeds both this NN input AND the per-tick OOB-termination check. The NN learns "small number = trouble; let body attitude / gyro / quat figure out which way to turn." Drops total tracker NN input count `48 → 45`. Rationale: cylinder-shaped arena needs cylinder-shaped signal (radius from vertical axis, not 3D Euclidean); single ray-projection scalar captures both proximity AND heading-into-vs-away-from boundary in one number; lets NN learn turn-recovery from attitude state without explicit bearing input. If smoke signal shows controllers struggle with patrol-recovery without bearing input, the 2-3 input variant (DIST_TO_BOUNDARY + body-frame bearing components) is a 031-candidate follow-up.
  - **Trail rabbit (FR-008)**: keep velocity-vector formulation (`target_pos − target_velocity_unit × 3.048m`) — real-time, not history-derived. **Drop the FR-008a nose-direction fallback for v1 smoke** — for source craft flying normally (which all v1 source dmps are: pastonly3 / more-rnn3 converged controllers in steady forward flight), velocity ≈ nose direction, and `|velocity| < 1e-3` is degenerate-only. v1 fallback when velocity is ~zero: rabbit collapses to target position itself (NN sees rabbit ≡ target; fitness gradient still points toward target). Saves the hysteresis-band complexity for a 031-candidate item if it's actually needed.
  - **Net NN topology**: pathgen `{33, 32, 16r, 3}` (1923 weights, unchanged); tracker `{45, 32, 16r, 3}` (2307 weights, +20%). Same hidden sizes both modes; only input layer width differs.

### Session 2026-05-04

- Q: Crash-penalty mechanic (FR-008b) — scenario-terminator vs fixed-negative-penalty vs per-tick-accumulating? → A: **Scenario-terminating with hull-strike telemetry**. When `p_crash` fires the scenario terminates immediately; no further fitness accrual; the score is whatever was accumulated up to the termination tick. Hull-strike count + termination tick recorded for M11c analytics. The opportunity-cost of forfeited remaining-tick fitness IS the penalty — naturally dominant once `p_crash` plateau is reached. The R3 `p_crash` curriculum (anneal `0.0 → 0.30` by gen 200) already supplies early-gen leniency, so a separate accumulating-penalty layer would double-count the deterrent.

- Q: Smoke-test scale — population × generations × scenarios for the M10 signal-or-not run? → A: **Population 5000 × 100 gens × 6 paths × 20 wind variations × 10 step**. Reasoning: interesting GA behavior doesn't surface until the variation ramp kicks in and a diverse population has had time to evolve structure — population diversity is load-bearing, so 5000 stays. Gens compress to 100 (vs 600+ for a converged run) and variations compress to 20 (vs 49 in pastonly3) and scenarios slice from the source dmp's full 245-294 down to 120 (6 paths × 20 winds). "10 step" is the variation-curriculum step parameter (exact autoc.ini name to be disambiguated in plan-phase task work). This **changes the FR-011 single-scenario v1 framing** — scenario slicing (selecting a path × wind subset of the source dmp) is now load-bearing for v1, not deferred.

- Q: Arena egress treatment (FR-016) + arena-awareness NN inputs? → A: **Scenario-terminating, parallel to crash hull (Option A)**, AND add arena-awareness inputs so the controller can predict edge proximity proactively rather than being reactively terminated. New typed NN inputs (FR-006 extension): `HOME_X`, `HOME_Y`, `HOME_Z` (chase body-frame unit vector pointing toward arming point) and `HOME_DIST` (3D Euclidean distance, meters) — mirroring the pathgen `target_x/y/z[NOW] + dist[NOW]` pattern so the architectural shape is familiar. Scalar (now-only), not history-arrayed (arena center is static; historical home-position adds no information). Total NN input count grows 44 → 48. The data.dat has carried `dhome` per-tick from prior pathgen runs, so the underlying computation is established — "we had this a while ago" — what's new is wiring it through FR-006's typed interface to the NN. Egress telemetry (egress tick + which bound was crossed: radius / floor / ceiling) recorded in M2 dmp parallel to hull-strike telemetry.

- Q: "030 done" line ceiling (D16) AND the implicit-vs-explicit pluggable-NN/sensor requirement? → A: **Option A — ceiling = M10 + M11a + M11b + M11c**. Smoke green floor plus per-tick dmp extractor + eval-fitness Bug 2 fix + tracker-specific analytics. Beyond this, all candidates are already 031-CANDIDATE BACKLOG entries — clean cut. **AND**: make explicit what was previously implicit — the spec needs **pluggable NN-sensor-input gathering** so one `autoc` binary supports both pathgen-mode (M1, 33 inputs) and tracker-mode (M2, 48 inputs) coexisting in tree. Sensor-input gathering becomes a strategy/plugin selected at config-load time per FR-011's mode dispatch; the NN forward-pass code consumes a mode-agnostic typed input array. Xiao firmware uses compile-time mode selection (since deploy targets a single mode at a time, avoiding runtime dispatch overhead). New **FR-019** added to capture this; FR-006 updated to reflect mode-aware enum pattern.

- Q: Pre-versioning dmp handling (FR-015a + Constitution V) — silent fallback / explicit opt-in / hard reject / promote-on-load? → A: **Option A — silent fallback acceptable, because pre-versioning dmps don't actually exist**. The existing `CEREAL_CLASS_VERSION(EvalResults, 1)` in [`include/autoc/rpc/protocol.h:336`](../../include/autoc/rpc/protocol.h) means historical dmps already carry version 1. Plan-research confirms this in M3. **The deeper architectural commitment**: versioning gates are at **milestone boundaries**, not arbitrary code commits. **Within a milestone**, schema churns freely (per [`feedback_no_cereal_versioning`](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md) — pre-M1 incremental versioning was avoided because history didn't matter); **at a milestone boundary**, the schema is **frozen** and that's what the version field discriminates. M1 = `version 1` (the frozen pathgen-mode dmp contract that 029 / pastonly3 / more-rnn3 stabilized). M2 = `version 2` (this 030 tracker-mode bump per FR-015a). Intra-milestone development of M2 schema churn during M5–M8 doesn't require sub-versions; the freeze happens when 030 ships. This refines Constitution V's loud-fail rule to apply *between* milestone-frozen versions, not between every working-development snapshot.

### Session 2026-04-29

- Q: Camera projection geometry + FOV + count for v1 baseline? → A: **Planar pinhole, 120° FOV, single forward-mounted.** Note also: the most likely future variant is *not* stereo (matched cameras for triangulation) but an **asymmetric dual-camera pair** — one wide-FOV (~120°+) for situational awareness + one narrower-FOV for precision tracking. The architecture must support this asymmetric pair without restructuring (FR-003b's "multiple cameras" requirement applies).

- Q: **Minimal-calibration design property** (raised alongside Q1) → A: Across all camera configurations and per-scenario variations, the controller architecture must work with **minimal-to-zero per-unit calibration**. Cheap flight-hardware cameras vary unit-to-unit; any approach that requires precise per-camera calibration (e.g., per-unit intrinsic-matrix tuning, per-unit baseline measurement for stereo) is a non-starter. This reinforces the choice to skip stereo (requires baseline calibration), and means the eventual per-scenario camera variation work ([§Compile-time-fixed vs PRNG-varied parameters](#)) is itself the *robustness training* that delivers minimal-calibration tolerance — variation in mount position / orientation / FOV during training teaches the controller to generalize across uncalibrated units.

- Q: Camera frame rate + how multi-frame-per-tick feeds the NN? → A: **30 Hz camera × explicit history at dphi-pattern time offsets**. Concretely: each NN tick (10 Hz), the NN sees **6 beacon-screen-position samples per beacon-axis at offsets `[-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now]`** (5 past + now, 100 ms uniform grid). This adapts the existing path-tracking input pattern in `include/autoc/nn/nn_inputs.h` (which today uses `[-0.9, -0.3, -0.1, now, +0.1, +0.5]` for the rabbit direction) but is **past+current only** because there is no lookahead for a target craft (it's a real-moving aircraft, not a known parametric path). See [US1 narrative below](#user-story-1--past-only-input-baseline-experiment-priority-p1--first-experiment-gates-the-rest) for the rationale behind the chosen offset distribution.
  - **Preserves the existing 6-slot-per-axis layout**: today's `NNInputs::target_x[6]` / `target_y[6]` / `target_z[6]` / `dist[6]` use 3 past + current + 2 future samples per axis. Tracker mode keeps **6 slots per axis** but redistributes them past-only — the future-sample slots are dropped and replaced with finer-grained recent-past slots.
  - **Per-beacon input shape** (refined by Q5 below to collapse to `(x, y, CEP)` per slot — historically `(x, y, visible)`, revised 2026-05-04 per design note D2): see Q5 for the final 36 + 8 = 44 input layout.
  - **Why 30 Hz over 60 Hz**: 30 Hz × 10 Hz NN tick gives 3 camera frames per tick + a 30-frame rolling buffer for the dphi-pattern selectivity. Fine-grained recent samples (−33 ms, −66 ms) are accessible — *strictly richer* than today's 10 Hz-tick-limited history. 60 Hz would give 60-frame buffers + −16 ms / −33 ms / −50 ms granularity, but at higher continuous compute load on the eventual flight hardware. 30 Hz is the cheaper deployable target; 60 Hz is the US3 sweep variant.
  - **Strategy mapping** (per FR-003e options): this is closest to (c) "latest + derived motion" but using *raw past samples at fixed offsets* rather than computed velocity / accel — same information density, more direct, no derivative computation in the NN's input layer.
  - **NN architecture interaction**: this works well with both recurrent and feedforward NN; the explicit history reduces the temporal-memory burden on a recurrent block (it has the past samples directly) and gives a feedforward NN something concrete to learn from. Couples to Q3 (NN architecture choice).
  - **Future-sample slot for the existing pattern**: deliberately dropped. v1 does NOT extrapolate to fill `+0.1s` / `+0.5s` slots. If experimentation shows the NN benefits from explicit forward-prediction, future iteration can add linearly-extrapolated future samples — but the recurrent NN should learn this implicitly anyway.

- Q: NN architecture for tracker mode? → A: **D-simple recurrent** (16-wide layer 2, ~1923 weights) — continue 028's architecture unchanged. Three reasons: (1) more-rnn3 evidence at gen 700+ demonstrates the architecture works well at this scale; (2) recurrent memory handles loss-of-signal recovery elegantly — the hidden state retains target-trajectory context across signal dropouts where Q2's explicit-history slots become no-signal sentinels; (3) keeping the architecture stable across pathgen-mode and tracker-mode lets a future feature train both modes against the *same* network shape, or transfer-learn between them.

  **Critical perception-boundary clarification**: the NN does **not** consume raw camera pixels. The deployed perception pipeline is **camera → beacon-detection on small FPGA / DSP → coordinate output → NN**. Specifically:
  - The flight-hardware camera produces a raw 2-channel sparse image (per the IR-color baseline architecture — dual-pass IR filter + Bayer sensor, where each beacon's wavelength registers in one color channel).
  - A small FPGA (or dedicated DSP block on the perception MCU) performs **thresholded centroid extraction per channel + a localization-quality estimate** — a near-trivial operation: scan rows, find above-threshold pixels per channel, compute weighted centroid, also compute a circular-error / cluster-spread metric. This is the *only* image-domain processing on the deployed hardware. Output: 3 numbers per camera frame per beacon — `(x, y, CEP)` for left and again for right (see Q5 below for the CEP semantics). Tens of bytes per frame, not megabytes.
  - **The NN input is the centroid + CEP output** — beacon screen coordinates + per-beacon localization-quality, fed through the type-safe sensor interface (FR-006) using names like `BEACON_L_SCREEN_X`. v1 input dimension stays in the ~20-30 float range (Q2's spec), comparable to today's path-tracking input.
  - **The sim mirrors this boundary exactly**: the analytic beacon-projection module in 029 produces the *same* coordinate-domain output the FPGA would produce on real hardware. There is no pixel buffer in the sim's perception path either; the renderer's 1st-person camera-POV mode (FR-012) renders for human inspection only, never consumed by the NN.

  This is what makes the 1923-weight recurrent NN viable for visual tracking: the heavy lifting (image → coordinate) is offloaded to a deterministic, calibration-light, FPGA-cheap front-end, and the NN solves the much smaller *control* problem of "given these beacon coordinates over time, command pitch/roll/throttle." Raw-pixel-to-control end-to-end is a different feature class entirely (017 visual target tracking territory) and is out of scope for 029.

- Q: Fitness formulation? → A: **Reuse pathgen cone-surface fitness** ([`include/autoc/eval/fitness_computer.h`](../../include/autoc/eval/fitness_computer.h)) — per-tick conical scoring against the playback target's position, with the target's position substituted for the rabbit position. Streak / multiplier infrastructure carries forward unchanged. This makes per-axis aggressiveness diagnostics (SC-006) directly comparable to pathgen mode and makes cross-run comparisons (pathgen vs tracker) apples-to-apples on the same fitness scale. Visual-lock-time enters as a *secondary* telemetry signal (not in fitness) — added to the per-gen log alongside fitness, available for analysis but not driving selection until empirical evidence shows it's needed.

- Q: Loss-of-signal handling for explicit history slots? → A: **Per-sample `CEP` localization-quality value for all history slots, plus a dramatic simplification of the per-beacon input** — see "perception-interface refinement" below. Loss-of-signal is **routine, not exceptional** — beacons are visible or not as a normal operating condition. The fitness pressure naturally evolves *orbital search behavior* in winning controllers when target is lost, since cone-surface fitness rewards re-acquiring quickly. The NN does not need a rich "why isn't this beacon visible" signal; it just needs to know how confident the perception output is at each historical time slot.

  **Perception-interface refinement** (raised by operator 2026-04-29; revised 2026-05-04 to fold the visibility signal into the localization-quality channel): the per-beacon, per-frame target input collapses to just `(screen_x, screen_y, CEP)` — three floats. **CEP** here is a *circular-error-probable* style scalar: 0 (or near-zero) when the centroid is sharply localized, growing larger as the cluster spreads or sits near edge / aberration zones, and a **sentinel value** (TBD — likely a large constant or a NaN-marker chosen to be distinguishable from any in-screen value) when the beacon is **off-screen, behind the camera, or otherwise undetected**. CEP therefore subsumes what an earlier draft of this spec called the `visible` flag: the NN gates on CEP magnitude (low → trust the (x, y); sentinel → ignore (x, y), search) and additionally has a continuous trust signal between those extremes (high CEP near edges → discount more strongly than a clean centroid in the middle of frame). The cheap-FPGA / DSP centroid extractor produces all three numbers cheaply (centroid is the weighted moment; CEP is the weighted second-moment / cluster-spread of the same threshold pass). Updated input shape:
  - Per beacon per camera per time-sample (history slot): 3 floats `(x, y, CEP)`
  - **NN-facing dtype is fp32**, but the underlying processing pipeline is **8-bit signed integer** (FPGA / DSP output dynamic range fits int8 for `x ∈ [-1, +1]`, `y ∈ [-1, +1]`, and CEP scaled to a comparable range with sentinel as INT8_MIN). The fp32 conversion is at the NN-input boundary; the controller therefore inherits **int8-quantization-tolerant training** as a load-bearing property of v1.
  - **`x` and `y` are uncalibrated screen-relative values in approximately [-1, +1]** — they are *not* corrected for lens aberrations, mount-orientation offset, or FOV-vs-spec mismatch. The "approximately" is the entire point: per-scenario PRNG-varied lens aberrations + alignment + FOV (FR-003a / camera-config split below) train the controller to tolerate uncalibrated camera pixels — that's how the minimal-calibration property (FR-003f) is delivered.
  - 6 history slots × 2 beacons × 1 camera × 3 floats = **36 floats** for beacon-related inputs
  - Plus aircraft state (quat 4 + airspeed 1 + gyros 3 = 8)
  - Total NN input = **44 floats** (modest growth vs the 33-float pathgen layout)
  - This *replaces* earlier discussion of separate `channel_response`, `in_fov_flag`, `behind_camera_flag`, and the binary `visible` flag — all of those are perception-interface noise the NN doesn't benefit from seeing once CEP carries continuous localization quality and sentinel handling for invisibility.

  This simplification flows downstream into FR-005 (projection output simplifies to `(x, y, CEP)` per beacon-camera tuple), FR-006 (the type-safe sensor interface uses names like `BEACON_L_X[t]`, `BEACON_L_Y[t]`, `BEACON_L_CEP[t]` for t in {-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now} — no `_VISIBLE` / `_CHANNEL` / `_INTENSITY` names), and FR-007 (no-signal handling reduces to "CEP = sentinel value" rather than separate occluded/behind/FOV cases).

**Secondary goal — camera-design sandbox**: the simulated camera model is a parameterized component, not a fixed model. 029 deliberately uses the sim as a vehicle for deciding *what camera characteristics to specify for the eventual flight hardware*. The camera model must support easy variation across mounting position, count (one camera vs stereo / multi-camera), field of view, projection geometry (planar / spherical / etc.), optical aberrations, color filtering, frame rate, latency, and shutter behavior. For v1, configurability via compile-time constants is acceptable; the structural property that matters is that *changing camera parameters* does not require restructuring the training mode.

**Hardware-cost constraint on configuration choices**: the flight-hardware target is **distributed low-compute** (small, cheap MCUs with limited DSP) — *not* a 1 TOPS GPU class platform. This shapes which camera configurations are realistically deployable: high frame rates, very wide FOV with heavy distortion correction, multi-camera stereo fusion, or full-frame NN inference all have real-time-feasibility limits on cheap hardware. The sim configurability surface must include the rich design space (FR-003 / FR-003a / FR-003c / FR-003d) so the operator can *experiment* with rich configs in simulation, but US3's experimental sweeps should be evaluated with an eye toward implementations that fit a constrained perception pipeline. A controller that only works with 90 Hz × stereo × full-frame fisheye-rectified input is interesting research but not a deployable result. Inform the v1 default config and the US3 sweep range with this constraint.

**Compile-time-fixed vs PRNG-varied parameters**: camera parameters split into two classes — same pattern as 025 craft variations applies to airframes:

- **Compile-time fixed (target-hardware spec)**: camera *type* (sensor model), frame rate, latency, projection geometry, color filter / spectral response, shutter mode + scan direction, camera count. These are the choices made when *ordering the flight hardware* — once the hardware is chosen, every flight unit has the same value.
- **PRNG-varied per scenario (manufacturing / installation tolerance)**: mount position (within installation tolerance), mount orientation (within installation tolerance), lens-specific parameters that vary across individual units (FOV within manufacturing spec, aberration coefficients within unit-to-unit variation). These would be sampled per scenario in a future iteration of training, the same way wind / craft are sampled today, to produce a controller that's robust to manufacturing variance across cheap cameras.

**v1 implements only the fixed dimension** — a single deterministic camera config, no per-scenario camera variation. The architectural split must be in place so a future feature can introduce camera variation without restructuring the training mode (analogous to how 025 craft variations adds an airframe-variation axis to the existing joint-PRNG sample).

**Beacon-and-camera baseline architecture (v1 target)**: the v1 hardware-design baseline is **two IR-color-distinguished wingtip beacons + a dual-pass-IR-filter color camera**:
- Left wingtip emits IR wavelength A (e.g., 850 nm); right wingtip emits IR wavelength B (e.g., 940 nm). Each beacon is bright with >180° emission cone (omnidirectional in the relevant hemisphere) so beacon visibility is approximately attitude-independent.
- Camera is a *cheap commodity color sensor* (RGB Bayer-pattern) with a dual-pass optical filter that admits only wavelengths A and B. Visible-light ambient is rejected. The camera's natural color-channel response distinguishes A from B without any per-pixel processing — A registers as one channel, B as another.
- Net effect: the camera produces a sparse 2-channel image where each channel-mapped pixel cluster localizes one specific beacon. Beacon identification (which is L, which is R) is *intrinsic to the optical stack*, not derived. Beacon detection on the deployed hardware reduces to thresholded centroid extraction per channel — fits comfortably in the cheap-MCU budget.

This baseline is what FR-003a's color-filtering stub becomes in v1: **load-bearing, not identity**. It encodes the L/R disambiguation that the controller relies on. Other camera effects (aberrations, motion blur, etc.) remain stubbed-out-but-interface-present per the configurability framework.

**GPU compute requirement for 029**: synthesizing the camera view per tick — even with analytic beacon projection — is non-trivial when multiplied across population × 245 scenarios × 600+ generations × ~30-90 camera frames per NN tick. The compute load is dominated by per-tick projection math (with optional aberrations / rolling-shutter modeling) across ~10⁶+ frame instances per gen. CPU-only is feasible at small scale but becomes the training-time bottleneck at full population. **029 plan should treat the [GPU-Native Evaluation backlog item](../BACKLOG.md) as a likely co-dependency**: either land it ahead of 029 long-run training (US4) or scope 029 v1 to operate at smaller pop / fewer scenarios until GPU eval is available. (This reverses the earlier "029 doesn't need GPU like 017 does" framing — analytic projection is cheap *per call* but multiplies up.)

## User Scenarios & Testing *(mandatory)*

### User Story 1 — Past-only input baseline experiment (Priority: P1) 🎯 First experiment, gates the rest

**Foundational assumption check.** Tracker mode has no lookahead — the target is a real-moving aircraft, not a known parametric path, so the NN cannot see "+0.1s" or "+0.5s" target positions the way pathgen mode can. Before investing in 029's full implementation (mod_robots playback, beacon projection, camera modeling, renderer changes), validate empirically that the recurrent NN architecture from 028 still trains well when the *future* time-sample slots are removed.

**The experiment** runs on the **existing 028 codebase** with one minimal change: redistribute the 6 time-sample slots in `NNInputs` to past-only with dense recent-past coverage. Concretely, the existing pattern `[-0.9s, -0.3s, -0.1s, now, +0.1s, +0.5s]` (3 past + now + 2 future, where the future slots come from path lookahead) becomes **`[-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now]`** — 5 past + now, uniform 100 ms grid, no future. The NN architecture, weight count, recurrent topology, training config (Path A: pop 5000 × 600 gens), and scenario set are all **unchanged**. The only difference is the time semantics of the 6 input slots per target-direction axis.

**Why this distribution** (post-clarify analysis):
- **Drops -1.0s far anchor**: at airframe time-constants (roll 100-300 ms, pitch 200-500 ms), beacon position 1 second ago is a weak predictor of target position now — a maneuvering aircraft completes most of a turn in that time. Far-history information for tracker mode is more useful encoded in the recurrent hidden state than as an explicit input slot.
- **All offsets at multiples of 100 ms** (= 1 NN tick at the existing 10 Hz tick rate). Avoids quantization debt for future camera variants — at 30 Hz camera, sub-tick offsets like -0.05s wouldn't quantize cleanly (1.5 frames back); at 20 Hz camera that becomes valid (1 frame), but for this experiment running on the 10 Hz tick foundation, sticking to 100 ms multiples is the conservative choice.
- **Dense recent past, uniform spacing**: 100 ms uniform grid covers the 500 ms recurrence-horizon of the D-simple block at fine granularity. The hidden state's job is extending context past 500 ms when needed; the explicit input slots' job is providing the precise short-term derivatives.
- **Aligns with tracker-mode-specific assumption**: visual tracking of a target craft is dominated by *recent* dynamics (where is the target *now*, where is it *going next 100-200 ms*). Long history (1+ second back) buys less for visual tracking than for path-tracking with smooth-curve lookahead.

**Why this priority**: This is the *single highest-leverage cheap experiment* for 029. Cost: ~2 lines of code in `nn_input_computation.cc` + comment update in `nn_inputs.h` + one full training run. Information value: validates whether the recurrent NN's hidden state can substitute for the explicit future samples. If yes (late-plateau fitness within ±10% of more-rnn3), 029's whole input model is sound — the NN can track without lookahead. If no, 029 needs architectural rethinking *before* spending months on the full implementation. Runs in parallel with the 028 flight; result is back well before 029 implementation kickoff.

**Independent Test**: Run the experiment to gen 600 (matched compute with more-rnn3). Compare against more-rnn3 on:
1. Late-plateau fitness (last 50 gens, re-evaluated on fixed-difficulty eval per [project_late_run_fitness_interpretation.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_late_run_fitness_interpretation.md)).
2. Per-axis aggressiveness shape (pt / rl / th dCtrl + ⟨|out|⟩ trajectories).
3. Streak quality emergence timing (pctInStreak crossing 30 % around gen X — earlier / on-time / later than more-rnn3).

Pass criterion: late-plateau fitness within ±10 % of more-rnn3, per-axis pattern qualitatively similar (same architecture-consistent bang-bang axis), streak quality emerging on schedule.

**Acceptance Scenarios**:

1. **Given** the existing 028 codebase at the more-rnn3 commit + Path A `autoc.ini`, **When** the operator changes `NNInputs` time offsets in `nn_input_computation.cc` to `[-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now]` (matching comment update in `nn_inputs.h`) and rebuilds, **Then** the build is clean, all tests pass, and a training run launches with the same scenario count, pop, gens, and seed convention as more-rnn3.
2. **Given** the experiment's training run completes 600 gens, **When** the late-plateau fitness is re-evaluated under fixed-difficulty eval and compared to more-rnn3's matched-gen fitness, **Then** the difference is within ±10 % — meaningful comparison, not catastrophic regression.
3. **Given** the experiment's late-run elite, **When** per-axis aggressiveness is computed using the existing 028 tools, **Then** the dCtrl / ⟨|out|⟩ pattern matches the architectural-consistency expectation (per [project_bangbang_axis_migration.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_bangbang_axis_migration.md): roll dominates on RNN architecture) — confirming the architecture is unchanged, only the inputs are.

**Naming convention**: this run is `more-rnn4-pastonly` (or operator's preferred name) — bookends the Path A series and gives 029 its baseline.

**Outputs**:
- `logs/autoc-029-pastonly.log`, `data.stc`, `data.dat`
- `specs/029-tracker-mode/pastonly_outcome.md` documenting fitness comparison + per-axis comparison + go/no-go decision for 029 architecture
- Updated `nn_input_computation.cc` time-offset values (revertable change; whether to keep depends on outcome)

**What this experiment does NOT prove**:
- That 029's beacon-camera input layout works (different sensor pattern entirely)
- That the joint-PRNG-with-library-index variation works
- That mod_robots playback works
- Anything about camera config, frame rate, latency, or perception interface

This experiment validates *only* the assumption "recurrent NN trains well without lookahead inputs." Everything else is downstream.

**Note on alternative time-offset distributions**: the chosen `[-0.5, -0.4, -0.3, -0.2, -0.1, now]` is a uniform-grid recent-past distribution that fits the 10 Hz tick quantization without remainder. If the experiment shows mediocre results, follow-up alternatives to consider:
- **Wider span with logarithmic spacing**: `[-0.9, -0.5, -0.3, -0.1, -0.05, now]` at 20 Hz tick (preserves existing -0.9s anchor for cleanest experimental control). Requires 20 Hz tick rate or upstream quantization adjustment.
- **Wider span at 100 ms grid**: `[-0.9, -0.5, -0.3, -0.2, -0.1, now]` (5 past + now, irregular spacing biased toward recent) — drops the +future / +0.5 future anchor but keeps -0.9 for long-history context. Tests whether long-history matters in the past-only regime.
- **Sub-100ms recent at 30 Hz tick**: e.g., `[-0.4, -0.2, -0.1, -0.066, -0.033, now]` aligned to 30 Hz camera. Becomes relevant if the experiment's outcome motivates exploring sub-tick recent-past fidelity. Out of scope for the v1 experiment which runs at 10 Hz tick.

The ordering above is rough priority for follow-ups if needed.

---

### User Story 2 — Operator launches tracker-mode training from a recorded pathgen run (Priority: P1) 🎯 MVP for full 029

The operator picks a source pathgen-mode training run (e.g., a specific gen of a prior more-rnn3 / pastonly3-class run), points `autoc-tracker.ini` at its S3 dmp, and launches a tracker-mode autoc training run that uses those per-scenario recordings as the target trajectories. Autoc loads the source dmp once at startup (no separate conversion tool — see FR-001) and distributes per-scenario target trajectories to workers analogous to how pathgen distributes path geometry today. Wind and craft variations from the source run carry over per scenario, so each tracker scenario flies in the same wind/airframe conditions as its source flight.

**Why this priority**: This is the gateway capability. Without a way to load the source trajectories and launch tracker mode, nothing else matters. The whole feature exists to unlock this workflow.

**Independent Test**: Given a recorded pathgen run's S3 .dmp output and an `autoc-tracker.ini` referencing it, the operator launches tracker mode and observes a multi-aircraft sim with the training craft pursuing the source-derived target trajectory, with NN training progressing per generation. The 030 run writes its own per-best-of-gen dmp output stream (separate S3 run-id) for renderer playback and analytics.

**Acceptance Scenarios**:

1. **Given** an existing pathgen-mode training run's S3 dmp file (referenced by S3 key in `autoc-tracker.ini`), **When** the operator launches autoc with `-i autoc-tracker.ini`, **Then** autoc loads the source dmp at startup, extracts per-scenario target trajectories from the embedded `aircraftStateList`, and distributes them to workers — no per-scenario playback files are produced as intermediate artifacts.
2. **Given** a tracker-mode run with a loaded source dmp, **When** each generation evaluates the population, **Then** each scenario simulates the chase craft against its assigned source-derived target trajectory under the source scenario's wind / craft / entry variation, and computes 030's own tracker-mode fitness against the trailing rabbit (FR-008) plus crash-hull penalty (FR-008b).
3. **Given** an in-progress tracker-mode run, **When** the operator inspects the per-generation log, **Then** they see fitness statistics analogous to pathgen mode (best/avg/worst, streak quality, per-axis aggressiveness) — all reflecting **the 030 run's own controller**, never anything from the source dmp's embedded fitness — plus tracker-specific signals (visual-lock fraction, occlusion-rate, crash-hull strike rate, beacon-projection statistics).

---

### User Story 3 — Operator experiments with camera configurations to inform flight-hardware choice (Priority: P1)

The operator changes camera parameters (mount position, FOV, count, future-stubbed effects like rolling shutter direction) — typically by editing compile-time constants — and runs short tracker-mode training sessions to observe how the controller's tracking behavior responds. The goal is to use the sim as a deliberate sandbox for choosing what camera characteristics to specify when designing the flight hardware.

**Why this priority**: P1 because **camera-design lead time is on the project's critical path**. The flight hardware spec depends on knowing what camera characteristics work for the controller, and that knowledge can only come from sim experimentation. This experimentation must happen *during* early 029 runs, not after a successful US4 training validates the approach — by the time US4 produces a polished result, hardware decisions need to already have been made. Operationally this US sits between US2 (gateway) and US4 (controlled training): once tracker mode launches, the operator iterates on camera config across short runs to identify a baseline before committing compute to the long-run training of US4.

**Independent Test**: Given the v1 implementation, an operator can change at least three camera parameters (e.g., FOV from 60° to 120°, mount position from nose to canopy, single-camera to stereo-pair) and re-launch tracker training. Each variant runs to completion without errors and produces a comparable per-axis aggressiveness / tracking-quality readout that lets the operator compare configurations.

**Acceptance Scenarios**:

1. **Given** a v1 tracker-mode build, **When** the operator changes the camera FOV constant and rebuilds, **Then** the new training run uses the new FOV consistently across all 245 scenarios, and a re-render of an old scenario with the new build shows the FOV cone updated.
2. **Given** the camera-config interface, **When** the operator switches the configuration to specify two cameras (forward + downward-tilted, say), **Then** training completes against both cameras' beacon projections being available to the NN, and the NN input layout scales accordingly without code restructuring.
3. **Given** a future enable of rolling-shutter modeling, **When** the operator turns it on (with horizontal scan direction), **Then** beacon screen-positions reflect per-row exposure-time skew when the training craft is rotating, and the change requires only flipping the rolling-shutter flag and re-running — no NN training-mode code changes.

---

### User Story 4 — Controller learns to track via beacon-camera signal alone (Priority: P1)

The training NN sees only 2D camera-projected beacon positions plus occlusion / FOV indicators. Across generations, the population evolves controllers that maintain visual lock on the target and minimize tracking error against the playback trajectory, even through transient signal loss (occlusion, target behind craft, target out of FOV).

**Why this priority**: This is the actual research outcome. Without the NN successfully training against the new sensor pattern, tracker mode is just plumbing. P1 because it's the load-bearing experimental result. Sequencing-wise it depends on US3 (a chosen camera baseline must be in place) but is independently testable once that baseline lands.

**Independent Test**: Run tracker mode for a meaningful budget (e.g., 400-600 gens at population 5000) and demonstrate that fitness descends meaningfully and the late-run elite maintains visual lock on the target trajectory across most of the scenario landscape, including recovering from short loss-of-signal events.

**Acceptance Scenarios**:

1. **Given** a fully wired tracker-mode setup with a chosen camera baseline, **When** training runs from gen 0, **Then** the elite's tracking error decreases monotonically (in trend) across generations with the same shape signatures observed in pathgen mode (initial exploration, streak quality emergence, late-run plateau).
2. **Given** a late-run elite, **When** evaluated on a scenario where the target briefly exits the FOV, **Then** the controller continues an estimated trajectory rather than catastrophically losing position, and re-acquires visual lock within a small number of ticks once the target re-enters FOV.
3. **Given** the per-axis aggressiveness telemetry from 028, **When** applied to a tracker-mode run, **Then** the same diagnostics (per-axis dCtrl, ⟨|out|⟩) work without modification — the controller's output behavior remains comparable to pathgen-mode controllers and the same smoothness analysis applies.

---

### User Story 5 — Operator inspects tracker-mode training in the renderer (Priority: P2)

The renderer displays both aircraft (training craft + playback target) in 3rd person, AND offers a 1st-person camera-POV view showing what the training craft's camera sees (beacons projected onto the screen-plane). The operator can scrub through a recorded scenario per tick (pause / step-forward / step-backward) in either mode and see exactly when visual lock was maintained vs. lost.

**Why this priority**: Diagnostic clarity. Once US2/US3/US4 produce data, this is how the operator interprets it. P2 because training can succeed without it (with text-only logs), but the iteration cycle is much slower without visual debugging — and the camera-config experiments in US3 are *significantly* more interpretable with the camera-POV mode (it shows directly how a configuration choice changes what the controller sees).

**Independent Test**: Given a completed tracker-mode generation's S3 dump, the operator opens the renderer, switches between 3rd-person and 1st-person camera-POV views, scrubs to a specific tick, and identifies per-tick beacon channel response, projection coordinates, and lock state. Both viewing modes work; per-tick scrub controls are responsive in both.

**Acceptance Scenarios**:

1. **Given** a recorded tracker-mode scenario in S3, **When** the renderer loads it in 3rd-person mode, **Then** the training craft and target craft are both displayed simultaneously, the L/R beacons are visible as colored points on the target's wingtips (per their channel response), the camera FOV cone from the training craft is drawn, and per-tick error metrics overlay updates as the operator scrubs.
2. **Given** the same scenario, **When** the operator switches to 1st-person camera-POV mode, **Then** the view changes to show what the training craft's camera sees — beacons appear as colored points at their projected screen positions, FOV bounds are the screen edges, and any aberration / rolling-shutter effects (when enabled in the camera config) are visible directly in the rendered frame.
3. **Given** either viewing mode, **When** the operator pauses and steps forward / backward by single ticks, **Then** the camera-POV / 3rd-person renders update per-tick, the overlay metrics update, and the operator can identify exactly the tick(s) at which visual lock was acquired or lost.
4. **Given** a sequence of camera configs being compared from US3, **When** the operator renders the *same scenario* under each config in camera-POV mode, **Then** the visual differences (FOV change, projection geometry change, etc.) are immediately apparent — making the renderer a direct sandbox for camera-design comparison.

---

### User Story 6 — Real-target-tracking bridge readiness (Priority: P3)

The 029 architecture is structured so that, in a future feature, the sim-beacon-projection layer can be replaced by a real-world target-pose inference layer (e.g., from camera vision, beacon detection, or radar) without changing the trained NN itself. The control-NN architecture, fitness framework, and training mode persist; only the perception layer changes.

**Why this priority**: This is a forward-looking architectural property, not a v1 capability. P3 because it informs design choices in 029 (clean separation between "how we get target signal" and "what the controller does with it") but is not directly delivered.

**Independent Test**: Demonstrate that the NN's input layout and the perception-to-NN interface are designed such that swapping the sim beacon-projection module for a hypothetical real-perception module would not require retraining the controller. (Validated by code review at end of 029 implementation; no real-perception module needs to exist yet.)

**Acceptance Scenarios**:

1. **Given** the 029 implementation is complete, **When** an architecture review is performed, **Then** the control NN is provably independent of how its inputs are sourced — the same NN binary would work against sim beacons, real beacons, or any other module that produces equivalent (x, y, occlusion-flag) tuples.
2. **Given** the spec for a hypothetical follow-on real-perception feature, **When** drafted, **Then** it can integrate cleanly without altering the 029 control-NN training mode or its fitness framework.

---

### Edge Cases

- **Target behind training craft (no signal)**: Both beacons are out of FOV. NN must handle "no input" gracefully — predicted recovery, search behavior, or maintain heading until re-acquisition.
- **One beacon visible, one occluded** (e.g., target banked relative to camera so one wing hides the other): NN sees partial information. Must still derive target attitude/range from a single beacon's position over time.
- **Target very close**: Beacon projections diverge widely on the screen. NN must handle high-angular-velocity inputs without bang-banging.
- **Target very far**: Beacons converge to nearly the same screen point. Range disambiguation is poor. NN must handle low-precision inputs.
- **Source-run scenario crashed**: A .dmp scenario where the source run's aircraft crashed mid-scenario produces a partial-trajectory playback. Handle by either truncating the tracker scenario at the crash point or filtering crashed scenarios out at source-dmp load time (per FR-013 — decision pending in plan / tasks; the source dmp itself IS the library per FR-001 revision 2026-05-04, so "library construction" no longer applies as a separate step).
- **Camera frame-of-reference under aircraft motion**: The camera moves with the training craft's body frame. NN inputs must be in a consistent, well-defined frame — and the NN must learn to compensate for its own motion-induced beacon shift.
- **Beacon projection numerical edge cases**: Target behind the camera plane (negative z in camera frame) must produce the no-signal indicator, not garbage screen coordinates.
- **Rolling shutter under high body rates**: a real flight camera with a rolling shutter sees beacons at different "exposure times" depending on screen y-position (or x-position, depending on scan direction). When the training craft is rolling/pitching fast, this causes apparent skew in beacon positions — the top of the frame samples the world at a slightly different moment than the bottom. v1 may stub rolling-shutter behavior as global-shutter (no skew), but the model's interface must accommodate the rolling-shutter case so a future enable doesn't require restructuring. Direction matters: horizontal rolling shutter under rapid roll creates different artifacts than vertical rolling shutter, and which is preferable for the controller is a design question this sim is meant to answer.
- **Multi-camera configurations**: a future variant might use two cameras (stereo for range disambiguation; or one forward + one downward for landing-style applications). The NN-input layout must scale with camera count without introducing a new architecture per configuration.
- **Frame rate ≫ NN tick rate (the multi-frame-per-tick case)**: at 90 Hz camera vs 10 Hz NN tick, 9 frames arrive per NN evaluation. The NN architecture choice for handling this (latest-only / N-stack / derived-motion / sub-tick recurrent) materially changes input dimension and what temporal information the controller sees. Frame rate becomes a knob that interacts with NN input layout — changing it requires re-deriving the input layout, not just a constant tweak.
- **Camera latency under high body rates**: with non-zero camera latency, the world has advanced by latency × angular_velocity worth of beacon shift between when a frame was captured and when the NN consumes it. The NN must learn to predict ahead, OR the controller's bandwidth must be lower than the camera latency permits. This is a design question — the sim is meant to expose how much latency the controller can tolerate before tracking degrades.
- **Frame rate / latency / NN tick rate interaction**: these three are coupled. A 90 Hz camera at 50 ms latency feeding a 10 Hz NN tick produces fundamentally different controller dynamics than 30 Hz camera at 0 ms latency. US3 camera-config experiments should sweep these as related variables, not in isolation.
- **Projection geometry at high FOV**: planar/rectilinear projection becomes unstable as FOV approaches 180° (objects near the edge stretch infinitely). Spherical/fisheye projection handles wide FOV gracefully but introduces curvature in the beacon-screen-coordinate space the NN must learn to interpret. This is a design choice with real consequences: a 90° planar gives clean angular relationships near center but cuts off the periphery; a 180° fisheye sees everything in front but distorts. US3 should explore both; the NN may behave very differently under each.
- **Aberration coupling with motion**: chromatic aberration matters most when color filtering is in use to distinguish beacons from background; if v1 stubs color filtering, chromatic aberration is moot. Motion blur becomes severe when the target's *angular* velocity in the camera frame is high (fast cross-traffic at close range), interacting with frame rate / latency. These coupling are why FR-003a treats aberrations as a unified extensibility surface rather than as orthogonal toggles.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST load a recorded source-run S3 .dmp file (a prior pathgen-mode training run's per-best-of-gen output for a specific gen, or a future real-flight-recorded trajectory bundle) **directly into autoc at startup** and use the per-scenario `aircraftStateList` plus joint-PRNG variation parameters embedded in the dmp as the per-scenario target-trajectory + wind/entry/craft seed source. **No separate conversion tool / no per-scenario playback files** — the source dmp itself is the "library" (revised 2026-05-04 from the earlier conversion-tool model). The architectural property that matters: per-scenario target-craft pose-per-tick is held in autoc memory after startup load and distributed to workers in the same shape pathgen output is distributed today.
- **FR-002**: System MUST make the per-tick target-craft pose available to the autoc-side beacon projection module (FR-005) for each scenario — sourced from the in-memory `SourceScenarioTrajectory` loaded per FR-001. **v1 deferral note (2026-05-04)**: v1 satisfies this with **autoc-memory-only** target-craft pose access — the projection module reads `target_pose_at_t_i` directly from memory; **crrcsim runs the chase craft alone**. Live two-aircraft display in crrcsim's 3D viewer during training (i.e., crrcsim hosting both via `mod_robots/RobotProgrammable`) is **deferred to post-v1** (a 030 v1+ / "video-enabled mode" line item — see [BACKLOG.md "[030 v1+]" entry](../BACKLOG.md) when added). The renderer's 3rd-person playback view of both aircraft (FR-012) reads target pose from the M2 dmp's copied `targetTrajectoryList` (FR-015 self-containedness) using its own VTK actors — also no crrcsim mod_robots dependency.
- **FR-003**: System MUST attach one or more virtual cameras to the training craft's body frame. Each camera's mounting position (offset from CG), orientation, **projection geometry** (planar / rectilinear pinhole, spherical / fisheye, optionally cylindrical or orthographic), **field of view** (degrees, horizontal + vertical, must accommodate wide-FOV cases up to and including hemispherical), and other projection-model parameters (e.g., focal length, principal point) MUST be configurable parameters (compile-time constants acceptable for v1) — *not* hardcoded values.

  **v1 baseline (refined 2026-05-04)**: **single camera**, **planar pinhole**, **120° FOV**, **30 Hz frame rate** (3 frames per 10 Hz NN tick), mounted **on top of the training craft's wing at the wing-chord centerline** (forward-facing), aligned with the body +x axis. This mount differs from a generic forward-fuselage location: it puts the camera *above* the airframe's primary occluding mass (fuselage + wings below it), which is the cheapest-deployable real flight-hardware mount — but the camera's downward field of view IS occluded by the wing it sits on, and *both* downward and forward field-of-view cones intersect the propeller arc on a tractor configuration.

  **Self-occlusion modeling**: the v1 camera projection module MUST model **airframe self-occlusion** (the aircraft body blocks part of the camera's geometric FOV) at first-order fidelity — i.e., when the line from camera to a beacon passes through the training craft's own airframe-volume, the beacon registers as occluded (CEP = sentinel) regardless of its FOV / channel / range. v1 acceptable approximation: a coarse body-shape proxy (e.g., a wing-chord-aligned box + tail boom) keyed to the training craft's CG and quat. Operator has reference video of the real airframe's apparent occlusion geometry from this mount position, available as research input during plan phase.

  **Prop-arc occlusion** is **out of scope for v1**. On a tractor (front-prop) airframe with a forward camera, the spinning prop intermittently occludes part of the FOV at a rate that interacts with the camera shutter (rolling-shutter resonance with prop RPM produces banding / strobe artifacts; global-shutter at non-resonant rates produces clean intermittent occlusion). This is real and load-bearing for the deployed real-flight pipeline but adds substantial sim complexity; v1 deliberately defers it. Research candidate: capture prop-occlusion geometry from a benchmark flight + decide modeling approach (analytic blade-arc subtraction vs noise-floor approximation) in a follow-on iteration. Mark as a known sim-to-real gap when v1 ships.
- **FR-003a**: Camera model MUST be **structurally extensible** to support optional effects without changing the training mode's architecture. Specifically:
  - **Optical aberrations**: radial distortion (barrel and pincushion), tangential distortion, chromatic aberration (per-channel focal-point shift, relevant when color filtering is in use), vignetting (intensity falloff toward image edge), defocus / motion blur (when the target moves faster than the camera can resolve sharply).
  - **Color filtering / spectral response**: per-channel sensitivity that affects beacon visibility (e.g., a Bayer filter or narrow-band filter that admits beacon LED wavelengths preferentially).
  - **Shutter behavior**: global vs rolling; rolling shutter direction — horizontal or vertical scan.

  v1 may stub all of these as identity / no-op; the *interface* must be in place so a future change to enable any of them does not require touching the NN training pipeline.
- **FR-003b**: System MUST support multiple-camera configurations as a future-friendly property — i.e., the architecture must NOT assume exactly one camera. v1 implements single-camera only, but the data flow from "camera N produces beacon projection" to NN inputs must accommodate camera count > 1 without restructuring. **Plausible future multi-camera variants the architecture must accommodate** (none is committed; sweep them in US3-style experiments):
  - **Asymmetric dual** — wide-FOV + narrow-FOV pair, complementary coverage (situational awareness + precision tracking).
  - **Stereoscopic dual** — two matched cameras with a known baseline for explicit depth recovery; doubles compute and cost, conflicts with the minimal-calibration property (FR-003f) since stereo needs baseline calibration. May still be worth experimenting with even though it's not the favored deploy direction.
  - **Wide + telephoto, slightly offset** — same family as asymmetric but with deliberate small mount offset for parallax-derived depth on near targets. Hybrid of the two above.
  - **Forward + downward-tilted secondary** — wider effective FOV without going to fisheye geometry on a single sensor.

  The architecture must not assume cameras have matching configs, matching mount positions, or matching projection geometries. Per-camera parameter blocks are independent.
- **FR-003f**: The control architecture MUST accommodate **minimal per-unit camera calibration**. The controller's behavior must not depend on precise knowledge of a specific camera unit's intrinsic parameters (focal length to sub-pixel, lens distortion coefficients to high precision, exact mount alignment, etc.). Real flight-hardware cameras vary unit-to-unit; the deployable result must work without per-unit calibration steps. The architectural mechanism that delivers this is the [PRNG-varied per-scenario camera-variation work](#) (deferred but architecturally enabled in v1) — training across uncalibrated-camera variations is what produces calibration-tolerant controllers.
- **FR-003c**: Camera **frame rate** MUST be configurable independently of the NN evaluation tick rate. Sim NN tick is 10 Hz (100 ms / tick); flight-hardware candidate cameras run at 30 / 60 / 90 Hz — 3 / 6 / 9 camera frames per NN tick. The architecture must accommodate frame-rate values in this range without restructuring. (v1 default frame rate: see open clarify questions.)
- **FR-003d**: System MUST model camera **latency** — the delay between physical world state and the camera frame the NN sees — as a configurable parameter. Real cameras have non-zero exposure-plus-readout-plus-processing delay; the sim must reflect this so the controller learns to compensate. v1 may default to 0 ms but the interface must accept and apply non-zero latency values. (v1 default latency: see open clarify questions.)
- **FR-003e**: System MUST define how multiple camera frames per NN tick feed into NN inputs. Frame rate > NN tick rate means N=3-9 frames are produced between consecutive NN evaluations; these are *history* available to the NN. Options include: (a) NN sees only the latest frame, (b) NN sees a stack of the last N frames as parallel input columns, (c) NN sees latest frame + derived velocity / acceleration from prior frames, (d) NN runs at camera frame rate (sub-tick updates) with recurrent state carried across frames. The chosen approach must be parameterized so US3's camera-config experimentation can compare them. (v1 strategy: see open clarify questions.)
- **FR-004**: System MUST place two wingtip beacons on the target aircraft at well-defined body-frame positions — one at each wingtip — each with a configurable **emission wavelength** (v1 baseline: distinct IR colors A and B for L vs R) and a configurable **emission cone**. **v1 baseline emission cone: ~270° outward-facing per beacon**, mounted so that each wingtip beacon's optical axis points laterally outward from the wing tip. The 270° cone (vs the >180° hemisphere of an earlier draft) reflects realistic LED + diffuser geometry and is wide enough that **at least one of the two beacons is visible from any direction in which the target craft itself is in view** — but each beacon individually CAN still be obscured (by the target's own wing/fuselage when the camera looks across the airframe, by the target attitude tilting one wingtip behind the body, by ambient backscatter, etc). The two-beacons-with-distinct-wavelengths-and-outward-cones design is what makes L/R disambiguation intrinsic to the optical stack while gracefully degrading to single-beacon inputs when one tip is occluded.

  **Beacon emission cone, mount-axis orientation, and wingtip body-frame position are explicitly designated as both PRNG-variation axes and experimental-design subjects** (parallel to camera mount + FOV in §"Compile-time-fixed vs PRNG-varied parameters" above):
  - **As variations**: per-scenario PRNG sampling of cone width / cone axis / wingtip mount position lets the controller train robustness to LED + diffuser unit-to-unit variance and target-airframe diversity (a future tracker target may not be hb1-shaped). v1 may hold these at the nominal 270° / outward / wingtip values, but the architectural split must be in place so a future iteration introduces sigmas without restructuring.
  - **As experimental-design subjects**: cone width and mount axis directly trade detection coverage against ambient interference and self-occlusion. The current 270° / outward baseline is a *first cut*, not a fixed answer. US3-style camera-config experiments must extend to beacon-config experiments — sweep cone width, mount axis (outward vs upward-tilted vs forward-tilted), wingtip extension distance — and observe controller fitness / lock-quality / occlusion-rate. The result feeds beacon-hardware spec the way camera experiments feed camera-hardware spec.
  - **Crossed-experiment expectation**: beacon cone × camera FOV × camera mount form a joint design space. Optimal beacon geometry depends on camera config and vice versa; experiments must be capable of varying them together. Don't hardcode "270° outward" as a permanent decision in any tooling that consumes beacon parameters.
- **FR-005**: System MUST project, each tick, the two beacons through each training craft's camera into 2D screen coordinates, producing for each (camera, beacon) tuple **a minimal 3-float output: `(screen_x, screen_y, CEP)`** (per Q5). `screen_x` and `screen_y` are **uncalibrated screen-relative values in approximately [-1, +1]** — relative to the camera frame's edges, not corrected for lens aberrations, mount-orientation offset, or FOV manufacturing variance. `CEP` is a circular-error-probable-style localization-quality scalar: small (≈ 0) when the centroid is well-localized in the middle of frame; larger when the centroid sits near the edge of frame, in an aberration zone, on a fast-moving target (motion blur), or other degradation; **a designated sentinel value (e.g., a large constant or NaN-marker)** when the beacon is off-screen / behind camera / undetected. The `(x, y, CEP)` triple *exactly mirrors* the deployed FPGA centroid extractor's output (centroid + cluster-spread + threshold-fail sentinel from a single-pass thresholded-row scan). Internally the projection module computes the underlying conditions for diagnostics / renderer use (FR-012, FR-015), but the NN-facing interface is the simplified 3-float tuple. Per-coordinate dynamic range fits **8-bit signed integer** in the perception pipeline (FPGA / DSP); fp32 conversion happens at the NN-input boundary (FR-006).
- **FR-006**: System MUST present beacon projection results to the control NN through a **type-safe sensor interface** (rolled in from the [Type-Safe NN Sensor Interface backlog item](../BACKLOG.md)) — *not* as opaque float[] indexed by magic numbers. Each NN input is named with type / units / range metadata. Concrete naming for v1 (per Q2 history pattern + Q5 simplification): `BEACON_L_X[-0.5s]`, `BEACON_L_X[-0.4s]`, `BEACON_L_X[-0.3s]`, `BEACON_L_X[-0.2s]`, `BEACON_L_X[-0.1s]`, `BEACON_L_X[now]`, `BEACON_L_Y[...]`, `BEACON_L_CEP[...]` for each of the 6 time offsets, mirrored for `BEACON_R_*`. Total per-beacon: 6 × 3 = 18 named inputs; per camera × 2 beacons = 36 beacon-related inputs. **Plus aircraft state**: 8 inputs (quat × 4, airspeed × 1, gyro × 3). **Plus arena-awareness** (~~added 2026-05-04~~ **amended 2026-05-07 per Session 2026-05-07 Q1**): **1 input** (`DIST_TO_BOUNDARY_ALONG_VEL`, scalar meters along velocity vector to nearest cylinder/floor/ceiling intersection — single source of truth shared with `arena.h::distanceToBoundary()` per-tick OOB check). **Total tracker-mode NN inputs = 36 + 8 + 1 = 45** (was 48 in the 2026-05-04 shape). NN dtype is fp32 at this interface; the underlying perception pipeline values are int8 (per FR-005) and converted at the boundary, so the controller trains tolerant to int8 quantization noise. This both (a) prevents the silent-corruption class of bugs that hit prior topology changes and (b) is required by the [Genome ablation tool backlog item](../BACKLOG.md) which masks inputs by name (e.g., `--zero-input BEACON_L_*` to ablate the left-beacon entirely; `--zero-input BEACON_*_CEP[*]` to ablate the localization-quality channel and force the NN to use raw coords only; `--zero-input DIST_TO_BOUNDARY_ALONG_VEL` to ablate arena-awareness). 030 *implements* the type-safe interface; the ablation tool *consumes* it.
- **FR-007**: When a beacon is not visible (out of FOV, behind the camera, or not detected), the system MUST emit `(screen_x = 0, screen_y = 0, CEP = SENTINEL)` for that beacon's coordinate slot, where `SENTINEL` is the designated invisibility marker (FR-005). The CEP value is the load-bearing signal — `screen_x` / `screen_y` are arbitrary when CEP is sentinel (zero is convention but the NN learns to gate on CEP, so the (x, y) values during invisibility don't drive output).
- **FR-008**: System MUST evaluate fitness based on tracking quality against a **trailing rabbit point**, not the target's own position. The rabbit is computed each tick as `target_position − target_velocity_unit × trail_distance`, where `trail_distance` is a configurable parameter; **v1 default = 10 ft ≈ 3.048 m** (committed 2026-05-04), tunable in `autoc-tracker.ini`. The existing pathgen cone-surface fitness ([`include/autoc/eval/fitness_computer.h`](../../include/autoc/eval/fitness_computer.h)) is reused with this trailing rabbit substituted for the path point. Streak / multiplier infrastructure carries forward unchanged. **Rationale**: rewarding the chase craft for sitting *on* the target's instantaneous position incentivizes collision; rewarding it for sitting *at the trail point* incentivizes tracking from a stable in-trail aspect — which is also the right precondition for the crash hull (FR-008b) and for real-target dogfight semantics. The perception input to the controller is unchanged — the NN still sees the **actual target's beacons**, not the rabbit point. The split between "where the perception sees" and "where the fitness rewards being" is the load-bearing teaching signal: the controller learns the offset implicitly from fitness gradient. Visual-lock-time fraction is added to per-gen telemetry as a *secondary* signal but does NOT participate in fitness selection. (See clarify Q4 for cone-surface rationale.)
- **FR-008a**: System MUST recompute the trailing rabbit *every tick* from the playback target's current pose+velocity (not from a pre-computed offset trajectory at library-build time). This preserves correctness when the target's velocity changes direction sharply — the rabbit always trails the *current* heading, not the heading at recording time. ~~Edge case: when target speed approaches zero (low-airspeed slow-flight or sim glitch), `target_velocity_unit` is undefined; the v1 fallback is to use **target body-frame `+x` (nose forward)** rotated to world via the target's quat~~ **Amended 2026-05-07 per Session 2026-05-07 Q1**: nose-direction fallback is **deferred to a 031-candidate backlog item** for v1 simplicity. v1 source dmps (pastonly3 / more-rnn3-class) are converged controllers in steady forward flight — `|target_velocity| < 1e-3` is a degenerate-only case in practice. v1 fallback when velocity is ~zero: rabbit collapses to target position itself (`rabbit = target_pos`, NN sees rabbit ≡ target; fitness gradient still points toward target). Promote to nose-direction trail if smoke signal indicates need (e.g., source dmps that include hover / stall sequences).
- **FR-008b**: System MUST support a **crash-on-intercept hull** around the actual target craft (NOT around the trailing rabbit). The hull is **enabled in v1 from day one** (not an optional follow-on). When the chase craft's position enters the hull at any 10 Hz fitness-evaluation tick AND `p_crash` fires for that tick, a "crash" event triggers and the scenario **terminates immediately** (clarified 2026-05-04, see [Clarifications session](#session-2026-05-04)) — no further fitness accrual; the scenario's final score is whatever was accumulated up to the termination tick. The "penalty" is the **opportunity cost of forfeited remaining-tick fitness** — naturally dominant once `p_crash` plateau is reached, with no separate negative-magnitude penalty added. Hull-strike count + termination tick are recorded as M2 dmp telemetry per FR-015 for M11c analytics. **v1 hull = `SPHERE` of 1 m radius** around target CG (committed 2026-05-04 — chase craft and target are both ~hb1-sized, so a 1 m sphere is a coarse-but-honest first approximation; airframe-shape fidelity comes later). Hull shape is configurable for future evolution:
  - **`SPHERE`** (v1): radius `r` around target CG. v1 default `r = 1 m`.
  - **`AABB_HB1`**: axis-aligned-in-target-body-frame box approximating the hb1 airframe extents (wingspan × chord × thickness), oriented by target's attitude per tick. Replaces sphere approximation with airframe-shape fidelity.
  - Future: **`MESH_<airframe>`** — when target airframe varies (per 025 craft variations), the hull tracks the actual airframe.

  **Probabilistic firing — committed for v1**: each tick at which the chase craft is inside the hull, the crash event fires with probability **`p_crash` per step** (configurable in `autoc-tracker.ini`). v1 default value TBD by plan (suggest starting around `p_crash = 0.05–0.15` per 100 ms tick; needs empirical tuning against fitness-landscape navigability). The probabilistic formulation supports curriculum learning (anneal `p_crash` low→high across gens) and softens the fitness-landscape cliff that a deterministic crash would impose on early-gen exploration. Deterministic-mode (`p_crash = 1.0`) is recoverable as a degenerate case of the same parameter.

  **crrcsim mode dependency**: collision-detection-with-target is a tracker-mode-only operation; pathgen mode does not have a target craft to collide with, so the existing crrcsim integration MUST gate this code path on the active mode. Ground-collision detection (existing crash logic) stays in both modes; target-hull collision is additive only when `Mode = Tracker`.
- **FR-009**: System MUST preserve absolute determinism: same Seed and same source library produce identical sequences of evaluations and identical fitness numbers.
- **FR-010**: System MUST preserve the joint-PRNG variation model — each scenario remains one PRNG-seeded joint sample. **Source-scenario-index** becomes one of the joint axes alongside wind/entry/craft (which are inherited from the source recording per FR-001).
- **FR-011**: System MUST support a new operating mode (selected by a separate config file, **`autoc-tracker.ini`**, alongside the existing `autoc.ini`) without breaking pathgen-mode training. Both modes MUST coexist in tree. The tracker-mode config is structurally a sibling of `autoc.ini` (same parser, mostly the same parameters) with the following deltas:
  - **Source-library reference**: a tracker-mode-specific parameter (e.g., `TrackerSourceRun = autoc-storage/<run-id>/gen<N>.dmp`) names the prior-run S3 dmp file(s) to use as target trajectories. Naming follows the same S3-key convention the xiao firmware uses for NN program references (profile prefix + bucket-key path), so the same key string round-trips between xiao logs and tracker config without rewriting. v1 expects to consume the dmp directly — the per-scenario target trajectories and joint-PRNG variation parameters embedded in the dmp are the library entries.
  - **Mutual exclusion with pathgen**: tracker-mode and pathgen are mutually exclusive — `autoc-tracker.ini` does NOT carry pathgen-specific parameters (path family selection, path-segment count, etc.); `autoc.ini` does NOT carry tracker source references. The autoc binary picks its mode based on which config file is passed (`-i autoc.ini` vs `-i autoc-tracker.ini`) — there is no boolean flag flipping mode within a config.
  - **Wind-and-entry replay assumption** (v1): tracker scenarios assume **the source dmp's wind seed and entry pose reproduce identically** when re-fed through the joint-PRNG (i.e., same scenario index → same wind / entry / craft sample). The C2 open consideration below (deterministic replay) flags the case where this assumption breaks; the v1 fallback is to bump the dmp schema version (per Constitution V) and embed wind + variation sub-seeds directly in the file rather than re-deriving them from scenario index.
  - **Scenario slicing** (load-bearing for v1 per 2026-05-04 clarification): the tracker config MUST support selecting a subset of source scenarios — at minimum, by **path index list × wind variation index list** (cross-product) so smoke-test selections like "6 paths × 20 winds = 120 scenarios" are concise to express. v1 syntax sketch (concrete form is plan-phase): `TrackerPathSubset = 0,1,2,3,4,5` + `TrackerWindSubset = 0-19` produces the smoke-test slice; defaulting both to "all" reproduces full-dmp coverage when an operator wants it. The earlier "single scenario index" deferral is **withdrawn** — smoke-test scale (5000 pop × 100 gens × 120 scenarios × 10 step variation curriculum, per Clarifications session 2026-05-04) requires the 120-scenario slice as the *minimum*, so the scenario-selection mechanism is on the v1 critical path.
- **FR-012**: Renderer MUST support **two viewing modes** for tracker-mode playback, plus an always-on perception overlay. **The renderer reads target-craft trajectory + attitude from the M2 dmp directly** (per FR-015's self-contained storage scheme); it does NOT reach into the original M1 source dmp at playback time. This keeps M2 dmps fully portable for analytics — an operator handed only the M2 dmp can play it back without needing the source artifact.
  - **3rd-person view**: both aircraft (training craft + playback target) rendered in the scene, beacons visible on the target wingtips, camera FOV cone drawn from the training craft, per-tick error metrics overlaid. (US5 primary use case.)
  - **1st-person camera-POV view**: render *from the training craft's camera* — i.e., what the controller "sees." Beacons appear as colored points (per their channel response) at their projected screen positions; FOV bounds are the screen edges; aberrations / rolling-shutter effects (when enabled) are visible directly. This is the most direct way to debug perception-pipeline questions and is uniquely valuable for camera-config experimentation (US3).
  - **Camera-POV mini-panel** (a small 2D overlay near the existing throttle / control-state HUD): shows the *current-tick* projected positions of left and right beacons in screen coordinates as the NN sees them, with the sentinel-CEP / off-screen state visually distinguishable (e.g., beacon dot dimmed, edge marker, or absent). This lets the operator watch what the perception pipeline is feeding the controller without having to switch to the full 1st-person camera view. **Visibility and update logic follow the same rules as the existing renderer HUD elements** (clock, attitude indicator, throttle bar, control-surface readout) — i.e., the mini-panel toggles on/off with whatever HUD-visibility control already exists, updates at the same per-tick cadence, and is positioned alongside them. New always-on UI is NOT introduced; the mini-panel slots into the existing HUD overlay system.
- **FR-012a**: The renderer MUST also support per-tick scrub controls (pause / step-forward / step-backward) per the [Renderer Playback Enhancements backlog item](../BACKLOG.md), rolled into 030. These controls work in both viewing modes and the mini-panel overlay.
- **FR-013**: System MUST handle source-run scenarios with terminal crashes — either by **skipping crashed scenarios at source-dmp load time** (filter them out of the in-memory `vector<SourceScenarioTrajectory>` before per-scenario distribution) or by truncating the playback at crash time. **v1 default: skip** (per tasks T023a) — simpler than truncate, and the scenario count is already a slice of the source dmp so dropping a few crashed entries doesn't break the per-scenario joint-PRNG sample. (Note: "library construction" framing in earlier drafts is obsolete per FR-001 revision 2026-05-04 — the source dmp itself IS the library, so filtering happens at load time.)
- **FR-014**: Per-axis aggressiveness telemetry (dCtrl, ⟨|out|⟩, per pt/rl/th) MUST work unchanged in tracker mode — the same analysis tooling applies.
- **FR-015**: Tracker-mode S3 dump format (the per-best-of-gen recording the renderer consumes) MUST extend the existing `EvalResults` schema with two classes of per-tick data, both **embedded directly in the M2 dmp** so the renderer plays back the M2 run without needing access to the original M1 source dmp:
  1. **Camera view data per tick** — the per-tick projection results (beacon positions / CEP per beacon-camera tuple) AND any state needed for camera-POV rendering (camera pose, FOV, projection geometry params at that tick). The richness exceeds bare NN sensor inputs because the renderer's camera-POV mode (FR-012) and the camera-POV mini-panel require the full projection state, not just what the NN saw.
  2. **Copied target-craft trajectory + attitude per tick** — for each tick during the M2 scenario, the M1-source-derived target craft's world position, attitude (quat), and velocity at that tick are **copied** from the source-scenario target trajectory (loaded per FR-001) into the M2 dmp. The renderer needs target pose to draw the target craft in 3rd-person view, the wingtip beacons on the target's body, and the trail-rabbit position used by fitness (FR-008). Copying — rather than storing an M1-dmp reference — makes the M2 dmp self-contained: an analyst with only the M2 dmp can fully replay the run, which is the same property pathgen-mode dmps have today (path geometry is recoverable from path-name + scenario seed; tracker-mode loses that recoverability if it leans on an external M1 file). **Storage cost**: per-tick (pos 3 + quat 4 + vel 3) ≈ 10 floats × ticks per scenario × scenarios per gen ≈ small fraction of overall dmp size; acceptable.

  v1 may serialize both classes directly as part of `EvalResults`; eventual extraction may move them to a paired sidecar. **Research-step framing** (per D14 / D15 below): the storage scheme is a deliberate design choice to favor self-contained M2 dmps over cross-file references — alternatives are open for plan-phase debate but the default is "copy what the renderer needs."
- **FR-015a**: Per [Constitution Principle V — Versioned Persistence Artifacts](../../.specify/memory/constitution.md), the dmp file MUST embed an explicit format version field. **Versioning gates are at milestone boundaries, not arbitrary commits** (clarified 2026-05-04, see [Clarifications session](#session-2026-05-04)). Concrete versions:
  - **`version 1`**: M1 pathgen-mode dmp — the frozen contract that 029 / pastonly3 / more-rnn3 stabilized. Already in place via `CEREAL_CLASS_VERSION(EvalResults, 1)` in [`include/autoc/rpc/protocol.h`](../../include/autoc/rpc/protocol.h). All historical S3 dmp artifacts carry this version stamp; no "pre-versioning" tail exists in practice.
  - **`version 2`**: M2 tracker-mode dmp — the 030-bumped schema with `cameraViewList` + `targetTrajectoryList` per FR-015. Frozen when 030 v1 ships at the M10/M11 ceiling.
  - **Intra-milestone development**: schema may churn during M5–M8 working development without sub-version increments; only the at-milestone-freeze schema is the contract version. This honors the existing greenfield-changes-don't-need-versions practice ([`feedback_no_cereal_versioning`](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md)) while applying Constitution V's loud-fail rule *between* frozen milestone versions.

  Readers MUST attempt backward-compatible loading (pathgen-mode-only readers fall back to version 1 and ignore the new tracker-mode fields) and MUST fail loudly with a clear error message when an unknown version is encountered (e.g., a future version 3 dmp fed to a v2-only reader). The 030 work introduces both the milestone-boundary versioning practice and its first load-bearing schema bump.
- **FR-016**: System MUST define an **arena** — a cylindrical region around the training-craft arming point with a configurable hard floor (minimum altitude AGL) and a configurable maximum radius — known to the fitness evaluator. The arena bounds are evaluated per tick; egress is a fitness-relevant event (penalty / scenario termination, exact treatment a plan-phase decision). v1 default arena: small radius (~50-100 m) and floor (~5-10 m AGL) suitable for the current real-flight test field; arena parameters live in `autoc-tracker.ini`. **Enforcement is committed for v1** (confirmed 2026-05-04) — the arena is not interface-only-deferred — because loss-of-signal *will* drive patrol-orbit behavior in tracker-mode training, and unbounded-volume search isn't a useful learning target. The arena serves two purposes:
  - **Bounded loitering / search behavior**: when the target is out of view, the controller should learn an in-arena search pattern rather than wandering away from the operator. This is currently emergent / unconstrained in pathgen mode; tracker mode makes it a first-class fitness signal because real-target tracking necessarily includes loss-of-signal recovery from inside a known operating volume. **This is exactly the v1 use-case** — the smoke-test 120-scenario slice (6 paths × 20 winds) will exercise loss-of-signal events naturally as the chase craft loses sight of the target craft, and the patrol-orbit emergence is one of the signals plan-phase analytics will look for.
  - **Real-flight envelope alignment**: the operator's flying field has hard physical bounds (terrain, range of pilot visibility, regulatory volume). Training inside an arena that approximates the field shape produces a controller whose behavior generalizes from sim to the same physical envelope.

  **Egress mechanic** (clarified 2026-05-04, see [Clarifications session](#session-2026-05-04)): when chase craft position violates any arena bound (radius, floor, or ceiling), the scenario **terminates immediately** at that tick — parallel structure to the FR-008b crash hull. No further fitness accrual; the scenario's score is whatever was accumulated up to the egress tick. Forfeited remaining-tick fitness IS the penalty (no separate negative magnitude). Egress telemetry (egress tick + which bound was crossed: `RADIUS` / `FLOOR` / `CEILING`) recorded in M2 dmp telemetry per FR-015 for M11c analytics.

  **Arena-awareness NN inputs** (~~clarified 2026-05-04~~ **amended 2026-05-07 per Session 2026-05-07 Q1**): the NN MUST receive a single scalar `DIST_TO_BOUNDARY_ALONG_VEL` (meters until the chase craft's current velocity vector intersects the cylinder wall, floor, or ceiling — minimum positive ray-projection distance over all three boundaries). Single source of truth: the same `arena.h::distanceToBoundary(chase_pos, vel_unit, FlightArena)` function is consumed by (a) the per-tick OOB-termination check, AND (b) this NN input — so what the controller sees and what the egress detector enforces are computed from the same code. Cylinder-shaped arena, cylinder-shaped signal: NN learns "small number = trouble approaching; let body attitude / quat / gyro figure out turn-recovery direction." Total NN input count grows 33 (pathgen) → 45 (tracker). ~~Mirrors the pathgen-mode `target_x/y/z[NOW] + dist[NOW]` pattern~~ — supersedes the body-frame-unit-vector-plus-distance shape originally specced 2026-05-04 (`HOME_X/Y/Z + HOME_DIST`, 4 inputs); see Session 2026-05-07 for the rationale (cylinder mismatch + minimalism). If smoke signal shows controllers struggle with patrol-recovery without explicit bearing input, the 2-3 input variant (`DIST_TO_BOUNDARY` + body-frame bearing components) is a 031-candidate follow-up.

  **Plan-phase research item — existing arena-like primitive**: [`src/autoc.cc:266`](../../src/autoc.cc) already references `ENTRY_SAFE_RADIUS` plus `ENTRY_SAFE_ALT_MIN/MAX` to clamp per-scenario *spawn* offsets. Those constants are **entry-time-only** (they bound where a scenario starts, not where the craft is allowed to fly mid-scenario). The plan phase MUST assess whether to (a) extend that primitive into an in-flight bound consumed by the fitness evaluator, (b) add a parallel `FLIGHT_ARENA_*` set of parameters with independent values, or (c) rename / refactor for clarity. Decide before writing the FR-016 implementation; do NOT silently overload the entry-time constants with a new fitness-time meaning.

  **Open follow-on**: if the arena training signal is weak, the next remediation is **richer "random" path coverage in M1 / pathgen training data** — the arena emergent-search behavior depends on the underlying controller having seen enough geometric variety in path geometries that it has primitives to compose a search pattern from. Track this as a contingency: weak arena-loiter performance → add path-5-style random variations to the M1 source library.
- **FR-017**: Beacon-position values flowing through the perception pipeline (from FPGA / DSP centroid extractor on real hardware; from analytic projection in sim) MUST round-trip through an **8-bit signed integer** quantization step prior to the fp32 conversion at the NN-input boundary (FR-006). v1 sim implementation: after computing each `(x, y, CEP)` triple per FR-005, quantize each coordinate to int8 (`[-128, +127]` mapped to `[-1.0, +1.0]` for x/y; CEP scaled to a comparable range with `INT8_MIN` reserved as the invisibility sentinel), then dequantize back to fp32 for the NN input. This bakes int8-tolerant training into v1 — the NN never sees the analytical-precision projection, only the quantized version. This is the architectural mechanism that makes the eventual deployed FPGA pipeline a drop-in for the sim's projection module: same quantization, same dynamic range, same sentinel.
- **FR-018**: Tracker-mode sim main loop MUST be driven by **M1-source-data timestamps**, not by an independent virtual clock. Concretely: the source-scenario target trajectory loaded per FR-001 carries a per-sample timestamp (typically 10 Hz from a pathgen-mode source, but potentially variable-rate from a future real-flight-recorded source). At each source sample `t_i`, the M2 main loop:
  1. Updates the target-craft state from source sample `t_i` (position, attitude, velocity).
  2. Computes the perception output (beacon `(x, y, CEP)` per FR-005) given the chase craft's current pose at `t_i`.
  3. Runs NN inference and sets chase-craft control outputs.
  4. Advances the chase-craft physics (crrcsim micro-steps) **until the simulated wall time reaches `t_{i+1}`**, at which point the loop iterates with the next source sample.

  **Why this matters**: drives the M2 sim cadence off the recorded source's own timeline rather than maintaining two independent clocks (sim virtual-clock vs source-data timeline). Three concrete benefits:
  - **No timer-sync drift**: M1 sample `t_i` and M2 perception sample at `t_i` are always co-located by construction; no interpolation / extrapolation of the target trajectory between source samples is required.
  - **Variable-rate sources work natively**: future real-flight-recorded targets won't have perfectly uniform 100 ms ticks (xiao+INAV telemetry has jitter; some samples drop). The M2 loop already handles "irregular sample spacing" because it just runs physics until the next stamped sample, whatever that interval is.
  - **Determinism**: same source dmp + same seed → same M2 trajectory, regardless of whether the operator's machine runs sim faster or slower than real-time. The seed-determined physics integration interpolates between source samples deterministically.

  pathgen-mode is unchanged by this; FR-018 only governs the tracker-mode main loop. (Plan-phase decision: how to structure the loop in `src/autoc.cc` so pathgen and tracker modes share what they can — the worker / population / fitness aggregation pipelines — while diverging cleanly on the per-tick stepping logic.)
- **FR-019**: System MUST provide a **pluggable NN sensor-input gathering interface** so a single `autoc` binary supports multiple training modes (pathgen / tracker / future) with different NN sensor-input layouts coexisting in tree (clarified 2026-05-04 — making explicit what FR-006 + FR-011 implied). Concrete shape:
  - **Mode-aware typed sensor enums** (FR-006 extension): each mode defines its own enum (e.g., `enum class PathgenInput` with 33 entries; `enum class TrackerInput` with **45 entries** per FR-006 + FR-016 arena-awareness — was 48 in the 2026-05-04 shape, simplified to 45 per Session 2026-05-07 Q1). The enums are independent — same name in different enums (e.g., `QUAT_W`) is allowed and disambiguated by enum scope. NN topology auto-derives input-layer width from the active enum's `COUNT` value at compile time per mode.
  - **Pluggable `gather_inputs(mode)` strategy**: each mode owns its own sensor-input-gathering function that populates the typed input array from the per-tick simulator state. The NN forward-pass code is mode-agnostic (consumes a `float[N]` of typed dimensionality `N = ModeInput::COUNT`).
  - **Mode dispatch at autoc startup** (autoc desktop, config-driven): the active mode is determined by the config file path — `-i autoc.ini` → pathgen mode → `PathgenInput` enum + `gather_pathgen_inputs()`; `-i autoc-tracker.ini` → tracker mode → `TrackerInput` enum + `gather_tracker_inputs()`. Both code paths are present in the same binary build; mode is selected at runtime per FR-011's mutual-exclusion rule.
  - **Compile-time mode selection on xiao** (embedded build): xiao firmware deploys to a single airframe + single controller mode at a time, so runtime mode-dispatch is unnecessary overhead. Use preprocessor selection (e.g., `-DAUTOC_MODE=PATHGEN` or `-DAUTOC_MODE=TRACKER`) to compile in only the active mode's enum + gather-function. Avoids both runtime branching and dead-code carry-along on the embedded target.
  - **NN topology shape per mode**: input-layer width follows the mode's enum `COUNT` (33 vs 48); other layer widths (hidden, recurrent, output) MAY be the same across modes for v1 architectural simplicity, MAY diverge in future iterations if mode-specific topology growth (R11 Path A) lands. The static_assert in `topology.h` couples the mode enum count to the topology weight count — adding/removing inputs in one mode forces a topology weight count update for that mode without affecting the other.
  - **Both modes coexist in CI / test suite**: pathgen-mode tests stay green at every milestone (Constitution II); tracker-mode tests light up as M5–M9 land. The mode-dispatch boundary is itself a contract test target (`tests/mode_dispatch_tests.cc` — added in M2 alongside the FR-006 typed scaffolding, locks in that switching configs produces the expected enum + gather-function pair).

### Key Entities

- **Source run**: a completed pathgen-mode training run (e.g., more-rnn3, pastonly3) — or, in future iterations, a bundle of real-flight-recorded trajectories — whose S3 .dmp output contains per-scenario aircraft trajectories. Identified by S3 prefix or run ID. Consumed by 030 *strictly as a trajectory input* (per FR-001 / D13); the source's own fitness, NN weights, and other metadata are irrelevant to the 030 run.
- **Source-scenario target trajectory**: one entry's worth of per-tick target-craft pose extracted from the source dmp's `aircraftStateList[scenario]` plus the joint-variation parameters (wind seed, craft variant, entry pose) of that source scenario. **No separate file format**: the source dmp IS the holding format; autoc loads it whole at startup and indexes by scenario. (Revised 2026-05-04 — this entity replaces the earlier "Library" / "Library entry" pair, which assumed a conversion-tool intermediate.)
- **Tracker-mode scenario**: one per-generation evaluation unit. Pairs one source-scenario target trajectory (loaded from the source dmp at startup) with one autoc population individual (training craft controller). Scenario count matches what the source dmp contains, possibly subset by `autoc-tracker.ini` selection per FR-011.
- **Beacon**: a point at a wingtip on the target aircraft body, in target body frame. Carries a *wavelength* (v1: distinct IR colors for L vs R) and an *emission cone* (**v1: ~270° outward-facing per beacon, optical axis pointing laterally outward from the wingtip — see FR-004 rationale**; brightness drops off outside the cone). The two outward-facing 270° cones overlap such that at least one beacon is visible whenever the target craft itself is in view, but each individually can be obscured by airframe self-shadowing or attitude. Projected per tick through each training-craft camera, and visibility is gated by both geometric FOV/behind-camera tests AND camera spectral response to the beacon wavelength.
- **IR / color filter**: the camera's optical-filter + sensor-response model. Maps beacon wavelength to camera channel signal. v1 baseline: dual-pass IR filter that admits only the two beacon wavelengths, producing binary channel separation (left beacon → channel A, right beacon → channel B). Architecture supports continuous spectral response curves for future variants.
- **Tracker-mode dump (S3 .dmp extension)**: an extended `EvalResults` schema that carries per-tick camera view data — projection results, camera pose/FOV/projection-geometry state, channel responses — in addition to the standard scenario / aircraft state. Rich enough to support 1st-person camera-POV renderer playback. Distinct from pathgen-mode dumps; tooling that reads tracker-mode dumps must be tracker-aware.
- **Camera**: a virtual optical sensor on the training craft. A *parameterized* model. Geometry: **projection model** (planar / rectilinear pinhole, spherical / fisheye, optionally cylindrical or orthographic) and **field of view** (horizontal + vertical degrees; the projection model determines what very-wide FOVs mean — a 180° pinhole is undefined, but a 180° fisheye is a hemispherical capture). Mounting (body-frame offset + orientation), aberrations (radial / tangential / chromatic / vignetting / motion blur — interface present, identity in v1), color filter response, shutter behavior (global vs rolling, scan direction), frame rate, and latency are all parameters. **v1 default**: single forward-mounted **planar pinhole at 120° FOV** with all optional effects stubbed out (per clarify Q1). The architecture must accommodate multiple cameras (e.g., the asymmetric wide-plus-narrow pair noted in Q1, or downward-tilted secondary, or fisheye-plus-narrow-rectilinear pair) — *not* matched stereo, since stereo requires baseline calibration that conflicts with the minimal-calibration design property (FR-003f).
- **Camera configuration**: a parameter set defining a single camera's behavior. Lives as compile-time constants in v1, with one configuration block per camera. Parameters are tagged by class — *compile-time fixed (target-hardware)* vs *PRNG-varied (per-flight tolerance)*; v1 treats both as compile-time constants but the architecture distinguishes them so future work can introduce per-scenario sampling for the varied class.
  - **Compile-time fixed (target-hardware spec)**:
    - Camera type / sensor model selector
    - Frame rate (Hz)
    - Latency (ms)
    - Projection geometry selector (planar / spherical / cylindrical / orthographic)
    - Color filter / spectral response
    - Shutter mode (global / rolling) + scan direction
  - **PRNG-varied per scenario (in v1: held at nominal; in future variants: sampled per scenario from a tolerance distribution)**:
    - Mounting offset (3-vector body-frame, varies within installation tolerance)
    - Orientation (quat or euler relative to body, varies within installation tolerance)
    - FOV (horizontal + vertical degrees, varies within manufacturing spec)
    - Projection-model-specific parameters (focal length for planar; sphere radius for spherical; varies within manufacturing spec)
    - Aberration parameters: radial distortion coefficients (k1, k2, k3), tangential coefficients (p1, p2), chromatic shift per channel, vignetting falloff, motion-blur kernel — all zeros / identity for v1; per-unit variation in future iteration
- **Frame buffer / history window**: the data structure that bridges camera frame rate to NN tick rate. Holds the last N camera frames per camera (where N = frame_rate / NN_tick_rate, e.g., 3-9 frames at 30-90 Hz with a 10 Hz NN tick). The contents are presented to the NN per the FR-003e strategy (latest only / stacked / derived motion / sub-tick recurrent). Sized at compile time per camera-frame-rate selection.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: An operator can point `autoc-tracker.ini` at a source-run S3 dmp key and have autoc load it at startup in under 5 minutes (modulo S3 download bandwidth) — no separate library-construction step (per FR-001 revision 2026-05-04: the source dmp IS the library).
- **SC-002**: Tracker-mode training launches successfully and runs end-to-end on the first attempt — no manual intervention required between source-dmp load and first generation evaluation.
- **SC-003 (v1 smoke test)**: A tracker-mode training run at **population 5000 × 100 gens × 120 scenarios (6 paths × 20 winds)** (per spec D13 + Session 2026-05-04 Q2) produces a fitness curve that does *something* — descends, plateaus, or fails informatively — meeting the v1 "loop closes and signals" floor.
- **SC-003a (v2+ converged target)**: A post-smoke tracker-mode training run at population 5000 × 600 gens × full-dmp scenario coverage produces a controller whose late-plateau fitness (re-evaluated on a fixed-difficulty eval) is meaningfully better than a randomly-initialized population's gen-0 fitness, and whose tracking-error trajectory descends with a shape qualitatively similar to pathgen-mode training (initial exploration phase, streak emergence, late-run refinement). Out of scope for v1.
- **SC-004**: The trained tracker-mode controller maintains visual lock on the target for at least 80% of ticks across the scenario set (i.e., for ≥ 80% of ticks, both beacons are within FOV) — meaning the controller is not chronically losing the target.
- **SC-005**: When the target briefly exits FOV (loss-of-signal events lasting ≤ 5 ticks), the elite controller re-acquires visual lock within a configurable bounded number of ticks after re-entry rather than diverging.
- **SC-006**: Per-axis aggressiveness diagnostics (dCtrl, ⟨|out|⟩) for a late-run tracker-mode controller are within the same range as pathgen-mode controllers — the smoothness/bang-bang behavior is comparable, not catastrophically worse due to the sparser input signal.
- **SC-007**: An operator can render a recorded tracker-mode scenario and visually identify per-tick lock state, beacon positions, and tracking error within 30 seconds of opening the renderer.
- **SC-008**: The full tracker-mode setup (autoc + crrcsim + renderer) builds clean from scratch on a fresh checkout — no manual steps beyond `bash scripts/rebuild.sh`. The source dmp is loaded directly at autoc startup per FR-001 — no separate library-construction tool exists (revision 2026-05-04).
- **SC-009**: An operator can change the camera configuration (mount position, FOV, projection model, camera count) by editing a single config block — at most 1-2 files — and rebuilding. No NN training-mode code edits required to switch configurations.
- **SC-010**: A camera-configuration change (e.g., FOV 60° → 120°) propagates correctly to: the projection module (per-tick beacon coordinates reflect the new FOV), the renderer (FOV cone in 3rd-person + camera-POV view both reflect new FOV), and any saved logs / S3 dumps (camera config is persisted alongside scenario data so renders remain accurate). A scenario rendered with the *wrong* config would otherwise be misleading; this requirement guarantees consistency.
- **SC-011**: All NN inputs and outputs are accessible by name through the type-safe sensor interface. An operator can list the active inputs (`BEACON_L_SCREEN_X`, `BEACON_R_OCCLUDED`, etc.) without grepping source code, and the genome ablation tool (separate backlog item) can mask inputs by name without touching 029's training code.
- **SC-012**: 1st-person camera-POV renderer mode visualizes a recorded tracker-mode scenario such that the operator can identify, within 5 seconds of opening a scenario, the moment(s) at which beacons crossed FOV boundaries or changed channel-response state. The view directly reflects what the controller saw, including channel coloring (per IR filter response) and FOV cutoff at the screen edges.

## Assumptions

- The source training run for the initial library will be a completed pathgen-mode run (more-rnn3 or successor). v1 takes the source run as fixed input; library curation / auto-bootstrapping is deferred.
- The crrcsim FDM is permissive enough to support two simultaneous aircraft instances with reasonable performance (validated by the crrcsim mod_robots architecture investigation in `~/.claude/projects/-home-gmcnutt-autoc/memory/reference_crrcsim_mod_robots.md`).
- The target aircraft's flight is *kinematically* faithful to the source run — i.e., aerodynamic plausibility is not required since the trajectory is replayed from a recording made by an actual physics-driven aircraft. A physics-driven target is a much bigger build, out of scope.
- The control NN architecture (recurrent, per 028 findings) is suitable for tracker mode with sparse visual inputs. Validated only after training; if it stalls similarly to 027's RNN runs, this is a feature-level signal that 029 needs its own clarify cycle on architecture.
- Determinism guarantees from `project_variation_design_principles` hold across the playback substrate — same library + same Seed = identical evaluation results.
- **Eventual perception pipeline targets low-compute distributed hardware** (small MCUs / SoCs with constrained DSP / no GPU-class compute). The 029 sim itself stubs the perception layer as analytic projection — this is sim-side work — but US3's camera-config experiments inform decisions about real flight hardware, and those decisions are constrained by what a cheap perception pipeline can process in real time: typically 30–60 Hz frame rate, modest FOV with simple distortion correction, single camera (or low-cost beacon-detection via thresholded brightness), and minimal aberration-correction overhead. A camera config that requires 1-TOPS-class compute to extract beacon coordinates is not a deployable result, regardless of how well it tracks in sim.

## Out of Scope

- **Real-world deployment**. Sim-only validation is the gate for 029. Flight test happens in a future feature.
- **Library curation / auto-bootstrapping**. v1 takes a single fixed source run snapshot. Tooling that extracts winners from each tracker run, prunes the library, evolves it generation-over-generation: deferred.
- **Replacing pathgen everywhere**. pathgen stays as the primary mode for regression / debug runs and for craft-variations training (025). Tracker mode is a *parallel* training mode, not a replacement.
- **Pixel-buffer image rendering / vision NN perception layer / camera pixel-to-(x, y) extraction.** The sim does *not* render or process pixel buffers; no vision NN infers target pose from images. Beacon projection is *analytic* — geometric projection of known target-pose-and-wavelength through the camera model to screen coords + CEP. The renderer's 1st-person camera-POV mode visualizes this analytically (drawing beacons as colored points on a virtual screen) — it does not run a real image-rendering pipeline. **The pipeline that bridges real camera pixels to the `(x, y, CEP)` triple — i.e., the FPGA / DSP centroid extractor + cluster-spread estimator — is a separate parallel feature** (a perception-front-end spec, expected to land alongside or in parallel with 030's controller-side work). 030's job is to validate that the *controller* works given (quantized, uncalibrated) projected beacon coordinates as input; the pixel-extraction work informs camera-spec choices in concert with 030's controller-side findings. The two features are intentionally decoupled around the `(x, y, CEP)` interface contract. (Note: optical effects like color filtering, aberrations, and rolling shutter ARE modeled — in their effect on beacon-projection results — but no image is actually rendered.)
- **Physics-driven target aircraft**. The target replays a kinematic trajectory; it does not respond to forces. Sufficient for 029 since the source recording was made by a physics-driven aircraft, so the trajectory is realistic. A physics-driven target is much bigger work and out of scope.
- **Self-occlusion by the target's own body**. Target beacons are point sources; FOV / behind-camera tests apply, but occlusion by the target's wing/fuselage is not modeled. The training craft's body also does not occlude its own camera (camera mounting assumed clear).
- **Cross-run library composition**. v1 uses one source run. Mixing library entries from multiple source runs (for diversity) is a deferred enhancement.
- **Per-scenario camera variation (manufacturing / installation tolerance)**. v1 treats the camera config as a single deterministic value per training run. PRNG-sampling mount position / orientation / FOV / aberrations per scenario — to train robustness against cheap-camera unit-to-unit variance — is a deferred enhancement. The architectural split between compile-time-fixed and PRNG-varied parameters is in place so this can be added without restructuring (analogous to how 025 craft variations adds an airframe-variation axis to the joint-PRNG sample).

## Connections to existing project memory + backlog roll-ins

**Memory references**:
- `reference_crrcsim_mod_robots.md` — the crrcsim multi-aircraft substrate this feature builds on. Identifies ~150 LOC of additions needed (RobotProgrammable subclass + Robots accessor + RobotPathProvider).
- `project_library_based_training.md` — strategic arc 029 is the first concrete instance of.
- `project_path5_random_intercept.md` — the path-5 random-intercept proof gates the leap from "track recorded sim trajectory" (029) to "track live real target" (a future feature beyond 029).
- `project_variation_design_principles.md` — the joint-PRNG sampling and determinism invariants 029 must respect.
- `project_post_028_routing.md` — sequencing context: 029 is one of the post-028 candidates alongside 025 craft variations.

**Backlog items rolled INTO 030 v1** (no longer separate features):

- **Type-Safe NN Sensor Interface** ([BACKLOG.md "[NEXT] Type-Safe NN Sensor Interface"](../BACKLOG.md)) — required by FR-006 because tracker mode introduces named sensor inputs (`BEACON_L_X`, `BEACON_L_CEP`, etc.) and the magic-number `float[]` pattern would compound the silent-corruption risk that hit prior topology changes. **The full files-to-touch list in the backlog entry (`topology.h`, `autoc.h`, `evaluator.cc`, `autoc.cc`, `aircraft_state.h`, etc.) becomes the 030 v1 implementation work.**
- **Renderer Playback Enhancements** ([BACKLOG.md "[NEXT] Renderer Playback Enhancements"](../BACKLOG.md)) — required by FR-012 / FR-012a because tracker mode adds 3rd-person + 1st-person + camera-POV mini-panel anyway; per-tick scrub lands as part of the same renderer touchpoint. The streak/multiplier overlay sub-task is also rolled in for tracker-mode debugging — knowing per-tick streak state is load-bearing when assessing whether a 030 v1 fitness signal is real or noise.
- **crrcsim mod_inputdev — link `autoc_common`** ([BACKLOG.md "[NEXT] crrcsim mod_inputdev"](../BACKLOG.md)) — **promoted to a 030 v1 prerequisite (rolled in 2026-05-04 backlog pass)**. The 028 telemetry incident exposed that any new file in `src/nn/` or `src/eval/` referenced transitively by the cherry-picked sources silently breaks the crrcsim build at link. 030 v1 *will* add new files in those directories (beacon projection module, type-safe sensor interface implementation, fitness-trail / crash-hull modules per FR-008/8a/8b). The link-against-`autoc_common` fix is essentially a precondition for 030 not exploding the crrcsim build mid-plan. Land before or alongside the first 030 file addition.
- **Per-axis / per-path analysis from S3 .dmp** ([BACKLOG.md "[NEXT] Per-axis / per-path analysis from S3 .dmp"](../BACKLOG.md)) — **rolled in 2026-05-04 backlog pass.** 030 v1's acceptance criterion (D13) is "loop runs end-to-end and the fitness curve does *something* — descends, plateaus, or fails informatively." Assessing that signal requires per-tick / per-path post-hoc analysis of the 030 dmp output stream. `data.dat` is overwritten the moment a follow-up training launches, so the .dmp-based extractor is the right tool for 030 (same problem the 029 pass already hit). Specifically: **build the `aircraft_state_extractor` per the backlog sketch as part of 030 v1 plumbing**, with column-set extended to cover the new tracker-mode fields (per-tick beacon (x, y, CEP), camera pose, target-craft pose, trail-rabbit position). The existing `plot_per_axis_time_series.py` adapts to the new column set.
- **Eval Fitness Computation Bug 2 (stale S3 fitness)** ([BACKLOG.md "[NEXT] Eval Fitness Computation — Bugs"](../BACKLOG.md)) — **rolled in 2026-05-04 backlog pass.** Bug 2 is about **the 030 run's own fitness display correctness in eval mode** — not about anything M1-source-related. When eval mode runs against a 030 best-of-gen weight set, the renderer today shows the gen-N training-time fitness baked into the NN01 format rather than the eval-mode tracker-mode fitness actually computed against the eval scenarios. v1 acceptance includes renderer inspection (US5), so this matters for 030's signal-or-not assessment when running held-out eval against 030's own outputs. (For *training-mode* per-best-of-gen dmps, the displayed fitness IS the gen's training-time tracker fitness, which is correct — Bug 2 doesn't fire there.) Source M1 dmp fitness never participates in any 030 display path; ignore it. Fix: update `genome.fitness` with the eval result before serializing to `evalResults` (or store eval fitness in a separate `evalResults` field), per the backlog entry.

**Backlog items confirmed as 030 dependencies (resolve relative ordering during plan)**:

- **GPU-Native Evaluation** ([BACKLOG.md "[NEXT] GPU-Native Evaluation"](../BACKLOG.md)) — required for the *converged* tracker-mode controller (US4 / full-dmp scenario coverage / 600+ gens / multi-frame-per-tick). Per D13's v1-as-plumbing framing, **GPU eval is NOT a v1 prerequisite**: v1 runs the 120-scenario slice (6 paths × 20 winds) at pop 5000 × 100 gens for the signal-or-not test, which is CPU-feasible in hours. v2+ unparks the GPU dependency once the loop is proven to close. Re-flag at the v1→v2 boundary; do not block v1 on GPU eval.
- **Renderer scrubbing with hidden state** ([BACKLOG.md "[DEFERRED — post-028] Renderer scrubbing with hidden state"](../BACKLOG.md)) — inherited 027 limitation. Recurrent NN's hidden state is not reconstructed when scrubbing the timeline backward in the renderer. v1 acceptable: forward-only scrub for the mini-panel is sufficient for the signal-or-not assessment. Flag as a known limitation; address only if 030 debugging turns out to need backward scrub.

**Backlog items aligned with 030 but kept separate**:

- **Genome ablation tool** ([BACKLOG.md "[NEXT — post more-rnn3] Genome ablation tool"](../BACKLOG.md)) — consumes 030's type-safe sensor interface (`--zero-input BEACON_L_*`, `--zero-input BEACON_*_CEP[*]`, etc.). v2+ research tool, not v1 plumbing. Designed in parallel; lands after.
- **Make pathgen.h Portable for Embedded** ([BACKLOG.md "[NEXT] Make pathgen.h Portable"](../BACKLOG.md)) — needed only when 030 weights ship to xiao firmware (post-v1). Defer.

**Backlog items that interact but stay deferred**:

- **Selection Strategy Alternatives (NSGA-II)** — note in plan; default to lexicase, revisit if multi-axis empirically required by tracker fitness.
- **Total Energy Management + Altitude-Aware Distance** ([BACKLOG.md "[DEFERRED] Total Energy Management"](../BACKLOG.md)) — informs 030 fitness formulation; may stay deferred but inform the choice when tuning trail distance + crash hull parameters.
- **Simulator Sampling Time Variation** ([BACKLOG.md "[DEFERRED] Simulator Sampling Time Variation"](../BACKLOG.md)) — same family as camera latency / frame-rate timing (FR-003c, FR-003d); could share infrastructure when both unpark.
- **Batch and Cache Deterministic Scenarios** ([BACKLOG.md "[NEXT] Batch and Cache"](../BACKLOG.md)) — performance optimization for full-pop runs; not v1 plumbing. Re-flag at v1→v2 alongside GPU eval.
- **Streak Threshold Ramp** ([BACKLOG.md "[DEFERRED] Streak Threshold Ramp"](../BACKLOG.md)) — already deferred; v1 doesn't need it; if 030 fitness landscape needs curriculum on streak threshold, this is the existing implementation sketch.

## Resolved clarify-pass questions

The clarifications session 2026-04-29 resolved the highest-impact architectural questions. Resolutions are recorded in detail in [§Clarifications](#clarifications). Summary:

| Q | Topic | Resolution |
|---|---|---|
| Q1 | Camera projection + FOV + count | Planar pinhole, 120° FOV, single forward-mounted. Future dual is asymmetric (wide+narrow), not stereo. |
| Q1+ | Minimal calibration | Architecture must work with minimal-to-zero per-unit calibration; per-scenario camera variation training delivers this robustness. |
| Q2 | Frame rate + history mapping | 30 Hz × explicit history at dphi-pattern offsets `[-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now]` (6 slots, past-only, 100 ms uniform grid). Preserves existing 6-slot-per-axis layout with future-sample slots redistributed to finer-grained recent-past (no lookahead for real targets). Final distribution refined in US1 narrative. |
| Q3 | NN architecture | D-simple recurrent (continue 028). NN consumes pre-extracted beacon coordinates from FPGA centroid output, NOT raw pixels. |
| Q4 | Fitness formulation | Reuse pathgen cone-surface fitness with target=playback-aircraft. Visual-lock-time as secondary telemetry only. |
| Q5 | Loss-of-signal + perception interface | (Original 2026-04-29) Per-sample `visible` flag; collapsed perception output to `(x, y, visible)` per beacon. (Revised 2026-05-04, design note D2): collapsed further to `(x, y, CEP)` per beacon — CEP carries continuous localization quality and a sentinel value subsumes the binary `visible` flag. Same dimensionality, richer semantics. Loss of signal remains routine; controllers learn orbital search behavior naturally from fitness pressure. |

## Remaining open clarify-pass questions (low-impact, can defer to plan)

The following were on the original list but didn't make the 5-question budget. None blocks plan-phase entry; flagged for resolution during plan or as low-risk defaults in tasks.

1. **Camera latency** — v1 default value (0 ms or a realistic 30-50 ms?). Range for US3 sweeps. Default: 0 ms for v1 baseline (cleanest fitness comparison vs pathgen mode), then sweep 30 / 50 ms in US3 to characterize sensitivity. Confirm in plan.
2. **Camera effects scope for v1** — confirmed load-bearing: color / IR filtering (drives `visible` flag via channel detection). Confirmed stubbed: aberrations, vignetting, motion blur, rolling shutter (interface-present, identity behavior). No clarify needed — record decision in plan.
3. **Source-scenario indexing in joint-PRNG** — each tracker scenario uses the *same* joint sample (wind seed, craft variation, entry pose) as its source scenario (deterministic reuse from the recording). The source-scenario index is what's seeded per scenario; everything else is inherited. (Replaces the earlier "library entry index" framing — the source dmp itself IS the library, per FR-001 revision 2026-05-04.) Confirm in plan.
4. **Scenario count** — superseded 2026-05-04 (Session Q2): v1 smoke uses a **120-scenario slice (6 paths × 20 winds)** of the source dmp's 245-294 scenarios. Full-dmp coverage waits for post-smoke runs. Expansion (75 craft variations × 5 paths = 375) is a future enhancement when multi-source-run libraries land.

## Open considerations surfaced during 029 / M1.3 flight tests (2026-05-03)

The 2026-05-03 flight of pastonly3 gen 391 (six paths, six natural
path-end terminations, see [`flight-results/flight-20260503/FLIGHT_REPORT.md`](../../flight-results/flight-20260503/FLIGHT_REPORT.md))
surfaced several library-substrate questions that need to be in the
030 mental model **before** we lock the v1 library schema or training
contract. Each interacts with the sim-vs-real gap that 030 + virtual-
beacon flight test (staged-path row 5) is supposed to close.

These are notes-to-think-about, not clarifications-to-resolve — they
should each spawn a clarify-pass question or a research task when 030
is unparked.

### C1. Library turn-direction symmetry — the rotational-asymmetry trap

If the 030 library is composed only of recorded flights that, in
aggregate, lean one way (e.g. predominantly CCW spirals because the
pastonly evolved strategy converged on a CCW-dominant orbit, or
because real flights happen to share a turn preference), the trained
controller will inherit that bias and **be unable to track a target
that turns the other way**. Mirror-symmetry is not a property the GA
will discover on its own — it has to be in the training distribution.

Open questions:
- Should the library composition explicitly enforce per-source-run
  mirror pairing (each recorded trajectory contributes both itself
  and its mirror — flip Y in NED + flip roll quaternion + flip
  rcData[0] / servo polarity)?
- Or should we rely on path 5 (random aerostandard) to cover the
  rotational-symmetry distribution at training time? — open question
  whether random-aerostandard's joint-PRNG actually samples both
  rotational directions uniformly. Verify before assuming.
- For real-flight library entries (post-M1.3), the flown craft will
  have its own per-flight bias; library curation needs an explicit
  axis on this.

### C2. Replay determinism — wind/entry coupling to library entries

For pathgen-mode the wind seed and entry pose are independent
joint-PRNG axes per scenario. For 030 the **library entry IS
itself a function of wind + entry + craft** at the time it was
recorded — i.e. the recorded target trajectory is conditioned on
specific environmental inputs.

Open questions for the v1 library contract:
- Does training on library entry `L_i` require replaying the *same*
  wind seed and *same* entry pose that produced the recording — i.e.
  the trainee craft sees the same environment as the target flew
  through? If yes, library entries carry their conditioning forward
  as a tuple `(trajectory, wind_seed, craft_params, entry_pose)` and
  scenario sampling uses that tuple as a unit.
- If not, what's the realism cost of decoupling? (Example: target
  recorded in calm air, trainee tracks through 5 m/s headwind →
  trainee crabs and the visual angle to target is offset; the target
  is no longer where the recording says it should be relative to the
  trainee's body frame.)
- Does each fresh tracker scenario need to re-fly the trainee through
  identical wind conditions to make the recorded target trajectory
  *kinematically reachable* — or do we accept that some library
  entries become unreachable under some wind variations (treating
  unreachability as a fitness penalty rather than a setup error)?
- If we *accept* unreachability, that's a richer fitness signal
  (controller must learn when to break off and re-acquire); if we
  *reject* it (force replay matching), determinism is tighter but
  the joint-PRNG variation budget shrinks because wind is now coupled
  to library index.

This question links directly to the existing clarify-pass question
#3 above ("Library indexing in joint-PRNG") — the deterministic-reuse
default currently captured there assumes the coupled-tuple answer;
make it explicit when 030 unparks.

### C3. Wing-tip pose data fidelity — pathgen vs real flights

Current pathgen output produces a **point-in-time rabbit position**
at each tick. It does NOT carry wing-tip attitude (i.e., the bank
angle and pitch of the *target craft* at that instant), because there
is no target craft — the rabbit is a moving point.

For 030 the target IS an aircraft, so wing-tip beacon positions
require target craft pose, not just position. The library substrate
must carry **target attitude (quaternion) per tick** alongside
position, or 030 has to synthesize attitude from position-history
(velocity-aligned + accel-derived bank) — which is fundamentally
the same problem as the pose-estimation problem the controller is
supposed to be solving. We should not feed the controller wing-tip
geometry that was reverse-engineered by the same kinematics the
controller is being trained to invert.

Open questions:
- Does the library entry schema carry full target pose (`quat` per
  tick) as a first-class field, or do we synthesize from velocity?
- For library entries derived from real flights (real INAV blackbox
  + xiao logs), the target pose IS measured (INAV quaternion stream).
  Use that directly. The asymmetry: pathgen-derived library entries
  need pose synthesis; real-flight-derived entries don't. Plan the
  schema to accommodate both shapes, with a flag or a difference of
  source kind.
- This is also the bridge to the "real flight as library source"
  strategic direction (per [`reference_crrcsim_mod_robots`](../../.claude/projects/-home-gmcnutt-autoc/memory/reference_crrcsim_mod_robots.md)
  / [`project_library_based_training`](../../.claude/projects/-home-gmcnutt-autoc/memory/project_library_based_training.md))
  — once real flights with measured pose are library entries, the
  pose-synthesis branch can be retired.

### C4. The variation budget is bigger than 030's current scope assumes

Per-scenario PRNG-varied parameters in 030 today (per
[Out of Scope](#out-of-scope) §"Per-scenario camera variation"):
deferred. But the variation budget that 030 + 025 + virtual-beacons +
real-beacons milestones jointly need is large:

| Variation axis             | Current home | 030 v1 status     | Real-flight risk if not varied |
|----------------------------|--------------|-------------------|--------------------------------|
| Wind                       | pathgen, 030 | included (49 winds in pastonly3) | low (well-covered) |
| Entry pose                 | pathgen      | currently 0.0 sigma in autoc.ini  | medium (real intercepts ARE varied) |
| Craft variations           | 025 (parked) | NOT in 030 v1     | high (real fixed-wing has unit-to-unit variance) |
| **Camera mount offset**    | 030 future   | deferred          | **high** (cheap-camera installation tolerance) |
| **Camera mount orientation** | 030 future | deferred          | **high** (per FR-003f, gates minimal-calibration) |
| **Camera FOV**             | 030 future   | deferred          | medium (lens unit-to-unit variation) |
| **Camera frame rate jitter** | 030 future | deferred          | medium (real perception pipeline timing) |
| **Camera aberrations**     | 030 future   | stubbed (interface in place) | low for v1, medium for real cameras |
| **Target wing-tip path variations** | 030 (this work) | undefined | **high** — see C3 |
| **Target turn-direction** | 030 (this work) | undefined | **high** — see C1 |
| Beacon LED wavelength tolerance | beacon-hw | deferred | medium |
| Beacon LED emission cone   | beacon-hw    | deferred          | medium |

The PRNG-budget concern: each axis added is a multiplicative factor
on the sample distribution; the joint-PRNG ramp pressures all of
them simultaneously. If 030 v1 lands without C1/C3 covered, those
become *real-flight surprises* rather than *training signals*, and
they'll trip the virtual-beacon flight test (staged-path row 5).
Pre-bake them into the spec when 030 unparks; at minimum, the
library schema should be able to carry per-entry (or per-scenario)
turn-direction-mirror flags, target-pose schema flags, and
camera-config-PRNG seed slots even if v1 leaves the sigmas at zero.

### C5. The "pathgen vs library" decision matrix (revisit on 030 unpark)

Open question that ties C1–C4 together: for 030 v1, does the library
ship with **synthesized library entries from a recorded pastonly3 /
more-rnn3 / 025 controller's pathgen-mode S3 dumps**, or with
**real-flight-recorded library entries from M1.3-class flights**?

| Source                | Wing-tip pose (C3)    | Turn-direction symmetry (C1) | Wind/entry (C2)     | Realism cost |
|-----------------------|-----------------------|------------------------------|---------------------|--------------|
| pathgen rabbit point  | synthesize from vel   | depends on path family       | trivial replay      | low fidelity |
| Sim controller S3 dump (pastonly3 dump-mode) | full quat available | inherits controller bias (e.g. CCW spiral) | trivial replay | medium fidelity |
| Real flight (xiao + INAV) | full quat measured | inherits pilot / controller bias | hard (must replay measured wind) | high fidelity but operational complexity |

The current spec assumption (Assumptions §1) is "completed pathgen-
mode run" → row 2 (sim controller S3 dump). The C1 turn-direction
trap most likely fires under that source: a pastonly-spiral
controller's S3 dump is probably CCW-dominant. **Mitigation needs
to be designed in v1**, not deferred. Mirror-pairing during library
ingest is a low-LOC fix; defer would mean it only surfaces during
M5 (virtual-beacon flight test) when a real human pilot flies a
mirror-paired intercept and the controller fails it.

## Design notes — 2026-05-04 operator pass

Companion to the C1–C5 considerations above. Where the C-series asks
*open questions*, this section records *load-bearing decisions* taken
during the 2026-05-04 design pass that are now reflected in the FRs
and in [Constitution Principle V](../../.specify/memory/constitution.md).
Cross-listed for trace-back during plan / tasks phases.

### D1. Beacon physical model — outward-facing wingtips, 270° cones

LEDs mount **outward** on each wingtip with the optical axis pointing
laterally outward; emission cone is **~270°** (vs the >180° hemisphere
of an earlier draft). Two consequences:

- **Single-beacon graceful degradation**: at any aspect angle where
  the target craft itself is in the camera's view, **at least one of
  the two beacons is in its emission cone toward the camera**. Both
  in-cone is the common case; one obscured (by the target's own
  airframe self-shadowing, or by attitude tilting one wingtip behind
  the body) is the routine fail case. Both out-of-cone is rare and
  geometrically signals "looking at the target's tail-on" — itself
  load-bearing information.
- **L/R disambiguation is intrinsic**: paired with distinct IR
  wavelengths per wingtip (FR-004), L vs R is decided at the optical
  filter — never at the controller. The controller treats `BEACON_L_*`
  and `BEACON_R_*` as ordered named inputs; the FPGA's two color
  channels supply that ordering.

Reflected in FR-004 (emission cone update) and the Beacon entity.

### D2. Perception interface — `(x, y, CEP)` per beacon, NOT `(x, y, visible)`

The earlier draft's `(x, y, visible)` 3-float per-beacon-per-slot
shape becomes **`(x, y, CEP)`** with the same dimensionality but
richer semantics. Why:

- **Visibility is not binary in practice**. A beacon at the screen
  edge with a smeared centroid (motion blur, aberration corner) is
  technically visible but its (x, y) is unreliable. The binary
  `visible` flag forced the NN to gate on a sharp boundary; the
  continuous CEP value lets it discount noisy centroids smoothly.
- **The FPGA / DSP produces CEP cheaply**. The same single-pass
  thresholded scan that yields the centroid (weighted first moment)
  also yields the cluster spread (weighted second moment). No extra
  hardware beyond what the centroid extraction already costs.
- **Sentinel-encoded invisibility subsumes the visible flag**. When
  no centroid is found (off-screen / behind / occluded / threshold
  fail), CEP is set to `INT8_MIN` (or another designated sentinel).
  The NN learns "CEP < threshold ⇒ trust (x, y); CEP = sentinel ⇒
  ignore (x, y)" without a separate boolean axis.

`x` and `y` are kept **uncalibrated screen-relative values in
~[-1, +1]** — the per-scenario PRNG-varied lens aberrations + mount
alignment + FOV are what teach the controller to be calibration-tolerant
(FR-003f). Calibration corrections at the perception output would
defeat the whole training mechanism.

Reflected in FR-005, FR-006, FR-007 and the Q5 perception-interface
refinement block.

### D3. Quantization — int8 pipeline, fp32 NN boundary

Beacon coordinates and CEP travel through the perception pipeline as
**8-bit signed integers**, dequantized to fp32 at the NN-input
boundary. This is **load-bearing for v1**, not a future hardware
optimization, because:

- **Sim and real pipelines must look identical to the controller.**
  The deployed FPGA's centroid output is int8 (or close — depends on
  resolution, but the dynamic range is comfortably sub-12-bit). If
  the sim trains the controller against fp32 analytic projection and
  the deploy hardware quantizes to int8, the controller meets a
  distribution shift at deploy time. Mirroring quantization in sim
  v1 forecloses that.
- **CEP sentinel encoding is natural in int8**. `INT8_MIN = -128` is
  trivially distinguishable from any in-range CEP value and is also
  easy to gate on in the perception extractor without conditional
  branches.
- **No NN-architecture change**. The NN itself stays fp32; the
  quantization is at the perception output, dequantized before the
  NN sees the value. So the GA-evolved policy is unchanged in shape;
  only the *training data distribution* (sim per-tick projections)
  carries the int8 noise floor.

Reflected in FR-005 (perception output) and FR-017 (the v1
implementation requirement).

### D4. Source data and mode selection — `autoc-tracker.ini`

030 ships a sibling config file alongside `autoc.ini`, named
`autoc-tracker.ini`. Selection is by file path (`-i autoc.ini` =
pathgen mode; `-i autoc-tracker.ini` = tracker mode); **no in-config
boolean flips mode**. The two configs are mutually exclusive in the
mode-determining parameters (pathgen path-family selection vs
tracker source-library reference) — fields specific to one mode
simply aren't present in the other config.

Source-library references use the same **S3-key-with-profile-prefix**
naming scheme that the xiao firmware emits in flight log entries —
the key string round-trips between xiao logs and tracker config
without rewriting. Concretely: `autoc-storage/<run-id>/gen<N>.dmp`.

For v1, **wind seeds and entry poses are assumed to reproduce
identically** when re-fed through the joint-PRNG (same scenario
index → same wind / craft / entry sample). If determinism breaks
(e.g., joint-PRNG consumer order changes between source-run training
and tracker-mode training), the v1 fallback is to **bump the dmp
schema version** (per Constitution V) and embed wind + variation
sub-seeds directly in the dmp file rather than re-deriving them from
scenario index. This is C2 above being given a concrete v1 fallback
path.

**Scenario slicing** — the ability to select a subset of source-run
scenarios by index — is architecturally in scope for v1 (so source
dumps can be crossed against future variation axes — craft, camera
config, mirror-pairing — without all-or-nothing inclusion) but the
parser implementation is plan-phase deferrable if v1's first
training run uses all scenarios.

Reflected in FR-011 (the new config file structure).

### D5. Renderer — always-on camera-POV mini-panel

Beyond the 1st-person camera-POV view (full-screen, FR-012 mode B),
the 3rd-person view picks up a **small always-on overlay panel**
near the existing throttle / control-state display showing the
current-tick projected positions of left and right beacons (and
their visibility / CEP-sentinel state). This lets the operator
*watch what the perception pipeline is feeding the controller in
real time* during 3rd-person playback without having to switch
camera modes.

The mini-panel is essentially a miniature 1st-person camera-POV
constrained to current-tick rendering — same projection state,
small footprint, compact label.

Reflected in FR-012 (third bullet) and FR-012a (scrub controls
apply to the mini-panel state too).

### D6. Arena — bounded operating volume around arming point

A new first-class concept (FR-016): a **cylindrical operating volume**
around the training-craft arming point, parameterized by **maximum
radius** and **hard floor** (minimum altitude AGL). Arena bounds are
*evaluated by the fitness module* — egress is a fitness-relevant
event (penalty, scenario termination, etc., per plan-phase decision).

Why this surfaces in 030 specifically and not earlier:

- **Real-target tracking necessarily includes loss-of-signal
  recovery**, and recovery must be bounded by the operator's flying
  field. A controller that "wanders away" when the target is out of
  view is acceptable in pathgen mode (path always exists; no LOS) and
  unacceptable for tracker mode (LOS is routine; recovery has to
  stay in the field).
- **Loitering / search is an emergent behavior the GA can find** if
  it has the geometric primitives, but it needs a fitness signal to
  shape it. The arena is that signal.
- **Real-flight envelope alignment**: arena parameters can be sized
  to the actual flying field (radius matches pilot-visibility range
  + regulatory volume; floor matches terrain). A controller trained
  inside that envelope is more directly deployable.

**Open contingency** (recorded in FR-016 itself): if the arena
fitness signal is too weak — i.e., the controller can't compose
a reasonable in-arena search pattern from its primitives — the next
remediation is **richer "random" path coverage in M1 / pathgen
training data** (path-5-style geometric variations) so the
underlying controller has the geometric vocabulary to compose
search patterns from. This couples the arena work to the M1
training-data design and is worth flagging during plan.

Reflected in FR-016.

### D7. DMP versioning and the parallel perception-front-end feature

Two cross-cutting design decisions captured in writing for the first
time on this pass:

- **DMP versioning starting now** (Constitution Principle V). All
  cereal-serialized `.dmp` files going forward carry an explicit
  format version field. Readers attempt back-compat where possible;
  fail loudly with a clear error otherwise. 030 is the first feature
  to act on this principle (FR-015a) since tracker-mode dmp files
  are a strict superset of pathgen-mode dmp files. Pre-versioned
  `.dmp` files from before this commit do not retroactively get
  versioned — they are treated as "version 0 / pre-versioning" and
  readers may either accept them (with a documented assumption of
  the legacy schema) or refuse them, at the consumer's choice.

- **Camera pixel → (x, y, CEP) extraction is a parallel feature** —
  promoted to a 031-candidate backlog entry on 2026-05-04 ([BACKLOG.md
  "[031 CANDIDATE] Parallel perception-front-end"](../BACKLOG.md)).
  030 owns the controller-side; the parallel feature owns the
  pixel-side; the `(x, y, CEP)` triple is the contract between them.
  See the BACKLOG entry for scope (centroid extraction, prop-arc
  occlusion, airframe self-occlusion mesh refinement, etc.). The
  decoupling matches the two-loop architecture
  ([memory: project_perception_control_two_loop](../../.claude/projects/-home-gmcnutt-autoc/memory/project_perception_control_two_loop.md)).

Reflected in FR-015a (versioning) and the Out-of-Scope camera-pixel
clarification.

### D8. Fitness target — trailing rabbit, not target position

The pathgen-mode fitness substitution "rabbit = path point" maps
naively to "rabbit = target position" in tracker mode. **That naive
mapping rewards collision**: a controller that flies its CG into the
target's CG maximizes cone-surface fitness. For real-target tracking
that is exactly the wrong objective.

The corrected formulation: **rabbit = `target_position − target_velocity_unit × trail_distance`**, with
`trail_distance` ~ 2-3 m configurable. The chase craft is rewarded for
sitting *behind* the target along its current heading, not on top of
it. Two consequences:

- The perception input is unchanged — the NN sees the **actual
  target's beacons**, not the rabbit. So the controller must
  *implicitly learn* that "the thing I see should be in front of me
  by ~2-3 m of velocity, not centered." This is genuine intelligence
  the GA has to evolve; the fitness gradient drives it.
- The trail offset preserves the cone-surface fitness machinery
  unchanged — just recompute the rabbit per tick from
  target pose + velocity. No new fitness shape needed.

Edge case (FR-008a): when target speed → 0, `velocity_unit` is
undefined. The v1 fallback uses target *body-frame* +x (nose
direction rotated to world by quat). Plan picks the transition
behavior between velocity-trail and nose-trail.

Reflected in FR-008 + FR-008a.

### D9. Crash-on-intercept — configurable hull, mode-dependent

The trail-offset rabbit (D8) makes the *target body* a no-go zone:
the controller is rewarded for trailing, but the target's actual
extent is closer than the rabbit. To turn that into a fitness signal,
add an explicit crash hull around the target craft.

- **Hull options**: sphere (~1 m radius v1 candidate), AABB-shaped to
  approximate hb1 dimensions, future per-airframe meshes (when 025
  craft variations land).
- **Probabilistic vs deterministic firing**: deterministic ("intercept
  always crashes") is the simple v1; probabilistic ("intercept
  crashes with `p_crash`") supports **curriculum**: anneal `p_crash`
  from low (early gens — controllers can experiment with close
  passes without instant termination, fitness landscape stays
  navigable) to high (late gens — collision becomes fatal). Plan
  picks based on whether deterministic crashes block early-gen
  exploration.
- **Mode-dependent crrcsim**: target-hull collision is tracker-mode
  only. Pathgen has no target craft to collide with; existing
  ground-collision detection stays in both modes; target-hull check
  is additive only when `Mode = Tracker`.

Reflected in FR-008b.

### D10. Camera v1 baseline — single, 30 fps, 120° FOV, top-of-wing-chord, with self-occlusion

The earlier "single forward-mounted, 120° FOV" placeholder gets
concrete v1 numbers. Single camera, **30 Hz frame rate** (3 frames
per 10 Hz NN tick), 120° FOV, mounted **on top of the wing at the
wing-chord centerline**.

Three things this mount commits to that earlier drafts left vague:

- **Position above airframe mass** — cheapest-deployable mount on
  the real fixed-wing trainer; minimizes downward-cone obstruction
  by fuselage. The wing *itself* still occludes downward FOV
  immediately below the camera.
- **Airframe self-occlusion modeled in v1** — first-order fidelity,
  coarse body proxy keyed to training-craft pose. Camera-to-beacon
  ray passing through proxy → beacon registers as occluded
  (CEP = sentinel). Operator has reference video of the actual
  airframe's apparent occlusion footprint from this mount; available
  as research input during plan phase.
- **Prop-arc occlusion deferred to a follow-on**. The real flight
  hardware will see prop-blade intermittent occlusion (potentially
  with rolling-shutter resonance banding at certain RPMs), but v1
  ships without modeling it. Marked as a known sim-to-real gap;
  prop-occlusion-modeling is a candidate for the
  perception-front-end parallel feature (D7) since blade-extraction
  belongs in the same image-domain processing module that does
  centroid extraction.

Reflected in FR-003 (v1 baseline + self-occlusion + prop deferral).

### D11. Arena — likely overlap with existing entry-time constants

[`src/autoc.cc:266`](../../src/autoc.cc) already enforces
`ENTRY_SAFE_RADIUS` and `ENTRY_SAFE_ALT_MIN/MAX` for *spawn-time*
position offsets — i.e., the per-scenario entry pose can't start
outside this cylinder. There is **no existing in-flight bound** —
once the scenario is rolling, the controller is unconstrained.

030's arena (FR-016) is fundamentally an *in-flight* bound consumed
by the fitness evaluator, not an entry-time clamp. So either:

- **Reuse and extend** the existing constants — promote them from
  entry-only to entry-and-flight, with a clear naming refactor so
  the dual purpose is obvious.
- **Add parallel constants** — keep `ENTRY_SAFE_*` for spawn,
  introduce `FLIGHT_ARENA_*` for the in-flight bound. They MAY have
  different values (entry-time tighter than flight-time, or
  vice-versa) so a separate pair gives more freedom.

Plan-phase decision; do **not** silently overload the existing
constants by adding fitness-time use without renaming. The existing
constants live in pathgen-mode code; tracker mode is the right place
to introduce the in-flight semantics and force the naming question.

Reflected in FR-016 (plan-phase research item).

### D12. Minisim — lean toward retire / stub (moved to BACKLOG)

Project-level decision; operator-confirmed 2026-05-04: lean toward
retire / stub. **Promoted out of the 030 spec** to
[BACKLOG.md "[BACKLOG] Minisim retire / stub / remove"](../BACKLOG.md)
on 2026-05-04. Decision happens at the moment a 030 schema change
first forces a touch on `tools/minisim.cc`; defaults to "ignore"
until then.

Cross-cuts the project; doesn't reflect in any 030 FR.

### D13. v1 scope strategy — smoke test, signal-or-not

030 v1 is **not** a polished feature delivery. It is a **smoke test**:
the smallest end-to-end pass that proves the architecture composes
into a runnable loop.

The smoke-test acceptance shape is four things:

1. **Run M2 mode from an M1 source dmp** — autoc-tracker.ini points
   at one M1 dmp, autoc loads it at startup, distributes
   per-scenario target trajectories to workers (FR-001 / FR-011 /
   FR-018).
2. **Test on a scenario slice** — `autoc-tracker.ini` selects
   **6 paths × 20 wind variations = 120 scenarios** from the source
   dmp (per FR-011 scenario-selection mechanism, load-bearing for
   v1 per Clarifications session 2026-05-04). The slice is small
   enough to keep run-time bounded and large enough to give the GA
   population diversity to evolve structure. Full-dmp 245-294
   scenario coverage is post-smoke.
3. **Save results** — 030's own dmp output stream + per-gen logs
   land in S3 + locally with version-bumped schema (FR-015 /
   FR-015a / Constitution V).
4. **Renderer animates the M2 result** — the FR-012 playback (with
   3rd-person view + camera-POV mini-panel) loads the M2 dmp and
   runs through the run end-to-end. This is the
   "did-the-loop-close-and-do-something-visible" verification.

Concretely the smoke test commits to defaults across all the
parameters the spec exposes:

- **Source-scenario slice** = 6 paths × 20 wind variations =
  120 scenarios per gen (clarified 2026-05-04). This IS the
  smoke-test minimum — population diversity is load-bearing and
  the variation ramp needs enough scenario breadth to do its job.
  Full-dmp 245-294 scenario coverage waits for post-smoke runs.
- **Single camera** (FR-003 v1 baseline), no multi-camera
  experiments.
- **Default trail distance, default crash hull**. No early
  parameter sweeps.
- **Default int8 quantization** (FR-005 / FR-017), no curriculum on
  noise floor yet.
- **Arena enforcement enabled** at field-realistic dimensions
  (FR-016, locked in 2026-05-04 — *not* deferred). Loss-of-signal
  drives patrol-orbit emergence; that's exactly what we want the
  smoke test to surface. No arena-shape sweeps yet — single
  field-realistic value.
- **Type-safe sensor interface at full scope** (FR-006, locked in
  2026-05-04 — *not* sim-only). Full files-to-touch from the
  backlog entry, including xiao-side and tests/scripts. Justified
  by the expectation that NN configuration will be iterated
  heavily during 030 research, and silent-corruption from magic-
  number indexing is exactly the failure mode the typed interface
  exists to prevent.
- **`autoc-tracker.ini` scenario-selection syntax** (FR-011) is
  load-bearing for v1 (per Clarifications 2026-05-04): supports
  cross-product path-list × wind-list subsetting so the smoke-test
  120-scenario slice (6 paths × 20 winds) is concisely expressed.
- **Population × generations × variation-curriculum**: 5000 × 100 ×
  10 step (clarified 2026-05-04). Population stays at converged-run
  scale (diversity is the load-bearing property); gens compress to
  100 (vs 600+ for full convergence); variation-curriculum step
  count is 10 (autoc.ini parameter name to be disambiguated in plan
  phase).

After the smoke test closes, the next loop is **post-run analytics
experimentation**: build / extend tools to look at what the M2 run
actually did (per-axis aggressiveness on tracker-mode outputs,
visual-lock-fraction time series, perception-error visualization,
crash-hull strike rate, etc.). That work refines the fundamental
tracking architecture and is the part of 030 most likely to extend
between "smoke-test green" and "030 declared done." Anything the
analytics surface that requires architectural change re-feeds the
v1 bag for another pass.

The expected v1 workflow:

```
operator launches autoc with autoc-tracker.ini
  → autoc fully loads the specified source-run dmp from S3 ONCE at
    startup (per FR-011) — extracts the per-scenario target
    trajectories from the dmp's aircraftStateList plus the joint-PRNG
    variation parameters, holds them in memory
  → autoc distributes those source-derived target trajectories to
    workers per scenario, ANALOGOUS TO HOW pathgen DISTRIBUTES PATH
    GEOMETRY TODAY — the source dmp simply replaces pathgen as the
    trajectory provider, nothing more architecturally exotic. Per-
    scenario distribution shape, worker contract, and per-tick state
    plumbing are all unchanged from the pathgen-mode pattern.
  → usual GA evolution (population × generations) on tracker-mode
    fitness — each worker simulates THIS run's chase craft against
    its assigned source-derived target trajectory and computes the
    cone-surface fitness against the trailing rabbit (FR-008 / 8a),
    with crash-hull penalty (FR-008b)
  → 030 writes ITS OWN dmp output stream per best-of-gen (separate
    S3 run-id from the source) — these are the dmp files the
    renderer and per-axis-from-dmp analytics consume during signal-
    or-not assessment
  → renderer plays back the 030 results (FR-012 — 3rd-person +
    camera-POV + mini-panel) showing THIS run's controller's tracking
    fitness, NOT the source M1 controller's fitness (which is purely
    a trajectory provenance detail and irrelevant to 030's outputs)
  → operator builds new analytics tools to assess what trained
  → repeat with whatever the assessment surfaces
```

**Architectural clarification**: the source M1 dmp is consumed
**strictly as a trajectory input**, replacing pathgen as the
provider of "where the rabbit / target is at each tick." Its
embedded fitness, its source NN weights, its per-axis aggressiveness
metrics, and so on are irrelevant to 030 training and irrelevant to
030's renderer output. The M2 (030) run has its own everything:
fitness, weights, dmp, renderer playback. *Only* the per-scenario
target-craft pose-per-tick stream from the M1 dmp crosses the
boundary into 030 — and that stream is then "the rabbit" for the
new training run. Aside: when "M1 dmp" is eventually replaced by
real-flight-recorded trajectories from xiao+INAV logs (per the
staged-path table earlier — virtual-beacon flight test and beyond),
this framing closes a quiet RL-flavored loop: the controller's own
prior real-flight outputs become the next iteration's training-
target distribution. That's downstream; v1 stays at "M1 sim dmp ⇒
M2 trajectory input."

The v1 acceptance criterion is "the loop runs end-to-end and the
fitness curve does *something* — descends, plateaus, or fails in an
informative way". Polished tracking quality, calibration tolerance,
multi-path generalization, all the variation-axes work — those are
v2+ once v1 confirms the loop closes and signals.

This framing means **the existing User Story priorities should be
re-read with a "120-scenario-slice plumbing first" filter** (per the
2026-05-04 Q2 clarification withdrawing the earlier single-scenario
deferral):

- US1 (past-only baseline experiment) — already done, in 029
- US2 (operator launches tracker training from a recorded run) — the
  v1 plumbing target
- US3 (camera-config experimentation) — **deferred from v1**
  notwithstanding the spec's earlier P1 framing; only earns its
  priority once v1 closes the loop
- US4 (controller learns via beacon-camera signal alone) — the
  signal-or-not test for v1
- US5 (renderer inspection) — needed for v1 to be debuggable
- US6 (real-target-tracking bridge readiness) — explicitly post-v1

The plan phase should re-sequence US tasks accordingly: anything
that doesn't move the needle on "loop closes + signals" is v2+.

This also recasts the C1–C5 considerations and D1–D12 design notes:
many are correct architectural commitments (record now, implement
later) rather than v1 work items. v1 implements the *interfaces*
they require (e.g., dmp versioning, type-safe sensor names, arena
parameters in ini, trail distance + hull params) but defers the
*sweep / curriculum / variation* work that would exercise those
interfaces. The interfaces have to land in v1 because retrofitting
them after the loop closes is more painful than building them in.

Reflected in: the User Story re-sequencing recommendation above, and
the spec's general bias toward "interface in v1, sigmas at zero,
sweeps in v2."

### D14. Timing model — drive the M2 loop off M1 source timestamps

The naive port of pathgen-mode's main loop runs an independent
virtual clock at the NN tick rate (10 Hz) and reads the target's
position from the source dmp by *interpolating* between recorded
samples. That works for uniform-rate sources but introduces two
problems: (a) every interpolation is a small approximation that
drifts deterministically away from the true recorded trajectory, and
(b) variable-rate sources (real-flight telemetry with jitter +
dropped samples) require interpolation logic that varies in quality
across the trajectory.

The cleaner design (FR-018): **drive the M2 main loop off the M1
source's own timestamps**. At each source sample `t_i`:

```
sample target state at t_i  →
  compute perception (beacon (x, y, CEP) from chase pose at t_i)  →
    NN inference, set chase controls  →
      run chase-craft physics until simulated time = t_{i+1}  →
        loop
```

Two clocks (sim virtual-clock vs source-data timeline) collapse to
one (the source timeline), with physics integration filling the
gaps between source samples. Determinism comes from the seed +
source dmp; sample-rate variability is handled natively because
"run until next sample" works the same regardless of whether the
next sample is 100 ms or 73 ms away.

This is a meaningful architectural change to the main loop. Plan
phase decides how to structure the source-driven loop alongside
the existing pathgen-mode loop (shared aggregation pipeline,
diverging stepping logic) without forking the worker contract.

Reflected in FR-018.

### D15. 030 research surface — what stays in v1, what moved out

Originally a sprawling "knowingly enabled by the architecture" list.
Pruned 2026-05-04 to the items that are actually load-bearing for
the smoke test, with everything else promoted to BACKLOG.md.

**Stays in 030 v1 (load-bearing for smoke test signal-or-not)**:

- **Error bars on the camera-POV display** — render CEP as a
  visible spread / ellipse around each projected beacon centroid,
  so the operator sees the perception-quality gradient
  continuously during smoke-test playback. Cheap, directly
  diagnostic. v1.
- **Source-timestamp authority vs sim-clock drift contract test**
  — confirm FR-018's determinism property end-to-end at non-real-
  time sim speeds. Cheap test, lands with the timing-model
  implementation. v1.
- **CEP semantics calibration sweep** (post-smoke-test, still v1)
  — once the loop closes, sweep CEP encoding (linear vs log-spread
  vs piecewise) to find the most-useful gradient shape. Plan-phase
  decides whether this is a v1 follow-on or v2.
- **Type-safe sensor interface scaffolding** — full scope (sim +
  xiao + tests + analysis scripts), confirmed v1 because operator
  expects to iterate the NN config heavily during 030 research.
  Optional / conditional input toggles emerge naturally from the
  scaffolding once it's in place.

**Moved to BACKLOG.md as 031 candidates** (2026-05-04):

- **Parallel perception-front-end** (camera pixels → `(x, y, CEP)`
  extraction; prop-arc occlusion; airframe self-occlusion mesh
  refinement) — see [BACKLOG.md "[031 CANDIDATE] Parallel
  perception-front-end"](../BACKLOG.md). 030 owns the controller-
  side; this owns the pixel-side.
- **Variable-rate / real-flight source robustness** — exercise
  FR-018 timing model under realistic xiao+INAV telemetry jitter
  and sample drops. See [BACKLOG.md "[031 CANDIDATE] Variable-rate
  / real-flight source robustness"](../BACKLOG.md).
- **Library curation + turn-direction mirror-pairing** — see
  [BACKLOG.md "[031 CANDIDATE] Library curation + turn-direction
  mirror-pairing"](../BACKLOG.md). Covers C1 (mirror-pairing) and
  C5 (mixed-source libraries) from above.
- **Renderer reverse-projection / dphi-overlay / crash-strike viz
  exotic goodies** — see [BACKLOG.md "[031 CANDIDATE] Renderer
  exotic goodies"](../BACKLOG.md). Research-grade analytics, not
  smoke-test-required.

**Genome ablation tool** ([BACKLOG.md "[NEXT — post more-rnn3]
Genome ablation tool"](../BACKLOG.md)) — already a separate backlog
entry; consumes the 030 v1 typed-sensor names. Lands after.

### D16. Plan-research owns checkpoint ordering — pre-plan housekeeping done 2026-05-04

The spec is intentionally large: architectural decisions
(perception-interface shape, timing model, fitness-target offset,
crash-hull semantics, dmp versioning, arena, ...) are coupled
enough that getting each of them on paper *now* was cheaper than
discovering coupling at implementation time.

**Pre-plan-research housekeeping (executed 2026-05-04 before plan
phase opens)**:

- **Smoke-test framing locked in (D13)**: 030 v1 = the four
  smoke-test deliverables, nothing more. Defaults committed across
  all exposed parameters.
- **Arena enforcement locked in for v1** (FR-016) — not deferred.
  Justified by loss-of-signal-drives-patrol expectation. Plan-phase
  research item on the underlying primitive (existing
  `ENTRY_SAFE_*` vs new `FLIGHT_ARENA_*`) remains; the *commitment
  to enforce* does not.
- **Type-safe sensor interface locked in at full scope** (FR-006)
  — sim + xiao + tests + scripts. Justified by NN-iteration
  intensity expected during 030 research.
- **025 reorders to post-030 research** (operator-confirmed
  2026-05-04): sim-to-real has been proven robust across multiple
  flights, so 025 craft variations are no longer the
  before-030-blocker the staged-path memory originally said. 025
  earns its priority when **camera variations need modeling** —
  which is tied to the 031 perception-front-end work, not to 030
  smoke-test. See updated [memory:
  project_perception_control_two_loop](../../.claude/projects/-home-gmcnutt-autoc/memory/project_perception_control_two_loop.md).
- **Items moved out of 030 spec to BACKLOG.md** (031 candidates +
  pure backlog): parallel perception-front-end (camera pixels →
  `(x, y, CEP)`); variable-rate / real-flight source robustness;
  library curation + turn-direction mirror-pairing (C1 / C5
  resolution); renderer reverse-projection / dphi-overlay /
  crash-strike-viz exotic goodies; multi-camera variant
  experiments; minisim retire/stub/remove; airframe self-occlusion
  mesh refinement; prop-arc occlusion modeling. All have BACKLOG.md
  entries pointing back here.

What plan-research must still produce before 030 implementation
work begins:

- **An ordered checkpoint sequence** delivering the smoke test
  (D13) end-to-end as the *first* visible green light. Plan-phase
  walks the FR list, the rolled-in backlog items
  (mod_inputdev linkage fix, per-tick dmp extractor, eval fitness
  Bug 2, type-safe sensor interface, renderer playback
  enhancements), and produces a build order with cheap-and-load-
  bearing checkpoints visible along the way.
- **A clear definition of "030 done"** — the smoke-test green is
  the *floor*; "030 done" is some checkpoint in the post-smoke-test
  analytics-experimentation ramp, chosen at plan time. Beyond that
  line, work moves to 031 (using the BACKLOG.md 031-CANDIDATE
  entries as the seed).
- **Resolution of the few remaining open questions**: `p_crash`
  v1 default (deterministic / zero / curriculum); arena-primitive
  reuse vs new constants; M1.3 re-fly decision (parallel-track,
  doesn't gate 030 plan).

So this spec is the *menu*; plan-research is where the meal gets
ordered. Implementation work flows from plan-research's checkpoint
sequence, not the spec's FR-numbering order.

Cross-cuts every section; doesn't reflect in any FR by itself.
