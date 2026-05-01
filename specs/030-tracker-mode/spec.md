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

## Overview

Today, autoc trains a controller to track a synthetic *rabbit point* moving along a `pathgen`-generated geometric path. 029 introduces a **distinct training mode** in which the target is **another aircraft** flying a *recorded* trajectory (from a prior pathgen-mode training run), and the controller's only signal about the target is the **2D screen position of two wingtip beacons** projected through a simulated forward-facing camera on the training craft.

The control NN's output remains pitch / roll / throttle. Its inputs change fundamentally: instead of a 3D target position handed to it directly, it receives sparse 2D visual signals subject to occlusion, FOV cutoff, and behind-the-craft loss. The fitness criterion is tracking quality against the recorded target trajectory.

This is the first concrete instance of the **library-based training** strategic direction (operator framing 2026-04-29) and the foundation for the eventual real-target-tracking milestone — the same NN architecture later replaces sim beacons with real-world target-pose inference, with no change to the controller's training framework.

## Clarifications

### Session 2026-04-29

- Q: Camera projection geometry + FOV + count for v1 baseline? → A: **Planar pinhole, 120° FOV, single forward-mounted.** Note also: the most likely future variant is *not* stereo (matched cameras for triangulation) but an **asymmetric dual-camera pair** — one wide-FOV (~120°+) for situational awareness + one narrower-FOV for precision tracking. The architecture must support this asymmetric pair without restructuring (FR-003b's "multiple cameras" requirement applies).

- Q: **Minimal-calibration design property** (raised alongside Q1) → A: Across all camera configurations and per-scenario variations, the controller architecture must work with **minimal-to-zero per-unit calibration**. Cheap flight-hardware cameras vary unit-to-unit; any approach that requires precise per-camera calibration (e.g., per-unit intrinsic-matrix tuning, per-unit baseline measurement for stereo) is a non-starter. This reinforces the choice to skip stereo (requires baseline calibration), and means the eventual per-scenario camera variation work ([§Compile-time-fixed vs PRNG-varied parameters](#)) is itself the *robustness training* that delivers minimal-calibration tolerance — variation in mount position / orientation / FOV during training teaches the controller to generalize across uncalibrated units.

- Q: Camera frame rate + how multi-frame-per-tick feeds the NN? → A: **30 Hz camera × explicit history at dphi-pattern time offsets**. Concretely: each NN tick (10 Hz), the NN sees **6 beacon-screen-position samples per beacon-axis at offsets `[-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now]`** (5 past + now, 100 ms uniform grid). This adapts the existing path-tracking input pattern in `include/autoc/nn/nn_inputs.h` (which today uses `[-0.9, -0.3, -0.1, now, +0.1, +0.5]` for the rabbit direction) but is **past+current only** because there is no lookahead for a target craft (it's a real-moving aircraft, not a known parametric path). See [US1 narrative below](#user-story-1--past-only-input-baseline-experiment-priority-p1--first-experiment-gates-the-rest) for the rationale behind the chosen offset distribution.
  - **Preserves the existing 6-slot-per-axis layout**: today's `NNInputs::target_x[6]` / `target_y[6]` / `target_z[6]` / `dist[6]` use 3 past + current + 2 future samples per axis. Tracker mode keeps **6 slots per axis** but redistributes them past-only — the future-sample slots are dropped and replaced with finer-grained recent-past slots.
  - **Per-beacon input shape** (refined by Q5 below to collapse to `(x, y, visible)` per slot): see Q5 for the final 36 + 8 = 44 input layout.
  - **Why 30 Hz over 60 Hz**: 30 Hz × 10 Hz NN tick gives 3 camera frames per tick + a 30-frame rolling buffer for the dphi-pattern selectivity. Fine-grained recent samples (−33 ms, −66 ms) are accessible — *strictly richer* than today's 10 Hz-tick-limited history. 60 Hz would give 60-frame buffers + −16 ms / −33 ms / −50 ms granularity, but at higher continuous compute load on the eventual flight hardware. 30 Hz is the cheaper deployable target; 60 Hz is the US3 sweep variant.
  - **Strategy mapping** (per FR-003e options): this is closest to (c) "latest + derived motion" but using *raw past samples at fixed offsets* rather than computed velocity / accel — same information density, more direct, no derivative computation in the NN's input layer.
  - **NN architecture interaction**: this works well with both recurrent and feedforward NN; the explicit history reduces the temporal-memory burden on a recurrent block (it has the past samples directly) and gives a feedforward NN something concrete to learn from. Couples to Q3 (NN architecture choice).
  - **Future-sample slot for the existing pattern**: deliberately dropped. v1 does NOT extrapolate to fill `+0.1s` / `+0.5s` slots. If experimentation shows the NN benefits from explicit forward-prediction, future iteration can add linearly-extrapolated future samples — but the recurrent NN should learn this implicitly anyway.

- Q: NN architecture for tracker mode? → A: **D-simple recurrent** (16-wide layer 2, ~1923 weights) — continue 028's architecture unchanged. Three reasons: (1) more-rnn3 evidence at gen 700+ demonstrates the architecture works well at this scale; (2) recurrent memory handles loss-of-signal recovery elegantly — the hidden state retains target-trajectory context across signal dropouts where Q2's explicit-history slots become no-signal sentinels; (3) keeping the architecture stable across pathgen-mode and tracker-mode lets a future feature train both modes against the *same* network shape, or transfer-learn between them.

  **Critical perception-boundary clarification**: the NN does **not** consume raw camera pixels. The deployed perception pipeline is **camera → beacon-detection on small FPGA / DSP → coordinate output → NN**. Specifically:
  - The flight-hardware camera produces a raw 2-channel sparse image (per the IR-color baseline architecture — dual-pass IR filter + Bayer sensor, where each beacon's wavelength registers in one color channel).
  - A small FPGA (or dedicated DSP block on the perception MCU) performs **thresholded centroid extraction per channel** — a near-trivial operation: scan rows, find above-threshold pixels per channel, compute weighted centroid. This is the *only* image-domain processing on the deployed hardware. Output: a few numbers per camera frame — `(beacon_L_x, beacon_L_y, beacon_L_intensity)` and `(beacon_R_x, beacon_R_y, beacon_R_intensity)`. Tens of bytes per frame, not megabytes.
  - **The NN input is the centroid output** — beacon screen coordinates + per-beacon channel intensity + `in_fov` / occlusion flags, fed through the type-safe sensor interface (FR-006) using names like `BEACON_L_SCREEN_X`. v1 input dimension stays in the ~20-30 float range (Q2's spec), comparable to today's path-tracking input.
  - **The sim mirrors this boundary exactly**: the analytic beacon-projection module in 029 produces the *same* coordinate-domain output the FPGA would produce on real hardware. There is no pixel buffer in the sim's perception path either; the renderer's 1st-person camera-POV mode (FR-012) renders for human inspection only, never consumed by the NN.

  This is what makes the 1923-weight recurrent NN viable for visual tracking: the heavy lifting (image → coordinate) is offloaded to a deterministic, calibration-light, FPGA-cheap front-end, and the NN solves the much smaller *control* problem of "given these beacon coordinates over time, command pitch/roll/throttle." Raw-pixel-to-control end-to-end is a different feature class entirely (017 visual target tracking territory) and is out of scope for 029.

- Q: Fitness formulation? → A: **Reuse pathgen cone-surface fitness** ([`include/autoc/eval/fitness_computer.h`](../../include/autoc/eval/fitness_computer.h)) — per-tick conical scoring against the playback target's position, with the target's position substituted for the rabbit position. Streak / multiplier infrastructure carries forward unchanged. This makes per-axis aggressiveness diagnostics (SC-006) directly comparable to pathgen mode and makes cross-run comparisons (pathgen vs tracker) apples-to-apples on the same fitness scale. Visual-lock-time enters as a *secondary* telemetry signal (not in fitness) — added to the per-gen log alongside fitness, available for analysis but not driving selection until empirical evidence shows it's needed.

- Q: Loss-of-signal handling for explicit history slots? → A: **Per-sample `visible` flags for all history slots, plus dramatic simplification of the per-beacon input** — see "perception-interface refinement" below. Loss-of-signal is **routine, not exceptional** — beacons are visible or not as a normal operating condition. The fitness pressure naturally evolves *orbital search behavior* in winning controllers when target is lost, since cone-surface fitness rewards re-acquiring quickly. The NN does not need a rich "why isn't this beacon visible" signal; it just needs to know whether each beacon is visible *now and at each historical time slot*.

  **Perception-interface refinement** (raised by operator alongside Q5): the per-beacon, per-frame target input collapses to just `(screen_x, screen_y, visible_flag)` — three floats. No channel response, no separate FOV / behind-camera / occlusion flags — these all collapse into the single `visible` indicator because the deployed FPGA's centroid extractor produces *exactly this* output (it either finds a centroid in a channel or it doesn't; the FPGA can't distinguish "out of FOV" from "behind craft" from "occluded by self" — all yield "no centroid"). Updated input shape:
  - Per beacon per camera per time-sample (history slot): 3 floats `(x, y, visible)`
  - 6 history slots × 2 beacons × 1 camera × 3 floats = **36 floats** for beacon-related inputs
  - Plus aircraft state (quat 4 + airspeed 1 + gyros 3 = 8)
  - Total NN input = **44 floats** (modest growth vs the 33-float pathgen layout — the per-sample visibility flags add 12 floats and the redistributed 6-slot history grid adds the rest)
  - This *replaces* earlier discussion of separate `channel_response`, `in_fov_flag`, `behind_camera_flag` per beacon — those are perception-interface noise the NN doesn't benefit from seeing.

  This simplification flows downstream into FR-005 (projection output simplifies to `(x, y, visible)` per beacon-camera tuple), FR-006 (the type-safe sensor interface uses names like `BEACON_L_X[t]`, `BEACON_L_VISIBLE[t]` for t in {-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now} — no `_CHANNEL` or `_INTENSITY` names), and FR-007 (no-signal handling reduces to "visible=0" rather than separate occluded/behind/FOV cases).

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

The operator picks a source pathgen-mode training run (e.g., a specific gen of a prior more-rnn3-class run), converts its 245 per-scenario recorded trajectories into a playback library, and launches a tracker-mode autoc training run that uses those recordings as the targets. Wind and craft variations from the source run carry over per scenario, so each tracker scenario flies in the same wind/airframe conditions as its source flight.

**Why this priority**: This is the gateway capability. Without a way to construct the target library and launch tracker mode, nothing else matters. The whole feature exists to unlock this workflow.

**Independent Test**: Given a recorded pathgen run's S3 .dmp output, the operator runs the conversion tool and obtains 245 playback files. They then launch tracker mode against that library and observe a multi-aircraft sim with the training craft pursuing the playback target, with NN training progressing per generation.

**Acceptance Scenarios**:

1. **Given** an existing pathgen-mode training run's S3 directory, **When** the operator runs the dmp-to-playback conversion tool with `--source-run <id> --source-gen <N>`, **Then** 245 playback files are produced — one per (path × wind) scenario — capturing aircraft pose per tick and the joint variation parameters (wind seed, craft variation) from the source.
2. **Given** a populated tracker-mode library, **When** the operator launches autoc with the tracker-mode config, **Then** each generation evaluates the population against 245 scenarios where the target craft replays its assigned trajectory and the joint variation matches the source.
3. **Given** an in-progress tracker-mode run, **When** the operator inspects the per-generation log, **Then** they see fitness statistics analogous to pathgen mode (best/avg/worst, streak quality, per-axis aggressiveness) plus tracker-specific signals (visual-lock fraction, occlusion-rate, beacon-projection statistics).

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
- **Source-run scenario crashed**: A .dmp scenario where the source run's aircraft crashed mid-scenario produces a partial-trajectory playback. Handle by either truncating the tracker scenario at the crash point or rejecting that .dmp during library construction.
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

- **FR-001**: System MUST provide a tool to convert a recorded pathgen-mode training run (S3 .dmp output for a specific gen) into a tracker-mode target library, where each library entry is a playback file capturing aircraft pose per tick plus the joint-PRNG variation parameters (wind seed, craft variation, entry pose) from the source scenario.
- **FR-002**: System MUST extend the simulator to host two aircraft simultaneously per scenario — the training craft (autoc-controlled) and the target craft (playback-driven from a library entry).
- **FR-003**: System MUST attach one or more virtual cameras to the training craft's body frame. Each camera's mounting position (offset from CG), orientation, **projection geometry** (planar / rectilinear pinhole, spherical / fisheye, optionally cylindrical or orthographic), **field of view** (degrees, horizontal + vertical, must accommodate wide-FOV cases up to and including hemispherical), and other projection-model parameters (e.g., focal length, principal point) MUST be configurable parameters (compile-time constants acceptable for v1) — *not* hardcoded values. **v1 default**: planar pinhole, 120° FOV, single forward-mounted.
- **FR-003a**: Camera model MUST be **structurally extensible** to support optional effects without changing the training mode's architecture. Specifically:
  - **Optical aberrations**: radial distortion (barrel and pincushion), tangential distortion, chromatic aberration (per-channel focal-point shift, relevant when color filtering is in use), vignetting (intensity falloff toward image edge), defocus / motion blur (when the target moves faster than the camera can resolve sharply).
  - **Color filtering / spectral response**: per-channel sensitivity that affects beacon visibility (e.g., a Bayer filter or narrow-band filter that admits beacon LED wavelengths preferentially).
  - **Shutter behavior**: global vs rolling; rolling shutter direction — horizontal or vertical scan.

  v1 may stub all of these as identity / no-op; the *interface* must be in place so a future change to enable any of them does not require touching the NN training pipeline.
- **FR-003b**: System MUST support multiple-camera configurations as a future-friendly property — i.e., the architecture must NOT assume exactly one camera. v1 may implement single-camera only, but the data flow from "camera N produces beacon projection" to NN inputs must accommodate camera count > 1 without restructuring. **The most likely future dual-camera variant is asymmetric** (wide-FOV + narrow-FOV pair, complementary coverage) rather than stereo (matched cameras for triangulation). The architecture should not assume cameras have matching configs.
- **FR-003f**: The control architecture MUST accommodate **minimal per-unit camera calibration**. The controller's behavior must not depend on precise knowledge of a specific camera unit's intrinsic parameters (focal length to sub-pixel, lens distortion coefficients to high precision, exact mount alignment, etc.). Real flight-hardware cameras vary unit-to-unit; the deployable result must work without per-unit calibration steps. The architectural mechanism that delivers this is the [PRNG-varied per-scenario camera-variation work](#) (deferred but architecturally enabled in v1) — training across uncalibrated-camera variations is what produces calibration-tolerant controllers.
- **FR-003c**: Camera **frame rate** MUST be configurable independently of the NN evaluation tick rate. Sim NN tick is 10 Hz (100 ms / tick); flight-hardware candidate cameras run at 30 / 60 / 90 Hz — 3 / 6 / 9 camera frames per NN tick. The architecture must accommodate frame-rate values in this range without restructuring. (v1 default frame rate: see open clarify questions.)
- **FR-003d**: System MUST model camera **latency** — the delay between physical world state and the camera frame the NN sees — as a configurable parameter. Real cameras have non-zero exposure-plus-readout-plus-processing delay; the sim must reflect this so the controller learns to compensate. v1 may default to 0 ms but the interface must accept and apply non-zero latency values. (v1 default latency: see open clarify questions.)
- **FR-003e**: System MUST define how multiple camera frames per NN tick feed into NN inputs. Frame rate > NN tick rate means N=3-9 frames are produced between consecutive NN evaluations; these are *history* available to the NN. Options include: (a) NN sees only the latest frame, (b) NN sees a stack of the last N frames as parallel input columns, (c) NN sees latest frame + derived velocity / acceleration from prior frames, (d) NN runs at camera frame rate (sub-tick updates) with recurrent state carried across frames. The chosen approach must be parameterized so US3's camera-config experimentation can compare them. (v1 strategy: see open clarify questions.)
- **FR-004**: System MUST place two wingtip beacons on the target aircraft at well-defined body-frame positions — one at each wingtip — each with a configurable **emission wavelength** (v1 baseline: distinct IR colors A and B for L vs R) and a configurable **emission cone** (v1 baseline: >180° hemisphere, approximating omnidirectional in-the-relevant-direction). The two-beacons-with-distinct-wavelengths design is what makes L/R disambiguation intrinsic to the optical stack.
- **FR-005**: System MUST project, each tick, the two beacons through each training craft's camera into 2D screen coordinates, producing for each (camera, beacon) tuple **a minimal 3-float output: `(screen_x, screen_y, visible_flag)`**. The `visible_flag` collapses three underlying conditions (out of FOV, behind camera, beacon-channel response below detection threshold) into a single binary indicator — this *exactly mirrors* the deployed FPGA centroid extractor's output, which cannot distinguish those conditions either (it either finds a centroid in the appropriate channel or it doesn't). Internally the projection module computes the underlying conditions for diagnostics / renderer use (FR-012, FR-015), but the NN-facing interface is the simplified 3-float tuple.
- **FR-006**: System MUST present beacon projection results to the control NN through a **type-safe sensor interface** (rolled in from the [Type-Safe NN Sensor Interface backlog item](../BACKLOG.md)) — *not* as opaque float[] indexed by magic numbers. Each NN input is named with type / units / range metadata. Concrete naming for v1 (per Q2 history pattern + Q5 simplification): `BEACON_L_X[-0.5s]`, `BEACON_L_X[-0.4s]`, `BEACON_L_X[-0.3s]`, `BEACON_L_X[-0.2s]`, `BEACON_L_X[-0.1s]`, `BEACON_L_X[now]`, `BEACON_L_Y[...]`, `BEACON_L_VISIBLE[...]` for each of the 6 time offsets, mirrored for `BEACON_R_*`. Total per-beacon: 6 × 3 = 18 named inputs; per camera × 2 beacons = 36; total target-related input = 36 named inputs (plus 8 aircraft state inputs = 44 total tracker-mode NN inputs). This both (a) prevents the silent-corruption class of bugs that hit prior topology changes and (b) is required by the [Genome ablation tool backlog item](../BACKLOG.md) which masks inputs by name (e.g., `--zero-input BEACON_L_*` to ablate the left-beacon entirely; `--zero-input BEACON_*_VISIBLE[*]` to ablate the visibility flags and force the NN to infer-from-coords-only). 029 *implements* the type-safe interface; the ablation tool *consumes* it.
- **FR-007**: When a beacon is not visible (out of FOV, behind the camera, or not detected), the system MUST emit `(screen_x=0, screen_y=0, visible_flag=0)` for that beacon's coordinate slot. The `visible_flag` is the load-bearing signal — `screen_x` / `screen_y` values are arbitrary when `visible_flag=0` (zero is convention; could be any value, since the NN learns to gate on the flag).
- **FR-008**: System MUST evaluate fitness based on tracking quality against the recorded target trajectory using the **existing pathgen cone-surface fitness** ([`include/autoc/eval/fitness_computer.h`](../../include/autoc/eval/fitness_computer.h)) with the playback target's position substituted for the rabbit position. Streak / multiplier infrastructure carries forward unchanged. Visual-lock-time fraction is added to per-gen telemetry as a *secondary* signal but does NOT participate in fitness selection. (See clarify Q4 for rationale.)
- **FR-009**: System MUST preserve absolute determinism: same Seed and same source library produce identical sequences of evaluations and identical fitness numbers.
- **FR-010**: System MUST preserve the joint-PRNG variation model — each scenario remains one PRNG-seeded joint sample. Library-entry-index becomes one of the joint axes alongside wind/entry/craft (which are inherited from the source recording).
- **FR-011**: System MUST support a new operating mode (selected by a separate config such as `autoc-tracker.ini`) without breaking pathgen-mode training. Both modes MUST coexist in tree.
- **FR-012**: Renderer MUST support **two viewing modes** for tracker-mode playback:
  - **3rd-person view**: both aircraft (training craft + playback target) rendered in the scene, beacons visible on the target wingtips, camera FOV cone drawn from the training craft, per-tick error metrics overlaid. (US5 primary use case.)
  - **1st-person camera-POV view**: render *from the training craft's camera* — i.e., what the controller "sees." Beacons appear as colored points (per their channel response) at their projected screen positions; FOV bounds are the screen edges; aberrations / rolling-shutter effects (when enabled) are visible directly. This is the most direct way to debug perception-pipeline questions and is uniquely valuable for camera-config experimentation (US3).
- **FR-012a**: The renderer MUST also support per-tick scrub controls (pause / step-forward / step-backward) per the [Renderer Playback Enhancements backlog item](../BACKLOG.md), rolled into 029. These controls work in both viewing modes.
- **FR-013**: System MUST handle source-run scenarios with terminal crashes — either by skipping crashed scenarios during library construction or by truncating the playback at crash time. Decision recorded in implementation.
- **FR-014**: Per-axis aggressiveness telemetry (dCtrl, ⟨|out|⟩, per pt/rl/th) MUST work unchanged in tracker mode — the same analysis tooling applies.
- **FR-015**: Tracker-mode S3 dump format (the per-best-of-gen recording the renderer consumes) MUST extend the existing `EvalResults` schema with **camera view data per tick**: the per-tick projection results (beacon positions / occlusion flags / channel responses for each camera) AND any state needed for camera-POV rendering (camera pose, FOV, projection geometry params at that tick). v1 may serialize this directly as part of `EvalResults`; eventual extraction may move it to a paired sidecar. The richness exceeds bare NN sensor inputs because the renderer's camera-POV mode requires the full projection state, not just what the NN saw. (Note: per [no cereal versioning policy](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md), this is a clean schema bump — old `.dmp` files become unloadable by tracker-mode-aware tools, but pathgen-mode tooling continues unchanged.)

### Key Entities

- **Source run**: a completed pathgen-mode training run (e.g., more-rnn3) whose S3 .dmp output contains 245 per-scenario aircraft trajectories. Identified by S3 prefix or run ID.
- **Library**: a directory of playback files derived from a single source run + chosen gen. 245 entries (one per source scenario). Each entry carries: pose-per-tick + variation parameters + provenance metadata.
- **Library entry**: one playback file. Contains aircraft pose per tick (position, attitude, optional velocity), the joint-variation parameters of the source scenario (wind seed, craft variant, entry pose), and metadata (source run, source gen, source scenario index).
- **Tracker-mode scenario**: one per-generation evaluation unit. Pairs one library entry (target trajectory) with one autoc population individual (training craft controller). 245 scenarios per gen.
- **Beacon**: a point at a wingtip on the target aircraft body, in target body frame. Carries a *wavelength* (v1: distinct IR colors for L vs R) and an *emission cone* (v1: >180° hemisphere; brightness drops off outside the cone). Projected per tick through each training-craft camera, and visibility is gated by both geometric FOV/behind-camera tests AND camera spectral response to the beacon wavelength.
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

- **SC-001**: An operator can convert a complete pathgen-run S3 directory into a 245-entry tracker library in under 5 minutes (modulo S3 download bandwidth).
- **SC-002**: Tracker-mode training launches successfully and runs end-to-end on the first attempt — no manual intervention required between library construction and first generation evaluation.
- **SC-003**: A tracker-mode training run at population 5000 × 600 generations produces a controller whose late-plateau fitness (re-evaluated on a fixed-difficulty eval) is meaningfully better than a randomly-initialized population's gen-0 fitness, and whose tracking-error trajectory descends with a shape qualitatively similar to pathgen-mode training (initial exploration phase, streak emergence, late-run refinement).
- **SC-004**: The trained tracker-mode controller maintains visual lock on the target for at least 80% of ticks across the scenario set (i.e., for ≥ 80% of ticks, both beacons are within FOV) — meaning the controller is not chronically losing the target.
- **SC-005**: When the target briefly exits FOV (loss-of-signal events lasting ≤ 5 ticks), the elite controller re-acquires visual lock within a configurable bounded number of ticks after re-entry rather than diverging.
- **SC-006**: Per-axis aggressiveness diagnostics (dCtrl, ⟨|out|⟩) for a late-run tracker-mode controller are within the same range as pathgen-mode controllers — the smoothness/bang-bang behavior is comparable, not catastrophically worse due to the sparser input signal.
- **SC-007**: An operator can render a recorded tracker-mode scenario and visually identify per-tick lock state, beacon positions, and tracking error within 30 seconds of opening the renderer.
- **SC-008**: The full tracker-mode setup (autoc + crrcsim + renderer + library tooling) builds clean from scratch on a fresh checkout — no manual steps beyond `bash scripts/rebuild.sh` and the library-construction tool.
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
- **Pixel-buffer image rendering / vision NN perception layer**. The sim does *not* render or process pixel buffers; no vision NN infers target pose from images. Beacon projection is *analytic* — geometric projection of known target-pose-and-wavelength through the camera model to screen coords + channel response. The renderer's 1st-person camera-POV mode visualizes this analytically (drawing beacons as colored points on a virtual screen) — it does not run a real image-rendering pipeline. The perception layer that bridges to real-world image input (camera frames → beacon-position extraction) is a separate future feature; 029's job is to validate that the controller works given clean projected beacon coordinates as input. (Note: optical effects like color filtering, aberrations, and rolling shutter ARE modeled — in their effect on beacon-projection results — but no image is actually rendered.)
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

**Backlog items rolled INTO 029** (no longer separate features):
- **Type-Safe NN Sensor Interface** — required by FR-006 because tracker mode introduces named sensor inputs (`BEACON_L_SCREEN_X`, etc.) and the magic-number `float[]` pattern would compound the silent-corruption risk that hit prior topology changes.
- **Renderer Playback Enhancements** (per-tick scrub + streak/multiplier overlay) — required by FR-012 / FR-012a because tracker mode adds 3rd-person + 1st-person modes anyway; per-tick scrub lands as part of the same renderer touchpoint.

**Backlog items confirmed as 029 dependencies (resolve relative ordering during plan)**:
- **GPU-Native Evaluation** — required for US4 long-run training at full population × 600+ generations × multi-frame-per-tick. Either land it ahead of 029, or scope 029 v1 to operate at smaller scale until GPU eval is available. Plan must commit on this.

**Backlog items aligned with 029 but kept separate**:
- **Genome ablation tool (post more-rnn3)** — consumes 029's type-safe sensor interface. Designed in parallel; lands after.
- **Eval Fitness Computation Bugs** (Bug 2 stale S3 fitness, Bug 3 eval-mode rabbit speed) — likely fix during 029 plan since 029 introduces a new fitness formulation; Bug 2 prevents a confusing artifact in tracker-mode S3 outputs.

**Backlog items that interact but stay deferred**:
- Selection Strategy Alternatives (NSGA-II) — note in plan; default to lexicase, revisit if multi-axis empirically required.
- Total Energy Management + Altitude-Aware Distance — informs 029 fitness formulation (Q7); may stay deferred but inform the choice.
- Simulator Sampling Time Variation — same family as camera latency / frame-rate timing; could share infrastructure.

## Resolved clarify-pass questions

The clarifications session 2026-04-29 resolved the highest-impact architectural questions. Resolutions are recorded in detail in [§Clarifications](#clarifications). Summary:

| Q | Topic | Resolution |
|---|---|---|
| Q1 | Camera projection + FOV + count | Planar pinhole, 120° FOV, single forward-mounted. Future dual is asymmetric (wide+narrow), not stereo. |
| Q1+ | Minimal calibration | Architecture must work with minimal-to-zero per-unit calibration; per-scenario camera variation training delivers this robustness. |
| Q2 | Frame rate + history mapping | 30 Hz × explicit history at dphi-pattern offsets `[-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now]` (6 slots, past-only, 100 ms uniform grid). Preserves existing 6-slot-per-axis layout with future-sample slots redistributed to finer-grained recent-past (no lookahead for real targets). Final distribution refined in US1 narrative. |
| Q3 | NN architecture | D-simple recurrent (continue 028). NN consumes pre-extracted beacon coordinates from FPGA centroid output, NOT raw pixels. |
| Q4 | Fitness formulation | Reuse pathgen cone-surface fitness with target=playback-aircraft. Visual-lock-time as secondary telemetry only. |
| Q5 | Loss-of-signal + perception interface | Per-sample `visible` flag; collapsed perception output to `(x, y, visible)` per beacon — drops separate channel/in_fov/behind_camera flags as deployment-pipeline noise. Loss of signal is routine; controllers learn orbital search behavior naturally from fitness pressure. |

## Remaining open clarify-pass questions (low-impact, can defer to plan)

The following were on the original list but didn't make the 5-question budget. None blocks plan-phase entry; flagged for resolution during plan or as low-risk defaults in tasks.

1. **Camera latency** — v1 default value (0 ms or a realistic 30-50 ms?). Range for US3 sweeps. Default: 0 ms for v1 baseline (cleanest fitness comparison vs pathgen mode), then sweep 30 / 50 ms in US3 to characterize sensitivity. Confirm in plan.
2. **Camera effects scope for v1** — confirmed load-bearing: color / IR filtering (drives `visible` flag via channel detection). Confirmed stubbed: aberrations, vignetting, motion blur, rolling shutter (interface-present, identity behavior). No clarify needed — record decision in plan.
3. **Library indexing in joint-PRNG** — each tracker scenario uses the *same* joint sample (wind seed, craft variation, entry pose) as its source scenario (deterministic reuse from the recording). The library entry index is what's seeded per scenario; everything else is inherited. Confirm in plan.
4. **Scenario count** — 245 scenarios from source run map 1:1 to 245 tracker-mode scenarios in v1. Expansion (75 craft variations × 5 paths = 375) is a future enhancement when multi-source-run libraries land. Confirm in plan.
