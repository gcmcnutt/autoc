# AutoC Backlog

**Last Updated**: 2026-05-04

---

## 030 spin-offs (deferred from 030 spec at plan-research scoping)

Items extracted from the [030 tracker-mode spec](030-tracker-mode/spec.md) on 2026-05-04, before plan-research begins. These were architecturally part of the 030 epic but earned defer-status under the smoke-test-first scoping (D13 / D16). Some are 031 (sibling-feature) candidates, some are pure backlog. Tagging as such; final 030/031 split is plan-research's call.

### [031 CANDIDATE] Parallel perception-front-end — camera pixels → (x, y, CEP)

- **Trigger**: real-flight beacon hardware build / virtual-beacon flight test (staged-path row 5+).
- **Scope**: the FPGA / DSP pipeline that bridges raw camera pixels to the `(x, y, CEP)` triple 030 consumes — thresholded centroid extraction per color channel, cluster-spread (CEP) computation, threshold-fail sentinel emission. **Includes**: prop-arc occlusion modeling (rolling-shutter resonance with prop RPM, banding artifacts at certain RPMs, intermittent global-shutter occlusion at non-resonant rates); airframe self-occlusion mesh refinement beyond 030 v1's coarse body proxy; LED wavelength + emission-cone tolerance characterization.
- **Why parallel rather than 030-internal**: shares only the `(x, y, CEP)` interface contract with 030; the engineering surface (FPGA design, threshold tuning, color-channel separation, real-camera characterization) is image-domain work that runs alongside 030's controller-side training without touching it. The two features inform each other through the contract: 030's per-scenario PRNG variation experiments tell perception-front-end what camera-spec tolerances need to be met; perception-front-end tells 030 what int8 noise floor and CEP magnitude distribution to actually quantize with.
- **Source design notes**: 030 spec D7 (DMP versioning + parallel feature), D10 (camera v1 baseline + prop-occlusion deferral).
- **Files likely**: new `src/perception/` module (or top-level `perception/` peer to `crrcsim/`), new spec dir `specs/031-perception-front-end/` when this unparks.

### [031 RESEARCH] Perception representation — event-camera + non-linear / dual-FOV optics

- **Trigger**: research thread for post-beacon perception (when 031 perception-front-end above unparks, or sooner if the 030 dual-camera variant experiment needs it). Capture as research now so it informs the camera-spec → resolution-budget conversation when we leave beacons.
- **Question 1 — Event-camera representation**: today the NN sees `(x, y, CEP) × history×6` for each wingtip beacon — a sampled raster of an underlying image-plane process. Once we drop beacons for full-image perception, the natural representation becomes either a dense pixel grid (huge input dim) or an event-stream representation (per-pixel intensity-change events with timestamp + polarity, native to event cameras / DVS sensors). Event-camera reps are sparse, latency-friendly, and naturally encode motion direction; they may be a better match for the controller's cone-tracking task than dense rasters.
- **Question 2 — Optics with non-uniform angular resolution**: a 120° FOV with uniform pixel pitch wastes resolution on the edges where the target rarely sits, and starves the center where it does. Two candidate architectures:
  - **Dual-camera**: wide (~180°) for acquire/orbit-recovery + narrow (~60°) for hi-res tracking. NN sees both feeds (concatenated or as separate channels). Mirrors how birds-of-prey use peripheral + foveal vision.
  - **Single non-linear lens**: fisheye / log-polar / panomorph optics that compress edges and expand the center on the same sensor. Lower hardware cost, but introduces lens calibration and non-linear NDC math; the NN has to learn the warp implicitly.
- **What the early minisim playback informed**: the 120° FOV + raw-NDC-projection presentation gives narrow beacon spacing close-in (~0.26 NDC at 10ft), and the controller learned an emergent orbit-to-reacquire when the target left the FOV — useful evidence that the current rep gets some way, but reacquisition cost in crrcsim's harder dynamics may push the topology budget higher than 030 v1 plans for. A richer rep (event stream) or smarter optics (dual-FOV / non-linear) could lower that topology demand instead.
- **Why research-track, not implementation**: needs a dataset + simulator camera model upgrade (or recorded event-camera bench data) before any controller work; coupled to the 031 perception-front-end FPGA / DSP scoping.
- **Source design notes**: this thread; 030 D10 (single-camera v1 baseline that this would supersede); see also `[BACKLOG] Multi-camera variant experiments` below for the controller-side experiment shell once a camera spec is chosen.

### [031 CANDIDATE] Variable-rate / real-flight source robustness

- **Trigger**: real-flight-recorded trajectories become available (post-virtual-beacon flight test).
- **Scope**: exercise the FR-018 timing model under realistic xiao+INAV telemetry jitter and dropped samples. Confirm determinism end-to-end at non-real-time sim speeds (contract test). 030 v1 timing model is *built to handle* variable-rate sources but is *exercised* only at uniform-rate pathgen-derived sources.
- **Source design notes**: 030 D14 (timing-model exploration row).

### [031 CANDIDATE] Library curation + turn-direction mirror-pairing

- **Trigger**: when 030 smoke-test green and operator wants real-target-class generalization, OR when the controller fails an OOD turn-direction test in early eval.
- **Scope**: library composition tooling — mirror-pair every recorded trajectory into a left-turn / right-turn matched set during ingest (flip Y in NED + flip roll quat + flip rcData[0]); cross-source-run mixing (sim + real, geometry-filtered); auto-bootstrapping (winners-of-tracker-runs feed back into the library). All currently in 030 spec's Out of Scope or C1 / C5 open considerations; collecting them here.
- **Source design notes**: 030 spec C1 (turn-direction symmetry trap), C5 (pathgen vs library decision matrix).
- **Why deferred from 030**: not load-bearing for the smoke test; matters once the operator wants generalization beyond what a single source dmp delivers.

### [031 CANDIDATE] Renderer "exotic goodies" — reverse-projection + dphi-overlay + crash-strike viz

- **Trigger**: post-smoke-test, when operator finds analytics-experimentation needs them to debug specific failures.
- **Scope** (from 030 D15):
  - **Reverse-projection overlay** — given chase pose + recorded perception output `(x, y, CEP)`, compute via reverse camera model (inverting aberrations + int8 quantization, propagating CEP into a 3D uncertainty cone) where the controller "thinks" the target is. Compare to where M1 target actually is (also in M2 dmp per FR-015). Visualizes perception error in 3D — directly diagnostic for "lost fitness because of perception, or because of control?"
  - **Old-style dphi/dtheta overlay** — from chase pose + M1 target's true position, compute what pathgen-mode's old `(dphi, dtheta, dist)` projection would have shown, render alongside the new beacon view. Comparison tool for debugging whether the new representation delivers equivalent or richer information than the old.
  - **Crash-hull strike visualization** — mark hull entries (and `p_crash` fires) in 3rd-person view; aids hull-curriculum tuning.
- **What stays in 030 v1 from D15**: error bars on the camera-POV display (CEP as visible ellipse spread) — cheap, directly load-bearing for smoke-test signal-or-not assessment.
- **Why deferred**: research-grade analytics; not required for smoke-test 4th deliverable.

### [030 v1 — UNPARKED 2026-05-08] CRRCSim mod_inputdev tracker integration (M11.preA)

- **Status**: UNPARKED. Routing decision 2026-05-08: m91 minisim run showed loop closes structurally, but smoke results that matter for sim-to-real must run on FDM-driven physics. Pulling crrcsim integration forward from `[030 v1+]` BACKLOG into v1 path before formal smoke + analytics.
- **Scope**: mirror M6a-M6e strategy split (`PathgenStepper` / `TrackerStepper`) into `crrcsim/src/mod_inputdev/inputdev.cpp`. Sub-checkpoints T079-T083 in `specs/030-tracker-mode/tasks.md` Phase 6.
- **Trigger satisfied**: minisim m91 informal smoke green; operator routing 2026-05-08 chose FDM-grade smoke as v1 acceptance.
- **Why this entry stays separate from M11.preB**: mod_inputdev integration (M11.preA) = make tracker training WORK on FDM. mod_robots/RobotProgrammable (M11.preB, below) = make two-aircraft display VISIBLE during training. They're orthogonal; M11.preA is load-bearing for v1 smoke, M11.preB is optional.

### [030 v1 — UNPARKED 2026-05-08] Live two-aircraft display in crrcsim (RobotProgrammable + mod_robots) — M11.preB

- **Status**: UNPARKED, optional. Land alongside M11.preA only if operator wants to watch tracker training in crrcsim's 3D viewer mid-run (rather than playback the M2 dmp via the M9 renderer after-the-fact).
- **Trigger**: M11.preA outcome — if FDM-grade smoke needs live visual debugging, M11.preB unblocks it. Otherwise defer further (post-v1).
- **Scope**: new `crrcsim/src/mod_robots/robot_programmable.{h,cc}` — `RobotBase` subclass consuming an in-memory pose stream pushed from autoc; `Robots::AddRobot` integration so autoc-side registers the programmable target per scenario; per-scenario teardown / reset. ~150 LOC sketch per [reference_crrcsim_mod_robots.md](../../.claude/projects/-home-gmcnutt-autoc/memory/reference_crrcsim_mod_robots.md). Sub-checkpoints T084-T085 in `specs/030-tracker-mode/tasks.md` Phase 6.
- **Source design notes**: 030 spec FR-002 (original v1 deferral note); 030 plan M4 (was deferred milestone description); research.md R1 (decision rationale).

### [030 v1+ — POST-FLIGHT-PROOF] Xiao tracker-mode prototype (own milestone)

- **Status**: parked at v1+. Xiao firmware currently has zero 030 tracker-mode awareness — `TrackerInputs`, `gather_tracker_inputs`, mode select, and beacon source are all absent. Confirmed 2026-05-09: `grep -rn 'TrackerInput\|gather_tracker\|TRACKER' xiao/src` returns empty; `gather_tracker_inputs` is gated `#ifndef ARDUINO` in [src/nn/evaluator.cc:427](../src/nn/evaluator.cc#L427) so it's intentionally excluded from the xiao build.
- **Trigger**: serious training results in sim with cruise-norm + dist-tanh inputs (post-2026-05-09). Don't ship to xiao before the desktop signal proves the architecture.
- **Scope** (≈M11.preB-sized, not a patch):
  1. Compile-time mode select: `-DAUTOC_MODE=TRACKER` plumbing in xiao/PlatformIO; cherry-pick `TrackerInputs` struct + `kCruiseSpeed_mps` + `kDistToBoundaryScale_m` constants from [include/autoc/nn/nn_inputs.h](../include/autoc/nn/nn_inputs.h).
  2. Beacon source: either fake (synthesized from a target pose stream over MSP for bench-only verification) or real (xiao camera SoM + perception front-end — much bigger lift, ties into the 031 perception-front-end backlog item).
  3. Port `gather_tracker_inputs` body to ARDUINO build (currently excluded). Distance-to-boundary needs an arena-config source on xiao; consider hard-coding for v1 since real flight has no cylinder.
  4. Renderer / blackbox parity for tracker NN log lines (currently only pathgen format is parsed by [tools/renderer.cc:2239](../tools/renderer.cc#L2239)).
- **Why parked**: tracker-mode v1 is sim-only (no hardware deployment in this milestone). Premature xiao porting risks debugging firmware against unstable sim-side inputs. Once cruise-norm + tanh saturation prove out in long bakes, snapshot the contract and port as one coherent piece.
- **Bundle: xiao NN-forward-pass codegen optimization** (filed 2026-05-13). Surfaced from real-flight xiao log line `MSP pipeline: eval=2.3/2.6/7.4ms` for the 1923-weight pathgen NN. Pure-MAC estimate on Cortex-M4F @ 64 MHz is ~0.37 ms; actual measured mean is 2.6 ms = **~6.5× overhead** beyond pure MAC. The bottleneck is the generic table-driven loop in `nn_forward_recurrent` ([src/nn/evaluator.cc:160-249](../src/nn/evaluator.cc#L160-L249)): `std::vector::size()` + `operator[]` indirect lookups in hot loops, `getTopology()` + `getRecurrent()` building std::vector from static arrays each call, `fast_tanh()` function-call overhead per neuron, telemetry-pointer null-check per neuron, buffer-swap pointer dance per layer, generic 64-wide stack-scratch + heap-fallback path. Replace with true straight-line codegen that emits unrolled MAC sequences with weights as inline `.rodata` immediates, tanh inlined as polynomial. Estimated 3-5× speedup on Cortex-M4F (compiler can keep weights in FPU registers, no vector indirection, no function-call overhead). Becomes load-bearing as NN topology grows (T-102 32r tracker, 032 wider inputs, eventual M3 CNN/transformer). Tool change: `tools/nn2cpp.cc` emits straight-line code per (layer, neuron) instead of writing the existing table-driven shim.

### [BACKLOG] Multi-camera variant experiments

- 030 v1 ships single-camera. Multi-camera *interface* is in v1 (per FR-003b — independent per-camera parameter blocks, no assumption of matching configs). *Exercising* the interface is backlog: asymmetric wide+narrow, stereoscopic, wide+telephoto with parallax offset, forward+downward-tilted secondary. Trigger: when 030 smoke-test green and operator wants to use the sim as a camera-spec sandbox (US3-style sweeps) for hardware-design decisions.

### [BACKLOG] ~~Minisim retire / stub / remove~~ → **034 US1 DONE** (2026-05-30)

- Project-level decision surfaced by 030 scoping, captured in 030 D12. Three options: ignore (default), stub, remove. Decision happens at the moment a 030 schema change first forces a touch on `tools/minisim.cc`. Default-to-ignore until then; if 030 plan-phase work hits minisim, upgrade to remove rather than spend porting cost.
- Files: `tools/minisim.cc`, `CMakeLists.txt:100-106`, audit `tests/` for hard dependencies.
- **Update 2026-05-06**: trigger fired during 030 M6a (PathgenStepper extraction) + M6c/d/e (tracker-mode dispatch wired in minisim per session 2026-05-06 routing decision). Per operator call, minisim stays alive through v1 tracker mode rather than retiring. Retirement decision now contingent on 030 smoke outcome.
- **CLOSED 2026-05-30 (034 US1)**: `tools/minisim.cc` deleted, build target removed, config field renamed `Minisim*` → `Worker*` across all `.ini` files (no back-compat alias per Constitution III). `PathgenStepper.{h,cc}` deleted (was minisim-only). crrcsim is the sole worker path. Test-of-record was the M1 eval-parity gate, satisfied implicitly across all 034 bakes.

### [BACKLOG] Airframe self-occlusion calibration (re-enable D10)

- **Surfaced 030 M8b (2026-05-07)**: spec D10 specifies airframe self-occlusion via a chase-body-frame AABB proxy; ray from camera to beacon hitting the proxy ⇒ beacon flagged occluded (sentinel). The placeholder hb1 proxy used through M5/M6/M8 is too coarse — `box ∈ [(-0.6, -0.6, -0.05), (+0.4, +0.6, +0.20)]` puts the camera mount (`z=-0.05`) **exactly on the proxy top boundary**, so any forward-and-slightly-down ray (the geometry seen at every M8b smoke tick) clips the proxy at t=0 and exits via the wing leading edge. Result: every beacon flags occluded for the first half-meter of any descending ray.
- **Workaround in M8b**: compile-time constant `kAirframeOcclusionEnabled = false` in `include/autoc/eval/camera_projection.h`. Production tracker mode currently runs **transparent** (no occlusion check). Per operator routing 2026-05-07: this is intentionally compile-time, not a runtime knob — once sim training shifts to real-flight prep, occlusion is on forever, so a .ini knob would just be lifecycle drag. One-line code change to flip when ready.
- **What's needed to flip back on**:
  - **(a) Real airframe geometry from operator** — actual hb1 fuselage + wing dimensions, not the placeholder. Operator committed to providing post-M8.
  - **(b) Camera mount with clearance** — either physical mast above wing top OR a tighter proxy that doesn't include the wing leading edge. Spec D10 may need refinement on what "top-of-wing-chord mount" means in the model.
  - **(c) Multi-shape proxy** — fuselage + wings as separate AABBs (or a coarse mesh) instead of a single union AABB. The single-AABB pessimism is the proximate cause; even with real dimensions, the bounding box of "fuselage + extended wings" includes a lot of empty space the camera CAN see through.
- **Test impact when re-enabling**: `tests/beacon_projection_tests.cc::AirframeProxyOccludesEmitsSentinel` and `AirframeProxyMissesDoesNotOcclude` construct AirframeProxy{} directly with `enabled` defaulting to `true` in the struct, so they exercise the occlusion-fires path regardless of the production constant. No test changes needed when flipping the constant.
- **Files**: `include/autoc/eval/camera_projection.h` (compile-time `kAirframeOcclusionEnabled` + `defaultAirframeProxyHB1()`), `src/eval/camera_projection.cc` (`projectBeacon` step 4c gates on `input.chase_airframe.enabled`).
- **Trigger to act**: when operator delivers real airframe geometry, or when smoke-test signal indicates training is hurt by the lack of occlusion (NN exploits an "X-ray vision" loophole that won't exist on the real hardware).

### [BACKLOG] ~~M1 vs M2 dmp disambiguation in S3~~ → **034 US3 DONE** (2026-05-31)

- **CLOSED 2026-05-31 (034 US3 / T033)**: chose option (A) run-id prefix. The run-id prefix is now caller-supplied per mode at the call site (`src/autoc.cc`): `tracker-` when `cfg.mode=="tracker"`, else `autoc-`. The mode→prefix decision is factored into the pure `autoc::runIdPrefixForMode()` (`include/autoc/util/run_id.h`) and unit-tested (`tests/run_id_prefix_tests.cc`, T034). Existing v=1 runs keep their `autoc-` prefix (no break).
- **Surfaced 030 M8b (2026-05-06)**: pathgen-mode (M1, source) dmps and tracker-mode (M2, output) dmps share the same S3 bucket (`autoc-storage`) and the same key shape (`<run-id>/genN.dmp`). The run-id format is identical (`autoc-<seed>-<timestamp>Z`) for both — there's no way to tell them apart from the key alone.
- **Why it matters**:
  - Tools that auto-pick "latest run, last gen" (renderer's no-key default, nnextractor's no-keyname default) will silently pick the WRONG kind. A v=2 tracker dmp picked by a v=1-only consumer (renderer pre-M9) chokes on `cameraViewList`; a v=1 source dmp picked by a tracker-aware consumer succeeds but misleads ("cameraViewList: empty (pathgen-mode dmp)" — operator wonders if their tracker run failed to record).
  - As more tracker runs accumulate alongside pathgen runs, the auto-pick mode becomes a coin flip.
- **Options**:
  - **(A) Run-id prefix convention**: tracker-mode autoc generates run-ids like `tracker-<seed>-<timestamp>Z` instead of `autoc-<seed>-<timestamp>Z`. Tools filter on prefix when auto-picking. Smallest change, easy to grep, doesn't break existing v=1 runs (they keep the old prefix).
  - **(B) Subdirectory split**: `autoc-storage/tracker/<id>/genN.dmp` vs `autoc-storage/<id>/genN.dmp`. Cleaner separation but more change.
  - **(C) Inline version suffix**: `genN-v2.dmp` per-file. Simplest filter rule but spreads the convention across every dmp filename.
  - **(D) Load + dispatch on cereal version field**: auto-pick scans S3 ListObjects, opens a few candidate files, picks the most-recent matching the wanted version. Wasteful for "auto-pick latest" since LIST is fast but GET is per-file.
- **My lean**: (A) — autoc reads `Mode = tracker` and prepends `tracker-` to its generated run-id. Operator-friendly grep + tools' auto-pick stays simple (`prefix=autoc-` for pathgen, `prefix=tracker-` for tracker).
- **Trigger to act**: when a non-trivial tracker run history accumulates AND the operator is mixing pathgen + tracker runs in the same workflow. Until then, manual key specification works around it.

### [BACKLOG] ~~AutocConfig auto-print / extensible parameter dump~~ → **034 US3 DONE** (2026-05-31)

- **CLOSED 2026-05-31 (034 US3 / T027)**: chose the X-macro option. `AUTOC_CONFIG_FIELDS(X)` is now the single-source field list; `include/autoc/util/config.h` (decl), `src/util/config.cc` (parse), and the `src/autoc.cc` startup print block all generate from it — the three-place edit is gone. The 034 craft σ knobs + `EnableCraftVariations` flag (T038) were added by editing the one macro list. A config-dump test asserts every active key prints (T028).
- **Surfaced 030 M6e (2026-05-06)**: `src/autoc.cc` startup logging is hand-coded `*logger.info() << "Key: " << cfg.field << endl` for every AutocConfig field. Adding the 030 tracker-mode block (~30 new fields across Source / Trail / CrashHull / Arena / Camera / Beacon) made the fragility obvious — every new knob requires both an AutocConfig field add, a parser line in `src/util/config.cc`, AND a manual print line in `autoc.cc`'s startup dump. Three-place edit per knob.
- **Why it matters**: future feature work (M7 tracker fitness, 031+ camera-config experimentation, etc.) will keep adding knobs. Drift between "config printed" and "config used" is silent — operator looks at the log, doesn't see the new field, assumes default; meanwhile the new knob is active in the run.
- **Options**:
  - **X-macro list** (`#define AUTOC_CONFIG_FIELDS(X) X(int, populationSize, 500) X(int, numberOfGenerations, 50) ...`): single source of truth, generates field decl + parse + print. Familiar pattern (already in tree for some structures?) but adds preprocessor density.
  - **Cereal-to-text adapter**: AutocConfig already could carry a `serialize` template; adding a JSON-archive output pass produces a structured dump for free. Cleaner from a typesystem perspective but requires cereal/json or hand-rolling.
  - **Reflection-via-tuple-of-members**: C++17 alternative — compile-time list of `std::tuple<std::string_view, MemberPtr>` pairs that print(field) + parse(field) iterate over. Modern but more complex to set up.
- **Trigger to act**: when the next milestone adds 5+ knobs (likely M7 tracker fitness), before the manual-print drift bites.
- Files: `include/autoc/util/config.h`, `src/util/config.cc`, `src/autoc.cc:1402-1462` (startup print block).

### [BACKLOG 037] Config system: stack INI files (base + overrides, last-value-wins)

- **Surfaced 037 (2026-06-09)**: the six `autoc*.ini` files duplicate most keys, so any change (new key,
  retuned value) must be hand-applied to each and they silently drift -- the 037 `ControlIntervalMsec`,
  craft actuator-dynamics sigmas, and tightened entry sigmas each had to be touched in 3-6 files, and the
  `autoc-eval-*.ini` copies lagged. inih only parses a single file.
- **Want**: replace the single-file load with a config system that supports STACKING multiple INI files
  in "last value wins" order, so e.g. `-i base.ini -i m2.ini -i eval.ini` composes (base + overrides).
  The eval / visual / tracker variants then become thin override files over one base, eliminating drift.
- **Options**: a small layered wrapper over inih (parse N files in order into one reader); or a more
  standard library (toml++ / libconfig / cpptoml) if we also want typed sections. Whatever lib must still
  feed the X-macro auto-parse/auto-print (`AUTOC_CONFIG_FIELDS`) and preserve determinism + the
  fail-loud-on-missing-required-key behavior (e.g. `ControlIntervalMsec`).
- **Care**: extend the existing single `-i` flag to a repeatable `-i` (order = precedence).
- **Trigger to act**: the next time an ini-wide change has to be hand-applied across all six files.
- Files: `src/util/config.cc` (load), `include/autoc/util/config.h`, the six `autoc*.ini`.

---

## 027 carry-forward → 028 deeper-rnn

[028 spec](028-deeper-rnn/spec.md) inherits the architectural and
incentive bets that 027 plumbed but did not validate.
[027 findings.md](027-recurrent-nn/findings.md) is the consolidated
record of what was built, what happened in rnn1/2/3, and the four
not-yet-disambiguated failure modes — required reading before
working any of the items below.

Items: either "in tree, restored later" (CADENCE7-REDUX markers —
flip to re-enable) or "investigate before retraining":

### [NEXT — 028] D-simple recurrent NN

- 1923-weight recurrent topology (16-wide layer 2 W_hh) plumbed in
  `include/autoc/nn/topology.h`, `src/nn/evaluator.cc`,
  `src/nn/population.cc`, `src/nn/serialization.cc`, `tools/nn2cpp.cc`,
  `tools/minisim.cc`, crrcsim `inputdev_autoc.{h,cpp}`.
- Currently disabled: `NN_RECURRENT[]` all-false in topology.h. Flip
  layer 2 to `true` and update `static_assert` from 1667 → 1923.
- Three rnn experiments (rnn1, rnn2, rnn3) failed to descend below
  cadence7 plateau — failure mode not yet diagnosed.

### [NEXT — 028] C2 stability lexicase axis (027 v4)

- `Σ_t (|out_pt|-1)+(|out_rl|-1)` per scenario, on `ScenarioScore`.
- Plumbed in `src/eval/fitness_decomposition.cc` and
  `src/eval/selection.cc` (commented out behind CADENCE7-REDUX).
- Restore: uncomment the `pool.push_back({s,
  &ScenarioScore::stability_score, 0.5})` in selection.cc.

### [NEXT — 028] C2 energy lexicase axis (027 v3)

- `Σ_t (out_th-1)/2` per scenario, on `ScenarioScore`. Same plumbing
  pattern as stability above; same restore path.

### [NEXT — 028] rnn1/2/3 failure-mode disambiguation

- Required prework before any 028 retraining. Four candidate
  failure modes documented in
  [`027/findings.md`](027-recurrent-nn/findings.md) §"Plausible
  failure modes" — covered by one cheap experiment.
- Cleanest first experiment: **D-alone diagnostic** (D-simple ON,
  C2 axes OFF). Outcomes drive different 028 directions per the
  table in [`028/spec.md`](028-deeper-rnn/spec.md).

### [DEFERRED — post-028] Re-enable Selection027 multi-objective tests

- 4 tests renamed `DISABLED_` in `tests/selection_tests.cc:152+`
  for the cadence7-redux build. Restore once C2 axes are
  uncommented in selection.cc.

### [DEFERRED — post-028] Xiao-side recurrent forward pass

- `nn2cpp` already emits the C code (W_hh mat-vec, hidden-state
  array, `nn_reset()`), but `xiao/src/generated/nn_program_generated.cpp`
  is not regenerated/built/flashed.
- Triggered by 028's sim gate clearing — same discipline as 027.

### [DEFERRED — post-028] Renderer scrubbing with hidden state

- 027 plan open decision #5: when user scrubs the timeline
  backwards, does the recurrent hidden state get recomputed from
  span start, or persisted forward-only? Currently no
  reconstruction; documented limitation.

---

## Legend

- `[NEXT]` - High priority, ready to start
- `[DEFERRED]` - Lower priority, will revisit
- `[DONE]` - Completed in 015 or prior

---

## NN Training Improvements (015) — active

Tracked in [specs/015-nn-training-improvements/tasks.md](015-nn-training-improvements/tasks.md).

Current milestone: robust repeatable training → flight test.

BIG-aero1 complete: 266 gens, fitness 2,955, 294/294 OK, zero crashes.

Generalization eval complete (T180–T184): 86–98% completion across novel paths,
random geometries, and 120% envelope stress. Controller has significant headroom.

Remaining 015 work:
- Phase 7: Xiao-GP sensor sync (blocker for flight test) ← NEXT
- ~~Phase 6: Aircraft parameter variation (sim-to-real)~~ → **034 US4 DONE** (2026-05-31): per-scenario CG / drag / trim / thrust / pitch-eff / roll-eff via the ScenarioMetadata craft-class draw + ramped applyVariationScale; see `specs/034-energy-objective-cleanup/`
- Phase 8: Polish (data.stc, arena layout, legacy tearout, memory leak check) → **partially 034**: minisim retired (US1), smoothness retired (US2), and the US3 fold-ins landed — config X-macro auto-print, right-sized seed cascade, per-(path,wind) variation-table resolution, S3 run-id mode prefix, lighter eval return path. FR-013 (crash-hull PRNG) and FR-014 (mod_inputdev link) were dropped from 034 as **already-satisfied in-tree** (D2 — verified, no work needed). data.stc retirement queued as 034 T052–T055 / 035 prereqs.

---

## Future Features (separate from 015)

### [FUTURE FEATURE — split from 035, 2026-06-04] Hull-crash-cost as a lexicase fitness dimension (M1/M2)

> **→ 038 US1 candidate (2026-06-16)**: pulled into [specs/038-accurate-m2/spec.md](038-accurate-m2/spec.md)
> as the headline. **Reframed there** per operator 2026-06-16: OOB keeps score-stop; hull gets a
> **member-level** penalty — start with **×0.5 of the member's total fitness per strike** (death-penalty
> as escalation), NOT a per-scenario lexicase axis and NOT scalar compositing. No tick-weighting (the
> score-stop already encodes early-vs-late; the gap is the *late* banked crash). Clarify/dig deeper at
> 038 plan phase.

- **Origin**: split out of 035 FR-008b. Energy is 035's job; hull-crash-cost earns its own feature because its penalty design is the hard part.
- **Problem**: in tracker (M2) bakes, hull-strikes grow *monotonically with tracking skill* (030: ~1→11/gen; 032: ~3× faster) — the chase learns to fly *into* the target as it sharpens. This **gates real-flight deployment**. It is a *fitness dimension* (NOT 036/island-selection work). Applies to **M1 and M2 and beyond** — energy + crash incentives are both wanted across modes.
- **Design constraints (operator, 2026-06-04) — the hard part this feature MUST solve**:
  1. **Not a score-stop.** The penalty must NOT merely zero/halt the scenario score on crash.
  2. **Not a flat scalar.** An early crash is worse than a late crash — *any* crash is bad, but timing must register (crash at tick 5 vs tick 500 must be distinguishable). A scalar discount also re-creates the 033 Pareto-corner collapse trap (`project_scalar_multiobjective_collapse`).
  3. **Credit assignment is the core difficulty.** A single individual that crashes in only ~1 of ~300 scenarios is hard to penalize: across the scenario set its one crash barely differentiates it from clean-flying siblings under lexicase (one crash-case among 300 rarely becomes the deciding test case), so a naive per-scenario `crash_cost` gets drowned out. The feature must make a rare-but-fatal crash *selection-decisive* without collapsing to a scalar.
- **Candidate metrics to evaluate (none chosen)**: dedicated `crash_cost` field on `ScenarioScore` with time-to-crash weighting; a per-individual crash-rate aggregate axis (fraction of scenarios with a strike) so a 1/300 crash still registers population-wide; a lexicase crash-priority pass / dominance rule that elevates any crash above tracking ties.
- **Source**: 035 spec FR-008b + Clarifications 2026-06-04; `#GenCrash hullStrike=N` telemetry from 035's M2 baseline run quantifies the current escalation rate and seeds this spec.
- **Sequencing (operator, 2026-06-08): after 037.** 037 is M1-smoothness / loop-rate focused (the faster-loop, smoother-M1 path); hull-crash-cost is the **M2-safety** fitness dimension and naturally follows once 037's faster-loop + local-IMU stack enables the M2 real-flight path it gates. Queue: 035 → 037 → hull-crash (036 islands stays back-pocket; un-backlog only if an M2 bake proves lottery-prone).
- **Fresh evidence (2026-06-08)**: 035 M2 bake t7 (`autoc-…2026-06-08T15:44:25.312Z`) is logging the escalation live — `hullStrike=6/294` by gen 109 and climbing with tracking skill; its final hull-strike curve will be the up-to-date baseline for this feature's penalty design.

### [DEFERRED] Selection Strategy Alternatives
- NSGA-II Pareto: non-dominated sort on (tracking RMSE, energy, worst-case spread)
- Rank-based fitness shaping: CMA-ES style rank-derived weights
- sep-CMA-ES optimizer: pop 5000→50, per-weight step size adaptation
- Only pursue if/when epsilon-lexicase plateaus

### [DEFERRED] Path-Relative Smoothness
- Normalize Δu by path curvature: penalize excess control, not turns
- On hold — slew limiting already killed bang-bang, lexicase hasn't plateaued

### [ABANDONED] INAV pt3 RC Smoothing Filter in CRRCSim (023 Phase 9a experiment)
- **Attempted 2026-04-13**: replicated INAV's `rc_smoothing.c` pt3 filter (3rd-order
  cascade Butterworth LPF) in CRRCSim `inputdev_autoc.cpp`. Filter ran at 333Hz
  FDM rate, smoothing 10Hz NN command steps before FDM surface application.
- **test5 (10kHz passthrough)**: confirmed filter code path is deterministic and
  doesn't degrade convergence when effectively disabled. Matched test4 baseline.
- **test6 (40Hz cutoff)**: training stunted — best stuck at -2225 through 55 gens
  vs test4's -4410 at the same point. Avg fitness ~40% lower, pctInStreak 3% vs 12%.
  The filter changes dynamics enough that the GA can't find productive policies.
- **20Hz also tried**: even worse stunting (not logged as formal test).
- **Root cause**: the filter mechanically prevents the NN from making the quick
  corrections it needs to avoid crashes and maintain streaks. The NN must learn
  that smooth commands are better through its own fitness signal, not be
  mechanically constrained.
- **Conclusion**: pt3 filter approach abandoned for training. If INAV's
  `rc_filter_lpf_hz` is enabled for real flight, the sim-to-real gap from
  not modeling it is acceptable — the NN learns direct control and the
  physical filter just smooths the edges. A fitness-based smoothness
  incentive (lexicase or per-step penalty) is the better path.
- **Code fully backed out** — no pt3 code remains in CRRCSim or AircraftState.

### [DEFERRED] Total Energy Management + Altitude-Aware Distance
- Current distance metric is flat Euclidean — treats "5m above" same as "5m below"
- In reality above is always safer (altitude = energy reserve), below-and-inside is worst
- Observed in T184: NN flies consistently low and outside at slow rabbit speeds ("race horse"
  effect — trained at 16±4 m/s, throttle oscillates at 12 m/s)
- Proposals:
  - Total energy (altitude + airspeed) as NN input or lexicase objective
  - Altitude-aware distance: asymmetric penalty (below penalized more than above)
  - Wider rabbit speed range in training (include 8–12 m/s slow regime)
- Also enables future tactics layer: arena boundary awareness, altitude floor guard
- Post-flight-test refinement — current flat Euclidean tracking is adequate for first flight

### [DEFERRED] Simulator Sampling Time Variation
- Training uses exact 100ms steps; real hardware has jitter (~100ms ± 10ms)
- Add configurable random dither to sim tick interval during training
- Makes NN robust to real-world MSP bus contention and sensor read latency
- Sim-to-real hardening item — after initial flight test data validates baseline

### [NEXT] GPU-Native Evaluation — required for 017
- Accelerate fitness evaluation on GPU (5000 sims/sec vs ~200)
- BIG-3 training was ~352M sim evaluations (pop=3000 × 294 scenarios × 400 gens)
- 017 (vision NN) at ~3K weights needs at minimum the same scale, likely 2-5×
- Beacon projection adds ~46B projection calls at 400 gens — CPU-only is ~77 min/gen
- Current crrcsim is single-threaded C++ with OpenGL dependency — not GPU-parallelizable
- Options: (a) GPU physics sim (CUDA/Vulkan compute), (b) lightweight FDM on GPU with
  beacon projection fused, (c) hybrid: crrcsim for physics, GPU batch for projection
- DGX Spark (GB10) may be sufficient for Option (c); Options (a/b) may need larger GPU
- **Blocking dependency for 017-phase3 at training scale**
- See: [017 spec](017-visual-target-tracking/spec.md)

### [DONE] Renderer: Path reveal timing with variable rabbit speed
- Fixed: renderer now uses rabbit odometer from AircraftState to reveal path
  segments by distance traveled, not time fraction. Stops at crash point.
- Was T516 from 020, closed 2026-03-31.

---

## Infrastructure

### [BACKLOG 038] Standardize training reporting — one `scripts/` wrapper (logfile in → all PNGs out)

> **→ 038 Phase-0 (P0-C) — IN PROGRESS (2026-06-16)**: building `scripts/generate_pngs.sh m1|m2
> <logfile>` + a maintained analytics home `src/analytics/` (dmp-fed plotters + `requirements.txt`)
> this session. See [specs/038-accurate-m2/spec.md](038-accurate-m2/spec.md) Phase 0.

### Per-gen analytics store (SQLite) — retire the CSV run-summary cache (out of 038 P0-C)

- **Surfaced 2026-06-16** building `generate_pngs.sh`: the incremental run-summary cache is a CSV file
  per run-id, appended via `--since-gen`, deduped by gen with `sort -u`, keyed on (run-id + dmp-dump
  build signature) to avoid splicing rows from different runs/builds. That's a relational table
  reimplemented in bash — "we're close to a SQLite use case" (operator).
- **Want**: a single SQLite DB of per-gen analytics. Schema sketch: `gen_summary(run_id, gen,
  best_fitness, mean_energy, mean_stability, mean_streak, crashes, aggr_*, dctrl_*, mag_*,
  path*_rollrate, path*_pitchrate, scenarios, mode, dmp_dump_sig, PRIMARY KEY(run_id, gen))`. Ingest =
  `INSERT OR REPLACE` per gen (dmp-dump writes rows, or the script pipes them in); plotting + cross-run
  comparison become `SELECT`s (the `--compare` overlay is then a `WHERE run_id IN (...)` instead of
  re-scanning old `.log`s). `dmp_dump_sig` column makes a build-math change a row-level concern, not a
  whole-file-invalidate.
- **Why it's clean**: append-once-immutable per (run_id, gen) matches the dmp model; kills the
  dedup-sort + file-key gymnastics; one queryable store for all runs; trivially supports
  compare-N-runs and "latest run" selection.
- **Relation**: extends [BACKLOG 038] reporting above + `project_dmp_driven_analytics_backlog`
  (move analytics off logfile-parsing onto the dmp; /tmp dmp cache). Likely lands as a 038 follow-on
  or its own small infra spec once the shell pipeline proves the report set.
- **Trigger**: when the CSV-cache bookkeeping or cross-run comparison gets painful enough — or when the
  dmp `/tmp` cache item is picked up (do them together).

- **Surfaced**: 037 (2026-06-15/16), during 037-t11-m2 reporting. Generating the standard per-run
  PNGs ([docs/REPORTS.md](../docs/REPORTS.md)) is a hand-run sequence of `python3` calls scattered
  across `specs/034-…`, `specs/035-…`, `specs/037-…`, with the S3 run-id, gen number, mode, and
  config (`autoc.ini` vs `autoc-tracker.ini`) threaded by hand each time. M1 = 4 reports; M2 = 6
  (adds `gen_diag` + `intercept_analysis`). Now that reporting is routine, this wants to be one
  command.
- **Want**: a single shell script in **`scripts/`** (cross-version utility per the scripts-dir scope
  rule) whose main param is just the **logfile name** (e.g. `logs/autoc-037-t11-m2.log`). It derives
  everything else from the log — run-id + `S3Bucket` + `Mode` are printed near the log head, latest
  gen from the last `#NNGen` — and calls the plotters to emit the full PNG set. "viola."
- **Stabilize the plotters**: move the dmp-fed (035+) plot pythons out of the frozen feature dirs into
  a **maintained analytics package** (e.g. `src/analytics/` or `analytics/`) so contract changes can
  edit them in place. This is a deliberate carve-out from the historical-scripts-immutable rule: the
  *pre-035 data.dat* scripts stay frozen; the dmp-fed set becomes the maintained, standardized one.
  Add a **`pyproject.toml` / `requirements.txt`** (numpy, matplotlib) so the reporting env is
  reproducible. Config via CLI flags, not env vars; config-file flag stays `-i`.
- **Already-landed groundwork (2026-06-15, this session)**: `tools/dmp_dump.cc` now emits a per-tick
  `stpPt` column for the **tracker** CSV (recomputed from the recorded target velocity + trail-rabbit,
  exactly as `fitness_decomposition.cc` — derived not recorded, same as pathgen; no dmp/format
  change, no run restart). `specs/037-20hz-control-loop/dynamics_progress.py` was made schema-generic
  (derives chaser→target distance; the regime panel degrades to intercept/patrol when `stpPt` is
  absent), so it now runs on M1 and M2. These are the kind of contract edits the standardized package
  should own.
- **Related**: the dmp-driven-analytics infra item below (default-to-latest-run, gen counts on
  stderr, `/tmp` dmp cache) and the self-describing-dmp item are natural companions — a standardized
  reporting front-end is where they'd surface.
- **Trigger to act**: 038, or the next time a report has to be regenerated for a routine M1/M2 bake.

### Integer-ms `simTimeMsec` truncation — stamp by rounding / step-count (out of 037 M2 source-spacing)

- **Surfaced**: 037 (2026-06-15), M2/t11 launch. The tracker source-spacing fail-loud rejected the
  t10 source: its recorded tick gaps are 49/50/51 ms (first gap deterministically 49) even though
  the true cadence is exactly 50 ms.
- **Root cause**: `SimStateHandler::getSimulationTimeSinceReset()` returns
  `(unsigned long)(sim_steps * Global::dt * 1000)` — the 200 Hz / 5 ms step clock **truncated** to
  integer ms ([crrcsim/src/SimStateHandler.cpp:392](../crrcsim/src/SimStateHandler.cpp#L392)).
  Combined with the `multiloop`/`dDeltaT` drift-corrector (lines 75-96), each control tick lands
  just under a 50-multiple and truncates down to `xx9`, re-syncing to exact 50-multiples every few
  ticks. So dmps store ±1 ms-jittered timestamps. (Same family as the crrcsim CLAUDE.md note:
  "sim_steps counter is wall-clock dependent because multiloop varies.")
- **Right fix**: stamp `simTimeMsec` cleanly — either **round** instead of truncate, or derive from
  the integer step-count directly (`sim_steps * 1000 / stepsPerSec`), so a 20 Hz run records exact
  50 ms gaps. Then the tracker source-spacing check could go back to a strict single-gap test.
- **DEFERRED — interim shipped 2026-06-15**: the M2 source-spacing checks
  ([crrcsim_tracker_helper.cpp](../crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp) +
  [tracker_stepper.cc](../src/eval/tracker_stepper.cc)) now test the **average** gap
  `(last-first)/(N-1)` (= 50.0 exactly, immune to the per-tick truncation jitter, still catches a
  real 100 ms-vs-50 ms mismatch). This unblocks t11 without touching the recording format. The
  proper stamp fix is a dmp-format/determinism change affecting every dmp — do it when a dmp break
  is acceptable anyway (e.g. bundled with the self-describing-dmp item below).

### Self-describing dmp — record config block in every gen dmp (out of 037 P-O13)

> **→ 038 P0-B candidate (2026-06-16, partial)**: 038 Phase-0 takes only the **renderer-side
> config-hygiene slice** (renderer/replay reads fitness/cadence params from the run/dmp, not the live
> `.ini`). The full `EvalResults` config-block serialization stays this item.

- **Surfaced**: 037 (2026-06-14), while building the renderer playback HUD (P-O12). Score replay in
  `dmp_dump.cc` and `tools/renderer.cc` reads fitness/cadence params (cone angle, dist scales, streak
  threshold/ramp/mult, `kCadenceTickScale`) from the *current* `autoc.ini` via
  `ConfigManager::getConfig()` — wrong if the ini drifts from the run that produced the dmp.
- **Scope when picked up**: serialize the fitness/cadence config block into `EvalResults` (every
  per-gen dmp — small fixed duplication, makes any dmp standalone-replayable; operator 2026-06-14).
  Flip readers (dmp_dump, renderer, analysis) to **prefer dmp-recorded config, fall back to ini** for
  pre-change dmps. The **only** remaining ini dependence is the S3 profile/bucket (bootstrap needed to
  *fetch* the dmp).
- **DEFERRED — keep dmp binary compat for now (operator 2026-06-14)**: this is a fail-loud
  `EvalResults` schema change that breaks reading current dmps (t6–t10). The format is being held
  stable for a while so t10+ dmps stay readable across builds; revisit when a dmp format break is
  acceptable anyway (e.g. bundled with the next schema-touching feature).
- **In the spirit of** `project_dmp_driven_analytics_backlog` (move analytics off the logfile/ini
  onto the dmp). Touch: `EvalResults` serialize (autoc write), `dmp_dump.cc` + `tools/renderer.cc`
  (read dmp-config-if-present else ConfigManager).

### Portable agent memory — repo-authoritative, ~/.claude as non-authoritative recall index (out of 032)

- **Surfaced**: 032 ("memory location debate", open since phase 1); resolved in principle 2026-06-10
  during the 037 t6 bake when the operator challenged storing project findings in the
  machine-local `~/.claude/.../MEMORY.md` while the project runs on multiple machines/repos with
  speckit as the documentation system.
- **Principle (operator 2026-06-10)**: agent memory is a **summary/reference layer over the in-repo
  specs — loaded automatically, but NEVER authoritative** for instructions, decisions, or project
  facts. The repo (specs/, docs/, CLAUDE.md) is the single source of truth; memory entries should be
  one-line hooks pointing at repo paths, so recall works without content forking.
- **Scope when picked up**:
  1. Migrate the `project_*` memories (bang-bang migration, servo-era metrics, spiral strategy,
     basin lottery, …) into a versioned in-repo home (e.g. `docs/NOTES.md` / `docs/lessons/`),
     leaving only pointer lines in `~/.claude` MEMORY.md.
  2. Move agent-workflow preferences (never-push, build conventions, regression-gate ownership)
     into the committed `CLAUDE.md` / `.claude/` so agents on every machine inherit them.
  3. Keep truly machine-local facts (paths, local build quirks) as the only `~/.claude`-resident
     content.
- **Why it matters**: today's memory is invisible to the Windows-side agent, unversioned,
  unreviewable, and prone to divergence from the specs it summarizes (two copies, one drifts).

### [NEXT — gated on r1/r2/r3 outcome] M1 basin-landscape protocol + improved pathgen baseline (pop=8000 / wind=36)

- **Surfaced 2026-05-24/25**: 033 phase 1–4 stalls were initially read as a code regression in `acf732f` (033 PRNG rework + smoothness penalty). The 2026-05-24/25 bisect proved they're almost certainly **seed-luck, not code**:
  - 029-end + Seed=1777749601 → bit-identical replay of pastonly3.
  - 030-end (8bfad02) + same seed → bit-identical through gen 105.
  - 032-end (a0d9c16) + same seed → bit-identical through gen 41 (past variation step #1).
  - a62257a (first 033 commit, structurally = 032 + crrcsim pointer bump) + same seed → bit-identical through gen 50.
  - a62257a + a *fresh* time-based seed (1779683898) → **stalled at −10k flat for 300+ gens**. Same code, different basin draw, completely different trajectory.
- **Historical pattern**: basin lottery has been there since 028. pastonly1 (cut at gen 218) plateaued near −18k, pastonly2 climbed to −52k, pastonly3 to −56k, more-rnn1 to a mediocre −30k slow-climb. The 033 phases 1–4 were N=4 unlucky fresh-seed draws read as a regression because the comparison point (pastonly3) was rare-good.
- **Experiment RESULT (2026-05-27): 2 of 3 climbers on pop=8000 / wind=36.** Three fresh-seed runs on 032 branch tip (a0d9c16), `PopulationSize = 8000` (was 5000), `WindScenarios = 36` (was 49), `Seed = -1`. Reference telemetry committed alongside this entry as `specs/032-tracker-nn-enhancements/pop8000-wind36-r{1,2,3}-data.stc` + the `_evolution_progress` / `_per_axis_*` PNGs.
  - **r1** (seed 1779727733): strong climber. Fast inflection ~gen 100, peak Best −39,495 at gen 307 (per-scenario −182.8 ≈ pastonly3-gen-500 quality). Stopped gen 333 to free the machine for r2.
  - **r2** (seed 1779802002): slow-then-strong climber. Looked like a slow-climb basin through gen 200 (70 gens behind r1), then inflected hard at gen 280 and converged onto the pastonly3 curve — Best −34,167 at gen 422 (per-scenario −158 ≈ pastonly3-gen-400). Stopped gen 422 for r3.
  - **r3** (seed 1779886360): **STUCK — throttle-peg dead-neuron basin.** Best flat at −10.6k from gen 200–276, avgMaxStreak frozen at ~4.8, bestSigma frozen at 0.141 (no annealing), and the smoking gun: throttle amplitude = exactly 1.000 (σ=0.000) across all 216 scenarios with throttle dCtrl = 0.000. Same attractor as 033 phase-3 (see phase3-stall-report) and the 2026-05-24 freshseed-on-a62257a stall. Stopped gen 276 — conclusive.
- **Interpretation**: pop=8000 / wind=36 **improves the climb rate (2/3 = 67% vs the recent 1/4 = 25%) but does NOT eliminate the basin lottery.** The throttle-peg stuck basin still catches roughly 1 in 3 fresh seeds. The M1 landscape has (at least) two attractors: the climbing family (pastonly2/3, r1, r2 — throttle modulates) and the throttle-pegged dead-neuron family (033 phases, a62257a-freshseed, r3 — throttle locks at +1.0 with zero variance). The config shifts the odds but the multi-basin structure is intrinsic, not config-induced.
- **Critical distinction (climber vs stuck, early-detection)**: r1 and r2 BOTH went through early throttle *saturation* (mean ~0.85, σ>0, modulating) and escaped it; r3 went to throttle *lock* (exactly 1.000, σ=0.000). The dead-neuron σ=0.000 signature — not the saturation level — is the reliable stuck-basin tell. Also: avgMaxStreak frozen + bestSigma not annealing past ~0.14 by gen 200 confirms it. (The gen-150 inflection heuristic from this entry's earlier draft is unreliable — r2 didn't inflect until gen 280 yet finished strong. Use the throttle-σ + sigma-anneal signals instead.)
- **Baseline decision**: by the ≥2-of-3 gate, pop=8000 / wind=36 qualifies as the improved default — committed to `autoc.ini` here. But document the caveat that it's a 67% climber config, not a fix; the stuck basin is still reachable.
- **Scope when this entry unparks**:
  1. **Adopt-or-reject the pop=8000 / wind=36 baseline** per r1/r2/r3 outcome. If ≥ 2 reach ≥ −40k by gen 800, the autoc.ini defaults flip (with `# was: pop=5000 wind=49` comment retained for history). If ≤ 1, the experiment is recorded as a negative result and the existing defaults stay.
  2. **Write a 2-page basin-landscape reference doc** classifying all historical training runs available in the repo (more-rnn1/2/3, pastonly1/2/3, 033 phase 1–4, 034 r1/r2/r3 once complete) into {climbing ≥ −40k, slow-climb −25k to −40k, stuck ≤ −15k, borderline} with one-line rationale each. Threshold values derived from the 029 reference set.
  3. **Codify the seed-vs-code disambiguation rule**: "Stall is *seed luck* until proven *code regression* by N ≥ 3 fresh seeds on the same code state." Future operators (including the operator after weeks away) read this first when an M1 stall happens, so they don't repeat the 033 4-bake bisect.
  4. The gen-150 inflection point is the *early-detection* signal — runs that have crossed −15k by gen 150 climb; runs flat near −10k at gen 150 stay flat. Useful for killing stuck runs early, not for final classification (rare late inflections exist).
- **Why a backlog entry and not a feature**: this is investigative + lightweight config-baseline work. No new training architecture, no schema change, no new code surface. The deliverable is (a) edit `autoc.ini` defaults, (b) write a reference doc, (c) update operator habits — all once the r1/r2/r3 evidence is in.
- **Related**:
  - [project_lexicase_mad_epsilon](../.claude/projects/-home-gmcnutt-autoc/memory/project_lexicase_mad_epsilon.md) — recommended follow-on when craft/camera variations land (5–6 dim regime). Constant absolute epsilon = 0.5 doesn't scale with per-scenario magnitude; MAD-relative is the natural fix. Not part of this entry.
  - The variation-budget table (3 dim → 36–49 scenarios marginal; 5–6 dim → 200–350 for pairwise) sits implicitly in this entry — when craft/camera dims land, `WindScenarios` will need to grow back up regardless of the pop=8000 outcome.
  - `[project_no_future_curve_shape]`, `[project_scalar_multiobjective_collapse]` — supporting memory entries.
- **Trigger to unpark**: DONE 2026-05-27 (2 of 3 climbers — see RESULT above). Remaining open work: write the basin-landscape reference doc + decide whether to actively attack the stuck basin (see the demetic-processing entry below).
- **Files**: `autoc.ini` (PopulationSize, WindScenarios), new `docs/basin-landscape-reference.md` (or wherever the operator routing prefers — possibly `specs/029-no-future-arch/` since the reference runs anchor there).
- **Critical normalization caveat (surfaced 2026-05-25)**: `Best` / `Avg` / `Worst` in `data.stc` and the gen log line are **sums over the scenarios in that run's config** (paths × winds). Comparing absolute Best across runs with different `WindScenarios` or `SimNumPathsPerGeneration` is apples-to-oranges. r1 (pop=8000 / wind=36 / 216 scenarios) at gen 181 has Best −23,174 vs pastonly3 (5000 / 49 / 294 scenarios) at gen 181 ≈ −25,800; the raw numbers suggest r1 is behind, but per-scenario (−107.3/scen vs −87.8/scen) r1 is ~22% MORE negative — i.e., a stronger controller. avgMaxStreak / pctInStreak are already per-scenario and directly comparable (r1 was 2× pastonly3's at gen 100, still ahead at gen 181). For the reference doc + thresholds, normalize either by (a) using per-scenario averages as the universal currency, OR (b) rescaling the historical thresholds to the run's scenario count (e.g., −40k climbing at 294 scenarios → ~−29k at 216). The threshold values quoted above (≥ −40k climbing, ≤ −15k stuck) are 294-scenario-anchored; the doc must spell this out and provide a rescaling formula or per-scenario equivalents.
- **Framing — this is "factory" investment, not control improvement (2026-05-27)**: the entire diversion from 029 through this basin work was *not* about making a better controller. M2 already has its own (different) NN topology and performed well on its own. The goal of all this is a **repeatable training environment** so M2 training can be clean and honest — being able to reproduce a source scenario's exact environment (wind/entry/rabbit at the exact tick) is what makes M2 fidelity analyzable. Mental model: this is the **factory ($$$) model** — the assembly line that produces controllers — which we keep refining as we improve control. The controller quality is the product; the determinism + basin reliability + reproducible-environment work is the tooling that lets the factory run without expensive re-rolls. Frame future "is this worth it?" decisions on this axis: does it make the factory cheaper/more reliable per controller produced, or does it improve the controller itself? Different budgets, different justifications.
- **Forward loop — craft + camera variations into M1 (2026-05-27)** *(camera-variations → 038 US2
  candidate, 2026-06-16: pulled into 038; the source-side-vs-M2-side question is US2's opening spike)*:
  somewhere in this sequence we go back and add **craft variations** and **camera variations (at least for seed)** into M1, to prepare the final generalization of the craft. The intent: M1 source trajectories should eventually span craft-parameter and camera-config diversity so that when M2 trains against them (in their own reproduced environments), the resulting controller generalizes across the real hardware envelope, not just one nominal airframe/camera. This is the "rinse/repeat" outer loop: improve the factory (determinism, basins, variation coverage) → produce a controller → learn → widen variation → repeat. The variation-budget table elsewhere in this entry (3 dim → 36–49 scenarios; 5–6 dim → 200–350 for pairwise) is the sizing guide for when craft+camera land and the dim count climbs to 5–6.
  - **Craft variations DONE 2026-05-31 (034 US4)**: 6 axes (CG / drag / trim / thrust scale / pitch-eff / roll-eff) plumbed through the `ScenarioMetadata.craft*` + `craftSeed` cascade, ramped via shared `applyVariationScale`, with eval-mode replay via `gEvalVariationScaleOverride`. Sensible σ defaults in autoc.ini + macro-disable knob `EnableCraftVariations`. Confirmation experiment ini (`autoc-craft-only.ini`) + per-scenario control × craft bias regression analysis = Phase 7 of 034. Camera variations are still pending (not in 034 scope — the cameraSeed insertion point is documented in `ScenarioMetadata` for a future feature).
- **Broader context**: this investigation was a sidetrack from the M2-reproducibility line of work. The next moves on the main path:
  1. **033 PRNG single-SHA bug hunt** *(→ 038 P0-A candidate, 2026-06-16: pulled into 038 Phase-0 as
     the PRNG-validation prerequisite)*: even with basin-lottery diagnosed, the `acf732f` (033 PRNG architecture rework) is still worth inspecting for a bug — particularly because phases 1–4 all stalled on what may have been *coincidentally* unlucky seeds but the PRNG cascade rework structurally changed the seed → scenario mapping. If there's a bug routing fresh seeds into a worse part of the basin landscape, finding it would matter for the M2 work.
  2. **M2 sim playback parity with M1 — source replayed *in its own original environment*** (clarified 2026-05-27): the load-bearing experiment after the basin work. The key design point: an M1 source trajectory was produced under a specific joint-PRNG scenario = (path_index, wind_seed, entry_pose, rabbit-speed profile). The source path is *shaped by* that wind — it crabbed, drifted, and adjusted throttle for those exact gusts at those exact ticks. For realistic M2 (tracker) training, the chase aircraft must fly through the **same air mass at the same time** — i.e., the M2 sim replays the source's *original variation scenario* so the chase feels the identical wind/conditions while pursuing. Otherwise there's a physics mismatch: the source moved as if wind-blown, but the chase tracks it in different (or calm) air, which is not what happens when a real chase pursues a real target through one shared air mass.
     - **Why this depends on the PRNG work**: faithfully reproducing "the source scenario's environment" requires the scenario to be deterministically regenerable from its seeds. That is exactly what the 033 PRNG rework must get right. Validating 033's PRNG ("mostly working" via the pop8k/wind36 run on 033 code) is therefore the *prerequisite* for honest M2 training — if you can't reproduce a source scenario's wind/entry/rabbit faithfully, you can't put the chase in the same world the source flew through.
     - **The validation question**: if M2 results track M1's training-time signal closely (chase in source's own environment), the M2 architecture + perception pipeline are validated; if they diverge, perception or some other M2-side delta is leaking. The basin-landscape work is housekeeping that lets us cleanly attribute any M2 divergence to *code* rather than *seed luck*.
     - **Sequencing**: this M2-parity run happens **before** smoothness is re-added (smoothness is a separate later objective; don't conflate it with the M2-fidelity question).
     - **Run-budget implication (clarified 2026-05-27)**: the basin lottery is intrinsic to the current NN topology + evolution settings (~1:3 to 1:4 fail rate even at pop=8000/wind=36 — see Interpretation bullet above). It is NOT an M1-only property, so the M2-parity training run is subject to the same odds. Plan for **2–3 M2 attempts** (r1/r2/r3-style) to land a non-stuck climber worth analyzing, rather than assuming a single M2 run validates or refutes the architecture. A stuck M2 run tells us nothing about M2 fidelity — only that we drew the throttle-lock basin again. Budget compute accordingly. (This also re-raises the standing question of whether to attack the topology/evolution settings to lower the intrinsic fail rate — see the demetic/island-model research entry below.)

### [RESEARCH — would demetic / island-model GA escape the stuck basin?] 

- **Surfaced 2026-05-27** from the pop=8000 / wind=36 result above: we now have direct evidence the M1 landscape has **at least two attractors** — the climbing family (throttle modulates) and the throttle-pegged dead-neuron family (throttle locks at +1.0, σ=0.000). A single panmictic population commits to ONE basin per run; whichever the early-gen dynamics fall into is where it stays (r1/r2 climbed, r3 locked). The whole pop is one bet on one basin.
- **The question**: would a **demetic (island-model) GA** — split the population into N semi-isolated subpopulations (demes), each evolving independently, with occasional migration of elites between them — let the run explore multiple basins *in parallel within a single bake* and migrate winners out of the stuck deme? Instead of 3 separate runs to find 2 climbers, one demetic run with (say) 4–8 demes would sample 4–8 basins simultaneously; as soon as one deme finds the climbing basin, migration pulls the throttle-pegged demes toward it. This directly targets the basin lottery rather than just re-rolling the dice (which is what r1/r2/r3 amounted to).
- **History / nuance**: demetic mode was tried before and is recorded at the bottom of this file as "~~Demetic Mode Elite~~ — superseded by lexicase selection." But that supersession was about *selection-pressure / diversity within a single population* — lexicase preserves behavioral diversity panmictically. The basin-escape question is **different**: lexicase keeps diverse *specialists* but they're all still descending into the same single basin's neighborhood; demes keep diverse *populations* that can occupy genuinely different basins. The two mechanisms are complementary, not redundant. Worth re-opening demetic specifically through the basin-diversity lens, not the diversity-of-specialists lens that lexicase already covers.
- **Design sketch (if pursued)**: N demes of pop/N each; intra-deme = current lexicase selection unchanged; inter-deme migration every K gens (ring or fully-connected topology, migrate top-M elites, replace worst-M in destination). Determinism contract (per [project_variation_design_principles](../.claude/projects/-home-gmcnutt-autoc/memory/project_variation_design_principles.md)) must hold: migration schedule + deme-assignment seeded from the master PRNG. Watch for the failure mode where a strong deme's elite migrates everywhere and collapses all demes into one basin too early (premature convergence — defeats the purpose); migration rate is the key knob.
- **Cheaper alternatives to weigh first**: (a) **throttle-init perturbation** — the stuck basin is specifically a throttle dead-neuron; biasing initial throttle weights away from saturation, or adding a tiny per-gen throttle-dither, might prevent the lock without architectural change; (b) **lexicase energy axis** — re-enable the commented-out `ScenarioScore::energy_score` pool (the `Σ(out_th−1)/2` term from 027 v3, still in `selection.cc` behind CADENCE7-REDUX) so a throttle-pegged individual fails the energy test cases and can't dominate selection — this may kill the throttle-peg basin directly and is a one-line uncomment vs a deme-architecture build.
- **Trigger to act**: when basin reliability becomes load-bearing — i.e., when we're spending real compute on M1 controllers we intend to fly, and a 1-in-3 stuck rate is too expensive to re-roll. Try the cheap alternatives (energy axis, throttle-init) before the demetic architecture lift. The energy-axis re-enable is the highest-leverage first experiment since it directly attacks the observed failure mode and is nearly free to test.
- **Added motivation (2026-05-27) — M2-experiment throughput**: the basin lottery now has a *second* cost beyond M1 controller production. The M2-parity experiment (see the M2 sim playback bullet above) needs a non-stuck training run to analyze, and it's subject to the same ~1:3 odds, so each M2 question costs 2–3 bakes (~30–40 hr each) just to clear the lottery. Dropping the intrinsic fail rate from ~30% toward single digits would pay for itself immediately in M2-experiment throughput — every M2 (and future M3) iteration re-pays the dice-roll tax until this is fixed. This makes the cheap energy-axis experiment more attractive to run *sooner* rather than deferring to "when we fly M1," because the factory-throughput cost is already being paid on every M2 iteration.

### [BACKLOG] Eval "playback re-eval" mode — read an old run back instead of seed-regenerating

- **Surfaced 2026-05-27** from the M2-environment-parity discussion (see the M1 basin-landscape entry's "M2 sim playback parity" bullet above). The confirmation question "is the M2 chase flying through the same *variations* the source flew through?" reduces to eval-mode semantics: eval already expects **exact replay**. But today's eval (`EvaluateMode=1`) regenerates scenarios **from the seed** — it does NOT read a prior recorded run and replay it. So we can re-derive a scenario's inputs deterministically, but we can't point eval at an old run's dmp and say "reproduce this exactly."
- **Scope clarification (what we actually want — NOT lockstep tick-replay)**: for a *generalization* goal, tick-exact replay of the chase is neither achievable nor needed — the chase trails ~10 ft so it samples the wind field at a different point than the target did. What matters is **variation parity**: source-target and chase fly through the *same generated world* (same wind_seed / entry / rabbit / future craft+camera draws), each sampling it from their own position. The heavyweight per-tick wind-witness machinery is overkill for that.
- **The bounded useful thing — a new eval sub-mode**: `EvaluateMode = playback` (or similar) that takes an **old run's dmp** (source trajectory + its `ScenarioMetadata` variation + recorded control commands from the full `AircraftState` records) and:
  - **(a) determinism witness (the "#1 source-replay" test)**: re-run the recorded control commands through the same FDM with the wind field regenerated from the dmp's variation seeds, from the recorded entry pose; assert the reproduced position/orientation/velocity matches the recorded trajectory within FP tolerance. Proves the harness reconstructs the source's environment + physics faithfully. This is the honest precondition for any M2-parity training.
  - **(b) re-eval witness**: re-run the *NN* against the exact recorded scenario environment and compare fitness to the originally-recorded fitness. This is the natural extension of the existing perf-rebuild bitwise gate ([reference_perf_build_reproducibility] — eval-vs-training exact match) from "within a build, regenerate from seed" to "across time, replay a recorded run."
- **Why it's a clean addition rather than bespoke tooling**: it's the same eval pipeline, just sourcing the scenario from a recorded dmp instead of the seed-regenerator. Reuses `source_dmp_loader` (already reads source dmps for tracker mode). The cross-version determinism question ("does today's build reproduce last month's recorded run?") and the M2-environment-parity question are the *same* mechanism.
- **Schema note (carried from the M2 discussion)**: the trimmed `SourceTickSample` ([source_trajectory.h:37](../include/autoc/eval/source_trajectory.h#L37)) drops `wind_velocity` and the control commands; the full `AircraftState` dmp ([aircraft_state.h:471](../include/autoc/eval/aircraft_state.h#L471)) keeps both. Mode (a) needs the control commands + entry pose, so playback re-eval must read the **full dmp**, not the trimmed transport library.
- **Verdict on priority**: probably **not worth building for the M2-fidelity question alone** (generalization doesn't need it). But it has independent value as a **cross-version / cross-host determinism regression harness** — which is more relevant now that spec-kit work spans multiple hosts. Trigger: when a determinism question costs real debugging time (e.g., suspected non-determinism in a bake, or validating a build on host B reproduces host A's run), OR when M2-parity validation is actually undertaken.

### [NEXT — post more-rnn3 completion] Genome ablation tool — generic weight/input editor + eval harness
- **Trigger**: when more-rnn3 finishes 800-gen run. Run alongside the usual subjective robustness eval tests (the existing post-run set) to add a quantitative ablation dimension.
- **Immediate use case**: answer the question "is the R in RNN actually helping, or is more-rnn3's outperformance just from +256 W_hh weights of capacity?" Take the more-rnn3 winning genome, **zero W_hh weights**, run eval pipeline, compare fitness.
  - If zeroed-W_hh fitness collapses (e.g., back to cadence7-redux's −33000): recurrence is load-bearing.
  - If zeroed-W_hh fitness ≈ original: recurrence is decorative; FF capacity at matched weight count would have done the same. (Then the 2-day matched-FF retrain becomes the next experiment.)
- **Generalization** (the actual ask — design as a reusable diagnostic): a NNGenome editor that takes a *mask spec* and an input genome `.dmp`, produces a modified genome, runs eval, reports fitness diff. Mask spec covers ablation regions like:
  - `--zero-whh` → zero all W_hh weight blocks (drop recurrence)
  - `--zero-input GYRO_P,GYRO_Q,GYRO_R` → zero specific NN input columns at each tick (drop rate gyros — does the controller still track without body-rate sensing?)
  - `--zero-input DPHI_NOW,...` → drop temporal history features
  - `--zero-layer N` → zero entire hidden layer (drop a feedforward stage)
  - Other "drop X and re-evaluate" patterns as they arise in research questions
- **Implementation sketch**:
  - New tool: `tools/nn_ablate.cc` (or extend `nnextractor`/`minisim`).
  - Accepts an input `.dmp` (or `nn_weights.dat`) + mask spec + autoc-eval.ini (or autoc.ini).
  - Loads NNGenome, applies mask (zeros specified weights/blocks OR rewrites `nn_gather_inputs` output to mask specific input fields).
  - Runs same eval pipeline as `runNNEvaluation()` — uses identical scenario set, deterministic seeds.
  - Outputs: original fitness, ablated fitness, Δ fitness, per-axis dCtrl/`<|out|>` Δ, per-scenario fitness Δ histogram.
- **Why this design** (vs one-off ablation scripts): research questions like "drop gyros," "drop recurrence," "drop temporal history" recur. A generic mask-based editor amortizes the eval-pipeline plumbing across all of them. Names like `GYRO_P` align with the [Type-Safe NN Sensor Interface](#next-type-safe-nn-sensor-interface) work — both share the "name input columns by enum" need.
- **Out of scope for v1**: weight perturbation (Gaussian noise), targeted permutation, partial zeroing (e.g., 50% of W_hh). Add only if specific research questions need them.

### [NEXT] Type-Safe NN Sensor Interface
- Currently NN inputs/outputs are opaque `float[]` arrays indexed by magic numbers
- Topology changes (29→27) caused silent serialization corruption — now crashes, but
  still fragile for future sensor additions (gravity vector, camera, etc.)
- Need: a typed sensor struct or enum-indexed map that
  - Names each input (e.g. `GYRO_P`, `QUAT_W`, `DPHI_NOW`)
  - Carries type, units, valid range metadata
  - Auto-generates topology count from struct definition
  - Serializes with field names or tags so format is self-describing
  - Compile-time error if evaluator.cc and topology.h disagree
- Also unify the scattered constants: NN_INPUT_COUNT in topology.h, autoc.h,
  evaluator.cc, tests, data.dat format comments, sim_response.py parser
- This is load-bearing for 021+ as we iterate on sensor inputs frequently
- Files that must change when NN_INPUT_COUNT changes (021 learnings):
  - `include/autoc/nn/topology.h` — count, weight count, topology string
  - `include/autoc/autoc.h` — duplicate defines (DISTANCE_TARGET etc.)
  - `src/nn/evaluator.cc` — nn_gather_inputs(), index mapping, comments
  - `src/autoc.cc` — data.dat format string, header, field indices
  - `tests/contract_evaluator_tests.cc` — topology assertions
  - `tests/nn_evaluator_tests.cc` — input layout assertions
  - `specs/019-improved-crrcsim/sim_response.py` — data.dat parser
  - `xiao/src/msplink.cpp` — xiao-side input gathering
  - `include/autoc/eval/aircraft_state.h` — nnInputs_ array size, serialization
- Consideration: prev commands (pitchCmd/rollCmd/throttleCmd feedback) as optional
  inputs — may want to toggle these on/off during experimentation. Type-safe
  interface should support optional/conditional inputs without recompiling everything

### [NEXT] Eval Fitness Computation — Bugs
- **Bug 1: Different metric** — ✅ FIXED post-022. Both training and eval now use
  `computeScenarioScores()` + `aggregateRawFitness()` (conical surface). Verified in
  fitness_decomposition.cc and autoc.cc.
- **Bug 2: Stale fitness in S3** — Eval uploads original NN weights (with training-time
  fitness baked into NN01 format) to S3 via `evalResults.gp`. Renderer deserializes
  this and shows the ORIGINAL stored fitness, not the eval result. Even with
  radically different eval scenarios, renderer always shows the training fitness.
  - Flow: `nn_weights.dat` (carries fitness from nnextractor) → `nn_deserialize` →
    `genome.fitness=508K` → raw bytes copied to `evalResults.gp` → S3 → renderer
  - The eval-computed fitness is only printed to console/stc, never stored
  - Fix: update `genome.fitness` with eval result before serializing to evalResults,
    OR store eval fitness in a separate evalResults field the renderer can read
- **Bug 3 (NEW 2026-04-07): Eval mode missing rabbit speed config** — `runNNEvaluation()`
  in `src/autoc.cc` (around L750-787) builds `EvalData` without setting `evalData.rabbitSpeedConfig`.
  Default is `{nominal=16.0, sigma=0.0}` from `RabbitSpeedConfig::defaultConfig()` —
  i.e., **constant 16 m/s rabbit**, NOT the configured 13±2 m/s from autoc-eval.ini.
  - Training path (L939-940 and L1028-1029) correctly sets `evalData.rabbitSpeedConfig = gRabbitSpeedConfig`.
  - Symptom: eval fitness on saved gen-N weights is "slightly different" from the training
    fitness reported at gen N — same NN, same scenarios, but different rabbit trajectories
    because rabbit speed profile is wrong.
  - Fix: add `evalData.rabbitSpeedConfig = gRabbitSpeedConfig;` (and `* computeVariationScale()`
    for consistency, though both return 1.0) at line ~756 in eval path.
  - Discovered while trying to reproduce 022 betterz2 gen 400 fitness in eval mode.

### [DONE 2026-04-07] Refactor Duplicate Fitness Constants
- ✅ Resolved by 022 conical-surface refactor. The old DISTANCE_TARGET / ATTITUDE_NORM
  constants no longer exist. fitness_computer.h is the single source of truth for
  the conical scoring surface (FitDistScaleBehind, FitDistScaleAhead, FitConeAngleDeg).

### [DEFERRED] Streak Threshold Ramp (022 T024)
- Originally proposed in 022: ramp `FitStreakThreshold` from min (e.g., 0.1, ~22m
  forgiving) to max (0.5, ~7m demanding) over training via `computeVariationScale()`.
- Goal: early generations get streak credit for "getting closer," late generations
  demand tight tracking.
- **Verdict 2026-04-07**: betterz2 (V4 conical, 400 gens) converged strongly without
  this. Not needed for current curriculum. Park as a potential tool if a future
  curriculum widens or training plateaus on a harder task.
- Implementation (when needed):
  - Add `FitStreakThresholdMin` / `FitStreakThresholdMax` to autoc.ini, config.h, config.cc
  - In `computeScenarioScores()`, interpolate threshold = min + (max-min) * computeVariationScale()
  - FitnessComputer constructor takes the interpolated threshold

### [030 v1 — UNPARKED 2026-05-08] Worker-side scenario priming + (deferred V2) LRU demand-fetch

- **Status**: UNPARKED. Forced into v1 path by 030 tracker-mode working-set explosion. Two-stage scope: V1 = static priming at worker init (this milestone); V2 = LRU + autoc-callback demand-fetch (deferred until libraries evolve over time).
- **Trigger fired 2026-05-08**: tracker training run (`autoc-tracker.ini`, pop=5000, 294 source scenarios, 20 threads) OOM'd on a 128 GB machine **before gen 1 finished**. Root cause: each per-individual `EvalData` now carries a deep copy of the full source library in `EvalData.sourceList` (`src/autoc.cc:1015-1024`) — ~8.5 MB per `EvalData` (294 SourceScenarioTrajectory × ~600 ticks × 48 B `SourceTickSample`). The unbounded `ThreadPool::enqueue` (`include/autoc/util/threadpool.h:120`) queues all 5000 individuals' EvalData up front, each pinned alive by a `shared_ptr` capture in the lambda — 5000 × 8.5 MB ≈ **42 GB of sourceList copies in flight**, plus push_back deep-copy churn on the autoc side and serialize/deserialize cost on every RPC. M1 pathgen runs (same pop, same threads, e.g. `gen9540.dmp` at gen 460 in the latest pop=5000 minisim run, 42 MB on disk) fit comfortably in 128 GB because pathgen `EvalData` is ~50–100 KB per individual; tracker mode is 80–170× heavier per-eval **even though the output dmp is the same size** — the explosion lives in the per-eval RPC, not the artifact.
- **Why it can't wait**: training stalls on this exact config until either V1 lands or pop is dropped (which would invalidate scale comparison vs M1). V1 is the smallest unblock — the 294-scenario library is deterministic and fits in worker RAM 1×, not 5000×.

#### V1 — Init-time priming (FORCED-NOW, this milestone)
- **Approach**: new `WorkerInit` RPC sent once per worker right after the worker's TCP accept (worker fork is in `threadpool.h:60-79`). Carries everything currently scenario-invariant in `EvalData`: `sourceList`, `cameraConfig`, `beaconLeftConfig`/`Right`, `airframeProxy`, `flightArena`, `rabbitSpeedConfig`, `crashHullRadius`, `trailDistance`. Worker caches in process-local state.
- Per-eval `EvalData` shrinks to: NN bytes (`gp` + `gpHash`), `controllerType`, `mode`, `isEliteReeval`, `pathList` (still per-gen), `scenarioList` (still per-gen), `pCrashThisGen` (gen-varying), `trackerSourcePreRollTicks`, plus scenario index references into the worker-resident library — drops the ~8.5 MB sourceList payload entirely. ~50–100 KB per `EvalData`, matching pathgen's footprint.
- Memory footprint: 5000 × 8.5 MB → 20 × 8.5 MB = **160 MB instead of 42 GB**. Wire bandwidth drops 80×.
- **Scope: BOTH minisim AND crrcsim mod_inputdev need the new RPC.** Minisim path stays alive through 030 v1 (per the [Minisim retire / stub / remove] entry above) and is currently the only working tracker path (M11.preA crrcsim integration not yet smoke-green). Both ends need symmetric `WorkerInit` handling. Files: `include/autoc/rpc/protocol.h` (new RPC type + `EvalData` field migration), `src/autoc.cc` (`buildEvalData` + threadpool dispatch), `tools/minisim.cc` (worker-side cache + dispatch shim), `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` + `CrrcsimTrackerHelper`.
- **Determinism**: scenarios are deterministic from joint-PRNG seeds (per [project_variation_design_principles.md](../.claude/projects/-home-gmcnutt-autoc/memory/project_variation_design_principles.md)). Static priming is correct as long as the library is stable across the run; library-evolves-over-time is the V2 trigger.
- **Sizing in the limit**: this is "send everything except the NN once at init." Same shape as the [BACKLOG] AutocConfig auto-print — anything that doesn't change per-eval should not be in per-eval RPC.
- **Out of scope for V1**: variable-rate / real-flight source robustness ([031 CANDIDATE] above) — V1 priming assumes a static deterministic library. Library curation / mirror-pairing ([031 CANDIDATE] above) — same.

#### V2 — LRU demand-fetch with autoc callback (DEFERRED)
- **When V1 isn't enough**: when the scenario library itself evolves over time — e.g., [project_library_based_training.md](../.claude/projects/-home-gmcnutt-autoc/memory/project_library_based_training.md) (winners-promote-into-library), or curriculum schedules that swap library subsets per phase, or scenarios > worker RAM (e.g., 10K+ entries in a future-extreme config). In those regimes "ship everything once at init" no longer holds.
- **Approach**: workers maintain an LRU cache keyed by `(scenarioId, libraryEpoch)`. On `EvalData` receive, worker looks up referenced scenarios in cache; on miss, fires a synchronous callback RPC to autoc requesting the missing entries; autoc serves from `gSourceTrajectoryList` (or its evolved equivalent). LRU evicts least-recent on capacity pressure. Sketch in `specs/archive/ZZZ-SCALEUP.md:1045-1055` (LocalScenarioCache prior art) — same pattern, smaller-scope first.
- **Why it stays deferred until needed**: LRU + bidirectional callback adds protocol complexity (a worker can now initiate an RPC to autoc, which means autoc grows a worker-init request handler thread or non-blocking reactor). V1 buys us the working set; V2's complexity is only justified when the library-evolves regime actually shows up.
- **Trigger to unpark V2**: first concrete milestone that requires a library that mutates within a run. Most likely: library-based-training milestone, or a 031 source-evolution feature.

- **Closes existing [NEXT] entry**: prior 3-bullet placeholder ("Send scenario table once at generation start, cache in crrcsim") subsumed and retired by V1 above.

### [NEXT] Trim EvalResults on the return path — score-only for non-elite
- **Sibling to** the [030 v1 — UNPARKED] "Worker-side scenario priming" entry above. Priming reduced **autoc → sim** transport; this entry reduces **sim → autoc** transport. Same family (cut wire size for the 5000-individual-per-gen hot path), different direction.
- **Surfaced 2026-05-08 priming smoke**: with priming wired (queue spike eliminated), autoc-side residual working set still sits at ~23–26 GB at pop=5000 / 294-scenario tracker training. Operator observation: the sim→autoc EvalResults are heavy on the way back too — per-tick `aircraftStateList` + `cameraViewList` + `targetTrajectoryList` for 294 scenarios × ~600 ticks ≈ 76+ MB serialized per individual. The non-elite training path collapses all of that into ~24 bytes per scenario via `computeScenarioScores` and discards the rest.
- **Approach**: add an `EvalResultsLite` (or equivalent) carrying only `crashReasonList`, `arenaEgressCount`, `hullStrikeCount`, scenarioList, and a pre-computed `std::vector<ScenarioScore>` (worker-side `computeScenarioScores`). Send `EvalResultsLite` for training evals; full `EvalResults` only when `isEliteReeval` (the elite is the only one whose per-tick state is dumped to S3). Worker already branches on `isEliteReeval` for `debugSamples` / `physicsTrace` — extend that branch to also gate per-tick state.
- **Memory benefit**: peak transient receiveRPC path drops from ~76 MB × 20 concurrent workers to ~tens of KB. ~3 GB of resident `WorkerContext.evalResults` collapses to ~MB. Combined with the `receiveRPC` triple-copy (vector + string + istringstream + deserialized object) the transient saving could be 8–10 GB on autoc. Won't fix all of the 25 GB — heap fragmentation, AWS SDK buffering, and S3 PutObject upload buffering for elite dmps are separate — but should take a meaningful bite out of it.
- **Scope: BOTH minisim AND crrcsim mod_inputdev** for the same reason as priming (minisim is still the only working tracker path).
- **Out of scope for v1**: changing dmp on-disk format. Elite-reeval still produces the full v=2 EvalResults dump as today; we're trimming the wire payload, not the artifact.
- **Trigger**: after priming validates at production scale and operator wants to push autoc-side memory further down. Naturally pairs with the V2 LRU demand-fetch direction in the priming entry — if we're refactoring the protocol anyway, do both directions in the same wire-format pass.

### [DEFERRED] Output Cleanup
- OutputDir config key, auto-created run subdirectory, clean eval prefix naming

### [DEFERRED] Training Run Archive Policy (moved from 024 T204)
- One-page `docs/TRAINING_RUN_ARCHIVE.md` documenting: naming scheme for
  training runs, retention policy (what to keep, what to discard), and a
  record of the current canonical run (`test4-data.dat`, now cadence7).
- Current ad-hoc: training runs live in `/home/gmcnutt/autoc/data.dat` +
  `logs/autoc-<feature>-<name>.log` + S3 bucket artifacts. No formal
  rotation; big `.dat` files accumulate.
- Write when the accumulated `.dat` size becomes a problem or when a
  new run needs canonical-reference status for post-flight analysis.

### [NEXT] Per-axis / per-path analysis from S3 .dmp instead of data.dat
- **Problem**: `data.dat` is overwritten at the start of every training run.
  The per-axis aggressiveness time-series + per-path roll/pitch rate analysis
  in `specs/029-no-future-arch/plot_per_axis_time_series.py` reads `data.dat`
  directly, so as soon as the next training launches, the prior run's
  analysis is no longer reproducible. Specific instance: pastonly2 finished
  2026-05-02, pastonly3 launched same day → pastonly2's data.dat lost. We
  cannot retroactively re-render pastonly2's per-path roll/pitch RATE chart
  with the new normalization (introduced 2026-05-02 in this same session).
- **Long-term solution**: extend the per-axis analysis tooling to read
  per-tick aircraft state from the S3 `.dmp` files (cereal-serialized
  `EvalResults` containing `aircraftStateList[scenario][tick]`). Each gen's
  best individual is dumped to S3 (per `src/autoc.cc:1119-1146`), so
  per-tick quaternion + outputs are recoverable for any prior run as long
  as S3 retention holds.
- **Implementation sketch**:
  - New tool: `tools/aircraft_state_extractor.cc` (or extend `nnextractor`).
  - CLI: `--source-run <S3-prefix> [--gens 0-800] --out per-tick.csv`
  - Iterates all gen .dmp files for the run, deserializes `EvalResults`,
    flattens `aircraftStateList[scenario][tick]` into a CSV with columns
    matching today's `data.dat` (Scn, Pth/Wnd:Step, qw qx qy qz, outPt
    outRl outTh, etc.).
  - Existing `plot_per_axis_time_series.py` reads either source (data.dat
    OR extracted CSV) — minor parser tweak.
- **Why this enables**: retroactive per-path metric refinement (like the rate
  normalization we just added), cross-run direct comparisons (pastonly2 vs
  pastonly3 vs more-rnn3 on the same metric, computed identically), and
  reproducibility of analysis when source data.dat has been overwritten.
- **Trigger**: next time we want to retroactively analyze a prior run with a
  metric we didn't compute at the time. Likely fires when 030 needs to
  cross-compare pastonly2 / pastonly3 / 025 controllers post-hoc, or when
  a flight-test outcome motivates re-checking some prior controller's
  per-path behavior.
- **Out of scope for v1**: extracting non-best individuals (only the gen's
  elite is dumped to S3); extracting per-tick PidInternals (not in
  EvalResults today — would need another schema add); replicating data.dat
  byte-for-byte (just the columns the per-axis analysis needs).

### [NEXT] Snapshot / resume training mid-run + adaptive gen-budget
- **Problem**: Path A config (pop 5000, gens 800, NNSigmaFloor 0.05) reliably
  hits sigma-floor around gen 500-550 across recent runs (more-rnn3, pastonly2,
  pastonly3). Once at floor, fitness drift over the remaining 250-300 gens is
  small (a few percent) — pure-exploitation refinement that could either be
  (a) skipped (terminating earlier saves ~24h compute per run) or (b) extended
  beyond 800 if the late-plateau is still moving (rare, but happens).
- **Today's blocker**: no checkpoint/resume mechanism. Each training run starts
  from gen 0; can't re-launch from gen 600 to extend, and can't kill at gen
  600 with confidence that "this is already the answer." So operator
  uniformly trains to gen 800 to confirm plateau is real even when the
  middle-third would have been definitive — wasted compute.
- **Long-term solution** (two parts):
  1. **Periodic checkpoint dump** — every N gens (say N=50), serialize the
     full population (NNGenome[]) to disk + log gen state. Same `cereal`
     stack as S3 dumps, just population-wide instead of best-only. ~1-2 GB
     per checkpoint at pop=5000 × 1923 weights — manageable rotation policy.
  2. **`--resume <checkpoint.dat>`** flag on autoc — load population, set
     gen counter, continue. Identical PRNG seeding semantics (one of the
     fields in the checkpoint).
- **Adaptive gen budget** (smaller follow-on): given checkpointing exists,
  add a runtime termination heuristic — e.g., "if last-N-gens fitness slope
  < threshold AND sigma at floor for ≥ M gens, stop." Operator-overridable.
  Saves the wasted late-plateau gens automatically.
- **Trigger**: when a 029-class run produces a clearly-converged controller
  before gen 800 AND the operator wants to run a *follow-up experiment*
  (variation in 025, derived features in 029-T061, etc.) and would benefit
  from "warm-start from the converged checkpoint" rather than re-training
  from gen 0. Concrete instance: pastonly3 may hit clean plateau by gen
  600 of 800, but operator continued because no resume mechanism exists.
- **Out of scope for v1**: distributed checkpointing across worker nodes
  (single-machine training only); checkpoint-format versioning (tracker-
  mode-aware); selective restart (e.g., load weights but reset variation
  ramp).

### [NEXT] Make pathgen.h Portable for Embedded
- Single pathgen.h that works on both desktop and embedded
- Current state: embedded_pathgen_selector.h is a manual clone of desktop pathgen.cc
  with different helpers — changes don't propagate (e.g. FortyFiveDegreeAngledLoop
  still at 0.5 rad step vs desktop 0.05 rad after fix 45df719)
- Immediate fix: update embedded FortyFiveDegreeAngledLoop to 0.05 rad step
- Long-term: refactor so both desktop and embedded use the same path generation code

---

## Embedded / Hardware

### [NEXT] USB Log Download from Xiao
- BLE log download is slow (BLE bandwidth-limited) AND unreliable in the
  field — drops/stalls observed even when no other 2.4 GHz interference is
  present. Pre-flight log retrieval blocks turnaround between flights.
- Need: USB-CDC (or USB Mass Storage if simpler) path to dump xiao flash
  log files. Xiao SAMD/nRF supports USB-CDC out of the box.
- Implementation sketch: enumerate flight log files on flash, expose a
  serial menu over USB (e.g., `LIST`, `DUMP <name>`, `ERASE`) — same
  protocol surface as BLE so the host-side download tool can share most
  code. Should not regress BLE path; both are useful (USB at the bench,
  BLE for opportunistic extracts).
- Hold the BLE-reliability investigation as a separate orthogonal item;
  USB download is the practical workaround.

### [NEXT] Export RC Commands to Xiao Log
- Log RC commands throughout entire flight for full playback visualization
- Location: xiao/src/msplink.cpp

### [ACTIVE → 021] Xiao Onboard IMU as AHRS Cross-Check
- Moved to [specs/021-xiao-ahrs-crosscheck/spec.md](021-xiao-ahrs-crosscheck/spec.md)
- P0 blocker: Mar 27 flight showed uncontrolled rotation — cannot tell if AHRS or gain mismatch
- LSM6DS3 + Madgwick on Xiao, log alongside INAV quat, compare post-flight

### [DEFERRED] GPS Dropout Handling During NN Control
- What happens when GPS drops out during NN-active control? Position freezes, NN sees stale data
- Need: xiao detects stale position (no update for N ms) and either disables autoc or flags it
- Also consider MSP communication loss detection and safe fallback

### [DEFERRED] Xiao Safety Checks Pre-Arm
- Ensure mode flip is safe: RC failsafe, RC disarm, hold/RTH should disarm co-processor

### [PRE-FLIGHT / 023] Failsafe Refinement and Bench Verification
- **Blocker for next flight test after 023 NN training lands.** Not a 023 spec
  deliverable, but must be addressed before flying the new NN policy. Source:
  `docs/failsafe-behavior-audit.md` (2026-04-08 audit, see also
  `docs/inav-signal-path-audit.md`).
- **Current state**: `xiao/inav-hb1.cfg:1419-1432` has `failsafe_procedure = DROP`.
  Acceptable for the current foamboard platform (~100g, minimal ground-impact
  risk) and has been running this way for a while. User's earlier aircraft used
  "launch, fly out, reset home, orbit home at 50m" — that is the aspirational
  target procedure for larger/future platforms.
- **Four specific items from the audit:**
  1. **Failsafe has NEVER been exercised in real flight** on this platform
     (audit confirmed no failsafe events in any 2026-04-07 flight log).
     DROP path has never actually fired during an autoc span. **Action:**
     bench test with physical SBUS disconnect during an active autoc span,
     verify the full chain works: INAV trips → DROP disarms → xiao detects
     FAILSAFE bit via `MSP2_INAV_LOCAL_STATE` → xiao calls `stopAutoc()` →
     recovery path (disarm/rearm OR stick-wiggle above `failsafe_stick_threshold = 50`).
  2. **SBUS receiver failsafe value behavior is unverified**. If the receiver
     is configured to "hold last values" on signal loss AND AUX1 was HIGH
     (armed) at the moment of loss, AUX1 stays HIGH after loss, keeping
     `BOXARM` active. The only thing that actually disarms is the main
     failsafe state machine calling `disarm(DISARM_FAILSAFE)` at
     `flight/failsafe.c:587`. **Action:** verify the receiver's failsafe
     channel config on the bench. Document the observed AUX1 behavior
     during signal loss.
  3. **Brittle recovery mechanics**. Depending on INAV settings, recovery
     from failsafe requires either (a) disarm + rearm (possibly in-flight
     if `nav_disarm_on_landing` etc. is off), or (b) sticks above
     `failsafe_stick_threshold = 50` clearing failsafe without disarm.
     Exact current behavior is undocumented for the hb1 platform. **Action:**
     document the recovery behavior, test both paths on the bench.
  4. **Aspirational upgrade path** — for larger/future aircraft or more
     aggressive flight envelopes, switch `failsafe_procedure` from DROP to
     LAND or RTH. Would require: `failsafe_fw_roll_angle` / `pitch_angle` /
     `yaw_rate` tuning for glide, `failsafe_min_distance` semantics, and
     re-auditing the MSP override + failsafe state machine interaction per
     C4 in the audit doc. Not for 023.
- **Why not in 023**: failsafe mechanism is orthogonal to NN representation +
  training work. But the NN policy being harder to pilot-debug raises the
  stakes on reliable control handoff, so failsafe refinement is pre-flight
  prerequisite work, not "nice to have."

### [NEXT / 023 follow-up] INAV Fork Patch: mspOverrideInit First-Frame Bug (C1)
- **Source**: `docs/failsafe-behavior-audit.md` §Latent Bug Assessment → C1.
- **Bug**: Even with `failsafe_recovery_delay = 0`, MSPRCOVERRIDE engage still
  pays a 200 ms floor because `mspOverrideCalculateChannels()` runs at 50 Hz
  from boot and pre-connection ticks update `validRxDataFailedAt = millis()`
  every tick. The first valid MSP frame sees a tiny
  `(validRxDataReceivedAt - validRxDataFailedAt)` difference and must wait
  the full `rxDataRecoveryPeriod` before `rxFailsafe` clears.
- **Fix**: in `src/main/rx/msp_override.c:mspOverrideInit()`, initialize
  `validRxDataReceivedAt = millis() + rxDataRecoveryPeriod`. OR: in
  `mspOverrideCalculateChannels()`, on the first
  `rxSignalReceived = true` transition, unconditionally set
  `rxFailsafe = false`. Either approach makes MSPRCOVERRIDE engage instant
  once xiao starts streaming frames.
- **Why not in 023**: INAV source change is out of scope for a feature
  focused on autoc NN representation + training. The sim already models
  the 750 ms delay via `EngageDelayMs` in Change 1b, and real flights
  work around it (pilot aligns, releases, throws the switch, aircraft
  coasts briefly on momentum). This is a correctness cleanup for a
  future release, not a 023 blocker.
- **Risk assessment**: low. The fix is local to `msp_override.c`, only
  affects the initial boot state, does not touch the main failsafe state
  machine, and does not change behavior for any scenario other than
  "first MSPRCOVERRIDE engage after boot". Bench verification is
  straightforward (measure engage delay before/after).

### [DEFERRED] Xiao-Side Independent RC Dropout Detection
- **Source**: `docs/failsafe-behavior-audit.md`.
- Currently xiao only detects failsafe via the `MSP_MODE_FAILSAFE` bit in
  `MSP2_INAV_LOCAL_STATE`. This means xiao's `stopAutoc("failsafe")` only
  fires AFTER INAV's main failsafe has already tripped. xiao has no
  independent way to detect RC dropout before INAV acknowledges it.
- Defense in depth: xiao could track MSP round-trip latency and pause
  autoc if a threshold is exceeded, OR directly monitor SBUS health via
  a separate serial channel.
- Not urgent for 023 — the current coupling works well enough for DROP.

### [DEFERRED] Speed Up Logfile Download
- BLE download may be over-bucketed from prior troubleshooting

---

## Visualization

### [NEXT] Renderer Playback Enhancements — per-tick scrub + streak/multiplier overlay
- **Per-tick scrub controls**: pause / step-forward-one-tick / step-backward-one-tick.
  Today the renderer plays a continuous timeline; for diagnostic work you want to
  freeze on a specific tick and walk one step at a time to see exactly when the NN
  does what.
- **In-streak counter overlay**: per-tick `streakCount` displayed alongside
  the rendered aircraft, so you can see "is this a streak tick? for how long?"
- **Points multiplier overlay**: per-tick `multiplier` (= 1 + (streakMultMax-1) ·
  streakCount/streakStepsToMax), the same value `FitnessComputer::applyStreak`
  uses to amplify stepPoints during fitness aggregation.
- **Data path**: per-tick streak/multiplier are NOT in `EvalResults` today
  (only scenario-aggregates `maxStreak` / `totalStreakSteps` / `maxMultiplier`
  on `ScenarioScore`). Two implementation options:
  - **Re-synthesize in renderer/script**: replay `FitnessComputer::applyStreak`
    against captured `aircraftStateList` + `pathList` + autoc.ini config knobs
    (`fitStreakThreshold`, `fitStreakRampSec`, `fitStreakMultiplierMax`).
    No schema change. Drift risk: renderer math must match training math.
  - **Capture per-tick on AircraftState**: add 2 fields (streakCount int +
    multiplier float) per tick. Ground truth, no drift, ~negligible dump size
    increase. Cereal schema bump (per project policy: no versioning shim, old
    dumps unloadable post-bump).
- Recommend the schema-bump path for permanent renderer feature; recompute
  is fine for one-off Python diagnostics in the meantime.
- **Files likely involved**: `tools/renderer.cc` (UI controls + overlay),
  `include/autoc/eval/aircraft_state.h` + cereal serialization (if capturing),
  `src/eval/fitness_computer.cc` (export the per-step multiplier alongside the
  streak update — it's already computed, just discarded).

### [DEFERRED 2026-05-08] Renderer 1st-person camera-POV view (was 030 T054)

- **Trigger**: when operator wants to *deeply inspect* one scenario from the chase craft's POV — full main-viewport render through the camera's pose+FOV. Beacons appear as colored points at projected `(screen_x, screen_y)` at full screen scale (vs the small mini-panel).
- **Why deferred from 030 v1**: the M9b mini-panel HUD (T055/commit 58cf328) covers the load-bearing "what does the NN see?" question for smoke-test signal-or-not. Full 1st-person view is research-grade analytics — useful when diagnosing a specific failure (e.g., "did the chase fly into a sentinel-rich scenario?") but not required to declare v1 done.
- **Scope**: second VTK renderer (or split-screen overlay) using chase camera pose from `cameraViewList[i][k].camera_pose_world_pos` + orientation as VTK camera; FOV deg → projection matrix; render full scene (target craft + arena + plane). Beacon dots already projected by M5 — render at fullscreen coords matching mini-panel.
- **Files likely**: `tools/renderer.cc` only. Roughly 150-300 LOC.

### [DEFERRED 2026-05-08] Renderer scrub controls (was 030 T057, rolled-in from earlier 028 entry)

- **Trigger**: when operator needs frame-by-frame inspection — pause / step-forward-one-tick / step-backward-one-tick. Animation already auto-pauses on left-click for camera interaction; full scrub adds tick-by-tick navigation.
- **Why deferred from 030 v1**: not load-bearing for smoke-test signal-or-not. Current pause-on-click + scrolling through generations covers the typical diagnostic workflow.
- See also the older "[NEXT] Renderer Playback Enhancements" entry below for scope notes.

### [DEFERRED 2026-05-08] Renderer streak/multiplier overlay (was 030 T058, rolled-in from earlier 028 entry)

- **Trigger**: when operator wants per-tick `streakCount` + `multiplier` shown alongside the rendered aircraft. Today `EvalResults` carries scenario-aggregates only.
- **Why deferred from 030 v1**: optional; smoke-test fitness signal reads from the gen-level fitness curve directly via `data.stc` + `plot_evolution_progress.py`.
- See "[NEXT] Renderer Playback Enhancements" entry below for the schema-bump-vs-resynthesize tradeoff.

### [DEFERRED] Blackbox Rendering Improvements
- Select path + blackbox log for comparisons, FPV mode

### [DEFERRED] CRRCSim Display Dependency
- CRRCSim requires valid DISPLAY even in headless mode

### [DEFERRED] Clean CRRCSim Shutdown
- Polling loop for keepalive; clean exit when autoc exits

---

## Code Cleanup

### [NEXT] crrcsim mod_inputdev — link autoc_common instead of cherry-picking sources
- **Current**: [crrcsim/src/mod_inputdev/CMakeLists.txt:21-23](../crrcsim/src/mod_inputdev/CMakeLists.txt#L21-L23) compiles three autoc-side files directly into `mod_inputdev.a`:
  ```
  ${CMAKE_SOURCE_DIR}/src/nn/evaluator.cc
  ${CMAKE_SOURCE_DIR}/src/nn/serialization.cc
  ${CMAKE_SOURCE_DIR}/src/eval/sensor_math.cc
  ```
- **Problem**: any new file in `src/nn/` or `src/eval/` that an above .cc references at link time silently breaks the crrcsim build at link, with `undefined reference` errors. The 028 telemetry.cc landing tripped this — `evaluator.cc` called `RecurrentTelemetry::activation_ratio()` which lived in `telemetry.cc`, and crrcsim's mod_inputdev didn't pick up telemetry.cc. Worked around by inlining the method in `evaluator.h`, but the architectural fragility remains.
- **Fix**: have `mod_inputdev` link against `autoc_common` (which is built by the parent autoc CMakeLists). crrcsim already builds via `add_subdirectory(crrcsim)` per Constitution Principle IV (Unified Build), so `autoc_common` is in scope. Remove the cherry-pick lines, add `target_link_libraries(mod_inputdev autoc_common)` (or thread it through the crrcsim link chain to wherever the final crrcsim binary links).
- **Risk**: low. autoc_common pulls in cereal/inih/Eigen/etc., all of which crrcsim already depends on transitively (mod_inputdev's evaluator.cc compile already needs them). Possible duplicate-symbol issues if any crrcsim file also defines something autoc_common does — none observed but worth checking on first build.
- **Alternate workaround**: keep the cherry-pick pattern but add a `mod_inputdev` build-time test that asserts no undefined references in the link target. Less clean but lower-risk.
- Triggered by: 028 telemetry signals (Apr 2026). Will re-trigger on the next autoc-side .cc addition that evaluator.cc transitively references at link.

### [DEFERRED] Memory Leak Investigation
- Small memory leak exists in autoc

### [DEFERRED] Cross-Platform Verification
- Train on aarch64, pull repo on x86
- Build and run renderer/nnextractor/eval against aarch64 S3 objects
- Validates cereal binary portability end-to-end

---

## Completed / Superseded

- ~~Sigma Floor~~ — done (015 Phase 1)
- ~~Curriculum Ramp~~ — done (015 Phase 2, wind scenario ramp)
- ~~Fitness Decomposition~~ — done (015 Phase 2, per-scenario scores)
- ~~Pareto Multi-Objective~~ — superseded by epsilon-lexicase (015 Phase 3)
- ~~Demetic Mode Elite~~ — superseded by lexicase selection
- ~~Wind Speed Variation~~ — done (WindScenarios with varied seeds)
- ~~Immelman Path Fix~~ — done (T121a, progressiveDistance split-S fixed)
- ~~Float Precision Non-Determinism~~ — done (integer timestamps in Path)
- ~~GP Eval Node Test Coverage~~ — superseded (GP removed, NN evaluator has tests)
- ~~Fitness Output Formatting~~ — superseded (aggregateRawFitness is canonical)
- ~~Training Record Consistency~~ — done (S3 upload in eval mode, consistent keys)
- ~~Consolidate PRNG~~ — done (rng.h covers all sites)
- ~~Upper-Level Intercept Director~~ — superseded by entry variation training
- ~~Future State Predictor NN~~ — superseded by temporal history + forecast inputs
- ~~Behavioral Cloning Bootstrap~~ — not needed, direct NN training working
