# AutoC Backlog

**Last Updated**: 2026-07-10

> **Routing (2026-07-10, 038 wrap)**: 038 closed — see [038 wrap](038-accurate-m2/wrap.md). Next feature
> is **039 (xiao back in shape)**; the M2-depth items below (predictor elevation, streak-proxy input,
> camera work) are **040** candidates.

---

## 039 deferrals

### [039 wrap — GATED on n>1 flight articles, set 2026-07-13] Pitch marginal-stability levers — choose after the next articles fly

First 20 Hz flight (wrap.md §3): the flight article is ~neutral CG; pitch oscillation under NN
control is **closed-loop plant sensitivity, not command roughness** — pitch-rate RMS by regime:
INAV acro untuned PID 24 °/s ("on a rail" at launch), pilot MANUAL 50 ("on edge"), NN-direct 141
(NN spans run `MANUAL|MSPRCOVERRIDE` — raw surfaces, no inner loop). Command-domain per-axis
reports (t5-class dCtrl) are blind to this by construction; the NN's pitch commands were the calm
axis. **Operator decision 2026-07-13: NO sim recalibration from this n=1 airframe** — the towed
streamer alone is a large effect; new flight articles are being built.

Levers to choose among when n>1 (not exclusive):
- **Forward CG ballast** on the article (hardware, cheapest, restores the static margin the sim
  plant already has).
- **Sim match**: reduce hb1_streamer static margin / pitch damping (Cm_alpha/Cm_q class) to the
  measured article; acceptance test = reproduce ~141 °/s pitch RMS vs ~95 roll replaying the
  recorded 20 Hz command stream (`flight-results/flight-20260713`, `specs/039-…/flight_report.py`).
- **Static-margin craft-variation axis** (034-style static-per-scenario) — trains robustness
  across CG placement instead of tuning to one article.
- **Architectural**: NN over INAV's rate loop (acro+override instead of manual+override) — the
  untuned PID already damps this plant to 24 °/s. Changes the action space (setpoints, not
  surfaces); sim must match; a feature of its own, NOT a tweak.

Related standing data from the same flight: routine ±7–8 g z-loads (peak +9.6 g in acro recovery
outside spans) + throttle 86% saturated → energy/smoothness objective (035-line) material.

**Data point 2 (flight 2026-07-20, `flight-results/flight-20260720/`)**: same config except the
tail boom re-seated (it had slid ~3/4" aft on broken dowel ends → 07-13 flew with CG further aft
than intended). Cleanest possible A/B: the NN's pitch command stream was statistically identical
across both flights (⟨|u|⟩ 0.374/0.376, dCtrl 0.315/0.313). Path-matched (paths 0/2/3) NN-direct
pitch-rate RMS 138.6 → 128.6 °/s (−7%), pitch/roll ratio 1.39 → 1.25 (−10%, the wind-resistant
number — roll went UP on ~8% hotter roll commands). Acro-PID floor unchanged (~28–37 °/s), so the
plant still rings at ~4.5× damped baseline. Read: aft-CG explains maybe a quarter of the excess
pitch motion, NOT the root cause; the too-steep wing leading edge on this article (known pitch
destabilizer on prior craft) + streamer remain in the confound pool. Reinforces waiting for the
new articles. Also: **new load record +11.2 g / −8.4 g** (span 2) on identical policy — loads
creeping up flight-over-flight; inspect airframe between flights.

### [040-fed, filed 2026-07-28] FDM propeller is the wrong part — 5.0×4.5 modelled vs 5.5×4 flown; fix only as part of a re-tune

Found during the 040 Stage-A airframe-fidelity check
([airframe-fidelity.md](040-camera-redo/airframe-fidelity.md), T004). **Everything else agrees** — mass
(0.515 vs 520 g), span and chord (exact), wing area — so this is the sole plant-affecting discrepancy.

| | modelled | flown |
|---|---|---|
| `crrcsim/models/hb1_streamer.xml` `<propeller D H>` | **0.127 / 0.114 m** (5.0 × 4.5) | **0.1397 / 0.1016 m** (5.5 × 4) |
| Δ | — | **+10% diameter, +21% disc area, −11% pitch** |

The modelled value is the airframe vendor's *stock recommended* prop; the flown one is the Master Airscrew
GF 5.5×4 confirmed by photo (Windsor Propeller). Both are legitimate — they are simply different props.

**⚠️ Do NOT fix this in isolation.** `hb1_streamer.xml` is a **stability-derivative model tuned against
observed flight** across 021/023 *with these propeller values already in place* — `Cl_da` alone was revised
four times to match measured roll rates. Whatever thrust error the wrong prop introduced has been silently
absorbed by the tuning. Correcting `D`/`H` alone would break that fit and could make the model **less**
faithful, not more. It is only meaningful as part of a re-tune against flight data.

**Home**: the flight-model fidelity re-tune — the same work as the pitch marginal-stability levers above,
which is already gated on **n>1 flight articles**. Pairs naturally with the deferred "less pitch
aggressiveness" direction (operator 2026-07-28) and with the static-margin craft-variation axis.

**Also worth carrying**: eCalc's wing-area entry of 17.42 dm² (270 in²) implies a ~9″ mean chord against a
measured 7″; the simulator's 0.136 m² is correct. That is a third-party data-entry error, and it
contaminates the **airspeed** axis of any eCalc partial-load table — fit throttle→RPM, never airspeed→RPM.

**Trigger**: the flight-model re-tune feature (n>1 articles), or any 040-successor that changes the plant
and must therefore rebake M1 anyway.

### [039 — BACKLOG, set 2026-07-10] Redefine flight boundaries generally for open flying — not the training cylinder

**Operator (during the 039 arena-placement clarifications)**: the training cylinder (R=80 m, ±K
band, engage-centered) is a *training construct* the 039 firmware carries only so the 038 NN's
arena inputs mean what they meant in training — its limits are **safety-only** this phase, and 039
deliberately dropped even the min-elevation clamp (arm-too-low = operator error). Once we get to
**open flying** (beyond the park test field; M2-era chasing of a real target), the boundary concept
needs a general redefinition, NOT an extrapolation of the training cylinder:

- **Geofence based on where we fly, not where we arm/engage** (the 039 spec's out-of-scope note) —
  a boundary owned by the operation/airspace, decoupled from the NN's per-engage arena inputs.
- Open questions to scope then: boundary shape/representation (cylinder vs polygon vs
  terrain/altitude-aware), who enforces it (NN input pressure vs supervisor/failsafe layer vs
  both), how the NN's trained arena-input semantics map onto an operational geofence (retrain with
  geofence-shaped inputs vs translate at the input boundary), and the altitude reference
  (arm-relative vs AGL/MSL — the 039 park≈sea-level shortcut won't survive terrain).
- **Trigger**: first open-flying / real-target-chase planning (M2 flight era), or any flight site
  where the engage-centered ±K construct stops matching the airspace we're allowed to use.
- Links: 039 spec Clarifications (arena vertical rule, 2026-07-10) + Out of Scope (geofence note);
  camera/M2 040 items below.

---

## 039 leftovers → 040 candidates (post-wrap triage 2026-07-27)

### [040 scope note — operator 2026-07-27] 040 likely SPLITS: enhanced simulation + additional M1 flight tests, while 031 gates the camera redo

Operator direction at the 039 ledger reconciliation: **040 is likely to split** rather than go straight at
the camera redo —
1. **Enhanced simulation** — the reopened sim-fidelity thread: does the sim latency model
   (`COMPUTE_LATENCY` 30 ms + servo v2) match measured 20 Hz hardware reality; amend + retrain or not
   (the analytical core 039's T022 memo never assembled; T024 holds the amend+retrain playbook). Absorbs
   [project_sim_latency] ("amend sim latency model + retrain M1; gates the deferred local-IMU / FR-006").
2. **Additional M1 flight tests** — more flights/articles on the standing v3 log + correlate tooling;
   builds the **n>1 articles** dataset that gates the 039 pitch levers (see "pitch levers" below/in wrap),
   and keeps sim-real in the loop per [project_038_regression_gate]. Per-flight clock-anchor fit remains
   required practice.
3. Meanwhile **031's 1-pixel flight tests** verify emitter/sensor RANGES in the field — the [040 — camera
   redo] items (perception front-end + representation, further down this file) **wait on those results**
   before committing to a camera spec.

### [039 leftover] 460800 baud-raise experiment (unexercised latency lever)

039 T021 was never run — 115200 proved sufficient in flight (zero overruns, 1339 engaged ticks), so the
baud raise is an **unexercised latency lever (D7: latency, not bandwidth), not a disproven one**. Cheap
bench experiment (`xiao/src/msplink.cpp:342` + INAV side) whenever pipeline latency becomes the binding
concern — natural companion to the enhanced-simulation latency work above.

## 038 deferrals

### [038 US3 follow-on — PULLED FORWARD from 040, likely next step 2026-07-07] Spherical/equidistant projection so `span` (|gap|) means one thing everywhere on the display

Surfaced by the t6 US3 predictor run (2026-07-07). The predictor forward-models the direct observable
`beacon_pair_span`, but today's **rectilinear** sim projection (`screen = tan θ / tan(fov/2)`) makes the
same angular wingspan read **wider toward the frame edge** — so `|gap|` depends on *where the target lands
on screen*, an ego-pointing contamination in the very quantity we predict. **Direction (operator)**: switch
the sim to a **spherical/equidistant** projection (`screen ∝ θ`) applied to **all** coordinates so `|gap|`
is a clean, position-invariant angular quantity wherever the target sits; then model the *actual* chosen
lens's projection once hardware is picked. This is an **objective-clarity** move (make the predicted signal
mean one thing), not tuning — see [feedback_clear_objectives_not_tuning].

- **Why now / next**: it directly sharpens the US3 forward-model (removes ego-contamination from `span`
  before asking the net to anticipate it) and is the operator's leading candidate for the step after t6.
- **Full rationale**: recorded once at [040 — camera redo / research] **Question 3** below (pulled forward
  here) and in [specs/038-accurate-m2/outcome.md](038-accurate-m2/outcome.md) → "US3 predictor — design
  rationale".
- **Implementation scoped (2026-07-09 recon — CHEAPER than first thought)**:
  - **Blast radius correction**: NO M1 source rebake and NO format/schema break. Camera projection runs
    CHASE-side at M2 train time (`TrackerStepper` → `projectBeacon`); the M1 source dmp records world-frame
    trajectories only. `BeaconObservation` fields, input count (58), dmp schema all unchanged — this is a
    pure semantic change of one function. Old M2 elites are invalid vs the new NDC (retrain = the next run
    anyway); cross-projection comparability breaks, as expected.
  - **Math (true equidistant / f-theta)** in `projectBeacon` (`src/eval/camera_projection.cc:166-182`):
    replace `u = y/x, v = z/x` + `fovHalfTan` normalization with θ = atan2(√(y²+z²), x),
    image-plane direction (y,z)/√(y²+z²), `screen_x = θ·dir_y/(fov_h/2 rad)`, `screen_y = θ·dir_z/(fov_v/2
    rad)`; clip |screen|>1; keep the x≤0 early-out (also guards the θ→π degenerate direction) + add a
    √(y²+z²)<eps center guard (→ 0,0). CEP edge factor + int8 quantization operate on NDC — unchanged.
  - **Anchor tests survive**: FOV-edge geometries map to ±1 in BOTH projections by construction (e.g. the
    existing `tan(60°)` edge test: equidistant gives 60°/60° = 1.0 too), center stays 0 — only mid-field
    values move (30° off-axis: 0.333 rectilinear → 0.500 equidistant). Add NEW tests: mid-angle = angular
    ratio, and the span-invariance property (same angular gap center vs off-axis ⇒ ≈same NDC span).
  - **Residual anisotropy to document**: per-axis normalization (120°H/90°V) keeps ±1 = FOV edge but makes
    span scale differ H-vs-V by the fixed ratio 60/45 — a KNOWN CONSTANT (tilt-coupled), unlike the
    position-dependent tan-stretch being removed.
- **Trigger**: operator 2026-07-09 — implement immediately after t8 stops (no build while it runs), as the
  t9 config. **DONE 2026-07-09 — baking as t9.**
- **t9 early finding (2026-07-09, gen ~57) + HYBRID fallback design**: t9's evolution near-froze (elite
  unreplaced ~57 gens, pop avg climbing ~5× slower than t8; ALL sensor pipelines verified clean — no
  deg/rad/sign/range error, no NaN, span·dist ≈ 0.98 as equidistant predicts). Mechanistic read: the
  rectilinear tan-stretch we removed was **accidentally a training aid** — NDC gradient at the frame EDGE
  was ~2.3/rad under rect vs a flat 0.955/rad under equidistant, and the first skill every M2 bootstraps is
  keep-in-frame (an edge phenomenon). Removing the ego-contamination also removed the edge alarm.
  **Operator direction (2026-07-09)**: if t9 stays flat → **HYBRID**: x/y perception PLANAR (rectilinear —
  edge amplification is a steering feature, and positions then match the classic planar sensor natively),
  while **span (and the predictor target) goes spherical-only** — compute the true ANGLE between the two
  beacon rays (reconstruct rays from NDC+FOV; great-circle separation), position-invariant AND free of the
  per-axis H/V anisotropy the equidistant NDC span carries. Small delta: revert projectBeacon to rect, swap
  compute_pair_span (+ span_rate + predictor realized-span) to the ray-angle form; same call sites.
  Decision point: t9 @ gen ~150.

### [038 US3 follow-on — IMPLEMENTED + RUNNING as t7, 2026-07-08] Share the M1 source's env/craft seeds with the M2 chase (do the two craft fly the same airspace?)

**STATUS 2026-07-08 — implemented, verified, baking as t7.** Knob `TrackerChaseUseSourceScenarioSeed`
(config.h + `chaseScenarioSeedAt()` at the 3 seed sites in `src/autoc.cc` + loud-fail 1:1 guard; field-count
test 97→98). Chose the **whole-`scenarioSeed` swap** (not per-class): the wind field is seeded worker-side
from `meta.scenarioSeed` (`inputdev_autoc.cpp:618`), so sourcing wind forces whole-seed granularity — which
covers wind/thermal/gust/entry/craft/crash-hull together; NN mutation + camera stay on the M2 seed. Gates
passed: **(a) knob-off bit-identical** — eval reproduced t6 gen344 `-13382.484965` exactly (`NN_EVAL_SAME`);
**(b) seed-sharing** — t7's scenario table == t5 source's, **294/294 seeds bit-identical**, ramp-independent
(pre-scale draws); **(c) t7 determinism** — eval of t7 gen4 elite `NN_EVAL_SAME`. Run: `autoc-038-t7-m2-
shared-env` (`autoc-m2/...9223370253324834099...`, master seed 1783529939, knob=1, same t5 gen800 source as
t6). **Open**: judge t7 vs t6 on fixed-eval competence (does shared airspace lift fitness/depth off t4's
plateau); per-class isolation (e.g. wind-only) is a follow-up needing a worker-side per-class seed override.

**Hypothesis (operator, gen ~318 t6 read):** M2 tracking looks "not too bad given the far more complex path,"
but the source path is the M1 **gen800** trajectory — recorded at ~full variation-ramp (scale mult ≈ 1.0):
strongest wind/gust/thermal + craft + entry. Meanwhile the M2 **chase** draws its *own, independent*
wind/thermal/gust/craft/entry from the fresh M2 seed — so **chase and target are in different air masses and
different airframes.** Since environmental + craft effects are large fractions of craft speed, this mismatch
may be inflating tracking error unphysically (the chase is fighting wind the target never flew, in a
differently-performing craft). **Does it matter? — the whole question.**

**What's already known (partial step 1 done 2026-07-08):**
- All per-scenario variation derives from one `scenarioSeed` → `deriveClassSubSeeds()` → 5 sub-seeds, FROZEN
  slot order **wind / rabbit / entry / craft / camera** (`include/autoc/util/scenario_prng.h:101`). Camera
  slot already reserved (future).
- The **M1 source dmp records `scenarioSeed` per scenario**, and `source_dmp_loader` already loads it into
  `traj.variation` (`src/eval/source_dmp_loader.cc:138`) — but only `windVariantIndex` is consumed today. The
  source's full variation provenance is loaded-but-unused (the "infra set, not used" the operator flagged).
- Today the M2 chase's {wind, entry, craft} come from the **M2** `scenarioSeed`, independent of the source.

**Research steps:**
1. **Examine the seeds available from the M1 dmp** — confirm `scenarioList[i].scenarioSeed` (+ resolved
   craft/entry/wind draws) is present and sufficient to reconstruct the source's wind/thermal/gust/craft
   fields via `deriveClassSubSeeds`. (Mostly done above; verify craft draws + entry are fully recoverable,
   not just the index.)
2. **Determine the plumbing + per-class provenance split.** Which classes should the chase inherit from the
   source vs draw fresh from M2:
   - **wind/thermal/gust** — strongest case to SHARE (same air mass). Derive chase wind sub-seed from the
     *source* `scenarioSeed_M1`.
   - **craft** — debatable: share ⇒ chase ≈ target airframe/performance; fresh ⇒ chase is a different
     aircraft. Operator leans "reconsider" given magnitude. Decide (and make it a knob).
   - **entry** — likely stays M2 (chase's initial *relative* pose in the cone is inherently chase-specific,
     not the target's own entry).
   - **NN mutation + camera(future)** — always the fresh M2 seed (the evolving thing + M2-only perception).
   - **ALSO resolve the magnitude/scale question**: sharing the *seed* isn't enough — the source flew at its
     recorded `variationScale` (~1.0 at gen800), but the M2 chase applies its *own* ramp (≈0 early). To truly
     share the air mass the chase's wind *magnitude* must also match the source's recorded scale, not the M2
     ramp. Decide seed-only vs seed+scale.
   - Identify the exact code point where the M2 worker seeds chase variations from `meta.scenarioSeed`
     (`src/autoc.cc` worker meta assignment ~848–904 / `applyVariationScale`) — that's where the per-class
     source-vs-M2 selection lands.
3. **Do the change as an ini knob** (e.g. `TrackerShareSourceEnvSeed` / per-class flags) so it's an A/B, not
   a fork — then run a **t7 alongside t6 on the same t5 gen800 source**, sharing ON, and compare tracking
   competence (mode_progress: perception / range / blind-streak; per-axis control quality). Determinism +
   replay must hold (`project_variation_design_principles`, `feedback_replay_scope_within_build`).

**Verification (operator requirement 2026-07-08 — "prove the seeds give similar results even under ramp
scaling"):** the seed-sharing is verifiable **ramp-independently**, because `ScenarioMetadata` records every
variation draw at **raw full magnitude, pre-scale** (`include/autoc/rpc/scenario_metadata.h` — "Drawn at full
magnitude per scenarioSeed... per-eval applied magnitude is scaled by `applyVariationScale()`"); `ramp_scale`
only multiplies the APPLIED value worker-side, never the recorded draw. Acceptance test, needs no new tooling:
- `dmp-dump --meta-only` on the **t5 source** and on **t7** both already emit, per scenario, the derived
  `seeds: {wind, rabbit, entry, craft, camera}` sub-seeds + raw `wind_dir_offset_deg` + `ramp_scale`
  (`tools/dmp_dump.cc:374`). With the knob ON, assert **t7's `wind`/`entry`/`craft` sub-seeds == t5's, and
  `wind_dir_offset_deg` == t5's, per matched (path_variant, wind_variant) scenario** — bit-exact, at ANY gen,
  because these are pre-scale. Sub-seed equality ⇒ every downstream draw is identical (pure fn of sub-seed +
  sigmas). (Optionally extend `--meta-only` to also print the raw entry offsets + craft deltas for a
  belt-and-suspenders check, but sub-seed equality is the rigorous primitive.)
- **Ramp story to confirm, not fear**: applied_env = raw_draw × `ramp_scale`. The t5 source was recorded at
  its gen800 `ramp_scale` ≈ 1.0; t7's chase applied env matches the source's **at scale 1.0** (t7's late gens
  / eval) and is a *known fraction* (raw × scale) earlier. Operator accepted this ("scaling is an M2 process
  thing... the final M2 path IS ramp=1"). So: seeds prove identical *fields*; scale is a separate, inspectable
  multiplier — verify both from the one `--meta-only` dump.

**Success = an answer to "does it matter":** t7 (shared airspace) vs t6 (independent) on the fixed-eval
competence comparators. If shared-airspace lifts tracking, it's a source-of-truth fix, not tuning. Pairs
with the spherical-projection item above (both sharpen *what the tracker is actually asked to do*). Files:
`src/autoc.cc` (worker variation seeding), `src/eval/source_dmp_loader.cc`, `src/eval/tracker_stepper.cc`,
`include/autoc/util/config.h` + the tracker inis (new knob).

### [038 follow-on — US3 predictor VERDICT + re-target design, 2026-07-09] Persistence baseline: the current prediction objective is structurally worthless — re-target to blindness-bridging horizons or drop the head

**Finding (t8 data, persistence baseline added to predictor_analysis.py 2026-07-09)**: the "predict
span(t+h)=span(t)" no-change bar sits at **≈0.01 NDC** for all three horizons (+50/100/150 ms) — the head's
error is 0.4–0.7 (~50× worse), AND even a *perfect* head would add ~nothing (persistence is already below
the σ-floor: span barely moves in 150 ms at 20 Hz). Compounding it, `computeSpanPredictionError` CEP-gates
BOTH pair endpoints — **blind gaps are excluded from scoring**, i.e. the objective measures only the regime
where prediction is information-free and skips the one (reacquire-through-blindness, the documented M2
bottleneck) where a forward model would pay. Answer to the operator's keep-the-head question: **not as
posed — structurally cannot provide useful signal.**

- **Re-target design (if kept)**: (1) horizons 0.5–2 s where span/bearing autocorrelation actually decays;
  (2) score across occlusion: pairs (t visible → t+h = reacquisition) — predict reappearance
  bearing/span; persistence collapses there, information content is real, and it aims at the ~8 s
  worst-blind problem; (3) CONSUME the forecast (feed aux outputs back as inputs / into the streak-proxy) —
  t6 proved passive tie-break scoring doesn't couple to control.
- **Else**: drop the aux head at the spherical-projection retrain (topology 7→3 back, or keep the wiring
  and re-purpose the 4 slots for the re-targeted quantities).
- **t9 UPDATE (2026-07-10) — small signal confirmed; operator decision: ELEVATE, WITHOUT TUNING.** Under
  the t9 angular target, all four error curves (3 horizons + closure) grind down **monotonically**
  post-takeoff (~0.74–0.86 @ gens 70–100 → 0.57–0.61 @ gen 331; closure dipping to 0.47) — the first run
  with a consistent downward trend (t8 only oscillated). The tie-break axis CAN couple when the target is
  honest; at ~−0.05/100 gens it's just ~900 gens too slow. Per [feedback_clear_objectives_not_tuning] the
  elevation must be STRUCTURAL (presence/coupling), not coefficients:
  1. **Consume the forecast** — feed aux outputs[3..6] back as NN inputs next tick (self-consumed forward
     model; closes the perception→prediction→control loop that passive scoring never did);
  2. **Promote the axis to first-class** — prediction as a full lexicase axis alongside score/energy
     (structural presence change, not an ε/weight tweak);
  3. **Re-target across blindness** (above) — score (t visible → t+h reacquisition) pairs where the
     information content actually lives.
  Candidate next-feature experiment: (1)+(3) together; (2) if the grind stays too slow.
  **Experimental-condition note (operator 2026-07-10)**: keep the curriculum OFF (`VariationRampStep=0`,
  the t8/t9 config) for these weak-signal experiments — a STATIONARY objective + bit determinism is what
  lets a third-order selection signal accumulate over hundreds of gens instead of being washed by a
  shifting landscape; t9's monotone grind is plausibly visible only because the ramp was off.
- **040 t1 UPDATE (2026-07-31, measured @ gen 179) — the t9 grind REVERSED, and there is now a second,
  purely mechanical reason the head cannot form. Routing: this is 041 material (operator 2026-07-31);
  040 stays camera-engine fidelity.**
  - **Observed**: all four curves rise instead of grinding down — err150 **0.40 (gen 1) → 0.74 (gen 179)**,
    err_rate 0.29 → 0.63. Against the permanent baselines in the same cache: persist150 = **0.0066**
    (the head is **113×** the no-change bar) and `span_scale` (σ of realized span) = **0.044 rad** (the
    head is **17× worse than emitting a constant 0.0**).
  - **New root cause, introduced by 040's own camera work**: T023-T035 / T033a moved bearings from ±1 NDC
    to **radians** and span to a great-circle angle, shrinking the predicted quantity ~20×. The aux
    outputs are raw **tanh, bounded ±1** (every layer applies `fast_tanh` — `src/nn/evaluator.cc:125`,
    `:217`). So the entire signal now lives in ~4% of the output range while the shared trunk saturates
    (W_hh spectral radius 4.73; control axes 97-100% over the amplitude budget) and drags the aux outputs
    to the rails. **err150 ≈ mean |aux output|** — the curve is now a *saturation* readout, NOT a
    prediction-skill readout. Do not read it as predictor progress while the head is in this state.
  - **The 038 elevation package is UNSTARTED**: `predictor` appears **0×** in `specs/040-camera-redo/`
    spec.md and tasks.md. Outputs `[3..6]` remain write-only leaves — not actuated
    (`src/nn/evaluator.cc:591-595`), no forecast slots in `TrackerInput`, output layer non-recurrent
    (`TRACKER_NN_RECURRENT = {false,false,true,false}`). t1 is the t6 predictor config with a new camera.
  - **Design note for 041 — prefer the structural re-target over rescaling.** Rescaling the aux target
    into tanh range is a coefficient fix on an objective the persistence bar already calls worthless, and
    it cuts against [feedback_clear_objectives_not_tuning]. Re-targeting to blindness-bridging horizons
    (0.5-2 s, visible→reacquisition pairs) collapses persistence, gives the head something only it can
    do, AND moves the target into a tanh-representable range as a side effect — one structural change
    instead of two.
  - **Cheap ablation while it sits**: `EnablePredictorHead = 0` costs nothing to try and answers whether
    the dead head + its lexicase axis are actively taxing the control search (119 output weights + one
    test case per scenario currently buying nothing).

### [038 follow-on — ~~BACKLOG~~ → **VALIDATED 2026-07-10 (t10, the 038 wrap exercise)**] M2 novel-path eval harness — measure M2 generalization

**Ran end-to-end at 038 wrap (t10)**: t5 M1 elite → `autoc-eval.ini` `random` 7 paths × 7 winds seed −1
(49/49 complete, per-scenario parity with training → M1 has no random-geometry gap) → repoint
`autoc-eval-tracker.ini` at that dmp in `autoc-eval` → t9 M2 elite 49/49 complete, **zero hull/OOB**.
Numbers + the read in [038 wrap §2](038-accurate-m2/wrap.md). **Metric lesson (operator)**: raw
per-scenario score is length-confounded (random scenarios ~2× longer, 896 vs 451 steps) — compare
**per-step rate** (0.076 vs 0.117 pts/step ≈ 65%) / pctInStreak (3.7% vs 7.5%) / error distance (same
~17 m band); and the random class plays as a mostly-continuous random-intercept problem, so a lower
streak fraction is partly path-class, not pure generalization. Harness is now config-only standing
practice = the wrap-gate for future M2 features. Original scoping below (eval-mode `generate_pngs`
comparator is the remaining nice-to-have).

**Gap (operator)**: every M2 judgment to date — training fitness, fixed-eval repro, all t4–t9 comparisons —
runs on the SAME 294 scenarios (the t5 source's 6 paths × 49 winds, and under
`TrackerChaseUseSourceScenarioSeed=1` even the same realized env). There is **no way to evaluate an M2
controller on paths/envs it never saw** — no OOD/robustness read exists for M2 the way path-5
random-intercept serves M1 ([project_path5_random_intercept]).

- **Canonical flow (operator 2026-07-10)** — the four-step pipeline that makes this a standing practice,
  not a one-off:
  1. **Run a real M1 training** (production bake) → pin the best M1.
  2. **Run a FEW eval configurations on that best M1** (nnextractor → `EvaluateMode=1`; different
     `RandomPathSeedB` / path mixes incl. the random-intercept class, fresh master seeds ⇒ different
     wind/entry/craft draws) → each produces a source-format dmp saved in the **autoc-eval bucket**
     (per-mode bucket contract; pin the keepers).
  3. **Run the M2 training off the real M1** (the step-1 training source, as today).
  4. **Eval the best M2 across the step-2 M1 eval results** (repoint `autoc-eval-tracker.ini`
     `TrackerSourceRun`/`TrackerSourceBucket` at each; knob=1 keeps shared-airspace semantics) →
     per-novel-source competence vs the training-set numbers = **the M2 robustness proxy**.
  All deterministic given seeds ("we have strong determinism").
- **Metric**: the generalization gap — mode_progress-class competence (perception / range / blind / streak)
  on the novel set vs the training set, same elite. A robust M2 shows a small gap; a memorized one
  collapses.
- **Also enables**: honest cross-feature M2 comparisons (today's are all in-distribution), and the
  train-across-multiple-sources robustness direction ([project_library_based_training], 031 library
  curation).
- Files: none new for v1 — it's config + operator workflow (bake novel M1 eval source → repoint M2 eval);
  optionally a `generate_pngs.sh` eval-mode comparator later.
- Panel now carries the persistence bar permanently (dashed per-horizon + diamonds; cache schema grew, old
  cache rows auto-refetch).

### [038 follow-on — CANDIDATE sensor input, 2026-07-09] Streak/in-envelope input — close the reward↔observation gap

**Operator (t8-era synthesis)**: we score streaks (the 5× multiplier is the dominant fitness shaping) but
the NN has **no way to know it is in one** — no clean "you are in the tracking regimen, go tighter" signal.
**The sharp version**: `FitStreakRampSec = 5.0 s`, but the NN's whole perceptual history window is **0.8 s**
— the reward is conditioned on ~6× more history than the policy can observe (a Markov violation from the
NN's seat). It cannot represent "how deep into the pocket am I", yet that's what the multiplier pays for.
This is an objective-PRESENCE gap per [feedback_clear_objectives_not_tuning], not tuning.

- **Design (camera-derived, deployable — operator: "rough equivalent from camera data")**: do NOT feed the
  fitness machinery's true streak state (sim-only privilege, violates the direct-observable principle).
  Instead derive from existing camera slots: `in_envelope(t)` = both beacons CEP-visible AND span within
  [lo, hi] AND pair centroid within a centered radius; `streak_proxy(t)` = min(consecutive-in-envelope time
  / 5 s, 1), resetting on dropout — one or two new tracker inputs (envelope flag + normalized duration).
  Proxy needn't match stpPt exactly; it needs to correlate — the NN learns the mapping. ms-based ⇒
  cadence-invariant. Works on real hardware (it's just camera history integration).
- **Cheap**: tracker-only input growth (58→59/60) — the T023 split AircraftState serialize means **no M1
  source rebake** (built exactly for this). Topology/fixture regen only.
- **Relation to the other levers**: this is "perceptual history" targeted at the ONE quantity the reward
  actually cares about — a features-version of US1/US2's longer memory, far cheaper than either. Pairs with
  predictor-head improvements (both are "give the trunk the tracking-regime state").
- **Context**: all M2 runs (t4/t6/t7/t8) learn fast then settle at ≈ the same fitness/competence regardless
  of ramp, env sharing, extra sensors, predictor — eliminated so far: reward shaping (037 t11–t15), env
  fidelity (t7/t8), situational sensors (t4), passive predictor (t6), W_hh capacity (rnn_capacity eff-rank
  ~11/16 stable). Remaining suspects: perceptual history depth, NN representation, craft physical limits
  (chase == target airframe ⇒ pure tail-chase of a max-performance gen-800 path can't close except by
  corner-cutting; the ~17 m median floor may sit near the physics limit — "looks decent" in playback
  supports this). Operator expectation: spherical projection + predictor polish = no big jump; this item is
  in the same "sharpen, don't expect miracles" class.

### [038 — follow-on lever, deferred 2026-06-30] US2 two-timescale recurrence (structural slow channel)

Deferred out of 038's **initial ablation wave** (operator 2026-06-30). 038 picks the two "add state" levers
— **US1 deeper history + US3 predictor head** — because a read of 037 leans "not enough state," and M2's
long loss-of-signal windows (several seconds → effectively a re-acquire) reward levers that add trajectory
state. **US2 (a fixed-leak slow recurrent channel) overlaps US1** (both add temporal memory) and is the
least-orthogonal of the three generic studies, so it is held back rather than run in parallel.

- **Launch-ready**: the starting config (hidden-2 16-wide fast + 8-wide fixed-leak slow, α≈0.9) and the
  format-break surface are recorded at [specs/038-accurate-m2/research.md](038-accurate-m2/research.md) §2,
  [data-model.md](038-accurate-m2/data-model.md) §2, and tasks **T019–T022** (held, not launched). It bakes
  off the same post-P0-D baseline (038 T013) when unparked.
- **Unpark trigger**: US1+US3 do **not** move an SC-001 ceiling, OR "not enough state" persists as the M2
  bottleneck after they land. Then run T019–T022 as the next ablation arm (and re-open the combine node).

### [038 — direction, discuss-after-baselines 2026-07-03] Learnable feedback + self-history + larger search space

**Lean (operator 2026-07-03)**: 038 favors **learnable feedback (internal memory / recurrence)** over piling
on more **hand-built calibrated input features**. Spectrum: hand-built (`closing_rate`, `span_rate`,
`dist_to_boundary`, `inward_body`, a proposed `jerk`/rate-history) → `US1` deeper history (hand-picked lags
as inputs) → `US3` predictor (learned + hand-designed aux target) → **`US2` two-timescale recurrence (the
net learns the feedback)**. The lean pulls toward the US2 end.

- **Key asymmetry**: current config gives *deliberate history to the TARGET* (perception channels carry the
  6-slot 0.8 s window) but **none to SELF** — `quat`/`airspeed`/`gyro` are instantaneous. Internal/self
  memory is the gap. "Overrun" = the chase **overshooting the target** (pursuit dynamics), NOT arena egress;
  the on-lean fix is learnable anticipatory feedback + more internal memory, NOT a hand-fed jerk sensor.
- **Instinct**: the right combination is **larger search space (pop) + more internal memory (recurrence)** —
  and those are the same coin: learnable feedback costs more evolutionary search (the +128-input FR-P0H
  enrichment already delayed t3's takeoff ~100 gens; added recurrent capacity is similar-or-worse + the
  basin lottery), so paying for it wants the pop/gen budget.
- **Judging shift**: `rnn_capacity` eff-rank + the evolution-chart feedback panels (`whh_xh_ratio`, block CV)
  become **primary "is it learning feedback" signals**, comparable to 037 — a ceiling lift with a *flat*
  eff-rank is off-thesis. Reconsider the US2 deferral above against this lean (US2 is the most on-thesis
  lever, yet it's the parked one).
- **History buffers are a FUDGE for weak internal memory (this inverts US1).** Historically internal-state
  evolution (recurrence) was tried, didn't take, so "direct memory" — the 6-slot hand-built lag buffers —
  was fudged in instead; it helped hugely and is why perception carries history. So **US1 (deeper history)
  doubles down on the crutch**, off-lean. Corollary: if internal memory genuinely cracks, we can **shed the
  raw input buffers** (the net evolves its own state) → smaller input vector, *less* enrichment tax. Cracking
  memory is the lever that **removes** tech debt, not adds it.
- **Candidate architecture — "deeper shift registers."** A middle path between free RNN and fixed input
  buffers: a couple of deeper shift-register-like delay lines the net reads/writes, deeper than today's
  single 16-wide recurrent layer (US2's two-timescale slow channel is one instance of this family).
- **Escape hatch**: if learnable/structured memory doesn't take, **brute-force the input buffers** — the
  known-to-work fallback (that's what the current history is).
- **Sequencing decision (2026-07-03)**: lead the initial wave with **US3** (learnable anticipation → directly
  targets M2 overrun / overshoot); **demote US1** to a gated M1-only control that must clear a *meaningful*
  bar to justify its format-break + xiao tech debt, OR drop it in favor of **un-parking US2**. Judge on
  *learnable-mechanism-used* (US3 predictor tracks realized optical state; US2 eff-rank / used modes), not
  just "a number moved."
- **Arc**: this supersedes the old "traditional inner-loop controller + SLM strategist" two-tier concept —
  now end-to-end **"photons in, control out."** 038's systematic input changes + regression discipline gave
  the good control-quality base to build the memory work on.
- **Trigger**: DISCUSS once the enriched M1 (t3) baseline **and its paired M2 baseline** have signal — then
  read the feedback-evolution panels vs 037 and decide pop-size + US2-unpark-vs-US1 + shed-buffers-if-memory-
  cracks.

---

## 030 spin-offs (deferred from 030 spec at plan-research scoping)

Items extracted from the [030 tracker-mode spec](030-tracker-mode/spec.md) on 2026-05-04, before plan-research begins. These were architecturally part of the 030 epic but earned defer-status under the smoke-test-first scoping (D13 / D16). Some are 031 (sibling-feature) candidates, some are pure backlog. Tagging as such; final 030/031 split is plan-research's call.

### [PARKED — camera bench era, relocated 2026-07-28] Camera hardware phase — recorder chain, clip format, loader

**Where it lives**: [`specs/040-camera-redo/camera-hardware-phase/`](040-camera-redo/camera-hardware-phase/)
— `plan.md` (the original 031 Phase-1 camera plan), `data-model.md`, `quickstart.md` (build-a-pod /
build-a-recorder walk-throughs), `recorder-status-codes.md`, `contracts/` (clip byte format, FPGA-recorder
contract, Python-loader contract, JSON sidecar schema), plus the `beacon-viewer/` and `beacon-loader/`
Python packages.

**Why it moved**: 040 was specified 2026-07-28 as *sim-side perception fidelity only* — none of this
material is in scope. Four of its filenames (`plan.md`, `data-model.md`, `quickstart.md`, `contracts/`)
collide with `/speckit.plan` output and would have been overwritten in place. Filed here so it stays
findable rather than being rediscovered by accident.

**NOT obsolete — actively referenced by the running 031 build**: `cad/beacon-eval/verified-bom-eval.md`
(§(a) hand-off to the cube-mounted target), `specs/031-beacon-camera/tasks.md` (T033 pod hand-build),
`schematic.md`, `spec.md`, and both `firmware/flight-recorder/` READMEs link into it. Links were rewritten
at the move; do not relocate again without re-running the sweep.

**Unpark trigger**: a camera bench exists — article 1 plus raw uncompressed high-bandwidth capture (the
same trigger as the deferred photon budget in the 040 spec). ⚠️ Re-validate first: these documents predate
the 031 1-bit acquisition phase and the 20 Hz / 480 fps / 200 Hz / 75 ms baseline, and carry legacy
240 fps / 100 Hz / 150 ms numbers.

### [040 — camera redo] Parallel perception-front-end — camera pixels → (x, y, CEP)

> Re-homed to feature **040 (camera redo)** on 2026-06-20: 031 narrowed to the 1-bit single-IR-sensor acquisition-research phase; the camera pipeline (this item + the FOV/representation item below) is the separate 040 effort. Emitters are shared; 040 replaces the single-sensor analog front end with a camera + bigger FPGA. The old camera `specs/031-beacon-camera/spec.md`+`plan.md` are 040's reference.

- **Trigger**: real-flight beacon hardware build / virtual-beacon flight test (staged-path row 5+).
- **Scope**: the FPGA / DSP pipeline that bridges raw camera pixels to the `(x, y, CEP)` triple 030 consumes — thresholded centroid extraction per color channel, cluster-spread (CEP) computation, threshold-fail sentinel emission. **Includes**: prop-arc occlusion modeling (rolling-shutter resonance with prop RPM, banding artifacts at certain RPMs, intermittent global-shutter occlusion at non-resonant rates); airframe self-occlusion mesh refinement beyond 030 v1's coarse body proxy; LED wavelength + emission-cone tolerance characterization.
- **Why parallel rather than 030-internal**: shares only the `(x, y, CEP)` interface contract with 030; the engineering surface (FPGA design, threshold tuning, color-channel separation, real-camera characterization) is image-domain work that runs alongside 030's controller-side training without touching it. The two features inform each other through the contract: 030's per-scenario PRNG variation experiments tell perception-front-end what camera-spec tolerances need to be met; perception-front-end tells 030 what int8 noise floor and CEP magnitude distribution to actually quantize with.
- **Source design notes**: 030 spec D7 (DMP versioning + parallel feature), D10 (camera v1 baseline + prop-occlusion deferral).
- **Files likely**: new `src/perception/` module (or top-level `perception/` peer to `crrcsim/`), new spec dir `specs/040-camera-redo/` when this unparks.

### [040 — camera redo / research] Perception representation — event-camera + non-linear / dual-FOV optics (FOV + variations input)

- **Trigger**: research thread for post-beacon perception (when 031 perception-front-end above unparks, or sooner if the 030 dual-camera variant experiment needs it). Capture as research now so it informs the camera-spec → resolution-budget conversation when we leave beacons.
- **Question 1 — Event-camera representation**: today the NN sees `(x, y, CEP) × history×6` for each wingtip beacon — a sampled raster of an underlying image-plane process. Once we drop beacons for full-image perception, the natural representation becomes either a dense pixel grid (huge input dim) or an event-stream representation (per-pixel intensity-change events with timestamp + polarity, native to event cameras / DVS sensors). Event-camera reps are sparse, latency-friendly, and naturally encode motion direction; they may be a better match for the controller's cone-tracking task than dense rasters.
- **Question 2 — Optics with non-uniform angular resolution**: a 120° FOV with uniform pixel pitch wastes resolution on the edges where the target rarely sits, and starves the center where it does. Two candidate architectures:
  - **Dual-camera**: wide (~180°) for acquire/orbit-recovery + narrow (~60°) for hi-res tracking. NN sees both feeds (concatenated or as separate channels). Mirrors how birds-of-prey use peripheral + foveal vision.
  - **Single non-linear lens**: fisheye / log-polar / panomorph optics that compress edges and expand the center on the same sensor. Lower hardware cost, but introduces lens calibration and non-linear NDC math; the NN has to learn the warp implicitly.
- **Question 2b — RAPTOR BINOCULAR arrangement: two IDENTICAL wide cameras, splayed, overlapping forward (operator 2026-07-28, "might be what we wind up doing after M2")**. Distinct from the wide+narrow sketch in Question 2 above — that one mimics the *foveal* adaptation (peripheral + high-acuity); this one mimics the *binocular* adaptation (lateral coverage + forward depth), and looks stronger. Two identical 120° cameras on the wing leading edge at ∓8″ (the 040 baseline mount, [040 input checklist](040-camera-redo/input-data-checklist.md)), splayed outward by α:

  | splay α | total coverage | binocular overlap | prop shadow |
  |---:|---:|---:|---|
  | 0° | 120° | 120° | in frame, 41–61° inboard |
  | **20°** | **160°** | **80°** | **gone** |
  | 30° | 180° | 60° | gone |
  | 45° | 210° | 30° | gone |

  **Solves three problems at once**: (a) **blindness** — 160–180° coverage attacks the documented M2 ceiling head-on ([project_m2_tracking_ceiling]: reacquire-through-blindness, ~8 s worst-case blind windows, reward-invariant ⇒ perception-capped); (b) **range where it matters** — the binocular lobe sits forward, exactly where a tail chase terminates, and stereo yields range from a **single** visible beacon, which is the case precisely when the target banks and hides a wingtip (beacon-separation ranging fails there); (c) **prop occlusion vanishes for free** — the shadow lies inboard, so **any splay > ~19° rotates it outside a ±60° field**. The splay chosen for coverage is the splay that kills the occlusion.
  - **Baseline caveat (carry the right expectation)**: ∓8″ ⇒ a 0.41 m stereo baseline vs the target's 0.772 m beacon separation, so as a *ranging-accuracy* upgrade it is ≈1.9× coarser than just using the target's own wingspan. Its value is **robustness, not precision** — single-beacon range, plus geometric rejection of ground-bounce/glint false blobs (a genuine beacon appears in both views at consistent disparity). Judge it on those, not on range error.
  - **Cost**: the NN input vector grows to two cameras' worth of observations — a real feature, not a tweak. Also doubles the perception front-end. Symmetric mounting cancels any asymmetric drag from a single LE camera.
  - **Trigger**: after M2 (operator). Natural pairing with the rear-facing-camera idea (same blindness target, different geometry) and with [031-fed] CEP realism.

- **Question 2c — PAN/TILT (and eventually zoom) single camera: decouple instantaneous field from coverage (operator 2026-07-28, "at some point that may be better than two cameras… we've done a bunch of pan tilt so far")**. The third architecture on the table alongside Question 2 (wide+narrow foveal) and Question 2b (raptor binocular). It attacks the **central tension** the 040 optics analysis exposed — *you cannot have 120° FOV and 100 m range on a 320×240 sensor* ([040 input checklist](040-camera-redo/input-data-checklist.md) §B4) — because a gimbal is the only option that gets both, at the cost of seeing **one place at a time**.

  - **Why it works quantitatively**: background-limited SNR ∝ f² ∝ **1/FOV²**, so narrowing buys range *quadratically*. Against the measured ~40 dB shortfall of a 120° optic: narrowing to **30° buys +12 dB**, to **15° buys +18 dB**. Combined with the ~20 dB available from emitter drive, aiming, sensor format and aperture (same §B4 lever budget), **a 15–30° gimballed camera is roughly what 100 m actually needs** — a normal lens on a normal gimbal, not the 3.4° exotic the original narrow design assumed.
  - **It matches the patrol/engage mode split better than any fixed arrangement.** Patrol wants ~100 m detection over a wide search volume and *can afford to sweep* because nothing is manoeuvring yet; engagement wants the target near boresight. That is a description of what a gimbal does. Fixed optics must trade instantaneous field against range; a gimbal trades *time* instead.
  - **Prior art in-house**: operator reports substantial pan/tilt experience, which materially lowers the build risk versus the dual-camera paths.

  **Costs and the real risks**:
  - **Slew rate is the binding constraint.** Target angular rate at terminal geometry: **~248°/s at 3 m** (13 m/s crossing), 74°/s at 10 m, 30°/s at 25 m. On top of that the gimbal must *reject body motion* — 039 flight data shows pitch-rate RMS 128–141°/s with roll capability to ~500°/s. So the gimbal is simultaneously stabilising and tracking, and terminal geometry is where it is hardest.
  - **Acquisition gets worse, not better.** A narrow field must *search* to acquire — cold acquisition becomes a slew pattern, potentially far slower than a wide fixed camera that simply sees everything. This is the classic trade (wide acquires fast and tracks poorly at range; narrow the reverse) and it argues for either a fast search mode or pairing with a wide fixed sensor.
  - Moving mass and power on a 520 g airframe; **moving parts on an aircraft whose mission involves collisions**; a second control loop with its own latency and dynamics.
  - **Pointing knowledge**: prefer resolving to body frame *in the front-end* so the NN interface stays unchanged, rather than adding gimbal angles to the input vector.

  **Zoom** (operator: "I get ahead of myself") would give the dual-FOV benefit — wide to acquire, narrow to track — on **one** sensor, at the cost of heavier/slower optics and a focal length that becomes a live variable the front-end must track for calibration.

  **Trigger**: post-M2, alongside the Question 2b evaluation — these are alternatives to each other, not a sequence, and the choice should be made against a calibrated camera link budget (article 1 + raw capture) rather than on paper.

- **Question 3 — Projection model: go spherical/equidistant, not rectilinear (operator direction 2026-07-07)**. *[PULLED FORWARD 2026-07-07 as a likely near-term step after t6 — see the promoted entry under "038 deferrals" at the top of this file; full rationale + blast radius stay here.]* Today the sim projects rectilinearly (pinhole): `screen = tan(θ)/tan(fov/2)` ([camera_projection.cc:168](../src/eval/camera_projection.cc)). Because NDC ∝ `tan(θ)`, a *fixed angular* wingspan reads **larger toward the frame edge** than at center — so `beacon_pair_span` (and `tilt`, `span_rate`, the CEP edge factor, and every raw beacon NDC input) carries an **ego-pointing contamination**: where you aim the camera changes the reading. This surfaced in the 038 US3 span-predictor discussion (2026-07-07): it muddies "what span means" — span is *meant* to be a clean range×aspect closure signal, but the tan-stretch injects an ego term. **Direction**: switch the sim to a **spherical / equidistant** projection (`screen ∝ θ`) applied uniformly to **all** coordinates, so NDC is ∝ angle and span becomes ego-pointing-invariant (a clean angular quantity). Then, once real camera hardware is chosen, **model that specific lens's projection function** (rectilinear / equidistant / equisolid / panomorph) in sim to match — projection becomes a *modeled physical property of the chosen optic*, not an arbitrary sim convention, and lens choice becomes a first-class sim variable (ties to the dual-FOV / non-linear-lens options above). **Blast radius**: `camera_projection.cc` `screen_x/y` + the FOV clip + CEP edge factor; changes every beacon NDC input + all derived features → a perception-representation change, so M1/M2 source dmps are invalidated and need a rebake (greenfield, no cereal bump per project practice). Couples to [031-fed] CEP-realism below and `project_cep_realism_backlog`. Span itself stays the metric — operator: "span is nice, keep it"; this only makes it mean one thing everywhere.
- **What the early minisim playback informed**: the 120° FOV + raw-NDC-projection presentation gives narrow beacon spacing close-in (~0.26 NDC at 10ft), and the controller learned an emergent orbit-to-reacquire when the target left the FOV — useful evidence that the current rep gets some way, but reacquisition cost in crrcsim's harder dynamics may push the topology budget higher than 030 v1 plans for. A richer rep (event stream) or smarter optics (dual-FOV / non-linear) could lower that topology demand instead.
- **Why research-track, not implementation**: needs a dataset + simulator camera model upgrade (or recorded event-camera bench data) before any controller work; coupled to the 031 perception-front-end FPGA / DSP scoping.
- **Source design notes**: this thread; 030 D10 (single-camera v1 baseline that this would supersede); see also `[BACKLOG] Multi-camera variant experiments` below for the controller-side experiment shell once a camera spec is chosen.

### [040-fed, filed 2026-07-28] Detection-quality degradation modes — make the detection envelope emergent instead of asserted

040 **asserts** the detection envelope (sensor good to ~100 m) and proxies signal-to-noise into the quality
value, rather than letting the signal budget set a cutoff ([040 spec](040-camera-redo/spec.md) FR-033a).
That was deliberate: the budget is not calibrated well enough to be trusted as a *limit*, and the physics
that would set a real limit is shelved. Operator framing 2026-07-28: *"we operate with uncertain or
tentative information — let's just say the sensor is good to 100 m for now with some s/n proxied in the
CEP; later we can add a lot more."*

**What would make the envelope emergent** (roughly in order of expected effect):

- **Ambient level / time-of-day** — the measured dominant limiter. 031 field tests: full sun rails a
  DC-coupled front end; direct-sunlight lock fell to 4.5–6 m bare/unfiltered vs 12.5 m dark. Needs sun
  position in the simulator.
- **Sun angle in or near the FOV** — no filter helps with in-band sunlight; also the AGC-response question
  (couples to the flight-data trigger already recorded for sun/glint).
- **Glint** — specular water/metal returning false point sources. Unknowable until a camera exists.
- **Atmospheric**: dust, haze, scintillation — blob spreads across more pixels, dimmer per pixel.
- **Sensor variation** — QE spread, read noise, well depth, exposure tolerance; needs article 1.
- **Optics degradation** — dirty lens, filter aging.

**Trigger**: a calibrated camera link budget exists (article 1 + raw uncompressed capture — the same
trigger as the deferred photon budget), OR field data shows the asserted 100 m envelope is materially wrong
in a way that changes M2 behaviour. Pairs with the [031-fed] CEP-realism items below and with the camera
PRNG slot item that follows.

### [BACKLOG — camera variations, scoped 2026-07-09] 5th PRNG class slot: mount-alignment 6-DOF first, then FOV/aberrations/noise

**Operator scoping (2026-07-09, during the t9 equidistant-projection change)**: camera variations are
**mostly an ALIGNMENT problem** — per-scenario **6-DOF camera-mount pose error** (3-axis rotation + 3-axis
translation of `camera_mount_chase_body` / `camera_orientation_chase_body`) is the main axis; then "perhaps
a little on FOV" (intrinsics tolerance), "maybe some aberrations" (residual lens-distortion after the
front-end's planar→angle calibration remap — see the PHYSICAL-CAMERA ASSUMPTION note in
`camera_projection.h`), and some perceptual noise (couples to the [031-fed] CEP-realism items, which stay
the noise-model home).

- **Plumbing is already reserved**: the `camera` class sub-seed is slot 5 of `deriveClassSubSeeds`
  (FROZEN order wind/rabbit/entry/craft/camera, `scenario_prng.h`), and `ScenarioMetadata` has the
  documented `cameraSeed` append point after `craftSeed`. Draw-and-discard convention like entry/wind/craft;
  σ knobs in the ini per the craft-variation pattern. Camera stays on the M2 seed even under
  `TrackerChaseUseSourceScenarioSeed=1` (chase-only perception).
- **Why rotation alignment is the headline**: real-hardware precedent — the INAV board-alignment 170°-vs-180°
  issue put a ~10° pitch bias into flight data ([project_board_alignment]); a mount-rotation error is exactly
  that class of defect for the camera. Under the t9 ANGULAR NDC a camera-rotation misalignment ≈ a constant
  additive NDC offset (clean, learnable robustness target — under rectilinear it was position-dependent).
- **Trigger**: pre-real-hardware M2 training (the run whose controller flies a physical camera), or 040
  front-end characterization telling us actual tolerance numbers — whichever first.

### [BACKLOG — M2 realism, filed 2026-07-31] Chase and target must eventually be DIFFERENT craft — share the air mass, not the airframe

**Current state (verified 040 t1, 2026-07-31)**: `TrackerChaseUseSourceScenarioSeed=1` shares ONE
`scenarioSeed` with the M1 source, and every class sub-seed derives from it — so the chase inherits the
target's **craft** draws along with wind/thermal/gust/entry/crash-hull. The two aircraft are therefore the
**same airframe realization**: identical CG, drag, trim, thrust, pitch/roll authority, servo slew, thrust
tau. The startup log says as much (`…wind/thermal/gust/entry/craft/crash-hull seeds…`).

**This is deliberate for now (operator 2026-07-31)**: the current question is "can we roughly track in the
same *temporal air environment* with an identical craft" — holding the airframe fixed isolates the
environment/perception question, which is what t7 was built to answer. Not a defect; a scoping choice.

- **Eventual target**: the air mass stays shared (both aircraft fly the same wind/thermal/gust field —
  that part of the seed sharing is physically right), but **everything else about the two craft becomes
  independent**. A chase that can only track a copy of itself has learned a weaker skill than one tracking
  an airframe with different speed, turn rate, and energy state.
- **Implementation shape**: split the shared seed by CLASS rather than sharing it wholesale — keep
  `wind` from the source, draw `craft` (and `entry`) from the chase's own M2 seed. The plumbing already
  distinguishes classes (`deriveClassSubSeeds`); what's missing is a per-class share/independent policy
  instead of today's all-or-nothing `chaseScenarioSeedAt()` swap. Same pattern the camera slot uses
  (chase-specific even under seed sharing — 040 US6 T070).
- **Trigger / relation to other work**: **co-evolution** is the likely forcing function (an evolved or
  adversarial target is by construction not the chase's twin); also any real-hardware M2 where the target
  is a different physical aircraft. Pairs with the library-based-training direction
  ([project_library_based_training]) — recorded real flights are inherently not the chase's airframe.
- **Watch item when it lands**: expect tracking difficulty to rise; the current M2 ceiling numbers were
  measured against a twin, so they are an OPTIMISTIC baseline for the differing-craft case.

### [031 CANDIDATE] Variable-rate / real-flight source robustness

- **Trigger**: real-flight-recorded trajectories become available (post-virtual-beacon flight test).
- **Scope**: exercise the FR-018 timing model under realistic xiao+INAV telemetry jitter and dropped samples. Confirm determinism end-to-end at non-real-time sim speeds (contract test). 030 v1 timing model is *built to handle* variable-rate sources but is *exercised* only at uniform-rate pathgen-derived sources.
- **Source design notes**: 030 D14 (timing-model exploration row).

### [031 CANDIDATE] Library curation + turn-direction mirror-pairing

- **Trigger**: when 030 smoke-test green and operator wants real-target-class generalization, OR when the controller fails an OOD turn-direction test in early eval.
- **Scope**: library composition tooling — mirror-pair every recorded trajectory into a left-turn / right-turn matched set during ingest (flip Y in NED + flip roll quat + flip rcData[0]); cross-source-run mixing (sim + real, geometry-filtered); auto-bootstrapping (winners-of-tracker-runs feed back into the library). All currently in 030 spec's Out of Scope or C1 / C5 open considerations; collecting them here.
- **Source design notes**: 030 spec C1 (turn-direction symmetry trap), C5 (pathgen vs library decision matrix).
- **Why deferred from 030**: not load-bearing for the smoke test; matters once the operator wants generalization beyond what a single source dmp delivers.

### [031 CANDIDATE] Renderer "exotic goodies" — reverse-projection + dphi-overlay + crash-strike viz + FOV dome

> **FOV-dome design (2026-07-09, deferred — reticle ticks landed instead)**: render the camera's angular
> acceptance as an in-world spherical cap mounted on the chase (camera at center, radius a few m), beacon
> dots ON the dome at their true ray directions. The t9 equidistant NDC inverts exactly:
> `a=sx·fovh/2, b=sy·fovv/2, θ=√(a²+b²), dir=(a,b)/θ, ray=(cosθ, sinθ·dir_y, sinθ·dir_z)` → rotate by
> chase/camera pose, `dot = cam_pos + R·ray`. Span = great-circle arc (constant size anywhere by
> construction); dot-ray vs true-target ray makes perception error VISIBLE in 3-space (pairs with the
> reverse-projection overlay below); the dome patch is the exact per-axis angular clip region and would
> replace the FOV pyramid whose straight corners are now approximate (true corners reach ~75°). Interim
> step DONE 2026-07-09: 15°-step angular reticle ticks on the 2D POV panel (even spacing == linear-in-angle
> statement; `tools/renderer.cc` updateCameraPOVMiniPanel).

- **Trigger**: post-smoke-test, when operator finds analytics-experimentation needs them to debug specific failures.
- **Scope** (from 030 D15):
  - **Reverse-projection overlay** — given chase pose + recorded perception output `(x, y, CEP)`, compute via reverse camera model (inverting aberrations + int8 quantization, propagating CEP into a 3D uncertainty cone) where the controller "thinks" the target is. Compare to where M1 target actually is (also in M2 dmp per FR-015). Visualizes perception error in 3D — directly diagnostic for "lost fitness because of perception, or because of control?"
  - **Old-style dphi/dtheta overlay** — from chase pose + M1 target's true position, compute what pathgen-mode's old `(dphi, dtheta, dist)` projection would have shown, render alongside the new beacon view. Comparison tool for debugging whether the new representation delivers equivalent or richer information than the old.
  - **Crash-hull strike visualization** — mark hull entries (and `p_crash` fires) in 3rd-person view; aids hull-curriculum tuning.
- **What stays in 030 v1 from D15**: error bars on the camera-POV display (CEP as visible ellipse spread) — cheap, directly load-bearing for smoke-test signal-or-not assessment.
- **Why deferred**: research-grade analytics; not required for smoke-test 4th deliverable.

### [031 — BACKLOG] OG0VA design-house inquiry (register spec + eval module)

- **Trigger**: when the OG0VA moves from paper-primary to an actual bring-up target (camera/040 work resumes, or the emitter power/osc study needs the real integration-time + FSIN/STROBE timing).
- **Scope**: OEM/design-house engagement (OmniVision direct, or Leopard Imaging / e-con) for (a) the full OG0VA register spec — exact integration-time min/granularity, FSIN/STROBE timing diagrams, gain map; (b) a stocked eval module or the OC0VA CameraCubeChip; (c) lead time + MOQ. Resolves research R4.
- **Why deferred (2026-07)**: sticking with OG0VA on paper for now. The [product brief](https://www.ovt.com/wp-content/uploads/2022/06/OG0VA-PB-v1.3-WEB.pdf) (480 fps @ 320×240, 60% QE @ 850 nm, global shutter, FSIN frame-sync + built-in STROBE, 1-lane MIPI, manual gain) is enough to design against. Exposure range is bounded mechanically (~tens of µs → ~2 ms full-frame @ 480 fps); exact numbers only needed at bring-up.

### [031 — STUDY] Code-derived shutter sync + sub-100% chip-on pulsing

- **Trigger**: after the emitter power/cooling + RC-osc-stability bench is characterized; when the emitter pulse regime (peak / duty) is being optimized against the camera.
- **Scope**: two coupled ideas —
  - **Adaptive exposure gating**: acquire wide-open (max shutter, to *locate* signal), then once a code is decoded, derive a per-emitter chip-phase/sync pulse from the code decoder and *narrow* the camera exposure to just around the expected pulse — improving background rejection and letting the emitter run a short pulse. Each emitter has its own independent RC clock, so this is a per-code phase estimate; **two emitters = two independent clocks**, so one narrowed window can't serve both at once (harder, not impossible — interleave, or narrow only after single-target hand-off).
  - **Sub-100% chip-on duty**: pulse the LED for only a fraction of each ON chip (e.g. 25%) instead of full-on, cutting average power / heat / di-dt (see the power-cooling budget thread). Bounded below by the async frame-capture floor (LED-on ≥ ~1 frame period so an un-synced camera never misses an ON chip); the code-derived sync above is what would let the pulse go *shorter* than that floor.
- **Payoff**: lower emitter average power → cooler LEDs, longer runtime, smaller supply-load transient → **better RC-osc stability (the 031 objective)**. Trades against matched-filter oversampling margin; needs the FSIN/STROBE + decoder-phase plumbing.
- **Flight caveat**: emitter (target craft) and camera (tracker craft) can't be genlocked in the air — the code-derived sync is receiver-side only (camera locks to the decoded code phase; emitter stays free-running).

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
- **040 T021 data point (2026-07-28) — the drift is now a correctness hazard, not just tedium.** Reproducing
  the t9 M2 training run for a bit-identity gate required hand-aligning **four** fields in
  `autoc-eval-tracker.ini` — not just the obvious `TrackerSourceRun`/`Bucket` + `Seed`, but the scenario
  **shape** (`SimNumPathsPerGeneration` 7→6, `WindScenarios` 7→49), because the file had been left
  configured for the t10 novel-geometry exercise. **One shared eval ini is doing two incompatible jobs**
  (training-repro vs novel-geometry generalisation) with only a comment separating them.
  - It failed loud and correctly — *"seed table (49) not 1:1 with source list (294)"*, the 7×7 t10 grid
    against the 6×49 t9 source — so the guard earned its place. But the guard only catches the *count*
    mismatch; a config that differed in sigmas or enables would have produced a plausible wrong number.
  - **Open design question the operator named (2026-07-28)**: must an M2 run assume the *same scenario
    shape* as its M1 source? Today the 1:1 seed-table check enforces exactly that, and it is what couples
    the eval ini so tightly to whichever run it last served. Relaxing it (M2 shape independent of M1, or
    running more scenarios than the source provides) is the design fork that has stalled this item.
  - **Status: deliberately deferred 2026-07-28** — *"we have had various ideas… gave up for now and keep
    the ugly setup"*. Recorded so the next attempt starts from the fork above rather than rediscovering it.
  - Related: **Self-describing dmp** (record the config block in every gen dmp) below — the other half of
    this problem, since a run that carries its own config makes "reproduce run X" a lookup instead of an
    archaeology exercise. The two should probably be designed together.
- **Want**: replace the single-file load with a config system that supports STACKING multiple INI files
  in "last value wins" order, so e.g. `-i base.ini -i m2.ini -i eval.ini` composes (base + overrides).
  The eval / visual / tracker variants then become thin override files over one base, eliminating drift.
- **Options**: a small layered wrapper over inih (parse N files in order into one reader); or a more
  standard library (toml++ / libconfig / cpptoml) if we also want typed sections. Whatever lib must still
  feed the X-macro auto-parse/auto-print (`AUTOC_CONFIG_FIELDS`) and preserve determinism + the
  fail-loud-on-missing-required-key behavior (e.g. `ControlIntervalMsec`).
- **Care**: extend the existing single `-i` flag to a repeatable `-i` (order = precedence).
- **Also want `[section]` grouping (2026-06-17)**: the flat key list has no structure, so related keys
  drift apart and flags vs parameters blur — e.g. 038's `EnableHullCrashPenalty` (a flag, belongs with
  the other `Enable*`) vs `HullCrashPenaltyFactor` (a parameter, belongs with the fitness/`Fit*` block)
  had to be hand-placed into two different spots in all six files. A standard `[section]`-based reader
  (toml++ / cpptoml) would let the schema express `[variations]` flags vs `[fitness]` params so
  placement is structural, not manual. Pairs naturally with the layered-stack idea above.
- **Also: decouple unit/contract tests from the production inis (2026-06-20)**. `contract_tracker_config_tests.cc`
  reads the *mutable* repo-root `autoc-tracker.ini` and pins **tunable** values as "drift gates" —
  `FlightArenaRadius==80`, `CepGateThreshold==1.25`, `BeaconEmissionConeDeg==270`, `BeaconLeftMountY==-0.45`,
  … — so tuning any of them breaks the unit suite. Surfaced when changing `FitStreakThreshold` 0.5→0.3:
  an over-long inline comment pushed that line past inih's 200-char `INI_MAX_LINE`, failing
  `OperatorIniParsesClean` (good catch) — but it exposed that the suite is hostage to the file we
  deliberately change. **Principle**: unit/contract tests validate the *parser + schema + loud-fail*
  against **test-owned fixtures** (the `writeTempIni(...)` tests already do this correctly), never assert
  mutable production values. **Fix**: strip the tunable-value pins; keep at most a minimal "production ini
  parses clean + required structural keys present (`Mode`, `TrackerSourceRun`)" guardrail (or drop the
  production-ini dependency entirely and rely on autoc's startup fail-loud + fixtures). Pairs with this
  item because the layered base+override stack shrinks the production-ini surface a test would touch, and
  a typed `[section]` schema makes "valid config" a structural property to fixture-test, not a value list.
- **Recurred + two concrete asks (operator 2026-07-07)**: adding `EnablePredictorHead = 1` (038 US3)
  with an explanatory inline comment made the tracker-ini line 216 bytes — past inih's ~200-char
  `INI_MAX_LINE` — so autoc aborted at startup with a bare `FATAL ERROR: Cannot parse configuration
  file 'autoc-tracker.ini'` (`src/util/config.cc:53-54`), **no line number, no reason**. Cost a bisect to
  localize. Two fixes wanted, independently useful:
  1. **Allow long lines**: raise or remove the `INI_MAX_LINE` cap (bump the inih define, or switch to a
     reader that streams lines) so a legal `key = value  # long comment` never fails on length. Comments
     documenting a knob shouldn't have to be truncated or moved off-line.
  2. **Diagnose the fault**: `ini_parse` returns the offending 1-based line number in `ParseError()` —
     surface it. The message should read `Cannot parse '<file>': line <N>: <the line text>` (+ a hint for
     the common causes: over-length line, missing `=`, stray `[`), instead of the current opaque string.
- **Trigger to act**: the next time an ini-wide change has to be hand-applied across all six files.
- Files: `src/util/config.cc` (load + `ParseError()` line-number surfacing), `include/autoc/util/config.h`,
  `tests/contract_tracker_config_tests.cc` (decouple from production ini), the six `autoc*.ini`.

### [BACKLOG — pre-work for the feature after 039] Trainer-populated (write-through) analytics cache — kill the S3 re-fetch tax

> **Scheduling (operator 2026-07-06)**: NOT an 038 item. Slate as pre-work / early task for the feature
> *after* 039 (touches the trainer hot-path + analytics format + generate_pngs; wants its own careful
> rebuild-perf pass, not a mid-038 insertion). Interim mitigation until then: `GENERATE_PNGS_CACHE=<persistent
> path>` so the cache survives reboots.


- **Surfaced 2026-07-06** (t5 PNG refreshes timing out on S3 fetch): `generate_pngs` derives the per-gen
  run-summary by pulling one dmp per gen from S3 and running `dmp-dump --run-summary`. During a live run
  (or right after a reboot wipes `/tmp/generate_pngs_cache`) that's hundreds of slow S3 round-trips for data
  the **trainer already had in-memory** when it computed the gen.
- **Want (operator 2026-07-06)**: have the **trainer append the per-gen run-summary row to the local cache
  as it runs**, right after the S3 dmp upload — a write-through cache in the volatile dir. `generate_pngs`
  then reads a warm cache and fetches nothing for gens the run produced; **S3 stays the source of truth /
  fallback** (missing cache ⇒ re-derive from S3 exactly as today). Pure optimization, no new correctness
  surface. Fixes the common case (live monitoring = always-warm cache); only post-reboot analysis of a
  *finished* run re-fetches.
- **Design**: factor the run-summary-row computation into ONE function used by both the trainer (live
  append) and `dmp-dump --run-summary` (post-hoc/fallback) so a trainer-written row is byte-identical to a
  dmp-derived one and can't drift. Write to the exact key `generate_pngs` reads
  (`${CACHE_DIR}/<run-id>__dmp<DSIG>_summary.csv`); the DSIG (dmp-build signature) keying already guards
  version splicing. Optionally also emit the `dynamics_progress` per-gen CSV row live (the other slow S3
  path). Consider `GENERATE_PNGS_CACHE=<persistent path>` as the interim mitigation until this lands.
- **Relates to**: `project_dmp_driven_analytics_backlog` ("run-summary slower than data.dat during live
  runs; cache to avoid re-fetch") and the log-slimming goal (the trainer already logs most of these fields
  in `#NNGen`/`#GenDiag`).
- **Files**: `src/autoc.cc` (per-gen live append), a shared run-summary-row helper (new or in
  `tools/dmp_dump.cc`'s run-summary path), `scripts/generate_pngs.sh` (already reads the cache — no change
  beyond finding it warm).

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

### [041 CANDIDATE — operator direction 2026-08-01] Make the predictor earn its keep

**Status change 2026-08-01**: this entry was first written around *design*
hypotheses. It has been rewritten around **measurement** — the span/closure head
was instrumented on the live 040 t1 run and carries **no usable information**.
That closes several questions the earlier draft left open, and reorders the work.

---

#### MEASURED — the head is not learning, and rescaling will not save it

Reproduce with `specs/040-camera-redo/predictor_signal.py` (new script, not an
edit to `predictor_analysis.py`). Source: 040 t1 elite @ gen ~709, 294 scenarios,
132,690 ticks, ~93-97k visible (t, t+h) pairs per horizon.

| horizon | r(level) | **r(Δspan)** | **r²(Δ)** | raw \|e\| | after IDEAL rescale | persistence |
|---|---:|---:|---:|---:|---:|---:|
| +50 ms | −0.061 | −0.024 | 0.06% | 0.6828 | 0.0347 | **0.00407** |
| +100 ms | +0.036 | +0.005 | 0.00% | 0.6845 | 0.0346 | **0.00584** |
| +150 ms | **+0.103** | −0.008 | 0.01% | 0.5530 | 0.0342 | **0.00748** |
| closure rate | — | −0.018 | 0.03% | — | — | — |

Also: **best error at every horizon occurs at GENERATION 1** — the random
initialisation. 709 generations produced wander in the 0.5-0.85 band, nothing else.

**Three findings, in order of importance:**

1. **THE METRIC IN USE IS THE WRONG ONE.** `computeSpanPredictionError` scores
   mean \|predicted − realized_span\|, and that conflates offset/scale error with
   information content. Span is SLOW — it moves ~0.0075 rad per 150 ms against a
   ~0.049 rad level — so **persistence ("assume no change") is already right to
   within 15%**. Predicting the *level* is therefore trivial and beats nothing.
   The only statistic that matters is **r(prediction, Δspan)**, and it is
   **≈ 0 at every horizon**.

2. **The +150 ms "tilt" is real but is the head learning the MEAN.** Operator
   spotted a tilt in the +150 ms calibration scatter — correctly, it is the only
   horizon with a positive level-correlation (r = +0.103). But r(Δ) = −0.008
   there. The head has partly learned "span is about 0.06", a constant that
   persistence already encodes. There is a physical reason the tilt appears only
   at the longest horizon: at +50 ms the change is buried in grid quantisation,
   and by +150 ms the closure trend has cleared the noise floor. The horizon 038
   picked for actuation-lag reasons is also the shortest one with any SNR.

3. **The parameterisation is broken, and fixing it is NOT sufficient.**
   Predicted mean +0.27, sd 0.59 vs realized mean +0.062, sd 0.049 — a ~12×
   scale error, because `fitness_decomposition.cc:70` compares a raw bounded NN
   output directly against a 0.049-rad target with no scaling, wasting ~95% of
   the output range. But applying the **ideal** linear recalibration still leaves
   4.6-8.5× WORSE than persistence. Scaling is a necessary fix, not the fix.

*(Ruled out: lexicase ε is NOT the problem. `LexicaseEpsilonMode = mad` is on and
the 0.5 floor is skipped entirely in MAD mode — `selection.cc:123-134`.)*

This quantifies 038's "structurally worthless as posed" and supplies the mechanism.

#### The cost being paid right now

The prediction axis is **one of three axes × 294 scenarios** in the lexicase pool
— a third of all test cases — selecting on a channel with r² ≈ 0. Individuals win
on prediction luck and carry mediocre control through. This is not an inert
axis, it is an actively harmful one.

---

#### E1 — ablate the head (do this first; cheap, and may be a win on its own)

`EnablePredictorHead` 0 vs 1, everything else fixed. **Not a neutral ablation** —
it returns a third of selection pressure from a dead channel to axes that mean
something, so the prior should be that tracking IMPROVES. Run on **M1**, which
climbs fast and reliably at pop 3000 / single longSequential / 16 winds
([[project_m1_basic_learner_validated]]) — hours, not a 27 h M2 bake.

If E1 wins, that is the finding, and everything below is optional.

#### E2 — is the target learnable AT ALL from these inputs?

Before building anything, settle whether Δspan is predictable from the 58-input
history window by ANY model. Fit an offline regressor (ridge / small MLP) on the
recorded per-tick CSV — no training run, no simulation, minutes of work. If an
offline model cannot beat persistence on Δspan either, **the task is impossible
as posed** and no architecture change rescues it. That is the cheapest possible
kill-shot and it should precede E3.

#### E3 — if E2 says the signal exists: fix the objective, then actuate

Only worth building if E2 clears. In order:
- **Score Δ, not level.** Target `span[t+h] − span[t]` against a persistence
  baseline of zero, so the objective cannot be satisfied by learning a constant.
- **Scale the output** into the target's domain so the usable range is not 5% of
  one output unit.
- **Then** actuate — the wrap's package (consume forecast as input; first-class
  axis; re-target across blindness). Feeding the forecast into the SAME recurrent
  net may be redundant (the hidden state that made the prediction also makes the
  control); the non-redundant signal is the **prediction ERROR**, a
  Kalman-innovation term saying "my model is wrong right now".

#### E4 — value head instead of world model (the bigger swing)

`span` is a *world-model* target: where the target will be, not whether that is
good. The objective is `stepPoints`. A **value head** predicting accumulated
future score has four advantages: prediction accuracy IS objective accuracy (no
proxy gap); the horizon becomes a discount rather than a guess; the Monte-Carlo
target costs **zero extra simulation** (`fitness_decomposition.cc` already
computes `stepPoints` per tick); and it **ports to M1 unchanged**, where a span
head cannot go at all because span needs beacons.

It also enables cheap lookahead: with a value head you do not simulate futures,
you **evaluate candidate actions** — k forward passes on a 16×16 RNN
(microseconds) rather than k physics steps. Rollout on the true FDM is ~10×
throughput (~6,270 sims/s → ~40 min/gen → ~3 weeks/800 gens) and is dead.

⚠️ Scoring actions rather than states is closer to Q than V, and this GA has no
actor-critic machinery. Biggest swing here, lowest confidence.

---

#### Honest caveats

- E1's measurements are from ONE elite of ONE run. Confirm on M1 before
  generalising.
- E4 is a proposal. 038 validated the critique; nothing has validated the fix.
- The r(Δ) ≈ 0 result says the CURRENT head learned nothing. It does not prove
  Δspan is unlearnable — that is exactly what E2 exists to decide.

**Prerequisite**: 040 closes and its t2 bake lands, so the predictor is judged
against a finished perception model rather than a moving one.

### [041 / BUG — found 2026-08-02] `prediction_score` is scored one tick out of alignment

**Found while chasing a renderer artefact** (the POV reticle swimming during
playback of a single scenario). The renderer bug is fixed; this one is real,
pre-existing since 038 US3, and touches TRAINING.

**The offset.** `inputdev_autoc.cpp:764` pushes the INITIAL aircraft state once
at scenario start, before any NN tick, while camera views only begin at tick 1.
So the two recorded arrays are **not** index-parallel despite the M8b comment
saying they are — 368 states against 367 camera views:

```
cameraViewList[j]  <->  aircraftStateList[j + 1]
```

**What it does and does NOT affect** (checked, not assumed):

| consumer | affected? | why |
|---|---|---|
| **Perception / NN inputs** | ❌ **no** | `trackerHelper_.tick(aircraftState, …)` computes from the LIVE state passed in. No index lookup anywhere in the sim path. |
| **score / energy / stability / streak** | ❌ **no** | derived from `aircraftStates` alone; never paired with camera views |
| `vis_frac` (avgVis diagnostic) | ⚠️ 1 tick | `fitness_decomposition.cc:256`, explicitly *"Observation-only; no effect on fitness or selection"*. 1 in ~370 — cosmetic |
| **`prediction_score`** | ✅ **YES** | `computeSpanPredictionError(aircraftStates, cameraViewList[i])` pairs `states[t]` with `cams[t]`. It is a **live lexicase axis** (`selection.cc:95`, `EnablePredictorHead=1`) |

So the NN's inputs and the main objective are clean. The **predictor axis** has
been scored against a target shifted one tick for its entire life: a +50 ms
prediction is compared against the span two ticks later, not one.

**Why this matters for [the 041 predictor work](#041-candidate--operator-direction-2026-08-01-make-the-predictor-earn-its-keep):**
E2 asks whether Δspan is learnable at all. That question is **confounded** until
this is fixed — the head has never been scored against the target it was
supposed to predict. A one-tick shift degrades a short-horizon prediction
badly, though it does not by itself explain the measured r(Δ) ≈ 0 at every
horizon *including* the closure rate. Fix it first, re-measure, then decide.

**Fix**: pair `cams[j]` with `states[j+1]` in `computeSpanPredictionError` (and
in the `vis_frac` lookup for tidiness). ⚠️ **This changes fitness**, so it must
NOT land while a bake is running — workers re-exec `build/autoc`, so rebuilding
mid-run would switch the objective underneath a live run.

**Trigger**: after the 040 t2 bake completes, and before 041 E2.

### [041 — BUG, DEFERRED ON PURPOSE 2026-08-02] The main M2 objective scores against the target one tick late

**Found by asking "where else does this shape appear?" after three display bugs
of the same kind** — which is the leverage argument in miniature: feature work
found the display bugs; a cross-cutting question found the objective one.

`fitness_decomposition.cc:205` pairs `targetTrajectoryList[i][stepIndex]` with
`aircraftStates[stepIndex]`, on the comment *"parallel-indexed … both pushed in
lockstep by the worker."* They are pushed in lockstep — but `inputdev_autoc.cpp:764`
pushes the INITIAL aircraft state once **before** the loop, unconditionally. So

```
states = 1 + N        targets = N        cameras = N     (368 / 367 / 367)
=> targets[j]  <->  states[j + 1]
```

**This is the objective, not a diagnostic.** `rabbitPosition` and
`targetPosition` feed `decomposeStepScore`, so the chase at tick *k* has been
scored against where the target was at tick *k+1* — since 030.

**Magnitude**: 50 ms at ~17 m/s ≈ **0.85 m**. Against `FitDistScaleBehind = 7 m`
that is ~12%; but the rabbit is *meant* to trail by `TrailDistance = 3.048 m`,
so the effective trail is ~2.2 m — a **28% reduction in the intended trail
distance**, systematically.

⚠️ **`CopiedTargetSample` carries NO timestamp**, so this is invisible in the
recorded data. Nothing short of reading the push sites could have caught it —
which is why it survived four features.

#### Why it is DEFERRED rather than fixed on the spot

Two earlier bugs from the same family were fixed immediately, because neither
touched the comparison basis (`prediction_score` was a broken axis with no
baseline value; the recorded camera pose was display-only). **This one IS the
objective**, and t2's whole purpose is the delta `t2 − t1`. Fixing mid-flight
would make those two runs differ in *two* ways — camera variation AND the
scoring basis — which is exactly the unattributable delta 038 taught us costs a
run.

The offset is systematic and **identical in t1, t2 and the 038-t9 baseline**, so
it shifts every absolute number while largely cancelling in the delta Phase 9
actually reports.

**Fix as the FIRST item of 041**, with a fresh t1′/t2′ baseline pair. 041 is
already a new-baseline moment for the predictor rework, so the correction rides
along free. **Reverse this call** if t2 returns an anomaly the 0.85 m offset
could explain — it is exactly the size that could flatter or damn close-in
tracking.

**Fix**: `targets[stepIndex - 1]`, guarded for `stepIndex >= 1`, plus the
zero-error-style assertion (see `SpanPrediction.PerfectPredictorScoresExactlyZero`).
**Proper indexing, not extra storage** (operator 2026-08-02) — and see the
grouped-record entry below for the structural version that makes the class
unrepresentable.

⚠️ **041 NEEDS A RESEARCH PHASE FIRST** (operator 2026-08-02). Not just this fix:
a deliberate dig through the code for this risk class *before* implementing
anything, since one cross-cutting question produced a finding that four features
of feature-shaped work had missed. Scope it as a phase with its own output — an
inventory of index-coupled contracts and structs with two lifetimes — rather
than folding it into the predictor work as a side task.

---

### [INFRA — 2026-08-02] Systematic scan: index-parallel collections with no assertion

**Operator direction**: "the backlog is a larger scan for this sort of
inconsistency across all functions."

**The failure class, stated precisely** so it can be scanned for rather than
stumbled on: *two collections written by different code paths, related by index,
with the invariant recorded only in a comment.* Four instances found in one
session, all with a comment asserting parallelism and **none with a test**:

| pair | verdict |
|---|---|
| `aircraftStateList` × `cameraViewList` → `prediction_score` | **was wrong** (fixed 2026-08-02) |
| `aircraftStateList` × `cameraViewList` → `vis_frac` | **was wrong** (fixed) |
| `aircraftStateList` × `targetTrajectoryList` → **the objective** | **wrong** (entry above) |
| recorded camera pose × projected bearings | **was wrong** (fixed) |

**What made them survive**: every one is invisible in the recorded data.
`CopiedTargetSample` has no timestamp; `CameraViewSample` has no tick index.
Nothing downstream could detect a shift, so the only witness was the code.

**Scan scope** — not just these arrays:
1. Every `std::vector` pair indexed by a shared loop variable across a
   producer/consumer boundary.
2. Every struct serving TWO lifetimes (RPC-only vs persisted). `ScenarioMetadata`
   in both roles cost a launch on 2026-08-02.
3. Every value duplicated in two places (`CameraConfig` default vs
   `hb1AirframeObstruction()` — that one HAS a test, and is the model to copy).
4. Every "compiled-in default vs recorded config" read — the self-describing-dmp
   item is the structural fix for a whole family.

**The test pattern that works**: construct data whose correct answer is EXACTLY
zero, then assert zero. Any pairing error becomes visible regardless of
tolerances, and a companion "shifted input must score visibly worse" test keeps
it honest. Both live in `fitness_decomposition_tests.cc` as a template.

**Operator direction 2026-08-02 on the fix shape**: *"stick with proper
indexing. Not additional storage."* A tick index per sample was one option and
is rejected — it pays permanent bytes on every tick of every dmp to paper over a
structural problem.

**The structural fix instead — retire the parallel lists.** Replace

```
aircraftStateList[i][k]   cameraViewList[i][k]   targetTrajectoryList[i][k]
```

with ONE list of grouped per-tick records:

```
tickList[i][k] = { state, cameraView, targetSample }
```

Then the invariant is not asserted, documented or tested — **it is
unrepresentable to get wrong**, because there is no second index to be off by.
It also deletes the whole failure class at the root rather than instrumenting
it: no tick index needed, no comment to trust, no test to remember. The extra
pre-loop initial state either joins the group or is stored once beside it, and
that choice becomes explicit instead of accidental.

Cost: a dmp schema change (greenfield, no version bump per project policy — old
dmps orphaned) plus every consumer. Real work, but bounded, and it is the last
time this class can bite.

### [SECONDARY / DOWN THE ROAD — operator 2026-08-03, downgraded 2026-08-04] Online craft identification: null the variations in flight, not in the weights

**Operator**: *"all craft variations have a chance to be nulled — and this isn't
during a training run — this is a dynamic online discovery of a craft
characteristic — the hardest of all, but common for people — trim, camera, etc —
all the variations that are craft related."*

Started as camera-boresight estimation; the general case is **every per-scenario
craft constant**, and the generalisation is the point.

#### What it changes

Today the variation set (034 US4's eight axes + 040's camera axes) exists to
force **robustness**: fixed weights must cope with the whole variation manifold
simultaneously. That is a large function to learn, and it is precisely the
search-space squeeze the operator named on 2026-08-03 — *"we have gradually been
increasing the complexity without additional weights to go along."*

The alternative is **adaptation instead of robustness**: identify the parameters
in flight, then control the identified plant. The weights no longer encode
"handle every craft" but "estimate this craft, then fly it" — a far smaller
function, learned with the same genome.

**It also reframes what the variations are for.** They stop being difficulty the
policy must tolerate and become **the training signal for the identifier**. A
variation the controller can null is no longer noise; it is what teaches it to
null.

#### Every axis is a constant mapping command → response

| axis | how a pilot reads it | plausible online estimator |
|---|---|---|
| `craftTrimDelta` (Cm_0) | "it wants to nose up" | running mean of pitch command in steady flight |
| camera boresight / roll | "the sight is off" | running mean of bearing (tail chase averages to centre) |
| `craftPitchEff` / `RollEff` | "controls feel heavy" | rate response ÷ commanded deflection |
| `craftThrustScale` | "it's a bit gutless" | accel ÷ commanded throttle |
| `craftDragDelta` | "it won't hold speed" | steady speed at known throttle |
| `craftServoSlew`, `thrustTau` | "it's laggy" | phase lag between command and response |
| `craftCGDelta` | "it's twitchy in pitch" | pitch-rate dynamics; couples with trim |

The shared structure: **a constant parameter, observable in the residual between
what was commanded and what happened.** First-order filters, not a bigger genome.

#### The hard parts, which are the interesting parts

1. **Observability requires excitation.** Trim shows up in steady flight; roll
   effectiveness needs roll input. A passive tail chase may never excite some
   modes. A pilot gets a control check on the ground — **the chase does not**,
   unless we deliberately spend early-episode ticks on exploratory inputs and
   pay for them in tracking. That trade is a real design decision, not a detail.
2. **Parameter coupling.** Drag and thrust both move steady speed; CG and trim
   both move pitch. Some pairs may be **unidentifiable separately** from passive
   observation, so the estimator should target the *combination* that affects
   control rather than the physical parameter.
3. **The bias/signal confound.** A persistent tracking lag looks identical to a
   boresight offset. Symmetric tail chase averages it out; asymmetric patterns
   would not.
4. **Ephemeral state is the FR-020a trap again.** Per-scenario, reset at every
   boundary, identically in both execution paths — the same trap that caught
   `AcquisitionState`, and `resetPerceptionState` is the one place that knows.
5. **Never adapt on no signal.** Do not integrate while blind, or the estimate
   drifts through exactly the dropouts it exists to survive.

#### ⚠️ The evidence that pointed here has been RETRACTED

This section previously argued from the t2 cap. **That cap was the
mount-inside-the-wing bug**, and t4 subsequently trained fine at ±10° with no
adaptation whatsoever. Nothing currently demonstrates that fixed weights cannot
absorb craft variation at the complexity we run today.

What survives is the *structural* argument: adaptation is cheaper than capacity,
and a GA pays for capacity twice (representation AND search). That is a reason to
expect this to matter eventually, not evidence that it matters now.

#### The experiment, if and when it is promoted

Raise variation until competence genuinely caps — **and prove the cap is not
plumbing** (a pinned diagnostic from generation 1 is the bug signature; a real
ceiling shows improvement then plateau). Then A/B the estimator against that
run. Starting from a demonstrated ceiling is what this entry lacked the first
time.

**Start explicit and start with ONE axis** (camera boresight, or trim — both are
running means and both are strongly observable in a tail chase). Feeding the NN
raw material and letting it learn the correction is more general and lands
straight back in the search-space problem.

Related: [[project_perception_control_two_loop]] — this is a third loop beside
perception and control, on a slower time constant.

### [SMALL — surfaced 2026-07-30 during 040 t1 launch] `tracker_dmp_inspect` bucket + .zst warts

Two papercuts hit while spot-checking the first gen of an M2 run. Neither is
load-bearing; both cost a few minutes each time.

1. **Wrong bucket for M2 dmps.** Given a tracker ini it resolves the fetch
   bucket from `TrackerSourceBucket` (`autoc-m1`, the M1 *source*) rather than
   `S3Bucket` (`autoc-m2`, where this run's own output lands), so inspecting a
   run's own gen dmp by S3 key fails with "specified key does not exist" — while
   the banner confusingly prints `bucket: autoc-m2`. It is reusing the
   source-dmp loader path. Workaround: `aws s3 cp` locally first.
2. **A local `.zst` path is not decompressed.** Passing a local `*.dmp.zst`
   feeds compressed bytes straight to cereal, which reads a garbage vector
   length and dies with `std::bad_alloc` — and the error text then blames a
   *version mismatch*, sending you hunting a schema bug that is not there. The
   S3 path decompresses; the local path does not. Workaround: `zstd -d` first.

**Fix**: honour `S3Bucket` when the key is not the configured source, and sniff
the zstd magic on local files. The misleading version-mismatch message is
arguably the worse of the two — it points at the wrong cause.

### [NARROWED 2026-07-30 — post-028] Renderer scrubbing with hidden state

> **Scope note**: 040 T065a shipped scrub over RECORDED playback, where a step
> back is an index decrement and no hidden state is involved. What remains open
> here is only the harder case this entry was really about — scrubbing a view
> that RE-RUNS the network, where RNN hidden state cannot be rewound.

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

### [FUTURE FEATURE — depends on how M2 goes] Two-sim co-evaluation — live M1 target + evolving M2 chase (shared air)

- **Idea (operator 2026-06-16)**: instead of M2 chasing a *recorded* M1 trajectory (forced playback
  under the source's fixed, different wind), run **two live sims per scenario** — a **target** flying
  the trained **M1 NN** (a robust generalist) and the **M2 chase** flying the evolving network — both
  in the **same air mass**. The target adapts to the scenario's wind live, so target + chase share one
  wind by construction; the playback-vs-wind-mismatch question dissolves, and the recorded-trajectory
  dependency goes away.
- **Evidence motivating it (2026-06-16 wind-mismatch study, `src/analytics/wind_study.py`)**: M1-source
  vs M2-chase steady wind-DIRECTION offsets diverge ~38° mean (35% of scenarios >45°), yet that
  mismatch is **uncorrelated** with per-scenario score (corr +0.03), tracking (−0.07), or completion
  (the 20 incomplete avg 36° ≈ overall 35°). So reproducing M1's wind (the "M2 sim playback parity"
  item below) looks **low-value** — co-sim makes wind shared regardless. **Caveat**: the study only
  measured the *steady direction* offset; gusts/thermals are unrecorded (`wind_velocity`=0 — see the
  recording-gap item in Infrastructure) — confirm before fully discounting playback parity.
- **CRRCSim hook**: CRRCSim has a robot-craft notion (mod_robots / `RobotProgrammable` — see the
  "[030 v1] Live two-aircraft display" item + [reference_crrcsim_mod_robots](../../.claude/projects/-home-gmcnutt-autoc/memory/reference_crrcsim_mod_robots.md)).
  Here the target would be a second **FDM-driven** craft running the M1 NN, not just a programmatic
  pose stream.
- **Cost/risk**: two FDMs + two NN forward passes per scenario tick → ~2× eval cost; determinism must
  hold for both crafts; the M1 NN is fixed (constant per scenario, not evolving). Big lift, long-term.
- **Trigger**: depending on how the current playback-M2 approach goes — if it plateaus too low, or the
  gust component turns out to matter once `wind_velocity` is recorded. **Supersedes** the "M2 sim
  playback parity" bullet if it lands.

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

### [STUDY — M2 fitness gradient, 2026-06-19] Negative reward when the chase gets *ahead* of the target

> **→ 038 US5 (2026-06-19)**: pulled into [specs/038-accurate-m2/spec.md](038-accurate-m2/spec.md) as
> US5 + FR-009 (signed-ahead reward, ini-switched). Kept here as the originating study notes.

- **What**: study whether the tracking fitness should go genuinely **negative** when the chase
  overshoots *past* the trail point and gets **ahead of** the target — not merely score lower. Today
  the cone scoring is asymmetric (`FitDistScaleAhead=2.0` vs `FitDistScaleBehind=7.0`): being ahead
  decays the (positive) reward faster than being behind, but it never crosses zero into a *penalty*.
  Operator intuition (2026-06-19): "if you get ahead of chase the gradient actually goes negative" —
  i.e. a region of the encounter where the *right* thing is for the controller to feel an actively
  repelling gradient (you've blown past the rabbit / it's about to overrun you), not just a smaller
  carrot.
- **Why it matters now**: connects to the **t12 displacement finding** (038 T001) — the hull penalty
  pushed the chase to hold a large standoff and bail OOB rather than risk getting close/ahead. A
  fitness that explicitly *repels* the ahead-of-target geometry might shape the same safety behavior
  through the reward gradient instead of (or alongside) the crash multiplier, and might reduce the
  OOB flyaway by giving the controller a gradient to *follow back* into the trail position rather than
  a cliff to flee from. Also relevant to the from-behind-overshoot blind spot (spec Edge Cases: a
  forward camera can't perceive the overrun; a reward-shaping term is the only lever there).
- **Open questions**: where exactly zero-crossing should sit (at the target? at the trail point + ε?);
  whether a negative region destabilizes lexicase (negative + positive scenario scores mixing under
  epsilon); interaction with the ahead/behind scale asymmetry (is this just `FitDistScaleAhead` going
  steep enough to cross zero, or a separate signed term?); unitless/optical-only consistency (FR-008).
- **Source**: operator review of t12 (037 hull-penalty run), 2026-06-19. A reward-shaping sibling to
  the crash-penalty work; evaluate as an alternative/complement to the t13 dual-penalty exchange-rate.

### [DEFERRED — 031-fed] CEP realism — evolve the sim beacon-CEP model

- **What**: the sim CEP (beacon localization uncertainty fed to the tracker NN) is a linear off-axis
  geometric placeholder (`src/eval/camera_projection.cc` step 5). The **real** CEP is a classic
  position-uncertainty (circular error probable) the camera DSP estimates, magnitude driven by
  Gold-code decode/reacquisition confidence (partial decode ⇒ high CEP) + intermittency/occlusion +
  apparent crossing-rate over the multi-frame code-acquisition window. Per-frame blur is low (global
  shutter @~480 fps) but the ~150 ms code period is the smear window — NOT the frame rate.
- **When**: revisit CEP modeling when 031's optical chain produces real footage — BEFORE any
  tracker-NN retraining against real camera output. CEP is both an NN input AND the visibility gate.
- **Detail**: sensor inventory + FR-011 (position- vs signal-quality) in
  [specs/038-accurate-m2/spec.md](038-accurate-m2/spec.md).
- **Note**: migrated here from machine-local `~/.claude` memory (doesn't travel cross-machine) per
  Constitution X + the portable-agent-memory item.

### [031-fed] CEP — physical model (two-component) for a cleaner tracker-mode first pass

Concrete physical CEP model derived from the **031 single-PD acquisition study**
([`acquisition-sim/`](031-beacon-camera/acquisition-sim/acquisition-results.md) +
[`acquisition-research-plan.md`](031-beacon-camera/acquisition-research-plan.md)).
Replaces the linear off-axis placeholder with a CEP grounded in the code physics — usable
**before real footage exists**. Intended pickup: **038** (or a tracker-mode follow-on) to give
M2 a cleaner first-pass CEP. Builds on the item above.

**CEP = combine(temporal decode-confidence, spatial apparent-motion):**

1. **Temporal / decode-confidence** — measurable on the single-PD bench *now*:
   - Driven by the Gold-code correlation **margin** `M = (peak − max_sidelobe)`, normalized by
     integrated chips / noise floor.
   - **Lock ladder**: `none → tentative (M > low) → confirmed (M > high, sustained ≥2 code periods)` —
     the continuous, unsync'd, no-gap stream enables multi-period confirmation.
   - **Inflated by**: dropout/erasure rate, partial-code fraction (early in a period), reacquisition.
   - **Maps to CEP**: strong margin → small CEP; near-threshold → large/unstable CEP; lost → the
     existing sentinel CEP (visibility gate).

2. **Spatial / apparent-motion** — camera phase only (needs the 2D array):
   - Blob displacement across pixels during the **~75 ms decode window** (200 Hz chip / 480 fps —
     *updated from the 150 ms in the item above, which assumed the old 100 Hz / 240 fps*) → a spatial σ.
     Worse at high body rate / close range / wide FOV (exactly where beacons also merge toward one pixel).

**Quantitative shape to seed the CEP curve** (from the 031 sim):
- CEP collapses with SNR margin: ≤0 dB/chip → lock <50 % over one period (large CEP); +6 dB/chip →
  ~99 % in ~50 ms (small CEP).
- **Erasure-aware**: a *wrong* chip costs ~2× a *missing* one → weight erasures below flips when
  inflating CEP (mark fades/saturation as erasures, not guesses).
- **Builds over time**: CEP starts high (tentative) and decreases as the code integrates / confirms
  across periods — full code 75 ms = 1.5 ticks @ 20 Hz, early lock ~55 ms.
- **Two-beacon CDMA**: an equal-amplitude second beacon sharing the detector/pixel costs ~1 SNR tier →
  higher CEP when beacons merge.

**First-pass implementation (038 / follow-on):**
- Replace the linear placeholder (`src/eval/camera_projection.cc` step 5) with a **decode-confidence
  CEP** — a monotonic map from a synthetic per-beacon "correlation-margin" proxy (derivable in sim from
  range / aspect / occlusion) to CEP, shaped to the 031 sim curves. Cleaner than linear, grounded in the
  code physics, no footage required.
- Add the **apparent-motion spatial σ** when the camera lands (calibrated from real footage per the
  item above). `CEP_total = combine(temporal, σ_motion)`.
- **Calibration source**: the 031 single-PD field bench (Exp A/B) measures real margin / dropout /
  time-to-confirm vs range / aspect / sun → the temporal-CEP calibration; apparent-motion σ comes later
  with the camera. `(x, y, CEP)` NN interface unchanged throughout.

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

### [037 close-out, 2026-06-21] Housekeeping carried forward from 037
- **svTau cleanup** (P-O8): `svTau` kept only to preserve draw order — doesn't help; remove the dead path.
- **Time-denominate the rate-dependent reports** (P-O11): raw tick-denominated streak metrics read 2× at
  20 Hz; partly addressed by `--tick-sec` in the analytics, but the streak/avgMaxStreak fields should be
  surfaced in seconds (or pctInStreak) consistently. Reporting hygiene.
- **Type-domain grep audit** (T028/T046, Principle VI): grep `src/eval/ src/nn/` for float/double drift on
  the 037-touched paths; confirm `gp_scalar`/`gp_fitness` convention. Cleanup, non-blocking.

### `wind_velocity` not recorded in the dmp (honest-recording gap)

- **Found 2026-06-16** (wind-study): `AircraftState::wind_velocity` is serialized but **never set** in
  the crrcsim→AircraftState record path — it is **zero in every dmp** (M1 + M2). The actual wind lives
  only inside crrcsim (`crrc_builtin_scenery.cpp` `flWindVel`/`effectiveWindDir` + gusts/thermals).
- **Impact**: you can't audit the wind a craft actually flew through from the dmp. This blocked the
  full (gusty) M1↔M2 wind-mismatch comparison — `src/analytics/wind_study.py` could only use the
  steady `windDirectionOffset` from `ScenarioMetadata`, not the realized gusty wind. Violates
  honest-recording ([feedback_honest_dmp_recording](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_honest_dmp_recording.md))
  for an environment input the controller implicitly experiences.
- **Fix**: copy crrcsim's per-tick wind into `AircraftState::setWindVelocity()` at record time (getter
  + setter already exist; `dmp_dump.cc` already emits `wN,wE,wD` columns — currently zeros). Then
  re-run `wind_study.py` against the *actual* experienced wind to settle the wind-vs-tracking question
  (and the co-sim-vs-playback-parity decision).
- **Trigger**: before any wind-parity / two-sim co-eval decision that needs the true wind, or next time
  the record path is touched.
- **→ 038 pre-work (2026-06-16)**: a recording change → fold into the 038 clean-slate dmp break, with
  simTimeMsec stamping + self-describing dmp. Then re-run `wind_study.py` against the real gusty wind.

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

> **→ 038 pre-work (2026-06-16)**: fold the stamp fix into 038 pre-work. 038 re-bakes M1/M2 anyway
> (hull-crash fitness change + camera variations + the source-side spike) — that IS the "dmp break is
> acceptable" moment this was waiting for. Bonus: `simTimeMsec` feeds the time-based NN history-lag
> selection + the `span_rate` gap denominator, so today's ±1 ms jitter adds noise to the rate inputs;
> exact 50 ms stamping cleans it. After the fix + a clean re-baked source, the M2 source-spacing check
> reverts to a strict single-gap test. **NOTE**: CRRCSim **submodule** change (pointer-bump-first),
> **determinism-affecting → retrain-from-scratch** — do it at the 038 clean-slate, NOT mid-bake (would
> perturb the live t11).

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

> **→ 038 P0-B — FULL version now viable (2026-06-16)**: 038 may break the dmp contract for a clean
> re-bake (operator), so the full `EvalResults` config-block serialization can land here — not just
> the renderer-config-hygiene slice. Bundle with the other clean-slate contract changes (simTimeMsec
> stamping, wind_velocity recording) so there's one dmp break, not several.

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
  2. **M2 sim playback parity with M1 — source replayed *in its own original environment*** (clarified 2026-05-27): the load-bearing experiment after the basin work.
     > **UPDATE 2026-06-16 (wind-study, may downgrade this item)**: a per-scenario M1-vs-M2
     > wind-direction comparison (`src/analytics/wind_study.py`) found the steady wind-direction
     > offsets diverge a lot (~38° mean, 35% > 45°) but are **uncorrelated** with M2 per-scenario score
     > (+0.03) or tracking (−0.07) — so steady-wind parity looks low-value. The **two-sim co-evaluation**
     > feature (Future Features) is the more compelling alternative. Caveat: only the steady direction
     > offset was measurable; the gusty `wind_velocity` is unrecorded (Infrastructure gap) — record it
     > and re-test before finally ranking this item. The key design point: an M1 source trajectory was produced under a specific joint-PRNG scenario = (path_index, wind_seed, entry_pose, rabbit-speed profile). The source path is *shaped by* that wind — it crabbed, drifted, and adjusted throttle for those exact gusts at those exact ticks. For realistic M2 (tracker) training, the chase aircraft must fly through the **same air mass at the same time** — i.e., the M2 sim replays the source's *original variation scenario* so the chase feels the identical wind/conditions while pursuing. Otherwise there's a physics mismatch: the source moved as if wind-blown, but the chase tracks it in different (or calm) air, which is not what happens when a real chase pursues a real target through one shared air mass.
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

### [BACKLOG 038] Eval at forced variation scale — `EvalVariationScaleOverride` knob (robustness vs repeatability)

- **Surfaced 2026-06-19** (operator) from the t14 curriculum-ramped crash penalty. Today **all** eval
  runs (`EvaluateMode=1`) pin the variation scale to the value **recorded in the saved genome**
  (`gEvalVariationScaleOverride = genome.variation_scale`, [src/autoc.cc](../src/autoc.cc) ~L1043;
  `computeVariationScale()` returns it). That is deliberate — it reproduces the exact training
  conditions for the **bitwise determinism gate** (repeatability). The cost: eval can only ever test a
  controller at the scale it was *saved* at.
- **The gap**: a run that **exits early** (before the ramp reaches 1.0 — ~gen 761 for the 800-gen
  tracker config, `VariationRampStep=40`) saves an elite at **partial** scale, so its eval reproduces
  that partial scale — never full variation. There is no way today to ask *"how does this controller
  hold up at full variation?"*
- **The knob**: an ini option (e.g. `EvalVariationScaleOverride = 1.0`, default = -1 → use the recorded
  scale) that forces the eval scale, decoupling it from the genome's recorded value. Forced-scale eval
  is **NOT bitwise-reproducible vs training** (different scale → different scenarios) — it's a *robustness
  probe*, run **instead of** the repeatability gate, not alongside it. Two distinct eval purposes:
  (1) repeatability (recorded scale, today's behavior, the bitwise gate); (2) robustness at a chosen
  scale (forced, new).
- **The fairness nuance (operator 2026-06-19)**: only meaningful for a **mature** elite. Running a
  **gen-7** elite (trained at scale≈0) at full variation is unfair — wildly OOD for a controller that
  never saw variation. A **gen-450** elite (trained under substantial variation) at full is a reasonable
  robustness test. Hence a *knob* you opt into per-run, not a default — the operator judges whether the
  elite is mature enough to warrant the forced-full eval.
- **Scope**: small — gate the `gEvalVariationScaleOverride` assignment behind the config value (use the
  override if ≥0, else fall back to the recorded `genome.variation_scale`). Fits 038's reporting/eval
  cluster. **Hold code** (t14 is running; clean separate feature).

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
- **Per-tick scrub controls — ✅ CLOSED 2026-07-30 by 040 T065a.** Unparked into 040
  rather than left deferred, because the US4 checkpoint ("does dropout and
  reacquisition look physical?") is NOT ANSWERABLE at realtime: warm relock is
  154 ms (3 ticks) and cold acquire 308 ms (6 ticks), so the behaviour-defining
  distinction plays out in under a third of a second. Shipped in
  `tools/renderer.{cc,h}`: `Space` play/pause (rapid-finish moved to `F`),
  `.`/`,` ±1 tick, `>`/`<` ±10, `Home`/`End`, `[`/`]` jump to previous/next
  perception event (lock-state transition or hull strike), plus a tick-index +
  pause readout on the stopwatch. Tick length is DERIVED from consecutive
  recorded timestamps, not assumed.
- The streak/multiplier overlay below is still open.
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

### [CLOSED 2026-07-30 by 040 T065a] Renderer scrub controls (was 030 T057, rolled-in from earlier 028 entry)

- **Closed**: shipped as the 040 playback transport. The "not load-bearing"
  judgement was right for 030's smoke test and wrong once US4 put 3-tick and
  6-tick events on screen — see the entry above for the shipped bindings.
- The old note that backstep is risky does not apply: playback is RECORDED data,
  so a step back is an index decrement. The hidden-state problem in the entry
  below only bites a renderer that re-runs the NN, which this one does not.

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
