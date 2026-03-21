# Feature 017: Visual Target Tracking — Beacon-Guided Pursuit

## Vision

Train an NN that can follow another aircraft using only camera imagery of
wing-tip LED beacons. The current GPS-based path follower (015) becomes the
lead aircraft autopilot. A new vision-based NN learns to pursue from projected
beacon positions on a simulated camera — no GPS target position, no data link.

## Background

The 015 NN follows a virtual rabbit — a point moving along a precomputed path,
with position known exactly in NED coordinates. This is "god-view" pursuit. Real
formation flight requires the follower to derive the leader's state from what
it can *see*. The step from NED coordinates to camera pixels is the core challenge.

The current 015 NN is valuable as infrastructure:
- It can fly realistic lead aircraft trajectories in sim (with attitude, bank,
  speed variation — not just geometric paths)
- Its training infrastructure (neuroevolution, lexicase, scenario tables) carries forward
- Its eval suite validates the vision NN against GPS ground truth

## Approach

### Phase 1: Recorded Lead Trajectories (017-phase1)

**Goal**: Use the trained 015 NN to generate a library of realistic lead
aircraft flight recordings — full state trajectories (position, attitude,
velocity, control surfaces) at 10Hz from simulated flights.

**What this produces**: Today's "yellow tape" (geometric rabbit path) becomes
tomorrow's "red tape" (realistic flight trajectory with dynamics). These
recordings include:
- NED position over time (the ground truth for scoring)
- Quaternion attitude (bank in turns, pitch changes, sideslip)
- Wing-tip beacon positions in world NED (derived from attitude + known geometry)
- Velocity, throttle, control surface state

**Method**:
- Run 015 NN in crrcsim across the eval suite scenarios (aeroStandard, random,
  progressive, etc.)
- Record full EvalResults state vectors at each sim step (already captured in data.dat)
- Extract as time-series files: one recording per scenario
- Each recording is a lead trajectory that a chase aircraft can follow

**Scale**: 294 training scenarios × ~13 seconds each = ~60 minutes of lead data.
Add wind/speed/path variations for diversity.

### Phase 2: Vision Sensor Simulation (017-phase2)

**Goal**: Given a chase aircraft's position/attitude and a lead trajectory
recording, compute what the chase camera sees — specifically, the projected
pixel positions of wing-tip LED beacons.

**Camera model**:
- Two cameras on chase aircraft (options TBD during design):
  - *Wing-tip stereo pair*: depth from disparity, ~90° FOV each, overlapping center
  - *Wide + narrow pair*: wide (180° fisheye) for search, narrow (60°) for track
  - *Single wide*: simplest, 180° FOV, range from inter-beacon separation only
- Pinhole projection (or equidistant fisheye for wide FOV)
- Bandpass filter model: beacons are bright points, background is dark
- Noise model: pixel jitter (±2px), detection dropout (5%), false positives

**Beacon model**:
- Two beacons per lead aircraft: port wing-tip (red), starboard wing-tip (green)
- Known separation (wing span, e.g. 1.2m) — this is the range cue
- World positions computed from lead's NED position + attitude quaternion + wing geometry

**Projection pipeline per sim step**:
1. Lead wing-tip positions in world NED (from recorded trajectory + aircraft geometry)
2. Transform to chase camera frame (chase position + attitude, camera mount offset)
3. Project to pixel coordinates (u, v) per beacon per camera
4. Apply detection model (in-FOV check, SNR model, dropout)

**Output**: Per-step sensor vector, e.g.:
```
[cam0_L_u, cam0_L_v, cam0_R_u, cam0_R_v, cam0_L_det, cam0_R_det,
 cam1_L_u, cam1_L_v, cam1_R_u, cam1_R_v, cam1_L_det, cam1_R_det]
```

**Key insight**: Beacon pixel positions implicitly encode:
- Bearing to target (pixel center offset)
- Range (inter-beacon pixel separation — larger = closer)
- Target bank angle (beacon pair tilt in image)
- Closure rate (frame-to-frame pixel motion)

The NN doesn't need explicit range computation — it learns the geometry.

**GPU acceleration**: The DGX Spark can parallelize projection across thousands
of scenarios. Each projection is a matrix multiply + clip — trivially parallel.
The bottleneck is scenario throughput, not per-frame compute.

### Phase 3: Vision NN Training (017-phase3)

**Goal**: Train an NN that takes beacon pixel positions (from Phase 2 sensor)
and produces 3-axis control output (pitch, roll, throttle) to pursue the lead.

**Architecture (beacon-coordinate approach)**:
- Replace 015 inputs [0-17] (dPhi/dTheta/dist temporal history) with beacon
  pixel coordinates in temporal history
- Per time slot: 4-8 values (per camera: L_u, L_v, R_u, R_v) + detection flags
- 6 temporal slots × values per slot = beacon inputs
- Plus: own-ship quaternion, airspeed, alpha, beta, command feedback (retained from 015)
- Output: same 3-axis [-1,1] control (pitch, roll, throttle)

**Training method**: Same neuroevolution + lexicase framework as 015.
- Population: 3000
- Scenarios: lead trajectory recordings × wind variations × entry variations
- Fitness: completion (did it track?), distance (GPS ground truth, for scoring only —
  not visible to NN), attitude, smoothness
- The NN sees only beacon pixels + own-ship state; GPS distance is used for
  fitness computation but never as NN input

**Search→Track behavior**:
- Chase aircraft starts in holding orbit above origin (or random position)
- Must detect beacons (any beacon pixel in FOV with sufficient SNR)
- Transition to pursuit when detected
- Track and maintain formation

**This is harder than 015** because:
- No direct range information (must infer from pixel geometry)
- Bearing precision depends on camera resolution and target distance
- Occlusion when target banks hard (one beacon hidden by fuselage)
- Detection dropout at long range (beacons below pixel threshold)
- 180° camera means barrel distortion at edges — pixel motion is nonlinear

**What makes it tractable**:
- Deterministic sim with known ground truth — same training infrastructure
- Low-dimensional sensor (beacon pixels, not raw images) — same NN scale
- 015 NN provides proven control foundation — can warm-start or transfer

### Phase 4: Flight Hardware (017-phase4) — future

**Not in scope for this feature.** Phase 4 planning depends on Phase 3 results:
- How much compute does beacon detection need? (blob detection vs learned detector)
- Can it fit on tiny FPGA (iCE40, ECP5) or needs RPi/Coral?
- Camera specs: resolution, FOV, frame rate, filter requirements
- Latency budget: detection + NN eval must fit in 100ms loop
- Weight/power budget for the follow aircraft

**Flight sequence** (eventual):
1. Lead aircraft flies with wing-tip LED beacons broadcasting
2. Follow aircraft in holding orbit, vision NN in search mode
3. Beacons detected → transition to pursuit
4. Track and follow using vision-only sensor input
5. Score against GPS ground truth (post-flight, not real-time)

## Milestone 1 (this feature): Sim-to-Sim Pursuit

**The first concrete deliverable**: A vision NN, trained entirely in simulation,
that can follow a simulated lead aircraft using only projected beacon pixel
positions. Validated against GPS ground truth in the eval suite.

Success = the vision NN achieves comparable tracking performance to the 015
GPS-based NN (>90% completion, <15m avg distance) on the standard eval scenarios,
using only beacon pixels as target input.

## Relationship to Existing Features

| Feature | Role in 017 |
|---------|-------------|
| 015 (NN training) | Provides lead aircraft autopilot + training infrastructure |
| 015 eval suite | Generates lead trajectory recordings (Phase 1) |
| 016 (eval automation) | Extended for vision metrics + beacon rendering validation |
| crrcsim | Runs lead aircraft sim; extended with beacon projection (Phase 2) |

## Technical Considerations

### Sensor dimensionality

With beacon coordinates (not raw pixels), the NN input stays manageable:
- 2 cameras × 2 beacons × 2 coords × 6 time slots = 48 beacon inputs
- Plus detection flags: 2 cameras × 2 beacons × 6 slots = 24
- Plus own-ship state: ~11 (quaternion, airspeed, alpha, beta, cmd feedback)
- Total: ~83 inputs → 32 → 16 → 3 (~3K weights)

This is 5× larger than 015 (643 weights) but well within neuroevolution reach
at pop=3000. No CNN, no GPU inference needed on embedded.

### If raw pixels are needed later

If beacon-coordinate approach doesn't generalize (e.g. can't handle partial
occlusion, variable lighting, non-cooperative targets), the path forward is:
- Small CNN beacon detector (80×60 filtered image → beacon coordinates)
- Trained supervised on sim renderings
- Runs on FPGA (iCE40 UP5K: 5K LUTs, 1Mbit RAM) or dedicated vision MCU
- Feeds detected coordinates to the pursuit NN (same as beacon-coordinate path)
- This separates perception (CNN) from control (pursuit NN) cleanly

### Training scale and GPU requirement

BIG-3 (015) was ~352M sim evaluations (pop=3000 × 294 scenarios × 400 gens).
The vision NN at ~3K weights needs at minimum the same scale, likely 2-5× for
the larger search space. That's 700M–1.75B evaluations, each with ~130 steps.

Beacon projection adds ~115M projection calls per generation. At 400 gens that's
~46B projections total. CPU at ~10M projections/sec = ~77 minutes of projection
time alone per generation — **unacceptable**.

**GPU-native evaluation is a blocking dependency** for Phase 3 training at scale.
See [BACKLOG: GPU-Native Evaluation](../BACKLOG.md). Options:
- **(a) GPU physics sim**: Port FDM to CUDA/Vulkan compute. Maximum throughput
  but major engineering effort. May need more than DGX Spark GB10.
- **(b) Lightweight GPU FDM + projection**: Simplified 6DOF (no rendering, no
  OpenGL) fused with beacon projection on GPU. Best balance.
- **(c) Hybrid**: crrcsim for physics (CPU, multi-threaded), GPU batch for
  beacon projection. Least effort but CPU physics becomes bottleneck.

DGX Spark (GB10, 128GB unified memory, 20 Grace cores + Blackwell GPU) is likely
memory-bandwidth-limited on CPU — millions of small matrix ops with poor cache
locality will saturate LPDDR5X before the 20 cores are compute-bound. This
reinforces the GPU path: the Blackwell GPU has ~200GB/s+ memory bandwidth and
massive parallelism for exactly this workload shape (batch small matrix ops).
Option (c) may still CPU-bottleneck on physics; Options (a/b) push everything
to GPU. To be determined during Phase 2 prototyping — may need larger GPU
instance if Spark GPU isn't sufficient at full training scale.

## Risk Assessment

| Risk | Impact | Mitigation |
|------|--------|------------|
| Beacon pixels insufficient for range | Can't maintain formation distance | Stereo camera pair; or accept range ambiguity, use closure rate |
| Neuroevolution too slow for 3K weights | Training doesn't converge in reasonable time | Warm-start from 015 weights (shared control outputs); or CMA-ES |
| Sim beacon projection doesn't transfer to real | Real detection differs | Noise model in Phase 2, domain randomization, sim-to-real tuning |
| Search phase too hard | NN can't find target in 180° FOV | Start with target in FOV (reduce search problem); curriculum ramp |
| Camera occlusion during bank | Lose both beacons | Temporal history carries through brief dropouts; train with dropout |

## Non-Goals

- Non-cooperative target tracking (no beacons) — future feature
- Multi-aircraft formation (>2 aircraft) — future feature
- Raw-pixel CNN on embedded — future, depends on Phase 3 results
- Full autonomous mission (takeoff, navigate, land) — separate concern
- Real-time GPS fusion with vision — vision-only for this feature

## Open Design Questions

1. **Camera configuration**: stereo wing-tip pair vs wide+narrow vs single wide?
   Affects NN input size, depth cue quality, search capability.
2. **Beacon separation**: using real aircraft wing span (~1.2m) or exaggerated
   for sim training? Separation sets minimum detection range.
3. **FOV model**: equidistant fisheye for 180° or rectilinear with limited FOV?
   Fisheye has nonlinear pixel→angle mapping the NN must learn.
4. **Search mode**: should the NN learn search behavior (scanning) or start with
   target in FOV? Curriculum: start easy (target ahead), ramp to full search.
5. **Transfer from 015**: can we initialize the control-output weights from 015
   and only train the vision-input weights? Or full retrain?
