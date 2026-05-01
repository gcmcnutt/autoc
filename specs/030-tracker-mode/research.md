# 029 tracker-mode — plan-phase research

Resolves R1–R7 from the 029 plan. Each section ends with a concrete recommendation
the plan can write phase-by-phase tasks against.

---

## R1 — crrcsim playback file format (exact byte layout)

**Decision**: Use the existing `.crrclog` binary record format unchanged for tracker-mode
target playback files. The file is a tagged binary stream after a UTF-8 ASCII XML header.
Concrete layout below — sufficient for a Python tool to write a valid file from scratch.

**Wire format** (canonical refs: `crrcsim/documentation/record_playback/design.txt:31-50`,
`crrcsim/src/mod_robots/robotfile.cpp:24-71` reader, `crrcsim/src/record.cpp:40-131`
writer, `crrcsim/src/mod_robots/fdm_playback.cpp:39-127` consumer):

1. **XML header** — ASCII, written by `SimpleXMLTransfer::print(out, 0)`. Root element
   name MUST be `CRRCSim_record` (enforced by
   `fdm_playback.cpp:143-146`). Header is followed by a single trailing `\n`
   byte (consumer skips it via `infile.read(buf, 1)` at `fdm_playback.cpp:141`).
   Header carries scenery/airplane/wind/game-mode XML; for tracker-mode targets the
   `airplane.file` and `airplane.graphics` attributes are read by `Robots::AddRobot`
   (`crrcsim/src/robots.cpp:43-76`) to load the visual model.

2. **Tagged record stream** — each record begins with one `char` tag byte:
   - `0x00`: position/attitude record. Fixed body of **20 bytes**:
     - `double timestep` (8 bytes, native LE) — seconds since previous 0x00 record;
       writer stores `dt*multiloop` (`record.cpp:122`).
     - 3 × `float position` (12 bytes total, native LE float) — `pos(0..2)` in
       crrcsim world frame (ft, NED-like with X-North, Y-East, Z-Down per crrcsim
       convention; the consumer doesn't reinterpret).
     - 3 × `int16 euler` (6 bytes, native LE signed) — phi, theta, psi each scaled
       by `ROBOT_EULER_TO_INT16 = 32767/(2π)` (`robotfile.h:28`,
       `record.cpp:127-129`). Reader divides by the same constant
       (`fdm_playback.cpp:107-109`).
   - `0x01`: control inputs (animated flaps/gear). **Not used by playback consumer**
     — the FDM reader's switch (`fdm_playback.cpp:46-94`) has no `0x01` case and
     would error. Safe to omit entirely from tracker-mode files.
   - `0x02`: marker. Body is `int32 marker_id` (4 bytes). Used for F3F-style sync
     points; tracker mode does not need them, omit entirely. Reader passes through
     `eF3F_*` state machine (`fdm_playback.cpp:46-67`); ignoring all markers is
     fine.
   - `0x03`: variable-length XML record. ASCII XML printed via
     `SimpleXMLTransfer::print(out, 0)` followed by a trailing `\n`. The recorder
     uses these to inject the description string at end-of-recording
     (`record.cpp:67-83`). Tracker mode can use them to embed scenario provenance
     (source run id, source gen, source scenario index, joint-PRNG params) so
     converter output is self-describing.

3. **Endianness** — code uses native memcpy via `(char*)&val`. crrcsim only ever
   ran on little-endian targets (x86, ARM); both writer and reader assume LE.
   Documented as a TODO in `robotfile.h:36`. The Python converter MUST emit
   little-endian (`struct.pack('<d3f3h', ...)` for one 0x00 record body).

4. **Position-record cadence** — recorder writes one 0x00 record per FDM tick
   (`record.cpp:115-130` invoked from main loop). `timestep` is the wall-clock
   step; consumer interpolates between successive 0x00 records using `timestep`
   (`fdm_playback.cpp:117-125`). Tracker mode's playback library should use a
   constant timestep matching crrcsim's autoc tick (`Global::dt` × multiloop).

**Rationale**: Reusing the format means zero crrcsim-side changes for the reader path
and full visualization-pipeline parity (the existing 3D viewer renders robots from
.crrclog files today). The format predates 029 and is deterministic; converter just
needs to emit the same bytes the live recorder would have written.

**Alternatives considered**:
- Custom JSON / cereal format. **Rejected** — would require new reader code in
  crrcsim and a new RobotBase subclass; existing `CRRC_AirplaneSim_Playback`
  already does the work.
- Use cereal-binary directly (matches autoc dump format). **Rejected** — crrcsim
  doesn't link cereal; would force cereal into mod_robots build.

**File:line citations**:
- `crrcsim/documentation/record_playback/design.txt:31-50` (canonical spec)
- `crrcsim/src/mod_robots/robotfile.h:28-124` (read/write helpers + scale const)
- `crrcsim/src/mod_robots/fdm_playback.cpp:39-127, 130-148` (reader + header check)
- `crrcsim/src/record.cpp:40-131` (writer)

---

## R2 — dmp file deserialization access from a converter tool

**Decision**: Link the converter tool (call it `tools/dmp_to_playback.cc`) against the
existing `autoc_common` static library wholesale. No carve-out is justified — the
relevant symbols (`EvalResults`, `ScenarioMetadata`, `gp_vec3`/`gp_quat` cereal
adapters, AircraftState getters) all live in headers, and `autoc_common.a` is already
linked by 5 other executables (autoc, minisim, renderer, nnextractor, nn2cpp) at
`CMakeLists.txt:105-109`. The dependency surface is bounded: 12 .cc files / 1637
LOC total (`config.cc, logger.cc, pathgen.cc, sensor_math.cc, fitness_computer.cc,
fitness_decomposition.cc, selection.cc, eval_logger.cc, evaluator.cc, serialization.cc,
population.cc, telemetry.cc`), with transitive deps only on `inih`, `cereal`, and
`Eigen3`.

**Implementation pattern**: `tools/nnextractor.cc` is the closest analog. Steps for
the converter:

1. Optional: `ConfigManager::initialize(configFile)` + AWS SDK init (skip if reading
   local .dmp file directly; only needed when fetching from S3, mirroring
   `nnextractor.cc:93-98`).
2. Read `.dmp` bytes. From local disk: open ifstream binary. From S3: replicate
   `nnextractor.cc:100-174` (use `ConfigManager::getS3Client()`).
3. `cereal::BinaryInputArchive ia(iss); ia(evalResults);` — `EvalResults` is
   already cereal-serialized (`include/autoc/rpc/protocol.h:248-253`).
4. Walk `evalResults.aircraftStateList[scenario][step]`, pull
   `state.getPosition()`, `state.getOrientation()` quat, `state.getSimTimeMsec()`
   (`include/autoc/rpc/protocol.h:295-330` shows the dump fields available).
5. Convert quat→euler (the existing `print` method shows the recipe at
   protocol.h:309-318) for the int16 euler slots in 0x00 records.
6. For each scenario in `evalResults.scenarioList`, write one playback file plus a
   small XML provenance record (record type 0x03) carrying
   `pathVariantIndex / windVariantIndex / windSeed / scenarioSequence` so the
   tracker-mode runtime can reconstitute joint-PRNG variation deterministically
   (FR-010).

**LOC estimate**: ~150-250 lines for the converter, dominated by argument parsing
(maybe 50) and per-scenario file write (~100) — the rest reuses existing helpers.

**Rationale**: Carving out a smaller subset (e.g., just `serialization.cc` +
`config.cc`) is achievable but saves only ~1100 LOC of compile cost while pulling in
test-coverage risk (some headers transitively need symbols from
`evaluator.cc`/`fitness_computer.cc` even if the converter doesn't call them — e.g.,
`AircraftState::serialize` lives inline in `aircraft_state.h:476-490` but its
companion functions touch `NN_INPUT_COUNT`-aware code). The wholesale link is
already-paid cost for nnextractor and pays back in zero new build entries.

**Alternatives considered**:
- New `autoc_serialization` micro-lib (just protocol.h consumers + cereal). **Rejected**
  — would require splitting `aircraft_state.h` from the NN headers it depends on, and
  the savings (~5 sec compile) don't justify the maintenance cost.
- Pure Python converter using struct/cereal-compatible parser. **Rejected** —
  cereal's binary format isn't documented; reverse-engineering is brittle. Future
  schema changes (cereal version bumps) would silently break.

**File:line citations**:
- `tools/nnextractor.cc:177-192` (canonical EvalResults deserialization)
- `include/autoc/rpc/protocol.h:234-336` (EvalResults schema + dump method)
- `CMakeLists.txt:83-109` (autoc_common composition + link sites)
- `include/autoc/eval/aircraft_state.h:295-490` (state accessors + cereal adapter)

---

## R3 — Camera projection math implementation strategy

**Decision**: Hand-roll the planar pinhole projection in pure Eigen — no new dep, no
reuse of crrcsim's OpenGL/VTK camera code. Place it in a new module
`include/autoc/eval/camera_projection.h` + `src/eval/camera_projection.cc` (same dir
where `sensor_math.cc` lives — that file already does the body-frame transform via
`getOrientation().inverse() * world_vec` pattern, `evaluator.cc:346-352`). The full
projection is ~30-50 LOC.

**Math** (analytic pinhole, target-craft beacon → screen `(x, y, visible)`):
```
beacon_world  = target_state.position + target_R_body_to_world * beacon_body_offset
beacon_in_camera = camera_R_world_to_body * (beacon_world - training_state.position)
                   - camera_offset_body          // camera mount offset from CG
// then: camera_R_body_to_camera * beacon_in_camera if camera not aligned with body
if (beacon_in_camera.z <= 0) → visible=0  (behind plane)
u_norm = beacon_in_camera.x / beacon_in_camera.z   // tan of horizontal angle
v_norm = beacon_in_camera.y / beacon_in_camera.z   // tan of vertical angle
fov_limit = tan(FOV_horizontal / 2)
if (|u_norm| > fov_limit || |v_norm| > fov_limit) → visible=0
screen_x = u_norm / fov_limit                      // normalized [-1, 1]
screen_y = v_norm / fov_limit
visible = 1
```

Eigen handles all of this with `gp_quat::inverse() * gp_vec3` for the rotation; no
projection-matrix construction is needed (would be over-engineering for a 3-float
output). Target body→world rotation is read from the playback target's
`getOrientation()`.

**Rationale**: The renderer's `vtkCamera` (`tools/renderer.cc:920-928`) is GPU-side
viewport math for the VTK pipeline — entirely the wrong abstraction for per-tick
analytic projection in the training inner loop. crrcsim's `Video::*` namespace is
SSG/OpenGL drawing code and isn't accessible from the autoc training pipeline at all
(autoc never links crrcsim's render code; they communicate over MSP). Eigen's
geometry module (`Projective3f`, `AngleAxisf`) supports arbitrary projection
matrices, but for the simple pinhole case the explicit math is shorter, faster, and
inspectable — and the project already has the pattern of "use Eigen vec3/quat math
inline" everywhere in evaluator.cc.

**Alternatives considered**:
- Eigen `Projective3f` + `Affine3f` matrices. **Rejected** — overhead of constructing
  4×4 matrices each tick when 3 dot products suffice; harder to read.
- Reuse the renderer's vtkCamera->WorldToView. **Rejected** — VTK isn't a runtime
  dep of the autoc training binary, and the call would require an OpenGL context.
- Bring in OpenCV for a battle-tested projection. **Rejected** — would add a heavy
  dep solely for ~30 LOC of math.

**Future extensibility hooks** (FR-003a, deferred to v1+): the function signature
should take a `CameraConfig` struct rather than scalar args, so radial-distortion /
fisheye / rolling-shutter can be added by extending the struct without changing
callers. The struct goes in `include/autoc/eval/camera_config.h` with compile-time
default values per FR-003 / Camera Configuration entity in the spec.

**File:line citations**:
- `src/nn/evaluator.cc:343-356` (existing inverse-quat * vec3 body-frame transform
  pattern — direct template for the projection math)
- `include/autoc/eval/sensor_math.h` (location pattern for related geometry helpers)
- `tools/renderer.cc:920-928, 3245-3260` (vtkCamera — the wrong tool, documented
  here only to confirm rejection)

---

## R4 — crrcsim multi-aircraft model + RobotProgrammable integration sizing

**Decision**:
**(a) Aircraft model**: target craft uses the **same `hb1_streamer.xml`** as the
training craft for v1. Single airframe in tree (`crrcsim/models/hb1_streamer.xml` is
the only file). The XML header inside each playback file declares `airplane.file =
hb1_streamer.xml` (per `crrcsim/src/robots.cpp:43-46` which reads `airplane.file`
from the header). Future multi-airframe variants (after 025 craft variations)
become a separate XML attribute in the playback file header.

**(b) RobotProgrammable estimate confirmation**: the prior 150 LOC estimate from
`reference_crrcsim_mod_robots.md` holds, but with one revision — **for 029 v1, no
new RobotProgrammable subclass is needed**. The existing `CRRC_AirplaneSim_Playback`
is exactly what 029 wants (kinematic playback from a binary file). The 150 LOC
mostly disappears. Required additions are smaller:

1. **`Robots::getRobotFDM(int idx)` accessor** (~5 LOC) in `crrcsim/src/robots.h:75`
   — needed so `inputdev_autoc` can query the playback target's pose per tick to
   compute beacon projections + (optionally) reuse it as the `RobotPathProvider`'s
   "rabbit". Currently `Robots::list` is private (`robots.h:75`).

2. **Per-scenario playback file selection** at scenario startup (~30 LOC). The
   training run's RPC passes `ScenarioMetadata` to crrcsim; tracker-mode adds a
   `targetLibraryPath` field. `inputdev_autoc` calls
   `Global::robots->RemoveAll()` then `AddRobot(targetLibraryPath)` between
   scenarios. Hook lives in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`
   near the existing scenario reset (need to verify exact line during plan).

3. **Beacon-projection module** (~80-120 LOC) in
   `crrcsim/src/mod_inputdev/inputdev_autoc/` (or a new sibling
   `beacon_projection.cc`): each tick, query the target's `getPos()` +
   `getPhi/Theta/Psi()`, compute world→camera transform with the training craft's
   pose, run the R3 projection math, append to the NN input buffer.

4. **NN input wiring** (the type-safe sensor interface — see R7). On the autoc
   side, reuse the existing tracker history queue. Modest delta in `evaluator.cc`.

5. **(Optional, deferred from prior estimate)** A new `RobotProgrammable` subclass
   for *live* trajectory injection — e.g., to drive the target via RPC for future
   real-target experiments. **Not needed for 029 v1** since file-based playback
   covers the spec. The interface stays clean for future addition.

**Total LOC estimate revised**: **~120-180 LOC** of crrcsim-side adds for FR-002 +
FR-003/FR-005 baseline. Less than the prior 150 estimate because the playback
substrate is reused as-is.

**Rationale**: `Robots::Update` already runs the playback FDM in lockstep with the
main FDM tick (`SimStateHandler.cpp:114`); `Robots::Reset` re-opens the file at
seekg(0) (`fdm_playback.cpp:170-185`). The integration seam is small. The
documented constraint from the memory note — "Only one trajectory source exists,
ignores control inputs" — is exactly what 029 wants for the kinematic playback
target.

**Alternatives considered**:
- New `RobotProgrammable` subclass that takes a vector<AircraftState> in memory
  rather than reading a file. **Rejected for v1** — file-based gives full
  symmetry with the existing playback flow and works with the converter tool's
  output trivially. Memory-based would require a new RPC channel from autoc to
  crrcsim to ship trajectories.
- Skip mod_robots; embed target trajectory directly into `inputdev_autoc` so
  there's no second FDM. **Rejected** — loses the renderer's
  3rd-person-with-target visualization (FR-012); robots' visual model
  registration via `Video::new_visualization` is exactly what makes the second
  aircraft appear in the renderer.

**File:line citations**:
- `crrcsim/src/robots.cpp:34-95` (multi-aircraft Update/Reset; AddRobot loads
  visual model from header.airplane.file)
- `crrcsim/src/SimStateHandler.cpp:110-114, 237` (main-loop integration)
- `crrcsim/src/mod_robots/robot.cpp:49-78` (loadAirplane chooses
  CRRC_AirplaneSim_Playback)
- `crrcsim/models/hb1_streamer.xml` (single airframe model in tree)

---

## R5 — GPU-Native Evaluation ordering decision for 029

**Decision**: **029 v1 ships scoped-down on CPU** at reduced training scale. GPU-Native
Evaluation is a parallel track (post-029 v1, before US4 long-run training at full pop).

**Compute load math**:
- Pop = 5000, scenarios = 245 (5 paths × 49 winds), ~130 ticks/scenario (typical
  termination).
- 028 baseline: 5000 × 245 × 130 = 1.59 × 10⁸ NN ticks/gen with no projection cost.
- 029 v1 added cost per tick: 2 beacons × 1 camera × 1 projection (analytic
  pinhole, ~30 FLOPs) ≈ negligible vs. NN forward pass (~3K weights = ~6K FLOPs
  for D-simple recurrent).
- 029 v1 with 30 Hz camera × 10 Hz NN tick + 4 history slots: per NN tick,
  amortized 0.4 projections per beacon (only 4 needed for the history; the
  remaining frames are discarded). Roughly +5-10% per-tick cost over 028.

The spec-line concern (line 73) about "millions of frame instances per gen" is **valid
for 017's vision NN at 30-90 frames/tick × stereo cameras × full-pop**, but **029 v1
has only 8 projection calls per NN tick** (4 history slots × 2 beacons × 1 camera).
The aggregate is dominated by the existing NN evaluation cost, not the projection.

**Estimated 029 v1 cost vs. 028 baseline**: +5-10% wall-clock per generation. 028's
800-gen run takes ~7-10 days; 029 v1 at matched scale would take ~7.5-11 days. **CPU
is feasible**.

**Where GPU eval becomes critical for 029**:
- Per-scenario camera variation (deferred per spec line 320) — adds
  PRNG-resampled camera config per scenario. CPU-tractable today; GPU pays back
  more if multiple cameras land.
- Sub-tick NN architecture (FR-003e option d) — running NN at 30/60/90 Hz instead
  of 10 Hz multiplies NN forward pass count 3-9×.
- Multi-camera asymmetric pair (FR-003b) — doubles projection count per tick, still
  small.
- Population scaling beyond 5000 to chase a richer search (US3 sweep variants). At
  pop 10000, +5% cost compounds with longer wall-clock.

**Recommended scoping for 029 v1**:
- Keep pop = 5000, scenarios = 245, gens = 600 (matches more-rnn3 cost envelope).
- Default frame rate = 30 Hz, single camera, history strategy (c) "latest +
  derived" with 4 history slots — the spec's chosen baseline.
- US4 first run in this regime: expected ~10-day wall-clock, no GPU required.
- Defer the US3 multi-camera / 60+ Hz sweeps to post-GPU-eval.

**Threshold for switching to GPU eval**: when projection cost > 25% of per-tick
total — happens around 6-8 cameras at 60+ Hz, or sub-tick NN updates.

**Rationale**: The spec's caveat anticipated the 017 cost model (raw-pixel-class
projection budget). Spec Q3 clarifications established 029 doesn't render pixels —
it does analytic 3-float projection. That fundamentally changes the cost balance.
See `specs/017-visual-target-tracking/spec.md:190-216` for the prior CPU-intractable
analysis (which assumed ~115M projections/gen for the vision NN config); 029 at the
spec's chosen baseline does ~10⁹ projections/gen but each is 30 FLOPs — well within
CPU.

**Alternatives considered**:
- Land GPU-Native Evaluation first as a 029 prereq. **Rejected** — risks 1-3 month
  delay on a research feature whose primary value is the architectural
  validation; CPU is sufficient for v1 scale.
- Scope 029 v1 even smaller (pop 1000, gens 200) to ship in ~1 day. **Rejected** —
  loses comparability with more-rnn3 (the natural baseline). The full 245-scenario
  × 5000-pop config is feasible; smaller scale just delays signal.

**File:line citations**:
- `specs/BACKLOG.md:152-162` (GPU-Native Evaluation entry, "blocking dependency for
  017-phase3 at training scale")
- `specs/017-visual-target-tracking/spec.md:190-216` (the cost analysis that made
  GPU eval blocking for 017 — different cost model than 029)
- `autoc.ini` (current config: PopulationSize=5000, NumberOfGenerations=800,
  WindScenarios=49, SimNumPathsPerGeneration=5)

---

## R6 — Camera latency modeling approach for v1

**Decision**: **Simple per-camera constant-delay model** for v1 — every camera frame
the NN sees is delayed by `latency_ms` (a `CameraConfig` field) relative to the
"true now" of the simulation. Implementation: each camera has a frame ring-buffer
sized `ceil(latency_ms / frame_period_ms) + 1`; per tick, push the freshly-projected
frame into the buffer and present `buffer[head - latency_frames]` to the NN.

**Rationale**:
- The spec calls v1 default 0 ms with US3 sweep at 30/50 ms (spec line 364). A
  uniform per-frame delay captures the dominant flight-hardware behavior:
  exposure-end → readout-complete → DSP-extracted → arrived-at-NN. The dominant
  term is the readout + DSP step which is approximately uniform across the
  sensor.
- Per-row exposure integration (true rolling-shutter modeling) is materially more
  complex: would need to track per-beacon exposure-time skew that depends on the
  beacon's row position at capture time, with the row position itself a function
  of where the projection lands. That's interesting research but not v1 — the
  spec explicitly stubs rolling shutter as global-shutter (FR-003a, see edge
  case at spec line 217).
- A constant delay is sufficient to expose the controller to "world has advanced
  by latency × angular_velocity worth of beacon shift" (spec line 220), which is
  the load-bearing question US3 wants answered.

**Storage**: the ring buffer is small (3-9 frames per camera at the spec's frame
rates); fits in `CameraState` per-camera in the input pipeline. Persistence across
NN ticks is needed (a 50 ms latency at 30 Hz spans 1.5 frames, requiring
prev-tick frames to still be addressable).

**Architectural extensibility hook**: keep the latency parameter in `CameraConfig`
as a *vector* (`std::array<float, 2>` for exposure_ms + readout_ms) even though v1
only sums them into a single delay. A future rolling-shutter v2 can interpret the
two components separately without changing the field type.

**Alternatives considered**:
- Per-row exposure-time integration. **Rejected for v1** — adds significant
  complexity (per-beacon row-time interpolation; pose interpolation between
  ticks) for a stubbed-by-spec feature.
- Stochastic latency (uniform jitter ± Δ ms). **Rejected for v1** — clean
  deterministic baseline first; jitter is a future variant per the
  "Simulator Sampling Time Variation" deferred backlog item
  (`specs/BACKLOG.md:146-150`), which already covers this surface for non-camera
  inputs and could share infrastructure later.
- Zero latency in v1 (drop the field entirely). **Rejected** — spec FR-003d
  requires the *interface* to accept non-zero values. v1 default 0 is fine, but
  the buffer + delay-index path must be wired.

**File:line citations**:
- `specs/029-tracker-mode/spec.md:220-221` (latency × angular_velocity rationale)
- `specs/029-tracker-mode/spec.md:364` (v1 default 0 ms, US3 sweep 30/50 ms)
- `specs/BACKLOG.md:146-150` (Simulator Sampling Time Variation — shared infra
  candidate)

---

## R7 — Type-safe NN sensor interface — implementation cost estimate

**Decision**: Refactor lands as **a single PR before any 029-specific work**, not in
parallel. The refactor has 12 well-defined touchpoints, low conceptual complexity,
and pays back immediately when 029 needs to add ~15 named beacon-related inputs to
`NNInputs`.

**Touchpoints + estimated migration cost** (all 12 files were enumerated in
`specs/BACKLOG.md:206-215` for the 021 changes; recounted from the live code):

| File | Current pattern | Migration cost |
|------|-----------------|----------------|
| `include/autoc/nn/nn_inputs.h` | Magic `float[6]` arrays + struct (33 LOC) | Replace with enum-tagged struct or named-member struct + reflection helpers (~80 LOC) |
| `include/autoc/nn/topology.h` | `NN_INPUT_COUNT` derived from sizeof(NNInputs) (line 41, 63) | Already derived correctly — minor comment update |
| `src/nn/evaluator.cc` | `nn_gather_inputs()` writes by member name (line 321-356); raw `float* in = ...` for logging (autoc.cc:669) | Update to use named setters; logging uses name→index map (~30 LOC change in evaluator.cc, ~15 in autoc.cc) |
| `src/autoc.cc` | sprintf format string with magic order + comment block (line 668-700+) | Replace with iteration over named-input enum (~40 LOC) |
| `tools/renderer.cc` | parses data.dat by capture group + position-indexed `nn.target_x[]` etc. (line 1809-1855+) | Move parser to use name-keyed dict; renderer reads by name (~30 LOC) |
| `tools/nn2cpp.cc` | Generates code that fills `NNInputs inputs = {}` directly (line 122-189) | Should "just work" if the struct stays struct-shaped (~5 LOC if any) |
| `xiao/src/msplink.cpp` | `(float*)&aircraft_state.getNNInputs()` reinterpret + magic field commentary (line 312-325) | Use named accessors; printf converted to per-name iteration (~25 LOC change) |
| `xiao/src/generated/nn_program_generated.cpp` | Generated by nn2cpp | Regenerate after nn2cpp update |
| `tests/contract_evaluator_tests.cc` | Topology assertions reference numeric layout | Update to assert named inputs (~15 LOC) |
| `tests/nn_evaluator_tests.cc` | Input-layout assertions | Same (~15 LOC) |
| `specs/019-improved-crrcsim/sim_response.py` | Position-indexed F0..F49 (line 12-21) | Either ignore (Python parser uses field positions; the data.dat header carries column names so parser could switch to header-driven) or update field index comments (~10 LOC) |
| `include/autoc/eval/aircraft_state.h` | `nnInputs_` member + cereal serialize | Reuse as-is; the struct is still cereal-friendly via member-by-member; minor adapter (~5 LOC) |

**Total estimate**: **~270-330 LOC of changes** across 12 files. Mechanical
refactor; no architectural risk.

**Recommended interface shape** (drives the mechanical work):
```cpp
// include/autoc/nn/nn_inputs.h
enum class NNInput : int {
    TARGET_X_T_NEG_900_MS = 0,   // -0.9s slot
    TARGET_X_T_NEG_300_MS,
    ... (5 more for target_x history)
    TARGET_Y_T_NEG_900_MS,
    ... (continue for target_y, target_z, dist)
    CLOSING_RATE,
    QUAT_W, QUAT_X, QUAT_Y, QUAT_Z,
    AIRSPEED,
    GYRO_P, GYRO_Q, GYRO_R,
    NN_INPUT_COUNT
};

struct NNInputs {
    std::array<float, static_cast<int>(NNInput::NN_INPUT_COUNT)> values;

    float& operator[](NNInput n) { return values[static_cast<int>(n)]; }
    float operator[](NNInput n) const { return values[static_cast<int>(n)]; }

    // Compatibility shim for existing struct-member-access call sites
    // (deprecate over time):
    float* target_x() { return &values[static_cast<int>(NNInput::TARGET_X_T_NEG_900_MS)]; }
    // ...
};

// 029 adds:
//   BEACON_L_X_T_NEG_900_MS .. BEACON_L_X_T_NOW    (4 slots)
//   BEACON_L_Y_*  (4)   BEACON_L_VISIBLE_*  (4)
//   BEACON_R_X_*  (4)   BEACON_R_Y_*  (4)   BEACON_R_VISIBLE_*  (4)
// = 24 named inputs added; replaces 24 of the existing 33 (target_x/y/z/dist 6-slot).
```

The named-member compatibility shim keeps `inputs.target_x[i] = ...` callsites
working during migration (a single internal pointer + index pattern), while new
code uses `inputs[NNInput::TARGET_X_T_NEG_900_MS]`.

**Sequencing recommendation**:

1. **Phase 0 (PR-029-A, 2-3 days, blocks rest of 029)**: Type-safe sensor interface
   refactor on the *existing* 33-input layout. Zero behavior change, all tests
   pass, all training compatibility preserved (data.dat format unchanged because
   sizeof(struct) unchanged and field order semantically unchanged).

2. **Phase 1 (PR-029-B, the rest of 029)**: Add the 24 beacon-related inputs,
   replace 24 of the existing 33. Now using the named interface, the change is a
   declarative edit to the enum + a swap-in in `nn_gather_inputs`. The 12
   downstream files keep working because the iteration model is name-driven.

**Rationale for sequencing**: doing the refactor in-flight with 029's beacon work
risks mixing two failure modes (refactor bugs vs. tracker-mode-logic bugs). The
033-input layout is large but stable; the refactor against it is safe. Once
landed, 029's input expansion is mechanical.

**Alternatives considered**:
- Roll the refactor into 029 as one PR. **Rejected** — touches the same files as
  029 spec changes; merge conflicts likely; bisecting failures becomes harder.
- Defer the refactor; do 029 with magic numbers. **Rejected** — spec FR-006 makes
  the named interface load-bearing for the genome-ablation tool (BACKLOG entry,
  spec lines 245, 339), and the `BEACON_*_VISIBLE[*]` ablation pattern requires
  name-based input masking. Adding 24 magic-number slots first then refactoring
  is strictly more total work.

**File:line citations**:
- `include/autoc/nn/nn_inputs.h:9-32` (current struct, 33 floats)
- `include/autoc/nn/topology.h:33-63` (count / weight derivation)
- `src/nn/evaluator.cc:321-441` (gather + dispatch)
- `src/autoc.cc:668-700` (data.dat formatter, magic-order 33-field sprintf)
- `tools/renderer.cc:1809-1855` (data.dat parser)
- `xiao/src/msplink.cpp:312-325` (xiao input gathering, magic-order printf)
- `tests/contract_evaluator_tests.cc`, `tests/nn_evaluator_tests.cc` (assertions)
- `specs/019-improved-crrcsim/sim_response.py:12-21` (Python parser)
- `specs/BACKLOG.md:193-219` (the original Type-Safe NN Sensor Interface entry —
  scope confirmed by current code)

---

## Cross-cutting summary for plan kickoff

- **R1 + R2** together let phase 1 ship FR-001 (the dmp→playback converter): ~250
  LOC standalone tool linked against autoc_common, emitting standard
  `.crrclog` files.
- **R3 + R4 + R6** are the crrcsim-side work for FR-002 / FR-003 / FR-005: ~250
  LOC of new C++ across `mod_inputdev/inputdev_autoc/` and a small accessor in
  `robots.h/cpp`. No new RobotProgrammable subclass needed for v1.
- **R5** says no GPU prereq — 029 v1 ships on CPU at the same compute envelope as
  more-rnn3.
- **R7** is a 270-330-LOC mechanical refactor that should land first as a
  blocker-PR; it makes the rest of 029's input plumbing trivial.

Total 029 v1 implementation surface: **~600-800 LOC of new C++** (plus ~300 LOC of
type-safe-sensor refactor that lands first), spread across one Python tool, two
crrcsim modules, and one autoc input-pipeline change.
