# 029 — Quickstart (operator walkthrough)

End-to-end recipe for executing the 029 phases. Audience: operator running through the implementation after `/speckit.tasks` produces tasks.md.

## Prerequisites

- Branch `029-tracker-mode` checked out, working tree clean
- 028 codebase at the more-rnn3 commit or later (`git log` shows the more-rnn3 baseline run committed)
- A completed pathgen-mode source training run (more-rnn3 final or a successor) with S3 .dmp files available at a known path
- `bash scripts/rebuild.sh` green on the 028 baseline

## Phase 1 — US1 past-only baseline experiment

**Runs in parallel with the 028 flight wait. Single-file change, then a full training run.**

### 1.1 Edit time-sample distribution

```bash
# Edit nn_inputs.h comment + nn_input_computation.cc time offsets per plan §1.1, §1.2
# Old time offsets: [-0.9, -0.3, -0.1, 0, +0.1, +0.5]
# New time offsets: [-0.5, -0.4, -0.3, -0.2, -0.1, 0]
```

The change is: in `src/nn/nn_input_computation.cc`, find the array of time offsets (used to look up rabbit position at past/future odometer values) and replace with the new past-only distribution. In `include/autoc/nn/nn_inputs.h`, update the `// Time samples:` comment to match.

### 1.2 Build + verify tests

```bash
bash scripts/rebuild.sh                                      # green
ctest --output-on-failure                                    # all 12 tests pass
cd xiao && ~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed   # green (constitutional req)
```

### 1.3 Launch the past-only training run

```bash
nohup ./build/autoc -c autoc.ini > logs/autoc-029-pastonly.log 2>&1 &
# Path A config: pop 5000, gens 600, recurrent NN, single seed
# Run name convention: more-rnn4-pastonly
```

### 1.4 Monitor every 50 gens

```bash
python3 specs/028-deeper-rnn/plot_evolution_progress.py \
    --focus more-rnn4-pastonly:data.stc \
    --total-gens 800 \
    --compare more-rnn3:logs/autoc-028-more-rnn3.log \
    --compare cadence7-redux:logs/autoc-027-cadence7redux.log \
    --out specs/029-tracker-mode/pastonly_evolution.png
```

Watch fitness slope and per-axis aggressiveness vs more-rnn3.

### 1.5 At gen 600 — analyze + decide

```bash
# Compute fixed-eval fitness
./build/autoc --eval -c autoc-eval.ini --weights gen_600.dat

# Per-axis aggressiveness
awk 'NR==1 || /^[0-9]+ [0-9]+ 00[01234]\/00:/' data.dat > /tmp/pastonly_starter_paths.dat
python3 specs/028-deeper-rnn/plot_per_axis_time_series.py \
    --in /tmp/pastonly_starter_paths.dat --label more-rnn4-pastonly \
    --total-gens 800 --pop-size 5000 \
    --out specs/029-tracker-mode/pastonly_per_axis.png

# Write outcome
$EDITOR specs/029-tracker-mode/pastonly_outcome.md
# Pass: late-plateau fitness within ±10 % of more-rnn3, per-axis pattern matches
# Fail: regression > 10 %, or per-axis pattern anomalous
```

**Gate**: PASS → proceed to Phase 2. FAIL → investigate alternative time distributions per plan §1.6 fallback list before declaring "no-lookahead doesn't work."

## Phase 2 — US2 substrate

### 2.1 Type-safe sensor interface refactor (lands first)

Per [contracts/nn_sensor_interface.md](./contracts/nn_sensor_interface.md). ~270-330 LOC refactor across 12 files.

```bash
# Land as a single PR before tracker-mode-specific work begins
# Verify all existing 028 tests still pass with refactored interface
ctest --output-on-failure
# Verify xiao build green
cd xiao && pio run -e xiaoblesense_arduinocore_mbed
# Verify training run produces identical fitness to pre-refactor baseline (smoke test)
nohup ./build/autoc -c autoc.ini > logs/autoc-029-refactor-smoke.log 2>&1 &
# After ~50 gens, compare fitness to a more-rnn3 prefix at same gen
```

### 2.2 crrcsim multi-aircraft accessor

Per [research.md R4](./research.md). Reuses existing `CRRC_AirplaneSim_Playback`. Add `Robots::getRobotFDM(int idx)` accessor to `crrcsim/src/mod_robots/robots.{h,cpp}`. ~5 LOC change.

```bash
bash scripts/rebuild.sh
ctest --output-on-failure
```

### 2.3 dmp-to-playback converter

Per [contracts/playback_file_format.md](./contracts/playback_file_format.md) and [research.md R2](./research.md).

```bash
# Build the new tool
make -j8 dmp_to_playback

# Convert a source pathgen run's gen N into a 245-entry playback library
./build/dmp_to_playback \
    --source-run "more-rnn3-2026-04-26T..." \
    --source-gen 600 \
    --output-dir libraries/more-rnn3-gen600/

# Verify
ls libraries/more-rnn3-gen600/    # 245 .crrclog files + library_metadata.json
python3 specs/029-tracker-mode/scripts/validate_library.py libraries/more-rnn3-gen600/
```

### 2.4 RobotPathProvider + tracker-mode autoc.ini

Per plan §2.4. Modify `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` to spawn the playback aircraft per scenario and wire the RobotPathProvider into the eval pipeline. New `autoc-tracker.ini` config.

```bash
# Smoke test — tracker mode launches end-to-end with placeholder zero-beacon-input
nohup ./build/autoc -c autoc-tracker.ini > logs/autoc-029-substrate-smoke.log 2>&1 &
# Watch first 5 gens — fitness will be uniformly bad (NN sees no target signal yet)
# Determinism check: re-run with same seed → identical fitness
```

### 2.5 Tracker-mode EvalResults schema bump

Per [contracts/tracker_dmp_schema.md](./contracts/tracker_dmp_schema.md). Modify `include/autoc/rpc/protocol.h`. Bump cereal class version per project policy.

```bash
bash scripts/rebuild.sh
ctest --output-on-failure
```

**Phase 2 exit gate**: tracker-mode autoc launches end-to-end, second aircraft visibly replays from library, fitness reported (uniformly bad), build green, all tests pass, xiao build green, determinism preserved.

## Phase 3 — US3 camera + beacon projection

### 3.1 Build the projection module

Per [contracts/beacon_projection_api.md](./contracts/beacon_projection_api.md) and [research.md R3](./research.md). ~30-50 LOC in pure Eigen.

### 3.2 Wire beacon projection into per-tick input gathering

Per data-model.md §6.1. The 44-input tracker-mode layout replaces the 33-input pathgen layout when `TrainingMode = TRACKER`.

### 3.3 Frame buffer / history window

Per data-model.md §8. Per-camera ring buffer holding last ~16 frames at 30 Hz; sample at offsets `[-0.5, -0.4, -0.3, -0.2, -0.1, now]` per NN tick.

### 3.4 First short tracker-mode training

```bash
# Edit autoc-tracker.ini for v1 baseline camera config
# (planar pinhole, 120° FOV, single forward-mounted, 30 Hz, 0 ms latency)
nohup ./build/autoc -c autoc-tracker.ini > logs/autoc-029-tracker-baseline-50gen.log 2>&1 &
# Target: 50-100 gens, demonstrate sane fitness descent + visual lock signal
```

### 3.5 Camera-config sweep (US3)

```bash
# For each variant (FOV 60° / 90° / 120°, mount nose / canopy, frame rate 30 / 60):
#   - Edit constexpr camera config in include/autoc/eval/camera_config.h
#   - Rebuild
#   - Launch short run (50 gens)
#   - Compare per-axis aggressiveness
# Document results in specs/029-tracker-mode/us3_camera_sweep.md
```

**Phase 3 exit gate**: at v1 baseline config, 50-100 gen run produces sane fitness descent + ≥ 70 % visual-lock fraction. ≥3 camera-config variants compared.

## Phase 4 — US4 long-run training

```bash
# Choose v1 baseline from US3 results, finalize autoc-tracker.ini
nohup ./build/autoc -c autoc-tracker.ini > logs/autoc-029-tracker-base.log 2>&1 &
# Path A config: pop 5000, gens 600, recurrent NN, single seed
# Run name: tracker-base
```

Monitor / analyze with the existing 028 telemetry tooling (works unchanged per FR-014). Outcome doc: `specs/029-tracker-mode/tracker-base_outcome.md` documenting whether SC-003 / SC-004 / SC-005 / SC-006 pass.

## Phase 5 — US5 renderer dual-mode + US6 review

### 5.1 Renderer dual-mode

Per FR-012 / FR-012a. Modify `tools/renderer.cc` for tracker-mode `.dmp` loading + 3rd-person/1st-person view selector + per-tick scrub controls.

```bash
./build/renderer --tracker-dump path/to/tracker_gen600.dmp
# Toggle 3rd-person / 1st-person camera-POV with keyboard shortcut
# Use scrub controls to step per tick
```

### 5.2 Architectural review

Code review at end of Phase 4 per US6. Confirm perception-to-NN interface separation (sim-beacon-projection module is replaceable by future real-perception module without retraining). Document outcome.

## Phase 6 — Close

```bash
# Update BACKLOG.md — mark rolled-in items complete
$EDITOR specs/BACKLOG.md
#   - Mark Type-Safe NN Sensor Interface as complete
#   - Mark Renderer Playback Enhancements (per-tick scrub) as complete
#   - Add findings cross-reference

# Findings doc
$EDITOR specs/029-tracker-mode/findings.md
#   - Summarize US1 past-only result, US3 camera-sweep result, US4 long-run result
#   - List open questions for next milestone (real-target tracking)

# Memory updates
$EDITOR ~/.claude/projects/-home-gmcnutt-autoc/memory/MEMORY.md
#   - Update post_028_routing memory: 029 outcome + next routing
```

## Troubleshooting

- **Phase 1 build break after time-offset edit**: most likely the time-offsets array isn't a free-standing constant; might be inlined. Search for `0.9` / `0.3` / `0.1` in `nn_input_computation.cc` and verify all 6 offset values are updated consistently.
- **Phase 2.3 dmp-to-playback fails to deserialize**: check the source dmp path is local OR S3 client is configured (autoc.ini has S3 credentials). Per R2, the converter links autoc_common which loads ConfigManager.
- **Phase 2.4 substrate smoke fails — fitness diverges across runs with same seed**: determinism break. Most likely the playback library is being loaded with a non-deterministic order (filesystem readdir). Sort entries by index before assigning to scenarios.
- **Phase 3 first tracker run — fitness flat at gen 0 baseline**: NN sees garbage beacon coords (projection bug). Use the camera-POV renderer mode to see what the controller is "seeing" — beacons should project to expected screen positions for known target poses.
- **Phase 4 long-run fitness regresses vs more-rnn3**: expected partly — tracker mode is harder than pathgen mode (sparse signal, no lookahead). The bar is "trained controller maintains visual lock ≥80 % of ticks" not "matches pathgen fitness." Use SC-004's lock-fraction metric, not raw pathgen-comparable fitness.
