# M11.preA Plan — CRRCSim tracker-mode integration (T079-T081)

**Status**: planning complete 2026-05-08; T079-T081 to land as a single PR.
**Operator decisions captured**: see chat 2026-05-08 (this file is the durable record).

## Scope

Tracker-mode training routes through `./scripts/crrcsim.sh` (FDM-grade) instead
of `./build/minisim` (kinematic). The strategy split (`PathgenStepper` /
`TrackerStepper`) that landed at minisim's M6a-M6e is mirrored into
`crrcsim/src/mod_inputdev/inputdev_autoc/` — but as a **helper class**, not a
full strategy interface, to keep the regression-tight pathgen-mode bitwise
gate trivially holding.

T082 (smoke run on crrcsim tracker) and T083 (formal SMOKE_REPORT.md) are
operator-driven, follow this PR.

## Architectural decisions (operator-confirmed)

### Factoring: Option C — Helper class
- New `CrrcsimTrackerHelper` class in
  `crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.{h,cpp}`
- `getInputData()` per-NN-tick body has `if (mode == "tracker") helper.tick(...)
  else { existing pathgen body }`
- **Pathgen body strictly unchanged** ⇒ rebuild-perf.sh + autoc-eval bitwise
  gate against `gen9200.dmp` baseline holds for free
- ~80 LOC overlap with minisim's `TrackerStepper` (per-tick flow control); load-
  bearing primitives (`projectBeacon`, `gather_tracker_inputs`,
  `computeTrailRabbit`, `CrashHull`, `FlightArena`) are shared via
  `autoc_common`

### Chase init: match M1 pathgen exactly
- No tracker-specific entry-position hack. `entryNorthOffset` stays at the
  scenario's variation value (default 0)
- FDM full reset via `Global::Simulation->reset(windSeed)`; chase pose read
  from FDM post-reset
- **Implication**: at tick 0, chase ≈ source (both at virtual origin); trail
  rabbit ≈ (-trail_distance, 0, 0) — i.e., 3m BEHIND chase. Chase will need
  source to pull ahead before trail-rabbit fitness stabilizes
- Diverges from minisim M9.preB (where chase is shifted +trail_distance and
  velocity copied from source[0]) — the minisim shift was specifically to
  defeat the kinematic 20-m/s init spike. Crrcsim FDM defaults hb1 to ~13 m/s
  cruise, no spike, so the minisim hack isn't needed. Smoke run will verify
- If first smoke shows pathological tick-0 fitness shape, add config knob
  `EnableTrackerEntryOffset` in autoc-tracker.ini (defaulting off) as a
  follow-up — not in this PR

### Pathgen scenario-init bookkeeping in tracker mode: skip
- `rabbitOdometer = 0`, `pathIndex = 0`, `rabbitSpeedProfile.clear()`,
  `engageDelayTicksRemaining = 0`, `crrcsimRabbitSpeed = 0`
- Tracker per-NN-tick body never reads these — they're inert
- Pathgen-mode init code path is strictly unchanged (mode != "tracker" ⇒
  existing block runs)

### M2 dmp output format: identical across minisim/crrcsim
- Already true at schema level — both write `EvalResults` v=2 via `sendRPC`
- Helper exposes `lastCameraView()` + `lastTargetSample()` + `hullFiredCount()`
- `inputdev_autoc.cpp` path-end push code mirrors minisim's M8b push, populating
  `evalResults.cameraViewList[scenario][tick]` + `targetTrajectoryList[s][t]` +
  `hullStrikeCount[s]` + `arenaEgressCount[s]`
- Same S3 prefix convention; run-id timestamp + autoc.ini config naming
  distinguish minisim vs crrcsim runs

### Helper location: in `crrcsim/src/mod_inputdev/inputdev_autoc/`
- Co-located with `inputdev_autoc.h/.cpp`
- crrcsim-namespaced; not visible to autoc-side
- Mirrors existing mod_inputdev_autoc structure

### Determinism
- Critical: operator may use crrcsim tracker to investigate m91 minisim
  non-determinism (recorded in
  `~/.claude/projects/-home-gmcnutt-autoc/memory/project_tracker_fitness_nondeterminism.md`)
- Helper has zero wall-clock / thread-id state
- PRNG seeds derive from `scenarioMetadata.scenarioSequence` (already
  deterministic)
- FDM determinism is the existing pathgen-mode invariant (rebuild-perf.sh
  bitwise gate); tracker mode doesn't perturb it

## Files touched (T079+T080+T081 unified)

| File | Change |
|------|--------|
| `crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.h` | NEW: declare helper class |
| `crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp` | NEW: implementation |
| `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h` | Add `trackerHelper_` member + per-path `cameraViewSamples_` / `targetSamples_` per-tick buffers |
| `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` | Per-path-init: `if (mode == "tracker") trackerHelper_.initScenario(...)`; per-NN-tick: `if (mode == "tracker") trackerHelper_.tick(...) else { existing pathgen body }`; per-path-end: push helper outputs into `evalResults.cameraViewList` etc. |
| `crrcsim/src/mod_inputdev/CMakeLists.txt` | Add new .cpp to build target |

No autoc_common changes — all primitives already there from M5/M6/M7.

## Helper API

```cpp
class CrrcsimTrackerHelper {
public:
  void initScenario(const SourceScenarioTrajectory& source,
                    const ScenarioMetadata& meta,
                    const EvalData& evalData,
                    AircraftState& chaseState,
                    NNControllerBackend& nn);

  // Per-NN-tick. chaseState is post-FDM-step (read by caller).
  // Returns CrashReason::None if scenario continues, else terminating reason.
  CrashReason tick(AircraftState& chaseState,
                   NNControllerBackend& nn,
                   const EvalData& evalData);

  const CameraViewSample& lastCameraView() const;
  const CopiedTargetSample& lastTargetSample() const;
  int hullFiredCount() const;

private:
  void projectAndShiftHistory(const SourceTickSample& target,
                              const AircraftState& chaseState,
                              const EvalData& evalData);

  size_t cursor_ = 0;
  TrackerHistoryWindow history_{};
  CrashHull crash_hull_{};
  uint32_t prng_state_ = 0;
  int hull_fired_count_ = 0;
  CameraViewSample last_camera_view_{};
  CopiedTargetSample last_target_sample_{};
  const SourceScenarioTrajectory* source_ = nullptr;
};
```

## Bitwise gate

- **Before T079-T081 land**: operator runs `bash scripts/rebuild-perf.sh` +
  `build/autoc -i autoc-eval.ini` against `gen9200.dmp`. Captures fitness
  baseline.
- **After T079-T081 land**: same gate, bitwise match required. Pathgen body
  is untouched ⇒ should hold trivially. Operator runs to confirm.
- **Tracker-mode**: not a bitwise gate (different output by design). T082
  smoke run compares fitness curve shape to m91 minisim baseline qualitatively.

## Implementation sequence (within this PR)

1. New helper .h/.cpp scaffolds (compiles, empty bodies)
2. CMakeLists update (helper builds into mod_inputdev)
3. inputdev_autoc.h: helper member + per-path tracker buffers
4. inputdev_autoc.cpp init() path: nothing new (helper init lives in scenario reset)
5. inputdev_autoc.cpp scenario-reset block: tracker-mode branch sets pathgen
   bookkeeping defaults, calls `trackerHelper_.initScenario(...)`
6. inputdev_autoc.cpp per-NN-tick body: tracker-mode branch calls
   `trackerHelper_.tick(...)`, else existing pathgen body
7. inputdev_autoc.cpp per-path-end: push helper buffers into `evalResults`
8. Helper implementation: `initScenario` (cursor=0, hull init, PRNG seed,
   history pre-fill from source[0]); `tick` (project beacons, shift history,
   gather_tracker_inputs, evaluateTracker, didCrashFire, checkArenaBounds);
   M2 dmp recording outputs

## What this PR does NOT do (explicitly)

- T082: smoke run with crrcsim tracker (operator-driven, separate)
- T083: formal SMOKE_REPORT.md (post-T082)
- M11.preB: live two-aircraft display in crrcsim (mod_robots side; optional
  follow-up only if M11.preA smoke needs visual mid-training debug)
- Any analytics (M11a-c follow M11.preA)
- Any change to autoc-side scenario distribution / EvalData shape — schema
  stays at v=2, all existing fields consumed identically

## Open follow-ups (outside this PR)

- `EnableTrackerEntryOffset` config knob (if M11.preA smoke shows the
  no-offset chase init is geometrically pathological)
- Potential code-share refactor: extract common per-tick tracker primitives
  from minisim's `TrackerStepper` and crrcsim's `CrrcsimTrackerHelper` into
  a shared autoc_common helper. Bounded ~80 LOC overlap; deferred until
  the v1 smoke green so refactor doesn't hold up the deliverable.
