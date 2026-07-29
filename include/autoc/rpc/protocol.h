/* test sim for aircraft */
#ifndef AUTOC_PROTOCOL_H
#define AUTOC_PROTOCOL_H

#include <vector>
#include <iostream>
#include <cstdint>
#include <arpa/inet.h>

#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>
#include <cereal/archives/binary.hpp>

#include <cstdio>

// Cross-platform serialization notes:
// - cereal binary archive serializes primitives in native format
// - Works across x86/ARM if both are little-endian (typical for modern systems)
// - IEEE 754 float representation is standard
// - Custom Eigen serialization (gp_vec3, gp_quat) uses element-wise saves (portable)
// - Current implementation: deterministic within same architecture

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "autoc/util/socket_wrapper.h"
#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/arena.h"               // FlightArena (030 M7a)
#include "autoc/eval/beacon_config.h"
#include "autoc/eval/camera_config.h"
#include "autoc/eval/camera_projection.h"   // AirframeObstruction
#include "autoc/eval/variation_generator.h"
// NOTE: include "autoc/eval/source_trajectory.h" lives AFTER
// ScenarioMetadata is defined below — source_trajectory.h depends on
// ScenarioMetadata as a member-by-value, so it must see the full type.
// The deferred include is at the EvalData declaration point.

// FNV-1a hash for serialized GP programs/bytecode (matches dtest tracker)
inline uint64_t hashByteVector(const std::vector<char>& data) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data.data());
  uint64_t hash = 0xcbf29ce484222325ULL;
  for (size_t i = 0; i < data.size(); ++i) {
    hash ^= bytes[i];
    hash *= 0x100000001b3ULL;
  }
  return hash;
}

// Hash a string using FNV-1a (deterministic, only depends on string content)
inline uint64_t hashString(const std::string& str) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(str.data());
  uint64_t hash = 0xcbf29ce484222325ULL;
  for (size_t i = 0; i < str.size(); ++i) {
    hash ^= bytes[i];
    hash *= 0x100000001b3ULL;
  }
  return hash;
}

namespace cereal {
    template<class Archive>
    void save(Archive& ar, const gp_vec3& v) {
        ar(v[0], v[1], v[2]);
    }
    template<class Archive>
    void load(Archive& ar, gp_vec3& v) {
        ar(v[0], v[1], v[2]);
    }
    template<class Archive>
    void save(Archive& ar, const gp_quat& q) {
        gp_scalar w = q.w(), x = q.x(), y = q.y(), z = q.z();
        ar(w, x, y, z);
    }
    template<class Archive>
    void load(Archive& ar, gp_quat& q) {
        gp_scalar w, x, y, z;
        ar(w, x, y, z);
        q = gp_quat(w, x, y, z);
    }
}


// ScenarioMetadata factored out to its own header (030 M6e) to break the
// circular include between protocol.h and source_trajectory.h. Existing
// consumers of protocol.h see the type unchanged via this re-include.
#include "autoc/rpc/scenario_metadata.h"
#include "autoc/eval/source_trajectory.h"

// Controller type tag for RPC wire format
enum class ControllerType : int {
  NEURAL_NET = 2   // NNGenome weight vector
};
// cereal doesn't serialize enum class directly - use int wrapper

// 030 M11.preA — Mode dispatch tag for RPC wire format. Replaces the
// std::string mode that landed at M6c (operator routing 2026-05-08:
// type-checked enum is safer + doesn't string-compare on every per-tick
// branch). Cereal-serialized as int (same pattern as ControllerType
// above). EvalData is RPC-only (NOT saved to dmps), so changing this
// wire-format byte layout is safe — autoc + workers rebuild together
// every commit.
enum class Mode : int {
  PATHGEN = 0,
  TRACKER = 1
};

// String ↔ enum helpers (for AutocConfig parsing from autoc.ini text and
// for human-readable log lines).
inline Mode parseModeName(const std::string& name) {
  if (name == "tracker") return Mode::TRACKER;
  return Mode::PATHGEN;  // default for "pathgen" or any unrecognized value
}
inline const char* modeToString(Mode m) {
  return (m == Mode::TRACKER) ? "tracker" : "pathgen";
}

// 030 V1 priming (UNPARKED 2026-05-08, "Worker-side scenario priming +
// (deferred V2) LRU demand-fetch") — first RPC every worker receives,
// once per worker, immediately after TCP accept. Carries scenario-shaped
// invariants that don't vary per-eval or per-gen, so they don't have to
// ride along with every per-individual EvalData. Eliminates the ~8.5 MB
// sourceList × 5000-individual queue inflation that OOM'd a 128 GB box
// before gen 1 in tracker training. See specs/BACKLOG.md "Worker-side
// scenario priming" entry for the V1/V2 split.
//
// 030 V1.5 (2026-05-08) — extended with pathList + scenarioMetaList.
// gPathSeed is run-constant (set once at startup), so generateSmoothPaths
// produces byte-identical output every gen — no reason to copy it 5000×
// per gen into per-eval EvalDatas. Same for scenario variation offsets
// (pre-fetched joint-PRNG draws, deterministic per scenario index). Both
// hoisted here. Per-eval EvalData collapses to ~50 B.
//
// Wire format: not stored on disk (RPC-only, like EvalData), so adding /
// removing fields is a same-rebuild change. No CEREAL_CLASS_VERSION.
struct WorkerInit {
  // Mode is run-static once .ini is parsed; worker uses it for all
  // dispatch instead of carrying it on each EvalData. Cereal-serialized
  // as int (same roundtrip pattern as ControllerType).
  Mode mode = Mode::PATHGEN;

  // 030 tracker-mode source library (FR-001 + FR-018). Empty in pathgen
  // mode. Worker-resident; per-eval EvalData refers to entries by index
  // into this list rather than copying samples.
  std::vector<SourceScenarioTrajectory> sourceList;

  // Scenario-invariant tracker configs (FR-003, FR-004, FR-008, FR-008b,
  // FR-016). All populated from autoc-tracker.ini once at startup;
  // none vary per-individual or per-generation.
  autoc::eval::CameraConfig cameraConfig;
  autoc::eval::BeaconConfig beaconLeftConfig;
  autoc::eval::BeaconConfig beaconRightConfig;
  autoc::eval::AirframeObstruction airframeObstruction;
  autoc::eval::FlightArena flightArena;

  // 030 M7d.b — crash-hull + trail-rabbit static params. Note
  // pCrashThisGen is NOT here; it ramps per-gen and stays in EvalData.
  gp_scalar crashHullRadius = static_cast<gp_scalar>(1.0);
  gp_scalar trailDistance = static_cast<gp_scalar>(3.048);

  // 032 phase 1 — CepGateThreshold for derived-feature gating. Read from
  // autoc-tracker.ini [DerivedFeatures] CepGateThreshold (default 1.25,
  // matches kCepSentinelThreshold). Same value drives both the autoc
  // crrcsim worker path via this field (workers are separate processes
  // path (via this field — workers are separate processes with no
  // ConfigManager). Per spec.md Q4 + research.md R2 + contracts/
  // ini_schema.md.
  gp_scalar cepGateThreshold = static_cast<gp_scalar>(1.25);

  // 037 T001 -- control-loop cadence (ms), set from ControlIntervalMsec in the
  // .ini (autoc parent has the only ConfigManager). The crrcsim worker has no
  // .ini access, so it takes its NN/sensor interval from here instead of the
  // former AUTOC_EVAL_INTERVAL_MSEC env var. 0 = unset sentinel; the worker
  // fail-louds on 0 (no silent fallback). Per contracts/cadence-config.md.
  unsigned long controlIntervalMsec = 0;

  // 030 V1.5 — run-static scenario library. Built once at startup from
  // generateSmoothPaths(gPathSeed) + the joint-PRNG variation table; the
  // worker dispatches per-eval scenarios by indexing into these in
  // lockstep (pathList[i] is parallel-indexed with scenarioMetaList[i]).
  // scenarioMetaList carries entry offsets at FULL SCALE; the worker
  // applies the per-eval variation_scale via applyVariationScale before
  // running each scenario.
  std::vector<std::vector<Path>> pathList;
  std::vector<ScenarioMetadata> scenarioMetaList;

  // 034 Phase 7 (2026-05-31) — worker needs to know the EnableWindVariations
  // policy to gate the per-scenario CRRC_Random seed. Pre-034: the worker
  // unconditionally seeded the wind simulator from
  // `ClassPRNG(deriveClassSubSeeds(scenarioSeed).wind).next()` regardless
  // of the autoc-side enable flag, so per-tick gusts/thermals VARIED across
  // scenarios even when `EnableWindVariations=0`. That broke the craft-
  // isolation experiment (US5 / autoc-craft-only.ini), where the user
  // expected all 36 scenarios to fly in identical wind/gust environments.
  // When this flag is false, inputdev_autoc uses a fixed constant
  // (kDisabledWindSeed = 0xC0FFEEu) for Global::Simulation->reset() so
  // every scenario sees the same wind/gust sequence. The autoc-side
  // windPRNG draw still happens (draw-and-discard, matches entry/wind
  // contract) so toggling the flag doesn't shift other-class draws.
  // Entry-class variations are already gated via meta.entry* = 0 from
  // autoc-side prefetchAllVariations; rabbit-class variations are gated
  // via cfg.sigma = 0 → generateSpeedProfile short-circuit. Neither needs
  // a separate enable flag here.
  bool enableWindVariations = true;

  // 037 servo v2 -- in-FDM servo model master switch (PWM latch + slew).
  // Set from ServoModelEnabled in the .ini by the autoc parent; the worker
  // gates the fdm_larcsim servo block on it. Per-scenario phase/slew draws
  // run regardless (draw-and-discard, same convention as wind above).
  bool servoModelEnabled = false;

  template<class Archive>
  void serialize(Archive& ar) {
    int m = static_cast<int>(mode);
    ar(m, sourceList, cameraConfig, beaconLeftConfig, beaconRightConfig,
       airframeObstruction, flightArena,
       crashHullRadius, trailDistance,
       pathList, scenarioMetaList,
       cepGateThreshold,
       enableWindVariations,    // 034 Phase 7 -- appended, no version bump
       controlIntervalMsec,     // 037 T001 -- appended, no version bump
       servoModelEnabled);      // 037 servo v2 -- appended, no version bump
    mode = static_cast<Mode>(m);
  }
};

// 030 V1.5 (2026-05-08) — slimmed per-eval EvalData. With pathList /
// scenarioMetaList moved to WorkerInit, per-eval payload collapses to
// NN bytes + a handful of per-eval scalars (~50 B). Eliminates ~7 MB
// pathList × 5000-individual queue inflation per gen and ~1.47M
// brk-arena allocations per gen (294 inner-vec copies × 5000 evals);
// expected autoc residual drops from ~25 GB to a few GB.
struct EvalData {
  std::vector<char> gp;
  uint64_t gpHash = 0;  // FNV-1a hash of payload for verification
  bool isEliteReeval = false;
  ControllerType controllerType = ControllerType::NEURAL_NET;
  RabbitSpeedConfig rabbitSpeedConfig = RabbitSpeedConfig::defaultConfig();

  // 030 V1.5 — per-eval scalars. Worker reconstructs the full per-eval
  // ScenarioMetadata for each scenario by copying init_.scenarioMetaList[i],
  // overriding scenarioSequence + enableDeterministicLogging from these
  // fields, and calling applyVariationScale(meta, variationScale).
  uint64_t scenarioSequence = 0;
  gp_scalar pCrashThisGen = static_cast<gp_scalar>(0.0);  // 030 M7d.b — gen-varying ramp
  gp_scalar variationScale = static_cast<gp_scalar>(1.0); // 030 V1.5 — gen-varying ramp [0..1]

  template<class Archive>
  void serialize(Archive& ar, const std::uint32_t version) {
    ar(gp, gpHash, isEliteReeval);
    int ct = static_cast<int>(controllerType);
    ar(ct);
    controllerType = static_cast<ControllerType>(ct);
    ar(rabbitSpeedConfig, scenarioSequence, pCrashThisGen, variationScale);
  }
};
CEREAL_CLASS_VERSION(EvalData, 1)

// CrashReason factored out to its own header (030 M7a) to break the
// circular include between protocol.h and arena.h. Existing consumers
// of protocol.h see the type unchanged via this re-include. crash_reason.h
// also provides inline crashReasonToString / crashReasonToCStr.
#include "autoc/rpc/crash_reason.h"

struct DebugSample {
  int pathIndex = -1;
  int stepIndex = -1;
  gp_scalar simTimeMsec = static_cast<gp_scalar>(0.0f);
  gp_scalar dtSec = static_cast<gp_scalar>(0.0f);
  gp_scalar simSteps = static_cast<gp_scalar>(0.0f);
  gp_vec3 velRelGround = gp_vec3::Zero();   // FDM ground-relative velocity (m/s)
  gp_vec3 velRelAirmass = gp_vec3::Zero();  // FDM airmass-relative velocity (m/s)
  gp_vec3 position = gp_vec3::Zero();
  gp_vec3 velocity = gp_vec3::Zero();
  gp_vec3 acceleration = gp_vec3::Zero();
  gp_vec3 accelPast = gp_vec3::Zero();
  gp_vec3 angularRates = gp_vec3::Zero();  // body P/Q/R rad/s
  gp_vec3 angularAccelPast = gp_vec3::Zero();
  gp_quat quatDotPast = gp_quat::Identity(); // store e_dot_past as quat-like container
  gp_vec3 windBody = gp_vec3::Zero();      // body-frame wind (m/s)
  gp_quat orientation = gp_quat::Identity();
  gp_scalar pitchCommand = static_cast<gp_scalar>(0.0f);
  gp_scalar rollCommand = static_cast<gp_scalar>(0.0f);
  gp_scalar throttleCommand = static_cast<gp_scalar>(0.0f);
  gp_scalar elevatorSim = static_cast<gp_scalar>(0.0f);
  gp_scalar aileronSim = static_cast<gp_scalar>(0.0f);
  gp_scalar throttleSim = static_cast<gp_scalar>(0.0f);
  gp_scalar massKg = static_cast<gp_scalar>(0.0f);
  gp_scalar density = static_cast<gp_scalar>(0.0f);
  gp_scalar gravity = static_cast<gp_scalar>(0.0f);
  gp_scalar alpha = static_cast<gp_scalar>(0.0f);
  gp_scalar beta = static_cast<gp_scalar>(0.0f);
  gp_scalar vRelWind = static_cast<gp_scalar>(0.0f);
  gp_vec3 localAirmass = gp_vec3::Zero();   // local-frame air velocity
  gp_vec3 gustBody = gp_vec3::Zero();       // body-frame gust applied
  gp_vec3 forceBody = gp_vec3::Zero();      // last total force (body)
  gp_vec3 momentBody = gp_vec3::Zero();     // last moment at CG (body)
  gp_vec3 vLocal = gp_vec3::Zero();         // local-frame velocity (ft/s -> m/s)
  gp_vec3 vLocalDot = gp_vec3::Zero();      // local-frame acceleration (ft/s^2 -> m/s^2)
  gp_vec3 omegaBody = gp_vec3::Zero();      // body rates (rad/s)
  gp_vec3 omegaDotBody = gp_vec3::Zero();   // body angular accel (rad/s^2)
  gp_scalar latGeoc = static_cast<gp_scalar>(0.0f);
  gp_scalar lonGeoc = static_cast<gp_scalar>(0.0f);
  gp_scalar radiusToVehicle = static_cast<gp_scalar>(0.0f); // meters
  gp_scalar latDotPast = static_cast<gp_scalar>(0.0f);
  gp_scalar lonDotPast = static_cast<gp_scalar>(0.0f);
  gp_scalar radiusDotPast = static_cast<gp_scalar>(0.0f);
  uint32_t rngState16 = 0;
  uint32_t rngState32 = 0;

  template<class Archive>
  void serialize(Archive& ar, const std::uint32_t /*version*/) {
    ar(pathIndex, stepIndex, simTimeMsec, dtSec, simSteps,
       velRelGround, velRelAirmass, position, velocity, acceleration,
       accelPast, angularRates, angularAccelPast, quatDotPast, windBody,
       orientation, pitchCommand, rollCommand, throttleCommand,
       elevatorSim, aileronSim, throttleSim, massKg, density, gravity,
       alpha, beta, vRelWind, localAirmass, gustBody, forceBody,
       momentBody, vLocal, vLocalDot, omegaBody, omegaDotBody,
       latGeoc, lonGeoc, radiusToVehicle, latDotPast, lonDotPast,
       radiusDotPast, rngState16, rngState32);
  }
};

// 030 M8a — Per-tick perception sample recorded into v=2 dmps (FR-015).
// One CameraViewSample per camera per NN tick; v1 = single forward camera
// so per-tick is a single sample. Carries camera world-pose (so the
// renderer can place a frustum) + the two BeaconObservations the chase
// craft saw. Self-contained per FR-015 — renderer reads M2 dmp only,
// never reaches back into M1 source dmp.
struct CameraViewSample {
    gp_vec3 camera_pose_world_pos = gp_vec3::Zero();
    gp_quat camera_pose_world_orient = gp_quat::Identity();
    float camera_fov_h_deg = 120.0f;   // raw-ok: cereal byte-format member (M8 v=2 dmp schema)
    float camera_fov_v_deg = 90.0f;    // raw-ok: cereal byte-format member (M8 v=2 dmp schema)
    autoc::eval::BeaconObservation beacon_left{};
    autoc::eval::BeaconObservation beacon_right{};

    template <class Archive>
    void serialize(Archive& ar) {
        ar(camera_pose_world_pos, camera_pose_world_orient,
           camera_fov_h_deg, camera_fov_v_deg,
           beacon_left, beacon_right);
    }
};

// 030 M8a — Per-tick target trajectory copy (FR-015 self-containedness).
// Copied from SourceScenarioTrajectory.samples[i] at the matching sim_time;
// `trail_rabbit_position` is computed by M7 (defaults to position when M7
// hasn't landed); `inside_crash_hull` is M7 telemetry (default false).
struct CopiedTargetSample {
    gp_vec3 position = gp_vec3::Zero();
    gp_quat orientation = gp_quat::Identity();
    gp_vec3 velocity = gp_vec3::Zero();
    gp_vec3 trail_rabbit_position = gp_vec3::Zero();  // FR-008 — M7 wires
    bool inside_crash_hull = false;                    // FR-008b — M7 wires

    template <class Archive>
    void serialize(Archive& ar) {
        ar(position, orientation, velocity, trail_rabbit_position, inside_crash_hull);
    }
};

// 038 P0-D-2 — self-describing run config. The fitness/cadence/crash-penalty
// parameters that used to be read from the live .ini at replay/render time are
// recorded into every dmp, so a dmp replays standalone even against a drifted
// .ini (renderer + dmp_dump prefer this block — P0-B/T010). Populated
// autoc-side once per run from AutocConfig + compiled cadence constants. Wire-
// format record struct (rpc-layer, outside the gp_* eval/nn type-domain).
struct RecordedRunConfig {
  // fitness cone (AutocConfig fit* — config.h)
  double fitDistScaleBehind = 0.0;
  double fitDistScaleAhead = 0.0;
  double fitConeAngleDeg = 0.0;
  double fitStreakThreshold = 0.0;
  double fitStreakRampSec = 0.0;
  double fitStreakMultiplierMax = 0.0;
  // cadence (compiled constants — aircraft_state.h)
  int    simTimeStepMsec = 0;    // = SIM_TIME_STEP_MSEC
  double cadenceTickScale = 0.0; // = kCadenceTickScale (derived)
  // crash penalty (AutocConfig — config.h)
  int    enableHullCrashPenalty = 0;
  double hullCrashPenaltyFactor = 0.0;
  double oobCrashPenaltyWeight = 0.0;

  template<class Archive>
  void serialize(Archive& ar) {
    ar(fitDistScaleBehind, fitDistScaleAhead, fitConeAngleDeg,
       fitStreakThreshold, fitStreakRampSec, fitStreakMultiplierMax,
       simTimeStepMsec, cadenceTickScale,
       enableHullCrashPenalty, hullCrashPenaltyFactor, oobCrashPenaltyWeight);
  }
};

struct EvalResults {
  std::vector<char> gp;
  uint64_t gpHash = 0;  // FNV-1a hash of gp buffer for verification
  std::vector<CrashReason> crashReasonList;
  std::vector<std::vector<Path>> pathList;
  std::vector<std::vector<AircraftState>> aircraftStateList;
  ScenarioMetadata scenario;
  std::vector<ScenarioMetadata> scenarioList;
  std::vector<std::vector<DebugSample>> debugSamples;  // Debug snapshots per path (only populated for elite reeval)
  std::vector<std::vector<PhysicsTraceEntry>> physicsTrace;  // Full physics state trace per path (only populated for elite reeval)
  int workerId = -1;
  int workerPid = 0;
  int workerEvalCounter = 0;  // Incremented per evaluation on the worker

  // 030 M8a — Tracker-mode v=2 fields (FR-015). Empty in pathgen-mode dmps;
  // populated by TrackerStepper at M8b. cameraViewList[scenario][tick]
  // and targetTrajectoryList[scenario][tick] parallel aircraftStateList's
  // per-scenario per-tick indexing. arenaEgressCount + hullStrikeCount
  // are per-scenario telemetry counters wired by M7.
  std::vector<std::vector<CameraViewSample>> cameraViewList;
  std::vector<std::vector<CopiedTargetSample>> targetTrajectoryList;
  std::vector<int> arenaEgressCount;  // M7 — per-scenario count of arena egress events
  std::vector<int> hullStrikeCount;   // M7 — per-scenario count of crash-hull p_crash fires

  // 033 §2.A + §2.B — Provenance header. Makes a dmp self-describing for
  // analysis: any operator picking up a dmp can read off the master seed
  // (full-run reproduction) without needing the matching log file or ini.
  // Populated autoc-side; the value is set once per run, copies into every
  // emitted dmp.
  uint64_t effectiveMasterSeed = 0;                              // 033 §2.A — MasterPRNG.init() input

  // 038 P0-D-2 — self-describing run config (fitness cone / cadence / crash
  // penalty). Appended at end of the v>=2 block per the no-cereal-versioning
  // policy; old dmps orphaned by the training reset. Read by renderer/dmp_dump
  // (T010) in preference to the live .ini.
  RecordedRunConfig runConfig;

  template<class Archive>
  void serialize(Archive& ar, const std::uint32_t version) {
    ar(gp, gpHash, crashReasonList, pathList, aircraftStateList,
       scenario, scenarioList, debugSamples, physicsTrace,
       workerId, workerPid, workerEvalCounter);
    if (version >= 2) {
      // 030 M8a — tracker-mode v=2 schema additions. v=1 dmps (pathgen
      // historical) read with these vectors empty; v=2 readers see full
      // population. Constitution V loud-fail behavior for v=3 lands via
      // cereal's class-version mechanism (verified in
      // tests/tracker_dmp_roundtrip_tests.cc).
      ar(cameraViewList, targetTrajectoryList, arenaEgressCount, hullStrikeCount);

      // 033 §2.A — master-seed provenance. Appended at end of v=2 block per
      // project no-cereal-versioning policy; old dmps are orphaned by the
      // training reset (M2-era no-version-revision policy).
      ar(effectiveMasterSeed);

      // 038 P0-D-2 — self-describing run config, appended after the master
      // seed (same no-version-bump policy; old dmps orphaned by retrain).
      ar(runConfig);
    }
  }

  void clear() {
    gp.clear();
    gpHash = 0;
    crashReasonList.clear();
    pathList.clear();
    aircraftStateList.clear();
    scenario = ScenarioMetadata();
    scenarioList.clear();
    debugSamples.clear();
    physicsTrace.clear();
    workerId = -1;
    workerPid = 0;
    workerEvalCounter = 0;
    // 030 M8a — tracker-mode v=2 fields.
    cameraViewList.clear();
    targetTrajectoryList.clear();
    arenaEgressCount.clear();
    hullStrikeCount.clear();
    // 033 §2.A — provenance header reset to "no-context" default.
    effectiveMasterSeed = 0;
    // 038 P0-D-2 — self-describing run config reset.
    runConfig = RecordedRunConfig();
  }

  void dump(std::ostream& os) {
    char buf[512];
    snprintf(buf, sizeof(buf), "EvalResults: crash[%zu] paths[%zu] states[%zu]\n Paths:\n",
      crashReasonList.size(), pathList.size(), aircraftStateList.size());
    os << buf;

    // 033 cleanup: windSeed display swapped for scenarioSeed (the new
    // per-scenario PRNG root). All class sub-seeds derive from this via
    // deriveClassSubSeeds.
    snprintf(buf, sizeof(buf), "Scenario: pathVariant=%d windVariant=%d scenarioSeed=0x%016llx seq=%llu bake=%llu\n",
      scenario.pathVariantIndex, scenario.windVariantIndex,
      static_cast<unsigned long long>(scenario.scenarioSeed),
      static_cast<unsigned long long>(scenario.scenarioSequence),
      static_cast<unsigned long long>(scenario.bakeoffSequence));
    os << buf;
    for (size_t i = 0; i < scenarioList.size(); ++i) {
      snprintf(buf, sizeof(buf), "  Scenario[%zu]: pathVariant=%d windVariant=%d scenarioSeed=0x%016llx seq=%llu bake=%llu\n",
        i, scenarioList[i].pathVariantIndex, scenarioList[i].windVariantIndex,
        static_cast<unsigned long long>(scenarioList[i].scenarioSeed),
        static_cast<unsigned long long>(scenarioList[i].scenarioSequence),
        static_cast<unsigned long long>(scenarioList[i].bakeoffSequence));
      os << buf;
    }

    for (size_t i = 0; i < crashReasonList.size(); i++) {
      snprintf(buf, sizeof(buf), "  Crash %3zu: %s\n", i, crashReasonToString(crashReasonList.at(i)).c_str());
      os << buf;
    }

    for (size_t i = 0; i < pathList.size(); i++) {
      for (size_t j = 0; j < pathList.at(i).size(); j++) {
        Path path = pathList.at(i).at(j);
        snprintf(buf, sizeof(buf), "  Path %3zu:%3zu: start[%8.2f %8.2f %8.2f] orient[%8.2f %8.2f %8.2f] dist[%8.2f] rad[%8.2f]\n",
          i, j,
          path.start[0], path.start[1], path.start[2],
          path.orientation.x(), path.orientation.y(), path.orientation.z(),
          path.distanceFromStart, path.radiansFromStart);
        os << buf;
      }
    }
    os << " Aircraft States:\n";
    for (size_t i = 0; i < aircraftStateList.size(); i++) {
      for (size_t j = 0; j < aircraftStateList.at(i).size(); j++) {
        AircraftState aircraftState = aircraftStateList.at(i).at(j);
        gp_quat orientQuatF = aircraftState.getOrientation();
        if (!std::isnan(orientQuatF.norm()) && std::abs(orientQuatF.norm() - 1.0f) > 1e-6f) {
          orientQuatF.normalize();
        }

        Eigen::Matrix<gp_scalar, 3, 1> euler = orientQuatF.toRotationMatrix().eulerAngles(2, 1, 0);
        gp_vec3 eulerWrapped;
        for (int axis = 0; axis < 3; ++axis) {
          eulerWrapped[axis] = std::atan2(std::sin(euler[axis]), std::cos(euler[axis]));
        }
        snprintf(buf, sizeof(buf),
          "  Path %3zu:%3zu: Time %5ld Index %3d: pos[%8.2f %8.2f %8.2f] orientRPY[%8.2f %8.2f %8.2f] quat[%+7.4f %+7.4f %+7.4f %+7.4f] vel[%8.2f] pitch[%5.2f] roll[%5.2f] throttle[%5.2f]\n",
          i, j,
          aircraftState.getSimTimeMsec(),
          aircraftState.getThisPathIndex(),
          aircraftState.getPosition()[0], aircraftState.getPosition()[1], aircraftState.getPosition()[2],
          eulerWrapped.x(), eulerWrapped.y(), eulerWrapped.z(),
          orientQuatF.w(), orientQuatF.x(), orientQuatF.y(), orientQuatF.z(),
          aircraftState.getRelVel(),
          aircraftState.getPitchCommand(),
          aircraftState.getRollCommand(),
          aircraftState.getThrottleCommand());
        os << buf;
      }
    }
  }
};
// 030 M8a — bumped 1 → 2 for tracker-mode dmp output (FR-015a + Constitution V).
// v=1 dmps still readable: cameraViewList / targetTrajectoryList /
// arenaEgressCount / hullStrikeCount remain empty when reading v=1.
// v=3+ dmps fail loudly via cereal's class-version mechanism — caller
// should treat as a Constitution V "schema mismatch" loud-fail trigger.
CEREAL_CLASS_VERSION(EvalResults, 2)

struct WorkerContext {
  std::unique_ptr<TcpSocket> socket;
  pid_t childPid = 0;
  EvalResults evalResults;
  int workerId = -1;
};

/*
 * generic RPC wrappers
 */
template<typename T>
void sendRPC(TcpSocket& socket, const T& data) {
  std::ostringstream os(std::ios::binary);
  {
    cereal::BinaryOutputArchive archive(os);
    archive(data);
  }
  std::string outbound_data = os.str();
  uint32_t size = htonl(static_cast<uint32_t>(outbound_data.size()));
  socket.write(&size, sizeof(size));
  socket.write(outbound_data.data(), outbound_data.size());
}

template<typename T>
T receiveRPC(TcpSocket& socket) {
  uint32_t size;
  socket.read(&size, sizeof(size));
  size = ntohl(size);
  std::vector<char> buffer(size);
  socket.read(buffer.data(), buffer.size());
  std::istringstream is(std::string(buffer.begin(), buffer.end()), std::ios::binary);
  cereal::BinaryInputArchive archive(is);
  T data;
  archive(data);
  return data;
}


#endif
