/* test sim for aircraft */
#ifndef MINISIM_H
#define MINISIM_H

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
#include "autoc/eval/beacon_config.h"
#include "autoc/eval/camera_config.h"
#include "autoc/eval/camera_projection.h"   // AirframeProxy
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

struct EvalData {
  std::vector<char> gp;
  uint64_t gpHash = 0;  // FNV-1a hash of payload for verification
  bool isEliteReeval = false;
  ControllerType controllerType = ControllerType::NEURAL_NET;
  std::vector<std::vector<Path>> pathList;
  ScenarioMetadata scenario;
  std::vector<ScenarioMetadata> scenarioList;
  RabbitSpeedConfig rabbitSpeedConfig = RabbitSpeedConfig::defaultConfig();

  // 030 M6c — runtime mode dispatch (FR-019). Worker selects PathgenStepper
  // vs TrackerStepper based on this field. "pathgen" is the default to
  // preserve existing pathgen-only behavior across the M6c plumbing commit.
  // "tracker" fires the source-trajectory-driven per-tick path (M6d/e).
  std::string mode = "pathgen";

  // 030 M6e — tracker-mode per-scenario payload (FR-001 + FR-018). Empty
  // when mode == "pathgen". When mode == "tracker", sourceList[i] is the
  // source-craft trajectory the chase craft for scenario i tracks; the
  // configs below are scenario-invariant (one set per worker job).
  std::vector<SourceScenarioTrajectory> sourceList;
  autoc::eval::CameraConfig cameraConfig;
  autoc::eval::BeaconConfig beaconLeftConfig;
  autoc::eval::BeaconConfig beaconRightConfig;
  autoc::eval::AirframeProxy airframeProxy;
  gp_vec3 homeWorld = gp_vec3::Zero();

  // 030 M8b geometry-fix — source pre-roll. Worker skips the first N
  // source-tick samples before chase starts evolving so source has a
  // head start. Computed at autoc-side from
  // ConfigManager::config->trackerSourcePreRollSec / 0.1; pre-converted
  // to ticks to keep the worker math integer.
  int trackerSourcePreRollTicks = 0;

  template<class Archive>
  void serialize(Archive& ar, const std::uint32_t version) {
    ar(gp, gpHash, isEliteReeval);
    int ct = static_cast<int>(controllerType);
    ar(ct);
    controllerType = static_cast<ControllerType>(ct);
    ar(pathList, scenario, scenarioList, rabbitSpeedConfig, mode);
    ar(sourceList, cameraConfig, beaconLeftConfig, beaconRightConfig,
       airframeProxy, homeWorld, trackerSourcePreRollTicks);
  }

  void sanitizePaths() {
    for (auto& pathGroup : pathList) {
      for (auto& path : pathGroup) {
        path.sanitize();
      }
    }
  }
};
CEREAL_CLASS_VERSION(EvalData, 1)

enum class CrashReason {
  None,             // Still flying
  Boot,             // Startup/initialization error
  Sim,              // Physics crash (ground impact, structural failure)
  Eval,             // Out of bounds (too far, too low, too high)
  TimeLimit,        // Simulation time cap reached (was: Time)
  RabbitComplete,   // Rabbit reached end of path — normal completion (was: Distance)
};

// Is this a real crash (penalizable) or normal termination?
inline bool isCrash(CrashReason reason) {
  return reason == CrashReason::Boot ||
         reason == CrashReason::Sim ||
         reason == CrashReason::Eval;
}

std::string crashReasonToString(CrashReason type);

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
  }

  void dump(std::ostream& os) {
    char buf[512];
    snprintf(buf, sizeof(buf), "EvalResults: crash[%zu] paths[%zu] states[%zu]\n Paths:\n",
      crashReasonList.size(), pathList.size(), aircraftStateList.size());
    os << buf;

    snprintf(buf, sizeof(buf), "Scenario: pathVariant=%d windVariant=%d windSeed=%u seq=%llu bake=%llu\n",
      scenario.pathVariantIndex, scenario.windVariantIndex, scenario.windSeed,
      static_cast<unsigned long long>(scenario.scenarioSequence),
      static_cast<unsigned long long>(scenario.bakeoffSequence));
    os << buf;
    for (size_t i = 0; i < scenarioList.size(); ++i) {
      snprintf(buf, sizeof(buf), "  Scenario[%zu]: pathVariant=%d windVariant=%d windSeed=%u seq=%llu bake=%llu\n",
        i, scenarioList[i].pathVariantIndex, scenarioList[i].windVariantIndex, scenarioList[i].windSeed,
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
