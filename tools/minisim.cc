/* minisim — lightweight simulator for NN controller evaluation */
#include <vector>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unistd.h>

#include "autoc/rpc/protocol.h"
#include "autoc/autoc.h"
#include "autoc/eval/pathgen_stepper.h"
#include "autoc/eval/tracker_stepper.h"
#include "autoc/eval/sensor_math.h"
#include "autoc/nn/mode.h"               // 030 M7a — getModeStrategyByName
#include "autoc/nn/nn_input_computation.h"
#include "autoc/nn/serialization.h"
#include "autoc/nn/evaluator.h"

using namespace std;

// Global aircraft state (declared extern in autoc.h)
AircraftState aircraftState;

std::string crashReasonToString(CrashReason type) {
  switch (type) {
  case CrashReason::None: return "None";
  case CrashReason::Boot: return "Boot";
  case CrashReason::Sim: return "Sim";
  case CrashReason::Eval: return "Eval";
  case CrashReason::TimeLimit: return "TimeLimit";
  case CrashReason::RabbitComplete: return "RabbitComplete";
  case CrashReason::HullStrike: return "HullStrike";
  default: return "*?*";
  }
}

namespace {

void ensureScenarioMetadata(EvalData& evalData) {
  if (evalData.scenarioList.size() == evalData.pathList.size()) {
    return;
  }
  evalData.scenarioList.assign(evalData.pathList.size(), evalData.scenario);
  for (size_t idx = 0; idx < evalData.scenarioList.size(); ++idx) {
    if (evalData.scenarioList[idx].pathVariantIndex < 0) {
      evalData.scenarioList[idx].pathVariantIndex = static_cast<int>(idx);
    }
  }
}

ScenarioMetadata scenarioForPathIndex(const EvalData& evalData, size_t idx) {
  if (idx < evalData.scenarioList.size()) {
    return evalData.scenarioList.at(idx);
  }
  ScenarioMetadata meta = evalData.scenario;
  if (meta.pathVariantIndex < 0) {
    meta.pathVariantIndex = static_cast<int>(idx);
  }
  return meta;
}

} // namespace

class SimProcess {
public:
  SimProcess(int id, unsigned short port) {
    socket_.connect("localhost", port);
    workerPid = static_cast<int>(getpid());
    workerId = id;
  }

  void run() {
    while (true) {
      EvalResults evalResults;
      evalResults.workerPid = workerPid;
      evalResults.workerId = workerId;
      evalResults.workerEvalCounter = ++evalCounter;

      EvalData evalData = receiveRPC<EvalData>(socket_);

      const uint64_t localGpHash = hashByteVector(evalData.gp);
      if (evalData.gpHash == 0) {
        evalData.gpHash = localGpHash;
      } else if (evalData.gpHash != localGpHash) {
        std::cerr << "[HASH_MISMATCH] workerId=" << workerId
                  << " expected=0x" << std::hex << evalData.gpHash
                  << " got=0x" << localGpHash << std::dec
                  << " size=" << evalData.gp.size()
                  << std::endl;
      }

      ensureScenarioMetadata(evalData);

      evalResults.gp = evalData.gp;
      evalResults.gpHash = localGpHash;
      if (!evalData.scenarioList.empty()) {
        evalResults.scenario = evalData.scenarioList.front();
      } else {
        evalResults.scenario = evalData.scenario;
      }
      evalResults.scenarioList.clear();
      evalResults.scenarioList.reserve(evalData.scenarioList.size());

      // Deserialize NN genome
      NNGenome nnGenome;
      if (!nn_detect_format(reinterpret_cast<const uint8_t*>(evalData.gp.data()), evalData.gp.size())) {
        std::cerr << "[MINISIM] Payload missing NN01 magic bytes, size=" << evalData.gp.size() << std::endl;
        continue;
      }
      if (!nn_deserialize(reinterpret_cast<const uint8_t*>(evalData.gp.data()), evalData.gp.size(), nnGenome)) {
        std::cerr << "[MINISIM] Failed to deserialize NN genome" << std::endl;
        continue;
      }

      // Validate topology matches the active mode's compile-time expectation
      // (030 M7a — FR-019 runtime mode dispatch). Pathgen genomes are
      // 33-input, tracker genomes are 45-input; minisim picks the
      // expected shape from the active ModeStrategy.
      {
        const ModeStrategy& mode = getModeStrategy(evalData.mode);
        std::vector<int> expectedTopology(mode.topology, mode.topology + mode.num_layers);
        if (nnGenome.topology != expectedTopology) {
          std::cerr << "[MINISIM] NN topology mismatch (mode=" << mode.name << "): "
                    << "file has " << nnGenome.weights.size()
                    << " weights but binary expects "
                    << mode.weight_count << " (" << mode.topology_string << ")"
                    << std::endl;
          continue;
        }
      }

      // Log topology once on first eval
      if (evalCounter == 1) {
        std::ostringstream topo;
        for (size_t i = 0; i < nnGenome.topology.size(); i++) {
          if (i > 0) topo << "x";
          topo << nnGenome.topology[i];
        }
        std::cerr << "[MINISIM] worker=" << workerId
                  << " topology=" << topo.str()
                  << " weights=" << nnGenome.weights.size()
                  << std::endl;
      }

      // Per-span NN controller — constructed once per span so the
      // recurrent hidden state (spec 027 D-simple) persists across ticks
      // within a scenario. reset() called at span start. Feedforward
      // genomes use this same instance with an empty hidden state buffer.
      NNControllerBackend nnBackend(nnGenome);

      // Evaluate each scenario. Per-tick logic lives behind the
      // ScenarioStepper strategy (030 M6a / FR-019). Mode dispatch (M6c):
      // EvalData::mode selects PathgenStepper vs TrackerStepper. Tracker
      // mode is wire-protocol-only in M6c — TrackerStepper lands at M6d
      // and source trajectories at M6e; until then, tracker scenarios
      // exit cleanly with a clear log so operator can verify the mode
      // signal flows end-to-end.
      const Mode evalMode = evalData.mode;
      if (evalCounter == 1) {
        std::cerr << "[MINISIM] worker=" << workerId
                  << " mode=" << modeToString(evalMode) << std::endl;
      }
      for (int i = 0; i < static_cast<int>(evalData.pathList.size()); i++) {
        // Path stays at canonical origin (Z=0); origin offset bridges raw→virtual
        std::vector<Path> path = evalData.pathList.at(i);
        std::vector<AircraftState> aircraftStateSteps;
        // 030 M8b — per-tick recording for tracker-mode dmp (FR-015).
        // Empty for pathgen scenarios; populated for tracker scenarios.
        std::vector<CameraViewSample> cameraViewSteps;
        std::vector<CopiedTargetSample> targetSampleSteps;

        ScenarioMetadata scenarioMeta = scenarioForPathIndex(evalData, static_cast<size_t>(i));
        CrashReason crashReason = CrashReason::None;
        int tracker_hull_fired = 0;  // 030 M7d.b — set by tracker branch

        if (evalMode == Mode::TRACKER) {
          // M6e — TrackerStepper integration. EvalData::sourceList[i]
          // carries this scenario's source trajectory; camera/beacon/
          // airframe/home come from the per-job evalData configs. If
          // sourceList is shorter than pathList (autoc populated fewer
          // sources than scenarios), fall back to Eval-crash for that
          // scenario rather than running with stale data.
          if (static_cast<size_t>(i) >= evalData.sourceList.size()
              || evalData.sourceList[i].samples.empty()) {
            if (evalCounter == 1) {
              std::cerr << "[MINISIM] tracker scenario " << i
                        << " missing source trajectory; recording Eval-crash."
                        << std::endl;
            }
            aircraftStateSteps.push_back(aircraftState);
            crashReason = CrashReason::Eval;
          } else {
            const SourceScenarioTrajectory& source = evalData.sourceList[i];

            // 030 M7d.b — CrashHull config from EvalData::crashHullRadius
            // (radius_m configurable via autoc-tracker.ini; SPHERE shape
            // hardcoded for v1, AABB_HB1 / MESH_AIRFRAME reserved). PRNG
            // seed = scenarioSequence cast to u32 — deterministic per
            // scenario, stream-isolated across scenarios.
            autoc::eval::CrashHull crashHull;
            crashHull.sphere_radius_m = evalData.crashHullRadius;
            const uint32_t prngSeed =
                static_cast<uint32_t>(scenarioMeta.scenarioSequence);
            autoc::eval::TrackerStepper stepper(
                nnBackend, aircraftState, source, scenarioMeta,
                evalData.cameraConfig,
                evalData.beaconLeftConfig,
                evalData.beaconRightConfig,
                evalData.airframeProxy,
                evalData.flightArena,
                evalData.trackerSourcePreRollTicks,
                crashHull,
                evalData.pCrashThisGen,
                prngSeed,
                evalData.trailDistance);
            stepper.initScenario();
            aircraftStateSteps.push_back(aircraftState);
            // 030 M8b — record initial-tick projection (initScenario
            // warm-starts history with the first source tick's
            // projection, populating lastCameraView / lastTargetSample).
            cameraViewSteps.push_back(stepper.lastCameraView());
            targetSampleSteps.push_back(stepper.lastTargetSample());

            while (crashReason == CrashReason::None) {
              crashReason = stepper.stepOnce();
              aircraftStateSteps.push_back(aircraftState);
              cameraViewSteps.push_back(stepper.lastCameraView());
              targetSampleSteps.push_back(stepper.lastTargetSample());
            }

            // 030 M7d.b — capture hull-fired count for hullStrikeCount[i]
            // pushback below. 0 typically; 1 if didCrashFire returned true
            // (scenario terminated with HullStrike).
            tracker_hull_fired = stepper.hullFiredCount();
          }
        } else {
          // pathgen-mode (default): existing behavior, byte-identical to M6a.
          autoc::eval::PathgenStepper stepper(nnBackend, aircraftState, path, scenarioMeta);
          stepper.initScenario();
          aircraftStateSteps.push_back(aircraftState);

          while (crashReason == CrashReason::None) {
            crashReason = stepper.stepOnce();
            aircraftStateSteps.push_back(aircraftState);
          }
        }

        {
          ScenarioMetadata meta = scenarioMeta;
          meta.originOffset = gp_vec3(0.0f, 0.0f, SIM_INITIAL_ALTITUDE);  // raw→virtual offset for renderer
          evalResults.scenarioList.push_back(meta);
        }
        evalResults.pathList.push_back(path);
        evalResults.aircraftStateList.push_back(aircraftStateSteps);
        evalResults.crashReasonList.push_back(crashReason);
        // 030 M8b — push per-tick tracker recording (empty in pathgen mode).
        // Renderer / per-tick extractor dispatches on cameraViewList[i]
        // empty-vs-populated to decide pathgen vs tracker render.
        evalResults.cameraViewList.push_back(cameraViewSteps);
        evalResults.targetTrajectoryList.push_back(targetSampleSteps);
        // 030 M7d.a/b — per-scenario counters. Arena egress: in tracker
        // mode, CrashReason::Eval is sourced exclusively from
        // checkArenaBounds (no other Eval source today), so derive
        // directly. Pathgen pushes 0 (no arena.h enforcement on M1
        // envelope). Hull-strike count: TrackerStepper.hullFiredCount()
        // returns 0 or 1 (scenario terminates on first fire). Pathgen 0.
        const int arena_egress = (evalMode == Mode::TRACKER
                                  && crashReason == CrashReason::Eval) ? 1 : 0;
        evalResults.arenaEgressCount.push_back(arena_egress);
        evalResults.hullStrikeCount.push_back(tracker_hull_fired);
      }

      sendRPC(socket_, evalResults);

      evalResults.pathList.clear();
      evalResults.aircraftStateList.clear();
      evalResults.cameraViewList.clear();
      evalResults.targetTrajectoryList.clear();
      evalResults.crashReasonList.clear();
      evalResults.scenarioList.clear();
    }
  }

private:
  TcpSocket socket_;
  int workerPid = 0;
  int workerId = 0;
  int evalCounter = 0;
};

int main(int argc, char* argv[]) {
  if (argc != 4) {
    std::cerr << "Usage: minisim <base> <id> <port>" << std::endl;
    return 1;
  }

  int id = std::atoi(argv[2]);
  unsigned short port = std::atoi(argv[3]);
  SimProcess sim_process(id, port);
  sim_process.run();

  return 0;
}
