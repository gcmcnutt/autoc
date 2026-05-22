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
#include "autoc/eval/scenario_meta_apply.h"  // 030 V1.5 — applyVariationScale
#include "autoc/eval/tracker_stepper.h"
#include "autoc/eval/sensor_math.h"
#include "autoc/util/scenario_prng.h"  // 033 — deriveClassSubSeeds
#include "autoc/nn/mode.h"               // 030 M7a — getModeStrategyByName
#include "autoc/nn/nn_input_computation.h"
#include "autoc/nn/serialization.h"
#include "autoc/nn/evaluator.h"

using namespace std;

// Global aircraft state (declared extern in autoc.h)
AircraftState aircraftState;

namespace {

// 030 V1.5 — build the per-eval ScenarioMetadata for scenario index idx
// by copying the cached base meta from init_.scenarioMetaList[idx],
// overriding the per-eval fields (scenarioSequence,
// enableDeterministicLogging) from EvalData, then applying the per-eval
// variation_scale. Byte-equivalent to the legacy autoc-side
// populateVariationOffsets path; preserves training fitness determinism.
ScenarioMetadata makePerEvalMeta(const WorkerInit& init,
                                 const EvalData& evalData,
                                 size_t idx) {
  ScenarioMetadata meta;
  if (idx < init.scenarioMetaList.size()) {
    meta = init.scenarioMetaList[idx];
  } else {
    meta.pathVariantIndex = static_cast<int>(idx);
  }
  meta.scenarioSequence = evalData.scenarioSequence;
  meta.bakeoffSequence = 0;
  meta.enableDeterministicLogging = evalData.isEliteReeval;
  autoc::eval::applyVariationScale(meta, evalData.variationScale);
  return meta;
}

} // namespace

class SimProcess {
public:
  SimProcess(int id, unsigned short port) {
    socket_.connect("localhost", port);
    workerPid = static_cast<int>(getpid());
    workerId = id;

    // 030 V1 priming (2026-05-08) — first RPC after TCP connect is
    // WorkerInit (sent by autoc's ThreadPool right after our accept on
    // the other side). Cache mode + source library + camera/beacon/
    // airframe/flightArena/preroll/crashHull/trail locally; per-eval
    // EvalData no longer carries them. See specs/BACKLOG.md
    // "Worker-side scenario priming" entry. Loud-fail if the priming
    // RPC fails — we can't run without it.
    init_ = receiveRPC<WorkerInit>(socket_);
    std::cerr << "[MINISIM] worker=" << workerId
              << " primed mode=" << modeToString(init_.mode)
              << " sourceList=" << init_.sourceList.size() << " scenarios"
              << std::endl;
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

      evalResults.gp = evalData.gp;
      evalResults.gpHash = localGpHash;
      // 030 V1.5 — scenario library is worker-resident in init_; per-eval
      // EvalData no longer carries pathList / scenarioList. Reserve based
      // on the cached pathList size.
      const size_t numScenarios = init_.pathList.size();
      evalResults.scenarioList.clear();
      evalResults.scenarioList.reserve(numScenarios);

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
      // expected shape from the active ModeStrategy. 030 V1 priming —
      // mode now comes from init_ (sent once at startup), not per-eval.
      const Mode evalMode = init_.mode;
      {
        const ModeStrategy& mode = getModeStrategy(evalMode);
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

      NNControllerBackend nnBackend(nnGenome);

      // 030 V1.5 — set evalResults.scenario from the first per-eval meta
      // we'll build below (kept for downstream renderer/log compat).
      bool firstScenarioSet = false;

      for (int i = 0; i < static_cast<int>(numScenarios); i++) {
        const std::vector<Path>& path = init_.pathList.at(i);
        std::vector<AircraftState> aircraftStateSteps;
        std::vector<CameraViewSample> cameraViewSteps;
        std::vector<CopiedTargetSample> targetSampleSteps;

        ScenarioMetadata scenarioMeta = makePerEvalMeta(init_, evalData, static_cast<size_t>(i));
        if (!firstScenarioSet) {
          evalResults.scenario = scenarioMeta;
          firstScenarioSet = true;
        }
        CrashReason crashReason = CrashReason::None;
        int tracker_hull_fired = 0;

        if (evalMode == Mode::TRACKER) {
          // 030 V1 priming — source library is worker-resident in init_.
          // Index by scenario position with modulo wrap-around (preserves
          // the pre-priming mapping where evalData.sourceList[i] was
          // gSourceTrajectoryList[i % gSourceTrajectoryList.size()]).
          if (init_.sourceList.empty()) {
            if (evalCounter == 1) {
              std::cerr << "[MINISIM] tracker mode but priming sourceList empty;"
                        << " recording Eval-crash for scenario " << i << std::endl;
            }
            aircraftStateSteps.push_back(aircraftState);
            crashReason = CrashReason::Eval;
          } else {
            const size_t srcIdx = static_cast<size_t>(i) % init_.sourceList.size();
            const SourceScenarioTrajectory& source = init_.sourceList[srcIdx];

            if (source.samples.empty()) {
              if (evalCounter == 1) {
                std::cerr << "[MINISIM] tracker scenario " << i
                          << " (srcIdx=" << srcIdx << ") source samples empty;"
                          << " recording Eval-crash." << std::endl;
              }
              aircraftStateSteps.push_back(aircraftState);
              crashReason = CrashReason::Eval;
            } else {
              autoc::eval::CrashHull crashHull;
              crashHull.sphere_radius_m = init_.crashHullRadius;
              // 033 cleanup — crash-hull PRNG seed now derives from the
              // RABBIT class sub-PRNG (per contracts/scenario_prng_chain.md
              // — M2 crash-hull is a rabbit-class consumer). Pre-033 used
              // ScenarioMetadata.windSeed; that field has been removed.
              // Deriving from meta.scenarioSeed via deriveClassSubSeeds
              // gives stable per-(path, wind) determinism — same scenario
              // gets identical crash-hull draws across train/elite-reeval.
              const auto subseeds =
                  autoc::util::deriveClassSubSeeds(scenarioMeta.scenarioSeed);
              const uint32_t prngSeed = subseeds.rabbit;
              // 033 §2.B — decode WorkerInit wire-format motion mode
              // (uint8) → autoc::eval::SmoothnessMotionMode enum at
              // scenario-construct boundary. Floor passes through as
              // gp_scalar. Mode validation already done at autoc-side
              // ini parse; defensive default below for any unrecognized
              // wire value.
              autoc::eval::SmoothnessMotionMode smoothMode =
                  autoc::eval::SmoothnessMotionMode::Pythagorean;
              switch (init_.smoothnessMotionMode) {
                  case 1: smoothMode = autoc::eval::SmoothnessMotionMode::Sum; break;
                  case 2: smoothMode = autoc::eval::SmoothnessMotionMode::Max; break;
                  default: smoothMode = autoc::eval::SmoothnessMotionMode::Pythagorean; break;
              }
              autoc::eval::TrackerStepper stepper(
                  nnBackend, aircraftState, source, scenarioMeta,
                  init_.cameraConfig,
                  init_.beaconLeftConfig,
                  init_.beaconRightConfig,
                  init_.airframeProxy,
                  init_.flightArena,
                  crashHull,
                  evalData.pCrashThisGen,
                  prngSeed,
                  init_.trailDistance,
                  init_.smoothnessPenaltyFloor,
                  smoothMode);
              stepper.initScenario();
              aircraftStateSteps.push_back(aircraftState);
              cameraViewSteps.push_back(stepper.lastCameraView());
              targetSampleSteps.push_back(stepper.lastTargetSample());

              while (crashReason == CrashReason::None) {
                crashReason = stepper.stepOnce();
                aircraftStateSteps.push_back(aircraftState);
                cameraViewSteps.push_back(stepper.lastCameraView());
                targetSampleSteps.push_back(stepper.lastTargetSample());
              }

              tracker_hull_fired = stepper.hullFiredCount();
            }
          }
        } else {
          // pathgen-mode (default). 033 §2.B — wires smoothness floor +
          // motion mode from WorkerInit (decoded from wire-format uint8).
          autoc::eval::SmoothnessMotionMode pgSmoothMode =
              autoc::eval::SmoothnessMotionMode::Pythagorean;
          switch (init_.smoothnessMotionMode) {
              case 1: pgSmoothMode = autoc::eval::SmoothnessMotionMode::Sum; break;
              case 2: pgSmoothMode = autoc::eval::SmoothnessMotionMode::Max; break;
              default: pgSmoothMode = autoc::eval::SmoothnessMotionMode::Pythagorean; break;
          }
          autoc::eval::PathgenStepper stepper(
              nnBackend, aircraftState, path, scenarioMeta,
              init_.smoothnessPenaltyFloor, pgSmoothMode);
          stepper.initScenario();
          aircraftStateSteps.push_back(aircraftState);

          while (crashReason == CrashReason::None) {
            crashReason = stepper.stepOnce();
            aircraftStateSteps.push_back(aircraftState);
          }
        }

        {
          ScenarioMetadata meta = scenarioMeta;
          meta.originOffset = gp_vec3(0.0f, 0.0f, SIM_INITIAL_ALTITUDE);
          evalResults.scenarioList.push_back(meta);
        }
        evalResults.pathList.push_back(path);  // 030 V1.5 — copied from init_.pathList[i]
        evalResults.aircraftStateList.push_back(aircraftStateSteps);
        evalResults.crashReasonList.push_back(crashReason);
        // 033 troubleshooting 2026-05-22 — bug fix: cameraViewList +
        // targetTrajectoryList are TRACKER-MODE-ONLY per the EvalResults
        // contract (protocol.h:359 "Empty in pathgen-mode dmps"). Prior
        // unconditional push_back of (empty in pathgen) inner vectors
        // produced an outer-vector-non-empty dmp that the renderer
        // (renderer.cc:400-402 dispatches on outer-vector emptiness)
        // mis-classified as tracker mode → rabbit/path render path
        // skipped. Worker-side fix preserves the documented contract;
        // autoc-side fitness_decomposition + data.dat already correctly
        // check inner-vec emptiness via `is_tracker`, so this fix is
        // additive (no autoc-side ripple).
        if (evalMode == Mode::TRACKER) {
            evalResults.cameraViewList.push_back(cameraViewSteps);
            evalResults.targetTrajectoryList.push_back(targetSampleSteps);
        }
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
  WorkerInit init_;
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
