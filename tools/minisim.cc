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
#include "autoc/eval/sensor_math.h"
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

      // Validate topology matches compiled-in expectations
      {
        std::vector<int> expectedTopology(NN_TOPOLOGY, NN_TOPOLOGY + NN_NUM_LAYERS);
        if (nnGenome.topology != expectedTopology) {
          std::cerr << "[MINISIM] NN topology mismatch: file has "
                    << nnGenome.weights.size() << " weights but binary expects "
                    << NN_WEIGHT_COUNT << " (" << NN_TOPOLOGY_STRING << ")" << std::endl;
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

      // Evaluate each path
      for (int i = 0; i < static_cast<int>(evalData.pathList.size()); i++) {
        // Path stays at canonical origin (Z=0); origin offset bridges raw→virtual
        std::vector<Path> path = evalData.pathList.at(i);
        std::vector<AircraftState> aircraftStateSteps;

        // Fixed initial orientation and position for deterministic evaluation
        // Virtual coordinates: start at origin (0,0,0). Path also at virtual origin.
        gp_quat aircraft_orientation = gp_quat(Eigen::AngleAxis<gp_scalar>(static_cast<gp_scalar>(M_PI), gp_vec3::UnitZ())) *
          gp_quat(Eigen::AngleAxis<gp_scalar>(0, gp_vec3::UnitY())) *
          gp_quat(Eigen::AngleAxis<gp_scalar>(0, gp_vec3::UnitX()));
        gp_vec3 initialPosition(0.0f, 0.0f, 0.0f);  // virtual origin
        gp_vec3 initial_velocity = aircraft_orientation * gp_vec3(SIM_INITIAL_VELOCITY, 0.0f, 0.0f);

        aircraftState = AircraftState{ 0, SIM_INITIAL_VELOCITY, initial_velocity, aircraft_orientation, initialPosition, 0.0f, 0.0f, SIM_INITIAL_THROTTLE, 0 };
        {
          gp_vec3 tangent;
          if (path.size() > 1)
            tangent = path[1].start - path[0].start;
          else
            tangent = gp_vec3::UnitX();
          double tn = tangent.norm();
          if (tn > 1e-6) tangent = tangent / tn;
          else tangent = gp_vec3::UnitX();
          aircraftState.resetHistory(path[0].start, tangent);
        }
        aircraftState.setRabbitOdometer(0.0f);

        // Get rabbit speed from scenario metadata
        gp_scalar rabbitSpeed = SIM_INITIAL_VELOCITY;  // default
        {
          ScenarioMetadata meta = scenarioForPathIndex(evalData, static_cast<size_t>(i));
          if (meta.rabbitSpeed > 0.0f) {
            rabbitSpeed = static_cast<gp_scalar>(meta.rabbitSpeed);
          }
        }
        aircraftState.setRabbitSpeed(rabbitSpeed);

        // Set initial rabbit position (odometer=0 = first path point)
        {
          VectorPathProvider pathProvider(path, 0);
          gp_vec3 rabbitPos = getInterpolatedTargetPosition(pathProvider, 0.0f, 0.0f);
          aircraftState.setRabbitPosition(rabbitPos);
        }
        aircraftStateSteps.push_back(aircraftState);

        unsigned long int duration_msec = 0;
        CrashReason crashReason = CrashReason::None;

        while (crashReason == CrashReason::None) {

          // Capture temporal history before NN evaluation
          {
            VectorPathProvider pathProvider(path, aircraftState.getThisPathIndex());
            gp_scalar rabbitOdo = aircraftState.getRabbitOdometer();
            gp_vec3 targetPos = getInterpolatedTargetPosition(
                pathProvider, rabbitOdo, 0.0f);
            gp_vec3 craftToTarget = targetPos - aircraftState.getPosition();
            gp_vec3 target_local = aircraftState.getOrientation().inverse() * craftToTarget;
            float distance = static_cast<float>(target_local.norm());

            // Path tangent for singularity fallback
            gp_vec3 posAhead = getInterpolatedTargetPosition(pathProvider, rabbitOdo, 0.5f);
            gp_vec3 tangent = posAhead - targetPos;
            double tn = tangent.norm();
            gp_vec3 tangent_body = (tn > 1e-6)
                ? aircraftState.getOrientation().inverse() * (tangent / tn)
                : gp_vec3::UnitX();

            gp_vec3 dir = computeTargetDir(target_local, distance, tangent_body);
            aircraftState.setRabbitPosition(targetPos);
            aircraftState.recordErrorHistory(dir, distance, duration_msec);
          }

          // Run NN controller
          {
            VectorPathProvider pathProvider(path, aircraftState.getThisPathIndex());
            NNControllerBackend nnBackend(nnGenome);
            nnBackend.evaluate(aircraftState, pathProvider);
          }

          // Advance aircraft state
          aircraftState.minisimAdvanceState(SIM_TIME_STEP_MSEC);
          duration_msec += SIM_TIME_STEP_MSEC;
          aircraftState.setSimTimeMsec(duration_msec);

          // Advance rabbit odometer
          gp_scalar dtSec = static_cast<gp_scalar>(SIM_TIME_STEP_MSEC) / 1000.0f;
          aircraftState.setRabbitOdometer(aircraftState.getRabbitOdometer() + rabbitSpeed * dtSec);

          // Crash detection: reconstruct raw position for OOB bounds check.
          // Position is virtual (Z≈0). Raw = virtual + (0,0,SIM_INITIAL_ALTITUDE).
          gp_vec3 rawForOOB = aircraftState.getPosition() + gp_vec3(0.0f, 0.0f, SIM_INITIAL_ALTITUDE);
          gp_scalar distanceFromOrigin = std::sqrt(rawForOOB[0] * rawForOOB[0] +
            rawForOOB[1] * rawForOOB[1]);
          if (rawForOOB[2] < (SIM_MAX_ELEVATION) ||
            rawForOOB[2] > (SIM_MIN_ELEVATION) ||
            distanceFromOrigin > SIM_PATH_RADIUS_LIMIT) {
            crashReason = CrashReason::Eval;
          }

          // Advance path index by scanning distanceFromStart against rabbit odometer
          while (aircraftState.getThisPathIndex() < static_cast<int>(path.size()) - 2 &&
                 path.at(aircraftState.getThisPathIndex()).distanceFromStart < aircraftState.getRabbitOdometer()) {
            aircraftState.setThisPathIndex(aircraftState.getThisPathIndex() + 1);
          }

          // Update rabbit position to match advanced odometer (for renderer error bars)
          {
            VectorPathProvider pathProvider(path, aircraftState.getThisPathIndex());
            gp_vec3 rabbitPos = getInterpolatedTargetPosition(
                pathProvider, aircraftState.getRabbitOdometer(), 0.0f);
            aircraftState.setRabbitPosition(rabbitPos);
          }

          aircraftStateSteps.push_back(aircraftState);

          if (duration_msec >= SIM_TOTAL_TIME_MSEC) {
            crashReason = CrashReason::TimeLimit;
          }
          if (aircraftState.getThisPathIndex() >= static_cast<int>(path.size()) - 2) {
            crashReason = CrashReason::RabbitComplete;
          }
        }

        {
          ScenarioMetadata meta = scenarioForPathIndex(evalData, static_cast<size_t>(i));
          meta.originOffset = gp_vec3(0.0f, 0.0f, SIM_INITIAL_ALTITUDE);  // raw→virtual offset for renderer
          evalResults.scenarioList.push_back(meta);
        }
        evalResults.pathList.push_back(path);
        evalResults.aircraftStateList.push_back(aircraftStateSteps);
        evalResults.crashReasonList.push_back(crashReason);
      }

      sendRPC(socket_, evalResults);

      evalResults.pathList.clear();
      evalResults.aircraftStateList.clear();
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
