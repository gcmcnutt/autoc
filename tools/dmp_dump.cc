// 035 FR-P02/P06 — dmp-dump: read a cereal EvalResults dmp (zstd or legacy
// plain; S3 or local) and emit a YAML metadata block + a per-tick CSV block.
// Replaces the retired per-step data.dat (FR-P05): every column is reconstructed
// from the serialized dmp, with derived columns recomputed via shared
// autoc_common math (FitnessComputer) so there is no drift.
//
// Usage:
//   dmp-dump <s3-uri | local-path> [--gen N] [--meta-only] [--csv-only] [-i ini]
//     s3://<bucket>/<run-id>/gen<N>.dmp.zst   (bucket from URI, NOT the ini)
//     s3://<bucket>/<run-id>/                  (run prefix; --gen N or latest)
//     /path/to/gen<N>.dmp[.zst]               (local dev convenience)
//
// Output: YAML metadata, then a "\n---\n" separator, then CSV.
// Fail-loud (Principle V/VII): a load failure exits non-zero with a clear
// stderr message and never emits partial-but-plausible output.

#include <getopt.h>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <aws/core/Aws.h>
#include <aws/s3/S3Client.h>

#include <cereal/archives/binary.hpp>

#include "autoc/rpc/protocol.h"               // EvalResults
#include "autoc/rpc/crash_reason.h"           // crashReasonToString
#include "autoc/util/config.h"
#include "autoc/util/s3_run_selector.h"       // s3GetDmpBlob, findLatestGenKey, extractGenNumber
#include "autoc/util/zstd_io.h"               // local-file inflate
#include "autoc/util/scenario_prng.h"         // deriveClassSubSeeds
#include "autoc/eval/fitness_decomposition.h" // computeScenarioScores, ScenarioScore
#include "autoc/eval/fitness_computer.h"      // FitnessComputer (derived columns)
#include "autoc/eval/aircraft_state.h"
#include "autoc/nn/serialization.h"           // nn_detect_format / nn_deserialize
#include "autoc/nn/evaluator.h"               // NNGenome (variation_scale)

namespace {

void printUsage(const char* prog) {
  std::cout <<
    "Usage: " << prog << " <s3-uri | local-path> [OPTIONS]\n"
    "  <s3-uri>     s3://<bucket>/<run-id>/gen<N>.dmp[.zst]  (bucket from URI)\n"
    "               s3://<bucket>/<run-id>/   (run prefix: --gen N, else latest)\n"
    "  <local-path> a .dmp[.zst] file\n"
    "Options:\n"
    "  --gen N        select gen N (actualGen) when given an S3 run prefix\n"
    "  --meta-only    emit only the YAML metadata block\n"
    "  --csv-only     emit only the CSV block\n"
    "  -i, --config   ini for S3 creds + fitness params (default autoc.ini)\n"
    "  -h, --help     this message\n"
    "\n"
    "CSV columns (pathgen): scenario,tick,px,py,pz,qw,qx,qy,qz,vx,vy,vz,\n"
    "  pitchCmd,rollCmd,thrCmd,out_pt,out_rl,out_th,dhome,dist,along,stpPt,mult,rampSc\n"
    "CSV columns (tracker): ...,out_th,dhome,rampSc,hull  (path-relative\n"
    "  derived columns are pathgen-only; hull = inside_crash_hull).\n";
}

// Parse "s3://bucket/key..." into (bucket, key). Returns false if not an s3 uri.
bool parseS3Uri(const std::string& uri, std::string& bucket, std::string& key) {
  const std::string scheme = "s3://";
  if (uri.rfind(scheme, 0) != 0) return false;
  const std::string rest = uri.substr(scheme.size());
  const auto slash = rest.find('/');
  if (slash == std::string::npos) { bucket = rest; key = ""; return true; }
  bucket = rest.substr(0, slash);
  key = rest.substr(slash + 1);
  return true;
}

std::string readLocalFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open local dmp: " + path);
  std::ostringstream oss;
  oss << f.rdbuf();
  std::string body = oss.str();
  return autoc::isZstdKey(path) ? autoc::zstdDecompress(body) : body;
}

// rampSc: the variation_scale baked into the elite genome at save time.
float genomeVariationScale(const EvalResults& r) {
  if (r.gp.empty()) return 1.0f;
  const auto* bytes = reinterpret_cast<const uint8_t*>(r.gp.data());
  if (!nn_detect_format(bytes, r.gp.size())) return 1.0f;
  NNGenome g;
  if (!nn_deserialize(bytes, r.gp.size(), g)) return 1.0f;
  return g.variation_scale;
}

}  // namespace

int main(int argc, char** argv) {
  static struct option long_options[] = {
    {"gen", required_argument, 0, 'g'},
    {"meta-only", no_argument, 0, 'm'},
    {"csv-only", no_argument, 0, 'c'},
    {"config", required_argument, 0, 'i'},
    {"help", no_argument, 0, 'h'},
    {0, 0, 0, 0}
  };
  int specifiedGen = -1;
  bool metaOnly = false, csvOnly = false;
  std::string configFile = "autoc.ini";
  int idx = 0, opt;
  while ((opt = getopt_long(argc, argv, "g:mci:h", long_options, &idx)) != -1) {
    switch (opt) {
      case 'g': specifiedGen = std::stoi(optarg); break;
      case 'm': metaOnly = true; break;
      case 'c': csvOnly = true; break;
      case 'i': configFile = optarg; break;
      case 'h': printUsage(argv[0]); return 0;
      default: printUsage(argv[0]); return 1;
    }
  }
  if (optind >= argc) { printUsage(argv[0]); return 1; }
  const std::string input = argv[optind];

  // --- load the cereal blob (S3 or local) -----------------------------------
  std::string blob;
  std::string srcBucket, srcKey;
  Aws::SDKOptions awsOptions;
  bool awsInit = false;
  try {
    std::string bucket, key;
    if (parseS3Uri(input, bucket, key)) {
      ConfigManager::initialize(configFile);   // S3 creds + fitness params
      Aws::InitAPI(awsOptions);
      awsInit = true;
      auto s3 = ConfigManager::getS3Client();
      if (!s3) throw std::runtime_error("no S3 client (check ini profile/creds)");
      if (key.empty() || key.back() == '/') {
        // run prefix: pick the requested gen, else the latest.
        if (specifiedGen >= 0) {
          if (!key.empty() && key.back() != '/') key += '/';
          key += "gen" + std::to_string(10000 - specifiedGen) + ".dmp.zst";
        } else {
          key = autoc::findLatestGenKey(*s3, bucket, key);
        }
      }
      blob = autoc::s3GetDmpBlob(*s3, bucket, key);
      srcBucket = bucket; srcKey = key;
    } else {
      // local file — still need fitness params for derived columns.
      ConfigManager::initialize(configFile);
      blob = readLocalFile(input);
      srcBucket = "(local)"; srcKey = input;
    }
  } catch (const std::exception& e) {
    std::cerr << "dmp-dump: load failed: " << e.what() << std::endl;
    if (awsInit) Aws::ShutdownAPI(awsOptions);
    return 1;
  }

  // --- deserialize ----------------------------------------------------------
  EvalResults results;
  try {
    std::istringstream iss(blob, std::ios::binary);
    cereal::BinaryInputArchive ia(iss);
    ia(results);
  } catch (const std::exception& e) {
    std::cerr << "dmp-dump: cereal deserialize failed for " << srcKey
              << ": " << e.what() << std::endl;
    if (awsInit) Aws::ShutdownAPI(awsOptions);
    return 1;
  }

  // tracker iff any scenario carries a target trajectory (same modality test
  // computeScenarioScores + the old data.dat writer used).
  bool isTracker = false;
  for (const auto& t : results.targetTrajectoryList) if (!t.empty()) { isTracker = true; break; }

  const int actualGen = autoc::extractGenNumber(srcKey);  // -1 if not a gen key
  const auto scores = computeScenarioScores(results);
  const float rampSc = genomeVariationScale(results);

  // --- YAML metadata --------------------------------------------------------
  if (!csvOnly) {
    std::cout << "run:\n";
    std::cout << "  bucket: " << srcBucket << "\n";
    std::cout << "  key: " << srcKey << "\n";
    std::cout << "  gen: " << actualGen << "\n";
    std::cout << "  mode: " << (isTracker ? "tracker" : "pathgen") << "\n";
    std::cout << "  gp_hash: " << std::hex << results.gpHash << std::dec << "\n";
    std::cout << "  scenario_count: " << scores.size() << "\n";
    std::cout << "  ramp_scale: " << rampSc << "\n";
    std::cout << "scenarios:\n";
    for (size_t i = 0; i < scores.size(); ++i) {
      const auto& s = scores[i];
      uint64_t seed = 0; int pvi = -1, wvi = -1;
      if (i < results.scenarioList.size()) {
        seed = results.scenarioList[i].scenarioSeed;
        pvi = results.scenarioList[i].pathVariantIndex;
        wvi = results.scenarioList[i].windVariantIndex;
      }
      const auto sub = autoc::util::deriveClassSubSeeds(seed);
      std::cout << "  - idx: " << i << "\n";
      std::cout << "    scenario_seed: " << seed << "\n";
      std::cout << "    path_variant: " << pvi << "\n";
      std::cout << "    wind_variant: " << wvi << "\n";
      std::cout << "    seeds: {wind: " << sub.wind << ", rabbit: " << sub.rabbit
                << ", entry: " << sub.entry << ", craft: " << sub.craft
                << ", camera: " << sub.camera << "}\n";
      std::cout << "    crash_reason: " << crashReasonToString(s.crashReason) << "\n";
      std::cout << "    score: " << s.score << "\n";
      std::cout << "    energy_score: " << s.energy_score << "\n";
      std::cout << "    stability_score: " << s.stability_score << "\n";
      std::cout << "    max_streak: " << s.maxStreak << "\n";
      std::cout << "    steps: " << s.steps_completed << "/" << s.steps_total << "\n";
    }
  }

  if (metaOnly) { if (awsInit) Aws::ShutdownAPI(awsOptions); return 0; }

  // --- CSV per-tick ---------------------------------------------------------
  if (!csvOnly) std::cout << "\n---\n";

  // Header (mode-specific: path-relative derived columns are pathgen-only).
  std::cout << "scenario,tick,px,py,pz,qw,qx,qy,qz,vx,vy,vz,"
               "pitchCmd,rollCmd,thrCmd,out_pt,out_rl,out_th,dhome";
  if (isTracker) std::cout << ",rampSc,hull\n";
  else           std::cout << ",dist,along,stpPt,mult,rampSc\n";

  const AutocConfig& cfg = ConfigManager::getConfig();
  int streakStepsToMax = static_cast<int>(cfg.fitStreakRampSec / (SIM_TIME_STEP_MSEC / 1000.0));
  if (streakStepsToMax < 1) streakStepsToMax = 1;

  for (size_t si = 0; si < results.aircraftStateList.size(); ++si) {
    const auto& states = results.aircraftStateList[si];
    if (states.empty()) continue;
    const bool sceneTracker = si < results.targetTrajectoryList.size()
                              && !results.targetTrajectoryList[si].empty();
    const std::vector<Path>* path =
        (si < results.pathList.size()) ? &results.pathList[si] : nullptr;

    FitnessComputer fc(cfg.fitDistScaleBehind, cfg.fitDistScaleAhead, cfg.fitConeAngleDeg,
                       cfg.fitStreakThreshold, streakStepsToMax, cfg.fitStreakMultiplierMax);
    fc.resetStreak();
    gp_vec3 prevTangent = gp_vec3::UnitX();

    for (size_t ti = 1; ti < states.size(); ++ti) {
      const auto& st = states[ti];
      const gp_vec3 pos = st.getPosition();
      const gp_quat q = st.getOrientation();
      const gp_vec3 vel = st.getVelocity();
      const float* out = st.getNNOutputs();
      const gp_scalar dhome = pos.norm();  // home = origin

      char buf[512];
      int n = snprintf(buf, sizeof(buf),
        "%zu,%zu,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.4f",
        si, ti, pos.x(), pos.y(), pos.z(),
        q.w(), q.x(), q.y(), q.z(),
        vel.x(), vel.y(), vel.z(),
        st.getPitchCommand(), st.getRollCommand(), st.getThrottleCommand(),
        out[0], out[1], out[2], dhome);
      std::cout.write(buf, n);

      if (sceneTracker) {
        const auto& targets = results.targetTrajectoryList[si];
        const int hull = (!targets.empty() && targets.at(std::min(ti, targets.size() - 1)).inside_crash_hull) ? 1 : 0;
        std::cout << "," << rampSc << "," << hull << "\n";
      } else if (path && !path->empty()) {
        const int pIdx = std::clamp(st.getThisPathIndex(), 0,
                                    static_cast<int>(path->size()) - 1);
        gp_vec3 tangent;
        if (pIdx + 1 < static_cast<int>(path->size())) {
          tangent = path->at(pIdx + 1).start - path->at(pIdx).start;
          double tn = tangent.norm();
          if (tn > 0.01) { tangent /= tn; prevTangent = tangent; } else tangent = prevTangent;
        } else tangent = prevTangent;
        const gp_vec3 offset = pos - path->at(pIdx).start;
        const double along = offset.dot(tangent);
        const double lateral = (offset - along * tangent).norm();
        const double dist = offset.norm();
        const double stp = fc.computeStepScore(along, lateral);
        const double mult = (stp > 0.0) ? fc.applyStreak(stp) / stp : 1.0;
        char d[160];
        int dn = snprintf(d, sizeof(d), ",%.4f,%.4f,%.4f,%.4f,%.4f\n",
                          dist, along, stp, mult, rampSc);
        std::cout.write(d, dn);
      } else {
        std::cout << ",,,,," << rampSc << "\n";
      }
    }
  }

  if (awsInit) Aws::ShutdownAPI(awsOptions);
  return 0;
}
