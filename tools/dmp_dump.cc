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
#include "autoc/eval/derived_features.h"      // compute_pair_span, compute_tilt (032 NN inputs)
#include "autoc/eval/camera_projection.h"     // kCepSentinelThreshold (CEP gate)
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
    "  --run-summary  per-gen aggregate CSV over ALL gens in the run (S3 only):\n"
    "                 gen,best_fitness,mean_energy,mean_stability,mean_streak,\n"
    "                 crashes,aggr_pitch,aggr_roll,aggr_throttle,scenarios.\n"
    "                 Analytics off the dmps (not the log). Pair with --stride N.\n"
    "  --stride N     run-summary: sample every Nth gen (default 1)\n"
    "  --since-gen N  run-summary: skip gens < N (header suppressed) — incremental:\n"
    "                 fetch only new gens and append to a cached CSV\n"
    "  -i, --config   ini for S3 creds/bucket (default autoc.ini); fitness cone +\n"
    "                 cadence now read from the dmp-recorded RecordedRunConfig\n"
    "  -h, --help     this message\n"
    "\n"
    "CSV columns (pathgen): scenario,tick,px,py,pz,qw,qx,qy,qz,vx,vy,vz,\n"
    "  pitchCmd,rollCmd,thrCmd,out_pt,out_rl,out_th,dhome,dist,along,stpPt,mult,rampSc\n"
    "CSV columns (tracker): ...,out_th,dhome,rampSc,hull,\n"
    "  tgX,tgY,tgZ,trX,trY,trZ,spn0,dspn,blC0,brC0,tltS,tltC,stpPt  (path-relative\n"
    "  derived columns are pathgen-only; hull = inside_crash_hull; tg*=target\n"
    "  pos, tr*=trail-rabbit pos, spn0/dspn=beacon-pair span + 1-tick diff,\n"
    "  blC0/brC0=left/right beacon cep, tltS/tltC=target tilt sin/cos, stpPt=\n"
    "  in-cone step score vs trail-rabbit (recomputed, tracking metric); the\n"
    "  span/tilt sensors are CEP-gated with the default sentinel threshold).\n";
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

// Global per-axis aggressiveness (dctrl/mag) over an elite dmp's full trajectory
// set — the single-scalar-per-gen form of per_axis_aggressiveness.
struct AxisAggr {
  double dctrl[3], mag[3];                        // [0]=pitch [1]=roll [2]=throttle
  double ratio(int a) const { return mag[a] > 1e-9 ? dctrl[a] / mag[a] : 0.0; }
};
AxisAggr computeAggr(const EvalResults& r) {
  double sd[3] = {0, 0, 0}, sm[3] = {0, 0, 0};
  long cd[3] = {0, 0, 0}, cm[3] = {0, 0, 0};
  for (const auto& states : r.aircraftStateList) {
    for (size_t t = 0; t < states.size(); ++t) {
      const float* o = states[t].getNNOutputs();
      for (int a = 0; a < 3; ++a) { sm[a] += std::abs(o[a]); cm[a]++; }
      if (t > 0) {
        const float* p = states[t - 1].getNNOutputs();
        for (int a = 0; a < 3; ++a) { sd[a] += std::abs(o[a] - p[a]); cd[a]++; }
      }
    }
  }
  AxisAggr ag;
  for (int a = 0; a < 3; ++a) {
    ag.dctrl[a] = cd[a] ? sd[a] / cd[a] : 0.0;
    ag.mag[a]   = cm[a] ? sm[a] / cm[a] : 0.0;
  }
  return ag;
}

// quat → (roll_deg, pitch_deg), aerospace Tait-Bryan ZYX (matches 034 plot).
void quatToRollPitchDeg(const gp_quat& q, double& roll_deg, double& pitch_deg) {
  const double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
  const double sinr = 2.0 * (qw * qx + qy * qz);
  const double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
  double sinp = 2.0 * (qw * qy - qz * qx);
  sinp = std::max(-1.0, std::min(1.0, sinp));
  roll_deg = std::atan2(sinr, cosr) * 180.0 / M_PI;
  pitch_deg = std::asin(sinp) * 180.0 / M_PI;
}
double unwrapDeg(double d) {
  while (d > 180.0) d -= 360.0;
  while (d <= -180.0) d += 360.0;
  return d;
}

// Per-path (0..5) airframe rotation rate (deg/sec): per scenario, total
// |Δroll|/|Δpitch| over ticks ÷ duration; mean over that path's wind variants.
constexpr int kMaxPaths = 6;
struct PerPathRates { double roll[kMaxPaths] = {0}; double pitch[kMaxPaths] = {0}; };
PerPathRates computePerPathRates(const EvalResults& r) {
  const double dt = SIM_TIME_STEP_MSEC / 1000.0;
  double sr[kMaxPaths] = {0}, sp[kMaxPaths] = {0};
  int n[kMaxPaths] = {0};
  for (size_t i = 0; i < r.aircraftStateList.size(); ++i) {
    const auto& states = r.aircraftStateList[i];
    if (states.size() < 2) continue;
    const int path = (i < r.scenarioList.size()) ? r.scenarioList[i].pathVariantIndex : -1;
    if (path < 0 || path >= kMaxPaths) continue;
    double droll = 0, dpitch = 0, lastR = 0, lastP = 0; bool have = false;
    for (size_t t = 0; t < states.size(); ++t) {
      double rd, pd; quatToRollPitchDeg(states[t].getOrientation(), rd, pd);
      if (have) { droll += std::abs(unwrapDeg(rd - lastR)); dpitch += std::abs(pd - lastP); }
      lastR = rd; lastP = pd; have = true;
    }
    const double dur = states.size() * dt;
    sr[path] += droll / dur; sp[path] += dpitch / dur; n[path]++;
  }
  PerPathRates pr;
  for (int p = 0; p < kMaxPaths; ++p)
    if (n[p]) { pr.roll[p] = sr[p] / n[p]; pr.pitch[p] = sp[p] / n[p]; }
  return pr;
}

// Iterate every (stride-th) gen dmp in a run and emit a per-gen aggregate CSV —
// the "analytics off the log" path: best fitness + energy/stability/streak +
// crash count + per-axis aggressiveness, all recomputed from the elite dmps.
int doRunSummary(const Aws::S3::S3Client& s3, const std::string& bucket,
                 std::string runPrefix, int stride, int sinceGen) {
  if (!runPrefix.empty() && runPrefix.back() != '/') {
    const auto slash = runPrefix.rfind('/');           // a gen key → strip to run
    runPrefix = (slash == std::string::npos) ? "" : runPrefix.substr(0, slash + 1);
  }
  if (runPrefix.empty()) {
    runPrefix = autoc::findLatestRun(s3, bucket);
    std::cerr << "dmp-dump: latest run = " << runPrefix << std::endl;
  }
  const auto keys = autoc::listRunGenKeys(s3, bucket, runPrefix);
  if (stride < 1) stride = 1;
  std::cerr << "dmp-dump: run-summary over " << keys.size() << " gens (stride "
            << stride << ") in " << runPrefix << std::endl;
  // --since-gen N skips gens < N (incremental: fetch only new gens and merge
  // with a cached CSV). Header suppressed when set so output is append-ready.
  if (sinceGen <= 0) {
    std::cout << "gen,best_fitness,mean_energy,mean_stability,mean_streak,crashes,"
                 "aggr_pitch,aggr_roll,aggr_throttle,"
                 "dctrl_pitch,dctrl_roll,dctrl_throttle,"
                 "mag_pitch,mag_roll,mag_throttle,"
                 "path0_rollrate,path1_rollrate,path2_rollrate,path3_rollrate,"
                 "path4_rollrate,path5_rollrate,"
                 "path0_pitchrate,path1_pitchrate,path2_pitchrate,path3_pitchrate,"
                 "path4_pitchrate,path5_pitchrate,scenarios\n";
  }
  for (size_t i = 0; i < keys.size(); i += stride) {
    const std::string& k = keys[i];
    const int gen = autoc::extractGenNumber(k);
    if (sinceGen > 0 && gen < sinceGen) continue;   // already cached → skip S3 GET
    EvalResults r;
    try {
      const std::string b = autoc::s3GetDmpBlob(s3, bucket, k);
      std::istringstream iss(b, std::ios::binary);
      cereal::BinaryInputArchive ia(iss);
      ia(r);
    } catch (const std::exception& e) {
      std::cerr << "  gen " << gen << " skipped: " << e.what() << std::endl;
      continue;
    }
    const auto sc = computeScenarioScores(r);
    double me = 0, ms = 0, mk = 0; int cr = 0;
    for (const auto& s : sc) { me += s.energy_score; ms += s.stability_score;
                               mk += s.maxStreak; if (s.crashed) cr++; }
    const double N = sc.empty() ? 1.0 : static_cast<double>(sc.size());
    const AxisAggr ag = computeAggr(r);
    const PerPathRates pp = computePerPathRates(r);
    printf("%d,%.6f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,",
           gen, aggregateRawFitness(sc), me / N, ms / N, mk / N, cr,
           ag.ratio(0), ag.ratio(1), ag.ratio(2),
           ag.dctrl[0], ag.dctrl[1], ag.dctrl[2], ag.mag[0], ag.mag[1], ag.mag[2]);
    for (int p = 0; p < kMaxPaths; ++p) printf("%.3f,", pp.roll[p]);
    for (int p = 0; p < kMaxPaths; ++p) printf("%.3f,", pp.pitch[p]);
    printf("%zu\n", sc.size());
    fflush(stdout);
    if ((i / stride) % 10 == 0)
      std::cerr << "  ... gen " << gen << " (" << (i + 1) << "/" << keys.size() << ")\n";
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  static struct option long_options[] = {
    {"gen", required_argument, 0, 'g'},
    {"meta-only", no_argument, 0, 'm'},
    {"csv-only", no_argument, 0, 'c'},
    {"run-summary", no_argument, 0, 's'},
    {"stride", required_argument, 0, 'S'},
    {"since-gen", required_argument, 0, 'G'},
    {"config", required_argument, 0, 'i'},
    {"help", no_argument, 0, 'h'},
    {0, 0, 0, 0}
  };
  int specifiedGen = -1;
  bool metaOnly = false, csvOnly = false, runSummary = false;
  int stride = 1;
  int sinceGen = 0;
  std::string configFile = "autoc.ini";
  int idx = 0, opt;
  while ((opt = getopt_long(argc, argv, "g:mcsS:G:i:h", long_options, &idx)) != -1) {
    switch (opt) {
      case 'g': specifiedGen = std::stoi(optarg); break;
      case 'm': metaOnly = true; break;
      case 'c': csvOnly = true; break;
      case 's': runSummary = true; break;
      case 'S': stride = std::stoi(optarg); break;
      case 'G': sinceGen = std::stoi(optarg); break;
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
      ConfigManager::initialize(configFile, std::cerr);   // S3 creds + fitness params (chatter→stderr, keep stdout clean for CSV/YAML)
      Aws::InitAPI(awsOptions);
      awsInit = true;
      auto s3 = ConfigManager::getS3Client();
      if (!s3) throw std::runtime_error("no S3 client (check ini profile/creds)");
      if (runSummary) {
        const int rc = doRunSummary(*s3, bucket, key, stride, sinceGen);  // per-gen CSV → stdout
        Aws::ShutdownAPI(awsOptions);
        return rc;
      }
      if (key.empty() || key.back() == '/') {
        // Resolve run + gen. Empty key = "no run given" → pick the LATEST run
        // first (else findLatestGenKey would scan the whole bucket and return
        // the max gen across ALL runs, not the newest run's gen).
        std::string runPrefix = key;
        if (runPrefix.empty()) {
          runPrefix = autoc::findLatestRun(*s3, bucket);  // FR-P07
          std::cerr << "dmp-dump: latest run = " << runPrefix << std::endl;
        }
        if (specifiedGen >= 0) {
          key = runPrefix + "gen" + std::to_string(10000 - specifiedGen) + ".dmp.zst";
        } else {
          key = autoc::findLatestGenKey(*s3, bucket, runPrefix);
        }
      }
      blob = autoc::s3GetDmpBlob(*s3, bucket, key);
      srcBucket = bucket; srcKey = key;
    } else {
      // local file — still need fitness params for derived columns.
      ConfigManager::initialize(configFile, std::cerr);
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
      // Full (un-ramped) per-scenario wind-direction offset draw, degrees.
      // Applied wind = base_dir + offset × variationScale(gen) (run-level ramp_scale).
      std::cout << "    wind_dir_offset_deg: "
                << (results.scenarioList[i].windDirectionOffset * 180.0 / M_PI) << "\n";
      std::cout << "    crash_reason: " << crashReasonToString(s.crashReason) << "\n";
      std::cout << "    score: " << s.score << "\n";
      std::cout << "    energy_score: " << s.energy_score << "\n";
      std::cout << "    stability_score: " << s.stability_score << "\n";
      std::cout << "    max_streak: " << s.maxStreak << "\n";
      // 037 T005 — streak_steps + max_multiplier complete the per-scenario
      // reconstructability set: with these, the dmp carries everything the
      // training log's per-scenario [N] OK/CRASH lines carried, so those
      // lines can be dropped from the log.
      std::cout << "    streak_steps: " << s.totalStreakSteps << "\n";
      std::cout << "    max_multiplier: " << s.maxMultiplier << "\n";
      std::cout << "    steps: " << s.steps_completed << "/" << s.steps_total << "\n";
    }
  }

  if (metaOnly) { if (awsInit) Aws::ShutdownAPI(awsOptions); return 0; }

  // --- CSV per-tick ---------------------------------------------------------
  if (!csvOnly) std::cout << "\n---\n";

  // Header (mode-specific: path-relative derived columns are pathgen-only).
  std::cout << "scenario,tick,px,py,pz,qw,qx,qy,qz,vx,vy,vz,"
               "pitchCmd,rollCmd,thrCmd,out_pt,out_rl,out_th,dhome,wN,wE,wD";
  // 038 — honest recording read straight from the recorded per-tick
  // TrackerInputs/NNInputs + tracker aux outputs: tracker adds inX/inY/inZ
  // (arena-inward body dir) + tSee (time-since-seen) + the 038 US3 aux
  // span-predictor OUTPUTS spP1/spP2/spP3 (predicted span @+50/+100/+150 ms) +
  // spdR (predicted closure rate); pathgen adds dBnd (dist-to-boundary tanh) +
  // inX/inY/inZ. (exit_dir removed 038 US3.)
  if (isTracker) std::cout << ",rampSc,hull,tgX,tgY,tgZ,trX,trY,trZ,spn0,dspn,blC0,brC0,tltS,tltC,stpPt,inX,inY,inZ,tSee,spP1,spP2,spP3,spdR\n";
  else           std::cout << ",dist,along,stpPt,mult,rampSc,dBnd,inX,inY,inZ\n";

  // 038 P0-B (T010): read the fitness cone / cadence from the dmp-recorded,
  // self-describing RecordedRunConfig — NOT the live .ini. M2 greenfield: no
  // fallback to ConfigManager (a pre-038 dmp with a zeroed runConfig is
  // reproduced by checking out the matching code, not by reading today's ini).
  // Only S3 bucket/profile still comes from ConfigManager.
  const RecordedRunConfig& rc = results.runConfig;
  const double dtSec = rc.simTimeStepMsec / 1000.0;
  int streakStepsToMax = static_cast<int>(rc.fitStreakRampSec / dtSec);
  if (streakStepsToMax < 1) streakStepsToMax = 1;

  for (size_t si = 0; si < results.aircraftStateList.size(); ++si) {
    const auto& states = results.aircraftStateList[si];
    if (states.empty()) continue;
    const bool sceneTracker = si < results.targetTrajectoryList.size()
                              && !results.targetTrajectoryList[si].empty();
    const std::vector<Path>* path =
        (si < results.pathList.size()) ? &results.pathList[si] : nullptr;

    FitnessComputer fc(rc.fitDistScaleBehind, rc.fitDistScaleAhead, rc.fitConeAngleDeg,
                       rc.fitStreakThreshold, streakStepsToMax, rc.fitStreakMultiplierMax);
    fc.resetStreak();
    gp_vec3 prevTangent = gp_vec3::UnitX();
    double prevSpan = 0.0;  // tracker spn0 at ti-1, for dspn (0 at first emitted tick)
    bool havePrevSpan = false;

    for (size_t ti = 1; ti < states.size(); ++ti) {
      const auto& st = states[ti];
      const gp_vec3 pos = st.getPosition();
      const gp_quat q = st.getOrientation();
      const gp_vec3 vel = st.getVelocity();
      const gp_vec3 wind = st.getWindVelocity();  // NED wind the craft flew through
      const float* out = st.getNNOutputs();
      const gp_scalar dhome = pos.norm();  // home = origin

      char buf[512];
      int n = snprintf(buf, sizeof(buf),
        "%zu,%zu,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f",
        si, ti, pos.x(), pos.y(), pos.z(),
        q.w(), q.x(), q.y(), q.z(),
        vel.x(), vel.y(), vel.z(),
        st.getPitchCommand(), st.getRollCommand(), st.getThrottleCommand(),
        out[0], out[1], out[2], dhome, wind.x(), wind.y(), wind.z());
      std::cout.write(buf, n);

      if (sceneTracker) {
        const auto& targets = results.targetTrajectoryList[si];
        const size_t tgi = std::min(ti, targets.size() - 1);
        const int hull = (!targets.empty() && targets.at(tgi).inside_crash_hull) ? 1 : 0;

        // Target + trail-rabbit pose (0s if the trajectory list is missing/empty).
        gp_vec3 tg = gp_vec3::Zero(), tr = gp_vec3::Zero();
        double stp = 0.0;  // in-cone step score (tracking metric)
        if (!targets.empty()) {
          tg = targets.at(tgi).position;
          tr = targets.at(tgi).trail_rabbit_position;
          // Per-tick in-cone step score, recomputed from recorded target
          // geometry EXACTLY as fitness_decomposition.cc's tracker branch:
          // rabbit = trail-rabbit, tangent = target-velocity unit (prevTangent
          // fallback when degenerate). This is the tracker counterpart to the
          // pathgen `stpPt` column — derived, not recorded, like pathgen — so
          // dynamics_progress et al. get a real per-tick tracking flag on M2.
          const gp_vec3 tvel = targets.at(tgi).velocity;
          const double vn = tvel.norm();
          gp_vec3 tangent;
          if (vn > 0.01) { tangent = tvel / vn; prevTangent = tangent; }
          else           { tangent = prevTangent; }
          const gp_vec3 offset = pos - tr;
          const double along = offset.dot(tangent);
          const double lateralDist = (offset - along * tangent).norm();
          stp = fc.decomposeStepScore(along, lateralDist).score;
        }

        // Beacon-derived sensors (spn0/dspn/tltS/tltC) — CEP-gated to match the
        // 032 NN-input semantics. The gate uses the default sentinel threshold
        // (cepGateThreshold isn't carried in EvalResults).
        double spn0 = 0.0, tltS = 0.0, tltC = 1.0;
        double blC0 = 0.0, brC0 = 0.0;
        const bool haveCam = si < results.cameraViewList.size()
                             && !results.cameraViewList[si].empty();
        if (haveCam) {
          const auto& cams = results.cameraViewList[si];
          const auto& cv = cams.at(std::min(ti, cams.size() - 1));
          const auto& bl = cv.beacon_left;
          const auto& br = cv.beacon_right;
          blC0 = bl.cep;
          brC0 = br.cep;
          const bool gated = (bl.cep >= autoc::eval::kCepSentinelThreshold)
                          || (br.cep >= autoc::eval::kCepSentinelThreshold);
          if (!gated) {
            spn0 = autoc::eval::compute_pair_span(bl.screen_x, bl.screen_y,
                                                  br.screen_x, br.screen_y);
            const auto t = autoc::eval::compute_tilt(bl.screen_x, bl.screen_y,
                                                     br.screen_x, br.screen_y);
            tltS = t.sin;
            tltC = t.cos;
          }
        }
        const double dspn = havePrevSpan ? (spn0 - prevSpan) : 0.0;
        prevSpan = spn0;
        havePrevSpan = true;

        // 038 — arena-inward + time_since_seen (from recorded TrackerInputs) +
        // the US3 aux span-predictor outputs out[3..6] (honest recording).
        const TrackerInputs& ti_in = st.getTrackerInputs();
        char tb[512];
        int tn = snprintf(tb, sizeof(tb),
          ",%.4f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f"
          ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
          rampSc, hull,
          tg.x(), tg.y(), tg.z(), tr.x(), tr.y(), tr.z(),
          spn0, dspn, blC0, brC0, tltS, tltC, stp,
          ti_in.inward_body_x, ti_in.inward_body_y, ti_in.inward_body_z,
          ti_in.time_since_seen, out[3], out[4], out[5], out[6]);
        std::cout.write(tb, tn);
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
        // 038 P0-D FR-P0H (B) — arena-awareness inputs from recorded NNInputs.
        const NNInputs& nn_in = st.getNNInputs();
        char d[256];
        int dn = snprintf(d, sizeof(d), ",%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f\n",
                          dist, along, stp, mult, rampSc,
                          nn_in.dist_to_boundary, nn_in.inward_body_x,
                          nn_in.inward_body_y, nn_in.inward_body_z);
        std::cout.write(d, dn);
      } else {
        // 038 P0-D FR-P0H (B) — no path geometry, but the recorded NNInputs
        // still carry the arena-awareness (B) inputs.
        const NNInputs& nn_in = st.getNNInputs();
        char d[128];
        int dn = snprintf(d, sizeof(d), ",,,,,%.4f,%.6f,%.6f,%.6f,%.6f\n",
                          rampSc, nn_in.dist_to_boundary, nn_in.inward_body_x,
                          nn_in.inward_body_y, nn_in.inward_body_z);
        std::cout.write(d, dn);
      }
    }
  }

  if (awsInit) Aws::ShutdownAPI(awsOptions);
  return 0;
}
