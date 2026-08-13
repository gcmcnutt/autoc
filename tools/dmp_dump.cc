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
#include "autoc/eval/specific_force.h"        // 041 T010 — shared body specific force
#include "autoc/eval/aircraft_state.h"
#include "autoc/nn/serialization.h"           // nn_detect_format / nn_deserialize
#include "autoc/nn/evaluator.h"               // NNGenome (variation_scale)

namespace {

// ---------------------------------------------------------------------------
// 040 T045 (FR-007b, SC-007) — obstruction-onset distribution.
//
// Reports where the chase's own airframe first takes signal, across the
// MOUNTING-ERROR ENVELOPE rather than at nominal only. That is the whole point:
// design intent and build reality have to be checked together, because a camera
// glued 20° off brings the propeller shadow back toward the boresight where a
// chased target actually lives.
//
// This reads only the ini — obstruction onset is a property of the
// configuration, not of any particular run — so it needs no dmp argument.
//
// THE ENVELOPE IS ASSUMED, and stated here rather than buried: per spec
// Session 2026-07-28, most of the mounting-error distribution sits within 10°
// with a hard limit at 20°. Modelled as a half-normal in tilt magnitude with
// σ = 6° (⇒ ~95th percentile at 11.8°), hard-clipped at 20°, swept
// deterministically rather than sampled so the report is reproducible. There is
// no measured glue-error distribution; when one exists it replaces σ only.
// Per the "let it ride" clarification this is an OBSERVATION, not a gate.
// ---------------------------------------------------------------------------
int emitObstructionReport(const AutocConfig& cfg,
                          const autoc::eval::AirframeObstruction& airframe,
                          const gp_vec3& mount) {
  autoc::eval::CameraConfig cam;
  cam.pixels_h = cfg.cameraPixelsH;
  cam.pixels_v = cfg.cameraPixelsV;
  cam.deg_per_px = static_cast<gp_scalar>(cfg.cameraDegPerPixel);
  cam.mount_offset_body = mount;

  std::cout << "obstruction_report:\n";
  std::cout << "  enabled: " << (airframe.enabled ? "true" : "false") << "\n";
  std::cout << "  mount_body_m: [" << mount.x() << ", " << mount.y() << ", "
            << mount.z() << "]\n";
  std::cout << "  nominal_fov_deg: [" << cam.fovHDeg() << ", " << cam.fovVDeg()
            << "]\n";

  if (!airframe.enabled) {
    std::cout << "  note: obstruction disabled — nothing to report\n";
    return 0;
  }

  // --- effective field at nominal alignment (T044, FR-012) ------------------
  const autoc::eval::EffectiveField f =
      autoc::eval::computeEffectiveField(cam, airframe, mount);
  std::cout << "  effective_field:\n";
  std::cout << "    nominal_sr: " << f.nominal_sr << "\n";
  std::cout << "    clear_frac: " << f.clearFraction() << "\n";
  std::cout << "    blocked_frac: " << f.blockedFraction()
            << "    # opaque (wing/nose) — signal lost\n";
  std::cout << "    attenuated_frac: " << f.attenuatedFraction()
            << "    # prop disc — reduced, NOT lost (FR-009)\n";

  // --- onset across the mounting-error envelope (T045, FR-007b) ------------
  // Deterministic sweep of tilt magnitude, weighted by the half-normal above.
  // Weights are accumulated so median / 95th read off the same curve.
  constexpr double kSigmaDeg = 6.0;   // raw-ok: report-local statistics, not eval math
  constexpr double kClipDeg = 20.0;   // raw-ok: report-local statistics, not eval math
  constexpr double kStepDeg = 0.25;   // raw-ok: report-local statistics, not eval math

  std::vector<std::pair<double, double>> onset_weight;  // raw-ok: report-local statistics. (onset_deg, weight)
  double total_w = 0.0;  // raw-ok: report-local statistics
  for (double tilt = 0.0; tilt <= kClipDeg + 1e-9; tilt += kStepDeg) {  // raw-ok: report-local statistics
    // Half-normal density (unnormalised; normalised via total_w below).
    const double w = std::exp(-0.5 * (tilt / kSigmaDeg) * (tilt / kSigmaDeg));  // raw-ok: report-local statistics
    const double onset = static_cast<double>(autoc::eval::obstructionOnsetDeg(  // raw-ok: report-local statistics
        cam, airframe, mount, static_cast<gp_scalar>(tilt)));
    onset_weight.emplace_back(onset, w);
    total_w += w;
  }
  // Onset DECREASES as tilt grows, so sort ascending to read percentiles from
  // the pessimistic tail inward.
  std::sort(onset_weight.begin(), onset_weight.end());

  auto percentile = [&](double p) {  // raw-ok: report-local statistics
    // p measured from the PESSIMISTIC end (smallest onset = worst case), so
    // "95th percentile" means "95% of mounts do at least this well".
    double acc = 0.0;  // raw-ok: report-local statistics
    for (const auto& ow : onset_weight) {
      acc += ow.second;
      if (acc / total_w >= (1.0 - p)) return ow.first;
    }
    return onset_weight.back().first;
  };

  const double nominal_onset = static_cast<double>(  // raw-ok: report-local statistics
      autoc::eval::obstructionOnsetDeg(cam, airframe, mount, 0.0f));
  const double clipped_extreme = onset_weight.front().first;  // raw-ok: report-local statistics

  std::cout << "  onset_deg_from_own_boresight:\n";
  std::cout << "    nominal: " << nominal_onset << "\n";
  std::cout << "    median: " << percentile(0.50) << "\n";
  std::cout << "    p95: " << percentile(0.95)
            << "    # 95% of mounts obstruct no closer in than this\n";
  std::cout << "    clipped_extreme: " << clipped_extreme
            << "    # the " << kClipDeg << "° glue-error limit\n";
  std::cout << "    envelope: half-normal sigma=" << kSigmaDeg
            << "deg clipped at " << kClipDeg << "deg (ASSUMED; FR-035)\n";
  if (nominal_onset < 0.0) {
    std::cout << "    note: no obstruction anywhere in the field at nominal\n";
  }
  return 0;
}

// ---------------------------------------------------------------------------
// 041 T010 — per-tick physics columns from the recorded PhysicsTraceEntry.
//
// This is a READER. The data has been recorded for every elite reeval since
// long before 041 (`inputdev_autoc.cpp:1047`) and had no consumer anywhere, so
// these columns work on EXISTING dmps and unblock Study A with no schema
// change.
//
// ⚠️ THE INDEX TRAP. `physicsTrace[si]` and `aircraftStateList[si]` are the same
// scenario, but NOT the same tick series: physics is captured inside the FDM,
// once per integration substep (`fdm_larcsim.cpp:925`), while aircraft states
// are recorded once per CONTROL tick. So physicsTrace[si] is MUCH longer, and
// `physicsTrace[si][ti]` is a different moment in time from `states[ti]`.
// Joining them by index is exactly the parallel-index failure class US1 exists
// to retire — so this joins on RECORDED TIME instead, and reports how well it
// matched rather than assuming it did.
struct PhysicsJoin {
  const std::vector<PhysicsTraceEntry>* trace = nullptr;
  size_t cursor = 0;      // monotone: both series are time-ordered
  size_t matched = 0;
  size_t unmatched = 0;
  double worstSkewMsec = 0.0;   // raw-ok: report-local diagnostic
  double tolMsec = 0.0;         // raw-ok: report-local diagnostic

  // Nearest recorded physics row WITHIN tolerance, or nullptr.
  //
  // ⚠️ The tolerance is not defensive padding — it is load-bearing. The FDM caps
  // the trace at `MAX_TRACE_STEPS = 35` (`fdm_larcsim.cpp:74`), which is roughly
  // the first 200 ms of a scenario, so for all but the first few control ticks
  // there is NO physics row. Returning the nearest one anyway would repeat the
  // 200 ms sample for the rest of an 18-second flight and read as real data.
  const PhysicsTraceEntry* at(double tMsec) {
    if (trace == nullptr || trace->empty()) { ++unmatched; return nullptr; }
    while (cursor + 1 < trace->size()
           && (*trace)[cursor + 1].simTimeMsec <= tMsec) {
      ++cursor;
    }
    // `cursor` is the last row at or before tMsec; the next one may be nearer.
    size_t best = cursor;
    if (cursor + 1 < trace->size()) {
      const double dCur = std::fabs((*trace)[cursor].simTimeMsec - tMsec);
      const double dNxt = std::fabs((*trace)[cursor + 1].simTimeMsec - tMsec);
      if (dNxt < dCur) best = cursor + 1;
    }
    const double skew = std::fabs((*trace)[best].simTimeMsec - tMsec);
    if (skew > tolMsec) { ++unmatched; return nullptr; }
    if (skew > worstSkewMsec) worstSkewMsec = skew;
    ++matched;
    return &(*trace)[best];
  }
};

// Body-frame specific force in g, plus the load factor, from a physics row.
//
// spec_force_world = a_world - g_world, with NED gravity (0, 0, +g) — see
// docs/COORDINATE_CONVENTIONS.md. The recorded `gravity` is used rather than a
// literal 9.81 because PhysicsTraceEntry carries FDM-NATIVE units (LaRCSim is
// ft-based), and dividing by the recorded value makes the result unit-free
// whichever it is.
//
// SIGN: `nz_g` is the aviation load factor — +1 in steady level flight, >1 in a
// pull-up, negative in a push-over — i.e. the NEGATED body-z specific force, so
// that it reads the way "+11.2 g / -8.4 g" is quoted in the flight reports.
// The raw sfx/sfy/sfz columns are emitted unnegated beside it so the convention
// is inspectable rather than buried in this comment.
// The math itself lives in autoc/eval/specific_force.h so this reader and the
// NN input (T039) cannot drift apart; this only adapts PhysicsTraceEntry's
// storage layout (quat is [x, y, z, w]; gp_quat is (w, x, y, z)).
autoc::eval::BodySpecificForce deriveLoad(const PhysicsTraceEntry& p) {
  const gp_quat q(static_cast<gp_scalar>(p.quat[3]), static_cast<gp_scalar>(p.quat[0]),
                  static_cast<gp_scalar>(p.quat[1]), static_cast<gp_scalar>(p.quat[2]));
  const gp_vec3 accWorld(static_cast<gp_scalar>(p.acc[0]),
                         static_cast<gp_scalar>(p.acc[1]),
                         static_cast<gp_scalar>(p.acc[2]));
  return autoc::eval::bodySpecificForce(accWorld, q, static_cast<gp_scalar>(p.gravity));
}

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
    "  -O, --obstruction-report\n"
    "                 040 T045: airframe obstruction-onset distribution across the\n"
    "                 mounting-error envelope, plus the effective field of view.\n"
    "                 Reads the ini ONLY — takes no dmp argument. Pair with -i.\n"
    "  -i, --config   ini for S3 creds/bucket (default autoc.ini); fitness cone +\n"
    "                 cadence now read from the dmp-recorded RecordedRunConfig\n"
    "  --physics      041 T010: append per-tick FDM physics columns from the\n"
    "                 recorded PhysicsTraceEntry (elite-reeval dmps only):\n"
    "                 accX,accY,accZ,odbP,odbQ,odbR,alpha,vRelWind + the DERIVED\n"
    "                 body-frame specific force sfx_g,sfy_g,sfz_g and load factor\n"
    "                 nz_g. Works on EXISTING dmps — this is a reader, not a\n"
    "                 recording change. Empty fields where no physics row matches.\n"
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
  // 041 T023 — iterates the STEPPED ticks only. Previously this walked
  // `aircraftStateList[i]`, whose slot 0 is the pre-loop initial state: it
  // carries no NN outputs, so every aggressiveness mean was diluted by one zero
  // sample per scenario (~0.2% at N=450). Excluding it is both correct and now
  // structural — the initial state is not in `ticks` to be walked by accident.
  for (const auto& scenarioTicks : r.tickList) {
    const auto& ticks = scenarioTicks.ticks;
    for (size_t t = 0; t < ticks.size(); ++t) {
      const float* o = ticks[t].state.getNNOutputs();
      for (int a = 0; a < 3; ++a) { sm[a] += std::abs(o[a]); cm[a]++; }
      if (t > 0) {
        const float* p = ticks[t - 1].state.getNNOutputs();
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
  for (size_t i = 0; i < r.tickList.size(); ++i) {
    const auto& states = r.tickList[i].ticks;
    if (states.size() < 2) continue;
    const int path = (i < r.scenarioList.size()) ? r.scenarioList[i].pathVariantIndex : -1;
    if (path < 0 || path >= kMaxPaths) continue;
    double droll = 0, dpitch = 0, lastR = 0, lastP = 0; bool have = false;
    for (size_t t = 0; t < states.size(); ++t) {
      double rd, pd; quatToRollPitchDeg(states[t].state.getOrientation(), rd, pd);
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
    {"obstruction-report", no_argument, 0, 'O'},
    {"physics", no_argument, 0, 'P'},          // 041 T010
    {"help", no_argument, 0, 'h'},
    {0, 0, 0, 0}
  };
  int specifiedGen = -1;
  bool metaOnly = false, csvOnly = false, runSummary = false;
  bool obstructionReport = false;
  bool physics = false;                        // 041 T010
  int stride = 1;
  int sinceGen = 0;
  std::string configFile = "autoc.ini";
  int idx = 0, opt;
  while ((opt = getopt_long(argc, argv, "g:mcsS:G:i:OPh", long_options, &idx)) != -1) {
    switch (opt) {
      case 'g': specifiedGen = std::stoi(optarg); break;
      case 'm': metaOnly = true; break;
      case 'c': csvOnly = true; break;
      case 's': runSummary = true; break;
      case 'S': stride = std::stoi(optarg); break;
      case 'G': sinceGen = std::stoi(optarg); break;
      case 'i': configFile = optarg; break;
      case 'O': obstructionReport = true; break;
      case 'P': physics = true; break;         // 041 T010
      case 'h': printUsage(argv[0]); return 0;
      default: printUsage(argv[0]); return 1;
    }
  }

  // 040 T045 — the obstruction report is derived from the INI alone (onset is a
  // property of the configuration, not of a run), so it is handled before the
  // dmp-argument check and takes no positional argument.
  if (obstructionReport) {
    // Config chatter to stderr so stdout stays a clean YAML block, matching the
    // other paths through this tool.
    ConfigManager::initialize(configFile, std::cerr);
    const AutocConfig& cfg = ConfigManager::getConfig();
    autoc::eval::AirframeObstruction a;
    a.enabled = (cfg.airframeObstructionEnabled != 0);
    a.wing_min = gp_vec3(static_cast<gp_scalar>(cfg.airframeWingMinX),
                         static_cast<gp_scalar>(cfg.airframeWingMinY),
                         static_cast<gp_scalar>(cfg.airframeWingMinZ));
    a.wing_max = gp_vec3(static_cast<gp_scalar>(cfg.airframeWingMaxX),
                         static_cast<gp_scalar>(cfg.airframeWingMaxY),
                         static_cast<gp_scalar>(cfg.airframeWingMaxZ));
    a.nose_min = gp_vec3(static_cast<gp_scalar>(cfg.airframeNoseMinX),
                         static_cast<gp_scalar>(cfg.airframeNoseMinY),
                         static_cast<gp_scalar>(cfg.airframeNoseMinZ));
    a.nose_max = gp_vec3(static_cast<gp_scalar>(cfg.airframeNoseMaxX),
                         static_cast<gp_scalar>(cfg.airframeNoseMaxY),
                         static_cast<gp_scalar>(cfg.airframeNoseMaxZ));
    a.prop_plane_x = static_cast<gp_scalar>(cfg.airframePropPlaneX);
    a.prop_axis_y = static_cast<gp_scalar>(cfg.airframePropAxisY);
    a.prop_axis_z = static_cast<gp_scalar>(cfg.airframePropAxisZ);
    a.prop_radius = static_cast<gp_scalar>(cfg.airframePropRadius);
    a.prop_attenuation = static_cast<gp_scalar>(cfg.airframePropAttenuation);
    const gp_vec3 mount(static_cast<gp_scalar>(cfg.cameraMountOffsetX),
                        static_cast<gp_scalar>(cfg.cameraMountOffsetY),
                        static_cast<gp_scalar>(cfg.cameraMountOffsetZ));
    return emitObstructionReport(cfg, a, mount);
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
  for (const auto& st : results.tickList)
    if (!st.ticks.empty() && st.ticks.front().targetSample.has_value()) { isTracker = true; break; }

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
  if (isTracker) std::cout << ",rampSc,hull,tgX,tgY,tgZ,trX,trY,trZ,spn0,dspn,blC0,brC0,tltS,tltC,stpPt,inX,inY,inZ,tSee,spP1,spP2,spP3,spdR";
  else           std::cout << ",dist,along,stpPt,mult,rampSc,dBnd,inX,inY,inZ";
  // 041 T037-T039 — the five A1 slots, read from the RECORDED NN input vector
  // in both modes. Short names match kPathgenInputMeta / kTrackerInputMeta so a
  // column and its metadata row cannot drift.
  //
  // ⚠️ Not optional decoration. Two consumers need these and had no source:
  //   * T061's smoke inspection — "IN_ENVELOPE toggles, ENVELOPE_SECS ramps and
  //     resets, ACCEL_Z reads ~1 g in level flight" was uninspectable, because
  //     dmp_dump emitted only a hand-picked subset of NNInputs.
  //   * Study A (T055) reads load from the recorded ACCEL_Z column, explicitly
  //     NOT from physicsTrace — the trace is capped at 175 ms per scenario
  //     (0.89% of ticks) and only captured on elite reeval.
  //
  // ⚠️ acX/acY/acZ are in NN units (specific force / kAccelScale_g), the value
  // the policy actually saw. Multiply by the dmp's recorded accelScaleG for g.
  // The sfx_g/sfy_g/sfz_g/nz_g columns under --physics are a DIFFERENT
  // quantity: derived from the physics trace, in g, and present on ~1% of
  // ticks. Same physics, different provenance and coverage — do not mix them in
  // one analysis without saying which is which.
  std::cout << ",env,envS,acX,acY,acZ";
  // 041 T010 — physics columns last, so every existing column index is
  // unchanged for readers that ignore the flag.
  if (physics) std::cout << ",phyMs,accX,accY,accZ,odbP,odbQ,odbR,alpha,vRelWind,sfx_g,sfy_g,sfz_g,nz_g";
  std::cout << "\n";

  // 038 P0-B (T010): read the fitness cone / cadence from the dmp-recorded,
  // self-describing RecordedRunConfig — NOT the live .ini. M2 greenfield: no
  // fallback to ConfigManager (a pre-038 dmp with a zeroed runConfig is
  // reproduced by checking out the matching code, not by reading today's ini).
  // Only S3 bucket/profile still comes from ConfigManager.
  const RecordedRunConfig& rc = results.runConfig;
  const double dtSec = rc.simTimeStepMsec / 1000.0;
  int streakStepsToMax = static_cast<int>(rc.fitStreakRampSec / dtSec);
  if (streakStepsToMax < 1) streakStepsToMax = 1;

  for (size_t si = 0; si < results.tickList.size(); ++si) {
    const auto& ticks = results.tickList[si].ticks;
    if (ticks.empty()) continue;
    const bool sceneTracker = ticks.front().targetSample.has_value();
    const std::vector<Path>* path =
        (si < results.pathList.size()) ? &results.pathList[si] : nullptr;

    FitnessComputer fc(rc.fitDistScaleBehind, rc.fitDistScaleAhead, rc.fitConeAngleDeg,
                       rc.fitStreakThreshold, streakStepsToMax, rc.fitStreakMultiplierMax);
    fc.resetStreak();
    gp_vec3 prevTangent = gp_vec3::UnitX();
    double prevSpan = 0.0;  // tracker spn0 at ti-1, for dspn (0 at first emitted tick)
    bool havePrevSpan = false;

    // 041 T010 — joined on time, not index (see PhysicsJoin).
    PhysicsJoin phy;
    if (physics && si < results.physicsTrace.size()) {
      phy.trace = &results.physicsTrace[si];
      // Half a control tick: a physics row must describe THIS tick, not a
      // neighbouring one.
      phy.tolMsec = rc.simTimeStepMsec / 2.0;
    }

    // 041 T023 — one series. `ti` is the 1-based tick NUMBER emitted in the CSV
    // (unchanged), and `ticks[ti - 1]` is that tick's complete record. The
    // clamped `min(ti, targets.size()-1)` lookups below are gone.
    for (size_t ti = 1; ti <= ticks.size(); ++ti) {
      const EvalTick& tickRecord = ticks[ti - 1];
      const auto& st = tickRecord.state;
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
        // 041 T023 — ⚠️ THIS READER HAD ITS OWN OFF-BY-ONE, in the opposite
        // direction from the objective's. It paired tick `ti` with
        // `targets[min(ti, size-1)]`, i.e. one tick LATER than the objective's
        // `targets[ti - 1]`. Every tracker column emitted here (tg*, tr*, spn0,
        // blC0/brC0, tltS/tltC, and the recomputed stpPt) therefore described a
        // different instant from the one the objective scored — a reader
        // silently disagreeing with the thing it exists to explain. Pre-041 CSV
        // extracts carry the old pairing; see artifacts/pre-break/README.md.
        const CopiedTargetSample* target =
            tickRecord.targetSample.has_value() ? &*tickRecord.targetSample : nullptr;
        const int hull = (target != nullptr && target->inside_crash_hull) ? 1 : 0;

        // Target + trail-rabbit pose (0s if the trajectory list is missing/empty).
        gp_vec3 tg = gp_vec3::Zero(), tr = gp_vec3::Zero();
        double stp = 0.0;  // in-cone step score (tracking metric)
        if (target != nullptr) {
          tg = target->position;
          tr = target->trail_rabbit_position;
          // Per-tick in-cone step score, recomputed from recorded target
          // geometry EXACTLY as fitness_decomposition.cc's tracker branch:
          // rabbit = trail-rabbit, tangent = target-velocity unit (prevTangent
          // fallback when degenerate). This is the tracker counterpart to the
          // pathgen `stpPt` column — derived, not recorded, like pathgen — so
          // dynamics_progress et al. get a real per-tick tracking flag on M2.
          const gp_vec3 tvel = target->velocity;
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
        if (tickRecord.cameraView.has_value()) {
          const CameraViewSample& cv = *tickRecord.cameraView;
          const auto& bl = cv.beacon_left;
          const auto& br = cv.beacon_right;
          blC0 = bl.cep;
          brC0 = br.cep;
          const bool gated = (bl.cep >= autoc::eval::kCepSentinelThreshold)
                          || (br.cep >= autoc::eval::kCepSentinelThreshold);
          if (!gated) {
            // 040 T031 — bearings are radians now; span is therefore a true
            // angular quantity and range is separation_m / span directly.
            spn0 = autoc::eval::compute_pair_span(
                bl.bearing_x_rad, bl.bearing_y_rad,
                br.bearing_x_rad, br.bearing_y_rad);
            const auto t = autoc::eval::compute_tilt(
                bl.bearing_x_rad, bl.bearing_y_rad,
                br.bearing_x_rad, br.bearing_y_rad);
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
          ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
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
        int dn = snprintf(d, sizeof(d), ",%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f",
                          dist, along, stp, mult, rampSc,
                          nn_in.dist_to_boundary, nn_in.inward_body_x,
                          nn_in.inward_body_y, nn_in.inward_body_z);
        std::cout.write(d, dn);
      } else {
        // 038 P0-D FR-P0H (B) — no path geometry, but the recorded NNInputs
        // still carry the arena-awareness (B) inputs.
        const NNInputs& nn_in = st.getNNInputs();
        char d[128];
        int dn = snprintf(d, sizeof(d), ",,,,,%.4f,%.6f,%.6f,%.6f,%.6f",
                          rampSc, nn_in.dist_to_boundary, nn_in.inward_body_x,
                          nn_in.inward_body_y, nn_in.inward_body_z);
        std::cout.write(d, dn);
      }

      // 041 T037-T039 — the five A1 slots, straight off the recorded input
      // vector. Mode-branched only because the two input structs are unrelated
      // types; the VALUES mean the same thing in both, which is the point.
      {
        char eb[160];
        int en;
        if (sceneTracker) {
          const TrackerInputs& in = st.getTrackerInputs();
          en = snprintf(eb, sizeof(eb), ",%.0f,%.6f,%.6f,%.6f,%.6f",
                        in.in_envelope, in.envelope_secs,
                        in.accel_x, in.accel_y, in.accel_z);
        } else {
          const NNInputs& in = st.getNNInputs();
          en = snprintf(eb, sizeof(eb), ",%.0f,%.6f,%.6f,%.6f,%.6f",
                        in.in_envelope, in.envelope_secs,
                        in.accel_x, in.accel_y, in.accel_z);
        }
        std::cout.write(eb, en);
      }

      // 041 T010 — physics tail, joined on recorded sim time. Empty fields when
      // this dmp carries no physics trace (i.e. was not an elite reeval), so a
      // missing trace reads as missing rather than as zeros.
      if (physics) {
        const PhysicsTraceEntry* p =
            phy.at(static_cast<double>(st.getSimTimeMsec()));
        if (p != nullptr) {
          const autoc::eval::BodySpecificForce dl = deriveLoad(*p);
          char pb[320];
          int pn = snprintf(pb, sizeof(pb),
            ",%.1f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
            p->simTimeMsec,
            p->acc[0], p->acc[1], p->acc[2],
            p->omegaDotBody[0], p->omegaDotBody[1], p->omegaDotBody[2],
            p->alpha, p->vRelWind,
            static_cast<double>(dl.g_units.x()), static_cast<double>(dl.g_units.y()),
            static_cast<double>(dl.g_units.z()), static_cast<double>(dl.load_factor_nz));
          std::cout.write(pb, pn);
        } else {
          std::cout << ",,,,,,,,,,,,";
        }
      }
      std::cout << "\n";
    }

    // Report the join quality per scenario rather than silently producing a
    // plausible CSV. A large skew means the control tick and the physics
    // substep have drifted apart and the load columns describe a different
    // moment from the rest of the row.
    if (physics && phy.unmatched > 0) {
      std::cerr << "dmp-dump --physics: scenario " << si
                << ": " << phy.matched << " ticks have physics, "
                << phy.unmatched << " do not"
                << " (worst matched skew " << phy.worstSkewMsec << " ms,"
                << " tolerance " << phy.tolMsec << " ms).";
      if (phy.matched > 0 && phy.trace != nullptr) {
        std::cerr << " NOTE: the FDM caps the trace at MAX_TRACE_STEPS"
                  << " (fdm_larcsim.cpp), so only the first ~200 ms of each"
                  << " scenario carries physics — the empty rows are expected"
                  << " on any pre-041 dmp, NOT a join failure.";
      }
      std::cerr << "\n";
    }
  }

  if (awsInit) Aws::ShutdownAPI(awsOptions);
  return 0;
}
