#ifndef NN_EVALUATOR_PORTABLE_H
#define NN_EVALUATOR_PORTABLE_H

#include <vector>
#include <cstdint>
#include <cmath>
#include "autoc/types.h"
#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/arena.h"   // 038 P0-D — FlightArena (cereal-free, xiao-safe) + inwardBodyDirection
#include "autoc/eval/fitness_decomposition.h"

// Neural network genome — the fundamental unit of neuroevolution.
//
// Weight layout (spec 027):
//   For each layer transition l → l+1 (in order):
//     [W_l (out*in row-major), B_l (out)]
//   Then, appended at the end of the weight vector:
//     For each recurrent layer l (in order of layer index):
//       [W_hh_l (size*size row-major)]
//   Feedforward case (no recurrent layers) matches pre-027 layout.
struct NNGenome {
    std::vector<float> weights;       // Weights + biases + optional W_hh blocks (see above)
    std::vector<int> topology;        // Layer sizes, e.g., {33, 32, 16, 3}
    std::vector<uint8_t> recurrent;   // Per-layer recurrent flag (0/1), same size as topology
    double fitness;                   // Aggregated fitness from evaluation
    std::vector<ScenarioScore> scenario_scores;  // Per-scenario decomposed scores (015)
    uint32_t generation;              // Generation when created
    float mutation_sigma;             // Per-individual mutation step size (self-adaptive)
    float variation_scale;            // computeVariationScale() at save time — eval uses directly
    std::string source;               // Provenance: "bucket/key" from S3 extraction

    NNGenome() : fitness(0.0), generation(0), mutation_sigma(0.1f), variation_scale(1.0f) {}
};

// Compute total weight+bias count from topology (feedforward only).
// Callers that need recurrent-aware count pass the recurrent flag vector.
int nn_weight_count(const std::vector<int>& topology);
int nn_weight_count(const std::vector<int>& topology,
                    const std::vector<uint8_t>& recurrent);

// Total recurrent hidden-state float count (sum of size for each recurrent
// layer). Zero when `recurrent` is empty or all-false.
int nn_hidden_state_count(const std::vector<int>& topology,
                          const std::vector<uint8_t>& recurrent);

// Forward pass: feedforward with tanh activation
// weights layout: row-major, layer-sequential [W1, B1, W2, B2, ...]
void nn_forward(const float* weights, const std::vector<int>& topology,
                const float* inputs, float* outputs);

// Signal 1 telemetry — see specs/028-deeper-rnn/data-model.md §2.
// Defined here (not telemetry.h) so that NNControllerBackend can hold one
// by value without a forward-decl/incomplete-type problem. All members are
// inline so this struct does not require linking telemetry.cc — important
// for crrcsim's mod_inputdev which compiles evaluator.cc directly without
// pulling in autoc_common.a.
struct RecurrentTelemetry {
    double xh_mag_sum = 0.0;
    double hh_mag_sum = 0.0;
    long long sample_count = 0;
    double activation_ratio() const {
        if (sample_count == 0) return 0.0;
        const double xh_mean = xh_mag_sum / static_cast<double>(sample_count);
        const double hh_mean = hh_mag_sum / static_cast<double>(sample_count);
        if (xh_mean < 1e-9) return 0.0;
        return hh_mean / xh_mean;
    }
    void reset() { xh_mag_sum = 0.0; hh_mag_sum = 0.0; sample_count = 0; }
};

// Recurrent forward pass: same as nn_forward but with a persistent hidden
// state. `hidden_state` is read-and-written — caller owns storage, zeros it
// on span start, and keeps it across ticks. Layout of hidden_state matches
// layer iteration order: for each recurrent layer, `topology[l]` floats are
// consumed/produced sequentially. W_hh blocks live at the tail of `weights`
// per the NNGenome layout above.
//
// If `telemetry` is non-null, the function accumulates per-recurrent-neuron
// magnitudes of |W_xh row · x_t + b| and |W_hh row · h_{t-1}| into the
// telemetry struct. Default null = no overhead, matches pre-028 behavior.
// See specs/028-deeper-rnn/data-model.md §2.
void nn_forward_recurrent(const float* weights,
                          const std::vector<int>& topology,
                          const std::vector<uint8_t>& recurrent,
                          const float* inputs, float* outputs,
                          float* hidden_state,
                          RecurrentTelemetry* telemetry = nullptr);

// Fast tanh via 512-entry LUT with linear interpolation
gp_scalar fast_tanh(gp_scalar x);

// Xavier/Glorot weight initialization
void nn_xavier_init(NNGenome& genome);

// Gather pathgen-mode NN sensor inputs (PathgenInput::COUNT = 33 floats) from
// aircraft state (raw, no normalization). Renamed from nn_gather_inputs in
// 030 M2b — the typed function name is the FR-006 + FR-019 pluggable-mode
// dispatch contract (see autoc/nn/mode.h kPathgenMode strategy bundle).
// Body unchanged from the pre-rename nn_gather_inputs (FP-bit-exact).
// 038 P0-D FR-P0H (B): pathgen (M1) now also carries arena-awareness inputs
// (dist_to_boundary + inward_body), so the gather takes the FlightArena. The
// desktop backend holds one (from config); nn2cpp bakes a compile-time literal
// into the xiao firmware.
void gather_pathgen_inputs(PathProvider& pathProvider, AircraftState& aircraftState,
                           const autoc::eval::FlightArena& arena,
                           NNInputs& inputs);

// 030 M6d — Tracker-mode NN sensor input gather (FR-006 + FR-016 + FR-019).
// 030 M7a — Reshape to use arena.h::distanceToBoundary for slot 44
// (Session 2026-05-07 Q1: drop HOME_X/Y/Z/HOME_DIST, add single
// DIST_TO_BOUNDARY_ALONG_VEL).
//
// Six-slot beacon-observation history per channel (oldest at index 0,
// "now" at index 5). Caller (TrackerStepper) owns + advances the history
// across ticks; this function is pure: read history + chase state +
// arena → fill TrackerInputs.
//
// Replaces the M2b stub `gather_tracker_inputs_stub` in src/nn/mode.cc.
struct TrackerHistoryWindow {  // raw-ok: NN-byte-format buffer
    float left_x[6];      // raw-ok: NN-byte-format buffer
    float left_y[6];      // raw-ok: NN-byte-format buffer
    float left_cep[6];    // raw-ok: NN-byte-format buffer
    float right_x[6];     // raw-ok: NN-byte-format buffer
    float right_y[6];     // raw-ok: NN-byte-format buffer
    float right_cep[6];   // raw-ok: NN-byte-format buffer

    // 032 PHASE 1 — Cached beacon-pair span across the history window.
    // Span = ||right.xy - left.xy|| in NDC, computed and CEP-gated at
    // projection time (src/eval/tracker_stepper.cc::projectAndShiftHistory
    // and the crrcsim mirror). Cached here so gather_tracker_inputs reads
    // it via the same memcpy pattern as the NDC channels — single
    // computation site, consistent CEP-gating semantics.
    float span[6];        // raw-ok: NN-byte-format buffer
};

// 037 T022 — deep per-tick beacon-observation ring. With the R5 ms-based
// lag set the 6 window slots are no longer the 6 most-recent ticks, so the
// steppers keep every tick's observation in this ring (depth = deepest lag
// in ticks + 1) and MATERIALIZE the 6-slot TrackerHistoryWindow gather-view
// from it at the lag offsets each tick. Shared by both tracker mirrors
// (src/eval/tracker_stepper.cc and crrcsim_tracker_helper.cpp) so the
// per-tick semantics cannot drift. Warm-up clamps to the oldest available
// record (mirrors AircraftState::getHistorical* CLAMP_DEF behavior).
struct TrackerObservationRing {
    struct Record {  // raw-ok: NN-byte-format staging (slots copy verbatim)
        float left_x, left_y, left_cep;     // raw-ok: NN-byte-format staging
        float right_x, right_y, right_cep;  // raw-ok: NN-byte-format staging
        float span;                         // raw-ok: NN-byte-format staging
    };
    static constexpr int kDepth = (kNNHistoryLagsMsec[0] / SIM_TIME_STEP_MSEC) + 1;

    Record ring[kDepth] = {};
    int index = 0;  // next write position
    int count = 0;  // valid records (0..kDepth)

    void reset() {
        index = 0;
        count = 0;
    }

    void push(const Record& r) {
        ring[index] = r;
        index = (index + 1) % kDepth;
        if (count < kDepth) ++count;
    }

    // n = lag in ticks; 0 = most recent. Clamps into the valid range.
    const Record& at(int n) const {
        n = CLAMP_DEF(n, 0, (count > 0 ? count : 1) - 1);
        int idx = (index - 1 - n + 2 * kDepth) % kDepth;
        return ring[idx];
    }

    void materialize(TrackerHistoryWindow& w) const {
        for (int s = 0; s < 6; ++s) {
            const Record& r = at(historyLagTicks(s));
            w.left_x[s] = r.left_x;        // raw-ok: NN-byte-format primitive
            w.left_y[s] = r.left_y;        // raw-ok: NN-byte-format primitive
            w.left_cep[s] = r.left_cep;    // raw-ok: NN-byte-format primitive
            w.right_x[s] = r.right_x;      // raw-ok: NN-byte-format primitive
            w.right_y[s] = r.right_y;      // raw-ok: NN-byte-format primitive
            w.right_cep[s] = r.right_cep;  // raw-ok: NN-byte-format primitive
            w.span[s] = r.span;            // raw-ok: NN-byte-format primitive
        }
    }
};

// 038 P0-D FR-P0H (A) — situational-awareness "target-lost" state (tracker
// only; M1's rabbit is always visible). Stateful and held/decaying across
// ticks; MUST reset per scenario/engage (un-reset state leaks across
// scenarios and breaks the FR-030 bitwise gate). Single-sourced here so
// TrackerStepper and CrrcsimTrackerHelper — and dmp_dump's honest
// reconstruction — share one update rule that cannot drift.
struct SituationalAwarenessState {
    int blind_ticks = 0;         // consecutive ticks with no visible beacon
    float exit_dir_sin = 0.0f;   // raw-ok: NN-byte-format staging — held last-seen bearing
    float exit_dir_cos = 1.0f;   // raw-ok: NN-byte-format staging — neutral (0,1) until first sighting

    void reset() {
        blind_ticks = 0;
        exit_dir_sin = 0.0f;
        exit_dir_cos = 1.0f;
    }

    // Per-tick update from the "now" beacon observation. A beacon is
    // "visible" when its CEP is below the sentinel threshold (matches the
    // fitness_decomposition.cc:200 visibility definition; caller passes
    // kCepSentinelThreshold). While visible: zero the blind counter and
    // refresh the held exit-bearing from the beacon-pair NDC centroid. While
    // blind: increment the counter and HOLD the last-seen bearing.
    void update(float left_x, float left_y, float left_cep,
                float right_x, float right_y, float right_cep,
                float visible_cep_threshold) {
        const bool visible = (left_cep  < visible_cep_threshold) ||
                             (right_cep < visible_cep_threshold);
        if (!visible) {
            ++blind_ticks;
            return;
        }
        blind_ticks = 0;
        const float cx = 0.5f * (left_x + right_x);
        const float cy = 0.5f * (left_y + right_y);
        const float mag = std::sqrt(cx * cx + cy * cy);
        if (mag > 1e-6f) {
            exit_dir_sin = cy / mag;
            exit_dir_cos = cx / mag;
        }
        // else: centroid at image center — hold the previous bearing.
    }

    // Write the three (A) slots into a TrackerInputs. time_since_seen =
    // tanh(blind_seconds / kTimeSinceSeenScale_s); 0 exactly when visible now.
    void writeInputs(TrackerInputs& out) const {
        const float blind_seconds = static_cast<float>(blind_ticks) *
            (static_cast<float>(SIM_TIME_STEP_MSEC) / 1000.0f);
        out.time_since_seen = std::tanh(blind_seconds / kTimeSinceSeenScale_s);  // raw-ok: NN-byte-format slot write
        out.exit_dir_sin = exit_dir_sin;   // raw-ok: NN-byte-format slot write
        out.exit_dir_cos = exit_dir_cos;   // raw-ok: NN-byte-format slot write
    }
};

void gather_tracker_inputs(const AircraftState& chase,
                           const TrackerHistoryWindow& history,
                           const autoc::eval::FlightArena& arena,
                           float cep_gate_threshold,
                           const SituationalAwarenessState& sa,
                           TrackerInputs& out);

#include "autoc/eval/backend.h"

// NN controller backend — plugs into unified eval pipeline.
//
// Spec 027: when `genome.recurrent` has any true entry, the backend carries
// a persistent `hidden_state_` across evaluate() calls. Callers MUST:
//   1. Construct the backend ONCE per span (not per tick).
//   2. Call reset() on span start to zero hidden state.
//   3. Call evaluate() on NN eval ticks (~10 Hz), not on intermediate
//      outer-frame ticks — matches spec 027 clarify Q4.
//
// Feedforward-only genomes (all recurrent flags false / empty) behave
// identically to pre-027 — hidden_state_ is empty and unused.
class NNControllerBackend : public ControllerBackend {
public:
    // 038 P0-D FR-P0H (B): the backend carries the FlightArena so the pathgen
    // evaluate() path can populate the arena-awareness inputs. No default per
    // the M2-era no-fallback policy — every call site passes the config arena
    // explicitly (tracker call sites pass it too even though evaluateTracker
    // gathers the arena separately; the member is simply unused there).
    NNControllerBackend(const NNGenome& genome, const autoc::eval::FlightArena& arena);

    void evaluate(AircraftState& aircraftState, PathProvider& pathProvider) override;
    const char* getName() const override { return "NeuralNet"; }

    // 030 M6d — Tracker-mode NN forward pass (FR-019). Parallel entry point
    // to evaluate(); caller pre-gathers TrackerInputs (48 floats) and passes
    // them in. Pathgen `evaluate` body is byte-identical pre-/post-M6d
    // (regression-tight invariant) — `evaluateTracker` is purely additive.
    void evaluateTracker(AircraftState& aircraftState, const TrackerInputs& inputs);

    // Zero the recurrent hidden state. Call on span/engage start.
    // No-op for feedforward networks.
    void reset();

    // 028 — telemetry capture for signal 1 (W_hh / W_xh activation ratio).
    // Off by default; enable only on best-of-gen evaluations to avoid
    // per-individual cost during selection. See specs/028-deeper-rnn/spec.md
    // §Clarifications and data-model.md §2.2.
    void enableTelemetryCapture();
    void disableTelemetryCapture();
    void resetTelemetry();
    double telemetryActivationRatio() const;
    long long telemetrySampleCount() const;

private:
    const NNGenome& genome_;
    autoc::eval::FlightArena arena_;   // 038 P0-D — config arena for pathgen (B) inputs
    std::vector<float> hidden_state_;  // Sized by nn_hidden_state_count(); empty for feedforward
    // 028: held by value; cheap, no allocation. Captures samples only when
    // telemetry_capture_enabled_ is true (see enableTelemetryCapture()).
    RecurrentTelemetry telemetry_;
    bool telemetry_capture_enabled_ = false;
};

// Expose LUT functions for testing
gp_scalar testFastTanh(gp_scalar x);

#endif
