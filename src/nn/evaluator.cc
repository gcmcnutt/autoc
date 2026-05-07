#include "autoc/nn/evaluator.h"
#include "autoc/nn/topology.h"
#include "autoc/nn/nn_input_computation.h"
#include "autoc/eval/arena.h"          // 030 M7a — FlightArena + distanceToBoundary
#include "autoc/eval/sensor_math.h"
#include "autoc/util/rng.h"
#include <cmath>
#include <array>
#include <algorithm>


// ============================================================
// T036: Weight count
// ============================================================

int nn_weight_count(const std::vector<int>& topology) {
    int count = 0;
    for (size_t i = 0; i + 1 < topology.size(); i++) {
        count += topology[i] * topology[i + 1] + topology[i + 1];
    }
    return count;
}

int nn_weight_count(const std::vector<int>& topology,
                    const std::vector<uint8_t>& recurrent) {
    int count = nn_weight_count(topology);
    // W_hh blocks appended for each recurrent layer: size × size (no bias).
    for (size_t l = 0; l < topology.size() && l < recurrent.size(); l++) {
        if (recurrent[l]) {
            count += topology[l] * topology[l];
        }
    }
    return count;
}

int nn_hidden_state_count(const std::vector<int>& topology,
                          const std::vector<uint8_t>& recurrent) {
    int count = 0;
    for (size_t l = 0; l < topology.size() && l < recurrent.size(); l++) {
        if (recurrent[l]) count += topology[l];
    }
    return count;
}

// ============================================================
// T035: fast_tanh LUT — 512 entries, domain [-5, 5]
// ============================================================

namespace {
constexpr int TANH_LUT_SIZE = 512;
constexpr gp_scalar TANH_DOMAIN = static_cast<gp_scalar>(5.0f);
std::array<gp_scalar, TANH_LUT_SIZE + 1> TANH_LUT{};
bool TANH_LUT_INIT = false;

inline void initTanhLut() {
    if (TANH_LUT_INIT) return;
    for (int i = 0; i <= TANH_LUT_SIZE; ++i) {
        gp_scalar x = -TANH_DOMAIN + static_cast<gp_scalar>(2.0f) * TANH_DOMAIN
                       * static_cast<gp_scalar>(i) / static_cast<gp_scalar>(TANH_LUT_SIZE);
        TANH_LUT[i] = static_cast<gp_scalar>(std::tanh(x));
    }
    TANH_LUT_INIT = true;
}
} // namespace

gp_scalar fast_tanh(gp_scalar x) {
    initTanhLut();

    // Clamp to domain
    if (x <= -TANH_DOMAIN) return static_cast<gp_scalar>(-1.0f);
    if (x >= TANH_DOMAIN) return static_cast<gp_scalar>(1.0f);

    // Map x from [-5, 5] to [0, TANH_LUT_SIZE]
    gp_scalar scaled = (x + TANH_DOMAIN) / (static_cast<gp_scalar>(2.0f) * TANH_DOMAIN)
                       * static_cast<gp_scalar>(TANH_LUT_SIZE);
    int i0 = static_cast<int>(scaled);
    if (i0 < 0) i0 = 0;
    if (i0 >= TANH_LUT_SIZE) i0 = TANH_LUT_SIZE - 1;
    gp_scalar frac = scaled - static_cast<gp_scalar>(i0);
    return TANH_LUT[i0] + (TANH_LUT[i0 + 1] - TANH_LUT[i0]) * frac;
}

// ============================================================
// T034: Forward pass — feedforward with tanh activation
// Weight layout: row-major, layer-sequential [W1, B1, W2, B2, ...]
// For layer l: W has shape [out_size x in_size], B has shape [out_size]
// output[j] = tanh(sum_i(W[j*in_size + i] * input[i]) + B[j])
// ============================================================

void nn_forward(const float* weights, const std::vector<int>& topology,
                const float* inputs, float* outputs) {
    if (topology.size() < 2) return;

    // Double-buffer: alternate between buf_a and buf_b for layer I/O
    int max_layer = *std::max_element(topology.begin(), topology.end());
    // Use stack allocation for typical sizes, heap for unusual
    float buf_a_stack[64], buf_b_stack[64];
    float* buf_a = (max_layer <= 64) ? buf_a_stack : new float[max_layer];
    float* buf_b = (max_layer <= 64) ? buf_b_stack : new float[max_layer];

    // Copy inputs to buf_a
    for (int i = 0; i < topology[0]; i++) {
        buf_a[i] = inputs[i];
    }

    const float* w_ptr = weights;
    float* current_in = buf_a;
    float* current_out = buf_b;

    for (size_t layer = 0; layer + 1 < topology.size(); layer++) {
        int in_size = topology[layer];
        int out_size = topology[layer + 1];

        // Compute: current_out[j] = tanh(sum(W[j*in_size+i] * current_in[i]) + B[j])
        const float* W = w_ptr;
        const float* B = w_ptr + in_size * out_size;

        for (int j = 0; j < out_size; j++) {
            float sum = B[j];
            for (int i = 0; i < in_size; i++) {
                sum += W[j * in_size + i] * current_in[i];
            }
            current_out[j] = static_cast<float>(fast_tanh(static_cast<gp_scalar>(sum)));
        }

        w_ptr += in_size * out_size + out_size;

        // Swap buffers
        float* tmp = current_in;
        current_in = current_out;
        current_out = tmp;
    }

    // Copy final output (current_in points to last layer's output after swap)
    int out_size = topology.back();
    for (int i = 0; i < out_size; i++) {
        outputs[i] = current_in[i];
    }

    // Clean up heap allocations if used
    if (max_layer > 64) {
        delete[] buf_a;
        delete[] buf_b;
    }
}

// ============================================================
// Recurrent forward pass (spec 027, D-simple)
// ============================================================
// Same as nn_forward but with a persistent hidden state for any layer
// flagged recurrent. W_hh blocks are appended at the end of the weights
// vector, in layer-index order, after all feedforward [W, B] pairs.
//
// For a recurrent layer l with size N:
//     h_t[j] = tanh(W_xh[j] · x_t + W_hh[j] · h_{t-1}[j] + B[j])
// i.e., we add the dot product of the row j of W_hh (size N) with the
// previous h_{t-1} (also size N) to the usual pre-activation sum.

void nn_forward_recurrent(const float* weights,
                          const std::vector<int>& topology,
                          const std::vector<uint8_t>& recurrent,
                          const float* inputs, float* outputs,
                          float* hidden_state,
                          RecurrentTelemetry* telemetry) {
    if (topology.size() < 2) return;

    int max_layer = *std::max_element(topology.begin(), topology.end());
    float buf_a_stack[64], buf_b_stack[64];
    float* buf_a = (max_layer <= 64) ? buf_a_stack : new float[max_layer];
    float* buf_b = (max_layer <= 64) ? buf_b_stack : new float[max_layer];

    for (int i = 0; i < topology[0]; i++) buf_a[i] = inputs[i];

    // Pre-compute W_hh offsets: feedforward block total first, then
    // W_hh blocks in layer-index order for each recurrent layer.
    int ff_total = 0;
    for (size_t i = 0; i + 1 < topology.size(); i++) {
        ff_total += topology[i] * topology[i + 1] + topology[i + 1];
    }
    const float* whh_base = weights + ff_total;

    // Compute a per-recurrent-layer starting offset into `hidden_state` and
    // into the W_hh blob, iterating in order.
    int hs_offset = 0;
    int whh_offset = 0;

    const float* w_ptr = weights;
    float* current_in = buf_a;
    float* current_out = buf_b;

    for (size_t layer = 0; layer + 1 < topology.size(); layer++) {
        int in_size = topology[layer];
        int out_size = topology[layer + 1];
        const bool out_is_recurrent = (layer + 1 < recurrent.size())
                                      && recurrent[layer + 1];

        const float* W = w_ptr;
        const float* B = w_ptr + in_size * out_size;

        for (int j = 0; j < out_size; j++) {
            float xh_part = B[j];
            for (int i = 0; i < in_size; i++) {
                xh_part += W[j * in_size + i] * current_in[i];
            }
            float hh_part = 0.0f;
            if (out_is_recurrent) {
                // Add W_hh row j · h_{t-1}
                const float* Whh_row = whh_base + whh_offset + j * out_size;
                const float* h_prev = hidden_state + hs_offset;
                for (int i = 0; i < out_size; i++) {
                    hh_part += Whh_row[i] * h_prev[i];
                }
            }
            const float sum = xh_part + hh_part;
            current_out[j] = static_cast<float>(fast_tanh(static_cast<gp_scalar>(sum)));

            // 028 telemetry: accumulate per-recurrent-neuron magnitudes when capture is on.
            // Skipped for non-recurrent neurons (out_is_recurrent==false) — the ratio
            // metric is only defined where W_hh exists.
            if (telemetry && out_is_recurrent) {
                telemetry->xh_mag_sum += static_cast<double>(std::fabs(xh_part));
                telemetry->hh_mag_sum += static_cast<double>(std::fabs(hh_part));
                telemetry->sample_count++;
            }
        }

        if (out_is_recurrent) {
            // Write h_t back to hidden_state (overwrites h_{t-1}).
            for (int j = 0; j < out_size; j++) {
                hidden_state[hs_offset + j] = current_out[j];
            }
            hs_offset += out_size;
            whh_offset += out_size * out_size;
        }

        w_ptr += in_size * out_size + out_size;

        float* tmp = current_in;
        current_in = current_out;
        current_out = tmp;
    }

    int out_size = topology.back();
    for (int i = 0; i < out_size; i++) outputs[i] = current_in[i];

    if (max_layer > 64) {
        delete[] buf_a;
        delete[] buf_b;
    }
}

// ============================================================
// T037: Xavier/Glorot initialization
// ============================================================

void nn_xavier_init(NNGenome& genome) {
    const int total = nn_weight_count(genome.topology, genome.recurrent);
    genome.weights.resize(total);

    int idx = 0;
    // Feedforward [W, B] pairs for each layer transition
    for (size_t layer = 0; layer + 1 < genome.topology.size(); layer++) {
        int fan_in = genome.topology[layer];
        int fan_out = genome.topology[layer + 1];
        double stddev = std::sqrt(1.0 / fan_in);

        int num_weights = fan_in * fan_out;
        for (int i = 0; i < num_weights; i++) {
            genome.weights[idx++] = static_cast<float>(rng::randGaussian(stddev));
        }
        for (int i = 0; i < fan_out; i++) genome.weights[idx++] = 0.0f;
    }
    // W_hh blocks for each recurrent layer, appended in layer-index order.
    // Xavier fan-in for the recurrent block uses the layer's own size +
    // its incoming size (it "sees" both x_t and h_{t-1}).
    for (size_t layer = 0; layer < genome.topology.size() && layer < genome.recurrent.size(); layer++) {
        if (!genome.recurrent[layer]) continue;
        int size = genome.topology[layer];
        int in_size = (layer > 0) ? genome.topology[layer - 1] : size;
        double stddev = std::sqrt(1.0 / (in_size + size));
        int num_weights = size * size;
        for (int i = 0; i < num_weights; i++) {
            genome.weights[idx++] = static_cast<float>(rng::randGaussian(stddev));
        }
    }
}

// ============================================================
// T040: Gather NN_INPUT_COUNT sensor inputs — raw, no normalization
// ============================================================
// Layout (33 inputs — 023 direction cosines, 029 US1 past-only redistribution):
//  0- 5: target_x [-0.5s,-0.4s,-0.3s,-0.2s,-0.1s,now]  body-frame unit-vec x
//  6-11: target_y [-0.5s,-0.4s,-0.3s,-0.2s,-0.1s,now]  body-frame unit-vec y
// 12-17: target_z [-0.5s,-0.4s,-0.3s,-0.2s,-0.1s,now]  body-frame unit-vec z
// 18-23: dist     [-0.5s,-0.4s,-0.3s,-0.2s,-0.1s,now]  raw metres
//    24: dDist/dt closing rate (m/s, positive = approaching)
// 25-28: quaternion (w, x, y, z)                         [-1,1]
//    29: airspeed (m/s)
// 30-32: gyro rates (p, q, r) in rad/s                   standard aerospace RHR
//
// 029 US1 past-only redistribution: all 6 slots use recorded aircraft history
// at tick offsets [5, 4, 3, 2, 1, 0]. Future-lookahead slots (+0.1s, +0.5s)
// are dropped because tracker mode (the eventual consumer of this layout) has
// no parametric path to look ahead on. PathProvider parameter is retained on
// the signature for API stability across nn2cpp / xiao callers but is unused
// here.

// History slot indices (tick offsets at SIM_TIME_STEP_MSEC = 100 ms):
// [5, 4, 3, 2, 1, 0] = [-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now]
static const int HIST_PAST[] = {5, 4, 3, 2, 1, 0};

void gather_pathgen_inputs([[maybe_unused]] PathProvider& pathProvider,
                           AircraftState& aircraftState,
                           NNInputs& inputs) {
    // target_x/y/z[0-5]: past history (direction cosines from recorded history)
    for (int i = 0; i < 6; i++) {
        gp_vec3 dir = aircraftState.getHistoricalTargetDir(HIST_PAST[i]);
        inputs.target_x[i] = static_cast<float>(dir.x());
        inputs.target_y[i] = static_cast<float>(dir.y());
        inputs.target_z[i] = static_cast<float>(dir.z());
    }

    // dist[0-5]: past history (raw metres)
    for (int i = 0; i < 6; i++)
        inputs.dist[i] = static_cast<float>(aircraftState.getHistoricalDist(HIST_PAST[i]));

    // closing_rate: dDist/dt (m/s, positive = approaching)
    {
        float dist_now  = static_cast<float>(aircraftState.getHistoricalDist(0));
        float dist_prev = static_cast<float>(aircraftState.getHistoricalDist(1));
        inputs.closing_rate = (dist_prev - dist_now) / 0.1f;  // divide by 0.1s tick
    }

    // quaternion attitude (w, x, y, z) — unit norm, components in [-1,1]
    {
        gp_quat q = aircraftState.getOrientation();
        inputs.quat_w = static_cast<float>(q.w());
        inputs.quat_x = static_cast<float>(q.x());
        inputs.quat_y = static_cast<float>(q.y());
        inputs.quat_z = static_cast<float>(q.z());
    }

    // airspeed (m/s, raw)
    inputs.airspeed = static_cast<float>(aircraftState.getRelVel());

    // gyro rates (p, q, r) in rad/s (raw, no scaling)
    // Body-frame angular rates, standard aerospace RHR convention.
    // CRRCSim FDM provides these directly; INAV requires pitch/yaw negation
    // at consumer boundary (see COORDINATE_CONVENTIONS.md).
    {
        gp_vec3 gyro = aircraftState.getGyroRates();
        inputs.gyro_p = static_cast<float>(gyro.x());  // p (roll rate, rad/s)
        inputs.gyro_q = static_cast<float>(gyro.y());  // q (pitch rate, rad/s)
        inputs.gyro_r = static_cast<float>(gyro.z());  // r (yaw rate, rad/s)
    }
}

// ============================================================
// T039: NNControllerBackend
// ============================================================

NNControllerBackend::NNControllerBackend(const NNGenome& genome)
    : genome_(genome) {
    // Allocate hidden-state buffer if any layer is recurrent. Zeros on construct.
    const int hs = nn_hidden_state_count(genome_.topology, genome_.recurrent);
    if (hs > 0) hidden_state_.assign(static_cast<size_t>(hs), 0.0f);
}

void NNControllerBackend::reset() {
    std::fill(hidden_state_.begin(), hidden_state_.end(), 0.0f);
}

void NNControllerBackend::enableTelemetryCapture() {
    telemetry_capture_enabled_ = true;
}

void NNControllerBackend::disableTelemetryCapture() {
    telemetry_capture_enabled_ = false;
}

void NNControllerBackend::resetTelemetry() {
    telemetry_.reset();
}

double NNControllerBackend::telemetryActivationRatio() const {
    return telemetry_.activation_ratio();
}

long long NNControllerBackend::telemetrySampleCount() const {
    return telemetry_.sample_count;
}

void NNControllerBackend::evaluate(AircraftState& aircraftState, PathProvider& pathProvider) {
    NNInputs inputs = {};
    gather_pathgen_inputs(pathProvider, aircraftState, inputs);

    float outputs[NN_OUTPUT_COUNT];  // raw-ok: NN-byte-format buffer (output of nn_forward, fp32 contract)
    if (hidden_state_.empty()) {
        nn_forward(genome_.weights.data(), genome_.topology,
                   reinterpret_cast<const float*>(&inputs), outputs);
    } else {
        RecurrentTelemetry* tlm = telemetry_capture_enabled_ ? &telemetry_ : nullptr;
        nn_forward_recurrent(genome_.weights.data(), genome_.topology,
                             genome_.recurrent,
                             reinterpret_cast<const float*>(&inputs), outputs,
                             hidden_state_.data(),
                             tlm);
    }

    // Set control commands: pitch, roll, throttle (already in [-1, 1] via tanh)
    aircraftState.setPitchCommand(static_cast<gp_scalar>(outputs[0]));
    aircraftState.setRollCommand(static_cast<gp_scalar>(outputs[1]));
    aircraftState.setThrottleCommand(static_cast<gp_scalar>(outputs[2]));

    // Capture actual NN I/O for diagnostics
    aircraftState.setNNData(inputs, outputs, NN_OUTPUT_COUNT);
}

// ============================================================
// 030 M6d — Tracker mode (FR-006 + FR-016 + FR-019)
// ============================================================
// Wrapped in #ifndef ARDUINO because xiao firmware cherry-picks this
// .cc but doesn't need tracker-mode dispatch (xiao = pathgen-only per
// FR-019 compile-time mode select). gather_tracker_inputs uses arena.h
// which keeps things cereal-free but transitively pulls in the
// FlightArena type definition; bodies are desktop-only.
#ifndef ARDUINO

void gather_tracker_inputs(const AircraftState& chase,
                           const TrackerHistoryWindow& history,
                           const autoc::eval::FlightArena& arena,
                           TrackerInputs& out) {
    // Beacon history: 6 slots per channel, copied as-is. Caller (TrackerStepper)
    // owns the ordering — index 0 = oldest (-0.5s), index 5 = "now".
    for (int i = 0; i < 6; ++i) {
        out.beacon_l_x[i]   = history.left_x[i];     // raw-ok: NN-byte-format primitive
        out.beacon_l_y[i]   = history.left_y[i];     // raw-ok: NN-byte-format primitive
        out.beacon_l_cep[i] = history.left_cep[i];   // raw-ok: NN-byte-format primitive
        out.beacon_r_x[i]   = history.right_x[i];    // raw-ok: NN-byte-format primitive
        out.beacon_r_y[i]   = history.right_y[i];    // raw-ok: NN-byte-format primitive
        out.beacon_r_cep[i] = history.right_cep[i];  // raw-ok: NN-byte-format primitive
    }

    // Aircraft attitude quaternion (w, x, y, z) — unit norm, components in [-1,1].
    {
        gp_quat q = chase.getOrientation();
        out.quat_w = static_cast<float>(q.w());
        out.quat_x = static_cast<float>(q.x());
        out.quat_y = static_cast<float>(q.y());
        out.quat_z = static_cast<float>(q.z());
    }

    // Airspeed (m/s, raw).
    out.airspeed = static_cast<float>(chase.getRelVel());

    // Body-frame angular rates (rad/s, standard aerospace RHR).
    {
        gp_vec3 gyro = chase.getGyroRates();
        out.gyro_p = static_cast<float>(gyro.x());
        out.gyro_q = static_cast<float>(gyro.y());
        out.gyro_r = static_cast<float>(gyro.z());
    }

    // 030 M7a — Arena-awareness input (FR-016 + Session 2026-05-07 Q1):
    // single ray-projection scalar shared with arena.h's per-tick OOB
    // termination check. Meters of safe forward flight along chase
    // velocity vector before ray intersects cylinder wall, floor, or
    // ceiling. NN learns "small number = trouble; turn-recovery direction
    // implicit from body attitude / quat / gyro" per Session 2026-05-07.
    out.dist_to_boundary_along_vel = static_cast<float>(   // raw-ok: NN-byte-format primitive
        autoc::eval::distanceToBoundary(chase.getPosition(),
                                         chase.getVelocity(),
                                         arena));
}

void NNControllerBackend::evaluateTracker(AircraftState& aircraftState,
                                          const TrackerInputs& inputs) {
    float outputs[NN_OUTPUT_COUNT];  // raw-ok: NN-byte-format buffer (output of nn_forward, fp32 contract)
    if (hidden_state_.empty()) {
        nn_forward(genome_.weights.data(), genome_.topology,
                   reinterpret_cast<const float*>(&inputs), outputs);
    } else {
        RecurrentTelemetry* tlm = telemetry_capture_enabled_ ? &telemetry_ : nullptr;
        nn_forward_recurrent(genome_.weights.data(), genome_.topology,
                             genome_.recurrent,
                             reinterpret_cast<const float*>(&inputs), outputs,
                             hidden_state_.data(),
                             tlm);
    }

    aircraftState.setPitchCommand(static_cast<gp_scalar>(outputs[0]));
    aircraftState.setRollCommand(static_cast<gp_scalar>(outputs[1]));
    aircraftState.setThrottleCommand(static_cast<gp_scalar>(outputs[2]));

    // setNNData(TrackerInputs) is deferred to M8 (cameraViewList in dmp v=2);
    // tracker-mode NN inputs aren't yet captured into the dmp output stream.
    // Honest-recording audit per memory:feedback_honest_dmp_recording lands
    // at the v=2 schema-bump boundary.
}

#endif  // ARDUINO — end of tracker-mode block (M6d/M7a)

// ============================================================
// Test helpers
// ============================================================

gp_scalar testFastTanh(gp_scalar x) {
    return fast_tanh(x);
}
