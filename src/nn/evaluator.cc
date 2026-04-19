#include "autoc/nn/evaluator.h"
#include "autoc/nn/topology.h"
#include "autoc/nn/nn_input_computation.h"
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
// T037: Xavier/Glorot initialization
// ============================================================

void nn_xavier_init(NNGenome& genome) {
    int total = nn_weight_count(genome.topology);
    genome.weights.resize(total);

    int idx = 0;
    for (size_t layer = 0; layer + 1 < genome.topology.size(); layer++) {
        int fan_in = genome.topology[layer];
        int fan_out = genome.topology[layer + 1];
        double stddev = std::sqrt(1.0 / fan_in);

        int num_weights = fan_in * fan_out;
        for (int i = 0; i < num_weights; i++) {
            genome.weights[idx++] = static_cast<float>(rng::randGaussian(stddev));
        }

        // Initialize biases to zero
        for (int i = 0; i < fan_out; i++) {
            genome.weights[idx++] = 0.0f;
        }
    }
}

// ============================================================
// T040: Gather NN_INPUT_COUNT sensor inputs — raw, no normalization
// ============================================================
// Layout (33 inputs — 023 direction cosines):
//  0- 5: target_x [-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  body-frame unit-vec x
//  6-11: target_y [-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  body-frame unit-vec y
// 12-17: target_z [-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  body-frame unit-vec z
// 18-23: dist     [-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  raw metres
//    24: dDist/dt closing rate (m/s, positive = approaching)
// 25-28: quaternion (w, x, y, z)                         [-1,1]
//    29: airspeed (m/s)
// 30-32: gyro rates (p, q, r) in rad/s                   standard aerospace RHR
//
// Past slots (n=9,3,1,0) use recorded aircraft history at those times.
// Forecast slots (+0.1s=offset 1, +0.5s=offset 5) use current aircraft
// position vs future rabbit path — curve-ahead estimation.

// History slot indices: [9, 3, 1, 0] = [-0.9s, -0.3s, -0.1s, now]
static const int HIST_PAST[] = {9, 3, 1, 0};
// Forecast offsets in path steps: +1 = +0.1s, +5 = +0.5s
static const float FORECAST_OFFSETS[] = {1.0f, 5.0f};

// Get path tangent (direction of travel) at the given odometer offset
static gp_vec3 getPathTangentAtOffset(PathProvider& pathProvider,
                                       gp_scalar currentOdometer,
                                       gp_scalar offsetMeters) {
    gp_vec3 pos = getInterpolatedTargetPosition(pathProvider, currentOdometer, offsetMeters);
    gp_vec3 posAhead = getInterpolatedTargetPosition(pathProvider, currentOdometer, offsetMeters + 0.5f);
    gp_vec3 tangent = posAhead - pos;
    double norm = tangent.norm();
    if (norm > 1e-6) return tangent / norm;
    return gp_vec3::UnitX();  // fallback
}

void nn_gather_inputs(PathProvider& pathProvider, AircraftState& aircraftState,
                      NNInputs& inputs) {
    gp_scalar rabbitOdo = aircraftState.getRabbitOdometer();
    gp_scalar rabbitSpeed = aircraftState.getRabbitSpeed();

    // Convert forecast offset steps to offset meters
    // FORECAST_OFFSETS are in steps (1 step = SIM_TIME_STEP_MSEC),
    // convert: offsetMeters = steps * (SIM_TIME_STEP_MSEC / 1000.0) * rabbitSpeed
    auto offsetStepsToMeters = [&](float steps) -> gp_scalar {
        return static_cast<gp_scalar>(steps) * (static_cast<gp_scalar>(SIM_TIME_STEP_MSEC) / 1000.0f) * rabbitSpeed;
    };

    // target_x/y/z[0-3]: past history (direction cosines from recorded history)
    for (int i = 0; i < 4; i++) {
        gp_vec3 dir = aircraftState.getHistoricalTargetDir(HIST_PAST[i]);
        inputs.target_x[i] = static_cast<float>(dir.x());
        inputs.target_y[i] = static_cast<float>(dir.y());
        inputs.target_z[i] = static_cast<float>(dir.z());
    }

    // target_x/y/z[4-5]: path-lookahead forecast (+0.1s, +0.5s)
    for (int i = 0; i < 2; i++) {
        gp_vec3 futureTarget = getInterpolatedTargetPosition(
            pathProvider, rabbitOdo, offsetStepsToMeters(FORECAST_OFFSETS[i]));
        gp_vec3 craftToTarget = futureTarget - aircraftState.getPosition();
        gp_vec3 target_local = aircraftState.getOrientation().inverse() * craftToTarget;
        float distance = static_cast<float>(target_local.norm());

        // Get path tangent for singularity fallback
        gp_vec3 pathTangent = getPathTangentAtOffset(pathProvider, rabbitOdo,
                                                      offsetStepsToMeters(FORECAST_OFFSETS[i]));
        gp_vec3 tangent_body = aircraftState.getOrientation().inverse() * pathTangent;
        float tangent_norm = static_cast<float>(tangent_body.norm());
        if (tangent_norm > 1e-6f) tangent_body = tangent_body / tangent_norm;

        gp_vec3 dir = computeTargetDir(target_local, distance, tangent_body);
        inputs.target_x[4 + i] = static_cast<float>(dir.x());
        inputs.target_y[4 + i] = static_cast<float>(dir.y());
        inputs.target_z[4 + i] = static_cast<float>(dir.z());
    }

    // dist[0-3]: past history (raw metres)
    for (int i = 0; i < 4; i++)
        inputs.dist[i] = static_cast<float>(aircraftState.getHistoricalDist(HIST_PAST[i]));

    // dist[4-5]: forecast — distance from current position to future rabbit (+0.1s, +0.5s)
    for (int i = 0; i < 2; i++) {
        gp_vec3 futureTarget = getInterpolatedTargetPosition(
            pathProvider, rabbitOdo, offsetStepsToMeters(FORECAST_OFFSETS[i]));
        inputs.dist[4 + i] = static_cast<float>(
            (futureTarget - aircraftState.getPosition()).norm());
    }

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
    : genome_(genome) {}

void NNControllerBackend::evaluate(AircraftState& aircraftState, PathProvider& pathProvider) {
    NNInputs inputs = {};
    nn_gather_inputs(pathProvider, aircraftState, inputs);

    float outputs[NN_OUTPUT_COUNT];
    nn_forward(genome_.weights.data(), genome_.topology,
               reinterpret_cast<const float*>(&inputs), outputs);

    // Set control commands: pitch, roll, throttle (already in [-1, 1] via tanh)
    aircraftState.setPitchCommand(static_cast<gp_scalar>(outputs[0]));
    aircraftState.setRollCommand(static_cast<gp_scalar>(outputs[1]));
    aircraftState.setThrottleCommand(static_cast<gp_scalar>(outputs[2]));

    // Capture actual NN I/O for diagnostics
    aircraftState.setNNData(inputs, outputs, NN_OUTPUT_COUNT);
}

// ============================================================
// Test helpers
// ============================================================

gp_scalar testFastTanh(gp_scalar x) {
    return fast_tanh(x);
}
