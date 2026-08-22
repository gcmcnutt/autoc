#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <getopt.h>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <algorithm>

#include "autoc/nn/serialization.h"
#include "autoc/nn/evaluator.h"
#include "autoc/eval/arena.h"   // 041 P2-3 — BakedArena defaults derive from FlightArena
#include "autoc/eval/cone_constants.h"  // 041 P5-3 — SCORE_GRAD_* needs the cone
#include "autoc/util/config.h"          // ...defaults derive from AutocConfig

void printUsage(const char* progName) {
    std::cout << "Usage: " << progName << " [OPTIONS]\n";
    std::cout << "Options:\n";
    std::cout << "  -i, --input FILE     Input NN weight file (required, NN01 format)\n";
    std::cout << "  -o, --output FILE    Output C++ source file (default: nn_program_generated.cpp)\n";
    std::cout << "  -f, --function NAME  Generated function name (default: generatedNNProgram)\n";
    std::cout << "  -u, --unrolled       Generate unrolled layer code (default: use nn_forward)\n";
    std::cout << "      --ini FILE       Config file the baked constants come from\n";
    std::cout << "                       (default: autoc.ini). The arena AND the tracking cone\n";
    std::cout << "                       are read from here and NOWHERE else -- use the SAME ini\n";
    std::cout << "                       the run used. There are deliberately no CLI overrides:\n";
    std::cout << "                       one source means the build log's provenance line is the\n";
    std::cout << "                       whole answer. (-i is the weight file for this tool, so\n";
    std::cout << "                       the config flag is spelled --ini.)\n";
    std::cout << "  --help               Show this help message\n";
    std::cout << "\n";
    std::cout << "Generates C++ source with embedded NN weights for desktop and embedded deployment.\n";
    std::cout << "The generated function has the same signature as generatedGPProgram().\n";
    std::cout << "\n";
    std::cout << "Examples:\n";
    std::cout << "  " << progName << " -i nn_weights.dat\n";
    std::cout << "  " << progName << " -i nn_weights.dat -o my_nn.cpp -f myNNFunction\n";
    std::cout << "  " << progName << " -i nn_weights.dat -u   # unrolled layer loops\n";
}

// 038 P0-D FR-P0H (B): pathgen NN inputs now include arena-awareness
// (dist_to_boundary + inward_body), so gather_pathgen_inputs needs a
// FlightArena. On the desktop the backend holds one from the ini; the xiao
// firmware has no ConfigManager, so nn2cpp bakes a compile-time literal from
// these values (default = FlightArena struct defaults, override with -a).
//
// 039 D5: the baked literal is the geometry TEMPLATE, not the placement.
// The generated code exposes it via generatedNNProgramArenaTemplate() and
// gathers against an EXTERN `nnActiveArena()` the consumer must define —
// the firmware re-centers per engage (resolveEngageArena), desktop harnesses
// construct one explicitly. No definition ⇒ link failure (Constitution VII:
// no silent fallback placement).
// ⛔ 041 P2-3 — DEFAULTS DERIVE FROM `FlightArena`, they are not restated.
// This struct used to carry its own literal 80 / 5 / 100 — a THIRD copy of the
// arena geometry, alongside `FlightArena`'s defaults and the .ini keys — and it
// was still at those numbers after the arena moved twice. A stale default here
// bakes the wrong containment template into flight firmware, and the aircraft
// would resolve its engage arena around a cylinder the policy was never trained
// in. Deriving costs nothing and cannot go stale.
struct BakedArena {
    double radius_m;
    double floor_agl_m;
    double ceiling_agl_m;
};

// 041 P5-3 — the tracking cone, baked for the same reason as the arena: the
// firmware computes SCORE_GRAD_* from the virtual target's geometry, and that
// closed form is only correct against the cone the objective was shaped with.
// ⛔ DEFAULTS DERIVE FROM `AutocConfig`, exactly as BakedArena derives from
// FlightArena, and for the identical reason — restating the six numbers here
// would make this a third copy that goes stale the next time the cone moves.
struct BakedCone {
    double distScaleBehind;
    double distScaleAhead;
    double coneAngleDeg;
    double streakThreshold;
    double streakRampSec;
    double streakMultiplierMax;
};

// Emit the cone constants the firmware needs for SCORE_GRAD_*.
static void emitConeContract(std::stringstream& code, const BakedCone& c) {
    code << "// 041 P5-3 — tracking-cone constants from codegen (-c B,A,D,T,R,M).\n";
    code << "// SCORE_GRAD_* is computed against THESE; they must match the run that\n";
    code << "// produced the weights above, or the gradient points somewhere training\n";
    code << "// never rewarded.\n";
    code << "const autoc::eval::ConeConstants& generatedNNProgramConeConstants() {\n";
    code << "    static const autoc::eval::ConeConstants c{"
         << std::fixed << std::setprecision(6)
         << c.distScaleBehind << ", " << c.distScaleAhead << ", "
         << c.coneAngleDeg << ", " << c.streakThreshold << ", "
         << c.streakRampSec << ", " << c.streakMultiplierMax << "};\n";
    code << "    return c;\n";
    code << "}\n\n";
}

// Emit the template accessor + the active-arena extern shared by both
// codegen paths (039 D5).
static void emitArenaContract(std::stringstream& code, const BakedArena& a) {
    code << "// 039 D5 — arena geometry TEMPLATE from codegen (-a R,F,C). Placement is\n";
    code << "// engage-scoped: the consumer owns the active arena (see nnActiveArena).\n";
    code << "const autoc::eval::FlightArena& generatedNNProgramArenaTemplate() {\n";
    code << "    static const autoc::eval::FlightArena t{"
         << std::fixed << std::setprecision(4)
         << a.radius_m << "f, " << a.floor_agl_m << "f, " << a.ceiling_agl_m << "f};\n";
    code << "    return t;\n";
    code << "}\n\n";
    code << "// Engage-scoped active arena — DEFINED BY THE CONSUMER (msplink.cpp on the\n";
    code << "// xiao, the test harness on desktop). Missing definition fails the link.\n";
    code << "extern const autoc::eval::FlightArena& nnActiveArena();\n\n";
}

// ============================================================
// 039 — compact SHA-256 (FIPS 180-4) for weight/firmware ids.
// Baked into the generated code so the flight log can tie a log
// file to the exact firmware + weight set (data-model.md §2.1).
// ============================================================
namespace sha256impl {
static const uint32_t K[64] = {
    0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
    0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
    0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
    0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
    0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
    0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
    0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
    0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
static inline uint32_t rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }
}  // namespace sha256impl

// SHA-256 of `data`; writes 32-byte digest to `out`.
static void sha256(const uint8_t* data, size_t len, uint8_t out[32]) {
    using namespace sha256impl;
    uint32_t h[8] = {0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,
                     0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
    // Padded message: len + 1 + zeros + 8-byte big-endian bit length
    size_t padded = ((len + 8) / 64 + 1) * 64;
    std::vector<uint8_t> msg(padded, 0);
    if (len > 0) std::copy(data, data + len, msg.begin());
    msg[len] = 0x80;
    uint64_t bits = static_cast<uint64_t>(len) * 8;
    for (int i = 0; i < 8; i++) msg[padded - 1 - i] = static_cast<uint8_t>(bits >> (8 * i));

    for (size_t off = 0; off < padded; off += 64) {
        uint32_t w[64];
        for (int i = 0; i < 16; i++) {
            w[i] = (static_cast<uint32_t>(msg[off + 4*i]) << 24) |
                   (static_cast<uint32_t>(msg[off + 4*i + 1]) << 16) |
                   (static_cast<uint32_t>(msg[off + 4*i + 2]) << 8) |
                   static_cast<uint32_t>(msg[off + 4*i + 3]);
        }
        for (int i = 16; i < 64; i++) {
            uint32_t s0 = rotr(w[i-15], 7) ^ rotr(w[i-15], 18) ^ (w[i-15] >> 3);
            uint32_t s1 = rotr(w[i-2], 17) ^ rotr(w[i-2], 19) ^ (w[i-2] >> 10);
            w[i] = w[i-16] + s0 + w[i-7] + s1;
        }
        uint32_t a=h[0],b=h[1],c=h[2],d=h[3],e=h[4],f=h[5],g=h[6],hh=h[7];
        for (int i = 0; i < 64; i++) {
            uint32_t S1 = rotr(e,6) ^ rotr(e,11) ^ rotr(e,25);
            uint32_t ch = (e & f) ^ (~e & g);
            uint32_t t1 = hh + S1 + ch + K[i] + w[i];
            uint32_t S0 = rotr(a,2) ^ rotr(a,13) ^ rotr(a,22);
            uint32_t mj = (a & b) ^ (a & c) ^ (b & c);
            uint32_t t2 = S0 + mj;
            hh=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
        }
        h[0]+=a; h[1]+=b; h[2]+=c; h[3]+=d; h[4]+=e; h[5]+=f; h[6]+=g; h[7]+=hh;
    }
    for (int i = 0; i < 8; i++) {
        out[4*i]   = static_cast<uint8_t>(h[i] >> 24);
        out[4*i+1] = static_cast<uint8_t>(h[i] >> 16);
        out[4*i+2] = static_cast<uint8_t>(h[i] >> 8);
        out[4*i+3] = static_cast<uint8_t>(h[i]);
    }
}

// Emit the identity/metadata trailer appended to BOTH emissions (039):
//   - generatedNNWeightId[8]:   first 8 bytes of SHA-256 of the weight file
//   - generatedNNFirmwareId[8]: first 8 bytes of SHA-256 of the generated
//     code text ABOVE this trailer (the trailer can't hash itself)
//   - topology string + counts for the boot banner (039 US1)
static void emitIdentityTrailer(std::stringstream& code, const NNGenome& genome,
                                const std::vector<uint8_t>& weightFileBytes) {
    uint8_t wid[32];
    sha256(weightFileBytes.data(), weightFileBytes.size(), wid);
    const std::string bodySoFar = code.str();
    uint8_t fid[32];
    sha256(reinterpret_cast<const uint8_t*>(bodySoFar.data()), bodySoFar.size(), fid);

    std::ostringstream topo;
    for (size_t i = 0; i < genome.topology.size(); i++) {
        if (i > 0) topo << "->";
        topo << genome.topology[i];
        if (i < genome.recurrent.size() && genome.recurrent[i]) topo << "r";
    }

    code << "// 039 — identity trailer. firmware_id hashes the code text above this\n";
    code << "// line (regenerating with identical inputs+tool reproduces it modulo the\n";
    code << "// timestamp comment); weight_id hashes the input weight file bytes.\n";
    auto emitId = [&code](const char* name, const uint8_t* id) {
        code << "const uint8_t " << name << "[8] = {";
        for (int i = 0; i < 8; i++) {
            if (i > 0) code << ", ";
            char b[8];
            std::snprintf(b, sizeof(b), "0x%02x", id[i]);
            code << b;
        }
        code << "};\n";
    };
    emitId("generatedNNWeightId", wid);
    emitId("generatedNNFirmwareId", fid);
    code << "const char* generatedNNTopologyString = \"" << topo.str() << "\";\n";
    code << "const int generatedNNInputCount = " << genome.topology.front() << ";\n";
    code << "const int generatedNNWeightCount = " << genome.weights.size() << ";\n";
}

// Generate code that uses the portable nn_forward() function with static weight array
std::string generatePortableCode(const NNGenome& genome, const std::string& functionName,
                                  const std::string& sourceFile, const BakedArena& arena, const BakedCone& cone,
                                  const std::vector<uint8_t>& weightFileBytes) {
    std::stringstream code;

    auto now = std::chrono::system_clock::now();
    auto now_t = std::chrono::system_clock::to_time_t(now);
    char timebuf[64];
    std::strftime(timebuf, sizeof(timebuf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&now_t));

    code << "// Auto-generated NN evaluator function\n";
    code << "// Generated: " << timebuf << "\n";
    code << "//\n";
    code << "// Source NN Information:\n";
    code << "//   Weight file: " << sourceFile << "\n";
    code << "//   Source:      " << genome.source << "\n";
    code << "//   Topology:    ";
    for (size_t i = 0; i < genome.topology.size(); i++) {
        if (i > 0) code << " -> ";
        code << genome.topology[i];
        if (i < genome.recurrent.size() && genome.recurrent[i]) code << "r";
    }
    code << "\n";
    code << "//   Weights:     " << genome.weights.size() << "\n";
    code << "//   Fitness:     " << std::fixed << std::setprecision(6) << genome.fitness << "\n";
    code << "//   Sigma:       " << std::fixed << std::setprecision(6) << genome.mutation_sigma << "\n";
    code << "//\n";
    code << "#include \"nn_program.h\"\n\n";

    // Topology
    code << "static const int nn_topology[] = {";
    for (size_t i = 0; i < genome.topology.size(); i++) {
        if (i > 0) code << ", ";
        code << genome.topology[i];
    }
    code << "};\n";
    code << "static const int nn_num_layers = " << genome.topology.size() << ";\n\n";

    // 041 P2-2 — FAIL-LOUD LAYOUT GUARD, emitted into the generated file.
    //
    // ⛔ This is why the generated file is safe to fly. `gather_pathgen_inputs`
    // writes `NN_INPUT_COUNT` floats into an `NNInputs`; the forward pass below
    // multiplies them against a weight array baked HERE, at codegen time. If the
    // two disagree — a firmware built against a stale generated file, which is
    // exactly the state this tree was in (37 baked vs 45 compiled) — the
    // forward pass reads past the end of `nn_weights` and flies on whatever is
    // next in flash. Nothing in the build said so, and nothing at runtime would:
    // the aircraft would simply fly badly, in the air, once.
    //
    // The assert lives in the GENERATED artifact rather than in nn2cpp because
    // the mismatch is between that artifact and whatever firmware tree it is
    // later compiled into — which nn2cpp cannot see.
    code << "// 041 P2-2 fail-loud layout guard — see tools/nn2cpp.cc for why.\n";
    code << "constexpr int kGeneratedNNInputCount = " << genome.topology.front() << ";\n";
    code << "static_assert(kGeneratedNNInputCount == NN_INPUT_COUNT,\n";
    code << "    \"Generated NN input count != the compiled NNInputs layout. This file \"\n";
    code << "    \"was generated against a different nn_inputs.h, so the forward pass \"\n";
    code << "    \"would read PAST THE END of nn_weights and fly on garbage. \"\n";
    code << "    \"REGENERATE with tools/nn2cpp -- never relax this.\");\n";
    // ⚠️ There is deliberately NO companion assert on the weight count against
    // NN_WEIGHT_COUNT. That would assert "this genome has the project's current
    // standard hidden widths", which is a different and weaker claim — it would
    // reject a legitimately narrower experimental genome while catching nothing
    // the input assert misses. The generated file's topology array and its
    // weight array are sized from each other by construction here, so the only
    // way they can disagree with the FIRMWARE is at the input boundary, which
    // is exactly what the assert above covers.
    code << "\n";


    // Recurrent flags (spec 027). Emitted even when all-false so xiao build
    // can switch on compile-time feedforward-only simpler path if wanted.
    const bool any_recurrent = std::any_of(genome.recurrent.begin(),
        genome.recurrent.end(), [](uint8_t v) { return v != 0; });
    code << "static const bool nn_recurrent[] = {";
    for (size_t i = 0; i < genome.topology.size(); i++) {
        if (i > 0) code << ", ";
        code << ((i < genome.recurrent.size() && genome.recurrent[i]) ? "true" : "false");
    }
    code << "};\n";

    int hidden_total = 0;
    for (size_t i = 0; i < genome.topology.size() && i < genome.recurrent.size(); i++) {
        if (genome.recurrent[i]) hidden_total += genome.topology[i];
    }
    code << "static float nn_hidden_state[" << (hidden_total > 0 ? hidden_total : 1) << "] = {0};\n";
    code << "static const int nn_hidden_state_size = " << hidden_total << ";\n\n";

    // Weights — formatted 8 per line for readability
    code << "static const float nn_weights[" << genome.weights.size() << "] = {\n";
    for (size_t i = 0; i < genome.weights.size(); i++) {
        if (i % 8 == 0) code << "    ";
        code << std::scientific << std::setprecision(8) << genome.weights[i] << "f";
        if (i < genome.weights.size() - 1) code << ",";
        if (i % 8 == 7 || i == genome.weights.size() - 1) code << "\n";
        else code << " ";
    }
    code << "};\n\n";

    // Topology vectors (needed by nn_forward / nn_forward_recurrent)
    code << "static const std::vector<int>& getTopology() {\n";
    code << "    static const std::vector<int> t(nn_topology, nn_topology + nn_num_layers);\n";
    code << "    return t;\n";
    code << "}\n";
    code << "static const std::vector<uint8_t>& getRecurrent() {\n";
    code << "    static const std::vector<uint8_t> r = [] {\n";
    code << "        std::vector<uint8_t> v(nn_num_layers);\n";
    code << "        for (int i = 0; i < nn_num_layers; i++) v[i] = nn_recurrent[i] ? 1 : 0;\n";
    code << "        return v;\n";
    code << "    }();\n";
    code << "    return r;\n";
    code << "}\n\n";

    // Reset function (spec 027 Q4): zero hidden state. Call on span start.
    code << "void generatedNNProgramReset() {\n";
    code << "    for (int i = 0; i < nn_hidden_state_size; i++) nn_hidden_state[i] = 0.0f;\n";
    code << "}\n\n";

    emitArenaContract(code, arena);
    emitConeContract(code, cone);

    // Main function
    code << "gp_scalar " << functionName << "(PathProvider& pathProvider, AircraftState& aircraftState, gp_scalar arg) {\n";
    code << "    NNInputs inputs = {};\n";
    code << "    gather_pathgen_inputs(pathProvider, aircraftState, nnActiveArena(), inputs);\n\n";
    code << "    float outputs[" << genome.topology.back() << "];\n";
    if (any_recurrent) {
        code << "    nn_forward_recurrent(nn_weights, getTopology(), getRecurrent(),\n";
        code << "                         reinterpret_cast<const float*>(&inputs), outputs, nn_hidden_state);\n\n";
    } else {
        code << "    nn_forward(nn_weights, getTopology(), reinterpret_cast<const float*>(&inputs), outputs);\n\n";
    }
    code << "    // Set control commands: pitch, roll, throttle (already [-1,1] via tanh)\n";
    code << "    aircraftState.setPitchCommand(static_cast<gp_scalar>(outputs[0]));\n";
    code << "    aircraftState.setRollCommand(static_cast<gp_scalar>(outputs[1]));\n";
    code << "    aircraftState.setThrottleCommand(static_cast<gp_scalar>(outputs[2]));\n\n";
    code << "    // Capture I/O for telemetry logging\n";
    code << "    aircraftState.setNNData(inputs, outputs, " << genome.topology.back() << ");\n\n";
    code << "    return static_cast<gp_scalar>(outputs[0]); // return pitch for compatibility\n";
    code << "}\n\n";

    // Source identifier
    code << "const char* generatedNNProgramSource = \"" << genome.source << "\";\n";

    emitIdentityTrailer(code, genome, weightFileBytes);

    return code.str();
}

// Generate code with unrolled layer loops — no std::vector, no nn_forward /
// nn_forward_recurrent call. 039 D6: recurrent layers unroll with persistent
// static hidden-state registers (h read during the layer MACs, committed
// after the full layer — double-buffered via the out buffer, so no
// read-after-write). Accumulation order matches nn_forward_recurrent
// exactly: sum = (B[j] + Σ W·x) + (Σ W_hh·h), tanh via fast_tanh — the
// desktop equivalence test asserts bit-identical outputs.
std::string generateUnrolledCode(const NNGenome& genome, const std::string& functionName,
                                  const std::string& sourceFile, const BakedArena& arena, const BakedCone& cone,
                                  const std::vector<uint8_t>& weightFileBytes) {
    std::stringstream code;

    auto now = std::chrono::system_clock::now();
    auto now_t = std::chrono::system_clock::to_time_t(now);
    char timebuf[64];
    std::strftime(timebuf, sizeof(timebuf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&now_t));

    code << "// Auto-generated NN evaluator function (unrolled)\n";
    code << "// Generated: " << timebuf << "\n";
    code << "//\n";
    code << "// Source NN Information:\n";
    code << "//   Weight file: " << sourceFile << "\n";
    code << "//   Source:      " << genome.source << "\n";
    code << "//   Topology:    ";
    for (size_t i = 0; i < genome.topology.size(); i++) {
        if (i > 0) code << " -> ";
        code << genome.topology[i];
        if (i < genome.recurrent.size() && genome.recurrent[i]) code << "r";
    }
    code << "\n";
    code << "//   Weights:     " << genome.weights.size() << "\n";
    code << "//   Fitness:     " << std::fixed << std::setprecision(6) << genome.fitness << "\n";
    code << "//   Sigma:       " << std::fixed << std::setprecision(6) << genome.mutation_sigma << "\n";
    code << "//\n";
    code << "#include \"nn_program.h\"\n\n";

    // Weights
    code << "static const float nn_weights[" << genome.weights.size() << "] = {\n";
    for (size_t i = 0; i < genome.weights.size(); i++) {
        if (i % 8 == 0) code << "    ";
        code << std::scientific << std::setprecision(8) << genome.weights[i] << "f";
        if (i < genome.weights.size() - 1) code << ",";
        if (i % 8 == 7 || i == genome.weights.size() - 1) code << "\n";
        else code << " ";
    }
    code << "};\n\n";

    // 041 P2-2 — FAIL-LOUD LAYOUT GUARD (see the same block in
    // generateNNProgramCode above, and tools/nn2cpp.cc's rationale there).
    // ⛔ THIS is the path the xiao actually uses: the unrolled generator. The
    // tree shipped 37 baked inputs against 45 compiled with nothing objecting,
    // which would have had the flight forward pass read past the end of
    // nn_weights. A compile error naming both numbers costs nothing.
    code << "// 041 P2-2 fail-loud layout guard — see tools/nn2cpp.cc for why.\n";
    code << "constexpr int kGeneratedNNInputCount = " << genome.topology.front() << ";\n";
    code << "static_assert(kGeneratedNNInputCount == NN_INPUT_COUNT,\n";
    code << "    \"Generated NN input count != the compiled NNInputs layout. This file \"\n";
    code << "    \"was generated against a different nn_inputs.h, so the unrolled \"\n";
    code << "    \"forward pass would read PAST THE END of nn_weights and fly on \"\n";
    code << "    \"garbage. REGENERATE with tools/nn2cpp -- never relax this.\");\n";
    // (No companion weight-count assert — see the rationale at the same guard in
    // generateNNProgramCode above.)
    code << "\n";

    // 039 D6 — persistent recurrent state registers, one static array per
    // recurrent layer, surviving between calls. W_hh blocks live at the tail
    // of nn_weights (NNGenome layout), in layer-index order.
    int ff_total = 0;
    for (size_t i = 0; i + 1 < genome.topology.size(); i++) {
        ff_total += genome.topology[i] * genome.topology[i + 1] + genome.topology[i + 1];
    }
    bool any_recurrent = false;
    for (size_t l = 0; l < genome.topology.size() && l < genome.recurrent.size(); l++) {
        if (genome.recurrent[l]) {
            any_recurrent = true;
            code << "// h state for recurrent layer " << l << " (size "
                 << genome.topology[l] << ")\n";
            code << "static float nn_h_l" << l << "[" << genome.topology[l] << "] = {0};\n";
        }
    }
    if (any_recurrent) code << "\n";

    // Reset hook: zero all recurrent state. Call at engage/span start
    // (nn_reset semantics — same contract as the table-driven emission).
    code << "void generatedNNProgramReset() {\n";
    for (size_t l = 0; l < genome.topology.size() && l < genome.recurrent.size(); l++) {
        if (genome.recurrent[l]) {
            code << "    for (int i = 0; i < " << genome.topology[l]
                 << "; i++) nn_h_l" << l << "[i] = 0.0f;\n";
        }
    }
    code << "}\n\n";

    emitArenaContract(code, arena);
    emitConeContract(code, cone);

    // Main function with unrolled loops
    int max_layer = *std::max_element(genome.topology.begin(), genome.topology.end());
    code << "gp_scalar " << functionName << "(PathProvider& pathProvider, AircraftState& aircraftState, gp_scalar arg) {\n";
    code << "    NNInputs inputs = {};\n";
    code << "    gather_pathgen_inputs(pathProvider, aircraftState, nnActiveArena(), inputs);\n";
    code << "    const float* input_floats = reinterpret_cast<const float*>(&inputs);\n\n";

    // Layer buffers — fixed-size on stack
    code << "    float buf_a[" << max_layer << "], buf_b[" << max_layer << "];\n\n";

    // Copy inputs to buf_a
    code << "    // Copy inputs\n";
    code << "    for (int i = 0; i < " << genome.topology.front() << "; i++) buf_a[i] = input_floats[i];\n\n";

    // Unrolled layer computation. Recurrent layers (039 D6): the out buffer
    // doubles as the h_new staging area — h is READ throughout the j loop and
    // committed only after the full layer, exactly nn_forward_recurrent's
    // ordering (no read-after-write on h within a tick).
    int weight_offset = 0;
    int whh_offset = 0;  // running offset into the W_hh tail (layer-index order)
    bool a_is_input = true;
    for (size_t layer = 0; layer + 1 < genome.topology.size(); layer++) {
        int in_size = genome.topology[layer];
        int out_size = genome.topology[layer + 1];
        const bool out_is_recurrent = (layer + 1 < genome.recurrent.size())
                                      && genome.recurrent[layer + 1];
        std::string in_buf = a_is_input ? "buf_a" : "buf_b";
        std::string out_buf = a_is_input ? "buf_b" : "buf_a";
        std::string h_arr = "nn_h_l" + std::to_string(layer + 1);

        code << "    // Layer " << layer << ": " << in_size << " -> " << out_size
             << (out_is_recurrent ? " (recurrent)" : "") << "\n";
        code << "    {\n";
        code << "        const float* W = nn_weights + " << weight_offset << ";\n";
        code << "        const float* B = nn_weights + " << (weight_offset + in_size * out_size) << ";\n";
        if (out_is_recurrent) {
            code << "        const float* Whh = nn_weights + " << (ff_total + whh_offset) << ";\n";
        }
        code << "        for (int j = 0; j < " << out_size << "; j++) {\n";
        code << "            float sum = B[j];\n";
        code << "            for (int i = 0; i < " << in_size << "; i++) {\n";
        code << "                sum += W[j * " << in_size << " + i] * " << in_buf << "[i];\n";
        code << "            }\n";
        if (out_is_recurrent) {
            code << "            float hh = 0.0f;\n";
            code << "            for (int i = 0; i < " << out_size << "; i++) {\n";
            code << "                hh += Whh[j * " << out_size << " + i] * " << h_arr << "[i];\n";
            code << "            }\n";
            code << "            sum = sum + hh;\n";
        }
        code << "            " << out_buf << "[j] = static_cast<float>(fast_tanh(static_cast<gp_scalar>(sum)));\n";
        code << "        }\n";
        if (out_is_recurrent) {
            code << "        // Commit h_t (after the full layer — double-buffered)\n";
            code << "        for (int j = 0; j < " << out_size << "; j++) "
                 << h_arr << "[j] = " << out_buf << "[j];\n";
        }
        code << "    }\n\n";

        weight_offset += in_size * out_size + out_size;
        if (out_is_recurrent) whh_offset += out_size * out_size;
        a_is_input = !a_is_input;
    }

    // After the loop, a_is_input tracks which buffer has the final result.
    // a_is_input flips each layer: for odd layer count result is in buf_b, even in buf_a.
    std::string result_buf = a_is_input ? "buf_a" : "buf_b";

    code << "    // Set control commands from " << result_buf << "\n";
    code << "    aircraftState.setPitchCommand(static_cast<gp_scalar>(" << result_buf << "[0]));\n";
    code << "    aircraftState.setRollCommand(static_cast<gp_scalar>(" << result_buf << "[1]));\n";
    code << "    aircraftState.setThrottleCommand(static_cast<gp_scalar>(" << result_buf << "[2]));\n\n";
    code << "    // Capture I/O for telemetry logging\n";
    code << "    aircraftState.setNNData(inputs, " << result_buf << ", 3);\n\n";
    code << "    return static_cast<gp_scalar>(" << result_buf << "[0]); // return pitch for compatibility\n";
    code << "}\n\n";

    // Source identifier
    code << "const char* generatedNNProgramSource = \"" << genome.source << "\";\n";

    emitIdentityTrailer(code, genome, weightFileBytes);

    return code.str();
}

int main(int argc, char** argv) {
    static struct option long_options[] = {
        {"input", required_argument, 0, 'i'},
        {"output", required_argument, 0, 'o'},
        {"function", required_argument, 0, 'f'},
        {"unrolled", no_argument, 0, 'u'},
        {"ini", required_argument, 0, 1001},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };

    std::string inputFile;
    std::string outputFile = "nn_program_generated.cpp";
    std::string functionName = "generatedNNProgram";
    bool unrolled = false;
    // Both are filled from the ini below -- no initializers, so a missed
    // assignment is a compile-time complaint rather than a plausible default.
    BakedArena arena{};
    BakedCone cone{};
    std::string iniFile = "autoc.ini";  // the ONE source for both
    int option_index = 0;
    int c;

    while ((c = getopt_long(argc, argv, "i:o:f:uh", long_options, &option_index)) != -1) {
        switch (c) {
            case 'i': inputFile = optarg; break;
            case 'o': outputFile = optarg; break;
            case 'f': functionName = optarg; break;
            case 'u': unrolled = true; break;
            case 1001: iniFile = optarg; break;
            case 'h': printUsage(argv[0]); return 0;
            case '?': printUsage(argv[0]); return 1;
            default: break;
        }
    }

    // 041 P5-3 — provenance for the baked constants, resolved and REPORTED.
    //
    // Order: --ini (the run's own config) < -c/-a (explicit override). Without
    // --ini the struct defaults are baked, which is right only when the run did
    // not override them — and that is precisely the case nobody can see. So the
    // source is printed either way: a wrong cone flies a policy against a
    // gradient its training never rewarded, and the failure is silent.
    // 041 P5-3 — the baked constants come from the ini and nowhere else.
    // ConfigManager exits(1) on a missing file, so there is no silent-default
    // path: either the constants are the run's, or nn2cpp does not run.
    ConfigManager::initialize(iniFile, std::cerr);
    {
        const AutocConfig& cfg = ConfigManager::getConfig();
        arena.radius_m      = cfg.flightArenaRadius;
        arena.floor_agl_m   = cfg.flightArenaFloorAGL;
        arena.ceiling_agl_m = cfg.flightArenaCeilingAGL;
        cone.distScaleBehind     = cfg.fitDistScaleBehind;
        cone.distScaleAhead      = cfg.fitDistScaleAhead;
        cone.coneAngleDeg        = cfg.fitConeAngleDeg;
        cone.streakThreshold     = cfg.fitStreakThreshold;
        cone.streakRampSec       = cfg.fitStreakRampSec;
        cone.streakMultiplierMax = cfg.fitStreakMultiplierMax;
    }

    if (inputFile.empty()) {
        std::cerr << "Error: Input file required (-i)" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    // Read NN weight file
    std::ifstream file(inputFile, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open input file: " << inputFile << std::endl;
        return 1;
    }

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<uint8_t> data(size);
    if (!file.read(reinterpret_cast<char*>(data.data()), size)) {
        std::cerr << "Error reading input file" << std::endl;
        return 1;
    }
    file.close();

    // Verify NN01 format
    if (!nn_detect_format(data.data(), data.size())) {
        std::cerr << "Error: Input file is not in NN01 format" << std::endl;
        return 1;
    }

    // Deserialize
    NNGenome genome;
    if (!nn_deserialize(data.data(), data.size(), genome)) {
        std::cerr << "Error deserializing NN genome" << std::endl;
        return 1;
    }

    // Generate code. 039 D6: -u covers recurrent genomes too (persistent
    // static state registers + straight-line MACs) — the pre-039 table-driven
    // fallback is gone.
    std::string code;
    if (unrolled) {
        code = generateUnrolledCode(genome, functionName, inputFile, arena, cone, data);
    } else {
        code = generatePortableCode(genome, functionName, inputFile, arena, cone, data);
    }

    // Write output
    std::ofstream outFile(outputFile);
    if (!outFile.is_open()) {
        std::cerr << "Error: Cannot create output file: " << outputFile << std::endl;
        return 1;
    }
    outFile << code;
    outFile.close();

    std::cout << "Generated " << (unrolled ? "unrolled" : "portable") << " NN evaluator: " << outputFile << std::endl;
    std::cout << "  Topology:   ";
    for (size_t i = 0; i < genome.topology.size(); i++) {
        if (i > 0) std::cout << " -> ";
        std::cout << genome.topology[i];
    }
    std::cout << std::endl;
    std::cout << "  Weights:    " << genome.weights.size() << std::endl;
    std::cout << "  Function:   " << functionName << "()" << std::endl;
    std::cout << "  Fitness:    " << std::fixed << std::setprecision(6) << genome.fitness << std::endl;
    // Provenance — printed ALWAYS, because "which cone was this flown against?"
    // must be answerable from the build log alone.
    std::cout << "  Arena:      " << std::setprecision(1) << arena.radius_m << ","
              << arena.floor_agl_m << "," << arena.ceiling_agl_m
              << "  [from " << iniFile << "]" << std::endl;
    std::cout << "  Cone:       " << std::setprecision(3)
              << cone.distScaleBehind << "," << cone.distScaleAhead << ","
              << cone.coneAngleDeg << "," << cone.streakThreshold << ","
              << cone.streakRampSec << "," << cone.streakMultiplierMax
              << "  [from " << iniFile << "]" << std::endl;

    return 0;
}
