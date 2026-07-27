// 039 T003/T004 — nn2cpp unrolled-recurrent emission equivalence (D6).
//
// Drives the REAL nn2cpp binary over a fixture recurrent genome and checks:
//   1. `-u` on a recurrent genome emits genuinely UNROLLED code (no
//      table-driven fallback — the pre-039 behavior at nn2cpp.cc:356-361).
//   2. The unrolled and table-driven emissions produce BIT-IDENTICAL
//      per-tick outputs across a multi-tick sequence including an
//      nn_reset() (generatedNNProgramReset) mid-sequence — same
//      accumulation order as nn_forward_recurrent ⇒ bit-comparable on
//      desktop (no -ffast-math in the harness).
//
// Method: each generated file is compiled (g++ subprocess) into its own
// harness executable together with the same closure the xiao build uses
// (src/nn/evaluator.cc + src/eval/sensor_math.cc + src/eval/arena.cc);
// both harnesses run an identical deterministic input sequence and print
// the raw float bit patterns of the 3 outputs per tick. Separate
// executables per emission avoid the generatedNNProgramReset /
// generatedNNProgramSource symbol collision.
//
// Paths are injected by CMake: NN2CPP_BIN, AUTOC_SOURCE_DIR,
// AUTOC_EIGEN_INC, AUTOC_CEREAL_INC.

#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "autoc/nn/evaluator.h"
#include "autoc/nn/serialization.h"

namespace {

std::string readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

void writeFile(const std::string& path, const std::string& content) {
    std::ofstream f(path, std::ios::binary);
    f << content;
    ASSERT_TRUE(f.good()) << "failed to write " << path;
}

// Run a command, capture combined stdout+stderr, return exit status.
int runCmd(const std::string& cmd, std::string& output) {
    const std::string full = cmd + " 2>&1";
    FILE* pipe = popen(full.c_str(), "r");
    if (!pipe) return -1;
    char buf[4096];
    output.clear();
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), pipe)) > 0) {
        output.append(buf, n);
    }
    return pclose(pipe);
}

// Fixture genome: 37 → 8 → 6r → 3, structurally the elite's shape
// (input → hidden → recurrent → output) but small enough to keep the
// harness compiles fast. Deterministic LCG weights.
NNGenome makeFixtureGenome() {
    NNGenome g;
    g.topology = {37, 8, 6, 3};
    g.recurrent = {0, 0, 1, 0};
    const int total = nn_weight_count(g.topology, g.recurrent);
    g.weights.resize(total);
    uint32_t lcg = 0x0390BEEFu;
    for (int i = 0; i < total; i++) {
        lcg = lcg * 1664525u + 1013904223u;
        // [-1, 1] with enough spread to keep tanh out of saturation rails
        g.weights[i] = (static_cast<float>((lcg >> 8) & 0xFFFF) / 65535.0f) * 2.0f - 1.0f;
    }
    g.fitness = -1.0;
    g.generation = 1;
    g.source = "fixture:nn2cpp_unroll_tests";
    return g;
}

// Deterministic multi-tick harness — one source, compiled once per
// generated emission. Prints raw output bit patterns per tick.
const char* kHarnessSource = R"HARNESS(
#include "nn_program.h"
#include <cstdio>
#include <cstring>

gp_scalar nnGenerated(PathProvider&, AircraftState&, gp_scalar);
void generatedNNProgramReset();

// 039 D5 — the consumer owns the engage-scoped arena (link-time contract).
static const autoc::eval::FlightArena g_arena{80.0f, 5.0f, 100.0f};
const autoc::eval::FlightArena& nnActiveArena() { return g_arena; }

static uint32_t lcg = 0x039C0DE5u;
static float frand() {
    lcg = lcg * 1664525u + 1013904223u;
    return (static_cast<float>((lcg >> 8) & 0xFFFF) / 65535.0f) * 2.0f - 1.0f;
}

int main() {
    Path p(gp_vec3(0.0f, 0.0f, 0.0f), gp_vec3(1.0f, 0.0f, 0.0f), 0.0f, 0.0f);
    SinglePathProvider provider(p);
    AircraftState st;
    st.setPosition(gp_vec3(10.0f, -5.0f, -30.0f));
    gp_quat q0(0.9f, 0.1f, -0.2f, 0.3f);
    q0.normalize();
    st.setOrientation(q0);
    st.setVelocity(gp_vec3(18.0f, 1.0f, -0.5f));
    st.setRelVel(18.0f);
    st.setGyroRates(gp_vec3(0.1f, -0.2f, 0.05f));
    st.resetHistory(gp_vec3(30.0f, 0.0f, -35.0f), gp_vec3(1.0f, 0.0f, 0.0f));

    generatedNNProgramReset();
    for (int t = 0; t < 24; ++t) {
        if (t == 12) generatedNNProgramReset();  // mid-sequence engage reset
        gp_vec3 dir(frand(), frand(), frand());
        float n = dir.norm();
        if (n > 1e-3f) dir = dir / n;
        float dist = 20.0f + 15.0f * frand();
        st.recordErrorHistory(dir, dist, static_cast<unsigned long>(t) * 50u);
        st.setGyroRates(gp_vec3(frand(), frand(), frand()));
        st.setRelVel(15.0f + 5.0f * frand());
        gp_quat q(1.0f + 0.1f * frand(), 0.2f * frand(), 0.2f * frand(), 0.2f * frand());
        q.normalize();
        st.setOrientation(q);
        st.setPosition(gp_vec3(50.0f * frand(), 50.0f * frand(), -30.0f + 10.0f * frand()));
        st.setVelocity(gp_vec3(20.0f * frand(), 20.0f * frand(), 5.0f * frand()));
        nnGenerated(provider, st, 0.0f);
        const float* out = st.getNNOutputs();
        uint32_t bits[3];
        memcpy(bits, out, sizeof(bits));
        printf("%d %08x %08x %08x\n", t, bits[0], bits[1], bits[2]);
    }
    return 0;
}
)HARNESS";

class Nn2cppUnrollTest : public ::testing::Test {
  protected:
    void SetUp() override {
        dir_ = ::testing::TempDir() + "nn2cpp_unroll_XXXXXX";
        std::vector<char> tmpl(dir_.begin(), dir_.end());
        tmpl.push_back('\0');
        ASSERT_NE(mkdtemp(tmpl.data()), nullptr) << "mkdtemp failed for " << dir_;
        dir_ = tmpl.data();

        // Serialize the fixture genome for nn2cpp.
        NNGenome g = makeFixtureGenome();
        std::vector<uint8_t> blob;
        nn_serialize(g, blob);
        std::ofstream wf(dir_ + "/fixture.dat", std::ios::binary);
        wf.write(reinterpret_cast<const char*>(blob.data()),
                 static_cast<std::streamsize>(blob.size()));
        ASSERT_TRUE(wf.good());
        wf.close();

        // Scratch nn_program.h mirroring xiao/include/nn_program.h (the
        // generated file's only include).
        writeFile(dir_ + "/nn_program.h",
                  "#pragma once\n"
                  "#include <autoc/nn/evaluator.h>\n"
                  "#include <autoc/eval/sensor_math.h>\n");
        writeFile(dir_ + "/harness_main.cpp", kHarnessSource);
    }

    // Generate code via the real nn2cpp binary.
    void generate(bool unrolled, const std::string& outCpp) {
        std::ostringstream cmd;
        cmd << NN2CPP_BIN << " -i " << dir_ << "/fixture.dat"
            << (unrolled ? " -u" : "")
            << " -a 80,5,100 -f nnGenerated -o " << outCpp;
        std::string out;
        int rc = runCmd(cmd.str(), out);
        ASSERT_EQ(rc, 0) << "nn2cpp failed:\n" << out;
    }

    // Compile one generated file + harness + the xiao-equivalent closure.
    void compileHarness(const std::string& genCpp, const std::string& exe) {
        std::ostringstream cmd;
        cmd << "g++ -std=c++17 -O1"
            << " -I" << dir_
            << " -I" << AUTOC_SOURCE_DIR << "/include"
            << " -isystem " << AUTOC_EIGEN_INC
            << " -isystem " << AUTOC_CEREAL_INC
            << " " << dir_ << "/harness_main.cpp"
            << " " << genCpp
            << " " << AUTOC_SOURCE_DIR << "/src/nn/evaluator.cc"
            << " " << AUTOC_SOURCE_DIR << "/src/eval/sensor_math.cc"
            << " " << AUTOC_SOURCE_DIR << "/src/eval/arena.cc"
            << " -o " << exe;
        std::string out;
        int rc = runCmd(cmd.str(), out);
        ASSERT_EQ(rc, 0) << "harness compile failed:\n" << out;
    }

    std::string dir_;
};

TEST_F(Nn2cppUnrollTest, UnrolledRecurrentMatchesTableDrivenBitExact) {
    const std::string unrolledCpp = dir_ + "/gen_unrolled.cpp";
    const std::string portableCpp = dir_ + "/gen_portable.cpp";
    generate(/*unrolled=*/true, unrolledCpp);
    generate(/*unrolled=*/false, portableCpp);

    // --- 1. The -u emission must be genuinely unrolled, not the
    //        table-driven fallback (pre-039 nn2cpp falls back with a
    //        warning when the genome has a recurrent layer).
    const std::string unrolledSrc = readFile(unrolledCpp);
    ASSERT_NE(unrolledSrc.find("(unrolled)"), std::string::npos)
        << "-u on a recurrent genome fell back to table-driven emission "
           "(no '(unrolled)' marker in the generated header comment)";
    ASSERT_EQ(unrolledSrc.find("nn_forward_recurrent"), std::string::npos)
        << "-u emission still calls the table-driven nn_forward_recurrent";
    // Recurrent unrolling needs persistent state registers + a reset hook.
    ASSERT_NE(unrolledSrc.find("generatedNNProgramReset"), std::string::npos)
        << "unrolled emission lacks the generatedNNProgramReset() hook";

    // --- 2. Bit-exact per-tick equivalence across 24 ticks incl. a
    //        mid-sequence reset.
    const std::string exeU = dir_ + "/harness_unrolled";
    const std::string exeP = dir_ + "/harness_portable";
    compileHarness(unrolledCpp, exeU);
    compileHarness(portableCpp, exeP);

    std::string outU, outP;
    ASSERT_EQ(runCmd(exeU, outU), 0) << outU;
    ASSERT_EQ(runCmd(exeP, outP), 0) << outP;
    ASSERT_FALSE(outU.empty());
    EXPECT_EQ(outU, outP)
        << "unrolled vs table-driven outputs diverge (bit-level)\n"
        << "unrolled:\n" << outU << "\ntable-driven:\n" << outP;
    // Note: an unrolled emission that zeroed h every tick (dead recurrence)
    // would diverge from the table-driven reference by tick 1, so the
    // equality above also covers "recurrence actually persists".

    std::istringstream lines(outU);
    std::vector<std::string> v;
    for (std::string l; std::getline(lines, l);) v.push_back(l);
    ASSERT_EQ(v.size(), 24u) << "harness did not run all 24 ticks";
}

}  // namespace
