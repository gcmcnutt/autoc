// 037 T016 -- NN-layout round-trip + fail-loud tests.
//
// Contract (contracts/nn-input-layout.md "Tests"):
//   1. Round-trip: an AircraftState serialized at the current layout reads
//      back with byte-identical NN inputs/outputs.
//   2. Fail-loud: a stream carrying a DIFFERENT history-layout version
//      (e.g. a pre-037 v=2 dmp, whose bytes at the marker position are the
//      first input float) triggers a clear layout error -- never a silent
//      parse (Principle V; no cereal version bump per
//      feedback_no_cereal_versioning).

#include <gtest/gtest.h>

#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>

#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>

#include "autoc/eval/aircraft_state.h"
#include "autoc/rpc/protocol.h"  // gp_vec3 / gp_quat cereal handlers

namespace {

AircraftState makeState() {
  AircraftState st{0,
                   18.0f,
                   gp_vec3(18.0f, 1.0f, -0.5f),
                   gp_quat::Identity(),
                   gp_vec3(10.0f, -20.0f, -30.0f),
                   0.25f,
                   -0.5f,
                   0.75f,
                   1234};
  // Populate the NN capture block with recognizable values.
  NNInputs in{};
  float* raw = reinterpret_cast<float*>(&in);  // raw-ok: NN-byte-format buffer
  for (int i = 0; i < NN_INPUT_COUNT; ++i) raw[i] = 0.5f + 0.25f * i;
  float out[NN_OUTPUT_COUNT];
  for (int i = 0; i < NN_OUTPUT_COUNT; ++i) out[i] = -0.5f + 0.5f * i;
  st.setNNData(in, out, NN_OUTPUT_COUNT);
  return st;
}

std::string serializeState(const AircraftState& st) {
  std::ostringstream os(std::ios::binary);
  {
    cereal::BinaryOutputArchive ar(os);
    ar(st);
  }
  return os.str();
}

TEST(NNLayout, RoundTripByteIdentical) {
  AircraftState st = makeState();
  const std::string bytes = serializeState(st);

  AircraftState back{};
  {
    std::istringstream is(bytes, std::ios::binary);
    cereal::BinaryInputArchive ar(is);
    ar(back);
  }

  // Byte-identical NN inputs + outputs after the round trip.
  EXPECT_EQ(0, std::memcmp(&st.getNNInputs(), &back.getNNInputs(),
                           sizeof(NNInputs)));
  EXPECT_EQ(0, std::memcmp(st.getNNOutputs(), back.getNNOutputs(),
                           NN_OUTPUT_COUNT * sizeof(float)));

  // And the re-serialized stream is byte-identical to the original.
  EXPECT_EQ(bytes, serializeState(back));
}

TEST(NNLayout, OldLayoutStreamFailsLoud) {
  AircraftState st = makeState();
  std::string bytes = serializeState(st);

  // Locate the count fields [inputCount=37][outputCount=3]; flip the
  // history-layout version to the pre-037 value 1 to reproduce what reading
  // an old-layout stream looks like. 038 US3 SPLIT serialize reordered the NN
  // block to [mode:1][historyLayout:4][inputCount:4][outputCount:4], so the
  // layout marker is now the 4 bytes IMMEDIATELY BEFORE the counts (pos − 4),
  // not after them.
  unsigned char needle[8];
  const uint32_t inCount = NN_INPUT_COUNT, outCount = NN_OUTPUT_COUNT;
  std::memcpy(needle, &inCount, 4);
  std::memcpy(needle + 4, &outCount, 4);
  const std::string pat(reinterpret_cast<const char*>(needle), 8);
  const size_t pos = bytes.find(pat);
  ASSERT_NE(std::string::npos, pos) << "count fields not found in stream";
  ASSERT_GE(pos, static_cast<size_t>(4)) << "no room for the layout marker before the counts";
  const uint32_t wrongLayout = 1;
  std::memcpy(&bytes[pos - 4], &wrongLayout, 4);

  AircraftState back{};
  std::istringstream is(bytes, std::ios::binary);
  cereal::BinaryInputArchive ar(is);
  try {
    ar(back);
    FAIL() << "expected a history-layout mismatch error";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(nullptr, std::strstr(e.what(), "history-layout mismatch"))
        << "unexpected error text: " << e.what();
  }
}

}  // namespace
