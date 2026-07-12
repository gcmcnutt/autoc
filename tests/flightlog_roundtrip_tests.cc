// 039 T011 — versioned binary flight log round-trip tests
// (contracts/flight-log-format.md §Tests, written FAILING-FIRST before
// xiao/include/flight_log_format.h exists — T012 makes these green).
//
// The format header is the single shared definition compiled by BOTH the
// xiao firmware and this desktop test target (plan.md: the xiao build cannot
// link desktop code, so the shared artifact is the contract + this header).
//
// Covered here (contract "Tests (write first)"):
//   1. Encode→decode round trip: every field |decoded − original| ≤ the
//      field's quantization step (1/scale).
//   2. Version loud-fail: validateFileHeader rejects version+1.
//   3. Scale-CRC loud-fail: corrupting a scale entry breaks the CRC.
//   4. Saturation: out-of-range input saturates (never wraps) and decodes
//      to the rail value, sign preserved.
//   5. Budget arithmetic: TickRecord ≤ 100 B (also a static_assert in the
//      header itself).
//   6. Stream walk: a synthetic file (FileHeader + EngageHeader + ticks +
//      0x00 buffer padding + event + span summary) parses record-for-record;
//      padding bytes between records are skipped; unknown type byte is a
//      loud parse error, not a silent skip.

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "flight_log_format.h"

using namespace flightlog;

namespace {

// Convenience: writer-side scale table (the contract's v1 table).
struct ScaleTableFixture {
  float scales[kNumScaledFields];
  ScaleTableFixture() { defaultScaleTable(scales); }
};

TEST(FlightLogFormat, TickRecordBudget) {
  // Contract: static assert record size ≤ 100 B (95-ish quantized + framing).
  EXPECT_LE(sizeof(TickRecord), 100u);
  // Spot-check the packed layout didn't grow padding.
  EXPECT_EQ(sizeof(TickRecord),
            1u + 4u + 2u + 2u * kNumInputs + 2u * kNumOutputs + 1u + 1u + 6u + 1u);
}

TEST(FlightLogFormat, RoundTripWithinQuantizationStep) {
  ScaleTableFixture t;

  // Representative in-range values per field group (PathgenInput slot order):
  // unit-vec components, raw-metre distances, closing rate, quat, airspeed,
  // gyro rad/s, tanh boundary distance, inward unit vec.
  float inputs[kNumInputs];
  for (int i = 0; i < 18; i++) inputs[i] = -1.0f + 2.0f * (float)i / 17.0f;  // target_x/y/z
  for (int i = 18; i < 24; i++) inputs[i] = 3.7f + 41.3f * (float)(i - 18);  // dist m
  inputs[24] = -11.25f;                                                       // closing m/s
  inputs[25] = 0.7071f; inputs[26] = -0.7071f; inputs[27] = 0.5f; inputs[28] = -0.5f;
  inputs[29] = 17.3f;                                                         // airspeed
  inputs[30] = -6.283f; inputs[31] = 2.5f; inputs[32] = -0.01f;               // gyro
  inputs[33] = 0.42f;                                                         // dist_to_boundary
  inputs[34] = 0.1f; inputs[35] = -0.9f; inputs[36] = 0.3f;                   // inward_body

  float outputs[kNumOutputs] = {-0.335f, 0.998f, 0.0421f};

  TickRecord rec;
  std::memset(&rec, 0, sizeof(rec));
  rec.type = kTick;
  encodeTick(inputs, outputs, t.scales, rec);

  float back_in[kNumInputs], back_out[kNumOutputs];
  decodeTick(rec, t.scales, back_in, back_out);

  for (int i = 0; i < kNumInputs; i++) {
    const float step = 1.0f / t.scales[i];
    EXPECT_LE(std::abs(back_in[i] - inputs[i]), step)
        << "input slot " << i << " scale " << t.scales[i];
  }
  for (int i = 0; i < kNumOutputs; i++) {
    const float step = 1.0f / t.scales[kNumInputs + i];
    EXPECT_LE(std::abs(back_out[i] - outputs[i]), step) << "output " << i;
  }
}

TEST(FlightLogFormat, FileHeaderValidatesCleanAndRejectsVersionBump) {
  uint8_t fw[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  uint8_t wt[8] = {9, 10, 11, 12, 13, 14, 15, 16};
  FileHeader h;
  initFileHeader(h, fw, wt, /*tick_ms=*/50);

  EXPECT_EQ(validateFileHeader(h), ValidateResult::kOk);
  EXPECT_EQ(h.magic, kMagic);
  EXPECT_EQ(h.format_version, kFormatVersion);
  EXPECT_EQ(h.tick_ms, 50);
  EXPECT_EQ(0, std::memcmp(h.firmware_id, fw, 8));
  EXPECT_EQ(0, std::memcmp(h.weight_id, wt, 8));

  // Contract: decoder rejects a header with version+1 — loud, not best-effort.
  FileHeader bumped = h;
  bumped.format_version = kFormatVersion + 1;
  EXPECT_EQ(validateFileHeader(bumped), ValidateResult::kBadVersion);

  // And a wrong magic is not a flight log at all.
  FileHeader wrongMagic = h;
  wrongMagic.magic ^= 0xFF;
  EXPECT_EQ(validateFileHeader(wrongMagic), ValidateResult::kBadMagic);
}

TEST(FlightLogFormat, ScaleTableCrcLoudFail) {
  uint8_t fw[8] = {0}, wt[8] = {0};
  FileHeader h;
  initFileHeader(h, fw, wt, 50);
  ASSERT_EQ(validateFileHeader(h), ValidateResult::kOk);

  // Corrupt one scale entry: the stored CRC no longer matches.
  h.scales[24] *= 2.0f;
  EXPECT_EQ(validateFileHeader(h), ValidateResult::kBadScaleCrc);

  // Corrupt the CRC itself over an intact table: same loud failure.
  FileHeader h2;
  initFileHeader(h2, fw, wt, 50);
  h2.scale_table_crc ^= 0xDEADBEEF;
  EXPECT_EQ(validateFileHeader(h2), ValidateResult::kBadScaleCrc);
}

TEST(FlightLogFormat, SaturationNeverWraps) {
  ScaleTableFixture t;

  // dist slot 18: scale 32 → rail at 32767/32 ≈ 1023.97 m. A 5 km reading
  // must clamp to the positive rail (wrap would produce a negative value).
  const float rail = 32767.0f / t.scales[18];
  int16_t enc = encodeScaled(5000.0f, t.scales[18]);
  EXPECT_EQ(enc, INT16_MAX);
  EXPECT_FLOAT_EQ(decodeScaled(enc, t.scales[18]), rail);

  int16_t encNeg = encodeScaled(-5000.0f, t.scales[18]);
  EXPECT_EQ(encNeg, -INT16_MAX);  // symmetric clamp: -32767, not -32768
  EXPECT_FLOAT_EQ(decodeScaled(encNeg, t.scales[18]), -rail);

  // Unit-bounded slot: 1.0001 saturates to exactly the +1 rail.
  int16_t encUnit = encodeScaled(1.0001f, t.scales[0]);
  EXPECT_EQ(encUnit, INT16_MAX);
  EXPECT_FLOAT_EQ(decodeScaled(encUnit, t.scales[0]), 1.0f);
}

TEST(FlightLogFormat, StreamWalkSkipsPaddingAndFailsLoudOnUnknownType) {
  uint8_t fw[8] = {0xAA}, wt[8] = {0xBB};
  FileHeader fh;
  initFileHeader(fh, fw, wt, 50);

  EngageHeader eh;
  std::memset(&eh, 0, sizeof(eh));
  eh.type = kEngageHeader;
  eh.engage_timestamp_ms = 120000;
  eh.span_id = 1;
  eh.origin_ned[0] = 1.5f; eh.origin_ned[1] = -2.5f; eh.origin_ned[2] = -30.0f;
  eh.floor_z_ned = 17.5f;
  eh.ceiling_z_ned = -77.5f;
  eh.path_index = 3;

  ScaleTableFixture t;
  float in[kNumInputs] = {0}, out[kNumOutputs] = {0.1f, -0.2f, 0.3f};
  TickRecord tick;
  std::memset(&tick, 0, sizeof(tick));
  tick.type = kTick;
  tick.timestamp_ms = 120050;
  tick.tick_counter = 0;
  tick.recurrent_reset = 1;
  encodeTick(in, out, t.scales, tick);

  EventRecord ev;
  ev.type = kEvent;
  ev.timestamp_ms = 130000;
  ev.code = kEventDisengage;
  ev.value = 0;

  SpanSummary ss;
  std::memset(&ss, 0, sizeof(ss));
  ss.type = kSpanSummary;
  ss.timestamp_ms = 130001;
  ss.span_id = 1;
  ss.ticks = 42;

  // Assemble a stream with word-alignment padding (0x00) between records,
  // exactly what the flash buffer staging produces at buffer boundaries.
  std::vector<uint8_t> blob;
  auto append = [&blob](const void* p, size_t n) {
    const uint8_t* b = static_cast<const uint8_t*>(p);
    blob.insert(blob.end(), b, b + n);
  };
  append(&fh, sizeof(fh));
  blob.insert(blob.end(), 3, 0x00);  // padding
  append(&eh, sizeof(eh));
  for (int i = 0; i < 3; i++) {
    tick.tick_counter = static_cast<uint16_t>(i);
    tick.recurrent_reset = (i == 0) ? 1 : 0;
    append(&tick, sizeof(tick));
  }
  blob.insert(blob.end(), 5, 0x00);  // padding
  append(&ev, sizeof(ev));
  append(&ss, sizeof(ss));
  blob.insert(blob.end(), 7, 0x00);  // trailing pad

  // Walk: expect exactly the record sequence back.
  StreamWalker w(blob.data(), blob.size());
  const FileHeader* gotFh = nullptr;
  const EngageHeader* gotEh = nullptr;
  int tickCount = 0, eventCount = 0, summaryCount = 0;
  RecordView v;
  while (w.next(v)) {
    switch (v.type) {
      case kFileHeader: gotFh = v.as<FileHeader>(); break;
      case kEngageHeader: gotEh = v.as<EngageHeader>(); break;
      case kTick: {
        const TickRecord* tr = v.as<TickRecord>();
        EXPECT_EQ(tr->tick_counter, tickCount);
        EXPECT_EQ(tr->recurrent_reset, tickCount == 0 ? 1 : 0);
        tickCount++;
        break;
      }
      case kEvent: eventCount++; break;
      case kSpanSummary: summaryCount++; break;
      default: FAIL() << "unexpected record type " << (int)v.type;
    }
  }
  EXPECT_FALSE(w.failed()) << "clean stream must not report a parse error";
  ASSERT_NE(gotFh, nullptr);
  EXPECT_EQ(validateFileHeader(*gotFh), ValidateResult::kOk);
  ASSERT_NE(gotEh, nullptr);
  EXPECT_EQ(gotEh->span_id, 1);
  EXPECT_FLOAT_EQ(gotEh->floor_z_ned, 17.5f);
  EXPECT_EQ(tickCount, 3);
  EXPECT_EQ(eventCount, 1);
  EXPECT_EQ(summaryCount, 1);

  // Unknown record type: loud parse failure (never a silent skip).
  std::vector<uint8_t> bad(blob);
  bad.push_back(0x7F);  // garbage type byte after trailing pad
  bad.push_back(0x01);
  StreamWalker wb(bad.data(), bad.size());
  while (wb.next(v)) {}
  EXPECT_TRUE(wb.failed());

  // Truncated mid-record: also loud.
  std::vector<uint8_t> trunc(blob.begin(), blob.begin() + sizeof(fh) + 3 + 10);
  StreamWalker wt2(trunc.data(), trunc.size());
  while (wt2.next(v)) {}
  EXPECT_TRUE(wt2.failed());
}

}  // namespace
