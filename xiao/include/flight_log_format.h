#pragma once

// 039 US3 / T012 — versioned binary flight log, v1 (int16-quantized packed).
//
// THE single shared format definition (FR-009): compiled by BOTH the xiao
// firmware (writer, flight_log.cpp) and the desktop test target
// (tests/flightlog_roundtrip_tests.cc). The python decoder
// (src/analytics/flightlog_decode.py) and the web decoder
// (xiao/web/flight_logger.html) implement this same contract
// (specs/039-xiao-20hz-flight/contracts/flight-log-format.md) — no third
// format definition beyond contract + this header.
//
// Self-contained: stdint/string.h only — no Arduino, no Eigen, no autoc
// headers (must compile on arm-none-eabi and desktop gcc identically).
//
// Stream layout (per flight file, byte stream on QSPI flash):
//   FileHeader (once) → { EngageHeader → TickRecord×N → SpanSummary }* with
//   EventRecords interleaved. Records are packed structs, each starting with
//   a nonzero type byte. 0x00 bytes BETWEEN records are buffer word-alignment
//   padding (flash staging pads to 4-byte words) — decoders skip them.
//   Records decode independently (stateless, contract req. 5): a corrupted
//   page costs those ticks only; tick_counter gaps are REPORTED, never
//   silently interpolated.
//
// Constitution V: format_version in the FileHeader; every decoder loud-fails
// on unknown version or scale-CRC mismatch. Changing ANY scale value ⇒
// format_version bump (the scale table travels in the header, CRC-guarded,
// so the decoder never depends on compiled-in scales — Constitution VII).

#include <stdint.h>
#include <string.h>

namespace flightlog {

// ---------------------------------------------------------------------------
// Identity + framing
// ---------------------------------------------------------------------------

// "AFL1" little-endian in the file ('A'=0x41 first byte on the wire).
constexpr uint32_t kMagic = 0x314C4641u;
// v2 (2026-07-11, pre-first-flight): TickRecord gains craft telemetry —
// pos (virtual NED), vel, rabbit (ground-truth target) — so the flight log is
// self-contained for the renderer (-x) and trajectory analysis without an
// INAV-blackbox join. v1 existed only for the 039 bench (T010 record).
// v3 (2026-07-12, arm→disarm self-containment for the INAV correlation run):
// FileHeader gains program[96] (nn2cpp program-source string — the human-
// readable identity behind weight_id); EventCodes 8-11 add disengage reason,
// failsafe + servo-switch transitions, and INAV-clock anchor pairs
// (EventRecord.timestamp_ms = xiao clock, value = INAV ms — fit offset/drift
// against blackbox time).
constexpr uint8_t kFormatVersion = 3;

enum RecordType : uint8_t {
  kPad = 0x00,          // buffer word-alignment filler — skipped, never emitted
  kFileHeader = 0x01,
  kEngageHeader = 0x02,
  kTick = 0x03,
  kEvent = 0x04,
  kSpanSummary = 0x05,
  kFlightState = 0x06,  // armed-but-not-engaged breadcrumb (continuous 'a' trace)
};

// EventRecord codes (console-class events mirrored into the log, FR-014).
enum EventCode : uint8_t {
  kEventArm = 1,           // value: flight number
  kEventDisarm = 2,        // value: 0
  kEventEngage = 3,        // value: span_id (EngageHeader follows separately)
  kEventDisengage = 4,     // value: span_id
  kEventFetchTimeout = 5,  // value: consecutive-failure count
  kEventLogDrop = 6,       // value: records dropped under buffer pressure (FR-008)
  kEventFlashFull = 7,     // value: bytes written when full hit
  // v3 arm→disarm self-containment:
  kEventDisengageReason = 8,  // value: DisengageReason — written right after kEventDisengage
  kEventFailsafe = 9,         // value: 1 = entered failsafe, 0 = cleared (while armed;
                              //   initial state emitted right after kEventArm)
  kEventServoSwitch = 10,     // value: 1 = servo/autoc switch active, 0 = released
                              //   (initial state emitted right after kEventArm)
  kEventInavClock = 11,       // value: INAV clock ms (autoc_state.timestamp_us/1000) sampled
                              //   at this record's timestamp_ms (xiao clock) — correlation
                              //   anchor pair; emitted at arm/engage/disengage/disarm
};

// kEventDisengageReason values (mirrors stopAutoc console strings).
enum DisengageReason : uint8_t {
  kReasonUnknown = 0,
  kReasonServoSwitch = 1,      // pilot released the autoc switch
  kReasonFailsafe = 2,
  kReasonDisarmed = 3,
  kReasonTimeout = 4,          // test-run duration elapsed
  kReasonPathComplete = 5,
  kReasonMspStateFailure = 6,  // MSP2_AUTOC_STATE fetch failed while engaged
  kReasonMissingLocalState = 7,
  kReasonAutocCancelled = 8,
};

// ---------------------------------------------------------------------------
// Quantization scale table (contract §4)
// ---------------------------------------------------------------------------
// One float scale per quantized field: encoded = round(value * scale),
// saturated to ±32767 (symmetric — INT16_MIN is never produced, so
// decode(-encode(x)) == -decode(encode(x))). decoded = raw / scale.
//
// Slot order = PathgenInput enum order (autoc/nn/nn_inputs.h), then the 3 NN
// outputs. 37 + 3 = 40 entries.

constexpr int kNumInputs = 37;
constexpr int kNumOutputs = 3;
// v2 telemetry triples appended after the NN block (slot order below):
// pos (virtual/engage-relative NED m), vel (NED m/s), rabbit (ground-truth
// target position, virtual NED m — the path-follow "rabbit" the dir/dist
// inputs were computed against).
constexpr int kNumScaledFields = kNumInputs + kNumOutputs + 9;
constexpr int kScalePosBase = 40;     // pos_n/e/d
constexpr int kScaleVelBase = 43;     // vel_n/e/d
constexpr int kScaleRabbitBase = 46;  // rabbit_n/e/d

constexpr float kScaleUnit = 32767.0f;  // [-1,1]-bounded: unit vecs, quat, tanh, outputs
constexpr float kScaleDistM = 32.0f;    // raw metres, ±1023.97 m, 1/32 m resolution
constexpr float kScaleSpeed = 256.0f;   // m/s (closing rate, airspeed, vel), ±128 m/s
constexpr float kScaleGyro = 900.0f;    // rad/s, ±36.4 rad/s (~2085 dps), ~0.0011 rad/s res
constexpr float kScalePosM = 16.0f;     // position m, ±2047.9 m, 6.25 cm resolution
                                        // (engage-relative; 60 s span cap bounds excursion)

// v2 writer scale table. The FileHeader CARRIES this table; decoders use the
// header copy (CRC-verified), not a compiled-in copy.
inline void defaultScaleTable(float out[kNumScaledFields]) {
  for (int i = 0; i < 18; i++) out[i] = kScaleUnit;   // target_x/y/z[6] unit vecs
  for (int i = 18; i < 24; i++) out[i] = kScaleDistM; // dist[6] raw metres
  out[24] = kScaleSpeed;                              // closing_rate
  for (int i = 25; i < 29; i++) out[i] = kScaleUnit;  // quat w,x,y,z
  out[29] = kScaleSpeed;                              // airspeed
  for (int i = 30; i < 33; i++) out[i] = kScaleGyro;  // gyro p,q,r
  out[33] = kScaleUnit;                               // dist_to_boundary (tanh)
  for (int i = 34; i < 37; i++) out[i] = kScaleUnit;  // inward_body unit vec
  for (int i = 37; i < 40; i++) out[i] = kScaleUnit;  // NN outputs (tanh)
  for (int i = 0; i < 3; i++) {
    out[kScalePosBase + i] = kScalePosM;              // pos (virtual NED m)
    out[kScaleVelBase + i] = kScaleSpeed;             // vel (NED m/s)
    out[kScaleRabbitBase + i] = kScalePosM;           // rabbit (virtual NED m)
  }
}

// CRC-32 (IEEE 802.3, poly 0xEDB88320, init/final-xor 0xFFFFFFFF). Bitwise —
// runs once per flight over 160 B; no table needed on the embedded side.
inline uint32_t crc32(const void* data, size_t len) {
  const uint8_t* p = static_cast<const uint8_t*>(data);
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= p[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

// ---------------------------------------------------------------------------
// Records — wire format, little-endian, packed.
// raw-ok: hardware byte layout (Constitution VI) — the desktop reader
// converts to gp_scalar domain at the decode boundary.
// ---------------------------------------------------------------------------

#pragma pack(push, 1)

struct FileHeader {
  uint8_t type;                        // kFileHeader
  uint32_t magic;                      // kMagic
  uint8_t format_version;              // kFormatVersion; reader loud-fails on mismatch
  uint8_t firmware_id[8];              // SHA-256[0..7] of generated NN source (nn2cpp)
  uint8_t weight_id[8];                // SHA-256[0..7] of the weight file (nn2cpp)
  char program[96];                    // v3: nn2cpp program-source string (NUL-padded,
                                       //   truncated if longer) — human-readable identity
                                       //   behind weight_id, e.g. "default:autoc-m1/.../genNNNN.dmp.zst"
  uint16_t tick_ms;                    // control tick (50 at 20 Hz) — rate self-describing
  float scales[kNumScaledFields];      // the quantization table used by THIS file
  uint32_t scale_table_crc;            // crc32 over `scales` bytes
};

struct EngageHeader {
  uint8_t type;                // kEngageHeader
  uint32_t engage_timestamp_ms;  // boot-relative millis at span activation
  uint16_t span_id;            // flight-unique, monotonically increasing from 1
  float origin_ned[3];         // raw INAV-frame NED engage point (arena anchor + virtual origin)
  float floor_z_ned;           // RESOLVED: z_engage + K  (raw NED, down-positive)
  float ceiling_z_ned;         // RESOLVED: z_engage − K
  int16_t path_index;          // selected path at engage
};

struct TickRecord {
  uint8_t type;                    // kTick
  uint32_t timestamp_ms;           // boot-relative millis at tick start
  uint16_t tick_counter;           // per-span, from 0; gaps ⇒ dropped ticks (reported)
  int16_t inputs[kNumInputs];      // post-gather values ACTUALLY fed to the NN (honest)
  int16_t outputs[kNumOutputs];    // NN outputs (roll, pitch, throttle), pre-RC-conversion
  // v2 telemetry (renderer/trajectory self-containment): virtual = engage-
  // relative NED; raw NED = virtual + EngageHeader.origin_ned.
  int16_t pos[3];                  // craft position, virtual NED m (scale kScalePosM)
  int16_t vel[3];                  // craft velocity, NED m/s (scale kScaleSpeed)
  int16_t rabbit[3];               // ground-truth target position, virtual NED m
  uint8_t recurrent_reset;         // 1 exactly on the first tick after span activation
  int8_t path_index;               // selected path (0-5) this span
  uint16_t rc_sent[3];             // raw MSP channel values sent (roll, pitch, throttle)
  uint8_t state_valid;             // MSP2_AUTOC_STATE fetch success this tick
};

struct EventRecord {
  uint8_t type;           // kEvent
  uint32_t timestamp_ms;
  uint8_t code;           // EventCode
  uint32_t value;         // code-specific payload
};

// Armed-but-not-engaged flight breadcrumb, written every control tick outside
// autoc spans (the old text log's whole-flight "Nav State" role): keeps the
// arm→disarm trace CONTINUOUS for the renderer's all-flight view. RAW INAV
// frame (home/arm origin) — no engage offset applies. Reuses the tick scale
// slots: pos → kScalePosBase, vel → kScaleVelBase, quat → kScaleUnit.
struct FlightStateRecord {
  uint8_t type;           // kFlightState
  uint32_t timestamp_ms;
  int16_t pos_raw[3];     // raw NED m (scale kScalePosM)
  int16_t vel[3];         // NED m/s (scale kScaleSpeed)
  int16_t quat[4];        // q_EB w,x,y,z (scale kScaleUnit)
};

// Span-summary at disengage: today's `loopStats` + MSP pipeline stats — the
// latency memo's flight-side numbers come from these (contract "Loop-health
// stats" clause; the console line they used to print is demoted by FR-014).
struct SpanSummary {
  uint8_t type;            // kSpanSummary
  uint32_t timestamp_ms;   // disengage time
  uint16_t span_id;
  // loopStats (main.h LoopStats)
  uint32_t ticks, overruns, resyncs, max_late_ms, total_late_ms;
  // PipelineStats (msplink.cpp) — µs; avg = sum/samples computed at write
  uint32_t samples;
  uint32_t fetch_min_us, fetch_avg_us, fetch_max_us;
  uint32_t eval_min_us, eval_avg_us, eval_max_us;
  uint32_t send_min_us, send_avg_us, send_max_us;
  uint32_t total_min_us, total_avg_us, total_max_us;
  uint32_t interval_min_us, interval_avg_us, interval_max_us;
  // logging health (FR-008): ticks written vs dropped/coalesced under pressure
  uint32_t ticks_logged, ticks_dropped;
  // T019: DWT cycle count of one unrolled NN eval (0 until measured; one
  // number per firmware image, captured at the first eval after boot)
  uint32_t dwt_eval_cycles;
};

#pragma pack(pop)

// Budget arithmetic (contract "Tests" + FR-008): 114 B/tick ⇒ 2×4 min ≈ 1.09 MB
// in the 2.04 MB region (~47% headroom).
static_assert(sizeof(TickRecord) <= 120, "TickRecord must stay <= 120 B (flash budget)");
static_assert(sizeof(TickRecord) ==
                  1 + 4 + 2 + 2 * kNumInputs + 2 * kNumOutputs + 18 + 1 + 1 + 6 + 1,
              "TickRecord must be packed (no compiler padding)");
static_assert(sizeof(FileHeader) == 1 + 4 + 1 + 8 + 8 + 96 + 2 + 4 * kNumScaledFields + 4,
              "FileHeader must be packed");
static_assert(sizeof(EngageHeader) == 1 + 4 + 2 + 12 + 4 + 4 + 2,
              "EngageHeader must be packed");
static_assert(sizeof(EventRecord) == 1 + 4 + 1 + 4, "EventRecord must be packed");
static_assert(sizeof(FlightStateRecord) == 1 + 4 + 6 + 6 + 8,
              "FlightStateRecord must be packed (25 B — ~500 B/s at 20 Hz outside spans)");

// ---------------------------------------------------------------------------
// Encode / decode helpers
// ---------------------------------------------------------------------------

// Saturating round-to-nearest. Symmetric clamp to ±32767 (never INT16_MIN)
// so saturation decodes to the exact rail value with sign preserved.
inline int16_t encodeScaled(float v, float scale) {
  float x = v * scale;
  if (x >= 32767.0f) return 32767;
  if (x <= -32767.0f) return -32767;
  return static_cast<int16_t>(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

inline float decodeScaled(int16_t raw, float scale) {
  return static_cast<float>(raw) / scale;
}

inline void initFileHeader(FileHeader& h, const uint8_t firmware_id[8],
                           const uint8_t weight_id[8], const char* program,
                           uint16_t tick_ms) {
  memset(&h, 0, sizeof(h));
  h.type = kFileHeader;
  h.magic = kMagic;
  h.format_version = kFormatVersion;
  memcpy(h.firmware_id, firmware_id, 8);
  memcpy(h.weight_id, weight_id, 8);
  // NUL-padded via the memset; silently truncated to 95 chars + NUL.
  strncpy(h.program, program, sizeof(h.program) - 1);
  h.tick_ms = tick_ms;
  defaultScaleTable(h.scales);
  h.scale_table_crc = crc32(h.scales, sizeof(h.scales));
}

enum class ValidateResult { kOk, kBadMagic, kBadVersion, kBadScaleCrc };

inline ValidateResult validateFileHeader(const FileHeader& h) {
  if (h.magic != kMagic) return ValidateResult::kBadMagic;
  if (h.format_version != kFormatVersion) return ValidateResult::kBadVersion;
  if (crc32(h.scales, sizeof(h.scales)) != h.scale_table_crc)
    return ValidateResult::kBadScaleCrc;
  return ValidateResult::kOk;
}

// Quantize the NN I/O + telemetry block of a TickRecord (caller fills aux).
inline void encodeTick(const float inputs[kNumInputs],
                       const float outputs[kNumOutputs], const float pos[3],
                       const float vel[3], const float rabbit[3],
                       const float scales[kNumScaledFields], TickRecord& rec) {
  for (int i = 0; i < kNumInputs; i++)
    rec.inputs[i] = encodeScaled(inputs[i], scales[i]);
  for (int i = 0; i < kNumOutputs; i++)
    rec.outputs[i] = encodeScaled(outputs[i], scales[kNumInputs + i]);
  for (int i = 0; i < 3; i++) {
    rec.pos[i] = encodeScaled(pos[i], scales[kScalePosBase + i]);
    rec.vel[i] = encodeScaled(vel[i], scales[kScaleVelBase + i]);
    rec.rabbit[i] = encodeScaled(rabbit[i], scales[kScaleRabbitBase + i]);
  }
}

inline void decodeTick(const TickRecord& rec, const float scales[kNumScaledFields],
                       float inputs[kNumInputs], float outputs[kNumOutputs],
                       float pos[3], float vel[3], float rabbit[3]) {
  for (int i = 0; i < kNumInputs; i++)
    inputs[i] = decodeScaled(rec.inputs[i], scales[i]);
  for (int i = 0; i < kNumOutputs; i++)
    outputs[i] = decodeScaled(rec.outputs[i], scales[kNumInputs + i]);
  for (int i = 0; i < 3; i++) {
    pos[i] = decodeScaled(rec.pos[i], scales[kScalePosBase + i]);
    vel[i] = decodeScaled(rec.vel[i], scales[kScaleVelBase + i]);
    rabbit[i] = decodeScaled(rec.rabbit[i], scales[kScaleRabbitBase + i]);
  }
}

// ---------------------------------------------------------------------------
// Stream walker (decoder side; also exercised by the round-trip tests).
// Skips 0x00 padding between records; stops with failed()=true on an unknown
// type byte or a record truncated by the end of the stream (loud, contract
// "Decoder MUST fail loud ... impossible field counts").
// ---------------------------------------------------------------------------

struct RecordView {
  uint8_t type = kPad;
  const uint8_t* data = nullptr;  // points at the type byte
  size_t size = 0;
  template <typename T>
  const T* as() const {
    return (size == sizeof(T)) ? reinterpret_cast<const T*>(data) : nullptr;
  }
};

inline size_t recordSizeForType(uint8_t type) {
  switch (type) {
    case kFileHeader: return sizeof(FileHeader);
    case kEngageHeader: return sizeof(EngageHeader);
    case kTick: return sizeof(TickRecord);
    case kEvent: return sizeof(EventRecord);
    case kSpanSummary: return sizeof(SpanSummary);
    case kFlightState: return sizeof(FlightStateRecord);
    default: return 0;  // unknown — parse error
  }
}

class StreamWalker {
 public:
  StreamWalker(const uint8_t* data, size_t len)
      : data_(data), len_(len), pos_(0), failed_(false) {}

  // Advances to the next record. Returns false at end-of-stream or on error
  // (check failed() to distinguish).
  bool next(RecordView& out) {
    if (failed_) return false;
    while (pos_ < len_ && data_[pos_] == kPad) pos_++;  // skip padding
    if (pos_ >= len_) return false;                      // clean EOF
    const uint8_t type = data_[pos_];
    const size_t sz = recordSizeForType(type);
    if (sz == 0 || pos_ + sz > len_) {
      failed_ = true;  // unknown type or truncated record — loud
      return false;
    }
    out.type = type;
    out.data = data_ + pos_;
    out.size = sz;
    pos_ += sz;
    return true;
  }

  bool failed() const { return failed_; }
  size_t position() const { return pos_; }

 private:
  const uint8_t* data_;
  size_t len_;
  size_t pos_;
  bool failed_;
};

}  // namespace flightlog
