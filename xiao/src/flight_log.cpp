#include "flight_log.h"

#include <string.h>

#include "flash_logger.h"
#include "main.h"

using namespace flightlog;

// Writer-side scale table — initialized with the v1 table at file begin; the
// same table travels inside the FileHeader (CRC-guarded), so decoders never
// depend on a compiled-in copy (Constitution VII).
static float g_scales[kNumScaledFields];
static bool g_scalesInit = false;

// Per-span counters (FR-008 drop accounting + tick_counter framing).
static uint16_t g_tickCounter = 0;
static uint32_t g_ticksLogged = 0;
static uint32_t g_ticksDropped = 0;
static uint32_t g_pendingDropNotice = 0;  // drops awaiting a kEventLogDrop record

static void noteDrop() {
  g_ticksDropped++;
  g_pendingDropNotice++;
}

// After a successful write, surface any accumulated drops as an EventRecord
// (they could not be written while the buffers were saturated).
static void flushDropNotice(uint32_t timestamp_ms) {
  if (g_pendingDropNotice == 0) {
    return;
  }
  EventRecord ev;
  ev.type = kEvent;
  ev.timestamp_ms = timestamp_ms;
  ev.code = kEventLogDrop;
  ev.value = g_pendingDropNotice;
  if (flashLoggerWriteBinary(&ev, sizeof(ev))) {
    g_pendingDropNotice = 0;
  }
}

void flightLogBeginFile(const uint8_t firmware_id[8], const uint8_t weight_id[8],
                        uint16_t tick_ms) {
  FileHeader h;
  initFileHeader(h, firmware_id, weight_id, tick_ms);
  memcpy(g_scales, h.scales, sizeof(g_scales));
  g_scalesInit = true;
  g_pendingDropNotice = 0;
  if (!flashLoggerWriteBinary(&h, sizeof(h))) {
    logPrint(ERROR, "Flight log: FileHeader write failed - flash log unusable this flight");
  }
}

void flightLogEvent(uint8_t code, uint32_t value) {
  EventRecord ev;
  ev.type = kEvent;
  ev.timestamp_ms = millis();
  ev.code = code;
  ev.value = value;
  if (flashLoggerWriteBinary(&ev, sizeof(ev))) {
    flushDropNotice(ev.timestamp_ms);
  }
  // Sparse events are not part of the per-tick drop budget; a lost event
  // under saturation is acceptable (the span summary still carries totals).
}

void flightLogEngage(uint16_t span_id, uint32_t engage_timestamp_ms,
                     const float origin_ned[3], float floor_z_ned,
                     float ceiling_z_ned, int16_t path_index) {
  g_tickCounter = 0;
  g_ticksLogged = 0;
  g_ticksDropped = 0;

  EngageHeader eh;
  eh.type = kEngageHeader;
  eh.engage_timestamp_ms = engage_timestamp_ms;
  eh.span_id = span_id;
  memcpy(eh.origin_ned, origin_ned, sizeof(eh.origin_ned));
  eh.floor_z_ned = floor_z_ned;
  eh.ceiling_z_ned = ceiling_z_ned;
  eh.path_index = path_index;
  if (!flashLoggerWriteBinary(&eh, sizeof(eh))) {
    logPrint(ERROR, "Flight log: EngageHeader write failed (span %u)", span_id);
  }
}

uint16_t flightLogTick(uint32_t timestamp_ms, const float inputs[kNumInputs],
                       const float outputs[kNumOutputs], const float pos[3],
                       const float vel[3], const float rabbit[3],
                       bool recurrent_reset, int8_t path_index,
                       const uint16_t rc_sent[3], bool state_valid) {
  const uint16_t counter = g_tickCounter++;
  if (!g_scalesInit) {
    // FileHeader never got written (flash uninitialized) — nothing to encode
    // against; count it so the bench review sees the hole.
    noteDrop();
    return counter;
  }

  TickRecord rec;
  rec.type = kTick;
  rec.timestamp_ms = timestamp_ms;
  rec.tick_counter = counter;
  encodeTick(inputs, outputs, pos, vel, rabbit, g_scales, rec);
  rec.recurrent_reset = recurrent_reset ? 1 : 0;
  rec.path_index = path_index;
  rec.rc_sent[0] = rc_sent[0];
  rec.rc_sent[1] = rc_sent[1];
  rec.rc_sent[2] = rc_sent[2];
  rec.state_valid = state_valid ? 1 : 0;

  if (flashLoggerWriteBinary(&rec, sizeof(rec))) {
    g_ticksLogged++;
    flushDropNotice(timestamp_ms);
  } else {
    noteDrop();  // tick_counter gap becomes visible to the decoder
  }
  return counter;
}

void flightLogSpanSummary(flightlog::SpanSummary& summary) {
  summary.type = kSpanSummary;
  summary.ticks_logged = g_ticksLogged;
  summary.ticks_dropped = g_ticksDropped;
  if (!flashLoggerWriteBinary(&summary, sizeof(summary))) {
    logPrint(ERROR, "Flight log: SpanSummary write failed (span %u)", summary.span_id);
  }
}

uint32_t flightLogTicksLogged() { return g_ticksLogged; }
uint32_t flightLogTicksDropped() { return g_ticksDropped; }
