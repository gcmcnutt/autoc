#pragma once

// 039 US3 (T013) — xiao-side binary flight-log writer.
// Thin record-building layer over flashLoggerWriteBinary(); the byte format
// is flight_log_format.h (the single shared definition, FR-009). Drop
// accounting per FR-008: failed appends are counted, never retried in-tick,
// and surfaced as a kEventLogDrop record once writes succeed again plus in
// the span-summary drop counter.
//
// raw-ok: hardware byte layout — this whole surface is the wire-format
// boundary (float = NN-byte-format buffers, uint = packed record fields);
// gp_scalar domain resumes at the desktop decode boundary.

#include <stdint.h>

#include "flight_log_format.h"

// Write the FileHeader. Call once per flight file, right after
// flashLoggerBeginFlight() succeeds (arm transition). program = nn2cpp
// program-source string (v3 provenance; truncated to 95 chars in the header).
void flightLogBeginFile(const uint8_t firmware_id[8], const uint8_t weight_id[8],
                        const char* program, uint16_t tick_ms);

// Sparse console-class event into the log (also independently logPrint'd to
// the console by the caller when human-relevant).
void flightLogEvent(uint8_t code, uint32_t value);

// Span activation: resolved engage-anchored arena provenance (FR-001).
// Resets the per-span tick/drop counters.
void flightLogEngage(uint16_t span_id, uint32_t engage_timestamp_ms,
                     const float origin_ned[3], float floor_z_ned,
                     float ceiling_z_ned, int16_t path_index);

// One control tick while engaged. inputs = the 37 post-gather values ACTUALLY
// fed to the NN this tick (honest recording); outputs = the 3 NN outputs.
// v2 telemetry: pos/vel = craft state (virtual NED / NED m/s), rabbit =
// ground-truth target position (virtual NED) this tick.
// Returns the per-span tick counter used (for callers that want it).
uint16_t flightLogTick(uint32_t timestamp_ms, const float inputs[flightlog::kNumInputs],
                       const float outputs[flightlog::kNumOutputs],
                       const float pos[3], const float vel[3], const float rabbit[3],
                       bool recurrent_reset, int8_t path_index,
                       const uint16_t rc_sent[3], bool state_valid);

// Span end: caller fills the loopStats/pipeline fields; this layer stamps
// type/span bookkeeping + the ticks_logged/ticks_dropped counters and writes.
void flightLogSpanSummary(flightlog::SpanSummary& summary);

// Armed-but-not-engaged breadcrumb (raw INAV frame) — keeps the arm→disarm
// trace continuous for the renderer's all-flight view. Call once per control
// tick while armed and outside a span.
void flightLogFlightState(uint32_t timestamp_ms, const float pos_raw[3],
                          const float vel[3], const float quat_wxyz[4]);

// Per-span logging health (feeds the span summary + bench review).
uint32_t flightLogTicksLogged();
uint32_t flightLogTicksDropped();
