#pragma once
// 045 F2 (tasks.md T018) — BLE flight-log download protocol v2, the firmware's send window.
// Contract: specs/045-m2-tracking/contracts/ble-download-v2.md. Pure logic (no Arduino headers), so tests/ble_dl2_tests.cc runs
// it on the desktop against a lossy, duplicating, reordering channel.
//
// Why (research.md R2, measured 2026-11-12): v1 was strict stop-and-wait, one 240 B chunk per ~97.5 ms round trip (two BLE
// connection intervals), ~2.4 kB/s. v2 keeps up to `window` offset-tagged chunks in flight, so several notifications go in each
// connection event. The receiver ACKs cumulatively (the highest contiguous byte it holds); the sender goes back to the ACK on a
// non-advancing ACK (the receiver saw a gap or a duplicate) or when the window stalls. Duplicates are ignored by offset.

#include <cstdint>

namespace dl2 {

constexpr uint32_t kHeaderBytes = 4;       // u32 little-endian offset before each payload
constexpr uint32_t kMaxWindow = 8;         // chunks in flight, upper bound on what the page may ask for. Measured 2026-11-12:
                                           // window 8 already loses notifications (ArduinoBLE's 256 B HCI receive ring
                                           // drops events when full); the page defaults to 2
constexpr uint32_t kRetransmitMs = 400;    // data outstanding and no ACK progress for this long ⇒ go back to the ACK
constexpr uint32_t kRewindHoldoffMs = 100; // ignore further non-advancing ACKs this soon after a rewind (old window draining)
constexpr uint32_t kInactivityMs = 5000;   // no ACK at all for this long ⇒ abandon the transfer, flash back to idle

// Payload bytes per notification for a negotiated ATT MTU: 3 B ATT notify header + our 4 B offset. 0 ⇒ MTU too small.
inline uint32_t chunkForMtu(uint32_t mtu) { return mtu > 3 + kHeaderBytes ? mtu - 3 - kHeaderBytes : 0; }

// The main loop skips its MSP polling only while a download runs on the ground (tasks.md T021). Arming stops BLE anyway.
inline bool shouldSkipMspPoll(bool downloading, bool armed) { return downloading && !armed; }

enum class Event : uint8_t { None, Done, Rewound, Inactive };

struct Sender {          // plain state; start() sets every field
  uint32_t size;         // bytes in the file
  uint32_t chunk;        // payload bytes per notification
  uint32_t window;       // chunks allowed in flight
  uint32_t sendOff;      // next byte to send
  uint32_t ackOff;       // receiver holds every byte below this
  uint32_t lastAckMs;    // last ACK of any kind (inactivity)
  uint32_t lastProgressMs; // last ACK progress or rewind (retransmit timer)
  uint32_t lastRewindMs;
  uint32_t rewinds;      // instrumentation
  bool active;
};

inline void start(Sender& s, uint32_t size, uint32_t chunk, uint32_t fromOff, uint32_t window, uint32_t nowMs) {
  s.size = size;
  s.chunk = chunk;
  s.window = window == 0 ? 1 : (window > kMaxWindow ? kMaxWindow : window);
  s.sendOff = s.ackOff = fromOff > size ? size : fromOff;
  s.lastAckMs = s.lastProgressMs = nowMs;
  s.lastRewindMs = nowMs - kRewindHoldoffMs;
  s.rewinds = 0;
  s.active = size > 0 && chunk > 0 && s.ackOff < size;
}

inline void rewind(Sender& s, uint32_t nowMs) {
  s.sendOff = s.ackOff;
  s.lastProgressMs = s.lastRewindMs = nowMs;
  s.rewinds++;
}

// The next chunk to notify, if the window has room: *off / *len, and sendOff advances.
inline bool next(Sender& s, uint32_t* off, uint32_t* len) {
  if (!s.active || s.sendOff >= s.size) return false;
  if (s.sendOff - s.ackOff >= s.window * s.chunk) return false;
  const uint32_t left = s.size - s.sendOff;
  *off = s.sendOff;
  *len = left < s.chunk ? left : s.chunk;
  s.sendOff += *len;
  return true;
}

// A cumulative ACK: the receiver holds every byte below ackOff.
inline Event onAck(Sender& s, uint32_t ackOff, uint32_t nowMs) {
  if (!s.active) return Event::None;
  s.lastAckMs = nowMs;
  if (ackOff > s.size) ackOff = s.size;
  if (ackOff > s.ackOff) {
    s.ackOff = ackOff;
    if (s.sendOff < s.ackOff) s.sendOff = s.ackOff;  // the receiver is ahead of a rewind
    s.lastProgressMs = nowMs;
    if (s.ackOff >= s.size) {
      s.active = false;
      return Event::Done;
    }
    return Event::None;
  }
  // No progress while data is outstanding: the receiver saw a gap or only duplicates. Go back, once per holdoff.
  if (ackOff == s.ackOff && s.sendOff > s.ackOff && nowMs - s.lastRewindMs >= kRewindHoldoffMs) {
    rewind(s, nowMs);
    return Event::Rewound;
  }
  return Event::None;
}

inline Event poll(Sender& s, uint32_t nowMs) {
  if (!s.active) return Event::None;
  if (nowMs - s.lastAckMs >= kInactivityMs) {
    s.active = false;
    return Event::Inactive;
  }
  if (s.sendOff > s.ackOff && nowMs - s.lastProgressMs >= kRetransmitMs) {
    rewind(s, nowMs);
    return Event::Rewound;
  }
  return Event::None;
}

}  // namespace dl2
