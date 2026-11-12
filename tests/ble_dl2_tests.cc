// 045 F2 / US2 T018 — BLE download protocol v2: the firmware's send window (xiao/include/ble_dl2.h) against a simulated BLE link
// that drops, duplicates and reorders notifications and drops ACKs. The receiver here implements the page's rules from
// specs/045-m2-tracking/contracts/ble-download-v2.md: accept in order, ignore duplicates by offset, ACK the highest contiguous
// byte every few chunks, at the end, and (rate-limited) on a gap or a duplicate.
#include "ble_dl2.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <deque>
#include <random>
#include <vector>

namespace {

struct Packet { uint32_t off; uint32_t len; };

// The page's rules (xiao/web/flight_logger.html mirrors these).
struct Receiver {
  std::vector<uint8_t> buf;
  uint32_t expected = 0, size = 0, sinceAck = 0, ackEvery = 4;
  int64_t lastAckMs = -1000;
  uint32_t dups = 0, gaps = 0;
  std::vector<uint32_t> acksOut;

  void sendAck(uint32_t nowMs) { acksOut.push_back(expected); lastAckMs = nowMs; sinceAck = 0; }
  void onChunk(const Packet& p, const std::vector<uint8_t>& src, uint32_t nowMs) {
    if (p.off == expected) {
      for (uint32_t i = 0; i < p.len; i++) buf[p.off + i] = src[p.off + i];
      expected += p.len;
      if (++sinceAck >= ackEvery || expected >= size) sendAck(nowMs);
    } else if (p.off < expected) {
      dups++;
      if (nowMs - lastAckMs >= 100) sendAck(nowMs);  // tell a rewound sender where we really are
    } else {
      gaps++;
      if (nowMs - lastAckMs >= 100) sendAck(nowMs);  // a gap: the sender should go back
    }
  }
};

struct Impairments { double drop, dup, swap, ackDrop; };

// Discrete-time link: a connection event every `intervalMs`, carrying up to `perEvent` notifications one way and the pending
// ACKs the other. Returns the simulated milliseconds to completion, or -1 on timeout.
int64_t simulate(const std::vector<uint8_t>& src, Receiver& rx, dl2::Sender& tx, const Impairments& imp, uint32_t seed,
                 uint32_t startMs, uint32_t stopAtBytes = 0, uint32_t intervalMs = 15, uint32_t perEvent = 4,
                 uint32_t limitMs = 600000) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::deque<Packet> air;
  for (uint32_t now = startMs; now < startMs + limitMs; now++) {
    uint32_t off, len;
    while (dl2::next(tx, &off, &len)) air.push_back({off, len});
    dl2::poll(tx, now);
    if (!tx.active && rx.expected >= rx.size) return now - startMs;
    if (now % intervalMs != 0) continue;
    // one connection event
    std::vector<Packet> delivered;
    for (uint32_t k = 0; k < perEvent && !air.empty(); k++) {
      Packet p = air.front();
      air.pop_front();
      if (u(rng) < imp.drop) continue;
      delivered.push_back(p);
      if (u(rng) < imp.dup) delivered.push_back(p);
    }
    for (size_t i = 0; i + 1 < delivered.size(); i++)
      if (u(rng) < imp.swap) std::swap(delivered[i], delivered[i + 1]);
    for (const auto& p : delivered) rx.onChunk(p, src, now);
    for (uint32_t a : rx.acksOut)
      if (u(rng) >= imp.ackDrop) dl2::onAck(tx, a, now);
    rx.acksOut.clear();
    if (stopAtBytes && rx.expected >= stopAtBytes) return -2;  // simulated disconnect
  }
  return -1;
}

std::vector<uint8_t> makeFile(uint32_t n) {
  std::vector<uint8_t> v(n);
  for (uint32_t i = 0; i < n; i++) v[i] = static_cast<uint8_t>((i * 2654435761u) >> 13);
  return v;
}

Receiver makeRx(uint32_t size, uint32_t window) {
  Receiver r;
  r.buf.assign(size, 0);
  r.size = size;
  r.ackEvery = window / 2 ? window / 2 : 1;
  return r;
}

}  // namespace

TEST(BleDl2, ChunkForMtu) {
  EXPECT_EQ(dl2::chunkForMtu(247), 240u);
  EXPECT_EQ(dl2::chunkForMtu(23), 16u);
  EXPECT_EQ(dl2::chunkForMtu(7), 0u);
}

TEST(BleDl2, MspPollSkippedOnlyWhileDownloadingOnTheGround) {
  EXPECT_TRUE(dl2::shouldSkipMspPoll(true, false));
  EXPECT_FALSE(dl2::shouldSkipMspPoll(true, true));   // arming wins the moment it happens
  EXPECT_FALSE(dl2::shouldSkipMspPoll(false, false));
  EXPECT_FALSE(dl2::shouldSkipMspPoll(false, true));
}

TEST(BleDl2, WindowBoundsChunksInFlight) {
  dl2::Sender s;
  dl2::start(s, 10000, 240, 0, 8, 0);
  uint32_t off, len, n = 0;
  while (dl2::next(s, &off, &len)) n++;
  EXPECT_EQ(n, 8u);
  EXPECT_EQ(dl2::onAck(s, 4 * 240, 1), dl2::Event::None);
  n = 0;
  while (dl2::next(s, &off, &len)) n++;
  EXPECT_EQ(n, 4u);  // the window slid by the four acknowledged chunks
}

TEST(BleDl2, CleanLinkIsByteIdenticalAndFast) {
  const auto src = makeFile(307968);
  dl2::Sender tx;
  dl2::start(tx, src.size(), 240, 0, 16, 0);
  Receiver rx = makeRx(src.size(), 16);
  const int64_t ms = simulate(src, rx, tx, {0, 0, 0, 0}, 1, 0);
  ASSERT_GT(ms, 0);
  EXPECT_EQ(rx.buf, src);
  EXPECT_EQ(tx.rewinds, 0u);
  // 4 notifications per 15 ms event ⇒ ~64 kB/s ideal; stop-and-wait over the same link would take > 9 s
  EXPECT_LT(ms, 9000) << "simulated ms: " << ms;
}

TEST(BleDl2, LossyDuplicatingReorderingLinkStillByteIdentical) {
  const auto src = makeFile(307968);
  for (uint32_t seed = 1; seed <= 20; seed++) {
    dl2::Sender tx;
    dl2::start(tx, src.size(), 240, 0, 16, 0);
    Receiver rx = makeRx(src.size(), 16);
    const int64_t ms = simulate(src, rx, tx, {0.05, 0.05, 0.10, 0.05}, seed, 0);
    ASSERT_GT(ms, 0) << "seed " << seed << " did not complete";
    EXPECT_EQ(rx.buf, src) << "seed " << seed;
  }
}

TEST(BleDl2, ResumeFromAnyCutPointIsByteIdentical) {
  const auto src = makeFile(65536 + 100);  // not a multiple of the chunk
  for (uint32_t cut : {1u, 240u, 241u, 30000u, 65000u}) {
    Receiver rx = makeRx(src.size(), 16);
    dl2::Sender tx;
    dl2::start(tx, src.size(), 240, 0, 16, 0);
    ASSERT_EQ(simulate(src, rx, tx, {0.02, 0.02, 0.05, 0.02}, cut, 0, cut), -2) << "cut " << cut;
    // reconnect: the page keeps its buffer and asks to continue from the highest contiguous byte (DL2:<file>:<offset>:<window>)
    dl2::Sender tx2;
    dl2::start(tx2, src.size(), 240, rx.expected, 16, 100000);
    // >= 0: a cut near the end may leave the receiver complete already, so the resumed sender is done at once
    ASSERT_GE(simulate(src, rx, tx2, {0.02, 0.02, 0.05, 0.02}, cut + 7, 100000), 0) << "cut " << cut;
    EXPECT_EQ(rx.buf, src) << "cut " << cut;
  }
}

TEST(BleDl2, InactivityAbandonsTheTransfer) {
  dl2::Sender s;
  dl2::start(s, 10000, 240, 0, 8, 1000);
  uint32_t off, len;
  while (dl2::next(s, &off, &len)) {}
  EXPECT_EQ(dl2::poll(s, 1000 + dl2::kRetransmitMs), dl2::Event::Rewound);  // stalled window: go back to the ACK
  EXPECT_NE(dl2::poll(s, 1000 + dl2::kInactivityMs - 1), dl2::Event::Inactive);
  EXPECT_TRUE(s.active);
  EXPECT_EQ(dl2::poll(s, 1000 + dl2::kInactivityMs), dl2::Event::Inactive);
  EXPECT_FALSE(s.active);
}

TEST(BleDl2, ResumeAtTheEndIsAlreadyDone) {
  dl2::Sender s;
  dl2::start(s, 1000, 240, 1000, 8, 0);
  uint32_t off, len;
  EXPECT_FALSE(s.active);
  EXPECT_FALSE(dl2::next(s, &off, &len));
}
