/* src_replay.h — the replay frame source (T018; the other half of the parity contract).
 *
 * Yields the identical FrameView sequence on every run over the same file. That determinism is not a
 * nicety — it is the property golden-vector replay tests (T043) and the oracle stand on.
 */
#ifndef BEACON_SRC_REPLAY_H
#define BEACON_SRC_REPLAY_H

#include "container.h"
#include "frame.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Open a .bcnr recording as a FrameSource. Fails loudly on magic/version mismatch naming both versions
 * (exit 2 at the CLI). On success *out is ready for next()/close(); close() frees everything. */
int bcn_replay_open(FrameSource **out, const char *path, char *err, size_t err_len);

/* The container header, for tools that need geometry/mode (oracle, score). Valid until close(). */
const BcnContainerHeader *bcn_replay_header(const FrameSource *src);

/* The frame header of the MOST RECENT frame returned by next() — replay-only metadata (flags, dual
 * clocks) that FrameView deliberately does not carry. */
const BcnFrameHeader *bcn_replay_last_frame_header(const FrameSource *src);

#ifdef __cplusplus
}
#endif
#endif /* BEACON_SRC_REPLAY_H */
