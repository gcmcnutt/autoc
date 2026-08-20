/* sink_record.h — the raw recorder (T017/T019/T020/T024; contracts/recording-container.md, R9).
 *
 * io/ may use POSIX; core/ may not. The recorder is deliberately a LIBRARY with the mode machine inside
 * it, so the unit tests exercise continuous, ring and burst with synthetic frames on a machine with no
 * camera — beacon_record (app/) is argument parsing around this.
 *
 * All three modes share ONE write path (T019): ring and burst differ only in WHICH frames reach it, so a
 * frame that survives selection is encoded by the same code bytes in every mode.
 */
#ifndef BEACON_SINK_RECORD_H
#define BEACON_SINK_RECORD_H

#include "config.h"
#include "container.h"
#include "frame.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct BcnRecorder BcnRecorder;

typedef struct {
    uint64_t frames_offered;   /* every frame pushed                                                    */
    uint64_t frames_written;   /* frames that reached the file                                          */
    uint64_t frames_dropped;   /* frames the mode selected but the queue could not take (IO too slow).
                                * NEVER silent (no-silent-caps): a nonzero count here is the recorder
                                * saying "the sink cannot keep up at this duty" in numbers.             */
    uint64_t bytes_written;    /* payload + headers, logical (pre-padding)                              */
    uint64_t ring_dumps;       /* ring mode: completed dumps                                            */
    int      o_direct;         /* 1 if the file is actually open O_DIRECT (tmpfs refuses it — that is
                                * an IO strategy change only; the bytes are identical either way)       */
} BcnRecorderStats;

/* Open a recorder per cfg's [record]+[camera] sections. mode/path may be overridden by the CLI (NULL =
 * take the config value). Allocates everything it will ever need — ring slots included — here, never in
 * push (R6: no allocation in the loop). */
int bcn_recorder_open(BcnRecorder **out, const BcnConfig *cfg,
                      const char *mode_override, const char *path_override,
                      char *err, size_t err_len);

/* Offer one frame. Applies the mode machine:
 *   continuous — every frame is written.
 *   ring       — frame is copied into the RAM ring (overwriting the oldest); nothing reaches disk until
 *                bcn_recorder_trigger(), which drains the ring through the continuous path with the
 *                first drained frame flagged TRIGGER_DUMP.
 *   burst      — frames [k, k+burst_frames) of every burst_every are written, first one flagged
 *                BURST_START; the rest are dropped (seq gaps are EXPLICIT AND LEGAL — the contract).
 * Dual-clock fields are written zero-when-absent (T024/R10): this bench has no FC, and a reader must
 * never mistake 0 for a valid INAV time. */
int bcn_recorder_push(BcnRecorder *r, const FrameView *fv, char *err, size_t err_len);

/* Ring mode: dump the ring's current contents now. No-op (success) in other modes. */
int bcn_recorder_trigger(BcnRecorder *r, char *err, size_t err_len);

/* Stats are updated by BOTH threads: offered/dropped by the capture side, written/bytes by the writer.
 * For a point-in-time coherent view (tests, end-of-run reporting), drain first. */
void bcn_recorder_stats(const BcnRecorder *r, BcnRecorderStats *out);

/* Block until every queued frame has reached the file. Capture may keep pushing while this waits — it
 * returns when the queue TOUCHES empty. Meant for tests and orderly shutdown, not the hot path. */
void bcn_recorder_drain(BcnRecorder *r);

/* Flush, finalise the header (start_t_us patches in), ftruncate away alignment padding, free. In ring
 * mode an UNTRIGGERED ring is dumped at close — a bench run that ends with an empty file because nobody
 * pressed the trigger is a wasted session, not a feature. */
int bcn_recorder_close(BcnRecorder *r, char *err, size_t err_len);

#ifdef __cplusplus
}
#endif
#endif /* BEACON_SINK_RECORD_H */
