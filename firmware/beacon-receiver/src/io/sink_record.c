/* sink_record.c — T017 continuous writer, T019 RAM ring, T020 burst, T024 zero-when-absent clocks.
 *
 * R9's shape: preallocated file, O_DIRECT, large aligned writes, so IO jitter stays off the §11.1
 * deadline. O_DIRECT has two sharp edges handled here rather than at every call site:
 *   - tmpfs (and some filesystems) refuse it at open — fall back to buffered, RECORDED in stats.o_direct
 *     rather than silently. The bytes on disk are identical either way; only the IO strategy differs.
 *   - it requires aligned buffers, offsets and lengths — so all writes go through one aligned bounce
 *     buffer, and close() truncates away the final block's padding to the logical length.
 *
 * The header is emitted lazily on the FIRST written frame, so start_t_us is that frame's real timestamp
 * instead of a value that would need patching at close (which O_DIRECT makes awkward: block 0 has long
 * since left the bounce buffer).
 */
#define _GNU_SOURCE
#include "sink_record.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define ALIGN        4096u
#define BOUNCE_BYTES (4u * 1024u * 1024u)   /* 4 MiB: "large aligned writes" per R9 */
#define PREALLOC     (256ull * 1024u * 1024u)
/* The push->writer queue (R6). Sized for the sink's REAL failure mode: capture-to-SD showed multi-second
 * write stalls, and at the burst duty of ~11 MB/s a 3 s stall needs ~34 MB of slack. 48 MiB rides it out;
 * anything longer drops frames WITH A COUNT rather than blocking capture.
 * (Found the hard way, 2026-08-19: the first, single-threaded recorder lost ~22 % of frames even to
 * tmpfs — the 4 MiB flush pwrite stalls capture ~40 ms = 11 frames at 276 fps, and the losses were
 * invisible because this pipeline's buffer metadata sequence counts DELIVERED frames, so "0 seq gaps"
 * proves nothing about drops. Timestamps are the only honest witness: bcnr_info's dt stats.) */
#define QUEUE_BYTES  (48u * 1024u * 1024u)

typedef struct {           /* one ring slot: frame metadata; payload lives in ring_payload */
    uint32_t seq;
    uint64_t t_us;
    uint32_t exposure_us;
    uint16_t gain_q8;
} RingSlot;

struct BcnRecorder {
    int      fd;
    uint32_t mode;                 /* BCN_MODE_*                                                        */
    uint16_t w, h;
    uint32_t payload_bytes;
    uint32_t nominal_fps;
    uint64_t build_id, config_hash;
    char     path[BCN_PATH_MAX];

    int      header_written;       /* emitted with the first written frame's t_us as start_t_us         */

    uint8_t *bounce;               /* ALIGN-aligned accumulation buffer                                 */
    uint32_t bounce_fill;
    uint64_t file_off;             /* aligned bytes already on disk                                     */
    uint64_t logical_bytes;        /* true content length; the file is truncated to this at close       */
    uint64_t prealloc_end;

    uint32_t burst_frames, burst_every;
    uint64_t offered;              /* index into the burst schedule                                     */

    RingSlot *slots;               /* ring mode only                                                    */
    uint8_t  *ring_payload;
    uint32_t  cap, head, count;

    /* writer thread (R6): push() only memcpys into this queue; ALL file IO happens on the thread.
     * Slot layout: RingSlot metadata + flags, payload compacted. SPSC: push side is the capture thread,
     * pop side is the writer; the mutex only guards the indices (memcpys happen outside it). */
    pthread_t       wr;
    pthread_mutex_t q_mtx;
    pthread_cond_t  q_cv;
    RingSlot  *q_slots;            /* q_cap entries                                                     */
    uint16_t  *q_flags;
    uint8_t   *q_payload;          /* q_cap * payload_bytes                                             */
    uint32_t   q_cap, q_head, q_tail, q_count;
    int        q_stop;             /* close() sets it; writer drains then exits                         */
    int        wr_started;
    int        wr_err;             /* sticky first IO error from the writer thread                      */
    char       wr_errmsg[BCN_ERR_MAX];

    BcnRecorderStats st;
};

/* ---- the aligned write path ------------------------------------------------------------------------ */

static int flush_bounce(BcnRecorder *r, int final, char *err, size_t err_len)
{
    uint32_t n = r->bounce_fill;
    uint32_t keep = 0;
    ssize_t wr;

    if (final) {
        uint32_t padded = (n + ALIGN - 1u) & ~(ALIGN - 1u);
        memset(r->bounce + n, 0, padded - n);
        n = padded;
    } else {
        keep = n & (ALIGN - 1u);   /* unaligned tail stays for the next fill */
        n -= keep;
    }
    if (n == 0) return 0;

    /* Stay inside the preallocation so O_DIRECT never extends the file mid-run (that is the jitter R9
     * exists to avoid). Failure is benign — the write below extends the file the slow way. */
    while (r->file_off + n > r->prealloc_end) {
        if (posix_fallocate(r->fd, (off_t)r->prealloc_end, (off_t)PREALLOC) != 0) break;
        r->prealloc_end += PREALLOC;
    }

    wr = pwrite(r->fd, r->bounce, n, (off_t)r->file_off);
    if (wr != (ssize_t)n) {
        snprintf(err, err_len, "recorder: write to %s at offset %llu failed: %s",
                 r->path, (unsigned long long)r->file_off, strerror(errno));
        return -1;
    }
    r->file_off += n;
    if (keep) memmove(r->bounce, r->bounce + n, keep);
    r->bounce_fill = keep;
    return 0;
}

static int emit_bytes(BcnRecorder *r, const uint8_t *p, uint32_t n, char *err, size_t err_len)
{
    while (n) {
        uint32_t room = BOUNCE_BYTES - r->bounce_fill;
        uint32_t take = n < room ? n : room;
        memcpy(r->bounce + r->bounce_fill, p, take);
        r->bounce_fill += take;
        p += take; n -= take;
        r->logical_bytes += take;
        if (r->bounce_fill == BOUNCE_BYTES && flush_bounce(r, 0, err, err_len) != 0) return -1;
    }
    return 0;
}

static int emit_header(BcnRecorder *r, uint64_t start_t_us, char *err, size_t err_len)
{
    BcnContainerHeader h;
    uint8_t buf[BCN_CONTAINER_HEADER_BYTES];
    memset(&h, 0, sizeof h);
    h.magic = BCN_CONTAINER_MAGIC;
    h.format_version = (uint16_t)BCN_CONTAINER_FORMAT_VERSION;
    h.header_bytes = (uint16_t)BCN_CONTAINER_HEADER_BYTES;
    h.width = r->w; h.height = r->h;
    h.bits_per_pixel = 8u;
    h.sensor_mode = (uint16_t)((r->w == 640u && r->h == 200u) ? 1u : 0u);
    h.nominal_fps = r->nominal_fps;
    h.start_t_us = start_t_us;
    h.build_id = r->build_id;
    h.config_hash = r->config_hash;
    h.mode = r->mode;
    bcn_container_header_encode(&h, buf);
    r->header_written = 1;
    return emit_bytes(r, buf, sizeof buf, err, err_len);
}

/* THE one write path all three modes share (T019). `stride` lets a live DMA frame with row padding go
 * straight to the bounce buffer without an intermediate compaction copy. */
static int write_frame(BcnRecorder *r, uint32_t seq, uint64_t t_us, uint32_t exposure_us,
                       uint16_t gain_q8, uint16_t flags,
                       const uint8_t *payload, uint32_t stride,
                       char *err, size_t err_len)
{
    BcnFrameHeader fh;
    uint8_t buf[BCN_FRAME_HEADER_BYTES];

    if (!r->header_written && emit_header(r, t_us, err, err_len) != 0) return -1;

    memset(&fh, 0, sizeof fh);
    fh.record_bytes = BCN_FRAME_HEADER_BYTES + r->payload_bytes;
    fh.seq = seq; fh.t_us = t_us;
    fh.exposure_us = exposure_us; fh.gain_q8 = gain_q8; fh.flags = flags;
    /* T024 / R10: this bench has no flight controller. The dual-clock fields are ZERO — the contract's
     * explicit "absent" value — never anything a reader could mistake for a real INAV time. When the MSP
     * link lands (spec §7.1.2) the recorder samples it and fills these; the wire shape is ready now. */
    fh.inav_t_us = 0u;         /* default-ok: 0 is the contract's "no INAV clock yet" */
    fh.inav_read_age_us = 0u;  /* default-ok: meaningless while inav_t_us == 0        */
    fh.gps_time_ms = 0u;       /* default-ok: 0 is the contract's "no GPS time"       */
    bcn_frame_header_encode(&fh, buf);

    if (emit_bytes(r, buf, sizeof buf, err, err_len) != 0) return -1;
    if (stride == r->w) {
        if (emit_bytes(r, payload, r->payload_bytes, err, err_len) != 0) return -1;
    } else {
        uint32_t y;
        for (y = 0; y < r->h; y++)
            if (emit_bytes(r, payload + (size_t)y * stride, r->w, err, err_len) != 0) return -1;
    }
    r->st.frames_written++;
    r->st.bytes_written = r->logical_bytes;
    return 0;
}

/* ---- the writer thread (R6) -------------------------------------------------------------------------- */

static void *writer_main(void *arg)
{
    BcnRecorder *r = arg;
    char err[BCN_ERR_MAX];

    for (;;) {
        uint32_t k;
        RingSlot meta;
        uint16_t flags;

        pthread_mutex_lock(&r->q_mtx);
        while (r->q_count == 0u && !r->q_stop)
            pthread_cond_wait(&r->q_cv, &r->q_mtx);
        if (r->q_count == 0u && r->q_stop) {
            pthread_mutex_unlock(&r->q_mtx);
            return NULL;
        }
        k = r->q_tail;
        meta = r->q_slots[k];
        flags = r->q_flags[k];
        pthread_mutex_unlock(&r->q_mtx);

        /* The payload memcpy'd by push() is stable until we advance q_tail — write it now. */
        if (!r->wr_err &&
            write_frame(r, meta.seq, meta.t_us, meta.exposure_us, meta.gain_q8, flags,
                        r->q_payload + (size_t)k * r->payload_bytes, r->w, err, sizeof err) != 0) {
            r->wr_err = 1;                       /* sticky: close() reports it; capture keeps counting */
            snprintf(r->wr_errmsg, sizeof r->wr_errmsg, "%s", err);
        }

        pthread_mutex_lock(&r->q_mtx);
        r->q_tail = (r->q_tail + 1u) % r->q_cap;
        r->q_count--;
        pthread_cond_signal(&r->q_cv);           /* wake a trigger() waiting for space */
        pthread_mutex_unlock(&r->q_mtx);
    }
}

/* Enqueue one frame for the writer. block=0 (capture path): a full queue DROPS the frame, counted —
 * capture must never wait on IO. block=1 (ring dump, close): waits for space, because those frames are
 * the entire point and their producer is not the camera. */
static int enqueue_frame(BcnRecorder *r, uint32_t seq, uint64_t t_us, uint32_t exposure_us,
                         uint16_t gain_q8, uint16_t flags,
                         const uint8_t *payload, uint32_t stride, int block)
{
    uint32_t k;
    pthread_mutex_lock(&r->q_mtx);
    while (r->q_count == r->q_cap && block && !r->wr_err)
        pthread_cond_wait(&r->q_cv, &r->q_mtx);
    if (r->q_count == r->q_cap) {
        pthread_mutex_unlock(&r->q_mtx);
        r->st.frames_dropped++;
        return 0;                                /* counted, never silent — see stats comment */
    }
    k = r->q_head;
    pthread_mutex_unlock(&r->q_mtx);

    r->q_slots[k].seq = seq;
    r->q_slots[k].t_us = t_us;
    r->q_slots[k].exposure_us = exposure_us;
    r->q_slots[k].gain_q8 = gain_q8;
    r->q_flags[k] = flags;
    if (stride == r->w) {
        memcpy(r->q_payload + (size_t)k * r->payload_bytes, payload, r->payload_bytes);
    } else {
        uint32_t y;
        uint8_t *dst = r->q_payload + (size_t)k * r->payload_bytes;
        for (y = 0; y < r->h; y++)
            memcpy(dst + (size_t)y * r->w, payload + (size_t)y * stride, r->w);
    }

    pthread_mutex_lock(&r->q_mtx);
    r->q_head = (r->q_head + 1u) % r->q_cap;
    r->q_count++;
    pthread_cond_signal(&r->q_cv);
    pthread_mutex_unlock(&r->q_mtx);
    return 0;
}

/* ---- lifecycle -------------------------------------------------------------------------------------- */

int bcn_recorder_open(BcnRecorder **out, const BcnConfig *cfg,
                      const char *mode_override, const char *path_override,
                      char *err, size_t err_len)
{
    BcnRecorder *r;
    const char *mode = mode_override ? mode_override : cfg->record_mode;
    const char *path = path_override ? path_override : cfg->record_path;
    unsigned w = 0, h = 0;

    if (sscanf(cfg->camera_mode, "%ux%u", &w, &h) != 2 || w == 0 || h == 0) {
        snprintf(err, err_len, "recorder: [camera] mode \"%s\" is not WxH", cfg->camera_mode);
        return -1;
    }
    r = calloc(1, sizeof *r);
    if (!r) { snprintf(err, err_len, "recorder: out of memory"); return -1; }
    r->fd = -1;
    r->w = (uint16_t)w; r->h = (uint16_t)h;
    r->payload_bytes = (uint32_t)w * (uint32_t)h;
    r->nominal_fps = cfg->fps;
    r->build_id = cfg->build_id; r->config_hash = cfg->config_hash;
    r->burst_frames = cfg->burst_frames; r->burst_every = cfg->burst_every;
    snprintf(r->path, sizeof r->path, "%s", path);

    if      (strcmp(mode, "continuous") == 0) r->mode = BCN_MODE_CONTINUOUS;
    else if (strcmp(mode, "ring")       == 0) r->mode = BCN_MODE_RING;
    else if (strcmp(mode, "burst")      == 0) r->mode = BCN_MODE_BURST;
    else {
        snprintf(err, err_len, "recorder: mode \"%s\" is not continuous|ring|burst", mode);
        free(r); return -1;
    }
    if (r->mode == BCN_MODE_BURST && (r->burst_frames == 0u || r->burst_every <= r->burst_frames)) {
        snprintf(err, err_len, "recorder: burst_frames (%u) must be >0 and < burst_every (%u)",
                 r->burst_frames, r->burst_every);
        free(r); return -1;
    }

    /* O_DIRECT first; tmpfs refuses it at open (EINVAL) — fall back, recorded in stats, never silent. */
    r->fd = open(r->path, O_WRONLY | O_CREAT | O_TRUNC | O_DIRECT, 0644);
    if (r->fd >= 0) r->st.o_direct = 1;
    else if (errno == EINVAL) r->fd = open(r->path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (r->fd < 0) {
        snprintf(err, err_len, "recorder: cannot open %s: %s", r->path, strerror(errno));
        free(r); return -1;
    }
    if (posix_fallocate(r->fd, 0, (off_t)PREALLOC) == 0) r->prealloc_end = PREALLOC;

    if (posix_memalign((void **)&r->bounce, ALIGN, BOUNCE_BYTES) == 0) memset(r->bounce, 0, BOUNCE_BYTES);
    else {
        r->bounce = NULL;
    }
    if (!r->bounce) {
        snprintf(err, err_len, "recorder: cannot allocate the %u MiB bounce buffer", BOUNCE_BYTES >> 20);
        close(r->fd); free(r); return -1;
    }

    if (r->mode == BCN_MODE_RING) {
        /* Everything the ring will ever hold is allocated HERE, never in push (R6). */
        r->cap = cfg->ring_seconds * cfg->fps;
        if (r->cap == 0u) {
            snprintf(err, err_len, "recorder: ring_seconds * fps = 0 slots — nothing to ring");
            free(r->bounce); close(r->fd); free(r); return -1;
        }
        r->slots = calloc(r->cap, sizeof *r->slots);
        r->ring_payload = malloc((size_t)r->cap * r->payload_bytes);
        if (r->ring_payload) memset(r->ring_payload, 0, (size_t)r->cap * r->payload_bytes);
        if (!r->slots || !r->ring_payload) {
            snprintf(err, err_len, "recorder: ring of %u frames x %u B = %.0f MB exceeds memory "
                     "(shrink ring_seconds — the 3A+ has 512 MB total)",
                     r->cap, r->payload_bytes, (double)r->cap * r->payload_bytes / 1e6);
            free(r->slots); free(r->ring_payload); free(r->bounce); close(r->fd); free(r);
            return -1;
        }
    }
    /* The writer-thread queue: sized in frames from QUEUE_BYTES, minimum 4. */
    r->q_cap = QUEUE_BYTES / r->payload_bytes;
    if (r->q_cap < 4u) r->q_cap = 4u;
    r->q_slots = calloc(r->q_cap, sizeof *r->q_slots);
    r->q_flags = calloc(r->q_cap, sizeof *r->q_flags);
    r->q_payload = malloc((size_t)r->q_cap * r->payload_bytes);
    /* Prefault the queue NOW (and the bounce, below): an untouched 48 MiB allocation page-faults ~64
     * times per 256 KB frame copy (~2-3 ms at A53 fault cost) — measured as the difference between 216
     * and ~276 fps capture, 2026-08-19. mlockall would do this too, but it needs privileges the bench
     * user does not have; memset needs none. */
    if (r->q_payload) memset(r->q_payload, 0, (size_t)r->q_cap * r->payload_bytes);
    if (!r->q_slots || !r->q_flags || !r->q_payload) {
        snprintf(err, err_len, "recorder: cannot allocate the %u MiB writer queue", QUEUE_BYTES >> 20);
        free(r->q_slots); free(r->q_flags); free(r->q_payload);
        free(r->slots); free(r->ring_payload); free(r->bounce); close(r->fd); free(r);
        return -1;
    }
    pthread_mutex_init(&r->q_mtx, NULL);
    pthread_cond_init(&r->q_cv, NULL);
    if (pthread_create(&r->wr, NULL, writer_main, r) != 0) {
        snprintf(err, err_len, "recorder: cannot start the writer thread");
        free(r->q_slots); free(r->q_flags); free(r->q_payload);
        free(r->slots); free(r->ring_payload); free(r->bounce); close(r->fd); free(r);
        return -1;
    }
    r->wr_started = 1;

    *out = r;
    return 0;
}

int bcn_recorder_push(BcnRecorder *r, const FrameView *fv, char *err, size_t err_len)
{
    r->st.frames_offered++;
    if (fv->w != r->w || fv->h != r->h) {
        snprintf(err, err_len, "recorder: frame %ux%u does not match configured %ux%u",
                 fv->w, fv->h, r->w, r->h);
        return -1;
    }

    switch (r->mode) {
    case BCN_MODE_CONTINUOUS:
        return enqueue_frame(r, fv->seq, fv->t_us, fv->exposure_us, fv->gain_q8, 0u,
                             fv->data, fv->stride, 0 /* capture never blocks on IO */);

    case BCN_MODE_RING: {
        RingSlot *s = &r->slots[r->head];
        uint8_t *dst = r->ring_payload + (size_t)r->head * r->payload_bytes;
        if (fv->stride == fv->w) {
            memcpy(dst, fv->data, r->payload_bytes);
        } else {
            uint32_t y;
            for (y = 0; y < fv->h; y++)
                memcpy(dst + (size_t)y * fv->w, fv->data + (size_t)y * fv->stride, fv->w);
        }
        s->seq = fv->seq; s->t_us = fv->t_us;
        s->exposure_us = fv->exposure_us; s->gain_q8 = fv->gain_q8;
        r->head = (r->head + 1u) % r->cap;
        if (r->count < r->cap) r->count++;
        return 0;
    }

    case BCN_MODE_BURST: {
        uint64_t phase = r->offered % r->burst_every;
        r->offered++;
        if (phase >= r->burst_frames) return 0;   /* deselected; the seq gap is the explicit evidence */
        return enqueue_frame(r, fv->seq, fv->t_us, fv->exposure_us, fv->gain_q8,
                             (phase == 0u) ? (uint16_t)BCN_FR_BURST_START : 0u,
                             fv->data, fv->stride, 0 /* capture never blocks on IO */);
    }
    }
    snprintf(err, err_len, "recorder: unreachable mode %u", r->mode);
    return -1;
}

int bcn_recorder_trigger(BcnRecorder *r, char *err, size_t err_len)
{
    uint32_t i, start;
    if (r->mode != BCN_MODE_RING || r->count == 0u) return 0;
    /* Oldest-first: when the ring has wrapped, the oldest slot is at head; before that, at 0. */
    start = (r->count == r->cap) ? r->head : 0u;
    /* Blocking enqueue: the dump's producer is the RAM ring, not the camera, so waiting for queue space
     * is correct here. Live consequence, stated plainly: a 71 MB ring at ~12 MB/s SD takes ~6 s, and the
     * capture loop calling this stalls for that long — camera frames during the dump are lost (they show
     * in the next segment's seq gap). Bench-acceptable; the flight host records continuous instead. */
    for (i = 0; i < r->count; i++) {
        uint32_t k = (start + i) % r->cap;
        const RingSlot *s = &r->slots[k];
        (void)err; (void)err_len;
        enqueue_frame(r, s->seq, s->t_us, s->exposure_us, s->gain_q8,
                      (i == 0u) ? (uint16_t)BCN_FR_TRIGGER_DUMP : 0u,
                      r->ring_payload + (size_t)k * r->payload_bytes, r->w, 1 /* block */);
        if (r->wr_err) { snprintf(err, err_len, "%s", r->wr_errmsg); return -1; }
    }
    r->count = 0u;                /* drained; capture continues into an empty ring */
    r->head = 0u;
    r->st.ring_dumps++;
    return 0;
}

void bcn_recorder_stats(const BcnRecorder *r, BcnRecorderStats *out) { *out = r->st; }

void bcn_recorder_drain(BcnRecorder *r)
{
    pthread_mutex_lock(&r->q_mtx);
    while (r->q_count > 0u && !r->wr_err)
        pthread_cond_wait(&r->q_cv, &r->q_mtx);
    pthread_mutex_unlock(&r->q_mtx);
}

int bcn_recorder_close(BcnRecorder *r, char *err, size_t err_len)
{
    int rc = 0;

    /* An untriggered ring is dumped rather than discarded — an empty file is a wasted bench session. */
    if (r->mode == BCN_MODE_RING && r->count > 0u)
        rc = bcn_recorder_trigger(r, err, err_len);

    /* Drain and stop the writer: everything queued gets written before the file is finalised. */
    if (r->wr_started) {
        pthread_mutex_lock(&r->q_mtx);
        r->q_stop = 1;
        pthread_cond_broadcast(&r->q_cv);
        pthread_mutex_unlock(&r->q_mtx);
        pthread_join(r->wr, NULL);
        if (r->wr_err && rc == 0) {
            snprintf(err, err_len, "%s", r->wr_errmsg);
            rc = -1;
        }
    }

    if (rc == 0 && !r->header_written)
        rc = emit_header(r, 0u /* default-ok: no frame ever arrived, so no start time exists */,
                         err, err_len);
    if (rc == 0) rc = flush_bounce(r, 1, err, err_len);
    if (rc == 0 && ftruncate(r->fd, (off_t)r->logical_bytes) != 0) {
        snprintf(err, err_len, "recorder: ftruncate(%s, %llu): %s",
                 r->path, (unsigned long long)r->logical_bytes, strerror(errno));
        rc = -1;
    }
    if (r->fd >= 0) {
        if (fsync(r->fd) != 0 && rc == 0) {
            snprintf(err, err_len, "recorder: fsync(%s): %s", r->path, strerror(errno));
            rc = -1;
        }
        close(r->fd);
    }
    free(r->bounce); free(r->slots); free(r->ring_payload);
    free(r->q_slots); free(r->q_flags); free(r->q_payload);
    free(r);
    return rc;
}
