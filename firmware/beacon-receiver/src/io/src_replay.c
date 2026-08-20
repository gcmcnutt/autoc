/* src_replay.c — T018: the replay frame source, behind the same FrameSource vtable as libcamera.
 *
 * Determinism is the deliverable: two opens of one file yield byte-identical FrameView sequences,
 * because everything comes off the disk in file order with no clock, no thread and no state that
 * survives between opens. (That sentence is tested — tests/unit/test_replay.c.)
 */
#define _GNU_SOURCE
#include "src_replay.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    FILE              *f;
    BcnContainerHeader hdr;
    BcnFrameHeader     last;       /* header of the most recent frame returned                          */
    uint8_t           *payload;    /* one frame, allocated at open (R6: never in next())                */
    uint32_t           payload_bytes;
    uint32_t           prev_seq;
    int                any_frame;  /* for gap accounting on the first frame                             */
} ReplayCtx;

static FrameStatus replay_next(FrameSource *self, FrameView *out)
{
    ReplayCtx *c = self->ctx;
    uint8_t hdr[BCN_FRAME_HEADER_BYTES];
    size_t n;

    n = fread(hdr, 1, sizeof hdr, c->f);
    if (n == 0 && feof(c->f)) return FRAME_END;
    if (n != sizeof hdr) {
        /* A partial header is a truncated file. Say where, then stop — never yield garbage. */
        fprintf(stderr, "replay: truncated frame header at offset %ld\n", ftell(c->f));
        return FRAME_ERROR;
    }
    if (bcn_frame_header_decode(hdr, &c->last) != 0) return FRAME_ERROR;
    if (c->last.record_bytes != BCN_FRAME_HEADER_BYTES + c->payload_bytes) {
        fprintf(stderr, "replay: frame record_bytes %u does not match geometry (%u expected) — "
                        "corrupt or mixed file\n",
                c->last.record_bytes, BCN_FRAME_HEADER_BYTES + c->payload_bytes);
        return FRAME_ERROR;
    }
    if (fread(c->payload, 1, c->payload_bytes, c->f) != c->payload_bytes) {
        fprintf(stderr, "replay: truncated payload for seq %u\n", c->last.seq);
        return FRAME_ERROR;
    }

    memset(out, 0, sizeof *out);
    out->data = c->payload;
    out->stride = c->hdr.width;   /* stored compact: stride == width, always */
    out->w = c->hdr.width;
    out->h = c->hdr.height;
    out->seq = c->last.seq;
    out->t_us = c->last.t_us;
    out->exposure_us = c->last.exposure_us;
    out->gain_q8 = c->last.gain_q8;

    c->prev_seq = c->last.seq;
    c->any_frame = 1;
    return FRAME_OK;
}

static void replay_close(FrameSource *self)
{
    ReplayCtx *c = self->ctx;
    if (c) {
        if (c->f) fclose(c->f);
        free(c->payload);
        free(c);
    }
    free(self);
}

int bcn_replay_open(FrameSource **out, const char *path, char *err, size_t err_len)
{
    FrameSource *src;
    ReplayCtx *c;
    uint8_t hdr[BCN_CONTAINER_HEADER_BYTES];

    src = calloc(1, sizeof *src);
    c = calloc(1, sizeof *c);
    if (!src || !c) { snprintf(err, err_len, "replay: out of memory"); free(src); free(c); return -1; }

    c->f = fopen(path, "rb");
    if (!c->f) {
        snprintf(err, err_len, "replay: cannot open %s: %s", path, strerror(errno));
        free(src); free(c); return -1;
    }
    if (fread(hdr, 1, sizeof hdr, c->f) != sizeof hdr) {
        snprintf(err, err_len, "replay: %s is shorter than a container header", path);
        fclose(c->f); free(src); free(c); return -1;
    }
    if (bcn_container_header_decode(hdr, &c->hdr) != 0 ||
        bcn_container_check(&c->hdr, err, err_len) != 0) {
        /* bcn_container_check already wrote the fail-loud message naming both versions */
        fclose(c->f); free(src); free(c); return -1;
    }

    c->payload_bytes = (uint32_t)c->hdr.width * c->hdr.height;
    c->payload = malloc(c->payload_bytes);
    if (!c->payload) {
        snprintf(err, err_len, "replay: cannot allocate a %u-byte frame", c->payload_bytes);
        fclose(c->f); free(src); free(c); return -1;
    }

    src->ctx = c;
    src->next = replay_next;
    src->close = replay_close;
    src->nominal_fps = c->hdr.nominal_fps;
    *out = src;
    return 0;
}

const BcnContainerHeader *bcn_replay_header(const FrameSource *src)
{
    return &((const ReplayCtx *)src->ctx)->hdr;
}

const BcnFrameHeader *bcn_replay_last_frame_header(const FrameSource *src)
{
    return &((const ReplayCtx *)src->ctx)->last;
}
