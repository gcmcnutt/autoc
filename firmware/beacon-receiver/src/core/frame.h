/* frame.h — the ONLY thing core/ knows about a camera (data-model.md §1, research.md R3).
 *
 * core/ is C11 with zero dependencies. That is load-bearing, not aesthetic (plan.md §16.1): it is why the
 * identical code runs live on the Pi, in replay on the dev box, and cross-compiled from WSL2 with nothing
 * but the distro cross-gcc.
 *
 * THE PARITY CONTRACT: identical FrameView sequences MUST produce identical outputs whether they came from
 * libcamera or from a replay file. Everything else in this feature is downstream of that sentence.
 */
#ifndef BEACON_FRAME_H
#define BEACON_FRAME_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One frame, borrowed. `data` is NOT owned and is valid only for the duration of the call it is passed to;
 * a consumer that needs it later must copy. This is what lets the live path hand over a DMA buffer and the
 * replay path hand over a slice of a mapped file with no allocation on either side. */
typedef struct {
    const uint8_t *data;        /* borrowed pixels, 8bpp                                                */
    uint16_t       stride;      /* bytes per row; >= w                                                  */
    uint16_t       w, h;        /* 640x400 or 640x200                                                   */
    uint32_t       seq;         /* monotonic frame counter; a GAP IS AN ERROR, not a warning            */
    uint64_t       t_us;        /* SOURCE-SUPPLIED. core/ never calls clock_gettime (R3) — a wall-clock
                                 * read here would break replay parity, silently and irreproducibly.    */
    uint32_t       exposure_us; /* libcamera metadata; normalises the sample series (spec §4)           */
    uint16_t       gain_q8;     /* ditto; 8 fractional bits                                             */
    uint16_t       _pad;        /* explicit — never let the compiler choose (plan.md §Contracts)        */
} FrameView;

/* Result of pulling a frame. A source distinguishes "finished cleanly" from "broke", because replay hitting
 * EOF and libcamera dropping the pipeline must not look alike to the caller. */
typedef enum {
    FRAME_OK    = 0,
    FRAME_END   = 1,   /* clean end of stream (replay reached EOF)                                      */
    FRAME_ERROR = 2    /* source failure; caller exits 3 per contracts/cli.md                           */
} FrameStatus;

/* The vtable every source implements: live libcamera (src_libcamera.cc), replay (src_replay.c), and the
 * injector's wrapper. Deliberately tiny — a source's whole job is to produce FrameViews in order. */
typedef struct FrameSource {
    void *ctx;                                                     /* implementation state              */
    FrameStatus (*next)(struct FrameSource *self, FrameView *out); /* blocks until the next frame       */
    void        (*close)(struct FrameSource *self);
    /* Nominal rate the source was configured for, used ONLY by the scheduler's cost model (R3) — never to
     * time anything. Real timing always comes from FrameView.t_us. */
    uint32_t nominal_fps;
} FrameSource;

#ifdef __cplusplus
}
#endif
#endif /* BEACON_FRAME_H */
