/* acquire.h — cold acquisition, stage 1 of US3 (T049 blink detection + T051's rate-agnosticism).
 *
 * Runs ONLY through the sched budget (R3): a pass reads the full field, which costs ~4 ms of WC reads
 * live — exactly what [sched] acquire_cost_us_per_pass charges for. Everything here is frame-data-pure,
 * so replay acquires at the same frame index live would have.
 */
#ifndef BEACON_ACQUIRE_H
#define BEACON_ACQUIRE_H

#include "config.h"
#include "frame.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ACQ_MAX_SEEDS 4

typedef struct {
    int32_t x_q8, y_q8;        /* M2 centre-origin q8                                                  */
    uint16_t strength;         /* |temporal diff| at the blink, for ranking                            */
} AcqSeed;

typedef struct {
    /* previous reduce4 plane (160x100 for 640x400) — the blink detector's memory */
    uint16_t prev[160 * 100];
    uint8_t  prev_valid;
    uint16_t plane_w, plane_h;
    /* T051 rate policy: nominal FIRST (it is nominal for a reason), rotate only when re-seeding a
     * previously-starved location — so a live emitter reset to 200 Hz (the volatile-'H' trap, 031 #2)
     * costs a few starve-reseed rounds, while the common case locks on the first try. */
    int32_t  last_seed_x_q8, last_seed_y_q8;
    uint8_t  rate_rr;
} Acquire;

void acquire_init(Acquire *a, const BcnConfig *cfg);

/* One budgeted pass: reduce4 the frame, temporal-diff against the previous pass's plane, return up to
 * max_seeds blink locations (strongest first). Self-normalising threshold — a blink must clear 4x the
 * field's mean |diff|, so room lighting changes do not spray seeds. */
uint8_t acquire_pass(Acquire *a, const FrameView *fv, AcqSeed *seeds, uint8_t max_seeds);

/* The chip-rate hypothesis for the NEXT seed (round-robins the config candidates). */
uint32_t acquire_next_rate_q8(Acquire *a, const BcnConfig *cfg, int32_t x_q8, int32_t y_q8);

#ifdef __cplusplus
}
#endif
#endif
