/* trail.h — the velocity-hypothesis trail search: T050 with the V² term removed by nomination.
 *
 * WHY THIS EXISTS (2026-08-30, results/stage1-chip-rate-and-q.md). Three attempts at improving the
 * aperture correlator's motion handling all failed and located the real limiter: a chicken-and-egg on
 * VELOCITY. The aperture follows the prediction, the prediction needs a velocity, and the velocity comes
 * from fixes that are failing at high rate — the tracker can only REFINE a velocity it already has.
 * Then two measurements settled what to build instead:
 *
 *   - ORACLE ceiling (trail_decode.py): given the true trajectory, decode is 100.0 % at q = 1.00 at
 *     every rate out to 40 °/s, on both pendulum clips. The photons are fine; only the velocity is
 *     missing.
 *   - the SEARCH itself (trail_search_proto.py): nominate a few bright candidates, test a coarse
 *     velocity grid per candidate against the ring of recent frames, adopt the best q. Also 100 %,
 *     0–40 °/s, err p50 0.5–0.85 M2 px — the search fully recovers the ceiling, with no tuning.
 *
 * The margin is what makes it cheap: a RIGHT (position, velocity) hypothesis scores q ≈ 1.00 —
 * saturated — while a wrong one scores like noise (~0.6 at best). Hence coarse steps (8 M2 px/s: the
 * trail error over one word stays inside the sampling box) and no refinement pass.
 *
 * The ring holds 5×5 box-filtered CROPS that follow the track's prediction, so the per-frame cost stays
 * ROI-proportional (a 96×96 crop is ~9 KB of WC read against the 256 KB frame that is off-limits), and a
 * trail sample is ONE u16 lookup. Everything is integer and frame-data-pure: live and replay agree.
 */
#ifndef BEACON_TRAIL_H
#define BEACON_TRAIL_H

#include "config.h"
#include "corr.h"
#include "frame.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TRAIL_CROP_MAX 96          /* native px per side                                               */
#define TRAIL_RING_MAX 80          /* frames — one code word (~74) with margin                         */
#define TRAIL_BOX      2           /* sampling half-box; the ring stores (2B+1)² box sums              */

typedef struct {
    uint64_t t_us;
    int16_t  cx, cy;               /* native centre this crop was taken at                              */
    uint8_t  valid;
    uint16_t box[TRAIL_CROP_MAX * TRAIL_CROP_MAX];   /* 5×5 box-filtered crop (edges 0)               */
} TrailEntry;

typedef struct {
    TrailEntry ring[TRAIL_RING_MAX];
    uint16_t head;                 /* next slot to write                                                */
    uint16_t count;                /* filled entries (saturates at ring_n)                              */
    /* from config */
    uint16_t crop;                 /* native px per side                                                */
    uint16_t ring_n;
    int32_t  vmax_q8, vstep_q8;    /* velocity grid, M2 px/s q8                                         */
    uint8_t  n_cands;
} Trail;

typedef struct {
    int32_t  x_q8, y_q8;           /* M2 centre-origin q8 — the beacon NOW                              */
    int32_t  vx_q8, vy_q8;         /* M2 px/s q8                                                        */
    uint16_t q_q8;
} TrailFix;

void trail_init(Trail *t, const BcnConfig *cfg);

/* Per frame: extract the crop at native centre (cx, cy), box-filter, push into the ring. */
void trail_frame(Trail *t, const FrameView *fv, int16_t cx, int16_t cy);

/* Per tick, when the aperture path failed: nominate candidates, test the velocity grid, return 1 with
 * the best hypothesis if any was evaluable (caller judges q against its own bar). tmpl = the track's
 * code template; the chip clock is the RECEIVER's (epoch + tracked rate). */
int trail_search(const Trail *t, const int8_t tmpl[CORR_N],
                 uint64_t epoch_us, uint32_t chip_hz_q8, uint64_t now_us,
                 uint16_t native_w, uint16_t native_h, uint8_t m2_div, TrailFix *out);

#ifdef __cplusplus
}
#endif
#endif
