/* track.h — one tracker: the spatial twin of the DPLL (T034–T037; spec §2.1/§2.2/§2.6).
 *
 * A slot owns: a chip clock (DPLL state), a spatial aperture at one scale of the ladder, per-pixel chip
 * bins over a sliding window, and an alpha-beta centering loop. Everything is integer (R2) and every
 * buffer is fixed-capacity, sized here (R6: no allocation after init).
 *
 * Coordinates: the M2 grid — 320×200 @ 0.304°/px, centre (0,0), +x right, +y down, q8 throughout.
 */
#ifndef BEACON_TRACK_H
#define BEACON_TRACK_H

#include "config.h"
#include "corr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TRK_MAX_EXTENT 24      /* plane px per side; config scale_extents[] must fit                   */
#define TRK_WIN 124            /* chip bins in the sliding window = 4 words — the reverify horizon     */

/* Scale ladder (spec §2.2): index into config scale_extents[]; factor = native px per plane px. */
#define TRK_SCALE_COARSE 0     /* 4x binned                                                            */
#define TRK_SCALE_MEDIUM 1     /* 2x binned — the M2 grid itself                                       */
#define TRK_SCALE_FINE   2     /* native                                                               */

/* Lifecycle (data-model §2). HOLD is EVIDENCE-BOUNDED: age and CEP against the same config keys the
 * scorer reads — never a countdown. */
#define TRK_DEAD      0u
#define TRK_CANDIDATE 1u
#define TRK_CONFIRMED 2u
#define TRK_HOLD      3u

typedef struct {
    /* identity + chip clock (the DPLL) */
    uint8_t  code_id;                  /* 0 = A, 1 = B                                                  */
    int8_t   tmpl[CORR_N];
    uint32_t chip_hz_q8;
    uint64_t epoch_us;                 /* chip 0 of the clock                                           */
    uint8_t  chip_phase;               /* code phase: template index at window chip 0                   */
    int64_t  first_chip;               /* oldest absolute chip with valid data (window clamp)           */
    int64_t  last_chip;                /* most recent absolute chip seen                                */

    /* spatial state — M2 q8 */
    int32_t  x_q8, y_q8;               /* filtered position AT state_t_us (the measurement epoch)       */
    uint64_t state_t_us;               /* the epoch x/y live at — measurements arrive at now - tau(K),
                                        * and tau MOVES when the AGC changes K, so the epoch is explicit
                                        * state, never an assumption                                     */
    int32_t  vx_q8, vy_q8;             /* px/s                                                          */
    int32_t  xp_q8, yp_q8;             /* prediction for the NEXT tick (ROI target)                     */
    int32_t  xr_q8, yr_q8;             /* reported now-position: state extrapolated by the measurement
                                        * latency tau — tau is FEED-FORWARD only, never in the loop      */
    uint16_t cep_q8;

    /* quality */
    uint16_t q_q8;                     /* slow, full-window correlation quality                         */
    uint16_t lock_health_q8;           /* fast, chip-rate decision-directed statistic (spec §2.6)       */
    uint16_t extent_q8;                /* q_fine / q_coarse proxy (spec §9)                             */
    uint16_t scint_q8;                 /* q variance over a window (spec §9)                            */
    uint8_t  scale;                    /* TRK_SCALE_*                                                   */
    uint8_t  t_int_chips;              /* adaptive integration length (spec §4)                         */
    uint8_t  state;                    /* TRK_*                                                         */
    uint8_t  measured_fix;             /* this tick was measurement-backed (spec §3.1 bit)              */
    uint8_t  saturated;                /* peak railed this tick (spec §5)                               */
    uint16_t age_ms;                   /* staleness of the measurement behind the prediction            */
    uint64_t last_fix_us;

    /* aperture sampling: per-pixel chip bins. bins[(py*E+px)*TRK_WIN + b] */
    uint16_t native_w, native_h;       /* frame geometry — needed to convert centre-origin M2 coords
                                        * to top-left plane px and back (parsed from cfg at seed)       */
    uint8_t  extent;                   /* E, plane px per side, from config scale_extents[scale]        */
    int16_t  roi_cx, roi_cy;           /* plane-px centre the ROI was extracted at (integer)            */
    int32_t  bins[TRK_MAX_EXTENT * TRK_MAX_EXTENT * TRK_WIN];
    uint8_t  counts[TRK_WIN];          /* per-chip sample count (same for every pixel)                  */
    int64_t  hp_energy;                /* running local-floor accumulator for lock_health normalisation */
    uint32_t hp_n;

    /* lock-health EWMA internals */
    int32_t  lh_acc_q8;
    uint16_t evidence_chips;           /* chips with usable evidence since the last tick               */
    uint16_t last_r_q8;                /* |innovation| of the last measurement, M2 q8 — the ladder's
                                        * climb gate: a loop still converging must not shrink its net   */
    uint8_t  ladder_dwell;             /* ticks before the ladder may move again                       */

    /* scintillation internals: EWMA of q and of |q - mean| */
    int32_t  q_mean_q8, q_absdev_q8;
} Track;

/* Reset a slot to track code `code_id` from a seed position (M2 q8) with a chip clock guess. */
void track_seed(Track *t, const BcnConfig *cfg, uint8_t code_id, uint8_t scale,
                int32_t x_q8, int32_t y_q8, uint32_t chip_hz_q8, uint64_t epoch_us, uint64_t now_us);

/* Per-frame: deposit one aperture of NORMALIZED per-pixel samples. `roi` is extent*extent values,
 * row-major, extracted by the engine at (roi_cx, roi_cy). Advances the chip window; runs the
 * chip-boundary lock-health update when a chip completes. */
void track_frame(Track *t, const BcnConfig *cfg, const int32_t *roi, int16_t roi_cx, int16_t roi_cy,
                 uint64_t t_us);

/* Per-tick: correlate the window, measure position, update alpha-beta + DPLL + ladder + integration
 * length, refresh q/lock_health/extent/scintillation, run the lifecycle bounds. dt_us = time since the
 * previous tick. Returns 1 if a record-worthy state remains (anything but DEAD). */
int track_tick(Track *t, const BcnConfig *cfg, uint64_t now_us, uint32_t dt_us);

/* Where the engine should extract the NEXT frame's ROI, in plane px of the track's scale. */
void track_roi_center(const Track *t, int16_t *cx, int16_t *cy);

#ifdef __cplusplus
}
#endif
#endif
