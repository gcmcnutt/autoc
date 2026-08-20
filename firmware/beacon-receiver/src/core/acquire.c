/* acquire.c — T049 (+ the rate-agnostic seed policy of T051). RANSAC proto-tracks (T050) and
 * decode-along-track (T051 full) come with the moving-acquire work; a static bench beacon and a slewing
 * rig both blink in place frame-to-frame, which is what this detects. */
#include "acquire.h"
#include <stdio.h>
#include "reduce.h"
#include <string.h>

void acquire_init(Acquire *a, const BcnConfig *cfg)
{
    unsigned w = 640, h = 400;
    memset(a, 0, sizeof *a);
    sscanf(cfg->camera_mode, "%ux%u", &w, &h);
    a->plane_w = (uint16_t)(w / 4u);
    a->plane_h = (uint16_t)(h / 4u);
    a->m2_mul = (uint8_t)(1280u / w);          /* coarse plane -> M2: x2 at 640-wide, x4 at 320-wide */
}

uint32_t acquire_next_rate_q8(Acquire *a, const BcnConfig *cfg, int32_t x_q8, int32_t y_q8)
{
    int32_t dx = x_q8 - a->last_seed_x_q8, dy = y_q8 - a->last_seed_y_q8;
    uint32_t hz;
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    if (dx < (8 << 8) && dy < (8 << 8)) {
        a->rate_rr = (uint8_t)((a->rate_rr + 1u) % cfg->n_chip_hz_candidates);   /* same spot: rotate */
    } else {
        a->rate_rr = 0u;                                    /* new spot: nominal first               */
    }
    hz = a->rate_rr == 0u ? cfg->chip_hz_nominal_q8
                          : cfg->chip_hz_candidates_q8[a->rate_rr];
    a->last_seed_x_q8 = x_q8;
    a->last_seed_y_q8 = y_q8;
    return hz;
}

uint8_t acquire_pass(Acquire *a, const FrameView *fv, AcqSeed *seeds, uint8_t max_seeds)
{
    static uint16_t cur[160 * 100];    /* scratch; single-threaded by the engine's construction        */
    const uint16_t W = a->plane_w, H = a->plane_h;
    uint32_t n = (uint32_t)W * H;
    uint64_t sum_abs = 0;
    uint32_t i;
    uint32_t thresh;
    uint8_t found = 0;

    if (n > sizeof cur / sizeof cur[0]) return 0;
    reduce4_scalar(fv->data, fv->stride, fv->w, fv->h, cur);   /* scalar: the reference IS the spec — 
                                                                * NEON here later must be bit-exact    */
    if (!a->prev_valid) {
        memcpy(a->prev, cur, n * sizeof *cur);
        a->prev_valid = 1u;
        return 0;
    }

    for (i = 0; i < n; i++) {
        int32_t d = (int32_t)cur[i] - a->prev[i];
        sum_abs += (uint64_t)(d < 0 ? -d : d);
    }
    /* self-normalising: a seed must clear 4x the field's mean |diff| AND an absolute floor (a beacon
     * chip flip moves a 4x4 bin by ~hundreds; sensor noise moves it by a few) */
    thresh = (uint32_t)(4u * (sum_abs / n));
    if (thresh < 64u) thresh = 64u;

    while (found < max_seeds) {
        uint32_t best = 0, best_i = 0;
        int32_t d;
        uint8_t k;
        for (i = 0; i < n; i++) {
            d = (int32_t)cur[i] - a->prev[i];
            if (d < 0) d = -d;
            if ((uint32_t)d > best) { best = (uint32_t)d; best_i = i; }
        }
        if (best < thresh) break;
        {
            uint16_t px = (uint16_t)(best_i % W), py = (uint16_t)(best_i / W);
            /* coarse plane px -> M2 centre-origin q8: m2 = (plane - W/2) * 2 */
            seeds[found].x_q8 = ((int32_t)px - W / 2) * a->m2_mul * 256;
            seeds[found].y_q8 = ((int32_t)py - H / 2) * a->m2_mul * 256;
            seeds[found].strength = best > 0xFFFF ? 0xFFFF : (uint16_t)best;
            found++;
            /* null out a 3x3 neighbourhood so the next-strongest is a DIFFERENT blinker */
            for (k = 0; k < 9; k++) {
                int32_t xx = (int32_t)px + (k % 3) - 1, yy = (int32_t)py + (k / 3) - 1;
                if (xx >= 0 && yy >= 0 && xx < W && yy < H)
                    a->prev[yy * W + xx] = cur[yy * W + xx];
            }
        }
    }
    memcpy(a->prev, cur, n * sizeof *cur);
    return found;
}
