/* trail.c — see trail.h for why. Integer only (R2), no allocation (R6), frame-data-pure (R3). */
#include "trail.h"
#include <string.h>

void trail_init(Trail *t, const BcnConfig *cfg)
{
    memset(t, 0, sizeof *t);
    t->crop = cfg->trail_crop_px;
    if (t->crop > TRAIL_CROP_MAX) t->crop = TRAIL_CROP_MAX;
    t->ring_n = cfg->trail_ring_frames;
    if (t->ring_n > TRAIL_RING_MAX) t->ring_n = TRAIL_RING_MAX;
    t->vmax_q8 = (int32_t)cfg->trail_vmax_q8;
    t->vstep_q8 = (int32_t)cfg->trail_vstep_q8 ? (int32_t)cfg->trail_vstep_q8 : (8 << 8);
    t->n_cands = cfg->trail_cands ? cfg->trail_cands : 1u;
}

/* Extract crop + separable 5×5 box filter. Edges (where the full box does not fit) are 0 — a trail
 * sample landing there reads as missing-ish (0 contributes nothing after mean removal on a dark scene).
 * Out-of-frame pixels read 0 for the same reason. Cost: one 9 KB WC read + ~2·5 adds/px. */
void trail_frame(Trail *t, const FrameView *fv, int16_t cx, int16_t cy)
{
    static uint16_t tmp[TRAIL_CROP_MAX * TRAIL_CROP_MAX];   /* single-threaded by construction */
    TrailEntry *e = &t->ring[t->head];
    const int C = t->crop, B = TRAIL_BOX;
    int x0 = cx - C / 2, y0 = cy - C / 2;
    int x, y;

    if (!t->ring_n) return;
    /* horizontal 5-sum of the (clipped) crop */
    for (y = 0; y < C; y++) {
        int sy = y0 + y;
        const uint8_t *row = (sy >= 0 && sy < fv->h) ? fv->data + (size_t)sy * fv->stride : NULL;
        for (x = 0; x < C; x++) {
            int acc = 0, k;
            if (row) {
                for (k = -B; k <= B; k++) {
                    int sx = x0 + x + k;
                    if (sx >= 0 && sx < fv->w) acc += row[sx];
                }
            }
            tmp[y * C + x] = (uint16_t)acc;
        }
    }
    /* vertical 5-sum */
    for (y = 0; y < C; y++) {
        for (x = 0; x < C; x++) {
            int acc = 0, k;
            if (y >= B && y < C - B) {
                for (k = -B; k <= B; k++) acc += tmp[(y + k) * C + x];
            }
            e->box[y * C + x] = (uint16_t)acc;
        }
    }
    e->t_us = fv->t_us;
    e->cx = cx;
    e->cy = cy;
    e->valid = 1u;
    t->head = (uint16_t)((t->head + 1u) % t->ring_n);
    if (t->count < t->ring_n) t->count++;
}

/* One trail hypothesis: bin samples along pos(t) = (x,y) + v·(t - now) into 31 chip bins, mean-remove,
 * best |corr|/energy over the 31 phases. Positions in native q8; v in native px/s q8. */
static uint16_t eval(const Trail *t, const int8_t tmpl[CORR_N],
                     uint64_t epoch_us, uint32_t chip_hz_q8, uint64_t now_us,
                     int32_t x_q8, int32_t y_q8, int32_t vx_q8, int32_t vy_q8)
{
    int64_t sums[CORR_N];
    uint16_t cnts[CORR_N];
    int64_t dev[CORR_N];
    const int C = t->crop, B = TRAIL_BOX;
    uint16_t i;
    int k;

    memset(sums, 0, sizeof sums);
    memset(cnts, 0, sizeof cnts);
    for (i = 0; i < t->count; i++) {
        const TrailEntry *e = &t->ring[i];
        int64_t back_us = (int64_t)e->t_us - (int64_t)now_us;          /* <= 0 */
        int px = (int)((x_q8 + (vx_q8 * back_us) / 1000000) >> 8);
        int py = (int)((y_q8 + (vy_q8 * back_us) / 1000000) >> 8);
        int lx = px - (e->cx - C / 2), ly = py - (e->cy - C / 2);
        uint16_t b;
        if (!e->valid || lx < B || ly < B || lx >= C - B || ly >= C - B) continue;
        b = (uint16_t)(((uint64_t)corr_chip_at(e->t_us, epoch_us, chip_hz_q8)) % CORR_N);
        sums[b] += e->box[ly * C + lx];
        cnts[b]++;
    }
    {
        int64_t total = 0, energy = 0, best = 0;
        int present = 0;
        for (k = 0; k < CORR_N; k++)
            if (cnts[k]) { dev[k] = (sums[k] * 256) / cnts[k]; total += dev[k]; present++; }
        if (present < 20) return 0;                                    /* not enough of the word seen */
        total /= present;
        for (k = 0; k < CORR_N; k++) {
            if (cnts[k]) { dev[k] -= total; energy += dev[k] < 0 ? -dev[k] : dev[k]; }
            else dev[k] = 0;
        }
        if (!energy) return 0;
        for (i = 0; i < CORR_N; i++) {                                 /* phase search */
            int64_t acc = 0;
            for (k = 0; k < CORR_N; k++)
                acc += tmpl[(k + i) % CORR_N] > 0 ? dev[k] : -dev[k];
            if (acc < 0) acc = -acc;
            if (acc > best) best = acc;
        }
        {
            int64_t q = (best * 256) / energy;
            return q > 0xFFFF ? 0xFFFF : (uint16_t)q;
        }
    }
}

int trail_search(const Trail *t, const int8_t tmpl[CORR_N],
                 uint64_t epoch_us, uint32_t chip_hz_q8, uint64_t now_us,
                 uint16_t native_w, uint16_t native_h, uint8_t m2_div, TrailFix *out)
{
    const int C = t->crop;
    int16_t cand_x[8], cand_y[8];
    uint8_t n_cand = 0;
    static uint16_t mx[TRAIL_CROP_MAX * TRAIL_CROP_MAX];   /* single-threaded by construction */
    const TrailEntry *last;
    uint16_t i;
    int x, y, c;

    if (t->count < t->ring_n || !t->ring_n) return 0;      /* need a full word of history */
    last = &t->ring[(t->head + t->ring_n - 1u) % t->ring_n];

    /* ---- nominate: maxima of max-over-last-12 (covers the Gold code's longest dark run), in the
     * latest crop's frame, sampled at the same ABSOLUTE position in each entry ---- */
    {
        memset(mx, 0, sizeof(uint16_t) * (size_t)C * (size_t)C);
        for (i = 0; i < 12u && i < t->ring_n; i++) {
            const TrailEntry *e = &t->ring[(t->head + t->ring_n - 1u - i) % t->ring_n];
            int dx = (last->cx - e->cx), dy = (last->cy - e->cy);
            for (y = 0; y < C; y++) {
                int sy = y + dy;
                if (sy < 0 || sy >= C) continue;
                for (x = 0; x < C; x++) {
                    int sx = x + dx;
                    uint16_t v;
                    if (sx < 0 || sx >= C) continue;
                    v = e->box[sy * C + sx];
                    if (v > mx[y * C + x]) mx[y * C + x] = v;
                }
            }
        }
        /* the prediction itself is always a candidate */
        cand_x[n_cand] = last->cx; cand_y[n_cand] = last->cy; n_cand++;
        while (n_cand < t->n_cands && n_cand < 8u) {
            int bi = -1, bx = 0, by = 0;
            uint16_t bv = 400;                              /* floor: ~16 ADU mean over the 5x5 box */
            for (y = TRAIL_BOX; y < C - TRAIL_BOX; y++)
                for (x = TRAIL_BOX; x < C - TRAIL_BOX; x++)
                    if (mx[y * C + x] > bv) { bv = mx[y * C + x]; bx = x; by = y; bi = 0; }
            if (bi < 0) break;
            cand_x[n_cand] = (int16_t)(last->cx - C / 2 + bx);
            cand_y[n_cand] = (int16_t)(last->cy - C / 2 + by);
            n_cand++;
            for (y = by - 2; y <= by + 2; y++)              /* suppress the 5x5 neighbourhood */
                for (x = bx - 2; x <= bx + 2; x++)
                    if (x >= 0 && y >= 0 && x < C && y < C) mx[y * C + x] = 0;
        }
    }

    /* ---- the grid ---- */
    {
        uint16_t best_q = 0;
        int32_t bx_q8 = 0, by_q8 = 0, bvx = 0, bvy = 0;
        int32_t v;
        for (c = 0; c < n_cand; c++) {
            int32_t cx_q8 = (int32_t)cand_x[c] << 8, cy_q8 = (int32_t)cand_y[c] << 8;
            int32_t vx, vy;
            for (vy = -t->vmax_q8; vy <= t->vmax_q8; vy += t->vstep_q8) {
                for (vx = -t->vmax_q8; vx <= t->vmax_q8; vx += t->vstep_q8) {
                    /* grid is M2 px/s; trails run in native px/s */
                    uint16_t q = eval(t, tmpl, epoch_us, chip_hz_q8, now_us, cx_q8, cy_q8,
                                      vx * m2_div, vy * m2_div);
                    if (q > best_q) { best_q = q; bx_q8 = cx_q8; by_q8 = cy_q8; bvx = vx; bvy = vy; }
                }
            }
        }
        if (!best_q) return 0;
        /* native centre-origin -> M2 q8 */
        out->x_q8 = (bx_q8 - ((int32_t)(native_w / 2) << 8)) / m2_div;
        out->y_q8 = (by_q8 - ((int32_t)(native_h / 2) << 8)) / m2_div;
        out->vx_q8 = bvx;
        out->vy_q8 = bvy;
        out->q_q8 = best_q;
        (void)v;
        return 1;
    }
}
