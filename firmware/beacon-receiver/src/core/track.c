/* track.c — T034 alpha-beta centering, T035 scale ladder, T036 DPLL, T037 lock_health, plus the
 * extent/scintillation outputs (spec §9). Integer-only (R2); fixed buffers (R6).
 *
 * The measurement: for a sub-pixel point source, correlating each candidate pixel independently and
 * taking the max costs no SNR (spec §2.2) — so the position measurement is a per-pixel |corr| surface
 * over the aperture: peak + 3×3 centroid. The aperture-SUM correlation is computed too — not for
 * position, but because q_peak/q_aperture is the extent discriminator (spec §9) and the sum is the
 * candidate-phase search input.
 *
 * Phase convention: chip_phase is ABSOLUTE — polarity(chip c) = tmpl[(c + chip_phase) mod 31].
 * TRK_WIN = 124 = 4·31, so the bin ring (b = c mod 124) is code-phase aligned by construction.
 */
#include "track.h"
#include <stdio.h>
#include <string.h>

static uint8_t scale_factor(uint8_t scale)
{
    return scale == TRK_SCALE_COARSE ? 4u : scale == TRK_SCALE_MEDIUM ? 2u : 1u;
}
/* Coordinate conventions, in one place because getting them half-right cost a debugging session:
 * M2 coords are CENTRE-origin q8 on the 2x-binned grid (data-model §2); plane px are TOP-LEFT-origin
 * integers on the plane of the track's scale factor f (4/2/1 native px per plane px). The origin shift
 * is half the plane, which is why the frame geometry lives in the Track. */
static int32_t plane_q8_to_m2_q8(const Track *t, int32_t plane_q8, uint8_t f)
{
    int32_t centered = plane_q8 - (((int32_t)(t->native_w / f) / 2) << 8);
    return centered * f / t->m2_div;
}
static int32_t plane_q8_to_m2_q8_y(const Track *t, int32_t plane_q8, uint8_t f)
{
    int32_t centered = plane_q8 - (((int32_t)(t->native_h / f) / 2) << 8);
    return centered * f / t->m2_div;
}
static int32_t m2_q8_to_plane_px(const Track *t, int32_t m2_q8, uint8_t f, int is_y)
{
    int32_t dim = is_y ? t->native_h : t->native_w;
    return ((m2_q8 * t->m2_div / f) >> 8) + (int32_t)(dim / f) / 2;
}

void track_seed(Track *t, const BcnConfig *cfg, uint8_t code_id, uint8_t scale,
                int32_t x_q8, int32_t y_q8, uint32_t chip_hz_q8, uint64_t epoch_us, uint64_t now_us)
{
    unsigned w = 640, h = 400;
    memset(t, 0, sizeof *t);
    sscanf(cfg->camera_mode, "%ux%u", &w, &h);
    t->native_w = (uint16_t)w;
    t->native_h = (uint16_t)h;
    t->m2_div = (uint8_t)(w >= 640 ? 2 : 1);   /* 640-wide: M2 = native/2. 320-wide: native IS M2. */
    t->code_id = code_id;
    corr_template(code_id ? cfg->code_b_bits : cfg->code_a_bits, t->tmpl);
    t->chip_hz_q8 = chip_hz_q8;
    t->epoch_us = epoch_us;
    t->x_q8 = t->xp_q8 = t->xr_q8 = x_q8;
    t->y_q8 = t->yp_q8 = t->yr_q8 = y_q8;
    t->scale = scale;
    t->extent = (uint8_t)cfg->scale_extents[scale];
    if (t->extent > TRK_MAX_EXTENT) t->extent = TRK_MAX_EXTENT;
    t->t_int_chips = cfg->integration_max_chips;   /* start long; AGC shortens on evidence (spec §4)  */
    if (t->t_int_chips > TRK_WIN) t->t_int_chips = TRK_WIN;
    t->state = TRK_CANDIDATE;
    t->chip_phase = 0xFF;                          /* unknown until the first successful search       */
    t->lock_health_q8 = 128;                       /* agnostic prior                                  */
    t->lh_acc_q8 = 128;
    t->last_chip = corr_chip_at(now_us, t->epoch_us, t->chip_hz_q8) - 1;
    t->first_chip = t->last_chip + 1;
    t->px_first_chip = t->first_chip;
    t->state_t_us = now_us;
    t->last_fix_us = now_us;
    t->cep_q8 = (uint16_t)(4u << 8);
}

/* Move a track to `want` (a scale index). See track.h for the two-evidence split this implements: a
 * WIDEN keeps the temporal window (apsum + counts + first_chip) and restarts only the spatial one, so
 * the track keeps correlating through the move instead of starting a 258 ms rebuild against a 150 ms
 * age bound; a CLIMB restarts both, because a finer plane shares neither. */
static void set_scale(Track *t, const BcnConfig *cfg, uint8_t want)
{
    uint8_t E_new = (uint8_t)cfg->scale_extents[want];
    if (E_new > TRK_MAX_EXTENT) E_new = TRK_MAX_EXTENT;

    memset(t->bins, 0, sizeof t->bins);          /* spatial evidence is aperture-relative: never valid
                                                  * on a different plane, in either direction         */
    t->px_first_chip = t->last_chip + 1;
    if (want > t->scale) {                       /* climb: the temporal window goes too — a finer
                                                  * aperture sees a different amount of sky, so its
                                                  * flux series is not a continuation of this one     */
        memset(t->counts, 0, sizeof t->counts);
        memset(t->apsum, 0, sizeof t->apsum);
        t->first_chip = t->last_chip + 1;
    }
    t->scale = want;
    t->extent = E_new;
    t->peak_px = (uint16_t)((E_new / 2u) * E_new + E_new / 2u);
    t->ladder_dwell = 6;
}

void track_roi_center(const Track *t, int16_t *cx, int16_t *cy)
{
    uint8_t f = scale_factor(t->scale);
    *cx = (int16_t)m2_q8_to_plane_px(t, t->xp_q8, f, 0);
    *cy = (int16_t)m2_q8_to_plane_px(t, t->yp_q8, f, 1);
}

/* ---- T037: decision-directed chip re-affirmation, run when a chip COMPLETES ----------------------- */
static void lock_health_chip(Track *t, int64_t chip)
{
    const uint16_t E = t->extent;
    uint16_t b = (uint16_t)(((uint64_t)chip) % TRK_WIN);
    uint16_t p, cpx;
    int64_t apsum = 0;
    int32_t centre, dev, absdev, floor_q;
    int8_t predicted, observed;

    if (t->chip_phase == 0xFF || t->counts[b] == 0) return;   /* no phase yet, or an empty chip       */

    /* THE TRACKED pixel (last measured peak), not the aperture centre: at sub-pixel straddle the
     * centre pixel holds half-amplitude and the sign test dips for no real reason — measured live as
     * lock_health 0.43..0.98 swings on a q=1.00 track. (Per-sample means; counts equal per pixel.)   */
    cpx = t->peak_px < E * E ? t->peak_px : (uint16_t)((E / 2u) * E + E / 2u);
    for (p = 0; p < E * E; p++) apsum += t->bins[p * TRK_WIN + b];
    centre = t->bins[cpx * TRK_WIN + b] / t->counts[b];
    dev = centre - (int32_t)(apsum / ((int64_t)E * E * t->counts[b]));
    absdev = dev < 0 ? -dev : dev;

    /* Local noise floor: EWMA of |dev| — this is what makes the statistic field-position independent
     * (RI corner falloff scales signal AND floor together; the ratio is what survives). */
    t->hp_energy += absdev - (t->hp_energy >> 6);             /* tau ~ 64 chips                       */
    t->hp_n++;
    floor_q = (int32_t)(t->hp_energy >> 6);

    if (t->hp_n < 8 || absdev * 2 < floor_q)                  /* not enough evidence: no update       */
        return;

    predicted = t->tmpl[(uint8_t)(((uint64_t)chip + t->chip_phase) % CORR_N)];
    observed = dev > 0 ? 1 : -1;
    /* alpha 1/4: a genuine break crashes lock_health in 2-3 chips (spec §2.6: ~1 chip detection),
     * while a single noisy chip cannot kill a healthy lock. */
    t->lh_acc_q8 += ((predicted == observed ? 256 : 0) - t->lh_acc_q8) / 4;
    if (t->lh_acc_q8 < 0) t->lh_acc_q8 = 0;
    if (t->lh_acc_q8 > 256) t->lh_acc_q8 = 256;
    t->lock_health_q8 = (uint16_t)t->lh_acc_q8;
    t->evidence_chips++;
}

void track_frame(Track *t, const BcnConfig *cfg, const int32_t *roi, int16_t roi_cx, int16_t roi_cy,
                 uint64_t t_us)
{
    const uint16_t E = t->extent;
    int64_t chip = corr_chip_at(t_us, t->epoch_us, t->chip_hz_q8);
    uint16_t p;
    (void)cfg;

    if (chip > t->last_chip) {
        int64_t c;
        lock_health_chip(t, t->last_chip);                    /* the chip that just completed         */
        if (chip - t->last_chip >= TRK_WIN) {                 /* long gap: the whole window is stale  */
            memset(t->counts, 0, sizeof t->counts);
            memset(t->apsum, 0, sizeof t->apsum);
            memset(t->bins, 0, sizeof(int32_t) * E * E * TRK_WIN);
            t->last_chip = chip - 1;
            t->first_chip = chip;
            t->px_first_chip = chip;
        }
        for (c = t->last_chip + 1; c <= chip; c++) {          /* clear bins being entered             */
            uint16_t b = (uint16_t)(((uint64_t)c) % TRK_WIN);
            t->counts[b] = 0;
            t->apsum[b] = 0;
            for (p = 0; p < E * E; p++) t->bins[p * TRK_WIN + b] = 0;
        }
        t->last_chip = chip;
    }
    {
        uint16_t b = (uint16_t)(((uint64_t)chip) % TRK_WIN);
        int32_t ap = 0;
        for (p = 0; p < E * E; p++) { t->bins[p * TRK_WIN + b] += roi[p]; ap += roi[p]; }
        t->apsum[b] += ap;                 /* the temporal half, maintained incrementally: it also
                                            * retires the per-tick E*E*TRK_WIN re-summation below */
        if (t->counts[b] < 255u) t->counts[b]++;
    }
    t->roi_cx = roi_cx;
    t->roi_cy = roi_cy;
}

/* ---- per-tick ------------------------------------------------------------------------------------ */

/* Copy the most recent K COMPLETE chips into a contiguous window, CLAMPED to chips that actually exist.
 * Without the clamp, a young track (fewer elapsed chips than K) computes negative absolute chips whose
 * (uint64_t) cast aliases LIVE ring bins into wrong window offsets — the correlation self-interferes to
 * q~0 and the candidate starves. (Found via the q=256-by-hand / q=0-in-tick discrepancy, 2026-08-19.)
 * *K_io is adjusted to the effective width. Returns c0, or -1 if fewer than 8 populated chips exist. */
static int64_t window(const Track *t, int64_t first_chip, uint16_t *K_io, const int32_t *pxbins,
                      int32_t *wbins, uint8_t *wcounts)
{
    uint16_t K = *K_io;
    int64_t c0 = t->last_chip - K;                 /* window = [c0, last_chip-1]: complete chips only */
    uint16_t k, filled = 0;
    if (c0 < first_chip) {
        c0 = first_chip;
        if (t->last_chip <= c0) return -1;
        K = (uint16_t)(t->last_chip - c0);
        *K_io = K;
    }
    for (k = 0; k < K; k++) {
        uint16_t b = (uint16_t)(((uint64_t)(c0 + k)) % TRK_WIN);
        wbins[k] = pxbins[b];
        wcounts[k] = t->counts[b];
        filled = (uint16_t)(filled + (t->counts[b] != 0));
    }
    return filled >= 8 ? c0 : -1;
}

int track_tick(Track *t, const BcnConfig *cfg, uint64_t now_us, uint32_t dt_us)
{
    const uint16_t E = t->extent;
    const uint8_t f = scale_factor(t->scale);
    int measured_this_tick = 0;
    uint16_t K = t->t_int_chips;
    int32_t wsum[TRK_WIN];
    uint8_t wcnt[TRK_WIN];
    int64_t c0;
    CorrResult ap;
    uint16_t p;

    if (t->state == TRK_DEAD) return 0;
    if (K > TRK_WIN) K = TRK_WIN;
    t->measured_fix = 0;
    t->evidence_chips = 0;

    /* The state (x,y) lives at the MEASUREMENT epoch — half an integration window behind the wall.
     * The flywheel below advances it every tick, measured or not; tau-extrapolated outputs are computed
     * at the end. measured_this_tick gates which branch ran. */

    /* Aperture-sum window: candidate phase search + identity re-verification + the extent denominator.
     * Reads the apsum ring maintained by track_frame rather than re-summing E*E*TRK_WIN bins here, so it
     * costs nothing per tick AND survives a widen (see track.h: temporal vs spatial evidence). */
    c0 = window(t, t->first_chip, &K, t->apsum, wsum, wcnt);   /* K may shrink for a young window   */
    if (c0 < 0) {
        /* nothing to correlate (dark / long occlusion): HOLD bookkeeping only */
        goto lifecycle;
    }
    {
        int8_t ta[CORR_N], tb[CORR_N];
        corr_template(cfg->code_a_bits, ta);
        corr_template(cfg->code_b_bits, tb);
        corr_search(wsum, wcnt, K, ta, tb, &ap);
    }
    t->last_corr = ap.corr;             /* see track.h — the promotion gate reads energy/level */
    t->last_energy = ap.energy;
    t->last_level = ap.level;

    if (ap.code_id == t->code_id && ap.q_q8 >= cfg->q_drop_q8) {
        /* T036, the DPLL's phase half: adopt/track the absolute phase. A drift of ±1 chip between
         * ticks is the rate discriminator: EWMA it into chip_hz. */
        uint8_t phase_abs = (uint8_t)((((int64_t)ap.phase - c0) % CORR_N + CORR_N) % CORR_N);
        if (t->chip_phase == 0xFF) {
            t->chip_phase = phase_abs;
        } else if (phase_abs != t->chip_phase) {
            /* T036, phase half only — RATE ADJUSTMENT IS DELIBERATELY OFF (2026-08-19). Both attempts
             * at closing the rate loop live destabilised it: naive dhz walked 115->109 Hz because a
             * rate change rebases corr_chip_at across the whole epoch-to-now span; the epoch-re-anchor
             * repair then sprayed 112..129 Hz. Meanwhile the fixed nominal rate held q=1.00 for whole
             * stretches — the bench emitter is +0.7 % off nominal and a K=31 window only accrues 0.2
             * chip of drift, well inside the correlator's tolerance. Phase adoption below absorbs the
             * residual as an occasional 1-chip re-index. Rate tracking returns as designed work (sim
             * against golden clips first), not live tuning: see bench journal 042 entry. */
            t->chip_phase = phase_abs;
        }
        t->q_q8 = ap.q_q8;
    } else {
        /* window does not carry this track's code at usable quality */
        t->q_q8 = ap.code_id == t->code_id ? ap.q_q8 : (uint16_t)(ap.q_q8 / 4u);
        goto lifecycle;
    }

    /* ---- position measurement: per-pixel |corr| surface at the known phase ---- */
    {
        /* 64-bit like the correlator it is filled from: truncating the per-pixel corr into an int32
         * surface would reintroduce, one step later, exactly the saturation corr.c now avoids. */
        int64_t surf[TRK_MAX_EXTENT * TRK_MAX_EXTENT];
        int64_t peak = 0;
        uint16_t peak_p = (uint16_t)((E / 2u) * E + E / 2u);
        /* The SPATIAL window can be younger than the temporal one — a widen keeps apsum and restarts
         * bins — so it gets its own clamp and therefore its own phase. counts[] is pixel-independent,
         * so one probe establishes c0p/Kp0 for every pixel. */
        uint16_t Kp0 = K;
        int64_t c0p = window(t, t->px_first_chip, &Kp0, t->bins, wsum, wcnt);
        uint8_t wphase;
        CorrResult pr;

        if (c0p < 0) goto surface_done;            /* spatial window still rebuilding after a widen  */
        wphase = (uint8_t)(((uint64_t)((c0p % CORR_N + CORR_N) % CORR_N) + t->chip_phase) % CORR_N);

        for (p = 0; p < E * E; p++) {
            uint16_t Kp = Kp0;                     /* same clamp state -> same c0/K for every pixel   */
            window(t, t->px_first_chip, &Kp, t->bins + (size_t)p * TRK_WIN, wsum, wcnt);
            corr_track(wsum, wcnt, Kp, t->tmpl, wphase, &pr);
            surf[p] = pr.corr > 0 ? pr.corr : 0;   /* wrong-polarity pixels are background, not signal */
            if (surf[p] > peak) { peak = surf[p]; peak_p = p; }
        }
        if (peak > 0) {
            t->peak_px = peak_p;
            /* 3×3 centroid around the peak. SATURATED (spec §5): amplitude is a lie once the peak
             * rails, so switch to a flat-top estimator — binarise the weights at 70 % of peak. */
            int64_t wx = 0, wy = 0, wt = 0;
            int64_t m2s_w = 0;                   /* second moment with the SAME weights as the centroid */
            int32_t px = (int32_t)(peak_p % E), py = (int32_t)(peak_p / E);
            int32_t peak_q8;
            uint16_t qpk;
            int dy, dx;
            for (dy = -1; dy <= 1; dy++) {
                for (dx = -1; dx <= 1; dx++) {
                    int32_t xx = px + dx, yy = py + dy;
                    int64_t wgt;
                    if (xx < 0 || yy < 0 || xx >= E || yy >= E) continue;
                    wgt = surf[yy * E + xx];
                    if (t->saturated) wgt = wgt * 10 >= peak * 7 ? 1 : 0;
                    wx += wgt * xx; wy += wgt * yy; wt += wgt;
                    m2s_w += wgt * (dx * dx + dy * dy);
                }
            }
            if (wt > 0) {
                int32_t mx_q8 = (int32_t)((wx << 8) / wt);
                int32_t my_q8 = (int32_t)((wy << 8) / wt);
                /* ROI coords -> plane px q8 -> M2 q8 (roi centre pixel index E/2 sits AT roi_cx) */
                int32_t plane_x_q8 = ((int32_t)t->roi_cx << 8) + mx_q8 - ((int32_t)(E / 2u) << 8);
                int32_t plane_y_q8 = ((int32_t)t->roi_cy << 8) + my_q8 - ((int32_t)(E / 2u) << 8);
                int32_t zx = plane_q8_to_m2_q8(t, plane_x_q8, f);
                int32_t zy = plane_q8_to_m2_q8_y(t, plane_y_q8, f);

                /* Innovation gate, before the update touches any state. The bench scene breathes
                 * in-band (LED-lamp PWM, journal trap #4), and roughly once in ~10^2 ticks an
                 * aperture-edge pixel out-peaks the beacon. Ungated, that one fix kicks v hard enough
                 * that HOLD extrapolates off the field and the track dies (live, 2026-08-19: solid
                 * q=1.00 locks at (8,16) dying to teleports like (-94,157)). Gated, a wild fix is a
                 * COASTED tick — the flywheel's whole job. Gate: 4*cep, floored at 2 M2 px. */
                {
                    int32_t gx = zx - (t->x_q8 + (int32_t)(((int64_t)t->vx_q8 * dt_us) / 1000000));
                    int32_t gy = zy - (t->y_q8 + (int32_t)(((int64_t)t->vy_q8 * dt_us) / 1000000));
                    int32_t gate = 4 * (int32_t)t->cep_q8;
                    if (gate < (2 << 8)) gate = 2 << 8;
                    if (gx < 0) gx = -gx;
                    if (gy < 0) gy = -gy;
                    if ((gx > gate || gy > gate) && t->state != TRK_CANDIDATE)
                        goto measurement_rejected;
                }

                /* T034: the alpha-beta update. Deliberately the SIMPLE form: innovate against the
                 * tick-epoch prediction, constant dt. z lags truth by tau (the window-centre effect),
                 * so the STATE converges to the lagged manifold — with UNBIASED velocity, because at
                 * steady state r -> 0 forces dx/dt = dz/dt = the true rate. The lag is then corrected
                 * exactly on the OUTPUT side (xr below). Two failed refinements are recorded here so
                 * nobody retries them: v*tau added to z (positive feedback through v, 3x blowup), and
                 * variable measurement-epoch gaps (clamped gaps spike the velocity gain, 5x blowup).
                 * Stability beat sophistication both times. */
                int32_t xp_meas = t->x_q8 + (int32_t)(((int64_t)t->vx_q8 * dt_us) / 1000000);
                int32_t yp_meas = t->y_q8 + (int32_t)(((int64_t)t->vy_q8 * dt_us) / 1000000);
                int32_t rx = zx - xp_meas, ry = zy - yp_meas;
                t->x_q8 = xp_meas + (int32_t)(((int64_t)cfg->alpha_q8 * rx) >> 8);
                t->y_q8 = yp_meas + (int32_t)(((int64_t)cfg->alpha_q8 * ry) >> 8);
                if (dt_us > 0) {
                    t->vx_q8 += (int32_t)((((int64_t)cfg->beta_q8 * rx) * 1000000) / ((int64_t)dt_us << 8));
                    t->vy_q8 += (int32_t)((((int64_t)cfg->beta_q8 * ry) * 1000000) / ((int64_t)dt_us << 8));
                }
                {
                    int32_t ax = rx < 0 ? -rx : rx, ay = ry < 0 ? -ry : ry;
                    int32_t m = ax > ay ? ax : ay;
                    t->last_r_q8 = m > 0xFFFF ? 0xFFFF : (uint16_t)m;
                }
                measured_this_tick = 1;

                /* CEP from the centroid spread (second moment around the peak), in M2 q8. MUST use
                 * the same weights the centroid used: the first version mixed raw |corr| moments with
                 * flat-top-binarised wt (<=9), saturating cep to 256 px — and the HOLD cep bound then
                 * killed the track on the next unmeasured tick (live autopsy 2026-08-19, deaths 2-5). */
                {
                    int32_t var_q8 = (int32_t)((m2s_w << 8) / wt / 2);
                    /* cep is a SIZE, not a position — scale only, no origin shift */
                    int32_t cep = (var_q8 > 0 ? var_q8 : 26);
                    cep = cep * f / t->m2_div;
                    t->cep_q8 = (uint16_t)(cep > 0xFFFF ? 0xFFFF : cep < 26 ? 26 : cep);
                }

                /* extent (spec §9): point source -> peak-pixel q >= aperture q; glitter -> below */
                qpk = 0;
                {
                    CorrResult pk;
                    uint16_t Kp = Kp0;
                    window(t, t->px_first_chip, &Kp, t->bins + (size_t)peak_p * TRK_WIN, wsum, wcnt);
                    corr_track(wsum, wcnt, Kp, t->tmpl, wphase, &pk);
                    qpk = pk.q_q8;
                }
                t->extent_q8 = (uint16_t)(((uint32_t)qpk * 256u) / (t->q_q8 ? t->q_q8 : 1u));
                peak_q8 = qpk;
                (void)peak_q8;

                /* A FIX MUST BE WORTH BELIEVING (q_fix). Measured on pend_ir against fiducial truth,
                 * with honest 64-bit q: on-beacon fixes sit at q p05 0.86 / p50 1.00, while fixes that
                 * landed >20 M2 px away sit at p50 0.45 / p95 0.67 — cleanly bimodal. Those bad fixes are
                 * not a competing source (they are scattered, not clustered on a lamp); they are the
                 * tracker still reporting a measurement after it has lost a FAST target, which is the
                 * coherence limit leaking into the wire contract. Gating here keeps 98 % of real fixes
                 * and removes 100 % of the off-target ones at 0.75.
                 *
                 * This is the gate T085 looked for and could not find, and the reason it could not is
                 * that q railed to 1.00 on 75 % of slot-ticks before corr.c was widened — the separation
                 * genuinely was not visible in the data it was measured on. */
                t->measured_fix = t->q_q8 >= cfg->q_fix_q8 &&
                                  (t->evidence_chips > 0 || t->lock_health_q8 >= cfg->lock_health_drop_q8);
                t->last_fix_us = now_us;
measurement_rejected: ;
            }
        }
surface_done: ;
    }

    /* scintillation (spec §9): EWMA absolute deviation of q */
    t->q_mean_q8 += ((int32_t)t->q_q8 - t->q_mean_q8) / 8;
    {
        int32_t d = (int32_t)t->q_q8 - t->q_mean_q8;
        if (d < 0) d = -d;
        t->q_absdev_q8 += (d - t->q_absdev_q8) / 8;
        t->scint_q8 = t->q_absdev_q8 > 0xFFFF ? 0xFFFF : (uint16_t)t->q_absdev_q8;
    }

lifecycle:
    /* Coast when unmeasured (the flywheel), then split the outputs by consumer:
     *   - the ROI target (xp) is where the beacon will average over the NEXT window: x + v*dt — the
     *     state already lives on the lagged manifold, so dt alone is correct here;
     *   - the reported now-position (xr) corrects the manifold lag: x + v*tau;
     *   - the record's x_pred (next tick, per contract) is xr + v*dt — filled in bank_emit. */
    if (!measured_this_tick) {
        t->x_q8 += (int32_t)(((int64_t)t->vx_q8 * dt_us) / 1000000);
        t->y_q8 += (int32_t)(((int64_t)t->vy_q8 * dt_us) / 1000000);
        /* UNCERTAINTY GROWS WHENEVER WE COAST — in CONFIRMED exactly as in HOLD, and this placement is
         * the whole point (2026-08-29). It used to live in the TRK_HOLD case alone, which made the
         * innovation gate above a LATCH: gate = 4*cep, cep frozen while CONFIRMED, so a state that had
         * fallen further behind than the gate rejected every subsequent measurement, was therefore never
         * corrected, and fell further behind still. Positive feedback — rejecting a fix is evidence the
         * state is WRONG, and the old code answered that evidence by rejecting harder.
         *
         * Caught on the golden clip: a PRECISION track sat frozen at x=45 with v stuck at (4.1, 21.1)
         * for 350 ms, mf=0 every tick, while the beacon walked to x=65 — cep still reporting 1.0 px. It
         * escaped only when q finally decayed enough to force HOLD, ~7 ticks of a dead track later, and
         * on the third occurrence it did not escape at all. This is also why a coasting track could
         * report a confident cep it had no evidence for: cep is the number the scorer and the §3.1
         * validity bound both read, and a flywheeling track's uncertainty is genuinely growing.
         *
         * Same growth law as HOLD used (12.5 % + 0.03 px per tick), so HOLD behaviour is unchanged
         * except that a MEASURED tick in HOLD no longer inflates — the centroid spread sets cep there,
         * which is the honest value. */
        {
            uint32_t grow = (uint32_t)t->cep_q8 + ((uint32_t)t->cep_q8 >> 3) + 8u;
            t->cep_q8 = grow > 0xFFFF ? 0xFFFF : (uint16_t)grow;
        }
    }
    {
        uint32_t chip_us = t->chip_hz_q8 ? (uint32_t)((256ull * 1000000ull) / t->chip_hz_q8) : 8696u;
        uint32_t tau_us = (uint32_t)K * chip_us / 2u;
        t->xr_q8 = t->x_q8 + (int32_t)(((int64_t)t->vx_q8 * tau_us) / 1000000);
        t->yr_q8 = t->y_q8 + (int32_t)(((int64_t)t->vy_q8 * tau_us) / 1000000);
        t->xp_q8 = t->x_q8 + (int32_t)(((int64_t)t->vx_q8 * dt_us) / 1000000);
        t->yp_q8 = t->y_q8 + (int32_t)(((int64_t)t->vy_q8 * dt_us) / 1000000);
    }

    t->age_ms = (uint16_t)((now_us - t->last_fix_us) / 1000u > 0xFFFF ? 0xFFFF
                           : (now_us - t->last_fix_us) / 1000u);

    /* T040's bounds live here so the state machine and the scorer read the SAME numbers (data-model §2):
     * promotion/demotion BETWEEN slots is the bank's job; the evidence bounds are per-track. */
    switch (t->state) {
    case TRK_CONFIRMED:
        /* HOLD entry gates on q ALONE for now. lock_health is reported but does not yet demote: its
         * per-chip sign test still dips on straddle even watching the peak pixel, and a q=1.00 track
         * being sent to HOLD by a proxy statistic was the live failure loop of 2026-08-19 (HOLD ->
         * widen -> bins reset -> re-affirmation window eaten -> dead at age 150 ms). Revisit when the
         * estimator earns back its vote (spec §2.6 wants it driving this — earned, not assumed). */
        if (!t->measured_fix && t->q_q8 < cfg->q_drop_q8) {
            t->state = TRK_HOLD;
            /* Entering HOLD at a fine scale is how a track dies BLIND: the aperture that lost the
             * beacon is the last place to keep staring. Widen immediately — the spatial version of
             * s7's unlocked fast-gear (and the cheap half of the guard's job). */
            if (t->scale > 0u) set_scale(t, cfg, (uint8_t)(t->scale - 1u));
        }
        break;
    case TRK_HOLD:
        /* cep inflation moved to the coast block above — it applies to CONFIRMED too, and for the same
         * reason. See the comment there. */
        if (t->measured_fix && t->q_q8 >= cfg->q_lock_q8) {
            t->state = TRK_CONFIRMED;
        } else if (t->age_ms > cfg->hold_max_age_ms || t->cep_q8 > cfg->hold_max_cep_px_q8) {
            t->state = TRK_DEAD;               /* evidence exhausted: §3.1's validity bound, exactly  */
        }
        break;
    default:
        break;
    }

    /* T035: the q-driven ladder, with hysteresis and a dwell so it cannot chatter. MEASURED ticks
     * only: a young/empty window reports q=0, and treating that as "weak, widen" demoted every fresh
     * candidate to coarse on its first tick — where daylight flicker owns the surface and nothing ever
     * confirms (found by replay-debug on a real daylight clip, 2026-08-20). Absence of evidence is not
     * evidence of weakness. */
    if (t->ladder_dwell) t->ladder_dwell--;
    else if (t->measured_fix && (t->state == TRK_CONFIRMED || t->state == TRK_CANDIDATE)) {
        uint8_t want = t->scale;
        /* Climb gate: strong, tight, AND SETTLED — the innovation must already be small relative to
         * the finer plane's pixel, or the shrunken aperture will lose a still-converging loop (the
         * f200-f250 death of 2026-08-19: promoted, climbed to fine at v-overshoot, gone in 3 ticks). */
        uint16_t finer_px_q8 = (uint16_t)((scale_factor((uint8_t)(t->scale + 1u)) << 8) / 2u);
        if (t->q_q8 >= cfg->q_lock_q8 && t->cep_q8 < (2u << 8) && t->scale < cfg->n_scales - 1u &&
            t->measured_fix && t->last_r_q8 < finer_px_q8)
            want = (uint8_t)(t->scale + 1u);   /* strong + tight + settled: climb toward fine         */
        else if (t->q_q8 < cfg->q_drop_q8 && t->scale > 0u)
            want = (uint8_t)(t->scale - 1u);   /* weak: widen                                         */
        if (want != t->scale) set_scale(t, cfg, want);
    }

    return t->state != TRK_DEAD;
}
