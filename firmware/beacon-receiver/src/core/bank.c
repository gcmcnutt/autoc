/* bank.c — T039 lifecycle/pairing/eviction, T040 evidence-bounded HOLD (the bounds themselves live in
 * track.c against the same config keys the scorer reads), T055 mirror-pair rule. */
#include "bank.h"
#include <string.h>

void bank_init(Bank *b, const BcnConfig *cfg)
{
    memset(b, 0, sizeof *b);
    b->max_slots = cfg->max_slots;
}

/* Eviction priority (data-model §3): CANDIDATE < GUARD < PRECISION; within a class, lowest lock_health
 * first. Returns a free or evictable slot for a new CANDIDATE, or -1 (candidates never evict guards or
 * precision trackers — a new maybe never outranks a working lock). */
static int find_slot(Bank *b)
{
    int i, worst = -1;
    uint16_t worst_lh = 0xFFFF;
    for (i = 0; i < b->max_slots; i++)
        if (!b->slots[i].used) return i;
    for (i = 0; i < b->max_slots; i++) {
        BankSlot *s = &b->slots[i];
        if (s->role == BANK_ROLE_CANDIDATE && s->trk.lock_health_q8 < worst_lh) {
            worst_lh = s->trk.lock_health_q8;
            worst = i;
        }
    }
    return worst;
}

int bank_seed(Bank *b, const BcnConfig *cfg, uint8_t code_id, uint8_t scale,
              int32_t x_q8, int32_t y_q8, uint32_t chip_hz_q8, uint64_t epoch_us, uint64_t now_us)
{
    int i = find_slot(b);
    if (i < 0) return -1;
    if (b->slots[i].used && b->slots[i].partner >= 0)          /* evicted candidate had a partner?    */
        b->slots[b->slots[i].partner].partner = -1;            /* (cannot happen for candidates, but  */
    memset(&b->slots[i], 0, sizeof b->slots[i]);               /*  cheap insurance)                   */
    track_seed(&b->slots[i].trk, cfg, code_id, scale, x_q8, y_q8, chip_hz_q8, epoch_us, now_us);
    b->slots[i].used = 1u;
    b->slots[i].role = BANK_ROLE_CANDIDATE;
    b->slots[i].partner = -1;
    return i;
}

static void bury(Bank *b, int i)
{
    BankSlot *s = &b->slots[i];
    if (s->partner >= 0) {
        b->slots[s->partner].partner = -1;
        /* a PRECISION dying frees its guard; a GUARD dying leaves precision solo until re-paired */
        if (s->role == BANK_ROLE_PRECISION && b->slots[s->partner].role == BANK_ROLE_GUARD)
            b->slots[s->partner].used = 0u;
    }
    memset(s, 0, sizeof *s);
}

void bank_tick(Bank *b, const BcnConfig *cfg)
{
    int i, j;

    /* burial + orphaned-guard rule (data-model §3: a GUARD may exist only with a live PRECISION) */
    for (i = 0; i < b->max_slots; i++) {
        BankSlot *s = &b->slots[i];
        if (!s->used) continue;
        if (s->trk.state == TRK_DEAD) { bury(b, i); continue; }
        if (s->role == BANK_ROLE_GUARD &&
            (s->partner < 0 || !b->slots[s->partner].used)) {
            bury(b, i);
            continue;
        }
    }

    /* promotion: CANDIDATE -> CONFIRMED on SUSTAINED lock_health (not one lucky word) */
    for (i = 0; i < b->max_slots; i++) {
        BankSlot *s = &b->slots[i];
        if (!s->used || s->role != BANK_ROLE_CANDIDATE) continue;
        if (s->trk.state == TRK_CANDIDATE &&
            s->trk.q_q8 >= cfg->q_lock_q8 && s->trk.lock_health_q8 >= cfg->lock_health_lock_q8) {
            if (++s->promote_streak >= 3u) {
                s->trk.state = TRK_CONFIRMED;
                s->role = BANK_ROLE_PRECISION;
                s->promote_streak = 0u;
            }
        } else if (s->promote_streak) {
            s->promote_streak--;
        }
        /* a candidate whose q never rises dies by eviction (find_slot) or by starvation here: */
        if (s->trk.age_ms > 2u * cfg->hold_max_age_ms && s->trk.q_q8 < cfg->q_drop_q8)
            bury(b, i);         /* starve fast: a wrong-rate candidate must free the spot for the
                                 * next rate hypothesis (acquire rotates on re-seed) */
    }

    /* guard allocation: every CONFIRMED PRECISION without a partner gets a GUARD one scale coarser,
     * same state, wider aperture — if a slot is free (guards never evict anyone). */
    for (i = 0; i < b->max_slots; i++) {
        BankSlot *s = &b->slots[i];
        int g;
        if (!s->used || s->role != BANK_ROLE_PRECISION || s->partner >= 0) continue;
        if (s->trk.state != TRK_CONFIRMED) continue;
        for (g = 0; g < b->max_slots && b->slots[g].used; g++) {}
        if (g == b->max_slots) continue;
        {
            uint8_t gscale = s->trk.scale > 0u ? (uint8_t)(s->trk.scale - 1u) : 0u;
            track_seed(&b->slots[g].trk, cfg, s->trk.code_id, gscale,
                       s->trk.x_q8, s->trk.y_q8, s->trk.chip_hz_q8, s->trk.epoch_us,
                       s->trk.last_fix_us);
            b->slots[g].trk.chip_phase = s->trk.chip_phase;   /* inherit the lock — that is the point */
            b->slots[g].trk.state = TRK_CONFIRMED;
            b->slots[g].used = 1u;
            b->slots[g].role = BANK_ROLE_GUARD;
            b->slots[g].partner = (int8_t)i;
            s->partner = (int8_t)g;
        }
    }

    /* guard rescue: precision in HOLD while its guard holds a confirmed lock -> re-center precision on
     * the guard (the "2 ticks instead of 1+ s" purchase, spec §2.4) */
    for (i = 0; i < b->max_slots; i++) {
        BankSlot *s = &b->slots[i];
        if (!s->used || s->role != BANK_ROLE_PRECISION || s->partner < 0) continue;
        if (s->trk.state == TRK_HOLD && b->slots[s->partner].trk.state == TRK_CONFIRMED) {
            const Track *g = &b->slots[s->partner].trk;
            s->trk.x_q8 = s->trk.xp_q8 = g->x_q8;
            s->trk.y_q8 = s->trk.yp_q8 = g->y_q8;
            s->trk.vx_q8 = g->vx_q8;
            s->trk.vy_q8 = g->vy_q8;
            s->trk.cep_q8 = g->cep_q8;
        }
    }

    /* T055, the mirror-pair rule (spec §9): two CONFIRMED tracks, same code -> the geometrically UPPER
     * (smaller y: +y is down) keeps CONFIRMED, the lower is flagged MULTIPATH_SUSPECT and KEPT — a
     * flagged ghost fix beats no fix if the direct path occludes. Guards are exempt (they duplicate
     * their partner's code by design). */
    for (i = 0; i < b->max_slots; i++) {
        BankSlot *si = &b->slots[i];
        if (!si->used || si->role == BANK_ROLE_GUARD || si->trk.state != TRK_CONFIRMED) continue;
        si->flags_extra &= (uint8_t)~BCN_F_MULTIPATH_SUSPECT;
    }
    for (i = 0; i < b->max_slots; i++) {
        BankSlot *si = &b->slots[i];
        if (!si->used || si->role == BANK_ROLE_GUARD || si->trk.state != TRK_CONFIRMED) continue;
        for (j = i + 1; j < b->max_slots; j++) {
            BankSlot *sj = &b->slots[j];
            if (!sj->used || sj->role == BANK_ROLE_GUARD || sj->trk.state != TRK_CONFIRMED) continue;
            if (si->trk.code_id != sj->trk.code_id) continue;
            if (si->trk.y_q8 <= sj->trk.y_q8) sj->flags_extra |= BCN_F_MULTIPATH_SUSPECT;
            else                              si->flags_extra |= BCN_F_MULTIPATH_SUSPECT;
        }
    }
}

uint8_t bank_slots_used(const Bank *b)
{
    uint8_t n = 0;
    int i;
    for (i = 0; i < b->max_slots; i++) n += b->slots[i].used;
    return n;
}

uint8_t bank_emit(const Bank *b, const BcnConfig *cfg, BcnRecord *rec)
{
    uint8_t n = 0;
    int i;
    (void)cfg;
    for (i = 0; i < b->max_slots && n < BCN_MAX_TRACKS; i++) {
        const BankSlot *s = &b->slots[i];
        const Track *t = &s->trk;
        BcnTrack *o;
        if (!s->used || s->role == BANK_ROLE_GUARD) continue;      /* guards are plumbing, not output */
        if (t->state != TRK_CONFIRMED && t->state != TRK_HOLD) continue;
        o = &rec->tracks[n++];
        o->code_id = t->code_id;
        o->x = t->xr_q8;     o->y = t->yr_q8;   /* tau-extrapolated "now", not the lagged state    */
        o->vx = t->vx_q8;    o->vy = t->vy_q8;
        /* x_pred targets the NEXT tick (record contract): the corrected now-position plus one tick */
        o->x_pred = t->xr_q8 + (int32_t)(((int64_t)t->vx_q8 * 50000) / 1000000);
        o->y_pred = t->yr_q8 + (int32_t)(((int64_t)t->vy_q8 * 50000) / 1000000);
        o->cep = t->cep_q8;
        o->q = t->q_q8;
        o->lock_health = t->lock_health_q8;
        o->extent = t->extent_q8;
        o->scintillation = t->scint_q8;
        o->chip_hz = t->chip_hz_q8;
        o->chip_phase = t->chip_phase == 0xFF ? 0u : t->chip_phase;
        o->t_int_chips = t->t_int_chips;
        o->scale = t->scale;
        o->age_ms = t->age_ms;
        o->flags = (uint16_t)(BCN_F_VALID
                   | (t->state == TRK_CONFIRMED ? BCN_F_LOCK : 0u)
                   | (t->state == TRK_HOLD ? (BCN_F_HOLD | BCN_F_EXTRAPOLATED) : 0u)
                   | (t->measured_fix ? BCN_F_MEASURED_FIX : 0u)
                   | (t->saturated ? BCN_F_SATURATED : 0u)
                   | s->flags_extra);
    }
    return n;
}
