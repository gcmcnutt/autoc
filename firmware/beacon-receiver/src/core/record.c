/* record.c — T011 reader-side version checks for the 20 Hz record (Constitution V). */
#include "record.h"
#include <stdio.h>
#include <string.h>

void bcn_record_init(BcnRecord *r, uint64_t build_id, uint64_t config_hash)
{
    memset(r, 0, sizeof *r);          /* tracks[] zeroed => VALID clear, the correct "nothing here" */
    r->magic          = BCN_RECORD_MAGIC;
    r->format_version = (uint16_t)BCN_RECORD_FORMAT_VERSION;
    r->header_bytes   = (uint16_t)BCN_RECORD_HEADER_BYTES;
    r->build_id       = build_id;
    r->config_hash    = config_hash;
}

int bcn_record_check(const BcnRecord *r, char *why, size_t why_len)
{
    if (!r || !why || why_len == 0) return -1;
    if (r->magic != BCN_RECORD_MAGIC) {
        snprintf(why, why_len, "record: bad magic 0x%08lX (expected 0x%08lX \"BCN1\") — this is not a "
                               "beacon record stream",
                 (unsigned long)r->magic, (unsigned long)BCN_RECORD_MAGIC);
        return -1;
    }
    if (r->format_version != BCN_RECORD_FORMAT_VERSION) {
        /* Both versions, always. Which side is stale is the only actionable fact, and a reader that says
         * only "version mismatch" forces the operator to go find out. Newer is an error too — there is no
         * partial interpretation of a layout we have not seen. */
        snprintf(why, why_len, "record: format_version %u but this reader implements %u — refusing to "
                               "interpret (Constitution V: no partial reads)",
                 (unsigned)r->format_version, (unsigned)BCN_RECORD_FORMAT_VERSION);
        return -1;
    }
    if (r->header_bytes != BCN_RECORD_HEADER_BYTES) {
        snprintf(why, why_len, "record: header_bytes %u but version %u defines %u",
                 (unsigned)r->header_bytes, (unsigned)BCN_RECORD_FORMAT_VERSION,
                 (unsigned)BCN_RECORD_HEADER_BYTES);
        return -1;
    }
    if (r->n_tracks > BCN_MAX_TRACKS) {
        snprintf(why, why_len, "record: n_tracks %u exceeds the fixed shape of %d",
                 (unsigned)r->n_tracks, BCN_MAX_TRACKS);
        return -1;
    }
    return 0;
}

/* ---- explicit little-endian codec (see record.h). Deliberately byte-by-byte: it is the thing the golden
 * vectors verify, and a memcpy would make that test a tautology. */
static void p16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void p32(uint8_t *p, uint32_t v) { int i; for (i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static void p64(uint8_t *p, uint64_t v) { int i; for (i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static uint16_t g16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t g32(const uint8_t *p)
{ uint32_t v = 0; int i; for (i = 0; i < 4; i++) v |= (uint32_t)p[i] << (8 * i); return v; }
static uint64_t g64(const uint8_t *p)
{ uint64_t v = 0; int i; for (i = 0; i < 8; i++) v |= (uint64_t)p[i] << (8 * i); return v; }

static void track_encode(const BcnTrack *t, uint8_t *o)
{
    p32(o +  0, (uint32_t)t->x);       p32(o +  4, (uint32_t)t->y);
    p32(o +  8, (uint32_t)t->vx);      p32(o + 12, (uint32_t)t->vy);
    p32(o + 16, (uint32_t)t->x_pred);  p32(o + 20, (uint32_t)t->y_pred);
    p32(o + 24, t->chip_hz);
    p16(o + 28, t->cep);               p16(o + 30, t->q);
    p16(o + 32, t->lock_health);       p16(o + 34, t->extent);
    p16(o + 36, t->scintillation);     p16(o + 38, t->flags);
    p16(o + 40, t->age_ms);
    o[42] = t->code_id; o[43] = t->chip_phase; o[44] = t->t_int_chips; o[45] = t->scale;
    p16(o + 46, 0u);
}

static void track_decode(const uint8_t *i, BcnTrack *t)
{
    t->x  = (int32_t)g32(i +  0); t->y  = (int32_t)g32(i +  4);
    t->vx = (int32_t)g32(i +  8); t->vy = (int32_t)g32(i + 12);
    t->x_pred = (int32_t)g32(i + 16); t->y_pred = (int32_t)g32(i + 20);
    t->chip_hz = g32(i + 24);
    t->cep = g16(i + 28); t->q = g16(i + 30);
    t->lock_health = g16(i + 32); t->extent = g16(i + 34);
    t->scintillation = g16(i + 36); t->flags = g16(i + 38);
    t->age_ms = g16(i + 40);
    t->code_id = i[42]; t->chip_phase = i[43]; t->t_int_chips = i[44]; t->scale = i[45];
    t->_pad = 0u;
}

void bcn_record_encode(const BcnRecord *r, uint8_t out[BCN_RECORD_WIRE_BYTES])
{
    int k;
    memset(out, 0, BCN_RECORD_WIRE_BYTES);
    p32(out +  0, r->magic);
    p16(out +  4, r->format_version);
    p16(out +  6, r->header_bytes);
    p64(out +  8, r->t_us);
    p32(out + 16, r->seq);
    p32(out + 20, r->tick_index);
    out[24] = r->n_tracks;
    out[25] = r->n_slots_used;
    p16(out + 26, 0u);
    p32(out + 28, (uint32_t)r->deadline_margin_us);
    p64(out + 32, r->build_id);
    p64(out + 40, r->config_hash);
    p64(out + 48, r->inav_t_us);
    p32(out + 56, r->inav_read_age_us);
    p32(out + 60, r->gps_time_ms);
    for (k = 0; k < BCN_MAX_TRACKS; k++)
        track_encode(&r->tracks[k], out + BCN_RECORD_HEADER_BYTES + (size_t)k * 48u);
}

int bcn_record_decode(const uint8_t in[BCN_RECORD_WIRE_BYTES], BcnRecord *out)
{
    int k;
    if (!in || !out) return -1;
    memset(out, 0, sizeof *out);
    out->magic              = g32(in +  0);
    out->format_version     = g16(in +  4);
    out->header_bytes       = g16(in +  6);
    out->t_us               = g64(in +  8);
    out->seq                = g32(in + 16);
    out->tick_index         = g32(in + 20);
    out->n_tracks           = in[24];
    out->n_slots_used       = in[25];
    out->deadline_margin_us = (int32_t)g32(in + 28);
    out->build_id           = g64(in + 32);
    out->config_hash        = g64(in + 40);
    out->inav_t_us          = g64(in + 48);
    out->inav_read_age_us   = g32(in + 56);
    out->gps_time_ms        = g32(in + 60);
    for (k = 0; k < BCN_MAX_TRACKS; k++)
        track_decode(in + BCN_RECORD_HEADER_BYTES + (size_t)k * 48u, &out->tracks[k]);
    return 0;
}
