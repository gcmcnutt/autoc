/* config.c — T010 loader + T012 config_hash/build_id.
 *
 * Constitution VII is the whole design: a key table, a seen-bitmap, and a missing key is a named, fatal
 * error. There is no code path that supplies a value the file did not.
 *
 * An UNKNOWN key is fatal too. That is not in the contract, and it is deliberate: a typo'd key is exactly
 * how a "missing" key gets reported by an operator who is certain they set it. Failing on the typo names
 * the actual problem instead of the symptom.
 */
#include "config.h"
#include "record.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#define MAX_LINE 512

typedef enum {
    T_STR, T_U32, T_U16, T_U8, T_Q8_16, T_Q8_32, T_CODE, T_FLAG, T_Q8LIST, T_U16LIST
} KType;

typedef struct {
    const char *sec, *key;
    KType  type;
    size_t off;        /* offset of the field (or array) in BcnConfig  */
    size_t cap;        /* T_STR: buffer size. lists: max entries.      */
    size_t count_off;  /* lists: offset of the count field             */
} KeySpec;

#define F(f) offsetof(BcnConfig, f)

/* The schema, in one place, matching contracts/config-schema.md key for key. */
static const KeySpec SPECS[] = {
    {"camera", "mode",                 T_STR,    F(camera_mode), sizeof(((BcnConfig*)0)->camera_mode), 0},
    {"camera", "fps",                  T_U32,    F(fps), 0, 0},
    {"camera", "exposure_min_us",      T_U32,    F(exposure_min_us), 0, 0},
    {"camera", "exposure_max_us",      T_U32,    F(exposure_max_us), 0, 0},
    {"camera", "gain_min_q8",          T_U16,    F(gain_min_q8), 0, 0},
    {"camera", "gain_max_q8",          T_U16,    F(gain_max_q8), 0, 0},

    {"code",   "n_chips",              T_U8,     F(n_chips), 0, 0},
    {"code",   "chip_hz_nominal",      T_Q8_32,  F(chip_hz_nominal_q8), 0, 0},
    {"code",   "chip_hz_candidates",   T_Q8LIST, F(chip_hz_candidates_q8), BCN_MAX_CHIP_CANDIDATES,
                                                 F(n_chip_hz_candidates)},
    {"code",   "code_a",               T_CODE,   F(code_a_bits), 0, 0},
    {"code",   "code_b",               T_CODE,   F(code_b_bits), 0, 0},

    {"bank",   "max_slots",            T_U8,     F(max_slots), 0, 0},
    {"bank",   "scale_extents",        T_U16LIST,F(scale_extents), BCN_MAX_SCALES, F(n_scales)},
    {"bank",   "alpha",                T_Q8_16,  F(alpha_q8), 0, 0},
    {"bank",   "beta",                 T_Q8_16,  F(beta_q8), 0, 0},
    {"bank",   "q_lock",               T_Q8_16,  F(q_lock_q8), 0, 0},
    {"bank",   "q_drop",               T_Q8_16,  F(q_drop_q8), 0, 0},
    {"bank",   "lock_health_lock",     T_Q8_16,  F(lock_health_lock_q8), 0, 0},
    {"bank",   "lock_health_drop",     T_Q8_16,  F(lock_health_drop_q8), 0, 0},
    {"bank",   "hold_max_age_ms",      T_U16,    F(hold_max_age_ms), 0, 0},
    {"bank",   "hold_max_cep_px",      T_Q8_16,  F(hold_max_cep_px_q8), 0, 0},

    {"agc",    "exposure_target_lo",   T_U16,    F(exposure_target_lo), 0, 0},
    {"agc",    "exposure_target_hi",   T_U16,    F(exposure_target_hi), 0, 0},
    {"agc",    "integration_min_chips",T_U8,     F(integration_min_chips), 0, 0},
    {"agc",    "integration_max_chips",T_U8,     F(integration_max_chips), 0, 0},
    {"agc",    "roi_driven",           T_FLAG,   F(roi_driven), 0, 0},

    {"record", "mode",                 T_STR,    F(record_mode), sizeof(((BcnConfig*)0)->record_mode), 0},
    {"record", "path",                 T_STR,    F(record_path), sizeof(((BcnConfig*)0)->record_path), 0},
    {"record", "ring_seconds",         T_U32,    F(ring_seconds), 0, 0},
    {"record", "burst_frames",         T_U32,    F(burst_frames), 0, 0},
    {"record", "burst_every",          T_U32,    F(burst_every), 0, 0},
    {"record", "trigger",              T_STR,    F(record_trigger), sizeof(((BcnConfig*)0)->record_trigger), 0},

    {"sched",  "acquire_cost_us_per_pass", T_U32, F(acquire_cost_us_per_pass), 0, 0},
    {"sched",  "acquire_passes_max",   T_U32,    F(acquire_passes_max), 0, 0},

    {"sync",   "fiducial_enabled",     T_FLAG,   F(fiducial_enabled), 0, 0},
    {"sync",   "fiducial_period_s",    T_U32,    F(fiducial_period_s), 0, 0},
    {"sync",   "msp_uart",             T_STR,    F(msp_uart), sizeof(((BcnConfig*)0)->msp_uart), 0},
    {"sync",   "msp_baud",             T_U32,    F(msp_baud), 0, 0},
};
#define NSPECS ((int)(sizeof(SPECS) / sizeof(SPECS[0])))

/* ---- tiny parsers. No allocation, no locale, no float. ---- */

static char *trim(char *s)
{
    char *e;
    while (*s && isspace((unsigned char)*s)) s++;
    if (!*s) return s;
    e = s + strlen(s) - 1;
    while (e > s && isspace((unsigned char)*e)) *e-- = '\0';
    return s;
}

static int parse_u64(const char *s, uint64_t *out)
{
    uint64_t v = 0;
    int any = 0;
    while (*s && isspace((unsigned char)*s)) s++;
    while (isdigit((unsigned char)*s)) { v = v * 10u + (uint64_t)(*s++ - '0'); any = 1; }
    while (*s && isspace((unsigned char)*s)) s++;
    if (!any || *s) return -1;
    *out = v;
    return 0;
}

/* "115.0" -> 29440. Decimal string to q8 with no float anywhere: the fraction is accumulated as an
 * integer numerator over a power-of-ten denominator, then scaled with round-half-up. */
static int parse_q8(const char *s, uint32_t *out)
{
    uint64_t ip = 0, frac = 0, den = 1, q;
    int any = 0;
    while (*s && isspace((unsigned char)*s)) s++;
    while (isdigit((unsigned char)*s)) { ip = ip * 10u + (uint64_t)(*s++ - '0'); any = 1; }
    if (*s == '.') {
        s++;
        while (isdigit((unsigned char)*s)) {
            if (den <= 100000000ull) { frac = frac * 10u + (uint64_t)(*s - '0'); den *= 10u; }
            s++; any = 1;
        }
    }
    while (*s && isspace((unsigned char)*s)) s++;
    if (!any || *s) return -1;
    q = ip * 256ull + (frac * 256ull + den / 2u) / den;
    if (q > 0xFFFFFFFFull) return -1;
    *out = (uint32_t)q;
    return 0;
}

/* "0100011001100111100101001011110" -> 0x2333CA5E. Chip 0 is the leftmost character and lands in the
 * most significant used bit, matching beacon-pod's gold_codes.h. Getting this mirrored would still
 * round-trip through every unit test and fail only against hardware. */
static int parse_code(const char *s, uint32_t *out)
{
    uint32_t v = 0;
    int n = 0;
    while (*s == '0' || *s == '1') { v = (v << 1) | (uint32_t)(*s - '0'); s++; n++; }
    while (*s && isspace((unsigned char)*s)) s++;
    if (*s || n == 0 || n > 32) return -1;
    *out = v;
    return 0;
}

static int store(const KeySpec *k, BcnConfig *c, const char *val, char *err, size_t errlen)
{
    uint8_t *base = (uint8_t *)c;
    uint64_t u;
    uint32_t q;

    switch (k->type) {
    case T_STR:
        if (strlen(val) + 1u > k->cap) {
            snprintf(err, errlen, "config: [%s] %s is too long (max %d chars)",
                     k->sec, k->key, (int)k->cap - 1);
            return -1;
        }
        memcpy(base + k->off, val, strlen(val) + 1u);
        return 0;

    case T_U32: case T_U16: case T_U8:
        if (parse_u64(val, &u) != 0) {
            snprintf(err, errlen, "config: [%s] %s = \"%s\" is not a non-negative integer",
                     k->sec, k->key, val);
            return -1;
        }
        if ((k->type == T_U32 && u > 0xFFFFFFFFull) ||
            (k->type == T_U16 && u > 0xFFFFull) ||
            (k->type == T_U8  && u > 0xFFull)) {
            snprintf(err, errlen, "config: [%s] %s = %s is out of range", k->sec, k->key, val);
            return -1;
        }
        if (k->type == T_U32) { uint32_t v = (uint32_t)u; memcpy(base + k->off, &v, sizeof v); }
        else if (k->type == T_U16) { uint16_t v = (uint16_t)u; memcpy(base + k->off, &v, sizeof v); }
        else { uint8_t v = (uint8_t)u; memcpy(base + k->off, &v, sizeof v); }
        return 0;

    case T_FLAG:
        if (parse_u64(val, &u) != 0 || u > 1u) {
            snprintf(err, errlen, "config: [%s] %s = \"%s\" must be 0 or 1", k->sec, k->key, val);
            return -1;
        }
        { uint8_t v = (uint8_t)u; memcpy(base + k->off, &v, sizeof v); }
        return 0;

    case T_Q8_16: case T_Q8_32:
        if (parse_q8(val, &q) != 0) {
            snprintf(err, errlen, "config: [%s] %s = \"%s\" is not a decimal number",
                     k->sec, k->key, val);
            return -1;
        }
        if (k->type == T_Q8_16) {
            if (q > 0xFFFFu) {
                snprintf(err, errlen, "config: [%s] %s = %s overflows q8.16", k->sec, k->key, val);
                return -1;
            }
            { uint16_t v = (uint16_t)q; memcpy(base + k->off, &v, sizeof v); }
        } else {
            memcpy(base + k->off, &q, sizeof q);
        }
        return 0;

    case T_CODE:
        if (parse_code(val, &q) != 0) {
            snprintf(err, errlen, "config: [%s] %s = \"%s\" must be a binary chip string",
                     k->sec, k->key, val);
            return -1;
        }
        memcpy(base + k->off, &q, sizeof q);
        return 0;

    case T_Q8LIST: case T_U16LIST: {
        char buf[MAX_LINE];
        char *p, *tok;
        uint8_t n = 0;
        if (strlen(val) + 1u > sizeof buf) {
            snprintf(err, errlen, "config: [%s] %s list is too long", k->sec, k->key);
            return -1;
        }
        memcpy(buf, val, strlen(val) + 1u);
        p = buf;
        while ((tok = p) != NULL && *p) {
            char *comma = strchr(p, ',');
            if (comma) { *comma = '\0'; p = comma + 1; } else { p = buf + strlen(buf); }
            tok = trim(tok);
            if (!*tok) continue;
            if (n >= k->cap) {
                snprintf(err, errlen, "config: [%s] %s has more than %d entries",
                         k->sec, k->key, (int)k->cap);
                return -1;
            }
            if (k->type == T_Q8LIST) {
                if (parse_q8(tok, &q) != 0) {
                    snprintf(err, errlen, "config: [%s] %s entry \"%s\" is not a decimal number",
                             k->sec, k->key, tok);
                    return -1;
                }
                memcpy(base + k->off + n * sizeof(uint32_t), &q, sizeof q);
            } else {
                if (parse_u64(tok, &u) != 0 || u > 0xFFFFull) {
                    snprintf(err, errlen, "config: [%s] %s entry \"%s\" is not a 16-bit integer",
                             k->sec, k->key, tok);
                    return -1;
                }
                { uint16_t v = (uint16_t)u; memcpy(base + k->off + n * sizeof(uint16_t), &v, sizeof v); }
            }
            n++;
            if (comma == NULL) break;
        }
        if (n == 0) {
            snprintf(err, errlen, "config: [%s] %s must list at least one entry", k->sec, k->key);
            return -1;
        }
        memcpy(base + k->count_off, &n, sizeof n);
        return 0;
    }
    }
    snprintf(err, errlen, "config: [%s] %s has an unhandled type", k->sec, k->key);
    return -1;
}

/* Cross-key validation — the checks that only make sense once everything is present. */
static int validate(const BcnConfig *c, char *err, size_t errlen)
{
    if (c->max_slots == 0u || c->max_slots > BCN_MAX_TRACKS) {
        snprintf(err, errlen, "config: [bank] max_slots = %u must be 1..%d (the record's fixed shape)",
                 (unsigned)c->max_slots, BCN_MAX_TRACKS);
        return -1;
    }
    if (c->n_chips == 0u || c->n_chips > 32u) {
        snprintf(err, errlen, "config: [code] n_chips = %u must be 1..32", (unsigned)c->n_chips);
        return -1;
    }
    if (c->exposure_min_us > c->exposure_max_us) {
        snprintf(err, errlen, "config: [camera] exposure_min_us (%u) exceeds exposure_max_us (%u)",
                 c->exposure_min_us, c->exposure_max_us);
        return -1;
    }
    if (c->gain_min_q8 > c->gain_max_q8) {
        snprintf(err, errlen, "config: [camera] gain_min_q8 (%u) exceeds gain_max_q8 (%u)",
                 (unsigned)c->gain_min_q8, (unsigned)c->gain_max_q8);
        return -1;
    }
    if (c->integration_min_chips == 0u || c->integration_min_chips > c->integration_max_chips) {
        snprintf(err, errlen, "config: [agc] integration_min_chips (%u) must be 1..integration_max_chips (%u)",
                 (unsigned)c->integration_min_chips, (unsigned)c->integration_max_chips);
        return -1;
    }
    if (c->q_drop_q8 > c->q_lock_q8) {
        snprintf(err, errlen, "config: [bank] q_drop must not exceed q_lock (hysteresis inverted)");
        return -1;
    }
    if (c->lock_health_drop_q8 > c->lock_health_lock_q8) {
        snprintf(err, errlen, "config: [bank] lock_health_drop must not exceed lock_health_lock");
        return -1;
    }
    if (c->hold_max_age_ms == 0u || c->hold_max_cep_px_q8 == 0u) {
        snprintf(err, errlen, "config: [bank] hold_max_age_ms and hold_max_cep_px bound the HOLD state "
                              "AND the spec §3.1 metric; neither may be zero");
        return -1;
    }
    return 0;
}

/* FNV-1a over the resolved config, excluding the two computed fields (which are last). The struct is
 * memset to zero before loading, so padding bytes are deterministic and the hash is reproducible across
 * runs and hosts — which is the only reason stamping it into artifacts proves anything (spec §16.2). */
uint64_t bcn_config_hash(const BcnConfig *c)
{
    const uint8_t *p = (const uint8_t *)c;
    size_t n = offsetof(BcnConfig, config_hash);
    uint64_t h = 1469598103934665603ull;
    size_t i;
    for (i = 0; i < n; i++) { h ^= p[i]; h *= 1099511628211ull; }
    return h;
}

uint64_t bcn_build_id(void)
{
#ifdef BCN_BUILD_ID
    return (uint64_t)BCN_BUILD_ID;
#else
    /* No git hash injected: fall back to the build timestamp so artifacts from different builds are at
     * least distinguishable. Never zero — zero would read as "unknown" everywhere it is stamped. */
    const char *s = __DATE__ __TIME__;
    uint64_t h = 1469598103934665603ull;
    while (*s) { h ^= (uint8_t)*s++; h *= 1099511628211ull; }
    return h | 1ull;
#endif
}

int bcn_config_load(const char *path, BcnConfig *out, char *err, size_t err_len)
{
    FILE *f;
    char line[MAX_LINE];
    char section[64];
    uint8_t seen[NSPECS];
    int lineno = 0, i;

    if (!path || !out || !err || err_len == 0) return -1;
    memset(out, 0, sizeof *out);      /* padding determinism for the hash; NOT a source of defaults */
    memset(seen, 0, sizeof seen);
    section[0] = '\0';
    err[0] = '\0';

    f = fopen(path, "r");
    if (!f) {
        snprintf(err, err_len, "config: cannot open \"%s\"", path);
        return -1;
    }

    while (fgets(line, sizeof line, f)) {
        char *s, *eq, *key, *val;
        lineno++;
        s = strchr(line, '#'); if (s) *s = '\0';
        s = strchr(line, ';'); if (s) *s = '\0';
        s = trim(line);
        if (!*s) continue;

        if (*s == '[') {
            char *close = strchr(s, ']');
            if (!close) {
                snprintf(err, err_len, "config: %s:%d: unterminated section header", path, lineno);
                fclose(f); return -1;
            }
            *close = '\0';
            snprintf(section, sizeof section, "%s", trim(s + 1));
            continue;
        }

        eq = strchr(s, '=');
        if (!eq) {
            snprintf(err, err_len, "config: %s:%d: expected \"key = value\"", path, lineno);
            fclose(f); return -1;
        }
        *eq = '\0';
        key = trim(s);
        val = trim(eq + 1);

        for (i = 0; i < NSPECS; i++) {
            if (strcmp(SPECS[i].sec, section) == 0 && strcmp(SPECS[i].key, key) == 0) break;
        }
        if (i == NSPECS) {
            /* Almost always a typo, and a typo here presents as a DIFFERENT key being missing. */
            snprintf(err, err_len, "config: %s:%d: unknown key \"%s\" in section [%s]",
                     path, lineno, key, section);
            fclose(f); return -1;
        }
        if (seen[i]) {
            snprintf(err, err_len, "config: %s:%d: [%s] %s is set twice", path, lineno, section, key);
            fclose(f); return -1;
        }
        if (store(&SPECS[i], out, val, err, err_len) != 0) { fclose(f); return -1; }
        seen[i] = 1u;
    }
    fclose(f);

    /* Constitution VII, the actual gate: nothing is defaulted, so anything unseen is fatal AND named. */
    for (i = 0; i < NSPECS; i++) {
        if (!seen[i]) {
            snprintf(err, err_len, "config: %s is missing required key \"%s\" in section [%s] "
                                   "(there are no defaults)", path, SPECS[i].key, SPECS[i].sec);
            return -1;
        }
    }

    if (validate(out, err, err_len) != 0) return -1;

    out->build_id    = bcn_build_id();
    out->config_hash = bcn_config_hash(out);
    return 0;
}
