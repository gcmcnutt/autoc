/* config.h — runtime configuration (contracts/config-schema.md, data-model.md §6).
 *
 * CONSTITUTION VII BINDS, and it is the reason this file is verbose. There are NO in-code defaults for any
 * key. A missing key is a startup error that NAMES the key and exits 1. This is the 032 cepGateThreshold
 * failure mode — correct-looking results produced by a stale hardcoded value — and every threshold below
 * is exactly that kind of key.
 *
 * Fields that legitimately zero-init carry a "default-ok: <reason>" annotation per the principle.
 *
 * DEVIATION FROM plan.md's DEPENDENCY LIST, recorded deliberately: the plan names inih for config, but
 * core/ is specified as zero-dependency and that property is called load-bearing (plan.md §16.1). It is
 * also load-bearing in practice — the Pi and the WSL2 cross path configure with BUILD_AUTOC=OFF, which
 * never reaches the repo's inih FetchContent, and a fresh Pi would need network to get it. So the ~60-line
 * INI subset this schema needs is hand-written in config.c. inih remains the parser for the autoc side.
 */
#ifndef BEACON_CONFIG_H
#define BEACON_CONFIG_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BCN_MAX_CHIP_CANDIDATES 8
#define BCN_MAX_SCALES          3
#define BCN_PATH_MAX            256
#define BCN_ERR_MAX             256

typedef struct {
    /* ---- [camera] */
    char     camera_mode[16];        /* "640x400" | "640x200"                                          */
    uint32_t fps;
    uint32_t exposure_min_us, exposure_max_us;
    uint16_t gain_min_q8, gain_max_q8;

    /* ---- [code] */
    uint8_t  n_chips;                /* 31                                                             */
    uint32_t chip_hz_nominal_q8;
    uint32_t chip_hz_candidates_q8[BCN_MAX_CHIP_CANDIDATES];
    uint8_t  n_chip_hz_candidates;
    uint32_t code_a_bits;            /* chip 0 in bit (n_chips-1), MSB-first — matches gold_codes.h     */
    uint32_t code_b_bits;

    /* ---- [bank] */
    uint8_t  max_slots;              /* <= BCN_MAX_TRACKS                                              */
    uint16_t scale_extents[BCN_MAX_SCALES];  /* px per correlator window, coarse -> fine               */
    uint8_t  n_scales;
    uint16_t alpha_q8, beta_q8;      /* alpha-beta centering gains                                     */
    uint16_t q_lock_q8, q_drop_q8;
    uint16_t lock_health_lock_q8, lock_health_drop_q8;
    /* These two MUST equal the spec §3.1 validity bounds. The HOLD state machine and the envelope scorer
     * read the same two numbers from the same place so they cannot drift apart. */
    uint16_t hold_max_age_ms;
    uint16_t hold_max_cep_px_q8;

    /* ---- [agc] */
    uint16_t exposure_target_lo, exposure_target_hi;
    uint8_t  integration_min_chips, integration_max_chips;
    uint8_t  roi_driven;             /* must be true in flight (spec §4/§9); loader enforces 0|1        */

    /* ---- [record] */
    char     record_mode[16];        /* continuous | ring | burst                                      */
    char     record_path[BCN_PATH_MAX];
    uint32_t ring_seconds;
    uint32_t burst_frames, burst_every;
    char     record_trigger[16];     /* manual | gpio                                                  */

    /* ---- [sched] — the cost model R3's replay virtualisation replays against */
    uint32_t acquire_cost_us_per_pass;
    uint32_t acquire_passes_max;

    /* ---- [sync] (spec §7.1.2) */
    uint8_t  fiducial_enabled;
    uint32_t fiducial_period_s;
    char     msp_uart[64];
    uint32_t msp_baud;

    /* ---- computed, not read from file (spec §16.2) */
    uint64_t config_hash;            /* over the fully-resolved config                                 */
    uint64_t build_id;
} BcnConfig;

/* Load and fully validate. Returns 0 on success. On failure returns non-zero and writes a message into
 * `err` that NAMES the offending key — the caller prints it and exits 1 (contracts/cli.md). */
int bcn_config_load(const char *path, BcnConfig *out, char *err, size_t err_len);

/* T012: stamped into every record and every recording so any artifact traces to the settings that made it.
 * bcn_config_hash is called by bcn_config_load; exposed for tests. */
uint64_t bcn_config_hash(const BcnConfig *c);
uint64_t bcn_build_id(void);

#ifdef __cplusplus
}
#endif
#endif /* BEACON_CONFIG_H */
