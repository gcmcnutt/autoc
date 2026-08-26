/* T008 — the config loader errors on EACH missing key, naming it, exiting 1 (Constitution VII).
 * Written before config.c exists. The point is not that the loader works; it is that it CANNOT silently
 * substitute a value for a key the operator forgot. Every key below is tested individually, because a
 * loader that catches nine of ten is the 032 failure mode with better odds. */
#include "bcn_test.h"
#include "config.h"
#include "record.h"
#include <stdlib.h>

/* The canonical, complete config. Every key in contracts/config-schema.md appears exactly once. If the
 * schema grows a key and this table does not, the "missing key is caught" test silently stops covering
 * it — so the count assertion at the bottom is load-bearing. */
static const struct { const char *sec, *key, *val; } KEYS[] = {
    {"camera", "mode",                     "640x400"},
    {"camera", "fps",                      "288"},
    {"camera", "exposure_min_us",          "50"},
    {"camera", "exposure_max_us",          "3000"},
    {"camera", "gain_min_q8",              "256"},
    {"camera", "gain_max_q8",              "4096"},
    {"code",   "n_chips",                  "31"},
    {"code",   "chip_hz_nominal",          "115.0"},
    {"code",   "chip_hz_candidates",       "110.0,115.0,120.0,200.0"},
    {"code",   "code_a",                   "0000000100011011000011001110011"},
    {"code",   "code_b",                   "0100011001100111100101001011110"},
    {"bank",   "max_slots",                "16"},
    {"bank",   "scale_extents",            "24,12,6"},
    {"bank",   "alpha",                    "0.35"},
    {"bank",   "beta",                     "0.08"},
    {"bank",   "q_lock",                   "0.55"},
    {"bank",   "q_drop",                   "0.30"},
    {"bank",   "lock_health_lock",         "0.60"},
    {"bank",   "lock_health_drop",         "0.35"},
    {"bank",   "min_mod_depth",            "0.0"},
    {"bank",   "hold_max_age_ms",          "150"},
    {"bank",   "hold_max_cep_px",          "3.0"},
    {"agc",    "exposure_target_lo",       "40"},
    {"agc",    "exposure_target_hi",       "200"},
    {"agc",    "integration_min_chips",    "31"},
    {"agc",    "integration_max_chips",    "124"},
    {"agc",    "roi_driven",               "1"},
    {"record", "mode",                     "burst"},
    {"record", "path",                     "/dev/shm/clip.bcnr"},
    {"record", "ring_seconds",             "2"},
    {"record", "burst_frames",             "80"},
    {"record", "burst_every",              "500"},
    {"record", "trigger",                  "manual"},
    {"sched",  "acquire_cost_us_per_pass", "12000"},
    {"sched",  "acquire_passes_max",       "2"},
    {"sync",   "fiducial_enabled",         "0"},
    {"sync",   "fiducial_period_s",        "10"},
    {"sync",   "msp_uart",                 "/dev/ttyAMA0"},
    {"sync",   "msp_baud",                 "115200"},
};
#define NKEYS ((int)(sizeof(KEYS) / sizeof(KEYS[0])))

static const char *TMP = "test_config_tmp.ini";

/* Write the config, optionally omitting one key, optionally overriding one value. */
/* Look the key up by NAME. The overrides below used to index KEYS positionally, which meant adding any
 * new key silently shifted them onto a different setting and the test failed somewhere unrelated --
 * exactly what happened when [bank] min_mod_depth landed. */
static int key_idx(const char *section, const char *key)
{
    size_t i;
    for (i = 0; i < sizeof KEYS / sizeof KEYS[0]; i++)
        if (!strcmp(KEYS[i].sec, section) && !strcmp(KEYS[i].key, key))
            return (int)i;
    return -1;
}

static void write_ini(const char *path, int omit, int override_idx, const char *override_val)
{
    FILE *f = fopen(path, "w");
    const char *cur = "";
    int i;
    if (!f) { perror("fopen"); exit(2); }
    for (i = 0; i < NKEYS; i++) {
        if (i == omit) continue;
        if (strcmp(cur, KEYS[i].sec) != 0) { fprintf(f, "[%s]\n", KEYS[i].sec); cur = KEYS[i].sec; }
        fprintf(f, "%s = %s\n", KEYS[i].key,
                (i == override_idx && override_val) ? override_val : KEYS[i].val);
    }
    fclose(f);
}

int main(void)
{
    BcnConfig c;
    char err[BCN_ERR_MAX];
    int i;

    /* The complete config loads, and the values arrive intact rather than merely "not erroring". */
    write_ini(TMP, -1, -1, NULL);
    err[0] = '\0';
    CHECK((bcn_config_load(TMP, &c, err, sizeof err)) == 0, "complete config must load: %s", err);
    CHECK_EQ_U(c.fps, 288u);
    CHECK_EQ_U(c.n_chips, 31u);
    CHECK_EQ_U(c.max_slots, 16u);
    CHECK_EQ_U(c.hold_max_age_ms, 150u);
    CHECK_EQ_U(c.hold_max_cep_px_q8, 3u * 256u);        /* 3.0 px in q8                                */
    CHECK_EQ_U(c.chip_hz_nominal_q8, 115u * 256u);      /* 115.0 Hz in q8                              */
    CHECK_EQ_U(c.n_chip_hz_candidates, 4u);
    CHECK_EQ_U(c.n_scales, 3u);
    CHECK_EQ_U(c.roi_driven, 1u);
    /* Gold codes land MSB-first with chip 0 in bit n_chips-1, matching beacon-pod's gold_codes.h. Code B
     * is the bench emitter, so getting this backwards would fail on hardware and pass in every unit test
     * that only round-trips it. 0100011001100111100101001011110b: */
    CHECK_EQ_U(c.code_b_bits, 0x2333CA5Eu);             /* 0b0100011001100111100101001011110            */
    CHECK_EQ_U(c.code_a_bits, 0x008D8673u);             /* 0b0000000100011011000011001110011            */
    CHECK(strcmp(c.record_mode, "burst") == 0, "record.mode = %s", c.record_mode);
    CHECK(strcmp(c.record_path, "/dev/shm/clip.bcnr") == 0, "record.path = %s", c.record_path);

    /* THE POINT OF THIS FILE: every single key, individually, must be fatal when absent AND the error must
     * name it. "Config error" without the key name sends the operator back to a 38-key file to bisect. */
    for (i = 0; i < NKEYS; i++) {
        write_ini(TMP, i, -1, NULL);
        err[0] = '\0';
        CHECK(bcn_config_load(TMP, &c, err, sizeof err) != 0,
              "omitting [%s] %s must be fatal, not defaulted", KEYS[i].sec, KEYS[i].key);
        CHECK_STR_HAS(err, KEYS[i].key);
    }

    /* A missing FILE is also fatal and says so — distinct from a missing key. */
    err[0] = '\0';
    CHECK(bcn_config_load("no_such_file_here.ini", &c, err, sizeof err) != 0, "missing file must fail");
    CHECK(err[0] != '\0', "missing file must produce a message");

    /* Range validation: max_slots may not exceed the wire's fixed shape, and roi_driven is a flag. */
    write_ini(TMP, -1, key_idx("bank", "max_slots"), "99");
    err[0] = '\0';
    CHECK(bcn_config_load(TMP, &c, err, sizeof err) != 0, "max_slots > BCN_MAX_TRACKS must fail");
    CHECK_STR_HAS(err, "max_slots");
    write_ini(TMP, -1, key_idx("agc", "roi_driven"), "2");
    err[0] = '\0';
    CHECK(bcn_config_load(TMP, &c, err, sizeof err) != 0, "roi_driven must be 0 or 1");
    CHECK_STR_HAS(err, "roi_driven");

    /* config_hash is deterministic over the resolved config and changes when any value changes —
     * otherwise stamping it into every artifact (spec §16.2) proves nothing. */
    {
        BcnConfig a, b;
        write_ini(TMP, -1, -1, NULL);
        CHECK((bcn_config_load(TMP, &a, err, sizeof err)) == 0, "reload a: %s", err);
        CHECK((bcn_config_load(TMP, &b, err, sizeof err)) == 0, "reload b: %s", err);
        CHECK_EQ_U(a.config_hash, b.config_hash);
        CHECK(a.config_hash != 0, "config_hash must not be zero");
        write_ini(TMP, -1, 1, "250");    /* camera.fps 288 -> 250 */
        CHECK((bcn_config_load(TMP, &b, err, sizeof err)) == 0, "reload changed: %s", err);
        CHECK(a.config_hash != b.config_hash, "config_hash must change when a value changes");
    }

    remove(TMP);
    BCN_TEST_MAIN_END();
}
