/* T041 — track lifecycle transitions including the evidence-bounded HOLD exit, exercised at the
 * bank/track API level with hand-driven state (no frames needed). */
#include "bcn_test.h"
#include "bank.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *INI = "test_bank_tmp.ini";

static void write_ini(void)
{
    FILE *f = fopen(INI, "w");
    if (!f) { perror("fopen"); exit(2); }
    fprintf(f,
        "[camera]\nmode = 640x400\nfps = 250\nexposure_min_us = 200\nexposure_max_us = 3000\n"
        "gain_min_q8 = 1024\ngain_max_q8 = 4096\n"
        "[code]\nn_chips = 31\nchip_hz_nominal = 115.0\nchip_hz_candidates = 115.0\n"
        "code_a = 0000000100011011000011001110011\ncode_b = 0100011001100111100101001011110\n"
        "[bank]\nmax_slots = 8\nscale_extents = 24,12,6\nalpha = 0.35\nbeta = 0.08\n"
        "q_lock = 0.55\nq_drop = 0.30\nq_fix = 0.75\nlock_health_lock = 0.60\nlock_health_drop = 0.35\n"
        "min_mod_depth = 0.0\n"
        "hold_max_age_ms = 150\nhold_max_cep_px = 3.0\n"
        "[agc]\nexposure_target_lo = 40\nexposure_target_hi = 200\nintegration_min_chips = 31\n"
        "integration_max_chips = 124\nroi_driven = 1\n"
        "[record]\nmode = continuous\npath = /dev/null\nring_seconds = 1\nburst_frames = 80\n"
        "burst_every = 500\ntrigger = manual\n"
        "[sched]\nacquire_cost_us_per_pass = 12000\nacquire_passes_max = 2\n"
        "[sync]\nfiducial_enabled = 0\nfiducial_period_s = 10\nmsp_uart = /dev/null\nmsp_baud = 115200\n");
    fclose(f);
}

/* Put a slot's track into a hand-chosen condition. */
static void force(Track *t, uint8_t state, uint16_t q, uint16_t lh, uint8_t measured)
{
    t->state = state;
    t->q_q8 = q;
    t->lock_health_q8 = lh;
    t->measured_fix = measured;
}

int main(void)
{
    char err[256];
    BcnConfig cfg;
    Bank bank;
    int s0;

    write_ini();
    CHECK((bcn_config_load(INI, &cfg, err, sizeof err)) == 0, "config: %s", err);

    /* ---- promotion needs SUSTAINED health: 3 consecutive good ticks, not one ---- */
    bank_init(&bank, &cfg);
    s0 = bank_seed(&bank, &cfg, 1u, TRK_SCALE_MEDIUM, 0, 0, 115u * 256u, 0u, 1000000u);
    CHECK(s0 >= 0, "seed");
    CHECK_EQ_U(bank.slots[s0].role, (uint64_t)BANK_ROLE_CANDIDATE);

    force(&bank.slots[s0].trk, TRK_CANDIDATE, 200, 200, 1);
    bank_tick(&bank, &cfg);
    CHECK_EQ_U(bank.slots[s0].trk.state, (uint64_t)TRK_CANDIDATE);   /* 1 good tick: still candidate */
    bank_tick(&bank, &cfg);
    CHECK_EQ_U(bank.slots[s0].trk.state, (uint64_t)TRK_CANDIDATE);   /* 2                             */
    bank_tick(&bank, &cfg);
    CHECK_EQ_U(bank.slots[s0].trk.state, (uint64_t)TRK_CONFIRMED);   /* 3: promoted                   */
    CHECK_EQ_U(bank.slots[s0].role, (uint64_t)BANK_ROLE_PRECISION);
    /* ...and a guard was allocated and paired */
    CHECK(bank.slots[s0].partner >= 0, "guard allocated");
    CHECK_EQ_U(bank.slots[bank.slots[s0].partner].role, (uint64_t)BANK_ROLE_GUARD);
    CHECK_EQ_U(bank.slots[bank.slots[s0].partner].trk.chip_phase, bank.slots[s0].trk.chip_phase);

    /* ---- orphaned guard dies the same tick its precision dies ---- */
    {
        int g = bank.slots[s0].partner;
        bank.slots[s0].trk.state = TRK_DEAD;
        bank_tick(&bank, &cfg);
        CHECK_EQ_U(bank.slots[s0].used, 0u);
        CHECK_EQ_U(bank.slots[g].used, 0u);
    }

    /* ---- eviction priority: candidates never evict a PRECISION; a new candidate evicts the
     * lowest-lock_health CANDIDATE when full ---- */
    bank_init(&bank, &cfg);
    {
        int i, k = -1;
        /* fill all 8 slots with candidates of rising lock_health */
        for (i = 0; i < 8; i++) {
            k = bank_seed(&bank, &cfg, 1u, TRK_SCALE_MEDIUM, i << 8, 0, 115u * 256u, 0u, 1000000u);
            CHECK(k >= 0, "fill %d", i);
            bank.slots[k].trk.lock_health_q8 = (uint16_t)(50 + i * 10);
        }
        /* promote slot 7 to precision so it becomes untouchable */
        force(&bank.slots[7].trk, TRK_CONFIRMED, 200, 200, 1);
        bank.slots[7].role = BANK_ROLE_PRECISION;
        /* bank full: the next seed must evict the WEAKEST candidate (slot 0, lh=50) */
        k = bank_seed(&bank, &cfg, 0u, TRK_SCALE_MEDIUM, 99 << 8, 0, 200u * 256u, 0u, 2000000u);
        CHECK_EQ_I(k, 0);
        CHECK_EQ_U(bank.slots[0].trk.code_id, 0u);                   /* the new one landed there      */
        CHECK_EQ_U(bank.slots[7].role, (uint64_t)BANK_ROLE_PRECISION); /* untouched                   */
    }

    /* ---- mirror-pair rule: same code, both confirmed -> LOWER (bigger y) flagged, KEPT ---- */
    bank_init(&bank, &cfg);
    {
        int a = bank_seed(&bank, &cfg, 1u, TRK_SCALE_MEDIUM, 0, -(10 << 8), 115u * 256u, 0u, 1000000u);
        int b = bank_seed(&bank, &cfg, 1u, TRK_SCALE_MEDIUM, 0, (20 << 8), 115u * 256u, 0u, 1000000u);
        BcnRecord rec;
        force(&bank.slots[a].trk, TRK_CONFIRMED, 200, 200, 1);
        force(&bank.slots[b].trk, TRK_CONFIRMED, 200, 200, 1);
        bank.slots[a].role = BANK_ROLE_PRECISION;   /* keep bank_tick from re-running promotion path */
        bank.slots[b].role = BANK_ROLE_PRECISION;
        bank_tick(&bank, &cfg);
        CHECK_EQ_U(bank.slots[a].flags_extra & BCN_F_MULTIPATH_SUSPECT, 0u);
        CHECK(bank.slots[b].flags_extra & BCN_F_MULTIPATH_SUSPECT, "lower twin flagged");
        CHECK_EQ_U(bank.slots[b].used, 1u);                          /* KEPT, not deleted             */
        bcn_record_init(&rec, 0, 0);
        CHECK(bank_emit(&bank, &cfg, &rec) >= 2, "both emitted");
        CHECK(rec.tracks[0].flags & BCN_F_VALID, "valid 0");
        CHECK((rec.tracks[0].flags & BCN_F_MULTIPATH_SUSPECT) == 0 ||
              (rec.tracks[1].flags & BCN_F_MULTIPATH_SUSPECT) == 0, "only one flagged");
    }

    /* ---- HOLD entry widens the ladder; HOLD exits by evidence bound (age), -> DEAD ---- */
    bank_init(&bank, &cfg);
    {
        int a = bank_seed(&bank, &cfg, 1u, TRK_SCALE_MEDIUM, 0, 0, 115u * 256u, 0u, 1000000u);
        Track *t = &bank.slots[a].trk;
        uint64_t now = 1000000u;
        int ticks = 0;
        force(t, TRK_CONFIRMED, 200, 200, 1);
        t->scale = TRK_SCALE_FINE;
        t->extent = (uint8_t)cfg.scale_extents[TRK_SCALE_FINE];
        t->last_fix_us = now;
        /* starve it: no frames deposited, so track_tick's window is empty -> no measurement.
         * HOLD entry gates on q alone (lock_health does not yet demote — see track.c): q 60 < q_drop
         * 76.8. */
        force(t, TRK_CONFIRMED, 60, 50, 0);
        now += 50000u;
        track_tick(t, &cfg, now, 50000u);
        CHECK_EQ_U(t->state, (uint64_t)TRK_HOLD);
        CHECK_EQ_U(t->scale, (uint64_t)TRK_SCALE_MEDIUM);   /* widened on HOLD entry                */
        /* age out: hold_max_age_ms = 150 -> dead within ~4 ticks */
        while (t->state == TRK_HOLD && ticks < 10) {
            now += 50000u;
            track_tick(t, &cfg, now, 50000u);
            ticks++;
        }
        CHECK_EQ_U(t->state, (uint64_t)TRK_DEAD);
        CHECK(ticks <= 4, "age bound must fire within hold_max_age_ms (took %d ticks)", ticks);
        bank_tick(&bank, &cfg);
        CHECK_EQ_U(bank.slots[a].used, 0u);                 /* buried                                */
    }

    remove(INI);
    BCN_TEST_MAIN_END();
}
