/* T033 — corr_track reproduces corr_search's answer at a known phase; plus the properties the design
 * leans on: Gold cross-code rejection, missing-bin tolerance, DC immunity, scale-freedom of q. */
#include "bcn_test.h"
#include "corr.h"
#include <string.h>

#define CODE_A_BITS 0x008D8673u
#define CODE_B_BITS 0x2333CA5Eu

/* Build a W-bin window of code `bits` at chip phase `ph`, amplitude amp, pedestal ped, count spc. */
static void synth(uint32_t bits, uint8_t ph, int32_t amp, int32_t ped, uint8_t spc,
                  int32_t *bins, uint8_t *counts, uint16_t W)
{
    int8_t t[CORR_N];
    uint16_t k;
    corr_template(bits, t);
    for (k = 0; k < W; k++) {
        int8_t chip = t[(k + ph) % CORR_N];
        bins[k] = (ped + chip * amp) * spc;    /* bin = sum of spc identical samples */
        counts[k] = spc;
    }
}

int main(void)
{
    int8_t ta[CORR_N], tb[CORR_N];
    int32_t bins[124];
    uint8_t counts[124];
    CorrResult s, t;

    corr_template(CODE_A_BITS, ta);
    corr_template(CODE_B_BITS, tb);

    /* templates: chip 0 of B is '0' -> -1; the code has 16 ones (Gold balance) */
    {
        int k, ones = 0;
        CHECK_EQ_I(tb[0], -1);
        CHECK_EQ_I(tb[1], 1);
        for (k = 0; k < CORR_N; k++) ones += tb[k] > 0;
        CHECK_EQ_I(ones, 16);
    }

    /* ---- search finds code B at every phase; track reproduces search exactly (T033 proper) ---- */
    {
        uint8_t ph;
        for (ph = 0; ph < CORR_N; ph++) {
            synth(CODE_B_BITS, ph, 100, 5000, 2, bins, counts, 31);
            corr_search(bins, counts, 31, ta, tb, &s);
            CHECK_EQ_U(s.code_id, 1u);
            CHECK_EQ_U(s.phase, ph);
            corr_track(bins, counts, 31, tb, ph, &t);
            CHECK_EQ_I(t.corr, s.corr);
            CHECK_EQ_U(t.q_q8, s.q_q8);
            CHECK(s.q_q8 > 230, "clean signal must score q near 256, got %u at ph %u", s.q_q8, ph);
        }
    }

    /* ---- cross-code rejection: code A input scored against B stays under the Gold bound (9/31) ---- */
    {
        synth(CODE_A_BITS, 7, 100, 5000, 2, bins, counts, 31);
        corr_search(bins, counts, 31, ta, tb, &s);
        CHECK_EQ_U(s.code_id, 0u);
        CHECK_EQ_U(s.phase, 7u);
        corr_track(bins, counts, 31, tb, 0, &t);
        {
            uint8_t ph;
            uint16_t worst = 0;
            for (ph = 0; ph < CORR_N; ph++) {
                corr_track(bins, counts, 31, tb, ph, &t);
                if (t.q_q8 > worst) worst = t.q_q8;
            }
            /* 9/31 of 256 = 74.3 -> integer arithmetic keeps every phase at or under ~80 */
            CHECK(worst <= 90, "cross-code q must stay near the 9/31 bound, got %u", worst);
        }
    }

    /* ---- DC immunity: pedestal 100x the amplitude changes NOTHING (mean removal is exact) ---- */
    {
        CorrResult lo, hi;
        synth(CODE_B_BITS, 11, 50, 0, 3, bins, counts, 31);
        corr_search(bins, counts, 31, ta, tb, &lo);
        synth(CODE_B_BITS, 11, 50, 5000, 3, bins, counts, 31);
        corr_search(bins, counts, 31, ta, tb, &hi);
        CHECK_EQ_I(hi.corr, lo.corr);
        CHECK_EQ_U(hi.q_q8, lo.q_q8);
    }

    /* ---- scale-freedom: 8x the amplitude, same q (it is corr/energy) ---- */
    {
        CorrResult a, b;
        synth(CODE_B_BITS, 3, 40, 1000, 2, bins, counts, 31);
        corr_search(bins, counts, 31, ta, tb, &a);
        synth(CODE_B_BITS, 3, 320, 1000, 2, bins, counts, 31);
        corr_search(bins, counts, 31, ta, tb, &b);
        CHECK(a.q_q8 >= b.q_q8 - 2 && a.q_q8 <= b.q_q8 + 2,
              "q must be amplitude-free: %u vs %u", a.q_q8, b.q_q8);
    }

    /* ---- missing chips: knock out 6 bins (occlusion) — identity and phase must survive ---- */
    {
        int k;
        synth(CODE_B_BITS, 19, 100, 2000, 2, bins, counts, 31);
        for (k = 4; k < 10; k++) { bins[k] = 0; counts[k] = 0; }
        corr_search(bins, counts, 31, ta, tb, &s);
        CHECK_EQ_U(s.code_id, 1u);
        CHECK_EQ_U(s.phase, 19u);
        CHECK(s.q_q8 > 200, "25 missing chips of 31... 6 of 31 missing should still score, got %u", s.q_q8);
    }

    /* ---- multi-word window: 93 bins (3 words) at a known phase ---- */
    {
        synth(CODE_B_BITS, 5, 60, 800, 2, bins, counts, 93);
        corr_search(bins, counts, 93, ta, tb, &s);
        CHECK_EQ_U(s.code_id, 1u);
        CHECK_EQ_U(s.phase, 5u);
        corr_track(bins, counts, 93, tb, 5, &t);
        CHECK_EQ_I(t.corr, s.corr);
    }

    /* ---- chip_at: exactness at the bench rate. 115 Hz exactly: chip 115 lands at t = 1 s ---- */
    {
        uint32_t hz_q8 = 115u * 256u;
        CHECK_EQ_I(corr_chip_at(1000000u, 0u, hz_q8), 115);
        CHECK_EQ_I(corr_chip_at(999999u, 0u, hz_q8), 114);   /* floor, not round */
        CHECK_EQ_I(corr_chip_at(0u, 0u, hz_q8), 0);
        /* 8.6956 ms = one chip at 115 Hz: floor boundaries land where 64-bit exact math says */
        CHECK_EQ_I(corr_chip_at(8695u, 0u, hz_q8), 0);
        CHECK_EQ_I(corr_chip_at(8696u, 0u, hz_q8), 1);
        /* q8 fraction: 115.79 Hz = 29642.24 -> q8 29642 */
        CHECK_EQ_I(corr_chip_at(1000000u, 0u, 29642u), 115);
    }

    BCN_TEST_MAIN_END();
}
