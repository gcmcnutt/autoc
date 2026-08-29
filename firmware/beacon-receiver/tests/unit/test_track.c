/* T076(b) — the widen re-binning must move the chip window onto the coarser plane WITHOUT losing or
 * inventing evidence.
 *
 * Why this test exists rather than trusting the golden clip: the golden fires the widen path exactly
 * twice in three seconds, and at both of those moments the track had already lost the beacon — so the
 * clip's before/after is identical whether the rebin is right, wrong, or a no-op. A transform with an
 * off-by-one in its centring would sail straight through it. The identity below is what the design
 * actually rests on, so it is asserted directly.
 *
 * THE IDENTITY. extract() gives plane pixel p the sum of f x f native pixels, normalised by
 * exposure x gain. A coarse pixel (factor f_new = r * f_old) therefore covers exactly the native area of
 * the r x r fine pixels beneath it, and is exactly their sum. Both apertures are centred on the same
 * predicted position, so the mapping is a pure scale about the centre:
 *
 *     old_x = E_old/2 + (new_x - E_new/2) * r        (and likewise in y)
 *
 * which is the same arithmetic engine.c's extract() does when it forms nx0 from (cx - extent/2 + px)*f.
 */
#include "bcn_test.h"
#include "track.h"
#include <stdlib.h>
#include <string.h>

#define E_OLD 12u        /* TRK_SCALE_MEDIUM, f = 2 — the default scale_extents = 24,12,6 */
#define E_NEW 24u        /* TRK_SCALE_COARSE, f = 4 */
#define R     2u         /* f_new / f_old */

/* Deterministic per-(pixel,bin) value with no symmetry, so a transposed or mis-centred mapping cannot
 * coincidentally produce the right sums. */
static int32_t val(uint16_t p, uint16_t b)
{
    return (int32_t)(p * 7919u + b * 104729u + 13u) % 100000 - 50000;
}

int main(void)
{
    Track *t = calloc(1, sizeof *t);
    uint16_t p, b;
    int64_t sum_before = 0, sum_after = 0;

    if (!t) { fprintf(stderr, "out of memory\n"); return 2; }

    /* A window with every chip present, and a distinct value in every (pixel, bin) cell. */
    for (b = 0; b < TRK_WIN; b++) t->counts[b] = (uint8_t)(1u + (b % 3u));
    for (p = 0; p < E_OLD * E_OLD; p++)
        for (b = 0; b < TRK_WIN; b++) {
            t->bins[(size_t)p * TRK_WIN + b] = val(p, b);
            sum_before += val(p, b);
        }

    track_rebin_coarser(t, (uint8_t)E_OLD, (uint8_t)E_NEW, (uint8_t)R);

    /* 1. Every new pixel is the sum of the r x r old pixels the centring maps onto it, or 0 where the
     *    old aperture did not reach. Checked on EVERY cell of the new plane, not a sample. */
    {
        int16_t nx, ny;
        int covered = 0, empty = 0;
        for (ny = 0; ny < (int16_t)E_NEW; ny++) {
            for (nx = 0; nx < (int16_t)E_NEW; nx++) {
                int16_t oy0 = (int16_t)(E_OLD / 2 + (ny - (int16_t)E_NEW / 2) * (int16_t)R);
                int16_t ox0 = (int16_t)(E_OLD / 2 + (nx - (int16_t)E_NEW / 2) * (int16_t)R);
                int inside = (oy0 >= 0 && ox0 >= 0 &&
                              oy0 + (int16_t)R <= (int16_t)E_OLD && ox0 + (int16_t)R <= (int16_t)E_OLD);
                uint16_t pn = (uint16_t)(ny * (int16_t)E_NEW + nx);
                if (inside) covered++; else empty++;
                for (b = 0; b < TRK_WIN; b += 17u) {     /* stride the bins: 8 per pixel, all 576 pixels */
                    int32_t want = 0;
                    uint8_t dy, dx;
                    for (dy = 0; dy < R; dy++)
                        for (dx = 0; dx < R; dx++) {
                            int16_t oy = (int16_t)(oy0 + dy), ox = (int16_t)(ox0 + dx);
                            if (oy >= 0 && ox >= 0 && oy < (int16_t)E_OLD && ox < (int16_t)E_OLD)
                                want += val((uint16_t)(oy * (int16_t)E_OLD + ox), b);
                        }
                    CHECK_EQ_I(t->bins[(size_t)pn * TRK_WIN + b], want);
                }
            }
        }
        /* The old aperture is 12 px at f=2 = 24 native px; the new plane is 24 px at f=4 = 96 native.
         * So exactly a 6x6 block of the new plane is covered and the remaining 540 pixels are empty. */
        CHECK_EQ_I(covered, 36);
        CHECK_EQ_I(empty, (int)(E_NEW * E_NEW) - 36);
    }

    /* 2. Nothing is lost and nothing is invented: total evidence is conserved exactly. This is the
     *    property that distinguishes a re-binning from a resampling. */
    for (p = 0; p < E_NEW * E_NEW; p++)
        for (b = 0; b < TRK_WIN; b++) sum_after += t->bins[(size_t)p * TRK_WIN + b];
    CHECK(sum_before == sum_after,
          "evidence must be conserved: %lld before, %lld after", (long long)sum_before,
          (long long)sum_after);

    /* 3. counts[] is per-CHIP and shared by every pixel, so it must survive untouched — that is what
     *    keeps window()'s "8 populated chips" test satisfied across the move, which is the whole reason
     *    a widened track can still correlate instead of starting a 258 ms rebuild. */
    for (b = 0; b < TRK_WIN; b++) CHECK_EQ_U(t->counts[b], 1u + (b % 3u));

    /* 4. The centre of the old aperture must land on the centre of the new one. A mis-centred rebin
     *    would put the beacon's accumulated evidence off-axis, which is precisely the failure the
     *    golden clip cannot see. */
    {
        uint16_t centre_new = (uint16_t)((E_NEW / 2u) * E_NEW + E_NEW / 2u);
        uint16_t o = (uint16_t)((E_OLD / 2u) * E_OLD + E_OLD / 2u);   /* old centre pixel */
        int32_t want = val(o, 0) + val((uint16_t)(o + 1u), 0)
                     + val((uint16_t)(o + E_OLD), 0) + val((uint16_t)(o + E_OLD + 1u), 0);
        CHECK_EQ_I(t->bins[(size_t)centre_new * TRK_WIN + 0], want);
    }

    free(t);
    BCN_TEST_MAIN_END();
}
