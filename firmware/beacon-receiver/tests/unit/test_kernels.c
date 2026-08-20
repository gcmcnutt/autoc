/* T026 — scalar reduce/hipass kernels against HAND-COMPUTED vectors (written before reduce.c/hipass.c).
 * These vectors are the kernels' permanent oracle (R11): the NEON path must later match the scalar path
 * bit-for-bit, and the scalar path must match THESE numbers, worked by hand below. */
#include "bcn_test.h"
#include "reduce.h"
#include "hipass.h"
#include <string.h>

int main(void)
{
    /* ---- reduce2 on a 4x4 with a stride of 6 (stride handling is where kernels usually lie) ----
     * src rows (first 4 of each 6-wide row are real, Xs are stride padding never to be read):
     *    1   2   3   4   X X
     *    5   6   7   8   X X
     *    9  10  11  12   X X
     *   13  14  15  16   X X
     * 2x2 sums, hand-worked: [1+2+5+6, 3+4+7+8] = [14, 22]
     *                        [9+10+13+14, 11+12+15+16] = [46, 54]                                  */
    {
        uint8_t src[4 * 6];
        uint16_t out[4];
        int y, x;
        memset(src, 0xEE, sizeof src);            /* poison the padding: reads of it will show */
        for (y = 0; y < 4; y++)
            for (x = 0; x < 4; x++)
                src[y * 6 + x] = (uint8_t)(y * 4 + x + 1);
        reduce2_scalar(src, 6, 4, 4, out);
        CHECK_EQ_U(out[0], 14u);
        CHECK_EQ_U(out[1], 22u);
        CHECK_EQ_U(out[2], 46u);
        CHECK_EQ_U(out[3], 54u);
    }

    /* ---- reduce2 saturating input: 4 x 255 = 1020 must survive (u16, no clip) ---- */
    {
        uint8_t src[2 * 2] = { 255, 255, 255, 255 };
        uint16_t out[1] = { 0 };
        reduce2_scalar(src, 2, 2, 2, out);
        CHECK_EQ_U(out[0], 1020u);
    }

    /* ---- reduce4 on an 8x4 ramp, stride 8: two 4x4 blocks ----
     * left block rows: 0..3 / 8..11 / 16..19 / 24..27 -> sum = (0+1+2+3)+(8+..11)+(16..19)+(24..27)
     *   = 6 + 38 + 70 + 102 = 216
     * right block: +4 per element, 16 elements -> 216 + 64 = 280                                    */
    {
        uint8_t src[8 * 4];
        uint16_t out[2];
        int i;
        for (i = 0; i < 32; i++) src[i] = (uint8_t)i;
        reduce4_scalar(src, 8, 8, 4, out);
        CHECK_EQ_U(out[0], 216u);
        CHECK_EQ_U(out[1], 280u);
    }

    /* ---- reduce4 max: 16 x 255 = 4080 ---- */
    {
        uint8_t src[16];
        uint16_t out[1] = { 0 };
        memset(src, 255, sizeof src);
        reduce4_scalar(src, 4, 4, 4, out);
        CHECK_EQ_U(out[0], 4080u);
    }

    /* ---- hipass: flat field -> exactly zero everywhere (the property that makes it a high-pass) ---- */
    {
        uint16_t src[5 * 5];
        int32_t out[5 * 5];
        int i;
        for (i = 0; i < 25; i++) src[i] = 777u;
        hipass_scalar(src, 5, 5, out);
        for (i = 0; i < 25; i++) CHECK_EQ_I(out[i], 0);
    }

    /* ---- hipass: single bright pixel of 100 on zero background, 5x5.
     * centre (2,2): 9*100 - 100 = 800.
     * 8-neighbours of it, e.g. (1,1): 9*0 - 100 = -100.
     * (2,1) (edge-adjacent-to-centre): also -100. Ring at distance 2 that still sees it: none (3x3
     * support only). Border row/col: DEFINED zero.                                                   */
    {
        uint16_t src[5 * 5];
        int32_t out[5 * 5];
        int i;
        memset(src, 0, sizeof src);
        src[2 * 5 + 2] = 100u;
        hipass_scalar(src, 5, 5, out);
        CHECK_EQ_I(out[2 * 5 + 2], 800);
        CHECK_EQ_I(out[1 * 5 + 1], -100);
        CHECK_EQ_I(out[1 * 5 + 2], -100);
        CHECK_EQ_I(out[3 * 5 + 3], -100);
        for (i = 0; i < 5; i++) {
            CHECK_EQ_I(out[i], 0);                 /* top border  */
            CHECK_EQ_I(out[20 + i], 0);            /* bottom      */
            CHECK_EQ_I(out[i * 5], 0);             /* left        */
            CHECK_EQ_I(out[i * 5 + 4], 0);         /* right       */
        }
    }

    /* ---- hipass: gradient — a pure ramp has zero curvature, so the interior must be zero.
     * src[y][x] = 10*x: row [0,10,20,30,40]. Interior (x,y)=(2,1): 9*20 - (10+20+30)*3 = 180-180 = 0. */
    {
        uint16_t src[5 * 5];
        int32_t out[5 * 5];
        int x, y;
        for (y = 0; y < 5; y++) for (x = 0; x < 5; x++) src[y * 5 + x] = (uint16_t)(10 * x);
        hipass_scalar(src, 5, 5, out);
        for (y = 1; y < 4; y++) for (x = 1; x < 4; x++)
            CHECK_EQ_I(out[y * 5 + x], 0);
    }

    BCN_TEST_MAIN_END();
}
