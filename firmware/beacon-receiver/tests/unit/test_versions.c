/* T009 — readers fail LOUDLY on version mismatch, naming BOTH versions (Constitution V).
 * Written before the check helpers exist. The failure this prevents is not a crash; it is a reader
 * cheerfully interpreting a v2 artifact with v1 field offsets and producing plausible numbers. */
#include "bcn_test.h"
#include "record.h"
#include "container.h"
#include <string.h>

int main(void)
{
    char why[256];

    /* ---- record ---- */
    {
        BcnRecord r;
        bcn_record_init(&r, 0xABCDEF0123456789ull, 0x1122334455667788ull);
        CHECK_EQ_U(r.magic, BCN_RECORD_MAGIC);
        CHECK_EQ_U(r.format_version, BCN_RECORD_FORMAT_VERSION);
        CHECK_EQ_U(r.header_bytes, BCN_RECORD_HEADER_BYTES);
        why[0] = '\0';
        CHECK((bcn_record_check(&r, why, sizeof why)) == 0, "a freshly-built record must verify: %s", why);

        /* Wrong magic: not a record at all. */
        r.magic = 0xDEADBEEFu;
        why[0] = '\0';
        CHECK(bcn_record_check(&r, why, sizeof why) != 0, "bad magic must fail");
        CHECK_STR_HAS(why, "magic");
        r.magic = BCN_RECORD_MAGIC;

        /* Newer version: an ERROR, explicitly not a best-effort read. Both versions must be named, or the
         * operator cannot tell which side is stale — which is the only actionable fact. */
        r.format_version = (uint16_t)(BCN_RECORD_FORMAT_VERSION + 1u);
        why[0] = '\0';
        CHECK(bcn_record_check(&r, why, sizeof why) != 0, "newer format_version must fail, not degrade");
        CHECK_STR_HAS(why, "2");   /* the artifact's version */
        CHECK_STR_HAS(why, "1");   /* this reader's version  */

        /* Older version: equally an error. There is no back-compat shim (Constitution III). */
        r.format_version = 0u;
        why[0] = '\0';
        CHECK(bcn_record_check(&r, why, sizeof why) != 0, "older format_version must fail");
        CHECK_STR_HAS(why, "0");
    }

    /* ---- container ---- */
    {
        BcnContainerHeader h;
        uint8_t buf[BCN_CONTAINER_HEADER_BYTES];
        BcnContainerHeader back;

        memset(&h, 0, sizeof h);
        h.magic = BCN_CONTAINER_MAGIC;
        h.format_version = BCN_CONTAINER_FORMAT_VERSION;
        h.header_bytes = (uint16_t)BCN_CONTAINER_HEADER_BYTES;
        h.width = 640; h.height = 400; h.bits_per_pixel = 8; h.sensor_mode = 0;
        h.nominal_fps = 276; h.start_t_us = 0x0102030405060708ull;
        h.build_id = 0xAAAABBBBCCCCDDDDull; h.config_hash = 0x0011223344556677ull;
        h.mode = BCN_MODE_BURST;

        why[0] = '\0';
        CHECK((bcn_container_check(&h, why, sizeof why)) == 0, "good header must verify: %s", why);

        /* The codec is hand-written per plan.md §Contracts at arm's length — so round-tripping it is a
         * real test, not a tautology over a memcpy. */
        bcn_container_header_encode(&h, buf);
        memset(&back, 0, sizeof back);
        CHECK((bcn_container_header_decode(buf, &back)) == 0, "decode must succeed");
        CHECK_EQ_U(back.width, 640u);
        CHECK_EQ_U(back.height, 400u);
        CHECK_EQ_U(back.nominal_fps, 276u);
        CHECK_EQ_U(back.start_t_us, 0x0102030405060708ull);
        CHECK_EQ_U(back.build_id, 0xAAAABBBBCCCCDDDDull);
        CHECK_EQ_U(back.config_hash, 0x0011223344556677ull);
        CHECK_EQ_U(back.mode, (uint32_t)BCN_MODE_BURST);
        /* Explicit little-endian, checked at the byte level: a big-endian writer must not pass. */
        CHECK_EQ_U(buf[0], 0x52u);  /* 'R' — low byte of 0x42434E52 */
        CHECK_EQ_U(buf[3], 0x42u);  /* 'B' — high byte             */
        CHECK_EQ_U(buf[4], BCN_CONTAINER_FORMAT_VERSION);

        h.format_version = (uint16_t)(BCN_CONTAINER_FORMAT_VERSION + 1u);
        why[0] = '\0';
        CHECK(bcn_container_check(&h, why, sizeof why) != 0, "newer container version must fail");
        CHECK_STR_HAS(why, "2");
        CHECK_STR_HAS(why, "1");
        h.format_version = BCN_CONTAINER_FORMAT_VERSION;

        h.magic = 0u;
        why[0] = '\0';
        CHECK(bcn_container_check(&h, why, sizeof why) != 0, "bad container magic must fail");
        CHECK_STR_HAS(why, "magic");
    }

    /* ---- frame header codec + the gap rule ---- */
    {
        BcnFrameHeader f, back;
        uint8_t buf[BCN_FRAME_HEADER_BYTES];
        memset(&f, 0, sizeof f);
        f.record_bytes = BCN_FRAME_HEADER_BYTES + 640u * 400u;
        f.seq = 12345u; f.t_us = 0x00000000DEADBEEFull;
        f.exposure_us = 200u; f.gain_q8 = 1024u; f.flags = BCN_FR_BURST_START;
        f.inav_t_us = 0u;             /* R10: 0 means ABSENT, and a reader must never read it as a time */
        f.inav_read_age_us = 0u; f.gps_time_ms = 0u;

        bcn_frame_header_encode(&f, buf);
        memset(&back, 0, sizeof back);
        CHECK((bcn_frame_header_decode(buf, &back)) == 0, "frame header decode");
        CHECK_EQ_U(back.seq, 12345u);
        CHECK_EQ_U(back.t_us, 0xDEADBEEFull);
        CHECK_EQ_U(back.exposure_us, 200u);
        CHECK_EQ_U(back.gain_q8, 1024u);
        CHECK_EQ_U(back.flags, (uint16_t)BCN_FR_BURST_START);
        CHECK_EQ_U(back.inav_t_us, 0u);

        /* Gaps are explicit and legal in ring/burst mode; contiguity is the thing that must be checkable. */
        CHECK_EQ_I(bcn_seq_is_gap(10u, 11u), 0);
        CHECK(bcn_seq_is_gap(10u, 12u) != 0, "a skipped seq is a discontinuity");
        CHECK(bcn_seq_is_gap(10u, 10u) != 0, "a repeated seq is a discontinuity too");
    }

    BCN_TEST_MAIN_END();
}
