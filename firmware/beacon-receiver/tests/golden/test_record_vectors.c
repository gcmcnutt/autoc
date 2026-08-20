/* T006b (verification half) — the committed golden byte vectors ARE the contract.
 *
 * FIRST ATTEMPT AT THIS FILE WAS WRONG AND IS WORTH RECORDING. It decoded each vector, re-encoded it, and
 * memcmp'd the two. That is a tautology: decode->encode reproduces whatever bytes it read, so a corrupted
 * vector — or a field the assertions did not name — sails through. Verified by corrupting one byte and
 * watching it pass.
 *
 * What has teeth: state the expected field values HERE, independently of tools/gen_record_vectors.c,
 * encode them, and byte-compare against the committed file. Any drift in the layout, the codec, or the
 * file fails. The duplication between this file and the generator is deliberate — two independent
 * statements of the same known values are the whole mechanism (plan.md §Contracts at arm's length).
 *
 * If this fails: the layout changed without a format_version bump, or the codec drifted. Both are the
 * mechanism working. DO NOT regenerate the vectors to make it pass.
 */
#include "bcn_test.h"
#include "record.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void load(const char *dir, const char *name, uint8_t *buf)
{
    char path[512];
    FILE *f;
    size_t n;
    snprintf(path, sizeof path, "%s/%s", dir, name);
    f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "cannot open golden vector %s\n", path); exit(2); }
    n = fread(buf, 1, BCN_RECORD_WIRE_BYTES, f);
    if (n != BCN_RECORD_WIRE_BYTES) {
        fprintf(stderr, "%s: expected %u bytes, got %u\n", path,
                (unsigned)BCN_RECORD_WIRE_BYTES, (unsigned)n);
        exit(2);
    }
    /* A vector must be EXACTLY the wire size — a longer file means the layout grew. */
    if (fread(buf, 1, 1, f) != 0) { fprintf(stderr, "%s: file is longer than the wire size\n", path); exit(2); }
    fclose(f);
}

/* Byte-compare and, on mismatch, say WHERE — "vectors differ" sends the reader to a hex editor. */
static void expect_bytes(const char *name, const uint8_t *want, const uint8_t *got)
{
    size_t i;
    for (i = 0; i < BCN_RECORD_WIRE_BYTES; i++) {
        if (want[i] != got[i]) {
            CHECK(0, "%s: first difference at byte %u (expected 0x%02X, file has 0x%02X)%s",
                  name, (unsigned)i, want[i], got[i],
                  i >= BCN_RECORD_HEADER_BYTES ? " — inside tracks[]" : " — inside the header");
            return;
        }
    }
    CHECK(1, "%s matches", name);
}

int main(int argc, char **argv)
{
    const char *dir = (argc >= 2) ? argv[1] : "record_vectors";
    uint8_t file[BCN_RECORD_WIRE_BYTES], mine[BCN_RECORD_WIRE_BYTES];
    BcnRecord r;
    char why[256];

    /* ---- v1_empty: a tick with nothing tracked, emitted as a record and never as silence. */
    bcn_record_init(&r, 0x0011223344556677ull, 0x8899AABBCCDDEEFFull);
    r.t_us = 0x0000000102030405ull;
    r.seq = 1u; r.tick_index = 1u;
    r.n_tracks = 0u; r.n_slots_used = 0u;
    r.deadline_margin_us = 5000;
    bcn_record_encode(&r, mine);
    load(dir, "v1_empty.bin", file);
    expect_bytes("v1_empty", mine, file);

    /* ---- v1_two_tracks: one per code, asymmetric values so a transposed field cannot pass by luck. */
    bcn_record_init(&r, 0x0011223344556677ull, 0x8899AABBCCDDEEFFull);
    r.t_us = 0x00000000AABBCCDDull;
    r.seq = 4242u; r.tick_index = 4242u;
    r.n_tracks = 2u; r.n_slots_used = 4u;
    r.deadline_margin_us = -1234;
    r.inav_t_us = 0x000000005566778Aull;
    r.inav_read_age_us = 3100u;
    r.gps_time_ms = 86399000u;
    r.tracks[0].code_id = BCN_CODE_A;
    r.tracks[0].x = -12345; r.tracks[0].y = 6789;
    r.tracks[0].vx = 250; r.tracks[0].vy = -9000;
    r.tracks[0].x_pred = -12000; r.tracks[0].y_pred = 6500;
    r.tracks[0].chip_hz = 115u * 256u;
    r.tracks[0].cep = 2u * 256u; r.tracks[0].q = 200u;
    r.tracks[0].lock_health = 180u; r.tracks[0].extent = 300u; r.tracks[0].scintillation = 12u;
    r.tracks[0].flags = BCN_F_VALID | BCN_F_LOCK | BCN_F_MEASURED_FIX;
    r.tracks[0].age_ms = 5u;
    r.tracks[0].chip_phase = 17u; r.tracks[0].t_int_chips = 31u; r.tracks[0].scale = 2u;
    r.tracks[1].code_id = BCN_CODE_B;
    r.tracks[1].x = 100; r.tracks[1].y = -200;
    r.tracks[1].vx = -30; r.tracks[1].vy = 40;
    r.tracks[1].x_pred = 95; r.tracks[1].y_pred = -195;
    r.tracks[1].chip_hz = 200u * 256u;
    r.tracks[1].cep = 3u * 256u; r.tracks[1].q = 150u;
    r.tracks[1].lock_health = 90u; r.tracks[1].extent = 260u; r.tracks[1].scintillation = 40u;
    r.tracks[1].flags = BCN_F_VALID | BCN_F_HOLD | BCN_F_EXTRAPOLATED | BCN_F_MULTIPATH_SUSPECT;
    r.tracks[1].age_ms = 140u;
    r.tracks[1].chip_phase = 0u; r.tracks[1].t_int_chips = 124u; r.tracks[1].scale = 0u;
    bcn_record_encode(&r, mine);
    load(dir, "v1_two_tracks.bin", file);
    expect_bytes("v1_two_tracks", mine, file);

    /* ---- v1_saturated: the flags a consumer must not silently ignore. */
    bcn_record_init(&r, 0x0011223344556677ull, 0x8899AABBCCDDEEFFull);
    r.t_us = 0x0000000000010000ull;
    r.seq = 7u; r.tick_index = 7u; r.n_tracks = 1u; r.n_slots_used = 1u;
    r.deadline_margin_us = 0;
    r.tracks[0].code_id = BCN_CODE_B;
    r.tracks[0].chip_hz = 115u * 256u;
    r.tracks[0].flags = BCN_F_VALID | BCN_F_LOCK | BCN_F_SATURATED | BCN_F_AGC_SETTLING;
    r.tracks[0].scale = 1u;
    bcn_record_encode(&r, mine);
    load(dir, "v1_saturated.bin", file);
    expect_bytes("v1_saturated", mine, file);

    /* ---- the decoder must agree with the encoder, or a reader and a writer drift apart inside one
     * implementation while both pass the byte comparison above. */
    load(dir, "v1_two_tracks.bin", file);
    CHECK((bcn_record_decode(file, &r)) == 0, "decode v1_two_tracks");
    why[0] = '\0';
    CHECK((bcn_record_check(&r, why, sizeof why)) == 0, "decoded vector must verify: %s", why);
    CHECK_EQ_U(r.seq, 4242u);
    /* Signedness is the classic silent wire bug: an unsigned reader turns -1234 us of MISSED deadline
     * into +4294966062, i.e. a comfortable margin, and the deadline metric quietly reads clean. */
    CHECK_EQ_I(r.deadline_margin_us, -1234);
    CHECK_EQ_I(r.tracks[0].x, -12345);
    CHECK_EQ_I(r.tracks[0].vy, -9000);
    CHECK_EQ_U(r.tracks[0].scintillation, 12u);
    CHECK_EQ_U(r.tracks[1].code_id, (uint64_t)BCN_CODE_B);
    CHECK_EQ_I(r.tracks[1].y, -200);
    CHECK_EQ_U(r.tracks[2].flags, 0u);                       /* unused slots: VALID clear */
    CHECK_EQ_U(r.tracks[BCN_MAX_TRACKS - 1].flags, 0u);

    /* Byte-level anchors a foreign implementation gets wrong first: magic endianness, and the version at
     * a stable offset so it is findable WITHOUT parsing the rest (Constitution V). */
    CHECK_EQ_U(file[0], 0x31u);  /* '1' — low byte of 0x42434E31 "BCN1" */
    CHECK_EQ_U(file[3], 0x42u);  /* 'B' — high byte                     */
    CHECK_EQ_U(file[4], BCN_RECORD_FORMAT_VERSION);
    CHECK_EQ_U(file[6], BCN_RECORD_HEADER_BYTES);

    BCN_TEST_MAIN_END();
}
