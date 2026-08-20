/* gen_record_vectors.c — T006b: emit the golden byte vectors for the 20 Hz record contract.
 *
 * WHY THIS EXISTS (plan.md §Contracts at arm's length). A shared header only delivers its static_assert
 * guarantee if both sides compile the SAME header — which is exactly what breeds the #ifdef thicket, and
 * the ATtiny412 (8-bit int, different alignment) settles it alone. So the contract is a data description,
 * and it emits canonical encoded records with known field values. Every implementation — Pi, xiao,
 * autoc-side analysis — runs these same bytes through its OWN hand-written codec in its OWN test suite.
 * Drift fails a test on whichever side is stale, with zero shared source and zero build coupling.
 *
 * This is MSP's arm's-length property with MSP's weakness removed: MSP drift is caught at runtime, golden
 * vectors catch it at test time.
 *
 * Regenerate ONLY with a format_version bump. Editing a vector to make a failing test pass defeats the
 * entire mechanism — the failing test is the mechanism working.
 */
#include "record.h"
#include <stdio.h>
#include <string.h>

static int emit(const char *dir, const char *name, const BcnRecord *r)
{
    char path[512];
    uint8_t buf[BCN_RECORD_WIRE_BYTES];
    FILE *f;
    snprintf(path, sizeof path, "%s/%s", dir, name);
    bcn_record_encode(r, buf);
    f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "gen: cannot write %s\n", path); return 1; }
    if (fwrite(buf, 1, sizeof buf, f) != sizeof buf) { fprintf(stderr, "gen: short write %s\n", path); fclose(f); return 1; }
    fclose(f);
    printf("  wrote %s (%u bytes)\n", path, (unsigned)sizeof buf);
    return 0;
}

int main(int argc, char **argv)
{
    const char *dir = (argc >= 2) ? argv[1] : ".";
    BcnRecord r;
    int rc = 0;

    /* v1_empty — the shape every consumer sees most often: a tick with nothing tracked. It is emitted as a
     * record with n_tracks = 0, NEVER as silence, so consumers can detect loss by seq gap. */
    bcn_record_init(&r, 0x0011223344556677ull, 0x8899AABBCCDDEEFFull);
    r.t_us = 0x0000000102030405ull;
    r.seq = 1u; r.tick_index = 1u;
    r.n_tracks = 0u; r.n_slots_used = 0u;
    r.deadline_margin_us = 5000;
    rc |= emit(dir, "v1_empty.bin", &r);

    /* v1_two_tracks — one of each code, exercising sign (negative x and vy), the q8 domains, and the two
     * SCORED flags. Deliberately asymmetric values so a transposed field cannot pass by luck. */
    bcn_record_init(&r, 0x0011223344556677ull, 0x8899AABBCCDDEEFFull);
    r.t_us = 0x00000000AABBCCDDull;
    r.seq = 4242u; r.tick_index = 4242u;
    r.n_tracks = 2u; r.n_slots_used = 4u;
    r.deadline_margin_us = -1234;               /* a MISSED deadline; signedness is the point */
    r.inav_t_us = 0x000000005566778Aull;
    r.inav_read_age_us = 3100u;
    r.gps_time_ms = 86399000u;

    r.tracks[0].code_id = BCN_CODE_A;           /* PORT / red */
    r.tracks[0].x = -12345; r.tracks[0].y = 6789;
    r.tracks[0].vx = 250; r.tracks[0].vy = -9000;
    r.tracks[0].x_pred = -12000; r.tracks[0].y_pred = 6500;
    r.tracks[0].chip_hz = 115u * 256u;          /* the bench emitter's 'H' mode, in q8 */
    r.tracks[0].cep = 2u * 256u; r.tracks[0].q = 200u;
    r.tracks[0].lock_health = 180u; r.tracks[0].extent = 300u; r.tracks[0].scintillation = 12u;
    r.tracks[0].flags = BCN_F_VALID | BCN_F_LOCK | BCN_F_MEASURED_FIX;
    r.tracks[0].age_ms = 5u;
    r.tracks[0].chip_phase = 17u; r.tracks[0].t_int_chips = 31u; r.tracks[0].scale = 2u;

    r.tracks[1].code_id = BCN_CODE_B;           /* STARBOARD / green — the bench emitter */
    r.tracks[1].x = 100; r.tracks[1].y = -200;
    r.tracks[1].vx = -30; r.tracks[1].vy = 40;
    r.tracks[1].x_pred = 95; r.tracks[1].y_pred = -195;
    r.tracks[1].chip_hz = 200u * 256u;          /* flight nominal, in q8 */
    r.tracks[1].cep = 3u * 256u; r.tracks[1].q = 150u;
    r.tracks[1].lock_health = 90u; r.tracks[1].extent = 260u; r.tracks[1].scintillation = 40u;
    r.tracks[1].flags = BCN_F_VALID | BCN_F_HOLD | BCN_F_EXTRAPOLATED | BCN_F_MULTIPATH_SUSPECT;
    r.tracks[1].age_ms = 140u;                  /* inside the 150 ms HOLD bound, only just */
    r.tracks[1].chip_phase = 0u; r.tracks[1].t_int_chips = 124u; r.tracks[1].scale = 0u;
    rc |= emit(dir, "v1_two_tracks.bin", &r);

    /* v1_saturated — the flags a consumer must not silently ignore: railed peak plus AGC settling. */
    bcn_record_init(&r, 0x0011223344556677ull, 0x8899AABBCCDDEEFFull);
    r.t_us = 0x0000000000010000ull;
    r.seq = 7u; r.tick_index = 7u; r.n_tracks = 1u; r.n_slots_used = 1u;
    r.deadline_margin_us = 0;                   /* exactly on the deadline: not a miss */
    r.tracks[0].code_id = BCN_CODE_B;
    r.tracks[0].chip_hz = 115u * 256u;
    r.tracks[0].flags = BCN_F_VALID | BCN_F_LOCK | BCN_F_SATURATED | BCN_F_AGC_SETTLING;
    r.tracks[0].scale = 1u;
    rc |= emit(dir, "v1_saturated.bin", &r);

    return rc;
}
