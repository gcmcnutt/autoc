/* bank.h — the candidate pool: 16 movable correlators (T039–T041; data-model §3, spec §2.4).
 *
 * Fixed capacity, eviction by priority, never realloc. Roles: every CONFIRMED track owns a
 * PRECISION+GUARD pair — one extra slot buys "guard re-centers in 2 ticks" instead of "full-field
 * reacquire, 1+ s outage", and 500°/s roll transients are a normal flight event.
 */
#ifndef BEACON_BANK_H
#define BEACON_BANK_H

#include "track.h"
#include "record.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BANK_ROLE_CANDIDATE 0u
#define BANK_ROLE_PRECISION 1u
#define BANK_ROLE_GUARD     2u

typedef struct {
    Track   trk;
    uint8_t used;
    uint8_t role;                      /* BANK_ROLE_*                                                  */
    int8_t  partner;                   /* paired slot index, -1 = none                                 */
    uint8_t promote_streak;            /* consecutive good ticks toward CANDIDATE -> CONFIRMED         */
    uint8_t flags_extra;               /* MULTIPATH_SUSPECT etc, OR'd into the record                  */
} BankSlot;

typedef struct {
    BankSlot slots[BCN_MAX_TRACKS];
    uint8_t  max_slots;                /* from config                                                  */
} Bank;

void bank_init(Bank *b, const BcnConfig *cfg);

/* Seed a new CANDIDATE (from acquisition, or manually for the US2 demo). Returns the slot index, or -1
 * if the bank is full of higher-priority work (eviction refused to make room). */
int bank_seed(Bank *b, const BcnConfig *cfg, uint8_t code_id, uint8_t scale,
              int32_t x_q8, int32_t y_q8, uint32_t chip_hz_q8, uint64_t epoch_us, uint64_t now_us);

/* Per-tick lifecycle: promotion (sustained lock_health), guard allocation for CONFIRMED tracks, orphan
 * cleanup, the same-code mirror-pair rule (spec §9: flag the lower, KEEP it), and burial of the dead.
 * Call AFTER every slot's track_tick. */
void bank_tick(Bank *b, const BcnConfig *cfg);

/* Fill the record's tracks[] (VALID/LOCK/HOLD/... flags per data-model §5). Returns n_tracks. */
uint8_t bank_emit(const Bank *b, const BcnConfig *cfg, BcnRecord *rec);

uint8_t bank_slots_used(const Bank *b);

#ifdef __cplusplus
}
#endif
#endif
