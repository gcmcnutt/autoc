/* record.h — the versioned 20 Hz record stream (contracts/record-wire-format.md, data-model.md §2/§4/§5).
 *
 * Constitution V binds: explicit version at a stable offset, readers fail loudly naming BOTH versions.
 * research.md R13 binds too — this is a versioned STRUCT, never a wire protocol. The transport is a plug,
 * so when the xiao and the receiver merge into one box the transport is DELETED rather than redesigned.
 *
 * Layout discipline (plan.md §Contracts at arm's length): fixed-width types only, explicit little-endian,
 * explicit padding, no bitfields, no enums as storage. The _Static_asserts below lock the layout WITHIN
 * this implementation; the cross-target mechanism is the golden byte vectors in tests/golden/, not this
 * header. No other implementation ever includes this file.
 */
#ifndef BEACON_RECORD_H
#define BEACON_RECORD_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BCN_RECORD_MAGIC          0x42434E31u  /* "BCN1" */
#define BCN_RECORD_FORMAT_VERSION 1u
#define BCN_RECORD_HEADER_BYTES   64u
/* Compile-time ceiling on the wire shape. config [bank] max_slots must be <= this and the loader checks
 * it; the struct is fixed-size because a consumer must be able to seek by record index. */
#define BCN_MAX_TRACKS            16

/* ---- flags (data-model.md §5). VALID and MEASURED_FIX are the two SCORED bits (spec §3.1); the rest are
 * diagnostic and may gain meanings within a format_version. NO BIT IS EVER REPURPOSED without a bump. */
#define BCN_F_VALID             (1u << 0)  /* this slot carries meaning this tick                       */
#define BCN_F_LOCK              (1u << 1)  /* CONFIRMED, measurement-backed this tick                   */
#define BCN_F_HOLD              (1u << 2)  /* extrapolating, still inside the §3.1 bound                */
#define BCN_F_EXTRAPOLATED      (1u << 3)  /* position from prediction, not measurement                 */
#define BCN_F_MULTIPATH_SUSPECT (1u << 4)  /* lower member of a same-code mirror pair (spec §9)         */
#define BCN_F_SATURATED         (1u << 5)  /* peak railed -> flat-top centroid estimator (spec §5)      */
#define BCN_F_MEASURED_FIX      (1u << 6)  /* chip re-affirmation covered this tick — §3.1 metric #2    */
#define BCN_F_AGC_SETTLING      (1u << 7)  /* exposure/gain changed inside the integration window       */

/* ---- code identity. The colour convention is old and load-bearing across beacon_display.py, the ASCII
 * scope and every plot: A = PORT = red, B = STARBOARD = green. Do not renumber. */
#define BCN_CODE_A 0u  /* PORT / red      */
#define BCN_CODE_B 1u  /* STARBOARD/green */

/* ---- Track: one confirmed beacon (data-model.md §2). 48 bytes, 4-byte aligned, padding explicit.
 * Fixed-point per R2: q8 = 8 fractional bits. Float is banned in the hot path because bit-exact replay
 * across A53, A76 and Grace is a hard requirement and FMA contraction differs between them. */
typedef struct {
    int32_t  x, y;             /* q8 px, M2 grid: 320x200 @ 0.304 deg/px, centre (0,0), +x right +y down */
    int32_t  vx, vy;           /* q8 px/s — alpha-beta velocity state, the "momentum" of spec §2.1       */
    int32_t  x_pred, y_pred;   /* q8 px at the NEXT control tick, not this one                           */
    uint32_t chip_hz;          /* q8 Hz — per-track DPLL state                                           */
    uint16_t cep;              /* q8 px                                                                  */
    uint16_t q;                /* q8 — slow, full-word correlation quality (scale-free)                  */
    uint16_t lock_health;      /* q8 — fast, chip-rate decision-directed statistic (spec §2.6)           */
    uint16_t extent;           /* q8 — q_fine/q_coarse: point source vs glitter path (spec §9)           */
    uint16_t scintillation;    /* q8 — q variance over a window (spec §9)                                */
    uint16_t flags;            /* BCN_F_*                                                                */
    uint16_t age_ms;           /* staleness of the measurement behind the prediction — REPORTED, not a
                                * constraint. The constraint is deadline_margin_us (spec §11.1).         */
    uint8_t  code_id;          /* BCN_CODE_A / BCN_CODE_B                                                */
    uint8_t  chip_phase;       /* 0..30                                                                  */
    uint8_t  t_int_chips;      /* current adaptive integration length (spec §4)                          */
    uint8_t  scale;            /* coarse / medium / fine (spec §2.2)                                     */
    uint16_t _pad;             /* explicit                                                               */
} BcnTrack;

/* ---- the record. Emitted EVERY tick without exception: a tick with nothing tracked emits n_tracks = 0,
 * never silence. Consumers detect loss by seq gap, which they cannot do against an absent record. */
typedef struct {
    uint32_t magic;              /* 0  BCN_RECORD_MAGIC                                                  */
    uint16_t format_version;     /* 4                                                                    */
    uint16_t header_bytes;       /* 6  for forward skip                                                   */
    uint64_t t_us;               /* 8  frame-clock time of the tick                                       */
    uint32_t seq;                /* 16 monotonic record counter                                           */
    uint32_t tick_index;         /* 20 control ticks since start                                          */
    uint8_t  n_tracks;           /* 24 populated entries in tracks[]                                      */
    uint8_t  n_slots_used;       /* 25 bank occupancy (diagnostic)                                        */
    uint16_t _reserved;          /* 26                                                                    */
    int32_t  deadline_margin_us; /* 28 SIGNED; negative = deadline MISSED (spec §11.1)                     */
    uint64_t build_id;           /* 32                                                                    */
    uint64_t config_hash;        /* 40                                                                    */
    uint64_t inav_t_us;          /* 48 INAV clock from the MOST RECENT MSP read; 0 = none yet (R10)        */
    uint32_t inav_read_age_us;   /* 56 age of that MSP read at this tick                                   */
    uint32_t gps_time_ms;        /* 60 absolute when available; 0 = none                                   */
    BcnTrack tracks[BCN_MAX_TRACKS]; /* 64 unused entries zeroed with VALID clear                          */
} BcnRecord;

#if defined(__cplusplus)
#define BCN_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#else
#define BCN_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#endif

/* ---- T006a: layout locks. These catch an accidental field edit HERE, in this implementation, at compile
 * time. They are deliberately NOT the cross-target mechanism — see tests/golden/record_vectors/. */
BCN_STATIC_ASSERT(sizeof(BcnTrack) == 48, "BcnTrack layout changed — bump format_version and regenerate golden vectors");
BCN_STATIC_ASSERT(offsetof(BcnTrack, x)             ==  0, "BcnTrack.x moved");
BCN_STATIC_ASSERT(offsetof(BcnTrack, chip_hz)       == 24, "BcnTrack.chip_hz moved");
BCN_STATIC_ASSERT(offsetof(BcnTrack, cep)           == 28, "BcnTrack.cep moved");
BCN_STATIC_ASSERT(offsetof(BcnTrack, flags)         == 38, "BcnTrack.flags moved");
BCN_STATIC_ASSERT(offsetof(BcnTrack, code_id)       == 42, "BcnTrack.code_id moved");

BCN_STATIC_ASSERT(offsetof(BcnRecord, magic)              ==  0, "record.magic moved");
BCN_STATIC_ASSERT(offsetof(BcnRecord, format_version)     ==  4, "record.format_version moved — it MUST stay at a stable offset (Constitution V)");
BCN_STATIC_ASSERT(offsetof(BcnRecord, header_bytes)       ==  6, "record.header_bytes moved");
BCN_STATIC_ASSERT(offsetof(BcnRecord, t_us)               ==  8, "record.t_us moved");
BCN_STATIC_ASSERT(offsetof(BcnRecord, seq)                == 16, "record.seq moved");
BCN_STATIC_ASSERT(offsetof(BcnRecord, tick_index)         == 20, "record.tick_index moved");
BCN_STATIC_ASSERT(offsetof(BcnRecord, n_tracks)           == 24, "record.n_tracks moved");
BCN_STATIC_ASSERT(offsetof(BcnRecord, n_slots_used)       == 25, "record.n_slots_used moved");
BCN_STATIC_ASSERT(offsetof(BcnRecord, deadline_margin_us) == 28, "record.deadline_margin_us moved");
BCN_STATIC_ASSERT(offsetof(BcnRecord, build_id)           == 32, "record.build_id moved");
BCN_STATIC_ASSERT(offsetof(BcnRecord, config_hash)        == 40, "record.config_hash moved");
BCN_STATIC_ASSERT(offsetof(BcnRecord, inav_t_us)          == 48, "record.inav_t_us moved");
BCN_STATIC_ASSERT(offsetof(BcnRecord, inav_read_age_us)   == 56, "record.inav_read_age_us moved");
BCN_STATIC_ASSERT(offsetof(BcnRecord, gps_time_ms)        == 60, "record.gps_time_ms moved");
BCN_STATIC_ASSERT(offsetof(BcnRecord, tracks)             == BCN_RECORD_HEADER_BYTES, "record.tracks must start at header_bytes");
BCN_STATIC_ASSERT(sizeof(BcnRecord) == BCN_RECORD_HEADER_BYTES + 48u * BCN_MAX_TRACKS, "BcnRecord has hidden padding");

/* ---- T011: reader-side version check (Constitution V). Returns 0 on OK, non-zero on mismatch, and fills
 * `why` with a message naming BOTH the artifact's version and this reader's. Callers exit 2
 * (contracts/cli.md). There is deliberately no "best effort" path: an unknown-but-newer version is an
 * error, not something to partially interpret. */
int bcn_record_check(const BcnRecord *r, char *why, size_t why_len);

/* ---- explicit little-endian codec. The in-memory struct above already matches the contract's offsets
 * (the _Static_asserts prove it), but the WIRE is defined as little-endian bytes, not as this struct — and
 * the golden vectors in tests/golden/record_vectors/ only have teeth if there is a real codec to test.
 * Every other implementation (xiao, analysis tooling) writes its own against the same vectors, with zero
 * shared source and zero #ifdef (plan.md §Contracts at arm's length). */
#define BCN_RECORD_WIRE_BYTES (BCN_RECORD_HEADER_BYTES + 48u * BCN_MAX_TRACKS)

void bcn_record_encode(const BcnRecord *r, uint8_t out[BCN_RECORD_WIRE_BYTES]);
int  bcn_record_decode(const uint8_t in[BCN_RECORD_WIRE_BYTES], BcnRecord *out);

/* Initialise a record's header fields. Everything the caller does not set stays zero, which for tracks[]
 * means VALID clear — the correct representation of "nothing here". */
void bcn_record_init(BcnRecord *r, uint64_t build_id, uint64_t config_hash);

#ifdef __cplusplus
}
#endif
#endif /* BEACON_RECORD_H */
