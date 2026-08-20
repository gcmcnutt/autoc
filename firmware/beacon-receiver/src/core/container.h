/* container.h — the versioned raw recording container (contracts/recording-container.md, R9).
 *
 * Constitution V binds. Consumers: src_replay.c, tools/oracle, tools/inject, tools/score, analysis scripts.
 *
 * WHY AN EXPLICIT CODEC RATHER THAN A CAST. The contract fixes byte offsets that are not naturally
 * alignable in C (start_t_us at 20, build_id at 28), so there is no struct whose in-memory layout equals
 * the on-disk layout without packing. Rather than reach for __attribute__((packed)) — which buys unaligned
 * loads and a compiler-specific guarantee — this header keeps a natural in-memory struct and hand-writes
 * the little-endian codec. That is also exactly what plan.md §Contracts at arm's length asks of every
 * implementation: its OWN codec, verified against shared golden byte vectors.
 */
#ifndef BEACON_CONTAINER_H
#define BEACON_CONTAINER_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BCN_CONTAINER_MAGIC          0x42434E52u  /* "BCNR" */
#define BCN_CONTAINER_FORMAT_VERSION 1u
#define BCN_CONTAINER_HEADER_BYTES   52u          /* on-disk, per the contract table */
#define BCN_FRAME_HEADER_BYTES       40u          /* on-disk, before payload         */

/* Recording mode. Written to the header so a tool can tell at a glance whether seq gaps are expected. */
#define BCN_MODE_CONTINUOUS 0u  /* flight host: Pi 5 + NVMe                                            */
#define BCN_MODE_RING       1u  /* bench host: bounded RAM, triggered dump                             */
#define BCN_MODE_BURST      2u  /* N contiguous frames every M; each burst >= 1 full word              */

/* Per-frame flags. burst_start marks a CORRELATABLE BOUNDARY: tools may correlate within a burst and
 * never across one (contracts/recording-container.md §Rules). */
#define BCN_FR_BURST_START  (1u << 0)
#define BCN_FR_TRIGGER_DUMP (1u << 1)
#define BCN_FR_FIDUCIAL_SEEN (1u << 2)

typedef struct {
    uint32_t magic;
    uint16_t format_version;
    uint16_t header_bytes;
    uint16_t width, height;
    uint16_t bits_per_pixel;   /* 8; 10 reserved                                                       */
    uint16_t sensor_mode;      /* enum: 0 = 640x400 stock, 1 = 640x200 patched, ...                    */
    uint32_t nominal_fps;
    uint64_t start_t_us;
    uint64_t build_id;
    uint64_t config_hash;
    uint32_t mode;             /* BCN_MODE_*                                                           */
    uint32_t _reserved;
} BcnContainerHeader;

typedef struct {
    uint32_t record_bytes;     /* header + payload                                                     */
    uint32_t seq;              /* monotonic; GAPS ARE EXPLICIT AND LEGAL in ring/burst mode            */
    uint64_t t_us;
    uint32_t exposure_us;
    uint16_t gain_q8;
    uint16_t flags;            /* BCN_FR_*                                                             */
    uint64_t inav_t_us;        /* 0 = none (R10). A reader must never mistake 0 for a valid INAV time. */
    uint32_t inav_read_age_us;
    uint32_t gps_time_ms;      /* 0 = none                                                             */
} BcnFrameHeader;

/* ---- codec. encode writes exactly *_HEADER_BYTES; decode reads the same and returns 0 on success. */
void bcn_container_header_encode(const BcnContainerHeader *h, uint8_t out[BCN_CONTAINER_HEADER_BYTES]);
int  bcn_container_header_decode(const uint8_t in[BCN_CONTAINER_HEADER_BYTES], BcnContainerHeader *out);
void bcn_frame_header_encode(const BcnFrameHeader *h, uint8_t out[BCN_FRAME_HEADER_BYTES]);
int  bcn_frame_header_decode(const uint8_t in[BCN_FRAME_HEADER_BYTES], BcnFrameHeader *out);

/* ---- T011: reader-side version check. 0 = OK; non-zero fills `why` naming BOTH versions. Callers exit 2.
 * An unknown-but-NEWER version is an error, not a best-effort read (Constitution V). */
int bcn_container_check(const BcnContainerHeader *h, char *why, size_t why_len);

/* Is a seq jump a discontinuity the caller must not correlate across? Always true for a gap — this exists
 * so the rule is written once instead of re-derived at each call site. */
static inline int bcn_seq_is_gap(uint32_t prev_seq, uint32_t this_seq) {
    return this_seq != prev_seq + 1u;
}

#ifdef __cplusplus
}
#endif
#endif /* BEACON_CONTAINER_H */
