/* container.c — T011 checks + the hand-written little-endian codec for the raw recording container.
 *
 * The codec is explicit byte-by-byte rather than a struct cast. Two reasons, both in the contract: the
 * on-disk offsets are not naturally alignable, and plan.md §Contracts at arm's length wants every
 * implementation to own its codec so the golden vectors have something real to verify.
 */
#include "container.h"
#include <stdio.h>
#include <string.h>

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v) { int i; for (i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static void put_u64(uint8_t *p, uint64_t v) { int i; for (i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t get_u32(const uint8_t *p)
{
    uint32_t v = 0; int i;
    for (i = 0; i < 4; i++) v |= (uint32_t)p[i] << (8 * i);
    return v;
}
static uint64_t get_u64(const uint8_t *p)
{
    uint64_t v = 0; int i;
    for (i = 0; i < 8; i++) v |= (uint64_t)p[i] << (8 * i);
    return v;
}

void bcn_container_header_encode(const BcnContainerHeader *h, uint8_t out[BCN_CONTAINER_HEADER_BYTES])
{
    memset(out, 0, BCN_CONTAINER_HEADER_BYTES);
    put_u32(out +  0, h->magic);
    put_u16(out +  4, h->format_version);
    put_u16(out +  6, h->header_bytes);
    put_u16(out +  8, h->width);
    put_u16(out + 10, h->height);
    put_u16(out + 12, h->bits_per_pixel);
    put_u16(out + 14, h->sensor_mode);
    put_u32(out + 16, h->nominal_fps);
    put_u64(out + 20, h->start_t_us);
    put_u64(out + 28, h->build_id);
    put_u64(out + 36, h->config_hash);
    put_u32(out + 44, h->mode);
    put_u32(out + 48, h->_reserved);
}

int bcn_container_header_decode(const uint8_t in[BCN_CONTAINER_HEADER_BYTES], BcnContainerHeader *out)
{
    if (!in || !out) return -1;
    memset(out, 0, sizeof *out);
    out->magic          = get_u32(in +  0);
    out->format_version = get_u16(in +  4);
    out->header_bytes   = get_u16(in +  6);
    out->width          = get_u16(in +  8);
    out->height         = get_u16(in + 10);
    out->bits_per_pixel = get_u16(in + 12);
    out->sensor_mode    = get_u16(in + 14);
    out->nominal_fps    = get_u32(in + 16);
    out->start_t_us     = get_u64(in + 20);
    out->build_id       = get_u64(in + 28);
    out->config_hash    = get_u64(in + 36);
    out->mode           = get_u32(in + 44);
    out->_reserved      = get_u32(in + 48);
    return 0;
}

void bcn_frame_header_encode(const BcnFrameHeader *h, uint8_t out[BCN_FRAME_HEADER_BYTES])
{
    memset(out, 0, BCN_FRAME_HEADER_BYTES);
    put_u32(out +  0, h->record_bytes);
    put_u32(out +  4, h->seq);
    put_u64(out +  8, h->t_us);
    put_u32(out + 16, h->exposure_us);
    put_u16(out + 20, h->gain_q8);
    put_u16(out + 22, h->flags);
    put_u64(out + 24, h->inav_t_us);
    put_u32(out + 32, h->inav_read_age_us);
    put_u32(out + 36, h->gps_time_ms);
}

int bcn_frame_header_decode(const uint8_t in[BCN_FRAME_HEADER_BYTES], BcnFrameHeader *out)
{
    if (!in || !out) return -1;
    memset(out, 0, sizeof *out);
    out->record_bytes     = get_u32(in +  0);
    out->seq              = get_u32(in +  4);
    out->t_us             = get_u64(in +  8);
    out->exposure_us      = get_u32(in + 16);
    out->gain_q8          = get_u16(in + 20);
    out->flags            = get_u16(in + 22);
    out->inav_t_us        = get_u64(in + 24);
    out->inav_read_age_us = get_u32(in + 32);
    out->gps_time_ms      = get_u32(in + 36);
    return 0;
}

int bcn_container_check(const BcnContainerHeader *h, char *why, size_t why_len)
{
    if (!h || !why || why_len == 0) return -1;
    if (h->magic != BCN_CONTAINER_MAGIC) {
        snprintf(why, why_len, "recording: bad magic 0x%08lX (expected 0x%08lX \"BCNR\") — this is not a "
                               "beacon recording",
                 (unsigned long)h->magic, (unsigned long)BCN_CONTAINER_MAGIC);
        return -1;
    }
    if (h->format_version != BCN_CONTAINER_FORMAT_VERSION) {
        snprintf(why, why_len, "recording: format_version %u but this reader implements %u — refusing to "
                               "interpret (Constitution V: no partial reads)",
                 (unsigned)h->format_version, (unsigned)BCN_CONTAINER_FORMAT_VERSION);
        return -1;
    }
    if (h->header_bytes != BCN_CONTAINER_HEADER_BYTES) {
        snprintf(why, why_len, "recording: header_bytes %u but version %u defines %u",
                 (unsigned)h->header_bytes, (unsigned)BCN_CONTAINER_FORMAT_VERSION,
                 (unsigned)BCN_CONTAINER_HEADER_BYTES);
        return -1;
    }
    if (h->bits_per_pixel != 8u) {
        snprintf(why, why_len, "recording: bits_per_pixel %u unsupported (only 8 is implemented; 10 is "
                               "reserved in the contract)", (unsigned)h->bits_per_pixel);
        return -1;
    }
    if (h->width == 0u || h->height == 0u) {
        snprintf(why, why_len, "recording: degenerate geometry %ux%u", (unsigned)h->width, (unsigned)h->height);
        return -1;
    }
    return 0;
}
