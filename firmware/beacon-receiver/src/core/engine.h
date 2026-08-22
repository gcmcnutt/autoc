/* engine.h — the whole receiver as a library: frames in, 20 Hz records out.
 *
 * This exists so that (a) beacon_trackd is argument parsing around it (plan.md: app/ is thin BY
 * ARCHITECTURE), and (b) T043's parity test runs the identical pipeline on the dev box with no camera.
 * Determinism contract: identical FrameView sequences -> byte-identical record sequences. No clock, no
 * thread, no allocation after engine_open (R3, R6).
 */
#ifndef BEACON_ENGINE_H
#define BEACON_ENGINE_H

#include "bank.h"
#include "agc.h"
#include "sched_virt.h"
#include "frame.h"
#include "record.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*EngineEmit)(const BcnRecord *rec, void *user);

typedef struct Engine Engine;

int  engine_open(Engine **out, const BcnConfig *cfg, EngineEmit emit, void *user,
                 char *err, size_t err_len);
/* Feed one frame. Emits 0..n records via the callback as tick boundaries pass. */
void engine_frame(Engine *e, const FrameView *fv);
/* Manual candidate seed (the US2 demo path; US3's acquisition replaces the need for it — tasks.md:
 * "you have to tell it where to look" is precisely what US3 removes). x/y in M2 q8. */
int  engine_seed(Engine *e, uint8_t code_id, int32_t x_q8, int32_t y_q8, uint32_t chip_hz_q8);
/* Current desired camera controls (the app actuates them per-request). */
void engine_controls(const Engine *e, uint32_t *exposure_us, uint16_t *gain_q8);
const Bank *engine_bank(const Engine *e);

/* ---- field map: the "am I even pointed at it?" display (operator ask 2026-08-21) ----------------
 * A coarse point-source CONTRAST map of the last frame at a tick, for the ASCII scope. Per cell it
 * reports max(px) - mean(px): a beacon (a point source in a dark cell) scores high, a uniformly bright
 * wall or an open window scores ~0. That is deliberately NOT raw intensity -- raw intensity shows room
 * lights and tells the operator nothing about where the beacon is.
 *
 * DEBUG PATH, not the record. It is deliberately kept out of BcnRecord: the record is a versioned wire
 * contract (Constitution V) and the scope is a convenience, so this rides the JSON sink as a separate
 * line instead of growing the format. It is also OFF unless asked for -- it costs a full-frame pass
 * (~256 kpx) per tick.
 *
 * engine_field_enable() turns it on. engine_field_map() returns the cell buffer (BCN_FIELD_W*BCN_FIELD_H
 * bytes, row-major, 0..255) or NULL if disabled/not yet filled. */
#define BCN_FIELD_W 64
#define BCN_FIELD_H 20
void engine_field_enable(Engine *e, int on);
const uint8_t *engine_field_map(const Engine *e);

void engine_close(Engine *e);

#ifdef __cplusplus
}
#endif
#endif
