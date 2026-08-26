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
 * (~256 kpx) per tick, MEASURED live on the Pi 5 at ~3.2 ms of deadline margin (median margin 38.9 ms
 * without it, 35.7 ms with, 0/500 misses either way). Affordable for aiming and capture; do not leave it
 * on for an envelope run where the margin itself is the measurement.
 *
 * engine_field_enable() turns it on. engine_field_map() returns the cell buffer (BCN_FIELD_W*BCN_FIELD_H
 * bytes, row-major, 0..255) or NULL if disabled/not yet filled. */
#define BCN_FIELD_W 64
#define BCN_FIELD_H 20
void engine_field_enable(Engine *e, int on);
const uint8_t *engine_field_map(const Engine *e);

/* ---- preview plane: the same idea at a resolution you can actually LOOK at (operator ask 2026-08-26)
 * The field map above is 64x20 because it targets a terminal; that is enough to answer "is the beacon on
 * the sensor" and, as pi/preview.py puts it, useless for anything else. This is the graphical counterpart:
 * a 2x2 MAX-pooled copy of the frame at M2 resolution.
 *
 * Two deliberate choices:
 *   - MAX pool, not mean. The beacon is a handful of saturated pixels; averaging a 2x2 with three dark
 *     neighbours drops it to a quarter and it disappears at exactly the moment it matters. Max keeps a
 *     point source at full amplitude, which is the whole point of the display.
 *   - M2 resolution, because that IS the tracker's coordinate system (spec: M2 = native/2, centre
 *     origin). A viewer can therefore draw track x/y straight onto the plane with no rescale, so an
 *     overlay cannot silently disagree with the estimate it is drawing. track_overlay.py's marker bug
 *     (a stray /2 that put every marker in the wrong quadrant, silently) is the reason that matters.
 *
 * Same contract as the field map otherwise: DEBUG PATH, off unless asked, rides the JSON sink as its own
 * line rather than growing the versioned BcnRecord, and costs one pass over the frame per tick. MEASURED
 * live on the Pi 5 (2026-08-26, 640x400): median margin 38.6 ms without, 35.1 ms with, worst-case 36.1 ->
 * 30.3 ms, 0/277 misses either way -- so ~3.5 ms per tick, in line with the field map's 3.2 ms. Do not
 * leave it on for an envelope run where deadline margin is itself the measurement.
 *
 * The buffer is inline and fixed-size, because the engine allocates exactly once at open and never
 * again. That bounds the supported frame to BCN_PREVIEW_MAX_*, so engine_preview_enable() RETURNS -1
 * rather than quietly falling back to a coarser plane or a cropped one -- a viewfinder that silently
 * changes its scale is worse than no viewfinder (Constitution V: fail loud). */
#define BCN_PREVIEW_DIV   2
#define BCN_PREVIEW_MAX_W 320
#define BCN_PREVIEW_MAX_H 200
/* Returns 0, or -1 if `on` and the frame exceeds BCN_PREVIEW_MAX_* after division. */
int  engine_preview_enable(Engine *e, int on);
/* Returns the plane (w*h bytes, row-major, 0..255) or NULL if disabled/not yet filled; *w and *h are
 * set to frame_w/BCN_PREVIEW_DIV and frame_h/BCN_PREVIEW_DIV. */
const uint8_t *engine_preview_map(const Engine *e, unsigned *w, unsigned *h);

void engine_close(Engine *e);

#ifdef __cplusplus
}
#endif
#endif
