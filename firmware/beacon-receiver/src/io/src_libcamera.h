/* src_libcamera.h — the live frame source (T021, R8). C interface over a C++ implementation, so app/
 * code stays C and the C++ surface area is exactly one file. Only builds where libcamera exists (the Pi,
 * or a cross build with a sysroot) — which is why it is a separate CMake target from beacon_io. */
#ifndef BEACON_SRC_LIBCAMERA_H
#define BEACON_SRC_LIBCAMERA_H

#include "config.h"
#include "frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Open the first camera, configured per cfg's [camera] section: RAW stream at mode/fps, manual exposure
 * and gain (auto-anything WILL sabotage code capture — README §Measured facts), per-frame metadata on.
 * Frame timestamps come from the SENSOR (SensorTimestamp), not the CPU clock — that is what makes the
 * recorded t_us usable for chip-rate work. */
int bcn_libcamera_open(FrameSource **out, const BcnConfig *cfg, char *err, size_t err_len);

#ifdef __cplusplus
}
#endif
#endif /* BEACON_SRC_LIBCAMERA_H */
