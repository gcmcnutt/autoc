# Contract: INAV → xiao state message, v2 (`MSP2_INAV_LOCAL_STATE`, 0x210E)

Producer: INAV fork `autoc` branch, `src/main/fc/fc_msp.c:672-725`. Consumer: xiao `msp_autoc_state_t`
(`xiao/include/MSP.h:261-283`). Little-endian, packed. **Clean cut** (Constitution III): firmware pairs must match; the xiao
rejects any other length.

| offset | field | type | unit | v1 |
|---|---|---|---|---|
| 0 | timestamp | u32 | µs | ✓ |
| 4 | pos x, y, z | 3 × i32 | cm, NEU | ✓ |
| 16 | vel x, y, z | 3 × i32 | cm/s | ✓ |
| 28 | quaternion w, x, y, z | 4 × f32 | — | ✓ |
| 44 | box-mode bitmask (first 32) | u32 | — | ✓ |
| 48 | rc ch8, ch9 | 2 × u16 | µs | ✓ |
| 52 | gyroADCf | 3 × i16 | °/s × 10 | ✓ |
| 58 | accADCf | 3 × i16 | mg | ✓ |
| **64** | **est_eph** | u16 | cm (saturating) | new |
| **66** | **est_epv** | u16 | cm (saturating) | new |
| **68** | **est_flags** | u8 | bit0 XY_VALID, bit1 Z_VALID, bit2 GPS_XY_VALID, bit3 GPS_Z_VALID, bit4 BARO_VALID | new |
| **69** | **gps_numsat** | u8 | — | new |
| **70** | **gps_fix** | u8 | 0 none / 2 2D / 3 3D | new |
| **71** | **acc_weight** | u8 | × 100 | new |
| **72** | **gps_age_ms** | u16 | ms since last GPS update (saturating) | new |
| **74** | **layout_version** | u8 | = 2 | new |
| **75** | reserved | u8 | 0 | new |

Total **76 bytes** (v1: 64).

Rules:
- The xiao requires `length == 76 && layout_version == 2`. Anything else ⇒ `state_valid = false` (existing behaviour: stop autoc,
  logged). There is no v1 fallback.
- The new fields are **advisory**: they are logged and drive `fault_flags` (data-model.md), never engagement or NN inputs.
- Box bits: the xiao must not assume fixed indices. At boot it requests `MSP_BOXIDS` and maps ARM / FAILSAFE / MSP RC OVERRIDE by
  permanent id, or fails loud (research R8).

Test: an INAV-side unit test, or a captured-frame fixture decoded by a desktop xiao test, checks the offsets, the saturation and
the version-byte rejection.
