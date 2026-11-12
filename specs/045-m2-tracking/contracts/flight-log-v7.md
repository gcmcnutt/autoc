# Contract: xiao flight log v7

> ⏸ **Deferred (spec § Clarifications 2026-11-12)** for the tick fields. A log change comes only with the ShadowStateRecord, if US6
> is adopted (then v7 = v6 + that record). Kept as the design of record for the tick fields.

Base: v6 (`xiao/include/flight_log_format.h`, `specs/039-xiao-20hz-flight/contracts/flight-log-format.md`). Changes only:

1. `kFormatVersion` = 7. v5 / v6 decoders **refuse** v7; v7 decoders refuse v5 / v6 (clean cut; the historical scripts keep reading
   the old logs with the old decoder revisions).
2. **TickRecord** appends, after the existing fields:

| field | type | note |
|---|---|---|
| est_eph | u16 cm | from state v2 |
| est_epv | u16 cm | |
| est_flags | u8 | as state v2 |
| gps_numsat | u8 | |
| acc_weight | u8 ×100 | |
| fault_flags | u8 | INVALID_XY, INVALID_Z, FROZEN, STEP_XY, STEP_Z, GPS_STALE, LOW_SATS (bit 0–6) |
| gps_age_ms | u16 | |

   +10 B ⇒ 158 B at N = 53.
3. **ShadowStateRecord** (new type id, next free after 0x06), 31 B, written after each TickRecord while engaged **only when the
   shadow frame runs**: type u8, tick_id u16, quat 4 × i16 (×32767), vel_ned 3 × i16 (cm/s), pos_ned 3 × i16 (dm, span-local),
   gyro_bias 3 × i16 (°/s × 1000), innovation u8 (GPS velocity innovation, m/s × 10, saturating), status u8 (HANDOVER /
   PROPAGATING / COASTING / FAULTED).
4. Budget assert: `(sizeof(TickRecord) + sizeof(ShadowStateRecord)) × 7200 ≤ 0.7 × log region` (data-model.md).

Consumers updated in the same change: `flight_log.cpp`, `flightlog_decode.py`, `flight_logger.html`, `tools/renderer.cc`,
`tests/flightlog_roundtrip_tests.cc` (+ v7 fixture, CMake ⇒ clean `rebuild-perf.sh`), and the decoder-importing 044 / 045 scripts.
CSV columns: the new fields are appended by name. `flight_gates.py` / `nav_span_scan.py` keep working; their column names are
unchanged.
