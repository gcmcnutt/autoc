# Contract: Versioned Binary Flight Log (v1, int16-quantized packed)

**Scope**: FR-007/008/009/014 + Constitution V. This document IS the single source of the format —
the xiao writer and the desktop decoder are both implemented against it (the xiao build cannot link
desktop code, so the shared artifact is this contract + the round-trip test).

## Requirements (MUST)

1. **Versioned**: FileHeader carries `format_version`; the decoder loud-fails on unknown version or
   `scale_table_crc` mismatch — never best-effort parses.
2. **Complete**: every TickRecord carries ALL 37 NN inputs (as fed to the NN — post-gather values,
   not re-derivable proxies), all 3 outputs, and the aux set (data-model.md §2.3). The honest-dmp
   policy applied to flight logs: if the NN saw it, the log has it.
3. **Engage provenance**: every span starts with an EngageHeader carrying the RESOLVED re-centered
   arena (origin + floor_Z/ceiling_Z after the FR-001 rule), span_id, engage timestamp.
4. **Quantization**: `[-1,1]`-bounded fields → i16/32767 (resolution ~3×10⁻⁵ ≫ needed); unbounded
   fields (closing_rate, gyro, airspeed) → i16 with per-field scales defined in a versioned scale
   table in this contract; encoder saturates (never wraps); decoder restores float with the same
   table. Changing ANY scale ⇒ format_version bump.
5. **Stateless records**: each TickRecord decodes independently (no inter-record deltas in v1) —
   a corrupted page costs those ticks only. `tick_counter` gaps are REPORTED by the decoder.
6. **Engagement-scoped**: TickRecords are written only while a span is engaged. Outside spans, only
   EventRecords (sparse) are written. Budget: ≈95 B/tick × 20 Hz ⇒ 2×(3–4 min) flights ≈ 0.9 MB in
   the 2.04 MB region (~55% headroom).
7. **Write path**: existing 3×4 KB async buffer ring + 256 B page writes; no in-loop data erase
   (unchanged); the metadata flush stays off the control-tick critical path. Under buffer pressure:
   drop/coalesce log writes and count them (EventRecord), never stall the tick (FR-008).
8. **Console split (FR-014)**: no per-tick console output. Console = EventRecords worth showing a
   human (armed/disarmed/engage/disengage/download/errors) + a ~2 Hz heartbeat line.
9. **BLE download unchanged** (DL/LIST/ERASE surface); the payload is just smaller (~7 s for a full
   2-flight log at ~125 KB/s).

## Decoder (ground) MUST

- Reconstruct float fields via the header-CRC-verified scale table; emit CSV/dataframe consumable by
  the sim-vs-real per-axis tooling (dCtrl/amplitude per axis).
- Fail loud on version/CRC mismatch, truncated header, or impossible field counts; report (not
  hide) tick_counter gaps and drop counts.
- **Ground tooling surfaces** (operator 2026-07-10): `xiao/web/flight_logger.html` must handle the
  downloaded binary — at minimum download the `.bin` intact; decode-in-browser and/or a
  dump-to-CSV export are the desired end state — AND the desktop decoder (`flightlog_decode.py`
  class tool) provides the authoritative CSV path. The html and desktop decoders implement THIS
  contract; no third format definition.
- **Loop-health stats**: the existing per-span counters (ticks / overruns / resyncs / maxLate /
  avgLate — `msplink.cpp:163-168`) MUST be carried in the log (span-summary EventRecord), since
  the console line they print to is demoted by FR-014. Missed/overrun ticks stay observable.

## v2 wire notes (as implemented — `xiao/include/flight_log_format.h` is the byte-exact source)

- **v2 (2026-07-11, pre-first-flight)**: TickRecord gains craft telemetry — `pos[3]`
  (virtual/engage-relative NED m), `vel[3]` (NED m/s), `rabbit[3]` (ground-truth target,
  virtual NED m) — making the log self-contained for the renderer (`-x`) and trajectory
  analysis (no INAV-blackbox join). Raw NED = virtual + EngageHeader origin. v1 existed
  only for the 039 bench validation (T010 artifact).
- **Framing**: every record starts with a nonzero type byte (0x01 FileHeader 224 B, 0x02
  EngageHeader 29 B, 0x03 TickRecord 114 B, 0x04 EventRecord 10 B, 0x05 SpanSummary 103 B);
  0x00 bytes BETWEEN records are flash-buffer word-alignment padding — decoders skip them.
  Unknown type or truncated record ⇒ loud parse failure.
- **v2 scale table** (49 entries carried IN the FileHeader, CRC-32-guarded; slot order =
  PathgenInput enum + 3 outputs + pos[3] + vel[3] + rabbit[3]): unit-bounded fields
  (target vecs, quat, dist_to_boundary, inward_body, outputs) → 32767; dist[6] → 32 (raw
  metres, ±1023.97 m); closing_rate + airspeed + vel → 256 (±128 m/s); gyro → 900
  (±36.4 rad/s); pos + rabbit → 16 (±2047.9 m, 6.25 cm). Encoder saturates symmetrically
  to ±32767 (never INT16_MIN, never wraps).
- **Readers**: `src/analytics/flightlog_decode.py` (authoritative CSV), in-browser decode +
  CSV export in `xiao/web/flight_logger.html`, the renderer `-x` mode (`tools/renderer.cc`
  parseXiaoDataBinary — legacy text logs still route to the old parser by sniffing), and
  the desktop round-trip test `tests/flightlog_roundtrip_tests.cc` — all against this contract.

## Tests (write first)

- Encode→decode round trip: every field |decoded − original| ≤ quantization step (desktop test
  compiling the encoder source or a bit-identical reference implementation of this contract).
- Version loud-fail: decoder rejects a header with version+1 and with a corrupted scale CRC.
- Saturation: out-of-range input saturates (never wraps) and decodes to the rail value.
- Budget arithmetic: static assert record size ≤ 120 B (v2: 114 B actual).
