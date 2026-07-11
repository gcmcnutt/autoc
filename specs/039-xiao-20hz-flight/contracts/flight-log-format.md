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

## Tests (write first)

- Encode→decode round trip: every field |decoded − original| ≤ quantization step (desktop test
  compiling the encoder source or a bit-identical reference implementation of this contract).
- Version loud-fail: decoder rejects a header with version+1 and with a corrupted scale CRC.
- Saturation: out-of-range input saturates (never wraps) and decodes to the rail value.
- Budget arithmetic: static assert record size ≤ 100 B.
