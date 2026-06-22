# Contract — `data-format.md` Canonical Chunked-Binary Clip Format (v1)

**Spec ref**: FR-4.2, FR-4.3, Principle V (Versioned Persistence Artifacts)
**Reference impl**: `specs/040-camera-redo/beacon-loader/beacon_loader/chunk.py` (Phase 1) — the loader IS the executable spec

This document is the **single source of truth for the byte layout** of recorded clips. Any change here MUST update both:
1. The FR-4.3 Python loader implementation, AND
2. The FPGA recorder write path (`firmware/flight-recorder/rtl/sd_writer.v` or successor).

The `format_version` field is bumped on any schema change; readers fail loud on mismatch.

## File layout

```
┌────────────────────────────────────────────────────────────────────┐
│ FILE HEADER (16 bytes, written once at session start)              │
│ ┌──────────────────────────────────────────────────────────────┐   │
│ │ magic            uint32 = 0x0BEAC031   (4 B, LE)             │   │
│ │ format_version   uint16 = 1            (2 B, LE)             │   │
│ │ header_size      uint16 = 16           (2 B, LE)             │   │
│ │ session_start_us uint64                (8 B, LE)             │   │
│ └──────────────────────────────────────────────────────────────┘   │
├────────────────────────────────────────────────────────────────────┤
│ CHUNK #0 (~1 second of frames, atomic-write boundary)              │
│ ┌── Chunk Header (32 bytes) ───────────────────────────────────┐   │
│ │ chunk_magic           uint32 = 0xCAFEC0DE                    │   │
│ │ chunk_format_version  uint16 = 1                             │   │
│ │ _pad0                 uint16 = 0                             │   │
│ │ chunk_size_bytes      uint32  (incl. header, excl. CRC tail) │   │
│ │ chunk_timestamp_us    uint64                                 │   │
│ │ bit_depth             uint8  (8 or 10)                       │   │
│ │ _pad1                 uint8 = 0                              │   │
│ │ frame_count           uint16                                 │   │
│ │ frame_width           uint16 = 320                           │   │
│ │ frame_height          uint16 = 240                           │   │
│ │ _reserved             uint8[4] = 0                           │   │
│ └──────────────────────────────────────────────────────────────┘   │
│ ┌── Frames (frame_count × frame_payload_size) ────────────────┐    │
│ │ Frame 0: frame_ts_delta_us uint16  | pixel_data byte[N]     │    │
│ │ Frame 1: ...                                                │    │
│ │ ...                                                         │    │
│ │ Frame (frame_count-1): ...                                  │    │
│ └─────────────────────────────────────────────────────────────┘    │
│ ┌── Trailer (4 bytes) ────────────────────────────────────────┐    │
│ │ crc32   uint32  CRC-32 of (header + frames), Castagnoli poly │   │
│ └─────────────────────────────────────────────────────────────┘    │
├────────────────────────────────────────────────────────────────────┤
│ CHUNK #1 ... (same structure)                                      │
│ ...                                                                │
├────────────────────────────────────────────────────────────────────┤
│ EOF (sealed by recorder, OR truncated mid-chunk by fault per FR-2.5)│
└────────────────────────────────────────────────────────────────────┘
```

Frame payload size N:
- 8-bit: `N = 320 × 240 = 76 800` → `frame_payload_size = 76 802` (incl. 2-byte ts delta)
- 10-bit packed: `N = 320 × 240 × 10 / 8 = 96 000` → `frame_payload_size = 96 002`

10-bit packing layout: 4 consecutive 10-bit samples packed into 5 bytes, MSB-first per the OG0VA/OV9281 native MIPI output format (preserves zero-copy through the pipeline).

## Endianness

**Little-endian throughout** (matches MIPI-D-PHY native + Linux/x86 host).

## Chunk types (by `frame_count` value)

| `frame_count` | Chunk type | Trailer | Use |
|---|---|---|---|
| 1..65 534 | **Data chunk** | CRC-32 (4 bytes) | Normal recording — frames per the Frame layout above |
| 0 | **Fault-sentinel chunk** | CRC-32 (4 bytes, covers header only) | Marks a discontinuity in the recording; carries `fault_code` in the `_reserved` field |
| 65 535 (0xFFFF) | Reserved (future use) | — | — |

### Fault-sentinel chunk layout

A fault-sentinel chunk has `frame_count = 0` and uses the first 4 bytes of `_reserved` as the **fault code**:

```
Chunk Header (32 bytes — same layout as a data chunk):
  chunk_magic           uint32 = 0xCAFEC0DE
  chunk_format_version  uint16 = 1
  _pad0                 uint16 = 0
  chunk_size_bytes      uint32 = 36   (header + CRC trailer, no frames)
  chunk_timestamp_us    uint64        FPGA µs at the time the fault was detected
  bit_depth             uint8         (matches the current capture-mode bit_depth)
  _pad1                 uint8  = 0
  frame_count           uint16 = 0    <-- THIS makes it a fault-sentinel
  frame_width           uint16        (matches current capture-mode width)
  frame_height          uint16        (matches current capture-mode height)
  _reserved             uint8[4]      <-- byte[0..3] is fault_code (uint32 LE)
Trailer (4 bytes):
  crc32                 uint32        CRC-32 of the 32-byte header
```

### Fault codes

| Code | Name | Meaning | Recorder follow-up |
|---|---|---|---|
| `0x00000001` | `SDIO_WRITE_ERROR_TRANSIENT` | Single SDIO `CMD25` failed; retry succeeded | Continue (no recorder reset needed; sentinel records the brief discontinuity) |
| `0x00000002` | `SDIO_WRITE_ERROR_RECOVERED` | SDIO `CMD25` failed; SD re-init succeeded; recording resumes | Continue |
| `0x00000003` | `RING_BUFFER_OVERRUN` | HyperRAM ring filled faster than SDIO drained; N frames dropped | The `_reserved[4..7]` (extending past the fault_code 4 bytes) encodes the dropped-frame count if `_reserved` is widened later — for v1 the sentinel just records the event; drop-count is in metadata via the loader |
| `0x00000004` | `SENSOR_I2C_RECOVERED` | Camera sensor I²C re-init was needed (transient disconnect / config-register glitch) | Continue |
| `0x00000005` | `MIPI_DESYNC_RECOVERED` | MIPI D-PHY lost lock; re-locked + resumed | Continue |
| `0x000000FF` | `FINAL_UNRECOVERABLE` | Recorder is sealing the file due to unrecoverable fault (SD-full, SD-removed, repeated SDIO failure) | Following this chunk: file is sealed; status LED RED-solid |
| `0x80000000..` | (reserved for 031-fpga and beyond) | — | — |

The loader treats any chunk with `frame_count == 0` as a fault sentinel: log the fault code into `metadata['fault_events']`, advance by `chunk_size_bytes`, continue iteration.

## Atomicity + recovery

- Recorder pre-allocates the file at session start; writes chunks via direct-sector SDIO `CMD25` (multi-block write).
- **Recoverable faults** (transient SDIO error, ring overrun, sensor desync) → recorder writes a fault-sentinel chunk + resets the affected subsystem + continues recording into the **same file**. The loader handles the gap by skipping the sentinel and continuing.
- **Unrecoverable faults** (SD-full, SD-removed, repeated SDIO re-init failure) → recorder writes a `FINAL_UNRECOVERABLE` sentinel + finalizes the file. Surviving chunks loadable.
- **Brown-out** (power lost mid-write) → no sentinel possible; the file is truncated at whatever sector the SD card last acknowledged. Loader's chunk-magic-mismatch path handles this.

A loader iterating chunks must:
1. Read 4 bytes at the current offset.
2. If they match `chunk_magic`, parse the chunk header.
3. If `frame_count == 0`, this is a fault-sentinel: log the `fault_code` from `_reserved[0..3]`, advance by `chunk_size_bytes`, continue iteration.
4. If `frame_count > 0`, parse the data chunk normally, advance by `chunk_size_bytes`, continue.
5. If the magic does NOT match (truncated or corrupted offset), stop iteration cleanly — return all chunks read so far + the partial-recovery warning.

The recorder does NOT update a "frame count" field in the file header mid-write — that would require an extra fsync per chunk and re-introduce per-frame-fsync overhead. The loader counts chunks + frames by iterating.

## CRC-32

The trailing CRC-32 (Castagnoli polynomial, `crc32c`) covers the chunk header + frame data. Loader computes + compares; mismatch → **warn**, not fail (per FR-2.5 partial-recovery contract).

## Reference test vectors

Phase 1 contract: the loader's `tests/test_loader_contract.py` includes a hand-constructed v1 clip file (2 chunks × 5 frames × 320×240 × 8-bit) and round-trips it through the loader. Diff-detecting any change to the byte layout. Any future schema change MUST update the test vector + bump `format_version` together.
