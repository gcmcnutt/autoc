# Data Model — 031 Beacon-Camera Phase 1

> **MOVED 2026-06-22 from `specs/031-beacon-camera/`. Parked 040 camera-redo reference — predates the 031 1-bit phase and is likely stale; re-validate (20 Hz / 480 fps / 200 Hz / 75 ms baseline + 031 field findings) on 040 restart. See [`README.md`](README.md).**

The Phase-1 data model is dominated by the **recorded raw-clip artifact**. No database, no service entities — the model is a file format with a JSON sidecar.

## Entities

### ClipFile

The complete on-disk artifact produced by the FPGA recorder. One ClipFile per recording session (bench scenario or flight). Comprises:

- **File header** (16 bytes, written once at session start)
- **Chunk[]** (1 or more ~1-second blocks, appended as the session runs)

Stored as a single binary file at `<session-id>.clip`. The matching JSONSidecar lives at `<session-id>.json` next to it.

| Field | Type | Notes |
|---|---|---|
| `magic` | `uint32` | `0x0BEAC031` — sentinel; loader hard-fails if absent |
| `format_version` | `uint16` | Phase 1 → `1`. **Principle V: fail-loud on mismatch** |
| `header_size` | `uint16` | Byte size of the file header (= 16 for v1) — future versions may extend |
| `session_start_us` | `uint64` | Wall-clock µs since epoch at session start (informational; FPGA monotonic clock is authoritative for in-clip timing per FR-2.4) |

After the file header, chunks are appended in order. The file may be truncated mid-chunk on power-loss / SD-full per FR-2.5; the loader recovers up to the last complete chunk.

### Chunk

Independently parseable ~1-second block. Two variants distinguished by `frame_count`:

- **Data chunk** (`frame_count > 0`): carries N frames of raw pixel data.
- **Fault-sentinel chunk** (`frame_count == 0`): marks a discontinuity in the recording — recorder hit a fault and either recovered (Class 1) or sealed the file (Class 2 `FINAL_UNRECOVERABLE`). Carries a 32-bit fault code in the `_reserved[0..3]` field.

| Field | Type | Notes |
|---|---|---|
| `chunk_magic` | `uint32` | `0xCAFEC0DE` — sentinel; loader skips invalid chunks |
| `chunk_format_version` | `uint16` | Per Principle V; must equal file-header `format_version` |
| `_pad0` | `uint16` | = 0 |
| `chunk_size_bytes` | `uint32` | Total bytes in this chunk *including* this header — used to find the next chunk without parsing each frame |
| `chunk_timestamp_us` | `uint64` | FPGA-monotonic-counter µs of the **first frame** in this chunk (for fault-sentinel: the µs at fault detection) |
| `bit_depth` | `uint8` | `8` or `10` (per FR-2.3) |
| `_pad1` | `uint8` | = 0 |
| `frame_count` | `uint16` | Number of frames in this chunk. **0 ⇒ fault-sentinel chunk; ≥1 ⇒ data chunk**. Typical data values: 240 for 1 s @ 240 fps, 480 for 1 s @ 480 fps |
| `frame_width` | `uint16` | `320` for Phase 1 (per FR-2.3) |
| `frame_height` | `uint16` | `240` for Phase 1 |
| `_reserved` | `uint8[4]` | **For fault-sentinel chunks: bytes [0..3] = `fault_code` (uint32 LE)**; for data chunks: zero-padded |
| `frames[frame_count]` | `Frame[]` | Frame payload (data chunks only; absent in fault-sentinels) |
| `crc32` | `uint32` | CRC-32 of the chunk body. Loader warns on mismatch but does not fail |

The fault-sentinel chunk is a recoverable-by-design discontinuity marker — see [contracts/data-format.md](./contracts/data-format.md) for the full fault-code table.

### Frame

One frame's worth of raw pixel data.

| Field | Type | Notes |
|---|---|---|
| `frame_timestamp_delta_us` | `uint16` | µs delta from `chunk_timestamp_us` — covers ~65 ms (sufficient for one chunk at 240–480 fps) |
| `pixel_data` | `byte[N]` | Raw pixel bytes. For 8-bit: `N = 320 × 240 = 76 800`. For 10-bit packed: `N = 320 × 240 × 10 / 8 = 96 000`. No padding |

Total per-frame size: 76 802 bytes (8-bit) or 96 002 bytes (10-bit packed).

### JSONSidecar

Per-clip metadata, separate from the binary. Stored on disk; loader returns it as a `dict` augmented with runtime fields (`recovered_frame_count`, `recovered_chunk_count`, `fault_events`, `fully_intact`, `partial_recovery_warning`, `truncated_at_offset`) — these augmented fields are **not** part of the on-disk schema; they reflect what the loader observed during parse.

See `contracts/json-sidecar-schema.json` for the formal on-disk schema. Field summary:

| Field | Type | Required | Notes |
|---|---|---|---|
| `clip_id` | string | yes | Session ID, e.g. `S6-2026-05-22-001` |
| `clip_file` | string | yes | Relative path to the `.clip` file |
| `format_version` | int | yes | Must match the `format_version` in the file header — sidecar/binary integrity check |
| `iso_start_time` | string (ISO 8601) | yes | Wall-clock session start |
| `sensor_model` | string | yes | `OG0VA` or `OV9281` |
| `sensor_fw_rev` | string | no | If available from sensor I²C |
| `lens_spec` | object | yes | `{model, h_fov_deg, f_number, filter_cwl_nm, filter_fwhm_nm}` |
| `capture_mode` | object | yes | `{resolution: [320, 240], fps: 240 or 480, bit_depth: 8 or 10}` |
| `sensor_settings` | object | yes | `{exposure_us, analog_gain, digital_gain}` |
| `ambient` | enum | yes | `indoor / outdoor_cloudy / outdoor_sun / dusk / dim` |
| `range_qualifier` | string | no | Free-text or numeric meters |
| `pose_qualifier` | enum | yes | `stationary / hand_panned / on_aircraft / in_flight` |
| `recording_mode` | enum | yes | `bench_tethered / flight_sd` |
| `notes` | string | no | Free-text |
| `in_flight_extras` | object | conditional | Required if `recording_mode = flight_sd`: `{target_craft_id, beacon_code_ids: [A, B], airframe_config, intended_pattern, ground_notes}` |

## Relationships

```
ClipFile
├── FileHeader (1, fixed 16 bytes)
└── Chunk[] (1..N, appended)
    ├── DataChunk (frame_count > 0)
    │   ├── ChunkHeader (32 bytes; _reserved zeroed)
    │   ├── Frame[] (typically 240 or 480)
    │   └── CRC-32 (4 bytes)
    └── FaultSentinelChunk (frame_count == 0)
        ├── ChunkHeader (32 bytes; _reserved[0..3] = fault_code)
        └── CRC-32 (4 bytes)

JSONSidecar
└── references ClipFile by clip_id + clip_file path
```

A typical Phase-1 flight clip is **a sequence of DataChunks** with **zero or more FaultSentinelChunks interleaved** at the points where the recorder hit transient faults and reset. A flight that ended cleanly (operator power-down, no faults) has no sentinels. A flight that filled the SD card has a single `FINAL_UNRECOVERABLE` sentinel at the end.

## Validation rules

| Rule | Applied by |
|---|---|
| `magic == 0x0BEAC031` | Loader file-open |
| `format_version == 1` (Phase 1) | Loader file-header check — **fail loud on mismatch per Principle V** |
| Each chunk has `chunk_magic == 0xCAFEC0DE` | Loader iterator — invalid chunks skipped + logged |
| `chunk_format_version == file_header.format_version` | Loader chunk-iterator |
| `frame_timestamp_delta_us` is monotonically increasing within a chunk | Loader (warn-only — frame drops surface here) |
| `chunk_timestamp_us` monotonically increasing across chunks | Loader (warn-only) |
| Sidecar `format_version == file_header.format_version` | Loader after both are opened |
| Sidecar `recording_mode == flight_sd` ⇒ `in_flight_extras` present | Loader sidecar-validation |
| CRC-32 per chunk | Loader (warn-only sanity check) |

## State transitions

The ClipFile transitions through:

```
[file pre-allocated, header written]
        │ recorder steady-state: append DataChunks
        ▼
[recording — header + N data chunks on disk]
        │
        ├── Class 1 transient fault detected
        │       │ insert FaultSentinelChunk (recoverable code)
        │       │ reset affected subsystem (~200 ms)
        │       │ resume DataChunk appends
        │       ▼
        │   [recording — DataChunks + 1+ sentinels]
        │
        ├── Class 2 unrecoverable fault (SD-full / re-init failed)
        │       │ insert FaultSentinelChunk (0xFF FINAL_UNRECOVERABLE)
        │       │ finalize file
        │       ▼
        │   [sealed file with terminal sentinel]
        │
        ├── Class 3 brown-out (power lost)
        │       │ no sentinel possible
        │       ▼
        │   [truncated file — loader detects via chunk-magic mismatch]
        │
        └── Operator power-down (normal completion)
                │ no sentinel; clean EOF after last DataChunk
                ▼
            [sealed file, fully_intact = True]
```

Once sealed (any path), the file is read-only and immutable.

## Versioning

`format_version = 1` is the on-ramp. Per the constitution (Principle V) and the project's no-cereal-versioning practice, any future schema change bumps the version directly; readers fail loud on mismatch. There is no "migration" path planned for Phase 1 → Phase 2 (031-fpga's clip writer will be v1 too unless a feature explicitly motivates a bump).
