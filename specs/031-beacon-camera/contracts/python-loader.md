# Contract — Python Loader API (`beacon_loader`)

**Spec ref**: FR-4.3
**Reference impl**: `tools/beacon-loader/beacon_loader/loader.py`
**Test ref**: `tools/beacon-loader/tests/test_loader_contract.py`

## Public API

```python
from beacon_loader import load_clip

frames, metadata = load_clip("path/to/session-id.clip")
```

### Signature

```python
def load_clip(clip_path: str | os.PathLike) -> tuple[np.ndarray, dict]:
    """
    Load a Phase-1 beacon-camera clip (binary + JSON sidecar pair).

    Args:
        clip_path: path to the .clip binary file. The sidecar is expected at
                   `<clip_path stem>.json` next to it.

    Returns:
        frames:    np.ndarray of shape (n_frames, height, width).
                   dtype = uint8 for bit_depth=8 clips
                   dtype = uint16 for bit_depth=10 clips (right-shifted to LSB; 0..1023 range)
        metadata:  dict parsed from the JSON sidecar, validated against
                   json-sidecar-schema.json; loader-augmented with:
                   - 'recovered_frame_count': int       (== frames.shape[0])
                   - 'recovered_chunk_count': int       (data chunks only, excludes sentinels)
                   - 'fault_events': list[dict]         (one entry per fault-sentinel chunk encountered)
                       each entry: {'offset_bytes': int, 'timestamp_us': int, 'fault_code': int,
                                    'fault_name': str}
                   - 'fully_intact': bool               (True if no fault events AND clean EOF)
                   - 'partial_recovery_warning': str    (only present if fully_intact = False)
                   - 'truncated_at_offset': int|None    (set if iteration stopped on invalid magic
                                                         — indicates brown-out / mid-CMD25 cut)

    Raises:
        FormatVersionError: format_version != 1
                            **MUST fail loud per Principle V — no silent default-init.**
        MagicMismatchError: file does not start with the magic sentinel
        SidecarValidationError: JSON sidecar fails schema validation
        SidecarMissingError: sidecar file not found
    """
```

### Auxiliary functions

```python
def iter_chunks(clip_path) -> Iterator[ChunkRecord]:
    """Stream chunks one at a time — for clips too large to hold all frames
       in RAM. Each ChunkRecord exposes .frames (np.ndarray), .timestamp_us,
       .bit_depth, .crc_ok (bool)."""

def validate_sidecar(sidecar_dict: dict) -> None:
    """Validate a sidecar dict against json-sidecar-schema.json. Raises
       SidecarValidationError on any schema deviation."""
```

## Error contract (Principle V — fail loud)

| Error | When | Loader behavior |
|---|---|---|
| `FormatVersionError` | File header `format_version != 1` (Phase 1) | **Raise immediately** with a message naming both the file version and the loader version: "Clip file is format_version=X, loader supports format_version=1. Re-record with current FPGA firmware or update the loader." **No partial load, no silent default.** |
| `MagicMismatchError` | File header `magic != 0x0BEAC031` OR sidecar is for a different clip family | Raise immediately with the file path + observed magic bytes |
| `ChunkMagicMismatch` (within iteration) | Chunk magic invalid mid-file | **Stop iteration cleanly** + set `fully_intact = False` + `truncated_at_offset = <offset>` + add a `partial_recovery_warning` to metadata. Caller still receives all chunks parsed so far. This is the FR-2.5 brown-out recovery path. |
| `ChunkVersionMismatch` | A chunk's `chunk_format_version` ≠ file `format_version` | Same as `ChunkMagicMismatch` — stop iteration cleanly, partial recovery |
| Fault-sentinel chunk (frame_count == 0) | Recorder injected a fault-sentinel per FR-2.5 reset-and-continue path | **Skip the sentinel + append to `metadata['fault_events']` + continue iteration**. NOT an exception — this is the documented resilience path. The frames already parsed are kept; iteration continues into subsequent data chunks. `fully_intact` becomes False if any fault_events are present. |
| `SidecarValidationError` | JSON sidecar fails schema validation | Raise immediately; the binary is fine, but the metadata is untrustworthy |
| `SidecarMissingError` | No sidecar at expected path | Raise immediately |
| CRC mismatch | Per-chunk CRC-32 fails | **Warn only** — chunk frames returned with a `crc_ok = False` field on the ChunkRecord |

## Performance contract

- **Streaming via `iter_chunks`**: O(1) RAM per chunk (~24 MB for a 480 fps 10-bit chunk).
- **Eager `load_clip`**: O(N) RAM in total frames. A 10-min flight at 240 fps 8-bit = ~11 GB → use `iter_chunks` for clips that big.
- **Throughput**: target ≥100 MB/s clip-file read on a desktop NVMe (well above any clip size produced in Phase 1).

## Round-trip + resilience tests

```python
# tests/test_loader_contract.py
def test_roundtrip_two_chunks_8bit():
    # Hand-construct a v1 clip file with 2 chunks × 5 frames each
    frames_in, sidecar_in = _make_test_clip(...)
    clip_path = tmp_path / "test.clip"
    _write_clip(clip_path, frames_in, sidecar_in)

    frames_out, metadata_out = load_clip(clip_path)

    np.testing.assert_array_equal(frames_in, frames_out)
    assert metadata_out['clip_id'] == sidecar_in['clip_id']
    assert metadata_out['fully_intact'] is True
    assert metadata_out['recovered_frame_count'] == 10
    assert metadata_out['fault_events'] == []

# tests/test_loader_resilience.py
def test_fault_sentinel_between_data_chunks():
    """Recorder reset-and-continue path: data chunk → SDIO transient fault sentinel
       → data chunk. Loader must skip the sentinel + return all frames."""
    clip_path = tmp_path / "test_with_sentinel.clip"
    _write_clip_with_sentinel(
        clip_path,
        chunks_before=[frames_a],            # 5 frames
        sentinel_fault_code=0x00000001,      # SDIO_WRITE_ERROR_TRANSIENT
        chunks_after=[frames_b],             # 5 more frames
    )

    frames_out, metadata = load_clip(clip_path)

    assert frames_out.shape[0] == 10  # both chunks parsed, sentinel skipped
    assert len(metadata['fault_events']) == 1
    assert metadata['fault_events'][0]['fault_code'] == 0x00000001
    assert metadata['fault_events'][0]['fault_name'] == 'SDIO_WRITE_ERROR_TRANSIENT'
    assert metadata['fully_intact'] is False
    assert 'partial_recovery_warning' in metadata
    assert metadata['truncated_at_offset'] is None  # clean EOF

def test_final_unrecoverable_sentinel_then_eof():
    """Unrecoverable fault path: data chunks → FINAL_UNRECOVERABLE sentinel → EOF.
       Loader must return all chunks before sentinel + flag the unrecoverable."""
    clip_path = tmp_path / "test_unrecoverable.clip"
    _write_clip_with_sentinel(
        clip_path,
        chunks_before=[frames_a, frames_b],
        sentinel_fault_code=0x000000FF,      # FINAL_UNRECOVERABLE
        chunks_after=[],                     # nothing after
    )

    frames_out, metadata = load_clip(clip_path)

    assert frames_out.shape[0] == 10
    assert metadata['fault_events'][0]['fault_code'] == 0x000000FF
    assert metadata['fully_intact'] is False

def test_brownout_truncated_mid_chunk():
    """Power-loss path: data chunks then a partial chunk header (no valid magic)."""
    clip_path = tmp_path / "test_brownout.clip"
    _write_clip_with_brownout(
        clip_path,
        chunks_complete=[frames_a],
        garbage_bytes_at_end=37,
    )

    frames_out, metadata = load_clip(clip_path)

    assert frames_out.shape[0] == 5
    assert metadata['fully_intact'] is False
    assert metadata['truncated_at_offset'] is not None
    assert metadata['fault_events'] == []  # brown-out leaves no sentinel
```

These three resilience tests are the loader-side proof that the recorder's three fault classes (Class 1 recoverable, Class 2 unrecoverable, Class 3 brown-out) are all handled without exceptions, with all surviving frames returned, and with the fault chain fully reported in metadata.

## Phase 2 onward

When 031-fpga's `(x, y, CEP)` output joins the format, it goes in a NEW chunk type (different `chunk_magic`) so old clip files remain v1-loadable AND old loaders simply skip the new chunk type. That's a forward-compatible *extension*, not a `format_version` bump. **A `format_version` bump is reserved for changes that break old-file readability.**
