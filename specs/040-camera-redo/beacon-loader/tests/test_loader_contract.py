"""T020 — FR-4.3 round-trip test.

The contract: write a known clip via the fixtures, ingest via load_clip,
get the same frames + metadata back.
"""

from __future__ import annotations

import numpy as np

from beacon_loader import load_clip

from tests._fixtures import write_clip_and_sidecar


def test_roundtrip_two_chunks_8bit(tmp_path):
    rng = np.random.default_rng(42)
    h, w = 240, 320
    frames_a = rng.integers(0, 256, size=(5, h, w), dtype=np.uint8)
    frames_b = rng.integers(0, 256, size=(5, h, w), dtype=np.uint8)

    clip_path = tmp_path / "test.clip"
    sidecar_in = write_clip_and_sidecar(
        clip_path,
        chunks=[
            {"type": "data", "frames": frames_a, "ts_us": 1_000_000},
            {"type": "data", "frames": frames_b, "ts_us": 1_021_000},
        ],
        bit_depth=8,
    )

    frames_out, metadata = load_clip(clip_path)

    # Frame data bit-identical
    np.testing.assert_array_equal(np.concatenate([frames_a, frames_b], axis=0), frames_out)

    # Metadata fields
    assert metadata["clip_id"] == sidecar_in["clip_id"]
    assert metadata["format_version"] == 1
    assert metadata["recovered_frame_count"] == 10
    assert metadata["recovered_chunk_count"] == 2
    assert metadata["fully_intact"] is True
    assert metadata["fault_events"] == []
    assert metadata["truncated_at_offset"] is None
    assert "partial_recovery_warning" not in metadata


def test_roundtrip_single_chunk_10bit(tmp_path):
    rng = np.random.default_rng(7)
    h, w = 240, 320
    # 10-bit values 0..1023
    frames = rng.integers(0, 1024, size=(3, h, w), dtype=np.uint16)

    clip_path = tmp_path / "test10.clip"
    write_clip_and_sidecar(
        clip_path,
        chunks=[{"type": "data", "frames": frames, "ts_us": 5_000_000}],
        bit_depth=10,
    )

    frames_out, metadata = load_clip(clip_path)

    np.testing.assert_array_equal(frames, frames_out)
    assert frames_out.dtype == np.uint16
    assert metadata["capture_mode"]["bit_depth"] == 10
    assert metadata["fully_intact"] is True


def test_per_frame_timestamps_monotonic(tmp_path):
    rng = np.random.default_rng(13)
    frames = rng.integers(0, 256, size=(10, 240, 320), dtype=np.uint8)
    clip_path = tmp_path / "ts.clip"
    write_clip_and_sidecar(
        clip_path,
        chunks=[{"type": "data", "frames": frames, "ts_us": 1_000_000}],
        bit_depth=8,
    )

    # Use iter_chunks to inspect timestamps directly
    from beacon_loader import iter_chunks
    records = list(iter_chunks(clip_path))
    assert len(records) == 1
    ts = records[0].frame_timestamps_us
    assert ts is not None
    # Monotonically strictly increasing
    assert np.all(np.diff(ts) > 0), f"timestamps not monotonic: {ts}"
    # First frame timestamp equals chunk timestamp
    assert ts[0] == 1_000_000
