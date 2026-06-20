"""T023 — Loader resilience tests (FR-2.5 reset-and-continue contract).

The recorder's three fault classes (Class 1 recoverable, Class 2 unrecoverable,
Class 3 brown-out) must all be handled by the loader without exceptions, with all
surviving frames returned, and the fault chain fully reported in metadata.
"""

from __future__ import annotations

import numpy as np

from beacon_loader import load_clip
from beacon_loader.schema import FAULT_CODES

from tests._fixtures import write_clip_and_sidecar


def test_fault_sentinel_between_data_chunks(tmp_path):
    """Class 1 SDIO_WRITE_ERROR_TRANSIENT — loader skips sentinel + continues."""
    rng = np.random.default_rng(11)
    h, w = 240, 320
    frames_a = rng.integers(0, 256, size=(5, h, w), dtype=np.uint8)
    frames_b = rng.integers(0, 256, size=(5, h, w), dtype=np.uint8)

    clip_path = tmp_path / "with_sentinel.clip"
    write_clip_and_sidecar(
        clip_path,
        chunks=[
            {"type": "data", "frames": frames_a, "ts_us": 1_000_000},
            {"type": "fault", "code": 0x00000001, "ts_us": 1_021_000},
            {"type": "data", "frames": frames_b, "ts_us": 1_022_000},
        ],
        bit_depth=8,
    )

    frames_out, metadata = load_clip(clip_path)

    # Both data chunks recovered; sentinel skipped
    assert frames_out.shape[0] == 10
    np.testing.assert_array_equal(np.concatenate([frames_a, frames_b], axis=0), frames_out)

    # Metadata records the fault
    assert len(metadata["fault_events"]) == 1
    event = metadata["fault_events"][0]
    assert event["fault_code"] == 0x00000001
    assert event["fault_name"] == "SDIO_WRITE_ERROR_TRANSIENT"
    assert event["timestamp_us"] == 1_021_000

    # Fully_intact = False because faults occurred (even though all frames recovered)
    assert metadata["fully_intact"] is False
    assert "partial_recovery_warning" in metadata
    assert metadata["truncated_at_offset"] is None  # clean EOF


def test_final_unrecoverable_sentinel_then_eof(tmp_path):
    """Class 2 unrecoverable — recorder seals file with FINAL_UNRECOVERABLE."""
    rng = np.random.default_rng(17)
    h, w = 240, 320
    frames_a = rng.integers(0, 256, size=(5, h, w), dtype=np.uint8)
    frames_b = rng.integers(0, 256, size=(5, h, w), dtype=np.uint8)

    clip_path = tmp_path / "unrecoverable.clip"
    write_clip_and_sidecar(
        clip_path,
        chunks=[
            {"type": "data", "frames": frames_a, "ts_us": 1_000_000},
            {"type": "data", "frames": frames_b, "ts_us": 1_021_000},
            {"type": "fault", "code": 0x000000FF, "ts_us": 1_042_000},
            # No chunks after → file sealed
        ],
        bit_depth=8,
    )

    frames_out, metadata = load_clip(clip_path)

    assert frames_out.shape[0] == 10
    assert len(metadata["fault_events"]) == 1
    assert metadata["fault_events"][0]["fault_code"] == 0x000000FF
    assert metadata["fault_events"][0]["fault_name"] == "FINAL_UNRECOVERABLE"
    assert metadata["fully_intact"] is False
    assert metadata["truncated_at_offset"] is None  # clean EOF (sealed properly)


def test_brownout_truncated_mid_chunk(tmp_path):
    """Class 3 brown-out — file ends with non-magic garbage; loader stops cleanly."""
    rng = np.random.default_rng(23)
    h, w = 240, 320
    frames_a = rng.integers(0, 256, size=(5, h, w), dtype=np.uint8)

    clip_path = tmp_path / "brownout.clip"
    write_clip_and_sidecar(
        clip_path,
        chunks=[
            {"type": "data", "frames": frames_a, "ts_us": 1_000_000},
            {"type": "garbage", "n": 37},   # simulates a truncated partial chunk header
        ],
        bit_depth=8,
    )

    frames_out, metadata = load_clip(clip_path)

    assert frames_out.shape[0] == 5
    np.testing.assert_array_equal(frames_a, frames_out)

    # Brown-out leaves no fault sentinel — discovered via truncation
    assert metadata["fault_events"] == []
    assert metadata["fully_intact"] is False
    assert metadata["truncated_at_offset"] is not None
    assert "partial_recovery_warning" in metadata


def test_multiple_fault_classes_in_one_file(tmp_path):
    """A realistic in-flight scenario: transient SDIO + ring overrun + final unrecoverable."""
    rng = np.random.default_rng(31)
    h, w = 240, 320
    chunks_data = [rng.integers(0, 256, size=(4, h, w), dtype=np.uint8) for _ in range(3)]

    clip_path = tmp_path / "messy.clip"
    write_clip_and_sidecar(
        clip_path,
        chunks=[
            {"type": "data", "frames": chunks_data[0], "ts_us": 1_000_000},
            {"type": "fault", "code": 0x00000001, "ts_us": 1_017_000},
            {"type": "data", "frames": chunks_data[1], "ts_us": 1_018_000},
            {"type": "fault", "code": 0x00000003, "ts_us": 1_035_000},
            {"type": "data", "frames": chunks_data[2], "ts_us": 1_036_000},
            {"type": "fault", "code": 0x000000FF, "ts_us": 1_053_000},
        ],
        bit_depth=8,
    )

    frames_out, metadata = load_clip(clip_path)

    assert frames_out.shape[0] == 12  # all three 4-frame chunks recovered
    assert len(metadata["fault_events"]) == 3
    fault_names = [e["fault_name"] for e in metadata["fault_events"]]
    assert fault_names == [
        "SDIO_WRITE_ERROR_TRANSIENT",
        "RING_BUFFER_OVERRUN",
        "FINAL_UNRECOVERABLE",
    ]
    assert metadata["fully_intact"] is False


def test_unknown_fault_code_named_gracefully(tmp_path):
    """Unknown fault codes (e.g. reserved 031-fpga range) MUST not crash the loader."""
    frames = np.zeros((1, 240, 320), dtype=np.uint8)
    clip_path = tmp_path / "future.clip"
    write_clip_and_sidecar(
        clip_path,
        chunks=[
            {"type": "data", "frames": frames, "ts_us": 0},
            {"type": "fault", "code": 0x80001234, "ts_us": 100},  # reserved range
        ],
        bit_depth=8,
    )

    frames_out, metadata = load_clip(clip_path)

    assert frames_out.shape[0] == 1
    assert len(metadata["fault_events"]) == 1
    assert metadata["fault_events"][0]["fault_code"] == 0x80001234
    # Unknown codes get a descriptive UNKNOWN_<hex> name
    assert metadata["fault_events"][0]["fault_name"] == "UNKNOWN_0x80001234"
    assert metadata["fully_intact"] is False


def test_known_fault_codes_all_named(tmp_path):
    """Every fault code in FAULT_CODES must produce its expected name when surfaced."""
    # Quick smoke check: all defined fault codes have unique names
    names = list(FAULT_CODES.values())
    assert len(names) == len(set(names)), "fault code names must be unique"
