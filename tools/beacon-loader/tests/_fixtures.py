"""Synthetic-clip writers — the executable spec for contracts/data-format.md.

Used by all test_loader_* files; not part of the public API.
"""

from __future__ import annotations

import json
import struct
import zlib
from pathlib import Path

import numpy as np

from beacon_loader.chunk import pack_10bit
from beacon_loader.schema import (
    CHUNK_HEADER_SIZE,
    CHUNK_MAGIC,
    CHUNK_TRAILER_SIZE,
    FILE_HEADER_SIZE,
    FILE_MAGIC,
    FORMAT_VERSION,
    frame_payload_size,
)


def write_file_header(
    f, format_version: int = FORMAT_VERSION, session_start_us: int = 0
) -> None:
    f.write(
        struct.pack(
            "<IHHQ",
            FILE_MAGIC,
            format_version,
            FILE_HEADER_SIZE,
            session_start_us,
        )
    )


def write_data_chunk(
    f,
    *,
    frames: np.ndarray,          # shape (n, h, w), dtype uint8 or uint16
    chunk_timestamp_us: int,
    bit_depth: int,
    per_frame_delta_us: int = 4167,  # ~240 fps frame period
) -> int:
    """Append one data chunk; return bytes written."""
    n, h, w = frames.shape
    per_frame_size = frame_payload_size(w, h, bit_depth)
    payload_size = n * per_frame_size
    chunk_size = CHUNK_HEADER_SIZE + payload_size + CHUNK_TRAILER_SIZE

    header = struct.pack(
        "<IHHIQBBHHHBBBB",
        CHUNK_MAGIC, FORMAT_VERSION, 0,  # chunk_magic, version, _pad0
        chunk_size, chunk_timestamp_us,
        bit_depth, 0, n,                # bit_depth, _pad1, frame_count
        w, h,                            # frame_width, frame_height
        0, 0, 0, 0,                      # _reserved (zeroed for data chunks)
    )

    payload = bytearray()
    for i in range(n):
        delta = (i * per_frame_delta_us) & 0xFFFF
        payload += struct.pack("<H", delta)
        if bit_depth == 8:
            payload += frames[i].astype(np.uint8).tobytes()
        else:
            payload += pack_10bit(frames[i].flatten())

    crc = zlib.crc32(header + bytes(payload)) & 0xFFFFFFFF
    f.write(header + bytes(payload) + struct.pack("<I", crc))
    return chunk_size


def write_fault_sentinel(
    f,
    *,
    fault_code: int,
    chunk_timestamp_us: int,
    bit_depth: int,
    frame_width: int = 320,
    frame_height: int = 240,
) -> int:
    """Append one fault-sentinel chunk; return bytes written."""
    chunk_size = CHUNK_HEADER_SIZE + 0 + CHUNK_TRAILER_SIZE  # no frames

    reserved = struct.pack("<I", fault_code)
    header = struct.pack(
        "<IHHIQBBHHHBBBB",
        CHUNK_MAGIC, FORMAT_VERSION, 0,
        chunk_size, chunk_timestamp_us,
        bit_depth, 0, 0,                 # frame_count = 0 ⇒ fault sentinel
        frame_width, frame_height,
        reserved[0], reserved[1], reserved[2], reserved[3],
    )

    crc = zlib.crc32(header) & 0xFFFFFFFF
    f.write(header + struct.pack("<I", crc))
    return chunk_size


def make_test_sidecar(
    clip_path: Path,
    *,
    fps: int = 240,
    bit_depth: int = 8,
    width: int = 320,
    height: int = 240,
    recording_mode: str = "bench_tethered",
    extra: dict | None = None,
) -> dict:
    """Build a schema-valid sidecar dict for the given clip."""
    sidecar: dict = {
        "clip_id": clip_path.stem,
        "clip_file": clip_path.name,
        "format_version": FORMAT_VERSION,
        "iso_start_time": "2026-05-18T12:00:00Z",
        "sensor_model": "OV9281",
        "lens_spec": {
            "model": "m12lenses-PT-02120+Edmund-65-679",
            "h_fov_deg": 120.0,
            "f_number": 2.0,
            "filter_cwl_nm": 850.0,
            "filter_fwhm_nm": 10.0,
        },
        "capture_mode": {
            "resolution": [width, height],
            "fps": fps,
            "bit_depth": bit_depth,
        },
        "sensor_settings": {
            "exposure_us": 500.0,
            "analog_gain": 1.0,
            "digital_gain": 1.0,
        },
        "ambient": "indoor",
        "pose_qualifier": "stationary",
        "recording_mode": recording_mode,
    }
    if recording_mode == "flight_sd":
        sidecar["in_flight_extras"] = {
            "target_craft_id": "hb1",
            "beacon_code_ids": [0, 1],
            "airframe_config": "wingtip-tape-mounted",
            "intended_pattern": "tracker-chase varying ranges",
        }
    if extra is not None:
        sidecar.update(extra)
    return sidecar


def write_clip_and_sidecar(
    clip_path: Path,
    chunks: list[dict],
    *,
    fps: int = 240,
    bit_depth: int = 8,
    width: int = 320,
    height: int = 240,
    sidecar_overrides: dict | None = None,
    file_header_format_version: int = FORMAT_VERSION,
) -> dict:
    """Compose a clip file + matching sidecar. Returns the sidecar dict.

    chunks: list of {"type": "data", "frames": np.ndarray, "ts_us": int}
                OR {"type": "fault", "code": int, "ts_us": int}
    """
    clip_path = Path(clip_path)
    sidecar_path = clip_path.with_suffix(".json")

    with open(clip_path, "wb") as f:
        write_file_header(f, format_version=file_header_format_version)
        for chunk in chunks:
            if chunk["type"] == "data":
                write_data_chunk(
                    f,
                    frames=chunk["frames"],
                    chunk_timestamp_us=chunk["ts_us"],
                    bit_depth=bit_depth,
                )
            elif chunk["type"] == "fault":
                write_fault_sentinel(
                    f,
                    fault_code=chunk["code"],
                    chunk_timestamp_us=chunk["ts_us"],
                    bit_depth=bit_depth,
                    frame_width=width,
                    frame_height=height,
                )
            elif chunk["type"] == "garbage":
                # Append `n` bytes of non-magic garbage (simulates brown-out mid-write)
                f.write(b"\xff" * chunk["n"])
            else:
                raise ValueError(f"Unknown chunk type: {chunk['type']}")

    sidecar = make_test_sidecar(
        clip_path,
        fps=fps,
        bit_depth=bit_depth,
        width=width,
        height=height,
    )
    if sidecar_overrides is not None:
        sidecar.update(sidecar_overrides)
    sidecar_path.write_text(json.dumps(sidecar, indent=2))
    return sidecar
