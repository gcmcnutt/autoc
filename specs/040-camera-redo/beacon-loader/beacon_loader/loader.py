"""High-level load_clip() public API.

Implements the contract from contracts/python-loader.md:
  load_clip(path) -> (np.ndarray, dict)

Caller receives all surviving frames + metadata enriched with:
  - recovered_frame_count, recovered_chunk_count
  - fault_events[]
  - fully_intact
  - partial_recovery_warning
  - truncated_at_offset
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np

from beacon_loader.chunk import iter_chunks, read_file_header
from beacon_loader.errors import (
    FormatVersionError,
    MagicMismatchError,
    SidecarMissingError,
    SidecarValidationError,
)
from beacon_loader.schema import validate_sidecar_dict


def load_clip(clip_path: str | os.PathLike) -> tuple[np.ndarray, dict]:
    """Load a Phase-1 beacon-camera clip (binary + JSON sidecar pair).

    See contracts/python-loader.md for the full contract.
    """
    clip_path = Path(clip_path)
    sidecar_path = clip_path.with_suffix(".json")

    if not clip_path.exists():
        raise FileNotFoundError(f"Clip binary not found: {clip_path}")
    if not sidecar_path.exists():
        raise SidecarMissingError(
            f"JSON sidecar not found at expected path: {sidecar_path}"
        )

    # Validate the sidecar first (cheap; surfaces schema errors before parsing the binary)
    with open(sidecar_path) as f:
        sidecar = json.load(f)
    validate_sidecar_dict(sidecar)

    # Read file header — raises FormatVersionError / MagicMismatchError on bad files
    with open(clip_path, "rb") as f:
        file_header = read_file_header(f)

    # Cross-check sidecar format_version against file header
    if sidecar["format_version"] != file_header.format_version:
        raise SidecarValidationError(
            f"Sidecar format_version={sidecar['format_version']} does not match "
            f"file header format_version={file_header.format_version}"
        )

    # Iterate chunks, collect frames + fault events
    all_frames: list[np.ndarray] = []
    fault_events: list[dict] = []
    data_chunk_count = 0
    last_offset_seen = file_header.header_size

    for record in iter_chunks(clip_path):
        if record.is_fault_sentinel:
            fault_events.append({
                "offset_bytes": record.offset_bytes,
                "timestamp_us": record.header.chunk_timestamp_us,
                "fault_code": record.fault_code,
                "fault_name": record.fault_name,
            })
        else:
            all_frames.append(record.frames)
            data_chunk_count += 1
        last_offset_seen = record.offset_bytes + record.header.chunk_size_bytes

    # Determine if file ended cleanly or via truncation
    file_size = clip_path.stat().st_size
    truncated_at_offset: int | None = None
    fully_intact = True
    partial_recovery_warning: str | None = None

    if last_offset_seen != file_size:
        # Iteration stopped before EOF — chunk-magic mismatch or other truncation
        truncated_at_offset = last_offset_seen
        fully_intact = False
        partial_recovery_warning = (
            f"Clip iteration stopped at offset {last_offset_seen} "
            f"(file size = {file_size} bytes; {file_size - last_offset_seen} trailing bytes unparsed). "
            f"Most likely cause: brown-out mid-CMD25 or SD-removal mid-write."
        )

    if fault_events:
        fully_intact = False
        if partial_recovery_warning is None:
            partial_recovery_warning = (
                f"Clip contains {len(fault_events)} fault-sentinel event(s) "
                f"— recorder reset-and-continue path was exercised. See metadata['fault_events']."
            )

    # Concatenate frames into one ndarray (handles empty case)
    if all_frames:
        frames = np.concatenate(all_frames, axis=0)
    else:
        # No data chunks (corner case — file with only a FINAL_UNRECOVERABLE sentinel)
        # Return an empty array with the expected dimensionality
        cap_mode = sidecar["capture_mode"]
        h, w = cap_mode["resolution"][1], cap_mode["resolution"][0]
        dtype = np.uint8 if cap_mode["bit_depth"] == 8 else np.uint16
        frames = np.empty((0, h, w), dtype=dtype)

    # Build augmented metadata
    metadata = dict(sidecar)
    metadata["recovered_frame_count"] = int(frames.shape[0])
    metadata["recovered_chunk_count"] = data_chunk_count
    metadata["fault_events"] = fault_events
    metadata["fully_intact"] = fully_intact
    if partial_recovery_warning is not None:
        metadata["partial_recovery_warning"] = partial_recovery_warning
    metadata["truncated_at_offset"] = truncated_at_offset

    return frames, metadata


# Re-export iter_chunks + validate_sidecar at this module so the package
# __init__ can pull them from one place.
__all__ = ["load_clip", "iter_chunks", "validate_sidecar"]


def validate_sidecar(sidecar_dict: dict) -> None:
    """Public alias for schema.validate_sidecar_dict — convenience for callers."""
    validate_sidecar_dict(sidecar_dict)
