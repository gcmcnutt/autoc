"""Schema constants + JSON sidecar validation.

The byte-layout constants here are the executable form of
contracts/data-format.md (Principle V — same source of truth).
"""

from __future__ import annotations

import json
from pathlib import Path

import jsonschema

from beacon_loader.errors import SidecarValidationError

# === Byte-layout constants (must match contracts/data-format.md v1) ============

FORMAT_VERSION = 1
FILE_MAGIC = 0x0BEAC031
CHUNK_MAGIC = 0xCAFEC0DE

FILE_HEADER_SIZE = 16          # magic(4) + format_version(2) + header_size(2) + session_start_us(8)
CHUNK_HEADER_SIZE = 32         # see contracts/data-format.md
CHUNK_TRAILER_SIZE = 4         # crc32

FRAME_TIMESTAMP_DELTA_SIZE = 2  # uint16 µs delta from chunk_timestamp_us


def frame_payload_size(width: int, height: int, bit_depth: int) -> int:
    """Bytes per frame including the 2-byte timestamp delta header.

    Per contracts/data-format.md:
      8-bit:  N = width * height
      10-bit packed: N = width * height * 10 / 8
    """
    pixels = width * height
    if bit_depth == 8:
        pixel_bytes = pixels
    elif bit_depth == 10:
        # 4 consecutive 10-bit samples packed into 5 bytes
        if pixels % 4 != 0:
            raise ValueError(
                f"10-bit packed format requires width*height divisible by 4; got {pixels}"
            )
        pixel_bytes = pixels * 10 // 8
    else:
        raise ValueError(f"Unsupported bit_depth: {bit_depth} (must be 8 or 10)")
    return FRAME_TIMESTAMP_DELTA_SIZE + pixel_bytes


# === Fault codes (must match contracts/data-format.md table) ==================

FAULT_CODES: dict[int, str] = {
    0x00000001: "SDIO_WRITE_ERROR_TRANSIENT",
    0x00000002: "SDIO_WRITE_ERROR_RECOVERED",
    0x00000003: "RING_BUFFER_OVERRUN",
    0x00000004: "SENSOR_I2C_RECOVERED",
    0x00000005: "MIPI_DESYNC_RECOVERED",
    0x000000FF: "FINAL_UNRECOVERABLE",
}


def fault_code_name(code: int) -> str:
    """Map a fault_code integer to a human-readable name.

    Unknown codes (including 031-fpga-and-beyond reserved range 0x80000000+) are
    returned as 'UNKNOWN_<hex>' rather than raising — the loader's job is to
    surface what's in the file, not to gate on unknown-but-skippable sentinels.
    """
    return FAULT_CODES.get(code, f"UNKNOWN_0x{code:08X}")


# === JSON sidecar schema validation ===========================================

_SIDECAR_SCHEMA_PATH = (
    Path(__file__).resolve().parent.parent.parent.parent
    / "specs" / "031-beacon-camera" / "contracts" / "json-sidecar-schema.json"
)

_cached_schema = None


def _load_schema() -> dict:
    """Lazy-load the JSON schema; cache it after first read."""
    global _cached_schema
    if _cached_schema is None:
        if not _SIDECAR_SCHEMA_PATH.exists():
            raise FileNotFoundError(
                f"JSON sidecar schema not found at expected path: {_SIDECAR_SCHEMA_PATH}"
            )
        with open(_SIDECAR_SCHEMA_PATH) as f:
            _cached_schema = json.load(f)
    return _cached_schema


def validate_sidecar_dict(sidecar: dict) -> None:
    """Validate a parsed sidecar dict against the canonical schema.

    Raises SidecarValidationError on any schema deviation.
    """
    try:
        jsonschema.validate(instance=sidecar, schema=_load_schema())
    except jsonschema.ValidationError as e:
        raise SidecarValidationError(
            f"JSON sidecar failed schema validation: {e.message} "
            f"(at path: {'.'.join(str(p) for p in e.absolute_path) or '<root>'})"
        ) from e
