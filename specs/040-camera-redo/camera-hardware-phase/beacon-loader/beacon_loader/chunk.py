"""Low-level binary chunk reader.

Implements the iter_chunks() streaming primitive. Higher-level load_clip()
in loader.py is built on top.

See contracts/data-format.md for the byte layout.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import BinaryIO, Iterator

import numpy as np

from beacon_loader.errors import FormatVersionError, MagicMismatchError
from beacon_loader.schema import (
    CHUNK_HEADER_SIZE,
    CHUNK_MAGIC,
    CHUNK_TRAILER_SIZE,
    FILE_HEADER_SIZE,
    FILE_MAGIC,
    FORMAT_VERSION,
    fault_code_name,
    frame_payload_size,
)

# === File header =================================================================

_FILE_HEADER_FMT = "<IHHQ"  # magic uint32, format_version uint16, header_size uint16, session_start_us uint64
assert struct.calcsize(_FILE_HEADER_FMT) == FILE_HEADER_SIZE


@dataclass
class FileHeader:
    magic: int
    format_version: int
    header_size: int
    session_start_us: int


def read_file_header(f: BinaryIO) -> FileHeader:
    """Read + validate the file header. Raises on magic / format_version mismatch."""
    raw = f.read(FILE_HEADER_SIZE)
    if len(raw) != FILE_HEADER_SIZE:
        raise MagicMismatchError(
            f"Clip file is too short to contain a file header "
            f"({len(raw)} bytes, expected ≥{FILE_HEADER_SIZE})"
        )
    magic, format_version, header_size, session_start_us = struct.unpack(_FILE_HEADER_FMT, raw)

    if magic != FILE_MAGIC:
        raise MagicMismatchError(
            f"Clip file magic mismatch: got 0x{magic:08X}, expected 0x{FILE_MAGIC:08X}. "
            f"This file does not appear to be a beacon-camera .clip"
        )
    if format_version != FORMAT_VERSION:
        # Principle V fail-loud: name BOTH the file version and the loader version.
        raise FormatVersionError(
            f"Clip file format_version={format_version}, loader supports format_version={FORMAT_VERSION}. "
            f"Re-record with current FPGA firmware OR update the loader."
        )
    return FileHeader(
        magic=magic,
        format_version=format_version,
        header_size=header_size,
        session_start_us=session_start_us,
    )


# === Chunk header ===============================================================

_CHUNK_HEADER_FMT = "<IHHIQBBHHHBBBB"
# Fields:
#  chunk_magic uint32, chunk_format_version uint16, _pad0 uint16,
#  chunk_size_bytes uint32, chunk_timestamp_us uint64,
#  bit_depth uint8, _pad1 uint8, frame_count uint16,
#  frame_width uint16, frame_height uint16,
#  _reserved[0..3] uint8 x4
assert struct.calcsize(_CHUNK_HEADER_FMT) == CHUNK_HEADER_SIZE


@dataclass
class ChunkHeader:
    chunk_magic: int
    chunk_format_version: int
    chunk_size_bytes: int
    chunk_timestamp_us: int
    bit_depth: int
    frame_count: int
    frame_width: int
    frame_height: int
    reserved: bytes  # 4 bytes — for fault-sentinels, bytes[0..3] = fault_code (uint32 LE)

    @property
    def is_fault_sentinel(self) -> bool:
        return self.frame_count == 0

    @property
    def fault_code(self) -> int:
        """Decode fault_code (uint32 LE) from _reserved[0..3]. Meaningful only for fault sentinels."""
        return int.from_bytes(self.reserved[:4], "little")


def parse_chunk_header(raw: bytes) -> ChunkHeader:
    """Parse 32-byte chunk header. Caller checks chunk_magic for validity."""
    (
        chunk_magic, chunk_format_version, _pad0,
        chunk_size_bytes, chunk_timestamp_us,
        bit_depth, _pad1, frame_count,
        frame_width, frame_height,
        r0, r1, r2, r3,
    ) = struct.unpack(_CHUNK_HEADER_FMT, raw)
    return ChunkHeader(
        chunk_magic=chunk_magic,
        chunk_format_version=chunk_format_version,
        chunk_size_bytes=chunk_size_bytes,
        chunk_timestamp_us=chunk_timestamp_us,
        bit_depth=bit_depth,
        frame_count=frame_count,
        frame_width=frame_width,
        frame_height=frame_height,
        reserved=bytes((r0, r1, r2, r3)),
    )


# === Frame unpacking ============================================================

def unpack_10bit_packed(packed: bytes, n_pixels: int) -> np.ndarray:
    """Unpack 4-pixels-in-5-bytes 10-bit packed format → uint16 array (0..1023).

    Layout per contracts/data-format.md: MSB-first per the OG0VA/OV9281 native MIPI output.
    Byte 0,1,2,3 = MSBs of 4 pixels; byte 4 = packed LSBs (2 bits each).

    NOTE: this is one common layout interpretation; the actual sensor wire-format
    layout MUST be confirmed at FPGA bring-up (T053) and may require a layout swap.
    The format is captured here as the canonical decoder; the recorder MUST emit
    the same byte-order it expects to be read back.
    """
    if n_pixels % 4 != 0:
        raise ValueError(f"10-bit packed requires n_pixels divisible by 4; got {n_pixels}")
    expected_bytes = n_pixels * 10 // 8
    if len(packed) != expected_bytes:
        raise ValueError(
            f"10-bit packed buffer size mismatch: got {len(packed)} bytes, expected {expected_bytes}"
        )

    out = np.empty(n_pixels, dtype=np.uint16)
    arr = np.frombuffer(packed, dtype=np.uint8)
    # Process 5-byte blocks
    block_count = n_pixels // 4
    blocks = arr.reshape(block_count, 5)
    # Top 8 bits of each of the 4 pixels in this block
    out_4 = out.reshape(block_count, 4)
    out_4[:, 0] = blocks[:, 0].astype(np.uint16) << 2
    out_4[:, 1] = blocks[:, 1].astype(np.uint16) << 2
    out_4[:, 2] = blocks[:, 2].astype(np.uint16) << 2
    out_4[:, 3] = blocks[:, 3].astype(np.uint16) << 2
    # LSB-2 bits of each (packed in byte 4): bits 7-6 → pixel 0, 5-4 → pixel 1, 3-2 → pixel 2, 1-0 → pixel 3
    out_4[:, 0] |= (blocks[:, 4].astype(np.uint16) >> 6) & 0x3
    out_4[:, 1] |= (blocks[:, 4].astype(np.uint16) >> 4) & 0x3
    out_4[:, 2] |= (blocks[:, 4].astype(np.uint16) >> 2) & 0x3
    out_4[:, 3] |= blocks[:, 4].astype(np.uint16) & 0x3
    return out


def pack_10bit(pixels: np.ndarray) -> bytes:
    """Inverse of unpack_10bit_packed — used by the test-vector writer."""
    n = pixels.size
    if n % 4 != 0:
        raise ValueError(f"10-bit pack requires n_pixels divisible by 4; got {n}")
    pix = pixels.astype(np.uint16).reshape(-1, 4)
    out = np.empty((pix.shape[0], 5), dtype=np.uint8)
    out[:, 0] = (pix[:, 0] >> 2).astype(np.uint8)
    out[:, 1] = (pix[:, 1] >> 2).astype(np.uint8)
    out[:, 2] = (pix[:, 2] >> 2).astype(np.uint8)
    out[:, 3] = (pix[:, 3] >> 2).astype(np.uint8)
    out[:, 4] = (
        ((pix[:, 0] & 0x3) << 6)
        | ((pix[:, 1] & 0x3) << 4)
        | ((pix[:, 2] & 0x3) << 2)
        | (pix[:, 3] & 0x3)
    ).astype(np.uint8)
    return out.tobytes()


# === Chunk record (yielded by iter_chunks) ====================================

@dataclass
class ChunkRecord:
    """One parsed chunk's data — either a DataChunk (frames present) or a FaultSentinel."""
    offset_bytes: int  # file offset where this chunk header started
    header: ChunkHeader
    frames: np.ndarray | None  # shape (frame_count, height, width); None for fault-sentinels
    frame_timestamps_us: np.ndarray | None  # absolute µs per frame; None for fault-sentinels
    crc_ok: bool
    crc_expected: int
    crc_computed: int

    @property
    def is_fault_sentinel(self) -> bool:
        return self.header.is_fault_sentinel

    @property
    def fault_code(self) -> int:
        return self.header.fault_code

    @property
    def fault_name(self) -> str:
        return fault_code_name(self.fault_code)


# === Iteration ================================================================

def iter_chunks(clip_path: str | Path) -> Iterator[ChunkRecord]:
    """Stream chunks one at a time from a .clip file.

    Yields ChunkRecord for both data chunks AND fault-sentinel chunks. Caller
    discriminates via record.is_fault_sentinel.

    Stops iteration cleanly when:
      - EOF reached normally (file fully intact)
      - Chunk magic mismatch encountered (brown-out / truncation per FR-2.5)
      - Chunk version mismatch (treated like brown-out — partial recovery)

    Either fault-tolerant stop is normal behavior — caller inspects fully_intact
    via the higher-level load_clip() metadata.
    """
    clip_path = Path(clip_path)
    with open(clip_path, "rb") as f:
        file_header = read_file_header(f)
        offset = file_header.header_size

        while True:
            # Try to read the next chunk header
            f.seek(offset)
            raw_hdr = f.read(CHUNK_HEADER_SIZE)
            if len(raw_hdr) < CHUNK_HEADER_SIZE:
                # Clean EOF
                return

            hdr = parse_chunk_header(raw_hdr)

            if hdr.chunk_magic != CHUNK_MAGIC:
                # Truncation / corruption — stop iteration cleanly (FR-2.5 brown-out path)
                return

            if hdr.chunk_format_version != FORMAT_VERSION:
                # Treat as truncation/corruption — refuse to parse with the wrong-version
                # interpretation; safer to stop than to silently produce wrong frames.
                return

            # Compute the absolute frame timestamps + parse pixel payload
            frame_count = hdr.frame_count
            payload_size = (
                frame_count * frame_payload_size(hdr.frame_width, hdr.frame_height, hdr.bit_depth)
                if frame_count > 0 else 0
            )
            payload = f.read(payload_size)
            if len(payload) != payload_size:
                # Truncated mid-payload — stop iteration cleanly
                return

            raw_crc = f.read(CHUNK_TRAILER_SIZE)
            if len(raw_crc) != CHUNK_TRAILER_SIZE:
                return
            crc_expected = struct.unpack("<I", raw_crc)[0]
            crc_computed = zlib.crc32(raw_hdr + payload) & 0xFFFFFFFF
            crc_ok = crc_expected == crc_computed

            if frame_count == 0:
                # Fault sentinel — no frames, just the header
                yield ChunkRecord(
                    offset_bytes=offset,
                    header=hdr,
                    frames=None,
                    frame_timestamps_us=None,
                    crc_ok=crc_ok,
                    crc_expected=crc_expected,
                    crc_computed=crc_computed,
                )
            else:
                frames, ts_us = _parse_frames(
                    payload,
                    frame_count=frame_count,
                    width=hdr.frame_width,
                    height=hdr.frame_height,
                    bit_depth=hdr.bit_depth,
                    chunk_ts_us=hdr.chunk_timestamp_us,
                )
                yield ChunkRecord(
                    offset_bytes=offset,
                    header=hdr,
                    frames=frames,
                    frame_timestamps_us=ts_us,
                    crc_ok=crc_ok,
                    crc_expected=crc_expected,
                    crc_computed=crc_computed,
                )

            offset += hdr.chunk_size_bytes


def _parse_frames(
    payload: bytes,
    frame_count: int,
    width: int,
    height: int,
    bit_depth: int,
    chunk_ts_us: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Parse the frames-block of a data chunk into (frames_array, abs_timestamps_us)."""
    per_frame_size = frame_payload_size(width, height, bit_depth)
    pixels_per_frame = width * height

    if bit_depth == 8:
        dtype = np.uint8
    else:  # 10
        dtype = np.uint16

    frames = np.empty((frame_count, height, width), dtype=dtype)
    timestamps_us = np.empty(frame_count, dtype=np.int64)

    for i in range(frame_count):
        base = i * per_frame_size
        delta_us = struct.unpack_from("<H", payload, base)[0]
        timestamps_us[i] = chunk_ts_us + delta_us
        pixel_bytes = payload[base + 2 : base + per_frame_size]
        if bit_depth == 8:
            frames[i] = np.frombuffer(pixel_bytes, dtype=np.uint8).reshape(height, width)
        else:
            frames[i] = unpack_10bit_packed(pixel_bytes, pixels_per_frame).reshape(height, width)

    return frames, timestamps_us
