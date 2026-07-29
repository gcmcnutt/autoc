"""beacon_loader — FR-4.3 Python loader for 031-beacon-camera .clip files.

Public API:
    load_clip(path) -> (np.ndarray, dict)
    iter_chunks(path) -> Iterator[ChunkRecord]
    validate_sidecar(sidecar_dict) -> None

See specs/040-camera-redo/camera-hardware-phase/contracts/python-loader.md for the full contract.
"""

from beacon_loader.loader import (
    load_clip,
    iter_chunks,
    validate_sidecar,
)
from beacon_loader.schema import (
    FORMAT_VERSION,
    FILE_MAGIC,
    CHUNK_MAGIC,
    FAULT_CODES,
    fault_code_name,
)
from beacon_loader.errors import (
    FormatVersionError,
    MagicMismatchError,
    SidecarValidationError,
    SidecarMissingError,
)
from beacon_loader.chunk import ChunkRecord

__all__ = [
    "load_clip",
    "iter_chunks",
    "validate_sidecar",
    "FORMAT_VERSION",
    "FILE_MAGIC",
    "CHUNK_MAGIC",
    "FAULT_CODES",
    "fault_code_name",
    "FormatVersionError",
    "MagicMismatchError",
    "SidecarValidationError",
    "SidecarMissingError",
    "ChunkRecord",
]

__version__ = "0.1.0"
