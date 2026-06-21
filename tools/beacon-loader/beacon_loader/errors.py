"""Error types — fail-loud per Principle V.

See contracts/python-loader.md for the error contract.
"""


class BeaconLoaderError(Exception):
    """Base for all beacon-loader exceptions."""


class FormatVersionError(BeaconLoaderError):
    """Raised when the on-disk format_version does not match the loader's supported version.

    Per Principle V: the loader MUST fail loud with a message naming both versions —
    never silently default-init, truncate, or otherwise pretend success.
    """


class MagicMismatchError(BeaconLoaderError):
    """Raised when the file-header magic does not match the expected sentinel."""


class SidecarValidationError(BeaconLoaderError):
    """Raised when the JSON sidecar fails schema validation."""


class SidecarMissingError(BeaconLoaderError):
    """Raised when the JSON sidecar file is not found next to the .clip file."""
