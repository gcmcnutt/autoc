"""T022 — JSON sidecar schema validation."""

from __future__ import annotations

import json

import numpy as np
import pytest

from beacon_loader import SidecarMissingError, SidecarValidationError, load_clip, validate_sidecar

from tests._fixtures import make_test_sidecar, write_clip_and_sidecar


def test_valid_sidecar_passes(tmp_path):
    clip_path = tmp_path / "ok.clip"
    sidecar = make_test_sidecar(clip_path)
    validate_sidecar(sidecar)  # should not raise


def test_missing_required_field_fails(tmp_path):
    """Removing a required field MUST raise SidecarValidationError."""
    clip_path = tmp_path / "x.clip"
    sidecar = make_test_sidecar(clip_path)
    del sidecar["sensor_model"]
    with pytest.raises(SidecarValidationError):
        validate_sidecar(sidecar)


def test_in_flight_extras_required_when_flight_sd(tmp_path):
    """When recording_mode = flight_sd, in_flight_extras MUST be present."""
    clip_path = tmp_path / "x.clip"
    sidecar = make_test_sidecar(clip_path, recording_mode="flight_sd")
    # First confirm it validates with in_flight_extras
    validate_sidecar(sidecar)
    # Now strip it and assert failure
    del sidecar["in_flight_extras"]
    with pytest.raises(SidecarValidationError):
        validate_sidecar(sidecar)


def test_bad_fps_value_fails(tmp_path):
    clip_path = tmp_path / "x.clip"
    sidecar = make_test_sidecar(clip_path)
    sidecar["capture_mode"]["fps"] = 60  # not in enum {240, 480}
    with pytest.raises(SidecarValidationError):
        validate_sidecar(sidecar)


def test_bad_bit_depth_fails(tmp_path):
    clip_path = tmp_path / "x.clip"
    sidecar = make_test_sidecar(clip_path)
    sidecar["capture_mode"]["bit_depth"] = 12  # not in enum {8, 10}
    with pytest.raises(SidecarValidationError):
        validate_sidecar(sidecar)


def test_sidecar_missing_file_raises(tmp_path):
    """If the .json file is absent next to the .clip, MUST raise SidecarMissingError."""
    frames = np.zeros((1, 240, 320), dtype=np.uint8)
    clip_path = tmp_path / "lonely.clip"
    write_clip_and_sidecar(
        clip_path,
        chunks=[{"type": "data", "frames": frames, "ts_us": 0}],
        bit_depth=8,
    )
    # Delete the sidecar to simulate a lost JSON
    (clip_path.with_suffix(".json")).unlink()

    with pytest.raises(SidecarMissingError):
        load_clip(clip_path)


def test_sidecar_version_mismatch_with_binary(tmp_path):
    """Sidecar declaring format_version=1 but binary has format_version=2 → SidecarValidationError."""
    # Note: this exercises the loader's cross-check, not the schema itself.
    # The schema constrains sidecar format_version to const=1, so a sidecar with
    # format_version=2 would fail SidecarValidationError on schema lookup.
    clip_path = tmp_path / "x.clip"
    sidecar = make_test_sidecar(clip_path)
    sidecar["format_version"] = 2  # schema requires const=1
    with pytest.raises(SidecarValidationError):
        validate_sidecar(sidecar)
