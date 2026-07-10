"""T021 — Principle V fail-loud on format_version mismatch."""

from __future__ import annotations

import numpy as np
import pytest

from beacon_loader import FormatVersionError, MagicMismatchError, load_clip

from tests._fixtures import write_clip_and_sidecar


def test_format_version_2_fails_loud(tmp_path):
    """A file with format_version=2 MUST raise FormatVersionError with both versions named."""
    frames = np.zeros((1, 240, 320), dtype=np.uint8)
    clip_path = tmp_path / "wrong_version.clip"
    write_clip_and_sidecar(
        clip_path,
        chunks=[{"type": "data", "frames": frames, "ts_us": 0}],
        bit_depth=8,
        file_header_format_version=2,    # NOT supported by this loader
        sidecar_overrides={"format_version": 1},  # sidecar fine; binary is the violator
    )

    with pytest.raises(FormatVersionError) as exc_info:
        load_clip(clip_path)

    msg = str(exc_info.value)
    # Per Principle V: error message MUST name both the file version + the loader version
    assert "format_version=2" in msg, f"Error must name file's format_version: {msg}"
    assert "format_version=1" in msg, f"Error must name loader's supported version: {msg}"


def test_bad_magic_fails_loud(tmp_path):
    """A file with the wrong magic sentinel MUST raise MagicMismatchError."""
    clip_path = tmp_path / "wrong_magic.clip"
    # Write 16 bytes that look like a header but with garbage magic
    with open(clip_path, "wb") as f:
        f.write(b"\x00\x00\x00\x00")     # magic = 0x00000000 (not 0x0BEAC031)
        f.write(b"\x01\x00\x10\x00")     # format_version=1, header_size=16
        f.write(b"\x00" * 8)              # session_start_us=0

    # Sidecar still has to exist for load_clip to even open the binary
    (clip_path.with_suffix(".json")).write_text(
        '{"clip_id":"x","clip_file":"wrong_magic.clip","format_version":1,'
        '"iso_start_time":"2026-05-18T12:00:00Z","sensor_model":"OV9281",'
        '"lens_spec":{"model":"x","h_fov_deg":120,"f_number":2,'
        '"filter_cwl_nm":850,"filter_fwhm_nm":10},'
        '"capture_mode":{"resolution":[320,240],"fps":240,"bit_depth":8},'
        '"sensor_settings":{"exposure_us":500,"analog_gain":1,"digital_gain":1},'
        '"ambient":"indoor","pose_qualifier":"stationary",'
        '"recording_mode":"bench_tethered"}'
    )

    with pytest.raises(MagicMismatchError) as exc_info:
        load_clip(clip_path)

    assert "magic" in str(exc_info.value).lower()


def test_no_partial_load_on_version_mismatch(tmp_path):
    """Loader MUST NOT return any frames when format_version is wrong — fail loud only."""
    frames = np.ones((3, 240, 320), dtype=np.uint8) * 42
    clip_path = tmp_path / "bad.clip"
    write_clip_and_sidecar(
        clip_path,
        chunks=[{"type": "data", "frames": frames, "ts_us": 0}],
        bit_depth=8,
        file_header_format_version=99,
    )

    with pytest.raises(FormatVersionError):
        load_clip(clip_path)

    # The frames were never returned (no exception-with-partial-data nonsense)
