"""Host-side reader for the StepFPGA beacon-correlator UART telemetry (031 acquisition phase)."""
from .frame import TelemetryFrame, MAGIC, LOCK_NAMES
from .reader import frames_from_lines, read_serial
from .mock import synth_frames, synth_lines

__all__ = ["TelemetryFrame", "MAGIC", "LOCK_NAMES",
           "frames_from_lines", "read_serial", "synth_frames", "synth_lines"]
