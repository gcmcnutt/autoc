"""Read + log the StepFPGA telemetry stream over USB-serial (after `usbipd attach`, see host/README.md).

Kept import-light: `pyserial` is imported lazily inside read_serial() so the parser/mock + tests run with
no hardware and no pyserial installed.
"""
from __future__ import annotations
import sys
from typing import Iterable, Iterator, TextIO
from .frame import TelemetryFrame


def frames_from_lines(lines: Iterable[str]) -> Iterator[TelemetryFrame]:
    """Parse an iterable of text lines → valid frames (silently skips noise/partial lines). Source-agnostic
    so it works identically over a real serial port, a recorded log, or the mock."""
    for line in lines:
        fr = TelemetryFrame.parse(line)
        if fr is not None:
            yield fr


def read_serial(port: str = "/dev/ttyACM0", baud: int = 115200, timeout: float = 1.0) -> Iterator[TelemetryFrame]:
    """Yield frames from a live serial port. `port` is the WSL device after `usbipd attach` (CDC-ACM →
    /dev/ttyACM0 typically). Raises a clear hint if the port is absent (usually = not attached)."""
    try:
        import serial  # pyserial
    except ImportError as e:
        raise SystemExit("pyserial not installed — `pip install -e firmware/beacon-decoder-stepfpga/host`") from e

    def _lines() -> Iterator[str]:
        try:
            ser = serial.Serial(port, baud, timeout=timeout)
        except serial.SerialException as e:
            raise SystemExit(
                f"cannot open {port}: {e}\n"
                f"  is the StepFPGA attached to WSL?  usbipd attach --wsl --busid 2-1  (bind once as admin)\n"
                f"  see firmware/beacon-decoder-stepfpga/host/README.md"
            ) from e
        with ser:
            while True:
                raw = ser.readline()
                if raw:
                    yield raw.decode("ascii", "replace")

    yield from frames_from_lines(_lines())


def main(argv: list[str] | None = None) -> int:
    """CLI: stream frames to stdout, optionally tee a raw log (the §6 'stream to laptop' logging path)."""
    import argparse
    ap = argparse.ArgumentParser(description="Read StepFPGA beacon telemetry over USB-serial.")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--log", help="append raw frame lines to this file")
    args = ap.parse_args(argv)
    logf: TextIO | None = open(args.log, "a") if args.log else None
    for fr in read_serial(args.port, args.baud):
        line = fr.format()
        print(f"seq={fr.seq:6d} adc={fr.adc:5d} A[corr={fr.corrA} {fr.lockA}] B[corr={fr.corrB} {fr.lockB}]")
        if logf:
            logf.write(line + "\n"); logf.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
