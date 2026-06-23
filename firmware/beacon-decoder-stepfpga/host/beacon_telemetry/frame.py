"""Telemetry frame contract — the StepFPGA correlator's UART output (gateware ↔ host).

ASCII, comma-separated, newline-terminated. ASCII is chosen for first bring-up: human-readable on any
serial terminal, trivial for the FPGA's lock-FSM to emit, and resync-tolerant (lock onto the MAGIC prefix
after line noise). The gateware (fpga-toolchain-plan §3.6 block 7) MUST emit this; this module parses it.

Line:  BCN,<seq>,<adc>,<corrA>,<lockA>,<marginA>,<corrB>,<lockB>,<marginB>\\n

  seq      uint   frame counter (wraps; for drop detection)
  adc      int    latest raw/decimated ADC sample
  corrX    int    correlation peak for beacon X (A/B)
  lockX     0|1|2 lock state: 0=no_lock, 1=tentative, 2=confirmed   (the §3.6 lock ladder)
  marginX  int    correlation-margin / SNR proxy for beacon X

Two beacons because two codes share the one detector (CDMA). Each beacon has its OWN lock/margin (the
per-beacon independent DPLL of acquisition-research-plan §5).
"""
from __future__ import annotations
from dataclasses import dataclass, fields as _fields

MAGIC = "BCN"
LOCK_NAMES = {0: "no_lock", 1: "tentative", 2: "confirmed"}


@dataclass
class TelemetryFrame:
    seq: int
    adc: int
    corrA: int
    lockA: int
    marginA: int
    corrB: int
    lockB: int
    marginB: int

    _ORDER = ("seq", "adc", "corrA", "lockA", "marginA", "corrB", "lockB", "marginB")

    @classmethod
    def parse(cls, line: str) -> "TelemetryFrame | None":
        """Parse one line. Returns None for noise / partial / malformed lines (caller resyncs)."""
        parts = line.strip().split(",")
        if len(parts) != 1 + len(cls._ORDER) or parts[0] != MAGIC:
            return None
        try:
            vals = [int(p) for p in parts[1:]]
        except ValueError:
            return None
        return cls(**dict(zip(cls._ORDER, vals)))

    def format(self) -> str:
        """Render the canonical line (no trailing newline) — used by the mock + round-trip tests."""
        return ",".join([MAGIC, *(str(getattr(self, k)) for k in self._ORDER)])

    @property
    def any_lock(self) -> bool:
        return self.lockA >= 1 or self.lockB >= 1
