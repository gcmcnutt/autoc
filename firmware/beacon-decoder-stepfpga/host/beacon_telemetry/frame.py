"""Telemetry frame contract — the StepFPGA correlator's UART output (gateware ↔ host).

ASCII, comma-separated, newline-terminated. ASCII is chosen for first bring-up: human-readable on any
serial terminal, trivial for the FPGA's lock-FSM to emit, and resync-tolerant (lock onto the MAGIC prefix
after line noise). The gateware (fpga-toolchain-plan §3.6 block 7) MUST emit this; this module parses it.

Line:  BCN,<seq>,<adc>,<corrA>,<lockA>,<marginA>,<corrB>,<lockB>,<marginB>,<rateA>,<rateB>,<recA>,<recB>\\n
  recX     uint   recovery latency in SAMPLES (signal-return -> confirmed lock), gateware-measured; ms = rec/0.48

  seq      uint   frame counter (wraps; for drop detection)
  adc      int    latest raw/decimated ADC sample
  corrX    int    correlation peak for beacon X (A/B)
  lockX     0|1|2 lock state: 0=no_lock, 1=tentative, 2=confirmed   (the §3.6 lock ladder)
  marginX  int    correlation-margin / SNR proxy for beacon X (the 0-9 quality)
  rateX    uint   DPLL rate estimate, offset-binary: slip = (rateX-32768)/32 ; chip_rate_Hz = N*480/(L+slip)
                  (N=31, L=74 -> see chip_rate_hz()). Held (frozen) through outages = the frequency flywheel.
                  NB 32768 (slip=0) = the TEMPLATE nominal (L=74 samples/period = 201.08 Hz), because L rounds
                  2.4*31=74.4 down; a locked 200 Hz emitter converges to slip~=+0.4 -> rate ~=32781 -> 200 Hz.

Two beacons because two codes share the one detector (CDMA). Each beacon has its OWN lock/margin/rate (the
per-beacon independent DPLL of acquisition-research-plan §5).
"""
from __future__ import annotations
from dataclasses import dataclass, fields as _fields

MAGIC = "BCN"
LOCK_NAMES = {0: "no_lock", 1: "tentative", 2: "confirmed"}   # the §3.6 lock ladder
# (A4d-2 ½-word "candidate" tier investigated & deferred — too false-alarm-prone at N=31; see DESIGN.md §5.)
RATE_BIAS = 32768          # offset-binary zero point
RATE_SCALE = 32           # internal IIR scaling (slip << 5)
N_CHIPS = 31              # gateware code length (keep in sync with s3.v localparam N)
SAMPLE_HZ = 480.0
NOM_SAMPLES_PER_PERIOD = round(2.4 * N_CHIPS)   # L in the gateware (74 for N=31)


def chip_rate_hz(rate: int) -> float:
    """Recover the emitter chip rate (Hz) from a telemetry rate field. NOM (200 Hz) at rate==RATE_BIAS.
    A faster emitter makes the correlation peak arrive earlier each period (negative phase-slip), so
    chip_rate = N*SAMPLE_HZ / (L + slip)."""
    slip = (rate - RATE_BIAS) / RATE_SCALE
    denom = NOM_SAMPLES_PER_PERIOD + slip
    return N_CHIPS * SAMPLE_HZ / denom if denom else float("inf")


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
    rateA: int = RATE_BIAS
    rateB: int = RATE_BIAS
    recA: int = 0          # recovery latency, SAMPLES from signal-return to last lock (gateware-measured)
    recB: int = 0

    _ORDER = ("seq", "adc", "corrA", "lockA", "marginA", "corrB", "lockB", "marginB",
              "rateA", "rateB", "recA", "recB")

    @property
    def chip_rate_a_hz(self) -> float: return chip_rate_hz(self.rateA)
    @property
    def chip_rate_b_hz(self) -> float: return chip_rate_hz(self.rateB)
    @property
    def recA_ms(self) -> float: return self.recA * 1000.0 / SAMPLE_HZ     # samples -> ms
    @property
    def recB_ms(self) -> float: return self.recB * 1000.0 / SAMPLE_HZ
    @property
    def recA_chips(self) -> float: return self.recA / 2.4                  # samples -> chips
    @property
    def recB_chips(self) -> float: return self.recB / 2.4

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
