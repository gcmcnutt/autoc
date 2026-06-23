"""Synthetic telemetry — exercise the reader/parser with NO hardware (and stand in until the gateware
emits real frames). Models a Stage-0/1 acquisition: beacon A ramps no_lock → tentative → confirmed."""
from __future__ import annotations
from typing import Iterator
from .frame import TelemetryFrame


def synth_frames(n: int = 60) -> Iterator[TelemetryFrame]:
    for seq in range(n):
        lockA = 0 if seq < 10 else (1 if seq < 20 else 2)
        corrA = 4 + seq // 4 if lockA else 2          # correlation climbs as it locks
        marginA = max(0, corrA - 3)
        adc = 2048 + (seq % 16) - 8                   # a little wobble around mid-scale
        yield TelemetryFrame(seq=seq, adc=adc, corrA=corrA, lockA=lockA, marginA=marginA,
                             corrB=2, lockB=0, marginB=0)


def synth_lines(n: int = 60, with_noise: bool = True) -> Iterator[str]:
    """Canonical lines, optionally interleaved with junk to exercise resync/skip."""
    for fr in synth_frames(n):
        if with_noise and fr.seq % 7 == 0:
            yield "garbage partial line"              # reader must skip this, not crash
        yield fr.format()
