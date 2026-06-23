"""Parser + reader tests — run with NO hardware (pure parse + mock stream)."""
from beacon_telemetry import TelemetryFrame, frames_from_lines, synth_lines, synth_frames


def test_roundtrip():
    fr = TelemetryFrame(seq=42, adc=2050, corrA=9, lockA=2, marginA=6, corrB=3, lockB=1, marginB=1)
    assert TelemetryFrame.parse(fr.format()) == fr


def test_skips_noise_and_partials():
    assert TelemetryFrame.parse("garbage partial line") is None
    assert TelemetryFrame.parse("BCN,1,2,3") is None          # too few fields
    assert TelemetryFrame.parse("BCN,a,b,c,d,e,f,g,h") is None  # non-int
    assert TelemetryFrame.parse("") is None


def test_reader_recovers_through_noise():
    # synth_lines injects junk lines; the reader must skip them and yield only valid frames in order.
    frames = list(frames_from_lines(synth_lines(n=30)))
    assert len(frames) == 30
    assert [f.seq for f in frames] == list(range(30))


def test_lock_ramp():
    frames = list(synth_frames(30))
    assert frames[0].lockA == 0 and not frames[0].any_lock     # no_lock early
    assert frames[-1].lockA == 2 and frames[-1].any_lock        # confirmed late
