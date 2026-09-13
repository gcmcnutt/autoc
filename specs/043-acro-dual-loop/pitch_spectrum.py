#!/usr/bin/env python3
"""043 — pitch/roll oscillation spectrum from a decoded INAV blackbox CSV.

Written for the 2026-09-13 (t3) analysis and the 2026-08-23 (041-t7) baseline
comparison in `flight-analysis-20260913.md`.

⛔ STDLIB ONLY — deliberately. There is no numpy/scipy venv on the WSL bench host
(`python3 -c "import numpy"` fails), and this script must run wherever the flight
logs are. The FFT below is a plain recursive radix-2; at 512-sample segments over
a 20 s span it costs well under a second. Do NOT "improve" it into a numpy
dependency without first checking that every host that reads flight logs has one.

Engaged spans are found from `mspOverrideFlags == 2` in the blackbox itself, NOT
from the xiao flight log. That matters: the 041-t7 xiao log is format v4 and
`flightlog_decode.py` (v5) correctly refuses it, so the blackbox is the ONLY way
to time-align that baseline. Verified against the 2026-09-13 xiao log, whose
ENGAGE/DISENGAGE events match the flag transitions to within ~1 s (the flag
leads/lags the event by one MSP cycle plus the 0.75 s ramp at value 3).

Usage:
  python3 pitch_spectrum.py flight-results/flight-20260913/blackbox_log_2026-09-13_112457.01.csv
  python3 pitch_spectrum.py <csv> --manual     # also spectra for the MANUAL gaps
"""
import argparse
import cmath
import csv
import math
import statistics as st
import sys

# Channels worth a spectrum. servo[0]/servo[1] are COMPUTED, not sampled, so a
# peak there cannot be an aliasing artefact — that is what makes them the
# tiebreaker when asking whether a tone is real airframe motion.
CHANNELS = [
    ("gyro pitch", "gyroADC[1]"),
    ("gyro roll", "gyroADC[0]"),
    ("setpt pitch", "axisRate[1]"),
    ("setpt roll", "axisRate[0]"),
    ("servo0", "servo[0]"),
    ("servo1", "servo[1]"),
]
BANDS = [(0.3, 1), (1, 2), (2, 3), (3, 5), (5, 10), (10, 20)]

# The oscillation band under investigation (flight-analysis-20260913.md §5).
OSC_LO, OSC_HI = 1.5, 3.5


def fft(a):
    n = len(a)
    if n == 1:
        return a
    even, odd = fft(a[0::2]), fft(a[1::2])
    tw = [cmath.exp(-2j * math.pi * k / n) * odd[k] for k in range(n // 2)]
    return ([even[k] + tw[k] for k in range(n // 2)] +
            [even[k] - tw[k] for k in range(n // 2)])


def welch(x, fs, nseg=512, overlap=0.5):
    """Hann-windowed Welch PSD. Returns (freqs, power) or ([], []) if too short."""
    step = int(nseg * (1 - overlap))
    win = [0.5 - 0.5 * math.cos(2 * math.pi * i / (nseg - 1)) for i in range(nseg)]
    wpow = sum(v * v for v in win)
    segs, i = [], 0
    while i + nseg <= len(x):
        s = x[i:i + nseg]
        mean = sum(s) / nseg
        spec = fft([(v - mean) * win[j] for j, v in enumerate(s)])
        segs.append([abs(spec[k]) ** 2 / (fs * wpow) for k in range(nseg // 2 + 1)])
        i += step
    if not segs:
        return [], []
    power = [sum(s[k] for s in segs) / len(segs) for k in range(nseg // 2 + 1)]
    return [k * fs / nseg for k in range(nseg // 2 + 1)], power


def band_power(freqs, power, lo, hi):
    return sum(power[k] for k in range(len(freqs)) if lo <= freqs[k] < hi)


def bandpass(x, fs, lo, hi):
    """Zero-phase brick-wall bandpass via FFT masking (for amplitude/gain/phase)."""
    n = len(x)
    mean = sum(x) / n
    padded = [v - mean for v in x]
    size = 1
    while size < n:
        size *= 2
    padded += [0.0] * (size - n)
    spec = fft([complex(v) for v in padded])
    for k in range(size):
        f = k * fs / size if k <= size // 2 else (k - size) * fs / size
        if not (lo <= abs(f) < hi):
            spec[k] = 0
    inv = fft([v.conjugate() for v in spec])
    return [(v.conjugate() / size).real for v in inv[:n]]


def goertzel(x, fs, f0):
    """Single-bin Hann-windowed DFT — used for the phase at the peak."""
    n = len(x)
    mean = sum(x) / n
    win = [0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1)) for i in range(n)]
    return sum((x[i] - mean) * win[i] * cmath.exp(-2j * math.pi * f0 * i / fs)
               for i in range(n))


def load(path):
    with open(path) as fh:
        return [{k.strip(): v.strip() for k, v in row.items()}
                for row in csv.DictReader(fh)]


def num(row, key):
    try:
        return float(row[key])
    except (KeyError, ValueError, TypeError):
        return float("nan")


def engaged_spans(rows, times):
    """Contiguous stretches of mspOverrideFlags == 2 (NN fully in command)."""
    spans, start = [], None
    for i, row in enumerate(rows):
        engaged = row.get("mspOverrideFlags", "") == "2"
        if engaged and start is None:
            start = times[i]
        elif not engaged and start is not None:
            spans.append((start, times[i - 1]))
            start = None
    if start is not None:
        spans.append((start, times[-1]))
    return [s for s in spans if s[1] - s[0] > 2.0]


def complement(spans, t0, t1, min_len=8.0):
    """The MANUAL stretches between engagements — the same-air control group."""
    out, cursor = [], t0
    for a, b in spans:
        if a - cursor >= min_len:
            out.append((cursor, a))
        cursor = b
    if t1 - cursor >= min_len:
        out.append((cursor, t1))
    return out


def describe(label, rows, idx, fs):
    nseg = 512 if len(idx) >= 700 else 256
    print("== %s  (%.1f s, %d samples, nseg=%d, df=%.2f Hz)"
          % (label, len(idx) * (1 / fs), len(idx), nseg, fs / nseg))
    peaks = {}
    for name, key in CHANNELS:
        x = [num(rows[i], key) for i in idx]
        if any(math.isnan(v) for v in x):
            continue
        freqs, power = welch(x, fs, nseg)
        if not freqs:
            continue
        total = band_power(freqs, power, 0.0, fs / 2)
        peak = max(range(1, len(freqs)), key=lambda k: power[k])
        # Report the global peak, but ALSO carry the strongest bin inside the
        # oscillation band. Span 2 of 2026-09-13 is why: the vertical-estimator
        # blow-up dumps so much power below 0.5 Hz that the global peak lands
        # there, and a gain/phase computed at 0.23 Hz is meaningless.
        in_band = [k for k in range(len(freqs)) if OSC_LO <= freqs[k] < OSC_HI]
        peaks[key] = (freqs[peak],
                      freqs[max(in_band, key=lambda k: power[k])] if in_band else None)
        shares = " ".join("%g-%g:%4.1f%%" % (a, b, 100 * band_power(freqs, power, a, b) / total)
                          for a, b in BANDS)
        print("   %-12s peak %5.2f Hz | %s" % (name, freqs[peak], shares))
    return peaks


def rate_loop_gain(rows, idx, fs, f0, lo=OSC_LO, hi=OSC_HI):
    """Achieved/commanded pitch rate at the oscillation peak.

    gain > 1 here is the whole point: it says the closed rate loop AMPLIFIES at
    this frequency, against the ~0.5-0.7 delivered fraction measured at low
    frequency in flight-analysis.md §10.
    """
    gyro = [num(rows[i], "gyroADC[1]") for i in idx]
    setpt = [num(rows[i], "axisRate[1]") for i in idx]
    gb, sb = bandpass(gyro, fs, lo, hi), bandpass(setpt, fs, lo, hi)
    ga = math.sqrt(2 * st.mean([v * v for v in gb]))
    sa = math.sqrt(2 * st.mean([v * v for v in sb]))
    phase = math.degrees(cmath.phase(goertzel(gyro, fs, f0) / goertzel(setpt, fs, f0)))
    mg, ms = st.mean(gb), st.mean(sb)
    cov = sum((gb[i] - mg) * (sb[i] - ms) for i in range(len(gb)))
    den = math.sqrt(sum((v - mg) ** 2 for v in gb) * sum((v - ms) ** 2 for v in sb))
    # blackbox gyro/axisRate are deci-deg/s
    print("   @%.2f Hz: gyro %5.1f deg/s | setpoint %5.1f deg/s | GAIN %.2f | "
          "phase %+6.1f deg (%+.0f ms) | r %.2f | full-band gyro RMS %.1f deg/s"
          % (f0, ga / 10, sa / 10, (ga / sa) if sa else float("nan"),
             phase, 1000 * phase / 360 / f0, cov / den if den else float("nan"),
             st.pstdev(gyro) / 10))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="decoded blackbox CSV (blackbox_decode output)")
    ap.add_argument("--manual", action="store_true",
                    help="also spectra for the MANUAL gaps (same-air control group)")
    args = ap.parse_args()

    rows = load(args.csv)
    if "mspOverrideFlags" not in rows[0]:
        sys.exit("ERROR: no mspOverrideFlags column — engaged spans cannot be found")
    times = [num(r, "time (us)") / 1e6 for r in rows]
    steps = [times[i + 1] - times[i] for i in range(len(times) - 1)]
    fs = 1 / st.median(steps)
    dropped = ""
    if fs < 200:
        dropped = ("   ⚠️ %.1f Hz — blackbox_rate_denom is decimating. Nyquist %.1f Hz; "
                   "the 2-3 Hz band is safe but nothing above ~30 Hz survives." % (fs, fs / 2))
    print("%s\n   %d rows, %.1f s, sample rate %.1f Hz" % (args.csv, len(rows),
                                                           times[-1] - times[0], fs))
    if dropped:
        print(dropped)
    print()

    spans = engaged_spans(rows, times)
    segments = [("ENGAGED span %d" % i, s) for i, s in enumerate(spans, 1)]
    if args.manual:
        segments += [("MANUAL %.0f-%.0f s" % g, g)
                     for g in complement(spans, times[0], times[-1])]

    for label, (a, b) in segments:
        idx = [i for i in range(len(times)) if a <= times[i] <= b]
        if len(idx) < 256:
            print("== %s  (%.1f s) — too short for a spectrum\n" % (label, b - a))
            continue
        peaks = describe(label, rows, idx, fs)
        if label.startswith("ENGAGED") and peaks.get("gyroADC[1]", (None, None))[1]:
            rate_loop_gain(rows, idx, fs, peaks["gyroADC[1]"][1])
        print()


if __name__ == "__main__":
    main()
