#!/usr/bin/env python3
"""043 — propeller order analysis from a decoded INAV blackbox CSV.

Written for `vibration-analysis-20261004.md` (2026-10-04 outdoor bench run,
`flight-results/flight-20261004/blackbox_log_2026-10-04_141959.TXT`).

Answers one question: is the throttle-tracking gyro line the SHAFT order (1/rev,
i.e. imbalance) or the BLADE-PASS order (2/rev on a 2-blade prop)? The method is
to take the dominant line `F` — which is unambiguous — and then measure the
amplitude at `F/2`, `1.5F` and `2F`. `1.5F` is a non-harmonic control: if
`A(F/2)` is not well above it, there is no subharmonic.

⛔ Do NOT "fix" this by searching for `f0` with a harmonic-sum score. It was
tried and is worse: a lone strong line at `F` scores better as `f0 = F` than as
`f0 = F/2`, so it reports blade-pass AS the shaft order. See the doc § 2.

⛔ STDLIB ONLY, same rule as `pitch_spectrum.py` — there is no numpy on the WSL
bench host, and this must run wherever the logs are. FFT and single-bin DFT are
imported from `pitch_spectrum` rather than duplicated.

⚠️ Sample rate MUST come from (n-1)/(t[-1]-t[0]), not median-dt. Median-dt
overstates fs by 1.09 % on this log, which is a ~4 Hz systematic error on every
reported frequency.

⚠️ Reads `gyroRaw[*]` — the PRE-FILTER tap. `gyroADC` is useless for vibration
work because `gyro_main_lpf_hz = 25` removes everything of interest. Requires
`blackbox GYRO_RAW` to have been enabled when the log was recorded.

⚠️ Amplitudes above ~256 Hz are understated: INAV hardcodes the MPU6000/6500
hardware DLPF to 256 Hz and exposes no setting for it. Frequencies are right.

Usage:
    ./vibration_order.py <decoded.csv> [--axis pitch] [--nps 2048] [--min-motor 1150]
"""

import argparse
import math
import sys

from pitch_spectrum import fft, goertzel, load, num

AXES = {"roll": 0, "pitch": 1, "yaw": 2}


def hann_power(seg):
    """Hann-windowed power spectrum of one segment (mean removed)."""
    n = len(seg)
    mean = sum(seg) / n
    win = [0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1)) for i in range(n)]
    spec = fft([complex((seg[i] - mean) * win[i], 0.0) for i in range(n)])
    return [abs(c) ** 2 for c in spec[: n // 2 + 1]], win


def amp_at(seg, fs, f0):
    """RMS-consistent amplitude at f0 via single-bin Hann DFT."""
    n = len(seg)
    if f0 <= 0 or f0 >= fs / 2:
        return 0.0
    win = [0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1)) for i in range(n)]
    corr = math.sqrt(sum(w * w for w in win) / n)
    return abs(goertzel(seg, fs, f0)) * 2.0 / n / corr


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--axis", default="pitch", choices=sorted(AXES))
    ap.add_argument("--nps", type=int, default=2048, help="FFT length (power of 2)")
    ap.add_argument("--min-motor", type=float, default=1150.0)
    args = ap.parse_args()

    if args.nps & (args.nps - 1):
        sys.exit("--nps must be a power of 2 (radix-2 FFT)")

    rows = load(args.csv)
    gkey = "gyroRaw[%d]" % AXES[args.axis]
    if gkey not in rows[0]:
        sys.exit("%s not in CSV — was `blackbox GYRO_RAW` enabled?" % gkey)

    t0 = num(rows[0], "time (us)") / 1e6
    t = [num(r, "time (us)") / 1e6 - t0 for r in rows]   # relative to log start
    g = [num(r, gkey) for r in rows]
    mot = [num(r, "motor[0]") for r in rows]

    fs = (len(t) - 1) / (t[-1] - t[0])          # NOT median-dt; see module docstring
    nps = args.nps
    print("%s: n=%d span=%.3f s  fs=%.2f Hz  Nyquist=%.1f Hz  df=%.2f Hz"
          % (args.csv, len(t), t[-1] - t[0], fs, fs / 2, fs / nps))
    print("axis=%s (gyroRaw, pre-filter)\n" % args.axis)
    print("  t(s) motor    F(Hz)  A(F/2)   A(F) A(1.5F)  A(2F) | shaft RPM if F=2/rev")

    acc = []
    for s in range(0, len(g) - nps, nps // 4):
        seg = g[s:s + nps]
        mseg = mot[s:s + nps]
        mbar = sum(mseg) / len(mseg)
        if mbar < args.min_motor:
            continue
        power, _ = hann_power(seg)
        # dominant line above 90 Hz, below Nyquist margin
        lo = int(90 * nps / fs)
        hi = int((fs / 2 - 20) * nps / fs)
        kmax = max(range(lo, min(hi, len(power))), key=lambda k: power[k])
        F = kmax * fs / nps
        a = [amp_at(seg, fs, F * m) for m in (0.5, 1.0, 1.5, 2.0)]
        acc.append((mbar, F, a))
        print("  %5.1f %5.0f  %7.1f  %6.2f %6.2f  %6.2f %6.2f | %7.0f"
              % (t[s], mbar, F, a[0], a[1], a[2], a[3], F / 2 * 60))

    if not acc:
        sys.exit("no windows above --min-motor")

    n = len(acc)
    mb = [x[0] for x in acc]
    fb = [x[1] for x in acc]
    mm, mf = sum(mb) / n, sum(fb) / n
    cov = sum((mb[i] - mm) * (fb[i] - mf) for i in range(n))
    den = math.sqrt(sum((v - mm) ** 2 for v in mb) * sum((v - mf) ** 2 for v in fb))
    ratios = sorted(x[2][0] / x[2][1] for x in acc if x[2][1] > 0)

    print("\nn=%d windows" % n)
    print("F vs motor command:  r = %+.4f" % (cov / den if den else float("nan")))
    print("A(F/2)/A(F): mean=%.3f median=%.3f" % (sum(ratios) / len(ratios),
                                                  ratios[len(ratios) // 2]))
    print("A(1.5F)/A(F) control: mean=%.3f"
          % (sum(x[2][2] / x[2][1] for x in acc if x[2][1] > 0) / n))
    print("A(2F)/A(F): mean=%.3f"
          % (sum(x[2][3] / x[2][1] for x in acc if x[2][1] > 0) / n))
    wot = [x for x in acc if x[0] > 1880]
    if wot:
        k = len(wot)
        print("motor>1880 (n=%d): F=%.1f Hz  F/2=%.1f Hz = %.0f RPM  A(F)=%.2f  A(F/2)=%.2f"
              % (k, sum(x[1] for x in wot) / k, sum(x[1] for x in wot) / k / 2,
                 sum(x[1] for x in wot) / k / 2 * 60,
                 sum(x[2][1] for x in wot) / k, sum(x[2][0] for x in wot) / k))

    print("\nIf A(F/2)/A(F) is well above the 1.5F control, F is the 2/rev")
    print("blade-pass line and F/2 is the 1/rev shaft (imbalance) line.")


if __name__ == "__main__":
    main()
