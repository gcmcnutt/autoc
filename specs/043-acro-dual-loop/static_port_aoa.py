#!/usr/bin/env python3
"""043 — static-port error vs autoc engagement, from decoded INAV blackbox CSVs.

Produces the tables in `vibration-analysis-20261004.md` § 6. Three tests:

  A  paired within-flight: baro residual RMS and |pitch rate|, engaged vs not.
     Same flight => same port, same weather, same day. This is the clean comparison.
  B  cross-correlation of baro residual against normal-accel residual over +/-0.5 s.
     Answers "does the pressure error LEAD the load?" (it does not -- peak is lag 0).
  C  the assumption-free one: between consecutive BaroAlt UPDATES, how far does the
     baro move vs the most it physically could at |navVel[2]| * dt? Engaged, that
     ratio is 5-15x. Unengaged it is ~1. No model, no integration, no filter.

Test C is the load-bearing one. A and B are supporting: A cannot separate a port
artifact from real manoeuvring (autoc really does climb and dive hard), and B cannot
either, because a real-motion oscillation and an AOA-driven pressure error BOTH sit
at lag 0. Only the magnitude argument in C settles it.

⛔ STDLIB ONLY -- deliberately, same rule as `pitch_spectrum.py`: there is no
numpy/scipy on the WSL bench host and this must run wherever the flight logs are.

⚠️ BaroAlt updates at only ~27 Hz however fast the log runs (median hold of 2 samples
at 59 Hz). Test C therefore steps on baro CHANGES, not log rows -- stepping per row
would divide every delta by the 2x oversampling and halve the answer.

⚠️ Engagement is `mspOverrideFlags == 2`, per `flight-analysis-20260913.md` § 0.
Logs without it (or without navVel[2]) are skipped rather than guessed at.

Usage:
    blackbox_decode --index 1 flight-results/flight-YYYYMMDD/blackbox_log_*.TXT
    ./static_port_aoa.py flight-results/flight-*/blackbox_log_*.01.csv

⚠️ Multi-session .TXT files need every --index, not just 1: see
`vibration-analysis-20261004.md` § 5.2's data-integrity note.
"""
import csv
import math
import os
import sys

ACC_1G = 2048.0          # acc_1G for MPU6000/6500 at the hardcoded +/-16 g FSR
RESID_WINDOW_S = 0.5     # moving-mean width for the baro/accel high-pass
MAX_LAG_S = 0.5


def highpass(x, width):
    """x minus its centred moving mean. Cheap, and flat enough above 1/width."""
    n = len(x)
    prefix = [0.0] * (n + 1)
    for i, v in enumerate(x):
        prefix[i + 1] = prefix[i] + v
    half = width // 2
    out = [0.0] * n
    for i in range(n):
        a, b = max(0, i - half), min(n, i + half + 1)
        out[i] = x[i] - (prefix[b] - prefix[a]) / (b - a)
    return out


def pearson(a, b):
    n = len(a)
    if n < 30:
        return float("nan")
    ma, mb = sum(a) / n, sum(b) / n
    va = sum((v - ma) ** 2 for v in a)
    vb = sum((v - mb) ** 2 for v in b)
    if va <= 0 or vb <= 0:
        return float("nan")
    return sum((a[i] - ma) * (b[i] - mb) for i in range(n)) / math.sqrt(va * vb)


def quantile(v, q):
    if not v:
        return float("nan")
    s = sorted(v)
    return s[min(len(s) - 1, int(q * len(s)))]


def rms(v):
    return math.sqrt(sum(y * y for y in v) / len(v)) if v else float("nan")


FIELDS = ("time (us)", "BaroAlt (cm)", "mspOverrideFlags",
          "accSmooth[2]", "gyroADC[1]", "navVel[2]")


def load(path):
    with open(path, newline="") as fh:
        reader = csv.reader(fh)
        index = {h.strip(): i for i, h in enumerate(next(reader))}
        missing = [f for f in FIELDS if f not in index]
        if missing:
            return None, missing
        cols = {k: [] for k in ("t", "baro", "eng", "accz", "pitch", "vz")}
        for row in reader:
            try:
                t = float(row[index["time (us)"]]) / 1e6
                baro = float(row[index["BaroAlt (cm)"]])
                accz = float(row[index["accSmooth[2]"]]) / ACC_1G
                pitch = float(row[index["gyroADC[1]"]])
                vz = abs(float(row[index["navVel[2]"]]))
            except (ValueError, IndexError):
                continue
            cols["t"].append(t)
            cols["baro"].append(baro)
            cols["accz"].append(accz)
            cols["pitch"].append(pitch)
            cols["vz"].append(vz)
            cols["eng"].append(row[index["mspOverrideFlags"]].strip() == "2")
        return cols, None


def analyse(path):
    cols, missing = load(path)
    name = os.path.basename(path)
    if cols is None:
        return name, None, "missing %s" % ",".join(missing)
    t = cols["t"]
    if len(t) < 800:
        return name, None, "too short (%d rows)" % len(t)
    fs = (len(t) - 1) / (t[-1] - t[0])
    width = max(5, int(RESID_WINDOW_S * fs))
    resid_b = highpass(cols["baro"], width)
    resid_a = highpass(cols["accz"], width)
    eng = cols["eng"]

    # --- A: paired engaged vs unengaged -------------------------------------
    be = [resid_b[i] for i in range(len(resid_b)) if eng[i]]
    bu = [resid_b[i] for i in range(len(resid_b)) if not eng[i]]
    pe = [abs(cols["pitch"][i]) for i in range(len(t)) if eng[i]]
    pu = [abs(cols["pitch"][i]) for i in range(len(t)) if not eng[i]]
    if len(be) < 200 or len(bu) < 200:
        return name, None, "no usable engaged/unengaged split"

    # --- B: lead/lag --------------------------------------------------------
    max_lag = int(MAX_LAG_S * fs)
    best_lag, best_r = 0, 0.0
    for lag in range(-max_lag, max_lag + 1):
        a, b = [], []
        for i in range(len(resid_b)):
            j = i + lag
            if 0 <= j < len(resid_a) and eng[i] and eng[j]:
                a.append(resid_b[i])
                b.append(resid_a[j])
        r = pearson(a, b)
        if r == r and abs(r) > abs(best_r):
            best_lag, best_r = lag, r

    # --- C: baro step vs physical bound, stepping on baro CHANGES only ------
    step_e, step_u, bound_e = [], [], []
    prev_b = prev_t = None
    for i in range(len(t)):
        b = cols["baro"][i]
        if prev_b is not None and b != prev_b:
            (step_e if eng[i] else step_u).append(abs(b - prev_b))
            if eng[i]:
                bound_e.append(cols["vz"][i] * (t[i] - prev_t))
        if prev_b is None or b != prev_b:
            prev_b, prev_t = b, t[i]

    return name, dict(
        fs=fs,
        resid_eng=rms(be), resid_un=rms(bu),
        pitch_eng=sum(pe) / len(pe), pitch_un=sum(pu) / len(pu),
        lag_ms=best_lag * 1000.0 / fs, corr=best_r,
        step_e95=quantile(step_e, 0.95), step_emax=max(step_e) if step_e else float("nan"),
        bound95=quantile(bound_e, 0.95),
        step_u95=quantile(step_u, 0.95),
        acc_p99=quantile([abs(v) for v in cols["accz"]], 0.99),
        acc_max=max(abs(v) for v in cols["accz"]),
    ), None


def main(paths):
    rows = []
    for p in paths:
        name, out, err = analyse(p)
        if out is None:
            print("skip %-44s %s" % (name, err), file=sys.stderr)
            continue
        rows.append((name, out))
    if not rows:
        print("no usable logs", file=sys.stderr)
        return 1

    print("A) baro residual + pitch activity, engaged vs unengaged (paired within flight)")
    print("%-40s %9s %8s %6s | %9s %8s %6s" % (
        "log", "resid E", "resid U", "x", "|pitch| E", "|pitch| U", "x"))
    for name, o in rows:
        print("%-40s %9.1f %8.1f %6.2f | %9.1f %8.1f %6.2f" % (
            name, o["resid_eng"], o["resid_un"], o["resid_eng"] / o["resid_un"],
            o["pitch_eng"], o["pitch_un"], o["pitch_eng"] / o["pitch_un"]))

    print("\nB) baro residual vs normal-accel residual, engaged. negative lag = baro LEADS.")
    print("%-40s %12s %10s" % ("log", "best lag ms", "r"))
    for name, o in rows:
        print("%-40s %12.0f %10.3f" % (name, o["lag_ms"], o["corr"]))

    print("\nC) baro step per UPDATE vs physical bound (|navVel[2]|*dt). cm.")
    print("%-40s %8s %8s %8s %7s | %8s" % (
        "log", "eng p95", "eng max", "bound95", "ratio", "unen p95"))
    for name, o in rows:
        ratio = o["step_e95"] / o["bound95"] if o["bound95"] else float("nan")
        print("%-40s %8.0f %8.0f %8.0f %7.1f | %8.0f" % (
            name, o["step_e95"], o["step_emax"], o["bound95"], ratio, o["step_u95"]))

    print("\nD) normal-load envelope, |accSmooth[2]| in g (15 Hz-filtered; rail is 15.9)")
    print("%-40s %8s %8s" % ("log", "p99", "max"))
    for name, o in rows:
        print("%-40s %8.2f %8.2f" % (name, o["acc_p99"], o["acc_max"]))
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1:]))
