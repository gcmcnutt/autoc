#!/usr/bin/env python3
r"""Xiao timestamp sampling-cadence distribution.

For a xiao flash log (column 2 = xiao millis() timestamp), compute deltas
between consecutive log lines and show the distribution. Confirms real
flight-time cadence against the nominal 100 ms NN tick.

Log line format:
    #NNNNNNNN <xiao_ms> <inav_ms> <level> <message>

Usage:
    python3 plot_xiao_cadence.py [input.txt] [output.png]

Default input: flight-results/flight-20260417/flight_log_2026-04-18T00-36-37.txt
Default output: specs/024-sim-real-fidelity/xiao_cadence_flight-20260417.png
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


DEFAULT_IN = Path("flight-results/flight-20260417/flight_log_2026-04-18T00-36-37.txt")
DEFAULT_OUT = Path("specs/024-sim-real-fidelity/xiao_cadence_flight-20260417.png")


def load(path: Path):
    """Return dict {category: ndarray of xiao_ms timestamps (int)} for
    categories: 'all', 'Nav State', 'NN'. Skips the armed boot line."""
    all_ts, nav_ts, nn_ts = [], [], []
    for line in path.read_text().splitlines():
        if not line or not line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 5:
            continue
        try:
            xiao_ms = int(parts[1])
        except ValueError:
            continue
        # parts[3] is log level, parts[4] is first message token
        msg_first = parts[4]
        msg_second = parts[5] if len(parts) > 5 else ""
        if msg_first == "Nav" and msg_second == "State:":
            nav_ts.append(xiao_ms)
            all_ts.append(xiao_ms)
        elif msg_first == "NN:" or (msg_first == "NN" and msg_second != "eval"):
            nn_ts.append(xiao_ms)
            all_ts.append(xiao_ms)
        elif msg_first == "Flash":
            # boot line — skip from cadence analysis
            continue
        else:
            all_ts.append(xiao_ms)
    return {
        "all": np.asarray(all_ts, dtype=np.int64),
        "Nav State": np.asarray(nav_ts, dtype=np.int64),
        "NN": np.asarray(nn_ts, dtype=np.int64),
    }


def stats(deltas: np.ndarray):
    if deltas.size == 0:
        return {}
    return {
        "n": int(deltas.size),
        "min": int(deltas.min()),
        "max": int(deltas.max()),
        "mean": float(deltas.mean()),
        "median": int(np.median(deltas)),
        "p01": int(np.percentile(deltas, 1)),
        "p05": int(np.percentile(deltas, 5)),
        "p95": int(np.percentile(deltas, 95)),
        "p99": int(np.percentile(deltas, 99)),
        "std": float(deltas.std()),
    }


def main():
    inp = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_IN
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_OUT

    series = load(inp)
    deltas_by = {k: np.diff(v) for k, v in series.items() if v.size > 1}

    print(f"Source: {inp}")
    for cat, d in deltas_by.items():
        s = stats(d)
        print(f"\n[{cat}] n_deltas={s['n']}  "
              f"median={s['median']}ms  mean={s['mean']:.2f}ms  std={s['std']:.2f}ms")
        print(f"  range=[{s['min']}, {s['max']}]ms  "
              f"p01={s['p01']}  p05={s['p05']}  p95={s['p95']}  p99={s['p99']}")

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Top-left: all log lines histogram
    d_all = deltas_by["all"]
    ax = axes[0, 0]
    # tight-binned histogram around the expected modes
    bins = np.arange(max(0, d_all.min() - 1), min(d_all.max() + 2, 300), 1)
    ax.hist(d_all, bins=bins, color="tab:blue", alpha=0.85,
            edgecolor="tab:blue")
    s = stats(d_all)
    ax.axvline(s["median"], color="red", linestyle="--", linewidth=1,
               label=f"median = {s['median']} ms")
    ax.set_title(f"All log lines — Δ xiao_ms (n={s['n']})")
    ax.set_xlabel("Delta (ms)")
    ax.set_ylabel("Count")
    ax.set_yscale("log")
    ax.grid(True, alpha=0.4, which="both")
    ax.legend()

    # Top-right: Nav State only (the 10 Hz NN tick stream)
    d_nav = deltas_by["Nav State"]
    ax = axes[0, 1]
    bins = np.arange(max(0, d_nav.min() - 1), min(d_nav.max() + 2, 300), 1)
    ax.hist(d_nav, bins=bins, color="tab:green", alpha=0.85,
            edgecolor="tab:green")
    s = stats(d_nav)
    ax.axvline(100, color="black", linestyle=":", linewidth=1,
               alpha=0.6, label="nominal 100 ms")
    ax.axvline(s["median"], color="red", linestyle="--", linewidth=1,
               label=f"median = {s['median']} ms")
    ax.set_title(f"Nav State (NN tick cadence) — Δ xiao_ms (n={s['n']})")
    ax.set_xlabel("Delta (ms)")
    ax.set_ylabel("Count")
    ax.set_yscale("log")
    ax.grid(True, alpha=0.4, which="both")
    ax.legend()

    # Bottom-left: Nav State delta over time (timeline)
    ax = axes[1, 0]
    ts_nav_sec = (series["Nav State"][1:] - series["Nav State"][0]) / 1000.0
    ax.plot(ts_nav_sec, d_nav, linewidth=0.6, color="tab:green", alpha=0.85)
    ax.axhline(100, color="black", linestyle=":", linewidth=0.8, alpha=0.7,
               label="nominal 100 ms")
    ax.set_title("Nav State cadence over flight time")
    ax.set_xlabel("Seconds since first Nav State")
    ax.set_ylabel("Δ xiao_ms (ms)")
    ax.grid(True, alpha=0.4)
    ax.legend()
    # zoom y-axis to see fine variation unless big spikes exist
    s = stats(d_nav)
    ax.set_ylim(max(0, s["p01"] - 20), min(s["max"] + 10, s["p99"] + 50))

    # Bottom-right: CDF for Nav State
    ax = axes[1, 1]
    sorted_d = np.sort(d_nav)
    cdf = np.arange(1, len(sorted_d) + 1) / len(sorted_d)
    ax.plot(sorted_d, cdf, color="tab:green", linewidth=1.5)
    ax.axvline(100, color="black", linestyle=":", linewidth=0.8,
               alpha=0.7, label="nominal 100 ms")
    for p_key, p_val in (("p05", 0.05), ("p50", 0.50), ("p95", 0.95)):
        q = np.quantile(d_nav, p_val)
        ax.axhline(p_val, color="gray", linewidth=0.4, alpha=0.5)
        ax.text(q, p_val, f" {p_key}={int(q)} ms", fontsize=9, va="bottom")
    ax.set_title("Nav State Δ CDF")
    ax.set_xlabel("Delta (ms)")
    ax.set_ylabel("Cumulative fraction")
    ax.grid(True, alpha=0.4)
    ax.legend()

    fig.suptitle(
        f"Xiao timestamp cadence — {inp.name}\n"
        f"Nominal: 100 ms NN tick (MSP_LOOP_INTERVAL_MSEC=50 × MSP_NN_EVAL_DIVISOR=2)",
        fontsize=11,
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
