#!/usr/bin/env python3
"""Flight-side bang-bang / control-effort visualization per engage span.

Sibling to specs/024-sim-real-fidelity/plot_control_aggressiveness.py
(which operates on training data.dat). This one parses xiao NN log
lines — one PNG per engage span, in the style of the join_analysis
per-span artifacts.

Per-span panels:
  1. NN output time series (outPt, outRl, outTh) — shows bang-bang shape
  2. |Δout| per tick time series — "stick speed"
  3. |out| per tick time series with amplitude-ceiling reference
  4. Histogram of each output's value (shows extreme-saturation incidence)

Usage:
    python3 plot_bangbang_flight.py <xiao_log> [out-dir]

    Default out-dir = same dir as the input log.
"""

import re
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SWITCH_ENABLED_RE = re.compile(r"Switch enabled|autoc enable|Rabbit start", re.I)
SWITCH_DISABLED_RE = re.compile(r"Switch disabled|Autoc disabled|Rabbit (complete|stopped|END)", re.I)
PATH_RE = re.compile(r"\bpath=(\d+)")
NN_RE = re.compile(
    r"NN:\s+idx=\d+.*?out=\[([-0-9.]+),([-0-9.]+),([-0-9.]+)\]"
)


def parse_spans(path):
    """Yield (span_idx, path_idx, rows) where rows = [(xiao_ms, pt, rl, th)]."""
    spans = []
    in_span = False
    cur_rows = []
    cur_path = -1
    span_idx = 0
    # The xiao log's span boundary is the "Nav Control: Switch enabled" -type
    # line; NN evals stream inside. "Autoc disabled / Switch disabled" closes.
    # We also accept a span implicitly: any NN line outside a marked span
    # gets included in the next open one.
    for line in path.read_text().splitlines():
        if not line.startswith("#"):
            continue
        parts = line.split(maxsplit=4)
        if len(parts) < 5:
            continue
        xiao_ms = int(parts[1])
        rest = parts[4]

        if SWITCH_DISABLED_RE.search(rest) and in_span:
            in_span = False
            if cur_rows:
                spans.append((span_idx, cur_path, cur_rows))
                span_idx += 1
            cur_rows = []
            cur_path = -1
            continue

        # Span-start: "NN Control: Switch enabled" (xiao emits this at engage).
        # Also accept "Rabbit start" / "Nav Control: ... enabled" for robustness.
        if (
            "Switch enabled" in rest
            or "rabbit start" in rest.lower()
        ):
            # Start a fresh span; flush any prior open one defensively.
            if cur_rows:
                spans.append((span_idx, cur_path, cur_rows))
                span_idx += 1
            cur_rows = []
            cur_path = -1
            in_span = True
            continue

        m = NN_RE.search(rest)
        if m:
            # Open a span lazily on the first NN eval if none was started
            # (handles logs where the "enabled" line format differs).
            if not in_span:
                in_span = True
            pt = float(m.group(1))
            rl = float(m.group(2))
            th = float(m.group(3))
            cur_rows.append((xiao_ms, pt, rl, th))
            continue

        pm = PATH_RE.search(rest)
        if pm and in_span:
            cur_path = int(pm.group(1))

    # Flush any still-open span at EOF
    if cur_rows:
        spans.append((span_idx, cur_path, cur_rows))
    return spans


def plot_span(span_idx, path_idx, rows, out_dir, log_name):
    xiao_ms = np.array([r[0] for r in rows], dtype=np.int64)
    t_s = (xiao_ms - xiao_ms[0]) / 1000.0
    pt = np.array([r[1] for r in rows])
    rl = np.array([r[2] for r in rows])
    th = np.array([r[3] for r in rows])

    dPt = np.abs(np.diff(pt))
    dRl = np.abs(np.diff(rl))
    dTh = np.abs(np.diff(th))
    dctrl = dPt + dRl + dTh  # per-tick total change
    mag = np.abs(pt) + np.abs(rl) + np.abs(th)

    dur_s = t_s[-1] - t_s[0]

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))

    # Panel 1: NN output time series
    ax = axes[0, 0]
    ax.plot(t_s, pt, linewidth=0.7, color="tab:blue", label="outPt")
    ax.plot(t_s, rl, linewidth=0.7, color="tab:orange", label="outRl")
    ax.plot(t_s, th, linewidth=0.7, color="tab:green", label="outTh")
    ax.axhline(0, color="gray", linewidth=0.4, alpha=0.5)
    ax.axhline(+1, color="red", linewidth=0.3, linestyle="--", alpha=0.4)
    ax.axhline(-1, color="red", linewidth=0.3, linestyle="--", alpha=0.4)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("NN output [-1, +1]")
    ax.set_title(f"NN outputs over span (n={len(rows)})")
    ax.set_ylim(-1.2, 1.2)
    ax.grid(True, linewidth=0.5, alpha=0.4)
    ax.legend(loc="upper left", fontsize=8)

    # Panel 2: |Δout| per tick (stick speed)
    ax = axes[0, 1]
    ax.plot(t_s[1:], dctrl, linewidth=0.7, color="tab:red", alpha=0.75,
            label="|ΔoutPt|+|ΔoutRl|+|ΔoutTh|")
    dctrl_mean = dctrl.mean()
    ax.axhline(dctrl_mean, color="tab:red", linewidth=1.2, linestyle="--",
               alpha=0.6, label=f"mean = {dctrl_mean:.2f}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("|Δout| per tick")
    ax.set_title("Stick-speed time series (bang-bang chatter)")
    ax.set_ylim(0, 3.2)
    ax.grid(True, linewidth=0.5, alpha=0.4)
    ax.legend(loc="upper left", fontsize=8)

    # Panel 3: |out| per tick (amplitude) with ceiling
    ax = axes[1, 0]
    ax.plot(t_s, mag, linewidth=0.7, color="tab:purple", alpha=0.75,
            label="|outPt|+|outRl|+|outTh|")
    mag_mean = mag.mean()
    ax.axhline(mag_mean, color="tab:purple", linewidth=1.2, linestyle="--",
               alpha=0.6, label=f"mean = {mag_mean:.2f}")
    ax.axhline(3.0, color="gray", linewidth=0.6, linestyle=":", alpha=0.7,
               label="ceiling = 3.0")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("|out| per tick")
    ax.set_title("Stick-amplitude time series")
    ax.set_ylim(0, 3.2)
    ax.grid(True, linewidth=0.5, alpha=0.4)
    ax.legend(loc="upper left", fontsize=8)

    # Panel 4: per-axis distributions
    ax = axes[1, 1]
    bins = np.linspace(-1.05, 1.05, 43)
    ax.hist(pt, bins=bins, alpha=0.55, color="tab:blue", label=f"outPt  <|·|>={np.mean(np.abs(pt)):.2f}")
    ax.hist(rl, bins=bins, alpha=0.55, color="tab:orange", label=f"outRl  <|·|>={np.mean(np.abs(rl)):.2f}")
    ax.hist(th, bins=bins, alpha=0.55, color="tab:green", label=f"outTh  <|·|>={np.mean(np.abs(th)):.2f}")
    ax.set_xlabel("NN output value")
    ax.set_ylabel("Count")
    ax.set_title("Output distribution (saturation at ±1 = bang-bang)")
    ax.grid(True, linewidth=0.5, alpha=0.4)
    ax.legend(loc="upper center", fontsize=8)

    fig.suptitle(
        f"{log_name} — span {span_idx+1}  path={path_idx}  duration={dur_s:.1f}s\n"
        f"mean dCtrl={dctrl_mean:.2f}/tick  mean |out|={mag_mean:.2f}/tick",
        fontsize=11,
    )
    fig.tight_layout()

    out = out_dir / f"bangbang_flight_{log_name}_span{span_idx+1}_path{path_idx}.png"
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"  wrote {out}")


def main():
    if len(sys.argv) < 2:
        print("Usage: plot_bangbang_flight.py <xiao_log> [out-dir]", file=sys.stderr)
        sys.exit(2)
    log = Path(sys.argv[1])
    out_dir = Path(sys.argv[2]) if len(sys.argv) > 2 else log.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    log_stem = log.stem

    spans = parse_spans(log)
    print(f"{log}: parsed {len(spans)} spans")
    for span_idx, path_idx, rows in spans:
        if not rows:
            continue
        print(f"span {span_idx+1}  path={path_idx}  n={len(rows)} NN evals")
        plot_span(span_idx, path_idx, rows, out_dir, log_stem)


if __name__ == "__main__":
    main()
