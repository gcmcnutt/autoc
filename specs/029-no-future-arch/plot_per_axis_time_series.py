#!/usr/bin/env python3
"""028 per-axis aggressiveness time-series — pt/rl/th evolution across gens.

Companion to specs/024-sim-real-fidelity/plot_control_aggressiveness.py.
That script aggregates pt+rl+th into a single dCtrl/<|out|> per path-gen.
This script breaks them out: 3 lines per panel (one per control axis), one
trace each for the population's mean over all included path/wind scenarios.

Why per-axis: bang-bang lives on a single axis (roll for the RNN
architecture, pitch for FF). The per-axis slope tells you which axis is
refining and which is stuck. From more-rnn3 mid-run: amplitude trends
down on all 3 axes slightly, but dCtrl plateaus on roll while throttle
*creeps up*. Aggregate hides both signals.

Usage:
    python3 plot_per_axis_time_series.py
        [--in data.dat] [--label NAME] [--total-gens N]
        [--pop-size N] [--out PNG]
"""

import argparse
import math
from collections import defaultdict
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def quat_to_roll_pitch_deg(qw, qx, qy, qz):
    """Body-to-world Hamilton quat → (roll_deg, pitch_deg) in aerospace
    Tait-Bryan ZYX convention (FRD body frame, NED world frame).

    roll  φ around body X (right-wing-down positive)
    pitch θ around body Y (nose-up positive)
    """
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return math.degrees(roll), math.degrees(pitch)


def unwrap_deg(d):
    """Wrap a delta-angle into (-180, +180] so consecutive-tick differences
    don't blow up across the ±180° roll discontinuity."""
    while d > 180.0:
        d -= 360.0
    while d <= -180.0:
        d += 360.0
    return d


def parse_pth_wnd(field: str):
    pw, _step, _ = field.split(":")
    path, wind = pw.split("/")
    return int(path), int(wind)


def load(path: Path, pop_size: int):
    """Stream rows; return (per_gen, per_gen_per_path).

    per_gen[gen] = (dpt, drl, dth, mpt, mrl, mth) means — population-wide
                   mean |Δout| per tick and mean |out| per tick, per axis.

    per_gen_per_path[gen][path_idx] = (mean_droll_rate_dps, mean_dpitch_rate_dps)
                   — per-scenario airframe rotation rate in DEG/SEC (computed
                   as scenario's total |Δroll_angle| / scenario's duration in
                   seconds, where duration = ticks × DT_SEC), averaged across
                   all wind variants for that path. Using rate instead of raw
                   total normalizes for variable scenario duration — early-gen
                   scenarios that crash early no longer look "low-roll" just
                   because they had less time to accumulate degrees.

    Streams to keep peak memory bounded.
    """
    DT_SEC = 0.1  # SIM_TIME_STEP_MSEC = 100ms — autoc tick rate
    div = pop_size + 1
    sums = defaultdict(lambda: {"dpt": 0.0, "drl": 0.0, "dth": 0.0,
                                "mpt": 0.0, "mrl": 0.0, "mth": 0.0,
                                "n": 0})
    # Per (gen, path_idx): accumulate per-scenario rates (deg/sec) + count
    path_sums = defaultdict(lambda: defaultdict(
        lambda: {"sum_droll_rate": 0.0, "sum_dpitch_rate": 0.0, "n_scen": 0}))
    cols = {}
    header = None
    last_key = None  # (path_idx, wind_idx, scn) — uniquely identifies one trajectory
    last_gen = None
    last_path = None
    last_pt = last_rl = last_th = None
    last_roll_deg = last_pitch_deg = None
    cur_traj_droll_deg = 0.0
    cur_traj_dpitch_deg = 0.0
    cur_traj_ticks = 0

    def flush_trajectory():
        nonlocal cur_traj_droll_deg, cur_traj_dpitch_deg, cur_traj_ticks
        # Need at least 2 ticks to compute any delta (and avoid div-by-zero)
        if (last_gen is not None and last_path is not None
                and cur_traj_ticks >= 2):
            duration_sec = cur_traj_ticks * DT_SEC
            ps = path_sums[last_gen][last_path]
            ps["sum_droll_rate"] += cur_traj_droll_deg / duration_sec
            ps["sum_dpitch_rate"] += cur_traj_dpitch_deg / duration_sec
            ps["n_scen"] += 1
        cur_traj_droll_deg = 0.0
        cur_traj_dpitch_deg = 0.0
        cur_traj_ticks = 0

    with open(path) as f:
        for raw in f:
            parts = raw.split()
            if header is None:
                header = parts
                cols["scn"] = header.index("Scn")
                cols["pw"] = header.index("Pth/Wnd:Step:")
                cols["pt"] = header.index("outPt")
                cols["rl"] = header.index("outRl")
                cols["th"] = header.index("outTh")
                cols["qw"] = header.index("qw")
                cols["qx"] = header.index("qx")
                cols["qy"] = header.index("qy")
                cols["qz"] = header.index("qz")
                continue
            if len(parts) <= max(cols.values()):
                continue
            try:
                scn = int(parts[cols["scn"]])
                gen = scn // div
                path_idx, wind_idx = parse_pth_wnd(parts[cols["pw"]])
                pt = float(parts[cols["pt"]])
                rl = float(parts[cols["rl"]])
                th = float(parts[cols["th"]])
                qw = float(parts[cols["qw"]])
                qx = float(parts[cols["qx"]])
                qy = float(parts[cols["qy"]])
                qz = float(parts[cols["qz"]])
            except (ValueError, IndexError):
                continue
            roll_deg, pitch_deg = quat_to_roll_pitch_deg(qw, qx, qy, qz)
            # Trajectory key — must include wind_idx to keep each
            # (path × wind × gen) scenario as its own trajectory; using
            # (path, scn) alone would lump all 49 winds for a given path
            # into one "trajectory" and double-count cross-scenario boundaries.
            cur_key = (path_idx, wind_idx, scn)
            if last_key is not None and last_key != cur_key:
                # Trajectory boundary — flush prior scenario
                flush_trajectory()
                # Reset per-tick prev state so we don't compute a delta across
                # the boundary
                last_pt = last_rl = last_th = None
                last_roll_deg = last_pitch_deg = None
            s = sums[gen]
            s["mpt"] += abs(pt); s["mrl"] += abs(rl); s["mth"] += abs(th)
            s["n"] += 1
            cur_traj_ticks += 1
            if last_pt is not None:
                dpt = abs(pt - last_pt)
                drl = abs(rl - last_rl)
                dth = abs(th - last_th)
                s["dpt"] += dpt
                s["drl"] += drl
                s["dth"] += dth
            if last_roll_deg is not None:
                droll = unwrap_deg(roll_deg - last_roll_deg)
                dpitch = pitch_deg - last_pitch_deg
                cur_traj_droll_deg += abs(droll)
                cur_traj_dpitch_deg += abs(dpitch)
            last_key = cur_key
            last_gen = gen
            last_path = path_idx
            last_pt, last_rl, last_th = pt, rl, th
            last_roll_deg, last_pitch_deg = roll_deg, pitch_deg

    # Flush the final trajectory
    flush_trajectory()

    # Means per gen
    out = {}
    for gen, s in sums.items():
        n = s["n"]
        if n < 2:
            continue
        out[gen] = (s["dpt"] / max(n - 1, 1), s["drl"] / max(n - 1, 1),
                    s["dth"] / max(n - 1, 1),
                    s["mpt"] / n, s["mrl"] / n, s["mth"] / n)

    # Per-gen per-path mean rate over scenarios (in deg/sec)
    out_path = {}
    for gen, paths in path_sums.items():
        per_path = {}
        for p, ps in paths.items():
            if ps["n_scen"] > 0:
                per_path[p] = (ps["sum_droll_rate"] / ps["n_scen"],
                               ps["sum_dpitch_rate"] / ps["n_scen"])
        if per_path:
            out_path[gen] = per_path
    return out, out_path


def smooth(vals, window=7):
    n = len(vals)
    out = []
    for i in range(n):
        lo = max(0, i - window // 2)
        hi = min(n, i + window // 2 + 1)
        out.append(sum(vals[lo:hi]) / (hi - lo))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", type=Path, default=Path("data.dat"))
    ap.add_argument("--label", default="run")
    ap.add_argument("--total-gens", type=int, default=800)
    ap.add_argument("--pop-size", type=int, default=5000)
    ap.add_argument("--out", type=Path,
                    default=Path("specs/028-deeper-rnn/per_axis_time_series.png"))
    args = ap.parse_args()

    print(f"Loading {args.inp} (pop={args.pop_size})…")
    data, data_path = load(args.inp, args.pop_size)
    if not data:
        raise SystemExit(f"no rows in {args.inp}")
    gens = sorted(data.keys())
    print(f"  → gens {gens[0]}-{gens[-1]} ({len(gens)} total)")

    dpt = [data[g][0] for g in gens]
    drl = [data[g][1] for g in gens]
    dth = [data[g][2] for g in gens]
    mpt = [data[g][3] for g in gens]
    mrl = [data[g][4] for g in gens]
    mth = [data[g][5] for g in gens]

    # Per-path totals: 5 lines, one per path
    path_indices = sorted({p for g in data_path for p in data_path[g].keys()})
    # Build per-path traces: for each path, list of (gen, total_drl, total_dpt)
    per_path_drl = {p: [] for p in path_indices}
    per_path_dpt = {p: [] for p in path_indices}
    per_path_gens = {p: [] for p in path_indices}
    for g in gens:
        if g not in data_path:
            continue
        for p in path_indices:
            if p in data_path[g]:
                tdrl, tdpt = data_path[g][p]
                per_path_drl[p].append(tdrl)
                per_path_dpt[p].append(tdpt)
                per_path_gens[p].append(g)

    # Late-run summary
    last50 = gens[-50:] if len(gens) >= 50 else gens
    idx_last = [gens.index(g) for g in last50]
    print(f"\nlate-run mean (last {len(last50)} gens):")
    for name, vals in [("pt", dpt), ("rl", drl), ("th", dth)]:
        m = sum(vals[i] for i in idx_last) / len(idx_last)
        print(f"  dCtrl_{name} = {m:.3f}")
    for name, vals in [("pt", mpt), ("rl", mrl), ("th", mth)]:
        m = sum(vals[i] for i in idx_last) / len(idx_last)
        print(f"  |out|_{name} = {m:.3f}")
    print(f"\nlate-run mean airframe rotation rate (deg/sec, normalized by scenario duration), by path (last {len(last50)} gens):")
    print("    path  |  roll rate (°/s)  |  pitch rate (°/s)")
    for p in path_indices:
        n = min(len(per_path_drl[p]), 50)
        if n == 0:
            continue
        m_drl = sum(per_path_drl[p][-n:]) / n
        m_dpt = sum(per_path_dpt[p][-n:]) / n
        print(f"      {p}   |    {m_drl:7.1f}        |    {m_dpt:7.1f}")

    fig, (ax_d, ax_m, ax_pdrl, ax_pdpt) = plt.subplots(
        4, 1, figsize=(14, 16), sharex=True)

    series = [
        ("pitch (pt)", "tab:red"),
        ("roll (rl)",  "tab:blue"),
        ("throttle (th)", "tab:green"),
    ]

    # Top: dCtrl per axis
    for (name, c), vals in zip(series, [dpt, drl, dth]):
        ax_d.plot(gens, vals, linewidth=0.7, alpha=0.30, color=c)
        ax_d.plot(gens, smooth(vals), linewidth=2.0, alpha=0.95, color=c, label=name)
    ax_d.axhline(0.27, color="gray", linestyle="--", linewidth=0.7, alpha=0.6,
                 label="per-axis budget 0.27 (sum 0.80/3)")
    ax_d.set_ylabel("Mean |Δout| per tick  (stick speed)")
    ax_d.set_title(
        f"{args.label} — per-axis aggressiveness time series\n"
        "top: change rate (bang-bang detector)  |  amplitude (saturation detector)\n"
        "bottom: per-path airframe rotation RATE (deg/sec, normalized by scenario duration)\n"
        "watch dCtrl slope — flat = bang-bang locked; trending down = smoothness emerging"
    )
    ax_d.grid(True, linewidth=0.5, alpha=0.4)
    ax_d.legend(loc="upper left", framealpha=0.9, fontsize=9)
    ax_d.set_ylim(bottom=0)

    # Mid: amplitude per axis
    for (name, c), vals in zip(series, [mpt, mrl, mth]):
        ax_m.plot(gens, vals, linewidth=0.7, alpha=0.30, color=c)
        ax_m.plot(gens, smooth(vals), linewidth=2.0, alpha=0.95, color=c, label=name)
    ax_m.axhline(0.67, color="gray", linestyle="--", linewidth=0.7, alpha=0.6,
                 label="per-axis budget 0.67 (sum 2.00/3)")
    ax_m.axhline(1.0, color="black", linestyle=":", linewidth=0.6, alpha=0.5,
                 label="full-throw ceiling 1.0")
    ax_m.set_ylabel("Mean |out| per tick  (stick amplitude)")
    ax_m.grid(True, linewidth=0.5, alpha=0.4)
    ax_m.legend(loc="upper right", framealpha=0.9, fontsize=9)
    ax_m.set_ylim(bottom=0, top=1.05)

    # Bottom-1: per-path total |Δroll| per scenario
    path_colors = ["tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple"]
    for p in path_indices:
        c = path_colors[p % len(path_colors)]
        gs = per_path_gens[p]
        ax_pdrl.plot(gs, per_path_drl[p], linewidth=0.7, alpha=0.30, color=c)
        ax_pdrl.plot(gs, smooth(per_path_drl[p]), linewidth=2.0, alpha=0.95,
                     color=c, label=f"path {p}")
    ax_pdrl.set_ylabel("Roll rate per scenario (deg/sec, from quat,\nnormalized by scenario duration)")
    ax_pdrl.grid(True, linewidth=0.5, alpha=0.4)
    ax_pdrl.legend(loc="upper left", framealpha=0.9, fontsize=9, ncol=2)
    ax_pdrl.set_ylim(bottom=0)

    # Bottom-2: per-path total |Δpitch| per scenario
    for p in path_indices:
        c = path_colors[p % len(path_colors)]
        gs = per_path_gens[p]
        ax_pdpt.plot(gs, per_path_dpt[p], linewidth=0.7, alpha=0.30, color=c)
        ax_pdpt.plot(gs, smooth(per_path_dpt[p]), linewidth=2.0, alpha=0.95,
                     color=c, label=f"path {p}")
    ax_pdpt.set_xlabel("Generation")
    ax_pdpt.set_ylabel("Pitch rate per scenario (deg/sec, from quat,\nnormalized by scenario duration)")
    ax_pdpt.grid(True, linewidth=0.5, alpha=0.4)
    ax_pdpt.legend(loc="upper left", framealpha=0.9, fontsize=9, ncol=2)
    ax_pdpt.set_xlim(0, args.total_gens)
    ax_pdpt.set_ylim(bottom=0)

    # 40-gen variation-ramp markers
    for ax in (ax_d, ax_m, ax_pdrl, ax_pdpt):
        for g in range(40, args.total_gens + 1, 40):
            ax.axvline(g, color="red", linewidth=0.4, alpha=0.25)

    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=110)
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
