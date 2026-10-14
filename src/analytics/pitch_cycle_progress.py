#!/usr/bin/env python3
"""
pitch_cycle_progress.py — THE RELAY, OVER GENERATIONS (043 t6, 2026-10-14).

WHY: the t6 plant has the aircraft's pitch short period (I_yy 0.0094, ~2.6-3 Hz), so for the first
time a genome that drives the pitch setpoint rail-to-rail pays for it in the sim the way it does in
the air. The gen-92 t6 elite sat at the ±240 deg/s cap 51 % of the time and flipped 5.5x/s; t5's
gen-92 elite was at the cap 44 % and ended at 1.4 % by its flown gen 488. The feature's exit is a
flight with the cycle reduced, and the in-run proxy for that is this trajectory: cap %, flips/s and
pitch-rate RMS of the elite falling with generations while tracking rises.

WHAT: for each sampled generation, `dmp-dump <run> --gen N --csv-only` (the per-tick CSV; expensive —
one S3 fetch per sampled gen, so it is cached like the per-path report), then medians over the 294
scenarios of:

  pitch:    RMS body pitch rate (deg/s; gyrQ is rad/s ÷ 6), mean |out_pt| (×240 = deg/s setpoint),
            % of ticks at the cap (|out| > 0.9), sign flips per second, share of pitch-rate power in
            2-3 Hz and in 3-5 Hz (Welch at the 20 Hz tick rate)
  roll:     mean |out_rl|, cap %, flips/s
  throttle: cap %

Four panels, run solid + `--ref` run dashed (t5 is the reference for t6; its cache is built once with
this same script against its pinned prefix, as the per-path report does).

Usage:
  pitch_cycle_progress.py --run s3://autoc-m1/<run>/ --gens 1-N --stride S -i autoc.ini --label NAME
      --total-gens 800 [--cache FILE] [--ref LABEL:CACHE] -o out.png
"""
import argparse
import csv
import math
import os
import subprocess
import sys
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DMP = os.path.join(REPO, "build", "dmp-dump")
GYRO_SCALE = 6.0
FS = 20.0
COLS = ["gen", "q_rms", "pt_abs", "pt_cap", "pt_flips", "pt_23", "pt_35", "rl_abs", "rl_cap", "rl_flips", "th_cap", "n"]


def welch_share(x, lo, hi, nseg=128):
    x = np.asarray(x, float) - np.mean(x)
    if len(x) < 64:
        return float("nan")
    nseg = min(nseg, (len(x) // 2) * 2)
    w = np.hanning(nseg)
    segs = [x[i:i + nseg] * w for i in range(0, len(x) - nseg + 1, nseg // 2)]
    P = np.mean([np.abs(np.fft.rfft(s)) ** 2 for s in segs], axis=0)
    fr = np.fft.rfftfreq(nseg, 1 / FS)
    tot = P[fr > 0.25].sum()
    return 100.0 * P[(fr >= lo) & (fr < hi)].sum() / tot if tot > 0 else float("nan")


def summarize_csv(text):
    r = csv.reader(text.splitlines())
    h = next(r)
    first, last = {}, {}
    for i, n in enumerate(h):
        first.setdefault(n, i); last[n] = i
    need = ["scenario", "out_pt", "out_rl", "out_th", "gyrQ"]
    if any(c not in first for c in need):
        return None
    iS, iP, iR, iT, iQ = first["scenario"], first["out_pt"], first["out_rl"], first["out_th"], last["gyrQ"]
    sc = defaultdict(list)
    for row in r:
        if len(row) < len(h):
            continue
        sc[int(row[iS])].append((float(row[iP]), float(row[iR]), float(row[iT]), float(row[iQ])))
    acc = defaultdict(list)
    for s, rows in sc.items():
        A = np.array(rows)
        if len(A) < 40:
            continue
        pt, rl, th, q = A[:, 0], A[:, 1], A[:, 2], np.degrees(A[:, 3] * GYRO_SCALE)
        dur = len(A) / FS
        acc["q_rms"].append(float(np.sqrt(np.mean(q ** 2))))
        acc["pt_abs"].append(float(np.mean(np.abs(pt)))); acc["pt_cap"].append(100 * float(np.mean(np.abs(pt) > 0.9)))
        acc["pt_flips"].append(float(np.sum(np.sign(pt[1:]) != np.sign(pt[:-1])) / dur))
        acc["pt_23"].append(welch_share(q, 2, 3)); acc["pt_35"].append(welch_share(q, 3, 5))
        acc["rl_abs"].append(float(np.mean(np.abs(rl)))); acc["rl_cap"].append(100 * float(np.mean(np.abs(rl) > 0.9)))
        acc["rl_flips"].append(float(np.sum(np.sign(rl[1:]) != np.sign(rl[:-1])) / dur))
        acc["th_cap"].append(100 * float(np.mean(np.abs(th) > 0.9)))
    if not acc["q_rms"]:
        return None
    out = {k: float(np.nanmedian(v)) for k, v in acc.items()}
    out["n"] = len(acc["q_rms"])
    return out


def load_cache(path):
    data = {}
    if path and os.path.exists(path):
        with open(path) as f:
            for row in csv.DictReader(f):
                data[int(row["gen"])] = {k: float(row[k]) for k in COLS[1:]}
    return data


def save_cache(path, data):
    if not path:
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", newline="") as f:
        w = csv.writer(f); w.writerow(COLS)
        for g in sorted(data):
            w.writerow([g] + [f"{data[g][k]:.5f}" for k in COLS[1:]])
    os.replace(tmp, path)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--run", required=True); p.add_argument("--gens", required=True); p.add_argument("--stride", type=int, default=5)
    p.add_argument("-i", "--config", default="autoc.ini"); p.add_argument("--label", default="run"); p.add_argument("--total-gens", type=int, default=800)
    p.add_argument("--cache", default=None); p.add_argument("--ref", default=None, help="LABEL:CACHE.csv of a reference run (dashed)")
    p.add_argument("-o", "--out", required=True)
    a = p.parse_args()
    g0, g1 = (int(x) for x in a.gens.split("-"))
    wanted = list(range(g0, g1 + 1, a.stride))
    if wanted[-1] != g1:
        wanted.append(g1)
    data = load_cache(a.cache)
    todo = [g for g in wanted if g not in data]
    print(f"{len(wanted)} gens requested, {len(todo)} to fetch ({len(wanted) - len(todo)} cached)")
    for n, g in enumerate(todo, 1):
        out = subprocess.run([DMP, a.run, "--gen", str(g), "--csv-only", "-i", a.config], capture_output=True, text=True)
        if out.returncode != 0:
            print(f"  [{n}/{len(todo)}] gen {g}: SKIP {out.stderr.strip()[:120]}", file=sys.stderr); continue
        s = summarize_csv(out.stdout)
        if s is None:
            print(f"  [{n}/{len(todo)}] gen {g}: no usable tick data, skip", file=sys.stderr); continue
        data[g] = s
        print(f"  [{n}/{len(todo)}] gen {g}: qRMS {s['q_rms']:.0f} deg/s, pitch cap {s['pt_cap']:.0f}%, flips {s['pt_flips']:.1f}/s, 2-3 Hz {s['pt_23']:.0f}%")
        if n % 5 == 0:
            save_cache(a.cache, data)
    save_cache(a.cache, data)
    gens = sorted(data)
    if not gens:
        print("nothing to plot", file=sys.stderr); return 1
    ref_label, ref = None, {}
    if a.ref:
        ref_label, ref_path = a.ref.split(":", 1); ref = load_cache(ref_path)
    rg = sorted(ref)

    fig, axes = plt.subplots(4, 1, figsize=(14, 14), sharex=True)
    def line(ax, key, color, label, style="-", src=data, gg=gens, lw=1.8):
        ax.plot(gg, [src[g][key] for g in gg], color=color, linestyle=style, linewidth=lw, label=label)
    ax = axes[0]
    line(ax, "q_rms", "tab:red", f"{a.label} pitch-rate RMS")
    if ref: line(ax, "q_rms", "tab:red", f"{ref_label} (reference)", "--", ref, rg, 1.3)
    ax.axhspan(124, 153, color="tab:red", alpha=0.08, label="g488 in flight: 124–153 deg/s")
    ax.set_ylabel("elite pitch-rate RMS (deg/s)"); ax.legend(fontsize=8, loc="upper right"); ax.grid(alpha=.3)
    ax = axes[1]
    line(ax, "pt_cap", "tab:red", f"{a.label} pitch"); line(ax, "rl_cap", "tab:blue", f"{a.label} roll"); line(ax, "th_cap", "tab:green", f"{a.label} throttle")
    if ref:
        line(ax, "pt_cap", "tab:red", f"{ref_label} pitch", "--", ref, rg, 1.3); line(ax, "rl_cap", "tab:blue", f"{ref_label} roll", "--", ref, rg, 1.3)
    ax.set_ylabel("ticks at the command cap (%)  |out| > 0.9"); ax.set_ylim(0, 100); ax.legend(fontsize=8, ncol=3, loc="upper right"); ax.grid(alpha=.3)
    ax = axes[2]
    line(ax, "pt_flips", "tab:red", f"{a.label} pitch"); line(ax, "rl_flips", "tab:blue", f"{a.label} roll")
    if ref:
        line(ax, "pt_flips", "tab:red", f"{ref_label} pitch", "--", ref, rg, 1.3); line(ax, "rl_flips", "tab:blue", f"{ref_label} roll", "--", ref, rg, 1.3)
    ax.set_ylabel("command sign flips per second"); ax.legend(fontsize=8, ncol=2, loc="upper right"); ax.grid(alpha=.3)
    ax = axes[3]
    line(ax, "pt_23", "tab:purple", f"{a.label} 2–3 Hz"); line(ax, "pt_35", "tab:orange", f"{a.label} 3–5 Hz")
    if ref: line(ax, "pt_23", "tab:purple", f"{ref_label} 2–3 Hz", "--", ref, rg, 1.3)
    ax.axhspan(31, 63, color="tab:purple", alpha=0.08, label="g488 in flight, 2–3 Hz share 31–63 %")
    ax.set_ylabel("share of pitch-rate power (%)"); ax.set_ylim(0, 100); ax.legend(fontsize=8, ncol=3, loc="upper right"); ax.grid(alpha=.3)
    for ax in axes: ax.set_xlim(0, a.total_gens)
    axes[-1].set_xlabel("generation")
    last = data[gens[-1]]
    fig.suptitle(f"{a.label} — the relay over generations  (gen {gens[-1]}: pitch RMS {last['q_rms']:.0f} deg/s, cap {last['pt_cap']:.0f} %, "
                 f"{last['pt_flips']:.1f} flips/s, 2–3 Hz {last['pt_23']:.0f} %)", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97)); fig.savefig(a.out, dpi=110); print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
