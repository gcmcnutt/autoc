#!/usr/bin/env python3
"""
tracking_by_wind.py — TRACKING BY REALIZED WIND, latest generation (043 t4 snapshot).

WHY (operator 2026-10-06): t4 trains on a 0–8 m/s wind envelope (t3 was 3.66 m/s
on every tick). The regiment-wide numbers now average calm scenarios with 15-kt
ones, and the 09-13 flight showed the failure that matters in wind is a
ONE-SIDED DOWNWIND BIAS (~25 m) rather than noise. So: bin every scenario of the
elite by the wind it actually flew in, and show tracking per bin.

WHAT: per scenario, realized wind speed = median over its ticks of
‖(wN, wE)‖ from the tick CSV (steady wind + thermal inflow; the sim records what
the FDM applied). Bins 0–2 / 2–4 / 4–6 / 6–8+ m/s. Per bin:

  1. tracking score PER STEP (−score / steps_taken from the meta; ↑ better —
     length-fair, so a 45 s random course is not flattered) with the scenario
     count and the crash count
  2. pctInStreak by bin = Σ streak_steps / Σ steps_taken (from the meta)
  3. rotRatio = Σ craft rotation / Σ path demand (the T102 axis; t3 ref 5.10x)
  4. heatmap: paths (rows) × wind bins (cols) of mean tracking score — which
     COURSE suffers in wind, which is the question the per-path and per-wind
     views cannot answer separately.

⚠️ Snapshot only (one generation): a per-generation version needs the full tick
CSV per sampled gen, which is the expensive fetch; do that later if this view
earns it. ⚠️ Until the variation ramp is well past gen 40 the realized wind is
near-constant and most bins are empty — the panel says so rather than plotting
nothing.

Usage:
  tracking_by_wind.py --csv tick.csv --meta meta.yaml --label NAME --gen N [--path3 randomA] -o out.png
"""
import argparse
import csv
import re
import sys
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BINS = [(0, 2), (2, 4), (4, 6), (6, 99)]
BIN_LABELS = ["0–2", "2–4", "4–6", "6+"]
NPATH = 6
PATH_NAMES = ["StraightAndLevel", "SpiralClimb", "HorizFigure8", "slot3", "HighPerchSplitS", "SeededRandomB"]

SC_RE = re.compile(r"^\s+- idx:\s*(\d+)")
KV_RE = re.compile(r"^\s+([a-z_]+):\s*(.+?)\s*$")
ROT_RE = re.compile(r"(-?[0-9.]+)\s+\(craft\s+(-?[0-9.]+)\s*/\s*path\s+(-?[0-9.]+)\)")


def read_meta(path):
    """idx -> dict(path, score, crashed, craft, path_rot)"""
    out, cur, idx = {}, None, None
    for line in open(path):
        m = SC_RE.match(line)
        if m:
            if cur is not None: out[idx] = cur
            idx, cur = int(m.group(1)), {}
            continue
        if cur is None: continue
        m = KV_RE.match(line)
        if not m: continue
        k, v = m.group(1), m.group(2)
        if k == "path_variant": cur["path"] = int(v)
        elif k == "score": cur["score"] = float(v)
        elif k == "crash_reason": cur["crashed"] = (v.strip() != "RabbitComplete")
        elif k == "streak_steps": cur["streak_steps"] = float(v)
        elif k == "steps":
            sm = re.match(r"\s*(\d+)\s*/\s*(\d+)", v)
            if sm: cur["steps"] = float(sm.group(1))
        elif k == "excess_rotation_rad":
            r = ROT_RE.search(v)
            if r: cur["craft"] = float(r.group(2)); cur["path_rot"] = float(r.group(3))
    if cur is not None: out[idx] = cur
    return out


def read_ticks(path):
    """idx -> dict(wind=[...], instreak=n, ticks=n)"""
    acc = defaultdict(lambda: {"wind": [], "instreak": 0, "ticks": 0})
    with open(path) as f:
        r = csv.reader(f); h = next(r); ix = {n: i for i, n in enumerate(h)}
        need = ["scenario", "wN", "wE", "mult"]
        missing = [c for c in need if c not in ix]
        if missing:
            sys.exit(f"tick CSV lacks {missing} — is this a post-038 pathgen dump?")
        for row in r:
            if len(row) < len(h): continue
            s = int(row[ix["scenario"]]); d = acc[s]
            d["wind"].append((float(row[ix["wN"]]) ** 2 + float(row[ix["wE"]]) ** 2) ** 0.5)
            d["ticks"] += 1
            if float(row[ix["mult"]]) > 1.0: d["instreak"] += 1
    return acc


def bin_of(w):
    for i, (lo, hi) in enumerate(BINS):
        if lo <= w < hi: return i
    return len(BINS) - 1


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--csv", required=True); p.add_argument("--meta", required=True)
    p.add_argument("--label", default="run"); p.add_argument("--gen", type=int, default=0)
    p.add_argument("--path3", default="slot3"); p.add_argument("-o", "--out", required=True)
    a = p.parse_args()

    meta = read_meta(a.meta); ticks = read_ticks(a.csv)
    names = list(PATH_NAMES); names[3] = a.path3
    rows = []
    for idx, m in meta.items():
        t = ticks.get(idx)
        if not t or not t["wind"] or "score" not in m: continue
        w = float(np.median(t["wind"]))
        rows.append(dict(idx=idx, path=m.get("path", idx // 49), wind=w, bin=bin_of(w),
                         score=-m["score"] / max(1.0, m.get("steps", 1.0)),           # per step (length-fair)
                         streak_steps=m.get("streak_steps", 0.0), steps=m.get("steps", 0.0),
                         crashed=bool(m.get("crashed")), craft=m.get("craft", 0.0), prot=m.get("path_rot", 0.0),
                         instreak=t["instreak"] / max(1, t["ticks"])))
    if not rows: sys.exit("no scenarios joined between meta and tick CSV")
    winds = np.array([r["wind"] for r in rows])
    spread = winds.max() - winds.min()

    nb = len(BINS)
    n = np.zeros(nb, int); score = np.full(nb, np.nan); crash = np.zeros(nb, int)
    instreak = np.full(nb, np.nan); rot = np.full(nb, np.nan)
    for b in range(nb):
        rs = [r for r in rows if r["bin"] == b]; n[b] = len(rs)
        if not rs: continue
        score[b] = np.mean([r["score"] for r in rs]); crash[b] = sum(r["crashed"] for r in rs)
        st = sum(r["steps"] for r in rs); instreak[b] = 100 * sum(r["streak_steps"] for r in rs) / st if st > 0 else np.nan
        c = sum(r["craft"] for r in rs); pr = sum(r["prot"] for r in rs)
        rot[b] = c / pr if pr > 1e-9 else np.nan
    heat = np.full((NPATH, nb), np.nan); heatn = np.zeros((NPATH, nb), int)
    for pth in range(NPATH):
        for b in range(nb):
            rs = [r for r in rows if r["path"] == pth and r["bin"] == b]
            heatn[pth, b] = len(rs)
            if rs: heat[pth, b] = np.mean([r["score"] for r in rs])

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    x = np.arange(nb)
    ax = axes[0, 0]
    ax.bar(x, np.nan_to_num(score), color="tab:blue", alpha=0.85)
    for i in range(nb):
        ax.text(i, (score[i] if not np.isnan(score[i]) else 0), f"n={n[i]}\ncrash {crash[i]}", ha="center", va="bottom", fontsize=9)
    top = np.nanmax(score) if np.any(~np.isnan(score)) else 1.0
    ax.set_ylim(0, top * 1.25)
    ax.set_xticks(x); ax.set_xticklabels([f"{l} m/s" for l in BIN_LABELS]); ax.set_ylabel("tracking score per step (↑ better)")
    ax.set_title("tracking by realized wind"); ax.grid(True, axis="y", linewidth=0.4, alpha=0.4)
    ax = axes[0, 1]
    ax.bar(x, np.nan_to_num(instreak), color="tab:orange", alpha=0.85)
    ax.set_xticks(x); ax.set_xticklabels([f"{l} m/s" for l in BIN_LABELS]); ax.set_ylabel("pctInStreak (%)  Σ streak_steps / Σ steps")
    ax.set_title("pctInStreak by wind bin"); ax.grid(True, axis="y", linewidth=0.4, alpha=0.4)
    ax = axes[1, 0]
    ax.bar(x, np.nan_to_num(rot), color="tab:brown", alpha=0.85)
    # No fixed reference line (a flat number reads as a target; none exists). 1.0x = the path's demand.
    ax.axhline(1.0, color="gray", linestyle=":", linewidth=0.7, label="1.0x = path demand")
    ax.set_xticks(x); ax.set_xticklabels([f"{l} m/s" for l in BIN_LABELS]); ax.set_ylabel("rotRatio = craft / path rotation")
    ax.set_title("excess rotation by wind bin"); ax.legend(fontsize=8); ax.grid(True, axis="y", linewidth=0.4, alpha=0.4)
    ax = axes[1, 1]
    im = ax.imshow(heat, aspect="auto", cmap="viridis", origin="upper")
    ax.set_xticks(x); ax.set_xticklabels([f"{l}" for l in BIN_LABELS]); ax.set_xlabel("wind bin (m/s)")
    ax.set_yticks(range(NPATH)); ax.set_yticklabels([f"p{i} {names[i]}" for i in range(NPATH)], fontsize=8)
    for pth in range(NPATH):
        for b in range(nb):
            if heatn[pth, b]:
                ax.text(b, pth, f"{heat[pth, b]:.2f}\n(n{heatn[pth, b]})", ha="center", va="center", fontsize=7,
                        color="white" if (np.nanmax(heat) and heat[pth, b] < 0.6 * np.nanmax(heat)) else "black")
    fig.colorbar(im, ax=ax, fraction=0.04, label="tracking score per step")
    ax.set_title("score: path × wind bin")
    note = "" if spread > 1.0 else f"  ⚠️ realized-wind spread only {spread:.2f} m/s — ramp not yet active, bins degenerate"
    fig.suptitle(f"{a.label} — gen {a.gen}: tracking by realized wind  (wind {winds.min():.1f}–{winds.max():.1f} m/s over {len(rows)} scenarios){note}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.96)); fig.savefig(a.out, dpi=110)
    print(f"wrote {a.out}  (bins n={[int(v) for v in n]}, spread {spread:.2f} m/s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
