#!/usr/bin/env python3
"""
tracking_by_path_progress.py — TRACKING OVER GENERATIONS, ONE LINE PER PATH (043 t4).

WHY (operator 2026-10-06): the evolution report's fitness panel is one number per
generation for the whole regiment. t4 changed the regiment in ways that are
PER-PATH by construction — slot 3 is now a second random course (cold-start
entry at ~20 m), slot 5 is the original random course (~38 m), slots 0/1/2/4 stay
latched tail-chases — and it added an objective axis whose whole point is that the
baseline differs per path (the straight line wandered 6.1x, the hard loop 4.0x).
A regiment-wide mean hides exactly the trade the operator is worried about:
"trades for worse tracking but closer in general". So: per path, over gens.

WHAT: for each sampled generation, `dmp-dump <run> --gen N --meta-only` yields the
elite's 294 per-scenario blocks (path_variant, score, crash_reason, max_streak,
excess_rotation_rad = craft/path). Grouped by path_variant (49 scenarios each):

  1. tracking score PER STEP — mean over scenarios of (−score / steps_taken), so UP is
     better and a 45 s random course is not flattered against a 17 s loop (the
     length-normalisation score_by_path.py already uses for M2)
  2. crashes per path      — count of the 49 that did not complete
  3. rotRatio per path     — Σ craft rotation / Σ path direction change; the
                             043 T102 axis quantity. Reference: t3 genome under
                             the t4 regiment = 5.10x overall.
  4. pctInStreak per path  — Σ streak_steps / Σ steps_taken (the regiment metric, by path)

Six path lines + a thick regiment line per panel. Path names follow
AeroStandardPathType with slot 3 shown as whichever the run used (the meta does
not say; the label is taken from --path3).

INCREMENTAL: one S3 fetch per sampled gen. --cache keeps a per-run CSV so a
re-run on a live bake fetches only the gens that have appeared since (same trick
as the run-summary and input_investment caches; key it on run-id + dmp-dump
signature from generate_pngs.sh so a tooling change invalidates it).

NO REBUILD NEEDED: everything here is already emitted by the current dmp-dump.

Usage:
  tracking_by_path_progress.py --run s3://autoc-m1/<run>/ --gens 1-N --stride S \\
      -i autoc.ini --label NAME --total-gens 800 [--cache FILE] [--path3 randomA] -o out.png
"""
import argparse
import csv
import os
import re
import subprocess
import sys
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DMP = os.path.join(REPO, "build", "dmp-dump")
NPATH = 6
PATH_NAMES = ["StraightAndLevel", "SpiralClimb", "HorizFigure8", "slot3", "HighPerchSplitS", "SeededRandomB"]

SC_RE = re.compile(r"^\s+- idx:\s*(\d+)")
KV_RE = re.compile(r"^\s+([a-z_]+):\s*(.+?)\s*$")
ROT_RE = re.compile(r"(-?[0-9.]+)\s+\(craft\s+(-?[0-9.]+)\s*/\s*path\s+(-?[0-9.]+)\)")


def fetch_meta(run, gen, ini):
    out = subprocess.run([DMP, run, "--gen", str(gen), "-i", ini, "--meta-only"],
                         capture_output=True, text=True)
    if out.returncode != 0:
        raise RuntimeError(f"dmp-dump --gen {gen} failed: {out.stderr.strip()[:200]}")
    return out.stdout


def parse_meta(text):
    """-> list of dicts: path, score, crashed, max_streak, craft, path_rot"""
    rows, cur = [], None
    for line in text.splitlines():
        m = SC_RE.match(line)
        if m:
            if cur: rows.append(cur)
            cur = {}
            continue
        if cur is None:
            continue
        m = KV_RE.match(line)
        if not m:
            continue
        k, v = m.group(1), m.group(2)
        if k == "path_variant": cur["path"] = int(v)
        elif k == "score": cur["score"] = float(v)
        elif k == "crash_reason": cur["crashed"] = (v.strip() != "RabbitComplete")
        elif k == "max_streak": cur["max_streak"] = float(v)
        elif k == "streak_steps": cur["streak_steps"] = float(v)
        elif k == "steps":
            sm = re.match(r"\s*(\d+)\s*/\s*(\d+)", v)
            if sm: cur["steps"] = float(sm.group(1)); cur["steps_total"] = float(sm.group(2))
        elif k == "excess_rotation_rad":
            r = ROT_RE.search(v)
            if r:
                cur["craft"] = float(r.group(2)); cur["path_rot"] = float(r.group(3))
    if cur: rows.append(cur)
    return [r for r in rows if "path" in r and "score" in r]


def summarize(rows):
    """-> dict keyed by path (0..5) and 'all': mean_score, crashes, rot_ratio, med_streak"""
    by = defaultdict(list)
    for r in rows:
        by[r["path"]].append(r); by["all"].append(r)
    out = {}
    for k, rs in by.items():
        sc = np.array([-r["score"] / max(1.0, r.get("steps", 1.0)) for r in rs])   # per step, up = better
        cr = sum(1 for r in rs if r.get("crashed"))
        craft = sum(r.get("craft", 0.0) for r in rs); prot = sum(r.get("path_rot", 0.0) for r in rs)
        ss = sum(r.get("streak_steps", 0.0) for r in rs); st = sum(r.get("steps", 0.0) for r in rs)
        out[k] = dict(mean_score=float(sc.mean()), crashes=cr,
                      rot_ratio=(craft / prot if prot > 1e-9 else float("nan")),
                      med_streak=(100.0 * ss / st if st > 0 else float("nan")))   # field kept; now pctInStreak %
    return out


CACHE_COLS = ["gen", "key", "mean_score", "crashes", "rot_ratio", "med_streak"]


def load_cache(path):
    data = {}
    if path and os.path.exists(path):
        with open(path) as f:
            for row in csv.DictReader(f):
                key = int(row["key"]) if row["key"].isdigit() else row["key"]   # path keys are ints in memory
                data.setdefault(int(row["gen"]), {})[key] = dict(
                    mean_score=float(row["mean_score"]), crashes=int(row["crashes"]),
                    rot_ratio=float(row["rot_ratio"]), med_streak=float(row["med_streak"]))
    return data


def save_cache(path, data):
    if not path: return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", newline="") as f:
        w = csv.writer(f); w.writerow(CACHE_COLS)
        for gen in sorted(data):
            for key, d in data[gen].items():
                w.writerow([gen, key, f"{d['mean_score']:.6f}", d["crashes"], f"{d['rot_ratio']:.6f}", f"{d['med_streak']:.3f}"])
    os.replace(tmp, path)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--run", required=True)
    p.add_argument("--gens", required=True, help="A-B inclusive")
    p.add_argument("--stride", type=int, default=5)
    p.add_argument("-i", "--config", default="autoc.ini")
    p.add_argument("--label", default="run")
    p.add_argument("--total-gens", type=int, default=800)
    p.add_argument("--cache", default=None)
    p.add_argument("--path3", default="slot3", help="name for slot 3 (randomA | FortyFive)")
    p.add_argument("--ref", default=None,
                   help="LABEL:CACHE.csv — overlay a reference run's regiment line (dashed) on every panel, "
                        "e.g. t3's post-hoc rotRatio trajectory (built once with this script against the pinned run)")
    p.add_argument("-o", "--out", required=True)
    a = p.parse_args()
    ref_label, ref = None, {}
    if a.ref:
        ref_label, ref_path = a.ref.split(":", 1)
        ref = load_cache(ref_path)
        if not ref:
            print(f"--ref cache {ref_path} is empty/missing — overlay skipped", file=sys.stderr)

    g0, g1 = (int(x) for x in a.gens.split("-"))
    wanted = list(range(g0, g1 + 1, a.stride))
    if wanted[-1] != g1: wanted.append(g1)          # always include the latest gen
    data = load_cache(a.cache)
    todo = [g for g in wanted if g not in data]
    print(f"{len(wanted)} gens requested, {len(todo)} to fetch ({len(wanted)-len(todo)} cached)")
    for n, gen in enumerate(todo, 1):
        try:
            rows = parse_meta(fetch_meta(a.run, gen, a.config))
        except RuntimeError as e:
            print(f"  [{n}/{len(todo)}] gen {gen}: SKIP {e}", file=sys.stderr); continue
        if len(rows) < NPATH:
            print(f"  [{n}/{len(todo)}] gen {gen}: only {len(rows)} scenarios parsed, skip", file=sys.stderr); continue
        data[gen] = summarize(rows)
        al = data[gen]["all"]
        print(f"  [{n}/{len(todo)}] gen {gen}: score/step {al['mean_score']:.3f} crashes {al['crashes']} rotRatio {al['rot_ratio']:.2f} pctInStreak {al['med_streak']:.1f}%")
        if n % 10 == 0: save_cache(a.cache, data)
    save_cache(a.cache, data)

    gens = sorted(g for g in data if g in set(wanted) or True)
    if not gens:
        print("nothing to plot", file=sys.stderr); return 1
    names = list(PATH_NAMES); names[3] = a.path3
    colors = plt.cm.tab10(np.linspace(0, 1, 10))
    fig, axes = plt.subplots(4, 1, figsize=(14, 15), sharex=True)
    panels = [("mean_score", "tracking score per step (↑ better)"),
              ("crashes", "crashes per path (of 49)"),
              ("rot_ratio", "rotRatio = craft rotation / path demand"),
              ("med_streak", "pctInStreak (%)  Σ streak_steps / Σ steps")]
    for ax, (key, title) in zip(axes, panels):
        for pth in range(NPATH):
            ys = [data[g][pth][key] if pth in data[g] else float("nan") for g in gens]
            ax.plot(gens, ys, color=colors[pth], linewidth=1.2, alpha=0.9, label=f"p{pth} {names[pth]}")
        # crashes: the regiment total (of 294) would leave the 0–49 per-path axis; show its mean per path
        ys = [data[g]["all"][key] / (NPATH if key == "crashes" else 1) for g in gens]
        ax.plot(gens, ys, color="black", linewidth=2.4, alpha=0.85,
                label="regiment (mean per path)" if key == "crashes" else "regiment")
        if ref:
            # The comparison that matters is the SLOPE at matched gens: t3's rotRatio fell
            # 10.2 → 5.85 over gens 50–396 with no rotation axis at all (natural decline), so
            # a run with the axis must fall FASTER than this, not merely below the 5.10x line.
            rg = sorted(ref)
            rys = [ref[g]["all"][key] / (NPATH if key == "crashes" else 1) for g in rg]
            ax.plot(rg, rys, color="tab:red", linestyle="--", linewidth=1.6, alpha=0.8,
                    label=f"{ref_label} regiment (no rotation axis)")
        ax.set_ylabel(title); ax.grid(True, linewidth=0.4, alpha=0.4)
        ax.set_xlim(0, a.total_gens)
        if key == "rot_ratio":
            # No fixed reference line: a flat number reads as a target, and none exists —
            # the comparison is the --ref run's own trajectory at matched gens (t3 fell
            # 11.8x → 4.55x with no rotation axis). 1.0x = the path's own demand.
            ax.axhline(1.0, color="gray", linestyle=":", linewidth=0.7, label="1.0x = path demand")
            ax.set_ylim(bottom=0)
        if key == "crashes":
            ax.set_ylim(0, 49)
    axes[0].legend(loc="upper left", ncol=4, fontsize=8, framealpha=0.9)
    axes[1].legend(loc="upper right", ncol=4, fontsize=8, framealpha=0.9)
    axes[2].legend(loc="upper right", fontsize=8, framealpha=0.9)
    axes[-1].set_xlabel("generation")
    last = data[gens[-1]]["all"]
    fig.suptitle(f"{a.label} — tracking by path over generations  (gen {gens[-1]}: score/step {last['mean_score']:.3f}, "
                 f"crashes {last['crashes']}/294, rotRatio {last['rot_ratio']:.2f}x, pctInStreak {last['med_streak']:.1f}%)", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(a.out, dpi=110)
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
