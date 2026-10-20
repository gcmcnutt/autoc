#!/usr/bin/env python3
"""
sc006.py — the attribution result (044 SC-006, research R9): does the history-equipped smoke (B) reach the plant-only
smoke's (A) best smoothness in fewer generations?

Inputs are the two runs' pitch_cycle_progress caches (per sampled generation: elite medians over the 96 scenarios of
pitch-rate RMS and 3–5 Hz share, roll-rate RMS and 3–5 Hz share) and the two training logs (`#NNGen ... pctInStreak=`).
Smoothness per axis is the absolute 3–5 Hz rate RMS — the SC-001 / SC-002 quantity: RMS × sqrt(share).

Early elites barely fly (short, crash-ending flights have trivially low rate RMS), so a generation only COUNTS once its
elite tracks: pctInStreak ≥ FLOOR × A's best-of-run pctInStreak (default 0.5). Among counted generations:
  A*  = A's best (lowest) 3–5 Hz RMS on each axis; G_A = the later of the two generations where A reached them.
  G_B = the first counted generation where B is at or below A* on BOTH axes with pctInStreak ≥ A's at that generation
        (A's value interpolated to B's sampled generations).
  SC-006 met iff G_B exists and G_B < G_A. Recorded either way — the bake keeps history regardless (clarification 1).

Usage:
  sc006.py --a CACHE_A.csv --log-a A.log --b CACHE_B.csv --log-b B.log [--floor 0.5]
"""
import argparse
import csv
import math
import re
import sys

import numpy as np


def load_cache(path):
    out = {}
    for row in csv.DictReader(open(path)):
        g = int(row["gen"])
        f = {k: float(v) for k, v in row.items() if k != "gen"}
        out[g] = {"pitch35": f["q_rms"] * math.sqrt(max(f["pt_35"], 0.0) / 100.0),
                  "roll35": f["p_rms"] * math.sqrt(max(f["rl_35"], 0.0) / 100.0)}
    return out


def load_pct(path):
    pct = {}
    for line in open(path, errors="replace"):
        m = re.search(r"#NNGen gen=(\d+) .*?pctInStreak=([-\d.]+)", line)
        if m:
            pct[int(m.group(1))] = float(m.group(2))
    return pct


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--a", required=True); ap.add_argument("--log-a", required=True)
    ap.add_argument("--b", required=True); ap.add_argument("--log-b", required=True)
    ap.add_argument("--floor", type=float, default=0.5)
    a = ap.parse_args()
    A, B = load_cache(a.a), load_cache(a.b)
    pA, pB = load_pct(a.log_a), load_pct(a.log_b)
    floor = a.floor * max(pA.values())
    countA = [g for g in sorted(A) if pA.get(g, 0.0) >= floor]
    if not countA:
        print("no generation of A clears the tracking floor"); return 1
    best = {}
    for ax in ("pitch35", "roll35"):
        g = min(countA, key=lambda k: A[k][ax])
        best[ax] = (A[g][ax], g)
    G_A = max(best["pitch35"][1], best["roll35"][1])
    gA = np.array(sorted(pA)); vA = np.array([pA[g] for g in gA])
    G_B = None
    for g in sorted(B):
        if pB.get(g, 0.0) < floor:
            continue
        if (B[g]["pitch35"] <= best["pitch35"][0] and B[g]["roll35"] <= best["roll35"][0]
                and pB[g] >= float(np.interp(g, gA, vA))):
            G_B = g
            break
    print(f"tracking floor: pctInStreak >= {floor:.1f} % ({a.floor:.2f} x A's best {max(pA.values()):.1f} %)")
    print(f"A best 3-5 Hz pitch RMS {best['pitch35'][0]:.1f} deg/s at gen {best['pitch35'][1]}; "
          f"roll {best['roll35'][0]:.1f} deg/s at gen {best['roll35'][1]}  ->  G_A = {G_A}")
    if G_B is None:
        print("B never reaches A's best on both axes at matched tracking  ->  SC-006 NOT met")
    else:
        print(f"B first at/below both at gen {G_B} (pitch {B[G_B]['pitch35']:.1f}, roll {B[G_B]['roll35']:.1f}, "
              f"pctInStreak {pB[G_B]:.1f} %)  ->  G_B = {G_B}: SC-006 {'MET' if G_B < G_A else 'NOT met'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
