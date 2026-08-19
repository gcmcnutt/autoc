#!/usr/bin/env python3
"""041 P2-5 — specific-energy progress over generations (dmp-dump-based).

WHY THIS IS A SEPARATE REPORT. `plot_evolution_progress.py` panel 4 plots the
scalar objective — Σ energy_score per generation, one number for the whole
elite. That answers "is the axis moving" and nothing else. It cannot answer the
question 041 actually asks, which is AC-1's third part:

    is energy improving BECAUSE the policy flies better,
    or because it has been MUTED?

Those look identical in a summed scalar and completely different per tick. 035
failed exactly here: its energy term fell beautifully while the whole regiment
went quiet, and the run-level number never said so.

⛔ THE UNITS CHANGED AT 041 P2-5. `energy_score` used to be a convex integral of
the THROTTLE COMMAND; it is now **metres of specific energy destroyed**,
Σ max(0, −Ps)·dt. Same field name, different quantity, no conversion between
them. This report therefore refuses to plot pre-041 runs on the same axes —
see --compare below.

PANELS (per generation, elite individual, over all its scenarios):

  1. Es STATE — median with a p05–p95 band, in metres above the hard deck.
     The observation TA03 found missing: the policy carried AIRSPEED and no
     height term at all, so Es was unobservable and the 035 axis was selecting
     on something the network could not see. Rising median = the policy is
     learning to hold energy; a collapsing band = it has found one altitude and
     stopped manoeuvring, which is muting.

  2. Ps RATE — median plus the p05 (the loss tail). The axis charges only the
     NEGATIVE part, so p05 is the quantity under selection pressure and the
     median says whether the policy is net-climbing or net-bleeding.

  3. ENERGY DESTROYED — Σ max(0, −Ps)·dt per scenario, i.e. the objective
     itself, but per-scenario-mean so it is comparable across runs of different
     scenario counts (the log's `energy=` is a raw sum over 294 scenarios and
     is not).

  4. ⭐ THE MUTING TEST — energy destroyed against tracking occupancy, one point
     per sampled generation, coloured by generation. TA03 measured
     corr(Ps, closure rate) = −0.048, i.e. the two axes are ORTHOGONAL, which
     is why lexicase can hold both. What we want to see is the cloud moving
     DOWN-AND-RIGHT: less waste at equal-or-better tracking. Down-and-LEFT is
     035 happening again — cheaper because it stopped trying.

Each generation costs one `dmp-dump --gen N --csv-only` S3 fetch, so sample with
--stride and use --cache (same incremental pattern as dynamics_progress.py:
computed rows persist, re-runs on a live run fetch only new gens).

Usage:
    python3 energy_progress.py --run s3://autoc-m1/<run-id>/ \
        --gens 1-157 --stride 5 -i autoc.ini \
        --cache /tmp/t4_energy_cache.csv \
        --label autoc-041-t4 -o out.png
"""
import argparse
import csv
import os
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Imported rather than restated so this report's "tracking" means exactly what
# every other report's does (dynamics_progress.py owns the rule).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from dynamics_progress import TRACK_THRESHOLD
except ImportError:  # standalone use
    TRACK_THRESHOLD = 0.5

CACHE_FIELDS = ["gen", "es_p05", "es_med", "es_p95",
                "ps_p05", "ps_med", "ps_p95",
                "destroyed_per_scenario", "track_pct", "ticks"]


def compute_gen_metrics(rows):
    """Per-generation energy summary from one gen's per-tick CSV."""
    if not rows:
        return None
    # 041 P2-4 columns. Absent => this dmp predates the schema; say so rather
    # than plotting zeros, which would read as "no energy destroyed".
    if "Es_m" not in rows[0]:
        raise KeyError("Es_m")

    es, ps, track = [], [], 0
    # Energy destroyed integrates max(0, -Ps)*dt per scenario, so it has to be
    # accumulated per scenario and averaged — not pooled over ticks.
    per_scenario = {}
    prev_t = {}
    for r in rows:
        scn = r.get("scenario", "0")
        try:
            e = float(r["Es_m"])
        except (KeyError, ValueError):
            continue
        es.append(e)
        if r.get("stpPt"):
            try:
                if float(r["stpPt"]) >= TRACK_THRESHOLD:
                    track += 1
            except ValueError:
                pass
        p = r.get("Ps_mps", "")
        if p not in ("", None):
            try:
                pv = float(p)
            except ValueError:
                continue
            ps.append(pv)
            # dt from the recorded sim clock; never an assumed tick.
            t = float(r.get("tick", 0))
            dt = 0.0
            if scn in prev_t:
                dt = max(0.0, t - prev_t[scn])
            prev_t[scn] = t
            if pv < 0 and dt > 0:
                per_scenario[scn] = per_scenario.get(scn, 0.0) + (-pv) * dt
            else:
                per_scenario.setdefault(scn, 0.0)

    if not es:
        return None
    es = np.array(es)
    psa = np.array(ps) if ps else np.array([0.0])
    nscn = max(1, len(per_scenario))
    return {
        "es_p05": float(np.percentile(es, 5)),
        "es_med": float(np.median(es)),
        "es_p95": float(np.percentile(es, 95)),
        "ps_p05": float(np.percentile(psa, 5)),
        "ps_med": float(np.median(psa)),
        "ps_p95": float(np.percentile(psa, 95)),
        "destroyed_per_scenario": float(sum(per_scenario.values()) / nscn),
        "track_pct": 100.0 * track / len(rows),
        "ticks": float(len(rows)),
    }


def fetch_gen_csv(dmp_dump, run, gen, ini):
    res = subprocess.run([dmp_dump, run, "--gen", str(gen), "--csv-only", "-i", ini],
                         capture_output=True, text=True)
    if res.returncode != 0:
        tail = res.stderr.strip().splitlines()[-1:] or ["(no stderr)"]
        raise RuntimeError(f"dmp-dump --gen {gen} failed: {tail}")
    return res.stdout


def load_cache(path):
    if not path or not os.path.isfile(path):
        return {}
    out = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                out[int(r["gen"])] = {k: float(r[k]) for k in CACHE_FIELDS if k != "gen"}
            except (KeyError, TypeError, ValueError):
                continue  # older schema row — drop it, the gen refetches
    return out


def save_cache(path, data):
    if not path:
        return
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CACHE_FIELDS)
        w.writeheader()
        for gen in sorted(data):
            w.writerow({"gen": gen, **data[gen]})


def parse_gens(spec, stride):
    lo, hi = (int(x) for x in spec.split("-"))
    gens = list(range(lo, hi + 1, stride))
    if gens and gens[-1] != hi:
        gens.append(hi)
    return gens


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--run", required=True, help="s3://<bucket>/<run-id>/")
    p.add_argument("--gens", required=True, help="LO-HI generation range (actualGen)")
    p.add_argument("--stride", type=int, default=5)
    p.add_argument("--dmp-dump", default="./build/dmp-dump")
    p.add_argument("-i", "--config", default="autoc.ini")
    p.add_argument("--cache", default=None)
    p.add_argument("--label", default="run")
    p.add_argument("--total-gens", type=int, default=None)
    p.add_argument("-o", "--out", required=True)
    args = p.parse_args()

    data = load_cache(args.cache)
    want = parse_gens(args.gens, args.stride)
    todo = [g for g in want if g not in data]
    print(f"{len(want)} gens requested, {len(todo)} to fetch ({len(want) - len(todo)} cached)")

    for i, g in enumerate(todo, 1):
        try:
            rows = list(csv.DictReader(fetch_gen_csv(
                args.dmp_dump, args.run, g, args.config).splitlines()))
            rec = compute_gen_metrics(rows)
        except KeyError:
            # ⛔ Loud, not silent. A dmp without Es_m predates 041 P2-4, and the
            # only honest thing is to stop: plotting the gens that DO have it
            # beside a gap would read as "energy was zero here".
            print(f"  gen {g}: no Es_m column — this dmp predates 041 P2-4.\n"
                  f"  Energy tracking is not reconstructible for it; there is no\n"
                  f"  migration path (FR-005 greenfield). Restrict --gens to the\n"
                  f"  post-break range.", file=sys.stderr)
            return 2
        except RuntimeError as e:
            print(f"  gen {g}: {e}", file=sys.stderr)
            continue
        if rec:
            data[g] = rec
            print(f"  [{i}/{len(todo)}] gen {g}: Es med {rec['es_med']:.1f}m "
                  f"Ps med {rec['ps_med']:+.2f} destroyed/scn {rec['destroyed_per_scenario']:.1f}m "
                  f"track {rec['track_pct']:.1f}%")
            save_cache(args.cache, data)  # persist as we go — interruptible

    gens = sorted(data)
    if not gens:
        print("no data", file=sys.stderr)
        return 1

    def col(k):
        return np.array([data[g][k] for g in gens])

    fig, ax = plt.subplots(2, 2, figsize=(15, 9))
    xmax = args.total_gens or max(gens)

    # --- 1. Es state ---
    a = ax[0][0]
    a.fill_between(gens, col("es_p05"), col("es_p95"), alpha=0.20, color="tab:blue",
                   label="p05–p95")
    a.plot(gens, col("es_med"), "tab:blue", lw=1.8, label="median")
    a.set_title("Specific energy $E_s$ (m above the hard deck)")
    a.set_ylabel("$E_s$ (m)")
    a.legend(fontsize=8, loc="best")

    # --- 2. Ps rate ---
    a = ax[0][1]
    a.plot(gens, col("ps_med"), "tab:green", lw=1.8, label="median $P_s$")
    a.plot(gens, col("ps_p05"), "tab:red", lw=1.3, alpha=0.85,
           label="p05 (the loss tail — what the axis charges)")
    a.axhline(0, color="gray", ls="--", lw=0.8, alpha=0.7)
    a.set_title("Specific excess power $P_s$ (m/s)")
    a.set_ylabel("$P_s$ (m/s)")
    a.legend(fontsize=8, loc="best")

    # --- 3. the objective ---
    a = ax[1][0]
    a.plot(gens, col("destroyed_per_scenario"), "tab:purple", lw=1.8)
    a.set_title("Energy DESTROYED per scenario  $\\Sigma\\max(0,-P_s)\\,dt$  (lower = better)")
    a.set_ylabel("metres of $E_s$")
    a.set_xlabel("Generation")

    # --- 4. the muting test ---
    a = ax[1][1]
    sc = a.scatter(col("track_pct"), col("destroyed_per_scenario"),
                   c=gens, cmap="viridis", s=38, zorder=3)
    fig.colorbar(sc, ax=a, label="generation")
    a.set_xlabel(f"tracking occupancy (% ticks, stpPt ≥ {TRACK_THRESHOLD})")
    a.set_ylabel("energy destroyed / scenario (m)")
    a.set_title("MUTING TEST -- want DOWN-and-RIGHT\n"
                "(down-and-LEFT = cheaper because it stopped trying = 035)",
                fontsize=10)

    for row in ax:
        for a in row:
            a.grid(True, lw=0.4, alpha=0.4)
    for a in (ax[0][0], ax[0][1], ax[1][0]):
        a.set_xlim(0, xmax)

    fig.suptitle(f"{args.label} — energy progress (041 P2-5 $P_s$ objective)",
                 fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(args.out, dpi=110)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
