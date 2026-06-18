"""Wind-environment mismatch study: M1 source vs M2 chase, per scenario.

Motivation (operator 2026-06-16): M2 (tracker) replays an M1 source trajectory
as the target, but the chase flies its OWN per-scenario wind draw — NOT the wind
the source flew through. Wind variation is direction-only (`windDirectionOffset`,
drawn ~N(0, WindDirectionSigma)); there is no per-scenario speed draw, so speed is
the shared base. With independent draws at the same sigma, M1 and M2 wind
DIRECTIONS diverge — a path flown for one wind can become physically infeasible
for the chase under a wind rotated tens of degrees. This quantifies that gap and
tests whether a bigger wind mismatch predicts worse chase tracking.

Two deltas:
  - FULL    : |off_M1 − off_M2|                       (intrinsic design mismatch)
  - APPLIED : |off_M1·ramp_M1 − off_M2·ramp_M2|       (what the chase feels NOW;
              the variation ramp scales the stored full draw per gen)

Inputs are two `dmp-dump --meta-only` YAML dumps (carry per-scenario
`wind_dir_offset_deg`, `path_variant`, `wind_variant`, `max_streak`, `score`,
`crash_reason`, and a top-level `ramp_scale`). Stdlib + numpy + matplotlib.
"""
import argparse
import re
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_meta(path):
    """Return (ramp_scale, list-of-per-scenario-dicts) from a --meta-only YAML."""
    ramp = 1.0
    scns = []
    cur = None
    with open(path) as f:
        for line in f:
            m = re.match(r"\s*ramp_scale:\s*([-0-9.eE]+)", line)
            if m and not cur:
                ramp = float(m.group(1))
            if re.match(r"\s*-\s*idx:\s*\d+", line):
                if cur:
                    scns.append(cur)
                cur = {"idx": int(re.search(r"idx:\s*(\d+)", line).group(1))}
                continue
            if cur is None:
                continue
            for key, cast in (("path_variant", int), ("wind_variant", int),
                              ("wind_dir_offset_deg", float), ("max_streak", int),
                              ("score", float)):
                mm = re.match(rf"\s*{key}:\s*([-0-9.eE]+)", line)
                if mm:
                    cur[key] = cast(mm.group(1))
            mc = re.match(r"\s*crash_reason:\s*(\S+)", line)
            if mc:
                cur["crash_reason"] = mc.group(1)
    if cur:
        scns.append(cur)
    return ramp, scns


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--m1-meta", required=True, help="source (M1) --meta-only YAML")
    p.add_argument("--m2-meta", required=True, help="chase (M2) --meta-only YAML")
    p.add_argument("--label", default="M2 vs M1 wind")
    p.add_argument("-o", "--out", required=True)
    args = p.parse_args()

    r1, s1 = parse_meta(args.m1_meta)
    r2, s2 = parse_meta(args.m2_meta)
    # Join on (path_variant, wind_variant) — the shared scenario slot.
    by_key1 = {(d["path_variant"], d["wind_variant"]): d for d in s1
               if "path_variant" in d and "wind_variant" in d}
    rows = []
    for d2 in s2:
        k = (d2.get("path_variant"), d2.get("wind_variant"))
        d1 = by_key1.get(k)
        if d1 is None:
            continue
        off1, off2 = d1["wind_dir_offset_deg"], d2["wind_dir_offset_deg"]
        rows.append({
            "key": k,
            "off1": off1, "off2": off2,
            "dfull": abs(wrap180(off1 - off2)),
            "dappl": abs(wrap180(off1 * r1 - off2 * r2)),
            "m2_streak": d2.get("max_streak", 0),
            "m2_score": d2.get("score", float("nan")),
            "m2_crash": d2.get("crash_reason", "none"),
        })
    if not rows:
        sys.exit("no joined scenarios — check (path_variant, wind_variant) keys")

    dfull = np.array([r["dfull"] for r in rows])
    dappl = np.array([r["dappl"] for r in rows])
    streak = np.array([r["m2_streak"] for r in rows], float)
    score = np.array([r["m2_score"] for r in rows], float)   # higher (less negative) = better
    n = len(rows)

    def stats(a):
        return f"mean {a.mean():.1f}°  med {np.median(a):.1f}°  p95 {np.percentile(a,95):.1f}°  max {a.max():.1f}°"
    print(f"=== wind mismatch: {args.label}  (n={n} joined scenarios) ===")
    print(f"  ramp_scale: M1={r1:.3f}  M2={r2:.3f}   (WindDirectionSigma equal both)")
    print(f"  |Δdir| FULL  (intrinsic): {stats(dfull)}")
    print(f"  |Δdir| APPLIED (now)    : {stats(dappl)}")
    print(f"  scenarios with FULL |Δdir| > 45°: {100*np.mean(dfull>45):.0f}%   > 90°: {100*np.mean(dfull>90):.0f}%")

    def corr(a, b):
        if a.std() == 0 or b.std() == 0:
            return float("nan")
        return float(np.corrcoef(a, b)[0, 1])
    # PRIMARY question: does a bigger wind mismatch ⇒ worse per-scenario SCORE?
    cs_appl, cs_full = corr(dappl, score), corr(dfull, score)
    ck_appl = corr(dappl, streak)
    print(f"  corr(|Δdir applied|, M2 score)      = {cs_appl:+.2f}   (negative ⇒ more mismatch → worse score)")
    print(f"  corr(|Δdir full|,    M2 score)      = {cs_full:+.2f}")
    print(f"  corr(|Δdir applied|, M2 max_streak) = {ck_appl:+.2f}")
    # Good-vs-bad split: do the BEST-scoring scenarios have SIMILAR wind?
    order = np.argsort(score)               # ascending: worst → best
    q = max(1, n // 4)
    worst, best = order[:q], order[-q:]
    print(f"  best-quartile score: mean |Δdir applied| {dappl[best].mean():.1f}°  "
          f"(score≈{score[best].mean():.0f})")
    print(f"  worst-quartile score: mean |Δdir applied| {dappl[worst].mean():.1f}°  "
          f"(score≈{score[worst].mean():.0f})")
    print("  → if best-quartile |Δdir| < worst-quartile, good scenarios = similar wind (hypothesis holds)")
    crashed = [r for r in rows if r["m2_crash"] != "RabbitComplete"]
    if crashed:
        cd = np.array([r["dappl"] for r in crashed])
        print(f"  crashed: {len(crashed)}/{n}  mean |Δdir applied| {cd.mean():.1f}° vs overall {dappl.mean():.1f}°")
    print("  caveat: craft variation also differs per scenario (a confound); wind is the lead hypothesis.")

    # ---- plot ----
    fig, axs = plt.subplots(1, 3, figsize=(16, 4.6))
    fig.suptitle(f"{args.label} — per-scenario wind-direction mismatch (M1 source vs M2 chase)\n"
                 f"n={n}, ramp M1={r1:.2f}/M2={r2:.2f}; wind variation is direction-only (σ shared), drawn independently",
                 fontsize=10)

    ax = axs[0]
    ax.hist(dfull, bins=24, alpha=0.6, label=f"FULL (intrinsic) mean {dfull.mean():.0f}°", color="tab:red")
    ax.hist(dappl, bins=24, alpha=0.6, label=f"APPLIED (now) mean {dappl.mean():.0f}°", color="tab:blue")
    ax.axvline(45, color="k", ls="--", lw=0.8, label="σ=45°")
    ax.set_xlabel("|Δ wind direction| (deg)"); ax.set_ylabel("scenarios")
    ax.legend(fontsize=8); ax.set_title("mismatch distribution", fontsize=10)

    ax = axs[1]
    cols = ["tab:red" if r["m2_crash"] != "RabbitComplete" else "tab:green" for r in rows]
    ax.scatter(dappl, score, c=cols, s=14, alpha=0.7)
    ax.set_xlabel("|Δ wind direction| applied (deg)"); ax.set_ylabel("M2 per-scenario score (higher=better)")
    ax.set_title(f"mismatch vs SCORE  (corr {cs_appl:+.2f}; red=crashed)", fontsize=10)
    if dappl.std() > 0 and np.isfinite(score).all():
        b = np.polyfit(dappl, score, 1)
        xs = np.array([dappl.min(), dappl.max()])
        ax.plot(xs, b[0]*xs + b[1], color="k", lw=1.0, ls=":")

    ax = axs[2]
    ax.scatter(np.array([r["off1"] for r in rows]), np.array([r["off2"] for r in rows]),
               s=14, alpha=0.6, color="tab:purple")
    lim = max(abs(np.array([r["off1"] for r in rows])).max(),
              abs(np.array([r["off2"] for r in rows])).max()) * 1.1
    ax.plot([-lim, lim], [-lim, lim], color="k", lw=0.8, ls="--", label="parity (same wind)")
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim)
    ax.set_xlabel("M1 source wind offset (deg)"); ax.set_ylabel("M2 chase wind offset (deg)")
    ax.set_title("per-scenario draws (off-diagonal = mismatch)", fontsize=10)
    ax.legend(fontsize=8)

    for ax in axs:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
