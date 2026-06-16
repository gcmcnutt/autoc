#!/usr/bin/env python3
"""035 M2 closure-dynamics + sensor-utility analysis (dmp-dump source).

Ported from `specs/034-energy-objective-cleanup/intercept_analysis.py`. ALL the
panels (A–G) and their logic are unchanged; only the DATA SOURCE moved off the
retired per-step `data.dat` tail-scan onto a `dmp-dump --csv-only` CSV (a single
gen, all scenarios, comma-separated with a header). Column names are remapped
from the old data.dat contract to the new dmp-dump names (see COL_MAP).

032 phase 1 — closure-dynamics + sensor-utility analysis.

Multi-panel figure examining whether the 032 sensors (spn0 = beacon-pair NDC
span as distance proxy, dspn = one-tick diff as closure-rate proxy) actually
correlate with the NN's throttle policy, and whether they help avoid overshoots.

Operates on whatever single gen the dmp-dump CSV contains — elite-reeval where
the elite NN ran against all scenarios at the end of that generation.

ALL panels measure chaser-to-target-craft geometry (`dist` = sqrt((X−tgX)² +
(Y−tgY)² + (Z−tgZ)²)). The 10ft trailing fitness-anchor (the legacy "rabbit"
waypoint, recorded as trX/trY/trZ) is a pure fitness-shaping device — it
shapes the reward landscape but is not what the chaser physically hits, not
what the cameras see, and not what these diagnostics measure. A sanity line
at load time confirms |rabbit − target| ≈ 3.048m to verify the fitness anchor
is positioned as intended.

Panels:
  A (top, wide):    K closest-approach scenarios — dist(t) / closing-rate /
                    throttle aligned at each scenario's closest-target tick.
  B (mid-left):     outTh vs spn0 hexbin + binned-mean (distance→throttle).
  C (mid-right):    outTh vs dspn hexbin + binned-mean (closure→throttle).
  D (bot-left):     spn0 vs real dist-to-target — sensor fidelity check.
                    Visible ticks (CEP < 1.25) should trace ~const/dist.
  E (bot-right):    Encounter min-dist histogram + hull marker.
  F (4th row left): HULL-STRIKE events — dist trace anchored at strike tick,
                    should dip into <1m hull. Closing rate on twin axis.
  G (4th row right):NEAR-MISS events — close + recovered, anchored at
                    closest-target tick.

Usage:
  dmp-dump s3://autoc-m2/ --csv-only -i autoc-tracker.ini | \
      python3 intercept_analysis.py --csv - --label NAME --gen N [--out PNG]
"""

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.collections import LineCollection
import numpy as np


# dmp-dump (tracker) CSV column name -> canonical name used by the panels.
# The canonical names are the old data.dat names the panels reference. Only the
# columns the panels actually consume are mapped; data.dat-only extras
# (along/stpPt/vxBdy/vyBdy/vzBdy) are gone and the panels don't need them.
COL_MAP = {
    "px": "X", "py": "Y", "pz": "Z",
    "out_pt": "outPt", "out_rl": "outRl", "out_th": "outTh",
    "hull": "hull",
    "tgX": "tgX", "tgY": "tgY", "tgZ": "tgZ",
    "trX": "trX", "trY": "trY", "trZ": "trZ",
    "spn0": "spn0", "dspn": "dspn",
    "blC0": "blC0", "brC0": "brC0",
    "tltS": "tltS", "tltC": "tltC",
}

# Color conventions for overlay traces
CR_COLOR = "#1B5E20"        # closing-rate trace (forest green, high contrast)
TRAIL_COLOR = "tab:olive"   # 3.048m fitness-anchor reference line
HULL_COLOR = "tab:red"      # 1m hull-strike reference line

SIM_TIME_STEP_SEC = 0.05  # seconds per tick — default 20 Hz (50 ms). Set via --tick-sec
                          # (10 Hz historical runs: 0.1). Used for closing-rate m/s only.
CEP_VISIBLE_THRESH = 1.25  # beacon cep < this ⇒ visible (== kCepSentinelThreshold)
# Beacon-visibility brightness levels for the F/G traces: both tips visible = bright,
# one = half, neither = dim. (level → alpha.)
VIS_ALPHA = {2: 0.85, 1: 0.45, 0: 0.12}


def _vis_level(data, lo, hi):
    """Per-tick beacon-visibility level over [lo:hi]: 2=both tips, 1=one, 0=none.
    From blC0/brC0 (left/right beacon cep); a cep < CEP_VISIBLE_THRESH is visible.
    Returns None if the cep columns are absent."""
    if "blC0" not in data or "brC0" not in data:
        return None
    bl = np.asarray(data["blC0"])[lo:hi]
    br = np.asarray(data["brC0"])[lo:hi]
    n = min(len(bl), len(br))
    return ((bl[:n] < CEP_VISIBLE_THRESH).astype(int)
            + (br[:n] < CEP_VISIBLE_THRESH).astype(int))


def _plot_vis_graded(ax, x, y, vis, base_color, lw=1.2):
    """Plot y(x) as a polyline whose per-segment brightness (alpha) encodes beacon
    visibility (vis level 2/1/0 → bright/half/dim). Falls back to a plain line if
    vis is None. Each segment colored by the visibility at its start tick."""
    x = np.asarray(x, float); y = np.asarray(y, float)
    if vis is None or len(x) < 2:
        ax.plot(x, y, color=base_color, alpha=0.5, linewidth=lw)
        return
    rgb = mcolors.to_rgba(base_color)[:3]
    pts = np.column_stack([x, y]).reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    m = min(len(segs), len(vis))
    colors = [(*rgb, VIS_ALPHA.get(int(vis[i]), 0.45)) for i in range(m)]
    ax.add_collection(LineCollection(segs[:m], colors=colors, linewidths=lw),
                      autolim=True)
    ax.autoscale_view()


def _derive_extras(d):
    """Add dist (chaser-to-target craft) and closing_rate to a per-scenario
    column dict, in-place.

    Replaces the on-disk `dist` column (which is chaser-to-fitness-anchor =
    chaser-to-rabbit) with chaser-to-target geometry derived from absolute
    positions. The fitness anchor is purely a reward-shaping device; physical
    interactions (collisions, sensor visibility) are all against the target."""
    if all(c in d for c in ("X", "Y", "Z", "tgX", "tgY", "tgZ")):
        dx = d["X"] - d["tgX"]
        dy = d["Y"] - d["tgY"]
        dz = d["Z"] - d["tgZ"]
        d["dist_target"] = np.sqrt(dx * dx + dy * dy + dz * dz)
        # closing_rate to actual target (m/s, +ve = approaching). Forward-
        # difference places the zero-crossing one tick after argmin(dist).
        dt = d["dist_target"]
        if len(dt) > 1:
            cr = np.zeros_like(dt)
            cr[1:] = -(dt[1:] - dt[:-1]) / SIM_TIME_STEP_SEC
            d["closing_rate"] = cr


def load_dmp_dump_csv(csv_src):
    """Read a `dmp-dump --csv-only` tracker CSV (single gen, all scenarios,
    comma-separated with a header). Group rows by the `scenario` column into
    per-scenario numpy-array column dicts, remapping dmp-dump column names to
    the canonical names the panels expect (COL_MAP).

    `csv_src` is a path or '-' for stdin. data.dat-only columns the panels
    used to reference (along/stpPt/vxBdy...) are simply absent and the panels
    tolerate that."""
    if str(csv_src) == "-":
        f = sys.stdin
        close = False
    else:
        f = open(csv_src, newline="")
        close = True
    try:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            raise RuntimeError("empty CSV — no header line")
        col_idx = {name: i for i, name in enumerate(header)}
        if "scenario" not in col_idx:
            raise RuntimeError("CSV has no 'scenario' column — is this a "
                               "tracker dmp-dump --csv-only stream?")
        # which source columns map to canonical names we'll collect
        wanted = [(src, dst, col_idx[src])
                  for src, dst in COL_MAP.items() if src in col_idx]
        missing = [src for src in COL_MAP if src not in col_idx]
        if missing:
            print(f"WARNING: dmp-dump CSV missing columns {missing} — "
                  "affected panels may be sparse", file=sys.stderr)

        scn_i = col_idx["scenario"]
        per_scn = defaultdict(lambda: defaultdict(list))
        for fields in reader:
            if not fields:
                continue
            try:
                scn_key = fields[scn_i]
            except IndexError:
                continue
            for src, dst, i in wanted:
                try:
                    per_scn[scn_key][dst].append(float(fields[i]))
                except (ValueError, IndexError):
                    per_scn[scn_key][dst].append(0.0)
    finally:
        if close:
            f.close()

    if not per_scn:
        raise RuntimeError("no parseable scenario rows in dmp-dump CSV")

    out = {}
    for k, cols in per_scn.items():
        out[k] = {c: np.array(v) for c, v in cols.items()}
        _derive_extras(out[k])
    return out


def find_local_minima(d, min_separation=8, min_prominence=0.5):
    """Simple local-minimum finder with prominence + separation gates.
    Returns list of indices into d. Used for encounter histogram."""
    if len(d) < 3:
        return []
    mins = []
    for i in range(1, len(d) - 1):
        if d[i] <= d[i - 1] and d[i] <= d[i + 1]:
            mins.append(i)
    if not mins:
        return []
    # Prominence: each min must be at least min_prominence below the
    # local-max between it and the next/prev min (approximate).
    pruned = []
    for j, i in enumerate(mins):
        lo = mins[j - 1] if j > 0 else 0
        hi = mins[j + 1] if j + 1 < len(mins) else len(d) - 1
        prom = min(d[lo:i + 1].max() - d[i], d[i:hi + 1].max() - d[i])
        if prom >= min_prominence:
            pruned.append(i)
    # Separation: when two pass prominence but are too close, keep the lower.
    if not pruned:
        return []
    keep = [pruned[0]]
    for i in pruned[1:]:
        if i - keep[-1] >= min_separation:
            keep.append(i)
        elif d[i] < d[keep[-1]]:
            keep[-1] = i
    return keep


def panel_a_intercept_events(ax_dist, ax_cr, ax_th, scenarios, k=6):
    """K closest-approach events aligned at closest-target tick.
    Three stacked subplots, shared X (tick relative to closest):
      - ax_dist: dist to target craft (m)
      - ax_cr:   closing rate −d(dist)/dt to target (m/s, +ve = approaching)
      - ax_th:   throttle output

    Answers: does relative speed actually drop on approach (controller
    decelerating into the trail zone)? Does throttle drop in sympathy?
    The 3.048m fitness-anchor line marks where the fitness gradient bottoms
    out — perfect tracking sits there with closing rate ≈ 0."""
    candidates = []
    for scn, data in scenarios.items():
        d = data.get("dist_target")
        if d is None or len(d) < 20:
            continue
        min_idx = int(np.argmin(d))
        if min_idx < 5 or min_idx > len(d) - 5:
            continue
        candidates.append((float(d[min_idx]), scn, min_idx))
    candidates.sort()
    picks = candidates[:k]

    if not picks:
        for ax in (ax_dist, ax_cr, ax_th):
            ax.text(0.5, 0.5, "no closure events with >=20 ticks",
                    transform=ax.transAxes, ha="center", va="center")
        return

    colors = plt.cm.tab10(np.linspace(0, 1, len(picks)))
    for (mind, scn, min_idx), color in zip(picks, colors):
        data = scenarios[scn]
        d = data["dist_target"]
        n = len(d)
        lo = max(0, min_idx - 20)
        hi = min(n, min_idx + 21)
        t_rel = np.arange(lo, hi) - min_idx
        ax_dist.plot(t_rel, d[lo:hi], color=color, alpha=0.85, linewidth=1.4,
                     label=f"scn={scn} min-d={mind:.2f}m")
        if "closing_rate" in data and len(data["closing_rate"]) >= hi:
            ax_cr.plot(t_rel, data["closing_rate"][lo:hi], color=color,
                       alpha=0.8, linewidth=1.4)
        if "outTh" in data and len(data["outTh"]) >= hi:
            ax_th.plot(t_rel, data["outTh"][lo:hi], color=color, alpha=0.8,
                       linewidth=1.4)

    for ax in (ax_dist, ax_cr, ax_th):
        ax.axvline(0, color="k", linewidth=0.5, alpha=0.4)
        ax.grid(True, alpha=0.3)

    ax_dist.axhline(3.048, color=TRAIL_COLOR, linewidth=0.8, alpha=0.7,
                    linestyle="-.", label="fitness anchor 3.048m")
    ax_dist.axhline(1.0, color=HULL_COLOR, linewidth=0.8, alpha=0.7,
                    linestyle="-.", label="hull radius 1m")
    ax_dist.set_ylabel("dist to target (m)")
    ax_dist.set_title(f"Panel A — {len(picks)} closest-approach events (t=0 at closest-target tick)",
                      fontsize=10)
    ax_dist.legend(loc="upper right", fontsize=7, ncol=2)

    ax_cr.set_ylabel("closing rate to target (m/s, +ve = approaching)")
    ax_cr.axhline(0, color="gray", linewidth=0.5, alpha=0.4)

    ax_th.set_ylabel("outTh (NN throttle)")
    ax_th.axhline(0, color="gray", linewidth=0.4, alpha=0.4)
    ax_th.set_ylim(-1.05, 1.05)
    ax_th.set_xlabel("tick (0 = closest-target approach)")


def panel_outTh_vs_sensor(ax, x_all, outTh_all, xlabel, title, n_bins=20):
    """Throttle-only hexbin + binned mean + ±1σ ribbon vs a sensor."""
    if len(x_all) == 0:
        ax.text(0.5, 0.5, "no data", transform=ax.transAxes, ha="center")
        return
    x_all = np.asarray(x_all)
    y_all = np.asarray(outTh_all)
    mask = np.isfinite(x_all) & np.isfinite(y_all)
    x_all, y_all = x_all[mask], y_all[mask]
    if len(x_all) < 10:
        ax.text(0.5, 0.5, "too few points", transform=ax.transAxes, ha="center")
        return
    # Clip to interpretable range
    xlo, xhi = np.percentile(x_all, 1), np.percentile(x_all, 99)
    cm = (x_all >= xlo) & (x_all <= xhi)
    x_all, y_all = x_all[cm], y_all[cm]

    ax.hexbin(x_all, y_all, gridsize=40, cmap="Blues", mincnt=1, alpha=0.55)
    bins = np.linspace(xlo, xhi, n_bins + 1)
    bin_centers = 0.5 * (bins[:-1] + bins[1:])
    means = np.full(n_bins, np.nan)
    stds = np.full(n_bins, np.nan)
    for i in range(n_bins):
        in_bin = (x_all >= bins[i]) & (x_all < bins[i + 1])
        if in_bin.sum() >= 10:
            means[i] = y_all[in_bin].mean()
            stds[i] = y_all[in_bin].std()
    valid = np.isfinite(means)
    if valid.sum() > 1:
        ax.plot(bin_centers[valid], means[valid], color="tab:red",
                linewidth=2.0, label="binned mean")
        ax.fill_between(bin_centers[valid], means[valid] - stds[valid],
                        means[valid] + stds[valid], color="tab:red",
                        alpha=0.15, label="±1σ")
    ax.axhline(0, color="k", linewidth=0.4, alpha=0.4)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("outTh (NN throttle)")
    ax.set_title(title, fontsize=10)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-1.05, 1.05)


def panel_e_encounter_histogram(ax, scenarios):
    """Histogram of ALL encounter min-distances to target. An 'encounter' is
    a local minimum in dist (≥0.5m prominence, ≥8 ticks separation). One
    scenario can have several encounters (lap structure of figure-eight
    paths). Counts events, not scenarios. Hull strikes populate the <1m tail."""
    encounter_dists = []
    for scn, data in scenarios.items():
        d = data.get("dist_target")
        if d is None or len(d) < 10:
            continue
        for i in find_local_minima(d, min_separation=8, min_prominence=0.5):
            encounter_dists.append(float(d[i]))
    if not encounter_dists:
        ax.text(0.5, 0.5, "no encounters found", transform=ax.transAxes,
                ha="center")
        return
    arr = np.array(encounter_dists)
    ax.hist(arr, bins=40, color="tab:blue", alpha=0.7, edgecolor="black",
            linewidth=0.4)
    ax.axvline(1.0, color=HULL_COLOR, linewidth=1.4, linestyle="-",
               label="hull radius 1.0m")
    ax.axvline(3.048, color=TRAIL_COLOR, linewidth=1.4, linestyle="-.",
               label="fitness anchor 3.048m")
    pct_in_hull = (arr < 1.0).mean() * 100
    pct_in_trail = (arr < 3.048).mean() * 100
    ax.set_xlabel("encounter min-dist to target (m)")
    ax.set_ylabel("encounter count")
    ax.set_title(f"Panel E — {len(arr)} encounters across all scenarios: "
                 f"{pct_in_hull:.0f}% inside hull, {pct_in_trail:.0f}% inside trail",
                 fontsize=10)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)


def classify_scenarios(scenarios):
    """Classify each scenario as 'hull_strike', 'near_miss', or 'normal'.
       - hull_strike: terminated at hull=1 (final tick inside hull AND scenario
         ended shorter than typical). Hull flag is authoritative.
       - near_miss:   min dist_target < 1m but did NOT terminate at hull=1.
       - normal:      min dist_target >= 1m.
    Returns dict scn_key -> classification."""
    out = {}
    lengths = [len(d.get("dist_target", [])) for d in scenarios.values()
               if "dist_target" in d]
    if not lengths:
        return out
    typical_len = int(np.median(lengths))
    for scn, data in scenarios.items():
        d = data.get("dist_target")
        if d is None or len(d) < 3:
            out[scn] = "normal"
            continue
        n = len(d)
        min_dist_tgt = float(d.min())
        ended_in_hull = False
        if "hull" in data and len(data["hull"]) > 0:
            ended_in_hull = data["hull"][-1] > 0.5 and n < typical_len * 0.95
        if ended_in_hull:
            out[scn] = "hull_strike"
        elif min_dist_tgt < 1.0:
            out[scn] = "near_miss"
        else:
            out[scn] = "normal"
    return out


def panel_event_overlay(ax, scenarios, picks, align_mode, title, color):
    """Overlay dist-to-target traces for a list of scenarios.

    align_mode == 'last_tick' (Panel F, hull strikes):
        Anchor at final tick (the strike). Window = last 40 ticks.
    align_mode == 'min_dist'  (Panel G, near-misses):
        Anchor at closest-target tick. Window = ±20 ticks.

    Closing rate (to target) on a twin axis in both modes.
    Returns count plotted."""
    if not picks:
        ax.text(0.5, 0.5, "no events of this class", transform=ax.transAxes,
                ha="center", va="center")
        ax.set_title(title, fontsize=10)
        return 0
    ax2 = ax.twinx()
    n_plotted = 0
    for scn in picks:
        data = scenarios[scn]
        d = data.get("dist_target")
        if d is None:
            continue
        n = len(d)
        if align_mode == "last_tick":
            anchor = n - 1
            lo = max(0, anchor - 40)
            hi = n
        else:  # min_dist
            anchor = int(np.argmin(d))
            if anchor < 3 or anchor > n - 3:
                continue
            lo = max(0, anchor - 20)
            hi = min(n, anchor + 21)
        t_rel = np.arange(lo, hi) - anchor
        # dist trace, brightness-graded by beacon visibility (bright=both tips,
        # dim=none) — shows whether we lose sight near the strike / closest approach.
        _plot_vis_graded(ax, t_rel, d[lo:hi], _vis_level(data, lo, hi), color, lw=1.1)
        if "closing_rate" in data and len(data["closing_rate"]) >= hi:
            ax2.plot(t_rel, data["closing_rate"][lo:hi], color=CR_COLOR,
                     alpha=0.40, linewidth=0.9, linestyle=":")
        n_plotted += 1
    ax.axvline(0, color="k", linewidth=0.6, alpha=0.5)
    ax.axhline(1.0, color=HULL_COLOR, linewidth=0.8, alpha=0.7, linestyle="-.",
               label="hull 1m")
    ax.axhline(3.048, color=TRAIL_COLOR, linewidth=0.8, alpha=0.7,
               linestyle="-.", label="fitness anchor 3.048m")
    ax2.axhline(0, color="gray", linewidth=0.4, alpha=0.4)
    # visibility-brightness legend proxies (bright=both beacons, half=one, dim=none)
    from matplotlib.lines import Line2D
    rgb = mcolors.to_rgba(color)[:3]
    vis_handles = [Line2D([0], [0], color=(*rgb, VIS_ALPHA[2]), lw=2, label="both vis"),
                   Line2D([0], [0], color=(*rgb, VIS_ALPHA[1]), lw=2, label="one vis"),
                   Line2D([0], [0], color=(*rgb, VIS_ALPHA[0]), lw=2, label="none vis")]
    anchor_label = "strike" if align_mode == "last_tick" else "closest-target"
    ax.set_xlabel(f"tick (0 = {anchor_label})")
    ax.set_ylabel("dist to target (m); line brightness = beacon vis", color=color)
    ax2.set_ylabel("closing rate to target (m/s, dotted)", color=CR_COLOR)
    ax.tick_params(axis="y", labelcolor=color)
    ax2.tick_params(axis="y", labelcolor=CR_COLOR)
    ax.set_title(f"{title} — n={n_plotted}", fontsize=10)
    h, _ = ax.get_legend_handles_labels()
    ax.legend(handles=h + vis_handles, loc="upper right", fontsize=6, ncol=2)
    ax.grid(True, alpha=0.3)
    return n_plotted


def panel_d_sensor_fidelity(ax, scenarios):
    """spn0 vs real dist-to-TARGET scatter (Y axis log-scaled).
    Beacons sit on the target craft, so spn0 ≈ k/dist_target for visible
    ticks → straight line on log-Y. (Previously plotted vs dist_to_rabbit,
    which was biased by the constant ~3m trail-distance offset.)
    CEP-gated ticks (spn=0) cannot be log-plotted; counted instead."""
    spn_all = []
    dist_all = []
    vis_all = []
    for scn, data in scenarios.items():
        if "spn0" not in data or "dist_target" not in data:
            continue
        n = len(data["spn0"])
        spn_all.append(data["spn0"])
        dist_all.append(data["dist_target"][:n])
        if "blC0" in data and "brC0" in data:
            visible = (data["blC0"][:n] < CEP_VISIBLE_THRESH) & (data["brC0"][:n] < CEP_VISIBLE_THRESH)
            vis_all.append(visible.astype(float))
        else:
            vis_all.append(np.ones(n))
    if not spn_all:
        ax.text(0.5, 0.5, "no spn/dist data", transform=ax.transAxes, ha="center")
        return
    spn_all = np.concatenate(spn_all)
    dist_all = np.concatenate(dist_all)
    vis_all = np.concatenate(vis_all)
    vis_mask = vis_all > 0.5
    n_gated = int((~vis_mask).sum())

    vis_d = dist_all[vis_mask]
    vis_s = spn_all[vis_mask]
    plot_mask = (vis_s > 0.001) & (vis_d > 0)
    ax.scatter(vis_d[plot_mask], vis_s[plot_mask], s=2, alpha=0.18,
               c="tab:blue", label=f"visible n={plot_mask.sum()}")

    if plot_mask.sum() > 100:
        k = np.median(vis_d[plot_mask] * vis_s[plot_mask])
        d_curve = np.linspace(max(0.5, vis_d[plot_mask].min()),
                              np.percentile(vis_d[plot_mask], 99), 200)
        ax.plot(d_curve, k / d_curve, "--", color=TRAIL_COLOR, linewidth=1.4,
                alpha=0.85, label=f"k/dist fit (k≈{k:.2f})")

    ax.set_yscale("log")
    ax.set_xlabel("real dist to target (m)")
    ax.set_ylabel("spn0 (NDC, log scale)")
    ax.set_xlim(0, min(80, np.percentile(dist_all, 99)))
    title_parts = ["Panel D — sensor fidelity: spn0 vs dist-to-target (log Y)"]
    if n_gated > 0:
        title_parts.append(f"  ({n_gated} CEP-gated ticks not shown — spn=0)")
    ax.set_title("".join(title_parts), fontsize=10)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3, which="both")


def panel_e_min_dist_histogram(ax, scenarios):
    """Per-scenario min-dist-to-TARGET histogram (alternate Panel E variant
    — currently unused in main(), but kept available)."""
    min_dists = []
    for scn, data in scenarios.items():
        d = data.get("dist_target")
        if d is None or len(d) < 5:
            continue
        min_dists.append(float(np.min(d)))
    if not min_dists:
        ax.text(0.5, 0.5, "no scenarios with dist", transform=ax.transAxes,
                ha="center")
        return
    min_dists = np.array(min_dists)
    ax.hist(min_dists, bins=40, color="tab:blue", alpha=0.7, edgecolor="black",
            linewidth=0.4)
    ax.axvline(1.0, color=HULL_COLOR, linewidth=1.4, linestyle="-",
               label="hull radius 1.0m (target)")
    ax.axvline(3.048, color=TRAIL_COLOR, linewidth=1.4, linestyle="-.",
               label="trail dist 3.048m")
    pct_in_hull = (min_dists < 1.0).mean() * 100
    pct_in_trail = (min_dists < 3.048).mean() * 100
    ax.set_xlabel("min dist to target per scenario (m)")
    ax.set_ylabel("scenario count")
    ax.set_title(f"Panel E — {len(min_dists)} scenarios: "
                 f"{pct_in_hull:.0f}% < hull, {pct_in_trail:.0f}% < trail",
                 fontsize=10)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)


def main():
    global SIM_TIME_STEP_SEC          # set from --tick-sec; closing-rate (m/s) divides by this
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", dest="csv", default="-",
                    help="dmp-dump --csv-only file, or '-' for stdin (default)")
    ap.add_argument("--gen", type=int, default=-1,
                    help="gen number for titles (display-only; -1 = unknown)")
    ap.add_argument("--label", default="035-m2-tracker")
    ap.add_argument("--k-scenarios", type=int, default=6,
                    help="Number of closest-approach scenarios to overlay in panel A")
    ap.add_argument("-o", "--out", type=Path, default=None,
                    help="default: specs/035-energy-lexicase-objective/<label>_intercept_analysis.png")
    ap.add_argument("--tick-sec", type=float, default=SIM_TIME_STEP_SEC,
                    help="seconds per control tick for closing-rate m/s (default 0.05 = 20 Hz; "
                         "pass 0.1 for 10 Hz runs)")
    args = ap.parse_args()
    if args.out is None:  # derive from --label (lexicographic autoc-035-t<N>-<reason>)
        args.out = Path(f"specs/035-energy-lexicase-objective/{args.label}_intercept_analysis.png")

    SIM_TIME_STEP_SEC = args.tick_sec   # closing-rate (m/s) divides by this; must match run cadence

    print(f"Reading dmp-dump CSV from {args.csv} (tick={SIM_TIME_STEP_SEC}s)...", file=sys.stderr)
    scenarios = load_dmp_dump_csv(args.csv)
    gen_str = str(args.gen) if args.gen >= 0 else "?"
    print(f"  gen = {gen_str}, scenarios = {len(scenarios)}", file=sys.stderr)

    # Sanity: rabbit should sit ~trail_distance (3.048m / 10ft) behind target
    rab_offsets = []
    for data in scenarios.values():
        if all(c in data for c in ("trX", "trY", "trZ", "tgX", "tgY", "tgZ")) \
                and len(data["trX"]) > 0:
            rdx = data["trX"] - data["tgX"]
            rdy = data["trY"] - data["tgY"]
            rdz = data["trZ"] - data["tgZ"]
            rab_offsets.append(np.sqrt(rdx*rdx + rdy*rdy + rdz*rdz))
    if rab_offsets:
        roff = np.concatenate(rab_offsets)
        print(f"  rabbit↔target offset: mean={roff.mean():.3f}m "
              f"[min={roff.min():.3f}, max={roff.max():.3f}] "
              f"(expected ≈3.048m / 10ft trail)", file=sys.stderr)
    else:
        print("  WARNING: no trX/tgX columns found — dist_target unavailable",
              file=sys.stderr)

    # Aggregate columns for throttle-vs-sensor panels
    spn_all, dspn_all, outTh_all = [], [], []
    for data in scenarios.values():
        if "spn0" not in data or "outTh" not in data:
            continue
        n = min(len(data["spn0"]), len(data.get("dspn", [])), len(data["outTh"]))
        if n == 0:
            continue
        spn_all.append(data["spn0"][:n])
        dspn_all.append(data["dspn"][:n])
        outTh_all.append(data["outTh"][:n])
    spn_all = np.concatenate(spn_all) if spn_all else np.array([])
    dspn_all = np.concatenate(dspn_all) if dspn_all else np.array([])
    outTh_all = np.concatenate(outTh_all) if outTh_all else np.array([])

    # Classify scenarios for hull-strike vs near-miss comparison
    classifications = classify_scenarios(scenarios)
    hull_picks = [s for s, c in classifications.items() if c == "hull_strike"]
    miss_picks = [s for s, c in classifications.items() if c == "near_miss"]

    # Figure: 4 rows × 2 cols
    #   Row 0 (wide): Panel A — 3-row mini-stack dist/closing/throttle
    #   Row 1:        B (outTh vs spn0)        | C (outTh vs dspn)
    #   Row 2:        D (sensor fidelity log-Y)| E (encounter histogram)
    #   Row 3:        F (hull-strike events)    | G (near-miss events)
    fig = plt.figure(figsize=(15, 18))
    gs = fig.add_gridspec(4, 2, height_ratios=[1.6, 1, 1, 1], hspace=0.32,
                          wspace=0.28)
    sub_top = gs[0, :].subgridspec(3, 1, hspace=0.10, height_ratios=[1, 1, 1])
    ax_a_dist = fig.add_subplot(sub_top[0])
    ax_a_cr = fig.add_subplot(sub_top[1], sharex=ax_a_dist)
    ax_a_th = fig.add_subplot(sub_top[2], sharex=ax_a_dist)
    plt.setp(ax_a_dist.get_xticklabels(), visible=False)
    plt.setp(ax_a_cr.get_xticklabels(), visible=False)
    ax_b = fig.add_subplot(gs[1, 0])
    ax_c = fig.add_subplot(gs[1, 1])
    ax_d = fig.add_subplot(gs[2, 0])
    ax_e = fig.add_subplot(gs[2, 1])
    ax_f = fig.add_subplot(gs[3, 0])
    ax_g = fig.add_subplot(gs[3, 1])

    panel_a_intercept_events(ax_a_dist, ax_a_cr, ax_a_th, scenarios,
                             k=args.k_scenarios)
    panel_outTh_vs_sensor(ax_b, spn_all, outTh_all,
                          "spn0 (NDC, distance proxy: bigger = closer)",
                          "Panel B — outTh vs spn0 (distance→throttle policy)")
    panel_outTh_vs_sensor(ax_c, dspn_all, outTh_all,
                          "dspn (closure-rate proxy: +ve = approaching)",
                          "Panel C — outTh vs dspn (closure→throttle policy)")
    panel_d_sensor_fidelity(ax_d, scenarios)
    panel_e_encounter_histogram(ax_e, scenarios)
    panel_event_overlay(ax_f, scenarios, hull_picks, "last_tick",
                        "Panel F — HULL-STRIKE events", "tab:red")
    panel_event_overlay(ax_g, scenarios, miss_picks, "min_dist",
                        "Panel G — NEAR-MISS events (close + recovered)",
                        "tab:blue")

    fig.suptitle(f"{args.label} — closure-dynamics + sensor-utility analysis  (gen {gen_str}, elite)",
                 fontsize=12, y=0.995)
    fig.savefig(args.out, dpi=120, bbox_inches="tight")
    print(f"wrote {args.out}", file=sys.stderr)


if __name__ == "__main__":
    main()
