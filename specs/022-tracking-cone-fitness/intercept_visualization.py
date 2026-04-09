#!/usr/bin/env python3
"""
Intercept tracking visualization for the final-gen NN.

Reads data.dat for the latest scenario ID (best individual at gen N) and
produces a multi-panel plot showing:

1. Lock duration histogram — distribution of streak lengths achieved
2. Time-in-cone fraction per path — % of steps inside streak threshold
3. Along/lateral position scatter — where the aircraft tracks (relative to rabbit)
4. Z position relative to path — gravity bias check (above vs below)
5. Lock-and-recover behavior — streak vs gap pattern over time
6. Per-path summary table — score, mean dist, mean strk

This shows "when this thing is tried, how close will we be?"
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict


# Column indices in data.dat (1-based per header inspection)
# Header: Scn Bake Pth/Wnd:Step:  Time Idx  dPh-9..ds+5 dd/dt qw qx qy qz vel gyrP gyrQ gyrR outPt outRl outTh pathX pathY pathZ X Y Z vxBdy vyBdy vzBdy dhome dist along rabVl stpPt mult rampSc
COL_PWS = 2  # 0-based: "ppp/wwww:ssss:" — field 3 in awk
COL_X = 38   # 0-based: aircraft virtual X
COL_Y = 39
COL_Z = 40
COL_PATH_X = 35
COL_PATH_Y = 36
COL_PATH_Z = 37
COL_DIST = 45
COL_ALONG = 46
COL_STPPT = 48
COL_MULT = 49

DATA_FILE = '/home/gmcnutt/autoc/data.dat'
OUT_PATH = '/home/gmcnutt/autoc/specs/022-tracking-cone-fitness/intercept_visualization.png'
STREAK_THRESHOLD = 0.5

PATH_NAMES = ['StraightAndLevel', 'SpiralClimb', 'HorizontalFigureEight',
              'FortyFiveAngledLoop', 'HighPerchSplitS']


def load_latest_scenario():
    """Load only the latest scenario ID's rows (last training generation logged)."""
    print(f"Reading {DATA_FILE} (~5GB) — finding latest scn ID...")

    # First pass: find the maximum scn ID (lexicographic max == numeric max for fixed-width)
    max_scn = None
    with open(DATA_FILE, 'r') as f:
        # Skip header
        next(f)
        for line in f:
            parts = line.split()
            if len(parts) < 2 or not parts[0][0].isdigit():
                continue
            scn = parts[0]
            if max_scn is None or scn > max_scn:
                max_scn = scn

    print(f"Latest scn ID: {max_scn}")

    # Second pass: collect rows for that scn
    rows = []
    with open(DATA_FILE, 'r') as f:
        next(f)  # skip header
        for line in f:
            parts = line.split()
            if len(parts) < 50 or parts[0] != max_scn:
                continue
            rows.append(parts)

    print(f"Loaded {len(rows)} rows for scenario {max_scn}")
    return max_scn, rows


def parse_pws(pws_field):
    """Parse 'ppp/wwww:ssss:' into (path, wind, step)."""
    # Format: "000/00:0001:"
    pws = pws_field.rstrip(':')
    path_str, rest = pws.split('/', 1)
    wind_str, step_str = rest.split(':', 1)
    return int(path_str), int(wind_str), int(step_str)


def main():
    max_scn, rows = load_latest_scenario()

    # Per-arena (path, wind) buckets
    arenas = defaultdict(list)  # (path, wind) -> list of step records
    for r in rows:
        try:
            path, wind, step = parse_pws(r[COL_PWS])
        except (ValueError, IndexError):
            continue
        record = {
            'step': step,
            'x': float(r[COL_X]),
            'y': float(r[COL_Y]),
            'z': float(r[COL_Z]),
            'path_x': float(r[COL_PATH_X]),
            'path_y': float(r[COL_PATH_Y]),
            'path_z': float(r[COL_PATH_Z]),
            'dist': float(r[COL_DIST]),
            'along': float(r[COL_ALONG]),
            'stpPt': float(r[COL_STPPT]),
            'mult': float(r[COL_MULT]),
        }
        arenas[(path, wind)].append(record)

    print(f"Parsed {len(arenas)} arenas")

    # ========================================================================
    # Compute per-arena and per-path statistics
    # ========================================================================
    per_path_stats = defaultdict(lambda: {
        'dist': [], 'along': [], 'lateral': [], 'z_offset': [],
        'stpPt': [], 'mult': [], 'in_streak': 0, 'total': 0,
        'streak_runs': [], 'arena_count': 0,
    })

    all_streak_lengths = []  # Across everything
    per_path_streak_lengths = defaultdict(list)

    for (path, wind), recs in arenas.items():
        per_path_stats[path]['arena_count'] += 1

        # Walk this arena, computing per-step quantities and tracking streak runs
        current_streak = 0
        for r in recs:
            # Lateral component: dist² = along² + lateral², so lateral = sqrt(dist² - along²)
            lat_sq = max(0, r['dist'] ** 2 - r['along'] ** 2)
            lateral = np.sqrt(lat_sq)
            # Aircraft Z minus path Z (positive = below path in NED)
            z_off = r['z'] - r['path_z']

            per_path_stats[path]['dist'].append(r['dist'])
            per_path_stats[path]['along'].append(r['along'])
            per_path_stats[path]['lateral'].append(lateral)
            per_path_stats[path]['z_offset'].append(z_off)
            per_path_stats[path]['stpPt'].append(r['stpPt'])
            per_path_stats[path]['mult'].append(r['mult'])
            per_path_stats[path]['total'] += 1

            # Streak detection: stpPt >= threshold
            if r['stpPt'] >= STREAK_THRESHOLD:
                current_streak += 1
                per_path_stats[path]['in_streak'] += 1
            else:
                if current_streak > 0:
                    per_path_stats[path]['streak_runs'].append(current_streak)
                    per_path_streak_lengths[path].append(current_streak)
                    all_streak_lengths.append(current_streak)
                current_streak = 0
        # End-of-arena streak
        if current_streak > 0:
            per_path_stats[path]['streak_runs'].append(current_streak)
            per_path_streak_lengths[path].append(current_streak)
            all_streak_lengths.append(current_streak)

    # ========================================================================
    # Build the figure
    # ========================================================================
    fig = plt.figure(figsize=(18, 12))
    gs = fig.add_gridspec(3, 3, hspace=0.4, wspace=0.35)

    # ----------------------------------------------------------------
    # Panel 1 (top-left): Lock duration histogram across all paths
    # ----------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, 0])
    if all_streak_lengths:
        bins = np.arange(0, max(50, max(all_streak_lengths)) + 5, 2)
        ax1.hist(all_streak_lengths, bins=bins, color='steelblue', edgecolor='navy')
        ax1.axvline(50, color='red', linestyle='--', label='ramp cap (50)')
        ax1.set_xlabel('Streak length (consecutive steps locked)')
        ax1.set_ylabel('Count')
        ax1.set_title(f'Lock duration distribution (all paths)\n{len(all_streak_lengths)} streak runs total')
        ax1.legend()
        ax1.grid(alpha=0.3)

    # ----------------------------------------------------------------
    # Panel 2 (top-middle): Time-in-cone per path
    # ----------------------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 1])
    paths_sorted = sorted(per_path_stats.keys())
    pct_in_streak = [per_path_stats[p]['in_streak'] / per_path_stats[p]['total'] * 100
                      for p in paths_sorted]
    colors = ['#2ecc71' if pct > 40 else ('#f39c12' if pct > 20 else '#e74c3c')
              for pct in pct_in_streak]
    bars = ax2.bar([f'P{p}' for p in paths_sorted], pct_in_streak, color=colors)
    for bar, pct in zip(bars, pct_in_streak):
        ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                 f'{pct:.0f}%', ha='center', fontsize=9)
    ax2.set_ylabel('% steps in streak (locked)')
    ax2.set_title('Time-in-cone per path')
    ax2.set_ylim(0, 100)
    ax2.grid(alpha=0.3, axis='y')

    # ----------------------------------------------------------------
    # Panel 3 (top-right): Mean lock length per path
    # ----------------------------------------------------------------
    ax3 = fig.add_subplot(gs[0, 2])
    mean_lock = [np.mean(per_path_streak_lengths[p]) if per_path_streak_lengths[p] else 0
                 for p in paths_sorted]
    max_lock = [max(per_path_streak_lengths[p]) if per_path_streak_lengths[p] else 0
                for p in paths_sorted]
    x_pos = np.arange(len(paths_sorted))
    ax3.bar(x_pos - 0.2, mean_lock, 0.4, label='mean', color='steelblue')
    ax3.bar(x_pos + 0.2, max_lock, 0.4, label='max', color='navy')
    ax3.axhline(50, color='red', linestyle='--', alpha=0.5, label='cap (50)')
    ax3.set_xticks(x_pos)
    ax3.set_xticklabels([f'P{p}' for p in paths_sorted])
    ax3.set_ylabel('Streak length (steps)')
    ax3.set_title('Lock length per path (mean / max)')
    ax3.legend(fontsize=8)
    ax3.grid(alpha=0.3, axis='y')

    # ----------------------------------------------------------------
    # Panel 4 (middle-left): Along vs lateral scatter — where the craft sits
    # ----------------------------------------------------------------
    ax4 = fig.add_subplot(gs[1, 0])
    # Aggregate across all paths, color by stpPt
    all_along = []
    all_lat = []
    all_stppt = []
    for p in paths_sorted:
        # Skip path 3 if you want clean signal — keep all for honest picture
        all_along.extend(per_path_stats[p]['along'])
        all_lat.extend(per_path_stats[p]['lateral'])
        all_stppt.extend(per_path_stats[p]['stpPt'])
    all_along = np.array(all_along)
    all_lat = np.array(all_lat)
    all_stppt = np.array(all_stppt)

    # 2D histogram for density
    h, xedges, yedges = np.histogram2d(all_along, all_lat, bins=[60, 30],
                                        range=[[-25, 5], [0, 25]])
    extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]
    ax4.imshow(np.log10(h.T + 1), extent=extent, origin='lower', aspect='auto',
               cmap='hot')
    # Mark rabbit position
    ax4.plot(0, 0, 'c*', markersize=18, markeredgecolor='white', label='rabbit')
    # Streak threshold contour (-7m back along axis or 5m lateral approx)
    ax4.set_xlabel('Along-track (m)  →  + = ahead, - = behind')
    ax4.set_ylabel('Lateral distance from rabbit (m)')
    ax4.set_title('Position distribution (rabbit-relative, all paths)\nlog density: brighter = more time spent')
    ax4.legend(loc='upper right')
    ax4.set_xlim(-25, 5)
    ax4.set_ylim(0, 25)

    # ----------------------------------------------------------------
    # Panel 5 (middle-mid): Z position relative to path — gravity bias check
    # ----------------------------------------------------------------
    ax5 = fig.add_subplot(gs[1, 1])
    all_z_off = []
    for p in paths_sorted:
        all_z_off.extend(per_path_stats[p]['z_offset'])
    all_z_off = np.array(all_z_off)

    # Aircraft Z - Path Z (NED): positive = aircraft below path, negative = above
    bins = np.linspace(-15, 15, 60)
    ax5.hist(all_z_off, bins=bins, color='steelblue', edgecolor='navy', alpha=0.7)
    ax5.axvline(0, color='red', linewidth=2, label='path altitude')
    mean_z = np.mean(all_z_off)
    median_z = np.median(all_z_off)
    ax5.axvline(mean_z, color='orange', linestyle='--', label=f'mean = {mean_z:+.2f}m')
    ax5.axvline(median_z, color='green', linestyle='--', label=f'median = {median_z:+.2f}m')
    ax5.set_xlabel('Aircraft Z − Path Z (m, NED: + = below)')
    ax5.set_ylabel('Count')
    ax5.set_title(f'Vertical position relative to path\n(gravity bias check: mean {mean_z:+.2f}m, median {median_z:+.2f}m)')
    ax5.legend(fontsize=8)
    ax5.grid(alpha=0.3)

    # ----------------------------------------------------------------
    # Panel 6 (middle-right): Distance to rabbit histogram
    # ----------------------------------------------------------------
    ax6 = fig.add_subplot(gs[1, 2])
    all_dist = []
    for p in paths_sorted:
        all_dist.extend(per_path_stats[p]['dist'])
    all_dist = np.array(all_dist)
    bins = np.linspace(0, 30, 60)
    ax6.hist(all_dist, bins=bins, color='purple', edgecolor='black', alpha=0.7)
    median_d = np.median(all_dist)
    p25, p50, p75, p90 = np.percentile(all_dist, [25, 50, 75, 90])
    ax6.axvline(median_d, color='red', linewidth=2, label=f'median = {median_d:.1f}m')
    ax6.set_xlabel('Distance to rabbit (m)')
    ax6.set_ylabel('Count')
    ax6.set_title(f'Distance distribution\np25={p25:.1f} p50={p50:.1f} p75={p75:.1f} p90={p90:.1f}m')
    ax6.legend()
    ax6.grid(alpha=0.3)

    # ----------------------------------------------------------------
    # Panel 7 (bottom-left): Sample arena trajectory (one of the better ones)
    # ----------------------------------------------------------------
    ax7 = fig.add_subplot(gs[2, 0])
    # Pick the best arena from path 1 (Spiral, usually clean)
    best_arena = None
    best_score = -1
    for (path, wind), recs in arenas.items():
        if path != 1:
            continue
        score = sum(r['stpPt'] for r in recs)
        if score > best_score:
            best_score = score
            best_arena = (path, wind, recs)
    if best_arena:
        path, wind, recs = best_arena
        steps = [r['step'] for r in recs]
        ax7.plot(steps, [r['stpPt'] for r in recs], 'b-', linewidth=1, label='stpPt')
        ax7.axhline(STREAK_THRESHOLD, color='red', linestyle='--', alpha=0.5, label='threshold')
        ax7.fill_between(steps, 0, [r['stpPt'] for r in recs],
                          where=[r['stpPt'] >= STREAK_THRESHOLD for r in recs],
                          alpha=0.3, color='green', label='in streak')
        ax7.set_xlabel('Step')
        ax7.set_ylabel('stpPt')
        ax7.set_title(f'Sample arena: path {path} ({PATH_NAMES[path]}), wind {wind}\nLock-and-recover pattern over time')
        ax7.set_ylim(0, 1.05)
        ax7.legend(fontsize=8, loc='upper right')
        ax7.grid(alpha=0.3)

    # ----------------------------------------------------------------
    # Panel 8 (bottom-mid): Per-path summary table
    # ----------------------------------------------------------------
    ax8 = fig.add_subplot(gs[2, 1])
    ax8.axis('off')
    table_data = []
    for p in paths_sorted:
        s = per_path_stats[p]
        if s['total'] == 0:
            continue
        mean_d = np.mean(s['dist'])
        med_d = np.median(s['dist'])
        pct = s['in_streak'] / s['total'] * 100
        max_l = max(per_path_streak_lengths[p]) if per_path_streak_lengths[p] else 0
        table_data.append([
            f'P{p} {PATH_NAMES[p][:14]}',
            f'{mean_d:.1f}',
            f'{med_d:.1f}',
            f'{pct:.0f}%',
            f'{max_l}',
        ])
    table = ax8.table(
        cellText=table_data,
        colLabels=['Path', 'mean dist', 'median', '% locked', 'max strk'],
        loc='center',
        cellLoc='center',
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.6)
    ax8.set_title(f'Per-path tracking summary (gen {int(max_scn[3:])})')

    # ----------------------------------------------------------------
    # Panel 9 (bottom-right): Path-3 (impossible) sample for comparison
    # ----------------------------------------------------------------
    ax9 = fig.add_subplot(gs[2, 2])
    p3_arena = None
    for (path, wind), recs in arenas.items():
        if path == 3 and wind == 0:
            p3_arena = (path, wind, recs)
            break
    if p3_arena:
        path, wind, recs = p3_arena
        steps = [r['step'] for r in recs]
        ax9.plot(steps, [r['stpPt'] for r in recs], 'r-', linewidth=1)
        ax9.axhline(STREAK_THRESHOLD, color='red', linestyle='--', alpha=0.5, label='threshold')
        ax9.fill_between(steps, 0, [r['stpPt'] for r in recs],
                          where=[r['stpPt'] >= STREAK_THRESHOLD for r in recs],
                          alpha=0.3, color='green')
        ax9.set_xlabel('Step')
        ax9.set_ylabel('stpPt')
        ax9.set_title(f'Path 3 ({PATH_NAMES[3]}) wind 0\nThe physically infeasible path')
        ax9.set_ylim(0, 1.05)
        ax9.legend(fontsize=8, loc='upper right')
        ax9.grid(alpha=0.3)

    fig.suptitle(f'Intercept tracking visualization — gen {int(max_scn[3:])} best individual\n'
                 f'{len(arenas)} arenas (5 paths × 49 winds), {sum(s["total"] for s in per_path_stats.values())} total steps',
                 fontsize=13)
    plt.savefig(OUT_PATH, dpi=120, bbox_inches='tight')
    print(f"Saved: {OUT_PATH}")

    # Print summary for terminal
    print()
    print("=" * 70)
    print("OVERALL TRACKING QUALITY")
    print("=" * 70)
    total_steps = sum(s['total'] for s in per_path_stats.values())
    total_locked = sum(s['in_streak'] for s in per_path_stats.values())
    print(f"Total steps logged: {total_steps}")
    print(f"Steps in streak:    {total_locked} ({total_locked*100/total_steps:.1f}%)")
    print(f"Streak runs:        {len(all_streak_lengths)}")
    print(f"Mean streak length: {np.mean(all_streak_lengths):.1f}")
    print(f"Max streak length:  {max(all_streak_lengths)}")
    print()
    print(f"Distance to rabbit: median={np.median(all_dist):.1f}m, mean={np.mean(all_dist):.1f}m")
    print(f"  p25={np.percentile(all_dist, 25):.1f}  p50={np.percentile(all_dist, 50):.1f}  "
          f"p75={np.percentile(all_dist, 75):.1f}  p90={np.percentile(all_dist, 90):.1f}")
    print()
    print(f"Z position (aircraft − path, NED + = below):")
    print(f"  median={np.median(all_z_off):+.2f}m  mean={np.mean(all_z_off):+.2f}m  std={np.std(all_z_off):.2f}m")


if __name__ == '__main__':
    main()
