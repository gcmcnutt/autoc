#!/usr/bin/env python3
"""Sensor self-consistency audit driver (024 WI1/WI2).

Auto-detects input format (blackbox CSV / xiao log / sim data.dat), reads
via sensor_self_check_lib, runs all 8 cross-checks, emits console summary,
PNG report, and markdown report.

Usage:
    python3 sensor_self_check.py <path-to-input-file> [out.png] [out.md]

Format detection:
    *.csv           -> INAV blackbox CSV
    *_log_*.txt     -> xiao flight log
    *.dat           -> sim data.dat
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from sensor_self_check_lib import (
    CanonicalData, read_blackbox_csv, read_xiao_log, read_sim_data_dat,
    run_all_checks, format_report, CHECKS,
)


def detect_and_read(path: str) -> CanonicalData:
    low = path.lower()
    if low.endswith('.csv'):
        return read_blackbox_csv(path)
    if low.endswith('.dat'):
        return read_sim_data_dat(path)
    if 'log' in low and low.endswith('.txt'):
        return read_xiao_log(path)
    raise ValueError(f"Can't detect format of {path}. Use .csv / .dat / *_log_*.txt")


def render_plots(d: CanonicalData, report: dict, out_png: str):
    """Render a 4x2 panel PNG (8 checks, one per panel)."""
    fig, axes = plt.subplots(4, 2, figsize=(14, 18))
    for idx, (name, result) in enumerate(report.items()):
        ax = axes[idx // 2, idx % 2]
        if 'skipped' in result:
            ax.text(0.5, 0.5, f"{name}\nSKIPPED\n({result['skipped']})",
                    ha='center', va='center', fontsize=10, alpha=0.6,
                    transform=ax.transAxes)
            ax.set_xticks([])
            ax.set_yticks([])
            continue
        # Build scatter for each axis
        _render_check_panel(ax, name, result, d)
    fig.suptitle(f"Sensor self-consistency audit — {d.source}\n"
                 f"{os.path.basename(d.meta.get('path', '?'))}  "
                 f"N={len(d.t_ms)}  "
                 f"span={(d.t_ms[-1]-d.t_ms[0])/1000:.1f}s",
                 fontsize=12)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(out_png, dpi=110, bbox_inches='tight')
    plt.close(fig)


def _render_check_panel(ax, name, result, d):
    """Render one check's summary panel — sign + r per axis as bar."""
    axes_labels = [k for k in result.keys() if isinstance(result[k], dict) and 'slope' in result[k]]
    if not axes_labels:
        # Free-form result (e.g., mean_offset_deg)
        txt_lines = [f"{name}"]
        for k, v in result.items():
            txt_lines.append(f"  {k}: {v}")
        ax.text(0.05, 0.95, '\n'.join(txt_lines), va='top', ha='left',
                transform=ax.transAxes, family='monospace', fontsize=9)
        ax.set_xticks([])
        ax.set_yticks([])
        return
    slopes = [result[a]['slope'] for a in axes_labels]
    rs = [result[a]['r'] for a in axes_labels]
    colors = ['green' if s > 0 else 'red' for s in slopes]
    xpos = np.arange(len(axes_labels))
    ax.bar(xpos, slopes, color=colors, alpha=0.7, label='slope')
    for i, (s, r) in enumerate(zip(slopes, rs)):
        ax.text(i, s + (0.05 if s >= 0 else -0.05),
                f'r={r:+.2f}', ha='center',
                va='bottom' if s >= 0 else 'top', fontsize=9)
    ax.axhline(0, color='black', lw=0.5)
    ax.axhline(1, color='gray', lw=0.3, linestyle=':', alpha=0.5)
    ax.set_xticks(xpos)
    ax.set_xticklabels(axes_labels)
    ax.set_ylabel('slope')
    fail = any(s < 0 for s in slopes)
    banner = '✗ FAIL' if fail else '✓ PASS'
    ax.set_title(f"{name}  [{banner}]", fontsize=10,
                 color='red' if fail else 'darkgreen')
    ax.grid(alpha=0.3, axis='y')


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    path = sys.argv[1]
    default_base = os.path.splitext(os.path.basename(path))[0]
    default_dir = os.path.dirname(os.path.abspath(path)) or '.'
    out_png = sys.argv[2] if len(sys.argv) > 2 else \
        os.path.join(default_dir, f'sensor_self_check_{default_base}.png')
    out_md = sys.argv[3] if len(sys.argv) > 3 else \
        os.path.join(default_dir, f'sensor_self_check_{default_base}.md')

    print(f"Reading {path} ...")
    d = detect_and_read(path)
    print(f"  parsed {len(d.t_ms)} samples ({d.source})")
    for k, v in d.meta.items():
        print(f"  {k}: {v}")
    print()

    report = run_all_checks(d)
    text = format_report(d, report)
    print(text)

    with open(out_md, 'w') as f:
        f.write(text + '\n')
    print(f"\nReport saved: {out_md}")

    render_plots(d, report, out_png)
    print(f"Plots saved: {out_png}")


if __name__ == '__main__':
    main()
