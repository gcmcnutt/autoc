"""Recurrent-capacity probe — pure weight analysis, no sim/sensor data.

Answers "is the 16-dim recurrent layer saturated (→ widen) or underused (→ 16 is
plenty)?" by SVD of the recurrent matrix W_hh extracted from an nn2cpp-emitted
weight dump. The singular-value spectrum of W_hh says how many independent
recurrent *modes* the layer actually uses:
  - effective rank ≈ N (flat spectrum)   → all modes used → likely capacity-bound → widen
  - effective rank ≪ N (decaying spectrum)→ recurrent layer underused → 16 is enough;
                                            the rising whh_xh_ratio is just reliance, not saturation

Weight layout (flat nn_weights[], from nn2cpp): for each transition l→l+1
[W (out·in) then B (out)], then for each recurrent layer [W_hh (size·size)].

Usage:
  ./build/nnextractor -k <run-id> -o /tmp/x.dat -i autoc-tracker.ini
  ./build/nn2cpp -w /tmp/x.dat -i autoc-tracker.ini -o /tmp/x.cpp
  python3 src/analytics/rnn_capacity.py --nn t11:/tmp/t11_nn.cpp --nn t12:/tmp/t12_nn.cpp \
    -o specs/037-20hz-control-loop/rnn_capacity.png
"""
import argparse
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_nn_cpp(path):
    txt = open(path).read()
    def grab_ints(name):
        m = re.search(rf"{name}\[\]\s*=\s*\{{([^}}]*)\}}", txt)
        return [int(x) for x in re.findall(r"-?\d+", m.group(1))]
    topo = grab_ints("nn_topology")
    rec_m = re.search(r"nn_recurrent\[\]\s*=\s*\{([^}]*)\}", txt)
    recurrent = [t.strip() == "true" for t in rec_m.group(1).split(",") if t.strip()]
    w_m = re.search(r"nn_weights\[\d+\]\s*=\s*\{([^}]*)\}", txt, re.S)
    weights = np.array([float(x) for x in re.findall(r"-?\d+\.?\d*(?:[eE][-+]?\d+)?", w_m.group(1))])
    return topo, recurrent, weights


def whh_blocks(topo, recurrent, weights):
    """Return list of (layer_index, size, W_hh ndarray) for each recurrent layer."""
    off = 0
    for l in range(len(topo) - 1):
        off += topo[l] * topo[l + 1] + topo[l + 1]   # W + B
    blocks = []
    for l in range(len(topo)):
        if l < len(recurrent) and recurrent[l]:
            s = topo[l]
            W = weights[off:off + s * s].reshape(s, s)
            blocks.append((l, s, W))
            off += s * s
    return blocks


def spectrum_stats(W):
    sv = np.linalg.svd(W, compute_uv=False)
    sv = sv[sv > 0]
    s2 = sv ** 2
    pr = (sv.sum() ** 2) / (s2.sum())          # participation ratio (eff. rank), 1..N
    stable = s2.sum() / (sv.max() ** 2)         # stable rank
    energy = np.cumsum(s2) / s2.sum()
    r90 = int(np.searchsorted(energy, 0.90) + 1)
    r95 = int(np.searchsorted(energy, 0.95) + 1)
    eig = np.abs(np.linalg.eigvals(W))
    return dict(sv=sv, n=len(sv), pr=pr, stable=stable, r90=r90, r95=r95,
                rho=float(eig.max()), fro=float(np.linalg.norm(W)))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--nn", action="append", required=True, help="NAME:nn2cpp.cpp (repeatable)")
    p.add_argument("-o", "--out", required=True)
    args = p.parse_args()

    runs = []
    for spec in args.nn:
        name, _, path = spec.partition(":")
        topo, rec, w = parse_nn_cpp(path)
        blocks = whh_blocks(topo, rec, w)
        if not blocks:
            print(f"{name}: no recurrent layer"); continue
        l, s, W = blocks[0]   # first (only) recurrent layer
        st = spectrum_stats(W)
        runs.append((name, s, st))
        print(f"=== {name}: W_hh {s}x{s} (layer {l}) ===")
        print(f"  effective rank (participation) : {st['pr']:.2f} / {s}")
        print(f"  stable rank (Fro²/σmax²)        : {st['stable']:.2f} / {s}")
        print(f"  rank for 90% / 95% energy       : {st['r90']} / {st['r95']}  of {s}")
        print(f"  spectral radius (max|eig|)      : {st['rho']:.3f}  (memory timescale; >1 = self-sustaining)")
        print(f"  ||W_hh||_F                      : {st['fro']:.3f}")
        print(f"  σ (top 8): {np.array2string(st['sv'][:8], precision=3, floatmode='fixed')}")

    if not runs:
        return
    # plot singular-value spectra (normalized) — flat = full-rank/saturated; steep = underused
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.6))
    colors = ["tab:blue", "tab:red", "tab:green", "tab:purple"]
    for i, (name, s, st) in enumerate(runs):
        c = colors[i % len(colors)]
        axs[0].plot(np.arange(1, st['n'] + 1), st['sv'], "o-", color=c, lw=1.6,
                    label=f"{name}: eff-rank {st['pr']:.1f}/{s}, ρ={st['rho']:.2f}")
        axs[1].plot(np.arange(1, st['n'] + 1), np.cumsum(st['sv']**2)/np.sum(st['sv']**2),
                    "o-", color=c, lw=1.6, label=name)
    axs[0].set_xlabel("singular-value index"); axs[0].set_ylabel("σ")
    axs[0].set_title("W_hh singular-value spectrum\n(flat→full-rank/saturated, steep→underused)", fontsize=10)
    axs[0].legend(fontsize=8); axs[0].grid(alpha=0.3)
    axs[1].axhline(0.9, color="gray", ls="--", lw=0.7)
    axs[1].set_xlabel("modes included"); axs[1].set_ylabel("cumulative energy (Σσ²)")
    axs[1].set_title("cumulative spectral energy", fontsize=10)
    axs[1].legend(fontsize=8); axs[1].grid(alpha=0.3)
    fig.suptitle("recurrent-layer (W_hh) capacity utilization — pure weight analysis", fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
