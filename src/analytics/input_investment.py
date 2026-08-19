#!/usr/bin/env python3
"""Per-input weight investment + capacity, OVER GENERATIONS (041).

Answers a question nothing else in the report set does: **is evolution
investing in the inputs we added?** 041 grew the M1 vector 37 -> 42
(IN_ENVELOPE, ENVELOPE_SECS, ACCEL_X/Y/Z), and H1a asks whether the learned
policy actually uses them. The ablation tool answers that definitively, but
only after the bake and only for one elite. This is the cheap continuous
proxy, available WHILE a run is in flight.

Three panels, one shared generation axis:

  1. PER-INPUT INVESTMENT — for each sampled gen, the L2 norm of each input's
     column in the first layer W_xh (out x in), normalised by the mean column
     norm at that gen. >1 means "this input carries more first-layer weight
     than the average input". The five 041 slots are drawn in colour against a
     grey band of the other 37, so divergence is visible at a glance.
  2. RECURRENT CAPACITY over gens — participation ratio (effective rank) of
     W_hh, reusing rnn_capacity's own spectrum_stats so the two reports cannot
     disagree. rnn_capacity gives this for ONE elite; this traces it.
  3. VARIATION SCALE — the curriculum ramp, drawn so that fitness step-downs
     at ramp boundaries are self-explanatory rather than alarming. (2026-08-17:
     11 of 11 elite-fitness regressions in the 041 t1 run landed exactly on a
     40-gen ramp boundary; without this panel that reads as non-determinism.)

⚠️ WHAT THIS IS NOT. Weight norm is NOT causal importance. A small weight on a
high-variance input can matter more than a large weight on a near-constant one,
and a column can stay large simply because nothing pruned it. So this is a
SCREEN, not a verdict:
  - a new slot's norm RISING against the pack is evidence of investment;
  - a new slot's norm FLAT at the initialisation scale is evidence of neglect,
    which would make the five inputs pure search-space cost;
  - neither outcome replaces the T068 ablation, which is the verdict.

Usage:
  input_investment.py --run s3://autoc-m1/<run-id>/ --gens 1-448 --stride 10 \\
      -i autoc.ini --label <name> -o out.png [--total-gens 800] [--ramp-step 40]
"""

import argparse
import os
import re
import subprocess
import sys
import tempfile

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# Reuse rnn_capacity's parser and spectrum maths rather than restating them —
# a second copy of the weight layout is exactly the drift this project keeps
# paying for (five-definitions-of-one-value, index-coupling inventory class E1).
from rnn_capacity import parse_nn_cpp, whh_blocks, spectrum_stats  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

# The 041 additions, by position in PathgenInput. Kept as NAMES and resolved
# against the metadata table below so a layout change cannot silently point
# these at the wrong columns.
# The slots this report exists to watch. 041 P2-2 replaced the envelope pair
# with the energy/boundary-rate/gradient trio, so the old IN_ENVELOPE and
# ENVELOPE_SECS entries here were naming inputs that no longer exist.
NEW_SLOTS = ["SPECIFIC_ENERGY", "BOUNDARY_CLOSURE_RATE",
             "SCORE_GRAD_X", "SCORE_GRAD_Y", "SCORE_GRAD_Z",
             "ACCEL_X", "ACCEL_Y", "ACCEL_Z"]
NEW_COLORS = ["tab:red", "tab:orange", "tab:cyan", "tab:olive", "tab:pink",
              "tab:blue", "tab:green", "tab:purple"]

# dmp-dump per-tick column -> input slot name. Only a SUBSET of the 42 inputs is
# emitted as CSV columns, which is why the contribution panel below covers these
# and not the whole vector (see its caveat).
# dmp-dump emits EVERY slot as of 041 P2-4, labelled by the metadata table's
# display_name. So the column -> slot mapping is DERIVED from that table rather
# than hand-listed: a hand-listed subset silently stops covering new inputs, and
# this file already had one that named two retired slots.
def csv_to_slot():
    pairs = _meta_pairs()
    return {short: name for name, short in pairs}


def input_stds(tick_csv):
    """Per-slot std of the RECORDED input values, in NN units.

    ⚠️ WHY THIS MATTERS. A weight norm alone is not what the network feels: the
    pre-activation contribution scales as weight x the input's spread. An input
    pinned near a constant contributes almost nothing no matter how large its
    weight. Measured on the 041 t1 run: ACCEL_Y std = 0.0129 against
    IN_ENVELOPE's 0.3754 — a 29x difference, because a coordinated turn keeps
    lateral accel at zero. Ranking those two by weight norm is meaningless.
    """
    import csv as _csv
    import statistics as _st
    try:
        with open(tick_csv) as fh:
            rows = list(_csv.DictReader(fh))
    except OSError:
        return {}
    out = {}
    for col, slot in csv_to_slot().items():
        vals = [float(r[col]) for r in rows if r.get(col) not in (None, "")]
        if len(vals) > 1:
            out[slot] = _st.pstdev(vals)
    return out


def _meta_pairs(mode_header="nn_inputs.h"):
    """[(SLOT_NAME, display_name), ...] in declaration order, from the C++ table.

    Parsed from the header rather than hardcoded: the table is already the single
    source of truth (static_assert'd against COUNT), and the ablation tool
    resolves the same names.

    ⚠️ THE TABLE CONTAINS A MACRO. 041 P2-1 factored the 20 craft slots that M1
    and M2 share into CRAFT_COMMON_INPUT_META, so `kPathgenInputMeta[]` ends with
    a bare macro reference and the literal entries live in the #define above it.
    The previous regex read only what was inside the array braces and returned 25
    names instead of 45 — every craft slot silently absent, which blanked the
    contribution panel and produced an empty-legend warning rather than an error.
    Expand the macro before parsing.
    """
    path = os.path.join(REPO, "include", "autoc", "nn", mode_header)
    txt = open(path).read()

    mac = re.search(r"#define\s+CRAFT_COMMON_INPUT_META\s+(.*?)(?=\n\n)", txt, re.S)
    macro_body = mac.group(1).replace("\\\n", " ") if mac else ""

    m = re.search(r"kPathgenInputMeta\[\]\s*=\s*\{(.*?)\n\};", txt, re.S)
    if not m:
        return []
    body = m.group(1).replace("CRAFT_COMMON_INPUT_META", macro_body)
    return re.findall(r'\{\s*"([A-Z0-9_]+)"\s*,\s*"([^"]+)"', body)


def input_names(mode_header="nn_inputs.h"):
    """Slot names in declaration order."""
    pairs = _meta_pairs(mode_header)
    return [n for n, _ in pairs] or None


def elite_weights(run_prefix, file_gen, ini, tmp):
    """nnextractor -> nn2cpp -> (topo, recurrent, weights) for one generation.

    ⚠️ nnextractor's -g takes the FILE number (10000 - actualGen), NOT the
    actual generation — the opposite of dmp-dump --gen. Caller converts.
    """
    run_id = run_prefix.rstrip("/").split("/")[-1]
    dat = os.path.join(tmp, f"g{file_gen}.dat")
    cpp = os.path.join(tmp, f"g{file_gen}.cpp")
    try:
        subprocess.run([os.path.join(REPO, "build", "nnextractor"),
                        "-k", run_id, "-g", str(file_gen), "-o", dat, "-i", ini],
                       check=True, capture_output=True, timeout=180)
        subprocess.run([os.path.join(REPO, "build", "nn2cpp"), "-i", dat, "-o", cpp],
                       check=True, capture_output=True, timeout=180)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return None
    return parse_nn_cpp(cpp)


def variation_scale(gen, total_gens, ramp_step):
    """Mirror of computeVariationScale() in src/autoc.cc — kept in step by hand.

    numSteps = total/step; scale = min((gen-1)/step, numSteps-1)/(numSteps-1),
    and a non-positive step means no ramp at all (scale 1.0 from gen 1, which
    is what M2 runs).
    """
    if ramp_step <= 0:
        return 1.0
    num_steps = total_gens // ramp_step
    if num_steps <= 1:
        return 1.0
    step_index = (gen - 1) // ramp_step
    return min(step_index, num_steps - 1) / float(num_steps - 1)


def main():
    p = argparse.ArgumentParser(description="per-input weight investment + capacity over gens")
    p.add_argument("--run", required=True, help="s3://bucket/<run-id>/")
    p.add_argument("--gens", required=True, help="A-B actual generation range")
    p.add_argument("--stride", type=int, default=10)
    p.add_argument("-i", "--config", required=True)
    p.add_argument("--label", required=True)
    p.add_argument("-o", "--out", required=True)
    p.add_argument("--total-gens", type=int, default=800)
    p.add_argument("--ramp-step", type=int, default=40)
    p.add_argument("--csv", help="also write the per-gen table here")
    p.add_argument("--tick-csv", help="per-tick CSV (dmp-dump --csv-only) for input stds")
    args = p.parse_args()

    lo, hi = (int(x) for x in args.gens.split("-"))
    gens = list(range(lo, hi + 1, args.stride))
    if gens[-1] != hi:
        gens.append(hi)

    names = input_names()
    rows, kept = [], []
    with tempfile.TemporaryDirectory() as tmp:
        for k, g in enumerate(gens, 1):
            parsed = elite_weights(args.run, 10000 - g, args.config, tmp)
            if parsed is None:
                print(f"  [{k}/{len(gens)}] gen {g}: no dmp/genome — skipped", file=sys.stderr)
                continue
            topo, recurrent, weights = parsed
            n_in, n_out = topo[0], topo[1]
            W = weights[:n_in * n_out].reshape(n_out, n_in)      # row-major [out x in]
            col = np.linalg.norm(W, axis=0)                       # per-INPUT column norm
            rel = col / col.mean() if col.mean() > 0 else col

            pr = np.nan
            blocks = whh_blocks(topo, recurrent, weights)
            if blocks:
                pr = spectrum_stats(blocks[0][2])["pr"]

            kept.append(g)
            rows.append((rel, pr))
            print(f"  [{k}/{len(gens)}] gen {g}: inputs={n_in} eff-rank={pr:.2f}", file=sys.stderr)

    if not rows:
        print("input_investment: no generations could be read", file=sys.stderr)
        return 1

    rel = np.array([r[0] for r in rows])          # [gens x inputs]
    pr = np.array([r[1] for r in rows])
    n_in = rel.shape[1]
    if names is None or len(names) != n_in:
        names = [f"IN{i}" for i in range(n_in)]

    stds = input_stds(args.tick_csv) if args.tick_csv else {}
    n_panels = 4 if stds else 3
    heights = [3, 2.2, 1.3, 1] if stds else [3, 1.3, 1]
    fig, ax = plt.subplots(n_panels, 1, figsize=(13, 3.6 * n_panels), sharex=True,
                           gridspec_kw={"height_ratios": heights})

    # --- panel 1: per-input investment ---------------------------------------
    new_idx = {nm: names.index(nm) for nm in NEW_SLOTS if nm in names}
    old_idx = [i for i in range(n_in) if i not in new_idx.values()]
    if old_idx:
        band = rel[:, old_idx]
        ax[0].fill_between(kept, band.min(axis=1), band.max(axis=1),
                           color="0.85", label=f"the other {len(old_idx)} inputs (range)")
        ax[0].plot(kept, band.mean(axis=1), color="0.45", lw=1.0, ls="--",
                   label="their mean")
    for (nm, idx), c in zip(new_idx.items(), NEW_COLORS):
        ax[0].plot(kept, rel[:, idx], color=c, lw=1.8, label=nm)
    ax[0].axhline(1.0, color="k", lw=0.6, alpha=0.5)
    ax[0].set_ylabel("first-layer column norm\n(relative to mean input)")
    ax[0].set_title(f"{args.label} — is evolution INVESTING in the 041 inputs?\n"
                    "above 1.0 = more first-layer weight than the average input "
                    "(a SCREEN, not causal importance — T068 ablation is the verdict)")
    ax[0].legend(fontsize=8, ncol=2, loc="upper left")
    ax[0].grid(True, lw=0.3, alpha=0.4)

    # --- panel 2 (only with --tick-csv): CONTRIBUTION = norm x std ----------
    # The panel above ranks by weight alone; this ranks by what the network
    # actually feels. Covers only the slots dmp-dump emits as columns.
    k = 1
    if stds:
        for nm, c in zip(NEW_SLOTS, NEW_COLORS):
            if nm in names and nm in stds:
                ax[k].plot(kept, rel[:, names.index(nm)] * stds[nm], color=c, lw=1.8,
                           label=f"{nm}  (std {stds[nm]:.3f})")
        for nm, c in [("DIST_TO_BOUNDARY", "0.4"), ("INWARD_BODY_X", "0.6")]:
            if nm in names and nm in stds:
                ax[k].plot(kept, rel[:, names.index(nm)] * stds[nm], color=c, lw=1.1,
                           ls=":", label=f"{nm}  (std {stds[nm]:.3f}, reference)")
        ax[k].set_ylabel("contribution scale\n(rel. weight x input std)")
        ax[k].set_title("what the network actually FEELS — weight alone overstates a "
                        "near-constant input.\n⚠️ std is from the LATEST gen's ticks and "
                        "applied across all gens; only the slots dmp-dump emits are shown",
                        fontsize=9)
        ax[k].legend(fontsize=7, ncol=2, loc="upper left")
        ax[k].grid(True, lw=0.3, alpha=0.4)
        k = 2

    # --- recurrent capacity -------------------------------------------------
    if np.isfinite(pr).any():
        ax[k].plot(kept, pr, color="tab:brown", lw=1.5)
        ax[k].set_ylabel("W_hh effective rank\n(participation ratio)")
        ax[k].set_title("recurrent capacity — flat near N = saturated (widen); "
                        "well below N = 16 is plenty", fontsize=9)
        ax[k].grid(True, lw=0.3, alpha=0.4)
    else:
        ax[k].text(0.5, 0.5, "no recurrent layer in this genome",
                   ha="center", va="center", transform=ax[k].transAxes)
    k += 1

    # --- panel 3: the curriculum ramp ---------------------------------------
    vs = [variation_scale(g, args.total_gens, args.ramp_step) for g in kept]
    ax[k].step(kept, vs, where="post", color="tab:gray", lw=1.5)
    ax[k].set_ylabel("variation scale")
    ax[k].set_xlabel("generation")
    ax[k].set_ylim(-0.05, 1.05)
    ax[k].set_title(f"curriculum ramp (step {args.ramp_step}) — elite fitness legitimately "
                    "WORSENS at each step; that is difficulty rising, not non-determinism",
                    fontsize=9)
    ax[k].grid(True, lw=0.3, alpha=0.4)

    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"wrote {args.out}")

    if args.csv:
        with open(args.csv, "w") as fh:
            fh.write("gen,eff_rank,variation_scale," + ",".join(names) + "\n")
            for i, g in enumerate(kept):
                fh.write(f"{g},{pr[i]:.4f},{vs[i]:.4f},"
                         + ",".join(f"{v:.5f}" for v in rel[i]) + "\n")
            print(f"wrote {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
