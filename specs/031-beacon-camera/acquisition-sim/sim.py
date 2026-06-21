#!/usr/bin/env python3
"""031 — Beacon acquisition-dynamics simulation.

Predicts what the single-IR-sensor bench should show (research-plan §9), focused on:

  1. TIME TO POSITIVE LOCK vs FRACTION OF CODE RECEIVED — integrate k of N chips
     (full bit length, 50%, 70%, ...) and find when a confident correct lock forms.
  2. MISSING-BIT response — single chip erased (occlusion/fade) → effect on lock.
  3. MISSING-MULTI-BIT response — j chips erased, random vs consecutive.
  4. BIT-FLIP response — chip received wrong (costs 2× an erasure) vs erasure.
  5. TWO-CODES-ON-ONE-DETECTOR (CDMA) — acquire code A with code B summed in.

Model: bipolar (±1) matched filter — the AC-coupled receiver maps LED off/on to ∓1.
Cold acquisition searches all N cyclic phases × all deployed codes; a trial "positively
locks" when the TRUE (code, phase) is the global correlation max AND the peak clears a
confidence floor (LOCK_CONF × effective chips). Per-chip SNR is swept around the lock
threshold — that's the regime (long range / heavy occlusion / off-axis) where partial-code
and dropout behaviour actually bites; at the link-budget's nominal ~40 dB margin, full-code
acquisition is trivially instant.

Outputs:
  - acquisition-results.md   (tables + summary)
  - results.json             (raw numbers)
  - *.png                    (plots, if matplotlib present)

Usage: python3 sim.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

# ===== Parameters =============================================================

N_CHIPS = 15
CHIP_RATE_HZ = 200.0                 # 480 fps baseline / 2.4 frames-per-chip = 200 Hz
                                     # (supersedes the spec's 100 Hz, which was 240 fps × 2.4 fpc).
                                     # Single-PD bench oversamples ~1000× so it's chip-rate-agnostic;
                                     # 200 Hz is the value representative of the 480 fps camera era.
CHIP_MS = 1000.0 / CHIP_RATE_HZ      # 5 ms/chip → full code = 75 ms
CTRL_TICK_MS = 50.0                  # 20 Hz control loop (037) — for "ticks to lock"
DEPLOYED_CODES = [0, 1]              # beacon A=code 0, B=code 1 (cold search spans these)
PER_CHIP_SNR_DB = [-6, -3, 0, 3, 6, 10]   # full-code post-corr ≈ per-chip + 10log10(15) = +11.8 dB
K_FRACTIONS = [2, 4, 6, 8, 10, 11, 13, 15]  # chips integrated (8≈53%, 11≈73%, 15=100%)
LOCK_CONF = 0.5                      # peak must exceed LOCK_CONF × effective-chips
LOCK_TARGET = 0.99                   # P(lock) target for "chips/ms to lock"
DROP_SNR_DB = [0, 6]                 # per-chip SNR for the dropout/flip tables (marginal, comfortable)
MC = 4000
RNG_SEED = 7

OUT_DIR = Path(__file__).resolve().parent


# ===== Gold code (length-15 family) ===========================================

def lfsr_sequence(seed, taps, length, n):
    state, out = seed, np.empty(n, dtype=np.int8)
    for i in range(n):
        out[i] = state & 1
        fb = bin(state & taps).count("1") & 1
        state = ((state >> 1) | (fb << (length - 1))) & ((1 << length) - 1)
    return out


def gold_code(pair_index=0):
    """One ±1 member of the N=15 Gold family (same construction as the NFR-4 sim)."""
    m_a = lfsr_sequence(0b0001, 0b0011, 4, 15)
    m_b = lfsr_sequence(0b0001, 0b1001, 4, 15)
    gold = m_a ^ np.roll(m_b, pair_index)
    return (2 * gold - 1).astype(np.float64)  # 0,1 → −1,+1


# ===== Core: one cold-acquisition trial =======================================

def acq_trial(templates, true_code, k, snr_db, rng,
              n_drop=0, drop_mode="random", n_flip=0, interferer=None):
    """Simulate integrating k chips and a full phase×code search.

    Returns (positive_lock, correct_id, peak, k_eff).
      positive_lock = correct (code,phase) is the global max AND peak > LOCK_CONF·k_eff
      correct_id    = correct (code,phase) is the global max (ignores confidence floor)
    """
    N = N_CHIPS
    sigma = 10.0 ** (-snr_db / 20.0)
    t_true = templates[true_code]
    phi = int(rng.integers(0, N))                 # unknown true phase
    pos = np.arange(k)
    chips = t_true[(phi + pos) % N].astype(float)  # transmitted ±1

    # CDMA interferer (other beacon present on the same detector), random phase
    if interferer is not None:
        iphi = int(rng.integers(0, N))
        chips = chips + templates[interferer][(iphi + pos) % N]

    # bit flips (chip received wrong) — invert before noise
    if n_flip > 0:
        chips[rng.choice(k, size=min(n_flip, k), replace=False)] *= -1

    r = chips + rng.normal(0.0, sigma, size=k)

    # erasures (chip missing — excluded from the correlation)
    valid = np.ones(k, dtype=bool)
    if n_drop > 0:
        if drop_mode == "consecutive":
            s = int(rng.integers(0, k))
            valid[(s + np.arange(n_drop)) % k] = False
        else:
            valid[rng.choice(k, size=min(n_drop, k), replace=False)] = False

    rv = r[valid]
    vpos = pos[valid]
    k_eff = int(valid.sum())
    if k_eff == 0:
        return False, False, 0.0, 0

    # vectorized phase search per code: scores[h] = sum_j rv[j]·tmpl[(h+vpos[j]) % N]
    best_val, best_key = -1e18, None
    own_val, own_h = -1e18, None
    for c, tmpl in templates.items():
        M = tmpl[(np.arange(N)[:, None] + vpos[None, :]) % N]   # (N, k_eff)
        scores = M @ rv                                          # (N,)
        h = int(np.argmax(scores))
        v = float(scores[h])
        if v > best_val:
            best_val, best_key = v, (c, h)
        if c == true_code:
            own_val, own_h = v, h

    # global: correct (code,phase) is the argmax across ALL codes (cold acquisition,
    #         must reject the wrong code too) — used for single-beacon §1.
    # owncode: best phase WITHIN the true code is correct (other codes treated as
    #          interference, not competing winners) — used for CDMA §4.
    positive_global = (best_key == (true_code, phi)) and (best_val > LOCK_CONF * k_eff)
    positive_owncode = (own_h == phi) and (own_val > LOCK_CONF * k_eff)
    return positive_global, positive_owncode, best_val, k_eff


def p_lock(templates, true_code, k, snr_db, rng, eval="global", **kw):
    """Return P(positive lock). eval='global' (cross-code argmax) or 'owncode' (true-code acquisition)."""
    g = o = 0
    for _ in range(MC):
        pg, po, _, _ = acq_trial(templates, true_code, k, snr_db, rng, **kw)
        g += pg
        o += po
    p = (g if eval == "global" else o) / MC
    return p, p


# ===== Sweeps =================================================================

def run():
    rng = np.random.default_rng(RNG_SEED)
    templates = {c: gold_code(c) for c in DEPLOYED_CODES}
    A = DEPLOYED_CODES[0]

    res = {"params": {
        "N": N_CHIPS, "chip_rate_hz": CHIP_RATE_HZ, "chip_ms": CHIP_MS,
        "ctrl_tick_ms": CTRL_TICK_MS, "deployed_codes": DEPLOYED_CODES,
        "lock_conf": LOCK_CONF, "lock_target": LOCK_TARGET, "mc": MC, "seed": RNG_SEED,
    }}

    # --- 1. Partial-code acquisition: P(lock) vs k vs per-chip SNR ---
    partial = {}
    for snr in PER_CHIP_SNR_DB:
        row = []
        for k in K_FRACTIONS:
            pl, _ = p_lock(templates, A, k, snr, rng)
            row.append(pl)
        partial[snr] = row
    res["partial_code"] = {"k": K_FRACTIONS, "snr_db": PER_CHIP_SNR_DB, "p_lock": partial}

    # chips/ms/ticks to reach LOCK_TARGET, per SNR
    to_lock = {}
    for snr in PER_CHIP_SNR_DB:
        kstar = next((k for k, pl in zip(K_FRACTIONS, partial[snr]) if pl >= LOCK_TARGET), None)
        to_lock[snr] = {
            "chips": kstar,
            "ms": None if kstar is None else kstar * CHIP_MS,
            "ticks_20hz": None if kstar is None else kstar * CHIP_MS / CTRL_TICK_MS,
        }
    res["time_to_lock"] = to_lock

    # --- 2. Missing-bit (erasure) response: full code, j drops ---
    drops = {}
    for snr in DROP_SNR_DB:
        drops[snr] = {"random": [], "consecutive": []}
        for j in range(0, 7):
            for mode in ("random", "consecutive"):
                pl, _ = p_lock(templates, A, N_CHIPS, snr, rng, n_drop=j, drop_mode=mode)
                drops[snr][mode].append(pl)
    res["dropouts"] = {"n_drop": list(range(7)), "snr_db": DROP_SNR_DB, "p_lock": drops}

    # --- 3. Bit-flip response (errors cost 2× erasures) ---
    flips = {}
    for snr in DROP_SNR_DB:
        flips[snr] = []
        for j in range(0, 7):
            pl, _ = p_lock(templates, A, N_CHIPS, snr, rng, n_flip=j)
            flips[snr].append(pl)
    res["flips"] = {"n_flip": list(range(7)), "snr_db": DROP_SNR_DB, "p_lock": flips}

    # --- 4. Two-codes-on-one-detector (CDMA): acquire A with B summed in ---
    cdma = {}
    for snr in PER_CHIP_SNR_DB:
        row = []
        for k in K_FRACTIONS:
            pl, _ = p_lock(templates, A, k, snr, rng, eval="owncode", interferer=DEPLOYED_CODES[1])
            row.append(pl)
        cdma[snr] = row
    res["cdma_two_beacon"] = {"k": K_FRACTIONS, "snr_db": PER_CHIP_SNR_DB, "p_lock": cdma}

    return res


# ===== Output =================================================================

def frac(k):
    return f"{100*k/N_CHIPS:.0f}%"


def render_md(r):
    p = r["params"]
    L = [
        "# 031 — Beacon acquisition-dynamics simulation results",
        "",
        "**Generated by** `tools/acquisition-sim/sim.py`",
        f"**Params**: N={p['N']} Gold, chip rate {p['chip_rate_hz']:.0f} Hz "
        f"({p['chip_ms']:.0f} ms/chip → full code {p['N']*p['chip_ms']:.0f} ms), "
        f"20 Hz control tick = {p['ctrl_tick_ms']:.0f} ms, deployed codes {p['deployed_codes']}, "
        f"lock = correct (code,phase) is argmax AND peak > {p['lock_conf']}×chips, "
        f"MC={p['mc']}, seed={p['seed']}.",
        "",
        "Per-chip SNR is swept around the lock threshold (full-code post-correlation gain = "
        "+10·log₁₀(15) = +11.8 dB on top of per-chip). At the link budget's nominal margin "
        "acquisition is instant; these tables map the *marginal* regime (range / occlusion / off-axis).",
        "",
        "## 1. Time to positive lock vs fraction of code received",
        "",
        "P(positive correct lock) integrating k of 15 chips. Columns = per-chip SNR (dB).",
        "",
    ]
    snrs = r["partial_code"]["snr_db"]
    L.append("| chips (k) | fraction | ms | 20 Hz ticks | " + " | ".join(f"{s:+d} dB" for s in snrs) + " |")
    L.append("|---:|---:|---:|---:|" + "|".join(["---:"] * len(snrs)) + "|")
    for i, k in enumerate(r["partial_code"]["k"]):
        cells = " | ".join(f"{r['partial_code']['p_lock'][s][i]:.3f}" for s in snrs)
        L.append(f"| {k} | {frac(k)} | {k*CHIP_MS:.0f} | {k*CHIP_MS/CTRL_TICK_MS:.1f} | {cells} |")
    L += ["", f"**Chips / latency to {p['lock_target']:.0%} lock** (— = not reached within one code period):", ""]
    L.append("| per-chip SNR | chips | ms | 20 Hz ticks |")
    L.append("|---:|---:|---:|---:|")
    for s in snrs:
        t = r["time_to_lock"][s]
        chips = "—" if t["chips"] is None else str(t["chips"])
        ms = "—" if t["ms"] is None else f"{t['ms']:.0f}"
        tk = "—" if t["ticks_20hz"] is None else f"{t['ticks_20hz']:.1f}"
        L.append(f"| {s:+d} dB | {chips} | {ms} | {tk} |")

    # Dropouts
    L += ["", "## 2. Missing-bit (erasure) response — full 15-chip code", "",
          "P(positive lock) with j chips erased (missing/occluded), random vs consecutive.", ""]
    for s in r["dropouts"]["snr_db"]:
        L += [f"### per-chip SNR = {s:+d} dB", "",
              "| erased chips | fraction lost | P(lock) random | P(lock) consecutive |",
              "|---:|---:|---:|---:|"]
        d = r["dropouts"]["p_lock"][s]
        for i, j in enumerate(r["dropouts"]["n_drop"]):
            L.append(f"| {j} | {frac(j)} | {d['random'][i]:.3f} | {d['consecutive'][i]:.3f} |")
        L.append("")

    # Flips
    L += ["## 3. Bit-flip response (a wrong chip costs ~2× an erasure)", "",
          "P(positive lock) with j chips received *wrong* (full code), vs the erasure column above.", ""]
    L.append("| flipped chips | " + " | ".join(f"P(lock) @ {s:+d} dB" for s in r["flips"]["snr_db"]) + " |")
    L.append("|---:|" + "|".join(["---:"] * len(r["flips"]["snr_db"])) + "|")
    for i, j in enumerate(r["flips"]["n_flip"]):
        L.append(f"| {j} | " + " | ".join(f"{r['flips']['p_lock'][s][i]:.3f}" for s in r["flips"]["snr_db"]) + " |")

    # CDMA
    L += ["", "## 4. Two codes on one detector (CDMA) — acquire A with B summed in", "",
          "P(lock) on code A while code B illuminates the same photodiode (random relative phase).", ""]
    L.append("| chips (k) | fraction | " + " | ".join(f"{s:+d} dB" for s in snrs) + " |")
    L.append("|---:|---:|" + "|".join(["---:"] * len(snrs)) + "|")
    for i, k in enumerate(r["cdma_two_beacon"]["k"]):
        L.append(f"| {k} | {frac(k)} | " + " | ".join(f"{r['cdma_two_beacon']['p_lock'][s][i]:.3f}" for s in snrs) + " |")

    L += ["", "## Reading this",
          "- **Erasure vs flip**: compare §2 (random) to §3 at the same SNR — a *wrong* chip hurts ~2× a *missing* one. "
          "Mark faded/saturated chips as ERASURES, not guesses (research-plan §7).",
          "- **Partial-code lock (§1)**: the smallest k hitting target is the lever for staying inside the 20 Hz "
          "control budget (150 ms full code = 3 ticks; a ~70%/110 ms early lock ≈ 2 ticks).",
          "- **CDMA (§4)** vs §1: the gap is the cost of a second beacon sharing the detector — the make-or-break "
          "two-beacon-on-one-pixel result.",
          "- These are predictions; the bench measures the real curves against them.",
          ""]
    return "\n".join(L) + "\n"


def plots(r):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib not installed — skipping PNGs)")
        return

    snrs = r["partial_code"]["snr_db"]
    ks = r["partial_code"]["k"]
    fr = [100 * k / N_CHIPS for k in ks]

    fig, ax = plt.subplots(figsize=(8, 5))
    for s in snrs:
        ax.plot(fr, r["partial_code"]["p_lock"][s], "o-", label=f"{s:+d} dB/chip")
    ax.axhline(r["params"]["lock_target"], color="red", ls="--", label="target")
    ax.set_xlabel("Fraction of code integrated (%)")
    ax.set_ylabel("P(positive lock)")
    ax.set_title("Partial-code acquisition vs fraction received")
    ax.legend(fontsize=8); ax.grid(alpha=0.3); ax.set_ylim(0, 1.05)
    fig.tight_layout(); fig.savefig(OUT_DIR / "partial-code-acquisition.png", dpi=100); plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 5))
    s0 = r["dropouts"]["snr_db"][0]
    nd = r["dropouts"]["n_drop"]
    ax.plot(nd, r["dropouts"]["p_lock"][s0]["random"], "o-", label="erasure (random)")
    ax.plot(nd, r["dropouts"]["p_lock"][s0]["consecutive"], "s-", label="erasure (consecutive)")
    ax.plot(r["flips"]["n_flip"], r["flips"]["p_lock"][s0], "^-", label="bit flip (wrong chip)")
    ax.set_xlabel("chips missing / wrong (of 15)")
    ax.set_ylabel("P(positive lock)")
    ax.set_title(f"Dropout & flip response (per-chip SNR {s0:+d} dB)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3); ax.set_ylim(0, 1.05)
    fig.tight_layout(); fig.savefig(OUT_DIR / "dropout-flip-response.png", dpi=100); plt.close(fig)


if __name__ == "__main__":
    print(f"Running acquisition sweep (MC={MC})...")
    results = run()
    (OUT_DIR / "acquisition-results.md").write_text(render_md(results))
    (OUT_DIR / "results.json").write_text(json.dumps(results, indent=2))
    plots(results)
    print("Wrote: acquisition-results.md, results.json (+ PNGs if matplotlib present)")
