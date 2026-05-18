#!/usr/bin/env python3
"""NFR-4 — Beacon MCU clock-drift simulation.

Per spec NFR-4: the beacon-channel design must maintain reliable detection under
±5% chip-rate drift at the ATtiny412 internal RC oscillator. This script:

  1. Generates an N=15 Gold-code template.
  2. Simulates the received chip-stream at chip-rate offset −5%..+5% in 1% steps,
     with additive Gaussian noise at the FR-1.4 link-budget design SNR.
  3. Runs a matched-filter correlator at the *nominal* (zero-offset) chip rate.
  4. Computes (a) post-correlation SNR loss vs offset, (b) acquisition probability,
     (c) tracking probability — both for single-hypothesis and multi-hypothesis
     receivers.
  5. Emits the decision: external crystal vs. multi-hypothesis matched filter
     vs. internal-RC-only vs. both.

Inputs/configurable knobs at the top of the file.
Outputs:
  - simulation-results.md       (decision + summary tables)
  - acquisition-prob.png        (P(acquire | drift, SNR))
  - snr-loss.png                (post-correlation SNR loss vs drift)

Usage:
  python3 sim.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

# ===== Configurable parameters (per spec FR-1.3, FR-1.4, NFR-4) ===============

N_CHIPS = 15                  # Gold-code length
NOMINAL_CHIP_RATE_HZ = 100.0  # spec firm
SAMPLE_RATE_HZ = 240.0        # camera fps = chip sampling rate (240 fps mode)
DESIGN_SNR_DB = 50.0          # per-frame received SNR at 100 m daylight (FR-1.4 link budget)
DERATED_SNR_DB = 30.0         # after 100x real-world derating (haze, dirty optics, etc.)
DRIFT_STEPS = np.arange(-5, 5.1, 1) / 100.0  # -5% to +5% in 1% steps (fractional)
MC_TRIALS = 1000              # Monte Carlo trials per (drift, SNR) point
ACQ_THRESHOLD_DB = 13.0       # lock threshold (per FR-1.4)
MULTI_HYPO_OFFSETS = np.arange(-5, 5.1, 1) / 100.0  # receiver tries each as a hypothesis
RNG_SEED = 42

OUT_DIR = Path(__file__).resolve().parent


# ===== Gold code generation ===================================================

def lfsr_step(state: int, taps: int, length: int) -> int:
    """One LFSR step: shift right + XOR feedback. Returns new state."""
    fb = bin(state & taps).count("1") & 1
    return ((state >> 1) | (fb << (length - 1))) & ((1 << length) - 1)


def lfsr_sequence(seed: int, taps: int, length: int, n_chips: int) -> np.ndarray:
    """Generate `n_chips` of LFSR output (LSB) starting from `seed`."""
    state = seed
    out = np.empty(n_chips, dtype=np.int8)
    for i in range(n_chips):
        out[i] = state & 1
        state = lfsr_step(state, taps, length)
    return out


def gold_code_15(pair_index: int = 0) -> np.ndarray:
    """One member of the N=15 Gold-code family as a ±1 antipodal sequence.

    Uses the standard pair of length-15 m-sequences:
      m1: taps 0b1001 (x^4 + x + 1)         — but we use length 15 = 2^4 - 1 not from these specific taps.

    For length-15 we use the length-4 LFSR with two distinct primitive polynomials:
      poly_a: x^4 + x + 1     (taps 0b0011)
      poly_b: x^4 + x^3 + 1   (taps 0b1001)

    Gold code = m_a XOR (cyclic-shift of m_b by `pair_index`).
    pair_index ∈ [0, 14] gives 15 distinct Gold codes; the family has 17 members
    counting the two parent m-sequences.

    Returns ±1 sequence of length 15.
    """
    length = 4
    m_a = lfsr_sequence(seed=0b0001, taps=0b0011, length=length, n_chips=15)
    m_b = lfsr_sequence(seed=0b0001, taps=0b1001, length=length, n_chips=15)
    m_b_shifted = np.roll(m_b, pair_index)
    gold = m_a ^ m_b_shifted
    return (2 * gold - 1).astype(np.float32)  # 0,1 → -1, +1


# ===== Drift simulation =======================================================

def simulate_received_signal(
    code: np.ndarray,
    chip_rate_actual_hz: float,
    sample_rate_hz: float,
    n_chips: int,
    snr_db: float,
    rng: np.random.Generator,
) -> tuple[np.ndarray, float]:
    """Sample the transmitted code at `chip_rate_actual_hz` then resample by the receiver at
    `sample_rate_hz`. Adds AWGN at the given SNR.

    Returns (received_samples, sample_dt_s).
    """
    code_period_s = n_chips / chip_rate_actual_hz
    sample_dt = 1.0 / sample_rate_hz
    n_samples = int(np.ceil(code_period_s * sample_rate_hz))

    # For each sample, find which chip index it lands in
    sample_times = np.arange(n_samples) * sample_dt
    chip_indices = np.minimum(
        (sample_times * chip_rate_actual_hz).astype(int),
        n_chips - 1,
    )
    signal = code[chip_indices].astype(np.float32)

    # Add AWGN
    snr_lin = 10.0 ** (snr_db / 10.0)
    noise_std = 1.0 / np.sqrt(snr_lin)
    noise = rng.normal(0.0, noise_std, size=signal.shape).astype(np.float32)
    return signal + noise, sample_dt


def matched_filter_correlate(
    received: np.ndarray,
    code: np.ndarray,
    nominal_chip_rate_hz: float,
    sample_rate_hz: float,
    n_chips: int,
    hypothesis_chip_rate_hz: float | None = None,
) -> float:
    """Run a matched filter at the hypothesis chip rate; return peak correlation magnitude.

    Returns the peak |correlation| in units of received-sample-RMS so it can be compared
    to a noise-floor threshold.
    """
    hypothesis_rate = hypothesis_chip_rate_hz or nominal_chip_rate_hz
    code_period_s = n_chips / hypothesis_rate
    sample_dt = 1.0 / sample_rate_hz
    template_len = int(round(code_period_s * sample_rate_hz))

    # Build template by upsampling the code to the receiver's sample rate
    template_times = np.arange(template_len) * sample_dt
    template_chip_idx = np.minimum(
        (template_times * hypothesis_rate).astype(int),
        n_chips - 1,
    )
    template = code[template_chip_idx].astype(np.float32)
    template -= template.mean()
    template_norm = np.linalg.norm(template)
    if template_norm == 0:
        return 0.0

    # Slide template across received; take peak
    L = min(len(received), template_len)
    r = received[:L]
    r_chunk = np.append(r, np.zeros(template_len - L, dtype=np.float32))
    r_chunk -= r_chunk.mean()
    corr = np.dot(r_chunk, template) / template_norm
    return float(np.abs(corr))


def post_correlation_snr_db(peak: float, noise_floor: float) -> float:
    if noise_floor <= 0 or peak <= 0:
        return -np.inf
    return 20.0 * np.log10(peak / noise_floor)


# ===== Main sweep =============================================================

def run_sweep() -> dict:
    rng = np.random.default_rng(RNG_SEED)
    code = gold_code_15(pair_index=0)

    nominal_rate = NOMINAL_CHIP_RATE_HZ

    # Noise-only baseline for the threshold
    noise_floor_corr = []
    for _ in range(200):
        noise_only, _ = simulate_received_signal(
            code, nominal_rate, SAMPLE_RATE_HZ, N_CHIPS, DERATED_SNR_DB, rng,
        )
        # Correlate noise against a *different* code → measures sidelobe noise floor
        bad_code = gold_code_15(pair_index=7)
        peak = matched_filter_correlate(
            noise_only, bad_code, nominal_rate, SAMPLE_RATE_HZ, N_CHIPS,
        )
        noise_floor_corr.append(peak)
    noise_floor_p95 = float(np.percentile(noise_floor_corr, 95))
    # Lock threshold is 13 dB above the noise floor
    lock_threshold = noise_floor_p95 * (10.0 ** (ACQ_THRESHOLD_DB / 20.0))

    results = {
        "noise_floor_p95": noise_floor_p95,
        "lock_threshold": lock_threshold,
        "drift_percent": [float(d * 100) for d in DRIFT_STEPS],
        "single_hypothesis": {
            "acquisition_prob": [],
            "snr_loss_db": [],
        },
        "multi_hypothesis": {
            "acquisition_prob": [],
            "snr_loss_db": [],
        },
    }

    # Reference: at-rate single-hypothesis peak (zero drift)
    ref_peaks = []
    for _ in range(MC_TRIALS):
        sig, _ = simulate_received_signal(
            code, nominal_rate, SAMPLE_RATE_HZ, N_CHIPS, DERATED_SNR_DB, rng,
        )
        peak = matched_filter_correlate(sig, code, nominal_rate, SAMPLE_RATE_HZ, N_CHIPS)
        ref_peaks.append(peak)
    ref_median_peak = float(np.median(ref_peaks))

    for drift in DRIFT_STEPS:
        actual_rate = nominal_rate * (1 + drift)

        # Single-hypothesis: receiver assumes nominal rate
        sh_peaks = []
        sh_acquired = 0
        for _ in range(MC_TRIALS):
            sig, _ = simulate_received_signal(
                code, actual_rate, SAMPLE_RATE_HZ, N_CHIPS, DERATED_SNR_DB, rng,
            )
            peak = matched_filter_correlate(sig, code, nominal_rate, SAMPLE_RATE_HZ, N_CHIPS)
            sh_peaks.append(peak)
            if peak >= lock_threshold:
                sh_acquired += 1
        sh_acq_prob = sh_acquired / MC_TRIALS
        sh_snr_loss_db = post_correlation_snr_db(ref_median_peak, 1.0) - post_correlation_snr_db(
            float(np.median(sh_peaks)), 1.0,
        )

        # Multi-hypothesis: receiver tries each candidate offset, picks best peak
        mh_peaks = []
        mh_acquired = 0
        for _ in range(MC_TRIALS):
            sig, _ = simulate_received_signal(
                code, actual_rate, SAMPLE_RATE_HZ, N_CHIPS, DERATED_SNR_DB, rng,
            )
            best_peak = 0.0
            for hypo_offset in MULTI_HYPO_OFFSETS:
                hypo_rate = nominal_rate * (1 + hypo_offset)
                peak = matched_filter_correlate(
                    sig, code, nominal_rate, SAMPLE_RATE_HZ, N_CHIPS,
                    hypothesis_chip_rate_hz=hypo_rate,
                )
                best_peak = max(best_peak, peak)
            mh_peaks.append(best_peak)
            if best_peak >= lock_threshold:
                mh_acquired += 1
        mh_acq_prob = mh_acquired / MC_TRIALS
        mh_snr_loss_db = post_correlation_snr_db(ref_median_peak, 1.0) - post_correlation_snr_db(
            float(np.median(mh_peaks)), 1.0,
        )

        results["single_hypothesis"]["acquisition_prob"].append(sh_acq_prob)
        results["single_hypothesis"]["snr_loss_db"].append(float(sh_snr_loss_db))
        results["multi_hypothesis"]["acquisition_prob"].append(mh_acq_prob)
        results["multi_hypothesis"]["snr_loss_db"].append(float(mh_snr_loss_db))

    return results


# ===== Decision logic =========================================================

def make_decision(results: dict) -> tuple[str, str]:
    """Apply the spec's 99% cold-acquisition target at ±5% drift, 30 dB derated SNR."""
    sh_acq = np.array(results["single_hypothesis"]["acquisition_prob"])
    mh_acq = np.array(results["multi_hypothesis"]["acquisition_prob"])

    sh_worst = float(sh_acq.min())
    mh_worst = float(mh_acq.min())

    if sh_worst >= 0.99:
        return (
            "(c) Internal RC + factory calibration only",
            f"Single-hypothesis correlator achieves {sh_worst:.1%} worst-case acquisition probability "
            "across ±5% drift — meets the 99% target without any crystal or multi-hypothesis hardware. "
            "No FPGA gate cost; no extra MCU part.",
        )
    elif mh_worst >= 0.99:
        return (
            "(b) Receiver-side multi-hypothesis matched filter",
            f"Single-hypothesis falls to {sh_worst:.1%} at worst-case drift (below 99% target). "
            f"Multi-hypothesis (11 hypotheses spanning ±5%) recovers to {mh_worst:.1%}. "
            "Beacon MCU remains internal-RC-only; the cost is borne on the 031-fpga receiver side "
            "as additional correlator instances (~11× the single-hypothesis gate cost in the correlator stage).",
        )
    elif sh_worst >= 0.50:
        return (
            "(a) External crystal on the beacon MCU",
            f"Multi-hypothesis only reaches {mh_worst:.1%} worst-case — below the 99% target. "
            f"An external crystal (~$0.30, ±20-100 ppm) eliminates the drift problem at the source. "
            "Simpler than (d) for the comparable receiver complexity savings.",
        )
    else:
        return (
            "(d) Both — external crystal AND multi-hypothesis matched filter",
            f"Worst-case acquisition probability is dire ({sh_worst:.1%} single / {mh_worst:.1%} multi). "
            "Defense in depth required: crystal-stabilize the beacon AND deploy multi-hypothesis on the receiver.",
        )


# ===== Output ==================================================================

def write_results(results: dict, decision: str, rationale: str) -> None:
    """Write simulation-results.md + JSON dump. Plot files written only if matplotlib is available."""
    out_md = OUT_DIR / "simulation-results.md"
    out_json = OUT_DIR / "results.json"

    out_md.write_text(_render_md(results, decision, rationale))
    out_json.write_text(json.dumps(results, indent=2))

    # Optional matplotlib plots
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        drift_pct = results["drift_percent"]

        fig, ax = plt.subplots(figsize=(8, 5))
        ax.plot(drift_pct, results["single_hypothesis"]["acquisition_prob"],
                "o-", label="Single-hypothesis", linewidth=2)
        ax.plot(drift_pct, results["multi_hypothesis"]["acquisition_prob"],
                "s-", label="Multi-hypothesis (11 offsets)", linewidth=2)
        ax.axhline(0.99, color="red", linestyle="--", label="99% target")
        ax.set_xlabel("Beacon MCU chip-rate drift (%)")
        ax.set_ylabel("Cold-acquisition probability")
        ax.set_title(f"NFR-4 cold-acquisition probability vs drift (SNR = {DERATED_SNR_DB} dB derated)")
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, 1.05)
        fig.tight_layout()
        fig.savefig(OUT_DIR / "acquisition-prob.png", dpi=100)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(8, 5))
        ax.plot(drift_pct, results["single_hypothesis"]["snr_loss_db"],
                "o-", label="Single-hypothesis", linewidth=2)
        ax.plot(drift_pct, results["multi_hypothesis"]["snr_loss_db"],
                "s-", label="Multi-hypothesis", linewidth=2)
        ax.set_xlabel("Beacon MCU chip-rate drift (%)")
        ax.set_ylabel("Post-correlation SNR loss (dB)")
        ax.set_title("NFR-4 SNR loss vs drift")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(OUT_DIR / "snr-loss.png", dpi=100)
        plt.close(fig)
    except ImportError:
        print("(matplotlib not installed — skipping PNG plots; JSON + MD output present)")


def _render_md(results: dict, decision: str, rationale: str) -> str:
    lines = [
        "# NFR-4 — Beacon MCU clock-drift simulation results",
        "",
        f"**Generated**: 2026-05-18 by `tools/nfr4-clockdrift-sim/sim.py`",
        f"**Parameters**: N_CHIPS={N_CHIPS}, nominal chip rate={NOMINAL_CHIP_RATE_HZ} Hz, "
        f"camera fps={SAMPLE_RATE_HZ}, derated SNR={DERATED_SNR_DB} dB, "
        f"acquisition threshold={ACQ_THRESHOLD_DB} dB, MC trials={MC_TRIALS} per point, RNG seed={RNG_SEED}",
        "",
        "## Decision",
        "",
        f"**{decision}**",
        "",
        rationale,
        "",
        "## Results table",
        "",
        "| Drift (%) | Single-hypothesis acq prob | Multi-hypothesis acq prob | SH SNR loss (dB) | MH SNR loss (dB) |",
        "|---:|---:|---:|---:|---:|",
    ]
    for i, d in enumerate(results["drift_percent"]):
        lines.append(
            f"| {d:+.1f} | "
            f"{results['single_hypothesis']['acquisition_prob'][i]:.3f} | "
            f"{results['multi_hypothesis']['acquisition_prob'][i]:.3f} | "
            f"{results['single_hypothesis']['snr_loss_db'][i]:+.2f} | "
            f"{results['multi_hypothesis']['snr_loss_db'][i]:+.2f} |"
        )

    lines += [
        "",
        f"**Noise-floor p95**: {results['noise_floor_p95']:.4f} (post-correlation, code-A vs code-B)",
        f"**Lock threshold** ({ACQ_THRESHOLD_DB} dB above noise floor): {results['lock_threshold']:.4f}",
        "",
        "## Caveats",
        "",
        "- Model is AWGN-only; ignores aliasing artifacts beyond the natural 2.4 fpc oversampling.",
        "- Multi-hypothesis sweep uses 11 evenly-spaced offsets covering ±5%; finer-grain (e.g. 21 offsets) "
        "  may slightly improve worst-case acquisition at the cost of additional FPGA gates.",
        "- Per-frame SNR assumed flat across the code period; real photon-flux varies with body rate, "
        "  blob defocus, and AGC dynamics (those are characterized via FR-5.1 bench-scenario recordings).",
        "",
        "## Files",
        "",
        "- `acquisition-prob.png` — P(acquire | drift) for both receiver variants",
        "- `snr-loss.png` — post-correlation SNR loss vs drift",
        "- `results.json` — raw numbers (machine-readable)",
        "",
        "## Spec impact",
        "",
        "Per NFR-4 the spec's Decisions-Locked table SHALL record the chosen path. "
        f"This run selects: **{decision}**.",
    ]
    return "\n".join(lines) + "\n"


# ===== Main ===================================================================

if __name__ == "__main__":
    print(f"Running NFR-4 sweep: {len(DRIFT_STEPS)} drift steps × {MC_TRIALS} MC trials...")
    results = run_sweep()
    decision, rationale = make_decision(results)
    write_results(results, decision, rationale)
    print(f"Decision: {decision}")
    print(f"Rationale: {rationale}")
    print(f"Wrote: simulation-results.md, results.json")
