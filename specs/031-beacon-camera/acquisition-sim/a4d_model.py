#!/usr/bin/env python3
"""A4d predictive model — code length vs acquisition latency, noise robustness, CDMA floor, clock-skew.

Predicts (before the hardware / v-sim sweep) how the Gold-code LENGTH N trades off against the things that
matter for the camera receiver: acquisition TIME (goal 100-150 ms), robustness over NOISE, separation of two
codes (CDMA cross-corr floor), and tolerance to inter-beacon CLOCK skew. The relative tradeoffs are analytic
(processing gain = 10log10(N); Gold 3-valued cross-corr bound; coherent matched-filter detection); absolute
SNR calibration comes from the bench sweep that this gates. Run:  python3 a4d_model.py

Anchors to the live S6 harness: 480 fps camera -> 2.4 samples/chip -> 200 Hz chip @ N=15 -> 75 ms period;
lock FSM needs MINLOCK consecutive good periods to confirm.
"""
import math

N_LIST       = [15, 31, 63]                 # Gold code lengths (n = 4,5,6)
CAMERA_FPS   = 480.0                         # baseline sample rate
FPC          = 2.4                           # samples per chip (camera frames per chip)
CHIP_BASE    = CAMERA_FPS / FPC             # 200 Hz at the camera frame rate
CHIP_RATES   = [200, 240, 300]              # push nearer the noise floor: fpc = 480/rate = 2.4, 2.0, 1.6
PER_SAMPLE_SNR_DB = 0.0                      # sensor per-frame SNR (fixed); proc gain/period = 10log10(N*fpc)
MINLOCK      = 2                             # consecutive good periods to confirm (lock FSM)
ACQ_GOAL_MS  = (100.0, 150.0)               # acquisition-time goal
SKEW         = 0.05                          # +/-5% inter-beacon clock tolerance (OSCH RC)
PER_CHIP_SNR_DB = [-9, -6, -3, 0, 3, 6]     # per-chip SNR sweep (post-corr = + 10log10(N))

def n_order(N):           return {15:4, 31:5, 63:6}[N]
def proc_gain_db(N):      return 10*math.log10(N)                 # coherent matched-filter gain
def gold_t(n):            return (2**((n+2)//2) + 1) if n % 2 == 0 else (2**((n+1)//2) + 1)
def xcorr_floor(N):       return gold_t(n_order(N)) / N           # |cross-corr| bound / N (CDMA separation)
def Q(x):                 return 0.5*math.erfc(x/math.sqrt(2.0))

# coherent detection: post-corr voltage SNR d = sqrt(N * snr_lin); P(detect | period) vs a CFAR threshold.
# threshold eta set for a fixed per-period false-alarm; calibrated so N=15 needs ~0 dB post-corr to start
# locking (matches the bench: clean signal locks, noise-floor does not).
PFA = 1e-3
def eta_for_pfa(pfa):     # inverse-Q via bisection
    lo, hi = 0.0, 10.0
    for _ in range(60):
        mid = (lo+hi)/2
        if Q(mid) > pfa: lo = mid
        else: hi = mid
    return (lo+hi)/2
ETA = eta_for_pfa(PFA)

def p_detect(N, per_chip_db):
    snr_lin = 10**(per_chip_db/10.0)
    d = math.sqrt(max(0.0, N * snr_lin))     # matched-filter detection index
    return Q(ETA - d)                        # P(corr peak clears threshold this period)

def acq_latency_ms(N, per_chip_db, chip_rate):
    """E[time] to MINLOCK consecutive good periods, given per-period P_d. Geometric run-length model."""
    p = p_detect(N, per_chip_db)
    if p < 1e-3: return None
    period_ms = 1000.0 * N / chip_rate
    # E[periods until a run of k successes], success prob p:  (p^-k - 1)/(1-p)  (-> k as p->1)
    k = MINLOCK
    e_periods = k if p > 0.99999 else (p**(-k) - 1) / (1 - p)
    return e_periods * period_ms

def fpc(cr):              return CAMERA_FPS / cr                  # camera frames (samples) per chip
def nyq_verdict(f):
    if f >= 2.3: return ">Nyquist — drift margin (every chip keeps ≥2 clean samples in any phase)"
    if f >= 1.95: return "Nyquist — NO margin (unsync phase drift puts samples on chip edges)"
    return "SUB-Nyquist — aliasing; chips can get 1 sample / land on the ramp"

def chip_rate_for_goal(N, goal_ms):
    """chip rate so that MINLOCK periods (best-case confirm) fits the goal -> required sample rate (fps)."""
    period_ms = goal_ms / MINLOCK
    cr = 1000.0 * N / period_ms
    return cr, cr * FPC

def main():
    print(f"# A4d predictive model — code length tradeoffs\n")
    print(f"MINLOCK={MINLOCK} periods to confirm · camera {CAMERA_FPS:.0f} fps / {FPC} fpc = {CHIP_BASE:.0f} Hz chip "
          f"@ N=15 · acquisition goal {ACQ_GOAL_MS[0]:.0f}-{ACQ_GOAL_MS[1]:.0f} ms · skew ±{SKEW*100:.0f}%\n")

    print("## 1. Structural tradeoffs (per code length)\n")
    print("| N | proc gain | CDMA floor | period @480fps | best confirm (2·period) | skew-walk/period | need ≥X fps for 150 ms |")
    print("|---|---|---|---|---|---|---|")
    for N in N_LIST:
        period = 1000.0 * N / CHIP_BASE
        confirm = MINLOCK * period
        walk = SKEW * N                                   # chips of phase walk per period at +/-5%
        cr, fps = chip_rate_for_goal(N, ACQ_GOAL_MS[1])
        print(f"| {N} | +{proc_gain_db(N):.1f} dB | {xcorr_floor(N):.2f} | {period:.0f} ms | {confirm:.0f} ms | "
              f"{walk:.2f} chip | {fps:.0f} fps ({cr:.0f} Hz chip) |")
    print()
    print(f"_Gain is rel. to a single chip; vs N=15 it is +{proc_gain_db(31)-proc_gain_db(15):.1f} dB (N=31), "
          f"+{proc_gain_db(63)-proc_gain_db(15):.1f} dB (N=63). CDMA floor drops {xcorr_floor(15):.2f}→"
          f"{xcorr_floor(63):.2f}. Skew-walk >1 chip/period needs a per-beacon DPLL._\n")

    print("## 2. Robustness over noise — P(detect per period) vs per-chip SNR\n")
    print("| per-chip SNR | " + " | ".join(f"N={N}" for N in N_LIST) + " |")
    print("|---|" + "---|"*len(N_LIST))
    for s in PER_CHIP_SNR_DB:
        print(f"| {s:+d} dB | " + " | ".join(f"{p_detect(N,s):.3f}" for N in N_LIST) + " |")
    print(f"\n_Longer codes shift the detection curve LEFT (lock at lower SNR): the noise robustness margin._\n")

    print("## 3. Acquisition time over noise (ms to confirm @ 480 fps camera)\n")
    print("| per-chip SNR | " + " | ".join(f"N={N}" for N in N_LIST) + " |")
    print("|---|" + "---|"*len(N_LIST))
    for s in PER_CHIP_SNR_DB:
        cells = []
        for N in N_LIST:
            t = acq_latency_ms(N, s, CHIP_BASE)
            cells.append("—" if t is None else f"{t:.0f}")
        print(f"| {s:+d} dB | " + " | ".join(cells) + " |")
    print(f"\n_At a FIXED 480 fps, a longer code = a longer period = slower acquisition. Only N=15 sits near the "
          f"{ACQ_GOAL_MS[0]:.0f}-{ACQ_GOAL_MS[1]:.0f} ms goal; N=31/63 hit the goal only with a faster camera "
          f"(table 1) OR partial/progressive correlation (½-code-word candidate)._\n")

    print("## 4. Chip rate vs samples/chip @ fixed 480 fps — pushing nearer the noise floor\n")
    print("Clocks are NOT synchronized, so the sample phase drifts through every chip — fpc>2 keeps a clean "
          "sample in each chip regardless of phase. Holding per-sample SNR fixed, proc gain/period = "
          "10log10(N·fpc) (fewer samples/period = nearer the floor).\n")
    print("| chip rate | fpc | N=15 period | 2·period (confirm) | samples/period | gain/period | per-period SNR @0 dB/sample | sampling |")
    print("|---|---|---|---|---|---|---|---|")
    for cr in CHIP_RATES:
        f = fpc(cr); M = 15 * f; per = 1000.0*15/cr
        gain = 10*math.log10(M); pp = PER_SAMPLE_SNR_DB + gain
        print(f"| {cr} Hz | {f:.1f} | {per:.0f} ms | {2*per:.0f} ms | {M:.0f} | +{gain:.1f} dB | {pp:+.1f} dB | {nyq_verdict(f)} |")
    print(f"\n_Up-rate trades both ways: 300 Hz cuts N=15 confirm to ~100 ms (room for a longer code in the goal) "
          f"but drops ~1.8 dB of per-period gain AND goes sub-Nyquist — risky with unsync clocks. 240 Hz (2.0 "
          f"fpc) sits exactly at Nyquist (no drift margin). 200 Hz (2.4 fpc) keeps the unsync margin — why it "
          f"was chosen. Net acquisition TIME to a confidence is ~rate-invariant (≈ samples/480 s); the rate "
          f"mainly trades per-period strength vs decision granularity and sampling robustness._\n")

    print("## 5. Frequency flywheel — coast through outages, fast re-acquire\n")
    print("Clocks are unsync'd but STABLE, so once a per-beacon DPLL learns a beacon's RATE it stays valid "
          "through a long outage: HOLD the frequency, dead-reckon the phase forward, and re-acquire is "
          "phase-only (~MINLOCK periods) instead of a cold frequency+phase search. Coast limit = when the "
          "held-rate error walks the predicted phase past the pull-in window (~1 chip).\n")
    print("| coast | chips elapsed | Δf/f to stay <1 chip | <0.5 chip | lock time to measure (0.1-chip phase) |")
    print("|---|---|---|---|---|")
    for coast in [1.0, 10.0]:
        chips = CHIP_BASE * coast
        print(f"| {coast:.0f} s | {chips:.0f} | {100/chips:.3f}% | {50/chips:.3f}% | ~{0.1*coast:.1f} s of lock |")
    print(f"\n_A 10 s coast needs the beacon rate known to ~0.05% — an AVERAGING problem, not a precision-clock "
          f"one: the all-digital DPLL/NCO times the beacon epoch against the 12 MHz xtal (±20-50 ppm, ~10000× "
          f"tighter than 0.05%), so ~1 s of lock suffices. The MachXO2 hard PLL (MHz clock-gen) plays NO role in "
          f"tracking a 200 Hz symbol rate. The real coast limit is the EMITTER oscillator's short-term drift "
          f"over the coast window: the RC OSCH (±5% absolute) must hold ~0.05% over ~10 s — likely OK but a SPEC "
          f"to characterize; a crystal-referenced emitter buys long coast outright. Two time constants: **LOCK "
          f"confidence** falls in ~150-300 ms (LOS → image predictor 'lost'), but **FREQUENCY MEMORY** persists "
          f"~10 s → re-acquire phase-only. The design's answer to occlusion/sun/clutter: coast the rate, re-lock "
          f"on phase — don't cold-restart._\n")

    print("## Conclusion\n")
    print("- **N=15** is the only length that meets the 100-150 ms acquisition goal at the 480 fps camera — but "
          "it has the weakest noise margin and the worst CDMA floor (0.60), matching the bench (two-signal+noise "
          "~75% confirmed, thin 1-bit margin).")
    print("- **N=31/63** add +3.2/+6.2 dB noise robustness and 2× better CDMA separation — the fix for the "
          "wrong-channel locks — but need either a faster camera (≥~990/2016 fps) or partial-correlation "
          "candidate detection to keep acquisition ≤150 ms, and a per-beacon DPLL for ±5% skew (walk >1 chip).")
    print("- **Decision lever:** code length is bought with sample rate (or partial correlation) + DPLL. The "
          "bench sweep (A4d-1) calibrates the absolute SNR axis; this model sets the expected shape.")
    print("- **Frequency flywheel:** because the unsync'd clocks are stable, the DPLL holds each beacon's rate "
          "through ~10 s outages and re-acquires phase-only (~MINLOCK periods) — so the slow part is the ONE "
          "cold acquisition; occlusion/sun/clutter recoveries are fast. This relaxes the code-length/acq-time "
          "tension: a longer (more robust) code costs latency only on the very first lock.")

if __name__ == "__main__":
    main()
