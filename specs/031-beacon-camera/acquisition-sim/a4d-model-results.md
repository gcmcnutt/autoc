# A4d predictive model — code length tradeoffs

MINLOCK=2 periods to confirm · camera 480 fps / 2.4 fpc = 200 Hz chip @ N=15 · acquisition goal 100-150 ms · skew ±5%

## 1. Structural tradeoffs (per code length)

| N | proc gain | CDMA floor | period @480fps | best confirm (2·period) | skew-walk/period | need ≥X fps for 150 ms |
|---|---|---|---|---|---|---|
| 15 | +11.8 dB | 0.60 | 75 ms | 150 ms | 0.75 chip | 480 fps (200 Hz chip) |
| 31 | +14.9 dB | 0.29 | 155 ms | 310 ms | 1.55 chip | 992 fps (413 Hz chip) |
| 63 | +18.0 dB | 0.27 | 315 ms | 630 ms | 3.15 chip | 2016 fps (840 Hz chip) |

_Gain is rel. to a single chip; vs N=15 it is +3.2 dB (N=31), +6.2 dB (N=63). CDMA floor drops 0.60→0.27. Skew-walk >1 chip/period needs a per-beacon DPLL._

## 2. Robustness over noise — P(detect per period) vs per-chip SNR

| per-chip SNR | N=15 | N=31 | N=63 |
|---|---|---|---|
| -9 dB | 0.043 | 0.132 | 0.392 |
| -6 dB | 0.125 | 0.382 | 0.813 |
| -3 dB | 0.364 | 0.803 | 0.994 |
| +0 dB | 0.783 | 0.993 | 1.000 |
| +3 dB | 0.991 | 1.000 | 1.000 |
| +6 dB | 1.000 | 1.000 | 1.000 |

_Longer codes shift the detection curve LEFT (lock at lower SNR): the noise robustness margin._

## 3. Acquisition time over noise (ms to confirm @ 480 fps camera)

| per-chip SNR | N=15 | N=31 | N=63 |
|---|---|---|---|
| -9 dB | 42159 | 10001 | 2853 |
| -6 dB | 5380 | 1467 | 865 |
| -3 dB | 773 | 434 | 635 |
| +0 dB | 218 | 313 | 630 |
| +3 dB | 152 | 310 | 630 |
| +6 dB | 150 | 310 | 630 |

_At a FIXED 480 fps, a longer code = a longer period = slower acquisition. Only N=15 sits near the 100-150 ms goal; N=31/63 hit the goal only with a faster camera (table 1) OR partial/progressive correlation (½-code-word candidate)._

## 4. Chip rate vs samples/chip @ fixed 480 fps — pushing nearer the noise floor

Clocks are NOT synchronized, so the sample phase drifts through every chip — fpc>2 keeps a clean sample in each chip regardless of phase. Holding per-sample SNR fixed, proc gain/period = 10log10(N·fpc) (fewer samples/period = nearer the floor).

| chip rate | fpc | N=15 period | 2·period (confirm) | samples/period | gain/period | per-period SNR @0 dB/sample | sampling |
|---|---|---|---|---|---|---|---|
| 200 Hz | 2.4 | 75 ms | 150 ms | 36 | +15.6 dB | +15.6 dB | >Nyquist — drift margin (every chip keeps ≥2 clean samples in any phase) |
| 240 Hz | 2.0 | 62 ms | 125 ms | 30 | +14.8 dB | +14.8 dB | Nyquist — NO margin (unsync phase drift puts samples on chip edges) |
| 300 Hz | 1.6 | 50 ms | 100 ms | 24 | +13.8 dB | +13.8 dB | SUB-Nyquist — aliasing; chips can get 1 sample / land on the ramp |

_Up-rate trades both ways: 300 Hz cuts N=15 confirm to ~100 ms (room for a longer code in the goal) but drops ~1.8 dB of per-period gain AND goes sub-Nyquist — risky with unsync clocks. 240 Hz (2.0 fpc) sits exactly at Nyquist (no drift margin). 200 Hz (2.4 fpc) keeps the unsync margin — why it was chosen. Net acquisition TIME to a confidence is ~rate-invariant (≈ samples/480 s); the rate mainly trades per-period strength vs decision granularity and sampling robustness._

## 5. Frequency flywheel — coast through outages, fast re-acquire

Clocks are unsync'd but STABLE, so once a per-beacon DPLL learns a beacon's RATE it stays valid through a long outage: HOLD the frequency, dead-reckon the phase forward, and re-acquire is phase-only (~MINLOCK periods) instead of a cold frequency+phase search. Coast limit = when the held-rate error walks the predicted phase past the pull-in window (~1 chip).

| coast | chips elapsed | Δf/f to stay <1 chip | <0.5 chip | lock time to measure (0.1-chip phase) |
|---|---|---|---|---|
| 1 s | 200 | 0.500% | 0.250% | ~0.1 s of lock |
| 10 s | 2000 | 0.050% | 0.025% | ~1.0 s of lock |

_A 10 s coast needs the beacon rate known to ~0.05% — buyable from ~1 s of prior lock (measuring phase to ~0.1 chip), or less if the OSCH short-term stability beats its ±5% absolute tolerance (it does). Two time constants: **LOCK confidence** falls in ~150-300 ms (LOS → image predictor 'lost'), but the **FREQUENCY MEMORY** persists ~10 s → re-acquire in ~MINLOCK periods. This is the design's answer to occlusion/sun/clutter: don't cold-restart — coast the rate, re-lock on phase._

## Conclusion

- **N=15** is the only length that meets the 100-150 ms acquisition goal at the 480 fps camera — but it has the weakest noise margin and the worst CDMA floor (0.60), matching the bench (two-signal+noise ~75% confirmed, thin 1-bit margin).
- **N=31/63** add +3.2/+6.2 dB noise robustness and 2× better CDMA separation — the fix for the wrong-channel locks — but need either a faster camera (≥~990/2016 fps) or partial-correlation candidate detection to keep acquisition ≤150 ms, and a per-beacon DPLL for ±5% skew (walk >1 chip).
- **Decision lever:** code length is bought with sample rate (or partial correlation) + DPLL. The bench sweep (A4d-1) calibrates the absolute SNR axis; this model sets the expected shape.
- **Frequency flywheel:** because the unsync'd clocks are stable, the DPLL holds each beacon's rate through ~10 s outages and re-acquires phase-only (~MINLOCK periods) — so the slow part is the ONE cold acquisition; occlusion/sun/clutter recoveries are fast. This relaxes the code-length/acq-time tension: a longer (more robust) code costs latency only on the very first lock.
