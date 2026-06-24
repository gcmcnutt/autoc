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

## Conclusion

- **N=15** is the only length that meets the 100-150 ms acquisition goal at the 480 fps camera — but it has the weakest noise margin and the worst CDMA floor (0.60), matching the bench (two-signal+noise ~75% confirmed, thin 1-bit margin).
- **N=31/63** add +3.2/+6.2 dB noise robustness and 2× better CDMA separation — the fix for the wrong-channel locks — but need either a faster camera (≥~990/2016 fps) or partial-correlation candidate detection to keep acquisition ≤150 ms, and a per-beacon DPLL for ±5% skew (walk >1 chip).
- **Decision lever:** code length is bought with sample rate (or partial correlation) + DPLL. The bench sweep (A4d-1) calibrates the absolute SNR axis; this model sets the expected shape.
