# NFR-4 — Beacon MCU clock-drift simulation results

**Generated**: 2026-05-18 by `specs/031-beacon-camera/nfr4-clockdrift-sim/sim.py`
**Parameters**: N_CHIPS=15, nominal chip rate=100.0 Hz, camera fps=240.0, derated SNR=30.0 dB, acquisition threshold=13.0 dB, MC trials=1000 per point, RNG seed=42

## Decision

**(c) Internal RC + factory calibration only**

Single-hypothesis correlator achieves 100.0% worst-case acquisition probability across ±5% drift — meets the 99% target without any crystal or multi-hypothesis hardware. No FPGA gate cost; no extra MCU part.

## Results table

| Drift (%) | Single-hypothesis acq prob | Multi-hypothesis acq prob | SH SNR loss (dB) | MH SNR loss (dB) |
|---:|---:|---:|---:|---:|
| -5.0 | 1.000 | 1.000 | +3.53 | -0.22 |
| -4.0 | 1.000 | 1.000 | +2.18 | -0.22 |
| -3.0 | 1.000 | 1.000 | +2.18 | -0.22 |
| -2.0 | 1.000 | 1.000 | +1.58 | -0.09 |
| -1.0 | 1.000 | 1.000 | +1.02 | +0.00 |
| +0.0 | 1.000 | 1.000 | -0.00 | -0.00 |
| +1.0 | 1.000 | 1.000 | +0.49 | +0.01 |
| +2.0 | 1.000 | 1.000 | +1.02 | +0.16 |
| +3.0 | 1.000 | 1.000 | +1.88 | +0.13 |
| +4.0 | 1.000 | 1.000 | +2.50 | +0.16 |
| +5.0 | 1.000 | 1.000 | +3.17 | +0.25 |

**Noise-floor p95**: 0.0630 (post-correlation, code-A vs code-B)
**Lock threshold** (13.0 dB above noise floor): 0.2815

## Caveats

- Model is AWGN-only; ignores aliasing artifacts beyond the natural 2.4 fpc oversampling.
- Multi-hypothesis sweep uses 11 evenly-spaced offsets covering ±5%; finer-grain (e.g. 21 offsets)   may slightly improve worst-case acquisition at the cost of additional FPGA gates.
- Per-frame SNR assumed flat across the code period; real photon-flux varies with body rate,   blob defocus, and AGC dynamics (those are characterized via FR-5.1 bench-scenario recordings).

## Files

- `acquisition-prob.png` — P(acquire | drift) for both receiver variants
- `snr-loss.png` — post-correlation SNR loss vs drift
- `results.json` — raw numbers (machine-readable)

## Spec impact

Per NFR-4 the spec's Decisions-Locked table SHALL record the chosen path. This run selects: **(c) Internal RC + factory calibration only**.
