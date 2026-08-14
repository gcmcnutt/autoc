# Study A — per-regime control aggressiveness (T054–T058)

**Run 2026-08-13** by `src/analytics/regime_load_study.py` over the T011a pre-break CSVs
(`../artifacts/pre-break/`). Regime rule imported from `dynamics_progress.py`, not restated, so these
numbers are comparable with every prior report.

⚠️ **CONTROL HALF ONLY.** Both comparators predate 041 T039, so neither carries `ACCEL_*` and **no load
number exists for either**. That is expected and accepted (operator 2026-08-10, *"old M1 is what it is"*).
Every 041 load figure will be a single-run profile of the NEW M1 (T071) — never an old-vs-new delta. The
load half of T056 and all of T057 are therefore **still open**, by design rather than by omission.

⚠️ Load must NOT be back-filled from `physicsTrace`: it covers ~1% of ticks (175 ms/scenario cap) and would
misstate a peak, which is the one statistic that matters for damage.

## Regime occupancy

| run | tracking | intercept | patrol | ticks |
|---|---:|---:|---:|---:|
| prior M1 (pathgen) | **30.9%** | 37.9% | 31.2% | 132 462 |
| 040-t4 (M2 tracker) | **8.1%** | 46.3% | 45.5% | 131 802 |

M2 spends **3.8× less time tracking** than M1 — an independent restatement of the known M2 depth cap,
arrived at here from control data rather than from the fitness curve.

## Per-regime aggressiveness — the headline

`dCtrl` = mean |Δoutput| per tick, `⟨|u|⟩` = mean |output|.

| run | axis | tracking dCtrl | intercept dCtrl | patrol dCtrl | swing |
|---|---|---:|---:|---:|---:|
| prior M1 | pitch | 0.412 | 0.338 | **0.077** | **5.4×** |
| prior M1 | roll | 0.345 | 0.305 | 0.203 | 1.7× |
| prior M1 | throttle | 0.336 | 0.164 | 0.096 | 3.5× |
| 040-t4 M2 | pitch | 0.410 | 0.564 | 0.290 | 1.9× |
| 040-t4 M2 | roll | 0.710 | 0.711 | 0.661 | 1.1× |
| 040-t4 M2 | throttle | 0.594 | 0.591 | 0.401 | 1.5× |

**1. Aggressiveness is strongly regime-dependent in M1 — a pooled number is actively misleading.** M1 pitch
`dCtrl` swings **5.4×** between tracking (0.412) and patrol (0.077). Pooled it reads ≈0.28, a value the
controller occupies in *no* regime. This is the concrete justification for FR-011a making the regime
breakdown **required** rather than optional in the ablation report.

**2. M2 is far more aggressive AND far closer to saturation than M1, everywhere.** Roll `dCtrl` 0.710 vs
0.345 in tracking (**2.1×**), and pitch `⟨|u|⟩` 0.926 vs 0.622 — i.e. M2 sits near full throw. M2's roll is
also nearly *flat* across regimes (1.1× swing), meaning it is not modulating effort by situation the way M1
does; it is agitating continuously.

**3. M1's patrol is genuinely calm** (pitch `dCtrl` 0.077, `⟨|u|⟩` 0.749 — large steady deflection, small
changes). M2 has no comparably quiet regime; its calmest axis-regime (pitch/patrol, 0.290) is still ~4×
M1's.

## H2 (T056) — does bank-axis aggressiveness predict throttle level?

**Verdict: refuted on the throttle half.** Correlations are negative and negligible-to-weak.

| run | regime | r | r² | p | verdict |
|---|---|---:|---:|---:|---|
| prior M1 | tracking | −0.127 | 1.6% | 9e-148 | weak |
| prior M1 | intercept | −0.081 | 0.7% | 9e-74 | negligible despite tiny p |
| prior M1 | patrol | −0.155 | **2.4%** | 3e-222 | weak |
| 040-t4 M2 | tracking | −0.010 | 0.01% | 0.30 | **not significant** |
| 040-t4 M2 | intercept | −0.005 | 0.00% | 0.23 | **not significant** |
| 040-t4 M2 | patrol | −0.085 | 0.7% | 1e-97 | negligible despite tiny p |

⚠️ **Read r², not p.** At n ≈ 50 000 ticks, r = 0.05 lands at p < 1e-20 while explaining 0.25% of the
variance. Every "significant" row above explains **≤ 2.4%**. The tool emits r² and an explicit effect-size
verdict beside p for exactly this reason — reporting p alone would have made a null result look like a
finding.

The load half (`dCtrl_bank ~ nz`) is unanswerable here and waits on the new M1.

## What this does and does not settle for the follow-on

- It does **not** remove the load/aggressiveness thread from the follow-on's scope. The 5.4× regime swing
  says there is real structure to act on; had aggressiveness been flat across regimes, a regime-aware
  objective would have had nothing to grip.
- It **does** remove one candidate mechanism: "bank agitation drives throttle" is not it (≤2.4% variance,
  and the sign is *negative* — more bank agitation coincides with slightly *lower* throttle).
- It gives the prior M1's per-regime profile in the only obtainable form (FR-011c), since that genome has
  37 inputs and can never be loaded or ablated by a 041 binary.

## Files

`<label>-regime-axis.csv` · `<label>-regime-load.csv` (empty pre-break) · `<label>-h2.csv` ·
`<label>-autocorr.csv` (empty pre-break) · `<label>-summary.txt`

Reproduce:

```bash
python3 src/analytics/regime_load_study.py \
  --csv specs/041-m2-depth/artifacts/pre-break/prior-m1-gen800-physics.csv.gz --label prior-m1
python3 src/analytics/regime_load_study.py \
  --csv specs/041-m2-depth/artifacts/pre-break/040-t4-m2-gen800-physics.csv.gz --label 040-t4-m2
```
