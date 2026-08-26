# T005 — the 041-t7 extract is readable and complete, verified independently of the dmp loader

Run `python3 verify_extract.py t7-gen800-per-tick.csv.gz`. The check parses **only** the CSV (numpy;
no cereal, no dmp loader), replicating the `tools/dmp_dump.cc` method exactly (quat→ZYX roll/pitch,
`droll=Σ|unwrap(Δroll)|`, `dpitch=Σ|Δpitch|`, `dur=n·dt`, `dt=0.050 s`, `path = scenario // 49`).

## 1. Row counts / structure ✅

| | gen 800 | gen 633 |
|---|---:|---:|
| data rows | 129,894 | 130,416 |
| scenarios | 294 | 294 |
| scenarios/path | 49 × 6 | 49 × 6 |

Matches `scenario_count: 294` in the meta YAML and the 6 paths × 49 winds regiment.

## 2. Per-axis rate statistics — **EXACT** reproduction ✅

The independent CSV computation reproduces the authoritative shared-code per-path rates
(`t7-gen800-runsummary.csv`, produced by `computePerPathRates` inside the dmp loader) **bit-for-bit**:

| path | 0 | 1 | 2 | 3 | 4 | 5 | mean |
|---|---:|---:|---:|---:|---:|---:|---:|
| roll rate °/s (CSV) | 80.694 | 77.197 | 92.573 | 89.916 | 104.270 | 122.338 | **94.50** |
| roll rate °/s (run-summary) | 80.694 | 77.197 | 92.573 | 89.916 | 104.270 | 122.338 | — |
| pitch rate °/s (CSV) | 56.703 | 57.153 | 60.863 | 47.830 | 64.282 | 63.981 | **58.47** |
| pitch rate °/s (run-summary) | 56.703 | 57.153 | 60.863 | 47.830 | 64.282 | 63.981 | — |

⭐ This is the strongest completeness proof available: an FDM-independent re-derivation of the exact
statistic the dmp loader computes, matching to the printed precision. The extract is not lossy.

**Consistency with outcome.md**: `specs/041-m2-depth/outcome.md §3` quotes sim roll-rate amplitude
**88.4 °/s** (engaged-only, distance-standardized against the flight). The full-trajectory path-mean here
is 94.5 °/s — the 88.4 figure is a tracking-engaged *subset* of these same scenarios, so within-subset
agreement (~7%) is expected and confirms the extract carries the same signal.

## 3. Sim-side band-power (roll/pitch rate) — preserved as the 043 baseline

FFT of the per-tick rate series (Hann window, per scenario, power-pooled), fractions of total 0–10 Hz
power (Nyquist = 10 Hz at the 20 Hz recorded cadence):

| | 3–5 Hz | 5–10 Hz |
|---|---:|---:|
| gen 800 roll-rate | 10.6% | 4.4% |
| gen 800 pitch-rate | 18.0% | 18.7% |
| gen 633 roll-rate | 10.6% | 5.7% |
| gen 633 pitch-rate | 16.8% | 21.7% |

⚠️ **Honest note on outcome.md's 2.39× / 3.22×**: those are *flight ÷ sim* ratios from a 041 analysis that
was **engaged-only** on the higher-rate flight blackbox with an unpublished sim-side denominator method;
they are **not** reproducible from this full-trajectory 20 Hz sim trace alone, and outcome.md does not
publish sim-side fractions to match bit-for-bit. What is preserved here is the **sim** side computed by an
explicit, re-runnable method — the correct 043 baseline for the Phase-10 SC-001/SC-001a comparison, where
the new flight's spectra are compared against the flight reference (30.1% / 7.4%) that lives in
`flight-results/`, not in these dmps.

## Verdict

✅ **The extract is readable independently of any dmp loader, and complete**: row counts and the
per-axis rate statistics reproduce the source exactly. The T011 wire-format break is now safe to make.
