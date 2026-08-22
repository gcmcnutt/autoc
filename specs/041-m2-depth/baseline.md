# 041-t7 is the M1 baseline of record

**Operator decision 2026-08-22**: *"Good enough — we have a subjective improvement and this current run is
the new baseline."*

Supersedes **038-t5** (`autoc-9223370253553029228-2026-07-06T01:35:46.579Z`), the prior-M1 comparator pinned
in `artifacts/pre-break/`.

---

## The run

| | |
|---|---|
| run | `autoc-041-t7-m1-inputscale` |
| run id | `autoc-9223370249590214474-2026-08-20T22:22:41.333Z` (bucket `autoc-m1`) |
| master seed | `1787264561` |
| config | `autoc.ini` — pop 5000, 20 Hz, lexicase (`score` + `energy_score`), `VariationRampStep=40` |
| inputs | 45, **P2-8 scaled** (`747a522`) |
| binary | clean `rebuild-perf.sh`, 48/48 suites |

⚠️ **The run is still going** — gen 575 of 800 as of this note. The *run* is designated baseline; the
**pinned generation is whatever it finishes at**, and the numbers below are a mid-run snapshot, not the
pin. Re-pin at completion.

## Why it earned it

| | 038-t5 (prior) | 041-t7 | |
|---|---:|---:|---|
| best fitness | −45,254 @ g800 | **−73,899 @ g551** | **+63%**, in ~2/3 the generations |
| all-time M1 record | −55,270 (034-origm1, g590) | **beaten at g289** | — |
| throttle pegged (>0.99) | 37.7% | **32.0%** | de-pegged |
| load p50 / p95 | 1.92 / 5.21 g | **1.44 / 4.22 g** | −25% / −19% |
| ticks ≥ 6 g | 2.15% | **0.51%** | **4.2× lower** |
| ticks ≥ 8 g | 0.118% | **0.020%** | **5.9× lower** |

⭐ **Better score AND lower airframe load AND lower throttle saturation, together.** These normally trade;
here they moved the same way, which is the substantive reason to re-baseline rather than the fitness number
alone.

## What changed to get here

**One thing: [P2-8 input scaling](../../include/autoc/nn/nn_inputs.h).** Four raw-units inputs
(`AIRSPEED`, `DIST`×6, `CLOSING_RATE`, `GYRO`×3) were divided by measured-p95 constants so every slot
reaches the first layer on comparable footing. Input spread had run 200:1 across the vector; the quiet slots
were never selected on at all (t5: every input's weight investment flat at ~1.0 for 475 generations).

⛔ **What did NOT get here**: the energy-axis rework (worth ~8%, [objective-amendment](objective-amendment.md))
and the variation ramp (t6 pegged throttle at 96.4% with it on). Both were tried first and neither was the
binding constraint. See [t5-wrap.md](t5-wrap.md).

## Consequences

* **Regression gate**: future `rebuild-perf.sh` `ScenarioScore` comparisons pin against t7's final dmp, not
  038-t5's.
* ⛔ **Weights are NOT portable backwards.** P2-8 changed input scales with the layout untouched
  (still float[45]), so a pre-P2-8 genome loads cleanly and flies wrong. `P2-9` (nn2cpp scale signature)
  exists precisely because the count assert does not catch this.
* **Analytics**: `regime_control` and `g_load` are new with this baseline; the aggressiveness dashed lines
  are 027-era *servo-demand references*, not limits — `P2-10` is meant to replace them with a measured one.
* ⚠️ **Cross-era comparison stays possible only via `artifacts/pre-break/`** — the v3→v4 schema break makes
  038-t5's dmp unreadable, and that archive's physics columns are 4 ticks/scenario (see its README).
